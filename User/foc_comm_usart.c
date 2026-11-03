/**
 * @file foc_comm_usart.c
 * @brief USART2 Modbus RTU：地址1，2 Mbps，8N2，功能码03/06/10。
 *
 * IOC负责硬件初始化；StartReceive启动DMA；Task在主循环解析、执行和回复。
 * 浮点数采用ABCD顺序：高16位寄存器在前，每个寄存器高字节在前。
 * 曲线每1ms采一组，5组组成一批；只保留最新批次，不补传历史。
 */
#include "foc_comm_usart.h"
#include "foc_manager.h"
#include "foc_math.h"
#include "usart.h"
#include <math.h>
#include <string.h>

#define MODBUS_ADDRESS       1U
#define RX_DMA_SIZE          512U
#define FRAME_SIZE           256U
#define FRAME_GAP_MS         3U  // 1ms计时下，至少留出2ms静默，不阻塞等待
#define SAMPLE_CHANNELS     10U
#define SAMPLES_PER_BATCH    5U
#define BATCH_REGISTERS    102U  // 2个时间戳寄存器 + 5组×10个float×2

static uint8_t rx_dma[RX_DMA_SIZE];
static uint8_t rx_frame[FRAME_SIZE];
static uint8_t tx_frame[FRAME_SIZE];
static uint16_t rx_read;
static uint16_t rx_last_write;
static uint32_t rx_last_tick;

static float samples[2][SAMPLES_PER_BATCH][SAMPLE_CHANNELS];
static uint32_t batch_tick[2]; // 每批第一组的HAL_GetTick，单位ms
static uint8_t sample_enabled;
static uint8_t sample_index;
static uint8_t write_batch;
static volatile uint8_t ready_batch = 1U;

/**
 * @brief  将两个字节还原为一个16位无符号整数。
 * @param[in] data 指向两个字节的指针，高字节在前、低字节在后。
 * @return 还原后的16位整数。
 * @note   用于读取请求中的寄存器地址、数量和值。
 */
static uint16_t Modbus_GetU16(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8) | data[1];
}

/**
 * @brief  将一个32位浮点数拆分到两个连续寄存器。
 * @param[out] registers 指向两个uint16_t的指针，依次写入浮点数的高、低16位。
 * @param[in] value 要写入的浮点数。
 * @return 无。
 * @note   只复制浮点数的二进制表示，不将浮点数转换为整数数值。
 */
static void Modbus_PutFloat(uint16_t *registers, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    registers[0] = (uint16_t)(bits >> 16);
    registers[1] = (uint16_t)bits;
}

/**
 * @brief  逐字节、逐位计算Modbus RTU的CRC16。
 * @param[in] data 待计算的数据起始地址。
 * @param[in] length 参与计算的字节数。
 * @return CRC16计算结果。
 * @note   发送时计算不含CRC的帧；接收时计算包含CRC的完整帧，结果应为0。
 */
static uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if (crc & 1U) crc = (crc >> 1) ^ 0xA001U;
            else crc >>= 1;
        }
    }
    return crc;
}

/**
 * @brief  为回复添加CRC，并启动USART2的DMA发送。
 * @param[in] length tx_frame中已有数据的字节数，不包含末尾的两个CRC字节。
 * @return 无。
 * @note   主循环调用，不等待发送完成；DMA发送期间不得改写tx_frame。
 *         当前请求为广播地址0时不发送回复。
 */
static void Modbus_Send(uint16_t length)
{
    if (rx_frame[0] == 0U) return; // 广播写入执行，但不回复
    uint16_t crc = Modbus_CRC16(tx_frame, length);
    tx_frame[length] = (uint8_t)crc; // CRC低字节先发送
    tx_frame[length + 1U] = (uint8_t)(crc >> 8);
    HAL_UART_Transmit_DMA(&huart2, tx_frame, length + 2U);
}

/**
 * @brief  启动USART2循环DMA接收，并允许记录曲线采样。
 * @return 无。
 * @note   在main中完成外设和FOC初始化后调用一次。
 *         串口及DMA配置由IOC生成，本函数不重复初始化硬件。
 */
void Modbus_StartReceive(void)
{
    HAL_UART_Receive_DMA(&huart2, rx_dma, sizeof(rx_dma));
    // 循环DMA持续接收，主循环读取写入位置，不需要半满/全满中断。
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT | DMA_IT_TC);
    sample_enabled = 1U;
}

/**
 * @brief  读取串口、CAN和按键共用的电机控制模式。
 * @return FOC_CONTROL_SPEED表示速度模式，FOC_CONTROL_POSITION表示位置模式。
 * @note   只读取foc_motor.control_mode，不改变目标值或电机状态。
 */
FOC_CONTROL_MODE FOC_COMM_USART_GetControlMode(void)
{
    return foc_motor.control_mode;
}

/**
 * @brief  在电机停止时选择控制模式。
 * @param[in] mode FOC_CONTROL_SPEED为速度模式，FOC_CONTROL_POSITION为位置模式。
 * @return 无。
 * @note   主循环调用；电机不在STOPPED状态时不修改模式，保留已有目标值。
 */
void FOC_COMM_USART_SetControlMode(FOC_CONTROL_MODE mode)
{
    if (foc_motor.motor_state != FOC_STATE_STOPPED) return;
    if (mode > FOC_CONTROL_POSITION) return;
    foc_motor.control_mode = mode;
}

/**
 * @brief  记录一组10通道数据，每5组发布一个完整采样批次。
 * @return 无。
 * @note   由SysTick每1ms调用，所有电机状态均采样；启动接收前不记录。
 *         两个固定缓冲区轮换使用，时间戳记录本批第一组的毫秒计数。
 *         通道顺序见头文件；位置以度保存，只保留最新完整批次。
 */
void Modbus_Sample(void)
{
    if (!sample_enabled) return;
    if (sample_index == 0U) batch_tick[write_batch] = HAL_GetTick();
    float *values = samples[write_batch][sample_index];
    uint32_t primask = __get_PRIMASK();
    __disable_irq(); // 只复制反馈值，避免复制一半时控制中断更新变量
    values[0] = foc_motor.loop_spd.speed_ref;
    values[1] = foc_motor.loop_spd.speed_fbk;
    values[2] = foc_motor.loop_cur.id_ref;
    values[3] = foc_motor.loop_cur.i_d;
    values[4] = foc_motor.loop_cur.iq_ref;
    values[5] = foc_motor.loop_cur.i_q;
    values[6] = foc_motor.loop_cur.v_d;
    values[7] = foc_motor.loop_cur.v_q;
    values[8] = foc_motor.loop_pos.position_ref;
    values[9] = foc_motor.encoder.angle_m;
    __set_PRIMASK(primask);
    values[8] *= 180.0f / FOC_PI;
    values[9] *= 180.0f / FOC_PI;
    if (++sample_index == SAMPLES_PER_BATCH)
    {
        sample_index = 0U;
        ready_batch = write_batch;
        write_batch ^= 1U;
    }
}

/**
 * @brief  处理功能码03，读取状态、目标值或最新完整采样批次。
 * @return 无。
 * @note   由Modbus_Task在主循环调用，从rx_frame读取起始地址和数量。
 *         将寄存器按高字节在前的顺序装入tx_frame后发送。
 *         仅接受头文件约定范围内的连续读取，不跨越寄存器区域。
 */
static void Modbus_ReadRegisters(void)
{
    uint16_t address = Modbus_GetU16(rx_frame + 2);
    uint16_t count = Modbus_GetU16(rx_frame + 4);
    uint16_t registers[BATCH_REGISTERS];
    uint16_t offset;
    if (count == 0U || count > 125U) return;

    if (address <= 0x0001U && count <= 2U - address)
    {
        registers[0] = (uint16_t)foc_motor.motor_state;
        registers[1] = (uint16_t)foc_motor.control_mode;
        offset = address;
    }
    else if (address >= 0x0010U && address <= 0x0013U && count <= 0x0014U - address)
    {
        Modbus_PutFloat(registers, foc_motor.speed_target);
        Modbus_PutFloat(registers + 2, foc_motor.position_target * 180.0f / FOC_PI);
        offset = address - 0x0010U;
    }
    else if (address >= 0x0100U && address <= 0x0165U && count <= 0x0166U - address)
    {
        float values[SAMPLES_PER_BATCH * SAMPLE_CHANNELS];
        uint32_t primask = __get_PRIMASK();
        __disable_irq(); // 复制完整批次后立即恢复中断，字节转换在外面做
        uint32_t tick = batch_tick[ready_batch];
        memcpy(values, samples[ready_batch], sizeof(values));
        __set_PRIMASK(primask);
        registers[0] = (uint16_t)(tick >> 16);
        registers[1] = (uint16_t)tick;
        for (uint16_t i = 0U; i < SAMPLES_PER_BATCH * SAMPLE_CHANNELS; ++i)
            Modbus_PutFloat(registers + 2U + i * 2U, values[i]);
        offset = address - 0x0100U;
    }
    else return;

    tx_frame[0] = MODBUS_ADDRESS;
    tx_frame[1] = 0x03U;
    tx_frame[2] = (uint8_t)(count * 2U);
    for (uint16_t i = 0U; i < count; ++i)
    {
        tx_frame[3U + i * 2U] = (uint8_t)(registers[offset + i] >> 8);
        tx_frame[4U + i * 2U] = (uint8_t)registers[offset + i];
    }
    Modbus_Send(3U + count * 2U);
}

/**
 * @brief  处理功能码06，写入控制模式或执行电机指令。
 * @return 无。
 * @note   由Modbus_Task在主循环调用，从rx_frame读取地址和值。
 *         地址0001选择模式；0002的值1/2/3分别调用启动、停止、回零。
 *         manager负责电机状态限制；指令回复不表示运动已经完成。
 */
static void Modbus_WriteRegister(void)
{
    uint16_t address = Modbus_GetU16(rx_frame + 2);
    uint16_t value = Modbus_GetU16(rx_frame + 4);
    switch (address)
    {
        case 0x0001: // STOP中选择速度/位置模式
            if (foc_motor.motor_state != FOC_STATE_STOPPED || value > 1U) return;
            FOC_COMM_USART_SetControlMode((FOC_CONTROL_MODE)value);
            break;
        case 0x0002: // 电机流程仍由manager处理
            switch (value)
            {
                case 1: FOC_Motor_Start();  break;
                case 2: FOC_Motor_Stop();   break;
                case 3: FOC_Motor_GoZero(); break;
                default: return;
            }
            break;
        default: return;
    }
    memcpy(tx_frame, rx_frame, 6U); // 功能码06原样回复地址和值
    Modbus_Send(6U);
}

/**
 * @brief  处理功能码10，将两个寄存器还原为一个浮点目标值。
 * @return 无。
 * @note   由Modbus_Task在主循环调用，仅处理一次写入一个float的请求。
 *         地址0010写转速，单位rpm；0012写位置，单位度，内部转换为rad。
 *         ALIGNING中不修改目标；运行中只修改当前控制模式对应的目标。
 *         更新用户目标后，由manager的运行状态函数应用到控制环。
 */
static void Modbus_WriteRegisters(void)
{
    uint16_t address = Modbus_GetU16(rx_frame + 2);
    uint32_t bits = ((uint32_t)Modbus_GetU16(rx_frame + 7) << 16)
                  | Modbus_GetU16(rx_frame + 9);
    float value;
    memcpy(&value, &bits, sizeof(value));
    if (foc_motor.motor_state == FOC_STATE_ALIGNING) return;
    switch (address)
    {
        case 0x0010: // 转速目标，rpm
            if (foc_motor.motor_state == FOC_STATE_RUNNING_POSITION) return;
            if (value > MOTOR_MAX_SPEED_RPM) value = MOTOR_MAX_SPEED_RPM;
            if (value < -MOTOR_MAX_SPEED_RPM) value = -MOTOR_MAX_SPEED_RPM;
            foc_motor.speed_target = value;
            break;
        case 0x0012: // 位置目标，通信使用度，电机内部使用rad
            if (foc_motor.motor_state == FOC_STATE_RUNNING_SPEED) return;
            value = fmodf(value, 360.0f) * FOC_PI / 180.0f;
            foc_motor.position_target = FOC_MATH_Normalize(value);
            break;
        default: return;
    }
    memcpy(tx_frame, rx_frame, 6U); // 功能码10回复起始地址和寄存器数量
    Modbus_Send(6U);
}

/**
 * @brief  检查DMA接收进度，取出完整请求并分发给对应功能码处理函数。
 * @return 无。
 * @note   启动接收后，在主循环中反复调用；不阻塞等待数据或发送完成。
 *         DMA写入位置连续3个毫秒计数不变后，按一帧处理已收到的数据。
 *         检查帧长度、设备地址和CRC后处理03/06/10，无效请求直接丢弃。
 *         DMA发送期间直接返回，避免覆盖正在发送的回复。
 */
void Modbus_Task(void)
{
    // 一次请求只产生一次回复，DMA发送期间不改写发送缓冲区。
    if (huart2.gState != HAL_UART_STATE_READY) return;
    uint16_t write = (RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx)) % RX_DMA_SIZE;
    if (write != rx_last_write)
    {
        rx_last_write = write;
        rx_last_tick = HAL_GetTick();
        return;
    }
    if (write == rx_read || HAL_GetTick() - rx_last_tick < FRAME_GAP_MS) return;

    uint16_t length = (write + RX_DMA_SIZE - rx_read) % RX_DMA_SIZE;
    if (length > FRAME_SIZE) { rx_read = write; return; }
    for (uint16_t i = 0U; i < length; ++i)
    {
        rx_frame[i] = rx_dma[rx_read];
        rx_read = (rx_read + 1U) % RX_DMA_SIZE;
    }
    if (length < 4U || (rx_frame[0] != MODBUS_ADDRESS && rx_frame[0] != 0U)) return;
    if (Modbus_CRC16(rx_frame, length) != 0U) return;

    switch (rx_frame[1])
    {
        case 0x03:
            if (length == 8U && rx_frame[0] != 0U) Modbus_ReadRegisters();
            break;
        case 0x06:
            if (length == 8U) Modbus_WriteRegister();
            break;
        case 0x10: // 基础版本：一次写两个寄存器，即一个float
            if (length == 13U && Modbus_GetU16(rx_frame + 4) == 2U && rx_frame[6] == 4U)
                Modbus_WriteRegisters();
            break;
    }
}
