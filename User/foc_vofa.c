/**
 * @file    foc_vofa.c
 * @brief   VOFA+遥测发送与命令处理
 */
#include "foc_vofa.h"
#include "foc_manager.h"   // 串口START/STOP复用现有电机启停函数
#include "motor_param.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define VOFA_TX_CHANNELS     10U // 原有8个通道不变，追加目标角度和实际角度
#define VOFA_TX_FRAME_SIZE   (VOFA_TX_CHANNELS * 4U + 4U) // 10个float及JustFloat帧尾，共44字节
#define VOFA_COMMAND_SIZE    32U // 单条ASCII命令缓冲区长度，字节
#define VOFA_RX_DMA_SIZE     512U // DMA直接循环写入此数组，不再每字节触发UART中断
#define VOFA_RX_MAX_PER_CALL 128U // 每轮最多解析128字节，避免连续串口数据长时间占用主循环
#define VOFA_TX_PERIOD_MS    1U // 最多1kHz遥测；速度每1ms更新，无需重复发送约4kHz的同一速度
static UART_HandleTypeDef *vofa_uart;
static FOC_ENCODER_HandleTypeDef *vofa_encoder;
static FOC_LOOP_CUR_HandleTypeDef *vofa_loop_cur;
static FOC_LOOP_SPD_HandleTypeDef *vofa_loop_spd;
static FOC_LOOP_POS_HandleTypeDef *vofa_loop_pos;
static volatile FOC_CONTROL_MODE control_mode = FOC_CONTROL_SPEED;
static volatile uint8_t telemetry_pending;
static volatile uint8_t tx_busy;
static uint8_t tx_buffer[VOFA_TX_FRAME_SIZE];
static volatile uint8_t rx_dma_buffer[VOFA_RX_DMA_SIZE]; // DMA与主循环共享，volatile确保每次读取内存
static volatile uint32_t rx_dma_wrap_count; // 接收满512字节时加1，用于区分空缓冲与整圈新数据
static uint32_t rx_read_count; // 主循环已处理的累计字节数
static volatile uint8_t rx_restart_pending; // 错误中断只置标志，主循环恢复接收
static uint32_t rx_overrun_count; // 主循环来不及处理时计数，并丢弃不完整命令
static uint32_t tx_last_tick;
static char command_buffer[VOFA_COMMAND_SIZE];
static uint8_t command_length;

/** @brief 启动已有的USART2循环接收DMA；半满中断不需要处理，满缓冲中断只计数。 */
static HAL_StatusTypeDef FOC_VOFA_StartRxDMA(void)
{
    rx_dma_wrap_count = 0U;
    rx_read_count = 0U;
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(vofa_uart, (uint8_t *)rx_dma_buffer, VOFA_RX_DMA_SIZE);
    if (status == HAL_OK) __HAL_DMA_DISABLE_IT(vofa_uart->hdmarx, DMA_IT_HT); // 少一次半满中断，短命令由主循环读取
    return status;
}

/** @brief 读取DMA累计接收字节数；短临界区只读取寄存器，不解析命令、不调用HAL发送。 */
static uint32_t FOC_VOFA_GetRxCount(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq(); // 保证“满缓冲计数”和DMA寄存器属于同一个快照，结束后恢复原来的中断状态
    uint32_t wraps = rx_dma_wrap_count;
    uint32_t remaining = __HAL_DMA_GET_COUNTER(vofa_uart->hdmarx);
    if (__HAL_DMA_GET_FLAG(vofa_uart->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(vofa_uart->hdmarx)) != 0U)
    {
        wraps++; // DMA已经回绕，但满缓冲中断尚未执行；暂时补算，不修改中断计数
        remaining = __HAL_DMA_GET_COUNTER(vofa_uart->hdmarx); // 回绕后的剩余量需要重新读取
        if (remaining == 0U) remaining = VOFA_RX_DMA_SIZE; // 恰好在重新装载的瞬间，不能把同一圈再加一次
    }
    __set_PRIMASK(primask);
    return wraps * VOFA_RX_DMA_SIZE + VOFA_RX_DMA_SIZE - remaining;
}

/**
 * @brief  切换到速度模式并清除旧目标与PI积分
 */
static void FOC_VOFA_SwitchToSpeed(void)
{
    NVIC_DisableIRQ(TIM6_DAC_IRQn);
    /* 切到速度模式时目标置零，避免旧位置目标引起突跳。 */
    FOC_LOOP_SPD_SetSpeedRef(vofa_loop_spd, 0.0f);
    FOC_PI_Reset(&vofa_loop_spd->pi_spd);
    FOC_PI_Reset(&vofa_loop_pos->pi_pos);
    control_mode = FOC_CONTROL_SPEED;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
 * @brief  以当前位置为目标切换到位置模式
 */
static void FOC_VOFA_SwitchToPosition(void)
{
    if (control_mode == FOC_CONTROL_POSITION)
    {
        return;
    }

    NVIC_DisableIRQ(TIM6_DAC_IRQn);
    /* 先读取当前位置，再将它作为位置目标。 */
    FOC_ENCODER_UpdateAngle(vofa_encoder);
    FOC_LOOP_POS_SetPositionRef(vofa_loop_pos, vofa_encoder->angle_m);
    FOC_LOOP_SPD_SetSpeedRef(vofa_loop_spd, 0.0f);
    FOC_PI_Reset(&vofa_loop_pos->pi_pos);
    FOC_PI_Reset(&vofa_loop_spd->pi_spd);
    control_mode = FOC_CONTROL_POSITION;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
 * @brief  解析一条已完成的ASCII控制命令
 * @param  command 以空字符结尾的命令字符串
 */
static void FOC_VOFA_ExecuteCommand(const char *command)
{
    float value;
    char *end;

    if (strcmp(command, "MS") == 0)
    {
        FOC_VOFA_SwitchToSpeed();
    }
    else if (strcmp(command, "MP") == 0)
    {
        FOC_VOFA_SwitchToPosition();
    }
    else if (strcmp(command, "START") == 0)
    {
        FOC_Motor_Start();   // 保留原有对齐流程；位置操作先发MP，等待RUNNING后再设置角度
    }
    else if (strcmp(command, "STOP") == 0)
    {
        FOC_Motor_Stop();   // 真正关闭输出，避免仅设零速时位置环继续保持电机
    }
    else if ((command[0] == 'S') || (command[0] == 'P') || (command[0] == 'A'))
    {
        value = strtof(&command[1], &end);
        if ((end == &command[1]) || (*end != '\0') || !isfinite(value))
        {
            return;
        }

        if (command[0] == 'A')
        {
            value = fmodf(value, 360.0f) * FOC_PI / 180.0f;   // A90表示90度，先归一化再换算为库函数使用的rad
            FOC_VOFA_SetControlMode(FOC_CONTROL_POSITION);   // 角度命令自动选择位置模式
        }
        else if (command[0] == 'P')
        {
            /* 位置模块按单圈角度工作，先取余以免超大输入反复减 2π。 */
            value = fmodf(value, FOC_TWO_PI);
        }

        NVIC_DisableIRQ(TIM6_DAC_IRQn);
        if (command[0] == 'S')
        {
            if (value > MOTOR_MAX_SPEED_RPM) value = MOTOR_MAX_SPEED_RPM;
            if (value < -MOTOR_MAX_SPEED_RPM) value = -MOTOR_MAX_SPEED_RPM;
            FOC_LOOP_SPD_SetSpeedRef(vofa_loop_spd, value);
        }
        else
        {
            FOC_LOOP_POS_SetPositionRef(vofa_loop_pos, value);
        }
        NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }
}

HAL_StatusTypeDef FOC_VOFA_Init(UART_HandleTypeDef *uart,
                                FOC_ENCODER_HandleTypeDef *encoder,
                                FOC_LOOP_CUR_HandleTypeDef *loop_cur,
                                FOC_LOOP_SPD_HandleTypeDef *loop_spd,
                                FOC_LOOP_POS_HandleTypeDef *loop_pos)
{
    vofa_uart = uart;
    vofa_encoder = encoder;
    vofa_loop_cur = loop_cur;
    vofa_loop_spd = loop_spd;
    vofa_loop_pos = loop_pos;
    control_mode = FOC_CONTROL_SPEED;
    telemetry_pending = 0U;
    tx_busy = 0U;
    command_length = 0U;
    rx_restart_pending = 0U;
    rx_overrun_count = 0U;
    tx_last_tick = HAL_GetTick() - VOFA_TX_PERIOD_MS;

    if (uart == NULL || uart->hdmarx == NULL || uart->hdmarx->Init.Mode != DMA_CIRCULAR) return HAL_ERROR; // 必须使用已有的循环RX DMA
    return FOC_VOFA_StartRxDMA();
}

FOC_CONTROL_MODE FOC_VOFA_GetControlMode(void)
{
    return control_mode;
}

void FOC_VOFA_SetControlMode(FOC_CONTROL_MODE mode)
{
    if (mode == control_mode) return;   // UI重复发送相同模式时不把正在使用的目标清零
    if (mode == FOC_CONTROL_SPEED) FOC_VOFA_SwitchToSpeed();
    else if (mode == FOC_CONTROL_POSITION) FOC_VOFA_SwitchToPosition();   // 先保持当前位置，再接受新目标
}

void FOC_VOFA_RequestTelemetry(void)
{
    telemetry_pending = 1U;
}

void FOC_VOFA_ProcessTx(void)
{
    // 电流环中断只置请求标志，帧打包与DMA发送留在主循环。
    if (telemetry_pending == 0U)
    {
        return;
    }
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - tx_last_tick) < VOFA_TX_PERIOD_MS) return; // 保留请求，等下一毫秒发送最新值
    telemetry_pending = 0U;
    if (tx_busy != 0U)
    {
        return; // DMA忙时丢弃本帧，避免等待影响FOC。
    }

    // 保留原有CH0～CH7，追加两个角度通道；DMA忙时仍直接跳过，不阻塞FOC。
    const volatile FOC_LOOP_SPD_HandleTypeDef *spd = vofa_loop_spd;
    const volatile FOC_LOOP_CUR_HandleTypeDef *cur = vofa_loop_cur;
    const float values[VOFA_TX_CHANNELS] = {
        spd->speed_ref, spd->speed_fbk, cur->id_ref, cur->i_d,
        cur->iq_ref, cur->i_q, cur->v_d, cur->v_q,
        vofa_loop_pos->position_ref * 180.0f / FOC_PI,   // CH8：目标单圈机械角，度
        vofa_encoder->angle_m * 180.0f / FOC_PI          // CH9：实际单圈机械角，度
    };
    for (uint8_t i = 0U; i < VOFA_TX_CHANNELS; ++i)
    {
        memcpy(&tx_buffer[i * 4U], &values[i], sizeof(float));
    }
    // JustFloat帧尾：00 00 80 7F。
    tx_buffer[VOFA_TX_FRAME_SIZE - 4U] = 0x00U;
    tx_buffer[VOFA_TX_FRAME_SIZE - 3U] = 0x00U;
    tx_buffer[VOFA_TX_FRAME_SIZE - 2U] = 0x80U;
    tx_buffer[VOFA_TX_FRAME_SIZE - 1U] = 0x7FU;

    tx_busy = 1U; // 提交前占用缓冲，避免完成中断先清零、主循环又错误置忙
    if (HAL_UART_Transmit_DMA(vofa_uart, tx_buffer, VOFA_TX_FRAME_SIZE) == HAL_OK)
    {
        tx_last_tick = now;
    }
    else tx_busy = 0U; // 提交失败时DMA不持有缓冲，可以在下一次遥测请求重试
}

void FOC_VOFA_ProcessRx(void)
{
    if (rx_restart_pending != 0U)
    {
        if (HAL_UART_AbortReceive(vofa_uart) != HAL_OK) return; // 只恢复接收，不停止正在进行的TX DMA
        command_length = VOFA_COMMAND_SIZE; // 错误后丢弃到下一个LF，避免将残缺数字误当完整命令
        rx_restart_pending = 0U;
        if (FOC_VOFA_StartRxDMA() != HAL_OK) rx_restart_pending = 1U; // 启动失败留待下轮处理，不在中断重试
        return;
    }

    for (uint32_t i = 0U; i < VOFA_RX_MAX_PER_CALL; ++i)
    {
        if (rx_restart_pending != 0U) return; // 串口报错后不继续执行缓冲中的旧命令
        uint32_t received = FOC_VOFA_GetRxCount();
        if ((uint32_t)(received - rx_read_count) > VOFA_RX_DMA_SIZE)
        {
            rx_overrun_count++; // DMA已覆盖未处理的数据；丢弃旧数据，并在LF处重新同步
            rx_read_count = received;
            command_length = VOFA_COMMAND_SIZE;
            return;
        }
        if (received == rx_read_count) break; // 不必等DMA写满或IDLE事件，短命令同样立即处理
        uint8_t byte = rx_dma_buffer[rx_read_count % VOFA_RX_DMA_SIZE];
        if (rx_restart_pending != 0U) return;
        if ((uint32_t)(FOC_VOFA_GetRxCount() - rx_read_count) > VOFA_RX_DMA_SIZE)
        {
            rx_overrun_count++; // 读取该字节时被高优先级中断打断，也要检查DMA是否覆盖了它
            rx_read_count = FOC_VOFA_GetRxCount();
            command_length = VOFA_COMMAND_SIZE;
            return;
        }
        rx_read_count++;

        if (byte == '\n')
        {
            if (command_length > 0U && command_length < VOFA_COMMAND_SIZE)
            {
                command_buffer[command_length] = '\0';
                FOC_VOFA_ExecuteCommand(command_buffer); // LF确认一条命令，在主循环直接执行，下一条仍由DMA保存
            }
            command_length = 0U;
        }
        else if (byte != '\r')
        {
            if (command_length < VOFA_COMMAND_SIZE - 1U) command_buffer[command_length++] = (char)byte;
            else command_length = VOFA_COMMAND_SIZE; // 超长命令整条丢弃，不能截取前31字符执行
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == vofa_uart) rx_dma_wrap_count++; // 循环DMA自动继续，不重新启动接收，不在中断解析命令
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == vofa_uart)
    {
        if (uart->RxState == HAL_UART_STATE_READY) rx_restart_pending = 1U; // HAL已终止接收，交给主循环恢复
        if (uart->gState == HAL_UART_STATE_READY) tx_busy = 0U; // DMA错误终止发送时也释放静态发送缓冲
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == vofa_uart)
    {
        // 释放发送标志，允许主循环提交下一帧DMA。
        tx_busy = 0U;
    }
}
