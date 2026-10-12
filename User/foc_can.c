#include "foc_can.h"
#include "fdcan.h"
volatile uint32_t foc_can_rx_count = 0U;     // 成功接收的报文数量
volatile uint8_t foc_can_rx_data[8] = {0};  // 最近一次收到的8字节数据
HAL_StatusTypeDef FOC_CAN_Init(void)
{
    FDCAN_FilterTypeDef filter = {0};   // 定义CAN接收过滤器

    filter.IdType = FDCAN_STANDARD_ID;       // 使用11位标准ID
    filter.FilterIndex = 0;                 // 使用第0号过滤器
    filter.FilterType = FDCAN_FILTER_MASK;  // 使用掩码匹配
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 匹配的消息放入FIFO0
    filter.FilterID1 = 0x100;               // 接收目标ID
    filter.FilterID2 = 0x7FF;               // 11位全部参与比较
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
        return HAL_ERROR;                   // 配置过滤器失败

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
            FDCAN_REJECT, FDCAN_REJECT,
            FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
        return HAL_ERROR;                   // 拒绝未匹配及远程帧

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
        return HAL_ERROR;                   // 启动CAN控制器失败

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        return HAL_ERROR;  // CAN启动失败
    }
    return HAL_OK;  // CAN启动成功
}

void FOC_CAN_ProcessRx(void)
{
    FDCAN_RxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};

    static int16_t target_speed = 0;
    static uint8_t start_pending = 0;

    // 读取硬件FIFO中的全部消息
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0,
                                   &header, data) != HAL_OK)
            break;

        // 检查CAN报文
        if (header.Identifier != 0x100U ||
            header.IdType != FDCAN_STANDARD_ID ||
            header.RxFrameType != FDCAN_DATA_FRAME ||
            header.DataLength != FDCAN_DLC_BYTES_8)
            continue;

        foc_can_rx_count++;

        // Byte1和Byte2组成16位参数
        uint16_t raw = (uint16_t)data[1] |
                       ((uint16_t)data[2] << 8);

        // 根据命令直接调用已有的FOC函数
        switch (data[0])
        {
            case 0x01:  // SET_MODE
                // 初版只有速度模式，无需切换
                break;

            case 0x02:  // SET_SPEED
                target_speed = (int16_t)raw;

                if (foc_motor.motor_state == FOC_STATE_RUNNING)
                {
                    FOC_LOOP_SPD_SetSpeedRef(
                        &foc_motor.loop_spd, (float)target_speed);
                }
                break;

            case 0x04:  // START
                if (foc_motor.motor_state == FOC_STATE_STOPPED)
                {
                    FOC_Motor_Start();

                    if (foc_motor.motor_state != FOC_STATE_STOPPED)
                        start_pending = 1;
                }
                break;

            case 0x05:  // STOP
                start_pending = 0;
                target_speed = 0;
                FOC_Motor_Stop();
                break;

            default:
                break;
        }
    }

    // 对齐完成后，再应用ESP32发送的目标速度
    if (start_pending)
    {
        if (foc_motor.motor_state == FOC_STATE_RUNNING)
        {
            FOC_LOOP_SPD_SetSpeedRef(
                &foc_motor.loop_spd, (float)target_speed);

            start_pending = 0;
        }
        else if (foc_motor.motor_state == FOC_STATE_STOPPED)
        {
            start_pending = 0;
        }
    }
}

static void FOC_CAN_Send(uint32_t id, uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};

    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0U)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data);
    }
}

// 将int16按小端格式放入两个字节
static void CAN_PutInt16(uint8_t *data, int16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)((uint16_t)value >> 8);
}

void FOC_CAN_SendFeedback(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();

    // 每100ms发送一次
    if ((uint32_t)(now - last_tick) < 100U)
        return;

    last_tick = now;

    // 1. 电机状态：0x180
    uint8_t status[8] = {0};
    status[0] = (uint8_t)foc_motor.motor_state;
    status[1] = 0;  // 速度模式
    FOC_CAN_Send(0x180U, status);

    // 2. 电流：0x182，放大100倍
    uint8_t current[8] = {0};

    CAN_PutInt16(&current[0], (int16_t)(foc_motor.loop_cur.iq_ref * 100.0f));
    CAN_PutInt16(&current[2], (int16_t)(foc_motor.loop_cur.i_q * 100.0f));
    CAN_PutInt16(&current[4], (int16_t)(foc_motor.loop_cur.id_ref * 100.0f));
    CAN_PutInt16(&current[6], (int16_t)(foc_motor.loop_cur.i_d * 100.0f));

    FOC_CAN_Send(0x182U, current);

    // 3. 速度：0x183，单位rpm
    uint8_t speed[8] = {0};

    CAN_PutInt16(&speed[0], (int16_t)foc_motor.loop_spd.speed_fbk);
    CAN_PutInt16(&speed[2], (int16_t)foc_motor.loop_spd.speed_ref);

    FOC_CAN_Send(0x183U, speed);
}