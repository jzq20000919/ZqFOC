#include "foc_can.h"
#include "fdcan.h"
#include "foc_manager.h"
#include "foc_comm_usart.h"   // 复用现有控制模式选择，不建立第二套模式状态
#define CAN_POSITION_FULL_CIRCLE 36000U   // uint16单圈角度：0～35999，单位0.01度
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

    return HAL_OK;  // CAN启动成功
}

void FOC_CAN_ProcessRx(void)
{
    FDCAN_RxHeaderTypeDef header = {0};
    uint8_t data[64] = {0};

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, data) != HAL_OK) break;
        if (header.Identifier != 0x100U || header.IdType != FDCAN_STANDARD_ID ||
            header.RxFrameType != FDCAN_DATA_FRAME || header.FDFormat != FDCAN_CLASSIC_CAN ||
            header.DataLength != FDCAN_DLC_BYTES_8) continue;

        foc_can_rx_count++;
        for (uint32_t i = 0U; i < 8U; ++i) foc_can_rx_data[i] = data[i];
        // 持续接收；ALIGNING中只执行STOP，不缓存旧目标等待以后执行。
        if (foc_motor.motor_state == FOC_STATE_ALIGNING && data[0] != 0x05U) continue;
        uint16_t raw = (uint16_t)data[1] | ((uint16_t)data[2] << 8);

        switch (data[0])
        {
            case 0x01:
                if (raw <= (uint16_t)FOC_CONTROL_POSITION)
                    FOC_COMM_USART_SetControlMode((FOC_CONTROL_MODE)raw);
                break;
            case 0x02:
                if (FOC_COMM_USART_GetControlMode() == FOC_CONTROL_SPEED)
                {
                    float rpm = (float)(int16_t)raw;
                    if (rpm > MOTOR_MAX_SPEED_RPM) rpm = MOTOR_MAX_SPEED_RPM;
                    if (rpm < -MOTOR_MAX_SPEED_RPM) rpm = -MOTOR_MAX_SPEED_RPM;
                    foc_motor.speed_target = rpm;
                    if (foc_motor.motor_state == FOC_STATE_RUNNING_SPEED)
                        FOC_LOOP_SPD_SetSpeedRef(&foc_motor.loop_spd, rpm);
                }
                break;
            case 0x03:
                if (FOC_COMM_USART_GetControlMode() == FOC_CONTROL_POSITION &&
                    raw < CAN_POSITION_FULL_CIRCLE)
                {
                    foc_motor.position_target = (float)raw * FOC_PI / 18000.0f;
                    if (foc_motor.motor_state == FOC_STATE_RUNNING_POSITION)
                        FOC_LOOP_POS_SetPositionRef(&foc_motor.loop_pos, foc_motor.position_target);
                }
                break;
            case 0x04:
                FOC_Motor_Start();
                break;
            case 0x05:
                FOC_Motor_Stop();
                break;
            default:
                break;
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

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
        return;                            // 硬件发送FIFO满时跳过本帧，不阻塞FOC主循环

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data) != HAL_OK)
        return;                            // 提交失败时不等待重试，HAL_OK也仅表示已提交发送
}

// 将int16按小端格式放入两个字节
static void CAN_PutInt16(uint8_t *data, int16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)((uint16_t)value >> 8);
}

/* ==================== 单圈角度打包 ==================== */
static void CAN_PutAngle(uint8_t *data, float angle_rad)
{
    uint16_t angle = (uint16_t)(angle_rad * 18000.0f / FOC_PI + 0.5f);   // rad转换为角度×100，四舍五入到0.01度
    if (angle >= CAN_POSITION_FULL_CIRCLE) angle = 0U;   // 359.995度舍入到360度时回到单圈零点
    data[0] = (uint8_t)angle;   // 使用uint16，保持大于327.67度的角度仍为正数
    data[1] = (uint8_t)(angle >> 8);
}

void FOC_CAN_SendFeedback(void)
{
    static uint32_t last_tick = 0;
    static uint8_t feedback_index = 0;   // 仅选择本次反馈类型，不缓存报文、不建立软件FIFO
    uint32_t now = HAL_GetTick();

    // 硬件发送FIFO只有3个槽；四类反馈分散发送，避免同时提交4帧使最后一类一直被跳过。
    if ((uint32_t)(now - last_tick) < 25U)
        return;

    last_tick = now;
    uint8_t data[8] = {0};   // HAL立即复制到硬件消息RAM，函数返回后不再使用此数组
    switch (feedback_index)
    {
        case 0:   // 0x180：Byte0状态、Byte1模式，余下字节为0
            data[0] = (uint8_t)foc_motor.motor_state;
            data[1] = (uint8_t)FOC_COMM_USART_GetControlMode();
            FOC_CAN_Send(0x180U, data);
            break;
        case 1:   // 0x181：实际角度、目标角度，uint16小端，单位0.01度
            CAN_PutAngle(&data[0], foc_motor.encoder.angle_m);
            CAN_PutAngle(&data[2], foc_motor.loop_pos.position_ref);
            FOC_CAN_Send(0x181U, data);
            break;
        case 2:   // 0x182：Iq_ref、Iq、Id_ref、Id，int16小端，电流×100
            CAN_PutInt16(&data[0], (int16_t)(foc_motor.loop_cur.iq_ref * 100.0f));
            CAN_PutInt16(&data[2], (int16_t)(foc_motor.loop_cur.i_q * 100.0f));
            CAN_PutInt16(&data[4], (int16_t)(foc_motor.loop_cur.id_ref * 100.0f));
            CAN_PutInt16(&data[6], (int16_t)(foc_motor.loop_cur.i_d * 100.0f));
            FOC_CAN_Send(0x182U, data);
            break;
        case 3:   // 0x183：实际速度、目标速度，int16小端，单位rpm
            CAN_PutInt16(&data[0], (int16_t)foc_motor.loop_spd.speed_fbk);
            CAN_PutInt16(&data[2], (int16_t)foc_motor.loop_spd.speed_ref);
            FOC_CAN_Send(0x183U, data);
            break;
        default: break;
    }
    feedback_index = (uint8_t)((feedback_index + 1U) % 4U);   // 每类正常反馈周期100ms，发送满时不阻塞重试
}
