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

    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        return HAL_ERROR;                   // 启用接收中断通知失败

    return HAL_OK;                          // 初始化成功
}
/**
 * @brief CAN FIFO0接收回调函数
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan != &hfdcan1) return;  // 确认来自FDCAN1

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;  // 确认有新消息

    FDCAN_RxHeaderTypeDef rx_header = {0};  // 保存接收报文的ID、长度等信息
    uint8_t rx_data[8] = {0};               // 临时保存接收的数据

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
            return;  // 读取失败则退出

        if (rx_header.IdType != FDCAN_STANDARD_ID ||
            rx_header.Identifier != 0x100U ||
            rx_header.RxFrameType != FDCAN_DATA_FRAME ||
            rx_header.DataLength != FDCAN_DLC_BYTES_8)
            continue;  // 只处理符合协议的8字节控制帧

        for (uint8_t i = 0; i < 8U; i++)
        {
            foc_can_rx_data[i] = rx_data[i];  // 保存接收数据
        }

        foc_can_rx_count++;  // 成功接收数量加1
    }
}
/**
 * @brief 解析ESP32发送的CAN控制命令
 */
uint8_t FOC_CAN_Decode(const uint8_t data[8], FOC_CAN_Command_t *cmd)
{
    if (data == NULL || cmd == NULL) return 0U;  // 检查指针

    switch (data[0])
    {
        case 0x01:  // SET_MODE
        case 0x02:  // SET_SPEED
        case 0x03:  // SET_POSITION
        case 0x04:  // START
        case 0x05:  // STOP
        case 0x06:  // CLEAR_FAULT
            break;

        default:
            return 0U;  // 未知命令
    }

    cmd->type = data[0];  // 保存命令编号

    uint16_t raw = (uint16_t)data[1] | ((uint16_t)data[2] << 8); // 合并参数
    cmd->value = (int16_t)raw;  // 按有符号16位整数解释

    return 1U;  // 解析成功
}