#ifndef FOC_CAN_H
#define FOC_CAN_H

#include "stm32g4xx_hal.h"

HAL_StatusTypeDef FOC_CAN_Init(void);   /**< @brief 配置接收过滤器并启动CAN，返回HAL初始化状态。 */
void FOC_CAN_ProcessRx(void);          /**< @brief 在主循环中轮询硬件FIFO0并执行速度控制命令。 */
void FOC_CAN_SendFeedback(void);       /**< @brief 每100ms尝试发送状态、速度及电流反馈，发送满时跳过。 */
#endif
