#ifndef FOC_CAN_H
#define FOC_CAN_H

#include "stm32g4xx_hal.h"

HAL_StatusTypeDef FOC_CAN_Init(void);   /**< @brief 配置接收过滤器并启动CAN，返回HAL初始化状态。 */
void FOC_CAN_ProcessRx(void);          /**< @brief 主循环轮询硬件FIFO0，执行速度/单圈位置、启停命令；对齐后再应用缓存目标。 */
void FOC_CAN_SendFeedback(void);       /**< @brief 每25ms发送一类反馈，四类各100ms；硬件FIFO满时跳过，不阻塞FOC。 */
#endif
