#ifndef FOC_CAN_H
#define FOC_CAN_H

#include "stm32g4xx_hal.h"

HAL_StatusTypeDef FOC_CAN_Init(void);   // 初始化CAN通信
void FOC_CAN_ProcessRx(void);  // 在主循环中取出并解析一条CAN消息
#endif