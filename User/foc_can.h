#ifndef FOC_CAN_H
#define FOC_CAN_H

#include "stm32g4xx_hal.h"

HAL_StatusTypeDef FOC_CAN_Init(void);   // 初始化CAN通信
typedef struct
{
    uint8_t type;     // 命令编号
    int16_t value;    // 命令参数
} FOC_CAN_Command_t;

uint8_t FOC_CAN_Decode(const uint8_t data[8], FOC_CAN_Command_t *cmd);  // 解析CAN命令
void FOC_CAN_ProcessRx(void);  // 在主循环中取出并解析一条CAN消息
#endif