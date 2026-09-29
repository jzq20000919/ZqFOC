#ifndef FOC_ENCODER_H
#define FOC_ENCODER_H
#include "stm32g4xx_hal.h"
#include "motor_param.h"

typedef struct
{
    TIM_HandleTypeDef *htim;//编码器接在哪个定时器上
    int32_t spd_last_count;//上一次的计数值
    int32_t count;//当前计数值
    float angle_M;//机械角度
    float angle_E;//电角度
    float angle_E_offset;//电角度零点偏置
    float speed;//rpm
} FOC_ENCODER_HandleTypeDef;

//初始化,需要一个定时器句柄和一个编码器目标
void FOC_ENCODER_Init(FOC_ENCODER_HandleTypeDef *encoder, TIM_HandleTypeDef *htim);

//更新编码器角度,需要一个编码器句柄和时间间隔dt
void FOC_ENCODER_UpdateAngle(FOC_ENCODER_HandleTypeDef *encoder);
//更新编码器速度
void FOC_ENCODER_UpdateSpeed(FOC_ENCODER_HandleTypeDef *encoder, float dt);

//设置电角度零点偏置
void FOC_ENCODER_Set_E_Offset(FOC_ENCODER_HandleTypeDef *encoder, float offset);


#endif