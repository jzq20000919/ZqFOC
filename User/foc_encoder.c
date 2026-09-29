/**
 * @file    foc_encoder.c
 * @brief   编码器角度与转速实现
 */
#include "foc_encoder.h"
#include "motor_param.h"
#include "foc_math.h"
void FOC_ENCODER_Init(FOC_ENCODER_HandleTypeDef *encoder, TIM_HandleTypeDef *htim)
{
    encoder->htim = htim;
    encoder->spd_last_count = 0;
    encoder->count = 0;
    encoder->angle_m = 0.0f;
    encoder->angle_e = 0.0f;
    encoder->angle_e_offset = ENCODER_ELECTRICAL_OFFSET;
    encoder->speed = 0.0f;
    __HAL_TIM_SET_COUNTER(encoder->htim, 0);
    HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL); 
}
void FOC_ENCODER_UpdateAngle(FOC_ENCODER_HandleTypeDef *encoder)
{
    // 1. 用定时器计数换算单圈机械角。
    encoder->count = __HAL_TIM_GET_COUNTER(encoder->htim);

    encoder->angle_m =(float)encoder->count* FOC_TWO_PI/ (float)ENCODER_CPR;

    // 2. 结合极对数与零点偏置得到电角度，并归一化到单圈。
    encoder->angle_e =encoder->angle_m * MOTOR_POLE_PAIRS+ encoder->angle_e_offset;
    encoder->angle_e =FOC_MATH_Normalize(encoder->angle_e);
}
void FOC_ENCODER_UpdateSpeed(FOC_ENCODER_HandleTypeDef *encoder, float dt)
{
    int32_t count_now;
    int32_t d_count;
    count_now = __HAL_TIM_GET_COUNTER(encoder->htim);
    d_count = count_now - encoder->spd_last_count;
    // 按单圈计数修正回绕，避免过零时产生虚假的转速尖峰。
    if (d_count > (ENCODER_CPR / 2))
    {
        d_count -= ENCODER_CPR;
    }
    else if (d_count < -(ENCODER_CPR / 2))
    {
        d_count += ENCODER_CPR;
    }
    if(dt >0.0f)
    {
        encoder->speed = (float)d_count * 60.0f / (float)ENCODER_CPR / dt;
    }
    else
    {
        encoder->speed = 0.0f;
    }
    encoder->spd_last_count = count_now;
}
void FOC_ENCODER_SetElectricalOffset(FOC_ENCODER_HandleTypeDef *encoder, float offset)
{
    encoder->angle_e_offset = FOC_MATH_Normalize(offset);
}

void FOC_ENCODER_CalibrateElectricalOffset(
    FOC_ENCODER_HandleTypeDef *encoder,
    float align_angle)
{
    float offset;
    // 以固定磁场对齐后的转子位置计算电角度零点。
    encoder->count = __HAL_TIM_GET_COUNTER(encoder->htim);

    encoder->angle_m =
        (float)encoder->count *
        FOC_TWO_PI /
        (float)ENCODER_CPR;
    offset = align_angle -encoder->angle_m * MOTOR_POLE_PAIRS;
    FOC_ENCODER_SetElectricalOffset(encoder, offset);
    // 同步角度与测速历史，避免首次速度更新出现计数跳变。
    FOC_ENCODER_UpdateAngle(encoder);
    encoder->spd_last_count = encoder->count;
    encoder->speed = 0.0f;
}
