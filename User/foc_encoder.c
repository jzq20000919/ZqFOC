/**
 * @file    foc_encoder.c
 * @brief   编码器角度与转速实现
 */
#include "foc_encoder.h"
#include "motor_param.h"
#include "foc_math.h"
#include <math.h>
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
    const int32_t counts_per_rev = (int32_t)ENCODER_CPR;
    count_now = __HAL_TIM_GET_COUNTER(encoder->htim);
    d_count = count_now - encoder->spd_last_count;
    // 用有符号计数比较，避免无符号转换把微小计数差误判为整圈回绕。
    if (d_count > (counts_per_rev / 2))
    {
        d_count -= counts_per_rev;
    }
    else if (d_count < -(counts_per_rev / 2))
    {
        d_count += counts_per_rev;
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

void FOC_ENCODER_CalibrateElectricalOffset(FOC_ENCODER_HandleTypeDef *encoder,float align_angle)
{
    float offset;
    // 以固定磁场对齐后的转子位置计算电角度零点。
    encoder->count = __HAL_TIM_GET_COUNTER(encoder->htim);
    encoder->angle_m =(float)encoder->count *FOC_TWO_PI /(float)ENCODER_CPR;
    offset = align_angle -encoder->angle_m * MOTOR_POLE_PAIRS;
    FOC_ENCODER_SetElectricalOffset(encoder, offset);
    // 同步角度与测速历史，避免首次速度更新出现计数跳变。
    FOC_ENCODER_UpdateAngle(encoder);
    encoder->spd_last_count = encoder->count;
    encoder->speed = 0.0f;
}

HAL_StatusTypeDef FOC_ENCODER_CalibrateElectricalOffsetSamples(
    FOC_ENCODER_HandleTypeDef *encoder,
    float align_angle,
    const float *angle_samples,
    uint8_t sample_count)
{
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;

    if (encoder == NULL || angle_samples == NULL || sample_count == 0U)
    {
        return HAL_ERROR;
    }

    // 圆周平均可避免机械角在0/2π处跨界时得到错误的平均值。
    for (uint8_t i = 0U; i < sample_count; ++i)
    {
        if (!isfinite(angle_samples[i]))
        {
            return HAL_ERROR;
        }
        sum_sin += sinf(angle_samples[i]);
        sum_cos += cosf(angle_samples[i]);
    }
    if (sum_sin * sum_sin + sum_cos * sum_cos < 1.0f)
    {
        return HAL_ERROR;
    }

    float angle_m_avg = FOC_MATH_Normalize(atan2f(sum_sin, sum_cos));
    float offset = align_angle - angle_m_avg * MOTOR_POLE_PAIRS;
    FOC_ENCODER_SetElectricalOffset(encoder, offset);
    FOC_ENCODER_UpdateAngle(encoder);
    return HAL_OK;
}
