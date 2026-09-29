#include "foc_encoder.h"
#include "motor_param.h"
#include "foc_math.h"
//初始化
void FOC_ENCODER_Init(FOC_ENCODER_HandleTypeDef *encoder, TIM_HandleTypeDef *htim)
{
    encoder->htim = htim;
    encoder->spd_last_count = 0;
    encoder->count = 0;
    encoder->angle_M = 0.0f;
    encoder->angle_E = 0.0f;
    encoder->angle_E_offset = 0.0f;
    encoder->speed = 0.0f;
    __HAL_TIM_SET_COUNTER(encoder->htim, 0);
    HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL); 
}
//编码器角度状态更新
void FOC_ENCODER_UpdateAngle(FOC_ENCODER_HandleTypeDef *encoder)
{
    /* 1. 读取当前编码器计数 */
    encoder->count = __HAL_TIM_GET_COUNTER(encoder->htim);

    /* 2. 计算单圈机械角，范围 0 ~ 2π */
    encoder->angle_M =(float)encoder->count* FOC_TWO_PI/ (float)ENCODER_CPR;

    /* 3. 机械角 -> 电角度 */
    encoder->angle_E =encoder->angle_M * MOTOR_POLE_PAIRS+ encoder->angle_E_offset;
    /* 4. 电角度限制在 0 ~ 2π */
    encoder->angle_E =FOC_Normalize(encoder->angle_E);
}
//编码器速度状态更新
void FOC_ENCODER_UpdateSpeed(FOC_ENCODER_HandleTypeDef *encoder, float dt)
{
    int32_t count1;
    int32_t d_count;
    count1 = __HAL_TIM_GET_COUNTER(encoder->htim);
    d_count = count1 - encoder->spd_last_count;
    //处理角度回绕
    if (d_count > (ENCODER_CPR / 2))
    {
        d_count -= ENCODER_CPR;
    }
    else if (d_count < -(ENCODER_CPR / 2))
    {
        d_count += ENCODER_CPR;
    }
    //计算速度，单位为rpm
    if(dt >0.0f)
    {
        encoder->speed = (float)d_count * 60.0f / (float)ENCODER_CPR / dt;
    }
    else
    {
        encoder->speed = 0.0f;
    }
    //保存此次的计数值为上一次的计数值
    encoder->spd_last_count = count1;
}
//获取偏置
void FOC_ENCODER_Set_E_Offset(FOC_ENCODER_HandleTypeDef *encoder, float offset)
{
    encoder->angle_E_offset = FOC_Normalize(offset);
}