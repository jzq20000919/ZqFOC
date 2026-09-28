#include "foc_encoder.h"
#include "motor_param.h"
#include "foc_math.h"
//初始化
void FOC_ENCODER_Init(FOC_ENCODER_HandleTypeDef *encoder, TIM_HandleTypeDef *htim)
{
    encoder->htim = htim;
    encoder->last_count = 0;
    encoder->count = encoder->last_count;
    encoder->angle_M = 0.0f;
    encoder->angle_E = 0.0f;
    encoder->angle_E_offset = 0.0f;
    encoder->speed = 0.0f;
    __HAL_TIM_SET_COUNTER(encoder->htim, 0);
    HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL); 
}
//编码器状态更新
void FOC_ENCODER_Update(FOC_ENCODER_HandleTypeDef *encoder, float dt)
{
    int32_t d_count;
    encoder->count = __HAL_TIM_GET_COUNTER(encoder->htim);
    encoder->angle_M = (float)encoder->count * FOC_TWO_PI / (float)ENCODER_CPR;
    encoder->angle_E = encoder->angle_M * MOTOR_POLE_PAIRS + encoder->angle_E_offset;
    encoder->angle_E = FOC_Normalize(encoder->angle_E);
    d_count = encoder->count - encoder->last_count;

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
    encoder->last_count = encoder->count;

}
//获取偏置
void FOC_ENCODER_Set_E_Offset(FOC_ENCODER_HandleTypeDef *encoder, float offset)
{
    encoder->angle_E_offset = FOC_Normalize(offset);
}