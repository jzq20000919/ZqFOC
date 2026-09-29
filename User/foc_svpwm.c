/**
 * @file    foc_svpwm.c
 * @brief   SVPWM占空比计算与输出
 */
#include "foc_svpwm.h"
#include "motor_param.h"
#include <math.h>

void FOC_SVPWM_Init(FOC_SVPWM_HandleTypeDef *svpwm, TIM_HandleTypeDef *htim)
{
    svpwm->htim = htim;
    svpwm->duty_u = 0.5f;
    svpwm->duty_v = 0.5f;
    svpwm->duty_w = 0.5f;
}

void FOC_SVPWM_Start(FOC_SVPWM_HandleTypeDef *svpwm)
{
    HAL_TIM_PWM_Start(svpwm->htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(svpwm->htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(svpwm->htim, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(svpwm->htim, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(svpwm->htim, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(svpwm->htim, TIM_CHANNEL_3);
}

void FOC_SVPWM_Update(FOC_SVPWM_HandleTypeDef *svpwm,   float v_alpha, float v_beta, float v_bus)
{
    float v_u, v_v, v_w;
    float v_max, v_min;
    float v_offset;
    uint32_t arr;

    if(v_bus <= 0.0f)
    {
        return;
    }
    // 1. 将α、β轴电压换算为三相电压。
    v_u = v_alpha;
    v_v = -0.5f *v_alpha + 0.5*FOC_SQRT3 * v_beta;
    v_w = -0.5f *v_alpha - 0.5*FOC_SQRT3 * v_beta;
    v_max = fmaxf(fmaxf(v_u, v_v), v_w);
    v_min = fminf(fminf(v_u, v_v), v_w);
    // 2. 注入公共偏置，使三相电压在母线范围内居中。
    v_offset = (v_max + v_min) * -0.5f;
    v_u += v_offset;
    v_v += v_offset;
    v_w += v_offset;
    
    svpwm->duty_u = 0.5f + v_u / v_bus ;
    svpwm->duty_v = 0.5f + v_v / v_bus ;
    svpwm->duty_w = 0.5f + v_w / v_bus ;

    // 3. 限制占空比，避开过窄的PWM脉冲。
    if(svpwm->duty_u > SVPWM_DUTY_MAX) svpwm->duty_u = SVPWM_DUTY_MAX;
    if(svpwm->duty_u < SVPWM_DUTY_MIN) svpwm->duty_u = SVPWM_DUTY_MIN;
    if(svpwm->duty_v > SVPWM_DUTY_MAX) svpwm->duty_v = SVPWM_DUTY_MAX;
    if(svpwm->duty_v < SVPWM_DUTY_MIN) svpwm->duty_v = SVPWM_DUTY_MIN;
    if(svpwm->duty_w > SVPWM_DUTY_MAX) svpwm->duty_w = SVPWM_DUTY_MAX;
    if(svpwm->duty_w < SVPWM_DUTY_MIN) svpwm->duty_w = SVPWM_DUTY_MIN;

    // 4. 按定时器自动重装值写入三相比较寄存器。
    arr = __HAL_TIM_GET_AUTORELOAD(svpwm->htim);
    __HAL_TIM_SET_COMPARE(svpwm->htim, TIM_CHANNEL_1, (uint32_t)(svpwm->duty_u * arr));
    __HAL_TIM_SET_COMPARE(svpwm->htim, TIM_CHANNEL_2, (uint32_t)(svpwm->duty_v * arr));
    __HAL_TIM_SET_COMPARE(svpwm->htim, TIM_CHANNEL_3, (uint32_t)(svpwm->duty_w * arr));
}
