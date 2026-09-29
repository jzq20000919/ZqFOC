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
    v_u = v_alpha;
    v_v = -0.5f *v_alpha + 0.5*FOC_SQRT3 * v_beta;
    v_w = -0.5f *v_alpha - 0.5*FOC_SQRT3 * v_beta;
    v_max = fmaxf(fmaxf(v_u, v_v), v_w);
    v_min = fminf(fminf(v_u, v_v), v_w);
    v_offset = (v_max + v_min) * -0.5f;
    v_u += v_offset;
    v_v += v_offset;
    v_w += v_offset;
    
    svpwm->duty_u = 0.5f + v_u / v_bus ;
    svpwm->duty_v = 0.5f + v_v / v_bus ;
    svpwm->duty_w = 0.5f + v_w / v_bus ;

    //ÏÞ·ù
    if(svpwm->duty_u > SVPWM_DUTY_MAX) svpwm->duty_u = SVPWM_DUTY_MAX;
    if(svpwm->duty_u < 0.0f) svpwm->duty_u = 0.0f;
    if(svpwm->duty_v > SVPWM_DUTY_MAX) svpwm->duty_v = SVPWM_DUTY_MAX;
    if(svpwm->duty_v < 0.0f) svpwm->duty_v = 0.0f;
    if(svpwm->duty_w > SVPWM_DUTY_MAX) svpwm->duty_w = SVPWM_DUTY_MAX;
    if(svpwm->duty_w < 0.0f) svpwm->duty_w = 0.0f;

    //Õ¼¿Õ±È-CCR
    arr = __HAL_TIM_GET_AUTORELOAD(svpwm->htim);
    __HAL_TIM_SET_COMPARE(svpwm->htim, TIM_CHANNEL_1, (uint32_t)(svpwm->duty_u * arr));
    __HAL_TIM_SET_COMPARE(svpwm->htim, TIM_CHANNEL_2, (uint32_t)(svpwm->duty_v * arr));
    __HAL_TIM_SET_COMPARE(svpwm->htim, TIM_CHANNEL_3, (uint32_t)(svpwm->duty_w * arr));
    // Implementation for updating SVPWM
}