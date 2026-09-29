/**
 * @file    foc_svpwm.h
 * @brief   SVPWM输出接口
 */
#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H
#include "stm32g4xx_hal.h"
#include "motor_param.h"

typedef struct 
{
    TIM_HandleTypeDef *htim; // 三相PWM定时器句柄
    float duty_u;            // U相占空比，0～1
    float duty_v;            // V相占空比，0～1
    float duty_w;            // W相占空比，0～1
} FOC_SVPWM_HandleTypeDef;

/**
 * @brief  初始化SVPWM句柄
 * @param  svpwm SVPWM句柄
 * @param  htim 三相PWM定时器句柄
 */
void FOC_SVPWM_Init(FOC_SVPWM_HandleTypeDef *svpwm, TIM_HandleTypeDef *htim);
/**
 * @brief  启动三相PWM输出
 * @param  svpwm SVPWM句柄
 */
void FOC_SVPWM_Start(FOC_SVPWM_HandleTypeDef *svpwm);
/**
 * @brief  将静止坐标系电压转换为三相PWM占空比
 * @param  svpwm SVPWM句柄
 * @param  v_alpha α轴电压，V
 * @param  v_beta β轴电压，V
 * @param  v_bus 母线电压，V
 */
void FOC_SVPWM_Update(FOC_SVPWM_HandleTypeDef *svpwm,   float v_alpha, float v_beta, float v_bus);



#endif // FOC_SVPWM_H
