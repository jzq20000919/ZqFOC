#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H
#include "stm32g4xx_hal.h"
#include "motor_param.h"

#define SVPWM_DUTY_MIN 0.05f
#define SVPWM_DUTY_MAX 0.95f

typedef struct 
{
    TIM_HandleTypeDef *htim;
    float duty_u;
    float duty_v;
    float duty_w;
} FOC_SVPWM_HandleTypeDef;

void FOC_SVPWM_Init(FOC_SVPWM_HandleTypeDef *svpwm, TIM_HandleTypeDef *htim);
void FOC_SVPWM_Update(FOC_SVPWM_HandleTypeDef *svpwm,   float v_alpha, float v_beta, float v_bus);


#endif // FOC_SVPWM_H