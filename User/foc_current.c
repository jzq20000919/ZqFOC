/**
 * @file    foc_current.c
 * @brief   A、B相电流采样与C相重构
 */
#include "foc_current.h"
#include "stm32g4xx_hal_adc_ex.h"
void FOC_CURRENT_Init(FOC_CURRENT_HandleTypeDef *current,ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2)
{
    current->hadc1 = hadc1;
    current->hadc2 = hadc2;

    current->raw_a = 0;
    current->raw_b = 0;
    current->raw_c = 0;

    current->i_a = 0.0f;
    current->i_b = 0.0f;
    current->i_c = 0.0f;
}

void FOC_CURRENT_Update(FOC_CURRENT_HandleTypeDef *current)
{
    // A、B两相由同一TIM1_CC4事件触发的ADC注入组采样。
    current->raw_a = HAL_ADCEx_InjectedGetValue(current->hadc1, ADC_INJECTED_RANK_1);
    current->raw_b = HAL_ADCEx_InjectedGetValue(current->hadc2, ADC_INJECTED_RANK_1);
    // 保持现有零点和换算系数，只对实际采样的A、B相做换算。
    current->i_a = ((float)current->raw_a - ADC_CURRENT_OFFSET_A) * CURRENT_SCALE;
    current->i_b = ((float)current->raw_b - ADC_CURRENT_OFFSET_B) * CURRENT_SCALE;
    // 三相星形电机满足ia + ib + ic = 0，C相由A、B相重构。
    current->i_c = -(current->i_a + current->i_b);
}
