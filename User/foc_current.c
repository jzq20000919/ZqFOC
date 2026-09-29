/**
 * @file    foc_current.c
 * @brief   三相电流采样实现
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
    // ADC1注入序列采A、C相，ADC2注入序列采B相。
    current-> raw_a = HAL_ADCEx_InjectedGetValue(current->hadc1, ADC_INJECTED_RANK_1);
    current-> raw_b = HAL_ADCEx_InjectedGetValue(current->hadc2, ADC_INJECTED_RANK_1);
    current-> raw_c = HAL_ADCEx_InjectedGetValue(current->hadc1, ADC_INJECTED_RANK_2);
    // 扣除各相零电流偏置，再按分流电阻与放大倍数换算为安培。
    current->i_a = ((float)current->raw_a - ADC_CURRENT_OFFSET_A) * CURRENT_SCALE;
    current->i_b = ((float)current->raw_b - ADC_CURRENT_OFFSET_B) * CURRENT_SCALE;
    current->i_c = ((float)current->raw_c - ADC_CURRENT_OFFSET_C) * CURRENT_SCALE;
}
