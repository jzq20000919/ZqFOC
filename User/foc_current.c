#include "stm32g4xx_hal_adc_ex.h"
#include "foc_current.h"
void FOC_Current_Init(FOC_Current_HandleTypeDef *current,ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2)
{
    current->hadc1 = hadc1;
    current->hadc2 = hadc2;

    current->raw_a = 0;
    current->raw_b = 0;
    current->raw_c = 0;

    current->I_a = 0.0f;
    current->I_b = 0.0f;
    current->I_c = 0.0f;
}

void FOC_Current_Update(FOC_Current_HandleTypeDef *current)
{
    current-> raw_a = HAL_ADCEx_InjectedGetValue(current->hadc1, ADC_INJECTED_RANK_1);
    current-> raw_b = HAL_ADCEx_InjectedGetValue(current->hadc2, ADC_INJECTED_RANK_1);
    current-> raw_c = HAL_ADCEx_InjectedGetValue(current->hadc1, ADC_INJECTED_RANK_2);
    current->I_a = (float)current->raw_a * CURRENT_SCALE;
    current->I_b = (float)current->raw_b * CURRENT_SCALE;
    current->I_c = (float)current->raw_c * CURRENT_SCALE;
}