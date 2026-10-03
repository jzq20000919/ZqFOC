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
    current->offset_a = ADC_CURRENT_OFFSET_A;
    current->offset_b = ADC_CURRENT_OFFSET_B;

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
    current->i_a = (current->offset_a - (float)current->raw_a) * CURRENT_SCALE;
    current->i_b = (current->offset_b - (float)current->raw_b) * CURRENT_SCALE;
    // 三相星形电机满足ia + ib + ic = 0，C相由A、B相重构。
    current->i_c = -(current->i_a + current->i_b);
}

HAL_StatusTypeDef FOC_CURRENT_CalibrateOffset(
    FOC_CURRENT_HandleTypeDef *current,
    TIM_HandleTypeDef *pwm_timer)
{
    const uint32_t phase_mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                                TIM_CCER_CC2E | TIM_CCER_CC2NE |
                                TIM_CCER_CC3E | TIM_CCER_CC3NE;
    uint32_t sum_a = 0U;
    uint32_t sum_b = 0U;

    if (current == NULL || pwm_timer == NULL ||
        current->hadc1 == NULL || current->hadc2 == NULL ||
        (pwm_timer->Instance->CCER & phase_mask) != 0U)
    {
        return HAL_ERROR;
    }

    // 清除启动阶段的旧完成标志，后续每次只读取新触发的注入结果。
    __HAL_ADC_CLEAR_FLAG(current->hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
    __HAL_ADC_CLEAR_FLAG(current->hadc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
    for (uint32_t i = 0U; i < CURRENT_OFFSET_SAMPLE_COUNT; ++i)
    {
        uint32_t start_tick = HAL_GetTick();
        while (!__HAL_ADC_GET_FLAG(current->hadc1, ADC_FLAG_JEOS) ||
               !__HAL_ADC_GET_FLAG(current->hadc2, ADC_FLAG_JEOS))
        {
            if ((uint32_t)(HAL_GetTick() - start_tick) >= CURRENT_OFFSET_SAMPLE_TIMEOUT_MS)
            {
                return HAL_TIMEOUT;
            }
        }
        current->raw_a = (uint16_t)HAL_ADCEx_InjectedGetValue(
            current->hadc1, ADC_INJECTED_RANK_1);
        current->raw_b = (uint16_t)HAL_ADCEx_InjectedGetValue(
            current->hadc2, ADC_INJECTED_RANK_1);
        sum_a += current->raw_a;
        sum_b += current->raw_b;
        __HAL_ADC_CLEAR_FLAG(current->hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
        __HAL_ADC_CLEAR_FLAG(current->hadc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
        HAL_Delay(CURRENT_OFFSET_SAMPLE_INTERVAL_MS);
    }

    current->offset_a = (float)sum_a / (float)CURRENT_OFFSET_SAMPLE_COUNT;
    current->offset_b = (float)sum_b / (float)CURRENT_OFFSET_SAMPLE_COUNT;
    return HAL_OK;
}
