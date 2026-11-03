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
    current->offset_sum_a = 0U;
    current->offset_sum_b = 0U;
    current->offset_count = 0U;
    current->offset_tick = HAL_GetTick();
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

void FOC_CURRENT_StartSampling(FOC_CURRENT_HandleTypeDef *current,
                               TIM_HandleTypeDef *timer, uint8_t interrupt)
{
    HAL_ADCEx_InjectedStart(current->hadc2);
    if (interrupt) HAL_ADCEx_InjectedStart_IT(current->hadc1);
    else HAL_ADCEx_InjectedStart(current->hadc1);
    HAL_TIM_PWM_Start(timer, TIM_CHANNEL_4);
}

void FOC_CURRENT_StopSampling(FOC_CURRENT_HandleTypeDef *current, TIM_HandleTypeDef *timer)
{
    HAL_TIM_PWM_Stop(timer, TIM_CHANNEL_4);
    HAL_ADCEx_InjectedStop_IT(current->hadc1);
    HAL_ADCEx_InjectedStop(current->hadc2);
}

uint8_t FOC_CURRENT_CalibrateOffset(
    FOC_CURRENT_HandleTypeDef *current, TIM_HandleTypeDef *pwm_timer)
{
    // 已完成时只返回结果，不重复采样或关闭后续对齐/运行使用的外设。
    if (current->offset_count == CURRENT_OFFSET_SAMPLE_COUNT) return 1U;

    uint32_t elapsed = HAL_GetTick() - current->offset_tick;
    if (elapsed < CURRENT_OFFSET_SAMPLE_INTERVAL_MS) return 0U;
    if (!__HAL_ADC_GET_FLAG(current->hadc1, ADC_FLAG_JEOS) ||
        !__HAL_ADC_GET_FLAG(current->hadc2, ADC_FLAG_JEOS))
        return 0U;

    current->raw_a = (uint16_t)HAL_ADCEx_InjectedGetValue(current->hadc1, ADC_INJECTED_RANK_1);
    current->raw_b = (uint16_t)HAL_ADCEx_InjectedGetValue(current->hadc2, ADC_INJECTED_RANK_1);
    current->offset_sum_a += current->raw_a;
    current->offset_sum_b += current->raw_b;
    current->offset_count++;
    current->offset_tick = HAL_GetTick();
    __HAL_ADC_CLEAR_FLAG(current->hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
    __HAL_ADC_CLEAR_FLAG(current->hadc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS);

    if (current->offset_count < CURRENT_OFFSET_SAMPLE_COUNT) return 0U;
    current->offset_a = (float)current->offset_sum_a / CURRENT_OFFSET_SAMPLE_COUNT;
    current->offset_b = (float)current->offset_sum_b / CURRENT_OFFSET_SAMPLE_COUNT;
    FOC_CURRENT_StopSampling(current, pwm_timer);
    return 1U;
}
