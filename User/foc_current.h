/**
 * @file    foc_current.h
 * @brief   三相电流采样接口
 */
#if !defined(FOC_CURRENT_H)
#define FOC_CURRENT_H
#include "stm32g4xx_hal.h"
#include "motor_param.h"
typedef struct
{
    ADC_HandleTypeDef *hadc1; // A、C相电流采样ADC句柄
    ADC_HandleTypeDef *hadc2; // B相电流采样ADC句柄

    uint16_t raw_a; // A相ADC原始值，计数
    uint16_t raw_b; // B相ADC原始值，计数
    uint16_t raw_c; // C相ADC原始值，计数

    float i_a; // A相电流，A
    float i_b; // B相电流，A
    float i_c; // C相电流，A

} FOC_CURRENT_HandleTypeDef;

/**
 * @brief  初始化三相电流采样句柄
 * @param  current 三相电流句柄
 * @param  hadc1 A、C相电流采样ADC句柄
 * @param  hadc2 B相电流采样ADC句柄
 */
void FOC_CURRENT_Init(FOC_CURRENT_HandleTypeDef *current,ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);
/**
 * @brief  读取注入组转换结果并换算三相电流
 * @param  current 三相电流句柄
 */
void FOC_CURRENT_Update(FOC_CURRENT_HandleTypeDef *current);

#endif // FOC_CURRENT_H

