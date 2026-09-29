/**
 * @file    foc_current.h
 * @brief   A、B相电流采样与C相重构接口
 */
#if !defined(FOC_CURRENT_H)
#define FOC_CURRENT_H
#include "stm32g4xx_hal.h"
#include "motor_param.h"
typedef struct
{
    ADC_HandleTypeDef *hadc1; // A相电流采样ADC句柄
    ADC_HandleTypeDef *hadc2; // B相电流采样ADC句柄

    uint16_t raw_a; // A相ADC原始值，计数
    uint16_t raw_b; // B相ADC原始值，计数
    uint16_t raw_c; // 仅兼容旧VOFA诊断通道，不再采样或参与控制，保持为0

    float i_a; // A相电流，A
    float i_b; // B相电流，A
    float i_c; // 由A、B相重构的C相电流，A

} FOC_CURRENT_HandleTypeDef;

/**
 * @brief  初始化电流采样句柄
 * @param  current 电流采样句柄
 * @param  hadc1 A相电流采样ADC句柄
 * @param  hadc2 B相电流采样ADC句柄
 */
void FOC_CURRENT_Init(FOC_CURRENT_HandleTypeDef *current,ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);
/**
 * @brief  读取A、B相注入转换结果并重构C相电流
 * @param  current 电流采样句柄
 */
void FOC_CURRENT_Update(FOC_CURRENT_HandleTypeDef *current);

#endif // FOC_CURRENT_H

