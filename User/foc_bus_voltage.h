/**
 * @file    foc_bus_voltage.h
 * @brief   母线电压采样接口
 */
#ifndef FOC_BUS_VOLTAGE_H
#define FOC_BUS_VOLTAGE_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

typedef struct
{
    ADC_HandleTypeDef *hadc; // 母线电压采样使用的ADC句柄
    uint16_t raw;             // ADC原始采样值，计数
    volatile float voltage;   // 母线电压，V
    uint32_t last_tick;
    uint8_t pending;          // 已触发常规转换，稍后读取
} FOC_BUS_VOLTAGE_HandleTypeDef;

/**
 * @brief  初始化母线电压采样句柄
 * @param  bus 母线电压句柄
 * @param  hadc 母线电压采样使用的ADC句柄
 */
void FOC_BUS_VOLTAGE_Init(FOC_BUS_VOLTAGE_HandleTypeDef *bus, ADC_HandleTypeDef *hadc);
/**
 * @brief  通过ADC常规组软件转换更新母线电压
 * @param  bus 母线电压句柄
 * @note 主循环反复调用：每10ms触发，完成后读取；不等待、不停止注入ADC。
 */
void FOC_BUS_VOLTAGE_Update(FOC_BUS_VOLTAGE_HandleTypeDef *bus);

#endif // FOC_BUS_VOLTAGE_H
