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
 * @note   调用会等待转换完成，应在主循环中使用；不停止ADC，以保留注入组转换。
 */
void FOC_BUS_VOLTAGE_Update(FOC_BUS_VOLTAGE_HandleTypeDef *bus);

#endif // FOC_BUS_VOLTAGE_H
