/**
 * @file    foc_bus_voltage.c
 * @brief   母线电压采样实现
 */
#include "foc_bus_voltage.h"
#include "motor_param.h"

void FOC_BUS_VOLTAGE_Init(FOC_BUS_VOLTAGE_HandleTypeDef *bus, ADC_HandleTypeDef *hadc)
{
    bus->hadc = hadc;
    bus->raw = 0;
    bus->voltage = 0.0f;
}

void FOC_BUS_VOLTAGE_Update(FOC_BUS_VOLTAGE_HandleTypeDef *bus)
{
    // 常规组在主循环中软件触发；保留ADC运行状态供注入组采样使用。
    HAL_ADC_Start(bus->hadc);
    HAL_ADC_PollForConversion(bus->hadc, 1);
    bus->raw = (uint16_t)HAL_ADC_GetValue(bus->hadc);
    bus->voltage = (float)bus->raw * (ADC_VREF / ADC_FULL_SCALE) * BUS_VOLTAGE_DIVIDER_RATIO;
}
