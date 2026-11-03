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
    bus->last_tick = HAL_GetTick() - 10U;
    bus->pending = 0U;
}

void FOC_BUS_VOLTAGE_Update(FOC_BUS_VOLTAGE_HandleTypeDef *bus)
{
    if (bus->pending != 0U)
    {
        if (__HAL_ADC_GET_FLAG(bus->hadc, ADC_FLAG_EOC))
        {
            bus->raw = (uint16_t)HAL_ADC_GetValue(bus->hadc);
            bus->voltage = (float)bus->raw * (ADC_VREF / ADC_FULL_SCALE) * BUS_VOLTAGE_DIVIDER_RATIO;
            bus->pending = 0U;
        }
        return;
    }
    if ((uint32_t)(HAL_GetTick() - bus->last_tick) < 10U) return;
    bus->last_tick = HAL_GetTick();
    HAL_ADC_Start(bus->hadc);
    bus->pending = 1U;
}
