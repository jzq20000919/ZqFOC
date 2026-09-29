#include "foc_bus_voltage.h"

void FOC_BUS_VOLTAGE_Init(FOC_BUS_VOLTAGE_HandleTypeDef *bus, ADC_HandleTypeDef *hadc)
{
    bus->hadc = hadc;
    bus->raw = 0;
    bus->voltage = 0.0f;
}

void FOC_BUS_VOLTAGE_Update(FOC_BUS_VOLTAGE_HandleTypeDef *bus)
{
    HAL_ADC_Start(bus->hadc);
    HAL_ADC_PollForConversion(bus->hadc, 1);
    bus->raw = (uint16_t)HAL_ADC_GetValue(bus->hadc);
    bus->voltage = (float)bus->raw * (3.3f / 4095.0f) * 26.0f;
}
