#ifndef FOC_BUS_VOLTAGE_H
#define FOC_BUS_VOLTAGE_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

typedef struct
{
    ADC_HandleTypeDef *hadc;
    uint16_t raw;
    volatile float voltage;
} FOC_BUS_VOLTAGE_HandleTypeDef;

void FOC_BUS_VOLTAGE_Init(FOC_BUS_VOLTAGE_HandleTypeDef *bus, ADC_HandleTypeDef *hadc);
void FOC_BUS_VOLTAGE_Update(FOC_BUS_VOLTAGE_HandleTypeDef *bus);

#endif // FOC_BUS_VOLTAGE_H
