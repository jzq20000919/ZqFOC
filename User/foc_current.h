#if !defined(FOC_CURRENT_H)
#define FOC_CURRENT_H
#include "stm32g4xx_hal.h"
#include "motor_param.h"
typedef struct
{
    ADC_HandleTypeDef *hadc1;
    ADC_HandleTypeDef *hadc2;

    uint16_t raw_a;
    uint16_t raw_b;
    uint16_t raw_c;

    float i_a;
    float i_b;
    float i_c;

} FOC_CURRENT_HandleTypeDef;

void FOC_CURRENT_Init(FOC_CURRENT_HandleTypeDef *current,ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);
void FOC_CURRENT_Update(FOC_CURRENT_HandleTypeDef *current);

#endif // FOC_CURRENT_H

