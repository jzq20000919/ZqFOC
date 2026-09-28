#if !defined(FOC_CURRENT_H)
#define FOC_CURRENT_H
#include "stm32g4xx_hal.h"
// 定义常量
#define SHUNT_RESISTANCE 0.003f//3mOhm采样电阻
#define VREF 3.3f//参考电压
#define ADC_RESOLUTION 4095.0f//ADC分辨率
#define ADC_OFFSET 2048.0f//ADC偏移
#define AMP_GAIN 10.0f//电流放大倍数
#define CURRENT_SCALE (VREF / (ADC_RESOLUTION * SHUNT_RESISTANCE * AMP_GAIN))
typedef struct
{
    ADC_HandleTypeDef *hadc1;
    ADC_HandleTypeDef *hadc2;

    uint16_t raw_a;
    uint16_t raw_b;
    uint16_t raw_c;

    float I_a;
    float I_b;
    float I_c;

} FOC_Current_HandleTypeDef;

void FOC_Current_Init(FOC_Current_HandleTypeDef *current,ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);
void FOC_Current_Update(FOC_Current_HandleTypeDef *current);

#endif // FOC_CURRENT_H

