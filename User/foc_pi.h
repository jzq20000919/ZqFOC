#ifndef FOC_PI_H
#define FOC_PI_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float integral;
    float output_min;
    float output_max;
} FOC_PI_HandleTypeDef;

void FOC_PI_Init(FOC_PI_HandleTypeDef *pi, float kp, float ki, float output_min, float output_max);
float FOC_PI_Update(FOC_PI_HandleTypeDef *pi, float reference, float feedback, float dt);
void FOC_PI_Reset(FOC_PI_HandleTypeDef *pi);
#endif // FOC_PI_H