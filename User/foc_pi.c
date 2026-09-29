/**
 * @file    foc_pi.c
 * @brief   通用PI控制器实现
 */
#include "foc_pi.h"

void FOC_PI_Init(FOC_PI_HandleTypeDef *pi, float kp, float ki, float output_min, float output_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
    pi->output_min = output_min;
    pi->output_max = output_max;
}

float FOC_PI_Update(FOC_PI_HandleTypeDef *pi, float reference, float feedback, float dt)
{
    float error = reference - feedback;
    pi->integral += pi->ki * error * dt;
    // 分别限制积分项与最终输出，避免积分项持续累积超出输出范围。
    if (pi->integral >pi->output_max)
     pi->integral = pi->output_max;
    if (pi->integral < pi->output_min) {
        pi->integral = pi->output_min;
    }
    float output = pi->kp * error + pi->integral;
    if (output < pi->output_min) {
        output = pi->output_min;
    } else if (output > pi->output_max) {
        output = pi->output_max;
    }
    return output;
}

void FOC_PI_Reset(FOC_PI_HandleTypeDef *pi)
{
    pi->integral = 0.0f;
}
