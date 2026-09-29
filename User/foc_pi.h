/**
 * @file    foc_pi.h
 * @brief   通用PI控制器接口
 */
#ifndef FOC_PI_H
#define FOC_PI_H

#include <stdint.h>

typedef struct {
    float kp;         // 比例增益，单位取决于所接控制环
    float ki;         // 积分增益，单位取决于所接控制环
    float integral;   // 积分项，单位与输出相同
    float output_min; // 输出下限，单位与输出相同
    float output_max; // 输出上限，单位与输出相同
} FOC_PI_HandleTypeDef;

/**
 * @brief  初始化PI控制器并清零积分项
 * @param  pi PI控制器句柄
 * @param  kp 比例增益
 * @param  ki 积分增益
 * @param  output_min 输出下限，单位由控制环决定
 * @param  output_max 输出上限，单位由控制环决定
 */
void FOC_PI_Init(FOC_PI_HandleTypeDef *pi, float kp, float ki, float output_min, float output_max);
/**
 * @brief  根据目标值与反馈值计算限幅后的PI输出
 * @param  pi PI控制器句柄
 * @param  reference 目标值，单位由控制环决定
 * @param  feedback 反馈值，单位与目标值相同
 * @param  dt 控制周期，s
 * @return 限幅后的PI输出，单位由控制环决定
 */
float FOC_PI_Update(FOC_PI_HandleTypeDef *pi, float reference, float feedback, float dt);
/**
 * @brief  清零PI积分项
 * @param  pi PI控制器句柄
 */
void FOC_PI_Reset(FOC_PI_HandleTypeDef *pi);
#endif // FOC_PI_H
