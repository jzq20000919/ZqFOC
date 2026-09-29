/**
 * @file    foc_encoder.h
 * @brief   编码器角度与转速接口
 */
#ifndef FOC_ENCODER_H
#define FOC_ENCODER_H
#include "stm32g4xx_hal.h"
#include "motor_param.h"

typedef struct
{
    TIM_HandleTypeDef *htim; // 编码器定时器句柄
    int32_t spd_last_count;   // 上次测速时的编码器计数，计数
    int32_t count;            // 当前编码器计数，计数
    float angle_m;            // 机械角度，rad
    float angle_e;            // 电角度，rad
    float angle_e_offset;     // 电角度零点偏置，rad
    float speed;              // 机械转速，rpm
} FOC_ENCODER_HandleTypeDef;

/**
 * @brief  初始化编码器并启动定时器编码器模式
 * @param  encoder 编码器句柄
 * @param  htim 编码器定时器句柄
 */
void FOC_ENCODER_Init(FOC_ENCODER_HandleTypeDef *encoder, TIM_HandleTypeDef *htim);

/**
 * @brief  根据当前计数更新机械角度和电角度
 * @param  encoder 编码器句柄
 */
void FOC_ENCODER_UpdateAngle(FOC_ENCODER_HandleTypeDef *encoder);
/**
 * @brief  根据计数差更新机械转速
 * @param  encoder 编码器句柄
 * @param  dt 两次测速间隔，s
 */
void FOC_ENCODER_UpdateSpeed(FOC_ENCODER_HandleTypeDef *encoder, float dt);

/**
 * @brief  设置电角度零点偏置
 * @param  encoder 编码器句柄
 * @param  offset 电角度零点偏置，rad
 */
void FOC_ENCODER_SetElectricalOffset(FOC_ENCODER_HandleTypeDef *encoder, float offset);

/**
 * @brief  根据对齐角度校准电角度零点偏置
 * @param  encoder 编码器句柄
 * @param  align_angle 对齐磁场的电角度，rad
 */
void FOC_ENCODER_CalibrateElectricalOffset(
    FOC_ENCODER_HandleTypeDef *encoder,
    float align_angle);

/**
 * @brief  对多次机械角采样做圆周平均，并设置电角度零点偏置
 * @param  encoder 编码器句柄
 * @param  align_angle 对齐磁场的电角度，rad
 * @param  angle_samples 机械角采样数组，单位rad
 * @param  sample_count 采样个数
 * @return HAL_OK 校准成功，HAL_ERROR 采样无效
 */
HAL_StatusTypeDef FOC_ENCODER_CalibrateElectricalOffsetSamples(
    FOC_ENCODER_HandleTypeDef *encoder,
    float align_angle,
    const float *angle_samples,
    uint8_t sample_count);
#endif
