/**
 * @file    foc_encoder.h
 * @brief   编码器角度与转速接口
 */
#ifndef FOC_ENCODER_H
#define FOC_ENCODER_H
#include "stm32g4xx_hal.h"
#include "motor_param.h"
#include "foc_svpwm.h"

typedef struct
{
    TIM_HandleTypeDef *htim; // 编码器定时器句柄
    int32_t spd_last_count;   // 上次测速时的编码器计数，计数
    int32_t count;            // 当前编码器计数，计数
    float angle_m;            // 机械角度，rad
    float angle_e;            // 电角度，rad
    float angle_e_offset;     // 电角度零点偏置，rad
    float speed;              // 机械转速，rpm
    float speed_raw;          // 未滤波的差分转速，rpm；保留它便于区分测速量化与真实低频振荡
    float aligned_raw_angle;  // 上电对齐位置x，只供首次机械零点标定使用
    float home_from_z;        // 固定偏移x-y，rad，保存在Flash
    float zero_raw_angle;     // 本次上电机械0点对应的原始角度
    uint8_t zero_saved;        // Flash有有效机械零点参数
    uint8_t zero_known;        // 本次上电已经恢复机械零点
    volatile uint8_t index_armed; // 寻Z时开启，只锁存一个上升沿
    volatile uint8_t index_found;
    volatile uint32_t index_count; // Z上升沿计数y
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
 * @brief  根据计数差计算speed_raw，再经一阶低通更新speed；不改变调用周期
 * @param  encoder 编码器句柄
 * @param  dt 两次测速间隔，s
 */
void FOC_ENCODER_UpdateSpeed(FOC_ENCODER_HandleTypeDef *encoder, float dt);
/** @brief 上电固定磁场对齐；0表示等待完成，1表示对齐结束。 */
uint8_t FOC_ENCODER_Align(FOC_ENCODER_HandleTypeDef *encoder,
                         FOC_SVPWM_HandleTypeDef *svpwm, float v_bus);

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
 */
void FOC_ENCODER_CalibrateElectricalOffsetSamples(
    FOC_ENCODER_HandleTypeDef *encoder,
    float align_angle,
    const float *angle_samples,
    uint8_t sample_count);

/** @brief 输出关闭后调用：根据x、y计算固定偏移，恢复坐标并保存Flash。 */
void FOC_ENCODER_CalibrateZero(FOC_ENCODER_HandleTypeDef *encoder);
/** @brief 根据本次Z计数恢复机械零点和电角度补偿；调用时与控制中断互斥。 */
void FOC_ENCODER_RestoreZero(FOC_ENCODER_HandleTypeDef *encoder);
/** @brief 判断当前是否已回到机械零点，供状态跳转使用。 */
uint8_t FOC_ENCODER_AtZero(FOC_ENCODER_HandleTypeDef *encoder);

#endif
