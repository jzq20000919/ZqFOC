/**
 * @file    foc_current.h
 * @brief   A、B相电流采样与C相重构接口
 */
#if !defined(FOC_CURRENT_H)
#define FOC_CURRENT_H
#include "stm32g4xx_hal.h"
#include "motor_param.h"
typedef struct
{
    ADC_HandleTypeDef *hadc1; // A相电流采样ADC句柄
    ADC_HandleTypeDef *hadc2; // B相电流采样ADC句柄

    uint16_t raw_a; // A相ADC原始值，计数
    uint16_t raw_b; // B相ADC原始值，计数

    float offset_a; // A相运行时零电流ADC计数
    float offset_b; // B相运行时零电流ADC计数

    float i_a; // A相电流，A
    float i_b; // B相电流，A
    float i_c; // 由A、B相重构的C相电流，A
    uint32_t offset_sum_a;
    uint32_t offset_sum_b;
    uint32_t offset_count;
    uint32_t offset_tick; // 分步校准进度，Init时清零

} FOC_CURRENT_HandleTypeDef;

/**
 * @brief  初始化电流采样句柄
 * @param  current 电流采样句柄
 * @param  hadc1 A相电流采样ADC句柄
 * @param  hadc2 B相电流采样ADC句柄
 */
void FOC_CURRENT_Init(FOC_CURRENT_HandleTypeDef *current,ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);
/**
 * @brief  读取A、B相注入转换结果并重构C相电流
 * @param  current 电流采样句柄
 */
void FOC_CURRENT_Update(FOC_CURRENT_HandleTypeDef *current);

/** @brief 开启注入采样与CH4触发；interrupt为0用于零点校准，为1用于电流环。 */
void FOC_CURRENT_StartSampling(FOC_CURRENT_HandleTypeDef *current,
                               TIM_HandleTypeDef *timer, uint8_t interrupt);
/** @brief 关闭CH4触发和两路注入采样。 */
void FOC_CURRENT_StopSampling(FOC_CURRENT_HandleTypeDef *current, TIM_HandleTypeDef *timer);

/**
 * @brief  三相功率输出关闭时，按固定间隔采样并校准A、B相电流零点
 * @param  current 电流采样句柄，注入ADC和TIM1 CH4须已启动
 * @param  pwm_timer 三相PWM定时器句柄
 * @return 重复调用：0表示尚未完成，1表示已经完成。
 * @note 每次只读取一个新样本，不等待、不延时。
 *       完成后自动关闭本次校准采样，后续调用直接返回1。
 */
uint8_t FOC_CURRENT_CalibrateOffset(
    FOC_CURRENT_HandleTypeDef *current,
    TIM_HandleTypeDef *pwm_timer);

#endif // FOC_CURRENT_H

