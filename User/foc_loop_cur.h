/**
 * @file    foc_loop_cur.h
 * @brief   FOC电流环接口
 */
#ifndef FOC_LOOP_CUR_H
#define FOC_LOOP_CUR_H

#include "foc_pi.h"
#include "foc_encoder.h"
#include "foc_current.h"
#include "foc_svpwm.h"
#include "foc_math.h"
typedef struct {
    FOC_ENCODER_HandleTypeDef *encoder; // 电角度反馈来源
    FOC_CURRENT_HandleTypeDef *current; // 三相电流采样来源
    FOC_SVPWM_HandleTypeDef *svpwm;     // 三相PWM输出句柄
    FOC_PI_HandleTypeDef pi_id;         // d轴电流PI控制器
    FOC_PI_HandleTypeDef pi_iq;         // q轴电流PI控制器
    float id_ref;   // d轴目标电流，A
    float iq_ref;   // q轴目标电流，A
    float i_alpha;  // α轴反馈电流，A
    float i_beta;   // β轴反馈电流，A
    float i_d;      // d轴反馈电流，A
    float i_q;      // q轴反馈电流，A
    float v_d;      // d轴输出电压，V
    float v_q;      // q轴输出电压，V
    float v_alpha;  // α轴输出电压，V
    float v_beta;   // β轴输出电压，V
} FOC_LOOP_CUR_HandleTypeDef;

/**
 * @brief  初始化电流环及d、q轴PI控制器
 * @param  loop_cur 电流环句柄
 * @param  encoder 编码器句柄
 * @param  current 三相电流句柄
 * @param  svpwm SVPWM句柄
 * @param  kp_id d轴比例增益
 * @param  ki_id d轴积分增益
 * @param  kp_iq q轴比例增益
 * @param  ki_iq q轴积分增益
 * @param  voltage_limit PI输出电压限幅，V
 */
void FOC_LOOP_CUR_Init(FOC_LOOP_CUR_HandleTypeDef *loop_cur, FOC_ENCODER_HandleTypeDef *encoder, FOC_CURRENT_HandleTypeDef *current, FOC_SVPWM_HandleTypeDef *svpwm,float kp_id,float ki_id,float kp_iq,float ki_iq,float voltage_limit);

/**
 * @brief  完成一次电流采样、坐标变换、PI计算和SVPWM更新
 * @param  loop_cur 电流环句柄
 * @param  v_bus 当前母线电压，V
 */
void FOC_LOOP_CUR_Update(FOC_LOOP_CUR_HandleTypeDef *loop_cur, float v_bus);

/**
 * @brief  设置d、q轴目标电流
 * @param  loop_cur 电流环句柄
 * @param  id_ref d轴目标电流，A
 * @param  iq_ref q轴目标电流，A
 */
void FOC_LOOP_CUR_SetReference(FOC_LOOP_CUR_HandleTypeDef *loop_cur, float id_ref, float iq_ref);

#endif // FOC_LOOP_CUR_H
