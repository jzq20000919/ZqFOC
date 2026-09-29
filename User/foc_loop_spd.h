/**
 * @file    foc_loop_spd.h
 * @brief   FOC速度环接口
 */
#if !defined(FOC_LOOP_SPD_H)
#define FOC_LOOP_SPD_H
#include "foc_pi.h"
#include "foc_encoder.h"
#include "foc_loop_cur.h"
#include "motor_param.h"

typedef struct {
    FOC_PI_HandleTypeDef pi_spd;          // 速度PI控制器
    FOC_ENCODER_HandleTypeDef *encoder;   // 转速反馈来源
    FOC_LOOP_CUR_HandleTypeDef *loop_cur; // 电流环句柄
    float speed_ref; // 目标机械转速，rpm
    float speed_fbk; // 反馈机械转速，rpm
    float iq_ref;    // q轴目标电流，A
} FOC_LOOP_SPD_HandleTypeDef;

/**
 * @brief  初始化速度环及其PI控制器
 * @param  loop_spd 速度环句柄
 * @param  encoder 编码器句柄
 * @param  loop_cur 电流环句柄
 * @param  kp 比例增益
 * @param  ki 积分增益
 */
void FOC_LOOP_SPD_Init(FOC_LOOP_SPD_HandleTypeDef *loop_spd,  FOC_ENCODER_HandleTypeDef *encoder, FOC_LOOP_CUR_HandleTypeDef *loop_cur,float kp, float ki);

/**
 * @brief  设置目标机械转速并按最大转速限幅
 * @param  loop_spd 速度环句柄
 * @param  speed_ref 目标机械转速，rpm
 */
void FOC_LOOP_SPD_SetSpeedRef(FOC_LOOP_SPD_HandleTypeDef *loop_spd, float speed_ref);

/**
 * @brief  更新速度PI并向电流环设置q轴目标电流
 * @param  loop_spd 速度环句柄
 * @param  dt 速度环周期，s
 */
void FOC_LOOP_SPD_Update(FOC_LOOP_SPD_HandleTypeDef *loop_spd, float dt);

#endif // FOC_LOOP_SPD_H
