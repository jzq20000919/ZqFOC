/**
 * @file    foc_loop_pos.h
 * @brief   FOC位置环接口
 */
#ifndef FOC_LOOP_POS_H
#define FOC_LOOP_POS_H

#include "foc_pi.h"
#include "foc_encoder.h"
#include "foc_loop_spd.h"
#include "motor_param.h"

typedef struct
{
    FOC_PI_HandleTypeDef pi_pos;          // 位置PI控制器
    FOC_ENCODER_HandleTypeDef *encoder;   // 机械角度反馈来源
    FOC_LOOP_SPD_HandleTypeDef *loop_spd; // 速度环句柄
    float position_ref; // 目标机械角度，rad
    float position_fbk; // 反馈机械角度，rad
    float speed_ref;    // 位置环输出的目标机械转速，rpm
} FOC_LOOP_POS_HandleTypeDef;

/**
 * @brief  初始化位置环及其PI控制器
 * @param  loop_pos 位置环句柄
 * @param  encoder 编码器句柄
 * @param  loop_spd 速度环句柄
 * @param  kp 比例增益
 * @param  ki 积分增益
 */
void FOC_LOOP_POS_Init(FOC_LOOP_POS_HandleTypeDef *loop_pos,  FOC_ENCODER_HandleTypeDef *encoder, FOC_LOOP_SPD_HandleTypeDef *loop_spd,float kp, float ki);
/**
 * @brief  设置目标机械角度并归一化到[0, 2π)
 * @param  loop_pos 位置环句柄
 * @param  position_ref 目标机械角度，rad
 */
void FOC_LOOP_POS_SetPositionRef(FOC_LOOP_POS_HandleTypeDef *loop_pos, float position_ref);
/**
 * @brief  根据机械角度误差更新速度环目标
 * @param  loop_pos 位置环句柄
 * @param  dt 位置环周期，s
 */
void FOC_LOOP_POS_Update(FOC_LOOP_POS_HandleTypeDef *loop_pos, float dt);

#endif // FOC_LOOP_POS_H
