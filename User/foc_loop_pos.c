/**
 * @file    foc_loop_pos.c
 * @brief   FOC位置环实现
 */
#include "foc_loop_pos.h"
#include "foc_math.h"
#include "motor_param.h"
void FOC_LOOP_POS_Init(FOC_LOOP_POS_HandleTypeDef *loop_pos,  FOC_ENCODER_HandleTypeDef *encoder, FOC_LOOP_SPD_HandleTypeDef *loop_spd,float kp, float ki)
{
    FOC_PI_Init(&loop_pos->pi_pos, kp, ki,-MOTOR_MAX_SPEED_RPM, MOTOR_MAX_SPEED_RPM);
    loop_pos->encoder = encoder;
    loop_pos->loop_spd = loop_spd;
    loop_pos->position_ref = 0.0f;
    loop_pos->position_fbk = 0.0f;
    loop_pos->speed_ref = 0.0f;
}
void FOC_LOOP_POS_SetPositionRef(FOC_LOOP_POS_HandleTypeDef *loop_pos, float position_ref)
{
    loop_pos->position_ref = FOC_MATH_Normalize(position_ref);
}
void FOC_LOOP_POS_Update(FOC_LOOP_POS_HandleTypeDef *loop_pos, float dt)
{
    float error;
    loop_pos ->position_fbk = loop_pos->encoder->angle_m;
    error = loop_pos->position_ref - loop_pos->position_fbk;
    // 取单圈内的最短角度误差，避免跨越零点时反向绕行。
    if (error > FOC_PI) 
    error -= FOC_TWO_PI;
    if (error < -FOC_PI) 
    error += FOC_TWO_PI;
    // 位置PI给出转速目标，再由速度环跟踪。
    loop_pos->speed_ref = FOC_PI_Update(&loop_pos->pi_pos, error, 0.0f, dt);
    FOC_LOOP_SPD_SetSpeedRef(loop_pos->loop_spd, loop_pos->speed_ref);
}
