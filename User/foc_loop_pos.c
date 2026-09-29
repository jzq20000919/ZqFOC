#include "foc_loop_pos.h"
#include "foc_math.h"
#include "motor_param.h"
void LOOP_POS_Init(LOOP_POS_HandleTypeDef *loop_pos,  FOC_ENCODER_HandleTypeDef *encoder, FOC_LOOP_SPD_HandleTypeDef *loop_spd,float kp, float ki)
{
    FOC_PI_Init(&loop_pos->pi_pos, kp, ki,-MOTOR_MAX_SPEED_RPM, MOTOR_MAX_SPEED_RPM);
    loop_pos->encoder = encoder;
    loop_pos->loop_spd = loop_spd;
    loop_pos->position_ref = 0.0f;
    loop_pos->position_fbk = 0.0f;
    loop_pos->speed_ref = 0.0f;
}
void LOOP_POS_SetPositionRef(LOOP_POS_HandleTypeDef *loop_pos, float position_ref)
{
    loop_pos->position_ref = FOC_Normalize(position_ref);
}
void LOOP_POS_Update(LOOP_POS_HandleTypeDef *loop_pos, float dt)
{
    float error;
    loop_pos ->position_fbk = loop_pos->encoder->angle_M;
    //位置误差
    error = loop_pos->position_ref - loop_pos->position_fbk;
    //将误差限制在 [-π, π] 范围内
    if (error > FOC_PI) 
    error -= FOC_TWO_PI;
    if (error < -FOC_PI) 
    error += FOC_TWO_PI;
    //位置PI输出转速目标
    loop_pos->speed_ref = FOC_PI_Update(&loop_pos->pi_pos, error, 0.0f, dt);
    LOOP_Spd_SetSpeedRef(loop_pos->loop_spd, loop_pos->speed_ref);
}