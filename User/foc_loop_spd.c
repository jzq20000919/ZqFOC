/**
 * @file    foc_loop_spd.c
 * @brief   FOC速度环实现
 */
#include "foc_loop_spd.h"

void FOC_LOOP_SPD_Init(FOC_LOOP_SPD_HandleTypeDef *loop_spd,  FOC_ENCODER_HandleTypeDef *encoder, FOC_LOOP_CUR_HandleTypeDef *loop_cur,float kp, float ki)
{
    loop_spd->encoder = encoder;
    loop_spd->loop_cur = loop_cur;
    loop_spd->speed_ref = 0.0f;
    loop_spd->speed_fbk = 0.0f;
    loop_spd->iq_ref = 0.0f;
    FOC_PI_Init(&loop_spd->pi_spd, kp, ki,-MOTOR_MAX_CURRENT,MOTOR_MAX_CURRENT);
}

void FOC_LOOP_SPD_SetSpeedRef(FOC_LOOP_SPD_HandleTypeDef *loop_spd, float speed_ref)
{
   if(speed_ref > MOTOR_MAX_SPEED_RPM)
   speed_ref = MOTOR_MAX_SPEED_RPM;
   if(speed_ref < -MOTOR_MAX_SPEED_RPM)
   speed_ref = -MOTOR_MAX_SPEED_RPM;
   loop_spd->speed_ref = speed_ref;
}

void FOC_LOOP_SPD_Update(FOC_LOOP_SPD_HandleTypeDef *loop_spd, float dt)
{
    FOC_ENCODER_UpdateSpeed(loop_spd->encoder, dt);
    loop_spd->speed_fbk = loop_spd->encoder->speed;
    // 速度PI输出q轴电流目标，d轴目标保持为零。
    loop_spd->iq_ref = FOC_PI_Update(&loop_spd->pi_spd, loop_spd->speed_ref, loop_spd->speed_fbk, dt);
    FOC_LOOP_CUR_SetReference(loop_spd->loop_cur, 0.0f, loop_spd->iq_ref);
}
