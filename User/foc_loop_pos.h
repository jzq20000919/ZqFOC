#ifndef LOOP_POS_H
#define LOOP_POS_H

#include "foc_pi.h"
#include "foc_encoder.h"
#include "foc_loop_spd.h"
#include "motor_param.h"

typedef struct
{
    FOC_PI_HandleTypeDef pi_pos;
    FOC_ENCODER_HandleTypeDef *encoder;
    FOC_LOOP_SPD_HandleTypeDef *loop_spd;
    float position_ref;
    float position_fbk;
    float speed_ref;
} FOC_LOOP_POS_HandleTypeDef;

void FOC_LOOP_POS_Init(FOC_LOOP_POS_HandleTypeDef *loop_pos,  FOC_ENCODER_HandleTypeDef *encoder, FOC_LOOP_SPD_HandleTypeDef *loop_spd,float kp, float ki);
void FOC_LOOP_POS_SetPositionRef(FOC_LOOP_POS_HandleTypeDef *loop_pos, float position_ref);
void FOC_LOOP_POS_Update(FOC_LOOP_POS_HandleTypeDef *loop_pos, float dt);

#endif // LOOP_POS_H