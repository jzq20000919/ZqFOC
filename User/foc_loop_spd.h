#if !defined(FOC_LOOP_SPD_H)
#define FOC_LOOP_SPD_H
#include "foc_pi.h"
#include "foc_encoder.h"
#include "foc_loop_cur.h"
#include "motor_param.h"

typedef struct {
    FOC_PI_HandleTypeDef pi_spd;//属于loop_spd自己，所以不需要指针
    FOC_ENCODER_HandleTypeDef *encoder;
    FOC_LOOP_CUR_HandleTypeDef *loop_cur;
    float speed_ref;
    float speed_fbk;
    float iq_ref;
} FOC_LOOP_SPD_HandleTypeDef;

void FOC_LOOP_SPD_Init(FOC_LOOP_SPD_HandleTypeDef *loop_spd,  FOC_ENCODER_HandleTypeDef *encoder, FOC_LOOP_CUR_HandleTypeDef *loop_cur,float kp, float ki);

void FOC_LOOP_SPD_SetSpeedRef(FOC_LOOP_SPD_HandleTypeDef *loop_spd, float speed_ref);

void FOC_LOOP_SPD_Update(FOC_LOOP_SPD_HandleTypeDef *loop_spd, float dt);

#endif // FOC_LOOP_SPD_H