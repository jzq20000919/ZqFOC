#ifndef FOC_LOOP_CUR_H
#define FOC_LOOP_CUR_H

#include "foc_pi.h"
#include "foc_encoder.h"
#include "foc_current.h"
#include "foc_svpwm.h"
#include "foc_math.h"
typedef struct {
    FOC_ENCODER_HandleTypeDef *encoder;
    FOC_CURRENT_HandleTypeDef *current;
    FOC_SVPWM_HandleTypeDef *svpwm;
    FOC_PI_HandleTypeDef pi_id;
    FOC_PI_HandleTypeDef pi_iq;
    float id_ref;
    float iq_ref;
    float i_alpha;
    float i_beta;
    float i_d;
    float i_q;
    float v_d;
    float v_q;
    float v_alpha;
    float v_beta;
} FOC_LOOP_CUR_HandleTypeDef;

void FOC_LOOP_CUR_Init(FOC_LOOP_CUR_HandleTypeDef *loop_cur, FOC_ENCODER_HandleTypeDef *encoder, FOC_CURRENT_HandleTypeDef *current, FOC_SVPWM_HandleTypeDef *svpwm,float kp_id,float ki_id,float kp_iq,float ki_iq,float voltage_limit);

void FOC_LOOP_CUR_Update(FOC_LOOP_CUR_HandleTypeDef *loop_cur, float v_bus);

void FOC_LOOP_CUR_SetReference(FOC_LOOP_CUR_HandleTypeDef *loop_cur, float id_ref, float iq_ref);

#endif // FOC_LOOP_CUR_H