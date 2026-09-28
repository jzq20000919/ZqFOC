#include "foc_loop_cur.h"
#include "foc_pi.h"
#include "math.h"
void FOC_Loop_Cur_Init(FOC_Loop_Cur_HandleTypeDef *loop_cur, FOC_ENCODER_HandleTypeDef *encoder, FOC_Current_HandleTypeDef *current, FOC_SVPWM_HandleTypeDef *svpwm,float kp_id,float kp_iq,float ki_id,float ki_iq,float voltage_limit)
{
    loop_cur->encoder = encoder;
    loop_cur->current = current;
    loop_cur->svpwm = svpwm;
    loop_cur->id_ref = 0.0f;
    loop_cur->iq_ref = 0.0f;
    loop_cur->i_alpha = 0.0f;
    loop_cur->i_beta = 0.0f;
    loop_cur->i_d = 0.0f;
    loop_cur->i_q = 0.0f;
    loop_cur->v_d = 0.0f;
    loop_cur->v_q = 0.0f;
    loop_cur->v_alpha = 0.0f;
    loop_cur->v_beta = 0.0f;

    FOC_PI_Init(&loop_cur->pi_id, kp_id, ki_id,-voltage_limit,voltage_limit);
    FOC_PI_Init(&loop_cur->pi_iq, kp_iq, ki_iq,-voltage_limit,voltage_limit);

}
//设定参考值函数
void FOC_Loop_Cur_SetReference(FOC_Loop_Cur_HandleTypeDef *loop_cur, float id_ref, float iq_ref)
{
    loop_cur->id_ref = id_ref;
    loop_cur->iq_ref = iq_ref;
}

// 更新电流环
void FOC_Loop_Cur_Update(FOC_Loop_Cur_HandleTypeDef *loop_cur,float V_bus)
{
    // Update the current loop
    FOC_Clarke_HandleTypeDef clarke_out;
    FOC_Park_HandleTypeDef park_out;
    FOC_inv_Park_HandleTypeDef inv_park_out;
    //更新电流采样
    FOC_Current_Update(loop_cur->current);
    //更新编码器角度
    FOC_Encoder_Update(loop_cur->encoder);
    // Perform Clarke transformation
    clarke_out= FOC_Clarke(loop_cur->current->I_a, loop_cur->current->I_b, loop_cur->current->I_c);
    loop_cur->i_alpha = clarke_out.I_alpha;
    loop_cur->i_beta = clarke_out.I_beta;

    // Perform Park transformation
    park_out= FOC_Park(loop_cur->i_alpha, loop_cur->i_beta, loop_cur->encoder->angle_E);
    loop_cur->i_d = park_out.I_d;
    loop_cur->i_q = park_out.I_q;

    // 电流Id/Iq的PI控制器更新
    loop_cur->v_d = 
    FOC_PI_Update(&loop_cur->pi_id, loop_cur->id_ref,loop_cur->i_d,CURRENT_LOOP_TS);
    loop_cur->v_q = 
    FOC_PI_Update(&loop_cur->pi_iq, loop_cur->iq_ref,loop_cur->i_q,CURRENT_LOOP_TS);
    //电压矢量限幅
    float v_limit;
    float v_mag;
    v_limit = 0.9f * V_bus * FOC_INV_SQRT3;
    v_mag = sqrtf(loop_cur->v_d * loop_cur->v_d
            + loop_cur->v_q * loop_cur->v_q);
    if(v_mag > v_limit)
    {
        float scale = v_limit/v_mag;
        loop_cur->v_d *= scale;
        loop_cur->v_q *= scale;
    }
    // 逆Park变换
    inv_park_out= FOC_inv_Park(loop_cur->v_d, loop_cur->v_q, loop_cur->encoder->angle_E);
    loop_cur->v_alpha = inv_park_out.V_alpha;
    loop_cur->v_beta = inv_park_out.V_beta;

    //SVPWM更新
    FOC_SVPWM_Update(loop_cur->svpwm, loop_cur->v_alpha, loop_cur->v_beta, V_bus);

}