#ifndef FOC_MATH_H
#define FOC_MATH_H
#include "motor_param.h"
typedef struct
{
    float I_alpha;
    float I_beta;
} FOC_Clarke_t;
typedef struct
{
    float I_d;
    float I_q;
} FOC_Park_t;

typedef struct
{
    float V_alpha;
    float V_beta;
} FOC_inv_Park_t;

FOC_Clarke_t FOC_Clarke(float I_a, float I_b, float I_c);
FOC_Park_t FOC_Park(float I_alpha, float I_beta, float theta_e);
FOC_inv_Park_t FOC_inv_Park(float V_d, float V_q, float theta_e);

float FOC_Normalize(float angle);


#endif // FOC_MATH_H