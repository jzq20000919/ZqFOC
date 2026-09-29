#ifndef FOC_MATH_H
#define FOC_MATH_H
#include "motor_param.h"
typedef struct
{
    float i_alpha;
    float i_beta;
} FOC_MATH_CLARKE_HandleTypeDef;
typedef struct
{
    float i_d;
    float i_q;
} FOC_MATH_PARK_HandleTypeDef;

typedef struct
{
    float v_alpha;
    float v_beta;
} FOC_MATH_INV_PARK_HandleTypeDef;

FOC_MATH_CLARKE_HandleTypeDef FOC_MATH_Clarke(float i_a, float i_b, float i_c);
FOC_MATH_PARK_HandleTypeDef FOC_MATH_Park(float i_alpha, float i_beta, float theta_e);
FOC_MATH_INV_PARK_HandleTypeDef FOC_MATH_InvPark(float v_d, float v_q, float theta_e);

float FOC_MATH_Normalize(float angle);


#endif // FOC_MATH_H