#include "foc_math.h"
#include <math.h>
#include "motor_param.h"

FOC_MATH_CLARKE_HandleTypeDef FOC_MATH_Clarke(float i_a, float i_b, float i_c)
{
    (void)i_c; // Unused parameter
    FOC_MATH_CLARKE_HandleTypeDef clarke_output;
    clarke_output.i_alpha = i_a;
    clarke_output.i_beta = (i_a + 2.0f * i_b) * FOC_INV_SQRT3;
    return clarke_output;
}

FOC_MATH_PARK_HandleTypeDef FOC_MATH_Park(float i_alpha, float i_beta, float theta_e)
{
    FOC_MATH_PARK_HandleTypeDef park_output;
    float sin_theta = sinf(theta_e);
    float cos_theta = cosf(theta_e);
    park_output.i_d = i_alpha * cos_theta + i_beta * sin_theta;
    park_output.i_q = -i_alpha * sin_theta + i_beta * cos_theta;
    return park_output;
}

FOC_MATH_INV_PARK_HandleTypeDef FOC_MATH_InvPark(float v_d, float v_q, float theta_e)
{
    FOC_MATH_INV_PARK_HandleTypeDef inv_park_output;
    float sin_theta = sinf(theta_e);
    float cos_theta = cosf(theta_e);
    inv_park_output.v_alpha = v_d * cos_theta - v_q * sin_theta;
    inv_park_output.v_beta = v_d * sin_theta + v_q * cos_theta;
    return inv_park_output;
}

float FOC_MATH_Normalize(float angle)
{
    while (angle >= FOC_TWO_PI)
    {
        angle -= FOC_TWO_PI;
    }

    while (angle < 0.0f)
    {
        angle += FOC_TWO_PI;
    }

    return angle;
}