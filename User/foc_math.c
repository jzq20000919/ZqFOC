#include "foc_math.h"
#include <math.h>
#include "motor_param.h"

FOC_Clarke_t FOC_Clarke(float I_a, float I_b, float I_c)
{
    (void)I_c; // Unused parameter
    FOC_Clarke_t clarke_output;
    clarke_output.I_alpha = I_a;
    clarke_output.I_beta = (I_a + 2.0f * I_b) * FOC_INV_SQRT3;
    return clarke_output;
}

FOC_Park_t FOC_Park(float I_alpha, float I_beta, float theta_e)
{
    FOC_Park_t park_output;
    float sin_theta = sinf(theta_e);
    float cos_theta = cosf(theta_e);
    park_output.I_d = I_alpha * cos_theta + I_beta * sin_theta;
    park_output.I_q = -I_alpha * sin_theta + I_beta * cos_theta;
    return park_output;
}

FOC_inv_Park_t FOC_inv_Park(float V_d, float V_q, float theta_e)
{
    FOC_inv_Park_t inv_park_output;
    float sin_theta = sinf(theta_e);
    float cos_theta = cosf(theta_e);
    inv_park_output.V_alpha = V_d * cos_theta - V_q * sin_theta;
    inv_park_output.V_beta = V_d * sin_theta + V_q * cos_theta;
    return inv_park_output;
}

float FOC_Normalize(float angle)
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