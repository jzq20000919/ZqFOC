/**
 * @file    foc_math.h
 * @brief   FOC坐标变换接口
 */
#ifndef FOC_MATH_H
#define FOC_MATH_H
#include "motor_param.h"
typedef struct
{
    float i_alpha; // α轴电流，A
    float i_beta;  // β轴电流，A
} FOC_MATH_CLARKE_HandleTypeDef;
typedef struct
{
    float i_d; // d轴电流，A
    float i_q; // q轴电流，A
} FOC_MATH_PARK_HandleTypeDef;

typedef struct
{
    float v_alpha; // α轴电压，V
    float v_beta;  // β轴电压，V
} FOC_MATH_INV_PARK_HandleTypeDef;

/**
 * @brief  将三相电流转换为静止坐标系电流
 * @param  i_a A相电流，A
 * @param  i_b B相电流，A
 * @param  i_c C相电流，A
 * @return 静止坐标系的α、β轴电流，A
 * @note   当前实现使用A、B相计算，C相参数保留在接口中。
 */
FOC_MATH_CLARKE_HandleTypeDef FOC_MATH_Clarke(float i_a, float i_b, float i_c);
/**
 * @brief  将静止坐标系电流转换为旋转坐标系电流
 * @param  i_alpha α轴电流，A
 * @param  i_beta β轴电流，A
 * @param  theta_e 电角度，rad
 * @return 旋转坐标系的d、q轴电流，A
 */
FOC_MATH_PARK_HandleTypeDef FOC_MATH_Park(float i_alpha, float i_beta, float theta_e);
/**
 * @brief  将旋转坐标系电压转换为静止坐标系电压
 * @param  v_d d轴电压，V
 * @param  v_q q轴电压，V
 * @param  theta_e 电角度，rad
 * @return 静止坐标系的α、β轴电压，V
 */
FOC_MATH_INV_PARK_HandleTypeDef FOC_MATH_InvPark(float v_d, float v_q, float theta_e);

/**
 * @brief  将角度归一化到[0, 2π)范围
 * @param  angle 输入角度，rad
 * @return 归一化后的角度，rad
 */
float FOC_MATH_Normalize(float angle);


#endif // FOC_MATH_H
