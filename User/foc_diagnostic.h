/**
 * @file    foc_diagnostic.h
 * @brief   临时电流采样与电流环实验模式
 */
#ifndef FOC_DIAGNOSTIC_H
#define FOC_DIAGNOSTIC_H

#define FOC_DIAGNOSTIC_NORMAL       0
#define FOC_DIAGNOSTIC_ZERO_CURRENT 1
#define FOC_DIAGNOSTIC_CURRENT_LOOP 2
#define FOC_DIAGNOSTIC_PWM_50       3

// 0：正常按键控制；1：无PWM零点采样；2：固定q轴电流环；3：三相固定50% PWM采样。
#define FOC_DIAGNOSTIC_MODE FOC_DIAGNOSTIC_NORMAL

#endif // FOC_DIAGNOSTIC_H
