/**
 * @file    foc_diagnostic.h
 * @brief   临时电流采样零点实验开关
 */
#ifndef FOC_DIAGNOSTIC_H
#define FOC_DIAGNOSTIC_H

// 1：仅采样三相ADC原始值，三相PWM和FOC闭环均不启动；0：恢复正常电机控制。
#define FOC_CURRENT_ZERO_TEST 1

#endif // FOC_DIAGNOSTIC_H
