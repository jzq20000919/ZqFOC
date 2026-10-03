/**
 * @file    foc_key.h
 * @brief   FOC按键模块接口
 */
#ifndef FOC_KEY_H
#define FOC_KEY_H

/** @brief GPIO初始化后记录按键初始状态，由FOC_Motor_Init调用。 */
void FOC_KEY_Init(void);
/** @brief FOC初始化完成后在主循环调用，处理启停及调速按键。 */
void FOC_KEY_Task(void);

#endif // FOC_KEY_H
