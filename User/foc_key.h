/**
 * @file    foc_key.h
 * @brief   FOC按键模块接口
 */
#ifndef FOC_KEY_H
#define FOC_KEY_H

/** @brief GPIO初始化后记录按键初始状态，由FOC_Motor_Init调用。 */
void FOC_KEY_Init(void);
/**
 * @brief FOC初始化完成后在主循环调用，处理启停、调速及反转按键。
 * @note 运行的速度模式下，SW2增加、SW3减少带符号的目标转速；
 *       可跨越零速，正目标正转、负目标反转，同时按下不调整目标。
 */
void FOC_KEY_Task(void);

#endif // FOC_KEY_H
