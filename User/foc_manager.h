/**
 * @file    foc_manager.h
 * @brief   单电机FOC对象与流程管理接口
 */
#ifndef FOC_MANAGER_H
#define FOC_MANAGER_H

#include "foc_encoder.h"
#include "foc_current.h"
#include "foc_bus_voltage.h"
#include "foc_svpwm.h"
#include "foc_loop_cur.h"
#include "foc_loop_spd.h"
#include "foc_loop_pos.h"
#include "foc_comm_usart.h"

typedef enum
{
  FOC_STATE_STOPPED = 0,
  FOC_STATE_ALIGNING,
  FOC_STATE_RUNNING_SPEED,
  FOC_STATE_RUNNING_POSITION
} FOC_Motor_State;

typedef struct
{
  volatile FOC_Motor_State motor_state; // 主循环写入，中断读取
  FOC_CONTROL_MODE control_mode;        // STOP中选择，运行时与速度/位置状态一致
  float speed_target;                   // 用户目标，与回零使用的临时目标分开
  float position_target;                // 用户单圈位置目标，rad
  volatile uint8_t control_active;      // 是否执行电流环和速度环
  volatile uint8_t position_active;     // 是否执行位置环

  FOC_ENCODER_HandleTypeDef encoder;
  FOC_CURRENT_HandleTypeDef current;
  FOC_BUS_VOLTAGE_HandleTypeDef bus_voltage;
  FOC_SVPWM_HandleTypeDef svpwm;

  FOC_LOOP_CUR_HandleTypeDef loop_cur;
  FOC_LOOP_SPD_HandleTypeDef loop_spd;
  FOC_LOOP_POS_HandleTypeDef loop_pos;
} FOC_Motor_HandleTypeDef;

extern FOC_Motor_HandleTypeDef foc_motor;

/** @brief 初始化后进入ALIGNING，Task完成上电对齐后进入STOPPED。 */
void FOC_Motor_Init(void);
/** @brief 主循环调用；只分发四个状态执行函数，不处理通信。 */
void FOC_Motor_Task(void);
/** @brief 仅在STOPPED接受；先回零，再进入选定的速度/位置运行状态。 */
void FOC_Motor_Start(void);
/** @brief 立即关闭功率输出、停止控制触发链并清除控制目标。 */
void FOC_Motor_Stop(void);
/** @brief 仅在STOPPED接受；回零后进入位置闭环，保持机械0°。 */
void FOC_Motor_GoZero(void);

#endif // FOC_MANAGER_H
