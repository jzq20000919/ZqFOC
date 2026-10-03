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

typedef enum
{
  FOC_STATE_STOPPED = 0,
  FOC_STATE_ALIGNING,
  FOC_STATE_ALIGN_RELEASE,
  FOC_STATE_RUNNING
} FOC_Motor_State;

typedef struct
{
  volatile FOC_Motor_State motor_state; // 主循环写入，中断读取

  FOC_ENCODER_HandleTypeDef encoder;
  FOC_CURRENT_HandleTypeDef current;
  FOC_BUS_VOLTAGE_HandleTypeDef bus_voltage;
  FOC_SVPWM_HandleTypeDef svpwm;

  FOC_LOOP_CUR_HandleTypeDef loop_cur;
  FOC_LOOP_SPD_HandleTypeDef loop_spd;
  FOC_LOOP_POS_HandleTypeDef loop_pos;
} FOC_Motor_HandleTypeDef;

extern FOC_Motor_HandleTypeDef foc_motor;

/** @brief 所有MX_*_Init完成后调用一次，初始化FOC并保持停机。 */
void FOC_Motor_Init(void);
/** @brief 在主循环中反复调用，推进按键、对齐、采样及通信任务。 */
void FOC_Motor_Task(void);
/** @brief 仅从STOPPED发起非阻塞对齐，闭环启动由Task继续推进。 */
void FOC_Motor_Start(void);
/** @brief 立即关闭功率输出、停止控制触发链并清除控制目标。 */
void FOC_Motor_Stop(void);

#endif // FOC_MANAGER_H
