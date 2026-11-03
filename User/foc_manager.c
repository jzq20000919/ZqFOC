/**
 * @file    foc_manager.c
 * @brief   四个状态执行函数，Task只负责分发。
 */
#include "foc_manager.h"
#include "adc.h"
#include "opamp.h"
#include "tim.h"

FOC_Motor_HandleTypeDef foc_motor;

static uint8_t power_on_alignment; // 1：上电标定，0：START或单独回零
static float run_position;
static FOC_Motor_State next_state;

static void FOC_Motor_HandleStopped(void);
static void FOC_Motor_HandleAligning(void);
static void FOC_Motor_HandleRunningSpeed(void);
static void FOC_Motor_HandleRunningPosition(void);

void FOC_Motor_Init(void)
{
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3);

  FOC_ENCODER_Init(&foc_motor.encoder, &htim3);
  FOC_CURRENT_Init(&foc_motor.current, &hadc1, &hadc2);
  FOC_BUS_VOLTAGE_Init(&foc_motor.bus_voltage, &hadc1);
  FOC_SVPWM_Init(&foc_motor.svpwm, &htim1);
  FOC_LOOP_CUR_Init(&foc_motor.loop_cur, &foc_motor.encoder, &foc_motor.current,
                   &foc_motor.svpwm, CURRENT_KP_D, CURRENT_KI_D,
                   CURRENT_KP_Q, CURRENT_KI_Q, CURRENT_PI_VOLTAGE_LIMIT);
  FOC_LOOP_SPD_Init(&foc_motor.loop_spd, &foc_motor.encoder, &foc_motor.loop_cur, SPEED_KP, SPEED_KI);
  FOC_LOOP_POS_Init(&foc_motor.loop_pos, &foc_motor.encoder, &foc_motor.loop_spd, POSITION_KP, POSITION_KI);

  foc_motor.control_mode = FOC_CONTROL_SPEED;
  foc_motor.speed_target = MOTOR_START_SPEED_RPM;
  foc_motor.position_target = 0.0f;
  foc_motor.control_active = 0U;
  foc_motor.position_active = 0U;
  FOC_CURRENT_StartSampling(&foc_motor.current, &htim1, 0U);
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
  __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
  foc_motor.current.offset_tick = HAL_GetTick();
  power_on_alignment = 1U;
  foc_motor.motor_state = FOC_STATE_ALIGNING;
}

void FOC_Motor_Start(void)
{
  if (foc_motor.motor_state != FOC_STATE_STOPPED) return;
  FOC_Motor_GoZero();
  next_state = foc_motor.control_mode == FOC_CONTROL_POSITION
             ? FOC_STATE_RUNNING_POSITION : FOC_STATE_RUNNING_SPEED;
  run_position = foc_motor.position_target;
}

void FOC_Motor_GoZero(void)
{
  if (foc_motor.motor_state != FOC_STATE_STOPPED) return;
  power_on_alignment = 0U;
  next_state = FOC_STATE_RUNNING_POSITION;
  run_position = 0.0f;

  foc_motor.loop_pos.pi_pos.output_min = -ENCODER_HOME_SPEED_RPM;
  foc_motor.loop_pos.pi_pos.output_max = ENCODER_HOME_SPEED_RPM;
  FOC_LOOP_POS_SetPositionRef(&foc_motor.loop_pos, 0.0f);
  FOC_LOOP_SPD_SetSpeedRef(&foc_motor.loop_spd,
      foc_motor.encoder.zero_known ? 0.0f : ENCODER_HOME_SPEED_RPM);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
  foc_motor.encoder.index_found = 0U;
  foc_motor.encoder.index_armed = !foc_motor.encoder.zero_known;
  foc_motor.position_active = foc_motor.encoder.zero_known;
  FOC_LOOP_CUR_Start(&foc_motor.loop_cur, foc_motor.bus_voltage.voltage);
  HAL_TIM_Base_Start_IT(&htim6);
  foc_motor.control_active = 1U;
  foc_motor.motor_state = FOC_STATE_ALIGNING;
}

void FOC_Motor_Stop(void)
{
  foc_motor.control_active = 0U;
  foc_motor.position_active = 0U;
  foc_motor.encoder.index_armed = 0U;
  FOC_SVPWM_Stop(&foc_motor.svpwm);
  FOC_CURRENT_StopSampling(&foc_motor.current, &htim1);
  HAL_TIM_Base_Stop_IT(&htim6);

  FOC_PI_Reset(&foc_motor.loop_cur.pi_id);
  FOC_PI_Reset(&foc_motor.loop_cur.pi_iq);
  FOC_PI_Reset(&foc_motor.loop_spd.pi_spd);
  FOC_PI_Reset(&foc_motor.loop_pos.pi_pos);
  FOC_LOOP_CUR_SetReference(&foc_motor.loop_cur, 0.0f, 0.0f);
  FOC_LOOP_SPD_SetSpeedRef(&foc_motor.loop_spd, 0.0f);
  foc_motor.loop_pos.pi_pos.output_min = -MOTOR_MAX_SPEED_RPM;
  foc_motor.loop_pos.pi_pos.output_max = MOTOR_MAX_SPEED_RPM;
  foc_motor.loop_spd.speed_fbk = 0.0f;
  foc_motor.loop_spd.iq_ref = 0.0f;
  foc_motor.loop_cur.i_d = 0.0f;
  foc_motor.loop_cur.i_q = 0.0f;
  foc_motor.loop_cur.v_d = 0.0f;
  foc_motor.loop_cur.v_q = 0.0f;
  FOC_ENCODER_UpdateAngle(&foc_motor.encoder);
  foc_motor.encoder.spd_last_count = foc_motor.encoder.count;
  foc_motor.encoder.speed = 0.0f;
  foc_motor.encoder.speed_raw = 0.0f;
  FOC_LOOP_POS_SetPositionRef(&foc_motor.loop_pos, foc_motor.encoder.angle_m);
  foc_motor.motor_state = FOC_STATE_STOPPED;
}

static void FOC_Motor_HandleStopped(void)
{
  FOC_ENCODER_UpdateAngle(&foc_motor.encoder);
}

static void FOC_Motor_HandleAligning(void)
{
  // 上电：对齐、首次寻Z和保存参数，完成后停机。
  if (power_on_alignment)
  {
    if (!FOC_CURRENT_CalibrateOffset(&foc_motor.current, &htim1)) return;
    if (!FOC_ENCODER_Align(&foc_motor.encoder, &foc_motor.svpwm, foc_motor.bus_voltage.voltage)) return;
    if (foc_motor.encoder.zero_saved)
    {
      FOC_Motor_Stop();
      return;
    }

    if (!foc_motor.encoder.index_found && !foc_motor.control_active)
    {
      foc_motor.encoder.index_armed = 1U;
      FOC_LOOP_SPD_SetSpeedRef(&foc_motor.loop_spd, ENCODER_HOME_SPEED_RPM);
      FOC_LOOP_CUR_Start(&foc_motor.loop_cur, foc_motor.bus_voltage.voltage);
      HAL_TIM_Base_Start_IT(&htim6);
      foc_motor.control_active = 1U;
    }
    if (foc_motor.encoder.index_found && foc_motor.control_active)
    {
      FOC_Motor_Stop();
      FOC_ENCODER_CalibrateZero(&foc_motor.encoder);
      foc_motor.motor_state = FOC_STATE_ALIGNING; // Flash完成后再进入STOP。
    }
    return;
  }

  // START或单独回零：恢复坐标，回到机械0点，再切换运行状态。
  if (!foc_motor.encoder.zero_known)
  {
    if (!foc_motor.encoder.index_found) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    FOC_ENCODER_RestoreZero(&foc_motor.encoder);
    FOC_LOOP_SPD_SetSpeedRef(&foc_motor.loop_spd, 0.0f);
    foc_motor.position_active = 1U;
    __set_PRIMASK(primask);
  }
  if (!FOC_ENCODER_AtZero(&foc_motor.encoder)) return;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  foc_motor.loop_pos.pi_pos.output_min = -MOTOR_MAX_SPEED_RPM;
  foc_motor.loop_pos.pi_pos.output_max = MOTOR_MAX_SPEED_RPM;
  foc_motor.position_active = (next_state == FOC_STATE_RUNNING_POSITION);
  foc_motor.control_mode = foc_motor.position_active ? FOC_CONTROL_POSITION : FOC_CONTROL_SPEED;
  if (foc_motor.position_active) foc_motor.position_target = run_position;
  foc_motor.motor_state = next_state;
  __set_PRIMASK(primask);
}

static void FOC_Motor_HandleRunningSpeed(void)
{
  FOC_LOOP_SPD_SetSpeedRef(&foc_motor.loop_spd, foc_motor.speed_target);
}

static void FOC_Motor_HandleRunningPosition(void)
{
  FOC_LOOP_POS_SetPositionRef(&foc_motor.loop_pos, foc_motor.position_target);
}

void FOC_Motor_Task(void)
{
  switch (foc_motor.motor_state)
  {
    case FOC_STATE_STOPPED:
      FOC_Motor_HandleStopped();
      break;
    case FOC_STATE_ALIGNING:
      FOC_Motor_HandleAligning();
      break;
    case FOC_STATE_RUNNING_SPEED:
      FOC_Motor_HandleRunningSpeed();
      break;
    case FOC_STATE_RUNNING_POSITION:
      FOC_Motor_HandleRunningPosition();
      break;
  }
}
