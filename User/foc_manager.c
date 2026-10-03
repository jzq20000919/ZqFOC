/**
 * @file    foc_manager.c
 * @brief   单电机FOC初始化、启停对齐与主循环任务
 */
#include "foc_manager.h"
#include "foc_key.h"
#include "foc_vofa.h"
#include "main.h"
#include "motor_param.h"
#include "adc.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"

FOC_Motor_HandleTypeDef foc_motor = {
  .motor_state = FOC_STATE_STOPPED
};

#define FOC_IDLE_TELEMETRY_PERIOD_MS 10U

static uint32_t bus_voltage_last_tick;
static uint8_t current_offset_valid;
static uint32_t idle_telemetry_last_tick;
static uint32_t alignment_start_tick;
static uint32_t alignment_sample_last_tick;
static uint32_t alignment_release_start_tick;
static uint8_t alignment_sample_count;
static float alignment_angle_samples[ENCODER_ALIGN_SAMPLE_COUNT];

/**
 * @brief  立即关闭电机输出并清除控制目标
 */
void FOC_Motor_Stop(void)
{
  // 切换状态。
  foc_motor.motor_state = FOC_STATE_STOPPED;

  // 关闭TIM1主输出。
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  // 初始化失败时也可能只启动了部分触发链，停机时全部关闭。
  HAL_TIM_Base_Stop_IT(&htim6);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
  HAL_ADCEx_InjectedStop_IT(&hadc1);
  HAL_ADCEx_InjectedStop(&hadc2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
  // 清除控制目标。
  FOC_LOOP_SPD_SetSpeedRef(&foc_motor.loop_spd, 0.0f);
  FOC_LOOP_CUR_SetReference(&foc_motor.loop_cur, 0.0f, 0.0f);
  // 清除控制器内部状态。
  FOC_PI_Reset(&foc_motor.loop_cur.pi_id);
  FOC_PI_Reset(&foc_motor.loop_cur.pi_iq);
  FOC_PI_Reset(&foc_motor.loop_spd.pi_spd);
  FOC_PI_Reset(&foc_motor.loop_pos.pi_pos);

  FOC_ENCODER_UpdateAngle(&foc_motor.encoder);
  foc_motor.encoder.spd_last_count = foc_motor.encoder.count;
  idle_telemetry_last_tick = HAL_GetTick();
  FOC_LOOP_POS_SetPositionRef(&foc_motor.loop_pos, foc_motor.encoder.angle_m);
  FOC_SVPWM_Update(&foc_motor.svpwm, 0.0f, 0.0f, foc_motor.bus_voltage.voltage);
}

/**
 * @brief  开始电角度对齐，控制中断仍保持关闭
 */
void FOC_Motor_Start(void)
{
  const uint32_t phase_mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                              TIM_CCER_CC2E | TIM_CCER_CC2NE |
                              TIM_CCER_CC3E | TIM_CCER_CC3NE;
  if (foc_motor.motor_state != FOC_STATE_STOPPED || current_offset_valid == 0U)
  {
    return;
  }
  FOC_BUS_VOLTAGE_Update(&foc_motor.bus_voltage);
  if (foc_motor.bus_voltage.voltage <= 0.0f)
  {
    return;
  }
  // 丢弃停机期间收到的旧目标，按键启动后从零目标开始。
  FOC_LOOP_SPD_SetSpeedRef(&foc_motor.loop_spd, 0.0f);
  FOC_LOOP_CUR_SetReference(&foc_motor.loop_cur, 0.0f, 0.0f);
  FOC_PI_Reset(&foc_motor.loop_cur.pi_id);
  FOC_PI_Reset(&foc_motor.loop_cur.pi_iq);
  FOC_PI_Reset(&foc_motor.loop_spd.pi_spd);
  FOC_PI_Reset(&foc_motor.loop_pos.pi_pos);
  FOC_SVPWM_Update(&foc_motor.svpwm, 0.0f, 0.0f, foc_motor.bus_voltage.voltage);
  FOC_SVPWM_Start(&foc_motor.svpwm);
  if ((htim1.Instance->CCER & phase_mask) != phase_mask ||
      (htim1.Instance->BDTR & TIM_BDTR_MOE) == 0U)
  {
    FOC_Motor_Stop();
    return;
  }
  FOC_SVPWM_Update(&foc_motor.svpwm, ENCODER_ALIGN_VOLTAGE, 0.0f, foc_motor.bus_voltage.voltage);
  alignment_start_tick = HAL_GetTick();
  alignment_sample_last_tick = alignment_start_tick;
  alignment_sample_count = 0U;
  foc_motor.motor_state = FOC_STATE_ALIGNING;
}

/**
 * @brief  分时采样编码器，撤掉对齐电压并等待释放后启动闭环
 */
static void FOC_Motor_ProcessAlignment(void)
{
  uint32_t now = HAL_GetTick();

  if (foc_motor.motor_state == FOC_STATE_ALIGN_RELEASE)
  {
    if (foc_motor.bus_voltage.voltage <= 0.0f ||
        (htim1.Instance->BDTR & TIM_BDTR_MOE) == 0U)
    {
      FOC_Motor_Stop();
      return;
    }
    if ((uint32_t)(now - alignment_release_start_tick) < ENCODER_ALIGN_RELEASE_TIME_MS)
    {
      return;
    }

    // 转子释放后重新同步角度与测速历史，避免首次速度反馈跳变。
    FOC_ENCODER_UpdateAngle(&foc_motor.encoder);
    foc_motor.encoder.spd_last_count = foc_motor.encoder.count;
    foc_motor.encoder.speed = 0.0f;
    foc_motor.loop_spd.speed_fbk = 0.0f;
    FOC_LOOP_SPD_SetSpeedRef(&foc_motor.loop_spd, 0.0f);
    FOC_LOOP_CUR_SetReference(&foc_motor.loop_cur, 0.0f, 0.0f);
    FOC_PI_Reset(&foc_motor.loop_cur.pi_id);
    FOC_PI_Reset(&foc_motor.loop_cur.pi_iq);
    FOC_PI_Reset(&foc_motor.loop_spd.pi_spd);
    FOC_PI_Reset(&foc_motor.loop_pos.pi_pos);
    FOC_LOOP_POS_SetPositionRef(&foc_motor.loop_pos, foc_motor.encoder.angle_m);
    FOC_SVPWM_Update(&foc_motor.svpwm, 0.0f, 0.0f, foc_motor.bus_voltage.voltage);

    // 控制器归零后启动ADC电流环与TIM6；成功后才设置启动速度目标。
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK ||
        HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK ||
        HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
      FOC_Motor_Stop();
      return;
    }
    if (FOC_VOFA_GetControlMode() == FOC_CONTROL_SPEED)
    {
      FOC_LOOP_SPD_SetSpeedRef(&foc_motor.loop_spd, MOTOR_START_SPEED_RPM);
    }
    foc_motor.motor_state = FOC_STATE_RUNNING;
    return;
  }

  if (foc_motor.motor_state != FOC_STATE_ALIGNING)
  {
    return;
  }
  if (foc_motor.bus_voltage.voltage <= 0.0f ||
      (htim1.Instance->BDTR & TIM_BDTR_MOE) == 0U)
  {
    FOC_Motor_Stop();
    return;
  }

  uint32_t elapsed = (uint32_t)(now - alignment_start_tick);
  if (elapsed < ENCODER_ALIGN_TIME_MS)
  {
    return;
  }
  if (elapsed > ENCODER_ALIGN_TIME_MS + ENCODER_ALIGN_SAMPLE_TIMEOUT_MS)
  {
    FOC_Motor_Stop();
    return;
  }

  if (alignment_sample_count == 0U ||
      (uint32_t)(now - alignment_sample_last_tick) >= ENCODER_ALIGN_SAMPLE_INTERVAL_MS)
  {
    FOC_ENCODER_UpdateAngle(&foc_motor.encoder);
    alignment_angle_samples[alignment_sample_count++] = foc_motor.encoder.angle_m;
    alignment_sample_last_tick = now;
  }
  if (alignment_sample_count < ENCODER_ALIGN_SAMPLE_COUNT)
  {
    return;
  }

  if (FOC_ENCODER_CalibrateElectricalOffsetSamples(
          &foc_motor.encoder, ENCODER_ALIGN_ANGLE,
          alignment_angle_samples, ENCODER_ALIGN_SAMPLE_COUNT) != HAL_OK)
  {
    FOC_Motor_Stop();
    return;
  }
  // 对齐电压撤去后等待转子稳定，期间ADC回调不运行电流PI。
  FOC_SVPWM_Update(&foc_motor.svpwm, 0.0f, 0.0f, foc_motor.bus_voltage.voltage);
  alignment_release_start_tick = now;
  foc_motor.motor_state = FOC_STATE_ALIGN_RELEASE;
}

/**
 * @brief  在CubeMX外设初始化完成后初始化FOC，功率输出保持关闭
 */
void FOC_Motor_Init(void)
{
  foc_motor.motor_state = FOC_STATE_STOPPED;
  current_offset_valid = 0U;
  alignment_start_tick = 0U;
  alignment_sample_last_tick = 0U;
  alignment_release_start_tick = 0U;
  alignment_sample_count = 0U;

  FOC_KEY_Init();

  // 先校准ADC并启动采样运放，为后续电流采样做准备。
  if(HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_OPAMP_Start(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_OPAMP_Start(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_OPAMP_Start(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_Delay(1);

  // 初始化传感器、PWM和三个控制环。
  FOC_ENCODER_Init(&foc_motor.encoder, &htim3);

  FOC_CURRENT_Init(&foc_motor.current, &hadc1, &hadc2);

  // 对齐和闭环控制前先取得母线电压。
  FOC_BUS_VOLTAGE_Init(&foc_motor.bus_voltage, &hadc1);
  FOC_BUS_VOLTAGE_Update(&foc_motor.bus_voltage);
  bus_voltage_last_tick = HAL_GetTick();

  FOC_SVPWM_Init(&foc_motor.svpwm, &htim1);

  FOC_LOOP_CUR_Init(&foc_motor.loop_cur,
                    &foc_motor.encoder,
                    &foc_motor.current,
                    &foc_motor.svpwm,
                    CURRENT_KP_D,
                    CURRENT_KI_D,
                    CURRENT_KP_Q,
                    CURRENT_KI_Q,
                    CURRENT_PI_VOLTAGE_LIMIT);
  FOC_LOOP_SPD_Init(&foc_motor.loop_spd, &foc_motor.encoder, &foc_motor.loop_cur, SPEED_KP, SPEED_KI);
  FOC_LOOP_POS_Init(&foc_motor.loop_pos, &foc_motor.encoder, &foc_motor.loop_spd, POSITION_KP, POSITION_KI);

  // 上电只开启CH4作为注入触发源，三相功率通道保持关闭。
  CLEAR_BIT(htim1.Instance->CCER,
            TIM_CCER_CC1E | TIM_CCER_CC1NE |
            TIM_CCER_CC2E | TIM_CCER_CC2NE |
            TIM_CCER_CC3E | TIM_CCER_CC3NE);
  HAL_StatusTypeDef offset_status = HAL_ERROR;
  if (HAL_ADCEx_InjectedStart(&hadc2) == HAL_OK &&
      HAL_ADCEx_InjectedStart(&hadc1) == HAL_OK &&
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) == HAL_OK)
  {
    offset_status = FOC_CURRENT_CalibrateOffset(&foc_motor.current, &htim1);
  }
  HAL_StatusTypeDef tim_stop_status = HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
  HAL_StatusTypeDef adc1_stop_status = HAL_ADCEx_InjectedStop(&hadc1);
  HAL_StatusTypeDef adc2_stop_status = HAL_ADCEx_InjectedStop(&hadc2);
  if (tim_stop_status != HAL_OK || adc1_stop_status != HAL_OK ||
      adc2_stop_status != HAL_OK)
  {
    offset_status = HAL_ERROR;
  }
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  current_offset_valid = (offset_status == HAL_OK) ? 1U : 0U;

  // 上电启动VOFA通信；功率输出仍由PD2按键启动。
  if(FOC_VOFA_Init(&huart2, &foc_motor.encoder, &foc_motor.loop_cur, &foc_motor.loop_spd, &foc_motor.loop_pos) != HAL_OK)
  {
    Error_Handler();
  }
  foc_motor.encoder.spd_last_count = __HAL_TIM_GET_COUNTER(&htim3);
  idle_telemetry_last_tick = HAL_GetTick();
}

/**
 * @brief  在主循环中推进低频任务和非阻塞对齐流程
 */
void FOC_Motor_Task(void)
{
  // 母线电压每10ms采样一次；通信打包和命令解析都在主循环中执行。
  uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - bus_voltage_last_tick) >= 10U)
  {
    bus_voltage_last_tick = now;
    FOC_BUS_VOLTAGE_Update(&foc_motor.bus_voltage);
  }
  FOC_KEY_Task();
  FOC_Motor_ProcessAlignment();
  now = HAL_GetTick();
  uint32_t elapsed_ms = (uint32_t)(now - idle_telemetry_last_tick);
  if (foc_motor.motor_state != FOC_STATE_RUNNING &&
      elapsed_ms >= FOC_IDLE_TELEMETRY_PERIOD_MS)
  {
    idle_telemetry_last_tick = now;
    FOC_ENCODER_UpdateSpeed(&foc_motor.encoder, (float)elapsed_ms * 0.001f);
    foc_motor.loop_spd.speed_fbk = foc_motor.encoder.speed;
    // 停机时由主循环请求遥测；运行时由ADC回调分频请求。
    FOC_VOFA_RequestTelemetry();
  }
  FOC_VOFA_ProcessTx();
  FOC_VOFA_ProcessRx();
}
