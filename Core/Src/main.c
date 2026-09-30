/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           main.c
  * @brief          FOC主程序
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  FOC_MOTOR_STOPPED = 0,
  FOC_MOTOR_ALIGNING,
  FOC_MOTOR_ALIGN_RELEASE,
  FOC_MOTOR_RUNNING
} FOC_MOTOR_State;
typedef struct
{
  GPIO_PinState last_level;
  GPIO_PinState stable_level;
  uint32_t last_change_tick;
} FOC_SPEED_BUTTON_State;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FOC_BUTTON_PORT        GPIOD
#define FOC_BUTTON_PIN         GPIO_PIN_2
#define FOC_BUTTON_DEBOUNCE_MS 30U
#define FOC_SPEED_UP_PORT      GPIOB
#define FOC_SPEED_UP_PIN       GPIO_PIN_6
#define FOC_SPEED_DOWN_PORT    GPIOC
#define FOC_SPEED_DOWN_PIN     GPIO_PIN_9
#define FOC_IDLE_TELEMETRY_PERIOD_MS 10U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FOC_ENCODER_HandleTypeDef foc_encoder;
FOC_CURRENT_HandleTypeDef foc_current;
FOC_BUS_VOLTAGE_HandleTypeDef foc_bus_voltage;
FOC_SVPWM_HandleTypeDef foc_svpwm;
FOC_LOOP_CUR_HandleTypeDef loop_cur;
FOC_LOOP_SPD_HandleTypeDef loop_spd;
FOC_LOOP_POS_HandleTypeDef loop_pos;
uint32_t bus_voltage_last_tick;
static uint8_t current_offset_valid;
static uint32_t idle_telemetry_last_tick;
static volatile FOC_MOTOR_State motor_state = FOC_MOTOR_STOPPED;
static GPIO_PinState button_last_level;
static GPIO_PinState button_stable_level;
static uint32_t button_last_change_tick;
static FOC_SPEED_BUTTON_State speed_up_button;
static FOC_SPEED_BUTTON_State speed_down_button;
static uint32_t alignment_start_tick;
static uint32_t alignment_sample_last_tick;
static uint32_t alignment_release_start_tick;
static uint8_t alignment_sample_count;
static float alignment_angle_samples[ENCODER_ALIGN_SAMPLE_COUNT];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  立即关闭电机输出并清除控制目标
 */
static void FOC_MOTOR_Stop(void)
{
  motor_state = FOC_MOTOR_STOPPED;

  // 先无条件关闭TIM1主输出，避免停机过程仍向三相施加电压。
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
  FOC_LOOP_SPD_SetSpeedRef(&loop_spd, 0.0f);
  FOC_LOOP_CUR_SetReference(&loop_cur, 0.0f, 0.0f);
  FOC_PI_Reset(&loop_cur.pi_id);
  FOC_PI_Reset(&loop_cur.pi_iq);
  FOC_PI_Reset(&loop_spd.pi_spd);
  FOC_PI_Reset(&loop_pos.pi_pos);
  FOC_ENCODER_UpdateAngle(&foc_encoder);
  foc_encoder.spd_last_count = foc_encoder.count;
  idle_telemetry_last_tick = HAL_GetTick();
  FOC_LOOP_POS_SetPositionRef(&loop_pos, foc_encoder.angle_m);
  FOC_SVPWM_Update(&foc_svpwm, 0.0f, 0.0f, foc_bus_voltage.voltage);
}

/**
 * @brief  开始电角度对齐，控制中断仍保持关闭
 */
static void FOC_MOTOR_BeginAlignment(void)
{
  const uint32_t phase_mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                              TIM_CCER_CC2E | TIM_CCER_CC2NE |
                              TIM_CCER_CC3E | TIM_CCER_CC3NE;
  if (current_offset_valid == 0U)
  {
    return;
  }
  FOC_BUS_VOLTAGE_Update(&foc_bus_voltage);
  if (foc_bus_voltage.voltage <= 0.0f)
  {
    return;
  }

  // 丢弃停机期间收到的旧目标，按键启动后从零目标开始。
  FOC_LOOP_SPD_SetSpeedRef(&loop_spd, 0.0f);
  FOC_LOOP_CUR_SetReference(&loop_cur, 0.0f, 0.0f);
  FOC_PI_Reset(&loop_cur.pi_id);
  FOC_PI_Reset(&loop_cur.pi_iq);
  FOC_PI_Reset(&loop_spd.pi_spd);
  FOC_PI_Reset(&loop_pos.pi_pos);

  FOC_SVPWM_Update(&foc_svpwm, 0.0f, 0.0f, foc_bus_voltage.voltage);
  FOC_SVPWM_Start(&foc_svpwm);
  if ((htim1.Instance->CCER & phase_mask) != phase_mask ||
      (htim1.Instance->BDTR & TIM_BDTR_MOE) == 0U)
  {
    FOC_MOTOR_Stop();
    return;
  }
  FOC_SVPWM_Update(&foc_svpwm, ENCODER_ALIGN_VOLTAGE, 0.0f, foc_bus_voltage.voltage);
  alignment_start_tick = HAL_GetTick();
  alignment_sample_last_tick = alignment_start_tick;
  alignment_sample_count = 0U;
  motor_state = FOC_MOTOR_ALIGNING;
}

/**
 * @brief  分时采样编码器，撤掉对齐电压并等待释放后启动闭环
 */
static void FOC_MOTOR_ProcessAlignment(void)
{
  uint32_t now = HAL_GetTick();

  if (motor_state == FOC_MOTOR_ALIGN_RELEASE)
  {
    if (foc_bus_voltage.voltage <= 0.0f ||
        (htim1.Instance->BDTR & TIM_BDTR_MOE) == 0U)
    {
      FOC_MOTOR_Stop();
      return;
    }
    if ((uint32_t)(now - alignment_release_start_tick) < ENCODER_ALIGN_RELEASE_TIME_MS)
    {
      return;
    }

    // 转子释放后重新同步角度与测速历史，避免首次速度反馈跳变。
    FOC_ENCODER_UpdateAngle(&foc_encoder);
    foc_encoder.spd_last_count = foc_encoder.count;
    foc_encoder.speed = 0.0f;
    loop_spd.speed_fbk = 0.0f;
    FOC_LOOP_SPD_SetSpeedRef(&loop_spd, 0.0f);
    FOC_LOOP_CUR_SetReference(&loop_cur, 0.0f, 0.0f);
    FOC_PI_Reset(&loop_cur.pi_id);
    FOC_PI_Reset(&loop_cur.pi_iq);
    FOC_PI_Reset(&loop_spd.pi_spd);
    FOC_PI_Reset(&loop_pos.pi_pos);
    FOC_LOOP_POS_SetPositionRef(&loop_pos, foc_encoder.angle_m);
    FOC_SVPWM_Update(&foc_svpwm, 0.0f, 0.0f, foc_bus_voltage.voltage);

    // 控制器归零后启动ADC电流环与TIM6；成功后才设置启动速度目标。
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK ||
        HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK ||
        HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
      FOC_MOTOR_Stop();
      return;
    }
    if (FOC_VOFA_GetControlMode() == FOC_CONTROL_SPEED)
    {
      FOC_LOOP_SPD_SetSpeedRef(&loop_spd, MOTOR_START_SPEED_RPM);
    }
    motor_state = FOC_MOTOR_RUNNING;
    return;
  }

  if (motor_state != FOC_MOTOR_ALIGNING)
  {
    return;
  }
  if (foc_bus_voltage.voltage <= 0.0f ||
      (htim1.Instance->BDTR & TIM_BDTR_MOE) == 0U)
  {
    FOC_MOTOR_Stop();
    return;
  }

  uint32_t elapsed = (uint32_t)(now - alignment_start_tick);
  if (elapsed < ENCODER_ALIGN_TIME_MS)
  {
    return;
  }
  if (elapsed > ENCODER_ALIGN_TIME_MS + ENCODER_ALIGN_SAMPLE_TIMEOUT_MS)
  {
    FOC_MOTOR_Stop();
    return;
  }

  if (alignment_sample_count == 0U ||
      (uint32_t)(now - alignment_sample_last_tick) >= ENCODER_ALIGN_SAMPLE_INTERVAL_MS)
  {
    FOC_ENCODER_UpdateAngle(&foc_encoder);
    alignment_angle_samples[alignment_sample_count++] = foc_encoder.angle_m;
    alignment_sample_last_tick = now;
  }
  if (alignment_sample_count < ENCODER_ALIGN_SAMPLE_COUNT)
  {
    return;
  }

  if (FOC_ENCODER_CalibrateElectricalOffsetSamples(
          &foc_encoder, ENCODER_ALIGN_ANGLE,
          alignment_angle_samples, ENCODER_ALIGN_SAMPLE_COUNT) != HAL_OK)
  {
    FOC_MOTOR_Stop();
    return;
  }
  // 对齐电压撤去后等待转子稳定，期间ADC回调不运行电流PI。
  FOC_SVPWM_Update(&foc_svpwm, 0.0f, 0.0f, foc_bus_voltage.voltage);
  alignment_release_start_tick = now;
  motor_state = FOC_MOTOR_ALIGN_RELEASE;
}

/**
 * @brief  轮询SW1并在稳定按下沿切换启停
 */
static void FOC_MOTOR_ProcessButton(void)
{
  GPIO_PinState level = HAL_GPIO_ReadPin(FOC_BUTTON_PORT, FOC_BUTTON_PIN);
  uint32_t now = HAL_GetTick();

  if (level != button_last_level)
  {
    button_last_level = level;
    button_last_change_tick = now;
  }
  if (level != button_stable_level &&
      (uint32_t)(now - button_last_change_tick) >= FOC_BUTTON_DEBOUNCE_MS)
  {
    button_stable_level = level;
    if (level == GPIO_PIN_SET)
    {
      if (motor_state == FOC_MOTOR_STOPPED)
      {
        if (current_offset_valid == 0U)
        {
          return;
        }
        FOC_MOTOR_BeginAlignment();
      }
      else
      {
        FOC_MOTOR_Stop();
      }
    }
  }
}
/**
 * @brief  检测调速按键经过消抖后的按下沿
 */
static uint8_t FOC_MOTOR_SpeedButtonPressed(GPIO_TypeDef *port, uint16_t pin,
                                             FOC_SPEED_BUTTON_State *button,
                                             uint32_t now)
{
  GPIO_PinState level = HAL_GPIO_ReadPin(port, pin);

  if (level != button->last_level)
  {
    button->last_level = level;
    button->last_change_tick = now;
  }
  if (level != button->stable_level &&
      (uint32_t)(now - button->last_change_tick) >= FOC_BUTTON_DEBOUNCE_MS)
  {
    button->stable_level = level;
    return (level == GPIO_PIN_SET) ? 1U : 0U;
  }
  return 0U;
}

/**
 * @brief  SW2/SW3按当前方向增减目标转速，每次按下只调整一次
 */
static void FOC_MOTOR_ProcessSpeedButtons(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t speed_up = FOC_MOTOR_SpeedButtonPressed(
      FOC_SPEED_UP_PORT, FOC_SPEED_UP_PIN, &speed_up_button, now);
  uint8_t speed_down = FOC_MOTOR_SpeedButtonPressed(
      FOC_SPEED_DOWN_PORT, FOC_SPEED_DOWN_PIN, &speed_down_button, now);

  if (motor_state != FOC_MOTOR_RUNNING ||
      FOC_VOFA_GetControlMode() != FOC_CONTROL_SPEED ||
      speed_up == speed_down)
  {
    return;
  }

  float speed_ref = loop_spd.speed_ref;
  float speed_magnitude = (speed_ref < 0.0f) ? -speed_ref : speed_ref;
  if (speed_up != 0U)
  {
    speed_magnitude += MOTOR_SPEED_BUTTON_STEP_RPM;
  }
  else
  {
    // 减速最低到零，避免一次按压跨过零速而突然反转。
    speed_magnitude = (speed_magnitude > MOTOR_SPEED_BUTTON_STEP_RPM)
                          ? speed_magnitude - MOTOR_SPEED_BUTTON_STEP_RPM
                          : 0.0f;
  }
  FOC_LOOP_SPD_SetSpeedRef(&loop_spd,
                           (speed_ref < 0.0f) ? -speed_magnitude : speed_magnitude);
}

/* USER CODE END 0 */

/**
  * @brief  完成外设与FOC初始化，并进入主循环
  * @return 不返回
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU配置 ---------------------------------------------------------------*/

  /* 初始化HAL、Flash接口与SysTick。 */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* 配置系统时钟。 */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* 初始化已配置的外设。 */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // 记录上电时的按键电平；若一直按住，必须先松开再按才启动。
  button_last_level = HAL_GPIO_ReadPin(FOC_BUTTON_PORT, FOC_BUTTON_PIN);
  button_stable_level = button_last_level;
  button_last_change_tick = HAL_GetTick();
  speed_up_button.last_level = HAL_GPIO_ReadPin(FOC_SPEED_UP_PORT, FOC_SPEED_UP_PIN);
  speed_up_button.stable_level = speed_up_button.last_level;
  speed_up_button.last_change_tick = button_last_change_tick;
  speed_down_button.last_level = HAL_GPIO_ReadPin(FOC_SPEED_DOWN_PORT, FOC_SPEED_DOWN_PIN);
  speed_down_button.stable_level = speed_down_button.last_level;
  speed_down_button.last_change_tick = button_last_change_tick;

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
  FOC_ENCODER_Init(&foc_encoder, &htim3);
  
  FOC_CURRENT_Init(&foc_current, &hadc1, &hadc2);
  
  // 对齐和闭环控制前先取得母线电压。
  FOC_BUS_VOLTAGE_Init(&foc_bus_voltage, &hadc1);
  FOC_BUS_VOLTAGE_Update(&foc_bus_voltage);
  bus_voltage_last_tick = HAL_GetTick();
  
  FOC_SVPWM_Init(&foc_svpwm, &htim1);
  
  FOC_LOOP_CUR_Init(&loop_cur,
                    &foc_encoder,
                    &foc_current,
                    &foc_svpwm,
                    CURRENT_KP_D,
                    CURRENT_KI_D,
                    CURRENT_KP_Q,
                    CURRENT_KI_Q,
                    CURRENT_PI_VOLTAGE_LIMIT);
  FOC_LOOP_SPD_Init(&loop_spd, &foc_encoder, &loop_cur, SPEED_KP, SPEED_KI);
  FOC_LOOP_POS_Init(&loop_pos, &foc_encoder, &loop_spd, POSITION_KP, POSITION_KI);

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
    offset_status = FOC_CURRENT_CalibrateOffset(&foc_current, &htim1);
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
  if(FOC_VOFA_Init(&huart2, &foc_encoder, &loop_cur, &loop_spd, &loop_pos) != HAL_OK)
  {
    Error_Handler();
  }
  foc_encoder.spd_last_count = __HAL_TIM_GET_COUNTER(&htim3);
  idle_telemetry_last_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* 主循环 */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 母线电压每10ms采样一次；通信打包和命令解析都在主循环中执行。
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - bus_voltage_last_tick) >= 10U)
    {
      bus_voltage_last_tick = now;
      FOC_BUS_VOLTAGE_Update(&foc_bus_voltage);
    }
    FOC_MOTOR_ProcessButton();
    FOC_MOTOR_ProcessSpeedButtons();
    FOC_MOTOR_ProcessAlignment();
    now = HAL_GetTick();
    uint32_t elapsed_ms = (uint32_t)(now - idle_telemetry_last_tick);
    if (motor_state != FOC_MOTOR_RUNNING &&
        elapsed_ms >= FOC_IDLE_TELEMETRY_PERIOD_MS)
    {
      idle_telemetry_last_tick = now;
      FOC_ENCODER_UpdateSpeed(&foc_encoder, (float)elapsed_ms * 0.001f);
      loop_spd.speed_fbk = foc_encoder.speed;
      // 停机时由主循环请求遥测；运行时由ADC回调分频请求。
      FOC_VOFA_RequestTelemetry();
    }
    FOC_VOFA_ProcessTx();
    FOC_VOFA_ProcessRx();
  }
  /* USER CODE END 3 */
}

/**
  * @brief  配置系统时钟
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 配置内部稳压器输出电压。
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /* 按设定参数初始化RCC振荡器。
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* 配置CPU、AHB和APB总线时钟。
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// TIM6每1ms更新速度环；位置模式下每5次更新一次位置环。
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t pos_cnt = 0;
  if(htim == &htim6 && motor_state == FOC_MOTOR_RUNNING)
  {
    if(FOC_VOFA_GetControlMode() == FOC_CONTROL_POSITION)
    {
      if(++pos_cnt >= 5U)
      {
        pos_cnt = 0U;
        FOC_LOOP_POS_Update(&loop_pos, POSITION_LOOP_TS);
      }
    }
    else
    {
      pos_cnt = 0U;
    }
    FOC_LOOP_SPD_Update(&loop_spd, SPEED_LOOP_TS);
  }
}
// ADC注入转换完成后运行16kHz电流环，每4次请求一次遥测。
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  static uint8_t telemetry_divider = 0U;
  if(hadc == &hadc1)
  {
    if (motor_state != FOC_MOTOR_RUNNING)
    {
      return;
    }
    FOC_LOOP_CUR_Update(&loop_cur, foc_bus_voltage.voltage);
    if(++telemetry_divider >= 4U)
    {
      telemetry_divider = 0U;
      FOC_VOFA_RequestTelemetry();
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  发生初始化错误时停止运行
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 初始化失败后禁止中断，并停留在此处。 */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  处理HAL断言失败
  * @param  file 触发断言的源文件名
  * @param  line 触发断言的行号
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* 可在此处处理断言信息。 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
