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
#include "foc_diagnostic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#if !FOC_CURRENT_ZERO_TEST
typedef enum
{
  FOC_MOTOR_STOPPED = 0,
  FOC_MOTOR_ALIGNING,
  FOC_MOTOR_RUNNING
} FOC_MOTOR_State;
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FOC_BUTTON_PORT        GPIOD
#define FOC_BUTTON_PIN         GPIO_PIN_2
#define FOC_BUTTON_DEBOUNCE_MS 30U
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
#if FOC_CURRENT_ZERO_TEST
static uint32_t diagnostic_telemetry_last_tick;
#else
static uint32_t idle_telemetry_last_tick;
static volatile FOC_MOTOR_State motor_state = FOC_MOTOR_STOPPED;
static GPIO_PinState button_last_level;
static GPIO_PinState button_stable_level;
static uint32_t button_last_change_tick;
static uint32_t alignment_start_tick;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if !FOC_CURRENT_ZERO_TEST
/**
 * @brief  立即关闭电机输出并清除控制目标
 */
static void FOC_MOTOR_Stop(void)
{
  FOC_MOTOR_State previous_state = motor_state;
  motor_state = FOC_MOTOR_STOPPED;

  // 先无条件关闭TIM1主输出，再停止触发源与各PWM通道。
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  if (previous_state == FOC_MOTOR_RUNNING)
  {
    HAL_TIM_Base_Stop_IT(&htim6);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    HAL_ADCEx_InjectedStop_IT(&hadc1);
    HAL_ADCEx_InjectedStop(&hadc2);
  }
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
  FOC_SVPWM_Update(&foc_svpwm, ENCODER_ALIGN_VOLTAGE, 0.0f, foc_bus_voltage.voltage);
  alignment_start_tick = HAL_GetTick();
  motor_state = FOC_MOTOR_ALIGNING;
}

/**
 * @brief  对齐时间结束后启动电流环与速度环触发链
 */
static void FOC_MOTOR_ProcessAlignment(void)
{
  if (motor_state != FOC_MOTOR_ALIGNING ||
      (uint32_t)(HAL_GetTick() - alignment_start_tick) < ENCODER_ALIGN_TIME_MS)
  {
    return;
  }

  if (foc_bus_voltage.voltage <= 0.0f)
  {
    FOC_MOTOR_Stop();
    return;
  }

  FOC_ENCODER_CalibrateElectricalOffset(&foc_encoder, ENCODER_ALIGN_ANGLE);
  FOC_SVPWM_Update(&foc_svpwm, 0.0f, 0.0f, foc_bus_voltage.voltage);
  FOC_LOOP_SPD_SetSpeedRef(&loop_spd, 0.0f);
  FOC_LOOP_CUR_SetReference(&loop_cur, 0.0f, 0.0f);
  FOC_PI_Reset(&loop_cur.pi_id);
  FOC_PI_Reset(&loop_cur.pi_iq);
  FOC_PI_Reset(&loop_spd.pi_spd);
  FOC_PI_Reset(&loop_pos.pi_pos);
  FOC_LOOP_POS_SetPositionRef(&loop_pos, foc_encoder.angle_m);

  // ADC注入组先就绪，最后启动CH4和TIM6，避免对齐磁场被电流环覆盖。
  motor_state = FOC_MOTOR_RUNNING;
  if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK ||
      HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK ||
      HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    FOC_MOTOR_Stop();
  }
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
        FOC_MOTOR_BeginAlignment();
      }
      else
      {
        FOC_MOTOR_Stop();
      }
    }
  }
}
#endif

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
#if !FOC_CURRENT_ZERO_TEST
  // 记录上电时的按键电平；若一直按住，必须先松开再按才启动。
  button_last_level = HAL_GPIO_ReadPin(FOC_BUTTON_PORT, FOC_BUTTON_PIN);
  button_stable_level = button_last_level;
  button_last_change_tick = HAL_GetTick();
#endif

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
  
  // 正常模式下对齐前先取得母线电压；实验模式不驱动电机。
  FOC_BUS_VOLTAGE_Init(&foc_bus_voltage, &hadc1);
#if !FOC_CURRENT_ZERO_TEST
  FOC_BUS_VOLTAGE_Update(&foc_bus_voltage);
  bus_voltage_last_tick = HAL_GetTick();
#endif
  
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

  // 实验模式保留串口遥测，但不启动三相功率输出和控制环。
  if(FOC_VOFA_Init(&huart2, &foc_encoder, &loop_cur, &loop_spd, &loop_pos) != HAL_OK)
  {
    Error_Handler();
  }
#if FOC_CURRENT_ZERO_TEST
  // 三相输出通道保持关闭；只启动CH4以沿用已验证的ADC注入触发路径。
  CLEAR_BIT(htim1.Instance->CCER,
            TIM_CCER_CC1E | TIM_CCER_CC1NE |
            TIM_CCER_CC2E | TIM_CCER_CC2NE |
            TIM_CCER_CC3E | TIM_CCER_CC3NE);
  if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK ||
      HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  diagnostic_telemetry_last_tick = HAL_GetTick();
#else
  foc_encoder.spd_last_count = __HAL_TIM_GET_COUNTER(&htim3);
  idle_telemetry_last_tick = HAL_GetTick();
#endif

  /* USER CODE END 2 */

  /* 主循环 */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if FOC_CURRENT_ZERO_TEST
    // 只计算静止电流反馈，不执行PI或更新PWM。
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - diagnostic_telemetry_last_tick) >= 10U)
    {
      FOC_MATH_CLARKE_HandleTypeDef clarke_out;
      FOC_MATH_PARK_HandleTypeDef park_out;
      const volatile FOC_CURRENT_HandleTypeDef *sample = &foc_current;
      float i_a = sample->i_a;
      float i_b = sample->i_b;
      float i_c = sample->i_c;
      FOC_ENCODER_UpdateAngle(&foc_encoder);
      clarke_out = FOC_MATH_Clarke(i_a, i_b, i_c);
      park_out = FOC_MATH_Park(clarke_out.i_alpha, clarke_out.i_beta,
                               foc_encoder.angle_e);
      loop_cur.i_d = park_out.i_d;
      loop_cur.i_q = park_out.i_q;
      diagnostic_telemetry_last_tick = now;
      FOC_VOFA_RequestTelemetry();
    }
#else
    // 母线电压每10ms采样一次；通信打包和命令解析都在主循环中执行。
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - bus_voltage_last_tick) >= 10U)
    {
      bus_voltage_last_tick = now;
      FOC_BUS_VOLTAGE_Update(&foc_bus_voltage);
    }
    FOC_MOTOR_ProcessButton();
    FOC_MOTOR_ProcessAlignment();
    now = HAL_GetTick();
    if (motor_state != FOC_MOTOR_RUNNING)
    {
      uint32_t elapsed_ms = (uint32_t)(now - idle_telemetry_last_tick);
      if (elapsed_ms >= FOC_IDLE_TELEMETRY_PERIOD_MS)
      {
        idle_telemetry_last_tick = now;
        FOC_ENCODER_UpdateSpeed(&foc_encoder, (float)elapsed_ms * 0.001f);
        loop_spd.speed_fbk = foc_encoder.speed;
        // 停机时没有ADC回调，由主循环以100Hz请求遥测。
        FOC_VOFA_RequestTelemetry();
      }
    }
#endif
    FOC_VOFA_ProcessTx();
#if !FOC_CURRENT_ZERO_TEST
    FOC_VOFA_ProcessRx();
#endif
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
#if !FOC_CURRENT_ZERO_TEST
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
#endif
// 实验模式只更新采样值；正常模式更新电流环并分频请求遥测。
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
#if !FOC_CURRENT_ZERO_TEST
  static uint8_t telemetry_divider = 0U;
#endif
#if FOC_CURRENT_ZERO_TEST
  if (hadc == &hadc1)
  {
    FOC_CURRENT_Update(&foc_current);
  }
#else
  if(hadc == &hadc1 && motor_state == FOC_MOTOR_RUNNING)
  {
    FOC_LOOP_CUR_Update(&loop_cur,foc_bus_voltage.voltage);
    if(++telemetry_divider >= 4U)
    {
      telemetry_divider = 0U;
      FOC_VOFA_RequestTelemetry();
    }
  }
#endif
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
