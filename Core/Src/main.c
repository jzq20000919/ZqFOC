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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  
  // 对齐前先取得有效母线电压，否则SVPWM会跳过输出。
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

  // 固定磁场完成电角度对齐；此时尚未启动注入采样，电流环不会改写PWM。
  FOC_SVPWM_Start(&foc_svpwm);
  FOC_SVPWM_Update(&foc_svpwm, ENCODER_ALIGN_VOLTAGE, 0.0f, foc_bus_voltage.voltage);
  HAL_Delay(ENCODER_ALIGN_TIME_MS);
  FOC_ENCODER_CalibrateElectricalOffset(&foc_encoder, ENCODER_ALIGN_ANGLE);
  FOC_SVPWM_Update(&foc_svpwm, 0.0f, 0.0f, foc_bus_voltage.voltage);

  // 对齐结束后启动ADC注入组与TIM1 CH4，形成16kHz电流环触发链。
  if(HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  if(FOC_VOFA_Init(&huart2, &foc_encoder, &loop_cur, &loop_spd, &loop_pos) != HAL_OK)
  {
    Error_Handler();
  }
  
  // 最后启动TIM6：速度环1kHz，位置模式下位置环200Hz。
  if(HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

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
  if(htim == &htim6)
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
// ADC注入完成后更新16kHz电流环；每4次仅请求一次遥测。
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  static uint8_t telemetry_divider = 0U;
  if(hadc == &hadc1)
  {
    FOC_LOOP_CUR_Update(&loop_cur,foc_bus_voltage.voltage);
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
