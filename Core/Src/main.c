/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
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
  //启动ADC校准
  if(HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  //启动OPAMP
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

  // 初始化编码器
  FOC_ENCODER_Init(&foc_encoder, &htim3);
  
  // 初始化三相电流采样
  FOC_CURRENT_Init(&foc_current, &hadc1, &hadc2);
  
  //初始化母线电压采样
  FOC_BUS_VOLTAGE_Init(&foc_bus_voltage, &hadc1);
  FOC_BUS_VOLTAGE_Update(&foc_bus_voltage);
  bus_voltage_last_tick = HAL_GetTick();
  
  //SVPWM
  FOC_SVPWM_Init(&foc_svpwm, &htim1);
  
  // 初始化电流环
  FOC_LOOP_CUR_Init(&loop_cur,
                    &foc_encoder,
                    &foc_current,
                    &foc_svpwm,
                    CURRENT_KP_D,
                    CURRENT_KI_D,
                    CURRENT_KP_Q,
                    CURRENT_KI_Q,
                    CURRENT_PI_VOLTAGE_LIMIT);
  // 初始化速度环
  FOC_LOOP_SPD_Init(&loop_spd, &foc_encoder, &loop_cur, SPEED_KP, SPEED_KI);
  // 初始化位置环
  FOC_LOOP_POS_Init(&loop_pos, &foc_encoder, &loop_spd, POSITION_KP, POSITION_KI);

  //启动三相互补PWM
  FOC_SVPWM_Start(&foc_svpwm);
  //施加固定电角度磁场
  FOC_SVPWM_Update(&foc_svpwm, ENCODER_ALIGN_VOLTAGE, 0.0f, foc_bus_voltage.voltage);
  HAL_Delay(ENCODER_ALIGN_TIME_MS);
  //计算电角度偏置
  FOC_ENCODER_CalibrateElectricalOffset(&foc_encoder, ENCODER_ALIGN_ANGLE);
  //停止施加磁场
  FOC_SVPWM_Update(&foc_svpwm, 0.0f, 0.0f, foc_bus_voltage.voltage);

  //启动ADC注入组转换
  if(HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  //启动定时器4驱动ADC
  if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  
  //启动1khz速度/位置环计时器
  if(HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - bus_voltage_last_tick) >= 10U)
    {
      bus_voltage_last_tick = now;
      FOC_BUS_VOLTAGE_Update(&foc_bus_voltage);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
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

  /** Initializes the CPU, AHB and APB buses clocks
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
// 速度环和位置环的定时回调
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t pos_cnt = 0;
  if(htim == &htim6)
  {
    if(++pos_cnt >=5)
    {
      pos_cnt = 0;
      FOC_LOOP_POS_Update(&loop_pos,POSITION_LOOP_TS);
    }
    FOC_LOOP_SPD_Update(&loop_spd,SPEED_LOOP_TS);

  }
}
//ADC注入转换回调
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if(hadc == &hadc1)
  {
    FOC_LOOP_CUR_Update(&loop_cur,foc_bus_voltage.voltage);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
