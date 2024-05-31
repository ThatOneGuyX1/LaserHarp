/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SINE_AMP 1.5
#define SINE_DC_OFFSET 1.5
#define SINE_STEPS 100

#define AMP_MIN 0
#define AMP_MAX 3
#define DAC_MIN 0
#define DAC_MAX 4095
#define SINE_SAMPLE_COUNT 100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
	void generate_sine_wave(void);
	int scale(float x, int x_min, int x_max, int y_min, int y_max);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int index = 0;
int out_state = 0;
const uint16_t sineLookupTable[] = {
		  2159,3297,4000,4000,3297,2159,1020,317,317,1020};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM15_Init(void);
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

//  2159,2171,2183,2195,2207,2219,2231,2244,
//  2256,2268,2280,2292,2304,2317,2329,2341,
//  2353,2365,2377,2389,2401,2413,2425,2437,
//  2449,2461,2473,2485,2497,2509,2521,2533,
//  2545,2557,2569,2581,2593,2605,2616,2628,
//  2640,2652,2664,2675,2687,2699,2710,2722,
//  2734,2745,2757,2768,2780,2792,2803,2814,
//  2826,2837,2849,2860,2871,2883,2894,2905,
//  2916,2928,2939,2950,2961,2972,2983,2994,
//  3005,3016,3027,3038,3048,3059,3070,3081,
//  3091,3102,3113,3123,3134,3144,3155,3165,
//  3176,3186,3196,3206,3217,3227,3237,3247,
//  3257,3267,3277,3287,3297,3307,3316,3326,
//  3336,3345,3355,3365,3374,3383,3393,3402,
//  3412,3421,3430,3439,3448,3457,3466,3475,
//  3484,3493,3502,3510,3519,3528,3536,3545,
//  3553,3562,3570,3578,3587,3595,3603,3611,
//  3619,3627,3635,3643,3651,3658,3666,3674,
//  3681,3689,3696,3703,3711,3718,3725,3732,
//  3739,3746,3753,3760,3767,3774,3780,3787,
//  3794,3800,3806,3813,3819,3825,3831,3838,
//  3844,3850,3855,3861,3867,3873,3878,3884,
//  3889,3895,3900,3905,3911,3916,3921,3926,
//  3931,3936,3941,3945,3950,3954,3959,3963,
//  3968,3972,3976,3981,3985,3989,3993,3996,
//  4000,4004,4008,4011,4015,4018,4021,4025,
//  4028,4031,4034,4037,4040,4043,4046,4048,
//  4051,4054,4056,4058,4061,4063,4065,4067,
//  4069,4071,4073,4075,4077,4078,4080,4081,
//  4083,4084,4085,4086,4088,4089,4089,4090,
//  4091,4092,4093,4093,4094,4094,4094,4095,
//  4095,4095,4095,4095,4095,4095,4094,4094,
//  4094,4093,4093,4092,4091,4090,4089,4089,
//  4088,4086,4085,4084,4083,4081,4080,4078,
//  4077,4075,4073,4071,4069,4067,4065,4063,
//  4061,4058,4056,4054,4051,4048,4046,4043,
//  4040,4037,4034,4031,4028,4025,4021,4018,
//  4015,4011,4008,4004,4000,3996,3993,3989,
//  3985,3981,3976,3972,3968,3963,3959,3954,
//  3950,3945,3941,3936,3931,3926,3921,3916,
//  3911,3905,3900,3895,3889,3884,3878,3873,
//  3867,3861,3855,3850,3844,3838,3831,3825,
//  3819,3813,3806,3800,3794,3787,3780,3774,
//  3767,3760,3753,3746,3739,3732,3725,3718,
//  3711,3703,3696,3689,3681,3674,3666,3658,
//  3651,3643,3635,3627,3619,3611,3603,3595,
//  3587,3578,3570,3562,3553,3545,3536,3528,
//  3519,3510,3502,3493,3484,3475,3466,3457,
//  3448,3439,3430,3421,3412,3402,3393,3383,
//  3374,3365,3355,3345,3336,3326,3316,3307,
//  3297,3287,3277,3267,3257,3247,3237,3227,
//  3217,3206,3196,3186,3176,3165,3155,3144,
//  3134,3123,3113,3102,3091,3081,3070,3059,
//  3048,3038,3027,3016,3005,2994,2983,2972,
//  2961,2950,2939,2928,2916,2905,2894,2883,
//  2871,2860,2849,2837,2826,2814,2803,2792,
//  2780,2768,2757,2745,2734,2722,2710,2699,
//  2687,2675,2664,2652,2640,2628,2616,2605,
//  2593,2581,2569,2557,2545,2533,2521,2509,
//  2497,2485,2473,2461,2449,2437,2425,2413,
//  2401,2389,2377,2365,2353,2341,2329,2317,
//  2304,2292,2280,2268,2256,2244,2231,2219,
//  2207,2195,2183,2171,2159,2146,2134,2122,
//  2110,2098,2086,2073,2061,2049,2037,2025,
//  2013,2000,1988,1976,1964,1952,1940,1928,
//  1916,1904,1892,1880,1868,1856,1844,1832,
//  1820,1808,1796,1784,1772,1760,1748,1736,
//  1724,1712,1701,1689,1677,1665,1653,1642,
//  1630,1618,1607,1595,1583,1572,1560,1549,
//  1537,1525,1514,1503,1491,1480,1468,1457,
//  1446,1434,1423,1412,1401,1389,1378,1367,
//  1356,1345,1334,1323,1312,1301,1290,1279,
//  1269,1258,1247,1236,1226,1215,1204,1194,
//  1183,1173,1162,1152,1141,1131,1121,1111,
//  1100,1090,1080,1070,1060,1050,1040,1030,
//  1020,1010,1001,991,981,972,962,952,
//  943,934,924,915,905,896,887,878,
//  869,860,851,842,833,824,815,807,
//  798,789,781,772,764,755,747,739,
//  730,722,714,706,698,690,682,674,
//  666,659,651,643,636,628,621,614,
//  606,599,592,585,578,571,564,557,
//  550,543,537,530,523,517,511,504,
//  498,492,486,479,473,467,462,456,
//  450,444,439,433,428,422,417,412,
//  406,401,396,391,386,381,376,372,
//  367,363,358,354,349,345,341,336,
//  332,328,324,321,317,313,309,306,
//  302,299,296,292,289,286,283,280,
//  277,274,271,269,266,263,261,259,
//  256,254,252,250,248,246,244,242,
//  240,239,237,236,234,233,232,231,
//  229,228,228,227,226,225,224,224,
//  223,223,223,222,222,222,222,222,
//  222,222,223,223,223,224,224,225,
//  226,227,228,228,229,231,232,233,
//  234,236,237,239,240,242,244,246,
//  248,250,252,254,256,259,261,263,
//  266,269,271,274,277,280,283,286,
//  289,292,296,299,302,306,309,313,
//  317,321,324,328,332,336,341,345,
//  349,354,358,363,367,372,376,381,
//  386,391,396,401,406,412,417,422,
//  428,433,439,444,450,456,462,467,
//  473,479,486,492,498,504,511,517,
//  523,530,537,543,550,557,564,571,
//  578,585,592,599,606,614,621,628,
//  636,643,651,659,666,674,682,690,
//  698,706,714,722,730,739,747,755,
//  764,772,781,789,798,807,815,824,
//  833,842,851,860,869,878,887,896,
//  905,915,924,934,943,952,962,972,
//  981,991,1001,1010,1020,1030,1040,1050,
//  1060,1070,1080,1090,1100,1111,1121,1131,
//  1141,1152,1162,1173,1183,1194,1204,1215,
//  1226,1236,1247,1258,1269,1279,1290,1301,
//  1312,1323,1334,1345,1356,1367,1378,1389,
//  1401,1412,1423,1434,1446,1457,1468,1480,
//  1491,1503,1514,1525,1537,1549,1560,1572,
//  1583,1595,1607,1618,1630,1642,1653,1665,
//  1677,1689,1701,1712,1724,1736,1748,1760,
//  1772,1784,1796,1808,1820,1832,1844,1856,
//  1868,1880,1892,1904,1916,1928,1940,1952,
//  1964,1976,1988,2000,2013,2025,2037,2049,
//  2061,2073,2086,2098,2110,2122,2134,2146

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_DAC1_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */



  HAL_TIM_Base_Start_IT(&htim15);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Check if button is pressed
	      int read = !(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
	      out_state = (read ^ out_state)? 1 : 0;



	      // Increment the index


	      // Calculate delay for the current frequency


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 10;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1000;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim15 )
  {
	  int wave_out = out_state?sineLookupTable[index]: 0;
	  	    // Get the current sine value


	        // Output to DAC

	  	    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,(uint32_t)  wave_out);
	  	    index++;
	  	    if (index >= 9) index = 0;

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

#ifdef  USE_FULL_ASSERT
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
