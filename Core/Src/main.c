/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws2812b.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "libPWM.h"
#include "sysInit.h"
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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char trans_str[64] = {0,};
uint32_t falling;
uint32_t period;
int out;
int A_chan, B_chan;
int b;
int state; 				// Button flag
int LED;
int led_num1 =1;
int led_num2 = 32;
int led_flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void measure_func1(void);
void compare_out(void);
void next_meas(void);
void mux_addr(uint8_t b2,uint8_t b3,uint8_t b4,uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4);
void measure_process(int number);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	int i = 0;
	for (i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  Mux_GPIO();
  ws2812b_init();
  while(!ws2812b_is_ready()) {}
  PWM_Init(36, 100, 50); 			// Значение сдвига фаз 25% от периода
  TIM3->CR1 |= TIM_CR1_CEN;
  //TIM3->CR1 |= TIM_CR1_CEN;
  //  mux_addr(0, 0, 0, 0, 0, 0, 1);
//  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
//  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
//    ws2812b_set(17, 25, 0, 0);
// 	ws2812b_send();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  snprintf(trans_str, 63, "Pulse %lu mks\r\n", falling);
//	  HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 1000);
//	  falling = 0;
//	  HAL_Delay(400);

//	  state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);
//
//	  if (state == 0 && b == 0)	// push the button
//	  {
//		  b = 1;
//	  }
//	  if (state ==1 && b == 1)
//	  {
//		  GPIOB->ODR ^= GPIO_ODR_ODR7;
//		  next_meas();
//		  b = 0;
//	  }

	  next_meas();
	  HAL_Delay(50);

//	  mux_addr(0, 0, 0, 0, 0, 1, 0);
//	  for (int num = 0; num < 4; num++) {
//		  measure_func();
//	  }
//	  HAL_Delay(500);
//	  if (falling > 19) {
//
//		  ws2812b_set(2, 60, 0, 0);
//		  ws2812b_send();
//	  } else {
//		  ws2812b_set(2, 0, 60, 0);
//		  ws2812b_send();
//	  }
//	  HAL_Delay(500);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) 								// колбек по захвату
{
        if(htim->Instance == TIM1)
        {
                if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) 							// RISING с LOW на HIGH
                {
                        __HAL_TIM_SET_COUNTER(&htim1, 0x0000); 							// обнуление счётчика
                }

                else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) 						// FALLING с HIGH на LOW
                {
                        falling = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2); 	// чтение значения в регистре захвата/сравнения
//                        snprintf(trans_str, 63, "Pulse %lu mks\r\n", falling);
//                        HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 1000);
                }
        }
}

void measure_func1(void) {

	//TIM3->CR1 |= TIM_CR1_CEN;
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 1000);
	snprintf(trans_str, 63, "Pulse %lu mks\r\n", falling);
	HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 1000);
	//HAL_Delay(400);
	//TIM3->CR1 &= ~TIM_CR1_CEN;
	HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_2);

}

/************************************************** Some function ****************************************************************/

void compare_out(void) {

	switch(out)
	{

	case 0:
		//ws2812b_buff_claer();
		led_num1 =1;
		led_num2 = 32;
		HAL_Delay(150);

		break;

	case 1:
		mux_addr(0, 0, 0, 0, 0, 0, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 0, 0, 0, 0, 1);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 2:
		mux_addr(0, 0, 0, 0, 0, 1, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 0, 0, 0, 1, 0);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 3:
		mux_addr(0, 0, 0, 0, 0, 1, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 0, 0, 0, 1, 1);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 4:
		mux_addr(0, 0, 0, 0, 1, 0, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 0, 0, 1, 0, 0);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 5:
		mux_addr(0, 0, 0, 0, 1, 0, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 0, 0, 1, 0, 1);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 6:
		mux_addr(0, 0, 0, 0, 1, 1, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 0, 0, 1, 1, 0);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 7:
		mux_addr(0, 0, 0, 0, 1, 1, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 0, 0, 1, 1, 1);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 8:
		mux_addr(0, 0, 0, 1, 0, 0, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 0, 1, 0, 0, 0);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 9:
		mux_addr(0, 0, 0, 1, 0, 0, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 0, 1, 0, 0, 1);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 10:
		mux_addr(0, 0, 0, 1, 0, 1, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 0, 1, 0, 1, 0);\
		measure_process(out);
		HAL_Delay(100);

		break;

	case 11:
		mux_addr(0, 0, 0, 1, 0, 1, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 0, 1, 0, 1, 1);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 12:
		mux_addr(0, 0, 0, 1, 1, 0, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 0, 1, 1, 0, 0);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 13:
		mux_addr(0, 0, 0, 1, 1, 0, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 0, 1, 1, 0, 1);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 14:
		mux_addr(0, 0, 0, 1, 1, 1, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 0, 1, 1, 1, 0);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 15:
		mux_addr(0, 0, 0, 1, 1, 1, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 0, 1, 1, 1, 1);
		measure_process(out);
		HAL_Delay(100);

		break;


/****************************************************************/

	case 16:
		mux_addr(0, 0, 1, 0, 0, 0, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 0, 0, 0, 0);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 17:
		mux_addr(0, 0, 1, 0, 0, 0, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 0, 0, 0, 1);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 18:
		mux_addr(0, 0, 1, 0, 0, 1, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 0, 0, 1, 0);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 19:
		mux_addr(0, 0, 1, 0, 0, 1, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 0, 0, 1, 1);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 20:
		mux_addr(0, 0, 1, 0, 1, 0, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 0, 1, 0, 0);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 21:
		mux_addr(0, 0, 1, 0, 1, 0, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 0, 1, 0, 1);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 22:
		mux_addr(0, 0, 1, 0, 1, 1, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 0, 1, 1, 0);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 23:
		mux_addr(0, 0, 1, 0, 1, 1, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 0, 1, 1, 1);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 24:
		mux_addr(0, 0, 1, 1, 0, 0, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 1, 0, 0, 0);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 25:
		mux_addr(0, 0, 1, 1, 0, 0, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 1, 0, 0, 1);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 26:
		mux_addr(0, 0, 1, 1, 0, 1, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 1, 0, 1, 0);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 27:
		mux_addr(0, 0, 1, 1, 0, 1, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 1, 0, 1, 1);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 28:
		mux_addr(0, 0, 1, 1, 1, 0, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 1, 1, 0, 0);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 29:
		mux_addr(0, 0, 1, 1, 1, 0, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 1, 1, 0, 1);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 30:
		mux_addr(0, 0, 1, 1, 1, 1, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 1, 1, 1, 0);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 31:
		mux_addr(0, 0, 1, 1, 1, 1, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 0, 1, 1, 1, 1, 1);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 32:
		mux_addr(0, 1, 0, 0, 0, 0, 0);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 1, 0, 0, 0, 0, 0);
		measure_process(out);
		HAL_Delay(100);

		break;

	case 33:
		mux_addr(0, 1, 0, 0, 0, 0, 1);
		led_flag = 1;
		measure_process(out);
		HAL_Delay(100);
		led_flag = 0;
		mux_addr(1, 1, 0, 0, 0, 0, 1);
		measure_process(out);
		HAL_Delay(100);
//		ws2812b_buff_claer();
//		led_num1 =1;
//		led_num2 = 32;
//		HAL_Delay(150);

		break;

	}
}

// measure function
void measure_process(int number)
{
	for (int num = 0; num < 6; num++) {
		measure_func1();
	}
	HAL_Delay(500);
	int led = number - 1;

	if (led_flag == 1) {
		if (led == 0) {
			//led_num1 = 0;
			LED = 0;
		} else if (led % 2 == 0) {
			LED = led_num1++;
		} else if (led % 2 > 0) {
			LED = led_num2--;
		}
	}
//	if (led == 0) {
//		//led_num1 = 0;
//		LED = 0;
//	} else if (led % 2 == 0) {
//		LED = led_num1++;
//	} else if (led % 2 > 0) {
//		LED = led_num2--;
//	}

	if (falling > 90 && falling < 100) {

		ws2812b_set(LED, 0, 25, 0);
		ws2812b_send();

	} else {

		ws2812b_set(LED, 25, 0, 0);
		ws2812b_send();

	}
	falling = 0;
}

void next_meas(void) {

	out = (out + 1) % 34;
	compare_out();

}

// принимает 0 или 1 нужно учитывать инверсию бита. Для установки 1 нужно записать 0, для 0 - 1(наоборот)
void mux_addr(uint8_t b2,uint8_t b3,uint8_t b4,uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4) {

	if (a1 == 0) {
		GPIOA->BSRR |= GPIO_BSRR_BR4;
	} else {
		GPIOA->BSRR |= GPIO_BSRR_BS4;
	}

	if (a2 == 0) {
		GPIOA->BSRR |= GPIO_BSRR_BR5;
	} else {
		GPIOA->BSRR |= GPIO_BSRR_BS5;
	}

	if (a3 == 0) {
		GPIOA->BSRR |= GPIO_BSRR_BR1;
	} else {
		GPIOA->BSRR |= GPIO_BSRR_BS1;
	}

	if (a4 == 0) {
		GPIOA->BSRR |= GPIO_BSRR_BR3;
	} else {
		GPIOA->BSRR |= GPIO_BSRR_BS3;
	}

	if (b4 == 0) {
		GPIOB->BSRR |= GPIO_BSRR_BR1;
	} else {
		GPIOB->BSRR |= GPIO_BSRR_BS1;
	}

	if (b3 == 0) {
		GPIOB->BSRR |= GPIO_BSRR_BR0;
	} else {
		GPIOB->BSRR |= GPIO_BSRR_BS0;
	}

	if (b2 == 0) {
		GPIOC->BSRR |= GPIO_BSRR_BR5;
	} else {
		GPIOC->BSRR |= GPIO_BSRR_BS5;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
