/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  USART1 is for debug logging (115200bits/s)
  USART2 is for Modbus communication (9600bits/s)
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <config.h>
#include <rs485.h>
#include <modbus.h>
#include <gate_driver.h>
#include <logger.h>
#include <sensor.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define WORK_STATE_MANUAL 0
#define WORK_STATE_AUTO 1

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Flags */
bool update_working_parameters_pending_flag = false;
bool modbus_request_pending_flag = false;

/* Global variables */
uint32_t gate_pulse_delay_counter_us = 0;
uint32_t update_parameter_counter_us = 0;
uint32_t rx_time_interval_counter = 0;
sensors_t sensors[TOTAL_SENSOR_NUMBER];
uint16_t dma_adc_array[ISOLATED_SENSOR_NUMBER];
int16_t temperature_error_state = 0;
uint8_t received_modbus_frame[RS_RX_BUFFER_SIZE];
uint16_t modbus_frame_byte_counter = 0;

static channel_t channel_array[OUTPUT_CHANNELS_NUMBER] = {
	{GATE1_Pin, 0, WORK_STATE_AUTO, INIT_CHANNEL_SETPOINT_C, 0, 0, GATE_IDLE},
	{GATE2_Pin, 1, WORK_STATE_AUTO, INIT_CHANNEL_SETPOINT_C, 0, 0, GATE_IDLE},
	{GATE3_Pin, 2, WORK_STATE_AUTO, INIT_CHANNEL_SETPOINT_C, 0, 0, GATE_IDLE}
};
uint8_t uart_rx_byte = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
int16_t pi_regulator(uint8_t channel, int16_t current_temp, int16_t target_temperature);
void update_working_parameters(void);
void update_modbus_registers(void);
void update_app_data(void);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  logger_init(&huart1, LEVEL_INFO);

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_adc_array, NON_ISOLATED_SENSOR_NUMBER);
  rs485_init(&huart2);
  update_working_parameters();
  HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1); // start Modbus communication
  if (status != HAL_OK)
  {
	  logger_log(LEVEL_ERROR, "ERR: cannot start HAL_UART_Transmit_IT\r\n");
  }

  logger_log(LEVEL_INFO, "INF: Init done. App is running\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1) // test loop
//  {
//	  HAL_Delay (1000);
//	  HAL_GPIO_TogglePin (GPIOB, LED_R_Pin);
//	  log_text(LEVEL_INFO, "Test loop running\r\n");
//
//	  log_text(LEVEL_INFO, "ABCDEF\r\n");
//	  log_text(LEVEL_INFO, "12345\r\n");
//
////	  char string_buffer1[] = "ABCDEF\r\n";
////	  char string_buffer2[] = "12345\r\n";
////	  HAL_UART_Transmit(&huart1, (uint8_t*)string_buffer1, strlen(string_buffer1), 1000);
////	  HAL_UART_Transmit(&huart1, (uint8_t*)string_buffer2, strlen(string_buffer2), 1000);
//
////	  int8_t buffer[] = "12345ABCDEF";
////	  rs485_transmit_byte_array(buffer, 10);
//  }
  while (1)
  {
//	logger_transmit_byte();

	// Modbus request can be waiting if update_working_parameters() is running. Could it be a problem?
	// Probably no, as a measured execution time of update_working_parameters() execution is around 1ms.
	if (update_working_parameters_pending_flag == true)
	{
		update_working_parameters();
//		log_text(LEVEL_DEBUG, "DGB: update_working_parameters() execution time in us: %d\r\n", update_parameter_counter_us);
	}

	if (modbus_request_pending_flag == true)
	{
		uint8_t response_buffer[RS_TX_BUFFER_SIZE];
		uint16_t response_size;
		update_modbus_registers(); // update Modbus registers with data from app
		if (modbus_process_frame(received_modbus_frame, modbus_frame_byte_counter, response_buffer, &response_size))
		{
			rs485_transmit_byte_array(response_buffer, response_size);
		}
		else
		{
			logger_log(LEVEL_ERROR, "ERR: cannot process Modbus frame\r\n");
		}
		update_app_data(); // update app with new data from processed Modbus frame (needed if it was write command)

		modbus_request_pending_flag = false;
		modbus_frame_byte_counter = 0;
		memset(received_modbus_frame, 0, sizeof(received_modbus_frame)); // clear buffer
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 6400;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS_DIR_GPIO_Port, RS_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI_EN_Pin|LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GATE1_Pin|GATE2_Pin|GATE3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TS4_EN_Pin TS5_EN_Pin TS6_EN_Pin */
  GPIO_InitStruct.Pin = TS4_EN_Pin|TS5_EN_Pin|TS6_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RS_DIR_Pin */
  GPIO_InitStruct.Pin = RS_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_EN_Pin LED_R_Pin */
  GPIO_InitStruct.Pin = SPI_EN_Pin|LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ZERO_CROSSING_Pin */
  GPIO_InitStruct.Pin = ZERO_CROSSING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ZERO_CROSSING_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GATE1_Pin GATE2_Pin GATE3_Pin */
  GPIO_InitStruct.Pin = GATE1_Pin|GATE2_Pin|GATE3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/* Timer 1 overflow interrupt callback */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		drive_fans(channel_array, OUTPUT_CHANNELS_NUMBER, gate_pulse_delay_counter_us);

		gate_pulse_delay_counter_us += MAIN_TIMER_RESOLUTION_US;
		update_parameter_counter_us += MAIN_TIMER_RESOLUTION_US;
		rx_time_interval_counter += MAIN_TIMER_RESOLUTION_US;

		if(update_parameter_counter_us >= WORKING_PARAMETERS_UPDATE_PERIOD_US)
		{
			update_working_parameters_pending_flag = true;
			update_parameter_counter_us = 0;
		}
	}

	if ( (rx_time_interval_counter > MAX_TIME_BETWEEN_FRAMES_US) && (!rs485_rx_buffer_empty()) )
	{
		rs485_get_frame(received_modbus_frame, RS_RX_BUFFER_SIZE);
		modbus_request_pending_flag = true;
	}
}

/* ADC conversion finished callback */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
//	 measured ADC read time is about 300us
	logger_log(LEVEL_DEBUG, "DBG: HAL_ADC_ConvCpltCallback\r\n");
}

/* UART RX finished callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	logger_log(LEVEL_DEBUG, "DBG: Serial received a byte: %02x\r\n", uart_rx_byte);

	if (modbus_request_pending_flag == true)
	{
		return;
	}
	else
	{
		rx_time_interval_counter = 0;

		if (rs485_collect_byte_to_buffer(&uart_rx_byte) && (modbus_frame_byte_counter < RS_RX_BUFFER_SIZE))
		{
			modbus_frame_byte_counter++;
		}
		else
		{
			logger_log(LEVEL_ERROR, "ERR: cannot get byte to buffer (buffer full)\r\n");
			modbus_frame_byte_counter = 0;
		}
	}

	// Prepare for next byte receiving
	HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
	if (status != HAL_OK)
	{
		logger_log(LEVEL_ERROR, "ERR, cannot start HAL_UART_Transmit_IT\r\n");
	}
}

/* UART TX finished callback */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		// debug uart transmit complete
		logger_transmit_complete();
//		HAL_UART_Transmit_IT(&huart1, str, 6);
	}
	if (huart->Instance == USART2)
	{
		// rs485 uart transmit complete
	}
}

void update_working_parameters()
{
	logger_log(LEVEL_INFO, "INF: Updating working parameters\r\n");
	HAL_GPIO_TogglePin(GPIOB, LED_R_Pin);

	// Update sensor connected status from user buttons
	sensors[0].connected_status = true; // always connected internally
	sensors[1].connected_status = true; // always connected internally
	sensors[2].connected_status = true; // always connected internally
//	sensors[3].connected_status = (bool)HAL_GPIO_ReadPin(GPIOD, TS4_sensor_connected_Pin);
//	sensors[4].connected_status = (bool)HAL_GPIO_ReadPin(GPIOD, TS5_sensor_connected_Pin);
//	sensors[5].connected_status = (bool)HAL_GPIO_ReadPin(GPIOD, TS6_sensor_connected_Pin);
	sensors[3].connected_status = true; // temporary disable that
	sensors[4].connected_status = true;
	sensors[5].connected_status = true;

	// get real ADC values to sensor_values array (for non-isolated sensors)
	for (int i = 0; i < NON_ISOLATED_SENSOR_NUMBER; i++)
	{
		sensors[i].adc_value = dma_adc_array[i];
	}
	// get real ADC values to sensor_values array (for isolated sensors)
	sensors[3].adc_value = 0; // TODO get real value from SPI ADC
	sensors[4].adc_value = 0; // TODO get real value from SPI ADC
	sensors[5].adc_value = 0; // TODO get real value from SPI ADC

	// calculate temperatures from ADC values
	sensors[0].temperature = ntc_to_temperature(sensors[0].adc_value);
	sensors[1].temperature = ntc_to_temperature(sensors[1].adc_value);
	sensors[2].temperature = ntc_to_temperature(sensors[2].adc_value);
	sensors[3].temperature = ntc_to_temperature(sensors[3].adc_value);
	sensors[4].temperature = ntc_to_temperature(sensors[4].adc_value);
	sensors[5].temperature = pt100_to_temperature(sensors[5].adc_value);
	temperature_error_state = check_for_error(sensors);

	// log sensor data
	logger_log(LEVEL_INFO, "SEN CH | ADC  | TEMP | USED | ERR |\r\n-------|------|------|------|-----|\r\n");
	for (int channel = 0; channel < TOTAL_SENSOR_NUMBER; channel++)
	{
	  logger_log(LEVEL_INFO, "     %d | %04d | %04d |  %d   |  %d  |\r\n", channel+1, sensors[channel].adc_value, sensors[channel].temperature, (int)(sensors[channel].connected_status), sensors[channel].error);
	}

	// run PI regulator calculations
	for (uint8_t i = 0; i < OUTPUT_CHANNELS_NUMBER; i++)
	{
		if (channel_array[i].work_state == WORK_STATE_AUTO)
		{
			channel_array[i].output_voltage_decpercent = pi_regulator(i, sensors[i].temperature, channel_array[i].setpoint);
		}
		channel_array[i].activation_delay_us = get_gate_delay_us(channel_array[i].output_voltage_decpercent);
	}

	// log fan channel data
	logger_log(LEVEL_INFO, "FAN CH | SETPOINT | VOLTAGE | DELAY_US |\r\n-------|----------|---------|----------|\r\n");
	for (int i = 0; i < OUTPUT_CHANNELS_NUMBER; i++)
	{
		logger_log(LEVEL_INFO, "   %d   |    %02d    |   %03d   |    %d   |\r\n", i+1, channel_array[i].setpoint/10, channel_array[i].output_voltage_decpercent/10, channel_array[i].activation_delay_us);
	}
	update_working_parameters_pending_flag = false;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_adc_array, NON_ISOLATED_SENSOR_NUMBER); // start ADC read for next update cycle
}


int16_t pi_regulator(uint8_t channel, int16_t current_temp, int16_t setpoint)
{
	int16_t error;
	static int16_t integral_error[3] = {0, 0, 0};
	int16_t output_voltage_decpercent;

	error = current_temp - setpoint;

	integral_error[channel] = integral_error[channel] + error;

	if ((error>0) && (integral_error[channel] <350*TIME_CONST))
		integral_error[channel] = 350*TIME_CONST;

	if (integral_error[channel] > INTEGRAL_ERROR_MAX)
		integral_error[channel] = INTEGRAL_ERROR_MAX;
	if (integral_error[channel] < INTEGRAL_ERROR_MIN)
		integral_error[channel] = INTEGRAL_ERROR_MIN;

	output_voltage_decpercent = PI_KP * error  + integral_error[channel]/TIME_CONST;

	if (output_voltage_decpercent > MAX_OUTPUT_VOLTAGE_DECPERCENT)
		output_voltage_decpercent = FULL_ON_OUTPUT_VOLTAGE_DECPERCENT;

	if (output_voltage_decpercent < MIN_OUTPUT_VOLTAGE_DECPERCENT)
		output_voltage_decpercent = FULL_OFF_OUTPUT_VOLTAGE_DECPERCENT;

	return output_voltage_decpercent;
};


void update_modbus_registers(void)
{
	logger_log(LEVEL_INFO, "INF: Updating Modbus registers with data from app before processing request\r\n");
	modbus_set_reg_value(0, channel_array[0].work_state);
	modbus_set_reg_value(1, channel_array[1].work_state);
	modbus_set_reg_value(2, channel_array[2].work_state);
	modbus_set_reg_value(3, channel_array[0].output_voltage_decpercent/VOLTAGE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(4, channel_array[1].output_voltage_decpercent/VOLTAGE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(5, channel_array[2].output_voltage_decpercent/VOLTAGE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(6, channel_array[0].setpoint/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(7, channel_array[1].setpoint/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(8, channel_array[2].setpoint/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(9, sensors[0].temperature/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(10, sensors[1].temperature/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(11, sensors[2].temperature/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(12, sensors[3].temperature/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(13, sensors[4].temperature/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(14, sensors[5].temperature/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(15, temperature_error_state);
}


void update_app_data(void)
{
	logger_log(LEVEL_INFO, "INF: Updating app data with data from Modbus registers\r\n");
	channel_array[0].work_state = modbus_get_reg_value(0);
	channel_array[1].work_state = modbus_get_reg_value(1);
	channel_array[2].work_state = modbus_get_reg_value(2);

	if ((modbus_get_reg_value(3)) != channel_array[0].output_voltage_decpercent/VOLTAGE_PRECISION_MULTIPLIER) // check if value changed
	{
		channel_array[0].work_state = WORK_STATE_MANUAL;
		channel_array[0].output_voltage_decpercent = modbus_get_reg_value(3)*VOLTAGE_PRECISION_MULTIPLIER;
	}

	if (modbus_get_reg_value(4) != channel_array[1].output_voltage_decpercent/VOLTAGE_PRECISION_MULTIPLIER) // check if value changed
	{
		channel_array[1].work_state = WORK_STATE_MANUAL;
		channel_array[1].output_voltage_decpercent = modbus_get_reg_value(4)*VOLTAGE_PRECISION_MULTIPLIER;
	}

	if (modbus_get_reg_value(5) != channel_array[2].output_voltage_decpercent/VOLTAGE_PRECISION_MULTIPLIER) // check if value changed
	{
		channel_array[2].work_state = WORK_STATE_MANUAL;
		channel_array[2].output_voltage_decpercent = modbus_get_reg_value(5)*VOLTAGE_PRECISION_MULTIPLIER;
	}

	if (modbus_get_reg_value(6) != (channel_array[0].setpoint/TEMPERATURE_PRECISION_MULTIPLIER)) // check if value changed
	{
		channel_array[0].work_state = WORK_STATE_AUTO;
		channel_array[0].setpoint = modbus_get_reg_value(6)*TEMPERATURE_PRECISION_MULTIPLIER;
	}

	if (modbus_get_reg_value(7) != (channel_array[1].setpoint/TEMPERATURE_PRECISION_MULTIPLIER)) // check if value changed
	{
		channel_array[1].work_state = WORK_STATE_AUTO;
		channel_array[1].setpoint = modbus_get_reg_value(7)*TEMPERATURE_PRECISION_MULTIPLIER;
	}

	if (modbus_get_reg_value(8) != (channel_array[2].setpoint/TEMPERATURE_PRECISION_MULTIPLIER)) // check if value changed
	{
		channel_array[2].work_state = WORK_STATE_AUTO;
		channel_array[2].setpoint = modbus_get_reg_value(8)*TEMPERATURE_PRECISION_MULTIPLIER;
	}
}

void reset_zero_crossing_counter(void)
{
	if (gate_pulse_delay_counter_us > HALF_SINE_PERIOD_US - 500)
	{
		gate_pulse_delay_counter_us = 0;
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
