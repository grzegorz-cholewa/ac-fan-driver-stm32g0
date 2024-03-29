/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <config.h>
#include <modbus.h>
#include <gate_driver.h>
#include <logger.h>

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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Global variables */
static uint32_t gate_pulse_delay_counter_us = 0; // software timer used for driving gates
static uint32_t zero_crossing_watchdog_counter_us = 0; // software timer counter for periodic zero crossing presence check
static uint32_t zero_crossing_events_counter = 0; // counts zero crossing events for periodic interrupt presence check
static uint32_t log_counter_us = 0; // software timer counter for periodic serial log
static uint32_t modbus_rx_time_interval_counter = 0; // software timer for Modbus frame receive end
static uint8_t modbus_buffer[MODBUS_RX_BUFFER_SIZE]; // Modbus buffer for incoming frame
static uint8_t * modbus_buffer_write_pointer = modbus_buffer; // write pointer for getting bytes to Modbus RX buffer
static bool modbus_request_pending_flag = false; // Flag indicating Modbus frame was collected and is ready for processing
static uint8_t uart1_rx_byte = 0;
static uint8_t uart2_rx_byte = 0;

static channel_t channel_array[OUTPUT_CHANNELS_NUMBER] = {
	{TRIG1_Pin, TRIG1_GPIO_Port, INIT_VOLTAGE, 0, GATE_IDLE},
	{TRIG2_Pin, TRIG2_GPIO_Port, INIT_VOLTAGE, 0, GATE_IDLE},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void log_working_params(void);
void update_working_params(void);
bool logger_transmit_byte(uint8_t * byte);
bool is_modbus_buffer_empty(void);
bool is_modbus_buffer_full(void);

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
  HAL_StatusTypeDef status;

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  logger_init(&logger_transmit_byte);
  logger_set_level(LEVEL_INFO);
  status = HAL_TIM_Base_Start_IT(&htim3);
  if (status != HAL_OK)
    {
  	  logger_log(LEVEL_ERROR, "Cannot start timer\r\n");
    }
    status = HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    if (status != HAL_OK)
    {
  	  logger_log(LEVEL_ERROR, "Cannot start UART1 receiving\r\n");
    }
   status = HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
   if (status != HAL_OK)
   {
 	  logger_log(LEVEL_ERROR, "Cannot start UART2 receiving\r\n");
   }

    logger_log(LEVEL_INFO, "App init done\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  update_working_params();
  while (1)
  {
	if (modbus_request_pending_flag == true)
	{
		uint8_t response_buffer[MODBUS_TX_BUFFER_SIZE];
		uint16_t response_size;
		int modbus_frame_len = modbus_buffer_write_pointer - modbus_buffer;
		if (modbus_process_frame(modbus_buffer, modbus_frame_len, response_buffer, &response_size))
		{
			HAL_GPIO_WritePin(RS_DIR_GPIO_Port, RS_DIR_Pin, GPIO_PIN_SET);
			HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, response_buffer, response_size, 100);
			if (status != HAL_OK)
			{
				logger_log(LEVEL_ERROR, "Modbus cannot send response");
			}
			HAL_GPIO_WritePin(RS_DIR_GPIO_Port, RS_DIR_Pin, GPIO_PIN_RESET);
		}
		else
		{
			logger_log(LEVEL_ERROR, "Can't process Modbus frame\r\n");
		}

		update_working_params();
		modbus_request_pending_flag = false;
		modbus_buffer_write_pointer = modbus_buffer;
		memset(modbus_buffer, 0, sizeof(modbus_buffer)); // clear buffer
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
  huart1.Init.BaudRate = 9600;
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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG1_Pin|TRIG2_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS_DIR_GPIO_Port, RS_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ZERO_CROSSING_Pin */
  GPIO_InitStruct.Pin = ZERO_CROSSING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ZERO_CROSSING_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CFG1_Pin CFG2_Pin CFG3_Pin */
  GPIO_InitStruct.Pin = CFG1_Pin|CFG2_Pin|CFG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG1_Pin TRIG2_Pin LED_Pin */
  GPIO_InitStruct.Pin = TRIG1_Pin|TRIG2_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RS_DIR_Pin */
  GPIO_InitStruct.Pin = RS_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS_DIR_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Timer 1 overflow interrupt callback */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		drive_fans(channel_array, OUTPUT_CHANNELS_NUMBER, gate_pulse_delay_counter_us);

		gate_pulse_delay_counter_us += MAIN_TIMER_RESOLUTION_US;
		log_counter_us += MAIN_TIMER_RESOLUTION_US;
		modbus_rx_time_interval_counter += MAIN_TIMER_RESOLUTION_US;
		zero_crossing_watchdog_counter_us += MAIN_TIMER_RESOLUTION_US;

		if ((log_counter_us > LOGGING_PERIOD_US))
		{
			log_working_params();
			log_counter_us = 0;
		}

		if (zero_crossing_watchdog_counter_us >= ZERO_CROSSING_CHECK_PERIOD_US)
		{
			// no zero crossing events since last check, set error
			if (zero_crossing_events_counter == 0)
			{
				modbus_set_reg_value(2, 1);
			}
			// detected any zero crossing events since last check, clear error
			else
			{
				modbus_set_reg_value(2, 0);
			}
			zero_crossing_watchdog_counter_us = 0;
			zero_crossing_events_counter = 0;
		}
	}

	if ( (modbus_rx_time_interval_counter > MAX_TIME_BETWEEN_FRAMES_US) && (!is_modbus_buffer_empty()) )
	{
		modbus_request_pending_flag = true;
	}
}


/* UART RX finished callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Modbus UART received byte
	if (huart->Instance == USART1)
	{
		logger_log(LEVEL_DEBUG, "UART1 (Modbus) received 0x%02x\r\n", uart1_rx_byte);

		if (modbus_request_pending_flag == true) // don't store new bytes while previous request is processed
		{
			return;
		}
		else
		{
			modbus_rx_time_interval_counter = 0;

			if (!is_modbus_buffer_full())
			{
				*modbus_buffer_write_pointer = uart1_rx_byte;
				modbus_buffer_write_pointer++;
			}
			else
			{
				logger_log(LEVEL_ERROR, "Cannot get byte to buffer (buffer full)\r\n");
				modbus_buffer_write_pointer =  modbus_buffer;
			}
		}

		// Prepare for next byte receiving
		HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
		if (status != HAL_OK)
		{
			logger_log(LEVEL_ERROR, "Cannot start UART1 receiving\r\n");
		}
	}

	// Serial debug UART received byte
	if (huart->Instance == USART2)
	{
		logger_log(LEVEL_DEBUG, "UART2 (debug) received 0x%02x\r\n", uart2_rx_byte);
		logger_set_level((uint8_t)uart2_rx_byte-'0');

		// Prepare for next byte receiving
		HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
		if (status != HAL_OK)
		{
			logger_log(LEVEL_ERROR, "Cannot start UART2 receiving\r\n");
		}
	}
}


/* UART TX finished callback */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		// Modbus UART transmit complete
	}
	if (huart->Instance == USART2)
	{
		// Debug UART transmit complete
		logger_transmit_next_byte();
	}
}


void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  // reset zero crossing counter
  if (gate_pulse_delay_counter_us > HALF_SINE_PERIOD_US - 500) // additional check to avoid some random reset
  {
	gate_pulse_delay_counter_us = ZERO_CROSSING_DETECTION_OFFSET_US;
  }

  zero_crossing_events_counter++;

}


void log_working_params()
{
	logger_log(LEVEL_INFO, "FAN CH | VOLTAGE | DELAY_US |\r\n");
	logger_log(LEVEL_INFO, "-------|---------|----------|\r\n");
	for (int i = 0; i < OUTPUT_CHANNELS_NUMBER; i++)
	{
		logger_log(LEVEL_INFO, "   %01d   |   %03d   |   %04d   |\r\n", i+1, channel_array[i].output_voltage_decpercent/10, channel_array[i].activation_delay_us);
	}
}


void update_working_params()
{
	for (uint8_t channel = 0; channel < OUTPUT_CHANNELS_NUMBER; channel++)
	{
		// get voltage from Modbus registers
		channel_array[channel].output_voltage_decpercent = modbus_get_reg_value(channel)*VOLTAGE_PRECISION_MULTIPLIER;

		// calculate gate delay
		if ( (channel_array[channel].output_voltage_decpercent <= MIN_OUTPUT_VOLTAGE_DECPERCENT) ||
			 (channel_array[channel].output_voltage_decpercent >= MAX_OUTPUT_VOLTAGE_DECPERCENT) )
		{
			// set '0' to indicate constant gate state
			channel_array[channel].activation_delay_us = 0;
		}
		else
		{
            channel_array[channel].activation_delay_us = get_gate_delay_us(channel_array[channel].output_voltage_decpercent);
		}
	}
}


bool logger_transmit_byte(uint8_t * byte)
{
	HAL_StatusTypeDef retVal = HAL_UART_Transmit_IT(&huart2, (uint8_t*)byte, 1);
	if (retVal == HAL_OK)
	{
		return true;
	}
	else
	{
		return false;
	}
}


bool is_modbus_buffer_empty(void)
{
	if (modbus_buffer_write_pointer == modbus_buffer)
		return true;
	else
		return false;
}


bool is_modbus_buffer_full(void)
{
	if (modbus_buffer_write_pointer <= modbus_buffer + MODBUS_RX_BUFFER_SIZE)
		return false;
	else
		return true;
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
