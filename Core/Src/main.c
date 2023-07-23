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
#include <rs485.h>
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

/* Flags */
bool modbus_request_pending_flag = false;

/* Global variables */
uint32_t gate_pulse_delay_counter_us = 0;
uint32_t log_counter_us = 0;
uint32_t rx_time_interval_counter = 0;
uint8_t received_modbus_frame[RS_RX_BUFFER_SIZE];
uint16_t modbus_frame_byte_counter = 0;
uint8_t uart1_rx_byte = 0;
uint8_t uart2_rx_byte = 0;

static channel_t channel_array[OUTPUT_CHANNELS_NUMBER] = {
	{TRIG1_Pin, TRIG1_GPIO_Port,  INIT_VOLTAGE, 0, GATE_IDLE},
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

void log_working_parameters(void);
void update_app_data(void);
void reset_zero_crossing_counter(void);
bool logger_transmit_byte(uint8_t * byte);

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
  rs485_init(&huart2);
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
  while (1)
  {

	if (modbus_request_pending_flag == true)
	{
		uint8_t response_buffer[RS_TX_BUFFER_SIZE];
		uint16_t response_size;
		if (modbus_process_frame(received_modbus_frame, modbus_frame_byte_counter, response_buffer, &response_size))
		{
			rs485_transmit_byte_array(response_buffer, response_size);
		}
		else
		{
			logger_log(LEVEL_ERROR, "Can't process Modbus frame\r\n");
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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ZERO_CROSSING_GPIO_Port, &GPIO_InitStruct);

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
		rx_time_interval_counter += MAIN_TIMER_RESOLUTION_US;

		if ( (log_counter_us > LOGGING_PERIOD_US))
		{
			log_working_parameters();
			log_counter_us = 0;
		}
	}

	if ( (rx_time_interval_counter > MAX_TIME_BETWEEN_FRAMES_US) && (!rs485_rx_buffer_empty()) )
	{
		rs485_get_frame(received_modbus_frame, RS_RX_BUFFER_SIZE);
		modbus_request_pending_flag = true;
	}
}


/* UART RX finished callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		// Modbus UART received byte
		logger_log(LEVEL_DEBUG, "UART1 received 0x%02x\r\n", uart1_rx_byte);

		if (modbus_request_pending_flag == true)
		{
			return;
		}
		else
		{
			rx_time_interval_counter = 0;

			if (rs485_collect_byte_to_buffer(&uart1_rx_byte) && (modbus_frame_byte_counter < RS_RX_BUFFER_SIZE))
			{
				modbus_frame_byte_counter++;
			}
			else
			{
				logger_log(LEVEL_ERROR, "Cannot get byte to buffer (buffer full)\r\n");
				modbus_frame_byte_counter = 0;
			}
		}

		// Prepare for next byte receiving
		HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart2, &uart1_rx_byte, 1);
		if (status != HAL_OK)
		{
			logger_log(LEVEL_ERROR, "Cannot start huart2 receiving\r\n");
		}
	}

	if (huart->Instance == USART2)
	{
		// debug UART received byte
		logger_log(LEVEL_DEBUG, "UART2 received 0x%02x\r\n", uart2_rx_byte);
		logger_set_level((uint8_t)uart2_rx_byte-'0');

		// Prepare for next byte receiving
		HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart1, &uart2_rx_byte, 1);
		if (status != HAL_OK)
		{
			logger_log(LEVEL_ERROR, "Cannot start huart1 receiving\r\n");
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
		logger_transmit_complete();
	}
}


void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  reset_zero_crossing_counter();
}


void log_working_parameters()
{
	logger_log(LEVEL_INFO, "FAN CH | VOLTAGE | DELAY_US  |\r\n");
	logger_log(LEVEL_INFO, "-------|---------|-----------|\r\n");
	for (int i = 0; i < OUTPUT_CHANNELS_NUMBER; i++)
	{
		logger_log(LEVEL_INFO, "   %01d   |   %03d   |    %d   |\r\n", i+1, channel_array[i].output_voltage_decpercent/10, channel_array[i].activation_delay_us);
	}
}


void update_app_data(void)
{
	logger_log(LEVEL_INFO, "Updating app data from registers\r\n");
	channel_array[0].output_voltage_decpercent = modbus_get_reg_value(0)*VOLTAGE_PRECISION_MULTIPLIER;
	channel_array[1].output_voltage_decpercent = modbus_get_reg_value(1)*VOLTAGE_PRECISION_MULTIPLIER;
	channel_array[2].output_voltage_decpercent = modbus_get_reg_value(2)*VOLTAGE_PRECISION_MULTIPLIER;
}

void reset_zero_crossing_counter(void)
{
	if (gate_pulse_delay_counter_us > HALF_SINE_PERIOD_US - 500)
	{
		gate_pulse_delay_counter_us = ZERO_CROSSING_DETECTION_OFFSET_US;
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
