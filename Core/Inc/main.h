/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ZERO_CROSSING_Pin GPIO_PIN_1
#define ZERO_CROSSING_GPIO_Port GPIOA
#define ZERO_CROSSING_EXTI_IRQn EXTI0_1_IRQn
#define DBG_TX_Pin GPIO_PIN_2
#define DBG_TX_GPIO_Port GPIOA
#define DBG_RX_Pin GPIO_PIN_3
#define DBG_RX_GPIO_Port GPIOA
#define CFG1_Pin GPIO_PIN_0
#define CFG1_GPIO_Port GPIOB
#define CFG2_Pin GPIO_PIN_1
#define CFG2_GPIO_Port GPIOB
#define CFG3_Pin GPIO_PIN_2
#define CFG3_GPIO_Port GPIOB
#define TRIG1_Pin GPIO_PIN_10
#define TRIG1_GPIO_Port GPIOA
#define TRIG2_Pin GPIO_PIN_11
#define TRIG2_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA
#define RS_TX_Pin GPIO_PIN_6
#define RS_TX_GPIO_Port GPIOB
#define RS_RX_Pin GPIO_PIN_7
#define RS_RX_GPIO_Port GPIOB
#define RS_DIR_Pin GPIO_PIN_8
#define RS_DIR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
