/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define TS4_CONN_Pin GPIO_PIN_13
#define TS4_CONN_GPIO_Port GPIOC
#define TS5_CONN_Pin GPIO_PIN_14
#define TS5_CONN_GPIO_Port GPIOC
#define TS6_CONN_Pin GPIO_PIN_15
#define TS6_CONN_GPIO_Port GPIOC
#define DBG_TX_Pin GPIO_PIN_2
#define DBG_TX_GPIO_Port GPIOA
#define DBG_RX_Pin GPIO_PIN_3
#define DBG_RX_GPIO_Port GPIOA
#define TS1_Pin GPIO_PIN_4
#define TS1_GPIO_Port GPIOA
#define TS2_Pin GPIO_PIN_5
#define TS2_GPIO_Port GPIOA
#define TS3_Pin GPIO_PIN_6
#define TS3_GPIO_Port GPIOA
#define GATE1_Pin GPIO_PIN_1
#define GATE1_GPIO_Port GPIOD
#define GATE2_Pin GPIO_PIN_2
#define GATE2_GPIO_Port GPIOD
#define GATE3_Pin GPIO_PIN_3
#define GATE3_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_3
#define LED_R_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_4
#define LED_G_GPIO_Port GPIOB
#define RS_DIR_Pin GPIO_PIN_5
#define RS_DIR_GPIO_Port GPIOB
#define RS_TX_Pin GPIO_PIN_6
#define RS_TX_GPIO_Port GPIOB
#define RS_RX_Pin GPIO_PIN_7
#define RS_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
