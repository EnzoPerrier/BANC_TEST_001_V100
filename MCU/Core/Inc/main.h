/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define OUT8_Pin GPIO_PIN_13
#define OUT8_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_14
#define BUZZER_GPIO_Port GPIOC
#define DIP1_Pin GPIO_PIN_15
#define DIP1_GPIO_Port GPIOC
#define DIP2_Pin GPIO_PIN_0
#define DIP2_GPIO_Port GPIOA
#define DIP3_Pin GPIO_PIN_1
#define DIP3_GPIO_Port GPIOA
#define Tx485_Pin GPIO_PIN_2
#define Tx485_GPIO_Port GPIOA
#define Rx485_Pin GPIO_PIN_3
#define Rx485_GPIO_Port GPIOA
#define RTS_485_Pin GPIO_PIN_4
#define RTS_485_GPIO_Port GPIOA
#define BP2_Pin GPIO_PIN_5
#define BP2_GPIO_Port GPIOA
#define BP3_Pin GPIO_PIN_6
#define BP3_GPIO_Port GPIOA
#define BP4_Pin GPIO_PIN_7
#define BP4_GPIO_Port GPIOA
#define V_CEL_Pin GPIO_PIN_0
#define V_CEL_GPIO_Port GPIOB
#define LEDR_Pin GPIO_PIN_1
#define LEDR_GPIO_Port GPIOB
#define LEDG_Pin GPIO_PIN_2
#define LEDG_GPIO_Port GPIOB
#define TX_COM_Pin GPIO_PIN_10
#define TX_COM_GPIO_Port GPIOB
#define RX_COM_Pin GPIO_PIN_11
#define RX_COM_GPIO_Port GPIOB
#define BP1_IRQ_Pin GPIO_PIN_12
#define BP1_IRQ_GPIO_Port GPIOB
#define LEDY_Pin GPIO_PIN_13
#define LEDY_GPIO_Port GPIOB
#define RELAIS_ALIM_418_Pin GPIO_PIN_14
#define RELAIS_ALIM_418_GPIO_Port GPIOB
#define OUT1_Pin GPIO_PIN_15
#define OUT1_GPIO_Port GPIOB
#define Tx232_Pin GPIO_PIN_9
#define Tx232_GPIO_Port GPIOA
#define Rx232_Pin GPIO_PIN_10
#define Rx232_GPIO_Port GPIOA
#define LED_CEL_Pin GPIO_PIN_11
#define LED_CEL_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define OUT2_Pin GPIO_PIN_15
#define OUT2_GPIO_Port GPIOA
#define OUT3_Pin GPIO_PIN_3
#define OUT3_GPIO_Port GPIOB
#define OUT4_Pin GPIO_PIN_4
#define OUT4_GPIO_Port GPIOB
#define OUT5_Pin GPIO_PIN_5
#define OUT5_GPIO_Port GPIOB
#define OUT6_Pin GPIO_PIN_6
#define OUT6_GPIO_Port GPIOB
#define OUT7_Pin GPIO_PIN_7
#define OUT7_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_8
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_9
#define I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
