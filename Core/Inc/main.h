/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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
#define WR_Pin GPIO_PIN_2
#define WR_GPIO_Port GPIOE
#define D0_Pin GPIO_PIN_3
#define D0_GPIO_Port GPIOE
#define D1_Pin GPIO_PIN_4
#define D1_GPIO_Port GPIOE
#define A0_Pin GPIO_PIN_5
#define A0_GPIO_Port GPIOE
#define A1_Pin GPIO_PIN_6
#define A1_GPIO_Port GPIOE
#define A2_Pin GPIO_PIN_13
#define A2_GPIO_Port GPIOC
#define A3_Pin GPIO_PIN_0
#define A3_GPIO_Port GPIOC
#define LE_Pin GPIO_PIN_1
#define LE_GPIO_Port GPIOC
#define CLK_262_Pin GPIO_PIN_5
#define CLK_262_GPIO_Port GPIOA
#define Relay_CH1_Pin GPIO_PIN_4
#define Relay_CH1_GPIO_Port GPIOC
#define Relay_CH3_Pin GPIO_PIN_0
#define Relay_CH3_GPIO_Port GPIOB
#define Relay_CH2_Pin GPIO_PIN_1
#define Relay_CH2_GPIO_Port GPIOB
#define TM1637_DIO_Pin GPIO_PIN_14
#define TM1637_DIO_GPIO_Port GPIOD
#define TM1637_CLK_Pin GPIO_PIN_15
#define TM1637_CLK_GPIO_Port GPIOD
#define SDIO3_Pin GPIO_PIN_3
#define SDIO3_GPIO_Port GPIOD
#define SDIO2_Pin GPIO_PIN_4
#define SDIO2_GPIO_Port GPIOD
#define SCLK_Pin GPIO_PIN_5
#define SCLK_GPIO_Port GPIOD
#define SDIO1_Pin GPIO_PIN_6
#define SDIO1_GPIO_Port GPIOD
#define CS_Pin GPIO_PIN_7
#define CS_GPIO_Port GPIOD
#define SDIO0_Pin GPIO_PIN_3
#define SDIO0_GPIO_Port GPIOB
#define UPDATE_Pin GPIO_PIN_4
#define UPDATE_GPIO_Port GPIOB
#define PS3_Pin GPIO_PIN_5
#define PS3_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_6
#define RST_GPIO_Port GPIOB
#define PS2_Pin GPIO_PIN_7
#define PS2_GPIO_Port GPIOB
#define PDC_Pin GPIO_PIN_8
#define PDC_GPIO_Port GPIOB
#define PS1_Pin GPIO_PIN_9
#define PS1_GPIO_Port GPIOB
#define PS0_Pin GPIO_PIN_1
#define PS0_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
