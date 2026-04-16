/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dac.h
  * @brief   This file contains all the function prototypes for
  *          the dac.c file
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
#ifndef __DAC_H__
#define __DAC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern DAC_HandleTypeDef hdac;
/* USER CODE BEGIN Private defines */
/* DAC waveform configuration constants (shared) */
#define DAC_TRI_SAMPLES      120U
#define DAC_TRI_TARGET_HZ    50000U
#define DAC_TRI_VREF_MV      3300U
#define DAC_TRI_OFFSET_MV    1650U
#define DAC_TRI_PK_MV        1000U

/* USER CODE END Private defines */

void MX_DAC_Init(void);

/* USER CODE BEGIN Prototypes */
void DAC_StartSine(uint32_t freq_hz, uint32_t amplitude_pk_mv, uint32_t offset_mv, float phase_deg);
void DAC_StartTriangle50k_2V(void);
/* Update DAC frequency/amplitude at runtime */
void DAC_SetFrequencyAndPhase(uint32_t freq_hz, uint32_t amplitude_pk_mv, uint32_t offset_mv, float phase_deg);
void DAC_SetFrequencyAmplitude(uint32_t freq_hz, uint32_t amplitude_pk_mv, uint32_t offset_mv);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __DAC_H__ */

