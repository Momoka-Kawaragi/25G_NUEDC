/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dac.c
  * @brief   This file provides code for the configuration
  *          of the DAC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "dac.h"
#include "tim.h"
#include <math.h>

/* USER CODE BEGIN 0 */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* Waveform buffer moved to file scope so it can be updated at runtime */
static uint16_t sine_buf[DAC_TRI_SAMPLES];

static uint32_t DAC_GetTim6ClockHz(void)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t ppre1 = RCC->CFGR & RCC_CFGR_PPRE1;
  if (ppre1 == RCC_CFGR_PPRE1_DIV1)
  {
    return pclk1;
  }
  return pclk1 * 2U;
}

void DAC_StartSine(uint32_t freq_hz, uint32_t amplitude_pk_mv, uint32_t offset_mv, float phase_deg)
{
  /* Delegate to runtime updater which fills buffer and configures DMA/timer */
  DAC_SetFrequencyAndPhase(freq_hz, amplitude_pk_mv, offset_mv, phase_deg);
}

void DAC_SetFrequencyAmplitude(uint32_t freq_hz, uint32_t amplitude_pk_mv, uint32_t offset_mv)
{
  DAC_SetFrequencyAndPhase(freq_hz, amplitude_pk_mv, offset_mv, 0.0f);
}

void DAC_SetFrequencyAndPhase(uint32_t freq_hz, uint32_t amplitude_pk_mv, uint32_t offset_mv, float phase_deg)
{
  uint32_t tim_clk;
  uint32_t sample_rate_hz;
  uint32_t period;
  uint32_t center_code;
  uint32_t delta_code;
  uint32_t min_code;
  uint32_t max_code;

  if (freq_hz == 0U)
  {
    return;
  }

  /* Fill waveform buffer (12-bit centered around offset) */
  center_code = (4095U * offset_mv) / DAC_TRI_VREF_MV;
  delta_code = (4095U * amplitude_pk_mv) / DAC_TRI_VREF_MV;
  min_code = (center_code > delta_code) ? (center_code - delta_code) : 0U;
  max_code = center_code + delta_code;
  if (max_code > 4095U)
  {
    max_code = 4095U;
  }

  float phase_rad = phase_deg * PI / 180.0f;
  for (uint32_t i = 0; i < DAC_TRI_SAMPLES; i++)
  {
    float theta = 2.0f * PI * ((float)i / (float)DAC_TRI_SAMPLES) + phase_rad;
    float s = sinf(theta);
    int32_t code = (int32_t)center_code + (int32_t)(s * (float)delta_code);
    if (code < (int32_t)min_code) code = (int32_t)min_code;
    if (code > (int32_t)max_code) code = (int32_t)max_code;
    sine_buf[i] = (uint16_t)code;
  }

  /* Configure timer and DMA */
  sample_rate_hz = freq_hz * DAC_TRI_SAMPLES;
  tim_clk = DAC_GetTim6ClockHz();
  if (sample_rate_hz == 0U)
  {
    return;
  }

  period = tim_clk / sample_rate_hz;
  if (period == 0U)
  {
    period = 1U;
  }

  htim6.Init.Prescaler = 0U;
  htim6.Init.Period = period - 1U;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  (void)HAL_TIM_Base_Stop(&htim6);
  (void)HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);

  if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)sine_buf, DAC_TRI_SAMPLES, DAC_ALIGN_12B_R) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
}

void DAC_StartTriangle50k_2V(void)
{
  /* Keep compatibility with existing calls: 50kHz, 1.65V +/-1.0V sine output. Default 0 deg phase. */
  DAC_StartSine(DAC_TRI_TARGET_HZ, DAC_TRI_PK_MV, DAC_TRI_OFFSET_MV, 0.0f);
}

/* USER CODE END 0 */

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

/* DAC init function */
void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

void HAL_DAC_MspInit(DAC_HandleTypeDef* dacHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(dacHandle->Instance==DAC)
  {
  /* USER CODE BEGIN DAC_MspInit 0 */

  /* USER CODE END DAC_MspInit 0 */
    /* DAC clock enable */
    __HAL_RCC_DAC_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DAC GPIO Configuration
    PA4     ------> DAC_OUT1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* DAC DMA Init */
    /* DAC1 Init */
    hdma_dac1.Instance = DMA1_Stream5;
    hdma_dac1.Init.Channel = DMA_CHANNEL_7;
    hdma_dac1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_dac1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_dac1.Init.Mode = DMA_CIRCULAR;
    hdma_dac1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_dac1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_dac1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(dacHandle,DMA_Handle1,hdma_dac1);

  /* USER CODE BEGIN DAC_MspInit 1 */

  /* USER CODE END DAC_MspInit 1 */
  }
}

void HAL_DAC_MspDeInit(DAC_HandleTypeDef* dacHandle)
{

  if(dacHandle->Instance==DAC)
  {
  /* USER CODE BEGIN DAC_MspDeInit 0 */

  /* USER CODE END DAC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC_CLK_DISABLE();

    /**DAC GPIO Configuration
    PA4     ------> DAC_OUT1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

    /* DAC DMA DeInit */
    HAL_DMA_DeInit(dacHandle->DMA_Handle1);
  /* USER CODE BEGIN DAC_MspDeInit 1 */

  /* USER CODE END DAC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
