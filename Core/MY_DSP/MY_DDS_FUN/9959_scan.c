/* 9959_scan.c - Sweep implementation using AD9959 control and ADC measurement */
#include "9959_scan.h"
#include "ad9959.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "delay.h"
#include "ZPN_Hmi.h"
#include <stdio.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim3;

/* Internal sweep buffers */
static uint32_t g_sweep_freq[501U];
static uint16_t g_sweep_level[501U];
static uint16_t g_sweep_phase[501U];

static uint16_t SCAN_ReadAdcAverage(ADC_HandleTypeDef *hadc, uint16_t sample_count)
{
  uint32_t sum = 0U;

  if (sample_count == 0U)
  {
    return 0U;
  }

  for (uint16_t i = 0; i < sample_count; i++)
  {
    if (HAL_ADC_Start(hadc) != HAL_OK)
    {
      return 0U;
    }
    if (HAL_ADC_PollForConversion(hadc, 20U) != HAL_OK)
    {
      HAL_ADC_Stop(hadc);
      return 0U;
    }
    sum += HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
  }

  return (uint16_t)(sum / sample_count);
}

const char *SCAN_ModelTypeToString(uint8_t model_type)
{
  switch (model_type)
  {
  case FILTER_MODEL_LOWPASS:
    return "LOWPASS";
  case FILTER_MODEL_HIGHPASS:
    return "HIGHPASS";
  case FILTER_MODEL_BANDPASS:
    return "BANDPASS";
  case FILTER_MODEL_BANDSTOP:
    return "BANDSTOP";
  default:
    return "UNKNOWN";
  }
}

uint8_t SCAN_RunAndExtract(FilterFeature_t *feature)
{
  uint32_t peak_freq_hz = 0U;
  uint16_t peak_idx = 0;
  uint16_t peak_level = 0;
  uint16_t min_idx = 0;
  uint16_t min_level = 0xFFFFU;

  if (feature == NULL) return 0U;
  if (SWEEP_POINT_COUNT < 5U || SWEEP_STOP_FREQ_HZ <= SWEEP_START_FREQ_HZ) return 0U;

  HAL_TIM_Base_Start(&htim3);

  Write_Amplitude(SWEEP_CHANNEL, SWEEP_OUTPUT_AMPLITUDE_CODE);
  Write_Phase(SWEEP_CHANNEL, 0U);
  AD9959_IO_Update();

  for (uint16_t i = 0; i < SWEEP_POINT_COUNT; i++)
  {
    /* Compute frequency per step. Prefer fixed step if SWEEP_STEP_HZ is defined. */
  #ifdef SWEEP_STEP_HZ
    uint32_t frequency_hz = SWEEP_START_FREQ_HZ + (uint32_t)((uint64_t)SWEEP_STEP_HZ * i);
  #else
    uint32_t frequency_hz = SWEEP_START_FREQ_HZ + (uint32_t)(((uint64_t)(SWEEP_STOP_FREQ_HZ - SWEEP_START_FREQ_HZ) * i) / (SWEEP_POINT_COUNT - 1U));
  #endif
    g_sweep_freq[i] = frequency_hz;
    Write_Frequence(SWEEP_CHANNEL, frequency_hz);
    AD9959_IO_Update();

    /* 在 1kHz 频点额外停留 1s */
    if (frequency_hz == 1000U)
    {
      HAL_Delay(2000U);
    }

    HAL_Delay(SWEEP_SETTLE_MS);
    /* 每次改频后，获取幅度和相位 ADC 值。 */
    g_sweep_level[i] = SCAN_ReadAdcAverage(&hadc1, SWEEP_SAMPLES_PER_POINT);
    g_sweep_phase[i] = SCAN_ReadAdcAverage(&hadc2, SWEEP_SAMPLES_PER_POINT);

    if (g_sweep_level[i] > peak_level)
    {
      peak_level = g_sweep_level[i];
      peak_idx = i;
      peak_freq_hz = frequency_hz;
    }

    if (g_sweep_level[i] < min_level)
    {
      min_level = g_sweep_level[i];
      min_idx = i;
    }
  }

  if (peak_level == 0U)
  {
    printf("[SWEEP] fail: peak level is zero\r\n");
    return 0U;
  }

  printf("[SWEEP] table begin, points=%u\r\n", (unsigned int)SWEEP_POINT_COUNT);
  printf("Amplitude Profile (Vpp mV):\r\n[");
  for (uint16_t i = 0; i < SWEEP_POINT_COUNT; i++)
  {
    /* 转换逻辑: Vpp = 0.1993 * ADC_Value + 2.6314 */
    float vpp_mv = 0.1993f * (float)g_sweep_level[i] + 2.6314f;
    printf("[%lu,%.2f]%s",
           (unsigned long)g_sweep_freq[i],
           vpp_mv,
           (i == SWEEP_POINT_COUNT - 1) ? "" : ",");
  }
  printf("]\r\n");

  printf("Phase Profile (Degrees):\r\n[");
  
  /* 寻找扫频过程中的相位最大值点及其索引 */
  uint16_t max_phase_idx = 0;
  float max_phase_deg = -999.0f;
  for (uint16_t i = 0; i < SWEEP_POINT_COUNT; i++) {
    float voltage = (float)g_sweep_phase[i] * 3.3f / 4096.0f;
    float current_phase = (1.83f - voltage) * 100.0f;
    if (current_phase > max_phase_deg) {
      max_phase_deg = current_phase;
      max_phase_idx = i;
    }
  }

  /* 计算相位平均值作为偏移量，用于将中轴移动到0点 */
  float phase_sum = 0.0f;
  for (uint16_t i = 0; i < SWEEP_POINT_COUNT; i++) {
    float voltage = (float)g_sweep_phase[i] * 3.3f / 4096.0f;
    float p = (1.83f - voltage) * 100.0f;
    if (i > max_phase_idx) {
      p = 2.0f * max_phase_deg - p;
    }
    phase_sum += p;
  }
  float phase_offset = phase_sum / (float)SWEEP_POINT_COUNT;

  for (uint16_t i = 0; i < SWEEP_POINT_COUNT; i++)
  {
    /* 1. ADC原始值转电压并根据公式转换为初始相位 */
    float voltage = (float)g_sweep_phase[i] * 3.3f / 4096.0f;
    float phase_deg = (1.83f - voltage) * 100.0f;

    /* 2. 优化逻辑：在相位最大值点之后，以该点相位为轴向上翻折 */
    if (i > max_phase_idx) {
      phase_deg = 2.0f * max_phase_deg - phase_deg;
    }

    /* 3. 添加偏置逻辑：减去平均值，将中轴移动到0 */
    phase_deg -= phase_offset;

    printf("[%lu,%.2f]%s",
           (unsigned long)g_sweep_freq[i],
           phase_deg,
           (i == SWEEP_POINT_COUNT - 1) ? "" : ",");
  }
  printf("]\r\n");
  printf("[SWEEP] table end\r\n");

  /* --- 1. 滤波器特征提取逻辑 --- */
  uint16_t start_level = g_sweep_level[0];
  uint16_t end_level = g_sweep_level[SWEEP_POINT_COUNT - 1];
  uint16_t mid_level = g_sweep_level[SWEEP_POINT_COUNT / 2];

  if (peak_level < 50U) {
    feature->model_type = FILTER_MODEL_UNKNOWN;
  } else if (start_level > peak_level * 0.7f && end_level < peak_level * 0.3f) {
    feature->model_type = FILTER_MODEL_LOWPASS;
  } else if (start_level < peak_level * 0.3f && end_level > peak_level * 0.7f) {
    feature->model_type = FILTER_MODEL_HIGHPASS;
  } else if (peak_idx > SWEEP_EDGE_AVG_POINTS && peak_idx < (SWEEP_POINT_COUNT - SWEEP_EDGE_AVG_POINTS) &&
             start_level < peak_level * 0.5f && end_level < peak_level * 0.5f) {
    feature->model_type = FILTER_MODEL_BANDPASS;
  } else if (mid_level < start_level * 0.5f && mid_level < end_level * 0.5f) {
    feature->model_type = FILTER_MODEL_BANDSTOP;
  } else {
    feature->model_type = FILTER_MODEL_UNKNOWN;
  }

  feature->valid = 1U;
  feature->f0_hz = peak_freq_hz;
  feature->peak_level = peak_level;
  feature->min_level = min_level;

  return 1U;
}
