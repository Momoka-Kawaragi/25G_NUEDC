/* 9959_scan.c - Sweep implementation using AD9959 control and ADC measurement */
#include "9959_scan.h"
#include "ad9959.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "delay.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;

/* Internal buffers and debug variables */
static uint32_t g_sweep_freq[SWEEP_POINT_COUNT];
static uint16_t g_sweep_level[SWEEP_POINT_COUNT];
static uint16_t g_sweep_in_level[SWEEP_POINT_COUNT];
static float g_sweep_gain_db[SWEEP_POINT_COUNT];
static float g_sweep_phase_deg[SWEEP_POINT_COUNT];
static uint8_t g_dbg_sweep_running = 0U;
static uint16_t g_dbg_sweep_index = 0U;
static uint32_t g_dbg_current_freq_hz = 0U;
static uint16_t g_dbg_current_level = 0U;
static uint16_t g_dbg_peak_level = 0U;
static uint16_t g_dbg_peak_index = 0U;
static uint32_t g_dbg_f0_hz = 0U;
static uint32_t g_dbg_f1_hz = 0U;
static uint32_t g_dbg_f2_hz = 0U;
static float g_dbg_q_factor = 0.0f;
static uint8_t g_dbg_feature_valid = 0U;
static float g_dbg_target_3db_level = 0.0f;
static uint8_t g_dbg_model_type = 0U;
static uint32_t g_dbg_fc_hz = 0U;

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#ifndef SWEEP_PHASE_SAMPLES
#define SWEEP_PHASE_SAMPLES 48U
#endif

#ifndef SWEEP_SAMPLE_INTERVAL_US
#define SWEEP_SAMPLE_INTERVAL_US 8U
#endif

static void SCAN_PC_Printf(const char *fmt, ...)
{
  char tx_buf[160];
  va_list args;
  int len;
  va_start(args, fmt);
  len = vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
  va_end(args);
  if (len <= 0) return;
  if (len > (int)sizeof(tx_buf)) len = (int)sizeof(tx_buf);
  HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)len, 1000U);
}

static void SCAN_VOFA_Send4(float c1, float c2, float c3, float c4)
{
  char tx_buf[128];
  int len = snprintf(tx_buf, sizeof(tx_buf), "%.3f,%.3f,%.3f,%.3f\n", c1, c2, c3, c4);
  if (len <= 0) return;
  if (len > (int)sizeof(tx_buf)) len = (int)sizeof(tx_buf);
  HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)len, 1000U);
}

static uint16_t SCAN_ReadAdcLevel(ADC_HandleTypeDef *hadc, uint16_t sample_count)
{
  uint64_t sum = 0;
  uint32_t mean = 0;
  uint64_t abs_sum = 0;
  if (sample_count == 0U) return 0U;
  for (uint16_t i = 0; i < sample_count; i++)
  {
    if (HAL_ADC_Start(hadc) != HAL_OK) return 0U;
    if (HAL_ADC_PollForConversion(hadc, 20U) != HAL_OK)
    {
      HAL_ADC_Stop(hadc);
      return 0U;
    }
    sum += HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
  }
  mean = (uint32_t)(sum / sample_count);
  for (uint16_t i = 0; i < sample_count; i++)
  {
    if (HAL_ADC_Start(hadc) != HAL_OK) return 0U;
    if (HAL_ADC_PollForConversion(hadc, 20U) != HAL_OK)
    {
      HAL_ADC_Stop(hadc);
      return 0U;
    }
    uint32_t raw = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    if (raw >= mean) abs_sum += (raw - mean);
    else abs_sum += (mean - raw);
  }
  return (uint16_t)(abs_sum / sample_count);
}

static uint16_t SCAN_ReadAdcSample(ADC_HandleTypeDef *hadc)
{
  if (HAL_ADC_Start(hadc) != HAL_OK) return 0U;
  if (HAL_ADC_PollForConversion(hadc, 20U) != HAL_OK)
  {
    HAL_ADC_Stop(hadc);
    return 0U;
  }
  {
    uint16_t raw = (uint16_t)HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return raw;
  }
}

static void SCAN_WrapPhase(float *phase_deg)
{
  while (*phase_deg > 180.0f) *phase_deg -= 360.0f;
  while (*phase_deg < -180.0f) *phase_deg += 360.0f;
}

static uint8_t SCAN_MeasureGainAndPhase(uint32_t frequency_hz,
                                        uint16_t *out_in_amp,
                                        uint16_t *out_out_amp,
                                        float *out_gain_db,
                                        float *out_phase_deg)
{
  uint16_t n;
  uint32_t t0;
  uint16_t in_buf[SWEEP_PHASE_SAMPLES];
  uint16_t out_buf[SWEEP_PHASE_SAMPLES];
  uint32_t ts_cycles[SWEEP_PHASE_SAMPLES];
  uint32_t sum_in = 0U;
  uint32_t sum_out = 0U;
  float mean_in;
  float mean_out;
  float i_in = 0.0f, q_in = 0.0f;
  float i_out = 0.0f, q_out = 0.0f;
  float amp_in;
  float amp_out;
  float phase_in;
  float phase_out;
  float phase_deg;

  if (frequency_hz == 0U || out_in_amp == NULL || out_out_amp == NULL || out_gain_db == NULL || out_phase_deg == NULL)
  {
    return 0U;
  }

  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
  {
    delay_init();
  }

  t0 = DWT->CYCCNT;
  for (n = 0U; n < SWEEP_PHASE_SAMPLES; n++)
  {
    uint16_t in_raw = SCAN_ReadAdcSample(&hadc2);
    uint16_t out_raw = SCAN_ReadAdcSample(&hadc1);
    in_buf[n] = in_raw;
    out_buf[n] = out_raw;
    sum_in += in_raw;
    sum_out += out_raw;
    ts_cycles[n] = DWT->CYCCNT - t0;
    delay_us(SWEEP_SAMPLE_INTERVAL_US);
  }

  mean_in = (float)sum_in / (float)SWEEP_PHASE_SAMPLES;
  mean_out = (float)sum_out / (float)SWEEP_PHASE_SAMPLES;

  for (n = 0U; n < SWEEP_PHASE_SAMPLES; n++)
  {
    float t = (float)ts_cycles[n] / (float)SystemCoreClock;
    float theta = 2.0f * PI * (float)frequency_hz * t;
    float c = cosf(theta);
    float s = sinf(theta);
    float xin = (float)in_buf[n] - mean_in;
    float xout = (float)out_buf[n] - mean_out;
    i_in += xin * c;
    q_in += xin * s;
    i_out += xout * c;
    q_out += xout * s;
  }

  amp_in = (2.0f / (float)SWEEP_PHASE_SAMPLES) * sqrtf(i_in * i_in + q_in * q_in);
  amp_out = (2.0f / (float)SWEEP_PHASE_SAMPLES) * sqrtf(i_out * i_out + q_out * q_out);

  phase_in = atan2f(q_in, i_in);
  phase_out = atan2f(q_out, i_out);
  phase_deg = (phase_out - phase_in) * (180.0f / PI);
  SCAN_WrapPhase(&phase_deg);

  *out_in_amp = (uint16_t)(amp_in + 0.5f);
  *out_out_amp = (uint16_t)(amp_out + 0.5f);
  if (amp_in > 1e-6f && amp_out > 1e-6f)
  {
    *out_gain_db = 20.0f * log10f(amp_out / amp_in);
  }
  else
  {
    *out_gain_db = -120.0f;
  }
  *out_phase_deg = phase_deg;
  return 1U;
}

static uint8_t SCAN_FindCrossFreq(const uint32_t *freq_axis, const uint16_t *level_axis, uint16_t start, int32_t step, uint16_t stop, float target, float *out_freq)
{
  uint16_t i = start;
  while (i != stop)
  {
    uint16_t next = (uint16_t)((int32_t)i + step);
    float y1 = (float)level_axis[i];
    float y2 = (float)level_axis[next];
    if ((y1 >= target && y2 <= target) || (y1 <= target && y2 >= target))
    {
      float x1 = (float)freq_axis[i];
      float x2 = (float)freq_axis[next];
      float denom = (y2 - y1);
      if (fabsf(denom) < 1e-6f) *out_freq = x1;
      else *out_freq = x1 + (target - y1) * (x2 - x1) / denom;
      return 1U;
    }
    i = next;
  }
  return 0U;
}

static uint8_t SCAN_ClassifyModel(const uint16_t *level_axis, uint16_t point_count, uint16_t peak_idx, uint16_t min_idx, float low_avg, float high_avg, uint16_t peak_level, uint16_t min_level)
{
  uint16_t edge = SWEEP_EDGE_AVG_POINTS;
  if (point_count < 10U) return FILTER_MODEL_UNKNOWN;
  if (edge * 2U >= point_count) edge = point_count / 4U;
  if ((peak_idx <= edge) && (low_avg > high_avg * 2.0f)) return FILTER_MODEL_LOWPASS;
  if ((peak_idx >= (point_count - 1U - edge)) && (high_avg > low_avg * 2.0f)) return FILTER_MODEL_HIGHPASS;
  if ((peak_idx > edge) && (peak_idx < (point_count - 1U - edge)) && ((float)peak_level > low_avg * 1.8f) && ((float)peak_level > high_avg * 1.8f)) return FILTER_MODEL_BANDPASS;
  if ((min_idx > edge) && (min_idx < (point_count - 1U - edge)) && (low_avg > (float)min_level * 2.0f) && (high_avg > (float)min_level * 2.0f)) return FILTER_MODEL_BANDSTOP;
  if (low_avg > high_avg * 1.2f) return FILTER_MODEL_LOWPASS;
  if (high_avg > low_avg * 1.2f) return FILTER_MODEL_HIGHPASS;
  (void)level_axis;
  return FILTER_MODEL_UNKNOWN;
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
  uint16_t peak_idx = 0;
  uint16_t peak_level = 0;
  uint16_t min_idx = 0;
  uint16_t min_level = 0xFFFFU;
  float target;
  float low_avg = 0.0f;
  float high_avg = 0.0f;
  float fc_cross = 0.0f;
  float f1_cross = 0.0f;
  float f2_cross = 0.0f;
  uint32_t edge_sum_low = 0U;
  uint32_t edge_sum_high = 0U;
  uint16_t edge_n;
  uint8_t model_type;
  uint8_t has_fc = 0U;
  uint8_t has_f1 = 0U;
  uint8_t has_f2 = 0U;

  if (feature == NULL) return 0U;
  if (SWEEP_POINT_COUNT < 5U || SWEEP_STOP_FREQ_HZ <= SWEEP_START_FREQ_HZ) return 0U;

#if SWEEP_PRINT_TEXT
  SCAN_PC_Printf("[SWEEP] start=%luHz stop=%luHz points=%u ampCode=%u\r\n",
                 (unsigned long)SWEEP_START_FREQ_HZ,
                 (unsigned long)SWEEP_STOP_FREQ_HZ,
                 (unsigned int)SWEEP_POINT_COUNT,
                 (unsigned int)SWEEP_OUTPUT_AMPLITUDE_CODE);
#endif

  g_dbg_sweep_running = 1U;
  g_dbg_sweep_index = 0U;
  g_dbg_current_freq_hz = 0U;
  g_dbg_current_level = 0U;
  g_dbg_peak_level = 0U;
  g_dbg_peak_index = 0U;
  g_dbg_target_3db_level = 0.0f;
  g_dbg_f0_hz = 0U;
  g_dbg_f1_hz = 0U;
  g_dbg_f2_hz = 0U;
  g_dbg_q_factor = 0.0f;
  g_dbg_feature_valid = 0U;
  g_dbg_model_type = 0U;
  g_dbg_fc_hz = 0U;

  HAL_TIM_Base_Start(&htim3);

  Write_Amplitude(SWEEP_CHANNEL, SWEEP_OUTPUT_AMPLITUDE_CODE);
  Write_Phase(SWEEP_CHANNEL, 0U);
  AD9959_IO_Update();

  edge_n = SWEEP_EDGE_AVG_POINTS;
  if (edge_n * 2U >= SWEEP_POINT_COUNT) edge_n = SWEEP_POINT_COUNT / 4U;

  for (uint16_t i = 0; i < SWEEP_POINT_COUNT; i++)
  {
    /* Compute frequency per step. Prefer fixed step if SWEEP_STEP_HZ is defined. */
  #ifdef SWEEP_STEP_HZ
    uint32_t frequency_hz = SWEEP_START_FREQ_HZ + (uint32_t)((uint64_t)SWEEP_STEP_HZ * i);
  #else
    uint32_t frequency_hz = SWEEP_START_FREQ_HZ + (uint32_t)(((uint64_t)(SWEEP_STOP_FREQ_HZ - SWEEP_START_FREQ_HZ) * i) / (SWEEP_POINT_COUNT - 1U));
  #endif
    g_sweep_freq[i] = frequency_hz;
    g_dbg_sweep_index = i;
    g_dbg_current_freq_hz = frequency_hz;
    Write_Frequence(SWEEP_CHANNEL, frequency_hz);
    AD9959_IO_Update();
    HAL_Delay(SWEEP_SETTLE_MS);

    /* extra 1s hold at minimum and maximum frequency points */
    if (i == 0U) HAL_Delay(1000);
    if (i == (SWEEP_POINT_COUNT - 1U)) HAL_Delay(1000);

    /* Measure amplitude response (ADC2=input, ADC1=output) and phase response. */
    {
      uint16_t in_amp = 0U;
      uint16_t out_amp = 0U;
      float gain_db = 0.0f;
      float phase_deg = 0.0f;
      if (SCAN_MeasureGainAndPhase(frequency_hz, &in_amp, &out_amp, &gain_db, &phase_deg) == 0U)
      {
        in_amp = SCAN_ReadAdcLevel(&hadc2, SWEEP_SAMPLES_PER_POINT);
        out_amp = SCAN_ReadAdcLevel(&hadc1, SWEEP_SAMPLES_PER_POINT);
        if (in_amp > 0U)
        {
          gain_db = 20.0f * log10f((float)out_amp / (float)in_amp);
        }
        else
        {
          gain_db = -120.0f;
        }
        phase_deg = 0.0f;
      }
      g_sweep_in_level[i] = in_amp;
      g_sweep_level[i] = out_amp;
      g_sweep_gain_db[i] = gain_db;
      g_sweep_phase_deg[i] = phase_deg;
    }
    g_dbg_current_level = g_sweep_level[i];

    if (i < edge_n) edge_sum_low += g_sweep_level[i];
    if (i >= (SWEEP_POINT_COUNT - edge_n)) edge_sum_high += g_sweep_level[i];

#if SWEEP_PRINT_VOFA
  SCAN_VOFA_Send4((float)frequency_hz, g_sweep_gain_db[i], g_sweep_phase_deg[i], (float)g_sweep_level[i]);
#endif

#if SWEEP_PRINT_TEXT
  SCAN_PC_Printf("[SWEEP] i=%u f=%luHz in=%u out=%u gain=%.2fdB phase=%.1fdeg\r\n",
                  (unsigned int)i,
                  (unsigned long)frequency_hz,
          (unsigned int)g_sweep_in_level[i],
          (unsigned int)g_sweep_level[i],
          g_sweep_gain_db[i],
          g_sweep_phase_deg[i]);
#endif

    if (g_sweep_level[i] > peak_level)
    {
      peak_level = g_sweep_level[i];
      peak_idx = i;
      g_dbg_peak_level = peak_level;
      g_dbg_peak_index = peak_idx;
    }

    if (g_sweep_level[i] < min_level)
    {
      min_level = g_sweep_level[i];
      min_idx = i;
    }
  }

  if (peak_level == 0U)
  {
    g_dbg_sweep_running = 0U;
#if SWEEP_PRINT_TEXT
    SCAN_PC_Printf("[SWEEP] fail: peak_level=0\r\n");
#endif
    /* post-sweep delay on failure */
    HAL_Delay(1000);
    return 0U;
  }

  low_avg = (float)edge_sum_low / (float)edge_n;
  high_avg = (float)edge_sum_high / (float)edge_n;

  model_type = SCAN_ClassifyModel(g_sweep_level, SWEEP_POINT_COUNT, peak_idx, min_idx, low_avg, high_avg, peak_level, min_level);

  feature->valid = 0U;
  feature->model_type = model_type;
  feature->fc_hz = 0U;
  feature->f0_hz = g_sweep_freq[peak_idx];
  feature->peak_level = peak_level;
  feature->min_level = min_level;
  feature->f1_hz = 0U;
  feature->f2_hz = 0U;
  feature->q_factor = 0.0f;

  g_dbg_f0_hz = feature->f0_hz;
  g_dbg_model_type = model_type;

  if (model_type == FILTER_MODEL_LOWPASS)
  {
    target = low_avg * 0.70710678f;
    has_fc = SCAN_FindCrossFreq(g_sweep_freq, g_sweep_level, 0U, 1, (SWEEP_POINT_COUNT - 1U), target, &fc_cross);
    g_dbg_target_3db_level = target;
    if (has_fc)
    {
      feature->fc_hz = (uint32_t)(fc_cross + 0.5f);
      feature->valid = 1U;
      g_dbg_fc_hz = feature->fc_hz;
      g_dbg_feature_valid = 1U;
    }
  }
  else if (model_type == FILTER_MODEL_HIGHPASS)
  {
    target = high_avg * 0.70710678f;
    has_fc = SCAN_FindCrossFreq(g_sweep_freq, g_sweep_level, (SWEEP_POINT_COUNT - 1U), -1, 0U, target, &fc_cross);
    g_dbg_target_3db_level = target;
    if (has_fc)
    {
      feature->fc_hz = (uint32_t)(fc_cross + 0.5f);
      feature->valid = 1U;
      g_dbg_fc_hz = feature->fc_hz;
      g_dbg_feature_valid = 1U;
    }
  }
  else if (model_type == FILTER_MODEL_BANDPASS)
  {
    target = (float)peak_level * 0.70710678f;
    g_dbg_target_3db_level = target;
    if (peak_idx > 0U) has_f1 = SCAN_FindCrossFreq(g_sweep_freq, g_sweep_level, peak_idx, -1, 0U, target, &f1_cross);
    if (peak_idx < (SWEEP_POINT_COUNT - 1U)) has_f2 = SCAN_FindCrossFreq(g_sweep_freq, g_sweep_level, peak_idx, 1, (SWEEP_POINT_COUNT - 1U), target, &f2_cross);
    if (has_f1 && has_f2 && (f2_cross > f1_cross))
    {
      float bw = f2_cross - f1_cross;
      feature->f1_hz = (uint32_t)(f1_cross + 0.5f);
      feature->f2_hz = (uint32_t)(f2_cross + 0.5f);
      feature->q_factor = ((float)feature->f0_hz) / bw;
      feature->valid = 1U;
      g_dbg_f1_hz = feature->f1_hz;
      g_dbg_f2_hz = feature->f2_hz;
      g_dbg_q_factor = feature->q_factor;
      g_dbg_feature_valid = feature->valid;
    }
  }
  else if (model_type == FILTER_MODEL_BANDSTOP)
  {
    feature->f0_hz = g_sweep_freq[min_idx];
    feature->valid = 1U;
    g_dbg_f0_hz = feature->f0_hz;
    g_dbg_feature_valid = 1U;
  }

#if SWEEP_PRINT_VOFA
  SCAN_VOFA_Send4((float)feature->model_type, (float)feature->fc_hz, (float)feature->f0_hz, feature->q_factor);
#endif

#if SWEEP_PRINT_TEXT
  if (feature->valid)
  {
    SCAN_PC_Printf("[SWEEP] model=%u fc=%luHz f0=%luHz f1=%luHz f2=%luHz Q=%.4f peak=%u min=%u\r\n",
                   (unsigned int)feature->model_type,
                   (unsigned long)feature->fc_hz,
                   (unsigned long)feature->f0_hz,
                   (unsigned long)feature->f1_hz,
                   (unsigned long)feature->f2_hz,
                   feature->q_factor,
                   (unsigned int)feature->peak_level,
                   (unsigned int)feature->min_level);
  }
  else
  {
    SCAN_PC_Printf("[SWEEP] partial model=%u fc=%luHz f0=%luHz peak=%u min=%u\r\n",
                   (unsigned int)feature->model_type,
                   (unsigned long)feature->fc_hz,
                   (unsigned long)feature->f0_hz,
                   (unsigned int)feature->peak_level,
                   (unsigned int)feature->min_level);
  }
#endif

  g_dbg_sweep_running = 0U;
  return 1U;
}
