/* 9959_scan.h - Sweep encapsulation for AD9959-based scan */
#ifndef __9959_SCAN_H__
#define __9959_SCAN_H__

#include <stdint.h>

/* Sweep configuration defaults (can be overridden before build) */
#define SWEEP_POINT_COUNT            501U
#define SWEEP_START_FREQ_HZ          1000U
#define SWEEP_STOP_FREQ_HZ           100000U
#define SWEEP_SETTLE_MS            100U
#define SWEEP_SAMPLES_PER_POINT      64U
#define SWEEP_OUTPUT_AMPLITUDE_CODE  1023U
#define SWEEP_CHANNEL                0U
#define SWEEP_EDGE_AVG_POINTS        5U
/* Frequency step per point (Hz). Set to 1000 for 1 kHz increments. */
/* Use 200 Hz step to increase resolution */
#ifndef SWEEP_STEP_HZ
#define SWEEP_STEP_HZ                200U
#endif

typedef struct
{
  uint8_t valid;
  uint8_t model_type;
  uint32_t fc_hz;
  uint32_t f0_hz;
  float q_factor;
  uint32_t f1_hz;
  uint32_t f2_hz;
  uint16_t peak_level;
  uint16_t min_level;
} FilterFeature_t;

typedef enum
{
  FILTER_MODEL_UNKNOWN = 0,
  FILTER_MODEL_LOWPASS = 1,
  FILTER_MODEL_HIGHPASS = 2,
  FILTER_MODEL_BANDPASS = 3,
  FILTER_MODEL_BANDSTOP = 4
} FilterModelType_t;

/* Run a sweep and extract filter feature. Returns 1 on success, 0 on fail. */
uint8_t SCAN_RunAndExtract(FilterFeature_t *feature);

/* Convert model type enum to short text for UI display. */
const char *SCAN_ModelTypeToString(uint8_t model_type);

/* Lookup sweep results for a given frequency. Returns ADC level. */
uint16_t SCAN_LookupLevelForFreq(uint32_t freq_hz);

#endif /* __9959_SCAN_H__ */
