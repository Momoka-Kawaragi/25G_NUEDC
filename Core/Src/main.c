/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "delay.h"
#include "../MY_DSP/MY_262/MAX262.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "si5351.h"
#include "ad9959.h"
#include "../MY_DSP/MY_Uart/frequency_control.h"
#include "../MY_DSP/MY_Uart/amplitude_control.h"
#include "../MY_DSP/MY_DDS_FUN/9959_scan.h"
#include "../MY_DSP/MY_FFT/Phase.h"
#include "ZPN_Hmi_Pack.h"
#include "ZPN_Uart.h"
#include "ZPN_Hmi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern ADC_HandleTypeDef hadc1;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* FilterFeature_t and model enum are defined in 9959_scan.h */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MHZ(x)    ((uint32_t)((x) * 1000000UL))
#define KHZ(x)    ((uint32_t)((x) * 1000UL))
#define HZ(x)     ((uint32_t)(x))
#define CMD_STATE1_CONTROL     0x04  // 状态1的原切换或控制位 (可根据需要调整)
#define CMD_RESET_OUTPUT       0x09
#define CMD_SCAN_IDENTIFY_ALT  0x05  // 扫频启动命令改为 0x05
#define CMD_SCAN_IDENTIFY_ALT_START 0x05
#define CMD_STATE_OFF          0x00
#define CMD_STATE_ON           0x01
#define DDS_STATE_1           1U
#define DDS_STATE_2           2U
#define DEFAULT_FREQUENCY_HZ  50000U
#define DEFAULT_AMPLITUDE_VAL 1023U
#define DEFAULT_PHASE_VAL     0U
#define STATE2_FREQUENCY_HZ   1000U
// 按标定关系 1023 -> 6080mV，2V 对应约 337 码。
#define STATE2_AMPLITUDE_VAL  337U

/* ===== 二维增益打表结构（频率 × 幅值 → DDS控制码）===== */
/* 打表范围：100Hz-3000Hz(步长100Hz)，1000mV-2000mV(步长100mV) */
typedef struct {
    uint32_t freq_hz;      // 频率 (Hz)
    uint16_t amplitude_mv; // 幅值 (mV)
    uint16_t dds_code;     // AD9959 幅度控制码
} GainTableEntry_2D_t;

/* 打表参数定义 */
#define GAIN_TABLE_2D_FREQ_MIN     100U
#define GAIN_TABLE_2D_FREQ_MAX     3000U
#define GAIN_TABLE_2D_FREQ_STEP    100U
#define GAIN_TABLE_2D_FREQ_COUNT   30U    // (3000-100)/100 + 1 = 30

#define GAIN_TABLE_2D_AMP_MIN      1000U
#define GAIN_TABLE_2D_AMP_MAX      2000U
#define GAIN_TABLE_2D_AMP_STEP     100U
#define GAIN_TABLE_2D_AMP_COUNT    11U    // (2000-1000)/100 + 1 = 11

#define GAIN_TABLE_2D_TOTAL        330U   // 30 * 11 = 330

/* 二维增益打表 - 330个数据点 */
/* 组织方式：按频率外层循环，幅值内层循环 */
/* TODO: 根据实测标定结果填充 dds_code 字段 */
static const GainTableEntry_2D_t g_gain_table_2d[GAIN_TABLE_2D_TOTAL] = {
    /* 频率 100Hz，幅值 1000mV-2000mV */
    {100, 1000, 0}, {100, 1100, 0}, {100, 1200, 0}, {100, 1300, 0}, {100, 1400, 0},
    {100, 1500, 0}, {100, 1600, 0}, {100, 1700, 0}, {100, 1800, 0}, {100, 1900, 0},
    {100, 2000, 0},
    
    /* 频率 200Hz，幅值 1000mV-2000mV */
    {200, 1000, 0}, {200, 1100, 0}, {200, 1200, 0}, {200, 1300, 0}, {200, 1400, 0},
    {200, 1500, 0}, {200, 1600, 0}, {200, 1700, 0}, {200, 1800, 0}, {200, 1900, 0},
    {200, 2000, 0},
    
    /* 后续 30×11=330 个数据点（等待标定数据填充） */
    /* 可用以下模板补充: {freq, amp, dds_code} */
    /* 300Hz */ {300, 1000, 0}, {300, 1100, 0}, {300, 1200, 0}, {300, 1300, 0}, {300, 1400, 0}, {300, 1500, 0}, {300, 1600, 0}, {300, 1700, 0}, {300, 1800, 0}, {300, 1900, 0}, {300, 2000, 0},
    /* 400Hz */ {400, 1000, 0}, {400, 1100, 0}, {400, 1200, 0}, {400, 1300, 0}, {400, 1400, 0}, {400, 1500, 0}, {400, 1600, 0}, {400, 1700, 0}, {400, 1800, 0}, {400, 1900, 0}, {400, 2000, 0},
    /* 500Hz */ {500, 1000, 0}, {500, 1100, 0}, {500, 1200, 0}, {500, 1300, 0}, {500, 1400, 0}, {500, 1500, 0}, {500, 1600, 0}, {500, 1700, 0}, {500, 1800, 0}, {500, 1900, 0}, {500, 2000, 0},
    /* 600Hz */ {600, 1000, 0}, {600, 1100, 0}, {600, 1200, 0}, {600, 1300, 0}, {600, 1400, 0}, {600, 1500, 0}, {600, 1600, 0}, {600, 1700, 0}, {600, 1800, 0}, {600, 1900, 0}, {600, 2000, 0},
    /* 700Hz */ {700, 1000, 0}, {700, 1100, 0}, {700, 1200, 0}, {700, 1300, 0}, {700, 1400, 0}, {700, 1500, 0}, {700, 1600, 0}, {700, 1700, 0}, {700, 1800, 0}, {700, 1900, 0}, {700, 2000, 0},
    /* 800Hz */ {800, 1000, 0}, {800, 1100, 0}, {800, 1200, 0}, {800, 1300, 0}, {800, 1400, 0}, {800, 1500, 0}, {800, 1600, 0}, {800, 1700, 0}, {800, 1800, 0}, {800, 1900, 0}, {800, 2000, 0},
    /* 900Hz */ {900, 1000, 0}, {900, 1100, 0}, {900, 1200, 0}, {900, 1300, 0}, {900, 1400, 0}, {900, 1500, 0}, {900, 1600, 0}, {900, 1700, 0}, {900, 1800, 0}, {900, 1900, 0}, {900, 2000, 0},
    /* 1000Hz */ {1000, 1000, 0}, {1000, 1100, 0}, {1000, 1200, 0}, {1000, 1300, 0}, {1000, 1400, 0}, {1000, 1500, 0}, {1000, 1600, 0}, {1000, 1700, 0}, {1000, 1800, 0}, {1000, 1900, 0}, {1000, 2000, 0},
    /* 1100Hz */ {1100, 1000, 0}, {1100, 1100, 0}, {1100, 1200, 0}, {1100, 1300, 0}, {1100, 1400, 0}, {1100, 1500, 0}, {1100, 1600, 0}, {1100, 1700, 0}, {1100, 1800, 0}, {1100, 1900, 0}, {1100, 2000, 0},
    /* 1200Hz */ {1200, 1000, 0}, {1200, 1100, 0}, {1200, 1200, 0}, {1200, 1300, 0}, {1200, 1400, 0}, {1200, 1500, 0}, {1200, 1600, 0}, {1200, 1700, 0}, {1200, 1800, 0}, {1200, 1900, 0}, {1200, 2000, 0},
    /* 1300Hz */ {1300, 1000, 0}, {1300, 1100, 0}, {1300, 1200, 0}, {1300, 1300, 0}, {1300, 1400, 0}, {1300, 1500, 0}, {1300, 1600, 0}, {1300, 1700, 0}, {1300, 1800, 0}, {1300, 1900, 0}, {1300, 2000, 0},
    /* 1400Hz */ {1400, 1000, 0}, {1400, 1100, 0}, {1400, 1200, 0}, {1400, 1300, 0}, {1400, 1400, 0}, {1400, 1500, 0}, {1400, 1600, 0}, {1400, 1700, 0}, {1400, 1800, 0}, {1400, 1900, 0}, {1400, 2000, 0},
    /* 1500Hz */ {1500, 1000, 0}, {1500, 1100, 0}, {1500, 1200, 0}, {1500, 1300, 0}, {1500, 1400, 0}, {1500, 1500, 0}, {1500, 1600, 0}, {1500, 1700, 0}, {1500, 1800, 0}, {1500, 1900, 0}, {1500, 2000, 0},
    /* 1600Hz */ {1600, 1000, 0}, {1600, 1100, 0}, {1600, 1200, 0}, {1600, 1300, 0}, {1600, 1400, 0}, {1600, 1500, 0}, {1600, 1600, 0}, {1600, 1700, 0}, {1600, 1800, 0}, {1600, 1900, 0}, {1600, 2000, 0},
    /* 1700Hz */ {1700, 1000, 0}, {1700, 1100, 0}, {1700, 1200, 0}, {1700, 1300, 0}, {1700, 1400, 0}, {1700, 1500, 0}, {1700, 1600, 0}, {1700, 1700, 0}, {1700, 1800, 0}, {1700, 1900, 0}, {1700, 2000, 0},
    /* 1800Hz */ {1800, 1000, 0}, {1800, 1100, 0}, {1800, 1200, 0}, {1800, 1300, 0}, {1800, 1400, 0}, {1800, 1500, 0}, {1800, 1600, 0}, {1800, 1700, 0}, {1800, 1800, 0}, {1800, 1900, 0}, {1800, 2000, 0},
    /* 1900Hz */ {1900, 1000, 0}, {1900, 1100, 0}, {1900, 1200, 0}, {1900, 1300, 0}, {1900, 1400, 0}, {1900, 1500, 0}, {1900, 1600, 0}, {1900, 1700, 0}, {1900, 1800, 0}, {1900, 1900, 0}, {1900, 2000, 0},
    /* 2000Hz */ {2000, 1000, 0}, {2000, 1100, 0}, {2000, 1200, 0}, {2000, 1300, 0}, {2000, 1400, 0}, {2000, 1500, 0}, {2000, 1600, 0}, {2000, 1700, 0}, {2000, 1800, 0}, {2000, 1900, 0}, {2000, 2000, 0},
    /* 2100Hz */ {2100, 1000, 0}, {2100, 1100, 0}, {2100, 1200, 0}, {2100, 1300, 0}, {2100, 1400, 0}, {2100, 1500, 0}, {2100, 1600, 0}, {2100, 1700, 0}, {2100, 1800, 0}, {2100, 1900, 0}, {2100, 2000, 0},
    /* 2200Hz */ {2200, 1000, 0}, {2200, 1100, 0}, {2200, 1200, 0}, {2200, 1300, 0}, {2200, 1400, 0}, {2200, 1500, 0}, {2200, 1600, 0}, {2200, 1700, 0}, {2200, 1800, 0}, {2200, 1900, 0}, {2200, 2000, 0},
    /* 2300Hz */ {2300, 1000, 0}, {2300, 1100, 0}, {2300, 1200, 0}, {2300, 1300, 0}, {2300, 1400, 0}, {2300, 1500, 0}, {2300, 1600, 0}, {2300, 1700, 0}, {2300, 1800, 0}, {2300, 1900, 0}, {2300, 2000, 0},
    /* 2400Hz */ {2400, 1000, 0}, {2400, 1100, 0}, {2400, 1200, 0}, {2400, 1300, 0}, {2400, 1400, 0}, {2400, 1500, 0}, {2400, 1600, 0}, {2400, 1700, 0}, {2400, 1800, 0}, {2400, 1900, 0}, {2400, 2000, 0},
    /* 2500Hz */ {2500, 1000, 0}, {2500, 1100, 0}, {2500, 1200, 0}, {2500, 1300, 0}, {2500, 1400, 0}, {2500, 1500, 0}, {2500, 1600, 0}, {2500, 1700, 0}, {2500, 1800, 0}, {2500, 1900, 0}, {2500, 2000, 0},
    /* 2600Hz */ {2600, 1000, 0}, {2600, 1100, 0}, {2600, 1200, 0}, {2600, 1300, 0}, {2600, 1400, 0}, {2600, 1500, 0}, {2600, 1600, 0}, {2600, 1700, 0}, {2600, 1800, 0}, {2600, 1900, 0}, {2600, 2000, 0},
    /* 2700Hz */ {2700, 1000, 0}, {2700, 1100, 0}, {2700, 1200, 0}, {2700, 1300, 0}, {2700, 1400, 0}, {2700, 1500, 0}, {2700, 1600, 0}, {2700, 1700, 0}, {2700, 1800, 0}, {2700, 1900, 0}, {2700, 2000, 0},
    /* 2800Hz */ {2800, 1000, 0}, {2800, 1100, 0}, {2800, 1200, 0}, {2800, 1300, 0}, {2800, 1400, 0}, {2800, 1500, 0}, {2800, 1600, 0}, {2800, 1700, 0}, {2800, 1800, 0}, {2800, 1900, 0}, {2800, 2000, 0},
    /* 2900Hz */ {2900, 1000, 0}, {2900, 1100, 0}, {2900, 1200, 0}, {2900, 1300, 0}, {2900, 1400, 0}, {2900, 1500, 0}, {2900, 1600, 0}, {2900, 1700, 0}, {2900, 1800, 0}, {2900, 1900, 0}, {2900, 2000, 0},
    /* 3000Hz */ {3000, 1000, 0}, {3000, 1100, 0}, {3000, 1200, 0}, {3000, 1300, 0}, {3000, 1400, 0}, {3000, 1500, 0}, {3000, 1600, 0}, {3000, 1700, 0}, {3000, 1800, 0}, {3000, 1900, 0}, {3000, 2000, 0}
};

// 扫频与特性提取参数（后续可通过串口配置）
#define SWEEP_POINT_COUNT            80U
#define SWEEP_START_FREQ_HZ          1000U
#define SWEEP_STOP_FREQ_HZ           200000U
#define SWEEP_SETTLE_MS              8U
#define SWEEP_SAMPLES_PER_POINT      32U
#define SWEEP_OUTPUT_AMPLITUDE_CODE  600U
#define SWEEP_CHANNEL                0U
#define SWEEP_ENABLE_DEMO            0U
#define SWEEP_PRINT_TEXT             0U
#define SWEEP_PRINT_VOFA             1U
#define SWEEP_EDGE_AVG_POINTS        5U
#define mode      2
#define freq      2
#define Q         0.707
#define channal   3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t g_dds_state = DDS_STATE_1;
static uint32_t last_set_vpp_mv = 2000; // 默认 2000mV
static uint32_t debug_freq_val = 0;     // Debug用：观测当前频率
static uint32_t debug_amp_code = 0;     // Debug用：观测当前幅度码值
static uint32_t g_sweep_freq[SWEEP_POINT_COUNT];
static uint16_t g_sweep_level[SWEEP_POINT_COUNT];
static FilterFeature_t g_filter_feature = {0};

int progress_state = 0; // 0: 扫频 1:adc采集 3.fft
/* Sweep debug/state variables moved into scan module */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Sweep functions moved to 9959_scan module (SCAN_RunAndExtract) */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void DDS_ApplyDefaultOutput(void)
{
  Write_Frequence(0, DEFAULT_FREQUENCY_HZ);
  Write_Amplitude(0, DEFAULT_AMPLITUDE_VAL);
  Write_Phase(0, DEFAULT_PHASE_VAL);
  AD9959_IO_Update();
}

static uint8_t Try_HandleScanIdentifyCommand(void)
{
  // 检查是否为命令 0x03。注意：gFrequencyCmd.frequency 也可能是 0x03（取决于发送端数据填充方式）
  if (gFrequencyCmd.cmd == CMD_SCAN_IDENTIFY_ALT)
  {
    const char *model_text = "SCAN_FAIL";
    HMI_SetText("t9", "SCANNING");
    if (SCAN_RunAndExtract(&g_filter_feature) != 0U)
    {
      model_text = SCAN_ModelTypeToString(g_filter_feature.model_type);
    }
    HMI_SetText("t9", model_text);
    DDS_ApplyDefaultOutput();
    gFrequencyCmd.cmd = 0U;
    gFrequencyCmd.frequency = 0U;
    return 1U;
  }
  return 0U;
}

/* Sweep implementation moved to Core/MY_DSP/MY_DDS_FUN/9959_scan.c */

/**
 * @brief 二维打表查询函数：根据频率和幅值查表获取DDS控制码
 * @param freq_hz: 输入频率 (Hz)
 * @param amplitude_mv: 输入幅值 (mV)
 * @return DDS 幅度控制码 (0 表示查表失败或超出范围)
 */
static uint16_t GainTable_2D_Lookup(uint32_t freq_hz, uint16_t amplitude_mv)
{
    // 1. 范围检查
    if (freq_hz < GAIN_TABLE_2D_FREQ_MIN || freq_hz > GAIN_TABLE_2D_FREQ_MAX ||
        amplitude_mv < GAIN_TABLE_2D_AMP_MIN || amplitude_mv > GAIN_TABLE_2D_AMP_MAX)
    {
        return 0;  // 超出范围
    }
    
    // 2. 频率量化到最近的打表点（四舍五入）
    uint32_t freq_offset = freq_hz - GAIN_TABLE_2D_FREQ_MIN;
    uint16_t freq_idx = (freq_offset + GAIN_TABLE_2D_FREQ_STEP / 2) / GAIN_TABLE_2D_FREQ_STEP;
    if (freq_idx >= GAIN_TABLE_2D_FREQ_COUNT)
        freq_idx = GAIN_TABLE_2D_FREQ_COUNT - 1;
    
    // 3. 幅值量化到最近的打表点（四舍五入）
    uint16_t amp_offset = amplitude_mv - GAIN_TABLE_2D_AMP_MIN;
    uint16_t amp_idx = (amp_offset + GAIN_TABLE_2D_AMP_STEP / 2) / GAIN_TABLE_2D_AMP_STEP;
    if (amp_idx >= GAIN_TABLE_2D_AMP_COUNT)
        amp_idx = GAIN_TABLE_2D_AMP_COUNT - 1;
    
    // 4. 一维化索引（行优先：频率外层，幅值内层）
    uint16_t table_index = freq_idx * GAIN_TABLE_2D_AMP_COUNT + amp_idx;
    
    if (table_index >= GAIN_TABLE_2D_TOTAL)
    {
        return 0;  // 索引越界
    }
    
    return g_gain_table_2d[table_index].dds_code;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */

  /* start microsecond delay and TIM2 PWM (fCLK) */
  //si5351_Init();
  delay_init();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  /* UART retarget test: should print to USART1 if retargeting works */
  printf("UART1 retarget test: Hello from USART1\r\n");

  /* Initialize MAX262 and program three logical channels with same fc/Q */
  MAX262_Init();
  Init_AD9959();   
  DDS_ApplyDefaultOutput();
  
  /* Initialize frequency control module to handle UART commands */
  ZPN_UART_Init();  // ← 启动UART2接收中断
  FrequencyControl_Init();  // ← 注册PACK模板（此步必须在频率控制之前）
  AmplitudeControl_Init();  // ← 共享频率控制的PACK模板

  /* 配置内部采样（TIM3）并启动 FFT DMA 采样 */
  /* FFT disabled for debug - do not start TIM3-triggered ADC DMA */
  // Phase_Set_SamplingRate_Internal(400000U); // 设置 TIM3 触发频率为 400kHz
  // FFT_App_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* FFT processing disabled for debug */
    // FFT_App_Process();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if SWEEP_ENABLE_DEMO
  (void)SCAN_RunAndExtract(&g_filter_feature);
  // 持续扫频：每轮都会更新 g_filter_feature（结果通过参数返回）。
#endif

    PACK_ParseFromRingBuffer();
    if (Try_HandleScanIdentifyCommand())
    {
      continue;
    }

    /* 状态 1 和状态 2 的逻辑处理 */
    if (g_dds_state == DDS_STATE_1)
    {
      /* 状态 1：同时接收并响应 0x01 (频率) 和 0x02 (幅值) 指令 */
      if (gFrequencyCmd.cmd == CMD_STATE1_CONTROL)
      {
        if ((uint8_t)gFrequencyCmd.frequency == CMD_STATE_OFF)
        {
          g_dds_state = DDS_STATE_2; // 55 04 00 FF 跳转至状态 2
        }
        else if ((uint8_t)gFrequencyCmd.frequency == CMD_STATE_ON)
        {
          // 状态1下的固定参数控制也可以保留
          Write_Frequence(0, STATE2_FREQUENCY_HZ);
          Write_Amplitude(0, STATE2_AMPLITUDE_VAL);
          AD9959_IO_Update();
        }
        gFrequencyCmd.cmd = 0;
        gFrequencyCmd.frequency = 0;
      }
      else if (gFrequencyCmd.cmd == CMD_SET_FREQUENCY) // 0x01
      {
        if (gFrequencyCmd.frequency > 0)
        {
          Write_Frequence(0, gFrequencyCmd.frequency);
          AD9959_IO_Update();
        }
        gFrequencyCmd.cmd = 0;
        gFrequencyCmd.frequency = 0;
      }
      else if (gFrequencyCmd.cmd == CMD_SET_AMPLITUDE) // 0x02
      {
        // 状态 1 下，除了设置幅度，还需要记录此时的基准毫伏值供状态 2 使用
        last_set_vpp_mv = gFrequencyCmd.frequency; 

        // 状态1改为复用 amplitude_control 模块的幅值设置函数
        AmplitudeControl_SetAmplitude(last_set_vpp_mv);
        gFrequencyCmd.cmd = 0;
        gFrequencyCmd.frequency = 0;
      }
    }
    else if (g_dds_state == DDS_STATE_2)
    {
      /* 状态 2：二维打表补偿逻辑 */
      if (gFrequencyCmd.cmd == CMD_STATE1_CONTROL && (uint8_t)gFrequencyCmd.frequency == CMD_STATE_ON)
      {
        g_dds_state = DDS_STATE_1; // 55 04 01 FF 返回状态 1
        gFrequencyCmd.cmd = 0;
        gFrequencyCmd.frequency = 0;
      }
      else if (gFrequencyCmd.cmd == CMD_SET_FREQUENCY) // 0x01
      {
          uint32_t freq_val = gFrequencyCmd.frequency;
          
          // 直接查表获取DDS控制码
          uint16_t dds_code = GainTable_2D_Lookup(freq_val, last_set_vpp_mv);
          
          if (dds_code > 0)
          {
            // 查表命中，应用结果
            Write_Frequence(0, freq_val);
            Write_Amplitude(0, dds_code);
            
            // 更新 Debug 变量
            debug_freq_val = freq_val;
            debug_amp_code = dds_code;
          }
          else
          {
            // 查表失败（超出范围或表项值为0），清零输出
            debug_freq_val = 0;
            debug_amp_code = 0;
            Write_Frequence(0, 0);
            Write_Amplitude(0, 0);
          }
          
          AD9959_IO_Update();
          
          gFrequencyCmd.cmd = 0;
          gFrequencyCmd.frequency = 0;
      }
      else if (gFrequencyCmd.cmd == CMD_SET_AMPLITUDE) // 0x02
      {
          // 状态 2 下，更新基准幅值
          last_set_vpp_mv = gFrequencyCmd.frequency;
          gFrequencyCmd.cmd = 0;
          gFrequencyCmd.frequency = 0;
      }
    }

    if (gFrequencyCmd.cmd == CMD_RESET_OUTPUT)
    {
      if ((uint8_t)gFrequencyCmd.frequency == 0U)
      {
        DDS_ApplyDefaultOutput();
      }
      gFrequencyCmd.cmd = 0;
      gFrequencyCmd.frequency = 0;
    }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
