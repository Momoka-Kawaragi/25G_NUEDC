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

/* 扫频结果查表 (频率(Hz), 增益(|H|)) 
 * 已知输入幅度为 1V，故增益 = 实测 Vpp / 1.0V */
typedef struct {
    uint32_t freq_hz;
    float gain;
} GainTable_t;

#define GAIN_TABLE_SIZE 30
static const GainTable_t g_gain_table[GAIN_TABLE_SIZE] = {
    {100, 5.173f}, {200, 4.960f}, {300, 4.773f}, {400, 4.453f}, {500, 4.160f},
    {600, 3.867f}, {700, 3.600f}, {800, 3.280f}, {900, 3.120f}, {1000, 2.800f},
    {1100, 2.640f}, {1200, 2.480f}, {1300, 2.320f}, {1400, 2.160f}, {1500, 2.160f},
    {1600, 2.0000f}, {1700, 1.840f}, {1800, 1.840f}, {1900, 1.680f}, {2000, 1.680f},
    {2100, 1.520f}, {2200, 1.520f}, {2300, 1.467f}, {2400, 1.360f}, {2500, 1.360f},
    {2600, 1.253f}, {2700, 1.200f}, {2800, 1.200f}, {2900, 1.200f}, {3000, 1.147f}
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
        
        // 转换公式：W_A_param = amplitude_mV * 1023 / 6080
        uint32_t amp_code = (last_set_vpp_mv * 1023U) / 6080U;
        if (amp_code > 1023U) amp_code = 1023U;
        Write_Amplitude(0, (uint16_t)amp_code);
        AD9959_IO_Update();
        gFrequencyCmd.cmd = 0;
        gFrequencyCmd.frequency = 0;
      }
    }
    else if (g_dds_state == DDS_STATE_2)
    {
      /* 状态 2：简化补偿逻辑：DDS输出幅度 = 设定幅值 / 该频率对应的增益 */
      if (gFrequencyCmd.cmd == CMD_STATE1_CONTROL && (uint8_t)gFrequencyCmd.frequency == CMD_STATE_ON)
      {
        g_dds_state = DDS_STATE_1; // 55 04 01 FF 返回状态 1
        gFrequencyCmd.cmd = 0;
        gFrequencyCmd.frequency = 0;
      }
      else if (gFrequencyCmd.cmd == CMD_SET_FREQUENCY) // 0x01
      {
          uint32_t freq_val = gFrequencyCmd.frequency;
          float current_gain = 1.0f; 

          // 核心补偿算法：线性插值查找增益
          if (freq_val <= g_gain_table[0].freq_hz) {
              current_gain = g_gain_table[0].gain;
          } else if (freq_val >= g_gain_table[GAIN_TABLE_SIZE-1].freq_hz) {
              current_gain = g_gain_table[GAIN_TABLE_SIZE-1].gain;
          } else {
              for (int i = 0; i < GAIN_TABLE_SIZE - 1; i++) {
                  if (freq_val >= g_gain_table[i].freq_hz && freq_val <= g_gain_table[i+1].freq_hz) {
                      float ratio = (float)(freq_val - g_gain_table[i].freq_hz) / (float)(g_gain_table[i+1].freq_hz - g_gain_table[i].freq_hz);
                      current_gain = g_gain_table[i].gain + ratio * (g_gain_table[i+1].gain - g_gain_table[i].gain);
                      break;
                  }
              }
          }

          if (freq_val > 0 && current_gain > 0.001f)
          {
            // 简化逻辑：直接用当前的幅度码值(或者设定的mV值)除以增益
            // 此处假设用户在状态1设定的幅度存储在全局或直接根据当前码值计算
            // V_dds_mv = (User_Vpp / Gain) * Compensation_Factor
            // 将整体输出放大 1.25 倍（增益系数可根据实际测量调整）
            float compensation_factor = 3.335f; 
            float v_dds_mv = ((float)last_set_vpp_mv / current_gain) * compensation_factor;
            
            uint32_t amp_code = (uint32_t)(v_dds_mv * 1023.0f / 6080.0f);
            if (amp_code > 1023U) amp_code = 1023U;
            
            // 更新 Debug 变量
            debug_freq_val = freq_val;
            debug_amp_code = amp_code;
            
            Write_Frequence(0, freq_val);
            Write_Amplitude(0, (uint16_t)amp_code);
          }
          else
          {
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
          // 状态 2 下如果收到 0x02 指令，更新基准幅度，并立即应用一次当前频率下的补偿
          last_set_vpp_mv = gFrequencyCmd.frequency;
          
          // 获取当前频率（假定 DDS 当前输出频率）进行实时补偿更新
          // 简单处理：清除指令，等待下次频率设置或强制重算
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
