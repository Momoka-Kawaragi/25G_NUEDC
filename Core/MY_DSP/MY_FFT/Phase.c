/**
 ******************************************************************************
 * @file    Phase.c
 * @brief   相位计算与FFT算法源文件 - 适配STM32F407系列
 * @author  
 * @version V2.0
 * @date    2026-01-23
 ******************************************************************************
 * @attention
 *
 * 本文件实现了基于FFT算法的相位差计算功能，适用于STM32F407系列微控制器。
 * 主要功能：
 * 1. ADC信号采集
 * 2. 1024点FFT变换
 * 3. 信号处理与输出
 * 
 * @note 整合了主函数中的FFT逻辑，提供统一的调用接口。
 * 
 ******************************************************************************
 */

#include "Phase.h"
#include "math.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "si5351.h" // 引入Si5351驱动
#include <stdio.h>

/* 全局变量定义 */

/*
  * @brief ADC参考电压（单位：V）
  */
float32_t Reference_Voltage = 3.3f;

/**
 * @brief 采样率 (Hz)
 * @note  默认20kHz，可通过Phase_Set_SamplingRate动态修改
 */
float32_t SAMPLING_RATE = 40000.0f;

/**
 * @brief ADC采集完成标志位
 * @note  在ADC DMA传输完成回调函数中置1
 */
volatile uint8_t ADC_COMPLETED = 0;

/**
 * @brief ADC DMA 原始数据缓冲区
 * @note  存储ADC采集的原始数据
 */
uint16_t ADC_Buffer[FFT_LENGTH] = {0};

/**
 * @brief ADC采集标志位
 * @note  1表示采集完成，可以处理数据
 */
volatile uint8_t ADC_Flag = 0;

/**
 * @brief FFT转换所需的浮点数缓冲区
 * @note  用于存储转换后的电压值，作为FFT输入
 */
static float32_t adc_float_buffer[FFT_LENGTH];

/**
 * @brief FFT输入缓冲区（复数格式）
 * @note  格式：[实部0, 虚部0, 实部1, 虚部1, ...]
 */
float32_t FFT_InputBuf[FFT_LENGTH * 2] = {0.0f};

/**
 * @brief FFT输出缓冲区（模值）
 * @note  存储FFT运算后的幅值
 */
float32_t FFT_OutputBuf[FFT_LENGTH] = {0.0f};

/**
 * @brief FFT变换标志位
 */
uint8_t ifftFlag = 0;

/**
 * @brief 位反转标志位
 */
uint8_t doBitReverse = 1;

/**
 * @brief 窗函数变量
 */
float32_t window;

/**
 * @brief 窗函数系数表
 * @note  存储预计算的窗函数系数，避免实时计算开销
 */
float32_t Window_Coeffs[FFT_LENGTH];

/*
 * @brief adc_float_buffer缓冲区的观察工具
*/
float32_t g_adc_float_buffer[FFT_LENGTH];


/* 内部静态变量，用于控制输出频率 */
static const arm_cfft_instance_f32 *CFFT_Instance = NULL;
/**********************************************************************************************************/
/**
 * @brief  设置内部采样率 (TIM3)
 * @param  sampling_rate_hz: 目标采样率 (Hz)
 * @note   TIM3时钟源为84MHz。公式: Update_Freq = 84MHz / ((PSC+1)*(ARR+1))
 *         为了保证分辨率，固定PSC，动态计算ARR。
 */
void Phase_Set_SamplingRate_Internal(uint32_t sampling_rate_hz)
{
    if (sampling_rate_hz == 0) return;

    // 84MHz时钟
    uint32_t tim_clock = 84000000;
    uint32_t psc = 0;
    uint32_t arr = 0;

    // 自动计算PSC和ARR
    // 目标: 找到合适的PSC，使得ARR在16位范围内 (0-65535)
    // 策略: 优先PSC=0 (高精度)，若溢出则增加PSC
    for (psc = 0; psc < 65536; psc++)
    {
        uint32_t target_arr = (tim_clock / (sampling_rate_hz * (psc + 1))) - 1;
        if (target_arr < 65536)
        {
            arr = target_arr;
            break;
        }
    }

    // 更新全局变量
    SAMPLING_RATE = (float32_t)tim_clock / ((psc + 1) * (arr + 1));

    // 修改寄存器
    /* Defensive checks: ensure TIM handle is valid */
    if (htim3.Instance == NULL) {
      return;
    }

    /* If ADC DMA is running, stop it before changing timer registers to avoid
       unintended update-triggered conversions or DMA races. Safe to call
       even if ADC isn't started. */
    HAL_ADC_Stop_DMA(&hadc1);
  #if USE_DUAL_ADC == 1
    HAL_ADC_Stop_DMA(&hadc2);
  #endif

    __HAL_TIM_SET_PRESCALER(&htim3, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);

    /* Note: generating an update may trigger an ADC conversion depending on
       the trigger wiring/config; prefer to configure while ADC is stopped. */
    // __HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);

    /* Avoid blocking printf here during early bring-up; if you need logging,
       use a non-blocking DMA transmit helper or re-enable this after
       confirming UART is stable. */
    // printf("TIM3 Config Updated: Fs=%.1fHz, PSC=%d, ARR=%d\r\n", SAMPLING_RATE, psc, arr);
}

/**
 * @brief  设置外部采样率 (Si5351)
 * @param  sampling_rate_hz: 目标采样率 (Hz)
 * @note   设置Si5351 CLK0输出指定频率，需外部连线触发ADC
 */
void Phase_Set_SamplingRate_External(uint32_t sampling_rate_hz)
{
    if (sampling_rate_hz == 0) return;

    // 初始化Si5351 (需确保I2C已初始化)
    // 注意: 如果si5351_Init()内部包含耗时操作，建议只在第一次调用
    static uint8_t is_si5351_init = 0;
    if (!is_si5351_init)
    {
        si5351_Init();
        is_si5351_init = 1;
    }

    // 设置CLK0频率
    // 假设SI5351驱动接受的单位是Hz，且函数名为SI5351_SetFrequency
    // 这里需根据实际驱动API调整
    // 假设 SI5351_SetFrequency(channel, freq_hz)
    // 注意: 原main.c中宏定义MHZ(x)是 x*1000000，这里直接传Hz
    SI5351_SetChannelPower(0, 1); // 开启CLK0
    SI5351_SetFrequency(0, sampling_rate_hz); 

    // 更新全局变量
    SAMPLING_RATE = (float32_t)sampling_rate_hz;
    
    printf("Si5351 Config Updated: Fs=%.1fHz (CLK0)\r\n", SAMPLING_RATE);
}

/**
 * @brief  统一设置采样率接口
 * @param  sampling_rate_hz: 目标采样率 (Hz)
 */
void Phase_Set_SamplingRate(uint32_t sampling_rate_hz)
{
#if ADC_TRIGGER_SOURCE == TRIGGER_SOURCE_EXTERNAL_SI5351
    Phase_Set_SamplingRate_External(sampling_rate_hz);
#else
    Phase_Set_SamplingRate_Internal(sampling_rate_hz);
#endif
}

/* 双ADC相关变量（保留以兼容现有逻辑，若未使用可移除） */
uint32_t ADC_Raw_Data[FFT_LENGTH] = {0};
uint16_t ADC_1_Value_DMA[FFT_LENGTH] = {0};
uint16_t ADC_2_Value_DMA[FFT_LENGTH] = {0};
float32_t ADC_1_Real_Value[FFT_LENGTH] = {0.0f};
float32_t ADC_2_Real_Value[FFT_LENGTH] = {0.0f};

/**
 * @brief  初始化窗函数系数
 * @note   默认使用汉宁窗(Hanning)
 */
void Phase_Window_Init(void)
{
    // 默认初始化为汉宁窗
    hannWin(FFT_LENGTH, Window_Coeffs);
}

/**
 * @brief  FFT应用初始化
 * @retval None
 * @note   初始化定时器和启动首次ADC DMA采集
 */
void FFT_App_Init(void)
{
    // 初始化窗函数
    Phase_Window_Init();

    // 根据宏定义选择FFT实例
    #if FFT_LENGTH == 64
        CFFT_Instance = &arm_cfft_sR_f32_len64;
    #elif FFT_LENGTH == 128
        CFFT_Instance = &arm_cfft_sR_f32_len128;
    #elif FFT_LENGTH == 256
        CFFT_Instance = &arm_cfft_sR_f32_len256;
    #elif FFT_LENGTH == 512
        CFFT_Instance = &arm_cfft_sR_f32_len512;
    #elif FFT_LENGTH == 1024
        CFFT_Instance = &arm_cfft_sR_f32_len1024;
    #elif FFT_LENGTH == 2048
        CFFT_Instance = &arm_cfft_sR_f32_len2048;
    #elif FFT_LENGTH == 4096
        CFFT_Instance = &arm_cfft_sR_f32_len4096;
    #else
        #error "Unsupported FFT Length! Please check Phase.h"
    #endif

    // 启动定时器（用于触发ADC）
    HAL_TIM_Base_Start(&htim3);
    
    // 启动ADC DMA采集
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_1_Value_DMA, FFT_LENGTH);
#if USE_DUAL_ADC == 1
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC_2_Value_DMA, FFT_LENGTH);
#endif
}

/**
 * @brief  FFT应用主处理函数
 * @retval None
 * @note   需在主循环(while(1))中持续调用。
 *         负责检查采集状态、数据转换、FFT运算及结果输出。
 */
void FFT_App_Process(void)
{
    if(ADC_Flag)
    {
        // 清除标志位，准备下一次采集
        ADC_Flag = 0;
        
        // 1. 数据预处理：将ADC原始值转换为电压值（浮点）
        // 默认处理ADC1
        for(int i = 0; i < FFT_LENGTH; i++)
        {
            adc_float_buffer[i] = (float32_t)ADC_1_Value_DMA[i] / 4096.0f * 3.3f;
            g_adc_float_buffer[i] = adc_float_buffer[i]; // 更新调试别名
            // 如果需要加窗，可以在此处调用 Apply_Hanning_Window
            // Apply_Hanning_Window(adc_float_buffer, FFT_LENGTH); 
        }

        // 2. 执行FFT变换
        // 注意：arm_cfft_f32_app 内部会处理 FFT_InputBuf 和 FFT_OutputBuf
        arm_cfft_f32_app(adc_float_buffer, CFFT_Instance);
        
        // 3. 结果处理与输出逻辑
#ifdef FFT_OUTPUT_FULL_SPECTRUM
        
    #ifdef FFT_OUTPUT_BINARY
        // [实时模式] 使用 VOFA+ JustFloat 协议发送二进制数据
        // 格式: [float data...] + [0x00 0x00 0x80 0x7f]
        // 优点: 极快，不阻塞，适合高刷新率
        
        // 仅发送前 512 个点 (对称频谱)
        // 注意: 直接发送浮点数组，最后追加 VOFA+ 尾帧
        static float32_t send_buf[FFT_LENGTH/2 + 1]; // +1用于尾帧(虽然尾帧是4字节，float也是4字节，刚好)
        
        // 拷贝前512个点
        memcpy(send_buf, FFT_OutputBuf, (FFT_LENGTH/2) * sizeof(float32_t));
        
        // 追加尾帧 0x7F800000 (小端序: 00 00 80 7F)
        // 这是浮点数的 NaN (Not a Number)，VOFA+ 用作帧结束符
        uint32_t tail = 0x7F800000; 
        memcpy(&send_buf[FFT_LENGTH/2], &tail, 4);
        
        // 使用 DMA 发送 (非阻塞)
        // 如果串口忙（上一帧还没发完），则跳过本次发送，保证系统实时性
        UART1_DMASendData((uint8_t*)send_buf, (FFT_LENGTH/2 + 1) * sizeof(float32_t));

    #else
        // [兼容模式] ASCII 文本输出
        // 缺点: 速度慢，可能阻塞
        static uint32_t last_print_tick = 0;
        if(HAL_GetTick() - last_print_tick > 1000) // 限制刷新率，防止卡死
        {
            last_print_tick = HAL_GetTick();
            printf("/*Start*/\n"); 
            for(int i=0; i < FFT_LENGTH/2; i++)
            {
                printf("%.3f\n", FFT_OutputBuf[i]);
            }
            printf("/*End*/\n");
        }
    #endif

#else
        // 简要信息输出 (调试助手查看)
        static uint32_t last_print_tick = 0;
        if(HAL_GetTick() - last_print_tick > 500)
        {
             last_print_tick = HAL_GetTick();
             // 查找最大幅值对应的频点
             int max_idx = Find_nMax(FFT_OutputBuf);
             // 计算频率: F = i * Fs / N
             float32_t freq = (float32_t)max_idx * SAMPLING_RATE / FFT_LENGTH;
             float32_t mag = FFT_OutputBuf[max_idx];
             
             // printf("Peak Freq: %.1f Hz, Mag: %.3f V\n", freq, mag);
        }
#endif
        
        // 4. 重新启动ADC DMA采集
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_1_Value_DMA, FFT_LENGTH);
#if USE_DUAL_ADC == 1
        HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC_2_Value_DMA, FFT_LENGTH);
#endif
    }
}

/**
 * @brief  打印ADC原始数据
 * @note   用于调试波形，替代FFT处理。需在主循环调用。
 */
void FFT_Print_RawData(void)
{
    if(ADC_Flag)
    {
        // 清除标志位，准备下一次采集
        ADC_Flag = 0;

        // 仅打印原始值，不进行FFT运算
        static uint32_t last_print_tick = 0;
        if(HAL_GetTick() - last_print_tick > 1000) // 每1秒打印一次，避免数据量过大阻塞
        {
            last_print_tick = HAL_GetTick();
            printf("/*WaveStart*/\n"); // 波形开始标记
            for(int i = 0; i < FFT_LENGTH; i++)
            {
                // 打印格式：索引, 原始ADC值(0-4095)
                printf("%d\r\n", ADC_1_Value_DMA[i]);
            }
            printf("/*WaveEnd*/\n"); // 波形结束标记
        }
        
        // 重新启动ADC DMA采集
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_1_Value_DMA, FFT_LENGTH);
#if USE_DUAL_ADC == 1
        HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC_2_Value_DMA, FFT_LENGTH);
#endif
    }
}


/**
 * @brief  FFT变换核心函数
 * @param  rawData: 输入实数数组
 * @param  fft_instance: FFT配置实例
 * @retval None
 */
void arm_cfft_f32_app(float32_t *rawData, const arm_cfft_instance_f32 *fft_instance)
{
  uint16_t n;
  uint16_t fftLen = fft_instance->fftLen;
  
  /* 1. 构建复数输入数组 */
  for (n = 0; n < fftLen; n++)
  {
    FFT_InputBuf[2 * n] = rawData[n];      // 实部
    FFT_InputBuf[2 * n + 1] = 0.0f;        // 虚部为0
  }
  
  /* 2. 执行FFT变换 */
  arm_cfft_f32(fft_instance, FFT_InputBuf, ifftFlag, doBitReverse);
  
  /* 3. 计算模值 */
  arm_cmplx_mag_f32(FFT_InputBuf, FFT_OutputBuf, fftLen);
}


/**
 * @brief  应用汉宁窗 (实际上应用当前配置的窗函数)
 * @param  signal: 信号数组
 * @param  length: 长度
 */
void Apply_Hanning_Window(float32_t *signal, uint16_t length)
{
    // 使用预计算的窗函数系数，效率更高
    Window_Apply(signal, Window_Coeffs, length);
}


/* 以下为原有双ADC相位计算相关函数，保留备用 */

void PhaseCalculate_ADC_Init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2)
{
#ifdef USE_ADC_CALIBRATION
  HAL_ADCEx_Calibration_Start(hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(hadc2, ADC_SINGLE_ENDED);
#endif
  HAL_ADC_Start(hadc2);
  HAL_ADC_Start_DMA(hadc1, (uint32_t *)ADC_1_Value_DMA, FFT_LENGTH);
  HAL_ADC_Start_DMA(hadc2, (uint32_t *)ADC_2_Value_DMA, FFT_LENGTH);
}

int Find_nMax(const float *ARR)
{
  if (ARR == NULL) return -1;
  float aMax = ARR[1];
  uint32_t nMax = 1;
  for (uint32_t i = 2; i < FFT_LENGTH / 2; i++)
  {
    if (ARR[i] > aMax)
    {
      aMax = ARR[i];
      nMax = i;
    }
  }
  return nMax;
}

float32_t Find_PhaseAngle(float32_t *signal)
{
  /* 确保使用正确的FFT实例 */
  const arm_cfft_instance_f32 *instance;
  #if FFT_LENGTH == 64
      instance = &arm_cfft_sR_f32_len64;
  #elif FFT_LENGTH == 128
      instance = &arm_cfft_sR_f32_len128;
  #elif FFT_LENGTH == 256
      instance = &arm_cfft_sR_f32_len256;
  #elif FFT_LENGTH == 512
      instance = &arm_cfft_sR_f32_len512;
  #elif FFT_LENGTH == 1024
      instance = &arm_cfft_sR_f32_len1024;
  #elif FFT_LENGTH == 2048
      instance = &arm_cfft_sR_f32_len2048;
  #elif FFT_LENGTH == 4096
      instance = &arm_cfft_sR_f32_len4096;
  #else
      instance = &arm_cfft_sR_f32_len1024; // 默认
  #endif

  arm_cfft_f32_app(signal, instance);
  int n_max = Find_nMax(FFT_OutputBuf);
  float32_t phase_rad = atan2f(FFT_InputBuf[2 * n_max + 1], FFT_InputBuf[2 * n_max]);
  return phase_rad * 180.0f / PI;
}

void Process_ADC_RawData(void)
{
  // if (ADC_COMPLETED) // 已经在外部等待了
  {
    ADC_COMPLETED = 0;
    for (int i = 0; i < FFT_LENGTH; i++)
    {
      uint16_t adc1 = ADC_1_Value_DMA[i];
      ADC_1_Real_Value[i] = ((float32_t)adc1 / 4095.0f) * Reference_Voltage;
#if USE_DUAL_ADC == 1
      uint16_t adc2 = ADC_2_Value_DMA[i];
      ADC_2_Real_Value[i] = ((float32_t)adc2 / 4095.0f) * Reference_Voltage;
#endif
    }
  }
}

float32_t Get_PhaseDifference(void)
{
  float32_t phase1, phase2, phase_diff;

  // 触发一次新的采集
  ADC_COMPLETED = 0;
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_1_Value_DMA, FFT_LENGTH);
#if USE_DUAL_ADC == 1
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC_2_Value_DMA, FFT_LENGTH);
#endif
  
  // 等待数据采集完成（带有超时机制）
  // 1024点 @ 20kHz 约需 51.2ms。使用HAL_GetTick更准确，超时设为100ms
  uint32_t start_tick = HAL_GetTick();
  while(!ADC_COMPLETED) 
  {
      if((HAL_GetTick() - start_tick) > 100) 
      {
          printf("ADC Timeout! (Flag not set within 100ms)\n");
          
          // 尝试强制重启，防止永久卡死
          ADC_Flag = 0;
          HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_1_Value_DMA, FFT_LENGTH);
          HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC_2_Value_DMA, FFT_LENGTH);
          
          return 0.0f; // 超时返回
      }
  }

  Process_ADC_RawData();
  
  phase1 = Find_PhaseAngle(ADC_1_Real_Value);
#if USE_DUAL_ADC == 1
  phase2 = Find_PhaseAngle(ADC_2_Real_Value);
  phase_diff = phase1 - phase2;
  if (phase_diff > 180.0f) phase_diff -= 360.0f;
  else if (phase_diff < -180.0f) phase_diff += 360.0f;
#else
  phase2 = 0.0f;
  phase_diff = 0.0f;
  printf("Warning: Single ADC Mode, Phase Diff is invalid.\r\n");
#endif
  
  // 重新启动采集 (假设在 Process_ADC_RawData 或回调中没有自动重启)
  // 如果是单次模式，需要在这里重启
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_1_Value_DMA, FFT_LENGTH);
#if USE_DUAL_ADC == 1
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC_2_Value_DMA, FFT_LENGTH);
#endif
  
  return phase_diff;
}

/**
 * @brief  ADC转换完成回调函数
 * @param  hadc: ADC句柄指针
 * @note   由HAL库中断处理函数调用
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
      HAL_ADC_Stop_DMA(hadc);
      ADC_Flag=1;
  }
  
  // 双ADC逻辑保留
  static uint8_t adc1_complete = 0;
  static uint8_t adc2_complete = 0;
  
  if (hadc->Instance == ADC1) adc1_complete = 1;
#if USE_DUAL_ADC == 1
  else if (hadc->Instance == ADC2) adc2_complete = 1;
#else
  // 单ADC模式下，ADC2完成标志始终为1，以便逻辑统一
  adc2_complete = 1;
#endif
    
  if (adc1_complete && adc2_complete)
  {
    ADC_COMPLETED = 1;
    adc1_complete = 0;
    adc2_complete = 0;
  }
}

uint16_t Get_FFT_Spectrum(float32_t* buffer, uint16_t length)
{
  uint16_t copy_length = (length < FFT_LENGTH) ? length : FFT_LENGTH;
  memcpy(buffer, FFT_OutputBuf, copy_length * sizeof(float32_t));
  return copy_length;
}

void Set_Reference_Voltage(float32_t voltage)
{
  if (voltage > 0.0f && voltage <= 5.0f)
  {
    Reference_Voltage = voltage;
  }
}

float32_t Get_Reference_Voltage(void)
{
  return Reference_Voltage;
}

void ADC_Signal_Collect_To_ADC_Buffer(void)
{
    // 此函数逻辑已整合到 FFT_App_Process 中，保留此空函数或重定向，以防其他地方调用
    // 或者直接在这里调用 Process
    // 但为避免递归或混乱，建议弃用此函数，改用 FFT_App_Process
}
