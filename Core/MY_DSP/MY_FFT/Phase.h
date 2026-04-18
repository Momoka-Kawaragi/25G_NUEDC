/**
 ******************************************************************************
 * @file    Phase.h
 * @brief   相位计算与FFT算法头文件 - 适配STM32F407系列
 * @author  
 * @version V2.0
 * @date    2026-01-23
 ******************************************************************************
 */

#ifndef __PHASE_H
#define __PHASE_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含必要的头文件 */
#include "main.h"           // STM32 HAL库头文件
#include "arm_math.h"       // CMSIS DSP数学库
#include "arm_const_structs.h"  // CMSIS FFT常量结构体
#include <string.h>         // 包含memcpy等函数
#include <stdio.h>
#include <math.h>
#include "stm32f4xx.h"
#include "WindowFunction.h" // 引入窗函数库

/* 宏定义 */

/**
 * @brief FFT变换长度
 * @note  支持64, 128, 256, 512, 1024, 2048, 4096
 */
#define FFT_LENGTH          2048

/**
 * @brief  系统配置宏
 */
#define USE_DUAL_ADC        0       // 1: 双ADC模式, 0: 单ADC模式
#define ADC_TRIGGER_SOURCE  0       // 0: 内部TIM3, 1: 外部Si5351

#define TRIGGER_SOURCE_INTERNAL_TIM3 0
#define TRIGGER_SOURCE_EXTERNAL_SI5351 1

/**
 * @brief 采样率 (Hz)
 * @note  此变量现在为动态可调，初始值仅为参考
 */
extern float32_t SAMPLING_RATE;


/* 测试宏定义 (取消注释以启用) */
// #define FFT_TEST_SIMULATION       // 启用内部模拟信号生成(无需外部信号源)
// #define FFT_OUTPUT_FULL_SPECTRUM  // 启用全频谱输出
// #define FFT_OUTPUT_BINARY         // 启用二进制(JustFloat)格式输出，需配合VOFA+使用


/* 外部变量声明 */

extern float32_t Reference_Voltage;
extern volatile uint8_t ADC_COMPLETED;
extern uint32_t ADC_Raw_Data[FFT_LENGTH];
extern uint8_t ifftFlag;
extern uint8_t doBitReverse;
extern uint16_t ADC_1_Value_DMA[FFT_LENGTH];
extern uint16_t ADC_2_Value_DMA[FFT_LENGTH];
extern float32_t ADC_1_Real_Value[FFT_LENGTH];
extern float32_t ADC_2_Real_Value[FFT_LENGTH];
extern float32_t FFT_InputBuf[FFT_LENGTH * 2];
extern float32_t FFT_OutputBuf[FFT_LENGTH];
extern uint16_t ADC_Buffer[FFT_LENGTH];
extern volatile uint8_t ADC_Flag;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern float32_t window;
extern float32_t g_adc_float_buffer[FFT_LENGTH];



/* 函数声明 */

/**
 * @brief  FFT应用初始化
 * @note   在主函数初始化阶段调用
 */
void FFT_App_Init(void);

/**
 * @brief  初始化窗函数系数
 * @note   默认使用汉宁窗(Hanning)
 */
void Phase_Window_Init(void);

/**
 * @brief  FFT应用主处理函数
 * @note   在主循环中调用
 */
void FFT_App_Process(void);

/**
 * @brief  打印ADC原始数据
 * @note   用于调试波形，替代FFT处理。需在主循环调用。
 */
void FFT_Print_RawData(void);

/**
 * @brief  设置采样率
 * @param  sampling_rate_hz: 目标采样率 (Hz)
 */
void Phase_Set_SamplingRate(uint32_t sampling_rate_hz);

/**
 * @brief  设置内部采样率 (TIM3)
 * @param  sampling_rate_hz: 目标采样率 (Hz)
 */
void Phase_Set_SamplingRate_Internal(uint32_t sampling_rate_hz);

/**
 * @brief  设置外部采样率 (Si5351)
 * @param  sampling_rate_hz: 目标采样率 (Hz)
 */
void Phase_Set_SamplingRate_External(uint32_t sampling_rate_hz);


/* 兼容旧接口函数 */
void PhaseCalculate_ADC_Init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);
void Process_ADC_RawData(void);
float32_t Get_PhaseDifference(void);
int Find_nMax(const float *ARR);
float32_t Find_PhaseAngle(float32_t *signal);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void CpltCallback(ADC_HandleTypeDef* hadc);
uint16_t Get_FFT_Spectrum(float32_t* buffer, uint16_t length);
void Set_Reference_Voltage(float32_t voltage);
float32_t Get_Reference_Voltage(void);
void ADC_Signal_Collect_To_ADC_Buffer(void);
void arm_cfft_f32_app(float32_t *rawData, const arm_cfft_instance_f32 *fft_instance);
void Apply_Hanning_Window(float32_t *signal, uint16_t length);

/* 内联函数 */

/**
 * @brief  判断FFT长度是否有效
 */
static inline uint8_t Is_FFT_Length_Valid(uint16_t length)
{
  return (length & (length - 1)) == 0;
}

/**
 * @brief  将ADC值转换为电压值
 */
static inline float32_t ADC_Value_To_Voltage(uint16_t adc_value)
{
  return ((float32_t)adc_value / 4095.0f) * Reference_Voltage;
}

/**
 * @brief  将电压值转换为ADC值
 */
static inline uint16_t Voltage_To_ADC_Value(float32_t voltage)
{
  return (uint16_t)((voltage / Reference_Voltage) * 4095.0f);
}

#ifdef __cplusplus
}
#endif

#endif /* __PHASE_H */
