/**
 * @file amplitude_control.h
 * @brief 串口屏幅值控制模块
 * @details 解析来自串口屏的幅值设置数据包，控制AD9959发射机的输出幅值
 * 
 * 数据包格式：55 02 [amplitude_mV:4字节大端] [checksum] FF
 * - 55：帧头
 * - 02：命令符（0x02表示设置幅值）
 * - 00 00 13 88：4字节大端序数据（十进制5000表示5000mV）
 * - 05：校验码（所有数据字节的累加和）
 * - FF：帧尾
 * 
 * 幅值转换关系（线性标定）：
 * - Write_Amplitude(0, 1023) → 6080mV
 * - 转换公式：W_A_param = amplitude_mV * 1023 / 6080
 *   其中 amplitude_mV 是接收到的毫伏值
 * 
 * 示例：
 * - 接收3000mV：param ≈ 505
 * - 接收5000mV：param ≈ 841
 */

#ifndef __AMPLITUDE_CONTROL_H__
#define __AMPLITUDE_CONTROL_H__

#include <stdint.h>

/* 命令符定义 */
#define CMD_SET_AMPLITUDE    0x02    // 设置幅值命令

/* 数据包结构体 */
typedef struct {
    uint8_t cmd;         // 命令符
    uint32_t amplitude;  // 幅值数据（接收的原始数据，单位见下面说明）
} AmplitudeCommand_t;

/**
 * @brief 初始化幅值控制模块
 * @note 应在main中调用此函数来注册UART数据包模板
 */
void AmplitudeControl_Init(void);

/**
 * @brief 处理幅值控制命令
 * @details 从UART2读取数据包，解析幅值命令，并更新AD9959输出
 * @note 应在main loop中周期性调用
 */
void AmplitudeControl_Process(void);

/**
 * @brief 手动设置AD9959幅值
 * @param amplitude_mv: 目标幅值（单位mV，根据线性标定内部转换为Write_Amplitude参数）
 * @note 这是底层函数，调用Write_Amplitude来更新幅值
 * 
 * 转换关系说明：
 * - Write_Amplitude(0, 1023) 对应 6080mV
 * - 线性转换公式：param = (amplitude_mv * 1023) / 6080
 */
void AmplitudeControl_SetAmplitude(uint32_t amplitude_mv);

/* 全局观察变量：用于调试时在 Watch 中查看最近计算的 Write_Amplitude 参数 */
extern volatile uint32_t g_write_amplitude_param;
/* 全局观察变量：用于调试时在 Watch 中查看最近计算的 amplitude_mv（单位 mV） */
extern volatile uint32_t g_amplitude_mv;

#endif /* __AMPLITUDE_CONTROL_H__ */
