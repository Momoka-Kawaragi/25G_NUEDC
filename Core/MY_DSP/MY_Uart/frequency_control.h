/**
 * @file frequency_control.h
 * @brief 串口屏频率控制模块
 * @details 解析来自串口屏的频率设置数据包，控制AD9959发射机的输出频率
 * 
 * 数据包格式：55 01 00 00 03 E8 EC FF
 * - 55：帧头
 * - 01：命令符（0x01表示设置频率）
 * - 00 00 03 E8：4字节大端序数据（十六进制3E8=十进制1000）
 * - EC：校验码（所有数据字节的累加和）
 * - FF：帧尾
 */

#ifndef __FREQUENCY_CONTROL_H__
#define __FREQUENCY_CONTROL_H__

#include <stdint.h>

/* 命令符定义 */
#define CMD_SET_FREQUENCY    0x01    // 设置频率命令

/* 数据包结构体 */
typedef struct {
    uint8_t cmd;        // 命令符
    uint32_t frequency; // 频率值（Hz）
} FrequencyCommand_t;

/**
 * @brief 导出全局命令结构体，供其他控制模块共享使用
 * @note 由FrequencyControl_Init注册到PACK框架
 *       - frequency_control.c 用于处理频率命令（cmd==0x01）
 *       - amplitude_control.c 用于处理幅值命令（cmd==0x02）
 */
extern FrequencyCommand_t gFrequencyCmd;

/**
 * @brief 初始化频率控制模块
 * @note 应在main中调用此函数来注册UART数据包模板
 */
void FrequencyControl_Init(void);

/**
 * @brief 处理频率控制命令
 * @details 从UART2读取数据包，解析频率命令，并更新AD9959输出
 * @note 应在main loop中周期性调用
 */
void FrequencyControl_Process(void);

/**
 * @brief 手动设置AD9959频率
 * @param frequency: 目标频率（单位Hz）
 * @note 这是底层函数，调用Write_Frequence来更新频率
 */
void FrequencyControl_SetFrequency(uint32_t frequency);

#endif /* __FREQUENCY_CONTROL_H__ */
