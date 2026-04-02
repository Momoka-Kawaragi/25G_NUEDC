/**
 * @file frequency_control.c
 * @brief 串口屏频率控制模块实现
 * @details 解析来自串口屏的频率设置数据包，控制AD9959发射机
 */

#include "frequency_control.h"
#include "ZPN_Hmi_Pack.h"
#include "ad9959.h"

/* 全局命令结构体，由PACK框架填充，供所有控制模块共享 */
FrequencyCommand_t gFrequencyCmd = {0};

/**
 * @brief 初始化频率控制模块
 * @details 设置PACK解析的变量模板
 * 
 * 数据包结构：
 * [帧头:0x55] [命令:1字节] [频率:4字节大端序] [校验:1字节] [帧尾:0xFF]
 * 
 * 模板变量映射：
 * - packVars[0]：cmd（1字节）
 * - packVars[1]：frequency（4字节）
 */
void FrequencyControl_Init(void)
{
    // 定义变量模板数组
    PackVariable_t vars[2];
    
    // 变量1：命令符（1字节）
    vars[0].type = PACK_TYPE_BYTE;
    vars[0].variable = (void *)&gFrequencyCmd.cmd;
    
    // 变量2：频率值（4字节大端序整数）
    vars[1].type = PACK_TYPE_INT;
    vars[1].variable = (void *)&gFrequencyCmd.frequency;
    
    // 将模板注册到PACK解析器
    PACK_SetTemplate(vars, 2);
}

/**
 * @brief 处理频率控制命令
 * @details 从UART2读取和解析数据包，根据命令执行相应操作
 * 
 * 工作流程：
 * 1. 调用PACK_ParseFromRingBuffer从环形缓冲区读取数据
 * 2. 当接收到完整数据包时，PACK_ParseFrame会自动填充gFrequencyCmd
 * 3. 检查命令符是否为设置频率命令
 * 4. 调用Write_Frequence更新AD9959的输出频率
 */
void FrequencyControl_Process(void)
{
    // 从UART2环形缓冲区解析数据包
    // 当找到完整的帧时会自动调用PACK_ParseFrame并填充gFrequencyCmd
    PACK_ParseFromRingBuffer();
    
    // 检查是否接收到有效的频率设置命令
    if (gFrequencyCmd.cmd == CMD_SET_FREQUENCY && gFrequencyCmd.frequency > 0)
    {
        // 调用AD9959的Write_Frequence函数设置频率
        // 第一个参数：通道编号（这里使用通道0）
        // 第二个参数：频率值（单位：Hz）
        Write_Frequence(0, gFrequencyCmd.frequency);
        
        // 只更新频率，不修改幅值、相位
        // 注意：不调用Write_Amplitude和Write_Phase，以保持现有设置
        AD9959_IO_Update();       // 更新IO
        
        // 清除命令标志，防止重复执行
        gFrequencyCmd.cmd = 0;
        gFrequencyCmd.frequency = 0;
    }
}

/**
 * @brief 手动设置AD9959频率
 * @param frequency: 目标频率（单位Hz）
 * @details 可以通过此函数直接设置频率，不依赖于串口接收
 */
void FrequencyControl_SetFrequency(uint32_t frequency)
{
    if (frequency > 0)
    {
        Write_Frequence(0, frequency);
        Write_Amplitude(0, 500);
        Write_Phase(0, 0);
        AD9959_IO_Update();
    }
}
