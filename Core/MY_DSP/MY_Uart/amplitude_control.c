/**
 * @file amplitude_control.c
 * @brief 串口屏幅值控制模块实现
 * @details 解析来自串口屏的幅值设置数据包，控制AD9959发射机
 * 
 * 架构说明：与frequency_control共享同一个PACK框架和全局cmd/data变量
 * 不需要自己的gAmplitudeCmd，直接使用frequency_control中的gFrequencyCmd
 */

#include "amplitude_control.h"
#include "frequency_control.h"  // ← 导入以访问gFrequencyCmd
#include "ZPN_Hmi_Pack.h"
#include "ad9959.h"

/* 全局观察变量定义（volatile 可防优化） */
volatile uint32_t g_write_amplitude_param = 0;
/* 记录最近一次转换得到的 amplitude_mv（单位：mV），便于调试观察 */
volatile uint32_t g_amplitude_mv = 0;

// 记住：这里不定义gAmplitudeCmd
// 幅值命令直接使用频率模块的全局变量，通过cmd值来区分

/**
 * @brief 幅值线性转换：毫伏(mV) → Write_Amplitude参数
 * @details 根据最新标定关系转换
 *          已知：Write_Amplitude(0, 1023) → 6080mV
 *          公式：param = amplitude_mV * 1023 / 6080（四舍五入）
 *
 * @param amplitude_mv: 输入幅值（单位：mV）
 * @return uint32_t: 对应的Write_Amplitude参数
 */
static uint32_t AmplitudeVoltageToParam(uint32_t amplitude_mv)
{
    // 将输入毫伏值写入观察变量，便于调试时在 Watch 中查看
    g_amplitude_mv = amplitude_mv;

    // 线性映射并做四舍五入，避免纯截断带来的系统偏差
    uint64_t scaled = (uint64_t)amplitude_mv * 1023ULL + 3040ULL;
    uint32_t write_amplitude_param = (uint32_t)(scaled / 6080ULL);

    // AD9959 幅值码上限 1023，超限时直接钳位
    if (write_amplitude_param > 1023U)
    {
        write_amplitude_param = 1023U;
    }

    return write_amplitude_param;
}

/**
 * @brief 初始化幅值控制模块
 * @details 注意：与频率控制共享PACK解析框架
 *          - 频率控制负责注册PACK模板（在main.c中先初始化频率控制）
 *          - 幅值控制通过cmd来区分数据类型
 *          - 两个模块都使用相同的cmd变量，根据不同值来判断
 * 
 * 重要：务必在main.c中按以下顺序初始化：
 * 1. FrequencyControl_Init()  // 注册PACK模板
 * 2. AmplitudeControl_Init()  // 共享模板，不再注册
 */
void AmplitudeControl_Init(void)
{
    // 幅值控制与频率控制共享PACK框架
    // PACK模板由FrequencyControl_Init在main.c中注册
    // 此函数仅作为初始化占位符
    
    // 注意：gAmplitudeCmd在本模块中作为接收缓冲使用
    // 实际解析结果为：
    // - cmd 由PACK_ParseFrame填充（同步的）
    // - amplitude 由PACK_ParseFrame填充（同步的）
}

/**
 * @brief 处理幅值控制命令
 * @details 从UART2读取和解析数据包，根据命令执行相应操作
 * 
 * 工作流程：
 * 1. gFrequencyCmd由PACK_ParseFrame填充（函数名保留历史，实际是通用cmd结构）
 * 2. 检查gFrequencyCmd.cmd是否为0x02（幅值命令）
 * 3. 使用gFrequencyCmd.frequency字段存储幅值数据
 * 4. 调用AmplitudeVoltageToParam转换为Write_Amplitude参数
 * 5. 调用Write_Amplitude更新AD9959的输出幅值
 */
void AmplitudeControl_Process(void)
{
    // 从UART2环形缓冲区解析数据包
    // PACK_ParseFrame会填充全局的gFrequencyCmd（虽然名字是frequency，但实际是通用的）
    PACK_ParseFromRingBuffer();
    
    // 检查是否接收到有效的幅值设置命令
    // 注意：使用gFrequencyCmd.frequency字段来存储幅值数据
    if (gFrequencyCmd.cmd == CMD_SET_AMPLITUDE && gFrequencyCmd.frequency > 0)
    {
        // 调用线性转换函数，将电压值转换为Write_Amplitude参数
        uint32_t write_amplitude_param = AmplitudeVoltageToParam(gFrequencyCmd.frequency);
        // 写入观察变量，便于调试时在 Watch 中查看
        g_write_amplitude_param = write_amplitude_param;
        
        // 调用AD9959的Write_Amplitude函数设置幅值
        // 第一个参数：通道编号（这里使用通道0）
        // 第二个参数：幅值参数（经过线性转换）
        Write_Amplitude(0, write_amplitude_param);
        
        // 只更新幅值，不修改频率、相位
        // 注意：不调用Write_Frequence和Write_Phase，以保持现有设置
        AD9959_IO_Update();     // 更新IO，使幅值改变生效
        
        // 清除命令标志，防止重复执行
        gFrequencyCmd.cmd = 0;
        gFrequencyCmd.frequency = 0;
    }
}

/**
 * @brief 手动设置AD9959幅值
 * @param amplitude_mv: 目标幅值（单位mV）
 * @details 可以通过此函数直接设置幅值，不依赖于串口接收
 * 
 * 示例：
 *   AmplitudeControl_SetAmplitude(3000);   // 设置幅值为3000mV
 *   AmplitudeControl_SetAmplitude(5000);   // 设置幅值为5000mV
 */
void AmplitudeControl_SetAmplitude(uint32_t amplitude_mv)
{
    if (amplitude_mv > 0)
    {
        uint32_t write_amplitude_param = AmplitudeVoltageToParam(amplitude_mv);
        // 写入观察变量，便于调试时在 Watch 中查看
        g_write_amplitude_param = write_amplitude_param;
        Write_Amplitude(0, write_amplitude_param);
        Write_Frequence(0, 0);
        Write_Phase(0, 0);
        AD9959_IO_Update();
    }
}
