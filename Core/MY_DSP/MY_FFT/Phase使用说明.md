# 相位计算库使用说明文档

本文档旨在说明如何使用 `Phase.c` 和 `Phase.h` 文件进行基于 FFT 的双通道相位差测量。该库适用于 STM32F407 系列，利用 CMSIS-DSP 库进行 FFT 运算。

## 1. 核心功能

本库主要提供以下功能：
1.  **双通道 ADC 数据采集**：利用 DMA 自动搬运 ADC1 和 ADC2 的数据。
2.  **FFT 频谱分析**：对采集到的时域信号进行快速傅里叶变换，提取频域特征。
3.  **相位差计算**：通过分析两个通道主频分量的相位角，计算它们之间的相位差。
4.  **频率与幅值检测**：识别信号的主要频率成分及其幅值。
5.  **动态采样率调节**：支持内部 TIM3 动态调频和 Si5351 外部时钟源。
6.  **单/双 ADC 模式切换**：支持配置仅使用 ADC1 或同时使用 ADC1/ADC2。

## 2. 关键配置 (`Phase.h`)

在使用前，请根据实际需求检查以下宏定义：

### 2.1 基础配置
*   **`FFT_LENGTH`**: FFT 点数。推荐使用 **1024** 或 **256**。
    *   点数越多，频率分辨率越高，但计算耗时越长，刷新率越低。
    *   必须是 2 的幂次方 (64, 128, 256, 512, 1024...)。
*   **`FFT_OUTPUT_FULL_SPECTRUM`**: 定义是否输出全频谱数据到串口。
    *   若定义，则 `FFT_App_Process` 会打印所有频谱数据。
    *   若注释掉，则仅打印主频和幅值等简要信息。

### 2.2 模式选择宏
*   **`USE_DUAL_ADC`**: ADC 工作模式选择。
    *   `1`: 双 ADC 模式 (ADC1 + ADC2)，可测相位差。
    *   `0`: 单 ADC 模式 (仅 ADC1)，仅测频率/幅值，相位差无效。
*   **`ADC_TRIGGER_SOURCE`**: 采样触发源选择。
    *   `TRIGGER_SOURCE_INTERNAL_TIM3` (0): 使用内部 TIM3 触发，软件可调分频。
    *   `TRIGGER_SOURCE_EXTERNAL_SI5351` (1): 使用外部 Si5351 模块产生的时钟触发 (需硬件连线)。

## 3. 主要函数接口

### 3.1 初始化与处理
*   **`void FFT_App_Init(void)`**: 初始化 FFT 应用（定时器、DMA、ADC）。
*   **`void FFT_App_Process(void)`**: 状态机处理函数，在主循环调用，用于非阻塞式处理和数据打印。
*   **`float32_t Get_PhaseDifference(void)`**: 阻塞式获取相位差（单次测量）。

### 3.2 动态采样率调节
*   **`void Phase_Set_SamplingRate(uint32_t sampling_rate_hz)`**
    *   **功能**：统一设置采样率接口。
    *   **行为**：根据 `ADC_TRIGGER_SOURCE` 宏自动调用内部或外部设置函数。
*   **`void Phase_Set_SamplingRate_Internal(uint32_t sampling_rate_hz)`**
    *   **功能**：修改 TIM3 的 Prescaler 和 Period 来改变触发频率。
    *   **注意**：基准时钟为 84MHz，会自动计算最优分频参数。
*   **`void Phase_Set_SamplingRate_External(uint32_t sampling_rate_hz)`**
    *   **功能**：设置 Si5351 通道 0 (CLK0) 输出指定频率。
    *   **硬件连接**：需将 Si5351 的 CLK0 引脚连接到 STM32 的 ADC 外部触发引脚 (TIM3_ETR -> TRGO)。

## 4. 使用示例 (`main.c`)

### 4.1 基础用法
```c
/* USER CODE BEGIN 2 */
Init_AD9959();
FFT_App_Init();
/* USER CODE END 2 */

while (1)
{
    float32_t phase_diff = Get_PhaseDifference();
    printf("Phase Diff: %.2f degree\n", phase_diff);
    HAL_Delay(500);
}
```

### 4.2 动态修改采样率
```c
// 设置采样率为 40kHz (内部TIM3模式)
Phase_Set_SamplingRate(40000); 

// 或者在运行时切换
if (user_button_pressed) {
    Phase_Set_SamplingRate(10000); // 切换到 10kHz
}
```

## 5. 常见问题与注意事项

1.  **外部时钟模式连线**：
    *   如果选择了 `TRIGGER_SOURCE_EXTERNAL_SI5351`，仅仅调用函数是不够的，必须确保硬件上 Si5351 的输出引脚连接到了 STM32 的定时器外部触发引脚 (ETR)，并且 CubeMX 中 TIM3 的 Slave Mode 配置为 External Clock Mode 2。
2.  **单 ADC 模式**：
    *   如果 `USE_DUAL_ADC` 设为 0，`Get_PhaseDifference` 将始终返回 0 并打印警告。此时只能用于测量 ADC1 的频率和幅值。
3.  **采样率限制**：
    *   **内部 TIM3**：最大采样率受限于 ADC 转换时间（通常 < 2MHz），且受限于 84MHz 的分频精度。
    *   **外部 Si5351**：取决于 Si5351 的输出能力和 ADC 的最大触发频率。
4.  **混叠现象**：
    *   始终牢记奈奎斯特采样定理：`信号频率 < 采样率 / 2`。如果输入 10MHz 信号而采样率为 20kHz，结果将不可信。

## 6. CubeMX 配置指南 (CubeMX Configuration Guide)

为了确保动态切换和触发源工作正常，请严格按照以下步骤配置 CubeMX：

### 6.1 内部时钟模式 (默认)
若宏定义 `ADC_TRIGGER_SOURCE` 设为 `0` (内部 TIM3)，配置如下：
*   **TIM3**:
    *   **Clock Source**: Internal Clock
    *   **Prescaler (PSC)**: 0 (会被代码覆盖，初始值不重要)
    *   **Counter Period (ARR)**: 4199 (会被代码覆盖，对应初始 20kHz)
    *   **Trigger Output (TRGO)**: **Update Event** (关键！必须选这个)

### 6.2 外部时钟模式 (Si5351)
若宏定义 `ADC_TRIGGER_SOURCE` 设为 `1` (外部 Si5351)，需进行如下修改：
1.  **硬件连接**:
    *   必须将 **Si5351 的 CLK0 引脚** 物理连接到 **STM32 的 TIM3_ETR 引脚** (通常是 **PD2**，请查阅数据手册确认)。
2.  **TIM3 CubeMX 配置**:
    *   **Slave Mode**: **External Clock Mode 2**
    *   **Trigger Source**: **ETR1**
    *   **Clock Polarity**: Non Inverted
    *   **Clock Prescaler**: No Division
    *   **Clock Filter**: 0
    *   **Prescaler (PSC)**: 0
    *   **Counter Period (ARR)**: 0 (关键！设置为 0 表示每收到 1 个外部脉冲就产生一次更新事件，即 1:1 透传)
    *   **Trigger Output (TRGO)**: **Update Event**
3.  **原理说明**:
    *   在此模式下，TIM3 充当“中继器”。Si5351 发出一个脉冲 -> TIM3 计数溢出 -> 产生 TRGO 信号 -> 触发 ADC 采样。
    *   此时调用 `Phase_Set_SamplingRate` 实际上是在改变 Si5351 的输出频率。

