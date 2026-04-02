# Phase.c/h 功能增强计划

本计划旨在增强 `Phase.c/h` 模块，集成 Si5351 外部时钟源支持、动态采样率调节以及单/双 ADC 模式切换功能。

## 1. 宏定义与配置增强 (Phase.h)

*   **单/双 ADC 模式切换宏**：
    *   新增宏 `ADC_MODE_DUAL` (默认开启) 和 `ADC_MODE_SINGLE`。
    *   通过 `#define USE_DUAL_ADC 1` (或 0) 来控制编译逻辑。
*   **采样时钟源选择宏**：
    *   新增宏 `ADC_TRIGGER_SOURCE`。
    *   可选值：`TRIGGER_SOURCE_INTERNAL_TIM3` (内部定时器) 或 `TRIGGER_SOURCE_EXTERNAL_SI5351` (外部时钟)。

## 2. 动态采样率调节函数 (Phase.c/h)

### 2.1 内部时钟模式 (TIM3)
*   **新增函数**：`void Phase_Set_SamplingRate_Internal(uint32_t sampling_rate_hz);`
*   **实现逻辑**：
    *   根据目标采样率自动计算 TIM3 的 `Prescaler` (预分频) 和 `Period` (重装载值)。
    *   公式：$PSC \times ARR = \frac{84,000,000}{Fs}$。
    *   调用 `__HAL_TIM_SET_PRESCALER` 和 `__HAL_TIM_SET_AUTORELOAD` 动态修改寄存器。
    *   更新全局变量 `SAMPLING_RATE` 以供 FFT 计算频率使用。

### 2.2 外部时钟模式 (Si5351)
*   **新增函数**：`void Phase_Set_SamplingRate_External(uint32_t sampling_rate_hz);`
*   **实现逻辑**：
    *   初始化 Si5351 模块 (需确保 `si5351_Init` 被调用)。
    *   设置 Si5351 通道 0 (CLK0) 输出目标频率。
    *   该频率信号需连接到 STM32 的外部触发引脚 (如 TIM3 ETR 或 ADC EXT_TRIG，需确认硬件连接)。**注意：** 用户提到“通道0作为adc时钟的外部时钟源”，这通常意味着 Si5351 CLK0 -> STM32 引脚 -> 触发 ADC。最简单的实现是 Si5351 -> TIM3 ETR -> TIM3 TRGO -> ADC。如果硬件未连接 ETR，则此功能仅能控制 Si5351 输出，需用户手动连线。
    *   更新全局变量 `SAMPLING_RATE`。

### 2.3 统一接口
*   **新增函数**：`void Phase_Set_SamplingRate(uint32_t sampling_rate_hz);`
*   **逻辑**：根据 `ADC_TRIGGER_SOURCE` 宏自动调用上述 Internal 或 External 函数。

## 3. 单/双 ADC 逻辑适配 (Phase.c)

*   **初始化适配**：
    *   `FFT_App_Init` 中根据 `USE_DUAL_ADC` 宏决定是否启动 ADC2 的 DMA (`HAL_ADC_Start_DMA(&hadc2...)`)。
*   **数据处理适配**：
    *   `Process_ADC_RawData` 中根据宏决定是否转换 ADC2 数据。
    *   `Get_PhaseDifference` 中：
        *   若单 ADC 模式：仅计算 ADC1 相位，返回 0 或特定值，打印警告。
        *   若双 ADC 模式：计算差值 (现有逻辑)。
*   **回调函数适配**：
    *   `HAL_ADC_ConvCpltCallback` 中根据宏判断是否需要等待双 ADC 完成标志。

## 4. 代码结构调整

### Phase.h
```c
// --- 配置区域 ---
#define USE_DUAL_ADC        1       // 1: 双ADC模式, 0: 单ADC模式
#define ADC_TRIGGER_SOURCE  0       // 0: 内部TIM3, 1: 外部Si5351

// --- 接口声明 ---
void Phase_Set_SamplingRate(uint32_t fs);
void Phase_Set_SamplingRate_Internal(uint32_t fs);
void Phase_Set_SamplingRate_External(uint32_t fs);
```

### Phase.c
*   实现上述配置函数。
*   修改 `FFT_App_Init`、`FFT_App_Process`、`Get_PhaseDifference` 以支持条件编译。

## 5. 验证计划
1.  **编译检查**：确保宏切换后代码无编译错误。
2.  **功能验证**：
    *   默认模式 (双ADC + 内部TIM3)：应与当前行为一致。
    *   单 ADC 模式：应只采集 ADC1，节省一半 DMA 带宽。
    *   动态调频：在 main 中调用 `Phase_Set_SamplingRate(40000)`，验证 TIM3 寄存器值是否改变，以及 FFT 测得的频率是否准确。

如果计划无误，我将开始修改代码。