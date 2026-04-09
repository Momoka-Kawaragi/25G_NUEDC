我将为你生成一份详细的 `FFT_相位计算库使用说明.md` 文档，并保存在 `e:\STM32\F_BEST\BEST_USE\Core\MY_DSP\MY_FFT\` 目录下。

该文档将包含以下核心内容：

1.  **STM32CubeMX 配置指南**：
    *   **DSP 库开启**：如何在 Software Packs 中启用 CMSIS-DSP。
    *   **时钟配置 (TIM3)**：如何设置 Prescaler 和 Period 以达到 20kHz 采样率。
    *   **双 ADC 配置 (ADC1 & ADC2)**：
        *   如何配置为独立模式或同步模式。
        *   如何设置外部触发源 (Timer 3 Trigger Out Event)。
        *   DMA 设置 (Normal 模式，数据宽度 Half Word)。
    *   **中断配置 (NVIC)**：开启 DMA 中断。

2.  **代码集成与使用**：
    *   **单通道 FFT 使用**：如何调用 `FFT_App_Init()` 和 `FFT_App_Process()` 快速实现单路信号频谱分析。
    *   **双通道相位差测量**：重点说明如何使用 `PhaseCalculate_ADC_Init()` 和 `Get_PhaseDifference()` 实现两个 ADC 同时读取并计算相位差。

3.  **关键参数说明**：
    *   解释 `FFT_LENGTH` 和 `SAMPLING_RATE` 的修改方法。

此计划仅生成文档，不修改现有代码。