#include "delay.h"

#include "stm32f4xx.h" 
// #include "main.h"

static uint32_t s_cycles_per_us = 0U;

void delay_init(void)
{
	// 使能 DWT 周期计数器
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // 使能追踪
	DWT->CYCCNT = 0;                                  // 复位计数
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              // 开启周期计数

	// 计算每微秒的 CPU 周期数（SystemCoreClock 由系统时钟配置提供）
	s_cycles_per_us = SystemCoreClock / 1000000U;
	if (s_cycles_per_us == 0U) {
		// 保护：避免除数为 0 的情况，给出最小值
		s_cycles_per_us = 1U;
	}
}

void delay_us(uint32_t us)
{
	// 若尚未初始化，进行一次最小初始化以提高健壮性
	if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U || s_cycles_per_us == 0U) {
		delay_init();
	}

	uint32_t start = DWT->CYCCNT;
	// 目标周期数；注意乘法可能在超长延时下溢出，但对常见 us 范围足够
	uint32_t target_cycles = us * s_cycles_per_us;
	while ((DWT->CYCCNT - start) < target_cycles) {
		__NOP();
	}
}

void delay_ms(uint32_t ms)
{
	while (ms--) {
		delay_us(1000U);
	}
}

