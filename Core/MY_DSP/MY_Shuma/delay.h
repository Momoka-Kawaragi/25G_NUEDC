/**
 * 简易延时库（微秒/毫秒）
 * 基于 Cortex-M4 DWT 周期计数器实现精确忙等。
 */

#ifndef USER_DELAY_H
#define USER_DELAY_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/**
 * 初始化延时功能：启用 DWT 周期计数器，并计算每微秒的 CPU 周期数。
 * 在使用 delay_us()/delay_ms() 之前调用一次。
 */
void delay_init(void);

/**
 * 微秒级延时（忙等）。
 * @param us 需要延时的微秒数。
 */
void delay_us(uint32_t us);

/**
 * 毫秒级延时（忙等）。
 * 如需低功耗或不占用 CPU，建议改用 HAL_Delay(ms)。
 * @param ms 需要延时的毫秒数。
 */
void delay_ms(uint32_t ms);

#endif /* USER_DELAY_H */

