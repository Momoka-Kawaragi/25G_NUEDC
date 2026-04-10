#ifndef __TM1637_H
#define __TM1637_H

#include <stdint.h>

#include "main.h"

// 键盘无输入/空字符串时的默认显示（4位数码管）
#ifndef TM1637_NO_INPUT_TEXT
#define TM1637_NO_INPUT_TEXT "----"
#endif

// TM1637 时序延时（微秒）。默认调慢一点更稳。
#ifndef TM1637_DELAY_US
#define TM1637_DELAY_US (50U)
#endif

// 是否使用开漏输出：
// - 0: 推挽输出（不依赖外部上拉，更容易点亮）
// - 1: 开漏输出（更“标准”，需要上拉）
#ifndef TM1637_USE_OPEN_DRAIN
#define TM1637_USE_OPEN_DRAIN (0)
#endif

void TM1637_Init(void);
void TM1637_SetBrightness(uint8_t brightness);
void TM1637_Clear(void);
void TM1637_DisplayString(const char *str);

static inline void TM1637_DisplayNoInput(void)
{
	TM1637_DisplayString(TM1637_NO_INPUT_TEXT);
}

#endif /* __TM1637_H */
