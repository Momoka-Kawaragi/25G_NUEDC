#include "TM1637.h"

#include "delay.h"

#include <stddef.h>
#include <string.h>

#ifndef TM1637_CLK_Pin
#error "Please define TM1637_CLK_Pin/TM1637_CLK_GPIO_Port in main.h"
#endif

#ifndef TM1637_DIO_Pin
#error "Please define TM1637_DIO_Pin/TM1637_DIO_GPIO_Port in main.h"
#endif

static uint8_t s_brightness = 7;

static void TM1637_Delay(void)
{
	delay_us(TM1637_DELAY_US);
}

static void TM1637_EnableGPIOClock(GPIO_TypeDef *port)
{
	if (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
	if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
	if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
	if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
	if (port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
#ifdef GPIOF
	if (port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
#endif
#ifdef GPIOG
	if (port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
#endif
#ifdef GPIOH
	if (port == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
#endif
#ifdef GPIOI
	if (port == GPIOI) __HAL_RCC_GPIOI_CLK_ENABLE();
#endif
}

static inline void TM1637_CLK_High(void)
{
	HAL_GPIO_WritePin(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, GPIO_PIN_SET);
}

static inline void TM1637_CLK_Low(void)
{
	HAL_GPIO_WritePin(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, GPIO_PIN_RESET);
}

static inline void TM1637_DIO_High(void)
{
	HAL_GPIO_WritePin(TM1637_DIO_GPIO_Port, TM1637_DIO_Pin, GPIO_PIN_SET);
}

static inline void TM1637_DIO_Low(void)
{
	HAL_GPIO_WritePin(TM1637_DIO_GPIO_Port, TM1637_DIO_Pin, GPIO_PIN_RESET);
}

static void TM1637_DIO_Output(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = TM1637_DIO_Pin;
	GPIO_InitStruct.Mode = (TM1637_USE_OPEN_DRAIN ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP);
	GPIO_InitStruct.Pull = (TM1637_USE_OPEN_DRAIN ? GPIO_PULLUP : GPIO_NOPULL);
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TM1637_DIO_GPIO_Port, &GPIO_InitStruct);
}

static void TM1637_DIO_Input(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = TM1637_DIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(TM1637_DIO_GPIO_Port, &GPIO_InitStruct);
}

static void TM1637_GPIO_Init(void)
{
	TM1637_EnableGPIOClock(TM1637_CLK_GPIO_Port);
	TM1637_EnableGPIOClock(TM1637_DIO_GPIO_Port);

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = TM1637_CLK_Pin;
	GPIO_InitStruct.Mode = (TM1637_USE_OPEN_DRAIN ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP);
	GPIO_InitStruct.Pull = (TM1637_USE_OPEN_DRAIN ? GPIO_PULLUP : GPIO_NOPULL);
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TM1637_CLK_GPIO_Port, &GPIO_InitStruct);

	TM1637_DIO_Output();
	TM1637_CLK_High();
	TM1637_DIO_High();
	HAL_Delay(10);
}

static void TM1637_Start(void)
{
	TM1637_DIO_Output();
	TM1637_DIO_High();
	TM1637_CLK_High();
	TM1637_Delay();
	TM1637_DIO_Low();
	TM1637_Delay();
	TM1637_CLK_Low();
}

static void TM1637_Stop(void)
{
	TM1637_DIO_Output();
	TM1637_CLK_Low();
	TM1637_DIO_Low();
	TM1637_Delay();
	TM1637_CLK_High();
	TM1637_Delay();
	TM1637_DIO_High();
	TM1637_Delay();
}

static uint8_t TM1637_WriteByte(uint8_t data)
{
	TM1637_DIO_Output();
	for (uint8_t i = 0; i < 8; i++) {
		TM1637_CLK_Low();
		TM1637_Delay();
		if (data & 0x01) TM1637_DIO_High(); else TM1637_DIO_Low();
		TM1637_Delay();
		TM1637_CLK_High();
		TM1637_Delay();
		data >>= 1;
	}

	// ACK: DIO 由 TM1637 拉低
	TM1637_CLK_Low();
	TM1637_Delay();
	TM1637_DIO_Input();
	TM1637_Delay();
	TM1637_CLK_High();
	TM1637_Delay();

	uint8_t ack = (HAL_GPIO_ReadPin(TM1637_DIO_GPIO_Port, TM1637_DIO_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

	TM1637_CLK_Low();
	TM1637_DIO_Output();
	TM1637_Delay();
	return ack;
}

static uint8_t TM1637_EncodeChar(char c)
{
	static const uint8_t tab[16] = {
		0x3F, 0x06, 0x5B, 0x4F,
		0x66, 0x6D, 0x7D, 0x07,
		0x7F, 0x6F, 0x77, 0x7C,
		0x39, 0x5E, 0x79, 0x71
	};

	if (c >= '0' && c <= '9') return tab[c - '0'];
	if (c >= 'A' && c <= 'F') return tab[c - 'A' + 10];
	if (c >= 'a' && c <= 'f') return tab[c - 'a' + 10];
	if (c == '-') return 0x40;
	if (c == ' ') return 0x00;
	return 0x00;
}

void TM1637_Init(void)
{
	TM1637_GPIO_Init();
	TM1637_SetBrightness(7);
	TM1637_Clear();
	TM1637_DisplayNoInput();
}

void TM1637_SetBrightness(uint8_t brightness)
{
	if (brightness > 7) brightness = 7;
	s_brightness = brightness;

	TM1637_Start();
	(void)TM1637_WriteByte(0x88 | 0x08 | (s_brightness & 0x07));
	TM1637_Stop();
}

void TM1637_Clear(void)
{
	TM1637_Start();
	(void)TM1637_WriteByte(0x40);
	TM1637_Stop();

	TM1637_Start();
	(void)TM1637_WriteByte(0xC0);
	for (uint8_t i = 0; i < 4; i++) {
		(void)TM1637_WriteByte(0x00);
	}
	TM1637_Stop();
}

void TM1637_DisplayString(const char *str)
{
	uint8_t segs[4] = {0, 0, 0, 0};

	if (str == NULL || str[0] == '\0') {
		str = TM1637_NO_INPUT_TEXT;
	}

	char filtered[32];
	size_t w = 0;
	for (size_t r = 0; str[r] != '\0' && w < (sizeof(filtered) - 1); r++) {
		if (str[r] == '.' && w == 0) continue;
		filtered[w++] = str[r];
	}
	filtered[w] = '\0';

	uint8_t digits[16];
	uint8_t dots[16];
	uint8_t count = 0;

	for (size_t i = 0; filtered[i] != '\0' && count < 16; i++) {
		if (filtered[i] == '.') {
			if (count > 0) dots[count - 1] = 1;
			continue;
		}
		digits[count] = (uint8_t)filtered[i];
		dots[count] = 0;
		count++;
	}

	// 右对齐显示：
	// - 字符数 >4：取最后 4 位
	// - 字符数 <4：左侧留空（不自动补 0）
	uint8_t shown = count;
	uint8_t start = 0;
	if (shown > 4) {
		start = (uint8_t)(shown - 4);
		shown = 4;
	}
	uint8_t offset = (shown < 4) ? (uint8_t)(4 - shown) : 0;
	for (uint8_t pos = 0; pos < 4; pos++) {
		int src = (int)pos - (int)offset;
		if (src < 0 || (uint8_t)src >= shown) {
			segs[pos] = 0x00;
			continue;
		}
		uint8_t idx = (uint8_t)(start + (uint8_t)src);
		uint8_t s = TM1637_EncodeChar((char)digits[idx]);
		if (dots[idx]) s |= 0x80;
		segs[pos] = s;
	}

	TM1637_Start();
	(void)TM1637_WriteByte(0x40);
	TM1637_Stop();

	TM1637_Start();
	(void)TM1637_WriteByte(0xC0);
	for (uint8_t i = 0; i < 4; i++) {
		(void)TM1637_WriteByte(segs[i]);
	}
	TM1637_Stop();

	TM1637_Start();
	(void)TM1637_WriteByte(0x88 | 0x08 | (s_brightness & 0x07));
	TM1637_Stop();
}
