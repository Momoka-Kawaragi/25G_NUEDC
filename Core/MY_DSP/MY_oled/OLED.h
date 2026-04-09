#ifndef OLED_OLED_H_
#define OLED_OLED_H_

#include "stm32f4xx_hal.h"
#include "oledfont.h"
extern I2C_HandleTypeDef  hi2c3;

// ---------------------------------- 定义列表 ---------------------------------- //
// 在 STM32 平台上使用 I2C 外设，当前工程使用：extern I2C_HandleTypeDef hi2c3;（SCL/SDA 由硬件/引脚配置决定）
// ----------------------------------------------------------------------------- //

// ---------------------------------- 函数列表 ---------------------------------- //
void OLED_WR_CMD(uint8_t cmd); // 写入单字节命令到 OLED（通过 I2C 发送命令）
void OLED_WR_DATA(uint8_t data); // 写入单字节数据到 OLED（通过 I2C 发送数据）
void OLED_Init(void); // OLED 初始化：初始化显示、配置参数并清屏
void OLED_Clear(void); // OLED 清屏：清除显示缓存并更新屏幕
void OLED_Display_On(void); // 打开 OLED 显示（退出掉电/休眠等低功耗模式）
void OLED_Display_Off(void); // 关闭 OLED 显示（进入休眠或关屏）
void OLED_Set_Pos(uint8_t x, uint8_t y); // 设置光标位置：列 x，页 y
void OLED_On(void); // 打开整个屏幕所有像素（常用于测试）
void OLED_ShowNum(uint8_t x,uint8_t y,unsigned int num,uint8_t len,uint8_t size2,uint8_t Color_Turn); // 在 (x,y) 显示整数数字，len 为位数，size2 为字号，Color_Turn 控制颜色/反色
void OLED_Showdecimal(uint8_t x,uint8_t y,float num,uint8_t z_len,uint8_t f_len,uint8_t size2, uint8_t Color_Turn); // 在 (x,y) 显示带小数的浮点数，z_len 整数位，f_len 小数位
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size,uint8_t Color_Turn); // 在 (x,y) 显示单个字符，Char_Size 为字号
void OLED_ShowString(uint8_t x,uint8_t y,char*chr,uint8_t Char_Size,uint8_t Color_Turn); // 在 (x,y) 显示字符串，直到遇到 '\0' 或换行
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no,uint8_t Color_Turn); // 在 (x,y) 显示内置汉字，no 为汉字索引
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t *  BMP,uint8_t Color_Turn); // 在矩形区域 (x0,y0)-(x1,y1) 绘制 BMP 点阵图
void OLED_HorizontalShift(uint8_t direction); // 水平整屏滚动：direction 指定滚动方向
void OLED_Some_HorizontalShift(uint8_t direction,uint8_t start,uint8_t end); // 指定 start-end 页 范围的水平滚动
void OLED_VerticalAndHorizontalShift(uint8_t direction); // 垂直加水平组合滚动
void OLED_DisplayMode(uint8_t mode); // 显示模式切换（例如正常/反显/局部模式）
void OLED_IntensityControl(uint8_t intensity); // 显示亮度/对比度控制，取值范围取决于驱动
// ----------------------------------------------------------------------------- //



#endif /* OLED_OLED_H_ */
