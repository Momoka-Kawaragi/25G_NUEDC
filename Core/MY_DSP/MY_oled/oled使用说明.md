参考文章：[4针0.96寸OLED的HAL库代码（硬件I2C/全代码/stm32f1/CubeMX配置/包含有正负浮点数/100%一次点亮）_hal库4针oled-CSDN博客](https://blog.csdn.net/LYH6767/article/details/126032948)

资源配置（F4）

![](配置.jpg)

代码修改指南

```C
#include "stm32f4xx_hal.h"//需要修改为F1或者F4的头文件
```

---

```C
void OLED_WR_CMD(uint8_t cmd)
{
	HAL_I2C_Mem_Write(&hi2c3 ,0x78,0x00,I2C_MEMADD_SIZE_8BIT,&cmd,1,0x100);
}

/**
 * @function: void OLED_WR_DATA(uint8_t data)
 * @description: 向设备写控制数据
 * @param {uint8_t} data 数据
 * @return {*}
 */
void OLED_WR_DATA(uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c3 ,0x78,0x40,I2C_MEMADD_SIZE_8BIT,&data,1,0x100);
}
```

其中`HAL_I2C_Mem_Write`的句柄需要改成你hal库生成的使用的句柄

---

具体使用

```C
	HAL_Delay(20);
	
	OLED_Init(); // OLED初始化
	OLED_Clear();
	
	OLED_ShowString(0,0,"UNICORN_LI",16, 1);    //反相显示8X16字符串
  	OLED_ShowString(0,2,"unicorn_li_123",12,0);//正相显示6X8字符串
 
  	OLED_ShowCHinese(0,4,0,1); //反相显示汉字“独”
    OLED_ShowCHinese(16,4,1,1);//反相显示汉字“角”
 	OLED_ShowCHinese(32,4,2,1);//反相显示汉字“兽”
    OLED_ShowCHinese(0,6,0,0); //正相显示汉字“独”
    OLED_ShowCHinese(16,6,1,0);//正相显示汉字“角”
    OLED_ShowCHinese(32,6,2,0);//正相显示汉字“兽”

    OLED_ShowNum(48,4,6,1,16, 0);//正相显示1位8X16数字“6”
    OLED_ShowNum(48,7,77,2,12, 1);//反相显示2位6X8数字“77”
    OLED_DrawBMP(90,0,122, 4,BMP1,0);//正相显示图片BMP1
    OLED_DrawBMP(90,4,122, 8,BMP1,1);//反相显示图片BMP1
 
    OLED_HorizontalShift(0x26);//全屏水平向右滚动播放
 
```

