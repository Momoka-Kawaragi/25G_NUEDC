/**************************************************************************/
/*!
    @file     si5351.c

    @author   K. Townsend (Adafruit Industries)

    @brief    SI5351 160MHz 时钟发生器驱动程序

    @section  REFERENCES (参考资料)

    Si5351A/B/C 数据手册:
    http://www.silabs.com/Support%20Documents/TechnicalDocs/Si5351.pdf

    手动生成 Si5351 寄存器映射:
    http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf

    @section  LICENSE
    (版权声明保留英文...)
*/
/**************************************************************************/
#include "main.h"
// 请根据您的芯片更改此处
#include "stm32f4xx_hal.h"
#include <math.h>
#include <si5351.h>

/**************************************************************************/
/*!
    初始化 I2C 并配置分线板 (在做任何其他事情之前调用此函数)
*/
/**************************************************************************/
err_t si5351_Init(void)
{

	/*!
	    构造函数 (初始化配置结构体)
	*/
	  m_si5351Config.initialised     = 0;
	  m_si5351Config.crystalFreq     = SI5351_CRYSTAL_FREQ_25MHZ;
	  m_si5351Config.crystalLoad     = SI5351_CRYSTAL_LOAD_10PF;
	  m_si5351Config.crystalPPM      = 30;
	  m_si5351Config.plla_configured = 0;
	  m_si5351Config.plla_freq       = 0;
	  m_si5351Config.pllb_configured = 0;
	  m_si5351Config.pllb_freq       = 0;
	  m_si5351Config.ms0_freq		 = 0;
	  m_si5351Config.ms1_freq		 = 0;
	  m_si5351Config.ms2_freq		 = 0;
	  m_si5351Config.ms0_r_div		 = 0;
	  m_si5351Config.ms1_r_div		 = 0;
	  m_si5351Config.ms2_r_div		 = 0;



  /* 禁用所有输出，将 CLKx_DIS 置高 */
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF));

  /* 关闭所有输出驱动器的电源 */
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_16_CLK0_CONTROL, 0x80));
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_17_CLK1_CONTROL, 0x80));
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_18_CLK2_CONTROL, 0x80));
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_19_CLK3_CONTROL, 0x80));
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_20_CLK4_CONTROL, 0x80));
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_21_CLK5_CONTROL, 0x80));
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_22_CLK6_CONTROL, 0x80));
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_23_CLK7_CONTROL, 0x80));

  /* 设置晶振 (XTAL) 的负载电容 */
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE,
                       m_si5351Config.crystalLoad));

  /* 根据需要设置中断掩码 (参见 AN619 中的寄存器 2 描述)。
     默认情况下，ClockBuilder Desktop 将此寄存器设置为 0x18。
     注意：最低有效半字节必须保持 0x8，但最高有效半字节可以根据需要进行修改。 */

  /* 重置 PLL 配置字段，以防我们再次调用 init */
  m_si5351Config.plla_configured = 0;
  m_si5351Config.plla_freq = 0;
  m_si5351Config.pllb_configured = 0;
  m_si5351Config.pllb_freq = 0;

  /* 全部完成！ */
  m_si5351Config.initialised = 1;

  return ERROR_NONE;
}


/**************************************************************************/
/*!
  @brief  使用整数值设置指定 PLL 的倍频器

  @param  pll   要配置的 PLL，必须是以下之一：
                - SI5351_PLL_A
                - SI5351_PLL_B
  @param  mult  PLL 整数倍频器 (必须在 15 到 90 之间)
*/
/**************************************************************************/
err_t si5351_setupPLLInt(si5351PLL_t pll, uint8_t mult)
{
  return si5351_setupPLL(pll, mult, 0, 1);
}

/**************************************************************************/
/*!
    @brief  设置指定 PLL 的倍频器

    @param  pll   要配置的 PLL，必须是以下之一：
                  - SI5351_PLL_A
                  - SI5351_PLL_B
    @param  mult  PLL 整数倍频器 (必须在 15 到 90 之间)
    @param  num   用于分数输出的 20 位分子 (0..1,048,575)。
                  设置为 '0' 以进行整数输出。
    @param  denom 用于分数输出的 20 位分母 (1..1,048,575)。
                  设置为 '1' 或更高以避免除零错误。

    @section PLL Configuration (PLL 配置)

    fVCO 是 PLL 输出，必须在 600..900MHz 之间，其中：

        fVCO = fXTAL * (a+(b/c))

    fXTAL = 晶振输入频率
    a     = 15 到 90 之间的整数
    b     = 分数分子 (0..1,048,575)
    c     = 分数分母 (1..1,048,575)

    注意：尽可能尝试使用整数以避免时钟抖动
    (仅使用 a 部分，将 b 设置为 '0'，c 设置为 '1')。

    参见: http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf
*/
/**************************************************************************/
err_t si5351_setupPLL(si5351PLL_t pll,
                                uint8_t     mult,
                                uint32_t    num,
                                uint32_t    denom)
{
  uint32_t P1;       /* PLL 配置寄存器 P1 */
  uint32_t P2;	     /* PLL 配置寄存器 P2 */
  uint32_t P3;	     /* PLL 配置寄存器 P3 */

  /* 基本验证 */
  ASSERT( m_si5351Config.initialised, ERROR_DEVICENOTINITIALISED );
  ASSERT( (mult > 14) && (mult < 91), ERROR_INVALIDPARAMETER ); /* mult = 15..90 */
  ASSERT( denom > 0,                  ERROR_INVALIDPARAMETER ); /* 避免除以零 */
  ASSERT( num <= 0xFFFFF,             ERROR_INVALIDPARAMETER ); /* 20 位限制 */
  ASSERT( denom <= 0xFFFFF,           ERROR_INVALIDPARAMETER ); /* 20 位限制 */

  /* 反馈 MultiSynth 分频器公式
   *
   * 其中: a = mult(整数倍频器), b = num分数输出的 20 位分子 且 c = denom分数输出的 20 位分母
   *
   * P1 寄存器是一个 18 位的值，使用以下公式：
   *
   * P1[17:0] = 128 * mult + floor(128*(num/denom)) - 512
   *
   * P2 寄存器是一个 20 位的值，使用以下公式：
   *
   * P2[19:0] = 128 * num - denom * floor(128*(num/denom))
   *
   * P3 寄存器是一个 20 位的值，使用以下公式：
   *
   * P3[19:0] = denom
   */

  /* 设置主 PLL 配置寄存器 */
  if (num == 0)
  {
    /* 整数模式 */
    P1 = 128 * mult - 512;
    P2 = num;
    P3 = denom;
  }
  else
  {
    /* 分数模式 */
    P1 = (uint32_t)(128 * mult + floor(128 * ((float)num/(float)denom)) - 512);
    P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num/(float)denom)));
    P3 = denom;
  }

  /* 获取 PLL 寄存器的适当起始地址 */
  //PLLA起始地址为26，PLLB为34
  uint8_t baseaddr = (pll == SI5351_PLL_A ? 26 : 34);

  /* 数据手册这里充满了拼写错误和不一致！（错误判断用于调试） */
  ASSERT_STATUS( si5351_write8( baseaddr,   (P3 & 0x0000FF00) >> 8));
  ASSERT_STATUS( si5351_write8( baseaddr+1, (P3 & 0x000000FF)));
  ASSERT_STATUS( si5351_write8( baseaddr+2, (P1 & 0x00030000) >> 16));
  ASSERT_STATUS( si5351_write8( baseaddr+3, (P1 & 0x0000FF00) >> 8));
  ASSERT_STATUS( si5351_write8( baseaddr+4, (P1 & 0x000000FF)));
  ASSERT_STATUS( si5351_write8( baseaddr+5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16) ));
  ASSERT_STATUS( si5351_write8( baseaddr+6, (P2 & 0x0000FF00) >> 8));
  ASSERT_STATUS( si5351_write8( baseaddr+7, (P2 & 0x000000FF)));

  /* 重置两个 PLL */
  ASSERT_STATUS( si5351_write8(SI5351_REGISTER_177_PLL_RESET, (1<<7) | (1<<5) ));

  /* 存储频率设置以供 MultiSynth 助手使用 */
  if (pll == SI5351_PLL_A)
  {
    float fvco = m_si5351Config.crystalFreq * (mult + ( (float)num / (float)denom ));
    m_si5351Config.plla_configured = 1; //true
    m_si5351Config.plla_freq = (uint32_t)floor(fvco);
  }
  else
  {
    float fvco = m_si5351Config.crystalFreq * (mult + ( (float)num / (float)denom ));
    m_si5351Config.pllb_configured = 1; //true
    m_si5351Config.pllb_freq = (uint32_t)floor(fvco);
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  使用整数输出配置 MultiSynth 分频器。

    @param  output    要使用的输出通道 (0..2)
    @param  pllSource 要使用的 PLL 输入源，必须是以下之一：
                      - SI5351_PLL_A
                      - SI5351_PLL_B
    @param  div       MultiSynth 输出的整数分频器，
                      必须是以下值之一：
                      - SI5351_MULTISYNTH_DIV_4
                      - SI5351_MULTISYNTH_DIV_6
                      - SI5351_MULTISYNTH_DIV_8
*/
/**************************************************************************/
err_t si5351_setupMultisynthInt(uint8_t               output,
                                          si5351PLL_t           pllSource,
                                          si5351MultisynthDiv_t div)
{
  return si5351_setupMultisynth(output, pllSource, div, 0, 1);
}


err_t si5351_setupRdiv(uint8_t  output, si5351RDiv_t div) {
  ASSERT( output < 3,                 ERROR_INVALIDPARAMETER);  /* 通道范围 */
  
  uint8_t Rreg, regval, rDiv;

  if (output == 0) Rreg = SI5351_REGISTER_44_MULTISYNTH0_PARAMETERS_3;
  if (output == 1) Rreg = SI5351_REGISTER_52_MULTISYNTH1_PARAMETERS_3;
  if (output == 2) Rreg = SI5351_REGISTER_60_MULTISYNTH2_PARAMETERS_3;

  si5351_read8(Rreg, &regval);

  regval &= 0x0F;
  uint8_t divider = div;
  divider &= 0x07;
  divider <<= 4;
  regval |= divider;
  si5351_write8(Rreg, regval);

  switch(div)
  {
  case 0:
  rDiv = 1;
  break;

  case 1:
  rDiv = 2;
  break;

  case 2:
  rDiv = 4;
  break;

  case 3:
  rDiv = 8;
  break;

  case 4:
  rDiv = 16;
  break;

  case 5:
  rDiv = 32;
  break;

  case 6:
  rDiv = 64;
  break;

  case 7:
  rDiv = 128;
  break;
  }

  switch(output)
  {
  case 0:
  m_si5351Config.ms0_r_div = rDiv;
  break;

  case 1:
  m_si5351Config.ms1_r_div = rDiv;
  break;

  case 2:
  m_si5351Config.ms2_r_div = rDiv;
  break;
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  配置 MultiSynth 分频器，该分频器根据指定的 PLL 输入确定输出时钟频率。

    @param  output    要使用的输出通道 (0..2)
    @param  pllSource 要使用的 PLL 输入源，必须是以下之一：
                      - SI5351_PLL_A
                      - SI5351_PLL_B
    @param  div       MultiSynth 输出的整数分频器。
                      如果使用纯整数值，此值必须是以下之一：
                      - SI5351_MULTISYNTH_DIV_4
                      - SI5351_MULTISYNTH_DIV_6
                      - SI5351_MULTISYNTH_DIV_8
                      如果使用分数输出，此值必须在 8 和 900 之间。
    @param  num       用于分数输出的 20 位分子
                      (0..1,048,575)。设置为 '0' 以进行整数输出。
    @param  denom     用于分数输出的 20 位分母
                      (1..1,048,575)。设置为 '1' 或更高以
                      避免除零错误。

    @section Output Clock Configuration (输出时钟配置)

    MultiSynth 分频器应用于指定的 PLL 输出，用于将 PLL 输出降低到有效范围 
    (500kHz 到 160MHz)。关系如下公式所示，其中 fVCO 是 PLL 输出频率，MSx 
    是 MultiSynth 分频器：

        fOUT = fVCO / MSx

    当使用整数时，有效的 MultiSynth 分频器是 4、6 或 8，
    或者任何介于 8 + 1/1,048,575 和 900 + 0/1 之间的分数值。

    分数模式分频器使用以下公式：

        a + b / c

    a = 整数值，在整数模式下 (MSx_INT=1) 必须是 4、6 或 8，
        或在分数模式下 (MSx_INT=0) 为 8..900。
    b = 分数分子 (0..1,048,575)
    c = 分数分母 (1..1,048,575)

    @note   尽可能尝试使用整数以避免时钟抖动

    @note   对于 > 150MHz 的输出频率，您必须将分频器设置为 4 并调整 PLL 
            以生成频率 (例如，PLL 为 640 以生成 160MHz 输出时钟)。
            这在驱动程序中尚未支持，目前限制频率为 500kHz .. 150MHz。

    @note   对于低于 500kHz 的频率 (低至 8kHz)，必须使用 Rx_DIV，
            但这目前尚未在驱动程序中实现。
*/
/**************************************************************************/
err_t si5351_setupMultisynth(uint8_t     output,
                                       si5351PLL_t pllSource,
                                       uint32_t    div,
                                       uint32_t    num,
                                       uint32_t    denom)
{
  uint32_t P1;       /* MultiSynth 配置寄存器 P1 */
  uint32_t P2;	     /* MultiSynth 配置寄存器 P2 */
  uint32_t P3;	     /* MultiSynth 配置寄存器 P3 */

  /* 基本验证 */
  ASSERT( m_si5351Config.initialised, ERROR_DEVICENOTINITIALISED);
  ASSERT( output < 3,                 ERROR_INVALIDPARAMETER);  /* 通道范围 */
  //ASSERT( div > 3,                    ERROR_INVALIDPARAMETER);  /* 分频器整数值 */
  //ASSERT( div < 901,                  ERROR_INVALIDPARAMETER);  /* 分频器整数值 */
  //ASSERT( denom > 0,                  ERROR_INVALIDPARAMETER ); /* 避免除以零 */
  //ASSERT( num <= 0xFFFFF,             ERROR_INVALIDPARAMETER ); /* 20 位限制 */
  //ASSERT( denom <= 0xFFFFF,           ERROR_INVALIDPARAMETER ); /* 20 位限制 */


  /* 确保请求的 PLL 已经初始化 */
  if (pllSource == SI5351_PLL_A)
  {
    ASSERT(m_si5351Config.plla_configured = 1, ERROR_INVALIDPARAMETER);
  }
  else
  {
    ASSERT(m_si5351Config.pllb_configured = 1, ERROR_INVALIDPARAMETER);
  }

  /* 输出 MultiSynth 分频器公式
   *
   * 其中: a = div, b = num 且 c = denom
   *
   * P1 寄存器是一个 18 位的值，使用以下公式：
   *
   * P1[17:0] = 128 * a + floor(128*(b/c)) - 512
   *
   * P2 寄存器是一个 20 位的值，使用以下公式：
   *
   * P2[19:0] = 128 * b - c * floor(128*(b/c))
   *
   * P3 寄存器是一个 20 位的值，使用以下公式：
   *
   * P3[19:0] = c
   */

  /* 设置主 PLL 配置寄存器 */
  if (num == 0)
  {
    /* 整数模式 */
    P1 = 128 * div - 512;
    P2 = num;
    P3 = denom;
  }
  else
  {
    /* 分数模式 */
    P1 = (uint32_t)(128 * div + floor(128 * ((float)num/(float)denom)) - 512);
    P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num/(float)denom)));
    P3 = denom;
  }

  /* 获取 PLL 寄存器的适当起始地址 */
  uint8_t baseaddr = 0;
  switch (output)
  {
    case 0:
      baseaddr = SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1;
      break;
    case 1:
      baseaddr = SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1;
      break;
    case 2:
      baseaddr = SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1;
      break;
  }

  /* 设置 MSx 配置寄存器 */
  si5351_write8( baseaddr,   (P3 & 0x0000FF00) >> 8);
  si5351_write8( baseaddr+1, (P3 & 0x000000FF));
  si5351_write8( baseaddr+2, (P1 & 0x00030000) >> 16);	/* 待办事项：稍后添加 DIVBY4 (>150MHz) 和 R0 支持 (<500kHz) */
  si5351_write8( baseaddr+3, (P1 & 0x0000FF00) >> 8);
  si5351_write8( baseaddr+4, (P1 & 0x000000FF));
  si5351_write8( baseaddr+5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16) );
  si5351_write8( baseaddr+6, (P2 & 0x0000FF00) >> 8);
  si5351_write8( baseaddr+7, (P2 & 0x000000FF));


  if (pllSource == SI5351_PLL_A)
  {
          float fvco = m_si5351Config.plla_freq / (div + ( (float)num / (float)denom ));
          switch (output)
          {
           case 0:
           m_si5351Config.ms0_freq = (uint32_t)floor(fvco);
           break;
           case 1:
           m_si5351Config.ms1_freq = (uint32_t)floor(fvco);
           break;
           case 2:
           m_si5351Config.ms2_freq = (uint32_t)floor(fvco);
           break;
          }
  }
  else
  {
          float fvco = m_si5351Config.pllb_freq / (div + ( (float)num / (float)denom));
          switch (output)
          {
           case 0:
           m_si5351Config.ms0_freq = (uint32_t)floor(fvco);
           break;
           case 1:
           m_si5351Config.ms1_freq = (uint32_t)floor(fvco);
           break;
           case 2:
           m_si5351Config.ms2_freq = (uint32_t)floor(fvco);
           break;
          }
  }



  /* 配置时钟控制并启用输出 */
  uint8_t clkControlReg = 0x0F;                             /* 8mA 驱动强度, MS0 作为 CLK0 源, 时钟不反转, 上电 */
  if (pllSource == SI5351_PLL_B) clkControlReg |= (1 << 5); /* 使用 PLLB */
  if (num == 0) clkControlReg |= (1 << 6);                  /* 整数模式 */
  switch (output)
  {
    case 0:
      ASSERT_STATUS(si5351_write8(SI5351_REGISTER_16_CLK0_CONTROL, clkControlReg));
      break;
    case 1:
      ASSERT_STATUS(si5351_write8(SI5351_REGISTER_17_CLK1_CONTROL, clkControlReg));
      break;
    case 2:
      ASSERT_STATUS(si5351_write8(SI5351_REGISTER_18_CLK2_CONTROL, clkControlReg));
      break;
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  启用或禁用所有时钟输出
*/
/**************************************************************************/
err_t si5351_enableOutputs(uint8_t enabled)
{
  /* 确保我们首先调用了 init */
  ASSERT(m_si5351Config.initialised, ERROR_DEVICENOTINITIALISED);

  /* 启用所需的输出 (见寄存器 3) */
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, enabled ? 0x00: 0xFF));

  return ERROR_NONE;
}

/* ---------------------------------------------------------------------- */
/* 私有函数                                                               */
/* ---------------------------------------------------------------------- */

/**************************************************************************/
/*!
    @brief  通过 I2C 写入寄存器和 8 位值
*/
/**************************************************************************/
err_t si5351_write8 (uint8_t reg, uint8_t value)
{
	HAL_StatusTypeDef status = HAL_OK;
  
	while (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(SI5351_ADDRESS<<1), 3, 100) != HAL_OK) { }

    status = HAL_I2C_Mem_Write(&hi2c2,							// i2c 句柄
    						  (uint8_t)(SI5351_ADDRESS<<1),		// i2c 地址，左对齐
							  (uint8_t)reg,						// 寄存器地址
							  I2C_MEMADD_SIZE_8BIT,				// si5351 使用 8 位寄存器地址
							  (uint8_t*)(&value),				// 将数据写入此变量
							  1,								// 预期返回多少字节
							  100);								// 超时

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  通过 I2C 读取一个 8 位值
*/
/**************************************************************************/
err_t si5351_read8(uint8_t reg, uint8_t *value)
{
	HAL_StatusTypeDef status = HAL_OK;

	while (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(SI5351_ADDRESS<<1), 3, 100) != HAL_OK) { }

    status = HAL_I2C_Mem_Read(&hi2c2,							// i2c 句柄
    						  (uint8_t)(SI5351_ADDRESS<<1),		// i2c 地址，左对齐
							  (uint8_t)reg,						// 寄存器地址
							  I2C_MEMADD_SIZE_8BIT,				// si5351 使用 8 位寄存器地址
							  (uint8_t*)(&value),				// 将数据写入此变量
							  1,								// 预期返回多少字节
							  100);								// 超时

  return ERROR_NONE;
}

/**
 * @brief Si5351 设置单个通道输出频率
 */
err_t SI5351_SetFrequency(uint8_t clk, uint32_t fout)
{
    if (clk > 2) return ERROR_INVALIDPARAMETER;

    uint32_t f_xtal = m_si5351Config.crystalFreq; // 25 MHz
    uint8_t pll = (clk == 0) ? SI5351_PLL_A : SI5351_PLL_B;

    uint32_t fvco;
    uint32_t mult, num = 0, denom = 1;

    /* =====================================================
     * 1. 自动选择合适的 VCO 频率（600~900 MHz）
     * ===================================================== */
    uint32_t target_fvco = fout * 32;// 选择一个初始 VCO 频率，大约是输出频率的 32 倍
    if (target_fvco < 600000000UL)  target_fvco = 600000000UL;
    if (target_fvco > 900000000UL)  target_fvco = 900000000UL;

    mult = target_fvco / f_xtal;
    fvco = f_xtal * mult;

    /* =====================================================
     * 2. 设置 PLL（使用整数模式）
     * ===================================================== */
    ASSERT_STATUS(si5351_setupPLL(pll, mult, 0, 1));

    /* =====================================================
     * 3. 分母参数：MS = fvco / fout
     * ===================================================== */
    float ms_f = (float)fvco / (float)fout;
    uint32_t ms_int = (uint32_t)ms_f;

    /* 是否使用分数模式？ */
    if (fabs(ms_f - ms_int) < 0.000001)
    {
        /* 整数 */
        num = 0;
        denom = 1;
    }
    else
    {
        /* 分数模式 */
        denom = 1000000;                      // 精度 1 ppm
        num = (uint32_t)((ms_f - ms_int) * denom);
    }

    /* =====================================================
     * 4. 配置 MultiSynth
     * ===================================================== */
    ASSERT_STATUS(si5351_setupMultisynth(clk, pll, ms_int, num, denom));

    /* =====================================================
     * 5. 自动 R 分频（低频处理）
     * ===================================================== */
    if (fout < 500000)   si5351_setupRdiv(clk, SI5351_R_DIV_8);
    if (fout <  62500)   si5351_setupRdiv(clk, SI5351_R_DIV_128);

    /* 启用 */
    ASSERT_STATUS(si5351_enableOutputs(1));

    return ERROR_NONE;
}

/**
 * @brief 开启或关闭 Si5351 的单个输出通道。
 */
err_t SI5351_SetChannelPower(uint8_t channel, uint8_t enabled)
{
    uint8_t reg;
    uint8_t value;
    err_t status;

    // 1. 参数校验: 仅支持 CLK0, CLK1, CLK2
    ASSERT(channel <= 2, ERROR_INVALIDPARAMETER);
    
    // 2. 映射通道到对应的 CLKx 控制寄存器 (从寄存器 16 开始)
    // 16 + 0 = 16 (CLK0), 16 + 1 = 17 (CLK1), 16 + 2 = 18 (CLK2)
    reg = SI5351_REGISTER_16_CLK0_CONTROL + channel;

    // 3. 读取当前寄存器值
    status = si5351_read8(reg, &value);
    ASSERT_STATUS(status);

    // 4. 修改 Power Down 位 (Bit 7)
    if (enabled) {
        // 开启 (Power Up): 清除 Bit 7 (Power Down = 0)
        value &= ~(1 << 7); 
    } else {
        // 关闭 (Power Down): 设置 Bit 7 (Power Down = 1)
        value |= (1 << 7);
    }

    // 5. 写入修改后的值
    status = si5351_write8(reg, value);
    ASSERT_STATUS(status);
    
    // 6. 如果是开启操作，确保全局输出也处于启用状态
    if (enabled) {
        // 调用全局启用函数（您的驱动中 1 表示启用）
        status = si5351_enableOutputs(1);
        ASSERT_STATUS(status);
    }

    return ERROR_NONE;
}

