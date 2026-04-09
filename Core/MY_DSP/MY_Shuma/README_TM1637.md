# TM1637 数码管驱动使用说明 (STM32 HAL 库版)

本驱动包含两个文件：`TM1637.c` 和 `TM1637.h`，用于驱动基于 TM1637 芯片的 4 位数码管模块。

## 1. CubeMX 配置 (GPIO 设置)

在使用代码之前，需要在 STM32CubeMX 中配置两个 GPIO 引脚，分别作为 **CLK** (时钟) 和 **DIO** (数据) 引脚。

1.  打开你的 STM32CubeMX 工程。
2.  在 **Pinout & Configuration** 页面，选择两个空闲的 GPIO 引脚（例如 `PA5` 和 `PA6`）。
3.  将这两个引脚的模式设置为 **GPIO_Output**。
4.  在 **GPIO** 配置标签页中，对这两个引脚进行如下设置：
    *   **GPIO Output Level**: `High` (默认拉高)
    *   **GPIO Mode**: `Output Push Pull` (推挽输出)
        *   *注意：如果在 `TM1637.h` 中开启了 `TM1637_USE_OPEN_DRAIN`，则这里需选 `Output Open Drain` 并外接上拉电阻。默认代码使用的是推挽，无需上拉。*
    *   **GPIO Pull-up/Pull-down**: `No pull-up and no pull-down` (或者 `Pull-up` 以此增加稳定性)
    *   **Maximum output speed**: `Low` 或 `Medium`
5.  **关键步骤**：在 **User Label** (用户标签) 一栏中，必须填入指定的名称，以便代码自动识别：
    *   将 CLK 引脚的 User Label 设为：`TM1637_CLK`
    *   将 DIO 引脚的 User Label 设为：`TM1637_DIO`
6.  点击 **GENERATE CODE** 生成代码。

> **原理说明**：CubeMX 会根据 User Label 在 `main.h` 中自动生成如下宏定义，驱动文件正是依赖这些宏来操作引脚的：
> ```c
> #define TM1637_CLK_Pin GPIO_PIN_5
> #define TM1637_CLK_GPIO_Port GPIOA
> #define TM1637_DIO_Pin GPIO_PIN_6
> #define TM1637_DIO_GPIO_Port GPIOA
> ```

## 2. 工程集成

1.  **复制文件**：
    *   将 `TM1637.c` 复制到工程的 `Core/Src` 目录下。
    *   将 `TM1637.h` 复制到工程的 `Core/Inc` 目录下。
2.  **添加编译项** (如果使用 Keil/IAR)：
    *   在 IDE 的工程文件树中，右键 `Application/User/Core` 文件夹，选择 "Add Existing Files"，将 `TM1637.c` 添加进去。

## 3. 代码调用

在 `main.c` 文件中进行如下修改：

### 3.1 包含头文件
在 `/* USER CODE BEGIN Includes */` 和 `/* USER CODE END Includes */` 之间添加：

```c
/* USER CODE BEGIN Includes */
#include "TM1637.h"
/* USER CODE END Includes */
```

### 3.2 初始化与测试
在 `main()` 函数的 `/* USER CODE BEGIN 2 */` 区域调用初始化函数：

```c
/* USER CODE BEGIN 2 */
// 初始化数码管
TM1637_Init();

// 设置亮度 (0-7)，默认是 7 (最亮)
TM1637_SetBrightness(2);

// 显示字符串测试
TM1637_DisplayString("1234");
HAL_Delay(1000);

// 显示带小数点的数字
TM1637_DisplayString("12.34");
/* USER CODE END 2 */
```

### 3.3 主循环调用 (可选)
你可以在 `while(1)` 中动态更新显示内容：

```c
/* USER CODE BEGIN WHILE */
int count = 0;
char buf[10];

while (1)
{
    // 格式化字符串
    sprintf(buf, "%4d", count); 
    
    // 显示数值
    TM1637_DisplayString(buf);
    
    count++;
    if (count > 9999) count = 0;
    
    HAL_Delay(100);
    
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */
```

## 4. API 函数说明

| 函数名 | 说明 | 参数 |
| :--- | :--- | :--- |
| `TM1637_Init` | 初始化引脚和模块 | 无 |
| `TM1637_SetBrightness` | 设置亮度 | `brightness`: 0 (最暗) ~ 7 (最亮) |
| `TM1637_DisplayString` | 显示字符串 | `str`: 要显示的字符串 (支持 0-9, A-F, 负号, 空格, 小数点) |
| `TM1637_Clear` | 清屏 | 无 |

## 5. 高级配置 (TM1637.h)

如果需要调整默认行为，可以修改 `TM1637.h` 中的宏定义：

*   **`TM1637_NO_INPUT_TEXT`**: 当传入空字符串时默认显示的内容（默认为 `"----"`）。
*   **`TM1637_DELAY_US`**: 通信时序延时。如果数码管显示乱码或闪烁，可以适当调大此值（默认 `50`us）。
*   **`TM1637_USE_OPEN_DRAIN`**:
    *   `0`: 使用推挽输出（默认，推荐）。
    *   `1`: 使用开漏输出（需要外部上拉电阻，适合 5V 电平匹配等特殊场景）。

---
**注意事项**：
*   请确保 `delay.h` 和 `delay_us()` 函数在你的工程中可用。如果你的工程没有微秒级延时函数，可能需要将 `TM1637.c` 中的 `delay_us(TM1637_DELAY_US)` 替换为简单的 `for` 循环延时或 HAL 库提供的其他延时方式。
