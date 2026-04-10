/*============================== 文件3：ZPN_Uart.c ==============================*/

#include "ZPN_Uart.h"
#include "stdarg.h"
#include "string.h"
#include "stdio.h"

/* ==================== 私有类型定义 ==================== */
/**
 * @struct UART1_TypeDef
 * @brief UART1 接收DMA双缓冲结构体（完全封装）
 * @details 将DMA硬件缓冲区和用户处理缓冲区绑定管理
 */
typedef struct
{
    uint16_t ReceiveNum;                           // 接收字节数，0表示无新数据
    uint8_t ReceiveData[UART1_DMA_RX_BUFFER_SIZE]; // 用户处理缓冲区
    uint8_t BuffTemp[UART1_DMA_RX_BUFFER_SIZE];    // DMA硬件直接写入区
} UART1_TypeDef;

/**
 * @struct RingBuffer_t
 * @brief UART2环形缓冲区结构体
 */
typedef struct
{
    uint16_t Head;                             // 读指针
    uint16_t Tail;                             // 写指针
    volatile uint16_t Length;                  // 当前数据长度
    uint8_t Ring_data[UART2_RING_BUFFER_SIZE]; // 环形数据区
} RingBuffer_t;

/* ==================== 静态变量 ==================== */
// UART1 DMA资源（双缓冲结构体）
static UART1_TypeDef uart1_dma = {0};
static uint8_t uart1_tx_buffer[UART1_DMA_TX_BUFFER_SIZE];
static volatile uint8_t uart1_tx_busy = 0;

// UART2环形缓冲区资源
static RingBuffer_t uart2_ring = {0};
static uint8_t uart2_rx_buffer[1]; // 单字节接收缓冲
static uint8_t uart2_tx_buffer[UART2_TX_BUFFER_SIZE];
static volatile uint8_t uart2_tx_busy = 0;

//// UART3阻塞资源
//static uint8_t uart3_tx_buffer[UART3_TX_BUFFER_SIZE];
//static uint8_t uart3_rx_buffer[UART3_TX_BUFFER_SIZE];
//static uint16_t uart3_rx_index = 0;
//static volatile uint8_t uart3_rx_ready = 0;

volatile uint8_t pack_parse_pending = 0; //数据帧处理空闲标志位

/* ==================== 私有函数（静态封装） ==================== */
/**
 * @brief 向环形缓冲区写入一个字节（内部函数）
 * @param data: 要写入的字节
 */
static void RingBuffer_Write(uint8_t data)
{
    if (uart2_ring.Length >= UART2_RING_BUFFER_SIZE)
    {
        return; // 缓冲区满，丢弃新数据
    }
    uart2_ring.Ring_data[uart2_ring.Tail] = data;
    uart2_ring.Tail = (uart2_ring.Tail + 1) % UART2_RING_BUFFER_SIZE;
    uart2_ring.Length++;
}

static void RingBuffer_Init(void)
{
    uart2_ring.Head = 0;
    uart2_ring.Tail = 0;
    uart2_ring.Length = 0;
}


/* ==================== UART1 DMA接口实现 ==================== */
/**
 * @brief 通过UART1使用DMA发送格式化字符串（非阻塞）
 * @param fmt: 格式字符串 (printf风格)
 * @param ...: 可变参数
 * @retval 成功返回发送的字节数，失败返回负值 (-1: 串口忙)。
 *
 * // 示例：发送VOFA+CSV数据
VOFA_SendSamples(channels, 4);
  └──> UART1_DMAPrintf("1.234,5.678,9.012,3.456\n");
       └──> vsnprintf() 生成字符串到 uart1_tx_buffer
            └──> HAL_UART_Transmit_DMA() 发送
 */
int UART1_DMAPrintf(const char *fmt, ...)
{
    if (uart1_tx_busy)
        return -1;

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf((char *)uart1_tx_buffer, UART1_DMA_TX_BUFFER_SIZE, fmt, args);
    va_end(args);

    if (len > 0 && len < UART1_DMA_TX_BUFFER_SIZE)
    {
        uart1_tx_busy = 1;
        if (HAL_UART_Transmit_DMA(&huart1, uart1_tx_buffer, len) == HAL_OK)
        {
            return len;
        }
        uart1_tx_busy = 0;
        return -2;
    }
    return len;
}

/**
 * @brief 通过UART1使用DMA发送二进制数据（非阻塞）
 * @param data: 数据缓冲区指针。
 * @param length: 数据长度。
 * @retval 0:成功, -1:参数错误, -2:发送忙, -3:发送启动失败
 *
 * // 示例：发送VOFA+图片
VOFA_SendImageData(image_binary, 1024);
  └──> UART1_DMASendData(image_binary, 1024);
       └──> memcpy() 复制到 uart1_tx_buffer
            └──> HAL_UART_Transmit_DMA() 发送
 */
int UART1_DMASendData(const uint8_t *data, uint16_t length)
{
    if (uart1_tx_busy)
        return -2;
    if (data == NULL || length == 0 || length > UART1_DMA_TX_BUFFER_SIZE)
        return -1;

    uart1_tx_busy = 1;
    memcpy(uart1_tx_buffer, data, length);
    if (HAL_UART_Transmit_DMA(&huart1, uart1_tx_buffer, length) == HAL_OK)
    {
        return 0;
    }
    uart1_tx_busy = 0;
    return -3;
}

/**
 * @brief 检查UART1是否有新数据
 * @retval 1:有新数据, 0:无数据
 * @note 可配合轮询或在中断后查询使用。
 */
uint8_t UART1_IsDataReady(void) { return uart1_dma.ReceiveNum > 0; }

/**
 * @brief 获取UART1接收的数据
 * @details 检查接收标志，若有数据则拷贝到用户缓冲区，并清除标志。
 * @param buf: 用户数据缓冲区指针。
 * @param bufSize: 用户缓冲区大小（用于防止溢出）。
 * @retval 实际拷贝的字节数，0表示无新数据。
 * @note 读取后内部数据标志清零，需等待下次接收完成才能再次读到数据。
 */
uint16_t UART1_GetReceivedData(uint8_t *buf, uint16_t bufSize)
{
    if (uart1_dma.ReceiveNum == 0 || buf == NULL)
        return 0;

    uint16_t copyLen = (uart1_dma.ReceiveNum < bufSize) ? uart1_dma.ReceiveNum : bufSize;
    memcpy(buf, uart1_dma.ReceiveData, copyLen);
    uart1_dma.ReceiveNum = 0; // 清除标志
    return copyLen;
}

/* ==================== UART2 环形缓冲接口 ==================== */
int UART2_SendPrintf(const char *fmt, ...)
{
    if (uart2_tx_busy)
        return -1;

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf((char *)uart2_tx_buffer, UART2_TX_BUFFER_SIZE - HMI_CMD_TAIL_LEN, fmt, args);
    va_end(args);

    if (len > 0)
    {
        // 将协议尾"\xFF\xFF\xFF"追加到格式化字符串末尾
        memcpy(uart2_tx_buffer + len, HMI_CMD_TAIL, HMI_CMD_TAIL_LEN);
        uart2_tx_busy = 1;
        HAL_UART_Transmit_IT(&huart2, uart2_tx_buffer, len + HMI_CMD_TAIL_LEN);
        return len;
    }
    return len;
}

/**
 * @brief 通过UART2发送二进制数据（非阻塞）
 * @param data: 数据缓冲区指针。
 * @param length: 数据长度。
 * @retval 0:成功, -1:参数错误, -2:发送忙
 */
int UART2_SendData(const uint8_t *data, uint16_t length)
{
    if (uart2_tx_busy)
        return -2;
    if (data == NULL || length == 0 || length + HMI_CMD_TAIL_LEN > UART2_TX_BUFFER_SIZE)
        return -1;

    memcpy(uart2_tx_buffer, data, length);
    memcpy(uart2_tx_buffer + length, HMI_CMD_TAIL, HMI_CMD_TAIL_LEN);
    uart2_tx_busy = 1;
    HAL_UART_Transmit_IT(&huart2, uart2_tx_buffer, length + HMI_CMD_TAIL_LEN);
    return 0;
}

/**
 * @brief 获取串口屏接收缓冲区当前数据长度
 * @retval 未读取的字节数。
 */
uint16_t UART2_GetRxBufferLength(void) { return uart2_ring.Length; }

/**
 * @brief 从UART2环形缓冲区读取数据
 * @param buf: 用户数据缓冲区指针。
 * @param bufSize: 用户缓冲区大小。
 * @retval 实际读取的字节数。
 * @warning 本函数非线程安全，应在主循环或确保与中断无冲突的临界区调用。
 */
uint16_t UART2_GetReceivedData(uint8_t *buf, uint16_t bufSize)
{
    if (buf == NULL || bufSize == 0)
        return 0;

    __disable_irq();
    uint16_t readLen = (uart2_ring.Length < bufSize) ? uart2_ring.Length : bufSize;
    for (uint16_t i = 0; i < readLen; i++)
    {
        buf[i] = uart2_ring.Ring_data[uart2_ring.Head];
        uart2_ring.Head = (uart2_ring.Head + 1) % UART2_RING_BUFFER_SIZE;
    }
    uart2_ring.Length -= readLen;
    __enable_irq();

    return readLen;
}

///* ==================== UART3 阻塞接口 ==================== */
///**
// * @brief UART3阻塞式格式化打印（带换行）
// * @details 适用于调试串口，自动添加\r\n，调用时CPU阻塞等待发送完成
// * @warning 会阻塞50-100ms（115200波特率下），**严禁在实时任务中调用**
// */
//int UART3_Printf(const char *fmt, ...)
//{
//    char buffer[UART3_TX_BUFFER_SIZE];
//    va_list args;
//    va_start(args, fmt);
//    int len = vsnprintf(buffer, UART3_TX_BUFFER_SIZE - 2, fmt, args);
//    va_end(args);

//    if (len > 0)
//    {
//        buffer[len++] = '\r';
//        buffer[len++] = '\n';
//        HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, 100); // 阻塞发送
//        return len;
//    }
//    return len;
//}

///**
// * @brief UART3阻塞式发送原始数据
// * @warning 会阻塞，适合初始化阶段或错误处理
// */
//int UART3_SendData(const uint8_t *data, uint16_t length)
//{
//    if (data == NULL || length == 0)
//        return -1;
//    HAL_UART_Transmit(&huart3, data, length, 100); // 阻塞发送
//    return 0;
//}

/* ==================== HAL中断回调 ==================== */
/**
 * @brief UART发送完成中断回调
 * @param huart: 触发中断的UART句柄指针
 * @retval None
 * @note 用于清除UART1和UART2的发送忙标志，允许下一次发送。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart1_tx_busy = 0;
    }
    else if (huart->Instance == USART2)
    {
        // 串口屏发送完成后，允许下一次发送
        uart2_tx_busy = 0;
    }
}

/**
 * @brief UART DMA空闲中断回调
 * @param huart: 触发中断的UART句柄指针
 * @param Size: 本次接收的数据长度（字节）
 * @retval None
 * @note 仅用于UART1，实现不定长数据帧接收。
 *       采用双缓冲设计，接收完成后从DMA区转存到用户区。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        if (Size > 0 && Size <= sizeof(uart1_dma.ReceiveData))
        {
            // 拷贝数据
            memcpy(uart1_dma.ReceiveData, uart1_dma.BuffTemp, Size);
            uart1_dma.ReceiveNum = Size;

            // 清空剩余区域！
            if (Size < sizeof(uart1_dma.ReceiveData))
            {
                memset(&uart1_dma.ReceiveData[Size], 0,
                       sizeof(uart1_dma.ReceiveData) - Size);
            }
        }
        // 重启DMA
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_dma.BuffTemp, UART1_DMA_RX_BUFFER_SIZE);
    }
}


/**
 * @brief UART接收完成中断回调
 * @param huart: 触发中断的UART句柄指针
 * @retval None
 * @note 仅用于UART2（串口屏）的中断接收。每次接收1字节后，存入环形缓冲区并自动重启接收。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // 关中断保护写操作
        __disable_irq();
        RingBuffer_Write(uart2_rx_buffer[0]);
        uint16_t current_len = uart2_ring.Length;
        __enable_irq();

        if (current_len >= PACK_MIN_FRAME_SIZE)
        {
             // 使用我们定义的全局标志
             pack_parse_pending = 1; 
        }

        // 重新启动接收
        HAL_UART_Receive_IT(&huart2, uart2_rx_buffer, 1);
    }
}



/* ==================== 模块初始化（关键） ==================== */
// 在 ZPN_Uart.c 中定义（对外不可见）
// 初始化函数调用它
void ZPN_UART_Init(void)
{
#if ZPN_UART_GLOBAL_ENABLE
    // 基础资源清零
    uart1_dma.ReceiveNum = 0;
    uart1_tx_busy = 0;
    uart2_tx_busy = 0;
    RingBuffer_Init();

    // UART1根据模式启动不同接收
#if UART1_MODE == 1 || UART1_MODE == 0
    // 启用UART1接收功能
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_dma.BuffTemp, UART1_DMA_RX_BUFFER_SIZE);
#endif

    // UART2模式选择
#if UART2_MODE == 1
    // HMI模式：中断接收，可触发Pack解析
    HAL_UART_Receive_IT(&huart2, uart2_rx_buffer, 1);
#endif

#endif
}
