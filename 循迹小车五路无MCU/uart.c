/**
 * @file    uart.c
 * @brief   MSPM0 UART 通用串口模块实现 (中断接收 + 阻塞发送)
 *
 * @note    架构:
 *          ┌─────────────────────────────────────────────────┐
 *          │  TX (发送): 阻塞方式                             │
 *          │  DL_UART_transmitDataBlocking()  → 简单可靠      │
 *          │                                                 │
 *          │  RX (接收): 中断 + 环形缓冲区                     │
 *          │  UART_ISR() → rx_ring_buf[] → UART_ReadByte()   │
 *          │  硬件 FIFO → ISR 批量搬移 → 用户 API 按需读取     │
 *          └─────────────────────────────────────────────────┘
 *
 *          波特率公式:
 *            DIV  = BUSCLK / (16 * baud)
 *            IBRD = DIV 整数部分
 *            FBRD = round((DIV 小数部分) * 64)
 *
 *          移植清单:
 *            1. uart.h / uart.c 放入工程
 *            2. 确保定义了 UART_INST, UART_INST_FREQ, UART_INT_IRQN, UART_ISR
 *               (有 SysConfig PRINT 模块 → 自动沿用, 无须手动定义)
 *            3. delay.h / delay.c (仅 UART_ReadBytes 超时需要)
 */

#include "uart.h"
#include "delay.h"     /* delay_ms — 仅 ReadBytes 超时需要 */
#include <stdarg.h>    /* va_list — 仅 UART_Printf 需要 */

/*---------------------------------------------------------------------------
 * DriverLib 兼容层：适配不同 MSPM0 SDK 版本的 UART 宏名
 *---------------------------------------------------------------------------*/
#ifndef DL_UART_MAIN_IIDX_RX_TIMEOUT
    #define DL_UART_MAIN_IIDX_RX_TIMEOUT DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR
#endif

#ifndef DL_UART_MAIN_INTERRUPT_RX_TIMEOUT
    #define DL_UART_MAIN_INTERRUPT_RX_TIMEOUT DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR
#endif

#ifndef DL_UART_RX_FIFO_LEVEL_BYTES_1
    #define DL_UART_RX_FIFO_LEVEL_BYTES_1 DL_UART_RX_FIFO_LEVEL_ONE_ENTRY
#endif

/*===========================================================================
 * 环形缓冲区 (RX Ring Buffer) — ISR 写入, 用户 API 读取
 *===========================================================================*/

/* 利用 2 的幂大小做快速取模: idx & (size-1) 代替 idx % size */
#define RB_MASK  (UART_RX_BUF_SIZE - 1)

#if (UART_RX_BUF_SIZE & (UART_RX_BUF_SIZE - 1)) != 0
  #error "UART_RX_BUF_SIZE 必须是 2 的幂 (如 64, 128, 256)"
#endif

static volatile uint8_t  rb_buf[UART_RX_BUF_SIZE];  /* 环形缓冲区存储        */
static volatile uint16_t rb_head = 0;               /* ISR 写入位置          */
static volatile uint16_t rb_tail = 0;               /* 用户读取位置          */

/* ---- 环形缓冲区内部辅助 (带临界区保护, 防止 ISR 与主循环竞态) ---- */

/** @brief 缓冲区是否为空 (head == tail) */
static inline bool rb_is_empty(void)
{
    return (rb_head == rb_tail);
}

/** @brief 缓冲区是否已满 ((head + 1) % size == tail) */
static inline bool rb_is_full(void)
{
    return (((rb_head + 1) & RB_MASK) == rb_tail);
}

/** @brief ISR 向缓冲区写入一个字节 (满则丢弃) */
static inline void rb_put(uint8_t byte)
{
    uint16_t next = (rb_head + 1) & RB_MASK;
    if (rb_is_full()) {
        /* 缓冲区满 — 丢弃最旧字节 (保证最新数据不丢) */
        rb_tail = (rb_tail + 1) & RB_MASK;
    }
    rb_buf[rb_head] = byte;
    rb_head = next;
}

/** @brief 用户从缓冲区读取一个字节 (空则返回 0) */
static inline uint8_t rb_get(void)
{
    uint8_t byte = rb_buf[rb_tail];
    rb_tail = (rb_tail + 1) & RB_MASK;
    return byte;
}

/** @brief 返回缓冲区中当前缓存的字节数 */
static inline uint16_t rb_count(void)
{
    return (rb_head - rb_tail) & RB_MASK;
}

/*===========================================================================
 * UART 接收中断服务函数 (ISR)
 *
 * 弱定义 — 如果用户在自己的代码中也定义了同名 ISR, 链接器会优先选择
 * 用户的版本, 本弱定义被忽略。这允许用户自定义中断处理而又不删本文件。
 *===========================================================================*/

/**
 * @brief  UART 接收中断服务函数 (弱定义)
 *
 * @note   自动从 UART_ISR 宏获取函数名 (默认 = UART0_IRQHandler)。
 *         每次触发时从 RX FIFO 批量读出数据, 塞入环形缓冲区。
 *
 *         如果你需要自定义 ISR (例如同时处理 TX 中断或错误),
 *         只需在自己的代码中定义一个同名的非 weak 函数, 本函数即被覆盖。
 *
 *         典型替换写法:
 *         void UART0_IRQHandler(void) {
 *             // 你的自定义逻辑 ...
 *             // 最后调用 uart_rx_isr_callback() 维持环形缓冲区工作
 *         }
 */
SYSCONFIG_WEAK void UART_ISR(void)
{
    /*
     * 读取中断号来判断中断源。
     * IID (Interrupt ID) 告诉我们是 RX / TX / 错误等。
     */
    uint8_t iid = DL_UART_Main_getPendingInterrupt(UART_INST);

    switch (iid) {
    case DL_UART_MAIN_IIDX_RX:
        /*
         * RX 中断 — FIFO 达到阈值 (当前配为 ≥1 字节即触发)。
         * 循环读取直到 FIFO 为空, 避免因 ISR 返回延迟导致溢出。
         */
        while (!DL_UART_Main_isRXFIFOEmpty(UART_INST)) {
            uint8_t byte = DL_UART_receiveDataBlocking(UART_INST);
            rb_put(byte);
        }
        break;

    case DL_UART_MAIN_IIDX_RX_TIMEOUT:
        /*
         * RX 超时中断 — FIFO 不满阈值但数据已停顿。
         * 与 RX 中断处理相同: 把 FIFO 中剩余数据搬空。
         */
        while (!DL_UART_Main_isRXFIFOEmpty(UART_INST)) {
            uint8_t byte = DL_UART_receiveDataBlocking(UART_INST);
            rb_put(byte);
        }
        break;

    case DL_UART_MAIN_IIDX_TX:
        /* TX 中断 — 当前不使用 (发送走阻塞方式), 仅清标志 */
        break;

    default:
        /*
         * 其他中断 (如错误中断): 这里可以加日志或错误处理。
         * 目前仅清标志, 避免反复进入 ISR。
         */
        break;
    }

    /*
     * 部分 SDK 版本需要手动清除综合中断状态。
     * DL_UART_Main_clearInterruptStatus() 清除指定 IID 对应的中断标志。
     */
    DL_UART_Main_clearInterruptStatus(UART_INST, iid);
}

/*===========================================================================
 * 内部辅助 — 波特率分频计算
 *===========================================================================*/

static void UART_CalcDividers(uint32_t baud, uint32_t *ibrd, uint32_t *fbrd)
{
    uint32_t div_16x  = 16UL * baud;
    uint32_t quotient  = UART_INST_FREQ / div_16x;
    uint32_t remainder = UART_INST_FREQ % div_16x;

    *ibrd = quotient;

    /* FBRD = round(remainder / div_16x * 64) */
    uint32_t fbrd_raw = (remainder * 64UL + div_16x / 2UL) / div_16x;
    if (fbrd_raw >= 64UL) {
        *ibrd += 1;
        *fbrd = 0;
    } else {
        *fbrd = fbrd_raw;
    }
}

/*===========================================================================
 * 初始化
 *===========================================================================*/

void UART_Init(uint32_t baudRate)
{
    uint32_t ibrd, fbrd;

    /* 1. 复位并上电 */
    DL_UART_Main_reset(UART_INST);
    DL_UART_Main_enablePower(UART_INST);

    /* 2. 时钟: BUSCLK, 不分频 */
    static const DL_UART_Main_ClockConfig clockCfg = {
        .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1,
    };
    DL_UART_Main_setClockConfig(UART_INST,
        (DL_UART_Main_ClockConfig *)&clockCfg);

    /* 3. 帧格式: 8N1, 全双工, 无流控 */
    static const DL_UART_Main_Config frameCfg = {
        .mode        = DL_UART_MAIN_MODE_NORMAL,
        .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity      = DL_UART_MAIN_PARITY_NONE,
        .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits    = DL_UART_MAIN_STOP_BITS_ONE,
    };
    DL_UART_Main_init(UART_INST, (DL_UART_Main_Config *)&frameCfg);

    /* 4. 波特率 */
    DL_UART_Main_setOversampling(UART_INST, DL_UART_OVERSAMPLING_RATE_16X);
    UART_CalcDividers(baudRate, &ibrd, &fbrd);
    DL_UART_Main_setBaudRateDivisor(UART_INST, ibrd, fbrd);

    /* 5. 使能 */
    DL_UART_Main_enable(UART_INST);
}

/**
 * @brief  开启 UART 接收中断
 *
 * @note   调用前需确保 UART 已初始化 (SYSCFG_DL_init 或 UART_Init)。
 *         调用后, 硬件收到数据 → 自动触发 ISR → 数据进入环形缓冲区。
 *         用户通过 UART_Available() / UART_ReadByte() 从缓冲区取数据。
 */
void UART_EnableRXInt(void)
{
    /* 清空环形缓冲区, 避免旧数据干扰 */
    rb_head = 0;
    rb_tail = 0;

    /*
     * 设置 RX FIFO 阈值为 1 字节 — 每收到 1 字节就触发中断。
     * 你可以调高阈值 (如 4 / 8) 来减少中断频率, 但会增加单字节延迟。
     * 注意: 部分 SDK 版本函数名可能是 DL_UART_Main_setRXFIFOThreshold
     */
    DL_UART_Main_setRXFIFOThreshold(UART_INST, DL_UART_RX_FIFO_LEVEL_BYTES_1);

    /*
     * 使能 RX 和 RX_TIMEOUT 两个中断源:
     * - RX:        FIFO 达到阈值时触发
     * - RX_TIMEOUT: FIFO 不满阈值但数据停顿超时, 确保残量数据不丢
     */
    DL_UART_Main_enableInterrupt(UART_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT);

    /* 在 NVIC 中使能该 UART 的中断线 */
    NVIC_EnableIRQ((IRQn_Type)UART_INT_IRQN);
}

/**
 * @brief  关闭 UART 接收中断 (恢复为仅发送模式)
 */
void UART_DisableRXInt(void)
{
    NVIC_DisableIRQ((IRQn_Type)UART_INT_IRQN);

    DL_UART_Main_disableInterrupt(UART_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT);
}

/*===========================================================================
 * 发送 (阻塞方式)
 *===========================================================================*/

void UART_SendByte(uint8_t byte)
{
    DL_UART_transmitDataBlocking(UART_INST, byte);
}

void UART_SendBytes(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        DL_UART_transmitDataBlocking(UART_INST, data[i]);
    }
}

void UART_SendString(const char *str)
{
    while (*str != '\0') {
        DL_UART_transmitDataBlocking(UART_INST, (uint8_t)(*str));
        str++;
    }
}

/*===========================================================================
 * 接收 (从环形缓冲区读取)
 *===========================================================================*/

/**
 * @brief  从环形缓冲区读取一个字节 (阻塞, 无超时)
 *
 * @note   如果缓冲区为空, 此函数会一直等待 ISR 写入数据。
 *         不想阻塞 → 先用 UART_Available() 判断再读。
 */
uint8_t UART_ReadByte(void)
{
    /* 空等待 — 等 ISR 写入至少 1 字节 */
    while (rb_is_empty()) {
        /* 无数据: 可在此处加 WFI 降低功耗 (__WFI()) */
    }
    return rb_get();
}

/**
 * @brief  从环形缓冲区读取指定长度 (带字节间超时)
 * @param  buf        接收缓冲区
 * @param  len        期望读取字节数
 * @param  timeoutMs  字节间超时 ms, 0 = 无限等待
 * @return 实际读取的字节数
 *
 * @note   超时从"上一个字节收到后"开始计时，每收到一个字节重置。
 *         如果在 timeoutMs 内没有新数据到来, 提前返回。
 *
 *         例: len=10, timeoutMs=50
 *           → 收到 6 字节后等了 50ms 还没第 7 个 → 返回 6
 */
uint16_t UART_ReadBytes(uint8_t *buf, uint16_t len, uint32_t timeoutMs)
{
    uint16_t received = 0;

    while (received < len) {
        /* 有数据 — 立即读取并重置等待 */
        if (!rb_is_empty()) {
            buf[received] = rb_get();
            received++;
            continue;
        }

        /* 无数据 且 timeout == 0 → 无限等 (一直轮询环形缓冲区) */
        if (timeoutMs == 0) {
            continue;
        }

        /* 无数据 → 1ms 粒度轮询, 超时退出 */
        delay_ms(1);
        if (timeoutMs <= 1) {
            break;
        }
        timeoutMs--;
    }

    return received;
}

/** @brief 环形缓冲区中是否有数据可读 (非阻塞) */
bool UART_Available(void)
{
    return !rb_is_empty();
}

/** @brief 返回环形缓冲区中当前缓存的字节数 */
uint16_t UART_AvailableCount(void)
{
    return rb_count();
}

/** @brief 清空环形缓冲区 (丢弃所有已缓冲但未读取的数据) */
void UART_FlushRX(void)
{
    /*
     * 临界区: 先关中断, 清空, 再开中断。
     * 用 CPSID / CPSIE 或 __disable_irq() / __enable_irq()。
     *
     * 这里用 NVIC 级别的开关 (只影响本 UART 中断),
     * 不影响其他外设中断。
     */
    NVIC_DisableIRQ((IRQn_Type)UART_INT_IRQN);
    rb_head = 0;
    rb_tail = 0;
    NVIC_EnableIRQ((IRQn_Type)UART_INT_IRQN);
}

/*===========================================================================
 * 运行时配置
 *===========================================================================*/

void UART_SetBaudRate(uint32_t baudRate)
{
    uint32_t ibrd, fbrd;

    DL_UART_Main_disable(UART_INST);

    DL_UART_Main_setOversampling(UART_INST, DL_UART_OVERSAMPLING_RATE_16X);
    UART_CalcDividers(baudRate, &ibrd, &fbrd);
    DL_UART_Main_setBaudRateDivisor(UART_INST, ibrd, fbrd);

    DL_UART_Main_enable(UART_INST);
}

/*===========================================================================
 * 格式化输出 (printf)
 *===========================================================================*/

#ifndef UART_NO_PRINTF

#ifndef UART_PRINTF_BUF_SIZE
  #define UART_PRINTF_BUF_SIZE  128
#endif

void UART_Printf(const char *fmt, ...)
{
    char buf[UART_PRINTF_BUF_SIZE];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    buf[sizeof(buf) - 1] = '\0';

    UART_SendString(buf);
}

#endif /* UART_NO_PRINTF */
