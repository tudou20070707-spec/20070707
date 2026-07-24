/**
 * @file    uart.h
 * @brief   MSPM0 UART 通用串口模块 (基于 TI DriverLib)
 *
 * @note    接收使用中断 + 环形缓冲区，发送使用阻塞方式。
 *          移植到新项目时，定义 UART_INST / UART_INST_FREQ 即可。
 *
 *          ╔══════════════════════════════════════════════════════════════╗
 *          ║  关于 UART_Init() 和 SYSCFG_DL_init() 的关系:              ║
 *          ║                                                            ║
 *          ║  如果 SysConfig 已配置了 PRINT/UART 模块，且你只使用发    ║
 *          ║  送功能 → 不需要调 UART_Init()，SYSCFG_DL_init() 已经     ║
 *          ║  配好了 TX/RX。想用中断接收 → 调 UART_EnableRXInt()。     ║
 *          ║                                                            ║
 *          ║  UART_Init() 的定位：无 SysConfig 时独立完成初始化。       ║
 *          ╚══════════════════════════════════════════════════════════════╝
 *
 *          典型用法 (SysConfig 已配 UART):
 *          ─────────────────────────────────────────────
 *          SYSCFG_DL_init();                          // 含 UART 初始化
 *          UART_EnableRXInt();                        // 开启接收中断
 *          UART_SendString("Hello UART!\r\n");
 *          UART_Printf("Counter = %d\r\n", count);
 *          if (UART_Available()) {
 *              uint8_t ch = UART_ReadByte();          // 从环形缓冲区读
 *          }
 *          ─────────────────────────────────────────────
 *
 *          典型用法 (无 SysConfig, 独立使用):
 *          ─────────────────────────────────────────────
 *          UART_Init(115200);
 *          UART_EnableRXInt();
 *          UART_SendString("Hello!\r\n");
 *          ─────────────────────────────────────────────
 */

#ifndef UART_H
#define UART_H

#include "Debug/ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/*===========================================================================
 * 可覆盖的默认配置 — 新项目在 ti_msp_dl_config.h 或本文件顶部覆盖
 *===========================================================================*/

/** @brief UART 外设实例 (默认沿用 SysConfig 的 PRINT 模块) */
#ifndef UART_INST
  #ifdef PRINT_INST
    #define UART_INST               PRINT_INST
  #else
    #error "[uart.h] 请定义 UART_INST (例如 #define UART_INST UART0)"
  #endif
#endif

/** @brief UART 模块总线时钟频率 Hz (默认沿用 SysConfig) */
#ifndef UART_INST_FREQ
  #ifdef PRINT_INST_FREQUENCY
    #define UART_INST_FREQ          PRINT_INST_FREQUENCY
  #else
    #define UART_INST_FREQ          40000000UL
  #endif
#endif

/** @brief UART 中断号 (默认沿用 SysConfig) */
#ifndef UART_INT_IRQN
  #ifdef PRINT_INST_INT_IRQN
    #define UART_INT_IRQN           PRINT_INST_INT_IRQN
  #else
    #define UART_INT_IRQN           UART0_INT_IRQn
  #endif
#endif

/** @brief UART ISR 函数名 (默认沿用 SysConfig 生成的宏) */
#ifndef UART_ISR
  #ifdef PRINT_INST_IRQHandler
    #define UART_ISR                PRINT_INST_IRQHandler
  #else
    #define UART_ISR                UART0_IRQHandler
  #endif
#endif

/** @brief 默认波特率 */
#ifndef UART_BAUD_RATE
  #define UART_BAUD_RATE            115200UL
#endif

/** @brief 接收环形缓冲区大小 (必须是 2 的幂, 如 64 / 128 / 256) */
#ifndef UART_RX_BUF_SIZE
  #define UART_RX_BUF_SIZE          256
#endif

/** @brief 接收超时 ms (用于 UART_ReadBytes 的字节间超时) */
#ifndef UART_RX_TIMEOUT_MS
  #define UART_RX_TIMEOUT_MS        100UL
#endif

/*===========================================================================
 * 初始化
 *===========================================================================*/

void UART_Init(uint32_t baudRate);
void UART_EnableRXInt(void);
void UART_DisableRXInt(void);

/*===========================================================================
 * 发送 (阻塞方式 — 简单可靠)
 *===========================================================================*/

void UART_SendByte(uint8_t byte);
void UART_SendBytes(const uint8_t *data, uint16_t len);
void UART_SendString(const char *str);

/*===========================================================================
 * 接收 (从环形缓冲区读取 — ISR 负责填充)
 *===========================================================================*/

/** @brief 从环形缓冲区读一个字节 (无数据时阻塞, 慎用) */
uint8_t UART_ReadByte(void);

/**
 * @brief  从环形缓冲区读取指定长度的字节
 * @param  buf        接收缓冲区
 * @param  len        期望读取字节数
 * @param  timeoutMs  字节间超时 ms (0=无限等待)
 * @return 实际读到的字节数
 */
uint16_t UART_ReadBytes(uint8_t *buf, uint16_t len, uint32_t timeoutMs);

/** @brief 查询环形缓冲区中是否有数据可读 (非阻塞) */
bool UART_Available(void);

/** @brief 返回环形缓冲区中当前缓存的字节数 */
uint16_t UART_AvailableCount(void);

/** @brief 清空环形缓冲区 (丢弃所有已缓存数据) */
void UART_FlushRX(void);

/*===========================================================================
 * 运行时配置
 *===========================================================================*/

void UART_SetBaudRate(uint32_t baudRate);

/*===========================================================================
 * 格式化输出 (可选 — 定义 UART_NO_PRINTF 可禁用)
 *===========================================================================*/

#ifndef UART_NO_PRINTF
  #include <stdio.h>
  void UART_Printf(const char *fmt, ...);
#endif

#endif /* UART_H */
