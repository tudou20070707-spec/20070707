/**
 * @file    segment_display.h
 * @brief   一位 0.56" 七段数码管模块 (共阳/共阴可配)
 *
 * @note    引脚: PA1=A, PA2=B, PA3=C, PA4=D, PA5=E, PA6=F, PA7=G
 *          限流电阻: 220Ω ~ 330Ω 串联各段
 *
 *          使用方式:
 *          ─────────────────────────────────────────────
 *          SegDisplay_Init();
 *          SegDisplay_Show(5);   // 显示 "5"
 *          SegDisplay_Off();     // 熄灭
 *          ─────────────────────────────────────────────
 */

#ifndef SEGMENT_DISPLAY_H
#define SEGMENT_DISPLAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 硬件配置
 *===========================================================================*/

/** @brief 数码管类型: 0=共阴, 1=共阳 (5611BS 共阳) */
#ifndef SEG_TYPE
#define SEG_TYPE            1       /* 1=共阳 (5611BS), 0=共阴 (5611AS) */
#endif

/** @brief 段选引脚 (同一 GPIO 端口) */
#define SEG_PORT            GPIOA
#define SEG_PIN_A           DL_GPIO_PIN_1    /* A 段 */
#define SEG_PIN_B           DL_GPIO_PIN_2    /* B 段 */
#define SEG_PIN_C           DL_GPIO_PIN_3    /* C 段 */
#define SEG_PIN_D           DL_GPIO_PIN_4    /* D 段 */
#define SEG_PIN_E           DL_GPIO_PIN_5    /* E 段 */
#define SEG_PIN_F           DL_GPIO_PIN_6    /* F 段 */
#define SEG_PIN_G           DL_GPIO_PIN_7    /* G 段 */

/*===========================================================================
 * 公开接口
 *===========================================================================*/

/**
 * @brief 初始化 7 段数码管 GPIO
 * @note  在 SYSCFG_DL_init() 之后调用。
 *        配置 PA1~PA7 为推挽输出, 初始全灭。
 *        调用前确保 SysConfig 中这些引脚未分配给其他外设。
 */
void SegDisplay_Init(void);

/**
 * @brief 显示单个数字
 * @param digit  0~9, 超出范围自动熄灭
 */
void SegDisplay_Show(uint8_t digit);

/** @brief 熄灭全部段 */
void SegDisplay_Off(void);

#ifdef __cplusplus
}
#endif

#endif /* SEGMENT_DISPLAY_H */
