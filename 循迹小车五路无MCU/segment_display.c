/**
 * @file    segment_display.c
 * @brief   一位七段数码管模块实现
 */

#include "segment_display.h"
#include "ti_msp_dl_config.h"

/*===========================================================================
 * 段码表
 *===========================================================================
 * 段位映射: PA7=G, PA6=F, PA5=E, PA4=D, PA3=C, PA2=B, PA1=A
 *
 * - 共阳 (SEG_TYPE=1): 输出 LOW=亮段, HIGH=灭段
 *   → 初始化时写 seg_on_mask 到 DOUT: 亮段=LOW, 灭段=HIGH
 *
 * - 共阴 (SEG_TYPE=0): 输出 HIGH=亮段, LOW=灭段
 *   → 使用相反的 mask
 *===========================================================================*/

#define SEG_ALL (SEG_PIN_A | SEG_PIN_B | SEG_PIN_C | SEG_PIN_D \
                 | SEG_PIN_E | SEG_PIN_F | SEG_PIN_G)

/* 每段点亮时对应的 GPIO 位掩码 */
static const uint8_t seg_on_mask[10] = {
    /* 0: ABCDEF */
    (SEG_PIN_A|SEG_PIN_B|SEG_PIN_C|SEG_PIN_D|SEG_PIN_E|SEG_PIN_F),
    /* 1: BC     */
    (SEG_PIN_B|SEG_PIN_C),
    /* 2: ABDEG  */
    (SEG_PIN_A|SEG_PIN_B|SEG_PIN_D|SEG_PIN_E|SEG_PIN_G),
    /* 3: ABCDG  */
    (SEG_PIN_A|SEG_PIN_B|SEG_PIN_C|SEG_PIN_D|SEG_PIN_G),
    /* 4: BCFG   */
    (SEG_PIN_B|SEG_PIN_C|SEG_PIN_F|SEG_PIN_G),
    /* 5: ACDFG  */
    (SEG_PIN_A|SEG_PIN_C|SEG_PIN_D|SEG_PIN_F|SEG_PIN_G),
    /* 6: ACDEFG */
    (SEG_PIN_A|SEG_PIN_C|SEG_PIN_D|SEG_PIN_E|SEG_PIN_F|SEG_PIN_G),
    /* 7: ABC    */
    (SEG_PIN_A|SEG_PIN_B|SEG_PIN_C),
    /* 8: ABCDEFG*/
    (SEG_PIN_A|SEG_PIN_B|SEG_PIN_C|SEG_PIN_D|SEG_PIN_E|SEG_PIN_F|SEG_PIN_G),
    /* 9: ABCDFG */
    (SEG_PIN_A|SEG_PIN_B|SEG_PIN_C|SEG_PIN_D|SEG_PIN_F|SEG_PIN_G),
};

/*===========================================================================
 * 公开接口
 *===========================================================================*/

void SegDisplay_Init(void)
{
    /*
     * SysConfig 已配置 PA1~PA7 为 GPIO 输出。
     * 这里只确保输出使能并初始灭灯, 不覆盖 IOMUX。
     */
    for (int i = 1; i <= 7; i++) {
        GPIOA->DOE31_0 |= (1U << i);
    }

    SegDisplay_Off();
}

void SegDisplay_Show(uint8_t digit)
{
    if (digit > 9) {
        SegDisplay_Off();
        return;
    }

#if SEG_TYPE == 1
    /* 共阳: 先全灭(HIGH), 再拉低亮段 */
    DL_GPIO_setPins(SEG_PORT, SEG_ALL);
    DL_GPIO_clearPins(SEG_PORT, seg_on_mask[digit]);
#else
    /* 共阴: 先全灭(LOW), 再拉高亮段 */
    DL_GPIO_clearPins(SEG_PORT, SEG_ALL);
    DL_GPIO_setPins(SEG_PORT, seg_on_mask[digit]);
#endif
}

void SegDisplay_Off(void)
{
#if SEG_TYPE == 1
    DL_GPIO_setPins(SEG_PORT, SEG_ALL);     /* 共阳: HIGH=灭 */
#else
    DL_GPIO_clearPins(SEG_PORT, SEG_ALL);   /* 共阴: LOW=灭 */
#endif
}
