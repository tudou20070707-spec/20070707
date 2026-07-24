/**
 * @file    grayscale.h
 * @brief   八路灰度循迹传感器模块 (I2C通信)
 *
 * @note    传感器布局 (俯视，车头朝前):
 *          [X1] [X2] [X3] [X4] [X5] [X6] [X7] [X8]
 *          最左                                    最右
 *
 *          I2C 通信参数:
 *          - 设备地址: 0x12
 *          - 寄存器 0x30 (只读): 8位探头状态, bit0=X1 ... bit7=X8
 *            bit=1 表示检测到黑线 (亮灯)
 *          - 寄存器 0x01 (只写): 1=进入校准, 0=退出校准
 *          - 白线=0, 黑线=1
 *
 *          共享 I2C1 总线 (与 SSD1306 OLED 共用 PA10/PA11)
 *
 *          使用方式:
 *          ─────────────────────────────────────────────
 *          Grayscale_Init();           // 自动触发校准
 *          GrayscaleSensor sensors;
 *          Grayscale_Read(&sensors);   // I2C 读取 8 路
 *          float error = Grayscale_ComputeError(&sensors);
 *          bool lost   = Grayscale_IsLineLost(&sensors);
 *          ─────────────────────────────────────────────
 */

#ifndef GRAYSCALE_H
#define GRAYSCALE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 可配置参数
 *===========================================================================*/

/** @brief 八路灰度传感器 I2C 从机地址 (7位) */
#ifndef GS_I2C_ADDR
#define GS_I2C_ADDR  0x12
#endif

/** @brief 传感器数量 */
#define GS_COUNT  8

/*===========================================================================
 * 传感器数据结构
 *===========================================================================*/

/** @brief 八路传感器原始读数 (X1=最左, X8=最右, 1=检测到黑线) */
typedef struct {
    uint8_t x1;     /**< 最左侧  bit0 */
    uint8_t x2;     /**<         bit1 */
    uint8_t x3;     /**<         bit2 */
    uint8_t x4;     /**<         bit3 */
    uint8_t x5;     /**<         bit4 */
    uint8_t x6;     /**<         bit5 */
    uint8_t x7;     /**<         bit6 */
    uint8_t x8;     /**< 最右侧  bit7 */
} GrayscaleSensor;

/*===========================================================================
 * 初始化
 *===========================================================================*/

/**
 * @brief 初始化灰度传感器 (触发自动校准)
 * @note  会在内部执行校准流程: 写寄存器0x01=1 → 等待 → 写0x01=0
 *         调用前需确保 SYSCFG_DL_init() 已完成 I2C1 配置
 */
void Grayscale_Init(void);

/*===========================================================================
 * 传感器读取
 *===========================================================================*/

/** @brief 通过 I2C 读取全部 8 路传感器的当前值 (读寄存器 0x30) */
void Grayscale_Read(GrayscaleSensor *sensor);

/*===========================================================================
 * 循迹算法
 *===========================================================================*/

/**
 * @brief 计算加权位置误差
 * @param sensor  当前传感器读数
 * @return 误差值 [-1.0, +1.0]
 *         负值 = 线偏左 (X1-X4 检测到线), 正值 = 线偏右 (X5-X8 检测到线)
 *         0 = 居中
 *         权重: X1=-7, X2=-5, X3=-3, X4=-1, X5=+1, X6=+3, X7=+5, X8=+7
 */
float Grayscale_ComputeError(const GrayscaleSensor *sensor);

/** @brief 判断是否完全脱离轨迹线 (8路全部不在线上) */
bool Grayscale_IsLineLost(const GrayscaleSensor *sensor);

/** @brief 判断是否检测到急弯 (只有最外侧传感器 X1 或 X8 在线) */
bool Grayscale_IsSharpTurn(const GrayscaleSensor *sensor);

/** @brief 获取最外侧触发的方向: 返回 -1=最左(X1), 0=非急弯, +1=最右(X8) */
int  Grayscale_SharpTurnDir(const GrayscaleSensor *sensor);

/** @brief 返回当前在线上的传感器数量 (调试用) */
uint8_t Grayscale_OnLineCount(const GrayscaleSensor *sensor);

#ifdef __cplusplus
}
#endif

#endif /* GRAYSCALE_H */
