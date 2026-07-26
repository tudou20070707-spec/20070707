/**
 * @file    grayscale.h
 * @brief   八路灰度循迹传感器模块 (I2C通信)
 *
 * @note    传感器布局 (俯视，车头朝前):
 *          [X1] [X2] [X3] [X4] [X5] [X6] [X7] [X8]
 *          最左                                    最右
 *
 *          I2C 通信参数:
 *          - 设备地址: 0x40 (拨码开关 000, 未经串口改址)
 *          - 查询指令 0x0C: 返回校准后的8路数字量
 *          - 返回帧: 指令回显 + 数据长度 + 数据 + 累加校验和
 *          - bit0=X8(最右) ... bit7=X1(最左), 原始数据 0=黑线, 1=白色
 *          - 驱动层取反后: 1=黑线, 0=白
 *
 *          共享 I2C1 总线 (与 SSD1306 OLED 共用 PA10/PA11)
 *          模块校准通过板载按键完成。
 *
 *          使用方式:
 *          ─────────────────────────────────────────────
 *          Grayscale_Init();           // 等待模块上电稳定
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
#define GS_I2C_ADDR  0x40
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

/** @brief 灰度模块读取结果；通信错误不会伪装成丢线数据 */
typedef enum {
    GS_READ_OK = 0,
    GS_READ_INVALID_ARG,
    GS_READ_TX_ERROR,
    GS_READ_RX_ERROR,
    GS_READ_ECHO_ERROR,
    GS_READ_LENGTH_ERROR,
    GS_READ_CHECKSUM_ERROR
} GrayscaleReadStatus;

/*===========================================================================
 * 初始化
 *===========================================================================*/

/**
 * @brief 初始化灰度传感器
 * @note  等待模块上电稳定；黑白阈值由模块板载按键校准。
 *        调用前需确保 SYSCFG_DL_init() 已完成 I2C1 配置。
 */
void Grayscale_Init(void);

/*===========================================================================
 * 传感器读取
 *===========================================================================*/

/**
 * @brief 通过 I2C 指令 0x0C 读取校准后的 8 路数字量
 * @note  仅在返回 GS_READ_OK 时更新 sensor；通信/校验错误不会写入伪造数据。
 */
GrayscaleReadStatus Grayscale_Read(GrayscaleSensor *sensor);

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

/** @brief 把应用层传感器数据打包为 active-high 位图 (bit0=X1 ... bit7=X8) */
uint8_t Grayscale_ActiveMask(const GrayscaleSensor *sensor);

/** @brief 返回当前在线上的传感器数量 (调试用) */
uint8_t Grayscale_OnLineCount(const GrayscaleSensor *sensor);

#ifdef __cplusplus
}
#endif

#endif /* GRAYSCALE_H */
