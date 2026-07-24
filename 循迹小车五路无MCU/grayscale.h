/**
 * @file    grayscale.h
 * @brief   五路灰度循迹传感器模块
 *
 * @note    传感器布局 (俯视，车头朝前):
 *          [L2] [L1] [M] [R1] [R2]
 *
 *          数字量输出: 0=检测到黑线, 1=未检测到(白色地面)
 *          (取决于模块设计，可通过 GRAYSCALE_INVERT 反转逻辑)
 *
 *          使用方式:
 *          ─────────────────────────────────────────────
 *          Grayscale_Init( ...5个引脚的端口和掩码... );
 *          GrayscaleSensor sensors;
 *          Grayscale_Read(&sensors);
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

/** @brief 传感器逻辑是否取反
 *  0 = 读到 1 表示在黑线上 (模块自带比较器, 黑线→HIGH)
 *  1 = 读到 0 表示在黑线上 (模块输出低电平有效)
 *  根据实际模块表现修改此值
 */
#ifndef GRAYSCALE_INVERT
#define GRAYSCALE_INVERT  1    /* 1=读到0表示在黑线上 (常见模块行为) */
#endif

/*===========================================================================
 * 传感器数据结构
 *===========================================================================*/

/** @brief 五路传感器原始读数 */
typedef struct {
    uint8_t r2;     /**< 最右侧  1=线上, 0=线外 */
    uint8_t r1;     /**< 右中                      */
    uint8_t m;      /**< 中间                      */
    uint8_t l1;     /**< 左中                      */
    uint8_t l2;     /**< 最左侧                    */
} GrayscaleSensor;

/*===========================================================================
 * 初始化
 *===========================================================================*/

/**
 * @brief 初始化五路灰度传感器引脚
 * @param r2_port, r2_pin  最右侧传感器 GPIO
 * @param r1_port, r1_pin  右中传感器 GPIO
 * @param m_port,  m_pin   中间传感器 GPIO
 * @param l1_port, l1_pin  左中传感器 GPIO
 * @param l2_port, l2_pin  最左侧传感器 GPIO
 *
 * 引脚应在 SysConfig 中配置为 Digital Input + Pull-Up + Hysteresis。
 * 调用者将 SysConfig 生成的 _PORT / _PIN 宏传入即可。
 */
void Grayscale_Init(
    void    *r2_port, uint32_t r2_pin,
    void    *r1_port, uint32_t r1_pin,
    void    *m_port,  uint32_t m_pin,
    void    *l1_port, uint32_t l1_pin,
    void    *l2_port, uint32_t l2_pin);

/*===========================================================================
 * 传感器读取
 *===========================================================================*/

/** @brief 读取全部 5 路传感器的当前值 */
void Grayscale_Read(GrayscaleSensor *sensor);

/*===========================================================================
 * 循迹算法
 *===========================================================================*/

/**
 * @brief 计算加权位置误差
 * @param sensor  当前传感器读数
 * @return 误差值 [-1.0, +1.0]
 *         负值 = 线偏左, 正值 = 线偏右, 0 = 居中
 *         权重: L2=-4, L1=-2, M=0, R1=+2, R2=+4
 */
float Grayscale_ComputeError(const GrayscaleSensor *sensor);

/** @brief 判断是否完全脱离轨迹线 (5路全部不在线上) */
bool Grayscale_IsLineLost(const GrayscaleSensor *sensor);

/** @brief 判断是否检测到急弯 (只有最外侧传感器在线) */
bool Grayscale_IsSharpTurn(const GrayscaleSensor *sensor);

/** @brief 获取最外侧触发的方向: 返回 -1=最左(L2), 0=非急弯, +1=最右(R2) */
int  Grayscale_SharpTurnDir(const GrayscaleSensor *sensor);

/** @brief 返回当前在线上的传感器数量 (调试用) */
uint8_t Grayscale_OnLineCount(const GrayscaleSensor *sensor);

#ifdef __cplusplus
}
#endif

#endif /* GRAYSCALE_H */
