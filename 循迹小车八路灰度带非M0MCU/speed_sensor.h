/**
 * @file    speed_sensor.h
 * @brief   双电机霍尔编码器速度测量模块
 *
 * @note    编码器规格: 13 PPR, 1:20 减速比
 *          - MOTOR_1: EAA/PA17, EAB/PA21  (GPIOA, GROUP1)
 *          - MOTOR_2: EBA/PB0,  EBB/PB1   (GPIOB, GROUP1)
 *
 *          速度测量: 50ms 窗口统计脉冲 → RPM → 归一化 [0,1]
 *          距离测量: 仅使用 MOTOR_1 编码器累计脉冲 (GetTotalPulses)
 */

#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================
 * 编码器硬件参数
 *===========================================================================*/

#define ENCODER_PPR              13       /* 电机轴每转脉冲数            */
#define GEAR_RATIO               20       /* 减速比                       */
#define OUTPUT_PPR               (ENCODER_PPR * GEAR_RATIO)
#define MOTOR_MAX_RPM            8000.0f  /* 最大转速, 用于归一化       */

/*===========================================================================
 * 速度测量参数 (5ms 控制周期 × 20 = 100ms 窗口)
 *===========================================================================*/

#define SPEED_WINDOW_MS          50
#define SPEED_WINDOW_CYCLES      (SPEED_WINDOW_MS / 5)

/*===========================================================================
 * 方向定义 (与 motor_control.h 保持一致)
 *===========================================================================*/

#define ENCODER_DIR_STOP          0U
#define ENCODER_DIR_FORWARD       1U
#define ENCODER_DIR_REVERSE       2U

/*===========================================================================
 * 公开函数 — 电机 A
 *===========================================================================*/

void     SpeedSensorA_Init(void);
bool     SpeedSensorA_Update(void);        /* 每 5ms 调用                */
float    SpeedSensorA_GetSpeed(void);      /* 归一化速度 [0, 1]          */
float    SpeedSensorA_GetRPM(void);        /* 电机轴 RPM                 */
uint8_t  SpeedSensorA_GetDirection(void);
uint32_t SpeedSensorA_GetTotalPulses(void);

/*===========================================================================
 * 公开函数 — 电机 B
 *===========================================================================*/

void     SpeedSensorB_Init(void);
bool     SpeedSensorB_Update(void);
float    SpeedSensorB_GetSpeed(void);
float    SpeedSensorB_GetRPM(void);
uint8_t  SpeedSensorB_GetDirection(void);
uint32_t SpeedSensorB_GetTotalPulses(void);

/*===========================================================================
 * ISR 回调 (GROUP1_IRQHandler 内部调用)
 *===========================================================================*/

void SpeedSensor_EAA_ISR(void);   /* MOTOR_1 E1A (PA17) 上升沿         */
void SpeedSensor_EAB_ISR(void);   /* MOTOR_1 E1B (PA18) 上升沿         */
void SpeedSensor_EBA_ISR(void);   /* MOTOR_2 E2A (PB0)  上升沿         */
void SpeedSensor_EBB_ISR(void);   /* MOTOR_2 E2B (PB1)  上升沿         */

#endif /* SPEED_SENSOR_H */
