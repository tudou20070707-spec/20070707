#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include <stdint.h>
#include "pid_controller.h"
#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------
 * 直流电机控制模块（支持多电机）
 *---------------------------------------------------------------------------
 * PWM 由主循环中的 GPIO 软件 PWM 实现, 不再依赖定时器 CCP 输出。
 *---------------------------------------------------------------------------*/

#define MOTOR_DIR_STOP      0U
#define MOTOR_DIR_FORWARD   1U
#define MOTOR_DIR_REVERSE   2U

typedef struct {
    PIDController pid;
    float target_speed;
    float measured_speed;
    float duty_cycle;
    uint8_t direction;

    /* ---- 硬件绑定 (每个电机可不同) ---- */
    void            *gpio_port;      /* AIN1/AIN2 所在 GPIO 端口     */
    uint32_t         ain1_pin;       /* AIN1 引脚掩码               */
    uint32_t         ain2_pin;       /* AIN2 引脚掩码               */
} MotorControl;

/*---------------------------------------------------------------------------
 * 初始化
 *---------------------------------------------------------------------------
 * 传入电机所绑定的 GPIO 硬件参数。
 * STBY 不在此函数中处理（如需要，在上层单独置高）。
 *---------------------------------------------------------------------------*/
void Motor_Init(MotorControl *motor,
                void *gpio_port,  uint32_t ain1_pin, uint32_t ain2_pin);

void Motor_SetDirection(MotorControl *motor, uint8_t direction);
void Motor_SetDutyCycle(MotorControl *motor, float duty);
void Motor_SetTargetSpeed(MotorControl *motor, float speed);
void Motor_Update(MotorControl *motor, float measured_speed);
void Motor_Stop(MotorControl *motor);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H_ */
