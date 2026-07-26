#include "motor_control.h"
#include "ti_msp_dl_config.h"

/*---------------------------------------------------------------------------
 * 内部辅助 — 设置方向 GPIO
 *---------------------------------------------------------------------------*/
static void Motor_SetGPIO(const MotorControl *motor, uint8_t dir)
{
    switch (dir) {
        case MOTOR_DIR_FORWARD:
            DL_GPIO_setPins(motor->gpio_port, motor->ain1_pin);
            DL_GPIO_clearPins(motor->gpio_port, motor->ain2_pin);
            break;

        case MOTOR_DIR_REVERSE:
            DL_GPIO_clearPins(motor->gpio_port, motor->ain1_pin);
            DL_GPIO_setPins(motor->gpio_port, motor->ain2_pin);
            break;

        default: /* MOTOR_DIR_STOP */
            DL_GPIO_clearPins(motor->gpio_port,
                motor->ain1_pin | motor->ain2_pin);
            break;
    }
}

/*===========================================================================
 * 公共接口
 *===========================================================================*/

void Motor_Init(MotorControl *motor,
                void *gpio_port,  uint32_t ain1_pin, uint32_t ain2_pin)
{
    /* 记录硬件绑定 */
    motor->gpio_port    = gpio_port;
    motor->ain1_pin     = ain1_pin;
    motor->ain2_pin     = ain2_pin;

    /* 初始化 PID 参数，后续可以根据实验结果再调 */
    PID_Init(&motor->pid, 2.5f, 30.0f, 0.03f, 0.005f, 0.0f, 1.0f);

    motor->target_speed   = 0.0f;
    motor->measured_speed = 0.0f;
    motor->duty_cycle     = 0.0f;
    motor->direction      = MOTOR_DIR_STOP;

    /* 初始方向为 STOP */
    Motor_SetGPIO(motor, MOTOR_DIR_STOP);
}

void Motor_SetDirection(MotorControl *motor, uint8_t direction)
{
    motor->direction = direction;
    Motor_SetGPIO(motor, direction);
}

void Motor_SetDutyCycle(MotorControl *motor, float duty)
{
    if (duty < 0.0f) {
        duty = 0.0f;
    } else if (duty > 1.0f) {
        duty = 1.0f;
    }

    motor->duty_cycle = duty;
    /* 实际 PWM 输出由主循环中的 GPIO 软件 PWM 完成 */
}

void Motor_SetTargetSpeed(MotorControl *motor, float speed)
{
    motor->target_speed = speed;
}

void Motor_Update(MotorControl *motor, float measured_speed)
{
    motor->measured_speed = measured_speed;

    /* 使用目标速度和实际测得速度计算控制量 */
    float control = PID_Update(&motor->pid, motor->target_speed, measured_speed);

    /* 把 PID 输出映射成占空比 */
    Motor_SetDutyCycle(motor, control);
}

void Motor_Stop(MotorControl *motor)
{
    motor->target_speed   = 0.0f;
    motor->measured_speed = 0.0f;
    motor->duty_cycle     = 0.0f;
    motor->direction      = MOTOR_DIR_STOP;
    Motor_SetGPIO(motor, MOTOR_DIR_STOP);
}
