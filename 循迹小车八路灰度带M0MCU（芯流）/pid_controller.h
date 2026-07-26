#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------
 * PID 控制器结构体
 *---------------------------------------------------------------------------
 * 作用：把目标值和测量值之间的误差变成一个可用于电机驱动的控制量。
 * 这里使用的是位置式 PID，适合你正在学习的“闭环速度控制”场景。
 *---------------------------------------------------------------------------*/
typedef struct {
    float kp;             /* 比例系数 */
    float ki;             /* 积分系数 */
    float kd;             /* 微分系数 */
    float integral;       /* 积分项累积值 */
    float prev_error;     /* 上一次误差 (调试用) */
    float prev_measured;  /* 上一次测量值 (用于微分) */
    float dt;             /* 控制周期，单位秒 */
    float out_min;        /* 输出限幅下限 */
    float out_max;        /* 输出限幅上限 */
} PIDController;

void PID_Init(PIDController *pid, float kp, float ki, float kd,
              float dt, float out_min, float out_max);
float PID_Update(PIDController *pid, float setpoint, float measured_value);
void PID_Reset(PIDController *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H_ */
