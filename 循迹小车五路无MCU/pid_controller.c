#include "pid_controller.h"

void PID_Init(PIDController *pid, float kp, float ki, float kd,
              float dt, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measured = 0.0f;
}

float PID_Update(PIDController *pid, float setpoint, float measured_value)
{
    float error = setpoint - measured_value;

    /* ---- 比例项 ---- */
    float p_term = pid->kp * error;

    /* ---- 积分项 (带抗饱和) ---- */
    pid->integral += error * pid->dt;
    float i_term = pid->ki * pid->integral;

    /* ---- 微分项 (在测量值上做微分, 避免 setpoint 突变引起冲击) ---- */
    float derivative = -(measured_value - pid->prev_measured) / pid->dt;
    pid->prev_measured = measured_value;
    float d_term = pid->kd * derivative;

    /* ---- 合成输出 ---- */
    float output = p_term + i_term + d_term;

    /* ---- 输出限幅 + 积分抗饱和 (clamping) ---- */
    if (output > pid->out_max) {
        /* 输出饱和在上限, 如果积分在推高输出则回退积分 */
        if (error > 0.0f) {
            pid->integral -= error * pid->dt;
        }
        output = pid->out_max;
    } else if (output < pid->out_min) {
        /* 输出饱和在下限, 如果积分在拉低输出则回退积分 */
        if (error < 0.0f) {
            pid->integral -= error * pid->dt;
        }
        output = pid->out_min;
    }

    /* 保存调试用的误差值 */
    pid->prev_error = error;

    return output;
}

void PID_Reset(PIDController *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measured = 0.0f;
}
