/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * 按钮控制电机走圈测试程序
 *
 *   上电: 电机停止, OLED 显示状态
 *   按按钮: 每按一次圈数+1 (0=无限, 1~5), OLED 实时显示
 *   2秒不按: 锁定圈数, 电机启动 + 循迹, 编码器计距
 *   走完: 短刹车停稳, OLED 闪烁 3 次, 回到待机
 *
 *   距离: 1 圈 ≈ 4 米 (实际按 PULSES_PER_LAP 标定)
 *   编码器: 13 PPR, 1:20 减速比, 48mm 轮径
 *   显示: SSD1306 OLED (I2C 0x3C)
 *   按钮: PA15, 内部上拉, 按下=GND, 50ms 消抖
 */

#include <stdint.h>
#include <math.h>
#include "ti_msp_dl_config.h"
#include "delay.h"
#include "uart.h"
#include "motor_control.h"
#include "speed_sensor.h"
#include "grayscale.h"
#include "pid_controller.h"
#include "oled.h"

/*===========================================================================
 * 硬件参数
 *===========================================================================*/

/* 轮子 + 编码器 */
#define WHEEL_DIAM_MM       48.0f
#define WHEEL_CIRCUM_MM     150.8f              /* π × 48mm               */
#define LAP_LENGTH_MM       4000.0f             /* 1 圈 = 4 米             */
#define PULSES_PER_REV      260U                /* 13 PPR × 20:1 减速比    */
#define PULSES_PER_LAP      5517U               /* 6034 - 517(30cm) = 5517 */
#define STOP_OFFSET_PULSES  108U                /* 短刹车固定补偿 ~6cm     */

/* 电机 */
#define RUN_DUTY            0.30f               /* 巡航占空比              */

/* 按钮 (PA15, INPUT + 内部上拉) */
#define BTN_PORT            GPIOA
#define BTN_PIN             DL_GPIO_PIN_15
#define BTN_DEBOUNCE_CYC    25U                 /* 50ms / 2ms = 25         */
#define BTN_TIMEOUT_CYC     1000U               /* 2000ms / 2ms = 1000     */
#define BTN_LONG_PRESS_CYC  750U                /* 1.5s / 2ms = 750        */

#define MAX_LAPS            5

/* OLED: SDA=PA10, SCL=PA11, I2C1, 128x64 */

/*===========================================================================
 * 循迹参数
 *===========================================================================*/
#define LINE_POLARITY        1.0f               /* 转向极性                    */
#define LOST_OUTER_DUTY      0.30f              /* 丢线时外轮占空比             */
#define LOST_MAX_TICKS       1000U              /* 丢线超时 2s (2ms×1000)      */

/*===========================================================================
 * 速度 PID 参数 (慢速层 — 编码器调速)
 *===========================================================================*/
#define BASE_DUTY_NOMINAL    0.35f              /* 巡航占空比初始值            */
#define BASE_DUTY_MIN        0.10f              /* PID 输出下限                */
#define BASE_DUTY_MAX        0.65f              /* PID 输出上限                */
#define SPEED_WINDOW_TICKS   25U                /* 调速周期 = 25×2ms = 50ms    */
#define SPEED_TARGET_PPS     50U                /* 目标: 每窗口 50 脉冲        */

static float g_base_duty = BASE_DUTY_NOMINAL;   /* 速度 PID 动态调整的基速     */

/* 循迹模式 */
typedef enum {
    MODE_NORMAL,
    MODE_LOST
} FollowMode;

/* 状态机 */
typedef enum {
    STATE_IDLE,                                 /* 待机, 等按钮            */
    STATE_COUNTING,                             /* 计数按钮次数            */
    STATE_RUNNING,                              /* 电机运行, 编码器计距    */
    STATE_DONE                                  /* 走完, 闪烁→待机          */
} State;

/*---------------------------------------------------------------------------
 * SysTick PWM (1kHz = 100kHz / 100step)
 *---------------------------------------------------------------------------*/
#define PWM_PERIOD          100U
#define SYSTICK_FREQ_HZ     100000U
static volatile uint8_t g_pwm_a = 0;
static volatile uint8_t g_pwm_b = 0;

/*===========================================================================
 * OLED 显示更新
 *===========================================================================*/
static void DrawStatus(State state, uint8_t laps,
                       uint32_t progress, uint32_t target)
{
    char buf[21];
    uint8_t i;

    /* Page 2: lap count */
    for (i = 0; i < 21; i++) buf[i] = ' ';
    buf[20] = '\0';
    OLED_ShowString(2, 0, buf);

    OLED_ShowString(2, 0, "LAPS: ");
    if (laps == 0) {
        OLED_ShowString(2, 36, "INF");
    } else {
        OLED_ShowNum(2, 36, laps);
    }

    /* Page 4: state */
    OLED_ShowString(4, 0, "                    ");
    switch (state) {
        case STATE_IDLE:     OLED_ShowString(4, 0, "[IDLE]");      break;
        case STATE_COUNTING: OLED_ShowString(4, 0, "[COUNTING]");  break;
        case STATE_RUNNING:  OLED_ShowString(4, 0, "[RUNNING]");   break;
        case STATE_DONE:     OLED_ShowString(4, 0, "[DONE]");      break;
    }

    /* Page 5: progress (RUNNING) or hint */
    OLED_ShowString(5, 0, "                    ");
    if (state == STATE_RUNNING) {
        if (target == 0) {
            OLED_ShowString(5, 0, "INF mode running...");
        } else {
            OLED_ShowNum(5, 0, (int32_t)progress);
            OLED_ShowString(5, 54, "/");
            OLED_ShowNum(5, 66, (int32_t)target);
        }
    } else if (state == STATE_IDLE) {
        OLED_ShowString(5, 0, "Press BTN: +1 lap");
    } else if (state == STATE_COUNTING) {
        OLED_ShowString(5, 0, "Wait 2s to start...");
    } else {
        OLED_ShowString(5, 0, "Complete!");
    }
}

/*===========================================================================
 * 主程序
 *===========================================================================*/
int main(void)
{
    MotorControl motor1, motor2;
    uint32_t tick = 0;

    /* ---- 状态 ---- */
    State    state           = STATE_IDLE;
    uint8_t  lap_count       = 0;
    uint32_t target_pulses   = 0;
    uint32_t start_pulses    = 0;
    uint32_t current_pulses  = 0;

    /* ---- 按钮消抖 ---- */
    uint8_t  btn_stable      = 1;    /* 消抖后稳定值 (1=未按) */
    uint8_t  btn_prev_raw    = 1;    /* 上一次原始读数         */
    uint8_t  btn_prev_stable = 1;    /* 上一次稳定值 (边沿检测)*/
    uint32_t btn_cnt         = 0;    /* 消抖计数器             */
    uint32_t idle_ticks      = 0;    /* 无操作计时 (超时用)    */
    uint8_t  btn_press       = 0;    /* 消抖后的按下边沿       */

    /* ---- 循迹 ---- */
    FollowMode follow_mode     = MODE_NORMAL;
    GrayscaleSensor gs;
    float    line_error        = 0.0f;
    float    last_error        = 0.0f;
    uint32_t lost_timer        = 0;

    /* ---- 速度 PID (慢速层) ---- */
    PIDController speed_pid;
    uint32_t last_speed_pulses  = 0;
    uint32_t speed_window_cnt   = 0;

    /* ---- DONE 闪烁 ---- */
    uint8_t  flash_count     = 0;
    uint32_t flash_timer     = 0;

    /* ---- 硬件初始化 ---- */
    SYSCFG_DL_init();
    SpeedSensorA_Init();
    SpeedSensorB_Init();
    UART_SendString("Motor Lap Test ready.\r\n");

    /* STBY 使能 */
    DL_GPIO_setPins(DC_MOTOR_1_PORT, DC_MOTOR_1_STBY_PIN);

    /* 提高 AIN1/AIN2 驱动强度 */
    IOMUX->SECCFG.PINCM[DC_MOTOR_1_AIN1_IOMUX] |= IOMUX_PINCM_DRV_DRVVAL1;
    IOMUX->SECCFG.PINCM[DC_MOTOR_1_AIN2_IOMUX] |= IOMUX_PINCM_DRV_DRVVAL1;

    /* 编码器上拉 */
    IOMUX->SECCFG.PINCM[DC_MOTOR_1_EAA_IOMUX] =
        (IOMUX->SECCFG.PINCM[DC_MOTOR_1_EAA_IOMUX] & ~0x00030000U) | 0x00010000U;
    IOMUX->SECCFG.PINCM[DC_MOTOR_1_EAB_IOMUX] =
        (IOMUX->SECCFG.PINCM[DC_MOTOR_1_EAB_IOMUX] & ~0x00030000U) | 0x00010000U;
    IOMUX->SECCFG.PINCM[DC_MOTOR_2_EBA_IOMUX] =
        (IOMUX->SECCFG.PINCM[DC_MOTOR_2_EBA_IOMUX] & ~0x00030000U) | 0x00010000U;
    IOMUX->SECCFG.PINCM[DC_MOTOR_2_EBB_IOMUX] =
        (IOMUX->SECCFG.PINCM[DC_MOTOR_2_EBB_IOMUX] & ~0x00030000U) | 0x00010000U;

    NVIC_EnableIRQ((IRQn_Type)DC_MOTOR_1_INT_IRQN);
    NVIC_EnableIRQ((IRQn_Type)DC_MOTOR_2_INT_IRQN);

    /* 按钮 PA15: SysConfig 已配置, 双重确认上拉 */
    IOMUX->SECCFG.PINCM[15] = (IOMUX->SECCFG.PINCM[15] & ~0x00030000U) | 0x00010000U;

    /* OLED 初始化 (I2C1, SDA=PA10, SCL=PA11, 已在 SysConfig 配置) */
    OLED_Init();
    OLED_ShowString(0, 18, "=== LAP TEST ===");
    OLED_ShowString(7, 6,  "Press BTN to start");

    /* PWM 引脚: 切回 GPIO 输出 (SysTick 软件 PWM 替代 TIMG CCP) */
    DL_GPIO_initDigitalOutput(GPIO_PWMA_C0_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_PWMB_C0_IOMUX);
    DL_GPIO_clearPins(GPIO_PWMA_C0_PORT, GPIO_PWMA_C0_PIN);
    DL_GPIO_clearPins(GPIO_PWMB_C0_PORT, GPIO_PWMB_C0_PIN);

    /* SysTick 100kHz → PWM = 1kHz */
    SysTick->LOAD = (CPUCLK_FREQ / SYSTICK_FREQ_HZ) - 1U;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
    NVIC_SetPriority(SysTick_IRQn, 0xFF);

    /* 电机初始化 (仅方向控制, 不走 PID) */
    Motor_Init(&motor1, DC_MOTOR_1_PORT, DC_MOTOR_1_AIN1_PIN, DC_MOTOR_1_AIN2_PIN);
    Motor_SetDirection(&motor1, MOTOR_DIR_REVERSE);
    Motor_Init(&motor2, DC_MOTOR_2_PORT, DC_MOTOR_2_BIN1_PIN, DC_MOTOR_2_BIN2_PIN);
    Motor_SetDirection(&motor2, MOTOR_DIR_REVERSE);

    /* 灰度传感器初始化 (8路 I2C, 地址 0x12, 与 OLED 共用 I2C1) */
    Grayscale_Init();

    /* 速度 PID 初始化: Kp=0.005, Ki=0.002, Kd=0, dt=50ms, 输出限幅 [0.10, 0.65] */
    PID_Init(&speed_pid, 0.005f, 0.002f, 0.0f, 0.05f, BASE_DUTY_MIN, BASE_DUTY_MAX);
    g_base_duty        = BASE_DUTY_NOMINAL;
    last_speed_pulses  = SpeedSensorA_GetTotalPulses();
    speed_window_cnt   = 0;

    /* 初始显示 */
    DrawStatus(STATE_IDLE, 0, 0, 0);

    /* ---- 主循环 (2ms / 500Hz) ---- */
    while (1)
    {
        delay_ms(2);
        tick++;

        /* ================================================================
         * 按钮消抖
         * ================================================================ */
        uint8_t btn_raw = (DL_GPIO_readPins(BTN_PORT, BTN_PIN) != 0) ? 1 : 0;

        if (btn_raw == btn_prev_raw) {
            if (btn_cnt < BTN_DEBOUNCE_CYC) btn_cnt++;
            if (btn_cnt >= BTN_DEBOUNCE_CYC) {
                btn_stable = btn_raw;
                btn_cnt    = BTN_DEBOUNCE_CYC;  /* 饱和 */
            }
        } else {
            btn_cnt = 0;
        }
        btn_prev_raw = btn_raw;

        /* 下降沿检测 (1→0 = 按下) */
        btn_press = (btn_stable == 0 && btn_prev_stable == 1);
        btn_prev_stable = btn_stable;

        /* ================================================================
         * 状态机
         * ================================================================ */
        switch (state) {

        /* ---------------------------------------------------------------
         * STATE_IDLE — 待机
         * --------------------------------------------------------------- */
        case STATE_IDLE:
            g_pwm_a = 0;
            g_pwm_b = 0;
            follow_mode = MODE_NORMAL;

            /* 测试模式: 持续读取八路灰度传感器并通过串口输出 */
            if ((tick % 250U) == 0U) {   /* 每 500ms 打印一次 */
                GrayscaleSensor gs_test;
                Grayscale_Read(&gs_test);
                UART_Printf("[TEST] GS:%d%d%d%d%d%d%d%d | raw:0x%02X | online:%d\r\n",
                            gs_test.x1, gs_test.x2, gs_test.x3, gs_test.x4,
                            gs_test.x5, gs_test.x6, gs_test.x7, gs_test.x8,
                            (unsigned)(gs_test.x1 | (gs_test.x2<<1) | (gs_test.x3<<2)
                                     | (gs_test.x4<<3) | (gs_test.x5<<4) | (gs_test.x6<<5)
                                     | (gs_test.x7<<6) | (gs_test.x8<<7)),
                            Grayscale_OnLineCount(&gs_test));
            }

            if (btn_press) {
                lap_count   = 0;   /* 0 = 无限模式 */
                idle_ticks  = 0;
                state       = STATE_COUNTING;
                DrawStatus(STATE_COUNTING, lap_count, 0, 0);
            }
            break;

        /* ---------------------------------------------------------------
         * STATE_COUNTING — 累积按钮次数
         * --------------------------------------------------------------- */
        case STATE_COUNTING:
            idle_ticks++;

            if (btn_press) {
                lap_count++;
                if (lap_count > MAX_LAPS) lap_count = 0;
                idle_ticks = 0;
                DrawStatus(STATE_COUNTING, lap_count, 0, 0);
            }

            /* 2 秒超时 → 锁定, 开始运行 */
            if (idle_ticks >= BTN_TIMEOUT_CYC) {
                /* 计算目标脉冲数 (0 = 无限模式, 不计算距离) */
                target_pulses  = (lap_count == 0) ? 0
                               : (uint32_t)lap_count * PULSES_PER_LAP
                                 + STOP_OFFSET_PULSES;
                start_pulses   = SpeedSensorA_GetTotalPulses();
                current_pulses = 0;
                state          = STATE_RUNNING;
                follow_mode    = MODE_NORMAL;
                /* 重置速度 PID 和测速基准 */
                PID_Reset(&speed_pid);
                g_base_duty       = BASE_DUTY_NOMINAL;
                last_speed_pulses = SpeedSensorA_GetTotalPulses();
                speed_window_cnt  = 0;
                DrawStatus(STATE_RUNNING, lap_count, 0, target_pulses);

                /* 恢复电机方向 (上一轮结束可能处于短刹车状态) */
                Motor_SetDirection(&motor1, MOTOR_DIR_REVERSE);
                Motor_SetDirection(&motor2, MOTOR_DIR_REVERSE);

                /* 启动电机 (直接占空比, 不用 PID) */
                uint8_t duty = (uint8_t)(RUN_DUTY * (float)PWM_PERIOD);
                g_pwm_a = duty;
                g_pwm_b = duty;

                if (lap_count == 0) {
                    UART_Printf("START: INFINITE mode\r\n");
                } else {
                    UART_Printf("START: %d lap(s), target=%lu pulses\r\n",
                                lap_count, target_pulses);
                }
            }
            break;

        /* ---------------------------------------------------------------
         * STATE_RUNNING — 电机运行, 编码器计距
         * --------------------------------------------------------------- */
        case STATE_RUNNING:
        {
            /* ---- 循迹: 读取灰度传感器 ---- */
            Grayscale_Read(&gs);
            line_error = Grayscale_ComputeError(&gs) * LINE_POLARITY;

            bool line_lost = Grayscale_IsLineLost(&gs);
            bool all_black = (Grayscale_OnLineCount(&gs) == 5);

            float left_duty  = 0.0f;
            float right_duty = 0.0f;

            /* ---- 循迹模式 ---- */
            switch (follow_mode) {

            case MODE_NORMAL:
                /* 在线时持续更新 last_error */
                if (!line_lost) {
                    last_error = line_error;
                }

                if (line_lost || all_black) {
                    /* 丢线: 进入 LOST, 用 last_error 保持方向 */
                    follow_mode = MODE_LOST;
                    lost_timer  = 0;
                } else {
                    /* 差速分级 */
                    float abs_err = fabsf(line_error);
                    float inner_ratio;
                    if      (abs_err < 0.05f) inner_ratio = 1.0f;
                    else if (abs_err < 0.20f) inner_ratio = 0.5f;
                    else if (abs_err < 0.50f) inner_ratio = 0.2f;
                    else                       inner_ratio = 0.0f;

                    if (line_error > 0.0f) {
                        left_duty  = g_base_duty;
                        right_duty = g_base_duty * inner_ratio;
                    } else {
                        left_duty  = g_base_duty * inner_ratio;
                        right_duty = g_base_duty;
                    }
                }
                break;

            case MODE_LOST:
                lost_timer++;

                if (!line_lost && !all_black) {
                    /* 找回线, 回到正常循迹 */
                    follow_mode = MODE_NORMAL;
                } else if (lost_timer > LOST_MAX_TICKS) {
                    /* 超时 2s: 强制直行 */
                    left_duty  = g_base_duty;
                    right_duty = g_base_duty;
                } else {
                    /* 保持丢线前方向, 外轮 LOST_OUTER_DUTY, 内轮停止 */
                    if (last_error > 0.0f) {
                        left_duty  = LOST_OUTER_DUTY;
                        right_duty = 0.0f;
                    } else {
                        left_duty  = 0.0f;
                        right_duty = LOST_OUTER_DUTY;
                    }
                }
                break;
            }

            /* ---- 慢速层: 只在直线巡航时跑速度 PID ---- */
            if (follow_mode == MODE_NORMAL && fabsf(line_error) < 0.2f) {
                speed_window_cnt++;
                if (speed_window_cnt >= SPEED_WINDOW_TICKS) {
                    speed_window_cnt = 0;
                    uint32_t now_pulses = SpeedSensorA_GetTotalPulses();
                    uint32_t delta = now_pulses - last_speed_pulses;
                    last_speed_pulses = now_pulses;

                    float measured = (float)delta;
                    float setpoint = (float)SPEED_TARGET_PPS;
                    g_base_duty = PID_Update(&speed_pid, setpoint, measured);
                }
            } else {
                speed_window_cnt  = 0;
                last_speed_pulses = SpeedSensorA_GetTotalPulses();
            }

            /* ---- 距离计数 ---- */
            current_pulses = SpeedSensorA_GetTotalPulses() - start_pulses;

            /* ---- 输出 PWM ---- */
            g_pwm_a = (uint8_t)(left_duty  * (float)PWM_PERIOD);
            g_pwm_b = (uint8_t)(right_duty * (float)PWM_PERIOD);

            /* 无限模式: 跳过距离检测, 一直循迹 */
            if (lap_count == 0) {
                if ((tick % 500U) == 0U) {
                    UART_Printf("GS:%d%d%d%d%d%d%d%d err=%+1.2f | "
                                "out:L=%3u R=%3u B=%3u | "
                                "pulses:%lu INF | %s\r\n",
                                gs.x1, gs.x2, gs.x3, gs.x4,
                                gs.x5, gs.x6, gs.x7, gs.x8,
                                line_error,
                                g_pwm_a, g_pwm_b,
                                (uint8_t)(g_base_duty * 100.0f),
                                current_pulses,
                                (follow_mode == MODE_NORMAL) ? "NORM" : "LOST");
                    DrawStatus(STATE_RUNNING, 0, current_pulses, 0);
                }
            } else {
                if ((tick % 500U) == 0U) {
                    UART_Printf("GS:%d%d%d%d%d%d%d%d err=%+1.2f | "
                                "out:L=%3u R=%3u B=%3u | "
                                "pulses:%lu/%lu | %s\r\n",
                                gs.x1, gs.x2, gs.x3, gs.x4,
                                gs.x5, gs.x6, gs.x7, gs.x8,
                                line_error,
                                g_pwm_a, g_pwm_b,
                                (uint8_t)(g_base_duty * 100.0f),
                                current_pulses, target_pulses,
                                (follow_mode == MODE_NORMAL) ? "NORM" : "LOST");
                }

                if (current_pulses >= target_pulses) {
                    /* 主动短刹车: AIN1=HIGH, AIN2=HIGH (TB6612 短刹车) */
                    DL_GPIO_setPins(DC_MOTOR_1_PORT,
                                    DC_MOTOR_1_AIN1_PIN | DC_MOTOR_1_AIN2_PIN);
                    DL_GPIO_setPins(DC_MOTOR_2_PORT,
                                    DC_MOTOR_2_BIN1_PIN | DC_MOTOR_2_BIN2_PIN);
                    g_pwm_a     = 0;
                    g_pwm_b     = 0;
                    state       = STATE_DONE;
                    flash_count = 0;
                    flash_timer = 0;
                    DrawStatus(STATE_DONE, lap_count, current_pulses, target_pulses);
                    UART_Printf("DONE: %lu pulses (overshoot=%ld)\r\n",
                                current_pulses,
                                (int32_t)current_pulses - (int32_t)target_pulses);
                } else {
                    if ((tick % 500U) == 0U) {
                        DrawStatus(STATE_RUNNING, lap_count,
                                   current_pulses, target_pulses);
                    }
                }
            }
        }
        break;

        /* ---------------------------------------------------------------
         * STATE_DONE — 完成闪烁
         * --------------------------------------------------------------- */
        case STATE_DONE:
            flash_timer++;

            /* 500ms ON / 300ms OFF 闪烁 */
            if (flash_count < 6) {
                uint32_t phase = flash_timer % 160U;   /* 800ms 周期         */
                if (phase < 100U) {
                    OLED_DisplayOn();
                } else {
                    OLED_DisplayOff();
                }
                if (phase == 0 && flash_timer > 0) {
                    flash_count++;
                }
            } else {
                /* 闪烁结束 → 回到待机 */
                lap_count = 0;
                OLED_DisplayOn();
                state = STATE_IDLE;
                DrawStatus(STATE_IDLE, 0, 0, 0);
            }
            break;
        }

        /* ================================================================
         * 心跳 LED (1.25Hz, 仅 IDLE/COUNTING 时)
         * ================================================================ */
        if (state == STATE_IDLE || state == STATE_COUNTING) {
            if ((tick % 200U) == 0U) {
                DL_GPIO_togglePins(LED_PORT, LED_PIN1_PIN);
            }
        } else {
            DL_GPIO_setPins(LED_PORT, LED_PIN1_PIN);   /* 运行时 LED 常亮    */
        }
    }
}

/*===========================================================================
 * SysTick ISR — 1kHz 软件 PWM (100kHz/100step)
 *===========================================================================*/
void SysTick_Handler(void)
{
    static uint8_t cnt = 0;

    cnt++;
    if (cnt >= PWM_PERIOD) {
        cnt = 0;
    }

    if (cnt < g_pwm_a) {
        DL_GPIO_setPins(GPIO_PWMA_C0_PORT, GPIO_PWMA_C0_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_PWMA_C0_PORT, GPIO_PWMA_C0_PIN);
    }

    if (cnt < g_pwm_b) {
        DL_GPIO_setPins(GPIO_PWMB_C0_PORT, GPIO_PWMB_C0_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_PWMB_C0_PORT, GPIO_PWMB_C0_PIN);
    }
}
