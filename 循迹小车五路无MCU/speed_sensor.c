/**
 * @file    speed_sensor.c
 * @brief   双电机霍尔编码器速度测量
 *
 *          MOTOR_1: EAA/PA21, EAB/PA22 → GPIOA → GROUP1
 *          MOTOR_2: EBA/PB0,  EBB/PB1  → GPIOB → GROUP1
 *          GROUP1_IRQHandler 同时处理两个 GPIO 端口
 */

#include "speed_sensor.h"
#include "ti_msp_dl_config.h"
#include "motor_control.h"

/*===========================================================================
 * 电机 A ISR 状态
 *===========================================================================*/
static volatile uint32_t a_pulse_count;
static volatile uint32_t a_total;
static volatile uint8_t  a_dir;

/*===========================================================================
 * 电机 B ISR 状态
 *===========================================================================*/
static volatile uint32_t b_pulse_count;
static volatile uint32_t b_total;
static volatile uint8_t  b_dir;

/*===========================================================================
 * 主循环状态
 *===========================================================================*/
static float    a_speed, a_rpm, b_speed, b_rpm;
static uint8_t  a_cur_dir, b_cur_dir;
static uint32_t a_cycles, b_cycles;

/*===========================================================================
 * 内部辅助
 *===========================================================================*/
static void chan_init(volatile uint32_t *pc, volatile uint32_t *pt,
                       volatile uint8_t *pd,
                       float *sp, float *rp, uint8_t *cd, uint32_t *cy)
{
    *pc = 0; *pt = 0; *pd = ENCODER_DIR_STOP;
    *sp = 0.0f; *rp = 0.0f; *cd = ENCODER_DIR_STOP; *cy = 0;
}

static bool chan_update(volatile uint32_t *pc, volatile uint8_t *pd,
                         float *sp, float *rp, uint8_t *cd, uint32_t *cy)
{
    (*cy)++;
    if (*cy < SPEED_WINDOW_CYCLES) return false;
    *cy = 0;

    uint32_t pulses;
    uint8_t  dir;
    __disable_irq();
    pulses = *pc;
    dir    = *pd;
    *pc    = 0;
    __enable_irq();

    float window_sec     = (float)SPEED_WINDOW_MS / 1000.0f;
    float pulses_per_sec = (float)pulses / window_sec;

    *rp = pulses_per_sec * 60.0f / (float)ENCODER_PPR;

    /* 滤除噪声: 超过 15000 RPM 视为干扰, 保持上一次有效值 */
    if (*rp > 15000.0f) {
        *rp = *sp * MOTOR_MAX_RPM;   /* 沿用上一次速度                  */
        return true;
    }

    *sp = *rp / MOTOR_MAX_RPM;
    if (*sp > 1.0f) *sp = 1.0f;
    if (*sp < 0.0f) *sp = 0.0f;

    *cd = dir;
    return true;
}

/*===========================================================================
 * 电机 A 公开接口
 *===========================================================================*/
void SpeedSensorA_Init(void) {
    chan_init(&a_pulse_count, &a_total, &a_dir, &a_speed, &a_rpm, &a_cur_dir, &a_cycles);
}
bool SpeedSensorA_Update(void) {
    return chan_update(&a_pulse_count, &a_dir, &a_speed, &a_rpm, &a_cur_dir, &a_cycles);
}
float    SpeedSensorA_GetSpeed(void)    { return a_speed; }
float    SpeedSensorA_GetRPM(void)      { return a_rpm; }
uint8_t  SpeedSensorA_GetDirection(void){ return a_cur_dir; }
uint32_t SpeedSensorA_GetTotalPulses(void) {
    uint32_t v; __disable_irq(); v = a_total; __enable_irq(); return v;
}

/*===========================================================================
 * 电机 B 公开接口
 *===========================================================================*/
void SpeedSensorB_Init(void) {
    chan_init(&b_pulse_count, &b_total, &b_dir, &b_speed, &b_rpm, &b_cur_dir, &b_cycles);
}
bool SpeedSensorB_Update(void) {
    return chan_update(&b_pulse_count, &b_dir, &b_speed, &b_rpm, &b_cur_dir, &b_cycles);
}
float    SpeedSensorB_GetSpeed(void)    { return b_speed; }
float    SpeedSensorB_GetRPM(void)      { return b_rpm; }
uint8_t  SpeedSensorB_GetDirection(void){ return b_cur_dir; }
uint32_t SpeedSensorB_GetTotalPulses(void) {
    uint32_t v; __disable_irq(); v = b_total; __enable_irq(); return v;
}

/*===========================================================================
 * ISR 回调
 *===========================================================================*/
void SpeedSensor_EAA_ISR(void)
{
    a_pulse_count++;
    a_total++;
    if (DL_GPIO_readPins(DC_MOTOR_1_PORT, DC_MOTOR_1_EAB_PIN) != 0)
        a_dir = MOTOR_DIR_FORWARD;
    else
        a_dir = MOTOR_DIR_REVERSE;
}

void SpeedSensor_EAB_ISR(void) { /* 保留 */ }

void SpeedSensor_EBA_ISR(void)
{
    b_pulse_count++;
    b_total++;
    if (DL_GPIO_readPins(DC_MOTOR_2_PORT, DC_MOTOR_2_EBB_PIN) != 0)
        b_dir = MOTOR_DIR_FORWARD;
    else
        b_dir = MOTOR_DIR_REVERSE;
}

void SpeedSensor_EBB_ISR(void) { /* 保留 */ }

/*===========================================================================
 * GROUP1_IRQHandler — 覆盖弱定义, 处理 GPIOA + GPIOB
 *===========================================================================*/
void GROUP1_IRQHandler(void)
{
    uint32_t groupIID = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

    switch (groupIID) {

    case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
    {
        uint32_t pend = DL_GPIO_getEnabledInterruptStatus(
            DC_MOTOR_1_PORT, DC_MOTOR_1_EAA_PIN | DC_MOTOR_1_EAB_PIN);
        if (pend & DC_MOTOR_1_EAA_PIN) {
            DL_GPIO_clearInterruptStatus(DC_MOTOR_1_PORT, DC_MOTOR_1_EAA_PIN);
            SpeedSensor_EAA_ISR();
        }
        if (pend & DC_MOTOR_1_EAB_PIN) {
            DL_GPIO_clearInterruptStatus(DC_MOTOR_1_PORT, DC_MOTOR_1_EAB_PIN);
            SpeedSensor_EAB_ISR();
        }
        break;
    }

    case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
    {
        uint32_t pend = DL_GPIO_getEnabledInterruptStatus(
            DC_MOTOR_2_PORT, DC_MOTOR_2_EBA_PIN | DC_MOTOR_2_EBB_PIN);
        if (pend & DC_MOTOR_2_EBA_PIN) {
            DL_GPIO_clearInterruptStatus(DC_MOTOR_2_PORT, DC_MOTOR_2_EBA_PIN);
            SpeedSensor_EBA_ISR();
        }
        if (pend & DC_MOTOR_2_EBB_PIN) {
            DL_GPIO_clearInterruptStatus(DC_MOTOR_2_PORT, DC_MOTOR_2_EBB_PIN);
            SpeedSensor_EBB_ISR();
        }
        break;
    }

    default:
        break;
    }
}
