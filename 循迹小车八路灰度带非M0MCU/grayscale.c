/**
 * @file    grayscale.c
 * @brief   八路灰度循迹传感器模块实现 (I2C通信)
 *
 * @note    I2C 外设: I2C1 (与 OLED 共用 PA10/PA11)
 *          设备地址: 0x12
 *          寄存器 0x30 (只读): 8位探头状态
 *          寄存器 0x01 (只写): 校准控制
 */

#include "grayscale.h"
#include "ti_msp_dl_config.h"
#include "delay.h"

/*===========================================================================
 * I2C 实例 (与 OLED 共用 I2C1)
 *===========================================================================*/

#define GS_I2C_INST   OLED_INST   /* I2C1, SysConfig 已初始化 */

/* 寄存器定义 */
#define GS_REG_CALIB   0x01       /* 校准控制: 1=开始, 0=退出 */
#define GS_REG_DATA    0x30       /* 传感器数据: bit0=X1 ... bit7=X8 */

/*===========================================================================
 * I2C 辅助函数
 *===========================================================================*/

/**
 * @brief  向传感器寄存器写入数据
 * @param  reg   寄存器地址
 * @param  data  待写入的数据
 * @return true=成功, false=出错
 */
static bool GS_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;

    uint16_t filled = DL_I2C_fillControllerTXFIFO(GS_I2C_INST, buf, 2);
    DL_I2C_startControllerTransfer(GS_I2C_INST, GS_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, filled);

    /* 等待传输完成 */
    while (!(DL_I2C_getControllerStatus(GS_I2C_INST)
           & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        /* 阻塞等待 */
    }

    /* 检查错误 */
    if (DL_I2C_getControllerStatus(GS_I2C_INST)
        & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_resetControllerTransfer(GS_I2C_INST);
        return false;
    }

    return true;
}

/**
 * @brief  从传感器寄存器读取数据
 * @param  reg   寄存器地址
 * @param  data  输出: 读取到的数据
 * @return true=成功, false=出错
 */
static bool GS_ReadReg(uint8_t reg, uint8_t *data)
{
    /* 第一步: 发送寄存器地址 */
    uint16_t filled = DL_I2C_fillControllerTXFIFO(GS_I2C_INST, &reg, 1);
    DL_I2C_startControllerTransfer(GS_I2C_INST, GS_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, filled);

    while (!(DL_I2C_getControllerStatus(GS_I2C_INST)
           & DL_I2C_CONTROLLER_STATUS_IDLE)) {
    }

    if (DL_I2C_getControllerStatus(GS_I2C_INST)
        & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_resetControllerTransfer(GS_I2C_INST);
        return false;
    }

    /* 第二步: 读取数据 */
    DL_I2C_startControllerTransfer(GS_I2C_INST, GS_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1);

    while (!(DL_I2C_getControllerStatus(GS_I2C_INST)
           & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        /* 阻塞等待 RX 传输完成 */
    }

    if (DL_I2C_getControllerStatus(GS_I2C_INST)
        & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_resetControllerTransfer(GS_I2C_INST);
        return false;
    }

    *data = DL_I2C_receiveControllerData(GS_I2C_INST);
    return true;
}

/*===========================================================================
 * 公开接口
 *===========================================================================*/

void Grayscale_Init(void)
{
    /*
     * 触发一次自动校准:
     *   1. 写寄存器 0x01 = 1 (进入校准模式)
     *   2. 等待模块完成校准 (约 200ms)
     *   3. 写寄存器 0x01 = 0 (退出校准模式)
     *
     * I2C1 已由 SYSCFG_DL_OLED_init() 初始化 (SYSCFG_DL_init 中调用),
     * 此处直接使用即可。
     */

    /* 等模块上电稳定 */
    delay_ms(50);

    /* 进入校准 */
    GS_WriteReg(GS_REG_CALIB, 0x01);

    /* 等待校准完成 (模块自动采样黑白线) */
    delay_ms(300);

    /* 退出校准 */
    GS_WriteReg(GS_REG_CALIB, 0x00);

    /* 再等一小段时间确保退出生效 */
    delay_ms(10);
}

void Grayscale_Read(GrayscaleSensor *sensor)
{
    uint8_t raw = 0;

    if (sensor == NULL) return;

    if (!GS_ReadReg(GS_REG_DATA, &raw)) {
        /* I2C 读失败 — 全部清零, 等效于丢线 */
        raw = 0;
    }

    /*
     * 解析各位: bit0=X1(最左), bit1=X2, ..., bit7=X8(最右)
     * 模块输出: 0=黑线, 1=白 → 取反后: 1=黑线, 0=白
     */
    raw = ~raw;
    sensor->x1 = (raw >> 0) & 0x01;
    sensor->x2 = (raw >> 1) & 0x01;
    sensor->x3 = (raw >> 2) & 0x01;
    sensor->x4 = (raw >> 3) & 0x01;
    sensor->x5 = (raw >> 4) & 0x01;
    sensor->x6 = (raw >> 5) & 0x01;
    sensor->x7 = (raw >> 6) & 0x01;
    sensor->x8 = (raw >> 7) & 0x01;
}

float Grayscale_ComputeError(const GrayscaleSensor *sensor)
{
    if (sensor == NULL) return 0.0f;

    /*
     * 加权误差计算 (8路)
     *
     * 权重分配 (从左到右):
     *   X1=-7, X2=-5, X3=-3, X4=-1, X5=+1, X6=+3, X7=+5, X8=+7
     *
     * 负值 = 线偏左 (X1~X4 检测到线)
     * 正值 = 线偏右 (X5~X8 检测到线)
     *
     * 归一化: 除以 (在线传感器数 × 7), 输出范围 [-1.0, +1.0]
     */

    static const int8_t weight[GS_COUNT] = {
        -7, -5, -3, -1,  1,  3,  5,  7
        /* X1  X2  X3  X4  X5  X6  X7  X8 */
    };

    const uint8_t *vals = &sensor->x1;
    int32_t sum_weighted = 0;
    int32_t sum_active   = 0;

    for (int i = 0; i < GS_COUNT; i++) {
        if (vals[i]) {
            sum_weighted += weight[i];
            sum_active   += 1;
        }
    }

    if (sum_active == 0) {
        return 0.0f;  /* 全部离线 — 保持当前方向 */
    }

    /*
     * 归一化: 最大绝对权重 = 7 (仅最外侧触发)
     * error = sum_weighted / (sum_active * 7)
     */
    float error = (float)sum_weighted / (float)(sum_active * 7);

    /* 限幅 */
    if (error >  1.0f) error =  1.0f;
    if (error < -1.0f) error = -1.0f;

    return error;
}

bool Grayscale_IsLineLost(const GrayscaleSensor *sensor)
{
    if (sensor == NULL) return true;

    return (sensor->x1 == 0 &&
            sensor->x2 == 0 &&
            sensor->x3 == 0 &&
            sensor->x4 == 0 &&
            sensor->x5 == 0 &&
            sensor->x6 == 0 &&
            sensor->x7 == 0 &&
            sensor->x8 == 0);
}

bool Grayscale_IsSharpTurn(const GrayscaleSensor *sensor)
{
    if (sensor == NULL) return false;

    /*
     * 急弯判断: 只有最外侧传感器在线, 其余全部离线
     * 这表示直角弯 — 线突然拐到了最边缘
     */
    uint8_t active = Grayscale_OnLineCount(sensor);

    if (active == 1) {
        /* 只有一处在线 → 检查是否最外侧 */
        if (sensor->x1 || sensor->x8) {
            return true;
        }
    }

    return false;
}

int Grayscale_SharpTurnDir(const GrayscaleSensor *sensor)
{
    if (!Grayscale_IsSharpTurn(sensor)) return 0;

    if (sensor->x1) return -1;  /* 急左转 */
    if (sensor->x8) return  1;  /* 急右转 */

    return 0;
}

uint8_t Grayscale_OnLineCount(const GrayscaleSensor *sensor)
{
    if (sensor == NULL) return 0;

    return sensor->x1 + sensor->x2 + sensor->x3 + sensor->x4
         + sensor->x5 + sensor->x6 + sensor->x7 + sensor->x8;
}
