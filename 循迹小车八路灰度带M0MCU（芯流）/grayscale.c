/**
 * @file    grayscale.c
 * @brief   八路灰度循迹传感器模块实现 (I2C通信)
 *
 * @note    I2C 外设: I2C1 (与 OLED 共用 PA10/PA11)
 *          设备地址: 0x40
 *          查询指令 0x0C: 返回校准后的8路数字量
 *          返回帧: 指令回显 + 长度 + 数据 + 累加校验和
 */

#include "grayscale.h"
#include "ti_msp_dl_config.h"
#include "delay.h"

/*===========================================================================
 * I2C 实例 (与 OLED 共用 I2C1)
 *===========================================================================*/

#define GS_I2C_INST   OLED_INST   /* I2C1, SysConfig 已初始化 */

#define GS_CMD_DIGITAL       0x0C
#define GS_DIGITAL_DATA_LEN  1U
#define GS_DIGITAL_FRAME_LEN 4U
#define GS_I2C_TIMEOUT       10000U
#define GS_RESPONSE_DELAY_US 200U

/*===========================================================================
 * I2C 辅助函数
 *===========================================================================*/

static bool GS_WaitForStatus(uint32_t status, bool set)
{
    uint32_t timeout = GS_I2C_TIMEOUT;

    while (timeout > 0U) {
        bool status_set =
            (DL_I2C_getControllerStatus(GS_I2C_INST) & status) != 0U;
        if (status_set == set) {
            return true;
        }
        timeout--;
    }

    DL_I2C_resetControllerTransfer(GS_I2C_INST);
    return false;
}

static GrayscaleReadStatus GS_FinishTransfer(GrayscaleReadStatus error_status)
{
    if (!GS_WaitForStatus(DL_I2C_CONTROLLER_STATUS_BUSY, false) ||
        !GS_WaitForStatus(DL_I2C_CONTROLLER_STATUS_IDLE, true)) {
        return error_status;
    }

    if ((DL_I2C_getControllerStatus(GS_I2C_INST) &
         DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        DL_I2C_resetControllerTransfer(GS_I2C_INST);
        return error_status;
    }

    return GS_READ_OK;
}

static GrayscaleReadStatus GS_SendCommand(uint8_t command)
{
    if (!GS_WaitForStatus(DL_I2C_CONTROLLER_STATUS_IDLE, true)) {
        return GS_READ_TX_ERROR;
    }

    DL_I2C_startControllerTransfer(GS_I2C_INST, GS_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);

    uint32_t timeout = GS_I2C_TIMEOUT;
    while (DL_I2C_isControllerTXFIFOFull(GS_I2C_INST)) {
        if (timeout-- == 0U) {
            DL_I2C_resetControllerTransfer(GS_I2C_INST);
            return GS_READ_TX_ERROR;
        }
    }
    DL_I2C_transmitControllerData(GS_I2C_INST, command);

    if (GS_FinishTransfer(GS_READ_TX_ERROR) != GS_READ_OK) {
        return GS_READ_TX_ERROR;
    }

    DL_I2C_flushControllerTXFIFO(GS_I2C_INST);
    return GS_READ_OK;
}

static GrayscaleReadStatus GS_ReadFrame(uint8_t *frame, uint8_t length)
{
    if (frame == NULL || length == 0U) {
        return GS_READ_INVALID_ARG;
    }

    if (!GS_WaitForStatus(DL_I2C_CONTROLLER_STATUS_IDLE, true)) {
        return GS_READ_RX_ERROR;
    }

    DL_I2C_startControllerTransfer(GS_I2C_INST, GS_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, length);

    uint8_t index = 0U;
    uint32_t timeout = GS_I2C_TIMEOUT * 50U;
    while (index < length) {
        if (!DL_I2C_isControllerRXFIFOEmpty(GS_I2C_INST)) {
            frame[index++] = DL_I2C_receiveControllerData(GS_I2C_INST);
            timeout = GS_I2C_TIMEOUT * 50U;
        } else if (timeout-- == 0U) {
            DL_I2C_resetControllerTransfer(GS_I2C_INST);
            return GS_READ_RX_ERROR;
        }
    }

    if (GS_FinishTransfer(GS_READ_RX_ERROR) != GS_READ_OK) {
        return GS_READ_RX_ERROR;
    }

    return GS_READ_OK;
}

static GrayscaleReadStatus GS_ReadDigital(uint8_t *raw)
{
    uint8_t frame[GS_DIGITAL_FRAME_LEN];
    GrayscaleReadStatus status;

    if (raw == NULL) {
        return GS_READ_INVALID_ARG;
    }

    status = GS_SendCommand(GS_CMD_DIGITAL);
    if (status != GS_READ_OK) {
        return status;
    }

    delay_cycles((CPUCLK_FREQ / 1000000U) * GS_RESPONSE_DELAY_US);

    status = GS_ReadFrame(frame, GS_DIGITAL_FRAME_LEN);
    if (status != GS_READ_OK) {
        return status;
    }

    if (frame[0] != GS_CMD_DIGITAL) {
        return GS_READ_ECHO_ERROR;
    }
    if (frame[1] != GS_DIGITAL_DATA_LEN) {
        return GS_READ_LENGTH_ERROR;
    }

    uint8_t checksum = (uint8_t)(frame[0] + frame[1] + frame[2]);
    if (frame[3] != checksum) {
        return GS_READ_CHECKSUM_ERROR;
    }

    *raw = frame[2];
    return GS_READ_OK;
}

/*===========================================================================
 * 公开接口
 *===========================================================================*/

void Grayscale_Init(void)
{
    /* I2C1 已由 SYSCFG_DL_init() 初始化，校准通过模块板载按键完成。 */
    delay_ms(50);
}

GrayscaleReadStatus Grayscale_Read(GrayscaleSensor *sensor)
{
    uint8_t raw;
    GrayscaleSensor sample;
    GrayscaleReadStatus status;

    if (sensor == NULL) {
        return GS_READ_INVALID_ARG;
    }

    status = GS_ReadDigital(&raw);
    if (status != GS_READ_OK) {
        return status;
    }

    /*
     * 解析各位: bit0=X8(最右), bit1=X7, ..., bit7=X1(最左)
     * AuroraT8 物理布局 bit0 对应最右侧探头, bit7 对应最左侧。
     * 模块输出: 0=黑线, 1=白 → 取反后: 1=黑线, 0=白
     */
    raw = (uint8_t)~raw;
    sample.x1 = (raw >> 7) & 0x01;  /* bit7 → X1 最左 */
    sample.x2 = (raw >> 6) & 0x01;
    sample.x3 = (raw >> 5) & 0x01;
    sample.x4 = (raw >> 4) & 0x01;
    sample.x5 = (raw >> 3) & 0x01;
    sample.x6 = (raw >> 2) & 0x01;
    sample.x7 = (raw >> 1) & 0x01;
    sample.x8 = (raw >> 0) & 0x01;  /* bit0 → X8 最右 */

    *sensor = sample;
    return GS_READ_OK;
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
    return Grayscale_OnLineCount(sensor) == 0U;
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

uint8_t Grayscale_ActiveMask(const GrayscaleSensor *sensor)
{
    if (sensor == NULL) return 0U;

    return (uint8_t)(sensor->x1 | (sensor->x2 << 1) |
                     (sensor->x3 << 2) | (sensor->x4 << 3) |
                     (sensor->x5 << 4) | (sensor->x6 << 5) |
                     (sensor->x7 << 6) | (sensor->x8 << 7));
}

uint8_t Grayscale_OnLineCount(const GrayscaleSensor *sensor)
{
    if (sensor == NULL) return 0;

    return sensor->x1 + sensor->x2 + sensor->x3 + sensor->x4
         + sensor->x5 + sensor->x6 + sensor->x7 + sensor->x8;
}
