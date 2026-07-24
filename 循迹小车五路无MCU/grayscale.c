/**
 * @file    grayscale.c
 * @brief   五路灰度循迹传感器模块实现
 */

#include "grayscale.h"
#include "ti_msp_dl_config.h"

/*===========================================================================
 * 内部状态 — 引脚绑定
 *===========================================================================*/

static struct {
    void     *port;
    uint32_t  pin;
} gs_pin[5];  /* 索引: 0=R2, 1=R1, 2=M, 3=L1, 4=L2 */

/*===========================================================================
 * 内部辅助
 *===========================================================================*/

/** @brief 读取单个传感器 (处理 INVERT) */
static inline uint8_t read_one(uint8_t idx)
{
    uint8_t raw = (DL_GPIO_readPins(gs_pin[idx].port, gs_pin[idx].pin) != 0) ? 1 : 0;
#if GRAYSCALE_INVERT
    return (raw == 0) ? 1 : 0;
#else
    return raw;
#endif
}

/*===========================================================================
 * 公开接口
 *===========================================================================*/

void Grayscale_Init(
    void    *r2_port, uint32_t r2_pin,
    void    *r1_port, uint32_t r1_pin,
    void    *m_port,  uint32_t m_pin,
    void    *l1_port, uint32_t l1_pin,
    void    *l2_port, uint32_t l2_pin)
{
    /* 保存硬件绑定 (索引与传感器位置对应) */
    gs_pin[0].port = r2_port;   gs_pin[0].pin = r2_pin;
    gs_pin[1].port = r1_port;   gs_pin[1].pin = r1_pin;
    gs_pin[2].port = m_port;    gs_pin[2].pin = m_pin;
    gs_pin[3].port = l1_port;   gs_pin[3].pin = l1_pin;
    gs_pin[4].port = l2_port;   gs_pin[4].pin = l2_pin;
}

void Grayscale_Read(GrayscaleSensor *sensor)
{
    if (sensor == NULL) return;

    sensor->r2 = read_one(0);
    sensor->r1 = read_one(1);
    sensor->m  = read_one(2);
    sensor->l1 = read_one(3);
    sensor->l2 = read_one(4);
}

float Grayscale_ComputeError(const GrayscaleSensor *sensor)
{
    if (sensor == NULL) return 0.0f;

    /*
     * 加权误差计算
     *
     * 权重分配:  L2=-4, L1=-2, M=0, R1=+2, R2=+4
     * 负值 = 线偏左, 正值 = 线偏右
     *
     * 当多个传感器同时检测到线时, 取加权平均,
     * 归一化到 [-1.0, +1.0]。
     */

    static const int8_t weight[5] = { 4, 2, 0, -2, -4 }; /* R2,R1,M,L1,L2 */

    int32_t sum_weighted = 0;
    int32_t sum_active   = 0;

    /* 按顺序读取: r2, r1, m, l1, l2 */
    const uint8_t *vals = &sensor->r2;
    for (int i = 0; i < 5; i++) {
        if (vals[i]) {
            sum_weighted += weight[i];
            sum_active   += 1;
        }
    }

    if (sum_active == 0) {
        return 0.0f;  /* 全部离线 — 保持当前方向 */
    }

    /*
     * 归一化: 最大绝对权重和 = 4 (仅最外侧触发)
     * 除以 max_abs_weight 使输出在 [-1.0, +1.0] 内
     */
    float error = (float)sum_weighted / (float)(sum_active * 4);

    /* 限幅 */
    if (error >  1.0f) error =  1.0f;
    if (error < -1.0f) error = -1.0f;

    return error;
}

bool Grayscale_IsLineLost(const GrayscaleSensor *sensor)
{
    if (sensor == NULL) return true;

    return (sensor->r2 == 0 &&
            sensor->r1 == 0 &&
            sensor->m  == 0 &&
            sensor->l1 == 0 &&
            sensor->l2 == 0);
}

bool Grayscale_IsSharpTurn(const GrayscaleSensor *sensor)
{
    if (sensor == NULL) return false;

    /*
     * 急弯判断: 只有最外侧传感器在线, 其他全部离线
     * 这表示直角弯 — 线突然拐到了最边缘
     */
    uint8_t active = Grayscale_OnLineCount(sensor);

    if (active == 1) {
        /* 只有一处在线 → 检查是否最外侧 */
        if (sensor->l2 || sensor->r2) {
            return true;
        }
    }

    return false;
}

int Grayscale_SharpTurnDir(const GrayscaleSensor *sensor)
{
    if (!Grayscale_IsSharpTurn(sensor)) return 0;

    if (sensor->l2) return -1;  /* 急左转 */
    if (sensor->r2) return  1;  /* 急右转 */

    return 0;
}

uint8_t Grayscale_OnLineCount(const GrayscaleSensor *sensor)
{
    if (sensor == NULL) return 0;

    return sensor->r2 + sensor->r1 + sensor->m + sensor->l1 + sensor->l2;
}
