#include "stm32f10x.h"
#include "LED.h"
#include "Delay.h"
#include "OLED.h"
#include <math.h>

/* ===================================================================
 *     STM32F103 + MPU9250 + OLED — 单文件版本
 * ===================================================================
 * 不再依赖 MPU9250.c / MPU9250.h，所有 I2C + 传感器代码集成在 main.c
 * =================================================================== */

/* ============================================================
 * 寄存器 & 地址定义
 * ============================================================ */
#define MPU_ADDR_W      0xD0        // MPU9250 写地址
#define MPU_ADDR_R      0xD1        // MPU9250 读地址
#define MAG_ADDR_W      0x18        // AK8963 磁力计写地址
#define MAG_ADDR_R      0x19        // 读地址

#define WHO_AM_I_VAL    0x71        // MPU9250 WHO_AM_I 期望值
#define MAG_WIA_VAL     0x48        // AK8963 WIA 期望值

/* MPU9250 寄存器 */
#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_INT_PIN_CFG  0x37
#define REG_USER_CTRL    0x6A
#define REG_WHO_AM_I     0x75
#define REG_ACCEL_XH     0x3B
#define REG_GYRO_XH      0x43
#define REG_TEMP_XH      0x41

/* AK8963 寄存器 */
#define REG_CNTL1        0x0A
#define REG_WIA          0x00
#define REG_HXL          0x03

/* 滤波参数 */
#define GYRO_DEADZONE     2
#define GYRO_DEADZONE_Y   10   // Z轴死区加大, 挡住残余零偏
#define LPF_A_GYRO        0.80f   // 陀螺仪滤波 (快)
#define LPF_A_ACCEL       0.30f   // 加速度滤波 (响应与平滑平衡)
#define LPF_A_MAG         0.20f   // 磁力计滤波 (稍慢, 去抖)
#define COMP_TRUST        0.98f
#define MAG_CORRECT       0.20f  // 磁力计修正权重 (0.20 = ~0.5s收敛)
#define STILL_THRESH      2000
#define DT                0.05f  // 采样间隔 (50ms主循环)

#define CALIB_N           400   // 校准采样加倍, 零偏更准
#define I2C_TMO           100000

/* ============================================================
 * 数据结构
 * ============================================================ */
typedef struct {
    int16_t Accel_X, Accel_Y, Accel_Z;
    int16_t Temperature;
    int16_t Gyro_X, Gyro_Y, Gyro_Z;
    int16_t Mag_X, Mag_Y, Mag_Z;
} IMU_Data;

typedef struct {
    float Roll, Pitch, Yaw;
} IMU_Angle;

/* ============================================================
 * 滤波器全局状态
 * ============================================================ */
static int16_t  gBiasX=0, gBiasY=0, gBiasZ=0;
static float    gMagOffX=0, gMagOffY=0, gMagOffZ=0;
static float    gGFx=0, gGFy=0, gGFz=0;
static float    gAFx=0, gAFy=0, gAFz=0;
static float    gMFx=0, gMFy=0, gMFz=0;
static float    gRoll=0, gPitch=0, gYaw=0;
static uint8_t  gMagOk = 0;      // 1=磁力计正常工作
static uint8_t  gMagAdr = 0x18;  // 磁力计写地址 (默认 0x0C<<1)

/* ============================================================
 * I2C 底层 (与诊断程序完全相同的实现)
 * ============================================================ */
static void I2C_Init_HW(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,  ENABLE);

    GPIO_InitTypeDef g;
    g.GPIO_Mode  = GPIO_Mode_AF_OD;
    g.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &g);
    GPIOB->BSRR = GPIO_Pin_6 | GPIO_Pin_7;

    I2C_InitTypeDef i;
    i.I2C_ClockSpeed        = 50000;          // 50kHz (内部上拉极限)
    i.I2C_Mode              = I2C_Mode_I2C;
    i.I2C_DutyCycle         = I2C_DutyCycle_2;
    i.I2C_OwnAddress1       = 0x00;
    i.I2C_Ack               = I2C_Ack_Enable;
    i.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i);
    I2C_Cmd(I2C1, ENABLE);
}

static void I2C_Recover(void)
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_Cmd(I2C1, DISABLE);
    Delay_ms(1);
    I2C_Cmd(I2C1, ENABLE);
}

static uint8_t I2C_Write(uint8_t addr, uint8_t reg, uint8_t dat)
{
    uint32_t t;

    /* 1. 发送 START */
    I2C_GenerateSTART(I2C1, ENABLE);
    t = I2C_TMO;
    while (!(I2C1->SR1 & I2C_SR1_SB))
        if (--t == 0) { I2C_Recover(); return 1; }

    /* 2. 发送从机地址(写) */
    I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
    t = I2C_TMO;
    while (!(I2C1->SR1 & I2C_SR1_ADDR))
        if (--t == 0) { I2C_Recover(); return 1; }
    /* 读 SR2 清 ADDR 标志 */
    (void)I2C1->SR2;

    /* 3. 发送寄存器地址 */
    I2C_SendData(I2C1, reg);
    t = I2C_TMO;
    while (!(I2C1->SR1 & I2C_SR1_TXE))
        if (--t == 0) { I2C_Recover(); return 1; }

    /* 4. 发送数据 */
    I2C_SendData(I2C1, dat);
    t = I2C_TMO;
    while (!(I2C1->SR1 & I2C_SR1_BTF))
        if (--t == 0) { I2C_Recover(); return 1; }

    /* 5. STOP */
    I2C_GenerateSTOP(I2C1, ENABLE);
    return 0;
}

static uint8_t I2C_Read(uint8_t addr, uint8_t reg, uint8_t *val)
{
    uint32_t t;

    /* 1. START */
    I2C_GenerateSTART(I2C1, ENABLE);
    t = I2C_TMO;
    while (!(I2C1->SR1 & I2C_SR1_SB))
        if (--t == 0) { I2C_Recover(); return 1; }

    /* 2. 发送从机地址(写) */
    I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
    t = I2C_TMO;
    while (!(I2C1->SR1 & I2C_SR1_ADDR))
        if (--t == 0) { I2C_Recover(); return 1; }
    (void)I2C1->SR2;  // 清 ADDR

    /* 3. 发送寄存器地址 */
    I2C_SendData(I2C1, reg);
    t = I2C_TMO;
    while (!(I2C1->SR1 & (I2C_SR1_TXE | I2C_SR1_BTF)))
        if (--t == 0) { I2C_Recover(); return 1; }

    /* 4. Repeated START */
    I2C_GenerateSTART(I2C1, ENABLE);
    t = I2C_TMO;
    while (!(I2C1->SR1 & I2C_SR1_SB))
        if (--t == 0) { I2C_Recover(); return 1; }

    /* 5. 发送从机地址(读) */
    I2C_Send7bitAddress(I2C1, addr | 0x01, I2C_Direction_Receiver);
    t = I2C_TMO;
    while (!(I2C1->SR1 & I2C_SR1_ADDR))
        if (--t == 0) { I2C_Recover(); return 1; }
    (void)I2C1->SR2;  // 清 ADDR

    /* 6. 单字节读取: 先关 ACK → STOP → 等数据 */
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);  // 提前发 STOP
    t = I2C_TMO;
    while (!(I2C1->SR1 & I2C_SR1_RXNE))
        if (--t == 0) { I2C_Recover(); I2C_AcknowledgeConfig(I2C1, ENABLE); return 1; }
    *val = I2C_ReceiveData(I2C1);

    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 0;
}

static void I2C_ReadMulti(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint32_t t;

    /* 1. START */
    I2C_GenerateSTART(I2C1, ENABLE);
    t = I2C_TMO;
    while (!(I2C1->SR1 & I2C_SR1_SB))
        if (--t == 0) { I2C_Recover(); return; }

    /* 2. 发送从机地址(写) */
    I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
    t = I2C_TMO;
    while (!(I2C1->SR1 & I2C_SR1_ADDR))
        if (--t == 0) { I2C_Recover(); return; }
    (void)I2C1->SR2;

    /* 3. 发送寄存器地址 */
    I2C_SendData(I2C1, reg);
    t = I2C_TMO;
    while (!(I2C1->SR1 & (I2C_SR1_TXE | I2C_SR1_BTF)))
        if (--t == 0) { I2C_Recover(); return; }

    /* 4. Repeated START */
    I2C_GenerateSTART(I2C1, ENABLE);
    t = I2C_TMO;
    while (!(I2C1->SR1 & I2C_SR1_SB))
        if (--t == 0) { I2C_Recover(); return; }

    /* 5. 发送从机地址(读) */
    I2C_Send7bitAddress(I2C1, addr | 0x01, I2C_Direction_Receiver);
    t = I2C_TMO;
    while (!(I2C1->SR1 & I2C_SR1_ADDR))
        if (--t == 0) { I2C_Recover(); return; }
    (void)I2C1->SR2;

    /* 6. 连续读取 */
    while (len) {
        if (len == 1) {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            I2C_GenerateSTOP(I2C1, ENABLE);  // 最后一个字节前发 STOP
        }
        t = I2C_TMO;
        while (!(I2C1->SR1 & I2C_SR1_RXNE))
            if (--t == 0) { I2C_Recover(); return; }
        *buf++ = I2C_ReceiveData(I2C1);
        len--;
    }
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

/* ============================================================
 * 传感器读写
 * ============================================================ */
static uint8_t ReadWHOAMI(void)
{
    uint8_t v = 0;
    I2C_Read(MPU_ADDR_W, REG_WHO_AM_I, &v);
    return v;
}

static uint8_t ReadMagWIA(void)
{
    uint8_t v = 0;
    I2C_Read(MAG_ADDR_W, REG_WIA, &v);
    return v;
}

static void Read6Axis(IMU_Data *d)
{
    uint8_t buf[14];
    I2C_ReadMulti(MPU_ADDR_W, REG_ACCEL_XH, buf, 14);
    d->Accel_X = (int16_t)((buf[0] << 8) | buf[1]);
    d->Accel_Y = (int16_t)((buf[2] << 8) | buf[3]);
    d->Accel_Z = (int16_t)((buf[4] << 8) | buf[5]);
    d->Temperature = (int16_t)((buf[6] << 8) | buf[7]);
    d->Gyro_X = (int16_t)((buf[8]  << 8) | buf[9]);
    d->Gyro_Y = (int16_t)((buf[10] << 8) | buf[11]);
    d->Gyro_Z = (int16_t)((buf[12] << 8) | buf[13]);
}

static uint8_t ReadMag(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t buf[7];
    if (!gMagOk) { *mx=0; *my=0; *mz=0; return 0; }
    I2C_ReadMulti(gMagAdr, REG_HXL, buf, 7);
    *mx = (int16_t)((buf[1] << 8) | buf[0]);
    *my = (int16_t)((buf[3] << 8) | buf[2]);
    *mz = (int16_t)((buf[5] << 8) | buf[4]);
    return 1;
}

/* ============================================================
 * MPU9250 初始化
 * ============================================================ */
static void MPU9250_Init_Simple(void)
{
    Delay_ms(100);

    /* --- MPU9250 6轴部分 (必须成功) --- */
    I2C_Write(MPU_ADDR_W, REG_PWR_MGMT_1,   0x00);  // 唤醒
    Delay_ms(10);
    I2C_Write(MPU_ADDR_W, REG_SMPLRT_DIV,   0x09);  // 100Hz
    I2C_Write(MPU_ADDR_W, REG_CONFIG,       0x03);  // DLPF 44Hz (响应与平滑的平衡)
    I2C_Write(MPU_ADDR_W, REG_GYRO_CONFIG,  0x00);  // ±250°/s
    I2C_Write(MPU_ADDR_W, REG_ACCEL_CONFIG, 0x00);  // ±2g

    /* --- 磁力计初始化 --- */
    /*
     * 启用 bypass 模式: AUX_SDA/SCL 直连主 I2C 总线
     * 注意: I2C_MST_EN 必须先关, BYPASS_EN 才能生效
     */
    I2C_Write(MPU_ADDR_W, REG_USER_CTRL,  0x00);  // 关闭 I2C Master
    Delay_ms(10);
    I2C_Write(MPU_ADDR_W, REG_INT_PIN_CFG, 0x02);  // BYPASS_EN = 1
    Delay_ms(50);  // 给足够时间让 bypass 生效

    /*
     * 尝试 AK8963 的几种可能地址:
     *   0x0C: CAD0=0, CAD1=0 (最常见)
     *   0x0D: CAD0=1 (部分模块)
     *   0x0E: CAD1=1 (少见)
     */
    {
        uint8_t mt=0, ok=0;
        uint8_t mag_addrs[] = {0x18, 0x1A, 0x1C};  // 写地址 (7位地址<<1)
        uint8_t i;
        for (i = 0; i < 3; i++) {
            ok = I2C_Read(mag_addrs[i], REG_WIA, &mt);
            if (ok == 0 && mt == MAG_WIA_VAL) {
                gMagOk = 1;
                gMagAdr = mag_addrs[i];
                /* AK8963 初始化: 掉电→连续模式 100Hz */
                I2C_Write(gMagAdr, REG_CNTL1, 0x00);   // 掉电
                Delay_ms(10);
                I2C_Write(gMagAdr, REG_CNTL1, 0x16);   // 100Hz 连续, 16位
                Delay_ms(10);
                break;
            }
        }
    }
}

/* ============================================================
 * 校准
 * ============================================================ */
static void Calibrate(void)
{
    int32_t sgx=0, sgy=0, sgz=0, smx=0, smy=0, smz=0;
    IMU_Data raw;
    uint16_t i;
    uint8_t  mc=0;

    for (i = 0; i < CALIB_N; i++) {
        Read6Axis(&raw);
        sgx += raw.Gyro_X;
        sgy += raw.Gyro_Y;
        sgz += raw.Gyro_Z;

        if (i % 10 == 0) {
            int16_t mx, my, mz;
            ReadMag(&mx, &my, &mz);
            smx += mx; smy += my; smz += mz;
            mc++;
        }
        Delay_ms(5);
    }

    gBiasX = (int16_t)(sgx / CALIB_N);
    gBiasY = (int16_t)(sgy / CALIB_N);
    gBiasZ = (int16_t)(sgz / CALIB_N);

    /* 硬铁偏移暂不校准 — 单姿态平均不是真正的硬铁校准,
     * 需要 360° 旋转采集 min/max 才能算对。先置零测试。 */
    (void)smx; (void)smy; (void)smz; (void)mc;
    gMagOffX = 0;
    gMagOffY = 0;
    gMagOffZ = 0;

    gGFx=0; gGFy=0; gGFz=0;
    gAFx=(float)raw.Accel_X; gAFy=(float)raw.Accel_Y; gAFz=(float)raw.Accel_Z;
    gMFx=0; gMFy=0; gMFz=0;

    {
        float ax=(float)raw.Accel_X, ay=(float)raw.Accel_Y, az=(float)raw.Accel_Z;
        gRoll  = atan2f(ay, az) * 57.29578f;
        gPitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.29578f;

        int16_t mx, my, mz;
        ReadMag(&mx, &my, &mz);
        float mxf=(float)mx-gMagOffX, myf=(float)my-gMagOffY, mzf=(float)mz-gMagOffZ;
        float r=gRoll*0.0174533f, p=gPitch*0.0174533f;
        float cr=cosf(r), sr=sinf(r), cp=cosf(p), sp=sinf(p);
        float mxc = mxf*cp + myf*sr*sp - mzf*cr*sp;
        float myc = myf*cr + mzf*sr;
        gYaw = atan2f(-myc, mxc) * 57.29578f;
    }

}

/* ============================================================
 * 获取姿态角
 * ============================================================ */
static void GetAngle(IMU_Angle *a)
{
    IMU_Data raw;
    Read6Axis(&raw);

    float gx = (float)(raw.Gyro_X - gBiasX);
    float gy = (float)(raw.Gyro_Y - gBiasY);
    float gz = (float)(raw.Gyro_Z - gBiasZ);

    if (gx < GYRO_DEADZONE   && gx > -GYRO_DEADZONE)   gx = 0;
    if (gy < GYRO_DEADZONE   && gy > -GYRO_DEADZONE)   gy = 0;
    if (gz < GYRO_DEADZONE_Y && gz > -GYRO_DEADZONE_Y) gz = 0;

    gGFx += LPF_A_GYRO  * (gx - gGFx);
    gGFy += LPF_A_GYRO  * (gy - gGFy);
    gGFz += LPF_A_GYRO  * (gz - gGFz);

    gAFx += LPF_A_ACCEL * ((float)raw.Accel_X - gAFx);
    gAFy += LPF_A_ACCEL * ((float)raw.Accel_Y - gAFy);
    gAFz += LPF_A_ACCEL * ((float)raw.Accel_Z - gAFz);

    float r_gyro = gRoll  + (gGFx / 131.0f) * DT;
    float p_gyro = gPitch + (gGFy / 131.0f) * DT;
    float y_gyro = gYaw   + (gGFz / 131.0f) * DT;

    float r_acc = atan2f(gAFy, gAFz) * 57.29578f;
    float p_acc = atan2f(-gAFx, sqrtf(gAFy*gAFy + gAFz*gAFz)) * 57.29578f;

    float tr = (gGFx > 655.0f || gGFx < -655.0f) ? COMP_TRUST : 0.85f;
    float tp = (gGFy > 655.0f || gGFy < -655.0f) ? COMP_TRUST : 0.85f;

    gRoll  = tr * r_gyro + (1.0f - tr) * r_acc;
    gPitch = tp * p_gyro + (1.0f - tp) * p_acc;

    /* 磁力计 Yaw — 只有磁力计正常时才用 */
    if (gMagOk) {
        int16_t mx, my, mz;
        ReadMag(&mx, &my, &mz);
        float mxf=(float)mx-gMagOffX, myf=(float)my-gMagOffY, mzf=(float)mz-gMagOffZ;

        gMFx += LPF_A_MAG * (mxf - gMFx);
        gMFy += LPF_A_MAG * (myf - gMFy);
        gMFz += LPF_A_MAG * (mzf - gMFz);

        float r=gRoll*0.0174533f, p=gPitch*0.0174533f;
        float cr=cosf(r), sr=sinf(r), cp=cosf(p), sp=sinf(p);
        float mxc = gMFx*cp + gMFy*sr*sp - gMFz*cr*sp;
        float myc = gMFy*cr + gMFz*sr;
        float y_mag = atan2f(-myc, mxc) * 57.29578f;

        float dy = y_mag - y_gyro;
        if (dy > 180.0f) dy -= 360.0f;
        if (dy < -180.0f) dy += 360.0f;
        gYaw = y_gyro + MAG_CORRECT * dy;

        if (gYaw > 180.0f)  gYaw -= 360.0f;
        if (gYaw < -180.0f) gYaw += 360.0f;
    } else {
        /* 无磁力计: 纯陀螺仪积分, Yaw 会漂移 (和 MPU6050 一样) */
        gYaw = y_gyro;
        if (gYaw > 180.0f)  gYaw -= 360.0f;
        if (gYaw < -180.0f) gYaw += 360.0f;
    }

    a->Roll  = gRoll;
    a->Pitch = gPitch;
    a->Yaw   = gYaw;

}

/* ============================================================
 * 主函数
 * ============================================================ */
int main(void)
{
    uint8_t  wai;
    IMU_Angle att;
    int ledTick = 0;

    LED_Init();
    Delay_ms(50);

    /* 1. 初始化 I2C + OLED */
    I2C_Init_HW();
    Delay_ms(10);
    OLED_Init();
    OLED_Clear();
    OLED_Refresh();

    OLED_ShowString(0, 0, "  SYSTEM START   ");
    OLED_ShowString(1, 0, "Read WHO_AM_I... ");
    OLED_Refresh();

    /* 2. 先读 WHO_AM_I — 最简操作，诊断程序已验证能通 */
    wai = ReadWHOAMI();

    OLED_ShowString(2, 0, "WHO_AM_I=        ");
    OLED_ShowInt(2, 10, wai);
    OLED_Refresh();
    Delay_ms(800);

    if (wai != WHO_AM_I_VAL) {
        LED_Set(2, 0);
        OLED_ShowString(3, 0, "Expect 0x71 FAIL ");
        OLED_Refresh();
        while (1) {
            LED_Set(1,1); Delay_ms(200);
            LED_Set(1,0); Delay_ms(200);
        }
    }

    OLED_ShowString(1, 0, "Init MPU9250... ");
    OLED_ShowString(2, 0, "                ");
    OLED_ShowString(3, 0, "                ");
    OLED_Refresh();

    /* 3. 初始化 MPU9250 (WHO_AM_I 已经通了，这里只写寄存器) */
    MPU9250_Init_Simple();

    /* 4. 磁力计状态已在 init 中由 gMagOk 确定 */

    LED_Set(2, 1);
    OLED_Clear();
    OLED_ShowString(0, 0, "MPU9250 FOUND!  ");
    if (gMagOk)
        OLED_ShowString(1, 0, "MAGNETOMETER OK ");
    else
        OLED_ShowString(1, 0, "MAG not found   ");

    OLED_ShowString(2, 0, "Calibrating...  ");
    OLED_ShowString(3, 0, "Keep STILL!     ");
    OLED_Refresh();

    /* 4. 校准 */
    Calibrate();

    OLED_Clear();
    OLED_ShowString(0, 0, "Calibration OK! ");
    OLED_ShowString(1, 0, "Starting...     ");
    OLED_Refresh();
    Delay_ms(1000);

    /* 5. 主循环 */
    while (1) {
        GetAngle(&att);

        OLED_ClearLine(0);
        OLED_ShowString(0, 0, gMagOk ? "~~ 9-AXIS ATT ~~" : "~~ 6-AXIS ATT ~~");

        OLED_ClearLine(1);
        OLED_ShowString(1, 0, "Roll :");
        OLED_ShowFloat(1, 7, att.Roll);
        OLED_ShowString(1, 13, "deg");

        OLED_ClearLine(2);
        OLED_ShowString(2, 0, "Pitch:");
        OLED_ShowFloat(2, 7, att.Pitch);
        OLED_ShowString(2, 13, "deg");

        OLED_ClearLine(3);
        OLED_ShowString(3, 0, "Yaw  :");
        OLED_ShowFloat(3, 7, att.Yaw);
        OLED_ShowString(3, 13, "deg");

        OLED_Refresh();

        ledTick++;
        if (ledTick >= 10) ledTick = 0;
        LED_Set(1, ledTick < 5);

        if (att.Roll > 15.0f || att.Roll < -15.0f ||
            att.Pitch > 15.0f || att.Pitch < -15.0f)
            LED_Set(3, 1);
        else
            LED_Set(3, 0);

        Delay_ms(50);
    }
}
