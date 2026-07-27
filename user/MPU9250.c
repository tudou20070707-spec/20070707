X#include "MPU9250.h"
#include "Delay.h"
#include <math.h>

/* ===================================================================
 *                           MPU9250 驱动文件
 * ===================================================================
 * 本文件实现通过 STM32F103 的 I2C1 硬件外设与 MPU9250 传感器通信。
 *
 * MPU9250 是一颗 9 轴 IMU，内部包含:
 *   - MPU6500 6 轴模块 (3轴加速度 + 3轴陀螺仪) → I2C 地址 0x68
 *   - AK8963  3 轴磁力计                        → I2C 地址 0x0C
 *
 * 与 MPU6050 的区别:
 *   1. WHO_AM_I = 0x71 (MPU6050 = 0x68)
 *   2. 内置磁力计，可实现无漂移 Yaw 角
 *   3. 需要通过 BYPASS_EN 使能辅助 I2C 总线直通
 *
 * 【I2C 通信原理简述】
 *   I2C (Inter-Integrated Circuit) 是一种两线式串行总线：
 *     - SCL (Serial Clock)    : 时钟线，由主机控制
 *     - SDA (Serial Data)     : 数据线，双向传输
 *   每个从设备有唯一的 7 位或 10 位地址。
 *
 * 【硬件连接 (默认使用 I2C1)】
 *   PB6 → SCL  (I2C1_SCL)
 *   PB7 → SDA  (I2C1_SDA)
 *   两个引脚必须外接 4.7kΩ 上拉电阻到 3.3V
 * =================================================================== */

/* ============================================================
 * 内部辅助函数声明
 * ============================================================ */
void I2C_MyInit(void);                                               // 初始化 I2C1 硬件
static uint8_t I2C_WriteReg(uint8_t SlaveAddr, uint8_t RegAddr, uint8_t Data);   // 通用写寄存器
static uint8_t I2C_ReadReg(uint8_t SlaveAddr, uint8_t RegAddr, uint8_t *pData);  // 通用读寄存器
static uint8_t I2C_ReadMulti(uint8_t SlaveAddr, uint8_t RegAddr, uint8_t *pBuf, uint8_t len); // 连续读

/* MPU9250 本体的快捷封装 */
static uint8_t MPU9250_WriteReg(uint8_t RegAddr, uint8_t Data);
static uint8_t MPU9250_ReadReg(uint8_t RegAddr, uint8_t *pData);
static uint8_t MPU9250_ReadMulti(uint8_t RegAddr, uint8_t *pBuf, uint8_t len);

/* AK8963 磁力计的快捷封装 */
static uint8_t AK8963_WriteReg(uint8_t RegAddr, uint8_t Data);
static uint8_t AK8963_ReadReg(uint8_t RegAddr, uint8_t *pData);
static uint8_t AK8963_ReadMulti(uint8_t RegAddr, uint8_t *pBuf, uint8_t len);

/* ============================================================
 * I2C1 硬件初始化
 * ------------------------------------------------------------
 * 时钟: 使能 GPIOB 和 I2C1 外设时钟
 * GPIO: PB6 (SCL) 和 PB7 (SDA) 配置为复用开漏输出
 *       ★ 启用内部上拉 (~40kΩ)，无需外接 4.7kΩ 电阻 ★
 * I2C:  10kHz 低速模式，配合内部弱上拉可靠通信
 *       (外接 4.7kΩ 上拉后可改回 100000)
 * ============================================================ */
void I2C_MyInit(void)
{
    /* --- 第一步：使能时钟 --- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,  ENABLE);

    /* --- 第二步：配置 SCL 和 SDA 引脚为复用开漏输出 --- */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;  // PB6 = SCL, PB7 = SDA
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*
     * ★ 启用 STM32 内部上拉电阻 (~40kΩ) ★
     * AF_OD 模式下，设置 ODR=1 会激活内部上拉。
     * 配合下面的 10kHz 低速 I2C，无需外接 4.7kΩ 电阻即可通信。
     */
    GPIOB->BSRR = GPIO_Pin_6 | GPIO_Pin_7;

    /* --- 第三步：配置 I2C 参数 --- */
    I2C_InitTypeDef I2C_InitStruct;
    I2C_InitStruct.I2C_ClockSpeed        = 10000;          // 10kHz 低速，匹配内部弱上拉
                                                           // 有外接 4.7kΩ 上拉后可改为 100000
    I2C_InitStruct.I2C_Mode              = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle         = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1       = 0x00;
    I2C_InitStruct.I2C_Ack               = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStruct);

    /* --- 第四步：使能 I2C1 外设 --- */
    I2C_Cmd(I2C1, ENABLE);
}

/* ============================================================
 * I2C 超时计数器
 * ------------------------------------------------------------
 * 每次 while 循环最多等待 I2C_TIMEOUT 次，防止硬件故障时死循环。
 * 100000 次 ≈ 约 10ms @ 72MHz (粗略估计)
 * ============================================================ */
#define I2C_TIMEOUT   100000

/* ============================================================
 * I2C 总线恢复
 * ------------------------------------------------------------
 * 当 I2C 通信超时后，复位外设并重新初始化。
 * STM32F103 的 PE=0 会触发内部状态机复位，所以需要
 * 完整重新配置 I2C 参数，不能只做 disable/enable。
 * ============================================================ */
static void I2C_BusReset(void)
{
    /* 先发 STOP 释放总线 */
    I2C_GenerateSTOP(I2C1, ENABLE);

    /* 关闭外设 */
    I2C_Cmd(I2C1, DISABLE);
    Delay_ms(2);

    /*
     * 重新配置 I2C（PE=0 会丢失部分状态，完整重配最安全）
     * 与 I2C_MyInit() 保持一致的参数
     */
    {
        I2C_InitTypeDef I2C_InitStruct;
        I2C_InitStruct.I2C_ClockSpeed        = 10000;
        I2C_InitStruct.I2C_Mode              = I2C_Mode_I2C;
        I2C_InitStruct.I2C_DutyCycle         = I2C_DutyCycle_2;
        I2C_InitStruct.I2C_OwnAddress1       = 0x00;
        I2C_InitStruct.I2C_Ack               = I2C_Ack_Enable;
        I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
        I2C_Init(I2C1, &I2C_InitStruct);
    }

    I2C_Cmd(I2C1, ENABLE);
    Delay_ms(1);
}

/* ============================================================
 * 通用 I2C 写寄存器
 * ------------------------------------------------------------
 * SlaveAddr: 7位地址左移一位后的写地址
 * RegAddr:  目标寄存器
 * Data:     写入数据
 * 返回值:   0 = 成功, 1 = 超时失败
 * ============================================================ */
static uint8_t I2C_WriteReg(uint8_t SlaveAddr, uint8_t RegAddr, uint8_t Data)
{
    uint32_t timeout;

    /* 发送 START */
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR)
    {
        if (--timeout == 0) { I2C_BusReset(); return 1; }
    }

    /* 发送从机地址 + 写方向 */
    I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR)
    {
        if (--timeout == 0) { I2C_BusReset(); return 1; }
    }

    /* 发送寄存器地址 */
    I2C_SendData(I2C1, RegAddr);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR)
    {
        if (--timeout == 0) { I2C_BusReset(); return 1; }
    }

    /* 发送数据 */
    I2C_SendData(I2C1, Data);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR)
    {
        if (--timeout == 0) { I2C_BusReset(); return 1; }
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    return 0;
}

/* ============================================================
 * 通用 I2C 读单个寄存器
 * ------------------------------------------------------------
 * SlaveAddr: 7位地址左移一位后的写地址
 * RegAddr:  目标寄存器
 * pData:    输出参数，读取到的数据（仅在成功时有效）
 * 返回值:   0 = 成功, 1 = 超时失败
 * ============================================================ */
static uint8_t I2C_ReadReg(uint8_t SlaveAddr, uint8_t RegAddr, uint8_t *pData)
{
    uint32_t timeout;

    /* 第一阶段: 写寄存器地址 */
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR)
    {
        if (--timeout == 0) { I2C_BusReset(); return 1; }
    }

    I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR)
    {
        if (--timeout == 0) { I2C_BusReset(); return 1; }
    }

    I2C_SendData(I2C1, RegAddr);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR)
    {
        if (--timeout == 0) { I2C_BusReset(); return 1; }
    }

    /* 第二阶段: 读数据 */
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR)
    {
        if (--timeout == 0) { I2C_BusReset(); return 1; }
    }

    I2C_Send7bitAddress(I2C1, SlaveAddr | 0x01, I2C_Direction_Receiver);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == ERROR)
    {
        if (--timeout == 0) { I2C_BusReset(); return 1; }
    }

    I2C_AcknowledgeConfig(I2C1, DISABLE);

    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) == ERROR)
    {
        if (--timeout == 0) { I2C_BusReset(); return 1; }
    }
    *pData = I2C_ReceiveData(I2C1);

    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    return 0;
}

/* ============================================================
 * 通用 I2C 连续读取多个寄存器
 * ------------------------------------------------------------
 * SlaveAddr: 7位地址左移一位后的写地址
 * RegAddr:  起始寄存器地址
 * pBuf:     接收缓冲区
 * len:      读取字节数
 * 返回值:   0 = 成功, 1 = 超时失败
 * ============================================================ */
static uint8_t I2C_ReadMulti(uint8_t SlaveAddr, uint8_t RegAddr, uint8_t *pBuf, uint8_t len)
{
    uint32_t timeout;

    /* 第一阶段: 写起始寄存器地址 */
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR)
    {
        if (--timeout == 0) { I2C_BusReset(); return 1; }
    }

    I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR)
    {
        if (--timeout == 0) { I2C_BusReset(); return 1; }
    }

    I2C_SendData(I2C1, RegAddr);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR)
    {
        if (--timeout == 0) { I2C_BusReset(); return 1; }
    }

    /* 第二阶段: 切换方向，连续读 */
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR)
    {
        if (--timeout == 0) { I2C_BusReset(); return 1; }
    }

    I2C_Send7bitAddress(I2C1, SlaveAddr | 0x01, I2C_Direction_Receiver);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == ERROR)
    {
        if (--timeout == 0) { I2C_BusReset(); return 1; }
    }

    while (len)
    {
        if (len == 1)
        {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
        }

        timeout = I2C_TIMEOUT;
        while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) == ERROR)
        {
            if (--timeout == 0) { I2C_BusReset(); return 1; }
        }
        *pBuf = I2C_ReceiveData(I2C1);
        pBuf++;
        len--;
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    return 0;
}

/* ============================================================
 * MPU9250 本体 (加速度+陀螺仪) 的 I2C 封装
 * ============================================================ */
static uint8_t MPU9250_WriteReg(uint8_t RegAddr, uint8_t Data)
{
    return I2C_WriteReg(MPU9250_ADDR_WRITE, RegAddr, Data);
}

static uint8_t MPU9250_ReadReg(uint8_t RegAddr, uint8_t *pData)
{
    return I2C_ReadReg(MPU9250_ADDR_WRITE, RegAddr, pData);
}

static uint8_t MPU9250_ReadMulti(uint8_t RegAddr, uint8_t *pBuf, uint8_t len)
{
    return I2C_ReadMulti(MPU9250_ADDR_WRITE, RegAddr, pBuf, len);
}

/* ============================================================
 * AK8963 磁力计的 I2C 封装
 * ============================================================ */
static uint8_t AK8963_WriteReg(uint8_t RegAddr, uint8_t Data)
{
    return I2C_WriteReg(AK8963_ADDR_WRITE, RegAddr, Data);
}

static uint8_t AK8963_ReadReg(uint8_t RegAddr, uint8_t *pData)
{
    return I2C_ReadReg(AK8963_ADDR_WRITE, RegAddr, pData);
}

static uint8_t AK8963_ReadMulti(uint8_t RegAddr, uint8_t *pBuf, uint8_t len)
{
    return I2C_ReadMulti(AK8963_ADDR_WRITE, RegAddr, pBuf, len);
}

/* ============================================================
 * MPU9250 初始化
 * ------------------------------------------------------------
 * 1. 初始化 I2C1 硬件外设
 * 2. 唤醒 MPU9250（退出睡眠模式）
 * 3. 配置基本参数（采样率、量程等）
 * 4. 启用 BYPASS 模式，初始化 AK8963 磁力计
 * ============================================================ */
void MPU9250_Init(void)
{
    /*
     * 注意: I2C_MyInit() 由 main() 在调用本函数之前完成，
     *       这里不再重复初始化，避免在 I2C 已使能状态下
     *       重复调用 I2C_Init 导致硬件状态异常。
     */

    /* 等待 MPU9250 上电稳定 */
    Delay_ms(100);

    /*
     * 【关键步骤】唤醒 MPU9250
     * 向 PWR_MGMT_1 写入 0x00: 清除 SLEEP 位，退出睡眠
     */
    MPU9250_WriteReg(MPU9250_PWR_MGMT_1, 0x00);
    Delay_ms(10);

    /*
     * 配置采样率分频器
     * 采样率 = 1kHz / (1 + SMPLRT_DIV)
     * SMPLRT_DIV = 9  →  采样率 = 100Hz
     */
    MPU9250_WriteReg(MPU9250_SMPLRT_DIV, 0x09);

    /*
     * 配置数字低通滤波器 (DLPF)
     * DLPF_CFG = 0x06 → 带宽约 5Hz
     */
    MPU9250_WriteReg(MPU9250_CONFIG, 0x06);

    /*
     * 配置陀螺仪量程
     * FS_SEL = 0x00 → ±250 °/s (灵敏度 131 LSB/°/s)
     */
    MPU9250_WriteReg(MPU9250_GYRO_CONFIG, 0x00);

    /*
     * 配置加速度计量程
     * AFS_SEL = 0x00 → ±2g (灵敏度 16384 LSB/g)
     */
    MPU9250_WriteReg(MPU9250_ACCEL_CONFIG, 0x00);

    /* ============================================================
     * 【MPU9250 特有步骤】启用 BYPASS 模式，初始化磁力计
     * ------------------------------------------------------------
     * MPU9250 内部有一个辅助 I2C 总线，AK8963 挂载在上面。
     * 要直接访问 AK8963，需要先启用 BYPASS 模式:
     *   INT_PIN_CFG (0x37) bit[1] BYPASS_EN = 1
     *
     * 启用 BYPASS 后，辅助 I2C 的信号直接连到主 I2C 总线，
     * STM32 可以直接向 AK8963 的地址 (0x0C) 发送读写命令。
     *
     * 注意: 必须确保 I2C_MST_EN (USER_CTRL bit5) = 0 才能用 BYPASS
     * ============================================================ */
    MPU9250_WriteReg(MPU9250_USER_CTRL,  0x00);  // 关闭 I2C 主模式
    Delay_ms(10);
    MPU9250_WriteReg(MPU9250_INT_PIN_CFG, 0x02);  // 启用 BYPASS
    Delay_ms(10);

    /*
     * 初始化 AK8963 磁力计
     * CNTL1 寄存器:
     *   0x00 → 掉电模式 (先掉电再配置，确保状态干净)
     *   0x12 → 连续测量 8Hz 模式 (低功耗，适合姿态检测)
     *   0x16 → 连续测量 100Hz 模式 (高速，但功耗大)
     *
     * BIT 位 = 0 → 14 位输出 (范围 ±4800μT, 灵敏度 0.6μT/LSB)
     */
    AK8963_WriteReg(AK8963_CNTL1, 0x00);  // 先进入掉电模式
    Delay_ms(10);
    AK8963_WriteReg(AK8963_CNTL1, 0x12);  // 连续测量 8Hz, 16位输出
    Delay_ms(10);
}

/* ============================================================
 * 读取 MPU9250 器件 ID (WHO_AM_I)
 * ------------------------------------------------------------
 * MPU9250 返回 0x71 (MPU6050 返回 0x68)
 * ============================================================ */
uint8_t MPU9250_ReadID(void)
{
    uint8_t id = 0;
    MPU9250_ReadReg(MPU9250_WHO_AM_I, &id);
    return id;
}

/* ============================================================
 * 读取 AK8963 磁力计器件 ID (WIA)
 * ------------------------------------------------------------
 * AK8963 返回 0x48
 * ============================================================ */
uint8_t MPU9250_ReadMagID(void)
{
    uint8_t id = 0;
    AK8963_ReadReg(AK8963_WIA, &id);
    return id;
}

/* ============================================================
 * 读取 MPU9250 全部传感器数据 (6轴加速度+陀螺仪)
 * ------------------------------------------------------------
 * 一次性读取 14 字节: 6字节加速度 + 2字节温度 + 6字节陀螺仪
 * pData: 指向 MPU9250_Data 结构体的指针
 * ============================================================ */
void MPU9250_ReadData(MPU9250_Data *pData)
{
    uint8_t buf[14];

    /* 从 ACCEL_XOUT_H (0x3B) 开始连续读取 14 个字节 */
    MPU9250_ReadMulti(MPU9250_ACCEL_XOUT_H, buf, 14);

    /* 数据拼接: 高字节在前 (Big Endian) */
    pData->Accel_X = (int16_t)((buf[0]  << 8) | buf[1]);
    pData->Accel_Y = (int16_t)((buf[2]  << 8) | buf[3]);
    pData->Accel_Z = (int16_t)((buf[4]  << 8) | buf[5]);

    pData->Temperature = (int16_t)((buf[6]  << 8) | buf[7]);

    pData->Gyro_X = (int16_t)((buf[8]  << 8) | buf[9]);
    pData->Gyro_Y = (int16_t)((buf[10] << 8) | buf[11]);
    pData->Gyro_Z = (int16_t)((buf[12] << 8) | buf[13]);
}

/* ============================================================
 * 仅读取加速度数据 (快速版)
 * ============================================================ */
void MPU9250_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];

    MPU9250_ReadMulti(MPU9250_ACCEL_XOUT_H, buf, 6);

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
}

/* ============================================================
 * 仅读取陀螺仪数据 (快速版)
 * ============================================================ */
void MPU9250_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    MPU9250_ReadMulti(MPU9250_GYRO_XOUT_H, buf, 6);

    *gx = (int16_t)((buf[0] << 8) | buf[1]);
    *gy = (int16_t)((buf[2] << 8) | buf[3]);
    *gz = (int16_t)((buf[4] << 8) | buf[5]);
}

/* ============================================================
 * 读取 AK8963 磁力计数据
 * ------------------------------------------------------------
 * AK8963 数据寄存器从 HXL (0x03) 开始，共 7 字节:
 *   HXL, HXH, HYL, HYH, HZL, HZH, ST2
 *
 * ST2 (Status 2) 必须读取才能锁存下一组数据。
 * 如果只读了 6 字节而没读 ST2，后续读取会返回旧数据。
 *
 * 16 位输出模式 (BIT=1): 灵敏度 0.15 μT/LSB, 范围 ±4800μT
 *
 * mx, my, mz: 输出的磁力计原始值 (16位有符号)
 * ============================================================ */
void MPU9250_ReadMag(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t buf[7];  // 6 bytes data + 1 byte ST2

    AK8963_ReadMulti(AK8963_HXL, buf, 7);

    /*
     * AK8963 数据格式: 低字节在前 (Little Endian)
     * 与 MPU9250 的加速度/陀螺仪 (Big Endian) 不同！
     * 所以是 高字节<<8 | 低字节
     */
    *mx = (int16_t)((buf[1] << 8) | buf[0]);
    *my = (int16_t)((buf[3] << 8) | buf[2]);
    *mz = (int16_t)((buf[5] << 8) | buf[4]);

    /*
     * 检查 ST2 的 HOFL 位 (bit 3)
     * HOFL = 1 表示数据溢出，但传感器通常不会溢出不处理即可
     */
    (void)buf[6];  // ST2 已读，数据已锁存
}

/* ===================================================================
 *                    姿态解算与滤波模块
 * ===================================================================
 * 本模块对原始传感器数据进行处理，输出稳定的姿态角度:
 *
 *   1. 静止校准  → 开机采集零偏，后续测量减去
 *   2. 死区处理  → 微小角速度直接置零
 *   3. 低通滤波  → 降低高频抖动
 *   4. 互补滤波  → 融合陀螺仪(短期精准) + 加速度计(长期准确)
 *   5. 磁力计修正 → MPU9250 特有! 磁力计修正 Yaw，不再漂移
 *
 * 【MPU9250 vs MPU6050 姿态解算的关键区别】
 *   MPU6050: Yaw 仅靠陀螺仪积分 → 持续漂移
 *   MPU9250: Yaw 靠陀螺仪 + 磁力计互补 → 不漂移
 * =================================================================== */

/* ============================================================
 * 滤波器状态变量 (模块级静态变量，外部不可见)
 * ============================================================ */

/* --- 陀螺仪零偏（开机校准得到） --- */
static int16_t  g_GyroBiasX = 0;
static int16_t  g_GyroBiasY = 0;
static int16_t  g_GyroBiasZ = 0;
static uint8_t  g_Calibrated = 0;

/* --- 磁力计校准 (硬铁偏移 + 比例) --- */
static float    g_MagOffsetX = 0;           // 磁力计 X 硬铁偏移
static float    g_MagOffsetY = 0;           // 磁力计 Y 硬铁偏移
static float    g_MagOffsetZ = 0;           // 磁力计 Z 硬铁偏移

/* --- 低通滤波器历史值 --- */
static float    g_GyroFiltX  = 0;
static float    g_GyroFiltY  = 0;
static float    g_GyroFiltZ  = 0;
static float    g_AccelFiltX = 0;
static float    g_AccelFiltY = 0;
static float    g_AccelFiltZ = 0;
static float    g_MagFiltX   = 0;
static float    g_MagFiltY   = 0;
static float    g_MagFiltZ   = 0;

/* --- 互补滤波器输出的角度 --- */
static float    g_Roll  = 0;
static float    g_Pitch = 0;
static float    g_Yaw   = 0;

/* --- 静止检测历史值 --- */
static int16_t  g_LastAccelX = 0;
static int16_t  g_LastAccelY = 0;
static int16_t  g_LastAccelZ = 0;

/* ============================================================
 * MPU9250 上电静止校准
 * ------------------------------------------------------------
 * 校准项目:
 *   1. 陀螺仪零偏 (与 MPU6050 相同)
 *   2. 加速度计初始 Roll / Pitch
 *   3. 磁力计初始 Yaw (MPU9250 特有!)
 *
 * 【调用时机】
 *   在 MPU9250_Init() 之后、主循环之前调用一次。
 *   使用前务必保持模块静止不动！
 * ============================================================ */
#define CALIB_SAMPLES   200

void MPU9250_Calibrate(void)
{
    int32_t sumGx = 0, sumGy = 0, sumGz = 0;
    int32_t sumMx = 0, sumMy = 0, sumMz = 0;
    uint16_t i;
    MPU9250_Data raw;

    for (i = 0; i < CALIB_SAMPLES; i++)
    {
        MPU9250_ReadData(&raw);

        sumGx += raw.Gyro_X;
        sumGy += raw.Gyro_Y;
        sumGz += raw.Gyro_Z;

        /* 同时采集磁力计数据用于初始偏航角 */
        if (i % 10 == 0)  // 磁力计 8Hz 输出，不宜太快
        {
            int16_t mx, my, mz;
            MPU9250_ReadMag(&mx, &my, &mz);
            sumMx += mx;
            sumMy += my;
            sumMz += mz;
        }

        Delay_ms(5);
    }

    /* 陀螺仪零偏 */
    g_GyroBiasX = (int16_t)(sumGx / CALIB_SAMPLES);
    g_GyroBiasY = (int16_t)(sumGy / CALIB_SAMPLES);
    g_GyroBiasZ = (int16_t)(sumGz / CALIB_SAMPLES);

    /* 磁力计硬铁偏移 (简单平均) */
    {
        uint8_t magSamples = CALIB_SAMPLES / 10;  // ≈ 20 次
        g_MagOffsetX = (float)sumMx / magSamples;
        g_MagOffsetY = (float)sumMy / magSamples;
        g_MagOffsetZ = (float)sumMz / magSamples;
    }

    /* 初始化低通滤波器状态 */
    g_GyroFiltX  = 0;
    g_GyroFiltY  = 0;
    g_GyroFiltZ  = 0;
    g_AccelFiltX = (float)raw.Accel_X;
    g_AccelFiltY = (float)raw.Accel_Y;
    g_AccelFiltZ = (float)raw.Accel_Z;
    g_MagFiltX   = 0;
    g_MagFiltY   = 0;
    g_MagFiltZ   = 0;

    /* 从加速度计计算初始 Roll / Pitch */
    {
        float ax = (float)raw.Accel_X;
        float ay = (float)raw.Accel_Y;
        float az = (float)raw.Accel_Z;

        g_Roll  = atan2f(ay, az) * 180.0f / 3.14159265f;
        g_Pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / 3.14159265f;
    }

    /*
     * 从磁力计计算初始 Yaw
     *
     * 磁力计测量的是地磁场在传感器坐标系上的投影。
     * 在水平放置时，Yaw = atan2(My, Mx) + 当地磁偏角
     *
     * 如果传感器不是水平的（有 Roll/Pitch），需要先做倾斜补偿:
     *   Mx_comp = Mx*cos(Pitch) + My*sin(Roll)*sin(Pitch) - Mz*cos(Roll)*sin(Pitch)
     *   My_comp = My*cos(Roll) + Mz*sin(Roll)
     *   Yaw = atan2(-My_comp, Mx_comp)
     */
    {
        int16_t mx, my, mz;
        float   mx_c, my_c;
        float   roll_rad  = g_Roll  * 3.14159265f / 180.0f;
        float   pitch_rad = g_Pitch * 3.14159265f / 180.0f;
        float   cos_r = cosf(roll_rad);
        float   sin_r = sinf(roll_rad);
        float   cos_p = cosf(pitch_rad);
        float   sin_p = sinf(pitch_rad);

        MPU9250_ReadMag(&mx, &my, &mz);

        /* 去硬铁偏移 */
        float mxf = (float)mx - g_MagOffsetX;
        float myf = (float)my - g_MagOffsetY;
        float mzf = (float)mz - g_MagOffsetZ;

        /* 倾斜补偿 */
        mx_c =  mxf * cos_p + myf * sin_r * sin_p - mzf * cos_r * sin_p;
        my_c =  myf * cos_r + mzf * sin_r;

        g_Yaw = atan2f(-my_c, mx_c) * 180.0f / 3.14159265f;
    }

    g_Calibrated = 1;

    /* 保存当前加速度值用于后续静止检测 */
    g_LastAccelX = raw.Accel_X;
    g_LastAccelY = raw.Accel_Y;
    g_LastAccelZ = raw.Accel_Z;
}

/* ============================================================
 * 获取滤波融合后的姿态角度
 * ------------------------------------------------------------
 * 【处理流程 — 完整的数据清洗管道】(MPU9250 增强版)
 *
 *   MPU9250 原始数据 (9轴)
 *       │
 *       ├──→ ① 零偏修正    g_corrected = g_raw - g_bias
 *       │
 *       ├──→ ② 死区处理    if (|g| < DEADZONE) g = 0
 *       │
 *       ├──→ ③ 低通滤波    g_filt = g_filt_old + α × (g - g_filt_old)
 *       │
 *       ├──→ ④ 加速度求    Roll_acc  = atan2(Ay, Az)
 *       │    Roll/Pitch   Pitch_acc = atan2(-Ax, √(Ay²+Az²))
 *       │
 *       ├──→ ⑤ 陀螺仪积分  angle_gyro = angle_old + gyro × dt
 *       │
 *       ├──→ ⑥ 互补滤波    angle = GYRO_TRUST × angle_gyro + (1-TRUST) × angle_accel
 *       │    (Roll/Pitch)
 *       │
 *       └──→ ⑦ 磁力计修正  Yaw_mag = atan2(-My_comp, Mx_comp)
 *            (Yaw)        Yaw = (1-MAG_CORRECT) × yaw_gyro + MAG_CORRECT × yaw_mag
 *                           ★ MPU9250 特有! Yaw 不再无限漂移 ★
 *
 * pAngle: 输出的姿态角度指针
 * ============================================================ */
#define DT                  0.10f           // 采样间隔 (秒)

void MPU9250_GetAngle(MPU9250_Angle *pAngle)
{
    MPU9250_Data raw;
    float gx, gy, gz;
    float ax, ay, az;
    float roll_acc, pitch_acc;
    float roll_gyro, pitch_gyro, yaw_gyro;
    float yaw_mag;

    if (pAngle == 0) return;

    /* ===== 第一步: 读原始数据 ===== */
    MPU9250_ReadData(&raw);

    /* ===== 第二步: 零偏修正 ===== */
    gx = (float)(raw.Gyro_X - g_GyroBiasX);
    gy = (float)(raw.Gyro_Y - g_GyroBiasY);
    gz = (float)(raw.Gyro_Z - g_GyroBiasZ);

    ax = (float)raw.Accel_X;
    ay = (float)raw.Accel_Y;
    az = (float)raw.Accel_Z;

    /* ===== 第三步: 死区处理 ===== */
    if (gx < GYRO_DEADZONE     && gx > -GYRO_DEADZONE)     gx = 0;
    if (gy < GYRO_DEADZONE     && gy > -GYRO_DEADZONE)     gy = 0;
    if (gz < GYRO_DEADZONE_YAW && gz > -GYRO_DEADZONE_YAW) gz = 0;

    /* ===== 第四步: 低通滤波 ===== */
    g_GyroFiltX  += LPF_ALPHA_GYRO  * (gx - g_GyroFiltX);
    g_GyroFiltY  += LPF_ALPHA_GYRO  * (gy - g_GyroFiltY);
    g_GyroFiltZ  += LPF_ALPHA_GYRO  * (gz - g_GyroFiltZ);

    g_AccelFiltX += LPF_ALPHA_ACCEL * (ax - g_AccelFiltX);
    g_AccelFiltY += LPF_ALPHA_ACCEL * (ay - g_AccelFiltY);
    g_AccelFiltZ += LPF_ALPHA_ACCEL * (az - g_AccelFiltZ);

    /* ===== 第五步: 陀螺仪积分 ===== */
    roll_gyro  = g_Roll  + (g_GyroFiltX / 131.0f) * DT;
    pitch_gyro = g_Pitch + (g_GyroFiltY / 131.0f) * DT;
    yaw_gyro   = g_Yaw   + (g_GyroFiltZ / 131.0f) * DT;

    /* ===== 第六步: 加速度计求 Roll 和 Pitch ===== */
    roll_acc  = atan2f(g_AccelFiltY, g_AccelFiltZ) * 180.0f / 3.14159265f;
    pitch_acc = atan2f(-g_AccelFiltX,
                       sqrtf(g_AccelFiltY * g_AccelFiltY +
                             g_AccelFiltZ * g_AccelFiltZ))
                * 180.0f / 3.14159265f;

    /* ===== 第七步: 自适应互补滤波 (Roll / Pitch) ===== */
    {
        float trust_roll, trust_pitch;

        trust_roll  = (g_GyroFiltX >  5.0f * 131.0f || g_GyroFiltX < -5.0f * 131.0f)
                      ? COMP_GYRO_TRUST : 0.85f;

        trust_pitch = (g_GyroFiltY >  5.0f * 131.0f || g_GyroFiltY < -5.0f * 131.0f)
                      ? COMP_GYRO_TRUST : 0.85f;

        g_Roll  = trust_roll  * roll_gyro  + (1.0f - trust_roll)  * roll_acc;
        g_Pitch = trust_pitch * pitch_gyro + (1.0f - trust_pitch) * pitch_acc;
    }

    /* ===== 第八步: 磁力计 Yaw 修正 (MPU9250 特有!) ===== */
    /*
     * 读取磁力计数据并做低通滤波
     */
    {
        int16_t mx, my, mz;
        MPU9250_ReadMag(&mx, &my, &mz);

        /* 去硬铁偏移 */
        float mxf = (float)mx - g_MagOffsetX;
        float myf = (float)my - g_MagOffsetY;
        float mzf = (float)mz - g_MagOffsetZ;

        /* 低通滤波 */
        g_MagFiltX += LPF_ALPHA_MAG * (mxf - g_MagFiltX);
        g_MagFiltY += LPF_ALPHA_MAG * (myf - g_MagFiltY);
        g_MagFiltZ += LPF_ALPHA_MAG * (mzf - g_MagFiltZ);

        /* 倾斜补偿 (将磁力计数据投影到水平面) */
        {
            float roll_rad  = g_Roll  * 3.14159265f / 180.0f;
            float pitch_rad = g_Pitch * 3.14159265f / 180.0f;
            float cos_r = cosf(roll_rad);
            float sin_r = sinf(roll_rad);
            float cos_p = cosf(pitch_rad);
            float sin_p = sinf(pitch_rad);

            float mx_c = g_MagFiltX * cos_p
                       + g_MagFiltY * sin_r * sin_p
                       - g_MagFiltZ * cos_r * sin_p;
            float my_c = g_MagFiltY * cos_r
                       + g_MagFiltZ * sin_r;

            yaw_mag = atan2f(-my_c, mx_c) * 180.0f / 3.14159265f;
        }

        /*
         * 互补融合陀螺仪 Yaw + 磁力计 Yaw
         * 陀螺仪积分提供短期响应速度
         * 磁力计提供长期零漂修正
         *
         * 注意: 当 Yaw 跨越 ±180° 边界时需要处理角度环绕
         */
        {
            float delta = yaw_mag - yaw_gyro;

            /* 处理角度环绕 (确保 delta 在 -180° ~ +180°) */
            if (delta > 180.0f)  delta -= 360.0f;
            if (delta < -180.0f) delta += 360.0f;

            g_Yaw = yaw_gyro + MAG_YAW_CORRECT * delta;
        }

        /* 归一化 Yaw 到 -180° ~ +180° */
        if (g_Yaw > 180.0f)  g_Yaw -= 360.0f;
        if (g_Yaw < -180.0f) g_Yaw += 360.0f;
    }

    /* ===== 第九步: 输出角度 ===== */
    pAngle->Roll  = g_Roll;
    pAngle->Pitch = g_Pitch;
    pAngle->Yaw   = g_Yaw;

    /* 保存当前加速度值用于静止检测 */
    g_LastAccelX = raw.Accel_X;
    g_LastAccelY = raw.Accel_Y;
    g_LastAccelZ = raw.Accel_Z;
}

/* ============================================================
 * 检测设备是否处于静止状态
 * ------------------------------------------------------------
 * 比较前后两帧加速度的差值，三个轴变化都很小 → 静止
 *
 * 返回值: 1 = 静止, 0 = 运动中
 * ============================================================ */
uint8_t MPU9250_IsStationary(void)
{
    MPU9250_Data raw;
    int16_t dx, dy, dz;

    MPU9250_ReadData(&raw);

    dx = raw.Accel_X - g_LastAccelX;
    dy = raw.Accel_Y - g_LastAccelY;
    dz = raw.Accel_Z - g_LastAccelZ;

    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    if (dz < 0) dz = -dz;

    if (dx < STATIONARY_THRESH && dy < STATIONARY_THRESH && dz < STATIONARY_THRESH)
    {
        return 1;
    }
    return 0;
}
