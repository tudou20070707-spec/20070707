#include "MPU6050.h"
#include "Delay.h"
#include <math.h>

/* ===================================================================
 *                           MPU6050 驱动文件
 * ===================================================================
 * 本文件实现通过 STM32F103 的 I2C1 硬件外设与 MPU6050 传感器通信。
 *
 * 【I2C 通信原理简述】
 *   I2C (Inter-Integrated Circuit) 是一种两线式串行总线：
 *     - SCL (Serial Clock)    : 时钟线，由主机控制
 *     - SDA (Serial Data)     : 数据线，双向传输
 *   每个从设备有唯一的 7 位或 10 位地址，MPU6050 默认 7 位地址为 0x68。
 *
 * 【一次典型的 I2C 写操作 (主机 → 从机)】
 *   1. 主机发送 START 起始信号
 *   2. 主机发送 7位从机地址 + 1位方向(0=写)
 *   3. 从机回应 ACK (拉低 SDA)
 *   4. 主机发送寄存器地址
 *   5. 从机回应 ACK
 *   6. 主机发送数据字节
 *   7. 从机回应 ACK
 *   8. 主机发送 STOP 停止信号
 *
 * 【一次典型的 I2C 读操作 (主机 ← 从机)】
 *   1. 主机发送 START
 *   2. 主机发送 7位从机地址 + 1位方向(0=写)，先告诉从机要读哪个寄存器
 *   3. 从机回应 ACK
 *   4. 主机发送目标寄存器地址
 *   5. 从机回应 ACK
 *   6. 主机再次发送 START (也叫 Repeated START，即重复起始条件)
 *   7. 主机发送 7位从机地址 + 1位方向(1=读)
 *   8. 从机回应 ACK
 *   9. 从机发送数据，主机回应 ACK（最后一字节回应 NACK）
 *  10. 主机发送 STOP
 *
 * 【硬件连接 (默认使用 I2C1)】
 *   PB6 → MPU6050 SCL  (I2C1_SCL)
 *   PB7 → MPU6050 SDA  (I2C1_SDA)
 * =================================================================== */

/* ============================================================
 * 内部辅助函数声明
 * ============================================================ */
void I2C_MyInit(void);                                   // 初始化 I2C1 硬件（公开，供 OLED 等共用）
static void MPU6050_WriteReg(uint8_t RegAddr, uint8_t Data);  // 向指定寄存器写入一个字节
static uint8_t MPU6050_ReadReg(uint8_t RegAddr);              // 从指定寄存器读取一个字节
static void MPU6050_ReadMulti(uint8_t RegAddr, uint8_t *pBuf, uint8_t len); // 连续读取多个字节

/* ============================================================
 * I2C1 硬件初始化
 * ------------------------------------------------------------
 * 时钟: 使能 GPIOB 和 I2C1 外设时钟
 * GPIO: PB6 (SCL) 和 PB7 (SDA) 配置为复用开漏输出
 *       两个引脚必须外接 4.7kΩ 上拉电阻到 3.3V
 * I2C:  配置为标准模式 (100kHz) 或快速模式 (400kHz)
 *       这里配置为 100kHz 标准模式，兼容性最好
 * ============================================================ */
void I2C_MyInit(void)
{
    /* --- 第一步：使能时钟 --- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  // GPIOB 挂载在 APB2 总线上
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,  ENABLE);  // I2C1 挂载在 APB1 总线上（注意!）

    /* --- 第二步：配置 SCL 和 SDA 引脚为复用开漏输出 --- */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_OD;          // 复用开漏输出（I2C 标准要求）
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;  // PB6 = SCL, PB7 = SDA
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;          // 50MHz 驱动能力
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* --- 第三步：配置 I2C 参数 --- */
    I2C_InitTypeDef I2C_InitStruct;
    I2C_InitStruct.I2C_ClockSpeed        = 100000;         // 时钟频率: 100kHz (标准模式)
                                                           //  也可设为 400000 进入快速模式
    I2C_InitStruct.I2C_Mode              = I2C_Mode_I2C;   // 工作模式: 标准 I2C 模式
    I2C_InitStruct.I2C_DutyCycle         = I2C_DutyCycle_2; // 快速模式占空比 (标准模式下无效)
    I2C_InitStruct.I2C_OwnAddress1       = 0x00;            // 本机地址 (STM32 作为主机时无需关心)
    I2C_InitStruct.I2C_Ack               = I2C_Ack_Enable;  // 使能 ACK 应答
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // 7位地址模式
    I2C_Init(I2C1, &I2C_InitStruct);

    /* --- 第四步：使能 I2C1 外设 --- */
    I2C_Cmd(I2C1, ENABLE);
}

/* ============================================================
 * MPU6050 写寄存器
 * ------------------------------------------------------------
 * 流程: START → 设备地址(写) → 寄存器地址 → 数据 → STOP
 * RegAddr: 目标寄存器地址
 * Data:    要写入的 8 位数据
 * ============================================================ */
static void MPU6050_WriteReg(uint8_t RegAddr, uint8_t Data)
{
    /* 第1步: 发送起始信号 */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* 等待起始信号发送完成 (SB=Start Bit 标志置位) */
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);

    /* 第2步: 发送 7 位设备地址 + 写方向位(0) */
    I2C_Send7bitAddress(I2C1, MPU6050_ADDR_WRITE, I2C_Direction_Transmitter);
    /* 等待从机应答 (ADDR 标志置位) */
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR);

    /* 第3步: 发送要写入的寄存器地址 */
    I2C_SendData(I2C1, RegAddr);
    /* 等待发送完成 (TXE=发送寄存器空, BTF=字节传输完成) */
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR);

    /* 第4步: 发送要写入的数据 */
    I2C_SendData(I2C1, Data);
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR);

    /* 第5步: 发送停止信号，释放总线 */
    I2C_GenerateSTOP(I2C1, ENABLE);
}

/* ============================================================
 * MPU6050 读单个寄存器
 * ------------------------------------------------------------
 * 流程: START → 设备地址(写) → 寄存器地址 →
 *       Repeated START → 设备地址(读) → 读取1字节(+NACK) → STOP
 * RegAddr: 目标寄存器地址
 * 返回值: 读取到的 8 位数据
 * ============================================================ */
static uint8_t MPU6050_ReadReg(uint8_t RegAddr)
{
    uint8_t data;

    /* --- 第一阶段: 告诉 MPU6050 我们要读哪个寄存器 --- */

    /* 发送起始信号 */
    I2C_GenerateSTART(I2C1, ENABLE);
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);

    /* 发送设备地址 + 写方向（先写寄存器地址） */
    I2C_Send7bitAddress(I2C1, MPU6050_ADDR_WRITE, I2C_Direction_Transmitter);
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR);

    /* 发送目标寄存器地址 */
    I2C_SendData(I2C1, RegAddr);
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR);

    /* --- 第二阶段: 切换到读模式，接收数据 --- */

    /* 再次发送起始信号 (Repeated START，不释放总线) */
    I2C_GenerateSTART(I2C1, ENABLE);
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);

    /* 发送设备地址 + 读方向 */
    I2C_Send7bitAddress(I2C1, MPU6050_ADDR_READ, I2C_Direction_Receiver);
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == ERROR);

    /*
     * 重要! 在读取最后一个字节之前，必须先关闭 ACK
     * 这样 MPU6050 收到 NACK 后就知道主机不再继续读，会自动释放总线
     * 这里只读 1 个字节，所以在接收前就要关闭 ACK
     */
    I2C_AcknowledgeConfig(I2C1, DISABLE);

    /*
     * 先清除 ADDR 标志（通过读 SR1 再读 SR2），
     * 然后发送 STOP（在最后一个字节接收前就设好 STOP），
     * 这样接收完这一字节后 STOP 自动生效
     */
    /* 等待 RXNE (接收寄存器非空)，即数据到达 */
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) == ERROR);

    /* 读取接收到的数据 */
    data = I2C_ReceiveData(I2C1);

    /* 发送停止信号 */
    I2C_GenerateSTOP(I2C1, ENABLE);

    /* 恢复 ACK 使能，为下次通信做准备 */
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    return data;
}

/* ============================================================
 * MPU6050 连续读取多个寄存器
 * ------------------------------------------------------------
 * 对于 MPU6050 传感器数据，所有 14 个数据寄存器 (0x3B~0x48)
 * 是连续排列的，可以一次连续读取，大幅提高效率。
 *
 * 流程: START → 设备地址(写) → 起始寄存器地址 →
 *       Repeated START → 设备地址(读) →
 *       读 Byte0(ACK) → 读 Byte1(ACK) → ... → 读 ByteN(NACK) → STOP
 *
 * RegAddr: 起始寄存器地址
 * pBuf:    接收缓冲区指针
 * len:     要读取的字节数
 * ============================================================ */
static void MPU6050_ReadMulti(uint8_t RegAddr, uint8_t *pBuf, uint8_t len)
{
    /* --- 第一阶段: 设置要读取的起始寄存器地址 --- */
    I2C_GenerateSTART(I2C1, ENABLE);
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);

    I2C_Send7bitAddress(I2C1, MPU6050_ADDR_WRITE, I2C_Direction_Transmitter);
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR);

    I2C_SendData(I2C1, RegAddr);
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR);

    /* --- 第二阶段: 切换到读模式，连续接收数据 --- */
    I2C_GenerateSTART(I2C1, ENABLE);
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);

    I2C_Send7bitAddress(I2C1, MPU6050_ADDR_READ, I2C_Direction_Receiver);
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == ERROR);

    /*
     * 连续读取 len 个字节:
     *   前 (len-1) 个字节: 收到后回复 ACK，告诉从机"继续发"
     *   最后 1 个字节:    收到前关闭 ACK，收到后发 STOP
     */

    while (len)
    {
        if (len == 1)
        {
            /* --- 最后一个字节: 关闭 ACK，准备 STOP --- */
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            /* 最后一个字节要发送 STOP，在接收完成前设置 */
        }

        /* 等待数据到达接收寄存器 */
        while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) == ERROR);

        /* 读取数据 */
        *pBuf = I2C_ReceiveData(I2C1);
        pBuf++;
        len--;
    }

    /* 全部数据接收完毕，发送停止信号 */
    I2C_GenerateSTOP(I2C1, ENABLE);

    /* 恢复 ACK 使能 */
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

/* ============================================================
 * MPU6050 初始化
 * ------------------------------------------------------------
 * 1. 初始化 I2C1 硬件外设
 * 2. 唤醒 MPU6050（退出睡眠模式）
 * 3. 配置基本参数（采样率、量程等）
 *
 * 注意: MPU6050 上电后默认处于 SLEEP 模式，
 *       必须先向 PWR_MGMT_1 写入 0x00 才能唤醒！
 * ============================================================ */
void MPU6050_Init(void)
{
    /* 先初始化 I2C 硬件 */
    I2C_MyInit();

    /*
     * 短暂延时，等待 MPU6050 上电稳定
     * MPU6050 上电后需要约 100ms 的稳定时间
     */
    Delay_ms(100);

    /*
     * 【关键步骤】唤醒 MPU6050
     * 上电后 MPU6050 的 PWR_MGMT_1 寄存器默认值为 0x40 (bit6 SLEEP=1)
     * 向该寄存器写入 0x00:
     *   - 清除 SLEEP 位 (bit6=0)，退出睡眠
     *   - 选择内部 8MHz RC 振荡器作为时钟源 (CLKSEL=000)
     */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);

    /* 等待芯片唤醒稳定 */
    Delay_ms(10);

    /*
     * 配置采样率分频器
     * 陀螺仪基准采样率为 1kHz
     * 采样率 = 1kHz / (1 + SMPLRT_DIV)
     * SMPLRT_DIV = 9  →  采样率 = 1kHz / (1+9) = 100Hz
     * 100Hz 对大多数姿态检测应用来说足够了
     */
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);

    /*
     * 配置数字低通滤波器 (DLPF)
     * DLPF_CFG = 0x06 → 带宽约 5Hz
     * 低带宽可以滤除高频噪声，适合姿态检测等慢速应用
     * 也可以设为 0x00 (260Hz带宽) 以获取更快的响应速度
     */
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);

    /*
     * 配置陀螺仪量程
     * FS_SEL = 0x00 << 3 = 0x00 → ±250 °/s
     * 量程对应关系:
     *   0x00 → ±250  °/s  (灵敏度 131 LSB/°/s)
     *   0x08 → ±500  °/s  (灵敏度 65.5 LSB/°/s)
     *   0x10 → ±1000 °/s  (灵敏度 32.8 LSB/°/s)
     *   0x18 → ±2000 °/s  (灵敏度 16.4 LSB/°/s)
     * 小量程 = 高精度，适合慢速运动检测
     */
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00);

    /*
     * 配置加速度计量程
     * AFS_SEL = 0x00 << 3 = 0x00 → ±2g
     * 量程对应关系:
     *   0x00 → ±2g  (灵敏度 16384 LSB/g)
     *   0x08 → ±4g  (灵敏度 8192 LSB/g)
     *   0x10 → ±8g  (灵敏度 4096 LSB/g)
     *   0x18 → ±16g (灵敏度 2048 LSB/g)
     */
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);
}

/* ============================================================
 * 读取 MPU6050 器件 ID (WHO_AM_I)
 * ------------------------------------------------------------
 * WHO_AM_I 寄存器 (0x75) 的默认值应该是 0x68
 * 如果读到的值不是 0x68，说明 I2C 通信异常，
 * 需要检查接线和上拉电阻。
 *
 * 返回值: 8 位器件 ID (正常应为 0x68)
 * ============================================================ */
uint8_t MPU6050_ReadID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/* ============================================================
 * 读取 MPU6050 全部传感器数据
 * ------------------------------------------------------------
 * 一次性读取 14 字节: 6字节加速度 + 2字节温度 + 6字节陀螺仪
 * MPU6050 内部寄存器 0x3B~0x48 是连续排列的，支持突发读取。
 *
 * pData: 指向 MPU6050_Data 结构体的指针，读取结果填入其中
 *
 * 原始值转换为物理量的公式:
 *   加速度 (°/s² 即 g):
 *     Ax = pData->Accel_X / 16384.0f  (量程 ±2g 时)
 *     其他量程: /8192(±4g), /4096(±8g), /2048(±16g)
 *
 *   角速度 (°/s):
 *     Gx = pData->Gyro_X / 131.0f     (量程 ±250°/s 时)
 *     其他量程: /65.5(±500°/s), /32.8(±1000°/s), /16.4(±2000°/s)
 *
 *   温度 (°C):
 *     Temp = pData->Temperature / 340.0f + 36.53f
 * ============================================================ */
void MPU6050_ReadData(MPU6050_Data *pData)
{
    uint8_t buf[14];  // 14 字节缓冲: Accel(6) + Temp(2) + Gyro(6)

    /* 从 ACCEL_XOUT_H (0x3B) 开始连续读取 14 个字节 */
    MPU6050_ReadMulti(MPU6050_ACCEL_XOUT_H, buf, 14);

    /*
     * 数据拼接: MPU6050 输出格式为【高字节在前，低字节在后】(Big Endian)
     * STM32 是小端模式 (Little Endian)，所以要:
     *   高字节 << 8 | 低字节
     * 然后将拼接结果强制转换为有符号 16 位整数 (int16_t)
     */

    /* --- 加速度数据 (每个轴 2 字节, 16位有符号数) --- */
    pData->Accel_X = (int16_t)((buf[0]  << 8) | buf[1]);   // 高字节(0x3B) | 低字节(0x3C)
    pData->Accel_Y = (int16_t)((buf[2]  << 8) | buf[3]);   // 高字节(0x3D) | 低字节(0x3E)
    pData->Accel_Z = (int16_t)((buf[4]  << 8) | buf[5]);   // 高字节(0x3F) | 低字节(0x40)

    /* --- 温度数据 (2 字节) --- */
    pData->Temperature = (int16_t)((buf[6]  << 8) | buf[7]); // 高字节(0x41) | 低字节(0x42)

    /* --- 陀螺仪数据 (每个轴 2 字节, 16位有符号数) --- */
    pData->Gyro_X = (int16_t)((buf[8]  << 8) | buf[9]);    // 高字节(0x43) | 低字节(0x44)
    pData->Gyro_Y = (int16_t)((buf[10] << 8) | buf[11]);   // 高字节(0x45) | 低字节(0x46)
    pData->Gyro_Z = (int16_t)((buf[12] << 8) | buf[13]);   // 高字节(0x47) | 低字节(0x48)
}

/* ============================================================
 * 仅读取加速度数据 (快速版)
 * ------------------------------------------------------------
 * 只读取 0x3B~0x40 共 6 个字节
 * ax, ay, az: 指向加速度三个轴的输出变量
 * ============================================================ */
void MPU6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];

    MPU6050_ReadMulti(MPU6050_ACCEL_XOUT_H, buf, 6);

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
}

/* ============================================================
 * 仅读取陀螺仪数据 (快速版)
 * ------------------------------------------------------------
 * 只读取 0x43~0x48 共 6 个字节
 * gx, gy, gz: 指向陀螺仪三个轴的输出变量
 * ============================================================ */
void MPU6050_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    MPU6050_ReadMulti(MPU6050_GYRO_XOUT_H, buf, 6);

    *gx = (int16_t)((buf[0] << 8) | buf[1]);
    *gy = (int16_t)((buf[2] << 8) | buf[3]);
    *gz = (int16_t)((buf[4] << 8) | buf[5]);
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
 *
 * 【常见问题 & 解决办法】
 *   Q: 静止时角速度不为 0？
 *   A: 用 Calibrate 求零偏，后续减去 → 见 MPU6050_Calibrate
 *
 *   Q: 姿态角不断漂移？
 *   A: 死区 + 互补滤波，加速度计定时修正 → 见 MPU6050_GetAngle
 *
 *   Q: 电机震动后数据乱跳？
 *   A: 低通滤波 α 调小(0.10→0.05) → 见 LPF_ALPHA_GYRO
 *
 *   Q: 刚上电和运行后数据不同？
 *   A: 温漂导致零偏变化，静止时重新校准 → 见 MPU6050_IsStationary
 * =================================================================== */

/* ============================================================
 * 滤波器状态变量 (模块级静态变量，外部不可见)
 * ============================================================ */

/* --- 陀螺仪零偏（开机校准得到） --- */
static int16_t  g_GyroBiasX = 0;           // X 轴零偏
static int16_t  g_GyroBiasY = 0;           // Y 轴零偏
static int16_t  g_GyroBiasZ = 0;           // Z 轴零偏
static uint8_t  g_Calibrated = 0;          // 是否已校准标志

/* --- 低通滤波器历史值 --- */
static float    g_GyroFiltX  = 0;          // 陀螺仪 X 滤波后
static float    g_GyroFiltY  = 0;          // 陀螺仪 Y 滤波后
static float    g_GyroFiltZ  = 0;          // 陀螺仪 Z 滤波后
static float    g_AccelFiltX = 0;          // 加速度 X 滤波后
static float    g_AccelFiltY = 0;          // 加速度 Y 滤波后
static float    g_AccelFiltZ = 0;          // 加速度 Z 滤波后

/* --- 互补滤波器输出的角度 --- */
static float    g_Roll  = 0;               // 横滚角 (°)
static float    g_Pitch = 0;               // 俯仰角 (°)
static float    g_Yaw   = 0;               // 偏航角 (°) — 仅陀螺仪积分

/* --- 静止检测历史值 --- */
static int16_t  g_LastAccelX = 0;          // 上一次加速度 X
static int16_t  g_LastAccelY = 0;          // 上一次加速度 Y
static int16_t  g_LastAccelZ = 0;          // 上一次加速度 Z

/* ============================================================
 * MPU6050 上电静止校准
 * ------------------------------------------------------------
 * 【为什么要校准？】
 *   每一颗 MPU6050 出厂时陀螺仪零偏都不同（工艺差异），
 *   而且焊接应力、PCB 弯曲、温度都会影响零偏。
 *   不校准的话，静止时读到的角速度不是 0，
 *   积分后角度会一直往一个方向漂。
 *
 * 【校准方法】
 *   连续采集 CALIB_SAMPLES 次陀螺仪数据，求平均值作为零偏。
 *   使用前务必保持模块静止不动！
 *
 * 【调用时机】
 *   在 MPU6050_Init() 之后、主循环之前调用一次。
 *   运行中如果检测到静止，也可以再次调用。
 * ============================================================ */
#define CALIB_SAMPLES   200                 // 校准采样次数 (200次 × 10ms = 2秒)

void MPU6050_Calibrate(void)
{
    int32_t sumX = 0, sumY = 0, sumZ = 0;   // 用 32 位累加避免溢出
    uint16_t i;
    MPU6050_Data raw;

    /*
     * 连续采集 CALIB_SAMPLES 次数据并累加
     * 注意: 不在这里用 Delay_ms(10)，而是直接循环读
     *       如果 I2C 是 100kHz，一次读数约 2ms，总共约 400ms
     */
    for (i = 0; i < CALIB_SAMPLES; i++)
    {
        MPU6050_ReadData(&raw);

        sumX += raw.Gyro_X;
        sumY += raw.Gyro_Y;
        sumZ += raw.Gyro_Z;

        /*
         * 短暂延时，让两次采样之间有间隔
         * 这样能覆盖几个振动周期，平均值更准
         */
        Delay_ms(5);
    }

    /* 求平均值，得到零偏 */
    g_GyroBiasX = (int16_t)(sumX / CALIB_SAMPLES);
    g_GyroBiasY = (int16_t)(sumY / CALIB_SAMPLES);
    g_GyroBiasZ = (int16_t)(sumZ / CALIB_SAMPLES);

    /*
     * 初始化低通滤波器状态
     * 第一次滤波时以零偏作为初始值，
     * 避免从 0 跳到真实值产生一个突变
     */
    g_GyroFiltX  = 0;
    g_GyroFiltY  = 0;
    g_GyroFiltZ  = 0;
    g_AccelFiltX = (float)raw.Accel_X;
    g_AccelFiltY = (float)raw.Accel_Y;
    g_AccelFiltZ = (float)raw.Accel_Z;

    /* 初始化姿态角: 用加速度计算出初始 Roll 和 Pitch */
    /*
     * 静止时加速度只受重力影响:
     *   Roll  = atan2(Ay, Az)
     *   Pitch = atan2(-Ax, sqrt(Ay² + Az²))
     *
     * 注意: Yaw 无法由加速度计确定（重力方向不能确定水平朝向），
     *       所以 Yaw 初始化为 0
     */
    {
        float ax = (float)raw.Accel_X;
        float ay = (float)raw.Accel_Y;
        float az = (float)raw.Accel_Z;

        g_Roll  = atan2f(ay, az) * 180.0f / 3.14159265f;
        g_Pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / 3.14159265f;
        g_Yaw   = 0;                     // 偏航角只能从 0 开始
    }

    g_Calibrated = 1;                    // 标记校准完成

    /* 保存当前加速度值用于后续静止检测 */
    g_LastAccelX = raw.Accel_X;
    g_LastAccelY = raw.Accel_Y;
    g_LastAccelZ = raw.Accel_Z;
}

/* ============================================================
 * 获取滤波融合后的姿态角度
 * ------------------------------------------------------------
 * 【处理流程 — 完整的数据清洗管道】
 *
 *   MPU6050 原始数据 (6轴)
 *       │
 *       ├──→ ① 零偏修正    g_corrected = g_raw - g_bias
 *       │                    消除每颗芯片的静态误差
 *       │
 *       ├──→ ② 死区处理    if (|g| < DEADZONE) g = 0
 *       │                    过滤掉传感器自身的微小噪声
 *       │
 *       ├──→ ③ 低通滤波    g_filt = g_filt_old + α × (g - g_filt_old)
 *       │                    平滑化数据，降低震动影响
 *       │
 *       ├──→ ④ 姿态解算    从加速度计求 Roll/Pitch
 *       │    (accel)       Roll_acc  = atan2(Ay, Az)
 *       │                  Pitch_acc = atan2(-Ax, sqrt(Ay²+Az²))
 *       │
 *       ├──→ ⑤ 陀螺仪积分  angle_gyro = angle_old + gyro × dt
 *       │                    注意: 积分会让误差累积，会漂移!
 *       │
 *       └──→ ⑥ 互补滤波    angle = 0.98 × angle_gyro + 0.02 × angle_accel
 *                           陀螺仪: 短期精准，响应快
 *                           加速度计: 长期准确，不会漂→用来修正
 *
 * 【dt 的确定】
 *   主循环延时 100ms，加 I2C 传输约 16ms，实际 dt ≈ 0.116s
 *   用固定的 0.1s 作为近似，误差在接受范围内
 *   如果主循环速度变化较大，应使用定时器精确测量 dt
 *
 * pAngle: 输出的姿态角度指针
 * ============================================================ */
#define DT                  0.10f           // 采样间隔 (秒)，配合 100ms 主循环

void MPU6050_GetAngle(MPU6050_Angle *pAngle)
{
    MPU6050_Data raw;
    float gx, gy, gz;                      // 处理后的陀螺仪 (°/s)
    float ax, ay, az;                      // 处理后的加速度 (LSB)
    float roll_acc, pitch_acc;            // 加速度计算出的角度
    float roll_gyro, pitch_gyro, yaw_gyro;// 陀螺仪积分角度

    if (pAngle == 0) return;                // 空指针保护

    /* ===== 第一步: 读原始数据 ===== */
    MPU6050_ReadData(&raw);

    /* ===== 第二步: 零偏修正 ===== */
    /*
     * 原始值减去校准时得到的零偏
     * 校准后静止状态陀螺仪读数应接近 0
     */
    gx = (float)(raw.Gyro_X - g_GyroBiasX);
    gy = (float)(raw.Gyro_Y - g_GyroBiasY);
    gz = (float)(raw.Gyro_Z - g_GyroBiasZ);

    ax = (float)raw.Accel_X;
    ay = (float)raw.Accel_Y;
    az = (float)raw.Accel_Z;

    /* ===== 第三步: 死区处理 ===== */
    /*
     * 角速度的绝对值很小时，大概率是噪声而非真实转动
     * 直接置零防止噪声不断积分为角度误差
     *
     * X/Y 轴有加速度计互补修正，死区可以设小
     * Z 轴 (Yaw) 没有加速度计修正，死区必须设大，否则残余零偏
     * 会一直积分 → 静止时 Yaw 永远在漂
     */
    if (gx < GYRO_DEADZONE     && gx > -GYRO_DEADZONE)     gx = 0;
    if (gy < GYRO_DEADZONE     && gy > -GYRO_DEADZONE)     gy = 0;
    if (gz < GYRO_DEADZONE_YAW && gz > -GYRO_DEADZONE_YAW) gz = 0;

    /* ===== 第四步: 低通滤波 (指数移动平均) ===== */
    /*
     * filtered = filtered_old + α × (raw - filtered_old)
     *
     * α 越大 = 滤波越弱, 响应越快, 噪声更多
     * α 越小 = 滤波越强, 响应越慢, 噪声更少
     *
     * 首次调用时 (g_Calibrated=1 且 g_GyroFiltX=0)，直接赋初值避免突变
     */
    g_GyroFiltX  += LPF_ALPHA_GYRO  * (gx - g_GyroFiltX);
    g_GyroFiltY  += LPF_ALPHA_GYRO  * (gy - g_GyroFiltY);
    g_GyroFiltZ  += LPF_ALPHA_GYRO  * (gz - g_GyroFiltZ);

    g_AccelFiltX += LPF_ALPHA_ACCEL * (ax - g_AccelFiltX);
    g_AccelFiltY += LPF_ALPHA_ACCEL * (ay - g_AccelFiltY);
    g_AccelFiltZ += LPF_ALPHA_ACCEL * (az - g_AccelFiltZ);

    /* ===== 第五步: 陀螺仪 LSB → °/s 转换并积分 ===== */
    /*
     * 灵敏度 131 LSB/°/s @ ±250°/s 量程
     * 角速度 (°/s) = 滤波后的陀螺仪值 / 131
     * 角度增量 (°)  = 角速度 × dt
     */
    roll_gyro  = g_Roll  + (g_GyroFiltX / 131.0f) * DT;
    pitch_gyro = g_Pitch + (g_GyroFiltY / 131.0f) * DT;
    yaw_gyro   = g_Yaw   + (g_GyroFiltZ / 131.0f) * DT;

    /* ===== 第六步: 加速度计求 Roll 和 Pitch ===== */
    /*
     * 重力在加速度计三轴上的投影 → 反三角函数求角度
     *
     *   静止水平放置: Ax≈0, Ay≈0, Az≈+1g
     *   roll  = atan2(Ay, Az)        → 此时 ≈ 0°
     *   pitch = atan2(-Ax, √(Ay²+Az²)) → 此时 ≈ 0°
     *
     *   注意: 运动加速度会干扰这个计算（所以不能直接用，要互补滤波修正）
     *   注意: Yaw 不能由加速度计得出（绕重力方向的旋转没有投影变化）
     */
    roll_acc  = atan2f(g_AccelFiltY, g_AccelFiltZ) * 180.0f / 3.14159265f;
    pitch_acc = atan2f(-g_AccelFiltX,
                       sqrtf(g_AccelFiltY * g_AccelFiltY +
                             g_AccelFiltZ * g_AccelFiltZ))
                * 180.0f / 3.14159265f;

    /* ===== 第七步: 自适应互补滤波融合 ===== */
    /*
     * 【为什么不固定 98:2？】
     *   固定比例有个死穴: 急停后陀螺仪的滞后残余一直在积分，
     *   加速度计每周期只修正 2%，要 5 秒才能拽回来。
     *   连续急停几次，误差叠上去 → 角度回不到零 → 你看到的问题。
     *
     * 【自适应策略】
     *   根据当前角速度大小动态调整权重:
     *
     *     |角速度| > 5°/s  (运动): 信陀螺 98%, 加速度 2%
     *       → 跟手, 不受运动加速度干扰, 响应快
     *
     *     |角速度| < 5°/s  (静止): 信陀螺 90%, 加速度 10%
     *       → 加速修正, 急停后 ~1s 收敛到真实角度
     *         不再累积误差, 不再看到 "0.1 0.1 地变"
     *
     *   5 °/s 的判断阈值 (131 LSB/°/s × 5 = 655 LSB):
     *     手动晃动模块远超此值, 静止/慢飘远低于此值
     *     刚好在 "真在动" 和 "传感器残余噪声" 之间切开
     */
    {
        float trust_roll, trust_pitch;

        /* Roll: 判断 X 轴角速度 */
        trust_roll  = (g_GyroFiltX >  5.0f * 131.0f || g_GyroFiltX < -5.0f * 131.0f)
                      ? COMP_GYRO_TRUST : 0.85f;  // 静止时 15% 修正 (原来 10%)

        /* Pitch: 判断 Y 轴角速度 */
        trust_pitch = (g_GyroFiltY >  5.0f * 131.0f || g_GyroFiltY < -5.0f * 131.0f)
                      ? COMP_GYRO_TRUST : 0.85f;

        g_Roll  = trust_roll  * roll_gyro  + (1.0f - trust_roll)  * roll_acc;
        g_Pitch = trust_pitch * pitch_gyro + (1.0f - trust_pitch) * pitch_acc;
    }
    g_Yaw = yaw_gyro;                    // Yaw 无法用加速度修正，会一直漂

    /* ===== 第八步: 输出角度 ===== */
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
 * 【判断方法】
 *   比较当前加速度与上一次加速度的差值，
 *   如果三个轴的差值都很小，认为设备是静止的。
 *
 *   这个判断不依赖陀螺仪（陀螺仪可能有零偏误差），
 *   而是依赖加速度计——静止时加速度只反映重力方向，非常稳定。
 *
 * 【用途】
 *   当设备持续静止时，可以重新校准陀螺仪零偏，
 *   减轻温漂带来的角度漂移。
 *
 *   用法示例:
 *     static uint16_t stillCount = 0;
 *     if (MPU6050_IsStationary()) {
 *         stillCount++;
 *         if (stillCount > 50) {  // 静止 5 秒 (50 × 100ms)
 *             MPU6050_Calibrate();
 *             stillCount = 0;
 *         }
 *     } else {
 *         stillCount = 0;
 *     }
 *
 * 返回值: 1 = 静止, 0 = 运动中
 * ============================================================ */
uint8_t MPU6050_IsStationary(void)
{
    MPU6050_Data raw;
    int16_t dx, dy, dz;

    MPU6050_ReadData(&raw);

    /* 计算加速度变化量（绝对值） */
    dx = raw.Accel_X - g_LastAccelX;
    dy = raw.Accel_Y - g_LastAccelY;
    dz = raw.Accel_Z - g_LastAccelZ;

    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    if (dz < 0) dz = -dz;

    /* 三个轴的变化量都小于阈值 → 静止 */
    if (dx < STATIONARY_THRESH && dy < STATIONARY_THRESH && dz < STATIONARY_THRESH)
    {
        return 1;
    }
    return 0;
}
