# 循迹小车系统设计文档

> **MCU**: TI MSPM0G3507 (Cortex-M0+, 64MHz, 无硬件 FPU)  
> **电机驱动**: TB6612FNG (双 H 桥)  
> **编码器**: 霍尔编码器, 13 PPR, 1:20 减速比  
> **传感器**: 八路灰度 (I2C 通信, 地址 0x12)  
> **开发环境**: TI Code Composer Studio + SysConfig  

---

## 目录

1. [系统架构](#1-系统架构)
2. [引脚分配](#2-引脚分配)
3. [时钟配置](#3-时钟配置)
4. [模块详解](#4-模块详解)
   - [4.1 电机控制 (motor_control)](#41-电机控制)
   - [4.2 PID 控制器 (pid_controller)](#42-pid-控制器)
   - [4.3 速度测量 (speed_sensor)](#43-速度测量)
   - [4.4 灰度循迹 (grayscale)](#44-灰度循迹)
   - [4.5 软件 PWM (SysTick)](#45-软件-pwm)
   - [4.6 串口调试 (uart)](#46-串口调试)
   - [4.7 延时 (delay)](#47-延时)
5. [主程序流程](#5-主程序流程)
6. [循迹控制算法](#6-循迹控制算法)
7. [已知问题与陷阱](#7-已知问题与陷阱)
8. [调参指南](#8-调参指南)

---

## 1. 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    主控制循环 (2ms / 500Hz)                       │
│                                                                  │
│  ┌──────────┐    ┌──────────┐    ┌──────────────┐               │
│  │ 灰度读取  │───▶│ 误差计算  │───▶│ 模式状态机    │               │
│  │ 8路 I2C   │    │ 加权平均  │    │ NORMAL/LOST  │               │
│  └──────────┘    └──────────┘    └──────┬───────┘               │
│                          ┌─────────────┼─────────────┐           │
│                          ▼             ▼             ▼           │
│                    ┌──────────┐              ┌──────────┐       │
│                    │ NORMAL   │              │ LOST     │       │
│                    │ 差速分级  │              │ 保持方向  │       │
│                    │ 直道+弯道 │              │ 自动找回  │       │
│                    └────┬─────┘              └────┬─────┘       │
│                         │                         │             │
│                         └──────────┬──────────────┘             │
│                                    ▼                            │
│                           ┌─────────────────┐                   │
│                           │ 限幅 [0, 1.0]   │                   │
│                           │ g_pwm_a / g_pwm_b│                  │
│                           └───────┬─────────┘                   │
│                                   ▼                             │
│                           ┌─────────────────┐                   │
│                           │ SysTick ISR     │                   │
│                           │ GPIO 软件 PWM   │                   │
│                           └───────┬─────────┘                   │
│                                   │ PA12/PB6                    │
│                                   ▼                             │
│                           ┌─────────────────┐                   │
│                           │ TB6612 → 电机   │                   │
│                           └─────────────────┘                   │
│                                                                  │
│   编码器: 计距(圈数控制+固定补偿) + 速度PID(慢速层稳速)              │
│   停止: 到达目标脉冲后 TB6612 短刹车 (AIN1=AIN2=HIGH)               │
└─────────────────────────────────────────────────────────────────┘
```

### 控制周期

| 环节 | 周期 | 说明 |
|---|---|---|
| 主控制循环 | 2ms (500Hz) | 灰度读取 → 误差计算 → 模式判断 → 差速 → PWM |
| 丢线检测 | 2ms (500Hz) | 全白或全黑立即进入 LOST, 不遗漏过渡态 |
| 速度 PID | 50ms (20Hz) | 只在 NORMAL 且 \|error\|<0.2 时更新, 调节 BASE_DUTY |
| PWM 输出 | 10μs (100kHz) | SysTick 中断驱动, 100 级分辨率, PWM=1kHz |

---

## 2. 引脚分配

### MCU → 电机驱动 (TB6612)

| 功能 | MSPM0 引脚 | TB6612 引脚 | 方向 | 说明 |
|---|---|---|---|---|
| STBY | **PA24** | STBY | OUT | 高电平使能, 初始化后始终拉高 |
| L-AIN1 | **PA8** | AIN1 | OUT | 方向控制 (初始 LOW) |
| L-AIN2 | **PA9** | AIN2 | OUT | 方向控制 (初始 LOW) |
| **L-PWM** | **PA12** | PWMA | OUT | 速度控制, GPIO 软件 PWM |
| R-BIN1 | **PB4** | BIN1 | OUT | 方向控制 (初始 LOW) |
| R-BIN2 | **PB5** | BIN2 | OUT | 方向控制 (初始 LOW) |
| **R-PWM** | **PB6** | PWMB | OUT | 速度控制, GPIO 软件 PWM |

**TB6612 真值表 (STBY=HIGH)**:

| IN1 | IN2 | PWM | 模式 |
|---|---|---|---|
| 0 | 0 | X | 停止 (高阻) |
| 0 | 1 | 1 | 反转 |
| 0 | 1 | 0 | 短路刹车 |
| 1 | 0 | 1 | 正转 |
| 1 | 0 | 0 | 短路刹车 |
| 1 | 1 | X | 短路刹车 |

### MCU → 编码器

| 功能 | MSPM0 引脚 | 配置 | 中断 |
|---|---|---|---|
| L-E1A (A相) | **PA17** | INPUT, **PULL_UP** | RISE, GROUP1_GPIOA |
| L-E1B (B相) | **PA21** | INPUT, **PULL_UP** | RISE, GROUP1_GPIOA |
| R-E2A (A相) | **PB0** | INPUT, **PULL_UP** | RISE, GROUP1_GPIOB |
| R-E2B (B相) | **PB1** | INPUT, **PULL_UP** | RISE, GROUP1_GPIOB |

> ⚠️ **重要**: 编码器通常为 NPN 开漏输出, **必须配置 PULL_UP**。  
> SysConfig 和手动 IOMUX 写 (`0x00010000U`) 均需设为上拉。PULL_DOWN 会导致编码器无信号。

### MCU → 灰度传感器 (I2C)

| 功能 | MSPM0 引脚 | 配置 |
|---|---|---|
| I2C SDA | **PA10** | I2C1, PULL_UP (与 OLED 共用) |
| I2C SCL | **PA11** | I2C1, PULL_UP (与 OLED 共用) |

> 八路灰度模块 I2C 地址: **0x12**。寄存器 0x30 返回 1 字节传感器状态 (bit0=X1...bit7=X8)，1=检测到黑线。寄存器 0x01 控制校准。

### 其他

| 功能 | MSPM0 引脚 | 说明 |
|---|---|---|
| LED | **PB14** | 心跳指示, 1.25Hz 闪烁 |
| UART TX | **PA28** | 调试输出, 115200 baud |
| I2C SDA | **PA10** | OLED + 灰度传感器 (共享 I2C1) |
| I2C SCL | **PA11** | OLED + 灰度传感器 (共享 I2C1) |

---

## 3. 时钟配置

```
内部 32MHz 振荡器 (SYSOSC)
    │
    ▼
PLL ×4 = 128MHz
    │
    ▼
UDIV ÷2 = 64MHz  ← CPUCLK (系统时钟)
    │
    ├── TIMG0 (PWMA): BUSCLK ÷2 = 32MHz (未使用定时器模式)
    ├── TIMG6 (PWMB): BUSCLK ÷2 = 32MHz (未使用定时器模式)
    └── SysTick: 直接使用 CPUCLK = 64MHz
```

SysTick 用于 GPIO 软件 PWM:
- `SysTick->LOAD = 64000000 / 100000 - 1 = 639`
- 中断频率: 100kHz
- PWM 分辨率: 100 级 (1%), PWM 输出频率 = 1kHz
- PWM 输出频率: 200Hz

> **时钟源**: 使用内部 SYSOSC (32MHz), 不依赖外部晶振, 所有开发板通用。
> 如需切换到外部 HFXT 晶振, 在 SysConfig 中配置 HFXT 并将 `forceDefaultClkConfig` 设为 `false`。

---

## 4. 模块详解

### 4.1 电机控制

**文件**: [`motor_control.h`](motor_control.h), [`motor_control.c`](motor_control.c)

#### 数据结构

```c
typedef struct {
    PIDController pid;          // 每个电机独立的 PID 控制器
    float target_speed;         // 目标速度 [0, 1.0]
    float measured_speed;       // 实测速度 [0, 1.0]
    float duty_cycle;           // PID 输出占空比 [0, 1.0]
    uint8_t direction;          // MOTOR_DIR_STOP/FORWARD/REVERSE

    void    *gpio_port;         // AIN1/AIN2 所在 GPIO 端口
    uint32_t ain1_pin;          // AIN1 引脚掩码
    uint32_t ain2_pin;          // AIN2 引脚掩码
} MotorControl;
```

#### 接口

| 函数 | 说明 |
|---|---|
| `Motor_Init(motor, port, ain1, ain2)` | 初始化, 绑定 GPIO, 设 STOP |
| `Motor_SetDirection(motor, dir)` | 切换方向 (同时操作 GPIO) |
| `Motor_SetTargetSpeed(motor, speed)` | 更新目标速度 (不操作硬件) |
| `Motor_Update(motor, measured_speed)` | 运行 PID, 更新 `duty_cycle` |
| `Motor_Stop(motor)` | 紧急停止, 方向→STOP, 占空比→0 |

#### 方向控制

```c
// motor_control.c Motor_SetGPIO()
FORWARD:  IN1=1, IN2=0  → 电机正转
REVERSE:  IN1=0, IN2=1  → 电机反转
STOP:     IN1=0, IN2=0  → TB6612 高阻停止
```

#### PWM 输出

PWM 不在此模块中直接操作硬件。`Motor_Update` 计算 `duty_cycle` 后, 由 `empty.c` 主循环将其转换为 SysTick ISR 使用的阈值:

```c
// empty.c 主循环中:
g_pwm_a = (uint8_t)(motor1.duty_cycle * 100.0f);
// SysTick ISR 根据 g_pwm_a 控制 GPIO PA12
```

**为什么不使用定时器 CCP 输出?**

TIMG0/TIMG6 的 CCP0 外设输出在 PA12/PB6 上不工作 (所有寄存器配置正确, 但引脚不受定时器控制)。根因可能是 MSPM0G3507 的芯片勘误或 SysConfig 生成的 PinMux 配置问题。解决方案是使用 SysTick + GPIO 软件 PWM, 效果相同。

### 4.2 PID 控制器

**文件**: [`pid_controller.h`](pid_controller.h), [`pid_controller.c`](pid_controller.c)

#### 算法: 位置式 PID + 抗积分饱和

```c
// pid_controller.c PID_Update()
float error = setpoint - measured_value;

// P (比例)
float p_term = kp * error;

// I (积分, 带 clamping 抗饱和)
integral += error * dt;
float i_term = ki * integral;

// D (微分, 在测量值上做微分, 避免 setpoint 突变冲击)
float derivative = -(measured_value - prev_measured) / dt;
float d_term = kd * derivative;

// 合成
float output = p_term + i_term + d_term;

// 输出限幅 + 积分退饱和 (clamping)
if (output > out_max) {
    if (error > 0) integral -= error * dt;  // 正向饱和, 退积分
    output = out_max;
} else if (output < out_min) {
    if (error < 0) integral -= error * dt;  // 负向饱和, 退积分
    output = out_min;
}
```

#### 当前参数

| 参数 | 值 | 说明 |
|---|---|---|
| Kp | **0.35** | 比例增益 |
| Ki | **0.15** | 积分增益 |
| Kd | **0.0** | 微分增益 (当前未使用) |
| dt | **0.005** | 控制周期 5ms |
| out_min | **0.0** | 输出下限 (0% 占空比) |
| out_max | **1.0** | 输出上限 (100% 占空比) |

#### 设计要点

- **微分在测量值上做** (derivative on measurement): 避免目标速度突变时产生微分冲击
- **积分 clamping**: 输出饱和时自动退积分, 防止积分深饱和导致系统失控
- **输出限幅 [0, 1]**: 直接对应 PWM 占空比

### 4.3 速度测量

**文件**: [`speed_sensor.h`](speed_sensor.h), [`speed_sensor.c`](speed_sensor.c)

#### 硬件参数

```c
#define ENCODER_PPR        13       // 编码器每转脉冲数
#define GEAR_RATIO         20       // 减速比 1:20
#define OUTPUT_PPR         (13*20)  // 输出轴每转 260 脉冲
#define MOTOR_MAX_RPM      8000.0f  // 归一化用的最大转速
```

#### 测量方法: 固定窗口脉冲计数

```
每 50ms (10 个主循环周期) 统计一次编码器脉冲数
  │
  ├── RPM = (pulses / 0.1s) × 60 / 13PPR
  │
  ├── 噪声过滤: RPM > 15000 → 丢弃, 沿用上次有效值
  │
  └── 归一化: speed = RPM / 8000, 限幅到 [0, 1]
```

#### 方向判断

```c
// ISR 中: 读 B 相电平判断方向
if (DL_GPIO_readPins(..., EAB_PIN) != 0)
    dir = FORWARD;   // A 相上升沿时 B 相为高 → 正转
else
    dir = REVERSE;   // A 相上升沿时 B 相为低 → 反转
```

#### ISR 架构

```
GROUP1_IRQHandler
    ├── GPIOA 中断 → EAA / EAB
    │   ├── EAA: a_pulse_count++, a_total++, 读 EAB 判向
    │   └── EAB: 保留 (不计数, 仅清中断标志)
    └── GPIOB 中断 → EBA / EBB
        ├── EBA: b_pulse_count++, b_total++, 读 EBB 判向
        └── EBB: 保留 (不计数, 仅清中断标志)
```

> **注意**: 当前只用了 A 相上升沿做单相计数 (13 PPR 有效)。如需倍频 (26 PPR 或 52 PPR), 在 EAB/EDB ISR 中也做脉冲累加。

#### 临界区保护

主循环用 `__disable_irq()` / `__enable_irq()` 保护脉冲计数读取:

```c
__disable_irq();
pulses = *pc;
*pc    = 0;       // 清零窗口计数器
__enable_irq();
```

丢失的脉冲数 < 1 (ISR 被屏蔽的时间极短), 可忽略。

### 4.4 灰度循迹

**文件**: [`grayscale.h`](grayscale.h), [`grayscale.c`](grayscale.c)

#### 传感器布局 (俯视, 车头朝前)

```
[X1] [X2] [X3] [X4] [X5] [X6] [X7] [X8]
 -7   -5   -3   -1   +1   +3   +5   +7   ← 权重
```

#### I2C 通信

- **设备地址**: 0x12 (与 OLED 0x3C 共享 I2C1 总线 PA10/PA11)
- **寄存器 0x30** (只读): 1 字节, bit0=X1 ... bit7=X8, 1=检测到黑线
- **寄存器 0x01** (只写): 1=进入校准, 0=退出校准
- **初始化**: `Grayscale_Init()` 自动触发校准流程 (写 0x01=1 → 等待 300ms → 写 0x01=0)

#### 误差计算: 加权平均法 (8路)

```c
// grayscale.c Grayscale_ComputeError()
static const int8_t weight[8] = { -7, -5, -3, -1, 1, 3, 5, 7 };
// 对应索引:                     X1  X2  X3  X4 X5 X6 X7 X8

// 加权和除以(在线传感器数 × 7)
float error = sum_weighted / (sum_active * 7);
// 输出范围: [-1.0, +1.0]
//   负值 = 线偏左 (X1~X4 检测到线)
//   正值 = 线偏右 (X5~X8 检测到线)
//   0    = 居中
```

#### 辅助函数

| 函数 | 说明 |
|---|---|
| `Grayscale_IsLineLost()` | 8路全为0 → 完全脱线 |
| `Grayscale_IsSharpTurn()` | 仅 X1 或 X8 触发 → 直角弯 |
| `Grayscale_OnLineCount()` | 当前在线上的传感器数量 |

### 4.5 软件 PWM

**实现位置**: [`empty.c`](empty.c) 的 `SysTick_Handler` ISR + 主循环

#### 为什么不用定时器 CCP

TIMG0 (PA12) 和 TIMG6 (PB6) 的 CCP 外设输出在本项目中不工作。经反复验证 (寄存器 CC 读回正确, 输出极性配置正确, GPIO 模式可正常控制), 结论是定时器外设输出无法驱动这两个引脚。

#### SysTick 软件 PWM 方案

```
配置:
  SysTick 中断频率 = 100kHz (每 10μs 一次)
  PWM 分辨率 = 100 级 (1% 步长)
  PWM 输出频率 = 1kHz

ISR 逻辑:
  cnt 每 10μs 自增 (0→99 循环)
  Motor 1: cnt < g_pwm_a → PA12 HIGH, else LOW
  Motor 2: cnt < g_pwm_b → PB6 HIGH, else LOW

主循环更新:
  每 2ms 将循迹计算结果写入 g_pwm_a / g_pwm_b
  每个变量 8 位, 32 位 CPU 上原子读写
```

```
100kHz 中断周期 (10μs per tick)
  │
  ├── tick 0:  cnt=0   (cnt < threshold → HIGH)
  ├── tick 1:  cnt=1
  ├── ...
  ├── tick 50: cnt=50  (cnt >= threshold → LOW, 若 duty=50%)
  ├── ...
  └── tick 99: cnt=99  (cnt >= threshold → LOW)
      └── cnt 归零, 重复
```

#### SysTick 初始化

```c
// empty.c
SysTick->LOAD = (CPUCLK_FREQ / 100000) - 1;  // 64M/100k - 1 = 639
SysTick->VAL  = 0;
SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |   // CPUCLK
                SysTick_CTRL_TICKINT_Msk   |   // 使能中断
                SysTick_CTRL_ENABLE_Msk;       // 使能定时器
NVIC_SetPriority(SysTick_IRQn, 0xFF);          // 最低优先级
```

> **优先级设计**: SysTick 设为最低优先级, 确保编码器 GPIO 中断可以抢占 SysTick, 脉冲计数不丢。

#### 优缺点

| 优点 | 缺点 |
|---|---|
| 绕过了 TIMG CCP 硬件 Bug | 100kHz ISR 消耗 ~5% CPU |
| GPIO 直接控制, 行为可预测 | CPU 睡眠时 PWM 停止 |
| 两个电机独立 PWM, 1% 精度 | 中断抖动 <100ns, 可忽略 |

### 4.6 串口调试

**文件**: [`uart.h`](uart.h), [`uart.c`](uart.c)

- **波特率**: 115200, 8N1
- **TX**: PA28 (阻塞发送, `DL_UART_transmitDataBlocking`)
- **RX**: 中断 + 256 字节环形缓冲区 (当前未使用 RX 功能)
- **printf**: `UART_Printf()` 使用栈上 128 字节缓冲区 + `vsnprintf`

#### 调试输出格式 (每秒一次, v3)

```
GS:11011000 err=+0.00 | out:L= 22 R= 22 | rpm:L=3692 R=3508 NORM
│       │              │       │         │       │       │
│       │              │       │         │       │       └─ 模式 (NORM/LOST)
│       │              │       │         │       └─ 右轮 RPM
│       │              │       │         └─ 左轮 RPM
│       │              │       └─ 右轮占空比 (%)
│       │              └─ 左轮占空比 (%)
│       └─ 位置误差 [-1.00, +1.00]
└─ 8路灰度值 (X1 X2 X3 X4 X5 X6 X7 X8, 1=在线上)
```

### 4.7 延时

**文件**: [`delay.h`](delay.h), [`delay.c`](delay.c)

```c
void delay_ms(uint32_t ms) {
    uint32_t cycles = (CPUCLK_FREQ / 1000U) * ms;
    delay_cycles(cycles);  // CPU 忙等待
}
```

> ⚠️ 这是总线等待 (busy-wait), CPU 不会进入低功耗模式。如果需要省电, 可改用定时器 + `__WFI()` 休眠方式。

---

## 5. 主程序流程 (v4 — 双层控制)

```c
// empty.c main()
main() {
    SYSCFG_DL_init();           // SysConfig 生成的硬件初始化
    SpeedSensorA_Init();        // 编码器 A 状态归零
    SpeedSensorB_Init();        // 编码器 B 状态归零

    // --- GPIO 初始化 ---
    STBY 拉高;                  // 使能电机驱动
    AIN1/AIN2 驱动强度提高;      // 确保方向引脚能驱动 TB6612
    编码器引脚 PULL_UP;         // NPN 开漏编码器必须上拉
    NVIC 使能编码器中断;
    PWM引脚 切为 GPIO 输出 LOW; // 绕开 TIMG CCP Bug
    SysTick 100kHz 初始化;      // 软件 PWM 时钟源

    // --- 模块初始化 ---
    Motor_Init(&motor1, PA8, PA9);   // 左电机
    Motor_SetDirection(&motor1, REVERSE);
    Motor_Init(&motor2, PB4, PB5);   // 右电机
    Motor_SetDirection(&motor2, REVERSE);
    Grayscale_Init();  /* 八路 I2C 灰度, 地址 0x12 */
    PID_Init(&speed_pid, 0.005, 0.002, 0, 0.05, 0.10, 0.65);

    FollowMode mode = MODE_NORMAL;

    // --- 主循环 (500Hz) ---
    while (1) {
        delay_ms(2);            // 2ms 控制周期
        tick++;

        // 心跳 LED
        if (tick % 200 == 0) LED_TOGGLE;

        // 1. 读取灰度传感器
        Grayscale_Read(&gs);
        error = Grayscale_ComputeError(&gs) * LINE_POLARITY;

        // 2. 模式状态机
        switch (mode) {
        case MODE_NORMAL:
            if (!line_lost) last_error = error;  // 在线时持续更新
            if (line_lost || all_black) {
                mode = MODE_LOST;                // 丢线, 保持方向
                lost_timer = 0;
            } else {
                // 正常差速分级
                inner_ratio = f(|error|);
                left  = error>0 ? BASE : BASE * inner_ratio;
                right = error>0 ? BASE * inner_ratio : BASE;
            }
            break;

        case MODE_LOST:
            lost_timer++;
            if (!line_lost && !all_black) {
                mode = MODE_NORMAL;              // 找回线
            } else if (lost_timer > 2s) {
                直行;                            // 超时保护
            } else {
                // last_error 方向, 外轮 LOST_DUTY, 内轮停
                left  = last_error>0 ? LOST_DUTY : 0;
                right = last_error>0 ? 0 : LOST_DUTY;
            }
            break;
        }

        // 3. 慢速层: 只在直行小误差时调速
        if (mode == NORMAL && |error| < 0.2)
            每50ms跑一次速度PID, 更新BASE;

        // 4. 输出 PWM
        g_pwm_a = (uint8_t)(left_duty * 100);
        g_pwm_b = (uint8_t)(right_duty * 100);

        // 5. 串口调试 (每秒)
        if (tick % 500 == 0) UART_Printf(...);
    }
}
```

---

## 6. 循迹控制算法 (v4 — 双层控制)

### 架构演进

| 版本 | 架构 | 问题 |
|---|---|---|
| v1 | 差速 → 速度 PID → 占空比 | 编码器 100ms 滞后, 直角弯来不及反应 |
| v2 | 位置 PID → 占空比 | PID 离散传感器参数难调, 直线蛇形 |
| v3 | 差速分级 + 直角弯专用 pivot | 直角弯触发条件脆弱, 固定脉冲数泛用性差 |
| **v4** | **差速分级 + 丢线保持** | 当前版本 |

v4 核心思想: **不显式检测直角弯。靠近弯道时误差自然增大→差速开始转弯→短暂丢线→LOST 保持方向→线出现自动回到 NORMAL。** 整个过程由传感器天然驱动, 不分弯道类型。

### 模式状态机

```
                    ┌─────────────┐
          ┌────────▶│ MODE_NORMAL │◀────────┐
          │         │ 八路权值差速  │         │
          │         └──────┬──────┘         │
          │                │                │
          │        丢线/全黑 │      找回线    │
          │                │                │
          │         ┌──────▼──────┐         │
          └─────────│ MODE_LOST   │─────────┘
                    │ last_error  │
                    │ 保持方向     │
                    └─────────────┘
```

### MODE_NORMAL — 差速分级

和 v3 思路相同, 八路加权误差直接查表得差速比:

```c
float abs_error = fabsf(error);

if      (abs_error < 0.05)  inner_ratio = 1.0;  // 居中 → 两轮同速
else if (abs_error < 0.20)  inner_ratio = 0.5;  // 微偏 → 内轮半速
else if (abs_error < 0.50)  inner_ratio = 0.2;  // 偏转 → 内轮 1/5 速
else                        inner_ratio = 0.0;  // 急弯 → 内轮停转
```

在线时持续更新 `last_error`, 作为丢线后的方向参考。

### MODE_LOST — 丢线保持

不显式区分"直角弯丢线"和"脱线"。统一策略: **用 last_error 方向继续转, 直到线回来**。

```
全白或全黑 → MODE_LOST
    │
    ├─ last_error > 0 → 左轮 LOST_DUTY, 右轮停 (右转找线)
    ├─ last_error < 0 → 左轮停, 右轮 LOST_DUTY (左转找线)
    │
    ├─ 找回线 → MODE_NORMAL
    └─ 超时 2s → 强制直行
```

### 速度 PID (慢速层)

只在 NORMAL 且 |error| < 0.2 时更新, 转弯和丢线期间冻结:

```
每50ms: 编码器脉冲增量 → 速度PID → 调整 BASE_DUTY
输出限幅: [0.10, 0.65]
```

### 参数总览

| 参数 | 默认值 | 说明 |
|---|---|---|
| `BASE_DUTY_NOMINAL` | **0.35** | 巡航占空比初始值, 速度 PID 动态调节 |
| `LINE_POLARITY` | **1.0** | 转向极性 |
| `LOST_OUTER_DUTY` | **0.30** | 丢线时外轮占空比 |
| `LOST_MAX_TICKS` | **1000** | 丢线超时 2s, 强制直行 |
| `SPEED_TARGET_PPS` | **50** | 目标速度, 每 50ms 窗口 50 脉冲 |
| `SPEED_WINDOW_TICKS` | **25** | 调速周期 25×2ms = 50ms |
| `PULSES_PER_LAP` | **5517** | 每圈目标脉冲数 |
| `STOP_OFFSET_PULSES` | **108** | 短刹车固定补偿 |

---

## 7. 已知问题与陷阱

### 7.1 v1 架构: 速度 PID 导致直角弯反应慢 (v2 修复)

**现象**: 直线循迹正常, 但直角弯完全无法通过。

**根因**: v1 "差速 → 目标速度 → 速度 PID → 占空比" 存在编码器 100ms 滞后。

**修复 (v2)**: 改为直接控制占空比, 绕过速度 PID。

### 7.1b v2 架构: 位置 PID 导致直线蛇形 (v3 修复)

**现象**: 位置 PID 在直线段左右摇摆, 参数无法同时兼顾直道稳定性和弯道响应。

**根因**: 5 路离散传感器, 误差量化跳变大 (0→0.25→0.50)。PID 的 Kd 项对离散跳变过度响应, 而 Kp 调低后缓弯又不够灵敏。

**修复 (v3)**: 直道改用原始差速分级策略, 直角弯用专用 pivot 模式。两种场景各自独立控制, 不再需要一组参数兼顾。

### 7.1c 直角弯检测方案 (v4 废弃)

v3 的直角弯显式检测 (M+R2 / M+L2 触发) 过于脆弱——线宽、车速、传感器安装位置都影响触发成功率, 且固定脉冲数无法适应不同弯角。v4 改为丢线保持方案, 不再显式检测直角弯。

### 7.2 TIMG CCP 输出不工作

**现象**: TIMG0/TIMG6 的 CCP0 外设输出无法驱动 PA12/PB6。寄存器配置正确 (CC 值可读回), 但引脚电平不受定时器控制。

**解决方案**: 使用 SysTick 中断 + GPIO 软件 PWM 替代。

**影响**: 100kHz ISR 消耗约 5% CPU, 电机不能运行时 CPU 不能进入深度睡眠 (不影响本项目需求)。

### 7.3 IOMUX 寄存器写反逻辑 Bug

**现象**: 代码注释写"编码器上拉", 但寄存器写入了 `0x00020000U` (下拉)。

**原因**: MSPM0 IOMUX PINCM 寄存器:
- `0x00010000U` = 上拉 (PU)
- `0x00020000U` = 下拉 (PD)

原代码 `(x & ~0x00010000U) | 0x00020000U` 清除了上拉位, 设置了下拉位。

**修复**: 改为 `(x & ~0x00030000U) | 0x00010000U` (清除所有 Pull 位, 设置上拉)。

### 7.4 SysConfig pull 配置不一致

**现象**: SysConfig 中 EAA/EAB 配为 PULL_UP, 但 EBA/EBB 误配为 PULL_DOWN。

**修复**: 统一所有编码器引脚为 PULL_UP。NPN 开漏编码器必须上拉才能工作。

### 7.5 PWM 比较值等于 Period 时硬件可能异常

**现象**: 定时器 CC 值 = Period 时, 某些硬件可能拒绝写入或行为异常。

**修复**: 钳位逻辑从 `if (CC > period) CC = period` 改为 `if (CC >= period) CC = period - 1`, 确保 CC 始终在有效范围 [0, period-1] 内。

### 7.6 浮点运算在 Cortex-M0+ 上无硬件支持

MSPM0G3507 是 Cortex-M0+ 核心, **没有硬件 FPU**。所有 `float` 运算由编译器软件模拟, 比整数运算慢 10-50 倍。

**当前影响**: 控制周期 5ms 足够宽裕 (浮点 PID 计算 <100μs), 不影响实时性。但功耗比定点数略高。

**优化建议**: 如需极致省电, 将所有浮点转为 Q15/Q23 定点数运算。

### 7.7 delay_ms 是忙等待

CPU 全速空转等待, 浪费功耗。如需节电, 可改用定时器中断 + `__WFI()` 休眠。

### 7.8 调试串口 RX 中断未使用但已使能

SysConfig 中 `UART1.enabledInterrupts = ["RX"]` 但 `direction = "TX"`。RX 功能未使用但中断已配置。量产时可关掉以省电。

### 7.9 停止方式: 短刹车替代滑行 (v5)

到达目标脉冲数后, 设置 TB6612 AIN1=AIN2=HIGH (短刹车模式), 相比直接切断 PWM 的滑行方式, 停车更快更准。但短刹车消除了一段固定的滑行距离, 需要通过 `STOP_OFFSET_PULSES` 补偿。

### 7.10 距离标定

`PULSES_PER_LAP` 和 `STOP_OFFSET_PULSES` 是实测标定值, 不是理论计算值。标定方法:
1. 设定 `PULSES_PER_LAP` 初始值, `STOP_OFFSET_PULSES = 0`
2. 跑 1 圈, 观察实际停止位置
3. 多跑: 减小 `PULSES_PER_LAP`; 少跑: 增大 `PULSES_PER_LAP`
4. 不同圈数偏差一致(固定偏移): 调整 `STOP_OFFSET_PULSES`
5. 重复直到 1~5 圈均准确

---

## 8. 调参指南 (v4)

### 可调参数一览

#### 循迹参数 (在 empty.c 中)

| 参数 | 默认值 | 作用 | 调大效果 | 调小效果 |
|---|---|---|---|---|
| `BASE_DUTY_NOMINAL` | **0.35** | 巡航占空比初始值 | 车更快, 弯道可能冲 | 车更慢更稳 |
| `LINE_POLARITY` | **1.0** | 转向极性 | 左右对调 | 不变 |
| `LOST_OUTER_DUTY` | **0.30** | 丢线时外轮占空比 | 丢线转弯更快 | 丢线转弯更温和 |
| `PULSES_PER_LAP` | **5517** | 每圈目标脉冲数 | 多跑 | 少跑 |
| `STOP_OFFSET_PULSES` | **108** | 短刹车固定补偿 | 多跑 | 少跑 |

#### 速度 PID 参数 (在 empty.c 中)

| 参数 | 默认值 | 说明 |
|---|---|---|
| `SPEED_TARGET_PPS` | **50** | 目标: 每 50ms 窗口 50 脉冲 |
| `BASE_DUTY_MIN` | **0.10** | PID 输出下限 |
| `BASE_DUTY_MAX` | **0.65** | PID 输出上限 |

#### PWM 参数 (在 empty.c 中)

| 参数 | 默认值 | 说明 |
|---|---|---|
| `PWM_PERIOD` | **100** | 分辨率, 100 = 1% 步长 |
| `SYSTICK_FREQ_HZ` | **100000** | SysTick 中断频率 (PWM=1kHz) |

#### 编码器参数 (在 speed_sensor.h 中)

| 参数 | 默认值 | 说明 |
|---|---|---|
| `ENCODER_PPR` | **13** | 编码器每转脉冲数 |
| `GEAR_RATIO` | **20** | 减速比 |
| `MOTOR_MAX_RPM` | **8000.0** | 归一化上限 |
| `SPEED_WINDOW_MS` | **50** | 测速窗口 (仅调试用) |

### 常见问题速查

| 症状 | 解决方法 |
|---|---|
| **直线不稳/蛇形** | 降低 `BASE_DUTY_NOMINAL`, 或调整差速分级阈值 |
| **直角弯转不过去** | 增大 `LOST_OUTER_DUTY`, 或检查丢线检测是否太迟钝 |
| **丢线后乱转** | 确认 `last_error` 方向正确, 检查 `LINE_POLARITY` |
| **丢线找不回来** | 增大 `LOST_OUTER_DUTY`, 或减小 `LOST_MAX_TICKS` |
| **直道太慢** | 增大 `SPEED_TARGET_PPS` 或 `BASE_DUTY_NOMINAL` |
| **弯道冲出去** | 降低 `BASE_DUTY_NOMINAL` |
| **电池没电越跑越慢** | 正常, 速度 PID 会自动补偿 |
| **每圈都多跑/少跑固定距离** | 调整 `PULSES_PER_LAP` (±17 脉冲 ≈ ±1cm) |
| **不同圈数偏差一样(固定偏移)** | 调整 `STOP_OFFSET_PULSES` |

---

## 文件清单

```
25CAR/
├── empty.c              # 主程序 (main, SysTick ISR, 循迹逻辑)
├── empty.syscfg          # SysConfig 工程文件 (引脚/外设配置)
├── motor_control.h/c     # 电机控制模块 (方向 + PID 封装)
├── pid_controller.h/c    # 位置式 PID 控制器
├── speed_sensor.h/c      # 双编码器速度测量 (+ ISR)
├── grayscale.h/c         # 八路灰度传感器循迹 (I2C)
├── uart.h/c              # UART 调试串口
├── delay.h/c             # 延时函数
├── ti_msp_dl_config.h    # 重定向到 Debug/ti_msp_dl_config.h
├── Debug/
│   ├── ti_msp_dl_config.h   # SysConfig 生成的头文件 (宏定义)
│   └── ti_msp_dl_config.c   # SysConfig 生成的初始化代码
└── SYSTEM_DESIGN.md      # 本文档
```
