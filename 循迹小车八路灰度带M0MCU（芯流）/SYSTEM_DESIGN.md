# 循迹小车系统设计文档

> **MCU**: TI MSPM0G3507 (Cortex-M0+, 64MHz, 无硬件 FPU)  
> **电机驱动**: TB6612FNG (双 H 桥)  
> **编码器**: 霍尔编码器, 13 PPR, 1:20 减速比  
> **传感器**: 八路灰度 — 芯流 AuroraT8 (I2C 命令/响应协议, 地址 0x40)  
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
│                    主控制循环 (1ms / 1000Hz)                      │
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
│                    │ +全黑保护 │              │ 自动找回  │       │
│                    └────┬─────┘              └────┬─────┘       │
│                         │                         │             │
│                         └──────────┬──────────────┘             │
│                                    ▼                            │
│                           ┌─────────────────┐                   │
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
│   编码器: 计距(圈数控制+固定补偿)                                   │
│   基速: 固定占空比 20% (速度PID已禁用)                              │
│   停止: 到达目标脉冲后 TB6612 短刹车 (AIN1=AIN2=HIGH)               │
└─────────────────────────────────────────────────────────────────┘
```

### 控制周期

| 环节 | 周期 | 说明 |
|---|---|---|
| 主控制循环 | 1ms (1000Hz) | 灰度读取 → 误差计算 → 模式判断 → 差速 → PWM |
| 丢线/全黑检测 | 1ms (1000Hz) | 全白或 ≥6路全黑立即进入 LOST, 不遗漏过渡态 |
| 速度 PID | 已禁用 | 固定基速 20%, 提高低速可靠性和直角弯通过率 |
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

> 灰度模块: **芯流 AuroraT8**, I2C 地址 **0x40**。  
> 协议: 发送 1 字节命令 (0x0C=校准数字量) → 等待 200µs → 读取 4 字节响应帧。  
> 帧格式: [回显(1B) | 数据长度(1B) | 数据(1B) | 累加校验和(1B)]。  
> 原始数据 0=黑线/1=白色, 驱动层取反后 1=黑线/0=白色。  
> 物理布局: bit0=最右探头(X8), bit7=最左探头(X1)。  
> 模块校准通过板载按键完成。

### 其他

| 功能 | MSPM0 引脚 | 说明 |
|---|---|---|
| LED | **PB14** | 心跳指示, ~2.5Hz 闪烁 |
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

> **时钟源**: 使用内部 SYSOSC (32MHz), 不依赖外部晶振, 所有开发板通用。

---

## 4. 模块详解

### 4.1 电机控制

**文件**: [`motor_control.h`](motor_control.h), [`motor_control.c`](motor_control.c)

#### 数据结构

```c
typedef struct {
    PIDController pid;          // 每个电机独立的 PID 控制器 (当前未使用)
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
| `Motor_SetDutyCycle(motor, duty)` | 更新占空比记录 (不操作硬件) |
| `Motor_SetTargetSpeed(motor, speed)` | 更新目标速度 (不操作硬件) |
| `Motor_Update(motor, measured_speed)` | 运行 PID, 更新 `duty_cycle` (当前未调用) |
| `Motor_Stop(motor)` | 紧急停止, 方向→STOP, 占空比→0 |

#### 方向控制

```c
// motor_control.c Motor_SetGPIO()
FORWARD:  IN1=1, IN2=0  → 电机正转
REVERSE:  IN1=0, IN2=1  → 电机反转
STOP:     IN1=0, IN2=0  → TB6612 高阻停止
```

#### PWM 输出

PWM 不在此模块中直接操作硬件。由 `empty.c` 主循环直接将差速计算结果写入 `g_pwm_a` / `g_pwm_b`, SysTick ISR 据此控制 GPIO:

```c
// empty.c 主循环中:
g_pwm_a = (uint8_t)(left_duty  * (float)PWM_PERIOD);
g_pwm_b = (uint8_t)(right_duty * (float)PWM_PERIOD);
// SysTick ISR 根据 g_pwm_a / g_pwm_b 控制 GPIO PA12 / PB6
```

**为什么不使用定时器 CCP 输出?**

TIMG0/TIMG6 的 CCP0 外设输出在 PA12/PB6 上不工作。解决方案是使用 SysTick + GPIO 软件 PWM, 效果相同。

### 4.2 PID 控制器

**文件**: [`pid_controller.h`](pid_controller.h), [`pid_controller.c`](pid_controller.c)

#### 算法: 位置式 PID + 抗积分饱和

```c
float error = setpoint - measured_value;
float p_term = kp * error;
integral += error * dt;
float i_term = ki * integral;
float derivative = -(measured_value - prev_measured) / dt;  // 在测量值上微分
float d_term = kd * derivative;
float output = p_term + i_term + d_term;
// 输出限幅 + 积分退饱和 (clamping)
```

#### 当前使用状态

- **速度 PID (慢速层)**: **已禁用** —— 固定基速 `g_base_duty = BASE_DUTY_NOMINAL = 0.20`
- **电机级 PID**: 已初始化但未调用 `Motor_Update`, 参数 Kp=2.5, Ki=30.0, Kd=0.03, dt=5ms

> 禁用速度 PID 的原因: 在低速场景下 PID 将基速压低至 15~18%, 差速后内轮扭矩不足, 导致过弯无力。  
> 固定基速 20% 配合柔化差速分级, 直线平稳、弯道响应快。

### 4.3 速度测量

**文件**: [`speed_sensor.h`](speed_sensor.h), [`speed_sensor.c`](speed_sensor.c)

#### 硬件参数

```c
#define ENCODER_PPR        13       // 编码器每转脉冲数
#define GEAR_RATIO         20       // 减速比 1:20
#define OUTPUT_PPR         (13*20)  // 输出轴每转 260 脉冲
#define MOTOR_MAX_RPM      8000.0f  // 归一化用的最大转速
```

#### 当前使用状态

仅使用 `SpeedSensorA_GetTotalPulses()` 做距离计数 (圈数控制)。速度测量功能 (`SpeedSensor_Update`) 已闲置, 因为速度 PID 已禁用。

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

#### 临界区保护

主循环用 `__disable_irq()` / `__enable_irq()` 保护脉冲计数读取。

### 4.4 灰度循迹

**文件**: [`grayscale.h`](grayscale.h), [`grayscale.c`](grayscale.c)

#### 传感器布局 (俯视, 车头朝前)

```
[X1] [X2] [X3] [X4] [X5] [X6] [X7] [X8]
 -7   -5   -3   -1   +1   +3   +5   +7   ← 权重
```

#### I2C 通信 — 芯流 AuroraT8

- **设备地址**: 0x40 (与 OLED 0x3C 共享 I2C1 总线 PA10/PA11)
- **协议**: 命令/响应帧
  1. 发送 1 字节命令 (0x0C = 校准数字量)
  2. 等待 200µs (传感器准备数据)
  3. 读取 4 字节帧: `[回显(1B) | 数据长度(1B) | 数据(1B) | 累加校验和(1B)]`
  4. 验证回显=0x0C, 长度=1, 校验和正确
  5. 提取数据字节, 取反后 bit7→X1(最左) … bit0→X8(最右), 1=黑线
- **初始化**: `Grayscale_Init()` 等待 50ms 上电稳定, 校准通过模块板载按键完成

#### 误差计算: 加权平均法 (8路)

```c
static const int8_t weight[8] = { -7, -5, -3, -1, 1, 3, 5, 7 };
// 对应索引:                     X1  X2  X3  X4 X5 X6 X7 X8

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
| `Grayscale_ActiveMask()` | 打包为位图 (bit0=X1 … bit7=X8, active-high) |

### 4.5 软件 PWM

**实现位置**: [`empty.c`](empty.c) 的 `SysTick_Handler` ISR + 主循环

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
  每 1ms 将循迹计算结果写入 g_pwm_a / g_pwm_b
  每个变量 8 位, 32 位 CPU 上原子读写
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
- **TX**: PA28 (阻塞发送)
- **RX**: 中断 + 256 字节环形缓冲区 (当前未使用 RX 功能)

#### 调试输出格式 (约每秒一次)

```
GS:00011000 err=+0.00 st=0 | out:L= 40 R= 40 B= 40 | fail:0/0 dir:1 lost:0 | pulses:719 INF | NORM
│                │          │       │       │        │       │     │     │       │         │
│                │          │       │       │        │       │     │     │       │         └─ 模式
│                │          │       │       │        │       │     │     │       └─ 脉冲数
│                │          │       │       │        │       │     │     └─ LOST计时
│                │          │       │       │        │       │     └─ 方向历史标志
│                │          │       │       │        │       └─ 总失败/连续失败
│                │          │       │       │        └─ 基速×100
│                │          │       │       └─ 右轮占空比
│                │          │       └─ 左轮占空比
│                │          └─ 灰度通信状态
│                └─ 位置误差 [-1.00, +1.00]
└─ 8路灰度值 (X1 X2 X3 X4 X5 X6 X7 X8, 1=在线上)
```

### 4.7 延时

**文件**: [`delay.h`](delay.h), [`delay.c`](delay.c)

```c
void delay_ms(uint32_t ms) {
    uint32_t cycles = (CPUCLK_FREQ / 1000U) * ms;
    delay_cycles(cycles);
}
```

> CPU 忙等待, 不进入低功耗模式。

---

## 5. 主程序流程 (v5 — 固定基速)

```c
main() {
    SYSCFG_DL_init();
    SpeedSensorA_Init(); SpeedSensorB_Init();

    // --- GPIO 初始化 ---
    STBY 拉高;
    AIN1/AIN2 驱动强度提高;
    编码器引脚 PULL_UP;
    NVIC 使能编码器中断;
    PWM引脚 切为 GPIO 输出 LOW;
    SysTick 100kHz 初始化;

    // --- 模块初始化 ---
    Motor_Init(&motor1, PA8, PA9);  Motor_SetDirection(&motor1, REVERSE);
    Motor_Init(&motor2, PB4, PB5);  Motor_SetDirection(&motor2, REVERSE);
    Grayscale_Init();  // 芯流 AuroraT8, I2C 地址 0x40
    PID_Init(&speed_pid, 0.005, 0.002, 0, 0.05, 0.30, 0.65);
    g_base_duty = BASE_DUTY_NOMINAL;  // 0.20, 速度PID已禁用

    while (1) {
        delay_ms(1);            // 1ms 控制周期 (1000Hz)
        tick++;

        switch (state) {
        case STATE_IDLE:
            g_pwm_a = g_pwm_b = 0;
            // 每500ms打印灰度测试数据
            if (btn_press) → STATE_COUNTING;
            break;

        case STATE_COUNTING:
            // 累积按钮次数, 2秒超时 → STATE_RUNNING
            break;

        case STATE_RUNNING:
            gs_status = Grayscale_Read(&gs);
            if (gs_status != GS_READ_OK) {
                // 通信错误 → 停车, 不执行回找
                left_duty = right_duty = 0;
            } else {
                line_error = ComputeError(&gs) * LINE_POLARITY;
                line_lost  = IsLineLost(&gs);
                all_black  = (OnLineCount(&gs) >= 6);

                if (!line_lost && !all_black) {
                    // NORMAL: 差速分级 + 更新 last_error
                    inner_ratio = table(|error|);
                    left/right = error>0 ? (BASE, BASE*ratio) : (BASE*ratio, BASE);
                } else {
                    // LOST: 用 last_error 方向回找, 2s超时停车
                    left/right = last_error>0 ? (LOST_DUTY, 0) : (0, LOST_DUTY);
                }
            }
            g_pwm_a = left_duty * 100;
            g_pwm_b = right_duty * 100;

            // 距离计数 → 到达目标 → STATE_DONE (短刹车)
            break;

        case STATE_DONE:
            // OLED闪烁3次 → STATE_IDLE
            break;
        }
    }
}
```

---

## 6. 循迹控制算法 (v5 — 固定基速 + 全黑保护)

### 架构演进

| 版本 | 架构 | 问题 |
|---|---|---|
| v1 | 差速 → 速度 PID → 占空比 | 编码器 100ms 滞后, 直角弯来不及反应 |
| v2 | 位置 PID → 占空比 | PID 离散传感器参数难调, 直线蛇形 |
| v3 | 差速分级 + 直角弯专用 pivot | 直角弯触发条件脆弱, 泛用性差 |
| v4 | 差速分级 + 丢线保持 | 基速被PID压低, 弯道无力; 全黑时误更新方向 |
| **v5** | **固定基速 + all_black保护 + 柔化差速** | 当前版本 |

v5 核心改进:
- **固定基速**: 禁用速度 PID, `g_base_duty` 保持 20%, 确保差速后内轮仍有足够扭矩
- **all_black 保护**: ≥6路全黑时进入 LOST 模式, 防止弯道顶点误读覆盖 `last_error`
- **柔化差速**: 扩宽死区、提高内轮最低比率, 减少直线蛇形

### 模式状态机

```
                    ┌─────────────┐
          ┌────────▶│ MODE_NORMAL │◀────────┐
          │         │ 八路权值差速  │         │
          │         └──────┬──────┘         │
          │                │                │
          │    丢线 或 ≥6黑  │      找回线    │
          │                │                │
          │         ┌──────▼──────┐         │
          └─────────│ MODE_LOST   │─────────┘
                    │ last_error  │
                    │ 保持方向     │
                    └─────────────┘
```

### MODE_NORMAL — 差速分级

```c
float abs_err = fabsf(error);

if      (abs_err < 0.08)  inner_ratio = 1.0;  // 居中 → 两轮同速
else if (abs_err < 0.25)  inner_ratio = 0.6;  // 微偏 → 内轮 60%
else if (abs_err < 0.55)  inner_ratio = 0.3;  // 偏转 → 内轮 30%
else                      inner_ratio = 0.1;  // 急弯 → 内轮 10%
```

error > 0.05 时才更新 `last_error`, 确保方向记录可靠。

### MODE_LOST — 丢线保持 (含全黑保护)

```
全白 (line_lost) 或 ≥6路全黑 (all_black) → MODE_LOST
    │
    ├─ 无方向历史 → 停车 (lost_timer=0)
    ├─ last_error > 0 → 左轮 30%, 右轮停
    ├─ last_error < 0 → 左轮停, 右轮 30%
    │
    ├─ 找回线且 <6路黑 → MODE_NORMAL
    └─ 超时 2s → 停车等待 (lost_timer=LOST_MAX_TICKS)
```

### 参数总览

| 参数 | 当前值 | 说明 |
|---|---|---|
| `BASE_DUTY_NOMINAL` | **0.20** | 固定巡航占空比 (速度PID已禁用) |
| `LINE_POLARITY` | **1.0** | 转向极性 |
| `DIRECTION_DEADBAND` | **0.05** | 误差绝对值>死区才更新 last_error |
| `LOST_OUTER_DUTY` | **0.30** | 丢线时外轮占空比 |
| `LOST_MAX_TICKS` | **2000** | 丢线超时 = 2000×1ms = 2s |
| `all_black` 阈值 | **≥6** | 全黑探头≥6触发 LOST 保护 |
| `PULSES_PER_LAP` | **5517** | 每圈目标脉冲数 |
| `STOP_OFFSET_PULSES` | **108** | 短刹车固定补偿 |
| `PWM_PERIOD` | **100** | 100级分辨率 = 1%步长 |
| 差速死区 | **0.08** | |error|<0.08 两轮同速 |
| 内轮最低比率 | **0.10** | 最急弯内轮为外轮的10% |

---

## 7. 已知问题与陷阱

### 7.1 灰度模块物理安装高度

**现象**: 模块装太低时, 弯道处线斜穿探头会导致多路同时饱和 (>6路黑), 误读方向。

**修复**: 适当抬高灰度模块 (~8-12mm), 使正常线上 2-3 个探头触发, 弯道处最多 5-6 个。

### 7.2 TIMG CCP 输出不工作

**现象**: TIMG0/TIMG6 的 CCP0 外设输出无法驱动 PA12/PB6。

**解决方案**: 使用 SysTick 中断 + GPIO 软件 PWM 替代。

### 7.3 IOMUX 寄存器写反逻辑 Bug

**现象**: 代码注释写"编码器上拉", 但寄存器写入了 `0x00020000U` (下拉)。

**修复**: MSPM0 IOMUX PINCM 寄存器 `0x00010000U` = 上拉, `0x00020000U` = 下拉。代码已改为 `0x00010000U`。

### 7.4 浮点运算在 Cortex-M0+ 上无硬件支持

MSPM0G3507 是 Cortex-M0+ 核心, **没有硬件 FPU**。所有 `float` 运算由编译器软件模拟。当前控制周期 1ms 足够宽裕。

### 7.5 速度 PID 禁用

v4 架构中的速度 PID 在低速场景下将 `g_base_duty` 从 35% 压低至 15~18%, 导致差速后内轮占空比仅 8~9%, 落地无足够扭矩, 过弯困难。v5 改为固定基速方案。

### 7.6 停止方式: 短刹车

到达目标脉冲数后, 设置 TB6612 AIN1=AIN2=HIGH (短刹车模式), 相比滑行停车更快更准。需通过 `STOP_OFFSET_PULSES` 补偿刹车距离。

---

## 8. 调参指南

### 可调参数一览

#### 循迹参数 (在 empty.c 中)

| 参数 | 当前值 | 作用 | 调大效果 | 调小效果 |
|---|---|---|---|---|
| `BASE_DUTY_NOMINAL` | **0.20** | 巡航占空比 | 车更快, 弯道可能冲 | 车更慢更稳 |
| `LINE_POLARITY` | **1.0** | 转向极性 | 左右对调 | 不变 |
| `LOST_OUTER_DUTY` | **0.30** | 丢线时外轮占空比 | 丢线转弯更快 | 丢线转弯更温和 |
| `DIRECTION_DEADBAND` | **0.05** | 方向更新死区 | 更不易被干扰 | 方向更灵敏 |
| `all_black` 阈值 | **6** | 全黑保护触发数 | 更早触发保护 | 更晚触发 |
| `PULSES_PER_LAP` | **5517** | 每圈目标脉冲数 | 多跑 | 少跑 |
| `STOP_OFFSET_PULSES` | **108** | 短刹车固定补偿 | 多跑 | 少跑 |

#### 差速分级 (在 empty.c MODE_NORMAL 中)

| 阈值 | 内轮比率 | 说明 |
|---|---|---|
| < 0.08 | 1.0 (同速) | 直线死区, 防止蛇形 |
| < 0.25 | 0.6 | 微偏, 内轮略慢 |
| < 0.55 | 0.3 | 中度偏转 |
| ≥ 0.55 | 0.1 | 急弯, 大幅差速 |

#### 速度 PID 参数 (当前禁用)

| 参数 | 值 | 说明 |
|---|---|---|
| Kp | 0.005 | 如需启用, 先设 BASE_DUTY_MIN=0.30 |
| Ki | 0.002 | |
| BASE_DUTY_MIN | **0.30** | 启用PID后防止基速过低 |
| BASE_DUTY_MAX | **0.65** | |
| SPEED_TARGET_PPS | **80** | 每50ms窗口80脉冲 |
| LOST_MAX_TICKS | **2000** | 丢线超时 2s (1ms×2000) |

### 常见问题速查

| 症状 | 解决方法 |
|---|---|
| **直线蛇形** | 增大差速死区 (>0.08), 或降低 BASE_DUTY_NOMINAL |
| **直角弯转不过去** | 检查灰度模块安装高度; 降低 BASE_DUTY_NOMINAL; 降低 all_black 阈值 |
| **直角弯反向转** | 抬高灰度模块 (物理); 检查 all_black ≥6 是否生效 |
| **丢线后乱转** | 确认 LINE_POLARITY=1.0; 增大 DIRECTION_DEADBAND |
| **丢线找不回来** | 增大 LOST_OUTER_DUTY, 或减小 LOST_MAX_TICKS |
| **直道太慢** | 增大 BASE_DUTY_NOMINAL |
| **弯道冲出去** | 降低 BASE_DUTY_NOMINAL |
| **电机无力/断断续续** | 确认 BASE_DUTY_NOMINAL≥0.20; 如启用PID确认 MIN≥0.30 |
| **每圈都多跑/少跑固定距离** | 调整 PULSES_PER_LAP |
| **不同圈数偏差一样(固定偏移)** | 调整 STOP_OFFSET_PULSES |

---

## 文件清单

```
24g8car_copy/
├── empty.c              # 主程序 (main, SysTick ISR, 循迹逻辑)
├── empty.syscfg          # SysConfig 工程文件 (引脚/外设配置)
├── motor_control.h/c     # 电机控制模块 (方向 + PID 封装)
├── pid_controller.h/c    # 位置式 PID 控制器
├── speed_sensor.h/c      # 双编码器速度测量 (+ ISR)
├── grayscale.h/c         # 八路灰度传感器循迹 (AuroraT8 I2C)
├── oled.h/c              # SSD1306 OLED 显示
├── uart.h/c              # UART 调试串口
├── delay.h/c             # 延时函数
├── segment_display.h/c   # 数码管显示 (预留)
├── ti_msp_dl_config.h    # 重定向到 Debug/ti_msp_dl_config.h
├── Debug/
│   ├── ti_msp_dl_config.h   # SysConfig 生成的头文件 (宏定义)
│   └── ti_msp_dl_config.c   # SysConfig 生成的初始化代码
└── SYSTEM_DESIGN.md      # 本文档
```
