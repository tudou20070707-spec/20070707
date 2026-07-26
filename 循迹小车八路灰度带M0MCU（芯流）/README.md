# 循迹小车 — 八路灰度 (芯流 AuroraT8)

> **MCU**: TI MSPM0G3507 | **传感器**: AuroraT8 八路灰度 (I2C 0x40)  
> **电机**: TB6612FNG 双H桥 | **编码器**: 霍尔 13PPR 1:20

按钮控制电机走圈 + 自动循迹。上电后按按钮设圈数 (0=无限), 2秒后自动启动循迹, 到达目标后短刹车停止。

## 硬件连接

| 功能 | 引脚 |
|---|---|
| 左电机 PWM | PA12 |
| 左电机 IN1/IN2 | PA8 / PA9 |
| 右电机 PWM | PB6 |
| 右电机 IN1/IN2 | PB4 / PB5 |
| 灰度 SDA/SCL | PA10 / PA11 (I2C1, 与 OLED 共用) |
| 左编码器 A/B | PA17 / PA21 |
| 右编码器 A/B | PB0 / PB1 |
| OLED SDA/SCL | PA10 / PA11 |
| 按钮 | PA15 |
| 调试串口 TX | PA28 (115200) |

## 快速开始

1. CCS 打开项目, Clean Build
2. 烧录到 MSPM0G3507
3. 上电, OLED 显示 "Press BTN to start"
4. 按按钮选择圈数 (0=无限, 1~5), 2秒后自动开始循迹
5. 串口 115200 查看实时调试数据

## 详细文档

参见 [SYSTEM_DESIGN.md](SYSTEM_DESIGN.md)
