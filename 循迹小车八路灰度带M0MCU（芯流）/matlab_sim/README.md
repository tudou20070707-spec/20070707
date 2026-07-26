# MATLAB PID 仿真说明

## 📁 文件说明

| 文件 | 用途 | 运行方式 |
|------|------|----------|
| `dc_motor_model.m` | 电机建模 — 查看开环特性、计算传递函数 | F5 运行 |
| `pid_simulation.m` | **主仿真脚本** — 复现C代码的PID闭环 | F5 运行 |
| `pid_tuner.m` | 参数扫描 — 多组Kp/Ki/Kd自动对比 | F5 运行 |

## 🚀 快速开始

1. MATLAB 中把 `matlab_sim/` 设为当前文件夹
2. 先运行 `dc_motor_model` → 看电机固有特性
3. 再运行 `pid_simulation` → 看闭环效果
4. 改参数后运行 `pid_tuner` → 对比不同参数

## 🔧 如何调整参数

### 方法1: 改 pid_simulation.m
打开文件，修改第 15-17 行:
```matlab
Kp = 0.35;
Ki = 0.15;
Kd = 0.0;    % 想加微分就改成比如 0.02
```
然后 F5 运行看效果。

### 方法2: 用 pid_tuner.m 批量对比
在 `params` 数组里添加你想试的 `[Kp, Ki, Kd, '标签']`:
```matlab
params = [
    0.35, 0.15, 0.0,  '当前 PI';
    0.50, 0.20, 0.0,  '我的新方案';
    % 加更多...
];
```

## 🧠 调参直觉

| 问题 | 可能原因 | 调整 |
|------|----------|------|
| 响应太慢 | Kp 太小 | ↑ Kp |
| 超调振荡 | Kp 太大 / 缺D | ↓ Kp, ↑ Kd |
| 有稳态误差 | Ki 太小 | ↑ Ki |
| 积分饱和 | 输出长时间触及限幅 | 已实现 anti-windup |
| 噪声敏感 | Kd 太大 | ↓ Kd, 或加低通滤波 |

## 📐 电机参数对照

仿真中的电机参数是**估测值**，你需要在实车上验证:

1. **测时间常数 τ**: 给电机 50% 占空比阶跃，记录转速达到终值63.2%的时间
2. **测稳态增益 K**: 给电机满占空比，记录最大稳定转速
3. 把实测值填入脚本中的 `R, L, Kb, Kt, J, B` 区域

## 🔗 Simulink 方案 (可选)

如果想用 Simulink 图形化仿真:

```
1. 新建 Simulink Model
2. 添加: Step → Sum → PID Controller → Transfer Fcn → Scope
3. Transfer Fcn 填入: K_norm / (tau * s + 1)
   (具体数值运行 dc_motor_model.m 后从命令行复制)
4. PID Controller 参数: P=0.35, I=0.15, D=0
5. 设置: Sample time = 0.005 (对应你的 5ms 控制周期)
```

Simulink 的优势是可以直接用 **PID Tuner App** 自动调参。

## 📊 输出示例

运行 `pid_simulation.m` 会生成 6 个子图:
- 速度跟踪曲线 (目标 vs 实际)
- PID 输出占空比
- 跟踪误差
- P/I/D 各项分解
- 积分累积
- 实际 RPM
