%% dc_motor_model.m
% 直流电机数学模型 — 一键运行 step response 看系统固有特性
%
% 电机传递函数 (简化一阶模型):
%   G(s) = K / (tau * s + 1)
%
% 其中 K = 稳态增益 (占空比 1.0 → 电机满载)
%     tau = 机械时间常数 (秒)

clear; close all; clc;

%% ===================== 电机物理参数 (可按实测修改) =====================
% 这些参数用于更精确的二阶模型，简化版只用 K 和 tau

R   = 6.0;       % 电枢电阻 (Ohm)
L   = 0.002;     % 电枢电感 (H)
Kb  = 0.0025;    % 反电动势常数 (V/rpm)  — 约 0.0025*8000 = 20V max
Kt  = 0.0239;    % 转矩常数 (Nm/A)      — 近似等于 Kb(SI) = 0.0025*60/(2*pi)=0.0239
J   = 1e-4;      % 转动惯量 (kg·m²)
B   = 1e-5;      % 粘性摩擦系数 (N·m·s)

%% ===================== 一阶简化模型 =====================
% 假设 L 很小, 机械主导:
%   ω(s)/V(s) = Kt / ((Js + B)R + Kt*Kb)
%   = (Kt/(B*R + Kt*Kb)) / ((J*R/(B*R + Kt*Kb))*s + 1)

K_gain   = Kt / (B*R + Kt*Kb);      % 稳态增益 (rad/s)/V
tau_mech = J*R / (B*R + Kt*Kb);     % 机械时间常数 (s)

fprintf('========== 电机模型参数 ==========\n');
fprintf('稳态增益   K  = %.4f (rad/s)/V\n', K_gain);
fprintf('机械时间常数 τ = %.4f s\n', tau_mech);
fprintf('V_max = %.1f V (假设电池 12V → 满载电压 ≈ 12V)\n', 12.0);

% 归一化到 [0,1]:
%    输入: 占空比 duty ∈ [0,1]  → 等效电压 = duty * V_battery
%    输出: 归一化转速 ∈ [0,1]   → 实际转速 / MAX_RPM_rads
V_battery  = 12.0;                           % 电池电压 (V)
MAX_RPM    = 8000;                            % 最大转速
MAX_RADS   = MAX_RPM * (2*pi/60);            % ≈ 837.76 rad/s

% 归一化增益: 占空比 → 归一化转速
K_norm     = K_gain * V_battery / MAX_RADS;

fprintf('归一化增益 K_norm = %.4f (占空比→归一化转速)\n', K_norm);
fprintf('==================================\n\n');

%% ===================== 系统阶跃响应 (开环) =====================
sys_ol = tf(K_norm, [tau_mech, 1]);

figure;
subplot(2,1,1);
step(sys_ol * 0.5, sys_ol * 0.8, sys_ol * 1.0);
title('开环阶跃响应 — 不同占空比');
ylabel('归一化转速'); xlabel('时间 (s)'); grid on;
legend('duty=0.5','duty=0.8','duty=1.0','Location','southeast');

subplot(2,1,2);
bode(sys_ol);
grid on;
