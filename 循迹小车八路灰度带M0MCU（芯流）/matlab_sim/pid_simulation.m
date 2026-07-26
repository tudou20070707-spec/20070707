%% pid_simulation.m
% =========================================================================
% PID 速度闭环仿真 — 完整复现你的 C 代码逻辑
% =========================================================================
% 使用方法:
%   1. 运行本脚本 (F5) 查看闭环控制效果
%   2. 修改下方 Kp/Ki/Kd 参数看变化
%   3. 在实车上测出 tau 后，替换下面的 tau 值
% =========================================================================

clear; close all; clc;

%% ==================== 可调参数 ====================
% ⚠️ 归一化 [0,1] 系统: Kp≈1~5, Ki≈10~60, Kd≈0.01~0.1
% 参考公式 (临界阻尼): Ki = (Kp+1)² / (4*tau)
%   例: Kp=2, tau=0.08 → Ki ≈ 9/0.32 = 28
Kp      = 2.00;      % 比例系数
Ki      = 30.0;      % 积分系数
Kd      = 0.03;      % 微分系数
dt      = 0.005;     % 控制周期 5ms (与你的 C 代码一致)
out_min = 0.0;       % 输出下限 (占空比 0%)
out_max = 1.0;       % 输出上限 (占空比 100%)

%% ==================== 电机模型 (一阶惯性环节) ====================
% 直流电机 + 负载的简化模型:
%    tau * dω_norm/dt + ω_norm = duty
%
% 物理含义:
%    duty=0.5 时, 稳态转速 = 0.5 * MAX_RPM = 4000 RPM
%    tau 越大响应越慢, 越小越快
%
% ⚠️ tau 需要在实车上测量:
%    给电机 50% 占空比阶跃, 记录转速从 0 升到 63.2% 终值的时间 = tau

tau     = 0.080;     % 机械时间常数 (秒) — 80ms 是典型小电机的初估值
MAX_RPM = 8000;      % 最大转速

%% ==================== 仿真场景设置 ====================
sim_time = 2.0;                     % 总仿真时间 (s)
t = 0:dt:sim_time;                  % 时间数组
N = length(t);

% 目标速度曲线 (归一化 [0,1], 对应 0 ~ MAX_RPM)
setpoint = zeros(1, N);
setpoint(t >= 0.1) = 0.4;           % 0.1s → 40% = 3200 RPM
setpoint(t >= 1.0) = 0.8;           % 1.0s → 80% = 6400 RPM

%% ==================== PID 初始化 (和 C 代码 PID_Init 一致) ====================
integral      = 0.0;
prev_error    = 0.0;
prev_measured = 0.0;

%% ==================== 仿真主循环 ====================
% 电机状态: 一阶惯性环节
%   omega_norm(k+1) = omega_norm(k) + (dt/tau) * (duty(k) - omega_norm(k))

omega_norm = 0.0;   % 归一化转速 [0, 1]

% 数据记录
log_output      = zeros(1, N);
log_measured    = zeros(1, N);
log_setpoint    = zeros(1, N);
log_error       = zeros(1, N);
log_integral    = zeros(1, N);
log_p_term      = zeros(1, N);
log_i_term      = zeros(1, N);
log_d_term      = zeros(1, N);
log_duty        = zeros(1, N);   % 上一拍的 duty, 用于电机模型

for k = 1:N
    % ---- 1. 传感器读数 (归一化到 [0,1]) ----
    measured = omega_norm;
    if measured > 1.0, measured = 1.0; end
    if measured < 0.0, measured = 0.0; end

    % ---- 2. PID 计算 (和 PID_Update 完全一致) ----
    error = setpoint(k) - measured;

    p_term = Kp * error;

    integral = integral + error * dt;
    i_term = Ki * integral;

    derivative = -(measured - prev_measured) / dt;  % 微分在测量值上
    prev_measured = measured;
    d_term = Kd * derivative;

    duty = p_term + i_term + d_term;

    % 输出限幅 + 积分抗饱和 (clamping) — 和 C 代码完全相同
    if duty > out_max
        if error > 0.0
            integral = integral - error * dt;
        end
        duty = out_max;
    elseif duty < out_min
        if error < 0.0
            integral = integral - error * dt;
        end
        duty = out_min;
    end

    prev_error = error;

    % ---- 3. 电机物理模型 (一阶惯性环节) ----
    % omega_norm(k+1) = omega_norm(k) + (dt/tau) * (duty - omega_norm(k))
    omega_norm = omega_norm + (dt / tau) * (duty - omega_norm);
    if omega_norm < 0.0, omega_norm = 0.0; end

    % ---- 4. 记录数据 ----
    log_output(k)   = duty;
    log_measured(k) = measured;
    log_setpoint(k) = setpoint(k);
    log_error(k)    = error;
    log_integral(k) = integral;
    log_p_term(k)   = p_term;
    log_i_term(k)   = i_term;
    log_d_term(k)   = d_term;
    log_duty(k)     = duty;
end

%% ==================== 绘图 ====================

% ---- 图1: 速度跟踪 (RPM) ----
figure('Name', 'PID 速度闭环仿真', 'Position', [100, 100, 900, 700]);

subplot(3,2,1);
plot(t, log_setpoint * MAX_RPM, 'b--', 'LineWidth', 1.2); hold on;
plot(t, log_measured * MAX_RPM, 'r-', 'LineWidth', 1.5);
ylabel('转速 (RPM)'); xlabel('时间 (s)');
title('速度跟踪');
legend('目标速度', '实际速度', 'Location', 'southeast');
grid on;

% ---- 图2: PID 输出 (占空比) ----
subplot(3,2,2);
plot(t, log_output, 'k-', 'LineWidth', 1.2);
ylabel('占空比'); xlabel('时间 (s)');
title('PID 输出 (占空比)');
ylim([-0.05, 1.05]);
grid on;

% ---- 图3: 误差 (RPM) ----
subplot(3,2,3);
plot(t, log_error * MAX_RPM, 'm-', 'LineWidth', 1.2);
ylabel('误差 (RPM)'); xlabel('时间 (s)');
title('跟踪误差 (目标RPM - 实际RPM)');
grid on;

% ---- 图4: P/I/D 各项贡献 ----
subplot(3,2,4);
plot(t, log_p_term, 'r-', 'LineWidth', 1); hold on;
plot(t, log_i_term, 'g-', 'LineWidth', 1);
plot(t, log_d_term, 'b-', 'LineWidth', 1);
ylabel('贡献值'); xlabel('时间 (s)');
title('P / I / D 各项');
legend('P', 'I', 'D', 'Location', 'best');
grid on;

% ---- 图5: 积分累积 ----
subplot(3,2,5);
plot(t, log_integral, 'g-', 'LineWidth', 1.2);
ylabel('积分累积'); xlabel('时间 (s)');
title('积分项累积值');
grid on;

% ---- 图6: 实际转速 RPM ----
subplot(3,2,6);
plot(t, log_measured * MAX_RPM, 'b-', 'LineWidth', 1.2);
ylabel('转速 (RPM)'); xlabel('时间 (s)');
title('实际转速 (RPM)');
grid on;

sgtitle(sprintf('PID 闭环控制  Kp=%.2f  Ki=%.2f  Kd=%.2f  dt=%dms  \\tau=%dms', ...
    Kp, Ki, Kd, round(dt*1000), round(tau*1000)));

%% ==================== 性能指标 ====================
fprintf('\n========== 闭环性能指标 ==========\n');

% 第一个阶跃 (0.1s → 0.4)
idx1 = find(t >= 0.1, 1);
idx2 = find(t >= 0.5, 1);
step1 = log_measured(idx1:idx2);
sp1   = 0.4;

% 上升时间 (10%→90%)
rise_start = find(step1 >= sp1 * 0.1, 1);
rise_end   = find(step1 >= sp1 * 0.9, 1);
if ~isempty(rise_start) && ~isempty(rise_end)
    rise_time = (rise_end - rise_start) * dt;
    fprintf('上升时间 (10%%→90%%):  %.0f ms\n', rise_time * 1000);
end

% 稳态误差
steady_vals = log_measured(idx2-50:idx2);
steady_err = sp1 - mean(steady_vals);
fprintf('稳态误差:              %.4f  (%.2f%%)\n', steady_err, steady_err/sp1*100);

% 超调量
overshoot = max(step1) - sp1;
if overshoot > 0
    fprintf('超调量:                %.4f  (%.1f%%)\n', overshoot, overshoot/sp1*100);
else
    fprintf('超调:                  无\n');
end

fprintf('\n提示: 如果仿真响应和实车不一致, 调整 tau 值即可\n');
fprintf('      tau 越大 = 电机响应越慢\n');
fprintf('      当前 tau = %.0f ms\n\n', tau*1000);
fprintf('==================================\n');
