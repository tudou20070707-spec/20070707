%% pid_auto_tune.m
% =========================================================================
% PID 自动寻优 — MATLAB 用优化算法帮你找到最好的 Kp/Ki/Kd
% =========================================================================
% 原理: 定义一个"代价函数"来评价控制效果，然后用 fminsearch 最小化它。
%
% 代价包含:
%   - 上升时间 (越短越好)
%   - 超调量   (越小越好)
%   - 稳态误差 (越小越好)
%
% 你只需要:
%   1. 修改下面的 tau 和电机参数
%   2. 按 F5 运行
%   3. 看优化后的结果
%   4. 把命令行输出的结果发给我分析
% =========================================================================

clear; close all; clc;

%% ==================== 电机模型 ====================
tau     = 0.080;      % 机械时间常数 (秒) — 实车测出后改这里
MAX_RPM = 8000;
dt      = 0.005;
out_min = 0;
out_max = 1;

%% ==================== 仿真场景 ====================
sim_time = 1.5;
t = 0:dt:sim_time;
N = length(t);

% 目标: 0.1s 时从 0 跳到 50% 转速 (4000 RPM)
setpoint = zeros(1, N);
setpoint(t >= 0.1) = 0.5;

%% ==================== 代价函数 ====================
% 权重: 你觉得什么最重要就调谁
w_rise  = 1.0;   % 上升时间权重
w_overs = 2.0;   % 超调权重 (通常更重要)
w_ss    = 5.0;   % 稳态误差权重 (最重要)

function cost = pid_cost(gains, t, setpoint, tau, dt, out_min, out_max, MAX_RPM, w_rise, w_overs, w_ss)
    Kp = gains(1);
    Ki = gains(2);
    Kd = gains(3);

    % 参数合法性检查
    if Kp < 0 || Ki < 0 || Kd < 0
        cost = 1e10;
        return;
    end

    N = length(t);
    integral = 0; prev_measured = 0;
    omega_norm = 0;

    measured_all = zeros(1, N);

    for k = 1:N
        measured = omega_norm;
        if measured > 1, measured = 1; end
        if measured < 0, measured = 0; end
        measured_all(k) = measured;

        error = setpoint(k) - measured;
        p_term = Kp * error;
        integral = integral + error * dt;
        i_term = Ki * integral;
        derivative = -(measured - prev_measured) / dt;
        prev_measured = measured;
        d_term = Kd * derivative;
        duty = p_term + i_term + d_term;

        if duty > out_max
            if error > 0, integral = integral - error * dt; end
            duty = out_max;
        elseif duty < out_min
            if error < 0, integral = integral - error * dt; end
            duty = out_min;
        end

        omega_norm = omega_norm + (dt / tau) * (duty - omega_norm);
        if omega_norm < 0, omega_norm = 0; end
    end

    % ---- 计算性能指标 ----
    idx1 = find(t >= 0.1, 1);
    idx_end = find(t >= 1.0, 1);
    if isempty(idx_end), idx_end = N; end
    step_resp = measured_all(idx1:idx_end);
    sp = 0.5;

    % 上升时间 (10%→90%)
    r10 = find(step_resp >= sp * 0.1, 1);
    r90 = find(step_resp >= sp * 0.9, 1);
    if isempty(r10) || isempty(r90)
        rise_time = 1.0;  % 惩罚: 根本没达到
    else
        rise_time = (r90 - r10) * dt;
    end

    % 超调量
    overshoot = max(0, max(step_resp) - sp) / sp;

    % 稳态误差
    ss_error = abs(sp - mean(step_resp(end-100:end))) / sp;

    % 综合代价 (越小越好)
    cost = w_rise * rise_time + w_overs * overshoot + w_ss * ss_error;
end

%% ==================== 运行优化 ====================
fprintf('========== PID 自动寻优 ==========\n');
fprintf('电机时间常数 tau = %.0f ms\n', tau * 1000);
fprintf('正在搜索最优 Kp, Ki, Kd ...\n\n');

% 初始猜测 (你当前的参数)
x0 = [0.35, 0.15, 0.0];

% 用 fminsearch 搜索 (不需要 Optimization Toolbox)
options = optimset('Display', 'iter', 'MaxFunEvals', 500, 'MaxIter', 200);
[x_opt, fval] = fminsearch(@(x) pid_cost(x, t, setpoint, tau, dt, ...
    out_min, out_max, MAX_RPM, w_rise, w_overs, w_ss), x0, options);

Kp_opt = x_opt(1);
Ki_opt = x_opt(2);
Kd_opt = x_opt(3);

fprintf('\n========== 优化结果 ==========\n');
fprintf('最优参数:  Kp = %.4f\n', Kp_opt);
fprintf('           Ki = %.4f\n', Ki_opt);
fprintf('           Kd = %.4f\n', Kd_opt);
fprintf('代价函数值: %.6f\n\n', fval);

%% ==================== 对比仿真: 原始 vs 优化 ====================
fprintf('正在运行对比仿真...\n');

% 原始参数仿真
[~, rpm_orig, duty_orig] = run_sim([0.35, 0.15, 0.0], t, setpoint, tau, dt, out_min, out_max, MAX_RPM);
% 优化参数仿真
[~, rpm_opt,  duty_opt]  = run_sim(x_opt, t, setpoint, tau, dt, out_min, out_max, MAX_RPM);

%% ==================== 对比图 ====================
figure('Name', 'PID 自动寻优结果', 'Position', [100, 100, 1000, 500]);

% 速度响应对比
subplot(1,2,1);
plot(t, setpoint * MAX_RPM, 'k--', 'LineWidth', 1.5); hold on;
plot(t, rpm_orig, 'r-', 'LineWidth', 1.2);
plot(t, rpm_opt, 'b-', 'LineWidth', 1.8);
ylabel('转速 (RPM)'); xlabel('时间 (s)');
title(sprintf('速度响应对比  (tau=%.0f ms)', tau*1000));
legend('目标', sprintf('原始 Kp=%.2f Ki=%.2f', 0.35, 0.15), ...
    sprintf('优化 Kp=%.2f Ki=%.2f Kd=%.2f', Kp_opt, Ki_opt, Kd_opt), ...
    'Location', 'southeast');
grid on;

% 占空比输出对比
subplot(1,2,2);
plot(t, duty_orig, 'r-', 'LineWidth', 1.2); hold on;
plot(t, duty_opt, 'b-', 'LineWidth', 1.8);
ylabel('占空比'); xlabel('时间 (s)');
title('PID 输出对比');
legend('原始', '优化', 'Location', 'southeast');
ylim([-0.05, 1.05]);
grid on;

sgtitle(sprintf('PID 自动寻优 — 原始 vs 优化  (代价: %.4f → %.4f)', ...
    pid_cost([0.35, 0.15, 0.0], t, setpoint, tau, dt, out_min, out_max, MAX_RPM, w_rise, w_overs, w_ss), ...
    fval));

%% ==================== 性能对比表 ====================
fprintf('========== 性能对比 ==========\n');
fprintf('%-20s %12s %12s\n', '指标', '原始(0.35/0.15/0)', '优化');
fprintf('%-20s %12s %12s\n', '----', '----------------', '-----');

perf_orig = calc_perf(rpm_orig, 0.5 * MAX_RPM, t, dt);
perf_opt  = calc_perf(rpm_opt,  0.5 * MAX_RPM, t, dt);

fprintf('%-20s %10.0f ms %10.0f ms\n', '上升时间(10-90%)', perf_orig(1), perf_opt(1));
fprintf('%-20s %10.1f %%  %10.1f %%\n', '超调量', perf_orig(2), perf_opt(2));
fprintf('%-20s %10.2f %% %10.2f %%\n', '稳态误差', perf_orig(3), perf_opt(3));
fprintf('==================================\n\n');

fprintf('提示:\n');
fprintf('  1. 把上面这行 "优化 Kp=... Ki=... Kd=..." 发给我\n');
fprintf('  2. 把图截屏发给我\n');
fprintf('  3. 告诉我 tau 的值\n');
fprintf('  4. 我来帮你判断这组参数是否适合实车部署\n');

%% ==================== 辅助函数 ====================
function [measured, rpm, duty_out] = run_sim(gains, t, setpoint, tau, dt, out_min, out_max, MAX_RPM)
    Kp = gains(1); Ki = gains(2); Kd = gains(3);
    N = length(t);
    integral = 0; prev_measured = 0;
    omega_norm = 0;
    measured = zeros(1, N);
    duty_out = zeros(1, N);

    for k = 1:N
        meas_val = omega_norm;
        if meas_val > 1, meas_val = 1; end
        if meas_val < 0, meas_val = 0; end
        measured(k) = meas_val;

        error = setpoint(k) - meas_val;
        p_term = Kp * error;
        integral = integral + error * dt;
        i_term = Ki * integral;
        derivative = -(meas_val - prev_measured) / dt;
        prev_measured = meas_val;
        d_term = Kd * derivative;
        duty = p_term + i_term + d_term;

        if duty > out_max
            if error > 0, integral = integral - error * dt; end
            duty = out_max;
        elseif duty < out_min
            if error < 0, integral = integral - error * dt; end
            duty = out_min;
        end
        duty_out(k) = duty;

        omega_norm = omega_norm + (dt / tau) * (duty - omega_norm);
        if omega_norm < 0, omega_norm = 0; end
    end
    rpm = measured * MAX_RPM;
end

function perf = calc_perf(rpm, sp_rpm, t, dt)
    idx1 = find(t >= 0.1, 1);
    idx_end = find(t >= 1.0, 1);
    if isempty(idx_end), idx_end = length(t); end
    step_rpm = rpm(idx1:idx_end);

    r10 = find(step_rpm >= sp_rpm * 0.1, 1);
    r90 = find(step_rpm >= sp_rpm * 0.9, 1);
    if isempty(r10) || isempty(r90)
        rise_time = 999;
    else
        rise_time = (r90 - r10) * dt * 1000;  % ms
    end

    overshoot_pct = max(0, max(step_rpm) - sp_rpm) / sp_rpm * 100;
    ss_error_pct  = abs(sp_rpm - mean(step_rpm(end-100:end))) / sp_rpm * 100;

    perf = [rise_time, overshoot_pct, ss_error_pct];
end
