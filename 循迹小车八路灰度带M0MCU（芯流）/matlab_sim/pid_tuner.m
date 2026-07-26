%% pid_tuner.m
% =========================================================================
% PID 参数扫描 & 自动对比 — 帮你找到更好的 Kp/Ki/Kd
% =========================================================================
% 运行这个脚本，它会用多组参数分别仿真，然后把 RPM 曲线画在一起对比。
% 你可以在下面的数组里添加自己想试的参数组合。
%
% 增益数量级说明 (归一化 [0,1] 系统):
%   Kp ≈ 1~5    — 比例项，error=0.1 时贡献 0.1~0.5 的占空比
%   Ki ≈ 10~60  — 积分项，需要在 ~0.2s 内积累到稳态 duty
%   Kd ≈ 0.01~0.1 — 微分项，抑制超调（有噪声时慎用）
% =========================================================================

clear; close all; clc;

%% ==================== 电机模型 (一阶惯性环节) ====================
tau     = 0.080;      % 机械时间常数 (秒) — 实车测出后改这里
MAX_RPM = 8000;

%% ==================== 仿真参数 ====================
dt      = 0.005;
out_min = 0;
out_max = 1;

%% ==================== 参数扫描列表 ====================
% pid_gains: 每行 [Kp, Ki, Kd]
% ⚠️ Ki 的数量级是 10~60，不是 0.1~3！

pid_gains = [
    0.35,  0.15, 0.0;    % 1  原始 (太慢)
    2.00, 20.00, 0.0;    % 2  Kp=2  Ki=20
    2.00, 35.00, 0.0;    % 3  Kp=2  Ki=35
    3.00, 40.00, 0.0;    % 4  Kp=3  Ki=40
    2.50, 30.00, 0.03;   % 5  Kp=2.5 Ki=30 Kd=0.03
    4.00, 55.00, 0.05;   % 6  激进 Kp=4 Ki=55 Kd=0.05
];

labels = {
    '原始 Kp=0.35 Ki=0.15';
    'Kp=2 Ki=20';
    'Kp=2 Ki=35';
    'Kp=3 Ki=40';
    'Kp=2.5 Ki=30 Kd=0.03';
    '激进 Kp=4 Ki=55 Kd=0.05';
};

n_trials = size(pid_gains, 1);

%% ==================== 仿真场景 ====================
sim_time = 2.0;
t = 0:dt:sim_time;
N = length(t);

setpoint_norm = zeros(1, N);
setpoint_norm(t >= 0.1) = 0.4;    % 0.1s → 40% = 3200 RPM
setpoint_norm(t >= 1.0) = 0.8;    % 1.0s → 80% = 6400 RPM

setpoint_rpm = setpoint_norm * MAX_RPM;

all_rpm = zeros(n_trials, N);

%% ==================== 逐组仿真 ====================
for trial = 1:n_trials
    Kp = pid_gains(trial, 1);
    Ki = pid_gains(trial, 2);
    Kd = pid_gains(trial, 3);

    % PID 状态初始化
    integral = 0; prev_measured = 0;
    omega_norm = 0;

    for k = 1:N
        % 传感器
        measured = omega_norm;
        if measured > 1, measured = 1; end
        if measured < 0, measured = 0; end
        all_rpm(trial, k) = measured * MAX_RPM;

        % PID (和 C 代码完全一致)
        error    = setpoint_norm(k) - measured;
        p_term   = Kp * error;
        integral = integral + error * dt;
        i_term   = Ki * integral;
        derivative    = -(measured - prev_measured) / dt;
        prev_measured = measured;
        d_term   = Kd * derivative;
        duty     = p_term + i_term + d_term;

        % 输出限幅 + 积分抗饱和
        if duty > out_max
            if error > 0, integral = integral - error * dt; end
            duty = out_max;
        elseif duty < out_min
            if error < 0, integral = integral - error * dt; end
            duty = out_min;
        end

        % 电机一阶惯性环节
        omega_norm = omega_norm + (dt / tau) * (duty - omega_norm);
        if omega_norm < 0, omega_norm = 0; end
    end
end

%% ==================== 对比图 ====================
figure('Name', 'PID 参数扫描对比', 'Position', [100, 100, 1000, 600]);

colors = lines(n_trials);

% 全图 — RPM
subplot(2,2,1);
plot(t, setpoint_rpm, 'k--', 'LineWidth', 1.5); hold on;
for i = 1:n_trials
    plot(t, all_rpm(i,:), 'Color', colors(i,:), 'LineWidth', 1.2);
end
ylabel('转速 (RPM)'); xlabel('时间 (s)');
title('速度响应对比 — 全图'); grid on;
legend(['目标', labels'], 'Location', 'southeast', 'FontSize', 7);

% 放大: 第一次阶跃 (0 → 3200 RPM)
subplot(2,2,2);
idx = t >= 0.08 & t <= 0.5;
plot(t(idx), setpoint_rpm(idx), 'k--', 'LineWidth', 1.5); hold on;
for i = 1:n_trials
    plot(t(idx), all_rpm(i,idx), 'Color', colors(i,:), 'LineWidth', 1.2);
end
ylabel('转速 (RPM)'); xlabel('时间 (s)');
title('放大: 第一次阶跃 (0→3200 RPM)'); grid on;

% 放大: 第二次阶跃 (3200 → 6400 RPM)
subplot(2,2,3);
idx = t >= 0.95 & t <= 1.5;
plot(t(idx), setpoint_rpm(idx), 'k--', 'LineWidth', 1.5); hold on;
for i = 1:n_trials
    plot(t(idx), all_rpm(i,idx), 'Color', colors(i,:), 'LineWidth', 1.2);
end
ylabel('转速 (RPM)'); xlabel('时间 (s)');
title('放大: 第二次阶跃 (3200→6400 RPM)'); grid on;

% 性能指标柱状图
subplot(2,2,4);
perf_data = zeros(n_trials, 3);
perf_label = cell(n_trials, 1);

for i = 1:n_trials
    idx1 = find(t >= 0.1, 1);
    idx2 = find(t >= 1.0, 1);
    step_rpm = all_rpm(i, idx1:idx2);
    sp_rpm = 0.4 * MAX_RPM;

    r10 = find(step_rpm >= sp_rpm*0.1, 1);
    r90 = find(step_rpm >= sp_rpm*0.9, 1);
    if ~isempty(r10) && ~isempty(r90)
        perf_data(i,1) = (r90 - r10) * dt * 1000;
    else
        perf_data(i,1) = NaN;  % 没达到 → 不画柱
    end

    perf_data(i,2) = abs(sp_rpm - mean(step_rpm(end-100:end))) / sp_rpm * 100;

    overshoot = max(step_rpm) - sp_rpm;
    if overshoot > 0
        perf_data(i,3) = overshoot / sp_rpm * 100;
    end

    % 短标签
    s = labels{i};
    if length(s) > 18, s = [s(1:15) '...']; end
    perf_label{i} = s;
end

bar(perf_data);
set(gca, 'XTickLabel', perf_label, 'FontSize', 6);
legend('上升时间(ms)', '稳态误差(%)', '超调(%)', 'FontSize', 8);
title('性能指标对比');
grid on;

sgtitle(sprintf('PID 参数扫描 — %d 组参数对比  (dt=%dms  \\tau=%dms)', ...
    n_trials, round(dt*1000), round(tau*1000)));

%% ==================== 命令行输出 ====================
fprintf('\n========== 参数对比汇总 ==========\n');
fprintf('%-35s %10s %10s %8s\n', '参数组', '上升(ms)', '稳态误差%', '超调%');
fprintf('%-35s %10s %10s %8s\n', '------', '-------', '--------', '----');
for i = 1:n_trials
    if isnan(perf_data(i,1))
        rise_str = 'N/A';
    else
        rise_str = sprintf('%.0f', perf_data(i,1));
    end
    fprintf('%-35s %10s %10.2f %8.1f\n', ...
        labels{i}, rise_str, perf_data(i,2), perf_data(i,3));
end
fprintf('\n提示: 如果所有参数结果相近, 说明 tau 太大(电机太慢), 减小 tau\n');
fprintf('      如果所有参数都有大超调, 说明 tau 太小(电机太快), 增大 tau\n');
fprintf('==================================\n');
