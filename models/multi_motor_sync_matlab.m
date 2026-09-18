%% 多电机协作（同步）控制仿真 - MATLAB 版
%  与 scripts/multi_motor_sync.py 完全同一套模型和参数，
%  用于后续在 MATLAB 里调试、或作为迁移 Simulink 的参照实现。
%  运行：直接 F5 / 命令行 multi_motor_sync_matlab
%  输出：当前目录下 speed_tracking.png / sync_error_comparison.png / metrics.csv

clear; clc; close all;   % 清空工作区变量 / 命令行 / 关闭所有图窗，保证每次都从干净状态起跑

%% 参数
P = load_params();                             % 参数集中在 load_params() 里，便于与 params.json 逐项对照
n = P.n;  dt = P.dt;  N = round(P.t_end/dt);   % 电机台数 / 步长 s / 总步数
t = (0:N-1)' * dt;                             % 时间向量（列向量，便于整列参与矩阵运算）

J  = P.J;  B = P.B;  Tmax = P.torque_limit;    % 转动惯量 / 粘性摩擦 / 转矩限幅
Kp = P.speed_kp;  Ki = P.speed_ki;             % 转速环 PI 增益
Ks = P.sync_kp;   Ksi = P.sync_ki;             % 同步补偿 PI 增益
% 注：上面这几行在主脚本里未被直接引用（各子函数统一从 P 取参数），
%     保留是为了断点调试时能在工作区里直接看到全套参数值。

%% 三种策略
w_ms = run_master_slave(P, t);         % 主从：1 号机为主机，2/3/4 号跟踪主机实际转速
w_cc = run_cross_coupling(P, t);       % 交叉耦合(CCC)：对"自身与平均转速之差"做补偿
w_dc = run_deviation_coupling(P, t);   % 偏差耦合(DCC)：对每一对偏差按惯量加权补偿
runs = {struct('name','主从控制','w',w_ms), ...   % 打包成元胞数组，便于后面循环统一处理
        struct('name','交叉耦合','w',w_cc), ...
        struct('name','偏差耦合','w',w_dc)};

%% 同步误差对比图
figure('Position',[100 100 900 450]); hold on;   % 新建图窗并叠加绘制（先 hold on 再逐条 plot）
cols = lines(3);                                  % 取 3 种默认配色，供三条曲线区分
metrics = cell(1,3);                              % 预分配指标结构体容器
for r = 1:3
    w = runs{r}.w;                                % 当前策略的转速轨迹 N×n
    se = max(w,[],2) - min(w,[],2);               % 逐行取极差即同步误差 max|ωi−ωj|（空 [] 表示按行求）
    plot(t, se, 'LineWidth', 1.2, 'Color', cols(r,:), 'DisplayName', runs{r}.name);
    metrics{r} = evaluate(runs{r}.name, w, se, t, P);   % 顺手把该策略的四项指标算出来
end
xline(P.t_load_step, 'r:', '电机2突加负载');   % 标出突加负载时刻，便于肉眼定位扰动
xlabel('时间 (s)'); ylabel('同步误差 max|ω_i−ω_j| (rad/s)');
title('三种同步策略的同步误差对比（四电机）'); grid on; legend show;
saveas(gcf, 'sync_error_comparison.png');      % 存到"当前工作目录"（MATLAB 不认 .m 所在目录）

%% 转速跟随图
figure('Position',[100 100 900 750]);
legendEntries = [arrayfun(@(i) sprintf('电机%d', i), 1:P.n, 'UniformOutput', false), {'给定'}];
% 图例项按台数动态生成，改 n 不用再手改 legend
for r = 1:3
    subplot(3,1,r); hold on;                  % 3 行 1 列中的第 r 格，叠加绘制
    plot(t, runs{r}.w, 'LineWidth', 1);       % n 条曲线 = n 台电机的转速
    plot(t, min(P.w_star*t/P.ramp_end, P.w_star), 'k--');   % 给定转速斜坡（min 实现"升到顶即保持"）
    xline(P.t_load_step, 'r:');
    title([runs{r}.name '：四电机转速跟随']); ylabel('\omega (rad/s)');
    grid on; legend(legendEntries{:});
end
xlabel('时间 (s)');                           % 只在最下面一格标 x 轴，避免重复
saveas(gcf, 'speed_tracking.png');

%% 指标 CSV
fid = fopen('metrics.csv', 'w', 'n', 'UTF-8');   % 'n' = 本机字节序；UTF-8 保证中文表头不乱码
fprintf(fid, '策略,同步误差RMS,突加载峰值,恢复时间s,电机2跌落\r\n');
for r = 1:3, m = metrics{r};
    fprintf(fid, '%s,%.3f,%.3f,%.3f,%.3f\r\n', m.name, m.rms, m.peak, m.rec, m.dip);
end
fclose(fid);
disp('完成：metrics.csv / speed_tracking.png / sync_error_comparison.png');

%% ===================== 策略实现 =====================
function w = run_master_slave(P, t)
    n = P.n; dt = P.dt; N = numel(t);          % 台数 / 步长 / 步数
    w = zeros(N, n); integ = zeros(1, n);      % 转速轨迹 N×n；n 路转速环 PI 积分器
    for k = 1:N-1                              % 循环到 N-1：每拍都要写第 k+1 行的状态
        wr = speed_ref(t(k), P);               % 本拍给定转速
        if k == 1, master = 0; else, master = w(k,1); end   % 主机"上一拍"转速（首拍尚无历史，取 0）
        refs = [wr, repmat(master, 1, n-1)];   % 1 号机跟踪给定；其余从机跟踪主机实际转速
        tl = load_torque(t(k), P);             % 本拍各机负载转矩
        for i = 1:n
            [te, integ(i)] = torque_pi(refs(i), w(k,i), integ(i), P);    % 转速环算电磁转矩
            w(k+1,i) = w(k,i) + dt*(te - P.B(i)*w(k,i) - tl(i))/P.J(i);  % 机械方程欧拉推进一拍
        end
    end
end

function w = run_cross_coupling(P, t)
    n = P.n; dt = P.dt; N = numel(t);
    w = zeros(N, n); integ = zeros(1, n); isync = zeros(1, n);   % 转速环 PI + 同步补偿积分器
    for k = 1:N-1
        wr = speed_ref(t(k), P);
        tl = load_torque(t(k), P);
        for i = 1:n
            others = w(k, [1:i-1, i+1:n]);     % 本拍除 i 号外的其他电机转速（用索引拼接剔除 i）
            se = w(k,i) - mean(others);        % 同步误差 ε_i = ω_i − avg(others)
            isync(i) = isync(i) + se*dt;       % 同步误差积分 ∫ε_i dt
            comp = -(P.sync_kp*se + P.sync_ki*isync(i));   % 负号：快的压低、慢的抬高
            [te, integ(i)] = torque_pi(wr + comp, w(k,i), integ(i), P);   % 修正参考后过转速环
            w(k+1,i) = w(k,i) + dt*(te - P.B(i)*w(k,i) - tl(i))/P.J(i);
        end
    end
end

function w = run_deviation_coupling(P, t)
    n = P.n; dt = P.dt; N = numel(t);
    w = zeros(N, n); integ = zeros(1, n); ipair = zeros(n);   % ipair(i,j)：i 对 j 的偏差积分器
    for k = 1:N-1
        wr = speed_ref(t(k), P);
        tl = load_torque(t(k), P);
        for i = 1:n
            others = [1:i-1, i+1:n];               % 除 i 以外的电机序号
            wj = P.J(others) / sum(P.J(others));   % 惯量加权 w_ij = J_j / Σ_{k≠i} J_k
            comp = 0;
            for q = 1:numel(others)
                j = others(q);                     % 对方电机序号
                dev = w(k,i) - w(k,j);             % 成对偏差 ω_i − ω_j
                ipair(i,j) = ipair(i,j) + dev*dt;  % 每对偏差各自积分（区别于 CCC 的单通道）
                comp = comp - wj(q)*(P.sync_kp*dev + P.sync_ki*ipair(i,j));   % 加权累加补偿量
            end
            [te, integ(i)] = torque_pi(wr + comp, w(k,i), integ(i), P);
            w(k+1,i) = w(k,i) + dt*(te - P.B(i)*w(k,i) - tl(i))/P.J(i);
        end
    end
end

%% ===================== 公共函数 =====================
function [te, integ] = torque_pi(ref, w, integ, P)
    e = ref - w;                              % 转速误差
    integ = integ + e*P.dt;                   % 累加积分项
    te = P.speed_kp*e + P.speed_ki*integ;     % PI 输出的原始转矩
    if te > P.torque_limit || te < -P.torque_limit
        te = max(min(te, P.torque_limit), -P.torque_limit);   % 双向饱和限幅
        integ = integ - e*P.dt;   % 条件抗饱和：触限幅就撤销本拍积分增量
    end
end

function wr = speed_ref(t, P)
    wr = min(P.w_star*t/P.ramp_end, P.w_star);   % 斜坡升速；min 保证到达目标后保持、不超调
end

function tl = load_torque(t, P)
    tl = zeros(1, P.n);                       % 先整体清零：1 号机全程空载
    if t >= P.t_load_step, tl(2) = P.tl_step; end   % 2 号机突加恒负载（单机卡滞工况）
    if t >= P.t_load_sin
        tl(3) = P.tl_sin_amp*sin(2*pi*P.tl_sin_freq*(t - P.t_load_sin));   % 3 号机周期波动负载
    end
    if t >= P.t_load_ramp                     % 4 号机线性斜坡加载至满值并保持（渐变负载工况）
        tl(4) = P.tl_ramp * min((t - P.t_load_ramp)/(P.t_ramp_full - P.t_load_ramp), 1);
    end
end

function m = evaluate(name, w, se, t, P)
    maskRamp = t > P.ramp_end + 0.1;                             % RMS 统计窗：升速结束后 0.1 s 起
    maskStep = t >= P.t_load_step & t <= P.t_load_step + 0.6;    % 突加后 0.6 s 观察窗
    idx0 = find(t >= P.t_load_step, 1);                          % 突加时刻对应的步号
    rec = NaN;                                                   % 默认 NaN，语义为"未恢复"
    for k = idx0:numel(t)
        seg = se(k:min(k+499, end));                             % 从 k 起取 500 步（50 ms）窗口
        if all(seg <= 0.5), rec = t(k) - P.t_load_step; break; end   % 窗口内全程不超阈值即判恢复
    end
    m = struct('name', name, ...
        'rms', sqrt(mean(se(maskRamp).^2)), ...                  % 同步误差均方根
        'peak', max(se(maskStep)), ...                           % 突加载期间误差峰值
        'rec', rec, ...                                          % 恢复时间 s
        'dip', P.w_star - min(w(maskStep, 2)));                  % 电机2 最大转速跌落（2 号机）
end

function P = load_params()
    P.n = 4; P.dt = 1e-4; P.t_end = 3.0;      % 电机台数 / 仿真步长 s / 总时长 s
    P.J = [0.010, 0.011, 0.0095, 0.0105];     % 四台电机转动惯量 kg·m^2（±10% 参数摄动）
    P.B = [0.0012, 0.0011, 0.0013, 0.00115];  % 粘性摩擦 N·m·s/rad
    P.torque_limit = 5.0;                     % 转矩限幅 N·m
    P.speed_kp = 0.8; P.speed_ki = 30.0;      % 转速环 PI 增益
    P.w_star = 120.0; P.ramp_end = 0.5;       % 目标转速 rad/s；斜坡升速结束时刻 s
    P.t_load_step = 1.2; P.tl_step = 1.8;     % 2 号机突加负载的时刻 s 与幅值 N·m
    P.t_load_sin = 2.0; P.tl_sin_amp = 0.4; P.tl_sin_freq = 2.0;   % 3 号机波动负载参数
    P.t_load_ramp = 1.8; P.t_ramp_full = 2.6; P.tl_ramp = 1.0;     % 4 号机斜坡负载参数
    P.sync_kp = 0.6; P.sync_ki = 5.0;         % 同步补偿 PI 增益 Ks / Ksi
end
