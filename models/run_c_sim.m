function run_c_sim()
% RUN_C_SIM  在 MATLAB 里调用 C 版仿真：编译(缺则补) -> 运行 exe -> 读结果 -> 画图
%
% 用法：MATLAB 里任意目录下直接敲 run_c_sim 即可。
% 流程：发现 build/multi_motor_sync.exe 不在，就用本机 MinGW gcc 现场编译；
%       然后 C 程序要求工作目录在仓库根目录（输出走相对路径 results/），
%       跑完自动把工作目录切回来；最后读 CSV 画同步误差对比图、打印指标表。
%
% 想改参数：去改 scripts/multi_motor_sync.c 顶部的宏区，删掉旧 exe 再跑本函数即可。

    root = fileparts(fileparts(mfilename('fullpath')));   % models/ 的上一级 = 仓库根目录
    exe  = fullfile(root, 'build', 'multi_motor_sync.exe');

    % ---- 1. 没有 exe 就现场编译（gcc 路径按本机实际位置写死，找不到再退回 PATH） ----
    if exist(exe, 'file') ~= 2
        gcc = 'C:\Users\31394\.workbuddy\binaries\mingw64\bin\gcc.exe';
        if exist(gcc, 'file') ~= 2
            gcc = 'gcc';                                   % 退路：用 PATH 里的 gcc
        end
        fprintf('未找到 exe，正在用 gcc 编译 ...\n');
        cmd = sprintf('"%s" -O2 -o "%s" "%s" -lm', gcc, exe, ...
                      fullfile(root, 'scripts', 'multi_motor_sync.c'));
        if system(cmd) ~= 0
            error('编译失败，请检查 gcc 路径是否正确。');
        end
    end

    % ---- 2. 运行 exe（必须 cwd = 仓库根目录；onCleanup 保证跑完/报错都切回原目录） ----
    oldDir = cd(root);
    c = onCleanup(@() cd(oldDir));
    system(['"' exe '"']);

    % ---- 3. 读 C 版输出的两份 CSV ----
    S = readtable(fullfile(root, 'results', 'sim_data_c.csv'));   % 转速长表：t, 策略, 电机, 转速
    M = readtable(fullfile(root, 'results', 'metrics_c.csv'));    % 指标表：RMS / 峰值 / 恢复 / 跌落

    % ---- 4. 同步误差对比图（在 MATLAB 里重画，方便叠加你自己的分析） ----
    figure('Position', [100 100 900 450]); hold on;
    cols  = lines(3);
    names = {'master_slave', 'cross_coupling', 'deviation_coupling'};
    cn    = {'主从控制', '交叉耦合', '偏差耦合'};
    for r = 1:3
        d = S(strcmp(S.strategy, names{r}), :);   % 取该策略的全部行（保持 CSV 原序）
        t = unique(d.t_s)';                       % 时间轴（C 侧每 10 ms 降采样一拍）
        w = reshape(d.speed_rad_s, [], 4);        % C 按电机1..4 连续写四行 -> 每列一台电机（改台数需同步这里）
        se = max(w, [], 2) - min(w, [], 2);       % 同步误差 = 四台转速的极差
        plot(t, se, 'LineWidth', 1.2, 'Color', cols(r,:), 'DisplayName', cn{r});
    end
    xline(1.2, 'r:', '电机2突加负载');             % 突加负载时刻（与 params.json 的 1.2 s 对应）
    xlabel('时间 (s)'); ylabel('同步误差 (rad/s)');
    title('C 版仿真结果（MATLAB 后处理）'); grid on; legend show;

    % ---- 5. 指标表直接打到命令行 ----
    disp('===== 指标（来自 metrics_c.csv） =====');
    disp(M);
end
