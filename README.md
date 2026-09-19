# SRT 仿真项目（srtfangzhen）

**多电机协作控制仿真研究**：面向多电机驱动的移动装备（多轮独立电驱动底盘、多执行机构作业平台等），
研究参数不一致与负载突变条件下多台电机的**转速同步控制**，对比三种经典策略的同步性能与跟随代价。

> 一句话：四台带参数摄动的电机 + 单机突加负载 + 渐变斜坡负载 + 周期波动负载，比较
> 主从 / 交叉耦合 / 偏差耦合谁的同步误差最小、恢复最快。

**当前形态：C 语言仿真内核 + Python 画图。** 仿真主体是 `scripts/multi_motor_sync.c`
（嵌入式风格：静态数组、零动态内存、模块化函数，参数用宏集中管理），
编译产物输出 CSV；`scripts/plot_results.py` 只读 CSV 出图。
这样仿真能力可以平移到嵌入式/实时平台（STM32 等用同一套内核思路）。

## 当前结论（四电机，首轮仿真）

| 策略 | 同步误差 RMS (rad/s) | 突加载误差峰值 (rad/s) | 恢复时间 (s) |
|---|---|---|---|
| 主从控制 | 0.190 | 1.441 | 0.056 |
| 交叉耦合 | 0.099 | 0.880 | 0.032 |
| 惯量加权偏差耦合 | **0.098** | **0.873** | **0.032** |

- 耦合类策略把同步误差**降低约 48%**，代价是其余电机转速跟着略降（平均转速牺牲）。
- 由 3 台扩展到 4 台（新增 4 号机渐变斜坡负载）后三种策略排序不变、指标仅小幅变化，
  初步验证了策略的可扩展性；CCC 与 DCC 的差异仍需更恶劣工况区分。

## 目录结构

```
srtfangzhen/
├─ docs/       方案与公式推导（control_scheme.md）
├─ models/     MATLAB：multi_motor_sync_matlab.m 参照实现；run_c_sim.m 一键调 C 版仿真+后处理
├─ scripts/    multi_motor_sync.c   C 版仿真内核（主入口）
│              plot_results.py     读 CSV 出图
├─ data/       params.json（参数参考；C 版参数在 .c 文件顶部宏区）
├─ build/      编译产物（已 gitignore）
└─ results/    输出：转速曲线、同步误差对比、指标柱状图、CSV 数据
```

## 环境依赖

| 工具 | 版本 | 用途 |
|---|---|---|
| C 编译器 | MinGW-w64 gcc 16.2（`C:\Users\31394\.workbuddy\binaries\mingw64\bin`） | 编译仿真内核 |
| Python | 3.13 + numpy 2.5 + matplotlib 3.11 | 仅画图 |

> ⚠️ 实测 TinyCC 0.9.27 (x64) 对本代码存在传参代码生成 bug（混合 double/int 参数的
> 函数会算错），**不要用 tcc 编译本项目**，一律用 gcc。

## 快速开始

```bash
# 1. 编译（仓库根目录执行）
gcc -O2 -o build/multi_motor_sync.exe scripts/multi_motor_sync.c -lm

# 2. 运行（必须在仓库根目录，输出走相对路径）
build\multi_motor_sync.exe
#  -> results/sim_data_c.csv + results/metrics_c.csv，控制台打印指标汇总

# 3. 画图
python scripts/plot_results.py
#  -> results/speed_tracking.png / sync_error_comparison.png / metrics_bar.png
```

改参数：C 版改 `multi_motor_sync.c` 顶部"参数区"的宏（与 `data/params.json` 数值一一对应），
改完重新编译。

## 开发规范

- `main` 保持"能跑"的状态，大改动开 `feature/xxx` 分支
- 提交信息写清楚**改了什么、为什么改**，一行写不下就空一行写正文
- 仿真输出（图片、大数据表）**不要**无脑提交，先看 `.gitignore`
- 模型文件改了记得把**参数设置**也记录在 docs 里，二进制模型没法看 diff

## 开发日志

| 日期 | 内容 |
|---|---|
| 2026-09-16 | 仓库初始化 |
| 2026-09-17 | 填入多电机协作控制仿真：三电机模型 + 主从/交叉耦合/偏差耦合三策略对比，首轮结果落库 |
| 2026-09-17 | 仿真内核由 Python 重写为 C（嵌入式风格），Python 退居画图；MATLAB 版保留 |
| 2026-09-18 | 全代码补齐逐行注释；新增 models/run_c_sim.m，MATLAB 一键调 C 版仿真并读结果 |
| 2026-09-18 | 仿真由 3 电机扩展为 4 电机（新增 4 号机渐变斜坡负载工况），代码按 N_MOTOR 通用化，公式与文档同步更新 |
