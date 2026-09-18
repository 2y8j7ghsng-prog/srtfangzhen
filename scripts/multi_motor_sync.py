# -*- coding: utf-8 -*-
"""
多电机协作（同步）控制仿真
========================================
四台 PMSM 简化模型（转速环 PI + 转矩饱和），对比三种经典同步策略：
  1. 主从控制（Master-Slave）
  2. 交叉耦合控制（Cross-Coupling Control, CCC）
  3. 惯量加权偏差耦合控制（Deviation Coupling Control, DCC）

运行：
  python scripts/multi_motor_sync.py
输出：
  results/speed_tracking.png        三种策略的转速跟随曲线
  results/sync_error_comparison.png 三种策略的同步误差对比
  results/metrics_bar.png           指标柱状图对比
  results/metrics.csv               指标数据表
  results/sim_data.csv              原始仿真数据（长表）

说明：本文件是首版 Python 实现，现已由 scripts/multi_motor_sync.c 取代
      （C 版是当前主入口，画图交给 plot_results.py）。此处保留作为可读性
      更强的算法参照与结果交叉验证。
"""

import json                      # 读取 data/params.json
import os                        # 路径拼接、创建输出目录
import csv                       # 写出指标表与原始数据长表

import numpy as np               # 向量化数值计算，仿真主体
import matplotlib                # 先导入顶层包，才能设置渲染后端

matplotlib.use("Agg")            # 无界面后端：只往文件写 PNG，不弹窗
import matplotlib.pyplot as plt  # 必须在 use("Agg") 之后导入才生效

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]  # 中文显示
plt.rcParams["axes.unicode_minus"] = False   # 负号用 ASCII '-'，否则部分中文字体下负号显示成方框

HERE = os.path.dirname(os.path.abspath(__file__))   # 本脚本目录：.../srtfangzhen/scripts
ROOT = os.path.dirname(HERE)                        # 仓库根目录：.../srtfangzhen
RESULTS = os.path.join(ROOT, "results")             # 输出目录
DATA = os.path.join(ROOT, "data")                   # 参数目录
os.makedirs(RESULTS, exist_ok=True)                 # 目录已存在时不报错

# ----------------------------------------------------------------------
# 1. 参数（与 data/params.json 保持一致）
# ----------------------------------------------------------------------
with open(os.path.join(DATA, "params.json"), "r", encoding="utf-8") as f:
    P = json.load(f)             # 全部参数集中从 json 读入，改工况不用动代码

N_MOTOR = P["n_motor"]           # 电机台数（=4）
DT = P["dt"]                     # 仿真步长 s（=1e-4，显式欧拉积分步长）
T_END = P["t_end"]               # 总仿真时长 s（=3.0）
T = np.arange(0.0, T_END, DT)    # 时间序列，等间隔步长 DT
STEPS = len(T)                   # 总步数（=30000）

# 四台同型号电机，带 ±10% 参数摄动（体现实际"多电机不一致性"）
J = np.array(P["J"])       # 转动惯量 kg·m^2，1/2/3/4 号分别为 0.010/0.011/0.0095/0.0105
B = np.array(P["B"])       # 粘性摩擦系数 N·m·s/rad
T_MAX = P["torque_limit"]  # 转矩限幅 N·m（模拟电机/变频器能力上限）

KP = P["speed_pi"]["kp"]   # 转速环比例增益 N·m/(rad/s)
KI = P["speed_pi"]["ki"]   # 转速环积分增益 N·m/rad

W_STAR = P["w_star"]           # 目标转速 rad/s（=120，约 1146 r/min）
RAMP_END = P["ramp_end"]       # 斜坡升速结束时刻 s（=0.5，软启动避免冲击）
T_LOAD_STEP = P["t_load_step"] # 电机 2 突加负载时刻 s（=1.2）
TL_STEP = P["tl_step"]         # 电机 2 突加负载幅值 N·m（=1.8）
T_LOAD_SIN = P["t_load_sin"]   # 电机 3 波动负载起始时刻 s（=2.0）
TL_SIN_AMP = P["tl_sin_amp"]   # 电机 3 波动负载幅值 N·m（=0.4）
TL_SIN_F = P["tl_sin_freq"]    # 电机 3 波动负载频率 Hz（=2.0）
T_LOAD_RAMP = P["t_load_ramp"] # 电机 4 斜坡负载起始时刻 s（=1.8）
T_RAMP_FULL = P["t_ramp_full"] # 电机 4 达到满负载的时刻 s（=2.6）
TL_RAMP = P["tl_ramp"]         # 电机 4 斜坡负载终值 N·m（=1.0）

K_SYNC_P = P["sync_pi"]["kp"]  # 同步补偿比例增益 Ks（=0.6）
K_SYNC_I = P["sync_pi"]["ki"]  # 同步补偿积分增益 Ksi（=5.0）


# ----------------------------------------------------------------------
# 2. 负载工况
# ----------------------------------------------------------------------
def load_torque(t):
    """四台电机的负载转矩 (N·m)，四种典型扰动覆盖"无扰/突变/周期/渐变"谱系"""
    tl = np.zeros(N_MOTOR)        # 先整体清零：1 号机全程空载（基准机）
    if t >= T_LOAD_STEP:
        tl[1] = TL_STEP                       # 电机 2 突加恒负载（模拟单机卡滞/切削阻力）
    if t >= T_LOAD_SIN:
        tl[2] = TL_SIN_AMP * np.sin(2 * np.pi * TL_SIN_F * (t - T_LOAD_SIN))
        # 上式即 0.4·sin(2π·2t') N·m，t' = t − T_LOAD_SIN 保证从 0 相位连续接入
    if t >= T_LOAD_RAMP:
        # 线性斜坡：未到满载时刻按比例上升，到点后钳位在终值 TL_RAMP
        a = min((t - T_LOAD_RAMP) / (T_RAMP_FULL - T_LOAD_RAMP), 1.0)
        tl[3] = TL_RAMP * a                   # 电机 4 渐变负载
    return tl


def speed_ref(t):
    """斜坡给定 0 -> W_STAR"""
    if t < RAMP_END:
        return W_STAR * t / RAMP_END   # 0~0.5 s 线性升速
    return W_STAR                      # 之后保持恒定目标转速


# ----------------------------------------------------------------------
# 3. 被控对象与转速环（含条件抗积分饱和）
# ----------------------------------------------------------------------
def torque_pi(ref, w, integ):
    """单台电机转速环 PI -> 电磁转矩，带条件积分抗饱和"""
    e = ref - w                              # 转速误差
    integ = integ + e * DT                   # 后向欧拉累加积分项
    te = KP * e + KI * integ                 # PI 输出的原始转矩
    if te > T_MAX:
        te = T_MAX
        integ = integ - e * DT   # 饱和时停止积分（conditional integration）
    elif te < -T_MAX:
        te = -T_MAX
        integ = integ - e * DT   # 负向同样退积分，避免退出饱和后大幅过冲
    return te, integ             # 返回（限幅后转矩, 更新后的积分项）


def plant_step(w, te, tl, i):
    """机械方程 J·dω/dt = Te − B·ω − TL 的欧拉离散"""
    return w + DT * (te - B[i] * w - tl[i]) / J[i]
    # 即 ω(k+1) = ω(k) + DT·(Te − B_i·ω − TL_i)/J_i，取第 i 台机的摄动参数


# ----------------------------------------------------------------------
# 4. 三种同步策略
# ----------------------------------------------------------------------
def run_master_slave():
    """主从控制：电机 1 为主机，2/3/4 号直接跟踪主机实际转速"""
    w = np.zeros((STEPS, N_MOTOR))       # 预存整条转速轨迹 [步, 电机]
    integ = np.zeros(N_MOTOR)            # 四路转速环 PI 积分器
    for k, t in enumerate(T):
        w_ref = speed_ref(t)
        # refs[0] 主机跟踪给定；refs[1..N-1] 从机全部跟踪主机"上一拍"转速
        # （对应实际系统中"主机测速→通信→从机执行"必然存在的一拍延迟）
        prev = w[k - 1, 0] if k > 0 else 0.0
        refs = np.full(N_MOTOR, prev)
        refs[0] = w_ref
        tl = load_torque(t)              # 本拍负载转矩
        for i in range(N_MOTOR):
            te, integ[i] = torque_pi(refs[i], w[k, i], integ[i])   # 转速环算转矩
            if k + 1 < STEPS:
                w[k + 1, i] = plant_step(w[k, i], te, tl, i)       # 机械方程推进一拍
    return w


def run_cross_coupling():
    """交叉耦合：每台电机对"自身与其他电机平均转速之差"做同步补偿"""
    w = np.zeros((STEPS, N_MOTOR))       # 转速轨迹
    integ_track = np.zeros(N_MOTOR)      # 四路转速环 PI 积分器
    integ_sync = np.zeros(N_MOTOR)       # 四路同步补偿积分器 ∫ε_i dt
    for k, t in enumerate(T):
        w_ref = speed_ref(t)
        tl = load_torque(t)
        for i in range(N_MOTOR):
            others = np.delete(w[k], i)             # 取本拍除 i 号外其余电机转速
            sync_err = w[k, i] - others.mean()      # 同步误差 ε_i = ω_i − avg(others)
            integ_sync[i] += sync_err * DT          # 同步误差积分
            comp = -(K_SYNC_P * sync_err + K_SYNC_I * integ_sync[i])
            # 负号：谁比大家快（ε_i>0）就压低谁的参考，反之抬高
            te, integ_track[i] = torque_pi(w_ref + comp, w[k, i], integ_track[i])
            if k + 1 < STEPS:
                w[k + 1, i] = plant_step(w[k, i], te, tl, i)
    return w


def run_deviation_coupling():
    """偏差耦合：对每对电机 (i,j) 的偏差分别补偿，权重按对方惯量占比分配"""
    w = np.zeros((STEPS, N_MOTOR))             # 转速轨迹
    integ_track = np.zeros(N_MOTOR)            # 四路转速环 PI 积分器
    integ_pair = np.zeros((N_MOTOR, N_MOTOR))  # 积分器 [i][j]：i 对 j 的偏差
    for k, t in enumerate(T):
        w_ref = speed_ref(t)
        tl = load_torque(t)
        for i in range(N_MOTOR):
            others = np.delete(np.arange(N_MOTOR), i)    # 除 i 以外的电机序号
            wj = J[others] / J[others].sum()             # 惯量加权系数 w_ij = J_j / Σ_{k≠i} J_k
            comp = 0.0
            for idx_j, j in enumerate(others):
                dev = w[k, i] - w[k, j]                  # 偏差 ω_i − ω_j
                integ_pair[i, j] += dev * DT             # 该对偏差各自积分（区别于 CCC 的单通道）
                comp -= wj[idx_j] * (K_SYNC_P * dev + K_SYNC_I * integ_pair[i, j])
            te, integ_track[i] = torque_pi(w_ref + comp, w[k, i], integ_track[i])
            if k + 1 < STEPS:
                w[k + 1, i] = plant_step(w[k, i], te, tl, i)
    return w


# ----------------------------------------------------------------------
# 5. 指标评价
# ----------------------------------------------------------------------
def sync_error(w_row):
    """同步误差 = max|ω_i − ω_j| = max − min（同型电机等价）"""
    return w_row.max() - w_row.min()    # 极差即最大同步偏差，三策略共用同一口径


def evaluate(w, name):
    se = np.array([sync_error(w[k]) for k in range(STEPS)])   # 逐拍同步误差序列
    mask_after_ramp = T > RAMP_END + 0.1                      # RMS 统计窗：升速结束后 0.1 s 起
    mask_step = (T >= T_LOAD_STEP) & (T <= T_LOAD_STEP + 0.6) # 突加后 0.6 s 观察窗

    peak_step = se[mask_step].max()                # 突加载期间同步误差峰值
    rms = np.sqrt((se[mask_after_ramp] ** 2).mean())   # 同步误差均方根

    # 恢复时间：突加负载后同步误差回落到 0.5 rad/s 以内并保持
    idx0 = int(np.searchsorted(T, T_LOAD_STEP))    # 突加时刻对应的步号
    rec = np.nan                                   # 默认 NaN，表示未恢复
    for k in range(idx0, STEPS):
        if all(se[j] <= 0.5 for j in range(k, min(k + 500, STEPS))):
            # 需要连续 500 步（50 ms）都不超阈值，防止"瞬时穿过"被误判为恢复
            rec = T[k] - T_LOAD_STEP               # 恢复时间 = 该时刻 − 突加时刻
            break

    dip = (W_STAR - w[mask_step, 1].min())  # 电机 2 最大转速跌落
    return {"策略": name,
            "同步误差RMS(rad/s)": round(rms, 3),
            "突加载同步误差峰值(rad/s)": round(peak_step, 3),
            "恢复时间(s)": None if np.isnan(rec) else round(rec, 3),   # NaN 转 None，写 CSV 更干净
            "电机2最大转速跌落(rad/s)": round(dip, 3)}, se


# ----------------------------------------------------------------------
# 6. 主流程：仿真 -> 出图 -> 存数据
# ----------------------------------------------------------------------
def main():
    print("运行主从控制仿真 ...")
    w_ms = run_master_slave()
    print("运行交叉耦合仿真 ...")
    w_cc = run_cross_coupling()
    print("运行偏差耦合仿真 ...")
    w_dc = run_deviation_coupling()

    runs = [("主从控制", w_ms), ("交叉耦合", w_cc), ("偏差耦合", w_dc)]   # (显示名, 转速轨迹)

    # ---- 转速跟随曲线 ----
    fig, axes = plt.subplots(3, 1, figsize=(9, 12), sharex=True, sharey=True)
    # 3 行 1 列（三策略各一格）；sharex/sharey 让三格坐标范围一致，便于纵向比较
    for ax, (name, w) in zip(axes, runs):
        for i in range(N_MOTOR):
            ax.plot(T, w[:, i], label=f"电机 {i + 1}")
        ax.plot(T, [speed_ref(t) for t in T], "k--", lw=1, label="给定转速")  # 目标转速参考线
        ax.axvline(T_LOAD_STEP, color="r", ls=":", lw=1)                     # 突加负载时刻
        ax.set_title(f"{name}：四电机转速跟随")
        ax.set_ylabel("转速 (rad/s)")
        ax.grid(alpha=0.3)                        # 淡网格，避免抢曲线
        ax.legend(loc="lower right", fontsize=8, ncols=2)   # 电机多了分两列
    axes[-1].set_xlabel("时间 (s)")                # 只在最下面一格标 x 轴
    fig.tight_layout()                             # 自动收紧边距
    fig.savefig(os.path.join(RESULTS, "speed_tracking.png"), dpi=150)
    plt.close(fig)                                 # 及时关闭释放内存

    # ---- 同步误差对比 ----
    fig, ax = plt.subplots(figsize=(9, 5))
    metrics = []                                   # 三个策略的指标字典
    colors = ["#1f77b4", "#d62728", "#2ca02c"]     # 固定配色，与柱状图保持一致
    for (name, w), c in zip(runs, colors):
        m, se = evaluate(w, name)                  # 画图的同时顺手把指标算出来
        metrics.append(m)
        ax.plot(T, se, label=name, color=c, lw=1.2)
    ax.axvline(T_LOAD_STEP, color="r", ls=":", lw=1)
    ax.annotate("电机 2 突加负载", xy=(T_LOAD_STEP, ax.get_ylim()[1] * 0.9),
                color="r", fontsize=9, ha="right")   # 竖线旁加标注，纵向取 y 轴上限的 90% 处
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("同步误差 max|ωi−ωj| (rad/s)")
    ax.set_title("三种同步策略的同步误差对比（四电机）")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS, "sync_error_comparison.png"), dpi=150)
    plt.close(fig)

    # ---- 指标柱状图 ----
    names = [m["策略"] for m in metrics]           # 三个策略的显示名
    keys = ["同步误差RMS(rad/s)", "突加载同步误差峰值(rad/s)", "电机2最大转速跌落(rad/s)"]
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    for ax, key in zip(axes, keys):
        vals = [m[key] for m in metrics]           # 该指标下三个策略的取值
        ax.bar(names, vals, color=colors)          # 柱色与曲线图一致
        ax.set_title(key, fontsize=10)
        ax.grid(axis="y", alpha=0.3)               # 只开横向网格线
        for x, v in enumerate(vals):
            ax.text(x, v, f"{v:.2f}", ha="center", va="bottom", fontsize=9)   # 柱顶标数值
    fig.suptitle("三种同步策略性能指标对比（四电机）")
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS, "metrics_bar.png"), dpi=150)
    plt.close(fig)

    # ---- 指标 CSV ----
    with open(os.path.join(RESULTS, "metrics.csv"), "w", newline="", encoding="utf-8-sig") as f:
        # utf-8-sig 带 BOM，Excel 打开中文表头不乱码
        writer = csv.DictWriter(f, fieldnames=list(metrics[0].keys()))   # 用第一个指标字典的键作表头
        writer.writeheader()
        writer.writerows(metrics)                  # 三个策略各写一行

    # ---- 原始数据长表 ----
    with open(os.path.join(RESULTS, "sim_data.csv"), "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(["t(s)", "策略", "电机", "转速(rad/s)"])   # 表头：长表格式（每行一个"时刻-电机"组合）
        step_out = 100  # 降采样输出，避免文件过大
        for name, w in runs:
            for k in range(0, STEPS, step_out):    # 每 100 步（10 ms）取一个点
                for i in range(N_MOTOR):
                    writer.writerow([round(T[k], 4), name, i + 1, round(w[k, i], 4)])

    # ---- 控制台摘要 ----
    print("\n===== 指标汇总 =====")
    print(f"{'策略':<8}{'RMS':>10}{'峰值':>10}{'恢复(s)':>10}{'跌落':>10}")
    for m in metrics:
        rec = m["恢复时间(s)"]
        print(f"{m['策略']:<8}{m['同步误差RMS(rad/s)']:>10.3f}"
              f"{m['突加载同步误差峰值(rad/s)']:>10.3f}"
              f"{rec if rec is not None else float('nan'):>10.3f}"   # 未恢复时以 nan 占位，保持列对齐
              f"{m['电机2最大转速跌落(rad/s)']:>10.3f}")
    print("\n图表与数据已输出到 results/")


if __name__ == "__main__":   # 仅当作为脚本直接运行时调用 main()，被 import 时不触发
    main()
