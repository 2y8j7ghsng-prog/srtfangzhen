# -*- coding: utf-8 -*-
"""
仿真结果画图脚本（配合 C 版仿真 multi_motor_sync.c）
==============================================
C 程序负责计算，输出：
  results/sim_data_c.csv  （t_s, strategy, motor, speed_rad_s）
  results/metrics_c.csv   （strategy, rms, peak, recover_s, dip）
本脚本只负责读 CSV -> 出图：
  results/speed_tracking.png
  results/sync_error_comparison.png
  results/metrics_bar.png

运行（在仓库根目录）：
  python scripts/plot_results.py
"""

import csv                        # 用 DictReader 按列名解析 C 版输出的 CSV
import os                         # 路径拼接、按本文件位置反推仓库根目录

import matplotlib                 # 先导入顶层包，才有办法设置渲染后端

matplotlib.use("Agg")             # 切到无界面后端：只往文件写 PNG，不弹窗（批处理友好）
import matplotlib.pyplot as plt   # 必须在 use("Agg") 之后导入，否则后端设置不生效

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]  # 中文显示
plt.rcParams["axes.unicode_minus"] = False   # 负号用 ASCII '-'，否则部分中文字体下负号会显示成方框

HERE = os.path.dirname(os.path.abspath(__file__))   # 本脚本目录：.../srtfangzhen/scripts
ROOT = os.path.dirname(HERE)                        # 仓库根目录：.../srtfangzhen
RES = os.path.join(ROOT, "results")                 # 结果目录：.../srtfangzhen/results
# 这样拼绝对路径，脚本在任意工作目录下执行都能找到 results/，不受 cwd 影响

# 画图参数（与 params.json / C 宏对应，仅用于标注参考线）
W_STAR = 120.0        # 目标转速，用来画水平参考线
RAMP_END = 0.5        # 斜坡升速结束时刻（本脚本暂未直接引用，保留以对齐参数口径）
T_LOAD_STEP = 1.2     # 电机2 突加负载时刻，用来画竖直参考线

# C 侧 ASCII 策略名 -> 中文
NAME_CN = {
    "master_slave": "主从控制",
    "cross_coupling": "交叉耦合",
    "deviation_coupling": "偏差耦合",
}
ORDER = ["master_slave", "cross_coupling", "deviation_coupling"]   # 绘图/图例顺序，与 C 侧输出顺序一致
COLORS = {"master_slave": "#1f77b4", "cross_coupling": "#d62728",
          "deviation_coupling": "#2ca02c"}                          # 固定配色，三张图里同一策略同色


def main():
    # ---- 读转速数据 ----
    # 结构：data[策略][电机号] -> [转速...]，times 同构、存对应时间轴
    data = {s: {1: [], 2: [], 3: []} for s in ORDER}
    times = {s: {1: [], 2: [], 3: []} for s in ORDER}
    with open(os.path.join(RES, "sim_data_c.csv"), encoding="utf-8") as f:
        for row in csv.DictReader(f):        # 按表头名取列，不依赖列顺序
            s, m = row["strategy"], int(row["motor"])
            times[s][m].append(float(row["t_s"]))
            data[s][m].append(float(row["speed_rad_s"]))
    # C 侧同一时刻连续写 3 行（电机1/2/3），因此三条曲线的时间轴天然对齐

    # ---- 读指标 ----
    metrics = {}
    with open(os.path.join(RES, "metrics_c.csv"), encoding="utf-8") as f:
        for row in csv.DictReader(f):
            metrics[row["strategy"]] = row   # 按策略名建索引；数值此时是字符串，用前再转 float

    # ---- 图 1：转速跟随 ----
    fig, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=True, sharey=True)
    # 3 行 1 列；sharex/sharey 让三格共用坐标范围，便于纵向直接比较
    for ax, s in zip(axes, ORDER):
        for m in (1, 2, 3):
            ax.plot(times[s][m], data[s][m], label=f"电机 {m}")
        ax.axhline(W_STAR, color="k", ls="--", lw=1, label="给定转速")   # 目标转速参考线
        ax.axvline(T_LOAD_STEP, color="r", ls=":", lw=1)                # 突加负载时刻竖线
        ax.set_title(f"{NAME_CN[s]}：三电机转速跟随")
        ax.set_ylabel("转速 (rad/s)")
        ax.grid(alpha=0.3)                       # 淡网格，避免抢曲线
        ax.legend(loc="lower right", fontsize=8) # 放在右下角，避开曲线主体
    axes[-1].set_xlabel("时间 (s)")               # 只在最下面一格标 x 轴标签
    fig.tight_layout()                            # 自动收紧边距，防标题/标签被裁
    fig.savefig(os.path.join(RES, "speed_tracking.png"), dpi=150)
    plt.close(fig)                                # 及时关闭释放内存，避免多图累积

    # ---- 图 2：同步误差 ----
    fig, ax = plt.subplots(figsize=(9, 5))
    for s in ORDER:
        # C 侧没有单独导出误差列，这里由三条转速现算：同步误差 = max|ωi−ωj| = max − min
        se = [max(a, b, c) - min(a, b, c)
              for a, b, c in zip(data[s][1], data[s][2], data[s][3])]
        ax.plot(times[s][1], se, label=NAME_CN[s], color=COLORS[s], lw=1.2)
    ax.axvline(T_LOAD_STEP, color="r", ls=":", lw=1)
    ax.annotate("电机 2 突加负载", xy=(T_LOAD_STEP, ax.get_ylim()[1] * 0.9),
                color="r", fontsize=9, ha="right")   # 竖线旁加标注，纵向取 y 轴上限的 90% 处
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("同步误差 max|ωi−ωj| (rad/s)")
    ax.set_title("三种同步策略的同步误差对比（C 版仿真）")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(RES, "sync_error_comparison.png"), dpi=150)
    plt.close(fig)

    # ---- 图 3：指标柱状图 ----
    keys = [("rms_rad_s", "同步误差RMS(rad/s)"),
            ("peak_rad_s", "突加载同步误差峰值(rad/s)"),
            ("dip_rad_s", "电机2最大转速跌落(rad/s)")]   # (CSV 列名, 图标题)
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    for ax, (key, title) in zip(axes, keys):
        names = [NAME_CN[s] for s in ORDER]
        vals = [float(metrics[s][key]) for s in ORDER]          # CSV 读入是字符串，需转 float 才能画
        ax.bar(names, vals, color=[COLORS[s] for s in ORDER])   # 柱色与曲线图保持一致
        ax.set_title(title, fontsize=10)
        ax.grid(axis="y", alpha=0.3)                            # 只开横向网格线
        for x, v in enumerate(vals):
            ax.text(x, v, f"{v:.2f}", ha="center", va="bottom", fontsize=9)   # 柱顶标数值
    fig.suptitle("三种同步策略性能指标对比（C 版仿真）")
    fig.tight_layout()
    fig.savefig(os.path.join(RES, "metrics_bar.png"), dpi=150)
    plt.close(fig)

    print("plots -> results/speed_tracking.png, sync_error_comparison.png, metrics_bar.png")


if __name__ == "__main__":   # 仅当作为脚本直接运行时调用 main()，被 import 时不触发
    main()
