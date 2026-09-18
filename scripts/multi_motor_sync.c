/*=====================================================================
 * 多电机协作（同步）控制仿真 —— C 语言实现（嵌入式风格）
 *
 * 模型：3 台带参数摄动的电机（转速环 PI + 转矩饱和 + 条件抗积分饱和）
 * 策略：主从 / 交叉耦合(CCC) / 惯量加权偏差耦合(DCC)
 * 输出：results/sim_data_c.csv  降采样转速数据（供 plot_results.py 画图）
 *       results/metrics_c.csv   指标汇总
 *
 * 编译（在本仓库根目录执行）：
 *   gcc -O2 -o build/multi_motor_sync.exe scripts/multi_motor_sync.c -lm   <- 推荐
 *   tcc     -o build/multi_motor_sync.exe scripts/multi_motor_sync.c       <- 勿加 -lm
 *   实测 TinyCC 0.9.27 (x64) 对本代码存在传参代码生成 bug（混合 double/int
 *   参数的函数会算错），一律优先用 gcc。
 *
 * 运行：
 *   build\multi_motor_sync.exe          （必须在仓库根目录运行，输出走相对路径）
 *
 * 设计取向：全程静态数组、零动态内存（不用 malloc）、参数用宏集中管理、
 *           函数职责单一，便于日后整体平移到 STM32 等嵌入式目标。
 *====================================================================*/
#include <stdio.h>   /* printf / fprintf / fopen / fclose —— 控制台打印与 CSV 输出 */
#include <stdlib.h>  /* 本文件未直接调用，保留以备扩展（如 atoi / exit） */
#include <math.h>    /* sin() —— 电机3 的周期波动负载；sqrt() —— 指标 RMS 计算 */
#include <direct.h>  /* _mkdir() —— Windows 下创建输出目录 results/ */

/*====================== 参数区（与 data/params.json 对应） ======================*/
#define N_MOTOR        3              /* 电机台数；三台同型号，参数带 ±10% 摄动 */
#define DT             1e-4           /* 仿真步长 s（0.1 ms），显式欧拉积分步长 */
#define T_END          3.0            /* 总仿真时长 s                  */
#define N_STEPS        30000          /* = T_END / DT（静态数组长度需整型常量） */

/* 三台同型号电机，±10% 参数摄动（体现多电机不一致性）
   const 修饰 -> 只读，编译器可放入 .rdata 段并做常量传播优化 */
static const double J_MOTOR[N_MOTOR] = { 0.010, 0.011, 0.0095 };  /* 转动惯量 kg*m^2（1/2/3 号） */
static const double B_MOTOR[N_MOTOR] = { 0.0012, 0.0011, 0.0013 };/* 粘性摩擦 N*m*s/rad（1/2/3 号） */

#define TORQUE_LIMIT   5.0            /* 转矩限幅 N*m（模拟电机/变频器能力上限） */
#define SPEED_KP       0.8            /* 转速环 Kp  N*m/(rad/s)    */
#define SPEED_KI       30.0           /* 转速环 Ki  N*m/rad        */

#define W_STAR         120.0          /* 目标转速 rad/s（≈1146 r/min） */
#define RAMP_END       0.5            /* 斜坡升速结束时刻 s（软启动，避免冲击） */
#define T_LOAD_STEP    1.2            /* 电机2 突加负载时刻 s      */
#define TL_STEP        1.8            /* 电机2 突加负载幅值 N*m    */
#define T_LOAD_SIN     2.0            /* 电机3 波动负载起始 s      */
#define TL_SIN_AMP     0.4            /* 电机3 波动负载幅值 N*m    */
#define TL_SIN_FREQ    2.0            /* 电机3 波动负载频率 Hz     */

#define SYNC_KP        0.6            /* 同步补偿比例增益 Ks       */
#define SYNC_KI        5.0            /* 同步补偿积分增益 Ksi      */

#define OUT_EVERY      100            /* CSV 降采样步数（100×1e-4 s = 10 ms 记一行） */
#define REC_THRESHOLD  0.5            /* 恢复判定阈值 rad/s（误差回落到此值内算恢复） */
#define REC_HOLD       500            /* 恢复需保持的步数（500×1e-4 s = 50 ms） */

/*====================== 数据结构 ======================*/
/* 单个策略跑完后算出的四项性能指标 */
typedef struct {
    const char *name;                 /* 策略名（ASCII，画图时映射中文） */
    double rms;                       /* 升速后同步误差 RMS  rad/s */
    double peak;                      /* 突加载后 0.6s 内误差峰值  */
    double rec;                       /* 恢复时间 s，-1 表示未恢复 */
    double dip;                       /* 电机2 最大转速跌落 rad/s  */
} Metrics;

/* 策略间的共享"轨迹暂存区"：每个策略跑完把轨迹写进来，紧接着由 evaluate()
   读取算指标，随后被下一个策略覆盖。放在静态区而非函数栈上，是因为
   30000 点 × 8 B × 2 条 ≈ 480 KB，会撑爆默认线程栈。 */
static double se_hist[N_STEPS];       /* 当前策略的同步误差历史（复用） */
static double w2_hist[N_STEPS];       /* 当前策略电机2 转速历史（复用） */

/*====================== 工况与被控对象 ======================*/
/* 给定转速：0~RAMP_END 段线性斜坡升速，之后保持 W_STAR */
static double speed_ref(double t)
{
    return (t < RAMP_END) ? (W_STAR * t / RAMP_END) : W_STAR;
}

/* 各电机负载转矩：
     - 1 号机 全程空载；
     - 2 号机 t≥1.2 s 起突加 1.8 N*m 恒负载（模拟单机卡滞 / 切削阻力）；
     - 3 号机 t≥2.0 s 起叠加 0.4·sin(2π·2t) N*m 周期波动负载。
   OUT: tl[N_MOTOR] —— 本步三台电机的负载转矩，缓冲区由调用者提供 */
static void load_torque(double t, double tl[N_MOTOR])
{
    int i;
    for (i = 0; i < N_MOTOR; i++) tl[i] = 0.0;   /* 先整体清零，避免残留上一拍的旧值 */
    if (t >= T_LOAD_STEP) tl[1] = TL_STEP;                       /* 电机2 突加恒负载 */
    if (t >= T_LOAD_SIN)
        tl[2] = TL_SIN_AMP * sin(2.0 * 3.14159265358979 * TL_SIN_FREQ
                                 * (t - T_LOAD_SIN));            /* 电机3 波动负载   */
    /* 注：π 直接写字面量，避免依赖非标准宏 M_PI，便于跨编译器移植 */
}

/* 转速环 PI -> 电磁转矩，条件积分抗饱和
   IN : ref     本步参考转速 rad/s
        w       本步实际转速   rad/s
   IN/OUT: integ 积分累加项（跨步长保持，故传指针）
   OUT: 限幅后的电磁转矩 N*m
   抗饱和思路：若 PI 输出触限幅，就撤销本步的积分增量，
               避免"积分继续累积 → 退出饱和时大幅过冲"。 */
static double torque_pi(double ref, double w, double *integ)
{
    double e = ref - w;                        /* 转速误差 */
    double te;
    *integ += e * DT;                          /* 后向欧拉累加积分项 */
    te = SPEED_KP * e + SPEED_KI * (*integ);   /* PI 输出的原始转矩 */
    if (te > TORQUE_LIMIT || te < -TORQUE_LIMIT) {
        te = (te > 0.0) ? TORQUE_LIMIT : -TORQUE_LIMIT;   /* 双向饱和限幅 */
        *integ -= e * DT;                      /* 饱和时退积分（撤销本步增量） */
    }
    return te;
}

/* 机械方程欧拉离散：J·dω/dt = Te − B·ω − TL
   => ω(k+1) = ω(k) + DT·(Te − B·ω(k) − TL) / J
   IN : w 本步转速，te 本步电磁转矩，tl 本步负载转矩，i 电机序号（取 J、B 的摄动值）
   OUT: 下一拍转速 */
static double plant_step(double w, double te, double tl, int i)
{
    return w + DT * (te - B_MOTOR[i] * w - tl) / J_MOTOR[i];
}

/* 同步误差：同型电机下 max|ωi−ωj| 等价于 max − min
   三种策略共用同一评价口径，对比才公平 */
static double sync_error(const double w[N_MOTOR])
{
    double mx = w[0], mn = w[0];   /* 以 1 号机转速初始化最大/最小值 */
    int i;
    for (i = 1; i < N_MOTOR; i++) {
        if (w[i] > mx) mx = w[i];  /* 更新最大转速 */
        if (w[i] < mn) mn = w[i];  /* 更新最小转速 */
    }
    return mx - mn;                /* 极差即最大同步偏差 */
}

/*====================== 三种策略 ======================*/
/* 主从：1 号机为主机跟踪给定，2/3 号跟踪主机"上一拍"的实际转速。
   IN: fp 已打开的 CSV 句柄（按 OUT_EVERY 降采样写转速）
   说明：从机参考取 prev_master 而非本拍的 w[0]，对应实际系统中
         "主机测速 → 通信 → 从机执行"必然存在的一拍延迟。 */
static void run_master_slave(FILE *fp)
{
    double w[N_MOTOR] = { 0, 0, 0 };           /* 三台电机转速状态，从静止起步 */
    double integ[N_MOTOR] = { 0, 0, 0 };       /* 三路转速环 PI 积分器 */
    double tl[N_MOTOR], refs[N_MOTOR], prev_master = 0.0;
    int k, i;

    for (k = 0; k < N_STEPS; k++) {
        double t = k * DT;                     /* 当前仿真时刻 s */
        load_torque(t, tl);                    /* 求本拍负载转矩 */
        refs[0] = speed_ref(t);                /* 主机：跟踪给定斜坡 */
        refs[1] = prev_master;                 /* 从机2：跟踪主机上一拍转速 */
        refs[2] = prev_master;                 /* 从机3：同上 */
        for (i = 0; i < N_MOTOR; i++) {
            double te = torque_pi(refs[i], w[i], &integ[i]);   /* 转速环算转矩 */
            w[i] = plant_step(w[i], te, tl[i], i);             /* 机械方程推进一拍 */
        }
        prev_master = w[0];                    /* 缓存主机本拍转速，供下拍从机使用 */
        se_hist[k] = sync_error(w);            /* 记录本拍同步误差 */
        w2_hist[k] = w[1];                     /* 记录本拍电机2 转速（算跌落用） */
        if (k % OUT_EVERY == 0)                /* 降采样写 CSV，避免文件过大 */
            for (i = 0; i < N_MOTOR; i++)
                fprintf(fp, "%.4f,master_slave,%d,%.4f\n", t, i + 1, w[i]);
    }
}

/* 交叉耦合：对"自身与其他电机平均转速之差"做同步补偿
   ε_i = ω_i − avg(ω_j, j≠i)，再补偿到参考转速：
   ω_ref,i = ω* − (Ks·ε_i + Ksi·∫ε_i dt)
   直观理解：谁比大家快就把谁的参考压低、反之抬高 —— 全场一起"扶"受扰的那台。 */
static void run_cross_coupling(FILE *fp)
{
    double w[N_MOTOR] = { 0, 0, 0 };           /* 三台电机转速状态 */
    double integ[N_MOTOR] = { 0, 0, 0 };       /* 三路转速环 PI 积分器 */
    double isync[N_MOTOR] = { 0, 0, 0 };       /* 三路同步补偿积分器 ∫ε_i dt */
    double tl[N_MOTOR];
    int k, i, j;

    for (k = 0; k < N_STEPS; k++) {
        double t = k * DT;
        load_torque(t, tl);
        for (i = 0; i < N_MOTOR; i++) {
            double sum = 0.0, se, comp, te;
            for (j = 0; j < N_MOTOR; j++)
                if (j != i) sum += w[j];       /* 累加除自身外其他电机转速 */
            se = w[i] - sum / (N_MOTOR - 1);   /* ε_i = ω_i − avg(others) */
            isync[i] += se * DT;               /* 同步误差积分 */
            comp = -(SYNC_KP * se + SYNC_KI * isync[i]);   /* 负号：快的压低、慢的抬高 */
            te = torque_pi(speed_ref(t) + comp, w[i], &integ[i]);  /* 修正参考后过转速环 */
            w[i] = plant_step(w[i], te, tl[i], i);                 /* 机械方程推进一拍 */
        }
        se_hist[k] = sync_error(w);
        w2_hist[k] = w[1];
        if (k % OUT_EVERY == 0)
            for (i = 0; i < N_MOTOR; i++)
                fprintf(fp, "%.4f,cross_coupling,%d,%.4f\n", t, i + 1, w[i]);
    }
}

/* 惯量加权偏差耦合：对每一对 (i,j) 的偏差分别补偿，权重取对方惯量占比
   w_ij = J_j / Σ_{k≠i} J_k，即大惯量电机的偏差更"可信"、更值得跟随。
   与 CCC 的区别：CCC 只有一条"对平均值"的补偿通道、一个积分器；
                 DCC 对每对偏差各有一个积分器，响应更偏向跟随高惯量电机。 */
static void run_deviation_coupling(FILE *fp)
{
    double w[N_MOTOR] = { 0, 0, 0 };              /* 三台电机转速状态 */
    double integ[N_MOTOR] = { 0, 0, 0 };          /* 三路转速环 PI 积分器 */
    double ipair[N_MOTOR][N_MOTOR] = { { 0 } };   /* 成对同步积分器 ipair[i][j]：i 对 j 的偏差积分 */
    double tl[N_MOTOR];
    int k, i, j;

    for (k = 0; k < N_STEPS; k++) {
        double t = k * DT;
        load_torque(t, tl);
        for (i = 0; i < N_MOTOR; i++) {
            double wsum = 0.0, comp = 0.0, te;
            for (j = 0; j < N_MOTOR; j++)
                if (j != i) wsum += J_MOTOR[j];   /* 权重分母 Σ_{j≠i} J_j */
            for (j = 0; j < N_MOTOR; j++) {
                if (j == i) continue;             /* 跳过自身，无自偏差项 */
                {
                    double wj = J_MOTOR[j] / wsum;    /* 惯量加权系数 w_ij */
                    double dev = w[i] - w[j];         /* 成对偏差 ω_i − ω_j */
                    ipair[i][j] += dev * DT;          /* 该对偏差的积分累加 */
                    comp -= wj * (SYNC_KP * dev + SYNC_KI * ipair[i][j]);  /* 加权累加补偿 */
                }
            }
            te = torque_pi(speed_ref(t) + comp, w[i], &integ[i]);   /* 修正参考后过转速环 */
            w[i] = plant_step(w[i], te, tl[i], i);                  /* 机械方程推进一拍 */
        }
        se_hist[k] = sync_error(w);
        w2_hist[k] = w[1];
        if (k % OUT_EVERY == 0)
            for (i = 0; i < N_MOTOR; i++)
                fprintf(fp, "%.4f,deviation_coupling,%d,%.4f\n", t, i + 1, w[i]);
    }
}

/*====================== 指标评价 ======================*/
/* 依据缓存在 se_hist / w2_hist 里的本次仿真轨迹，算出四项指标
   IN : name 策略名（写进结构体，便于最后成表输出）
   OUT: Metrics —— RMS / 突加载峰值 / 恢复时间 / 电机2 最大转速跌落 */
static Metrics evaluate(const char *name)
{
    Metrics m;
    int k, k0, ok;
    /* 先把各时刻换算成"步下标"，避免在循环里反复做浮点除法 */
    int start = (int)((RAMP_END + 0.1) / DT);       /* RMS 统计起点：升速结束后 0.1 s */
    int step0 = (int)(T_LOAD_STEP / DT);            /* 突加负载时刻的步号 */
    int step1 = (int)((T_LOAD_STEP + 0.6) / DT);    /* 突加后 0.6 s 观察窗终点 */
    double sumsq = 0.0, w2min = 1e9;
    int n_rms = 0;

    m.name = name;
    m.rms = 0.0; m.peak = 0.0; m.rec = -1.0; m.dip = 0.0;   /* 先填默认值，杜绝读到未初始化内存 */

    /* ① 同步误差 RMS：升速稳定后全程均方根，反映整体同步水平 */
    for (k = start; k < N_STEPS; k++) { sumsq += se_hist[k] * se_hist[k]; n_rms++; }
    m.rms = sqrt(sumsq / n_rms);          /* 步数 3e4 量级，int 计数足够 */

    /* ② 突加载误差峰值、③ 电机2 最低转速：都只看突加后 0.6 s 窗口 */
    for (k = step0; k <= step1; k++) {
        if (se_hist[k] > m.peak) m.peak = se_hist[k];   /* 窗口内最大同步误差 */
        if (w2_hist[k] < w2min) w2min = w2_hist[k];     /* 窗口内电机2 最低转速 */
    }
    m.dip = W_STAR - w2min;   /* 电机2 最大转速跌落（真实值） */

    /* ④ 恢复时间：从突加时刻起，找第一个"误差回落到阈值内、且连续保持 REC_HOLD 步"的时刻 */
    for (k = step0; k < N_STEPS; k++) {
        ok = 1;
        for (k0 = k; k0 < k + REC_HOLD && k0 < N_STEPS; k0++)
            if (se_hist[k0] > REC_THRESHOLD) { ok = 0; break; }   /* 保持期内一旦超阈值即判失败 */
        if (ok) { m.rec = (k - step0) * DT; break; }              /* 恢复时间 = 步差 × 步长 */
    }
    return m;   /* 若始终未满足保持条件，m.rec 保持 -1，语义为"未恢复" */
}

/*====================== 主流程 ======================*/
int main(void)
{
    FILE *fp;
    Metrics ms[3];                /* 三个策略各存一份指标 */
    int i;

    _mkdir("results");   /* 已存在则返回 -1，此处忽略即可 */

    /* ---- 跑三种策略，转速数据顺带写进同一个 CSV ---- */
    fp = fopen("results/sim_data_c.csv", "w");
    if (!fp) { printf("[ERR] cannot open results/sim_data_c.csv\n"); return 1; }
    fprintf(fp, "t_s,strategy,motor,speed_rad_s\n");   /* 长表表头 */

    printf("run master_slave ...\n");       run_master_slave(fp);
    ms[0] = evaluate("master_slave");       /* 必须跑完立刻评分：se_hist 会被下一策略覆盖 */
    printf("run cross_coupling ...\n");     run_cross_coupling(fp);
    ms[1] = evaluate("cross_coupling");
    printf("run deviation_coupling ...\n"); run_deviation_coupling(fp);
    ms[2] = evaluate("deviation_coupling");
    fclose(fp);                             /* 转速数据已全部落盘 */

    /* ---- 指标写 CSV ---- */
    fp = fopen("results/metrics_c.csv", "w");
    if (!fp) { printf("[ERR] cannot open results/metrics_c.csv\n"); return 1; }
    fprintf(fp, "strategy,rms_rad_s,peak_rad_s,recover_s,dip_rad_s\n");
    for (i = 0; i < 3; i++)
        fprintf(fp, "%s,%.3f,%.3f,%.3f,%.3f\n",
                ms[i].name, ms[i].rms, ms[i].peak, ms[i].rec, ms[i].dip);
    fclose(fp);

    /* ---- 控制台汇总表 ---- */
    printf("\n===== metrics =====\n");
    printf("%-20s%10s%10s%10s%10s\n", "strategy", "RMS", "peak", "rec(s)", "dip");
    for (i = 0; i < 3; i++)
        printf("%-20s%10.3f%10.3f%10.3f%10.3f\n",
               ms[i].name, ms[i].rms, ms[i].peak, ms[i].rec, ms[i].dip);
    printf("\noutput: results/sim_data_c.csv, results/metrics_c.csv\n");
    return 0;   /* 正常退出 */
}
