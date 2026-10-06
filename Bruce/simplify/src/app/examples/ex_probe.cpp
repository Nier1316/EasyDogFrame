// =====================================================================
//  真机激励探针（Example60）—— 悬空 + 500 Hz 数据集录制
//  见 include/app/examples/ex_probe.h 的分工说明。
// =====================================================================
#include "app/examples/ex_probe.h"
#include "app/examples_common.h"
#include "transport/usb2can_transport.h"
#include "motor/motor_manager.h"
#include "motor/motor_calibration.h"
#include "runtime/thread_manager.h"
#include "common/log_control.h"
#include "common/s2r_dataset.h"
#include "motion/robot_calibration.h"
#include "strategy/rl_controller.h"   // rl::WHEEL_KVP / WHEEL_KVI
#include "strategy/imu_device.h"
#include <cstdio>
#include <cstring>
#include <ctime>
#include <chrono>
#include <sys/stat.h>
#include <cmath>
#include <string>
#include <unistd.h>

using logctl::LogCat;

namespace {

constexpr int   HZ       = 500;              // 激励与录制节拍（对齐 motor_send）
constexpr float DT       = 1.0f / HZ;
constexpr float POS_LIMIT = 1.2f;            // 位置保护 (rad)

// 悬空前提确认
bool confirm_suspended() {
    printf("⚠️ 安全前提：狗必须【吊起/架起】，腿与轮**全部离地**。\n");
    printf("   本示例会施加方波/扫频激励，肢体必然摆动 —— 请清空周围、随时可断电。\n");
    printf("   Ctrl+C 硬急停；|位置| > %.1f rad 自动中止。\n", POS_LIMIT);
    printf("准备好后按回车开始（先按 Ctrl+C 可取消）: ");
    fflush(stdout);
    int c; while ((c = getchar()) != '\n' && c != EOF) {}
    return !g_rl_stop;
}

bool init_all(MotorManager& mm, ThreadManager& tm) {
    mm.SetTransport(&Usb2CanTransport::GetInstance());
    if (!mm.Initialize(tm)) { printf("[ERROR] MotorManager 初始化失败\n"); return false; }
    tm.start_thread("motor_receive");
    tm.start_thread("motor_send");
    sleep(1);

    // 腿 IMPEDANCE（激励用位置/力矩）、轮 SPEED（速度环激励）——与 Ex36/37 同约定
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++) mm.SetControlMode(cp, mi, IMPEDANCE);
    for (int cp = 0; cp < 4; cp++) mm.SetControlMode(cp, 4, SPEED);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++) mm.PreEnableZeroTorque(cp, mi);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++) mm.EnableMotor(cp, mi);
    usleep(300000);

    // 轮子先给 0 速（弱增益软启动，避免使能瞬间假速度偏移）
    for (int cp = 0; cp < 4; cp++) mm.SendSpeed(cp, 4, 0.0f, rl::WHEEL_SOFT_KVP, 0.0f);
    return true;
}

// 所有腿回到零扭矩自由、轮子 0 速
void all_free(MotorManager& mm) {
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) mm.SendImpedance(cp, mi, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        mm.SendSpeed(cp, 4, 0.0f, rl::WHEEL_SOFT_KVP, 0.0f);
    }
}

// 位置保护：任一被激励关节越限 → true
bool any_over_limit(MotorManager& mm, int cp, int mi) {
    return fabsf(mm.GetStatus(cp, mi).position) > POS_LIMIT;
}

// 读 IMU 并注入数据集（每拍调用；IMU 自身 100 Hz，数据集里为采样保持）
void pump_imu(ImuDevice* imu) {
    if (!imu) return;
    float g[3], q[4];
    imu->GetGyro(g[0], g[1], g[2]);
    imu->GetQuat(q[0], q[1], q[2], q[3]);
    S2RDataset::inst().SetImu(g, q);
}

int ask_int(const char* prompt, int lo, int hi, int def) {
    printf("%s [%d~%d，默认 %d]: ", prompt, lo, hi, def);
    fflush(stdout);
    char line[32];
    if (!fgets(line, sizeof line, stdin)) return def;
    int v = def;
    if (sscanf(line, "%d", &v) != 1) return def;
    if (v < lo || v > hi) return def;
    return v;
}

// 单关节方波（力矩脉冲或位置阶跃）。period_ms = 半周期长度，cycles = 循环数。
void run_square(MotorManager& mm, ImuDevice* imu, int cp, int mi,
                bool torque_mode, float amp, int half_ms, int cycles) {
    const int half = half_ms * HZ / 1000;
    const int total = half * 4 * cycles;   // 4 个半周期 = 1 个完整循环 (+/0/−/0)
    const float base = mm.GetStatus(cp, mi).position;
    for (int k = 0; k < total && !g_rl_stop; k++) {
        const int phase = (k / half) % 4;               // 0:+  1:0  2:−  3:0
        const float s = (phase == 0) ? +1.0f : (phase == 2) ? -1.0f : 0.0f;
        if (torque_mode)
            mm.SendImpedance(cp, mi, base, 0.0f, 0.0f, 0.0f, s * amp);
        else
            mm.SendImpedance(cp, mi, base + s * amp, 0.0f, 250.0f, 10.0f, 0.0f);
        pump_imu(imu);
        if (any_over_limit(mm, cp, mi)) { printf("[WARN] 位置越限，中止\n"); break; }
        usleep(1000000 / HZ);
    }
    mm.SendImpedance(cp, mi, base, 0.0f, 0.0f, 0.0f, 0.0f);
}

// 单关节位置 chirp
void run_chirp(MotorManager& mm, ImuDevice* imu, int cp, int mi,
               float amp, float f0, float f1, float dur_s) {
    const int total = (int)(dur_s * HZ);
    const float base = mm.GetStatus(cp, mi).position;
    for (int k = 0; k < total && !g_rl_stop; k++) {
        const float t = k * DT;
        // φ(t) = 2π(f0·t + ½·(f1−f0)/T·t²)  —— 线性扫频的相位积分
        const float phase = 2.0f * (float)M_PI *
            (f0 * t + 0.5f * (f1 - f0) / dur_s * t * t);
        mm.SendImpedance(cp, mi, base + amp * sinf(phase), 0.0f, 250.0f, 10.0f, 0.0f);
        pump_imu(imu);
        if (any_over_limit(mm, cp, mi)) { printf("[WARN] 位置越限，中止\n"); break; }
        usleep(1000000 / HZ);
    }
    mm.SendImpedance(cp, mi, base, 0.0f, 0.0f, 0.0f, 0.0f);
}

// 轮速伺服：阶跃序列 + chirp（SPEED 模式）
void run_wheel_servo(MotorManager& mm, ImuDevice* imu, int cp) {
    const float steps[] = {0.0f, 1.0f, 2.0f, 5.0f, 0.0f, -1.0f, -2.0f, -5.0f, 0.0f};
    for (float v : steps) {
        for (int k = 0; k < HZ && !g_rl_stop; k++) {           // 每档 1 s
            mm.SendSpeed(cp, 4, v, rl::WHEEL_KVP, rl::WHEEL_KVI);
            pump_imu(imu);
            usleep(1000000 / HZ);
        }
        printf("  [轮速阶跃] ω_des = %+.1f rad/s\n", v);
    }
    // 扫频 0.1 → 20 Hz，幅值 2 rad/s，8 s
    const float amp = 2.0f, f0 = 0.1f, f1 = 20.0f, dur = 8.0f;
    const int total = (int)(dur * HZ);
    for (int k = 0; k < total && !g_rl_stop; k++) {
        const float t = k * DT;
        const float phase = 2.0f * (float)M_PI *
            (f0 * t + 0.5f * (f1 - f0) / dur * t * t);
        mm.SendSpeed(cp, 4, amp * sinf(phase), rl::WHEEL_KVP, rl::WHEEL_KVI);
        pump_imu(imu);
        usleep(1000000 / HZ);
    }
    mm.SendSpeed(cp, 4, 0.0f, rl::WHEEL_SOFT_KVP, 0.0f);
}

} // namespace

void Example60_SysIdProbe() {
    printf("\n========== 示例 60：真机激励探针（500 Hz 数据集录制）==========\n");
    printf("[产物] log/dataset_<ts>.csv（188 列：16×(mode+5命令+3反馈+temp+vbus) + IMU + cmd）\n");
    printf("[分析] tool/dataset_health.py / tool/delay_fit.py / tool/wheel_servo_fit.py\n");
    printf("[模式] 1 单关节力矩脉冲  2 单关节位置阶跃  3 单关节位置chirp\n");
    printf("       4 轮速阶跃+扫频   5 全部12关节chirp\n\n");

    const int mode = ask_int("选择模式", 1, 5, 1);
    if (mode <= 3) {
        printf("（模式 1/2/3 针对单个腿关节：motor 1=hip 2=thigh 3=calf）\n");
    } else if (mode == 4) {
        printf("（模式 4 针对单个轮：motor 4）\n");
    }

    MotorManager& mm = MotorManager::GetInstance();
    ThreadManager tm;
    if (!init_all(mm, tm)) return;

    signal(SIGINT, rl_signal_handler);
    g_rl_stop = 0;

    ImuDevice imu;
    bool imu_ok = imu.Initialize("/dev/ttyUSB0", 115200);
    if (imu_ok) imu.SetMount(ImuMount::Z_DOWN_X);
    else printf("[WARN] IMU 未打开，数据集里的 gyro/quat 将为默认值\n");

    if (!confirm_suspended()) { imu.Shutdown(); return; }

    char note[160];
    snprintf(note, sizeof note, "Example60 SysIdProbe mode=%d", mode);
    if (!S2RDataset::inst().Begin(note)) { imu.Shutdown(); return; }
    S2RDataset::inst().Meta("example", "Example60_SysIdProbe");
    // 策略权重版本（追溯"同一策略下的数据"用；换权重时同步改这里）
    S2RDataset::inst().Meta("weight", "iteration_9754.pkl");
    S2RDataset::inst().Meta("mode", std::to_string(mode).c_str());

    all_free(mm);
    usleep(300000);

    if (mode == 1 || mode == 2 || mode == 3) {
        const int cp = ask_int("CAN 口", 0, 3, 0);
        const int mi = ask_int("关节 (1=hip 2=thigh 3=calf)", 1, 3, 2);
        char buf[64];
        snprintf(buf, sizeof buf, "can=%d motor=%d", cp, mi);
        S2RDataset::inst().Meta("target", buf);

        if (mode == 1) {
            const float amp = 2.0f;    // N·m（悬空小幅，见 Ex58 通道校验结论）
            printf("[INFO] 力矩脉冲 ±%.1f N·m，半周期 300 ms × 4 循环（kp=kd=0 纯力矩源）\n", amp);
            run_square(mm, imu_ok ? &imu : nullptr, cp, mi, true, amp, 300, 4);
        } else if (mode == 2) {
            const float amp = 0.08f;   // rad
            printf("[INFO] 位置阶跃 ±%.3f rad，半周期 400 ms × 4 循环（kp=250 kd=10）\n", amp);
            run_square(mm, imu_ok ? &imu : nullptr, cp, mi, false, amp, 400, 4);
        } else {
            printf("[INFO] 位置 chirp 0.1→5 Hz，幅值 0.06 rad，10 s（kp=250 kd=10）\n");
            run_chirp(mm, imu_ok ? &imu : nullptr, cp, mi, 0.06f, 0.1f, 5.0f, 10.0f);
        }
    } else if (mode == 4) {
        const int cp = ask_int("CAN 口（轮子在该路的 motor 4）", 0, 3, 0);
        char buf[32]; snprintf(buf, sizeof buf, "can=%d wheel", cp);
        S2RDataset::inst().Meta("target", buf);
        printf("[INFO] 轮速阶跃 0/±1/±2/±5 rad/s（各 1 s）+ chirp 0.1→20 Hz 幅值 2 rad/s 8 s\n");
        printf("[INFO] kvp=%.2f ki=%.3f（rl::WHEEL_KVP/KVI）\n", rl::WHEEL_KVP, rl::WHEEL_KVI);
        run_wheel_servo(mm, imu_ok ? &imu : nullptr, cp);
    } else {
        printf("[INFO] 全部 12 个腿关节逐个 chirp：0.5→15 Hz，幅值 0.04 rad，每个 8 s（总约 100 s）\n");
        for (int cp = 0; cp < 4 && !g_rl_stop; cp++) {
            for (int mi = 1; mi <= 3 && !g_rl_stop; mi++) {
                printf("  → CAN%d motor%d\n", cp, mi);
                run_chirp(mm, imu_ok ? &imu : nullptr, cp, mi, 0.04f, 0.5f, 15.0f, 8.0f);
            }
        }
    }

    all_free(mm);
    usleep(200000);
    S2RDataset::inst().Finish();

    printf("\n[INFO] 数据 → %s\n", S2RDataset::inst().path());
    printf("[下一步] python3 tool/dataset_health.py %s\n", S2RDataset::inst().path());
    printf("         python3 tool/delay_fit.py    %s --auto\n", S2RDataset::inst().path());
    if (mode == 4)
        printf("         python3 tool/wheel_servo_fit.py %s --wheel 0\n", S2RDataset::inst().path());

    printf("\n[INFO] 全部零扭矩/0 速，正在失能...\n");
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++) mm.DisableMotor(cp, mi);
    signal(SIGINT, SIG_DFL);
    if (imu_ok) imu.Shutdown();
    tm.stop_thread("motor_receive");
    tm.stop_thread("motor_send");
    mm.Stop();
    printf("[INFO] 示例60 完成\n");
}


// =====================================================================
//  示例 62：PACE 式真机辨识数据采集（12 腿关节同时位置 chirp）
//
//  依据：Bjelonic/Tischhauser/Hutter, IJRR 2026 (arXiv 2509.06342)
//    §2.1 数据采集：固定 base、无接触、**所有关节同时**做 chirp、**加在关节位置目标层**、PD 跟踪；
//                   序列 20~60 s；日志 400~10000 Hz；**故意用低增益**。
//    §2.2 待辨识：p = [Ia, d, τf, q̃b(每关节), Td(全局)]ᵀ ∈ R^(4n+1)，n=12 → 49 个参数。
//    目标：时均关节位置平方误差（Eq.3），开环回放同一位置目标 ⇒ 无相位漂移。
//
//  ⚠️ 为什么不用 MotorManager 的 500 Hz 发送节拍直接录：
//     ThreadManager 的 interval 是**整数毫秒**，无法表达 400 Hz 的 2.5 ms；
//     且 PACE 要求"一步仿真 = 一个样本"的**严格 400 Hz 等距**时间基。
//     故本示例**自己持有 400 Hz 循环**并写专属 CSV（发送线程仍以 500 Hz 重发同一目标，不影响正确性）。
//
//  产物：log/pace/chirp_<时间戳>.csv
//     列：epoch, t_actual_s, des_LF_HAA..des_RH_KFE(12), meas_LF_HAA..meas_RH_KFE(12)
//     · 列序 = PACE 的 joint_order（LF/RF/LH/RH × HAA/HFE/KFE，**按腿分组**），
//       与本框架 CAN 序（FL/FR/RL/RR × hip/thigh/calf，也按腿分组）**一一对应**，无需重排。
//     · t_actual_s 仅作诊断；导出时用 **理想等距网格** `arange(N)/400`（PACE 要求仿真速率一致），
//       并校验实际周期偏差 —— 超限应判该次采集无效。
// =====================================================================
void Example62_PaceChirpCollect() {
    printf("\n========== 示例 62：PACE 式真机辨识采集（全关节同时位置 chirp）==========\n");

    // ---------------- 可调参数（改这里即可）----------------
    constexpr float DURATION_S = 20.0f;    // 序列时长（PACE：20~60 s）
    constexpr float F0_HZ      = 0.1f;     // 起始频率
    constexpr float F1_HZ      = 10.0f;    // 终止频率（PACE 官方例程上限）
    constexpr float AMP_RAD    = 0.06f;    // chirp 幅值（保守；现场按机械余量确认）
    constexpr float KP         = 50.0f;    // ⚠ 论文明确要求**低增益**（ANYmal 用 85/0.6；Go2W 部署用 50/1.0）
    constexpr float KD         = 1.0f;
    constexpr int   HZ         = 400;      // 辨识采样/控制频率（PACE 整机日志用 400 Hz）
    constexpr float POS_LIMIT  = 2.5f;     // status 坐标的硬保护（thigh 在 DEFAULT_POSE 就有 ~-1.16）
    constexpr float DEV_LIMIT  = 0.5f;     // 偏离 chirp 偏置超过该值即中止（真正有意义的保护）
    // ------------------------------------------------------

    const int N = (int)(DURATION_S * HZ);
    printf("[参数] 时长 %.0fs | chirp %.2f→%.2f Hz | 幅值 %.3f rad | kp/kd %.0f/%.1f | %d Hz | %d 拍\n",
           DURATION_S, F0_HZ, F1_HZ, AMP_RAD, KP, KD, HZ, N);

    printf("\n⚠️ 前置条件（PACE §2.1，缺一不可）：\n");
    printf("   1) 狗必须**刚性固定**（吊带会摆动 ⇒ 违反 fixed-base 前提；有夹具则夹紧）\n");
    printf("   2) 机身尽量水平；**轮子与腿全部离地**、无任何接触（含腿间）\n");
    printf("   3) 固件补偿开关（cogging/摩擦补偿）在整个辨识与后续部署期间**保持一致**\n");
    printf("   4) kp/kd 与后续部署**完全相同**（PACE 禁止把 PD 增益与动力学联合优化）\n");
    printf("准备好后按回车开始（Ctrl+C 取消）: ");
    fflush(stdout);
    { int c; while ((c = getchar()) != '\n' && c != EOF) {} }
    if (g_rl_stop) { printf("[INFO] 已取消\n"); return; }

    MotorManager& mm = MotorManager::GetInstance();
    ThreadManager tm;
    if (!init_all(mm, tm)) return;          // 复用 ex_probe 的初始化（腿 IMPEDANCE / 轮 SPEED 0 速）
    signal(SIGINT, rl_signal_handler);
    g_rl_stop = 0;

    // ---------- 初始姿态：用 DEFAULT_POSE 作为 chirp 的偏置（= PACE 的 trajectory_bias）----------
    // 注意：不是"起立"，本示例假设狗已被固定、腿悬空；直接把腿 PD 拉到 DEFAULT_POSE 附近。
    float base[12];
    for (int leg = 0; leg < 4; leg++)
        for (int j = 0; j < 3; j++) base[leg * 3 + j] = rl::DEFAULT_POSE[leg * 3 + j];
    // ⚠️ 坐标约定（2026-10-05 修的一个真 bug）：
    //   `rl::DEFAULT_POSE` / 观测 / 动作 / 仿真 都在 **URDF 约定**；
    //   而 `SendImpedance` 收的是 **GetStatus(status) 约定**，两者差一个
    //   `URDF = CONV_A·status + CONV_B`（thigh 还要**反号**且差 ~0.96 rad、calf 差 ~1.28 rad）。
    //   所以：**下发前必须 urdf_to_status，记录时必须 status_to_urdf**，
    //   否则一来拉到的姿态完全不对（thigh 会差 ~78°），二来导出的 .pt 不在仿真坐标系里。
    printf("\n[1/3] 把 12 个腿关节 PD 拉到 DEFAULT_POSE（3 s）...\n");
    for (int k = 0; k < 3 * HZ && !g_rl_stop; k++) {
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++) {
                const int i = leg * 3 + j;
                mm.SendImpedance(leg, j + 1, rl::urdf_to_status(base[i], i), 0, KP, KD, 0);
            }
        usleep(1000000 / HZ);
    }
    for (int leg = 0; leg < 4; leg++)
        for (int j = 0; j < 3; j++) {
            const int i = leg * 3 + j;
            // 用实测（转回 URDF）作为 chirp 偏置，比标称值更稳
            base[i] = rl::status_to_urdf(mm.GetStatus(leg, j + 1).position, i);
        }
    printf("      偏置（URDF 坐标，实测 joint_order）: ");
    for (int i = 0; i < 12; i++) printf("%.3f ", base[i]);
    printf("\n");

    // ---------- 建 CSV ----------
    ::mkdir("log", 0755); ::mkdir("log/pace", 0755);
    char ts[32], path[128];
    { time_t now = time(nullptr); strftime(ts, sizeof ts, "%Y%m%d_%H%M%S", localtime(&now)); }
    snprintf(path, sizeof path, "log/pace/chirp_%s.csv", ts);
    FILE* f = fopen(path, "w");
    if (!f) { printf("[ERROR] 无法创建 %s\n", path); return; }
    const char* jn[12] = {"LF_HAA","LF_HFE","LF_KFE","RF_HAA","RF_HFE","RF_KFE",
                          "LH_HAA","LH_HFE","LH_KFE","RH_HAA","RH_HFE","RH_KFE"};
    fprintf(f, "epoch,t_actual_s");
    for (int i = 0; i < 12; i++) fprintf(f, ",des_%s", jn[i]);
    for (int i = 0; i < 12; i++) fprintf(f, ",meas_%s", jn[i]);
    fprintf(f, "\n");
    // meta（PACE 复现必需的现场信息）
    FILE* fm = fopen((std::string(path) + ".meta.txt").c_str(), "w");
    if (fm) {
        fprintf(fm, "source=Example62_PaceChirpCollect\n");
        fprintf(fm, "duration_s=%.3f\nf0_hz=%.3f\nf1_hz=%.3f\namp_rad=%.4f\n", DURATION_S, F0_HZ, F1_HZ, AMP_RAD);
        fprintf(fm, "kp=%.2f\nkd=%.2f\nrate_hz=%d\n", KP, KD, HZ);
        fprintf(fm, "joint_order=");
        for (int i = 0; i < 12; i++) fprintf(fm, "%s%s", jn[i], i < 11 ? "," : "");
        fprintf(fm, "\nnote=固定基座/离地/固件补偿开关 由操作者填写\n");
        fclose(fm);
    }

    // ---------- 主循环：400 Hz 严格等距 ----------
    printf("\n[2/3] 开始 chirp（%.0f s @ %d Hz）... Ctrl+C 中止\n", DURATION_S, HZ);
    const auto t0 = std::chrono::steady_clock::now();
    int written = 0; double sum_dt = 0; double max_dt = 0; int aborted = 0;
    auto prev = t0;
    for (int k = 0; k < N && !g_rl_stop; k++) {
        const float t = (float)k / HZ;                       // 理想等距时间
        const float ph = 2.0f * (float)M_PI *
                         (F0_HZ * t + (F1_HZ - F0_HZ) / (2.0f * DURATION_S) * t * t);
        const float w = sinf(ph);
        // 全关节同时、同一 chirp（PACE 官方例程亦如此），偏置 = base 实测
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++) {
                const int i = leg * 3 + j;
                // 下发：URDF 目标 → status（见上面坐标约定说明）
                mm.SendImpedance(leg, j + 1, rl::urdf_to_status(base[i] + AMP_RAD * w, i), 0, KP, KD, 0);
            }
        // 记录（同一拍内先下发再读反馈）
        const auto now = std::chrono::steady_clock::now();
        const double t_act = std::chrono::duration<double>(now - t0).count();
        fprintf(f, "%d,%.6f", k, t_act);
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++)
                fprintf(f, ",%.6f", base[leg * 3 + j] + AMP_RAD * w);
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++) {
                const int i = leg * 3 + j;
                const float p_status = mm.GetStatus(leg, j + 1).position;
                const float p_urdf   = rl::status_to_urdf(p_status, i);   // 记录用 URDF 坐标
                fprintf(f, ",%.6f", p_urdf);
                // 保护：偏离偏置过多、或 status 绝对值失控 ⇒ 中止
                if (fabsf(p_urdf - base[i]) > DEV_LIMIT || fabsf(p_status) > POS_LIMIT) aborted = 1;
            }
        fprintf(f, "\n");
        written++;
        // 周期统计（诊断）
        const double dt = std::chrono::duration<double>(now - prev).count();
        prev = now; sum_dt += dt; if (dt > max_dt) max_dt = dt;
        if (aborted) { printf("\n[WARN] 位置越限（>%.2f rad），中止\n", POS_LIMIT); break; }
        usleep(1000000 / HZ);
    }
    fclose(f);

    // ---------- 收尾 ----------
    printf("[3/3] 结束：回偏置姿态、失能\n");
    for (int k = 0; k < HZ && !g_rl_stop; k++) {
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++) {
                const int i = leg * 3 + j;
                mm.SendImpedance(leg, j + 1, rl::urdf_to_status(base[i], i), 0, KP, KD, 0);
            }
        usleep(1000000 / HZ);
    }
    all_free(mm);
    printf("\n========== 采集结果 ==========\n");
    printf("  有效拍数: %d / %d（%s）\n", written, N, aborted ? "有越限中止" : "完整");
    if (written > 1) {
        const double mean_dt = sum_dt / (written - 1);
        printf("  实际平均周期: %.6f s（目标 %.6f，偏差 %+.3f%%）  最大单拍 %.6f s\n",
               mean_dt, 1.0 / HZ, (mean_dt - 1.0 / HZ) / (1.0 / HZ) * 100.0, max_dt);
        printf("  ⚠ 判据：平均周期偏差应 <1%%，且无长于 3 倍周期的大跳（否则该次采集作废重采）\n");
    }
    printf("  CSV: %s\n", path);
    printf("  下一步: python3 tool/pace_export_dataset.py %s   （导出 PACE .pt）\n", path);

    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++) mm.DisableMotor(cp, mi);
    signal(SIGINT, SIG_DFL);
    tm.stop_thread("motor_receive");
    tm.stop_thread("motor_send");
    mm.Stop();
    printf("[INFO] 示例62 完成\n");
}
