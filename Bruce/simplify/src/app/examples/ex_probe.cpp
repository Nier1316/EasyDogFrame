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
