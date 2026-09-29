// =====================================================================
//  系统辨识 / 通道校验示例（Example58 / Example59）
//  详见 include/app/examples/ex_sysid.h 的「分工」说明。
//  共同前提：狗必须**吊起/架起**（腿悬空），否则重力矩/自由摆动都不成立。
// =====================================================================
#include "app/examples/ex_sysid.h"
#include "app/examples_common.h"
#include "transport/usb2can_transport.h"
#include "motor/motor_manager.h"
#include "motor/motor_calibration.h"
#include "runtime/thread_manager.h"
#include "common/log_control.h"
#include "motion/robot_calibration.h"
#include "motion/leg_kinematics.h"
#include "strategy/imu_device.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <sys/stat.h>
#include <unistd.h>

using logctl::LogCat;

namespace {

// ---------------------------------------------------------------- 小工具

void wait_ms(int ms) { usleep(ms * 1000); }

// 等待回车（清空行缓冲；Ctrl+C 也能退出）
void wait_enter(const char* prompt) {
    printf("%s", prompt);
    fflush(stdout);
    int c;
    while ((c = getchar()) != '\n' && c != EOF) {}
}

// 读一个浮点（空行 → 用默认值）
float read_float(const char* prompt, float def) {
    printf("%s [默认 %.3f]: ", prompt, def);
    fflush(stdout);
    char line[64];
    if (!fgets(line, sizeof line, stdin)) return def;
    if (line[0] == '\n' || line[0] == '\0') return def;
    float v = def;
    if (sscanf(line, "%f", &v) != 1) return def;
    return v;
}

// 时间戳字符串
void make_stamp(char* out, size_t n) {
    time_t now = time(nullptr);
    strftime(out, n, "%Y%m%d_%H%M%S", localtime(&now));
}

// 确保 log/sysid 目录存在
void ensure_log_dir() {
    ::mkdir("log", 0755);
    ::mkdir("log/sysid", 0755);
}

// 3×3 线性方程组（高斯消元，列主元）
bool solve3(float A[3][3], float b[3], float x[3]) {
    float M[3][4];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) M[i][j] = A[i][j];
        M[i][3] = b[i];
    }
    for (int col = 0; col < 3; col++) {
        int piv = col;
        for (int r = col + 1; r < 3; r++)
            if (fabsf(M[r][col]) > fabsf(M[piv][col])) piv = r;
        if (fabsf(M[piv][col]) < 1e-12f) return false;
        if (piv != col)
            for (int j = col; j < 4; j++) { float t = M[piv][j]; M[piv][j] = M[col][j]; M[col][j] = t; }
        for (int r = 0; r < 3; r++) {
            if (r == col) continue;
            float f = M[r][col] / M[col][col];
            for (int j = col; j < 4; j++) M[r][j] -= f * M[col][j];
        }
    }
    for (int i = 0; i < 3; i++) x[i] = M[i][3] / M[i][i];
    return true;
}

// 电机初始化：达妙 USB2CAN + 收发线程 + 阻抗模式（零扭矩预置）+ 使能
bool init_zero_torque(MotorManager& mm, ThreadManager& tm) {
    mm.SetTransport(&Usb2CanTransport::GetInstance());
    if (!mm.Initialize(tm)) { printf("[ERROR] MotorManager 初始化失败\n"); return false; }
    tm.start_thread("motor_receive");
    tm.start_thread("motor_send");
    sleep(1);

    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++) mm.SetControlMode(cp, mi, IMPEDANCE);
    wait_ms(100);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++) mm.PreEnableZeroTorque(cp, mi);
    wait_ms(100);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++) mm.EnableMotor(cp, mi);
    wait_ms(300);
    return true;
}

void shutdown_all(MotorManager& mm, ThreadManager& tm) {
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++) mm.DisableMotor(cp, mi);
    signal(SIGINT, SIG_DFL);
    tm.stop_thread("motor_receive");
    tm.stop_thread("motor_send");
    mm.Stop();
}

// 采样 (θ, τ) 均值；返回 false 表示越限/中止
bool sample_theta_tau(MotorManager& mm, uint8_t cp, uint8_t mi, int ms,
                      float pos_limit, float* theta, float* tau, float* tau_sd, int* n) {
    double s_theta = 0, s_tau = 0, s_tau2 = 0;
    int cnt = 0;
    for (int i = 0; i < ms && !g_rl_stop; i++) {
        MotorStatus st = mm.GetStatus(cp, mi);
        if (fabsf(st.position) > pos_limit) return false;
        s_theta += st.position;
        s_tau   += st.torque;
        s_tau2  += st.torque * st.torque;
        cnt++;
        wait_ms(1);
    }
    if (cnt < 10) return false;
    *theta  = (float)(s_theta / cnt);
    *tau    = (float)(s_tau / cnt);
    float var = (float)(s_tau2 / cnt) - (*tau) * (*tau);
    *tau_sd = var > 0.0f ? sqrtf(var) : 0.0f;
    *n      = cnt;
    return true;
}

} // namespace

// =====================================================================
//  示例 58：16 路力矩通道校验
// =====================================================================
void Example58_TorqueChannelCheck() {
    printf("\n========== 示例 58：力矩通道校验（零偏 / 增益 / 符号 / 线性度）==========\n");
    printf("[目的] 在用 τ 反解外力（接触力/GRF/摩擦）之前，先证明 τ 通道可信。\n");
    printf("[原理] 阻抗模式 τ = kp(θ_des−θ) + kd(ω_des−ω) + τ_ff；令 kp=kd=0 ⇒ τ = τ_ff，\n");
    printf("       电机成为纯开环力矩源，回读帧的 τ 应≈τ_ff。\n");
    printf("[做法] 逐电机施加 0 → +T1 → +T2 → 0 → −T1 → −T2 → 0，每档稳定后采样均值。\n");
    printf("\n");
    printf("⚠️ 安全前提：狗必须【吊起/架起】（腿与地面分离）。\n");
    printf("⚠️ 施加力矩时被测量的腿会摆动 —— 请用手【轻扶】被测量的肢体（不要用力顶）。\n");
    printf("⚠️ 位置越限（|θ|>%.1f rad）会立即中止该电机。Ctrl+C 硬急停。\n", 1.2f);
    printf("\n");
    wait_enter("准备好后按回车开始（Ctrl+C 取消）...");
    if (g_rl_stop) { printf("[INFO] 已取消\n"); return; }

    printf("\n是否也测试 4 个轮电机？(y/N)\n");
    printf("  轮子施加力矩会【转起来】（惯量小、摩擦小）；只有你能按住轮子或让轮子抵住地面时才选 y。\n");
    printf("  腿关节单独的力矩方向验证另有 Example34/48。默认只测 12 个腿关节。\n");
    printf("输入 y 回车 = 腿+轮；直接回车 = 仅腿：");
    fflush(stdout);
    char line[16];
    bool test_wheels = false;
    if (fgets(line, sizeof line, stdin) && (line[0] == 'y' || line[0] == 'Y')) test_wheels = true;

    const float POS_LIMIT = 1.2f;
    const float T_LEG[2]   = {2.0f, 5.0f};    // 腿：量程 120/200 N·m，取小值即可
    const float T_WHEEL[2] = {0.5f, 1.0f};    // 轮：量程 52，空载摩擦仅 0.4~0.8，取更小值

    MotorManager& mm = MotorManager::GetInstance();
    ThreadManager tm;
    if (!init_zero_torque(mm, tm)) return;

    signal(SIGINT, rl_signal_handler);
    g_rl_stop = 0;

    ensure_log_dir();
    char stamp[32];
    make_stamp(stamp, sizeof stamp);
    char csv_path[160];
    snprintf(csv_path, sizeof csv_path, "log/sysid/torque_check_%s.csv", stamp);
    FILE* csv = fopen(csv_path, "w");
    if (csv) fprintf(csv, "can,motor,level0,read_p1,read_p2,read_0b,read_n1,read_n2,sd_max,ok\n");

    int n_pass = 0, n_test = 0;
    printf("\n%-6s %-8s %8s %8s %8s %8s %8s %7s  %s\n",
           "电机", "量程Nm", "零偏", "+T1回读", "+T2回读", "−T1回读", "−T2回读", "最大σ", "判定");
    printf("---------------------------------------------------------------------------\n");

    for (int cp = 0; cp < 4 && !g_rl_stop; cp++) {
        for (int mi = 1; mi <= 4 && !g_rl_stop; mi++) {
            if (mi == 4 && !test_wheels) continue;
            const bool is_wheel = (mi == 4);
            const float* T = is_wheel ? T_WHEEL : T_LEG;
            const float levels[7] = {0.0f, T[0], T[1], 0.0f, -T[0], -T[1], 0.0f};
            float read[7];
            float sdmax = 0.0f;
            bool ok = true;

            for (int k = 0; k < 7; k++) {
                if (g_rl_stop) { ok = false; break; }
                // kp=kd=0 ⇒ 固件按 τ = τ_ff 输出
                mm.SendImpedance(cp, mi, 0.0f, 0.0f, 0.0f, 0.0f, levels[k]);
                wait_ms(is_wheel ? 150 : 300);      // 建立（轮的惯量小，用更短时间以限制转速）
                float th, tau, sd;
                int n;
                if (!sample_theta_tau(mm, cp, mi, is_wheel ? 150 : 250,
                                      POS_LIMIT, &th, &tau, &sd, &n)) { ok = false; break; }
                read[k] = tau;
                if (sd > sdmax) sdmax = sd;
            }
            mm.SendImpedance(cp, mi, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);   // 复位为零扭矩

            // ---- 判定 ----
            // 增益：|回读 − 指令| ≤ max(0.5, 25%·|指令|)
            // 符号：正指令回读为正、负指令回读为负
            bool gain_ok = true, sign_ok = true;
            if (ok) {
                for (int s = 0; s < 2; s++) {
                    float tol = fmaxf(0.5f, 0.25f * T[s]);
                    if (fabsf(read[1 + s] - T[s]) > tol) gain_ok = false;
                    if (fabsf(read[4 + s] + T[s]) > tol) gain_ok = false;
                }
                if (!(read[1] > 0.0f && read[2] > 0.0f)) sign_ok = false;
                if (!(read[4] < 0.0f && read[5] < 0.0f)) sign_ok = false;
            }
            bool pass = ok && gain_ok && sign_ok && fabsf(read[0]) < 0.5f;
            n_test++;
            if (pass) n_pass++;

            printf("CAN%d-%d %8.0f %8.3f %8.3f %8.3f %8.3f %8.3f %7.3f  %s%s%s%s\n",
                   cp, mi, is_wheel ? TORQUE_CMD_LIMIT[MOTOR_WHEEL] : TORQUE_CMD_LIMIT[mi - 1],
                   read[0], read[1], read[2], read[4], read[5], sdmax,
                   pass ? "PASS" : "WARN",
                   (!ok) ? " [采样中止]" : "",
                   (ok && !gain_ok) ? " [增益偏差]" : "",
                   (ok && !sign_ok) ? " [符号错]" : "");

            if (csv)
                fprintf(csv, "%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n",
                        cp, mi, read[0], read[1], read[2], read[3], read[4], read[5], sdmax,
                        ok && pass ? 1 : 0);
        }
    }

    if (csv) { fclose(csv); printf("\n[INFO] 明细 → %s\n", csv_path); }

    printf("\n========== 汇总 ==========\n");
    printf("  测试电机 %d 个，通过 %d 个\n", n_test, n_pass);
    printf("  判据：|τ_回读 − τ_指令| ≤ max(0.5, 25%%·|τ_指令|) 且符号一致 且 |零偏| < 0.5 N·m\n");
    if (n_pass < n_test)
        printf("  ⚠️ 有未通过项：先查该电机的 0x60 量程寄存器（Example24）、线束、以及是否被外力顶住；\n"
               "     通道不可信时不要用 τ 做任何反向估计（接触力/摩擦/重力）。\n");
    else
        printf("  ✅ 全部通过：τ 通道可用于接触力/摩擦/重力估计的输入。\n");
    printf("  下一步：Example59（重力矩系数与质量-质心）。\n");

    printf("\n[INFO] 全部零扭矩，正在失能...\n");
    shutdown_all(mm, tm);
    printf("[INFO] 示例58 完成\n");
}

// =====================================================================
//  示例 59：重力矩系数（m·g·d）与质量-质心辨识
// =====================================================================
void Example59_GravityMassIdentify() {
    printf("\n========== 示例 59：重力矩系数与质量-质心辨识 ==========\n");
    printf("[目的] 填 robot_calibration.h 里仍是 TODO 的 LINK_DYNAMICS / BODY_MASS。\n");
    printf("       这是「用 τ 反解接触力 f = (Jᵀ)⁻¹τ」的前提：不扣掉自重，\n");
    printf("       摩擦(1.6~6.2 N·m)×J 条件数(≈15) 会造成几十牛的力误差。\n");
    printf("[模型] 只让一个关节转动、其余关节锁定 ⇒ 该关节绕自身轴刚性转动，\n");
    printf("       重力矩 τ_g(θ) = m·g·d·sin(θ+φ) = a·sinθ + b·cosθ。\n");
    printf("       对 [sinθ, cosθ, 1] 做线性最小二乘即得：\n");
    printf("         G_j = √(a²+b²) = m·g·d（重力矩系数幅值，**不需要扫到力臂最大处**）\n");
    printf("[做法] 每个关节在参考姿态附近双向慢扫 7 个角度（共 14 点），\n");
    printf("       每点稳定后读 (θ, τ) 均值 → 拟合 → 由称重质量反推 d 与绕质心惯量。\n");
    printf("[分工] J/B/f_c 由 Example47 辨识、腿摩擦由 Example54 辨识，本示例不重复。\n");
    printf("\n");
    printf("⚠️ 安全前提：狗必须【吊起/架起】（腿悬空），机身尽量【水平】。\n");
    printf("⚠️ 扫描幅度默认 ±0.20 rad、每点 0.6s，很慢；Ctrl+C 硬急停。\n");
    printf("⚠️ 建议先跑 Example58 确认力矩通道可信，否则本示例结果无意义。\n");
    printf("\n");
    wait_enter("准备好后按回车开始（Ctrl+C 取消）...");
    if (g_rl_stop) { printf("[INFO] 已取消\n"); return; }

    // ---- 称重输入 ----
    printf("\n---- 台秤称重（用于把 G_j 换算成质心距离 d）----\n");
    printf("单位 kg。留空=用默认值；填 0 = 跳过该量（只输出 G_j，不反推 d）。\n");
    float m_body  = read_float("机身（不含腿）质量", 0.0f);
    float m_thigh = read_float("单条大腿组件质量（大腿+其电机）", 0.0f);
    float m_calf  = read_float("单条小腿组件质量（小腿+轮+轮电机）", 0.0f);
    float J47     = read_float("Example47 测得的 J（绕关节轴，kg·m²；未知填 0）", 0.0f);

    const float SWEEP   = 0.20f;   // 单侧扫描幅度 (rad)
    const int   N_STEP  = 7;       // 单向点数
    const float HOLD_KP = 250.0f;  // 锁定/定位刚度（与 JOINT_IMPEDANCE 同量级）
    const float HOLD_KD = 10.0f;
    const float GRAV    = 9.81f;
    const float POS_LIMIT = 2.0f;

    MotorManager& mm = MotorManager::GetInstance();
    ThreadManager tm;
    if (!init_zero_torque(mm, tm)) return;

    signal(SIGINT, rl_signal_handler);
    g_rl_stop = 0;

    // ---- 可选 IMU：检查机身是否水平 ----
    ImuDevice imu;
    bool imu_ok = imu.Initialize("/dev/ttyUSB0", 115200);
    if (imu_ok) {
        imu.SetMount(ImuMount::Z_DOWN_X);
        wait_ms(300);
        float q[4];
        imu.GetQuat(q[0], q[1], q[2], q[3]);
        // projected_gravity = world2self(quat, [0,0,-1])
        float g[3];
        const float down[3] = {0.0f, 0.0f, -1.0f};
        {
            const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
            const float s = 2.0f * qw * qw - 1.0f;
            const float cx = qy * down[2] - qz * down[1];
            const float cy = qz * down[0] - qx * down[2];
            const float cz = qx * down[1] - qy * down[0];
            const float w2 = qw * 2.0f;
            const float d  = (qx * down[0] + qy * down[1] + qz * down[2]) * 2.0f;
            g[0] = down[0] * s - cx * w2 + qx * d;
            g[1] = down[1] * s - cy * w2 + qy * d;
            g[2] = down[2] * s - cz * w2 + qz * d;
        }
        printf("\n[IMU] projected_gravity = (%.3f, %.3f, %.3f)（水平应为 0,0,−1）\n", g[0], g[1], g[2]);
        float tilt = acosf(fmaxf(-1.0f, fminf(1.0f, -g[2]))) * 180.0f / (float)M_PI;
        printf("[IMU] 机身倾角 ≈ %.1f°%s\n", tilt,
               tilt > 10.0f ? "  ⚠️ 超过 10°：重力矩会被投影缩短，d 的换算有偏差，建议垫平后重跑" : "");
    } else {
        printf("\n[WARN] IMU 未打开，跳过水平检查（请自行确认机身水平）\n");
    }

    ensure_log_dir();
    char stamp[32];
    make_stamp(stamp, sizeof stamp);
    char sum_path[160];
    snprintf(sum_path, sizeof sum_path, "log/sysid/gravity_summary_%s.csv", stamp);
    FILE* fsum = fopen(sum_path, "w");
    if (fsum) fprintf(fsum, "can,motor,a,b,c,G_Nm,phi_rad,rms_Nm,mass_kg,d_m,d_mm,J_joint,I_com,ok\n");

    // 每个关节的拟合结果
    struct Fit { float a, b, c, G, phi, rms; bool ok; };
    Fit fit[4][3];
    memset(fit, 0, sizeof fit);

    printf("\n%-7s %9s %9s %9s %10s %8s %8s  %s\n",
           "关节", "a", "b", "c(Nm)", "G=m·g·d", "φ(rad)", "残差RMS", "备注");
    printf("----------------------------------------------------------------------------------\n");

    for (int leg = 0; leg < 4 && !g_rl_stop; leg++) {
        // 参考姿态：进入本腿测试时的实际关节角（真机指令角坐标 = leg_fk/GetStatus 同一坐标）
        float ref[3];
        for (int j = 0; j < 3; j++) ref[j] = mm.GetStatus(leg, j + 1).position;

        for (int joint = 0; joint < 3 && !g_rl_stop; joint++) {
            const float base = ref[joint];

            // 双向扫描：上行 + 下行，抑制库仑摩擦造成的迟滞
            const int N_SAMP = 2 * N_STEP;
            float theta[N_SAMP], tau[N_SAMP], sd[N_SAMP];
            bool ok = true;

            for (int pass = 0; pass < 2 && ok; pass++) {
                for (int k = 0; k < N_STEP; k++) {
                    if (g_rl_stop) { ok = false; break; }
                    int step = (pass == 0) ? k : (N_STEP - 1 - k);
                    float frac = (float)step / (float)(N_STEP - 1) * 2.0f - 1.0f;  // −1..+1
                    float target = base + frac * SWEEP;

                    // 三个关节都下发：被测关节到目标，其余关节锁在参考角
                    for (int j = 0; j < 3; j++)
                        mm.SendImpedance(leg, j + 1, (j == joint) ? target : ref[j],
                                         0.0f, HOLD_KP, HOLD_KD, 0.0f);
                    wait_ms(600);   // 准静态稳定（惯性/粘性项 → 0）

                    int idx = pass * N_STEP + k;
                    float th, tq, tqsd;
                    int n;
                    if (!sample_theta_tau(mm, leg, joint + 1, 250, POS_LIMIT,
                                          &th, &tq, &tqsd, &n)) { ok = false; break; }
                    theta[idx] = th; tau[idx] = tq; sd[idx] = tqsd;
                }
            }
            // 被测关节回参考位
            mm.SendImpedance(leg, joint + 1, base, 0.0f, HOLD_KP, HOLD_KD, 0.0f);
            wait_ms(300);

            if (!ok) {
                printf("C%d-%s   采样中止（越限/急停）\n", leg, joint == 0 ? "hip" : joint == 1 ? "thigh" : "calf");
                continue;
            }

            // 越限/无运动检查
            float tmin = theta[0], tmax = theta[0];
            for (int k = 1; k < N_SAMP; k++) {
                if (theta[k] < tmin) tmin = theta[k];
                if (theta[k] > tmax) tmax = theta[k];
            }
            const float range = tmax - tmin;

            // ---- 最小二乘：τ ≈ a·sinθ + b·cosθ + c ----
            float ATA[3][3] = {{0}}, ATb[3] = {0};
            for (int k = 0; k < N_SAMP; k++) {
                float row[3] = {sinf(theta[k]), cosf(theta[k]), 1.0f};
                for (int r = 0; r < 3; r++) {
                    ATb[r] += row[r] * tau[k];
                    for (int cc = 0; cc < 3; cc++) ATA[r][cc] += row[r] * row[cc];
                }
            }
            float x[3];
            if (!solve3(ATA, ATb, x)) {
                printf("C%d-%s   拟合失败（矩阵奇异）\n", leg, joint == 0 ? "hip" : joint == 1 ? "thigh" : "calf");
                continue;
            }
            const float a = x[0], b = x[1], c = x[2];
            const float G = sqrtf(a * a + b * b);
            const float phi = atan2f(b, a);

            // 残差 RMS
            double se = 0;
            for (int k = 0; k < N_SAMP; k++) {
                float pred = a * sinf(theta[k]) + b * cosf(theta[k]) + c;
                se += (pred - tau[k]) * (pred - tau[k]);
            }
            const float rms = (float)sqrt(se / N_SAMP);

            fit[leg][joint] = {a, b, c, G, phi, rms, true};

            // 采样期间的力矩噪声上限（质量指标：σ 大说明该点不稳/有干扰）
            float sd_max = 0.0f;
            for (int k = 0; k < N_SAMP; k++) if (sd[k] > sd_max) sd_max = sd[k];

            // ---- 换算 ----
            const char* jn = (joint == 0) ? "hip" : (joint == 1) ? "thigh" : "calf";
            // 绕该关节轴"刚性转动组件"的质量：hip=整条腿；thigh=大腿+小腿+轮；calf=小腿+轮
            float mass_asm = 0.0f;
            if (joint == 0) mass_asm = m_thigh + m_calf;
            else if (joint == 1) mass_asm = m_thigh + m_calf;
            else mass_asm = m_calf;

            char note[128] = "";
            float d_m = 0.0f, I_com = 0.0f;
            if (mass_asm > 0.01f) {
                d_m = G / (mass_asm * GRAV);
                if (J47 > 0.0f) I_com = J47 - mass_asm * d_m * d_m;
                snprintf(note, sizeof note, "m=%.2fkg d=%.1fmm%s", mass_asm, d_m * 1000.0f,
                         (J47 > 0.0f) ? "" : "（J 未填，I_c 跳过）");
            } else {
                snprintf(note, sizeof note, "未称重 → 只给 G_j");
            }
            if (range < 0.05f) snprintf(note + strlen(note), sizeof note - strlen(note), " ⚠位置几乎不动");
            snprintf(note + strlen(note), sizeof note - strlen(note), " στ≤%.2f", sd_max);

            char jname[16];
            snprintf(jname, sizeof jname, "C%d-%s", leg, jn);
            printf("%-7s %9.3f %9.3f %9.3f %10.3f %8.3f %8.3f  %s\n",
                   jname, a, b, c, G, phi, rms, note);

            if (fsum)
                fprintf(fsum, "%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.4f,%.1f,%.4f,%.4f,%d\n",
                        leg, joint + 1, a, b, c, G, phi, rms,
                        mass_asm, d_m, d_m * 1000.0f, J47, I_com, 1);
        }
    }

    // ---- 汇总与配置建议 ----
    printf("\n========== 汇总 ==========\n");
    printf("重力矩系数 G_j = m·g·d (N·m)：\n");
    printf("%-8s %10s %10s %10s\n", "腿", "hip", "thigh", "calf");
    for (int leg = 0; leg < 4; leg++) {
        printf("CAN%d    ", leg);
        for (int j = 0; j < 3; j++)
            printf("%10.3f ", fit[leg][j].ok ? fit[leg][j].G : 0.0f);
        printf("\n");
    }
    printf("\n⚠️ 上表是**参考姿态附近**测得的幅值；G_j 与姿态无关（同一关节刚性转动），\n"
           "   所以可直接用于 tau_ff 表的量级校核。四条腿同一关节的 G_j 应接近，\n"
           "   若某条腿明显偏离，优先怀疑该腿的力矩通道或机械卡滞。\n");

    // 建议写入配置（先算平均，避免逐腿不同值）
    float G_avg[3] = {0, 0, 0};
    int   G_cnt[3] = {0, 0, 0};
    for (int leg = 0; leg < 4; leg++)
        for (int j = 0; j < 3; j++)
            if (fit[leg][j].ok) { G_avg[j] += fit[leg][j].G; G_cnt[j]++; }
    for (int j = 0; j < 3; j++) if (G_cnt[j]) G_avg[j] /= G_cnt[j];

    printf("\n---- 建议写入 include/motion/robot_calibration.h 的值 ----\n");
    printf("（LINK_DYNAMICS 需要 mass/com/inertia；这里给的是**可直接使用的等效量**）\n");
    if (m_calf > 0.01f) {
        printf("  calf  轴：G=%.3f N·m → mass=%.3f kg, com=%.4f m（从关节轴量起）\n",
               G_avg[2], m_calf, G_avg[2] / (m_calf * GRAV));
        if (J47 > 0.0f) {
            float d = G_avg[2] / (m_calf * GRAV);
            printf("           inertia(绕质心) = J47 − m·d² = %.3f − %.3f·%.4f² = %.4f kg·m²\n",
                   J47, m_calf, d, J47 - m_calf * d * d);
        }
    } else printf("  calf  ：未称重，只记录了 G=%.3f N·m\n", G_avg[2]);
    if (m_thigh + m_calf > 0.01f) {
        printf("  thigh 轴：G=%.3f N·m → 组件 mass=%.3f kg, com=%.4f m\n",
               G_avg[1], m_thigh + m_calf, G_avg[1] / ((m_thigh + m_calf) * GRAV));
    } else printf("  thigh ：未称重，只记录了 G=%.3f N·m\n", G_avg[1]);
    if (m_thigh + m_calf > 0.01f) {
        printf("  hip   轴：G=%.3f N·m → 整腿 mass=%.3f kg, com=%.4f m\n",
               G_avg[0], m_thigh + m_calf, G_avg[0] / ((m_thigh + m_calf) * GRAV));
    } else printf("  hip   ：未称重，只记录了 G=%.3f N·m\n", G_avg[0]);
    printf("  BODY_MASS = %.3f kg（台秤称得；含电池）\n", m_body);
    printf("\n⚠️ 直接把上面的 mass/com 填进 LINK_DYNAMICS 只是**起点**：LINK_DYNAMICS 是逐连杆的\n"
           "   参数，而这里测的是「绕该关节轴刚性转动的整个组件」的等效值。要把组件等效值拆成\n"
           "   逐连杆值，需要已知各连杆质量；建议先按上表填入等效值并在仿真里验证足端力\n"
           "   （leg_foot_force_body 的 f_z 在静止单腿支撑时应 ≈ m·g）。\n");

    if (fsum) { fclose(fsum); printf("\n[INFO] 汇总 → %s\n", sum_path); }
    printf("[INFO] 单点明细未落盘（本示例只保留拟合结果与残差）；如需原始点可加打印。\n");

    if (imu_ok) imu.Shutdown();
    printf("\n[INFO] 全部零扭矩，正在失能...\n");
    shutdown_all(mm, tm);
    printf("[INFO] 示例59 完成\n");
}
