#include "common/s2r_recorder.h"

#include <cmath>
#include <ctime>
#include <cstdio>
#include <sys/stat.h>
#include <sys/types.h>

// 绘图 python 解释器（本机 MJX 环境有 numpy/matplotlib；其它机器改这一行）
// 也可改成在 PATH 上的 python3（若系统装过 numpy+matplotlib）。
static const char* kPlotPy = "/home/bruce/miniforge3/envs/MJX/bin/python";

S2RRecorder& S2RRecorder::inst() {
    static S2RRecorder r;
    return r;
}

bool S2RRecorder::begin(const char* note) {
    if (fp_) return true;                      // 已开启，忽略重复
    ::mkdir("log", 0755);
    char stamp[64];
    time_t now = time(nullptr);
    strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S", localtime(&now));
    snprintf(dir_, sizeof dir_, "log/rlrun_%s", stamp);
    ::mkdir(dir_, 0755);
    snprintf(trace_, sizeof trace_, "%s/trace.csv", dir_);

    fp_ = fopen(trace_, "w");
    if (!fp_) { printf("[S2R][ERROR] 无法创建 %s\n", trace_); return false; }

    // 表头（列名与 Ex56 记录一致；约定见头文件注释）
    fputs("t_s,step,phase,cmd_vx,cmd_vyaw", fp_);
    for (int i = 0; i < 16; i++) fprintf(fp_, ",qt%d", i);
    for (int i = 0; i < 16; i++) fprintf(fp_, ",qtv%d", i);
    for (int i = 0; i < 16; i++) fprintf(fp_, ",q%d", i);
    for (int i = 0; i < 16; i++) fprintf(fp_, ",qd%d", i);
    for (int i = 0; i < 16; i++) fprintf(fp_, ",trq%d", i);
    for (int i = 0; i < 4;  i++) fprintf(fp_, ",wvel%d", i);
    fputs(",qw,qx,qy,qz,gx,gy,gz,yaw,roll,pitch,yaw_rate\n", fp_);
    fflush(fp_);

    // meta 注记
    char mp[320];
    snprintf(mp, sizeof mp, "%s/meta.txt", dir_);
    FILE* mf = fopen(mp, "w");
    if (mf) {
        fprintf(mf, "# S2R RL telemetry run\n");
        fprintf(mf, "ts=%s\n", stamp);
        if (note) fprintf(mf, "note=%s\n", note);
        fprintf(mf, "freq=50Hz policy\n");
        fprintf(mf, "IMU=Z_DOWN_X (gyro body-frame)\n");
        fprintf(mf, "cols: qt/qtv/q/qd/trq 全 POLICY 序 URDF; trq 腿×CONV_A(thigh=-1); wvel=vel_policy轮\n");
        fprintf(mf, "wheel tau 未含(SPEED固件环); yaw_rate=gyro[2]\n");
        fclose(mf);
    }
    printf("[S2R] 记录开启 → %s/trace.csv%s%s\n", dir_,
           note ? "（" : "", note ? note : "");
    return true;
}

void S2RRecorder::step(int step, int phase, const float cmd[3],
                       const float qt[16], const float qtv[16],
                       const float pos_policy[16], const float vel_policy[16],
                       const float tau_policy[16],
                       const float quat[4], const float gyro[3]) {
    if (!fp_) return;
    const float t_s = step * 0.02f;            // 50Hz

    // quat → 世界系 ZYX 欧拉
    const float qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
    float yaw   = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
    float roll  = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
    float sp    = 2.0f * (qw * qy - qz * qx);
    float pitch = asinf(sp > 1.0f ? 1.0f : (sp < -1.0f ? -1.0f : sp));

    fprintf(fp_, "%.3f,%d,%d,%.4f,%.4f", t_s, step, phase, cmd[0], cmd[2]);
    for (int i = 0; i < 16; i++) fprintf(fp_, ",%.5f", qt[i]);
    for (int i = 0; i < 16; i++) fprintf(fp_, ",%.5f", qtv[i]);
    for (int i = 0; i < 16; i++) fprintf(fp_, ",%.5f", pos_policy[i]);
    for (int i = 0; i < 16; i++) fprintf(fp_, ",%.5f", vel_policy[i]);
    for (int i = 0; i < 16; i++) fprintf(fp_, ",%.4f", tau_policy[i]);
    for (int i = 0; i < 4;  i++) fprintf(fp_, ",%.4f", vel_policy[12 + i]);
    fprintf(fp_, ",%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f\n",
            qw, qx, qy, qz, gyro[0], gyro[1], gyro[2], yaw, roll, pitch, gyro[2]);
}

void S2RRecorder::finish() {
    if (!fp_) return;
    fclose(fp_);
    fp_ = nullptr;
    printf("[S2R] 记录结束 → %s/trace.csv\n", dir_);
    printf("[S2R] 绘图(手动): %s tool/plot_rlrun.py %s\n", kPlotPy, trace_);
}
