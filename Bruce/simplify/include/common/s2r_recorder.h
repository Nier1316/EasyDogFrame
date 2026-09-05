#ifndef S2R_RECORDER_H
#define S2R_RECORDER_H

/**
 * @file    s2r_recorder.h
 * @brief   RL 遥测自动记录器（sim2real 复盘用，2026-09-05）
 *
 * 用途：每次跑 RL 相关 example（Ex36/37/56 等），自动把主循环每步遥测落盘到
 *       log/rlrun_<时间戳>/trace.csv，主循环退出后打印绘图命令（tool/plot_rlrun.py
 *       手动跑，出髋/腿/轮三张追踪图）。
 *
 * 坐标约定（trace.csv 列，与 Ex56 记录一致）：
 *   - 除 cmd 外全部 POLICY 序、URDF 约定（16 = 12 腿 + 4 轮）
 *   - q_target(urdf)  / 轮下发速度  / q / qd / tau(×CONV_A, thigh=-1)
 *   - wvel(4) 取 vel_policy[12..15]；quat(w,x,y,z)/gyro(体坐标, Z_DOWN_X)
 *   - yaw/roll/pitch 世界系 ZYX 欧拉（自 quat）；yaw_rate = gyro[2]
 *
 * 调用（example 内）：
 *   begin("说明")  → RL 主循环每步 step(...) → 退出处 finish()
 *   输入全 POLICY 序数组，example 在 send 循环里组好 qt/qtv 传入。
 */

#include <cstdio>

class S2RRecorder {
public:
    static S2RRecorder& inst();

    /// 建 log/rlrun_<ts>/ 并开 trace.csv（表头）+ meta.txt。已开启则忽略。note 写入 meta。
    bool begin(const char* note);
    /// 是否已开启（主循环里用，避免每步开销）
    bool active() const { return fp_ != nullptr; }
    const char* dir() const { return dir_; }

    /// 记录一步（50Hz）。输入均 POLICY 序：
    ///   qt  = 12 腿 q_target(urdf) + 轮区 0；qtv = 腿 0 + 4 轮下发速度(policy 12..15)
    ///   pos/vel/tau = 实际（urdf/policy，tau 已 ×CONV_A）；quat[4]/gyro[3] IMU
    void step(int step, int phase, const float cmd[3],
              const float qt[16], const float qtv[16],
              const float pos_policy[16], const float vel_policy[16],
              const float tau_policy[16],
              const float quat[4], const float gyro[3]);

    /// 关文件 + 打印绘图命令（不自动跑，用户手动执行）
    void finish();

private:
    S2RRecorder() : fp_(nullptr) { dir_[0] = 0; }
    ~S2RRecorder() { if (fp_) finish(); }
    S2RRecorder(const S2RRecorder&) = delete;
    S2RRecorder& operator=(const S2RRecorder&) = delete;

    char dir_[256];     // log/rlrun_<ts>
    char trace_[320];   // .../trace.csv
    FILE* fp_;
};

#endif // S2R_RECORDER_H
