/**
 * @file    s2r_dataset.h
 * @brief   统一 sim2real 数据集录制器（L0.5 公共工具，500 Hz）
 *
 * 目的：把「我命令了什么 + 实际发生了什么 + 共用同一个时钟」写进**同一行**。
 *       这是把真机数据回馈到仿真训练的唯一充分集：只有指令与反馈在同一时间基准上，
 *       才能把「控制延迟」与「执行器动态」分开（见 docs/SIM2REAL_DATA_FEEDBACK.md）。
 *
 * 与 MotorLogger / S2RRecorder 的分工：
 *   - MotorLogger（log/send_*.csv、log/recv_*.csv）：**分文件**记录发送与接收，
 *     离线要按时间戳 join；且 SEND 默认关。适合人工查帧，不适合回放拟合。
 *   - S2RRecorder（log/rlrun_<ts>/trace.csv）：**50 Hz 策略层**遥测 + 绘图。
 *     执行器与延迟信息在 500 Hz 域里，50 Hz 会把它混叠掉。
 *   - **本录制器**：500 Hz、**指令与反馈同帧同行** + IMU + 电压/温度，专供
 *     「轨迹回放拟合」与「延迟/伺服辨识」（见 tool/dataset_health.py、
 *     tool/delay_fit.py、tool/wheel_servo_fit.py）。
 *
 * 产物：`log/dataset_<时间戳>.csv`（+ 同名 `.meta.txt`）
 *
 * ---- CSV 列（共 188 列，UTF-8，逗号分隔，带表头）----
 *   wall_ms                    系统墙钟 epoch 毫秒（与 sim2sim --record 的 wall_ms 直接对齐）
 *   t_ms                       相对 Begin() 的毫秒（float，3 位小数，单调）
 *   c_mode_00..15              控制模式：0=IMPEDANCE 1=SPEED 2=POSITION
 *   c_pos_00..15  c_vel_..  c_kp_..  c_kd_..  c_tau_..     ← 下发（标定后统一坐标）
 *   m_pos_00..15  m_vel_..  m_tau_..                      ← 最新反馈（同一坐标）
 *   m_temp_00..15              电机温度 °C（1 Hz 轮询；未轮询到为 0）
 *   m_vbus_00..15              母线电压 V（1 Hz 轮询每路 1 号电机；未轮询到为 0）
 *   gyro_0..2  quat_w,x,y,z    IMU（采样保持）
 *   cmd_vx,cmd_vy,cmd_wz       上层速度命令
 *
 *   ⚠ 电机索引 = **CAN 顺序**：i = can_port*4 + (motor_id-1)
 *     → 00=FL-hip 01=FL-thigh 02=FL-calf 03=FL-wheel, 04=FR-…, 08=RL-…, 12=RR-…
 *     （不是 POLICY 顺序；POLICY 映射见 include/strategy/rl_controller.h）
 *
 *   ⚠ `c_*` 的语义随 `c_mode_i` 变化（离线分析必须按 mode 解释）：
 *     mode=0 IMPEDANCE: c_pos=目标位置, c_vel=目标速度, c_kp=kp, c_kd=kd, c_tau=τ_ff
 *     mode=1 SPEED:     c_pos=0,        c_vel=目标速度, c_kp=kvp, c_kd=0,  c_tau=ki
 *     mode=2 POSITION:  c_pos=目标位置, c_vel=0,        c_kp=kvp, c_kd=kp, c_tau=kvi
 *
 * ---- 实时性约定（重要）----
 *   生产者是 500 Hz 的发送线程。**Push 绝不阻塞、绝不分配内存、绝不做 I/O**：
 *   只做一次环形缓冲的拷贝；缓冲满则丢弃并计数（`dropped`），宁可丢数据也不拖慢控制环。
 *   格式化与写盘全部在独立的写线程里完成（1 MB 全缓冲 + 周期 flush）。
 */
#ifndef S2R_DATASET_H
#define S2R_DATASET_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <thread>

namespace s2rd {
constexpr int NUM_MOTORS = 16;
}

/** 一行 = 一个 500 Hz 收发节拍的快照（字段与 CSV 列一一对应） */
struct DatasetSample {
    uint64_t t_us    = 0;            // 相对 Begin() 微秒（由录制器盖章）
    int64_t  wall_ms = 0;            // epoch 毫秒（由录制器盖章）
    uint8_t  mode[s2rd::NUM_MOTORS] = {0};
    float c_pos[s2rd::NUM_MOTORS] = {0}, c_vel[s2rd::NUM_MOTORS] = {0};
    float c_kp[s2rd::NUM_MOTORS]  = {0}, c_kd[s2rd::NUM_MOTORS]  = {0};
    float c_tau[s2rd::NUM_MOTORS] = {0};
    float m_pos[s2rd::NUM_MOTORS] = {0}, m_vel[s2rd::NUM_MOTORS] = {0};
    float m_tau[s2rd::NUM_MOTORS] = {0};
    float m_temp[s2rd::NUM_MOTORS] = {0}, m_vbus[s2rd::NUM_MOTORS] = {0};
    float gyro[3] = {0}, quat[4] = {1, 0, 0, 0};
    float cmd[3]  = {0};
};

class S2RDataset {
public:
    static S2RDataset& inst();

    /** 建 log/dataset_<ts>.csv + meta，启动写线程。已开启则忽略并返回 true。 */
    bool Begin(const char* note);
    /** 停写线程 + drain + 关文件 + 打印统计（丢帧数/行数） */
    void Finish();

    bool active() const { return m_active.load(std::memory_order_acquire); }
    const char* path() const { return m_path; }
    uint64_t rows() const { return m_rows.load(); }
    uint64_t dropped() const { return m_dropped.load(); }

    /**
     * 生产者接口（500 Hz 控制线程调用）：录制器会补上 t_us/wall_ms，
     * 并把最新一次 SetImu/SetCmd 的值并进这一行。**不阻塞**。
     */
    void Push(const DatasetSample& s);

    /** 上层在任意时刻注入最新 IMU / 速度命令（采样保持；线程安全） */
    void SetImu(const float gyro[3], const float quat[4]);
    void SetCmd(const float cmd[3]);
    /** 元数据追加一行 key=value（可在录制中途调用，写进 .meta.txt） */
    void Meta(const char* key, const char* value);

private:
    S2RDataset() = default;
    ~S2RDataset();
    S2RDataset(const S2RDataset&) = delete;
    S2RDataset& operator=(const S2RDataset&) = delete;

    void WriterLoop();
    void WriteRow(const DatasetSample& s);
    void WriteHeader();
    static int64_t WallMsNow();

    static constexpr size_t kCap = 8192;   // 8192 × ~0.6 KB ≈ 5 MB 环形缓冲
    DatasetSample m_buf[kCap];
    std::atomic<size_t> m_head{0};         // 生产者写
    std::atomic<size_t> m_tail{0};         // 消费者读
    std::atomic<bool> m_active{false};
    std::atomic<bool> m_running{false};
    std::atomic<uint64_t> m_rows{0};
    std::atomic<uint64_t> m_dropped{0};

    std::thread m_writer;
    std::mutex m_cv_mtx;
    std::condition_variable m_cv;

    FILE* m_fp = nullptr;
    FILE* m_meta = nullptr;
    char m_path[256] = {0};
    char m_metapath[256] = {0};
    std::atomic<int64_t> m_t0_us{0};       // Begin() 时刻（steady_clock 微秒）

    // 最新 IMU / 命令（采样保持）
    std::mutex m_snap_mtx;
    float m_gyro[3] = {0};
    float m_quat[4] = {1, 0, 0, 0};
    float m_cmd[3]  = {0};
};

#endif // S2R_DATASET_H
