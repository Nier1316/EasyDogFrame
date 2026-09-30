/**
 * @file    s2r_dataset.cpp
 * @brief   统一 sim2real 数据集录制器实现（见 s2r_dataset.h）
 */
#include "common/s2r_dataset.h"

#include <chrono>
#include <cstring>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>

namespace {

int64_t steady_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

} // namespace

S2RDataset& S2RDataset::inst() {
    static S2RDataset r;
    return r;
}

S2RDataset::~S2RDataset() { Finish(); }

int64_t S2RDataset::WallMsNow() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch()).count();
}

void S2RDataset::WriteHeader() {
    fputs("wall_ms,t_ms", m_fp);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",c_mode_%02d", i);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",c_pos_%02d", i);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",c_vel_%02d", i);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",c_kp_%02d", i);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",c_kd_%02d", i);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",c_tau_%02d", i);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",m_pos_%02d", i);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",m_vel_%02d", i);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",m_tau_%02d", i);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",m_temp_%02d", i);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",m_vbus_%02d", i);
    fputs(",gyro_0,gyro_1,gyro_2,quat_w,quat_x,quat_y,quat_z"
          ",cmd_vx,cmd_vy,cmd_wz\n", m_fp);
}

bool S2RDataset::Begin(const char* note) {
    if (m_active.load()) return true;

    ::mkdir("log", 0755);
    char stamp[32];
    time_t now = time(nullptr);
    strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S", localtime(&now));
    snprintf(m_path, sizeof m_path, "log/dataset_%s.csv", stamp);
    snprintf(m_metapath, sizeof m_metapath, "log/dataset_%s.meta.txt", stamp);

    m_fp = fopen(m_path, "w");
    if (!m_fp) { printf("[S2R-DS][ERROR] 无法创建 %s\n", m_path); return false; }
    // 1 MB 全缓冲：把 500 Hz 的写系统调用摊薄
    static char iobuf[1 << 20];
    setvbuf(m_fp, iobuf, _IOFBF, sizeof iobuf);

    m_meta = fopen(m_metapath, "w");

    WriteHeader();

    m_head.store(0);
    m_tail.store(0);
    m_rows.store(0);
    m_dropped.store(0);
    m_t0_us.store(steady_us());

    if (m_meta) {
        fprintf(m_meta, "# S2R 统一数据集（500 Hz：指令 + 反馈 + 共时间戳）\n");
        fprintf(m_meta, "start_ts=%s\n", stamp);
        if (note) fprintf(m_meta, "note=%s\n", note);
        fprintf(m_meta, "freq=500Hz (每 2ms 一行，由 SendOnce 触发)\n");
        fprintf(m_meta, "motor_order=can (i = can*4 + motor_id-1: 00=FL-hip..03=FL-wheel)\n");
        fprintf(m_meta, "coord=标定后统一坐标（= GetStatus / SendImpedance 的坐标）\n");
        fprintf(m_meta, "c_semantics=IMPEDANCE(0):pos/vel/kp/kd/tau_ff | SPEED(1):- /vel/kvp/-/ki | POSITION(2):pos/-/kvp/kp/kvi\n");
        fprintf(m_meta, "imu=Z_DOWN_X (gyro 机体系 rad/s; quat w,x,y,z body<-world)\n");
        fprintf(m_meta, "vbus=1Hz 轮询每路 motor1; temp=1Hz 轮询全部16个; 未轮询到记 0\n");
        fprintf(m_meta, "align=wall_ms 与 sim2sim --record / log/rl_*.csv 的 wall_ms 可直接对齐\n");
        // 计数块：定宽（%-20llu），以便录制中途就地覆盖刷新而不留残字
        m_meta_counter_off = ftell(m_meta);
        fprintf(m_meta, "rows_now=%-20d\ndropped_now=%-20d\n", 0, 0);
        fflush(m_meta);
    }

    m_active.store(true);
    m_running.store(true);
    m_writer = std::thread(&S2RDataset::WriterLoop, this);

    printf("[S2R-DS] 数据集录制开启 → %s\n", m_path);
    printf("[S2R-DS] 列数 188（16 电机 × (mode+5命令+3反馈+temp+vbus) + IMU + cmd）；"
           "meta → %s\n", m_metapath);
    return true;
}

void S2RDataset::WriteRow(const DatasetSample& s) {
    fprintf(m_fp, "%lld,%.3f", (long long)s.wall_ms, (double)s.t_us / 1000.0);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",%d", (int)s.mode[i]);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",%.5g", s.c_pos[i]);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",%.5g", s.c_vel[i]);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",%.5g", s.c_kp[i]);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",%.5g", s.c_kd[i]);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",%.5g", s.c_tau[i]);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",%.5g", s.m_pos[i]);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",%.5g", s.m_vel[i]);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",%.5g", s.m_tau[i]);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",%.4g", s.m_temp[i]);
    for (int i = 0; i < s2rd::NUM_MOTORS; i++) fprintf(m_fp, ",%.4g", s.m_vbus[i]);
    fprintf(m_fp, ",%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.4g,%.4g,%.4g\n",
            s.gyro[0], s.gyro[1], s.gyro[2],
            s.quat[0], s.quat[1], s.quat[2], s.quat[3],
            s.cmd[0], s.cmd[1], s.cmd[2]);
}

void S2RDataset::WriterLoop() {
    uint64_t since_flush = 0;
    while (m_running.load(std::memory_order_acquire)) {
        size_t t = m_tail.load(std::memory_order_relaxed);

        if (t == m_head.load(std::memory_order_acquire)) {
            // 队列空：等一小会（控制线程会在 Push 后 notify）
            std::unique_lock<std::mutex> lk(m_cv_mtx);
            m_cv.wait_for(lk, std::chrono::milliseconds(20),
                          [&] { return m_head.load() != m_tail.load() || !m_running.load(); });
            continue;
        }

        while (t != m_head.load(std::memory_order_acquire)) {
            WriteRow(m_buf[t]);
            t = (t + 1) % kCap;
            m_tail.store(t, std::memory_order_release);
            m_rows.fetch_add(1, std::memory_order_relaxed);
            if (++since_flush >= 500) {        // 每秒 flush 一次
                fflush(m_fp);
                FlushCounters();               // 同时刷新 meta 里的 rows/dropped
                since_flush = 0;
            }
        }
    }

    // drain 剩余
    size_t t = m_tail.load();
    while (t != m_head.load()) {
        WriteRow(m_buf[t]);
        t = (t + 1) % kCap;
        m_tail.store(t);
        m_rows.fetch_add(1, std::memory_order_relaxed);
    }
}

void S2RDataset::Push(const DatasetSample& in) {
    if (!m_active.load(std::memory_order_acquire)) return;

    DatasetSample s = in;                    // 一次拷贝（~0.6 KB，无分配）
    s.t_us    = (uint64_t)(steady_us() - m_t0_us.load(std::memory_order_relaxed));
    s.wall_ms = WallMsNow();
    {
        std::lock_guard<std::mutex> lk(m_snap_mtx);
        s.gyro[0] = m_gyro[0]; s.gyro[1] = m_gyro[1]; s.gyro[2] = m_gyro[2];
        s.quat[0] = m_quat[0]; s.quat[1] = m_quat[1];
        s.quat[2] = m_quat[2]; s.quat[3] = m_quat[3];
        s.cmd[0]  = m_cmd[0];  s.cmd[1]  = m_cmd[1];  s.cmd[2]  = m_cmd[2];
    }

    const size_t h    = m_head.load(std::memory_order_relaxed);
    const size_t next = (h + 1) % kCap;
    if (next == m_tail.load(std::memory_order_acquire)) {
        // 队列满：丢弃并计数（绝不阻塞控制环）
        m_dropped.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    m_buf[h] = s;
    m_head.store(next, std::memory_order_release);
    m_cv.notify_one();
}

void S2RDataset::SetImu(const float gyro[3], const float quat[4]) {
    std::lock_guard<std::mutex> lk(m_snap_mtx);
    for (int i = 0; i < 3; i++) m_gyro[i] = gyro[i];
    for (int i = 0; i < 4; i++) m_quat[i] = quat[i];
}

void S2RDataset::SetCmd(const float cmd[3]) {
    std::lock_guard<std::mutex> lk(m_snap_mtx);
    for (int i = 0; i < 3; i++) m_cmd[i] = cmd[i];
}

void S2RDataset::Meta(const char* key, const char* value) {
    if (!m_meta || !key) return;
    std::lock_guard<std::mutex> lk(m_meta_mtx);
    fseek(m_meta, 0, SEEK_END);
    fprintf(m_meta, "%s=%s\n", key, value ? value : "");
    fflush(m_meta);
}

void S2RDataset::FlushCounters() {
    if (!m_meta) return;
    std::lock_guard<std::mutex> lk(m_meta_mtx);
    fseek(m_meta, m_meta_counter_off, SEEK_SET);
    fprintf(m_meta, "rows_now=%-20llu\ndropped_now=%-20llu\n",
            (unsigned long long)m_rows.load(std::memory_order_relaxed),
            (unsigned long long)m_dropped.load(std::memory_order_relaxed));
    fflush(m_meta);
    fseek(m_meta, 0, SEEK_END);   // 复位，保证后续 Meta()/Finish() 都是追加
}

void S2RDataset::Finish() {
    if (!m_active.load()) return;
    m_active.store(false);
    m_running.store(false);
    m_cv.notify_all();
    if (m_writer.joinable()) m_writer.join();

    if (m_fp) {
        fflush(m_fp);
        fclose(m_fp);
        m_fp = nullptr;
    }
    if (m_meta) {
        fseek(m_meta, 0, SEEK_END);            // FlushCounters 可能把位置留在计数块
        fprintf(m_meta, "rows=%llu\ndropped=%llu\n",
                (unsigned long long)m_rows.load(), (unsigned long long)m_dropped.load());
        fclose(m_meta);
        m_meta = nullptr;
    }
    printf("[S2R-DS] 录制结束 → %s（%llu 行，丢弃 %llu）\n", m_path,
           (unsigned long long)m_rows.load(), (unsigned long long)m_dropped.load());
    if (m_dropped.load() > 0)
        printf("[S2R-DS][WARN] 有 %llu 行因写不过来被丢弃 —— 检查磁盘/缓冲，"
               "或缩短单次录制时长。\n", (unsigned long long)m_dropped.load());
}
