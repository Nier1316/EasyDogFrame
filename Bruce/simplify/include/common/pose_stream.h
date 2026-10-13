/**
 * @file    pose_stream.h
 * @brief   姿态流：把 16 个 URDF 关节角 + IMU 姿态通过 UDP 发给可视化工具（L0 公共工具，调试用）
 *
 * 用途：让 `tool/pose_view.py`（Python + MuJoCo）**实时**显示狗的姿态。
 *
 * 为什么必须由本进程往外发：达妙 USB2CAN 的**同一个适配器同一时刻只能被一个进程占用**
 * （SDK 用 libusb claim 接口并 detach 内核驱动），所以可视化进程没法自己去读 CAN。
 *
 * 设计要点：
 *   · **UDP + 非阻塞**：viewer 没开也完全不影响机器人；发送永不阻塞控制环；
 *     丢帧无所谓（viewer 永远只用最新一帧）。
 *   · **默认关闭**：环境变量 `POSE_STREAM=1` 才创建 socket。不开时 `send()` 只有一次
 *     bool 判断，零开销、零行为变化。
 *   · 发送的是**URDF 坐标 + POLICY 关节序**（12 腿 + 4 轮），即 MuJoCo 模型期望的那一层；
 *     坐标换算在调用方完成（见 `rl::status_to_urdf`），本文件不做任何物理换算。
 *
 * 环境变量：
 *   POSE_STREAM=1        启用（其它值/未设 = 关闭）
 *   POSE_HOST=127.0.0.1  目标地址（默认本机）
 *   POSE_PORT=45555      目标端口（默认 45555，与 pose_view.py 一致）
 *   POSE_HZ=100          发送上限（默认 100 Hz；rlStep 本身 50 Hz，故通常不触发限流）
 *
 * 报文格式（**改动必须同步 tool/pose_view.py 的 POSE_FMT**）：
 *   Python 侧: '<IIIIQ4f3f16f12f' = 164 字节
 *   偏移  类型       字段
 *     0   uint32    magic = 0x444F4750 ('DOGP')
 *     4   uint32    ver   = 1
 *     8   uint32    seq   帧序号（接收端据此统计丢包）
 *    12   uint32    flags bit0=quat 有效 / bit1=gyro 有效 / bit2=q_target 有效
 *    16   uint64    t_us  发送端 steady_clock 微秒
 *    24   float[4]  quat  体→世界 (w,x,y,z)，与 MuJoCo free joint 同约定
 *    40   float[3]  gyro  机体系 rad/s
 *    52   float[16] q     实际关节角 (URDF, POLICY 序：12 腿 + 4 轮)
 *   116   float[12] q_tgt 指令目标 (URDF, POLICY 序：仅 12 腿；供以后做"幽灵"叠加)
 */
#ifndef POSE_STREAM_H_
#define POSE_STREAM_H_

#include <arpa/inet.h>      // inet_pton / htons
#include <fcntl.h>          // fcntl（兜底设非阻塞）
#include <netinet/in.h>     // sockaddr_in
#include <sys/socket.h>     // socket / sendto
#include <unistd.h>         // close

#include <chrono>
#include <cstddef>          // offsetof
#include <cstdint>
#include <cstdio>
#include <cstdlib>          // getenv / atoi
#include <cstring>

namespace pose_stream {

constexpr uint32_t kMagic = 0x444F4750u;   // 'DOGP'
constexpr uint32_t kVer   = 1u;
constexpr size_t   kBytes = 164;           // 真实发送长度（结构体尾部可能有对齐填充，不用 sizeof）

constexpr uint32_t kFlagQuat   = 1u << 0;
constexpr uint32_t kFlagGyro   = 1u << 1;
constexpr uint32_t kFlagTarget = 1u << 2;

/** 线上布局。⚠ 各字段偏移必须与 Python 的 struct 格式严格一致（下面有 static_assert 兜底）。 */
struct Packet {
    uint32_t magic;
    uint32_t ver;
    uint32_t seq;
    uint32_t flags;
    uint64_t t_us;
    float    quat[4];
    float    gyro[3];
    float    q[16];
    float    q_target[12];
};
static_assert(offsetof(Packet, ver)      ==   4, "报文布局变了，请同步 pose_view.py");
static_assert(offsetof(Packet, seq)      ==   8, "报文布局变了，请同步 pose_view.py");
static_assert(offsetof(Packet, flags)    ==  12, "报文布局变了，请同步 pose_view.py");
static_assert(offsetof(Packet, t_us)     ==  16, "报文布局变了，请同步 pose_view.py");
static_assert(offsetof(Packet, quat)     ==  24, "报文布局变了，请同步 pose_view.py");
static_assert(offsetof(Packet, gyro)     ==  40, "报文布局变了，请同步 pose_view.py");
static_assert(offsetof(Packet, q)        ==  52, "报文布局变了，请同步 pose_view.py");
static_assert(offsetof(Packet, q_target) == 116, "报文布局变了，请同步 pose_view.py");

/**
 * @brief 姿态流单例。首次使用时按环境变量初始化。
 *
 * 线程安全约定：外部约定**只从 RL 控制环那一个线程调 send()**（与 MotorManager 的使用方式一致），
 * 内部因此不加锁 —— 加锁会拖累 50 Hz 控制环，而这里本来也不需要多写者。
 */
class Stream {
public:
    static Stream& inst() {
        static Stream s;
        return s;
    }

    /** 是否已启用（未设 POSE_STREAM 时为 false，send() 会立刻返回） */
    bool enabled() const { return fd_ >= 0; }

    const char* host() const { return host_; }
    int port() const { return port_; }

    /**
     * @brief 发一帧姿态
     * @param q16_urdf  16 个关节角（**URDF 坐标 + POLICY 序**：12 腿 + 4 轮）
     * @param quat      体→世界四元数 (w,x,y,z)；nullptr 表示无效
     * @param gyro      机体系角速度 rad/s；nullptr 表示无效
     * @param q_tgt12   12 个腿的目标角（URDF/POLICY 序）；nullptr 表示不带
     *
     * 不阻塞、不分配、失败静默（viewer 没开是常态）。超出 POSE_HZ 则丢弃本帧。
     */
    void send(const float q16_urdf[16], const float* quat, const float* gyro,
              const float* q_tgt12 = nullptr) {
        if (fd_ < 0) return;

        const auto now = std::chrono::steady_clock::now();
        if (min_interval_us_ > 0) {
            const int64_t us = std::chrono::duration_cast<std::chrono::microseconds>(
                                   now.time_since_epoch()).count();
            if (last_us_ && us - last_us_ < min_interval_us_) return;
            last_us_ = us;
        }

        Packet p{};
        p.magic = kMagic;
        p.ver   = kVer;
        p.seq   = seq_++;
        p.flags = 0;
        if (quat) { p.flags |= kFlagQuat; std::memcpy(p.quat, quat, sizeof p.quat); }
        if (gyro) { p.flags |= kFlagGyro; std::memcpy(p.gyro, gyro, sizeof p.gyro); }
        std::memcpy(p.q, q16_urdf, sizeof p.q);
        if (q_tgt12) {
            p.flags |= kFlagTarget;
            std::memcpy(p.q_target, q_tgt12, sizeof p.q_target);
        }
        p.t_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count());

        // 非阻塞 UDP：失败（含 viewer 未监听导致的 ECONNREFUSED）一律忽略
        ssize_t rc = ::sendto(fd_, &p, kBytes, 0,
                              reinterpret_cast<const sockaddr*>(&dst_), sizeof dst_);
        (void)rc;
        sent_++;
    }

    /** 已发送帧数（诊断用） */
    uint64_t sent() const { return sent_; }

private:
    Stream() {
        const char* en = std::getenv("POSE_STREAM");
        if (!en || !*en || *en == '0') return;      // 默认关闭：fd_ 保持 -1

        const char* h = std::getenv("POSE_HOST");
        host_ = (h && *h) ? h : "127.0.0.1";
        const char* ps = std::getenv("POSE_PORT");
        port_ = (ps && *ps) ? std::atoi(ps) : 45555;
        const char* hz = std::getenv("POSE_HZ");
        const int rate = (hz && *hz) ? std::atoi(hz) : 100;
        min_interval_us_ = (rate > 0) ? (1000000 / rate) : 0;

        std::memset(&dst_, 0, sizeof dst_);
        dst_.sin_family = AF_INET;
        dst_.sin_port   = htons(static_cast<uint16_t>(port_));
        if (::inet_pton(AF_INET, host_, &dst_.sin_addr) != 1) {
            printf("[PoseStream] 🔴 POSE_HOST=%s 不是合法 IPv4，姿态流未启用\n", host_);
            return;
        }

        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            printf("[PoseStream] 🔴 socket() 失败，姿态流未启用\n");
            return;
        }
        // 非阻塞：宁可丢帧，绝不阻塞控制环
        int fl = ::fcntl(fd_, F_GETFL, 0);
        if (fl >= 0) ::fcntl(fd_, F_SETFL, fl | O_NONBLOCK);

        printf("[PoseStream] ✅ 姿态流已启用 → udp://%s:%d（上限 %d Hz，%zu 字节/帧）"
               "  接收端：tool/pose_view.py --udp %d\n",
               host_, port_, rate, kBytes, port_);
    }

    ~Stream() {
        if (fd_ >= 0) ::close(fd_);
    }

    Stream(const Stream&) = delete;
    Stream& operator=(const Stream&) = delete;

    int             fd_ = -1;
    int             port_ = 45555;
    const char*     host_ = "127.0.0.1";
    sockaddr_in     dst_{};
    int64_t         min_interval_us_ = 0;
    int64_t         last_us_ = 0;
    uint32_t        seq_ = 0;
    uint64_t        sent_ = 0;
};

/** 便捷入口 */
inline Stream& instance() { return Stream::inst(); }

}  // namespace pose_stream

#endif  // POSE_STREAM_H_
