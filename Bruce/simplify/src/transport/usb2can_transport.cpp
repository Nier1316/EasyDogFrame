/**
 * @file    usb2can_transport.cpp
 * @brief   达妙 USB2FDCAN 传输后端实现（官方 SDK，多双路模块）
 */
#include "transport/usb2can_transport.h"
#include <cstdio>
#include <cstring>
#include <chrono>
#include <atomic>
#include <cstdlib>   // getenv（DMCAN_RXDIAG）

// 诊断计数
static std::atomic<int> g_u2c_send_fail{0};   // 发送失败计数（仅诊断，成功后清零）

// =====================================================================
//  onRecv 到达间隔探针（2026-10-08）
// =====================================================================
//  目的：判定"CAN 回帧被攒成 ~100 ms 一批"到底发生在哪一层。onRecv 是我们代码
//  第一次看到帧的地方，它前面的整条链（驱动器 → USB → libusb → libdm_device.so 内部
//  的 usb_handle_thread_func / moodycamel 队列 → 回调派发）全是黑箱。
//
//    回调到达间隔 ≈ 2 ms   ⇒ 厂商栈投递正常 ⇒ 瓶颈在我们自己的消费路径
//    回调到达间隔 ≈ 100 ms ⇒ 攒批在 SDK 内部或设备侧 ⇒ 只能换传输层（见 docs）
//
//  开关：环境变量 DMCAN_RXDIAG=1（默认关）。关时只有一次 atomic 读，无开销。
//  输出：Usb2CanTransport::shutdown() 时打印对数分桶直方图。
//  注意：直方图用 relaxed 原子，允许多设备多回调线程并发；诊断时会略微扰动时序，
//        但只影响 onRecv 内部几条指令，不改变攒批结论。
namespace {
std::atomic<uint64_t> g_rx_gap_bucket[24];   // 桶 i：间隔 ∈ [2^(i-1), 2^i) µs
std::atomic<uint64_t> g_rx_gap_total{0};
std::atomic<int>      g_rxdiag_flag{-1};     // -1 未读环境变量 / 0 关 / 1 开

inline bool rxdiag_on() {
    int v = g_rxdiag_flag.load(std::memory_order_relaxed);
    if (v < 0) {
        const char* s = getenv("DMCAN_RXDIAG");
        v = (s && *s && *s != '0') ? 1 : 0;
        g_rxdiag_flag.store(v, std::memory_order_relaxed);
    }
    return v == 1;
}

inline int gap_bucket(uint64_t us) {
    int i = 0;
    while (us > 1 && i < 23) { us >>= 1; ++i; }
    return i;
}

inline int64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

Usb2CanTransport& Usb2CanTransport::GetInstance() {
    static Usb2CanTransport instance;
    return instance;
}

Usb2CanTransport::~Usb2CanTransport() {
    shutdown();
}

// 打开物理设备 dev_idx（find 仅一次，避免 SDK 重复 find 析构崩）
bool Usb2CanTransport::ensureDevice(uint8_t dev_idx, const TransportConfig& cfg) {
    if (m_devices.count(dev_idx) && m_devices[dev_idx]) return true;   // 已打开

    if (!m_ctx) {
        dmcan_context_create(&m_ctx);
        if (!m_ctx) { printf("[Usb2Can] 创建 context 失败\n"); return false; }
    }
    if (!m_found) {
        m_dev_count = dmcan_find_devices(m_ctx);   // 只调一次
        m_found = true;
        printf("[Usb2Can] 发现 %d 个达妙设备\n", m_dev_count);
    }
    if (dev_idx >= (uint8_t)m_dev_count) {
        printf("[Usb2Can] 设备 #%u 超出已发现 %d 个（检查 USB 模块插入）\n", dev_idx, m_dev_count);
        return false;
    }

    dmcan_device_handle* dev = nullptr;
    if (!dmcan_device_get(m_ctx, &dev, dev_idx)) {
        printf("[Usb2Can] 取设备句柄 #%u 失败\n", dev_idx);
        return false;
    }
    if (!dmcan_device_open(dev)) {
        printf("[Usb2Can] 打开设备 #%u 失败\n", dev_idx);
        return false;
    }

    // 双路模块：通道 0/1 都使能 + 设波特率（默认设备预设 1M；cfg.usb_baud 非 0 时改）
    for (uint8_t ch = 0; ch < 2; ch++) {
        dmcan_device_enable_channel(dev, ch);
        dmcan_channel_can_info_t info;
        dmcan_device_get_channel_baudrate(dev, ch, &info);
        if (cfg.usb_baud != 0 && cfg.usb_baud != info.can_baudrate) {
            info.channel = ch;
            info.canfd   = false;
            info.can_baudrate = cfg.usb_baud;
            info.can_sp  = 0.75f;
            dmcan_device_set_channel_baudrate(dev, ch, info);
        }
    }
    dmcan_device_hook_recv_callback(dev, &Usb2CanTransport::onRecv);

    m_devices[dev_idx] = dev;
    m_handle_to_devidx[dev] = dev_idx;
    printf("[INFO] Usb2Can: 设备 #%u 已打开（双通道 0/1）\n", dev_idx);
    return true;
}

bool Usb2CanTransport::open(uint8_t idx, const TransportConfig& cfg) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_channels.count(idx)) return true;   // 已打开（幂等）

    uint8_t dev_idx = idx / 2;   // 逻辑路 → 物理设备
    uint8_t ch      = idx % 2;   // 物理通道
    if (!ensureDevice(dev_idx, cfg)) return false;

    Channel c;
    c.dev_idx = dev_idx;
    c.ch = ch;
    m_channels[idx] = c;
    printf("[INFO] Usb2Can: CAN%u <- 设备 #%u 通道 %u\n", idx, dev_idx, ch);
    return true;
}

bool Usb2CanTransport::send(uint8_t idx, const CanFrame& f) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_channels.find(idx);
    if (it == m_channels.end()) return false;
    auto dit = m_devices.find(it->second.dev_idx);
    if (dit == m_devices.end() || !dit->second) return false;
    bool ok = dmcan_device_send_can(dit->second, it->second.ch, f.id,
                                    /*canfd*/ false, /*ext*/ f.is_extended,
                                    /*rtr*/ false,   /*brs*/ false,
                                    /*dlen*/ f.dlc, (uint8_t*)f.data);
    if (ok) return true;
    if (++g_u2c_send_fail % 20 == 1)
        printf("[Usb2Can] send 失败 (%d) CAN%u id=0x%03X\n", g_u2c_send_fail.load(), idx, f.id);
    return false;
}

bool Usb2CanTransport::sendBatch(uint8_t idx, const CanFrame* f, int n) {
    for (int i = 0; i < n; i++)
        if (!send(idx, f[i])) return false;
    return true;
}

void Usb2CanTransport::pushFrame(uint8_t idx, const CanFrame& f) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_channels[idx].rx.push_back(f);
    m_cv.notify_one();
}

// ⚠ 诊断开关：打印 USB2CAN 收到的原始帧（id + 原始字节），定位"控制回帧不更新观测"问题。
// 每 500ms 打印一次汇总 + 最新一帧内容。已定位根因（recv 只取 1 帧积压滞后），保持关闭。
static bool g_rx_dump = false;

// dmcan 接收回调（SDK 内部线程）：handle→设备索引, frame->head.channel→物理通道 → 逻辑 idx
void Usb2CanTransport::onRecv(dmcan_device_handle* handle, usb_rx_frame_t* frame) {
    RxCount().fetch_add(1, std::memory_order_relaxed);   // 心跳：回调是否还在触发
    if (rxdiag_on()) {            // 诊断：记录本次回调与上一次的到达间隔（每个回调线程各一份）
        static thread_local int64_t last = 0;
        const int64_t t = now_us();
        if (last) g_rx_gap_bucket[gap_bucket((uint64_t)(t - last))].fetch_add(1, std::memory_order_relaxed);
        last = t;
        g_rx_gap_total.fetch_add(1, std::memory_order_relaxed);
    }
    if (!frame) return;

    CanFrame cf;
    cf.id          = frame->head.can_id;
    cf.is_extended = frame->head.ext;
    cf.dlc         = (uint8_t)dmcan_utils_get_len_from_dlc(frame->head.dlc);
    if (cf.dlc > 8) cf.dlc = 8;
    std::memcpy(cf.data, frame->payload, cf.dlc);

    auto& self = GetInstance();
    uint8_t dev_idx = 0;
    {
        std::lock_guard<std::mutex> lock(self.m_mutex);
        auto it = self.m_handle_to_devidx.find(handle);
        if (it != self.m_handle_to_devidx.end()) dev_idx = it->second;
    }
    uint8_t idx = dev_idx * 2 + (frame->head.channel & 1);

    // 限流打印：每 500ms 输出该周期内收到的帧数 + 最新一帧 id/字节。
    // 判读：正常使能后应持续收到 id∈[51,54] 的控制回帧，且 data[1..2]（位置 16bit）随物理运动变化。
    if (g_rx_dump) {
        static std::chrono::steady_clock::time_point g_last = std::chrono::steady_clock::now();
        static int g_cnt = 0;
        static uint32_t g_lid = 0;
        static uint8_t g_l8[8] = {0};
        g_cnt++;
        g_lid = cf.id;
        memcpy(g_l8, cf.data, 8);
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - g_last).count() >= 500) {
            printf("[RX] 500ms %3d帧 | 最新 id=0x%02X(%u) ext=%u dlc=%u | %02X %02X %02X %02X %02X %02X %02X %02X\n",
                   g_cnt, g_lid, g_lid, cf.is_extended, cf.dlc,
                   g_l8[0], g_l8[1], g_l8[2], g_l8[3],
                   g_l8[4], g_l8[5], g_l8[6], g_l8[7]);
            g_cnt = 0;
            g_last = now;
        }
    }

    self.pushFrame(idx, cf);
}

bool Usb2CanTransport::recv(uint8_t idx, std::vector<CanFrame>& out, int timeout_ms) {
    std::unique_lock<std::mutex> lock(m_mutex);
    // ⚠ 不能用 m_channels[idx]：operator[] 会给**未打开的通道**凭空建一个默认条目
    //   （dev_idx=0/ch=0），之后 open(idx) 会据此误判"已打开"而跳过真正打开设备。
    auto it = m_channels.find(idx);
    if (it == m_channels.end()) return false;
    auto& rx = it->second.rx;
    if (rx.empty()) {
        m_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                      [&]() { return !rx.empty(); });
    }
    if (rx.empty()) return false;
    // ⚠ 一次性取空队列（修复"观测恒初值"）：
    //   迁移 USB2CAN 时 recv 曾只取 1 帧队头——高速回帧下旧帧积压，
    //   上层永远解到最早的帧，current_position 停在旧值（电机实际已到位）。
    //   CANET 的 VCI_Receive 一次返回一批缓存帧，无此问题；这里对齐该语义。
    while (!rx.empty()) {
        out.push_back(rx.front());
        rx.pop_front();
    }
    return true;
}

bool Usb2CanTransport::close(uint8_t idx) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_channels.find(idx);
    if (it != m_channels.end()) m_channels.erase(it);
    // 设备保持打开（同一设备两路共享）；shutdown 统一关
    return true;
}

void Usb2CanTransport::shutdown() {
    if (rxdiag_on() && g_rx_gap_total.load() > 1) {
        printf("\n[DMCAN_RXDIAG] onRecv 到达间隔直方图（共 %llu 次回调，总帧数 %llu）\n",
               (unsigned long long)g_rx_gap_total.load(),
               (unsigned long long)RxCount().load());
        for (int i = 1; i < 24; i++) {
            const uint64_t c = g_rx_gap_bucket[i].load();
            if (!c) continue;
            printf("    %7llu µs ~ %7llu µs : %8llu 次%s\n",
                   1ULL << (i - 1), (1ULL << i) - 1, (unsigned long long)c,
                   (i <= 2) ? "   ← 连续投递（健康）" : (i >= 16 ? "   ← ~100 ms 一批（攒批在 SDK/设备侧）" : ""));
        }
        printf("  判读：绝大多数落在 ≈2 ms(桶 9~11) ⇒ 厂商栈正常；落在 ≈100 ms(桶 16~17) ⇒ 只能换传输层\n");
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto& p : m_devices)
         if (p.second) dmcan_device_close(p.second);   // 有 "libusb_transfer_cancelled" 噪音，不崩
    m_devices.clear();
    m_handle_to_devidx.clear();
    m_channels.clear();
    // ⚠ 不调用 dmcan_context_destroy：SDK 析构有 bug
}
