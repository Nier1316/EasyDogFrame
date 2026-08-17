/**
 * @file   can_device.cpp
 * @brief  CanDevice 的实现：将 CANET 库的 C 风格 VCI_* 接口封装为 RAII 对象
 */
#include "can_device.h"
#include <cstdio>
#include <cstring>
//test commit
// 构造：仅记录设备索引，不做任何 IO；真正打开设备放到 Initialize
CanDevice::CanDevice(uint8_t device_idx)
    : m_device_idx(device_idx), m_is_running(false) {
}

// 析构：确保设备被正确关闭（Stop + CloseDevice），避免泄漏底层句柄
CanDevice::~CanDevice() {
    Shutdown();
}

// 初始化全流程：OpenDevice → ConfigureDevice（含 InitCAN）
// 任一步失败都要回滚（关闭已打开的设备），保证状态一致
bool CanDevice::Initialize(const CanDeviceConfig& config) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!OpenDevice()) {
        printf("[ERROR] Failed to open CAN device %d\n", m_device_idx);
        return false;
    }

    if (!ConfigureDevice(config)) {
        printf("[ERROR] Failed to configure CAN device %d\n", m_device_idx);
        CloseDevice();    // 回滚：关闭已打开的设备
        return false;
    }

    return true;
}

// 启动 CAN 通道：配置好但未启动时，设备无法收发
bool CanDevice::Start() {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (VCI_StartCAN(VCI_CANETE, m_device_idx, 0) != STATUS_OK) {
        printf("[ERROR] Failed to start CAN device %d\n", m_device_idx);
        return false;
    }

    m_is_running = true;
    printf("[INFO] CAN device %d started\n", m_device_idx);
    return true;
}

// 停止 CAN 通道；返回复位是否成功。失败仅告警，仍置 m_is_running=false，
// 以便后续 CloseDevice 继续收尾。
bool CanDevice::Stop() {
    std::lock_guard<std::mutex> lock(m_mutex);

    bool ok = (VCI_ResetCAN(VCI_CANETE, m_device_idx, 0) == STATUS_OK);
    if (!ok) {
        printf("[WARNING] Failed to reset CAN device %d\n", m_device_idx);
    }

    m_is_running = false;
    printf("[INFO] CAN device %d stopped\n", m_device_idx);
    return ok;
}

// 彻底收尾：先 Stop 释放 CAN 通道，再 Close 释放底层句柄
void CanDevice::Shutdown() {
    Stop();
    CloseDevice();
}

// 发送单帧：必须在已 Start 的状态下调用，否则直接拒绝
bool CanDevice::SendFrame(const VCI_CAN_OBJ& frame) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_is_running) {
        printf("[WARNING] CAN device %d is not running\n", m_device_idx);
        return false;
    }

    // VCI_Transmit 返回实际发送的帧数；这里发 1 帧，预期返回 1
    ULONG sent = VCI_Transmit(VCI_CANETE, m_device_idx, 0, (PVCI_CAN_OBJ)&frame, 1);
    return sent == 1;
}

// 批量接收：一次最多读 100 帧；无帧时返回 false 而非阻塞失败
bool CanDevice::ReceiveFrames(std::vector<VCI_CAN_OBJ>& frames, int timeout_ms) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_is_running) {
        return false;
    }

    VCI_CAN_OBJ buffer[100];  // 局部缓冲，避免频繁分配
    ULONG cnt = VCI_Receive(VCI_CANETE, m_device_idx, 0, buffer, 100, timeout_ms);

    if (cnt > 0) {
        frames.clear();       // 清空调用方传入的容器
        for (ULONG i = 0; i < cnt; i++) {
            frames.push_back(buffer[i]);
        }
        return true;
    }

    return false;  // 无帧可读（正常的"超时/空队列"，非错误）
}

// ------------------------------------------------------------------
//            私有辅助：对 CANET VCI_* API 的直接封装
// ------------------------------------------------------------------

// 打开设备：必须在任何其他 VCI_ 调用之前执行
bool CanDevice::OpenDevice() {
    if (VCI_OpenDevice(VCI_CANETE, m_device_idx, 0) != STATUS_OK) {
        return false;
    }
    printf("[INFO] CAN device %d opened\n", m_device_idx);
    return true;
}

// 关闭设备；失败只告警，方便调用方继续执行清理流程
bool CanDevice::CloseDevice() {
    if (VCI_CloseDevice(VCI_CANETE, m_device_idx) != STATUS_OK) {
        printf("[WARNING] Failed to close CAN device %d\n", m_device_idx);
        return false;
    }
    printf("[INFO] CAN device %d closed\n", m_device_idx);
    return true;
}

// 配置 TCP 参数 + 初始化 CAN
// 服务器模式：设置 CMD_SRCPORT（本机监听端口）
// 客户端模式：设置 CMD_DESIP + CMD_DESPORT（远端服务器地址）
bool CanDevice::ConfigureDevice(const CanDeviceConfig& config) {
    // 1) 设置 TCP 工作模式（SERVER / CLIENT）
    DWORD workMode = config.work_mode;
    if (VCI_SetReference(VCI_CANETE, m_device_idx, 0, CMD_TCP_TYPE, &workMode) != STATUS_OK) {
        return false;
    }

    // 2) 按模式设置地址信息
    if (config.work_mode == TCP_SERVER) {
        // 服务器模式：只需要本机监听端口
        DWORD port = config.port;
        if (VCI_SetReference(VCI_CANETE, m_device_idx, 0, CMD_SRCPORT, &port) != STATUS_OK) {
            return false;
        }
        printf("[INFO] CAN device %d configured as TCP server on port %d\n", m_device_idx, port);
    } else {
        // 客户端模式：需要远端 IP 和端口
        if (VCI_SetReference(VCI_CANETE, m_device_idx, 0, CMD_DESIP, (void*)config.server_ip) != STATUS_OK) {
            return false;
        }
        DWORD port = config.port;
        if (VCI_SetReference(VCI_CANETE, m_device_idx, 0, CMD_DESPORT, &port) != STATUS_OK) {
            return false;
        }
        printf("[INFO] CAN device %d configured as TCP client to %s:%d\n",
               m_device_idx, config.server_ip, port);
    }

    // 3) 初始化 CAN 通道（InitConfig 传 NULL 表示使用默认配置）
    if (VCI_InitCAN(VCI_CANETE, m_device_idx, 0, NULL) != STATUS_OK) {
        return false;
    }

    return true;
}
