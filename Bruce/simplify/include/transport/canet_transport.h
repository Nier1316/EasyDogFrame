/**
 * @file    canet_transport.h
 * @brief   CANET TCP 传输后端（L1，CanTransport 的 CANET 实现）
 * @details 原 BspCan：封装 CANET 库调用，为上层提供统一的 CAN 收发接口。
 *          实现 CanTransport 纯虚接口；保留原方法签名（BspCanFrame 为 CanFrame 别名）
 *          以兼容既有调用点。
 */
#ifndef CANET_TRANSPORT_H_
#define CANET_TRANSPORT_H_

#include <cstdint>
#include <vector>
#include <memory>
#include <mutex>
#include <map>
#include "common/types.h"
#include "transport/can_transport.h"
#include "transport/can_device.h"
#include "CANET.h"           // VCI_CAN_OBJ 等类型定义

// 兼容别名：旧调用点用 BspCanFrame 即通用 CanFrame
using BspCanFrame = CanFrame;

/**
 * @class CanetTransport
 * @brief CANET TCP 传输后端（单例），实现 CanTransport 接口
 *        所有 CAN 操作都通过此类进行，隐藏 CANET 库细节
 */
class CanetTransport : public CanTransport {
public:
    // 单例入口
    static CanetTransport& GetInstance();

    // ============ CanTransport 接口实现 ============

    /** open = InitDevice + StartDevice（cfg 取 tcp_ip/tcp_port/tcp_mode） */
    bool open(uint8_t idx, const TransportConfig& cfg) override;

    /** send = SendFrame */
    bool send(uint8_t idx, const CanFrame& f) override;

    /** sendBatch = SendFramesBatch */
    bool sendBatch(uint8_t idx, const CanFrame* f, int n) override;

    /** recv = ReceiveFrames */
    bool recv(uint8_t idx, std::vector<CanFrame>& out, int timeout_ms) override;

    /** close = StopDevice + CloseDevice */
    bool close(uint8_t idx) override;

    /** shutdown = ShutdownAll */
    void shutdown() override;

    // ============ 设备生命周期 ============

    /**
     * 初始化指定 CAN 设备
     * @param device_idx: 设备索引（0-3）
     * @param config: 设备配置（TCP 模式、端口等）
     * @return: 成功返回 true
     */
    bool InitDevice(uint8_t device_idx, const CanDeviceConfig& config);

    /**
     * 启动 CAN 设备（使其能收发）
     * @param device_idx: 设备索引
     * @return: 成功返回 true
     */
    bool StartDevice(uint8_t device_idx);

    /**
     * 停止 CAN 设备
     * @param device_idx: 设备索引
     * @return: 成功返回 true
     */
    bool StopDevice(uint8_t device_idx);

    /**
     * 关闭 CAN 设备（释放资源）
     * @param device_idx: 设备索引
     * @return: 成功返回 true
     */
    bool CloseDevice(uint8_t device_idx);

    // ============ 数据收发 ============

    /**
     * 发送单个 CAN 帧
     * @param device_idx: 目标设备索引
     * @param frame: 帧数据
     * @return: 成功返回 true
     */
    bool SendFrame(uint8_t device_idx, const BspCanFrame& frame);

    /**
     * 便捷发送函数：直接发送数据
     * @param device_idx: 目标设备索引
     * @param can_id: CAN ID
     * @param data: 数据指针（8 字节）
     * @param dlc: 数据长度（0-8）
     * @return: 成功返回 true
     */
    bool Can_Tx(uint8_t device_idx, uint32_t can_id, const uint8_t* data, uint8_t dlc = 8);

    /**
     * 批量接收 CAN 帧
     * @param device_idx: 源设备索引
     * @param frames: 输出容器（调用者负责清空）
     * @param timeout_ms: 阻塞等待时间（毫秒）
     * @return: 有帧返回 true，无帧返回 false
     */
    bool ReceiveFrames(uint8_t device_idx,
                       std::vector<BspCanFrame>& frames,
                       int timeout_ms = 100);

    /**
     * 批量发送多帧（一次 VCI_Transmit，避开 SDK ~10ms 逐帧地板）
     * @param device_idx: 目标设备索引
     * @param frames: 帧数组（最多支持 16 帧）
     * @return: 全部发出返回 true，部分发出返回 false 并打印警告
     */
    bool SendFramesBatch(uint8_t device_idx, const std::vector<BspCanFrame>& frames);

    // ============ 全局控制 ============

    /**
     * 关闭所有设备并释放资源
     */
    void ShutdownAll();

private:
    CanetTransport();
    ~CanetTransport();

    // 禁用拷贝
    CanetTransport(const CanetTransport&) = delete;
    CanetTransport& operator=(const CanetTransport&) = delete;

    // ============ 私有辅助函数：格式转换 ============

    /**
     * 将 BspCanFrame 转换为 VCI_CAN_OBJ
     * @param frame: 应用层帧格式
     * @param vci_frame: 输出的 CANET 库帧格式
     */
    static void ConvertFrameToVci(const BspCanFrame& frame, VCI_CAN_OBJ& vci_frame);

    /**
     * 将 VCI_CAN_OBJ 转换为 BspCanFrame
     * @param vci_frame: CANET 库帧格式
     * @param frame: 输出的应用层帧格式
     */
    static void ConvertFrameFromVci(const VCI_CAN_OBJ& vci_frame, BspCanFrame& frame);

    // 内部实现细节（对外隐藏）
    std::map<uint8_t, std::unique_ptr<CanDevice>> m_devices;
    mutable std::mutex m_mutex;
};

#endif // CANET_TRANSPORT_H_
