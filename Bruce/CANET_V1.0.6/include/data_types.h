/**
 * @file    data_types.h
 * @brief   CAN 电机控制框架的通用数据类型定义
 * @details 本文件定义了框架中使用的所有公共数据结构、枚举和常量，
 *          包括电机命令、电机状态、CAN 设备配置、电机配置和错误码。
 *          所有上层模块（Motor / CanDevice / MotorManager / MotorController）
 *          都依赖本文件中的定义。
 */
#ifndef DATA_TYPES_H_
#define DATA_TYPES_H_

#include <cstdint>     // 固定宽度整型：uint8_t / uint16_t 等
#include <cstring>     // memset、memcpy 等内存操作函数

// =====================================================================
//                         CAN 帧相关定义
// =====================================================================

/**
 * @brief 电机命令类型枚举
 * @note  每个电机命令帧的第二个字节（data[1]）用于标识命令类型
 */
enum MotorCommandType {
    CMD_SET_SPEED     = 0x01,  // 设置速度（目标转速）
    CMD_SET_TORQUE    = 0x02,  // 设置扭矩（力矩控制）
    CMD_SET_DIRECTION = 0x03,  // 设置方向（正转/反转）
    CMD_STOP          = 0x04,  // 立即停止电机
    CMD_RESET         = 0x05,  // 复位（清错误、回初始状态）
    CMD_QUERY_STATUS  = 0x06   // 主动查询电机状态
};

/**
 * @brief 电机控制命令结构体
 * @details 上层应用通过该结构体向 Motor 模块下发控制指令；
 *          Motor::EncodeCommand() 会将该结构体打包成 CAN 帧的 8 字节数据区。
 */
struct MotorCommand {
    uint8_t  motor_id;      // 电机 ID（1~3，同一 CAN 口内的编号）
    uint8_t  cmd_type;      // 命令类型（MotorCommandType 枚举值）
    uint16_t speed;         // 目标速度（0~65535，高字节在前）
    uint16_t torque;        // 目标扭矩（0~65535，高字节在前）
    uint8_t  direction;     // 旋转方向（0=反向，1=正向）
    uint8_t  reserved[2];   // 保留字节，扩展用，默认填 0

    // 默认构造函数：所有字段初始化为 0，避免读取到未初始化的垃圾数据
    MotorCommand() : motor_id(0), cmd_type(0), speed(0), torque(0), direction(0) {
        memset(reserved, 0, sizeof(reserved));
    }
};

/**
 * @brief 电机状态结构体
 * @details Motor::DecodeStatus() 从 CAN 接收帧中解析出该结构体；
 *          上层应用通过 Motor::GetStatus() 读取最新状态。
 */
struct MotorStatus {
    uint8_t  motor_id;      // 电机 ID（回填，便于上层识别来源）
    uint16_t speed;         // 当前实际速度
    uint16_t torque;        // 当前实际扭矩
    uint8_t  temperature;   // 电机温度（单位：°C）
    uint8_t  error_code;    // 错误码，0 表示正常（见 ErrorCode 枚举）
    uint8_t  state;         // 运行状态：0=停止，1=运行中，2=故障
    uint8_t  reserved;      // 保留字节，扩展用

    // 默认构造函数：所有字段清零，表示"无效/未更新"状态
    MotorStatus() : motor_id(0), speed(0), torque(0), temperature(0),
                    error_code(0), state(0), reserved(0) {}
};

// =====================================================================
//                       CAN 设备（CANET）配置
// =====================================================================

// TCP 工作模式常量
#define TCP_CLIENT  0   // 客户端模式
#define TCP_SERVER  1   // 服务器模式

/**
 * @brief CAN 设备（CANET 转换器）配置
 * @details 每个 CANET 设备通过 TCP 与上位机通信，本结构体描述其连接参数。
 *          服务器模式下只需 port；客户端模式下还需要 server_ip。
 */
struct CanDeviceConfig {
    uint8_t     device_idx;    // CANET 设备索引（对应 can0~can3，取值 0~3）
    uint16_t    port;          // TCP 端口（服务器监听端口或客户端目标端口）
    const char* server_ip;     // 远端服务器 IP（仅客户端模式需要；服务器模式置 nullptr）
    uint8_t     work_mode;     // 工作模式：TCP_SERVER=1 / TCP_CLIENT=0

    CanDeviceConfig() : device_idx(0), port(0), server_ip(nullptr), work_mode(0) {}
};

// =====================================================================
//                           电机配置
// =====================================================================

/**
 * @brief 单个电机的配置信息
 * @details MotorManager 根据该结构体创建 Motor 实例，完成
 *          (CAN 口, 电机 ID) → (发送 CAN ID, 接收 CAN ID) 的映射。
 */
struct MotorConfig {
    uint8_t  can_port;     // 所在 CAN 口（0~3，对应 can0~can3）
    uint8_t  motor_id;     // 同一 CAN 口内的电机编号（1~3）
    uint16_t rx_id;        // 接收帧 CAN ID（电机 → 上位机，如 51/52/53）
    uint16_t tx_id;        // 发送帧 CAN ID（上位机 → 电机，如 1/2/3）

    MotorConfig() : can_port(0), motor_id(0), rx_id(0), tx_id(0) {}

    // 便捷构造函数，一次传入全部配置
    MotorConfig(uint8_t port, uint8_t id, uint16_t rx, uint16_t tx)
        : can_port(port), motor_id(id), rx_id(rx), tx_id(tx) {}
};

// =====================================================================
//                           错误码定义
// =====================================================================

/**
 * @brief 系统统一错误码
 * @note  填入 MotorStatus::error_code 或框架内部日志；0 表示正常。
 */
enum ErrorCode {
    ERR_OK                = 0x00,   // 正常，无错误
    ERR_MOTOR_OVERHEAT    = 0x01,   // 电机过热（温度超限）
    ERR_MOTOR_OVERCURRENT = 0x02,   // 电机过流（电流超限）
    ERR_MOTOR_STALL       = 0x03,   // 电机堵转（扭矩异常 / 负载过大）
    ERR_COMMUNICATION     = 0x04,   // CAN 通信错误（丢帧 / 校验失败）
    ERR_TIMEOUT           = 0x05,   // 命令或响应超时
    ERR_INVALID_PARAM     = 0x06,   // 无效参数（例如速度超范围）
    ERR_DEVICE_OFFLINE    = 0x07    // 设备离线（CANET 未连接或电机失联）
};

#endif // DATA_TYPES_H_
