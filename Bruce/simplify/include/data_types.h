/**
 * @file    data_types.h
 * @brief   CAN 电机控制框架的通用数据类型定义
 * @details 本文件定义框架使用的公共数据结构与常量：电机状态、CAN 设备配置。
 *          上层模块（CanDevice / MotorManager / RobotApp）依赖本文件。
 */
#ifndef DATA_TYPES_H_
#define DATA_TYPES_H_

#include <cstdint>     // 固定宽度整型：uint8_t / uint16_t 等
#include <cstring>     // memset、memcpy 等内存操作函数
#include "motor_drive/ele_motor_def.h"  // 电机定义（包括 ControlMode）

// =====================================================================
//                           电机状态
// =====================================================================

/**
 * @brief 电机状态结构体
 * @details 由 unpack_frame() 从 CAN 接收帧解析，MotorManager::GetStatus() 汇总返回。
 *          按硬件实际返回帧格式解析（位置/速度/扭矩/错误码）。
 */
struct MotorStatus {
    uint8_t motor_id;   // CAN_ID（data[0] bit3-0）
    bool    enable;     // 使能状态（应用层命令态）
    float   position;   // rad，由 data[1-2] 16位解码
    float   velocity;   // rad/s，由 data[3]高4位+data[5] 12位解码
    float   torque;     // Nm，由 data[3]低4位+data[4] 12位解码
    uint8_t error_code; // 电机错误码（固件原始错误寄存器值，非 0 即故障）

    // 默认构造函数：所有字段清零，表示"无效/未更新"状态
    MotorStatus() : motor_id(0), enable(false),
                    position(0), velocity(0), torque(0), error_code(0) {}
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

#endif // DATA_TYPES_H_
