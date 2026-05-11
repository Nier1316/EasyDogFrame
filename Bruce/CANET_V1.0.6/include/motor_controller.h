/**
 * @file    motor_controller.h
 * @brief   面向应用层的高级电机控制器
 * @details MotorController 是应用代码直接使用的门面（Facade）类：
 *          - 隐藏 MotorManager 的复杂性，提供语义化的 Move/Stop/Reset 接口
 *          - 内置默认的 CANET 设备配置（4 个 TCP 服务器，端口 4001~4004）
 *          - 封装错误处理策略（过热 → 停机，堵转 → 复位等）
 *          - 提供友好的状态打印工具
 *          一般情况下，应用层只需要依赖这一个类。
 */
#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include <cstdint>
#include "motor_manager.h"
#include "data_types.h"

class MotorController {
public:
    MotorController();
    ~MotorController();  // 析构时自动 Stop，防止遗留线程

    // ---------------- 系统生命周期 ----------------
    bool Initialize();                 // 构造默认 CAN 配置并初始化 MotorManager
    bool Start();                      // 启动 MotorManager（必须先 Initialize）
    bool Stop();                       // 停止并释放
    bool IsRunning() const;            // 当前是否在运行

    // ---------------- 高级控制接口（推荐应用层使用）----------------
    // 以指定速度和方向转动电机（封装 CMD_SET_SPEED）
    bool MoveMotor(uint8_t can_port, uint8_t motor_id, uint16_t speed, uint8_t direction);
    // 设置扭矩（力矩模式）
    bool SetMotorTorque(uint8_t can_port, uint8_t motor_id, uint16_t torque);
    bool StopMotor(uint8_t can_port, uint8_t motor_id);   // 停止单个电机
    bool StopAllMotors();                                 // 广播停止全部电机（紧急停机）
    bool ResetMotor(uint8_t can_port, uint8_t motor_id);  // 复位单个电机（清故障）
    // 发送原始 CAN 帧（测试/调试用）：直接指定 ID 和数据，不经过 Motor 抽象层
    bool SendRawFrame(uint8_t can_port, uint32_t id, const uint8_t* data, uint8_t len);

    // ---------------- 状态监控 ----------------
    void PrintMotorStatus(uint8_t can_port, uint8_t motor_id); // 打印指定电机的详细状态
    void PrintAllMotorStatus();                                // 打印全部电机的摘要
    MotorStatus GetMotorStatus(uint8_t can_port, uint8_t motor_id); // 直接拿结构体

    // ---------------- 错误处理 ----------------
    // 检查健康状态，不健康时在控制台打印警告
    bool CheckMotorHealth(uint8_t can_port, uint8_t motor_id);
    // 根据错误码执行默认处理策略（停机 / 复位 / 记录）
    void HandleMotorError(uint8_t can_port, uint8_t motor_id, uint8_t error_code);

    uint32_t GetMotorCount() const;   // 当前受控电机总数

private:
    MotorManager& m_motor_manager;    // 持有单例引用，所有操作转发给它
    bool          m_is_initialized;   // 是否已 Initialize，防止重复或未初始化使用

    // ---------------- 文字化工具（把数值转成可读字符串）----------------
    const char* GetMotorStateName(uint8_t state);   // 0/1/2 → "停止"/"运行"/"故障"
    const char* GetErrorCodeName(uint8_t error_code); // ErrorCode → 中文描述
};

#endif // MOTOR_CONTROLLER_H_
