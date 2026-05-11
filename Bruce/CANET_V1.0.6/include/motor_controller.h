/**
 * @file    motor_controller.h
 * @brief   面向应用层的高级电机控制器
 * @details MotorController 是应用代码直接使用的门面（Facade）类：
 *          - 隐藏 MotorManager 的复杂性，提供硬件协议支持的完整功能
 *          - 内置默认的 CANET 设备配置（4 个 TCP 服务器，端口 4001~4004）
 *          - 支持三种控制模式：阻抗控制、速度控制、位置控制
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

    // 系统生命周期
    bool Initialize();                 // 构造默认 CAN 配置并初始化 MotorManager
    bool Start();                      // 启动 MotorManager（必须先 Initialize）
    bool Stop();                       // 停止并释放
    bool IsRunning() const;            // 当前是否在运行

    // 特殊指令接口
    bool EnableMotor(uint8_t can_port, uint8_t motor_id);
    bool DisableMotor(uint8_t can_port, uint8_t motor_id);
    bool SetMotorZero(uint8_t can_port, uint8_t motor_id);
    bool ClearMotorError(uint8_t can_port, uint8_t motor_id);
    bool SetControlMode(uint8_t can_port, uint8_t motor_id, ControlMode mode);

    // 控制指令接口（三种模式）
    bool ImpedanceControl(uint8_t can_port, uint8_t motor_id,
                          float pos, float vel, float kp, float kd, float torque);
    bool SpeedControl(uint8_t can_port, uint8_t motor_id,
                      float vel, float kp, float ki);
    bool PositionControl(uint8_t can_port, uint8_t motor_id,
                         float pos, float kvp, float kp, float kd, float kvi);

    // 状态监控
    void PrintMotorStatus(uint8_t can_port, uint8_t motor_id); // 打印指定电机的详细状态
    void PrintAllMotorStatus();                                // 打印全部电机的摘要
    MotorStatus GetMotorStatus(uint8_t can_port, uint8_t motor_id); // 直接拿结构体

    // 错误处理
    bool CheckMotorHealth(uint8_t can_port, uint8_t motor_id);
    void HandleMotorError(uint8_t can_port, uint8_t motor_id, uint8_t error_code);

    // 参数读写接口
    bool ReadMotorParam(uint8_t can_port, uint8_t motor_id, uint8_t param_type);
    bool WriteMotorParam(uint8_t can_port, uint8_t motor_id, uint8_t param_type, float value);
    float GetMotorParam(uint8_t can_port, uint8_t motor_id, uint8_t param_type);

    uint32_t GetMotorCount() const;   // 当前受控电机总数

private:
    MotorManager& m_motor_manager;    // 持有单例引用，所有操作转发给它
    bool          m_is_initialized;   // 是否已 Initialize，防止重复或未初始化使用

    // 文字化工具（把数值转成可读字符串）
    const char* GetMotorStateName(uint8_t state);   // 0/1/2 → "停止"/"运行"/"故障"
    const char* GetErrorCodeName(uint8_t error_code); // ErrorCode → 中文描述
};

#endif // MOTOR_CONTROLLER_H_
