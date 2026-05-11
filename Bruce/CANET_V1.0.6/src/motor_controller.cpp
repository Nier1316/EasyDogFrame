/**
 * @file   motor_controller.cpp
 * @brief  MotorController 的实现：应用层门面，封装默认配置与错误处理策略
 */
#include "motor_controller.h"
#include <cstdio>

// 构造：持有单例引用；尚未调用 Initialize 时 m_is_initialized = false
MotorController::MotorController()
    : m_motor_manager(MotorManager::GetInstance()), m_is_initialized(false) {
}

// 析构：即使用户忘记调用 Stop，也保证线程/设备被清理
MotorController::~MotorController() {
    Stop();
}

// 初始化：使用默认策略创建 4 个 CANET 配置（TCP 服务器，端口 4001~4004）
// 若需要自定义配置，用户可直接调用 MotorManager::Initialize 传入自定义列表
bool MotorController::Initialize() {
    printf("[INFO] Initializing MotorController...\n");

    std::vector<CanDeviceConfig> can_configs;

    // 为 can0~can3 各生成一份默认配置；端口号按索引递增避免冲突
    for (uint8_t i = 0; i < 4; i++) {
        CanDeviceConfig config;
        config.device_idx = i;
        config.port       = 4001 + i;   // 4001 / 4002 / 4003 / 4004
        config.server_ip  = "192.168.0.178";  // 连接到 CANET 设备
        config.work_mode  = TCP_CLIENT; // 作为客户端连接到 CANET 设备
        can_configs.push_back(config);
    }

    if (!m_motor_manager.Initialize(can_configs)) {
        printf("[ERROR] Failed to initialize MotorManager\n");
        return false;
    }

    m_is_initialized = true;
    printf("[INFO] MotorController initialized\n");
    return true;
}

// 启动：必须先 Initialize，防止对未初始化的 manager 调用 Start
bool MotorController::Start() {
    if (!m_is_initialized) {
        printf("[ERROR] MotorController not initialized\n");
        return false;
    }

    if (!m_motor_manager.Start()) {
        printf("[ERROR] Failed to start MotorManager\n");
        return false;
    }

    printf("[INFO] MotorController started\n");
    return true;
}

// 停止：幂等 —— 未初始化时直接返回 true，避免重复停止时误报错
bool MotorController::Stop() {
    if (!m_is_initialized) {
        return true;
    }

    m_motor_manager.Stop();
    printf("[INFO] MotorController stopped\n");
    return true;
}

bool MotorController::IsRunning() const {
    return m_motor_manager.IsRunning();
}

// 所有"动作类"接口的通用前置检查：未运行时拒绝发送，避免静默失败
// ------------------------------------------------------------------
// 新增控制接口：特殊指令
bool MotorController::EnableMotor(uint8_t can_port, uint8_t motor_id) {
    if (!IsRunning()) {
        printf("[ERROR] MotorController is not running\n");
        return false;
    }

    MotorCommand cmd;
    cmd.motor_id = motor_id;
    cmd.cmd_type = CMD_ENABLE;

    if (!m_motor_manager.SendMotorCommand(can_port, motor_id, cmd)) {
        printf("[ERROR] Failed to send enable command to motor (can_port=%d, motor_id=%d)\n",
               can_port, motor_id);
        return false;
    }

    printf("[INFO] Enable command sent: can_port=%d, motor_id=%d\n", can_port, motor_id);
    return true;
}

bool MotorController::DisableMotor(uint8_t can_port, uint8_t motor_id) {
    if (!IsRunning()) {
        printf("[ERROR] MotorController is not running\n");
        return false;
    }

    MotorCommand cmd;
    cmd.motor_id = motor_id;
    cmd.cmd_type = CMD_DISABLE;

    if (!m_motor_manager.SendMotorCommand(can_port, motor_id, cmd)) {
        printf("[ERROR] Failed to send disable command to motor (can_port=%d, motor_id=%d)\n",
               can_port, motor_id);
        return false;
    }

    printf("[INFO] Disable command sent: can_port=%d, motor_id=%d\n", can_port, motor_id);
    return true;
}

bool MotorController::SetMotorZero(uint8_t can_port, uint8_t motor_id) {
    if (!IsRunning()) {
        printf("[ERROR] MotorController is not running\n");
        return false;
    }

    MotorCommand cmd;
    cmd.motor_id = motor_id;
    cmd.cmd_type = CMD_SET_ZERO;

    if (!m_motor_manager.SendMotorCommand(can_port, motor_id, cmd)) {
        printf("[ERROR] Failed to send set zero command to motor (can_port=%d, motor_id=%d)\n",
               can_port, motor_id);
        return false;
    }

    printf("[INFO] Set zero command sent: can_port=%d, motor_id=%d\n", can_port, motor_id);
    return true;
}

bool MotorController::ClearMotorError(uint8_t can_port, uint8_t motor_id) {
    if (!IsRunning()) {
        printf("[ERROR] MotorController is not running\n");
        return false;
    }

    MotorCommand cmd;
    cmd.motor_id = motor_id;
    cmd.cmd_type = CMD_CLEAR_ERROR;

    if (!m_motor_manager.SendMotorCommand(can_port, motor_id, cmd)) {
        printf("[ERROR] Failed to send clear error command to motor (can_port=%d, motor_id=%d)\n",
               can_port, motor_id);
        return false;
    }

    printf("[INFO] Clear error command sent: can_port=%d, motor_id=%d\n", can_port, motor_id);
    return true;
}

bool MotorController::SetControlMode(uint8_t can_port, uint8_t motor_id, ControlMode mode) {
    // 控制模式通过后续的控制指令隐含设置
    return true;
}

// 新增控制接口：控制指令（三种模式）
bool MotorController::ImpedanceControl(uint8_t can_port, uint8_t motor_id,
                                       float pos, float vel, float kp, float kd, float torque) {
    if (!IsRunning()) {
        printf("[ERROR] MotorController is not running\n");
        return false;
    }

    MotorCommand cmd;
    cmd.motor_id = motor_id;
    cmd.cmd_type = CMD_IMPEDANCE_CTRL;
    cmd.mode = IMPEDANCE;
    cmd.pos = pos;
    cmd.vel = vel;
    cmd.kp = kp;
    cmd.kd = kd;
    cmd.torque = torque;

    if (!m_motor_manager.SendMotorCommand(can_port, motor_id, cmd)) {
        printf("[ERROR] Failed to send impedance control command to motor (can_port=%d, motor_id=%d)\n",
               can_port, motor_id);
        return false;
    }

    printf("[INFO] Impedance control command sent: can_port=%d, motor_id=%d\n", can_port, motor_id);
    return true;
}

bool MotorController::SpeedControl(uint8_t can_port, uint8_t motor_id,
                                   float vel, float kp, float ki) {
    if (!IsRunning()) {
        printf("[ERROR] MotorController is not running\n");
        return false;
    }

    MotorCommand cmd;
    cmd.motor_id = motor_id;
    cmd.cmd_type = CMD_SPEED_CTRL;
    cmd.mode = SPEED;
    cmd.vel = vel;
    cmd.kp_speed = kp;
    cmd.ki_speed = ki;

    if (!m_motor_manager.SendMotorCommand(can_port, motor_id, cmd)) {
        printf("[ERROR] Failed to send speed control command to motor (can_port=%d, motor_id=%d)\n",
               can_port, motor_id);
        return false;
    }

    printf("[INFO] Speed control command sent: can_port=%d, motor_id=%d\n", can_port, motor_id);
    return true;
}

bool MotorController::PositionControl(uint8_t can_port, uint8_t motor_id,
                                      float pos, float kvp, float kp, float kd, float kvi) {
    if (!IsRunning()) {
        printf("[ERROR] MotorController is not running\n");
        return false;
    }

    MotorCommand cmd;
    cmd.motor_id = motor_id;
    cmd.cmd_type = CMD_POSITION_CTRL;
    cmd.mode = POSITION;
    cmd.pos = pos;
    cmd.kp_speed = kvp;
    cmd.kp = kp;
    cmd.kd = kd;
    cmd.ki_speed = kvi;

    if (!m_motor_manager.SendMotorCommand(can_port, motor_id, cmd)) {
        printf("[ERROR] Failed to send position control command to motor (can_port=%d, motor_id=%d)\n",
               can_port, motor_id);
        return false;
    }

    printf("[INFO] Position control command sent: can_port=%d, motor_id=%d\n", can_port, motor_id);
    return true;
}

// 格式化打印单个电机的完整状态，便于调试与演示
void MotorController::PrintMotorStatus(uint8_t can_port, uint8_t motor_id) {
    MotorStatus status = GetMotorStatus(can_port, motor_id);

    printf("\n========== Motor Status ==========\n");
    printf("CAN Port:     %d\n", can_port);
    printf("Motor ID:     %d\n", status.motor_id);
    printf("Position:     %.2f rad\n", status.position);
    printf("Velocity:     %.2f rad/s\n", status.velocity);
    printf("Torque:       %.2f Nm\n", status.torque);
    printf("Enable:       %s\n", status.enable ? "Yes" : "No");
    printf("Fault:        %s\n", status.fault ? "Yes" : "No");
    printf("ACK:          %s\n", status.ack ? "Yes" : "No");
    printf("Error Code:   0x%02x (%s)\n", status.error_code, GetErrorCodeName(status.error_code));
    printf("==================================\n\n");
}

// 批量打印全部电机的一行摘要；用于快速巡检
void MotorController::PrintAllMotorStatus() {
    auto statuses = m_motor_manager.GetAllMotorStatus();

    printf("\n========== All Motors Status ==========\n");
    printf("Total Motors: %zu\n\n", statuses.size());

    for (const auto& status : statuses) {
        printf("Motor (id=%d): Pos=%.2f, Vel=%.2f, Torque=%.2f, Enable=%s, Fault=%s, Error=0x%02x\n",
               status.motor_id, status.position, status.velocity, status.torque,
               status.enable ? "Y" : "N", status.fault ? "Y" : "N", status.error_code);
    }

    printf("========================================\n\n");
}

// 直接透传到 manager；保留这层封装是为了未来在 controller 里加缓存或过滤
MotorStatus MotorController::GetMotorStatus(uint8_t can_port, uint8_t motor_id) {
    return m_motor_manager.GetMotorStatus(can_port, motor_id);
}

// 健康检查：
//   1) 电机存在吗？不存在 → 打印错误并返回 false
//   2) Motor::IsHealthy() 为假 → 打印警告并返回 false
//   3) 否则打印 INFO 并返回 true
bool MotorController::CheckMotorHealth(uint8_t can_port, uint8_t motor_id) {
    Motor* motor = m_motor_manager.GetMotor(can_port, motor_id);
    if (!motor) {
        printf("[ERROR] Motor not found: can_port=%d, motor_id=%d\n", can_port, motor_id);
        return false;
    }

    if (!motor->IsHealthy()) {
        printf("[WARNING] Motor is not healthy: can_port=%d, motor_id=%d, error=0x%02x (%s)\n",
               can_port, motor_id, motor->GetErrorCode(), motor->GetErrorMessage());
        return false;
    }

    printf("[INFO] Motor is healthy: can_port=%d, motor_id=%d\n", can_port, motor_id);
    return true;
}

// 按错误码执行默认处理策略（集中式故障响应）
//   过热 / 过流 → 立即停机，避免设备损坏
//   堵转         → 复位，尝试恢复自动控制
//   其它         → 仅打印日志，交给上层决策
void MotorController::HandleMotorError(uint8_t can_port, uint8_t motor_id, uint8_t error_code) {
    printf("[ERROR] Motor error detected: can_port=%d, motor_id=%d, error=0x%02x (%s)\n",
           can_port, motor_id, error_code, GetErrorCodeName(error_code));

    switch (error_code) {
        case ERR_MOTOR_OVERHEAT:
            // 高温继续运行有烧毁风险，立即停机
            printf("[ACTION] Stopping motor due to overheat\n");
            DisableMotor(can_port, motor_id);
            break;

        case ERR_MOTOR_OVERCURRENT:
            // 过流通常意味着负载异常或短路，必须立即切断
            printf("[ACTION] Stopping motor due to overcurrent\n");
            DisableMotor(can_port, motor_id);
            break;

        case ERR_MOTOR_STALL:
            // 堵转多为机械卡阻，先清错，等待上层决定是否重启
            printf("[ACTION] Clearing error due to stall\n");
            ClearMotorError(can_port, motor_id);
            break;

        default:
            // 未预定义的错误：只记录，不自动动作，避免错误处理成为新故障源
            printf("[ACTION] No specific action for this error\n");
            break;
    }
}

// 透传 manager 的电机总数，方便上层做界面/循环
uint32_t MotorController::GetMotorCount() const {
    return m_motor_manager.GetMotorCount();
}

// 数字 → 中文状态名；仅用于日志/打印，不参与控制逻辑
const char* MotorController::GetMotorStateName(uint8_t state) {
    switch (state) {
        case 0:  return "停止";
        case 1:  return "运行";
        case 2:  return "故障";
        default: return "未知";   // 协议升级未适配时走这里
    }
}

// 错误码 → 中文描述；与 Motor::GetErrorMessage 内容保持一致，避免双源歧义
const char* MotorController::GetErrorCodeName(uint8_t error_code) {
    switch (error_code) {
        case ERR_OK:                return "正常";
        case ERR_MOTOR_OVERHEAT:    return "电机过热";
        case ERR_MOTOR_OVERCURRENT: return "电机过流";
        case ERR_MOTOR_STALL:       return "电机堵转";
        case ERR_COMMUNICATION:     return "通信错误";
        case ERR_TIMEOUT:           return "超时";
        case ERR_INVALID_PARAM:     return "无效参数";
        case ERR_DEVICE_OFFLINE:    return "设备离线";
        default:                    return "未知错误";
    }
}

// =====================================================================
//                    参数读写接口实现
// =====================================================================

bool MotorController::ReadMotorParam(uint8_t can_port, uint8_t motor_id, uint8_t param_type) {
    if (!IsRunning()) {
        printf("[ERROR] MotorController is not running\n");
        return false;
    }

    Motor* motor = m_motor_manager.GetMotor(can_port, motor_id);
    if (!motor) {
        printf("[ERROR] Motor not found: can_port=%d, motor_id=%d\n", can_port, motor_id);
        return false;
    }

    if (!motor->ReadParam(param_type)) {
        printf("[ERROR] Failed to send read parameter command to motor (can_port=%d, motor_id=%d, param_type=0x%02x)\n",
               can_port, motor_id, param_type);
        return false;
    }

    printf("[INFO] Read parameter command sent: can_port=%d, motor_id=%d, param_type=0x%02x\n",
           can_port, motor_id, param_type);
    return true;
}

bool MotorController::WriteMotorParam(uint8_t can_port, uint8_t motor_id, uint8_t param_type, float value) {
    if (!IsRunning()) {
        printf("[ERROR] MotorController is not running\n");
        return false;
    }

    Motor* motor = m_motor_manager.GetMotor(can_port, motor_id);
    if (!motor) {
        printf("[ERROR] Motor not found: can_port=%d, motor_id=%d\n", can_port, motor_id);
        return false;
    }

    if (!motor->WriteParam(param_type, value)) {
        printf("[ERROR] Failed to send write parameter command to motor (can_port=%d, motor_id=%d, param_type=0x%02x, value=%.2f)\n",
               can_port, motor_id, param_type, value);
        return false;
    }

    printf("[INFO] Write parameter command sent: can_port=%d, motor_id=%d, param_type=0x%02x, value=%.2f\n",
           can_port, motor_id, param_type, value);
    return true;
}

float MotorController::GetMotorParam(uint8_t can_port, uint8_t motor_id, uint8_t param_type) {
    Motor* motor = m_motor_manager.GetMotor(can_port, motor_id);
    if (!motor) {
        printf("[ERROR] Motor not found: can_port=%d, motor_id=%d\n", can_port, motor_id);
        return 0.0f;
    }

    // 这里返回的是最后一次读取的参数值
    // 实际的参数值需要通过 ReadMotorParam 后等待响应获取
    printf("[INFO] Getting motor parameter: can_port=%d, motor_id=%d, param_type=0x%02x\n",
           can_port, motor_id, param_type);
    return 0.0f;  // 预留接口，实际实现需要参数缓存机制
}
