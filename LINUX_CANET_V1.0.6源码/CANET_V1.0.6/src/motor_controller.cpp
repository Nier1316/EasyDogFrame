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
        config.server_ip  = nullptr;    // 服务器模式不需要目标 IP
        config.work_mode  = TCP_SERVER; // 上位机作为服务器，等待 CANET 设备连入
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
// 让电机按指定速度/方向转动：封装 CMD_SET_SPEED
bool MotorController::MoveMotor(uint8_t can_port, uint8_t motor_id, uint16_t speed, uint8_t direction) {
    if (!IsRunning()) {
        printf("[ERROR] MotorController is not running\n");
        return false;
    }

    MotorCommand cmd;
    cmd.motor_id  = motor_id;
    cmd.cmd_type  = CMD_SET_SPEED;
    cmd.speed     = speed;
    cmd.direction = direction;

    if (!m_motor_manager.SendMotorCommand(can_port, motor_id, cmd)) {
        printf("[ERROR] Failed to send move command to motor (can_port=%d, motor_id=%d)\n",
               can_port, motor_id);
        return false;
    }

    printf("[INFO] Move command sent: can_port=%d, motor_id=%d, speed=%d, direction=%d\n",
           can_port, motor_id, speed, direction);
    return true;
}

// 扭矩控制：适合需要恒力输出的场景（如夹爪、牵引）
bool MotorController::SetMotorTorque(uint8_t can_port, uint8_t motor_id, uint16_t torque) {
    if (!IsRunning()) {
        printf("[ERROR] MotorController is not running\n");
        return false;
    }

    MotorCommand cmd;
    cmd.motor_id = motor_id;
    cmd.cmd_type = CMD_SET_TORQUE;
    cmd.torque   = torque;

    if (!m_motor_manager.SendMotorCommand(can_port, motor_id, cmd)) {
        printf("[ERROR] Failed to send torque command to motor (can_port=%d, motor_id=%d)\n",
               can_port, motor_id);
        return false;
    }

    printf("[INFO] Torque command sent: can_port=%d, motor_id=%d, torque=%d\n",
           can_port, motor_id, torque);
    return true;
}

bool MotorController::StopMotor(uint8_t can_port, uint8_t motor_id) {
    if (!IsRunning()) {
        printf("[ERROR] MotorController is not running\n");
        return false;
    }

    MotorCommand cmd;
    cmd.motor_id = motor_id;
    cmd.cmd_type = CMD_STOP;      // STOP 命令不需要 speed/torque/direction

    if (!m_motor_manager.SendMotorCommand(can_port, motor_id, cmd)) {
        printf("[ERROR] Failed to send stop command to motor (can_port=%d, motor_id=%d)\n",
               can_port, motor_id);
        return false;
    }

    printf("[INFO] Stop command sent: can_port=%d, motor_id=%d\n", can_port, motor_id);
    return true;
}

// 紧急停机：对所有电机广播 STOP；即便某一路失败，其它电机仍会收到停止命令
bool MotorController::StopAllMotors() {
    if (!IsRunning()) {
        printf("[ERROR] MotorController is not running\n");
        return false;
    }

    MotorCommand cmd;
    cmd.cmd_type = CMD_STOP;   // motor_id 由 BroadcastCommand 内部逐一填入

    if (!m_motor_manager.BroadcastCommand(cmd)) {
        printf("[ERROR] Failed to broadcast stop command\n");
        return false;
    }

    printf("[INFO] Stop command broadcast to all motors\n");
    return true;
}

// 复位电机：用于清除堵转/过载等触发的故障态，使电机重新可控
bool MotorController::ResetMotor(uint8_t can_port, uint8_t motor_id) {
    if (!IsRunning()) {
        printf("[ERROR] MotorController is not running\n");
        return false;
    }

    MotorCommand cmd;
    cmd.motor_id = motor_id;
    cmd.cmd_type = CMD_RESET;

    if (!m_motor_manager.SendMotorCommand(can_port, motor_id, cmd)) {
        printf("[ERROR] Failed to send reset command to motor (can_port=%d, motor_id=%d)\n",
               can_port, motor_id);
        return false;
    }

    printf("[INFO] Reset command sent: can_port=%d, motor_id=%d\n", can_port, motor_id);
    return true;
}

// 格式化打印单个电机的完整状态，便于调试与演示
void MotorController::PrintMotorStatus(uint8_t can_port, uint8_t motor_id) {
    MotorStatus status = GetMotorStatus(can_port, motor_id);

    printf("\n========== Motor Status ==========\n");
    printf("CAN Port:     %d\n", can_port);
    printf("Motor ID:     %d\n", status.motor_id);
    printf("Speed:        %d\n", status.speed);
    printf("Torque:       %d\n", status.torque);
    printf("Temperature:  %d°C\n", status.temperature);
    printf("State:        %s\n", GetMotorStateName(status.state));
    printf("Error Code:   0x%02x (%s)\n", status.error_code, GetErrorCodeName(status.error_code));
    printf("==================================\n\n");
}

// 批量打印全部电机的一行摘要；用于快速巡检
void MotorController::PrintAllMotorStatus() {
    auto statuses = m_motor_manager.GetAllMotorStatus();

    printf("\n========== All Motors Status ==========\n");
    printf("Total Motors: %zu\n\n", statuses.size());

    for (const auto& status : statuses) {
        // 注意：MotorStatus 自身不带 can_port 字段，此处仅展示 motor_id
        //       若需区分 CAN 口，应改用 MotorManager::GetAllMotors() + Motor::GetCanPort()
        printf("Motor (can_port=?, id=%d): Speed=%d, Torque=%d, Temp=%d°C, State=%s, Error=0x%02x\n",
               status.motor_id, status.speed, status.torque, status.temperature,
               GetMotorStateName(status.state), status.error_code);
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
// 如果需要更复杂的策略（重试次数、退避、报警上报），可在此扩展
void MotorController::HandleMotorError(uint8_t can_port, uint8_t motor_id, uint8_t error_code) {
    printf("[ERROR] Motor error detected: can_port=%d, motor_id=%d, error=0x%02x (%s)\n",
           can_port, motor_id, error_code, GetErrorCodeName(error_code));

    switch (error_code) {
        case ERR_MOTOR_OVERHEAT:
            // 高温继续运行有烧毁风险，立即停机
            printf("[ACTION] Stopping motor due to overheat\n");
            StopMotor(can_port, motor_id);
            break;

        case ERR_MOTOR_OVERCURRENT:
            // 过流通常意味着负载异常或短路，必须立即切断
            printf("[ACTION] Stopping motor due to overcurrent\n");
            StopMotor(can_port, motor_id);
            break;

        case ERR_MOTOR_STALL:
            // 堵转多为机械卡阻，先复位清错，等待上层决定是否重启
            printf("[ACTION] Resetting motor due to stall\n");
            ResetMotor(can_port, motor_id);
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
