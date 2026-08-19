/**
 * @file    motor_manager.cpp
 * @brief   16电机批量管理器实现
 */
#include "motor_manager.h"
#include "motor_calibration.h"
#include "motor_logger.h"
#include <cstdio>
#include <chrono>

// 单例实现
MotorManager& MotorManager::GetInstance() {
    static MotorManager instance;
    return instance;
}

MotorManager::MotorManager() {
}

MotorManager::~MotorManager() {
    Stop();
}

// 初始化4路 CANET TCP 连接，创建16个电机对象，并向外部 ThreadManager 注册任务函数
bool MotorManager::Initialize(ThreadManager& thread_mgr) {
    printf("[INFO] MotorManager initializing...\n");

    // 1. 通过 BspCan 初始化4路 CAN 设备
    const char* server_ip = "192.168.0.178";
    const uint16_t base_port = 4001;

    for (uint8_t i = 0; i < CAN_PORTS; i++) {
        // 配置 TCP 参数
        CanDeviceConfig config;
        config.device_idx = i;
        config.port = base_port + i;
        config.server_ip = server_ip;
        config.work_mode = TCP_CLIENT;  // 客户端模式

        // 通过 BspCan 初始化设备
        if (!BspCan::GetInstance().InitDevice(i, config)) {
            printf("[ERROR] Failed to initialize CAN device %d\n", i);
            return false;
        }

        // 启动设备
        if (!BspCan::GetInstance().StartDevice(i)) {
            printf("[ERROR] Failed to start CAN device %d\n", i);
            return false;
        }

        printf("[INFO] CAN device %d initialized and started\n", i);
    }

    // 2. 创建16个电机对象，配置参数
    for (uint8_t can_port = 0; can_port < CAN_PORTS; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= MOTORS_PER_CAN; motor_id++) {
            EleMotor& motor = m_motors[can_port][motor_id - 1];
            // 状态字段统一由 init() 负责，不在此处逐个手抄：
            // 原先手抄漏了 control_mode / hw_control_mode / mode_settle_ticks，
            // 靠 static 单例的零初始化碰巧得到 hw_control_mode=0，
            // 与 control_mode=IMPEDANCE(0) 相等 → 模式同步判断永远不成立，
            // MOTOR_WR_CONTROL_MODE 一次都没发过，等于假设固件默认就是阻抗模式。
            // init() 里 hw_control_mode=-1（未知），首次下发前必定真正同步一次。
            motor.init();
            motor.device_idx = can_port;   // 身份字段不属于状态，init() 不管
            motor.motor_id = motor_id;
            printf("[INFO] Motor created: CAN%d, motor_id=%d\n", can_port, motor_id);
        }
    }

    // 3. 初始化日志系统
    MotorLogger::GetInstance().Init();

    // 4. 向外部 ThreadManager 注册任务函数（不在此处启动，由 RobotApp 统一启动）
    thread_mgr.register_thread(
        "motor_receive",
        [this]() { ReceiveThreadFunc(); },
        ThreadMode::LOOP, 10, 80  // 10ms 间隔：对齐 SDK 实际接收节拍（VCI_Receive 内部 ~10ms 轮询）
    );
    thread_mgr.register_thread(
        "motor_send",
        [this]() { SendThreadFunc(); },
        ThreadMode::LOOP, 1, 80  // 1ms 间隔：发送路径实测 ~0.01ms，可真正达到 1ms
    );

    m_initialized = true;
    printf("[INFO] MotorManager initialized successfully\n");
    return true;
}

// 关闭所有 CAN 设备（线程由外部 ThreadManager 统一停止，此处不操作线程）
void MotorManager::Stop() {
    printf("[INFO] MotorManager stopping...\n");

    // 关闭日志
    MotorLogger::GetInstance().Shutdown();

    if (!m_initialized) {
        // Initialize() 从未成功（例如示例在初始化前就退出/报错），设备未打开。
        // 直接返回，避免对不存在的设备逐个报 "Device not found"（噪音）。
        printf("[INFO] MotorManager 未初始化，跳过设备关闭\n");
        return;
    }

    // 通过 BspCan 关闭所有设备
    for (uint8_t i = 0; i < CAN_PORTS; i++) {
        BspCan::GetInstance().StopDevice(i);
        BspCan::GetInstance().CloseDevice(i);
    }
    m_initialized = false;

    printf("[INFO] MotorManager stopped\n");
}

// 单次 CAN 轮询，由 ThreadManager 以 LOOP 模式每 1ms 调用一次
void MotorManager::ReceiveThreadFunc() {
    for (uint8_t can_port = 0; can_port < CAN_PORTS; can_port++) {
        std::vector<BspCanFrame> frames;
        // timeout=10ms：对齐 ZLG CANET SDK 的实际接收节拍。
        // 实测 VCI_Receive 内部按 ~10ms 粒度轮询，请求的 timeout<10ms 一律被
        // 抬升到 ~10ms（timeout 扫描：1/5/10ms 都 ~10ms 返回，20/50/100 才生效）。
        // 因此传 1ms 是自欺欺人，直接传 10ms 更诚实；且数据到达最坏等一个节拍
        // ~10ms，正好匹配 CONTROL_HZ=100 的反馈需求。另注意 WaitTime=0 会被
        // 当作无限阻塞，绝不能用 0（否则某路无帧时接收线程卡死在 VCI_Receive）。
        if (BspCan::GetInstance().ReceiveFrames(can_port, frames, 10)) {
            for (const auto& frame : frames) {
                if (frame.id >= 51 && frame.id <= 54) {
                    uint8_t motor_id = frame.id - 50;
                    if (motor_id >= 1 && motor_id <= MOTORS_PER_CAN) {
                        std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
                        EleMotor& motor = m_motors[can_port][motor_id - 1];
                        // 直接解包 CAN 帧数据
                        unpack_frame(motor, frame.data, frame.dlc);
                    }
                }
            }
        }
    }
}

// 写固件控制模式后等待其生效的周期数（发送线程 1ms/周期 → 约 20ms）
static const int MODE_SETTLE_TICKS = 20;

// 在使能之前把固件控制模式写下去。
//
// 为什么需要单独提供这个接口：SendThreadFunc 的第一行是
// `if (!motor.enabled) continue;`，模式同步写在它之后，所以未使能的电机
// 永远发不出模式帧。仅靠 SendSpeed/SendImpedance 设置 control_mode 字段，
// 真正的 0x5B 帧只会在使能之后才上总线——等于"先使能、后写模式"。
// 后果：电机在固件默认模式（阻抗）下被使能，固件拿一个未知的位置目标
// 去闭环，轮电机会在使能瞬间就转起来（实测 CAN1）。
void MotorManager::SetControlMode(uint8_t can_port, uint8_t motor_id, int mode) {
    if (can_port >= CAN_PORTS || motor_id < 1 || motor_id > MOTORS_PER_CAN) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.control_mode = mode;
    // 直接在调用线程发帧，不经过 SendThreadFunc，因此不受 enabled 限制
    float2bag(motor, (float)mode, 1, MOTOR_WR_CONTROL_MODE);
    // 标记已同步，避免 SendThreadFunc 使能后再写一次（多一次瞬态）
    motor.hw_control_mode = mode;
    // 计时在使能后才开始递减（SendThreadFunc 跳过未使能电机），
    // 相当于使能后再留一段窗口给固件生效
    motor.mode_settle_ticks = MODE_SETTLE_TICKS;
    printf("[INFO] CAN%d motor%d 预写固件控制模式 -> %d\n", can_port, motor_id, mode);
}

// 读固件参数寄存器。回帧由接收线程解包，未识别的类型会打印
// "[PARAM] CANx motory type=0xNN value=..."（见 ele_motor.cpp 的 default 分支）。
// 与 SetControlMode 同理：直接在调用线程发帧，因此使能前也能读。
void MotorManager::ReadParam(uint8_t can_port, uint8_t motor_id, uint8_t type) {
    if (can_port >= CAN_PORTS || motor_id < 1 || motor_id > MOTORS_PER_CAN) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    float2bag(motor, 0.0f, 0, type);   // RW=0 读，参数值置 0
}

// 电机控制接口实现
void MotorManager::EnableMotor(uint8_t can_port, uint8_t motor_id) {
    if (can_port >= CAN_PORTS || motor_id < 1 || motor_id > MOTORS_PER_CAN) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.enable();
    printf("[INFO] Motor enabled: CAN%d, motor_id=%d\n", can_port, motor_id);
}

void MotorManager::DisableMotor(uint8_t can_port, uint8_t motor_id) {
    if (can_port >= CAN_PORTS || motor_id < 1 || motor_id > MOTORS_PER_CAN) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.disable();
    printf("[INFO] Motor disabled: CAN%d, motor_id=%d\n", can_port, motor_id);
}

void MotorManager::SetZero(uint8_t can_port, uint8_t motor_id) {
    if (can_port >= CAN_PORTS || motor_id < 1 || motor_id > MOTORS_PER_CAN) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.target_position = 0;
    // 发送归零命令
    float2bag(motor, 0.0f, 1, MOTOR_ANGLE_ZERO);
    printf("[INFO] Motor zeroed: CAN%d, motor_id=%d\n", can_port, motor_id);
}

void MotorManager::ClearError(uint8_t can_port, uint8_t motor_id) {
    if (can_port >= CAN_PORTS || motor_id < 1 || motor_id > MOTORS_PER_CAN) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.error_code = 0;
    // 发送清错命令
    float2bag(motor, 0.0f, 1, MOTOR_CLEAR_ERROR);
    printf("[INFO] Motor error cleared: CAN%d, motor_id=%d\n", can_port, motor_id);
}

void MotorManager::SendImpedance(uint8_t can_port, uint8_t motor_id,
                                  float pos, float vel, float kp, float kd, float torque) {
    if (can_port >= CAN_PORTS || motor_id < 1 || motor_id > MOTORS_PER_CAN) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.target_position = pos;
    motor.target_speed = vel;
    motor.kp = kp;
    motor.kd = kd;
    motor.target_torque = torque;
    motor.control_mode = IMPEDANCE;
}

void MotorManager::SendSpeed(uint8_t can_port, uint8_t motor_id,
                              float vel, float kp, float ki) {
    if (can_port >= CAN_PORTS || motor_id < 1 || motor_id > MOTORS_PER_CAN) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.target_speed = vel;
    motor.kvp = kp;  // SPEED 下发路径读 kvp，不是 kp
    motor.ki = ki;
    motor.control_mode = SPEED;
}

void MotorManager::SendPosition(uint8_t can_port, uint8_t motor_id,
                                 float pos, float kvp, float kp, float kd, float kvi) {
    if (can_port >= CAN_PORTS || motor_id < 1 || motor_id > MOTORS_PER_CAN) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.target_position = pos;
    motor.kvp = kvp;
    motor.kp = kp;
    motor.kd = kd;
    motor.ki = kvi;
    motor.control_mode = POSITION;
}

MotorStatus MotorManager::GetStatus(uint8_t can_port, uint8_t motor_id) const {
    MotorStatus status;
    if (can_port >= CAN_PORTS || motor_id < 1 || motor_id > MOTORS_PER_CAN) {
        return status;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    const EleMotor& motor = m_motors[can_port][motor_id - 1];
    status.motor_id = motor.motor_id;
    status.enable = motor.enabled;
    status.position = motor.current_position;
    status.velocity = motor.current_speed;
    status.torque = motor.current_torque;
    status.error_code = motor.error_code;
    return status;
}

// 单次发送任务，由外部 ThreadManager 以 LOOP 模式驱动
void MotorManager::SendThreadFunc() {
    for (uint8_t can_port = 0; can_port < CAN_PORTS; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= MOTORS_PER_CAN; motor_id++) {
            std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
            EleMotor& motor = m_motors[can_port][motor_id - 1];

            if (!motor.enabled) continue;

            // 固件模式必须与期望模式一致，否则电机会用错误的字节布局解释控制帧。
            // 不一致时先写模式，之后等 MODE_SETTLE_TICKS 个周期再发新布局的控制帧。
            if (motor.hw_control_mode != motor.control_mode) {
                float2bag(motor, (float)motor.control_mode, 1, MOTOR_WR_CONTROL_MODE);
                motor.hw_control_mode = motor.control_mode;
                motor.mode_settle_ticks = MODE_SETTLE_TICKS;
                printf("[INFO] CAN%d motor%d 切换固件控制模式 -> %d\n",
                       can_port, motor_id, motor.control_mode);
                continue;
            }
            // 固件模式刚变更，等其生效后再下发新布局的控制帧
            if (motor.mode_settle_ticks > 0) {
                motor.mode_settle_ticks--;
                continue;
            }

            // 将上层统一坐标系的目标值逆标定回电机原始坐标系
            float tgt_pos = motor.target_position;
            float tgt_vel = motor.target_speed;
            float send_pos = tgt_pos;
            float send_vel = tgt_vel;
            float send_torque = motor.target_torque;
            ApplyMotorCalibrationInverse(can_port, motor_id, send_pos, send_vel, &send_torque);

            // 日志: 用户目标值 → 逆标定后实际发送值
            MotorLogger::GetInstance().LogSend(can_port, motor_id,
                tgt_pos, tgt_vel, send_pos, send_vel,
                motor.kp, motor.kd, motor.control_mode);

            switch (motor.control_mode) {
                case IMPEDANCE:
                    set_motor_para_bt(motor,
                        send_pos, send_vel,
                        motor.kp, motor.kd, send_torque, IMPEDANCE);
                    break;
                case SPEED:
                    set_motor_para_bt(motor,
                        send_vel, motor.kvp, 0, 0, motor.ki, SPEED);
                    break;
                case POSITION:
                    set_motor_para_bt(motor,
                        send_pos, motor.kvp, motor.kp,
                        motor.kd, motor.ki, POSITION);
                    break;
            }
        }
    }
}
