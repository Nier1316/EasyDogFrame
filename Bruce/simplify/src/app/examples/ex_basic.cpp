// ===== 基础运动示例 17-23 =====
// 由 src/app/example.cpp 拆分而来（阶段3：示例拆包），公共 helper 见 app/examples_common.h
#include "app/examples/ex_basic.h"
#include "app/examples_common.h"
#include "transport/canet_transport.h"
#include "motor/motor_manager.h"
#include "motor/motor_calibration.h"
#include "runtime/thread_manager.h"
#include "common/motor_logger.h"
#include "common/log_control.h"
#include "motion/robot_calibration.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <thread>
#include <chrono>

using logctl::LogCat;
#include "motion/SimSync.h"
#include "motion/leg_kinematics.h"
#include "motion/wheel_position_loop.h"
#include "strategy/xbox_controller.h"
#include <termios.h>
#include <fcntl.h>
#include <poll.h>

void Example17_SimSyncIntegration() {
    printf("\n========== Example 17: SimSync Simulation Integration ==========\n");
    printf("[INFO] Connecting to MATLAB simulation at 127.0.0.1:12345\n");
    printf("[INFO] Make sure quadruped_realtime.m is running first\n\n");

    SimSync sim("127.0.0.1", 12345);
    if (!sim.connected()) {
        printf("[ERROR] Failed to connect to simulation server.\n");
        printf("[ERROR] Start quadruped_realtime.m in MATLAB first.\n");
        return;
    }
    printf("[INFO] Connected to simulation!\n\n");

    // 关节角度映射表：CAN端口 → SimSync数组偏移
    struct LegMap {
        uint8_t can_port;      // CAN 端口
        uint8_t joint_offset;  // 在 joints 数组中的起始偏移
        const char* name;
    };
    const LegMap legs[4] = {
        {0, 0,  "FL"},    // CAN0 = 左前腿
        {1, 3,  "FR"},    // CAN1 = 右前腿
        {2, 6,  "RL"},    // CAN2 = 左后腿
        {3, 9,  "RR"},    // CAN3 = 右后腿
    };

    float joints[12] = {0};

    // 设定站立姿态（所有腿相同）
    auto set_standing = [&]() {
        for (int i = 0; i < 4; i++) {
            joints[legs[i].joint_offset + 0] = 0.0f;    // θ1: 髋外摆 0°
            joints[legs[i].joint_offset + 1] = -30.0f;   // θ2: 大腿 -30°
            joints[legs[i].joint_offset + 2] = 60.0f;    // θ3: 小腿 60°
        }
    };

    // ============ Phase 1: 站立姿态 ============
    printf("[Phase 1] Standing pose (3 seconds)...\n");
    set_standing();
    for (int i = 0; i < 150; i++) {  // 3s × 50Hz
        if (!sim.send_deg(joints)) {
            printf("[ERROR] Lost connection during Phase 1.\n");
            return;
        }
        usleep(20000);  // 20ms = 50Hz
    }
    printf("[Phase 1] Done.\n\n");

    // ============ Phase 2: Trot 步态运动 ============
    // FL(0) + RR(3) 同相，FR(1) + RL(2) 反相
    const float PERIOD     = 2.0f;     // 步态周期（秒）
    const float AMP_HIP    = 5.0f;     // 髋外摆幅度
    const float AMP_THIGH  = 15.0f;    // 大腿摆动幅度
    const float AMP_CALF   = 15.0f;    // 小腿摆动幅度
    const float OFFSET     = 30.0f;    // 大腿向前偏移基值
    const float CALF_BASE  = 60.0f;    // 小腿角度基值

    printf("[Phase 2] Trot gait motion (20 seconds)...\n");
    for (int frame = 0; frame < 1000; frame++) {  // 20s × 50Hz
        float t = frame * 0.02f;
        float phase = 2.0f * M_PI * t / PERIOD;

        for (int leg = 0; leg < 4; leg++) {
            // FL(0) 和 RR(3) 同相 → phase
            // FR(1) 和 RL(2) 反相 → phase + π
            float leg_phase = (leg == 1 || leg == 2) ? phase + M_PI : phase;
            float s = sinf(leg_phase);

            joints[legs[leg].joint_offset + 0] =  AMP_HIP * s;         // θ1: 髋外摆
            joints[legs[leg].joint_offset + 1] = -OFFSET + AMP_THIGH * s;  // θ2: 大腿
            joints[legs[leg].joint_offset + 2] =  CALF_BASE - AMP_CALF * s; // θ3: 小腿
        }

        if (!sim.send_deg(joints)) {
            printf("[ERROR] Lost connection during Phase 2.\n");
            return;
        }

        // 每 25 帧（0.5 秒）打印一次
        if (frame % 25 == 0) {
            printf("  t=%5.1f | FL:θ1=%+5.1f θ2=%+5.1f θ3=%5.1f | FR:θ1=%+5.1f θ2=%+5.1f θ3=%5.1f\n",
                   t,
                   joints[0], joints[1], joints[2],
                   joints[3], joints[4], joints[5]);
        }

        usleep(20000);
    }
    printf("[Phase 2] Done.\n\n");

    // ============ Phase 3: 回到站立姿态 ============
    printf("[Phase 3] Return to standing pose (1 second)...\n");
    set_standing();
    for (int i = 0; i < 50; i++) {
        sim.send_deg(joints);
        usleep(20000);
    }
    printf("[Phase 3] Done.\n");

    printf("\n[INFO] Example17 completed. Close the MATLAB figure window to stop the simulation.\n");
    fflush(stdout);
}

/**
 * @brief Example18: 基于 IK 的真实电机控制 + 可选仿真同步
 *
 * 流程:
 *   1. 足端位置 (身体坐标系) → hip_rotation_matrix 逆变换 → 髋坐标系
 *   2. leg_ik() 解算得关节指令角 (rad)
 *   3. 经 ApplyMotorCalibrationInverse 后发送给真实电机
 *   4. 可选: SimSync 同步发仿真看一眼
 *
 * 足端轨迹: 原地 Trot 步态 (对角线腿同相摆动)
 */
void Example18_LegIKControl() {
    printf("\n========== Example 18: IK-Based Motor Control ==========\n");
    printf("[INFO] This example controls real motors using inverse kinematics.\n");
    printf("[INFO] Robot will perform a trot gait via foot trajectory.\n\n");

    // ---- 初始化 ----
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] Failed to initialize MotorManager\n");
        return;
    }

    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // ---- 可选: 连接 MATLAB 仿真 ----
    SimSync sim("127.0.0.1", 12345);
    if (sim.connected()) {
        printf("[INFO] SimSync connected to MATLAB simulation\n");
    } else {
        printf("[INFO] SimSync not connected (simulation visualization disabled)\n");
    }

    // ---- 使能所有 12 个电机 ----
    printf("[INFO] Enabling all 12 motors...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) {
            motor_mgr.EnableMotor(cp, mi);
        }
    }
    sleep(1);
    printf("[INFO] All motors enabled\n\n");
    fflush(stdout);

    // ---- 控制参数 ----
    const float KP = 200.0f;
    const float KD = 15.0f;
    const float PERIOD = 1.0f;        // 步态周期 (秒)
    const float STEP_LEN = 0.25f;      // 步长 (m)
    const float STEP_HEIGHT = 0.25f;   // 抬腿高度 (m)
    const int   HZ = 1000;             // 控制频率 1kHz
    const int   TOTAL_FRAMES = 20000;  // 运行 10 秒

    // 站立基值: 足端在身体坐标系中的位置
    // 计算: 用 leg_fk_all 验证过的站立姿态 [0°, -30°, 60°]
    float stand_foot[4][3];
    float stand_q[12];
    for (int leg = 0; leg < 4; leg++) {
        stand_q[leg*3 + 0] = deg2rad(0);
        stand_q[leg*3 + 1] = deg2rad(-30);
        stand_q[leg*3 + 2] = deg2rad(60);
    }
    leg_fk_all(stand_q, stand_foot);

    // ---- 主循环 ----
    printf("Time(s) | Phase | FL_z    | FR_z    | RL_z    | RR_z\n");
    printf("--------|-------|---------|---------|---------|---------\n");
    fflush(stdout);

    for (int frame = 0; frame < TOTAL_FRAMES; frame++) {
        float t = frame * (1.0f / HZ);
        float phase = 2.0f * M_PI * t / PERIOD;

        // 12 个关节角度 (度, 给 SimSync)
        float sim_joints_deg[12];

        for (int leg = 0; leg < 4; leg++) {
            // 对角线腿 (FL+RR 同相, FR+RL 同相)
            float leg_phase = (leg == FL || leg == RR) ? phase : phase + M_PI;
            float swing = std::sin(leg_phase);
            float lift = (1.0f - std::cos(leg_phase)) * 0.5f;  // 0→1 抬腿

            // 足端位移量 (身体坐标系)
            float dx = STEP_LEN * swing;          // X 方向摆动
            float dy = 0.0f;                       // Y 方向(外摆)不动
            float dz = STEP_HEIGHT * lift;         // Z 方向抬腿

            float foot_target_body[3] = {
                stand_foot[leg][0] + dx,
                stand_foot[leg][1] + dy,
                stand_foot[leg][2] + dz,
            };

            // 身体坐标系 → 髋坐标系
            float p_hip[3];
            float R[3][3], Rt[3][3];
            hip_rotation_matrix(static_cast<LegIndex>(leg), R);
            // 逆变换 (R^T)
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    Rt[i][j] = R[j][i];
                }
            }
            float mount_to_foot[3];
            for (int i = 0; i < 3; i++) {
                mount_to_foot[i] = foot_target_body[i] - LEG_MOUNT[leg][i];
            }
            for (int i = 0; i < 3; i++) {
                p_hip[i] = 0;
                for (int j = 0; j < 3; j++) {
                    p_hip[i] += Rt[i][j] * mount_to_foot[j];
                }
            }

            // IK 解算
            float q_cmd[3];
            leg_ik(p_hip, LEG_L1, LEG_L2, LEG_L3,
                   THETA1_OFFSET, THETA2_OFFSET, THETA3_OFFSET,
                   q_cmd);

            // 钳位到限位范围（指令角坐标，与标定坐标系一致）
            q_cmd[0] = clamp(q_cmd[0],
                deg2rad(LOWER_LIMIT_THETA1_DEG),
                deg2rad(UPPER_LIMIT_THETA1_DEG));
            q_cmd[1] = clamp(q_cmd[1],
                deg2rad(LOWER_LIMIT_THETA2_DEG),
                deg2rad(UPPER_LIMIT_THETA2_DEG));
            q_cmd[2] = clamp(q_cmd[2],
                deg2rad(LOWER_LIMIT_THETA3_DEG),
                deg2rad(UPPER_LIMIT_THETA3_DEG));

            // IK 输出的指令角即为标定坐标系下的目标位置
            // SendThreadFunc 内部自动做逆标定
            for (int j = 0; j < 3; j++) {
                motor_mgr.SendImpedance(leg, j + 1, q_cmd[j], 0.0f, KP, KD, 0.0f);
            }

            // SimSync 用: 指令角 (度)
            sim_joints_deg[leg*3 + 0] = rad2deg(q_cmd[0]);
            sim_joints_deg[leg*3 + 1] = rad2deg(q_cmd[1]);
            sim_joints_deg[leg*3 + 2] = rad2deg(q_cmd[2]);
        }

        // 可选: 发送到仿真
        if (sim.connected()) {
            sim.send_deg(sim_joints_deg);
        }

        // 打印 (500ms 一次)
        if (frame % 500 == 0) {
            MotorStatus st_fl = motor_mgr.GetStatus(FL, 1);
            printf("%7.2f | %5.1f | %7.4f | %7.4f | %7.4f | %7.4f\n",
                   t, phase/(2*M_PI)*PERIOD,
                   st_fl.position, 0.0f, 0.0f, 0.0f);
            fflush(stdout);
        }

        usleep(1000000 / HZ);  // 20ms
    }

    // ---- 归零站立 ----
    printf("\n[INFO] Returning to standing position...\n");
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < 3; j++) {
            motor_mgr.SendImpedance(leg, j + 1, stand_q[leg*3 + j], 0.0f, KP, KD, 0.0f);
        }
    }
    usleep(500000);

    // ---- 禁用所有电机 ----
    printf("[INFO] Disabling all motors...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) {
            motor_mgr.DisableMotor(cp, mi);
        }
    }

    // ---- 清理 ----
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] Example18 completed\n");
    fflush(stdout);
}

// ================= 示例 19：读取当前姿态并缓慢移动到站立 =================
void Example19_ReadAndStand() {
    printf("\n========== Example 19: Read Current Pose & Slow Stand ==========\n");
    printf("[INFO] Phase 1: Read current joint angles\n");
    printf("[INFO] Phase 2: Slowly interpolate to standing pose\n\n");

    // ---- 初始化 ----
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] Failed to initialize MotorManager\n");
        return;
    }

    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // ---- 使能所有 12 个电机 ----
    printf("[INFO] Enabling all 12 motors...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) {
            motor_mgr.EnableMotor(cp, mi);
        }
    }
    sleep(1);
    printf("[INFO] All motors enabled\n\n");

    // ======== 阶段 1：读取当前姿态 ========
    printf("========== Phase 1: Current Joint Angles ==========\n");
    const char* leg_names[] = {"FL(CAN0)", "FR(CAN1)", "RL(CAN2)", "RR(CAN3)"};
    const char* joint_names[] = {"Hip", "Thigh", "Calf"};
    float cur_phys[4][3];  // 当前物理角 (rad)

    for (int leg = 0; leg < 4; leg++) {
        printf("  %s:\n", leg_names[leg]);
        for (int j = 0; j < 3; j++) {
            MotorStatus st = motor_mgr.GetStatus(leg, j + 1);
            cur_phys[leg][j] = st.position;  // 已标定的物理角 (rad)
            printf("    %s: %7.4f rad (%6.2f°)  %s%s\n",
                   joint_names[j],
                   st.position,
                   rad2deg(st.position),
                   st.enable ? "" : " [DISABLED]",
                   st.error_code ? " [FAULT]" : "");
        }
    }
    printf("\n");

    // ======== 阶段 2：缓慢移动到站立姿态 ========
    // 站立姿态 (标定坐标系): 指令角 [0°, -60°, 60°]
    //   标定坐标系 0 = 物理零位
    printf("========== Phase 2: Moving to Standing Pose ==========\n");
    printf("  Target: Hip=%5.1f°, Thigh=%5.1f°, Calf=%5.1f°\n\n",
           0.0f, -30.0f, 60.0f);
    fflush(stdout);

    const float TGT_PHYS[3] = {
        deg2rad(0.0f),       // Hip:   0°  horizontal
        deg2rad(-60.0f),     // Thigh: -60°
        deg2rad(60.0f),      // Calf:   60°
    };

    const float STAND_DURATION = 10.0f;  // 过渡时间 (秒)
    const int   HZ = 100;               // 控制频率 100Hz
    const int   TOTAL_FRAMES = (int)(STAND_DURATION * HZ);
    const float KP = 200.0f;    // 降低刚度减少振荡
    const float KD = 20.0f;    // 提高阻尼抑制超调

    for (int frame = 0; frame <= TOTAL_FRAMES; frame++) {
        float t = (float)frame / TOTAL_FRAMES;  // 0.0 → 1.0

        for (int leg = 0; leg < 4; leg++) {
            for (int j = 0; j < 3; j++) {
                // 线性插值 (物理角)，SendThreadFunc 内部会自动做逆标定
                float pos = cur_phys[leg][j] + (TGT_PHYS[j] - cur_phys[leg][j]) * t;
                motor_mgr.SendImpedance(leg, j + 1, pos, 0.0f, KP, KD, 0.0f);
            }
        }

        if (frame % 50 == 0) {  // 每 0.5 秒打印进度
            printf("  [Progress] %3.0f%%\n", t * 100.0f);
            fflush(stdout);
        }
        usleep(1000000 / HZ);
    }

    // ---- 保持站立 ----
    printf("\n[INFO] Holding standing pose...\n");
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < 3; j++) {
            motor_mgr.SendImpedance(leg, j + 1, TGT_PHYS[j], 0.0f, KP, KD, 0.0f);
        }
    }
    sleep(20);

    // ---- 禁用所有电机 ----
    printf("[INFO] Disabling all motors...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) {
            motor_mgr.DisableMotor(cp, mi);
        }
    }

    // ---- 清理 ----
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] Example19 completed\n");
    fflush(stdout);
}

// ================= 示例 20：选择电机移动到物理零位 =================
void Example20_MoveToPhysicalZero() {
    printf("\n========== Example 20: Move Selected Motor to Physical Zero ==========\n");
    printf("[INFO] Calibrated coordinate: position 0 = physical zero\n");
    printf("  Motor 1 (Hip):   target = 0  (horizontal)\n");
    printf("  Motor 2 (Thigh): target = 0  (vertical)\n");
    printf("  Motor 3 (Calf):  target = π/2  (90°, physical limit)\n\n");

    // ---- 先选择目标电机（只使能选中的电机，避免全部上电过流） ----
    int can_port = -1, motor_id = -1;
    printf("选择 CAN 端口 (0~3): ");
    fflush(stdout);
    if (scanf("%d", &can_port) != 1) { printf("[ERROR] 输入无效\n"); return; }
    printf("选择电机 ID (1~3): ");
    fflush(stdout);
    if (scanf("%d", &motor_id) != 1) { printf("[ERROR] 输入无效\n"); return; }

    if (!(can_port >= 0 && can_port <= 3 && motor_id >= 1 && motor_id <= 3)) {
        printf("[ERROR] CAN 端口 0~3, 电机 ID 1~3\n");
        return;
    }

    // ---- 初始化 ----
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }

    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // ---- 只使能选中的单个电机 ----
    printf("[INFO] 使能 CAN%d-M%d...\n", can_port, motor_id);
    motor_mgr.EnableMotor(can_port, motor_id);
    usleep(300000);

    // ---- 读取当前位置 ----
    const float KP = 200.0f, KD = 10.0f;
    MotorStatus st = motor_mgr.GetStatus(can_port, motor_id);
    float start_pos = st.position;
    printf("[INFO] 当前位置: %.4f rad (%.2f°)\n", start_pos, rad2deg(start_pos));

    // 先发当前位置保持不动
    motor_mgr.SendImpedance(can_port, motor_id, start_pos, 0.0f, KP, KD, 0.0f);
    usleep(100000);

    // ---- 计算目标 ----
    float tgt_pos;
    const char* desc;
    if (motor_id == 3) {
        tgt_pos = M_PI_2;           // 小腿从水平位弯曲 90°
        desc = "小腿 90°";
    } else if (motor_id == 1) {
        tgt_pos = 0.0f;
        desc = "髋水平";
    } else {
        tgt_pos = 0.0f;
        desc = "大腿竖直";
    }

    printf("\n[INFO] CAN%d-M%d: %s\n", can_port, motor_id, desc);
    printf("[INFO] 当前: %.4f rad (%.2f°)  →  目标: %.4f rad (%.2f°)\n\n",
           start_pos, rad2deg(start_pos), tgt_pos, rad2deg(tgt_pos));
    fflush(stdout);

    // ---- 2 秒缓慢插值到目标 ----
    const int HZ = 100;
    const int FRAMES = 200;
    for (int f = 0; f <= FRAMES; f++) {
        float t = (float)f / FRAMES;
        float pos = start_pos + (tgt_pos - start_pos) * t;
        motor_mgr.SendImpedance(can_port, motor_id, pos, 0.0f, KP, KD, 0.0f);
        if (f % 50 == 0) {
            MotorStatus st_now = motor_mgr.GetStatus(can_port, motor_id);
            printf("  [%3d%%] cmd=%.4f rad (%.2f°), actual=%.4f rad (%.2f°)\n",
                   (int)(t * 100), pos, rad2deg(pos),
                   st_now.position, rad2deg(st_now.position));
            fflush(stdout);
        }
        usleep(1000000 / HZ);
    }

    // ---- 保持 2 秒 ----
    printf("\n[INFO] 保持中...\n");
    motor_mgr.SendImpedance(can_port, motor_id, tgt_pos, 0.0f, KP, KD, 0.0f);
    sleep(2);

    st = motor_mgr.GetStatus(can_port, motor_id);
    printf("[INFO] 最终: CAN%d-M%d = %.4f rad (%.2f°)\n\n",
           can_port, motor_id, st.position, rad2deg(st.position));

    // ---- 清理 ----
    printf("[INFO] 禁用电机...\n");
    motor_mgr.DisableMotor(can_port, motor_id);
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] Example20 完成\n");
    fflush(stdout);
}

// ================= 示例 21：Xbox 手柄控制 =================
void Example21_XboxControllerControl() {
    printf("\n========== 示例 21：Xbox 手柄控制 ==========\n");
    printf("[INFO] A键  = 起立（记录按下时的姿态作为返回点）\n");
    printf("[INFO] B键  = 缓慢回到起立前的初始姿态\n");
    printf("[INFO] 十字键↑ = 升高身体  |  十字键↓ = 降低身体\n");
    printf("[INFO] 右摇杆上下 = 前进/后退  |  右摇杆左右 = 差速转向\n");
    printf("[INFO] Back键 = 退出\n\n");

    // ---- 控制参数 ----
    // 关节 kp/kd/重力前馈已移到 include/motor_calibration.h 的 JOINT_IMPEDANCE 表，
    // 按 [can_port][joint] 逐关节可调，用 GetJointImpedance(leg, j+1) 取。
    // 控制周期、机身高度范围、轮电机参数见 robot_calibration.h §5，
    // 此处不再重复定义（同名局部常量会遮蔽表中的值，改表不生效）。
    const int   HZ = CONTROL_HZ;                 // 主循环频率
    const float WHEEL_DEAD_ZONE = 0.05f;         // 轮子摇杆死区 (归一化，仅本例使用)

    // ---- 初始化 ----
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }

    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 先把 16 个电机的固件模式全部写好，再使能。
    // SetControlMode 直接发 0x5B 帧、不经发送线程，所以未使能也能写进去。
    // 若省掉这步，电机会在固件默认模式（阻抗）下被使能，固件拿一个未知的
    // 位置目标去闭环——实测 CAN1 轮电机在使能瞬间就转起来。
    // ⚠ 轮不能以 SPEED 模式直接使能：固件使能初始化动作（疑似转子对齐）+ 速度反馈
    //    启动期假偏移（实测 -43/±48 rad/s，见 Example23 注释）会绕过/被速度环追 → 疯转。
    //    故轮先以阻抗模式使能，稳定后再切到速度环。
    printf("[INFO] 预写固件控制模式（关节=阻抗，轮=阻抗先使能）...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) {
            motor_mgr.SetControlMode(cp, mi, IMPEDANCE);
        }
        motor_mgr.SetControlMode(cp, 4, IMPEDANCE);
    }
    usleep(100000);   // 留 100ms 给固件写入生效（远大于 20ms 的 settle 窗口）

    // 使能前安全预置：全电机零扭矩阻抗帧（直发，覆盖固件残留目标，避免使能瞬间动作）
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.PreEnableZeroTorque(cp, mi);
    usleep(100000);

    printf("[INFO] 使能全部电机...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 4; mi++) {
            motor_mgr.EnableMotor(cp, mi);
        }
    }
    usleep(200000);

    // 轮切到速度环（固件已使能稳定，切模式不再触发使能初始化疯转）。
    // ⚠ 真机验证点：切 SPEED 后轮应保持静止；若仍异常则轮改走阻抗前馈扭矩。
    printf("[INFO] 轮切到速度模式（使能后切换）...\n");
    for (int cp = 0; cp < 4; cp++) {
        motor_mgr.SetControlMode(cp, 4, SPEED);
        motor_mgr.SendSpeed(cp, 4, 0.0f, WHEEL_KVP, WHEEL_KVI);   // 目标速度 0，等 A 键起立
    }
    usleep(100000);

    // ---- 初始化 Xbox 手柄 ----
    XboxController controller;
    bool controller_ok = controller.Initialize();
    if (!controller_ok) {
        printf("[ERROR] 未检测到 Xbox 手柄，退出\n");
    }

    // ---- 预计算站立姿态足端位置 ----
    float stand_q[12] = {};
    float base_foot_body[4][3] = {};
    if (controller_ok) {
        // 站立指令角见 robot_calibration.h §5 STAND_*_DEG
        for (int leg = 0; leg < 4; leg++) {
            stand_q[leg * 3 + 0] = deg2rad(STAND_HIP_DEG);
            stand_q[leg * 3 + 1] = deg2rad(STAND_THIGH_DEG);
            stand_q[leg * 3 + 2] = deg2rad(STAND_CALF_DEG);
        }
        leg_fk_all(stand_q, base_foot_body);

        printf("[INFO] 站立足端位置 (身体坐标系):\n");
        for (int leg = 0; leg < 4; leg++) {
            printf("  腿%d: [%+.4f, %+.4f, %+.4f] m\n",
                   leg, base_foot_body[leg][0], base_foot_body[leg][1], base_foot_body[leg][2]);
        }
    }

    // ---- 状态变量 ----
    bool standing = false;          // 是否已起立
    float body_height = 0.0f;       // 身体高度偏移量
    bool prev_a = false;            // A键上一帧状态（用于上升沿检测）
    bool prev_b = false;            // B键上一帧状态
    bool prev_back = false;         // Back键上一帧状态

    // 按下 A 键那一刻的关节姿态，B 键据此原路返回。
    // 必须在起立阶段之外声明：主循环里 B 键要用到它。
    float start_pos[4][3] = {};

    // ---- 阶段 1：等待 A 键起立 ----
    if (controller_ok) {
        printf("\n[INFO] 按下 A 键起立...\n");
        fflush(stdout);

        while (!standing) {
            controller.Poll();
            if (!controller.IsConnected()) {
                printf("[ERROR] 手柄断开连接\n");
                break;
            }

            const XboxState& state = controller.GetState();

            // 记录手柄输入（与电机日志同时间戳基准，便于后续对齐排查）
            MotorLogger::GetInstance().LogXbox(
                state.left_stick_x, state.left_stick_y,
                state.right_stick_x, state.right_stick_y,
                state.left_trigger, state.right_trigger,
                state.a, state.b, state.x, state.y, state.lb, state.rb,
                state.back, state.start, state.ls, state.rs,
                state.dpad_up, state.dpad_down, state.dpad_left, state.dpad_right);

            // A键上升沿检测
            bool a_pressed = state.a && !prev_a;
            prev_a = state.a;

            if (a_pressed) {
                standing = true;
                // 在按下的这一刻记录姿态：此时电机还没被起立指令推动，
                // 读到的就是真正的初始位置，B 键返回的目标即为此。
                for (int leg = 0; leg < 4; leg++) {
                    for (int j = 0; j < 3; j++) {
                        start_pos[leg][j] = motor_mgr.GetStatus(leg, j + 1).position;
                    }
                }
                printf("[INFO] A键按下！已记录初始姿态，开始起立...\n");
                for (int leg = 0; leg < 4; leg++) {
                    printf("  腿%d 初始角: [%+.4f, %+.4f, %+.4f] rad\n",
                           leg, start_pos[leg][0], start_pos[leg][1], start_pos[leg][2]);
                }
                fflush(stdout);
            }

            usleep(10000);
        }
    }

    // ---- 阶段 2：2秒插值到站立姿态 ----
    if (controller_ok && standing) {
        const int TOTAL_INTERP_FRAMES = STAND_INTERP_FRAMES;  // 见 robot_calibration.h §5
        // start_pos 已在 A 键按下时记录，此处直接用

        // 线性插值过渡
        for (int f = 0; f <= TOTAL_INTERP_FRAMES; f++) {
            float t = (float)f / TOTAL_INTERP_FRAMES;
            if (t > 1.0f) t = 1.0f;

            for (int leg = 0; leg < 4; leg++) {
                for (int j = 0; j < 3; j++) {
                    float pos = start_pos[leg][j]
                              + (stand_q[leg * 3 + j] - start_pos[leg][j]) * t;
                    const JointImpedanceParam& ip = GetJointImpedance(leg, j + 1);
                    // 前馈随插值系数 t 渐入，避免起立结束切主循环时扭矩跳变
                    motor_mgr.SendImpedance(leg, j + 1, pos, 0.0f,
                                            ip.kp, ip.kd, ip.tau_ff * t);
                }
            }

            if (f % 500 == 0) {   // 500Hz 循环下 0.1s→1s 一条，避免刷屏
                printf("  起立中... %3.0f%%\n", t * 100.0f);
                fflush(stdout);
            }

            usleep(1000000 / HZ);
        }

        printf("[INFO] 已站立。十字键↑↓=高度, 右摇杆=轮子, Back=退出.\n\n");
        fflush(stdout);

        // ---- 阶段 3：主控制循环 ----
        int frame = 0;
        while (true) {
            controller.Poll();
            if (!controller.IsConnected()) {
                printf("[INFO] 手柄断开，退出...\n");
                break;
            }

            const XboxState& state = controller.GetState();

            // 记录手柄输入（与电机日志同时间戳基准，便于后续对齐排查）
            MotorLogger::GetInstance().LogXbox(
                state.left_stick_x, state.left_stick_y,
                state.right_stick_x, state.right_stick_y,
                state.left_trigger, state.right_trigger,
                state.a, state.b, state.x, state.y, state.lb, state.rb,
                state.back, state.start, state.ls, state.rs,
                state.dpad_up, state.dpad_down, state.dpad_left, state.dpad_right);

            // Back键上升沿 → 退出
            bool back_pressed = state.back && !prev_back;
            prev_back = state.back;
            if (back_pressed) {
                printf("[INFO] Back键按下，退出...\n");
                break;
            }

            // B键上升沿 → 缓慢回到 A 键按下时记录的初始姿态
            bool b_pressed = state.b && !prev_b;
            prev_b = state.b;
            if (b_pressed) {
                printf("[INFO] B键按下！缓慢回到初始姿态...\n");
                fflush(stdout);

                // 先停轮子：坐下过程中轮子不该继续转
                for (int cp = 0; cp < 4; cp++) {
                    motor_mgr.SendSpeed(cp, 4, 0.0f, WHEEL_KVP, WHEEL_KVI);
                }

                // 从"当前实际位置"而非站立目标插值：手柄调过高度后
                // 两者已不同，用实际位置起步才不会有跳变。
                float from_pos[4][3];
                for (int leg = 0; leg < 4; leg++) {
                    for (int j = 0; j < 3; j++) {
                        from_pos[leg][j] = motor_mgr.GetStatus(leg, j + 1).position;
                    }
                }

                for (int f = 0; f <= STAND_INTERP_FRAMES; f++) {
                    float t = (float)f / STAND_INTERP_FRAMES;
                    if (t > 1.0f) t = 1.0f;

                    for (int leg = 0; leg < 4; leg++) {
                        for (int j = 0; j < 3; j++) {
                            float pos = from_pos[leg][j]
                                      + (start_pos[leg][j] - from_pos[leg][j]) * t;
                            const JointImpedanceParam& ip = GetJointImpedance(leg, j + 1);
                            // 前馈随 t 渐出，落地后不再顶着前馈
                            motor_mgr.SendImpedance(leg, j + 1, pos, 0.0f,
                                                    ip.kp, ip.kd,
                                                    ip.tau_ff * (1.0f - t));
                        }
                    }

                    if (f % 500 == 0) {   // 0.2s→1s 一条
                        printf("  回落中... %3.0f%%\n", t * 100.0f);
                        fflush(stdout);
                    }
                    usleep(1000000 / HZ);
                }

                printf("[INFO] 已回到初始姿态，退出主循环\n");
                fflush(stdout);
                break;
            }

            // ---- 身体高度调节（十字键 ↑/↓，替代扳机） ----
            // RT 扳机硬件漂移（未按下就输出 0.496），改用数字量十字键，无漂移、可靠。
            // 步长与扳机满程速率一致（HEIGHT_ADJUST_RATE / HZ），按住即连续调节。
            const float HEIGHT_STEP = HEIGHT_ADJUST_RATE / HZ;
            if (state.dpad_up)
                body_height += HEIGHT_STEP;
            if (state.dpad_down)
                body_height -= HEIGHT_STEP;
            body_height = clamp(body_height, BODY_HEIGHT_MIN, BODY_HEIGHT_MAX);

            // ---- 轮子控制（右摇杆：Y 轴前后，X 轴左右转，SPEED 模式）----
            // 摇杆归一化输入，各自去死区
            float fwd_stick  = -state.right_stick_y;   // 上推为正 = 前进
            float turn_stick =  state.right_stick_x;   // 右推为正 = 右转
            if (fabsf(fwd_stick)  < WHEEL_DEAD_ZONE) fwd_stick  = 0.0f;
            if (fabsf(turn_stick) < WHEEL_DEAD_ZONE) turn_stick = 0.0f;

            // 差速：右转时左侧加速、右侧减速。X 轴单独推即原地转向。
            float v_fwd  = fwd_stick  * WHEEL_MAX_SPEED;
            float v_turn = turn_stick * WHEEL_MAX_TURN;
            float v_left  = v_fwd + v_turn;
            float v_right = v_fwd - v_turn;

            // 合成速度可能超过单轮上限。按同一比例缩放两侧而非各自钳位，
            // 否则左右差值被改变，转弯半径会随速度漂移。
            float v_peak = fmaxf(fabsf(v_left), fabsf(v_right));
            if (v_peak > WHEEL_SPEED_CAP) {
                float scale = WHEEL_SPEED_CAP / v_peak;
                v_left  *= scale;
                v_right *= scale;
            }

            // CAN0(FL)/CAN2(RL) 为左侧，CAN1(FR)/CAN3(RR) 为右侧
            motor_mgr.SendSpeed(0, 4, v_left,  WHEEL_KVP, WHEEL_KVI);
            motor_mgr.SendSpeed(2, 4, v_left,  WHEEL_KVP, WHEEL_KVI);
            motor_mgr.SendSpeed(1, 4, v_right, WHEEL_KVP, WHEEL_KVI);
            motor_mgr.SendSpeed(3, 4, v_right, WHEEL_KVP, WHEEL_KVI);

            // ---- 四腿 IK 解算并发送指令 ----
            for (int leg = 0; leg < 4; leg++) {
                // 根据身体高度偏移量调整足端目标 Z 分量
                float foot_target_body[3] = {
                    base_foot_body[leg][0],
                    base_foot_body[leg][1],
                    base_foot_body[leg][2] - body_height,
                };

                // 身体坐标系 → 髋坐标系: R^T × (foot_body - LEG_MOUNT)
                float R[3][3], Rt[3][3];
                hip_rotation_matrix(static_cast<LegIndex>(leg), R);
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        Rt[i][j] = R[j][i];

                float mount_to_foot[3];
                for (int i = 0; i < 3; i++)
                    mount_to_foot[i] = foot_target_body[i] - LEG_MOUNT[leg][i];

                float p_hip[3] = {0};
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        p_hip[i] += Rt[i][j] * mount_to_foot[j];

                // IK 反解
                float q_cmd[3];
                leg_ik(p_hip, LEG_L1, LEG_L2, LEG_L3,
                       THETA1_OFFSET, THETA2_OFFSET, THETA3_OFFSET, q_cmd);

                // 钳位到关节限位
                q_cmd[0] = clamp(q_cmd[0],
                    deg2rad(LOWER_LIMIT_THETA1_DEG), deg2rad(UPPER_LIMIT_THETA1_DEG));
                q_cmd[1] = clamp(q_cmd[1],
                    deg2rad(LOWER_LIMIT_THETA2_DEG), deg2rad(UPPER_LIMIT_THETA2_DEG));
                q_cmd[2] = clamp(q_cmd[2],
                    deg2rad(LOWER_LIMIT_THETA3_DEG), deg2rad(UPPER_LIMIT_THETA3_DEG));

                // 发送阻抗控制指令（kp/kd/前馈见 JOINT_IMPEDANCE 表）
                for (int j = 0; j < 3; j++) {
                    const JointImpedanceParam& ip = GetJointImpedance(leg, j + 1);
                    motor_mgr.SendImpedance(leg, j + 1, q_cmd[j], 0.0f,
                                            ip.kp, ip.kd, ip.tau_ff);
                }
            }

            // 每 0.5 秒打印状态（轮子显示目标角速度与 CAN0 反馈速度）
            if (frame % 500 == 0) {   // 500Hz 主循环下 0.1s→1s 一条
                MotorStatus w0 = motor_mgr.GetStatus(0, 4);
                MotorStatus w1 = motor_mgr.GetStatus(1, 4);
                printf("  高度=%+.3fm  轮目标 左=%+.2f 右=%+.2f  "
                       "实测 轮0=%+.2f 轮1=%+.2f rad/s  十字键↑=%d ↓=%d\n",
                       body_height, v_left, v_right,
                       w0.velocity, w1.velocity,
                       state.dpad_up, state.dpad_down);
                fflush(stdout);
            }

            frame++;
            usleep(1000000 / HZ);
        }
    } // 起立 & 主循环结束

    // ---- 清理 ----
    printf("[INFO] 正在关闭...\n");

    // 停止轮电机（速度模式，目标速度归零）
    for (int cp = 0; cp < 4; cp++) {
        motor_mgr.SendSpeed(cp, 4, 0.0f, WHEEL_KVP, WHEEL_KVI);
    }

    // 禁用所有 16 个电机
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 4; mi++) {
            motor_mgr.DisableMotor(cp, mi);
        }
    }

    controller.Shutdown();
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] 示例21 完成\n");
    fflush(stdout);
}

// ================= 示例 22：起立 + 轮子阻抗模式测试 =================
void Example22_StandAndWheelTest() {
    printf("\n========== 示例 22：起立 + 轮子阻抗模式测试 ==========\n");
    printf("[INFO] 阶段1: 读取当前姿态\n");
    printf("[INFO] 阶段2: 缓慢起立\n");
    printf("[INFO] 阶段3: 4秒后轮子以 1rad/s 滚动 (KP=3, KD=0.3)\n\n");

    // ---- 初始化 ----
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }

    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // ---- 使能所有 12 条腿电机 + 4 个轮电机 ----
    printf("[INFO] 使能 12 条腿电机...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) {
            motor_mgr.EnableMotor(cp, mi);
        }
    }
    printf("[INFO] 使能 4 个轮电机...\n");
    for (int cp = 0; cp < 4; cp++) {
        motor_mgr.EnableMotor(cp, 4);
    }
    usleep(500000);  // 等轮电机完全就绪

    // 轮电机统一用厂家速度模式（SPEED），上位机只给目标角速度
    printf("[INFO] 轮电机速度归零保持...\n");
    const float WHEEL_LOOP_VEL = 1.0f;   // 阶段3滚动角速度 (rad/s)
    const float WHEEL_KVP      = 1.0f;   // 速度环 Kp — 初值，实测再调
    const float WHEEL_KVI      = 0.0f;   // 速度环 Ki — 初值，实测再调
    for (int cp = 0; cp < 4; cp++) {
        MotorStatus st = motor_mgr.GetStatus(cp, 4);
        motor_mgr.SendSpeed(cp, 4, 0.0f, WHEEL_KVP, WHEEL_KVI);  // 目标速度 0
        printf("  CAN%d-M4: %.4f rad (%.2f°)\n", cp, st.position, rad2deg(st.position));
    }
    fflush(stdout);

    // ======== 阶段 1：等待数据就绪并读取当前姿态 ========
    printf("========== 阶段1: 等待数据就绪 ==========\n");
    {
        MotorStatus st = motor_mgr.GetStatus(0, 1);
        int retry = 0;
        while (fabsf(st.position) < 0.001f && fabsf(st.velocity) < 0.001f && retry < 100) {
            usleep(50000);
            st = motor_mgr.GetStatus(0, 1);
            retry++;
        }
        printf("[INFO] 等待 %d 次 (%.1f s) 后数据就绪\n", retry, retry * 0.05f);
    }

    float cur_pos[4][3];
    printf("当前关节角度:\n");
    for (int leg = 0; leg < 4; leg++) {
        printf("  CAN%d: ", leg);
        for (int j = 0; j < 3; j++) {
            MotorStatus st = motor_mgr.GetStatus(leg, j + 1);
            cur_pos[leg][j] = st.position;
            printf("  M%d=%.2f°", j + 1, rad2deg(st.position));
        }
        printf("\n");
    }
    fflush(stdout);

    // ======== 阶段 2：缓慢起立 ========
    printf("\n========== 阶段2: 起立 ==========\n");
    const float TGT[3] = { deg2rad(0.0f), deg2rad(-30.0f), deg2rad(60.0f) };
    const float KP = 150.0f, KD = 20.0f;
    const int   STAND_DURATION = 3;
    const int   HZ = 100;
    const int   TOTAL_FRAMES = STAND_DURATION * HZ;

    for (int frame = 0; frame <= TOTAL_FRAMES; frame++) {
        float t = (float)frame / TOTAL_FRAMES;
        for (int leg = 0; leg < 4; leg++) {
            for (int j = 0; j < 3; j++) {
                float pos = cur_pos[leg][j] + (TGT[j] - cur_pos[leg][j]) * t;
                motor_mgr.SendImpedance(leg, j + 1, pos, 0.0f, KP, KD, 0.0f);
            }
        }
        if (frame % 50 == 0) {
            printf("  起立中... %3.0f%%\n", t * 100.0f);
            fflush(stdout);
        }
        usleep(1000000 / HZ);
    }

    // 保持站立 4 秒
    printf("\n[INFO] 保持站立 4 秒...\n");
    for (int sec = 1; sec <= 4; sec++) {
        for (int leg = 0; leg < 4; leg++) {
            for (int j = 0; j < 3; j++) {
                motor_mgr.SendImpedance(leg, j + 1, TGT[j], 0.0f, KP, KD, 0.0f);
            }
        }
        // 持续保持轮子速度为 0（速度模式锁停）
        for (int cp = 0; cp < 4; cp++) {
            motor_mgr.SendSpeed(cp, 4, 0.0f, WHEEL_KVP, WHEEL_KVI);
        }
        sleep(1);
        printf("  %d/4 秒\n", sec);
        fflush(stdout);
    }

    // ======== 阶段 3：轮电机速度模式滚动 ========
    printf("\n========== 阶段3: 轮电机速度模式滚动 (%.0frad/s, KVP=%.1f, KVI=%.1f) ==========\n",
           WHEEL_LOOP_VEL, WHEEL_KVP, WHEEL_KVI);
    printf("[INFO] 轮子开始滚动...\n");
    fflush(stdout);

    const int WHEEL_DURATION = 10;

    // 保持腿姿态
    for (int leg = 0; leg < 4; leg++)
        for (int j = 0; j < 3; j++)
            motor_mgr.SendImpedance(leg, j + 1, TGT[j], 0.0f, KP, KD, 0.0f);

    const int WHEEL_HZ = 50;
    for (int sec = 0; sec < WHEEL_DURATION; sec++) {
        for (int f = 0; f < WHEEL_HZ; f++) {
            for (int cp = 0; cp < 4; cp++) {
                // 恒定目标角速度，电机端闭速度环
                motor_mgr.SendSpeed(cp, 4, WHEEL_LOOP_VEL, WHEEL_KVP, WHEEL_KVI);
            }
            usleep(1000000 / WHEEL_HZ);
        }

        // 重新发送腿姿态
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++)
                motor_mgr.SendImpedance(leg, j + 1, TGT[j], 0.0f, KP, KD, 0.0f);

        MotorStatus w0 = motor_mgr.GetStatus(0, 4);
        printf("  轮子 %d/%d 秒  目标=%.2f 轮0实测=%.2f rad/s\n", sec + 1, WHEEL_DURATION,
               WHEEL_LOOP_VEL, w0.velocity);
        fflush(stdout);
    }

    // ---- 清理 ----
    printf("\n[INFO] 停止并禁用所有电机...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 4; mi++) {
            motor_mgr.SendImpedance(cp, mi, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
    }
    usleep(500000);

    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 4; mi++) {
            motor_mgr.DisableMotor(cp, mi);
        }
    }

    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] 示例22 完成\n");
    fflush(stdout);
}
void Example23_SingleCanKeyboardControl() {
    printf("\n========== 示例 23：单路 CAN 键盘控制 ==========\n");

    // ---- 选择 CAN 路（与其它示例一致，阻塞式 scanf，此时终端为正常行模式）----
    int can_port = -1;
    printf("请输入要控制的 CAN 路编号 (0~3): ");
    fflush(stdout);
    // 调试器（cppdbg externalConsole=false）下 stdin 是 gdb 分配的 pty，
    // isatty 为真但键盘输入不一定送得到 scanf（实测立刻无效输入退出）。
    // 改用 poll 限时等输入：3s 内有有效数字 → 用输入；超时/EOF/无效 → 默认 CAN1，
    // 这样直接 F5 调试也能跑起来，真终端下交互照常。
    // 注意：flush 换行只在成功读到数字后执行，避免在静默 pty 上被 getchar 卡死。
    struct pollfd pfd;
    pfd.fd = fileno(stdin);
    pfd.events = POLLIN;
    pfd.revents = 0;
    int pr = poll(&pfd, 1, 3000);
    bool got_input = (pr > 0 && (pfd.revents & POLLIN) &&
                      scanf("%d", &can_port) == 1 && can_port >= 0 && can_port <= 3);
    if (got_input) {
        // 吞掉行尾换行，避免污染后续 raw 读取
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) {}
    } else {
        printf("\n[INFO] 无有效输入（3s 超时/EOF/调试器），默认选择 CAN1\n");
        can_port = 1;
    }
    printf("[INFO] 选择 CAN%d（腿 %d + 轮）\n", can_port, can_port);

    // 键盘输入日志：提前初始化日志系统。选路发生在 MotorManager::Initialize()
    // 之前，而后者内部才调 MotorLogger::Init()；Init 幂等，之后再次调用为空操作。
    // 把选路结果记为 key 日志的第一行（frame=-1），便于区分不同运行。
    MotorLogger::GetInstance().Init();
    MotorLogger::GetInstance().LogKey(-1, 0, "CAN_SELECT", can_port);

    // ---- 控制参数 ----
    const float KP = 150.0f;                 // 关节阻抗刚度
    const float KD = 20.0f;                  // 关节阻抗阻尼
    const int   HZ = 100;                    // 主循环频率
    const float HEIGHT_STEP  = 0.002f;       // 每帧按住方向键的高度调节量 (m)
    const float BODY_HEIGHT_MIN = -0.15f;    // 最低（深蹲）
    const float BODY_HEIGHT_MAX =  0.10f;    // 最高（站立）
    const float WHEEL_SPEED  = 1.5f;         // 轮子正/反转角速度 (rad/s)
    const float WHEEL_KVP    = 3.0f;         // 速度环 Kp — 初值，实测再调
    const float WHEEL_KVI    = 0.3f;         // 速度环 Ki — 初值，实测再调
    // 软启动：切入速度环时先用弱增益，再把增益渐变到额定。
    // 依据：CAN1 轮固件速度反馈在启动期存在假偏移（实测 -43rad/s，或饱和 ±48），
    // 满增益会把假偏移当真实误差去追 → 疯转 ~1s。弱增益下假偏移只产生小力矩，
    // 等偏移自行消退（实测 ~1s）后再升到额定，正常控制且不踢腿。
    const float WHEEL_SOFT_KVP = 0.3f;       // 软启动初始 Kp（额定的 1/10）
    const float WHEEL_SOFT_KVI = 0.03f;      // 软启动初始 Ki（额定的 1/10）
    const int   WHEEL_SOFT_FRAMES = 150;     // 软启动时长 (1.5s @100Hz)
    // 松开方向键后轮子指令保持的帧数（终端无按键释放事件，用超时判定停止）
    const int   WHEEL_HOLD_FRAMES = 8;

    // ---- 初始化 ----
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 只使能选中路的 3 个关节 + 1 个轮
    printf("[INFO] 使能 CAN%d 电机...\n", can_port);
    for (int mi = 1; mi <= 4; mi++) motor_mgr.EnableMotor(can_port, mi);
    usleep(300000);

    // ---- 预计算该腿站立姿态足端位置 ----
    const int leg = can_port;  // CAN 路 == LegIndex (CAN0=FL...CAN3=RR)
    float stand_q[12] = {};
    float base_foot_body[4][3] = {};
    for (int l = 0; l < 4; l++) {
        stand_q[l * 3 + 0] = deg2rad(0);    // Hip
        stand_q[l * 3 + 1] = deg2rad(-30);  // Thigh
        stand_q[l * 3 + 2] = deg2rad(60);   // Calf
    }
    leg_fk_all(stand_q, base_foot_body);

    // ---- 缓慢移动到站立姿态（2 秒插值）----
    printf("[INFO] 移动到站立姿态...\n");
    float start_pos[3];
    for (int j = 0; j < 3; j++)
        start_pos[j] = motor_mgr.GetStatus(can_port, j + 1).position;

    const int STAND_FRAMES = 200;  // 2s × 100Hz
    for (int f = 0; f <= STAND_FRAMES; f++) {
        float t = (float)f / STAND_FRAMES;
        for (int j = 0; j < 3; j++) {
            float pos = start_pos[j] + (stand_q[leg * 3 + j] - start_pos[j]) * t;
            motor_mgr.SendImpedance(can_port, j + 1, pos, 0.0f, KP, KD, 0.0f);
        }
        usleep(1000000 / HZ);
    }
    printf("[INFO] 已站立\n");

    // 轮电机初始速度归零（厂家 SPEED 模式）。
    // 用弱增益切入：这一次 SendSpeed 会触发固件切速度环（control_mode=SPEED），
    // 若此时用额定增益，假速度偏移会在第一帧就被当成 +43 的误差去追 → 开机踢腿。
    // 弱增益只产生 ~1/10 的力矩，给假偏移留出 ~1s 的消退窗口。
    motor_mgr.SendSpeed(can_port, 4, 0.0f, WHEEL_SOFT_KVP, WHEEL_SOFT_KVI);

    // ---- 进入终端 raw 模式，开始键盘控制 ----
    printf("\n[操作] ↑/↓ = 升高/降低身体   ←/→ = 轮子反转/正转 0.5rad/s   q = 退出\n\n");
    fflush(stdout);

    RawTerminal term;
    if (!term.ok) {
        printf("[ERROR] 无法设置终端 raw 模式，退出\n");
    } else {
        float body_height = 0.0f;   // 身体高度偏移量
        int   wheel_hold  = 0;      // 轮子指令保持计数（>0 表示最近有左右键）
        float wheel_stick = 0.0f;   // 轮子摇杆等效输入 [-1,1]
        const float dt = 1.0f / HZ;
        bool running = true;
        int  frame = 0;

        while (running) {
            // 一帧内消化所有已缓冲按键（raw 非阻塞，可能积压多个）
            bool got_wheel_key = false;
            KeyDir k;
            while ((k = poll_key()) != KeyDir::NONE) {
                // 按键事件日志（key_code 与 KeyDir 对应：1↑ 2↓ 3← 4→ 5q），
                // 时间戳与 send/recv 同一基准，用于对齐"哪次按键导致电机怎么动"。
                const char* kname = (k == KeyDir::UP)    ? "UP" :
                                    (k == KeyDir::DOWN)  ? "DOWN" :
                                    (k == KeyDir::LEFT)  ? "LEFT" :
                                    (k == KeyDir::RIGHT) ? "RIGHT" : "QUIT";
                MotorLogger::GetInstance().LogKey(frame, static_cast<int>(k), kname);
                switch (k) {
                    case KeyDir::UP:    body_height += HEIGHT_STEP; break;
                    case KeyDir::DOWN:  body_height -= HEIGHT_STEP; break;
                    case KeyDir::LEFT:  wheel_stick = -1.0f; got_wheel_key = true; break;  // 反转
                    case KeyDir::RIGHT: wheel_stick = +1.0f; got_wheel_key = true; break;  // 正转
                    case KeyDir::QUIT:  running = false; break;
                    default: break;
                }
            }
            body_height = clamp(body_height, BODY_HEIGHT_MIN, BODY_HEIGHT_MAX);

            // 轮子：终端无“松开”事件，用超时判定停止。有左右键则续期保持计数
            if (got_wheel_key) wheel_hold = WHEEL_HOLD_FRAMES;
            if (wheel_hold > 0) { wheel_hold--; }
            else                { wheel_stick = 0.0f; }  // 超时无键 → 停

            // 轮子：速度模式下发目标角速度
            float wheel_speed = wheel_stick * WHEEL_SPEED;
            // 速度环增益软启动：前 WHEEL_SOFT_FRAMES 帧从弱增益线性渐变到额定。
            // 这样即使假速度偏移尚未消退，速度环也只会输出小力矩，不会疯转；
            // 偏移消退后增益已到位，控制手感与原来一致。
            float kfr = (frame < WHEEL_SOFT_FRAMES) ? (float)frame / WHEEL_SOFT_FRAMES : 1.0f;
            float kvp = WHEEL_SOFT_KVP + (WHEEL_KVP - WHEEL_SOFT_KVP) * kfr;
            float kvi = WHEEL_SOFT_KVI + (WHEEL_KVI - WHEEL_SOFT_KVI) * kfr;
            motor_mgr.SendSpeed(can_port, 4, wheel_speed, kvp, kvi);

            // 腿 IK：站立足端 + 高度偏移
            float foot_target_body[3] = {
                base_foot_body[leg][0],
                base_foot_body[leg][1],
                base_foot_body[leg][2] - body_height,
            };
            float R[3][3], Rt[3][3];
            hip_rotation_matrix(static_cast<LegIndex>(leg), R);
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) Rt[i][j] = R[j][i];
            float mount_to_foot[3];
            for (int i = 0; i < 3; i++)
                mount_to_foot[i] = foot_target_body[i] - LEG_MOUNT[leg][i];
            float p_hip[3] = {0};
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) p_hip[i] += Rt[i][j] * mount_to_foot[j];

            float q_cmd[3];
            leg_ik(p_hip, LEG_L1, LEG_L2, LEG_L3,
                   THETA1_OFFSET, THETA2_OFFSET, THETA3_OFFSET, q_cmd);
            q_cmd[0] = clamp(q_cmd[0], deg2rad(LOWER_LIMIT_THETA1_DEG), deg2rad(UPPER_LIMIT_THETA1_DEG));
            q_cmd[1] = clamp(q_cmd[1], deg2rad(LOWER_LIMIT_THETA2_DEG), deg2rad(UPPER_LIMIT_THETA2_DEG));
            q_cmd[2] = clamp(q_cmd[2], deg2rad(LOWER_LIMIT_THETA3_DEG), deg2rad(UPPER_LIMIT_THETA3_DEG));
            for (int j = 0; j < 3; j++)
                motor_mgr.SendImpedance(can_port, j + 1, q_cmd[j], 0.0f, KP, KD, 0.0f);

            if (frame == 0) {
                // 软启动起点快照：|v|>5 说明速度通道确实带着假偏移，
                // 此时增益很低（kvp=0.3），即使偏移被追也只会轻推一下。
                MotorStatus w0 = motor_mgr.GetStatus(can_port, 4);
                printf("\n[INFO] 轮软启动起点: v_fb=%.2f rad/s  kvp=%.2f ki=%.3f\n",
                       w0.velocity, WHEEL_SOFT_KVP, WHEEL_SOFT_KVI);
            }
            if (frame % 50 == 0) {
                MotorStatus wst = motor_mgr.GetStatus(can_port, 4);
                printf("\r  高度=%+.3fm  轮 目标=%.2f 实测=%.2f rad/s  增益kvp=%.2f  ",
                       body_height, wheel_speed, wst.velocity, kvp);
                fflush(stdout);
            }
            frame++;
            usleep(1000000 / HZ);
        }
    }
    // term 析构：恢复终端

    // ---- 清理 ----
    printf("\n[INFO] 正在关闭...\n");
    motor_mgr.SendImpedance(can_port, 4, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);  // 轮零扭矩
    for (int mi = 1; mi <= 4; mi++) motor_mgr.DisableMotor(can_port, mi);

    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] 示例23 完成\n");
    fflush(stdout);
}

// ================= 示例 24：只读固件参数诊断 =================
// 全程不使能任何电机，只发读参数帧，安全可反复运行。
// 用途：
//   1) 核对固件量程与 MOTOR_LIMITS 是否一致（kd_max 项目写 500，厂商参考是 100）
//   2) 读上电初始速度反馈，判断 CAN1 轮电机 -43 rad/s 的假速度是否与使能无关
//   3) 读固件当前控制模式，确认上电默认值
