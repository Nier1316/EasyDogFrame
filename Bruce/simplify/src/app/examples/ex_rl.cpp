// ===== RL/前馈/延迟辨识示例 25,30-32,35-38 =====
// 由原单文件 example.cpp 示例拆包而来（现拆到 src/app/examples/），公共 helper 见 app/examples_common.h
#include "app/examples/ex_rl.h"
#include "app/examples_common.h"
#include "motion/motion_controller.h"
#include "transport/canet_transport.h"
#include "transport/usb2can_transport.h"
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
#include "strategy/rl_controller.h"
#include "strategy/mlp.h"
#include "strategy/policy_test_ref.h"
#include "strategy/imu_device.h"
#include "strategy/xbox_controller.h"
#include <termios.h>
#include <fcntl.h>
#include <poll.h>

void Example25_RLPolicyControl() {
    printf("\n========== Example 25: RL Policy Control (dogurdf) ==========\n");
    printf("[INFO] 50 Hz RL 循环，Ctrl+C 急停（失能所有电机）\n");
    printf("[INFO] 零位对齐已做（L形测量）：观测/动作经 rl::CONV_* 转 URDF 约定\n");
    printf("[INFO] 数值为近似，低增益验证后微调；Ctrl+C 急停\n\n");

    const int HZ = 50;  // 与训练一致（CONTROL_DT = 0.02 s）

    // ---- 初始化电机 ----
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    // CAN1 显式换达妙 USB2CAN（需 udev 0666 + 波特率 1M 匹配电机总线；
    // 其余 3 路走 motor_manager.cpp 的默认后端 USB2CAN）
    motor_mgr.SetChannelTransport(1, &Usb2CanTransport::GetInstance());
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 预写固件模式：腿 IMPEDANCE + 轮 SPEED（固件阻抗忽略 vel_des，轮速度控制走 SPEED）
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) {
            motor_mgr.SetControlMode(cp, mi, IMPEDANCE);
        }
        motor_mgr.SetControlMode(cp, 4, SPEED);
    }
    usleep(100000);

    // 使能 16 电机
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 4; mi++) {
            motor_mgr.EnableMotor(cp, mi);
        }
    }
    usleep(200000);

    // ---- 初始化 IMU（可选）----
    ImuDevice imu;
    // 实际安装：Z 轴朝下、绕 X 轴翻面（X 不变，Y/Z 反向）
    imu.SetMount(ImuMount::Z_DOWN_X);
    const char* imu_port = "/dev/ttyUSB0";
    bool imu_ok = imu.Initialize(imu_port, 115200);
    if (!imu_ok) {
        printf("[WARN] IMU 打开失败 (%s)，gyro/quat 用默认值（机器人会失控，务必急停）\n",
               imu_port);
    }

    // ---- 手柄速度命令（可选；映射对齐 sim2sim.py 的 GamepadReader）----
    XboxController controller;
    bool pad_ok = controller.Initialize();
    if (!pad_ok)
        printf("[WARN] 手柄未连接，cmd 保持原地站立；接入后需重启程序生效\n");

    // ---- 起立到真机实测站立指令角（CAN order，12 腿关节）----
    float stand_q[12];
    for (int leg = 0; leg < 4; leg++) {
        stand_q[leg * 3 + 0] = deg2rad(STAND_HIP_DEG);
        stand_q[leg * 3 + 1] = deg2rad(STAND_THIGH_DEG);
        stand_q[leg * 3 + 2] = deg2rad(STAND_CALF_DEG);
    }
    float start_pos[12];
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < 3; j++) {
            start_pos[leg * 3 + j] = motor_mgr.GetStatus(leg, j + 1).position;
        }
    }

    printf("[INFO] 起立中...\n");
    const int STAND_FRAMES = 500;  // 500 / 50 Hz = 10 s
    for (int f = 0; f <= STAND_FRAMES; f++) {
        if (g_rl_stop) break;
        float t = (float)f / STAND_FRAMES;
        for (int leg = 0; leg < 4; leg++) {
            for (int j = 0; j < 3; j++) {
                float pos = start_pos[leg * 3 + j]
                          + (stand_q[leg * 3 + j] - start_pos[leg * 3 + j]) * t;
                motor_mgr.SendImpedance(leg, j + 1, pos, 0.0f, 200.0f, 20.0f, 0.0f);
            }
            motor_mgr.SendSpeed(leg, 4, 0.0f, rl::WHEEL_SOFT_KVP, 0.0f);  // 轮 0 速弱增益（SPEED 软启动）
        }
        usleep(1000000 / HZ);
    }
    printf("[INFO] 起立完成，进入 RL 循环\n");

    // ---- RL 主循环（50 Hz）----
    g_rl_stop = 0;
    signal(SIGINT, rl_signal_handler);

    float last_action[16] = {0.0f};
    float wheel_v_lp[4] = {0.0f, 0.0f, 0.0f, 0.0f};   // 轮速目标低通状态（SPEED 下发前平滑）
    // 速度命令量程，与 sim2sim.py GamepadReader 默认一致（max_vx=1.0, max_vyaw=1.0）
    const float MAX_VX = 1.0f;   // m/s
    const float MAX_WZ = 1.0f;   // rad/s
    // 初始原地站立；手柄接入后每步由摇杆覆盖（左摇杆Y=前进, 右摇杆X=偏航, B=急停）
    float cmd[3] = {0.0f, 0.0f, 0.0f};
    int step = 0;

    printf("[INFO] RL 循环启动，Ctrl+C 急停\n");
    while (!g_rl_stop) {
        // 1) 读 16 电机（CAN order，标定后 = 指令角）
        float pos_can[16], vel_can[16];
        for (int cp = 0; cp < 4; cp++) {
            for (int mi = 1; mi <= 4; mi++) {
                MotorStatus st = motor_mgr.GetStatus(cp, mi);
                int mjx = cp * 4 + (mi - 1);
                pos_can[mjx] = st.position;
                vel_can[mjx] = st.velocity;
            }
        }

        // 2) CAN order -> policy order -> URDF 约定。
        //    真机 GetStatus 角用电机标定约定，与策略训练用的 URDF 约定存在
        //    每关节符号/偏移差异（大腿符号相反），经 rl::CONV_* 转成 URDF 约定，
        //    策略观测/动作才能与训练一致。
        float pos_policy[16], vel_policy[16];
        for (int i = 0; i < 16; i++) {
            pos_policy[i] = rl::status_to_urdf(pos_can[rl::MJX_TO_POLICY[i]], i);
            vel_policy[i] = rl::status_vel_to_urdf(vel_can[rl::MJX_TO_POLICY[i]], i);
        }

        // 3) 读 IMU
        float gyro[3] = {0.0f, 0.0f, 0.0f};
        float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        if (imu_ok) {
            imu.GetGyro(gyro[0], gyro[1], gyro[2]);
            imu.GetQuat(quat[0], quat[1], quat[2], quat[3]);
        }

        // 3.5) 手柄速度命令（对齐 sim2sim.py GamepadReader）：
        //      左摇杆上推 = +vx 前进；右摇杆左推 = +wz 左转(CCW)；vy 恒 0；B 键急停
        if (pad_ok) {
            controller.Poll();
            const XboxState& st = controller.GetState();
            cmd[0] = -st.left_stick_y  * MAX_VX;
            cmd[1] =  0.0f;                              // vy 恒 0，与训练一致
            cmd[2] = -st.right_stick_x * MAX_WZ;
            if (st.b) {
                printf("[WARN] 手柄 B 键急停\n");
                g_rl_stop = 1;
            }
        }

        // 4) 观测 -> 推理 -> 下发
        float obs[64];
        rl::build_observation(gyro, quat, pos_policy, vel_policy,
                              last_action, cmd, step, obs);
        float action[16];
        rl::mlp_forward(obs, action);

        for (int cp = 0; cp < 4; cp++) {
            for (int mi = 1; mi <= 4; mi++) {
                int mjx = cp * 4 + (mi - 1);
                int p = rl::POLICY_TO_MJX[mjx];
                if (mi <= 3) {
                    // 动作目标角是 URDF 约定，转回真机 GetStatus 约定再下发
                    float q_target = rl::urdf_to_status(rl::leg_pos_target(action[p], p), p);
                    // 扭矩前馈：JOINT_IMPEDANCE.tau_ff（重力）+ 腿摩擦前馈（库仑）
                    const JointImpedanceParam& ip = GetJointImpedance(cp, mi);
                    float q_t_urdf = rl::leg_pos_target(action[p], p);
                    float tau_pd = rl::LEG_KP * (q_t_urdf - pos_policy[p])
                                 - rl::LEG_KD * vel_policy[p];
                    float tau_ff = ip.tau_ff + rl::leg_friction_ff(tau_pd, vel_policy[p], p);
                    motor_mgr.SendImpedance(cp, mi, q_target, 0.0f,
                                            rl::LEG_KP, rl::LEG_KD, tau_ff);
                } else {
                    int w_idx = p - rl::NUM_LEG_JOINTS;   // POLICY 轮索引 12..15 → 0..3
                    // 轮子：SPEED 固件速度环（2026-08-29 迁移）。固件阻抗忽略 vel_des 实测轮
                    // 不动，速度控制改走 SendSpeed(vel/kvp/ki)，固件内部 1kHz 闭环。
                    // 目标速度一阶低通抑制推杆/松杆 cmd 骤变振荡（τ≈100ms @50Hz）。
                    float tv_raw = rl::WHEEL_VEL_SCALE * action[p];
                    wheel_v_lp[w_idx] += rl::WHEEL_CMD_ALPHA * (tv_raw - wheel_v_lp[w_idx]);
                    // 轮速目标：低通后原样执行（死区已弃用，见 rl_controller.h WHEEL_CMD_DEADZONE）
                    float cmd_v = wheel_v_lp[w_idx];
                    // 站立门控：无移动指令时轮子强制静止（策略站立时后轮常有正 action，真机会精确执行成溜车）
                    if (fabsf(cmd[0]) < rl::WHEEL_CMD_MOVE_THR && fabsf(cmd[2]) < rl::WHEEL_CMD_MOVE_THR)
                        cmd_v = 0.0f;
                    motor_mgr.SendSpeed(cp, mi, cmd_v, rl::WHEEL_KVP, rl::WHEEL_KVI);
                }
            }
        }

        // 5) 更新 last_action
        for (int i = 0; i < 16; i++) {
            last_action[i] = action[i];
        }

        // 6) 跌倒检测（重力投影 z > -0.34 ≈ 70° 倾斜）
        if (obs[8] > -0.34f) {
            printf("[WARN] 跌倒检测触发 (proj_grav_z=%.2f)，急停\n", obs[8]);
            break;
        }

        step++;
        usleep(1000000 / HZ);
    }

    signal(SIGINT, SIG_DFL);

    // ---- 清理：失能 + 停线程 ----
    printf("[INFO] 正在失能...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 4; mi++) {
            motor_mgr.DisableMotor(cp, mi);
        }
    }

    imu.Shutdown();
    controller.Shutdown();
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    printf("[INFO] Example25 完成\n");
}

// ================= 示例 26：键盘输入接收测试（纯诊断，不碰电机） =================
// 用途：在开电机之前，先确认当前运行环境（集成终端 / 调试器 Debug Console）
// 到底能不能收到键盘输入。Example23 的控制依赖两段输入通路：
//   阶段1 行模式选路（poll + scanf）—— 决定默认选哪路 CAN
//   阶段2 raw 模式方向键（poll_key） —— 主控制循环的方向/高度/轮子
// 哪一段收不到，Example23 就永远跑不起来。本示例把两段拆开单独测。
void Example30_RLPolicyLinkTest() {
    printf("\n========== 示例 30：RL 策略链路离线验证 ==========\n");
    printf("[INFO] 不碰 CAN。用 REF_OBS(64) 跑 mlp_forward，对比 REF_ACTION(16)。\n");

    // 1) MLP 前向 vs 参考输出
    float act[16];
    rl::mlp_forward(REF_OBS, act);
    float max_err = 0.0f;
    int   max_idx = -1;
    for (int i = 0; i < rl::ACTION_DIM; i++) {
        float e = std::fabs(act[i] - REF_ACTION[i]);
        if (e > max_err) { max_err = e; max_idx = i; }
    }
    printf("\n  mlp_forward(REF_OBS) vs REF_ACTION:\n");
    printf("    最大绝对误差 = %.6e（%s，idx=%d）\n", max_err,
           max_err < 1e-3f ? "通过" : "失败", max_idx);
    printf("    %s\n", max_err < 1e-3f
        ? "  [OK] MLP 权重与网络结构正确"
        : "  [FAIL] 权重/结构有问题，需用 tool/export_policy.py 重新导出");
    for (int i = 0; i < rl::ACTION_DIM; i++) {
        printf("      a[%2d]  got=%.6f  ref=%.6f\n", i, act[i], REF_ACTION[i]);
    }

    // 2) 观测构建样本（固定输入），供与 Python sim2sim._build_observation 比对。
    //    这里 gyro 用机体系，quat 用单位四元数（机身水平），pos/vel 全 0（=default 附近）。
    float gyro[3] = {0.1f, -0.2f, 0.3f};
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float pos[16], vel[16];
    for (int i = 0; i < 16; i++) { pos[i] = 0.0f; vel[i] = 0.0f; }
    float la[16] = {0.0f};
    float cmd[3] = {0.3f, 0.0f, 0.1f};
    float obs[64];
    rl::build_observation(gyro, quat, pos, vel, la, cmd, 10, obs);
    printf("\n  build_observation 样本输出（gyro=[0.1,-0.2,0.3], quat=单位, pos/vel=0, step=10）:\n");
    for (int i = 0; i < 64; i += 8) {
        printf("    obs[%2d..%2d] = %.6f %.6f %.6f %.6f | %.6f %.6f %.6f %.6f\n",
               i, i + 7, obs[i], obs[i + 1], obs[i + 2], obs[i + 3],
               obs[i + 4], obs[i + 5], obs[i + 6], obs[i + 7]);
    }

    printf("\n[INFO] 示例30 完成（未初始化 CAN）。\n");
    fflush(stdout);
}

// ================= 示例 31：RL 零位对齐（摆腿读角，不使能电机） =================
// 目的：测量"仿真默认姿态"对应的真机标定后指令角，填入 rl::DEFAULT_POSE。
// 做法：不使能电机（零扭矩，可自由摆腿），周期发 MOTOR_OR_angle 读参数帧，
//       GetStatus 返回标定后指令角。用户手动把每只脚（轮轴心）摆到仿真默认
//       足端位置，从显示中读取各关节角。
// 仿真默认足端（body 系，轮轴心，来自 dogurdf sim2sim/MJCF）：
//   FL/FR = (±0.267, ±0.2558, -0.3364)  RL/RR = (-0.386, ±0.2558, -0.3364)
// 注意：DEFAULT_POSE 只需 12 个腿关节角（轮子在策略 obs 里不参与 joint_pos_rel），
//       轮子填 0。40 秒窗口，每 0.5s 刷新显示，结束时打印最终候选数组。
void Example31_RLZeroAlign() {
    printf("\n========== 示例 31：RL 零位对齐 + 关节范围扫描 ==========\n");
    printf("[INFO] 不使能电机（零扭矩），用读参数帧读位置。电机驱动须供电。\n");
    printf("[INFO] 自动记录每个关节的 min/max：来回掰各关节到机械极限，程序帮你记。\n");
    printf("[INFO] 也可顺带把脚摆到仿真默认足端位置读角（DEFAULT_POSE 候选）。\n");
    printf("[INFO] 90 秒窗口，每 0.5s 刷新。\n\n");

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");   // 只需接收线程解包读参回帧
    sleep(1);

    // 仿真默认足端目标（body 系，轮轴心）
    const float target[4][3] = {
        { 0.267f,  0.2558f, -0.3364f},   // FL
        { 0.267f, -0.2558f, -0.3364f},   // FR
        {-0.386f,  0.2558f, -0.3364f},   // RL
        {-0.386f, -0.2558f, -0.3364f},   // RR
    };
    const char* legname[4] = {"FL", "FR", "RL", "RR"};

    printf("仿真默认足端目标（body 系，轮轴心，米）:\n");
    for (int leg = 0; leg < 4; leg++)
        printf("  %s: (%.3f, %.3f, %.3f)   相对髋: (%.3f, %.3f, %.3f)\n",
               legname[leg], target[leg][0], target[leg][1], target[leg][2],
               target[leg][0] - LEG_MOUNT[leg][0],
               target[leg][1] - LEG_MOUNT[leg][1],
               target[leg][2] - LEG_MOUNT[leg][2]);
    printf("\n[操作] 逐腿摆到目标位置，读显示中的关节角并记录。\n");
    printf("       髋X轴/大腿/小腿电机驱动供电即可，不必使能（零扭矩）。\n\n");
    fflush(stdout);

    using clk = std::chrono::steady_clock;
    auto t_start = clk::now();
    int ms_since = 0;
    int quiet_cnt = 0;   // 连续无响应计数（判断电机驱动是否供电）
    float min_pos[16], max_pos[16];
    bool have_reading = false;

    while (true) {
        int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            clk::now() - t_start).count();
        if (elapsed >= 90000) break;

        // 周期读 16 电机角度（读参数帧，非使能控制帧）
        float pos_can[16];
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 4; mi++)
                motor_mgr.ReadParam(cp, mi, MOTOR_OR_angle);
        usleep(60000);   // 等回帧（10ms 地板 × 串行，16 帧约需 160ms，取 60ms 偏紧——多轮后跟上）

        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 4; mi++)
                pos_can[cp * 4 + (mi - 1)] = motor_mgr.GetStatus(cp, mi).position;

        // 更新 min/max
        if (!have_reading) {
            for (int i = 0; i < 16; i++) min_pos[i] = max_pos[i] = pos_can[i];
            have_reading = true;
        } else {
            for (int i = 0; i < 16; i++) {
                if (pos_can[i] < min_pos[i]) min_pos[i] = pos_can[i];
                if (pos_can[i] > max_pos[i]) max_pos[i] = pos_can[i];
            }
        }

        // 无响应检测：若全部 ≈0 且长时间不变，提示驱动可能断电
        bool any = false;
        for (int i = 0; i < 16; i++) if (std::fabs(pos_can[i]) > 1e-4f) any = true;
        if (!any) quiet_cnt++;
        else quiet_cnt = 0;

        // 显示（每 0.5s 刷一次：当前值 + min/max 范围）
        ms_since += 60;
        if (ms_since >= 500) {
            ms_since = 0;
            int remain = (90000 - elapsed) / 1000;
            printf("\r[剩 %3ds] 当前: ", remain);
            for (int leg = 0; leg < 4; leg++) {
                int base = leg * 4;
                printf("%s(h%6.1f t%6.1f c%6.1f w%6.1f)  ",
                       legname[leg],
                       rad2deg(pos_can[base + 0]), rad2deg(pos_can[base + 1]),
                       rad2deg(pos_can[base + 2]), rad2deg(pos_can[base + 3]));
            }
            printf("\n         范围: ");
            for (int leg = 0; leg < 4; leg++) {
                int base = leg * 4;
                printf("%s[h%5.1f..%5.1f t%5.1f..%5.1f c%5.1f..%5.1f]  ",
                       legname[leg],
                       rad2deg(min_pos[base + 0]), rad2deg(max_pos[base + 0]),
                       rad2deg(min_pos[base + 1]), rad2deg(max_pos[base + 1]),
                       rad2deg(min_pos[base + 2]), rad2deg(max_pos[base + 2]));
            }
            printf("\n");
            fflush(stdout);
        }
        if (quiet_cnt >= 5) {
            printf("\n[WARN] 连续多轮读不到位置（全 0）。确认电机驱动已供电、CAN 正常。\n");
            quiet_cnt = 0;
        }
    }

    // ---- 结束时打印关节范围扫描结果 + DEFAULT_POSE 候选 ----
    printf("\n\n========== 关节范围扫描结果（机械真实限位，deg）==========\n");
    for (int leg = 0; leg < 4; leg++) {
        int base = leg * 4;
        printf("  %s: hip[%7.1f, %7.1f]  thigh[%7.1f, %7.1f]  calf[%7.1f, %7.1f]\n",
               legname[leg],
               rad2deg(min_pos[base + 0]), rad2deg(max_pos[base + 0]),
               rad2deg(min_pos[base + 1]), rad2deg(max_pos[base + 1]),
               rad2deg(min_pos[base + 2]), rad2deg(max_pos[base + 2]));
    }
    printf("\n对照项目旧限位(deg): hip[-60,0] thigh[-70,90] calf[60,180]\n");
    printf("对照 URDF 限位(deg): hip[-34.4,34.4] thigh[-40.1,100.3] calf[-57.3,20.1]\n");

    float pos_can[16];
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            pos_can[cp * 4 + (mi - 1)] = motor_mgr.GetStatus(cp, mi).position;

    printf("\n========== 当前快照：DEFAULT_POSE 候选（POLICY order, rad）==========\n");
    printf("（12 腿关节 FL,FR,RL,RR 各 hip/thigh/calf，轮子=0；把每腿摆到目标足端后读）\n");
    printf("const float DEFAULT_POSE[16] = {\n");
    for (int p = 0; p < 16; p++) {
        float v = 0.0f;
        if (p < 12) v = pos_can[rl::POLICY_TO_MJX[p]];
        printf("    %.4ff,", v);
        if ((p + 1) % 3 == 0) printf("    // %s\n", legname[p / 3]);
        else if ((p + 1) % 4 == 0) printf("\n");
    }
    printf("};\n");

    thread_mgr.stop_thread("motor_receive");
    motor_mgr.Stop();
    printf("\n[INFO] 示例31 完成（未使能任何电机）。\n");
    fflush(stdout);
}

// ================= 示例 32：RL 默认姿态验证（命令到 DEFAULT_POSE，低增益） =================
// 目的：在跑完整 RL 循环前，验证 GetStatus↔URDF 转换是否正确——命令 16 电机
//      到 rl::DEFAULT_POSE（URDF 仿真默认，经 urdf_to_status 转回真机指令角），
//      低增益缓慢到位，确认能摆出仿真默认站姿、各关节方向正确。
// 增益：kp=150/kd=20（与 RL 起立一致），5s 慢插值；若姿态异常可立即 Ctrl+C。
void Example32_RLPoseCheck() {
    printf("\n========== 示例 32：RL 默认姿态验证 ==========\n");
    printf("[INFO] 命令 16 电机到 DEFAULT_POSE（经转换），kp=150/kd=20，5s 慢到位。\n");
    printf("[INFO] 预期：摆出仿真默认站姿（脚在默认位置）；异常立即 Ctrl+C。\n\n");

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);
    const char* legname[4] = {"FL", "FR", "RL", "RR"};

    // 写阻抗 + 使能
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.SetControlMode(cp, mi, IMPEDANCE);
    usleep(200000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.EnableMotor(cp, mi);
    usleep(300000);

    // 目标：DEFAULT_POSE（URDF）转真机指令角。
    // 注意腿/轮判定：轮子是每路第 4 个电机（mjx%4==3），不是 mjx>=12！
    // 曾用 mjx<12 误把 RR 的腿关节（mjx=12,13,14）当轮子置 0，RR 姿态全错。
    float tgt_gs[16];
    for (int mjx = 0; mjx < 16; mjx++) {
        int p = rl::POLICY_TO_MJX[mjx];
        if (mjx % 4 != 3)  // 腿关节
            tgt_gs[mjx] = rl::urdf_to_status(rl::DEFAULT_POSE[p], p);
        else               // 轮子：自由
            tgt_gs[mjx] = 0.0f;
    }

    // 读取当前角度作为起点
    float start[16];
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            start[cp * 4 + (mi - 1)] = motor_mgr.GetStatus(cp, mi).position;

    printf("目标(真机指令角,deg): ");
    for (int leg = 0; leg < 4; leg++)
        printf("%s(h%6.1f t%6.1f c%6.1f)  ", legname[leg],
               rad2deg(tgt_gs[leg * 4 + 0]), rad2deg(tgt_gs[leg * 4 + 1]),
               rad2deg(tgt_gs[leg * 4 + 2]));

    // 目标角超限核对（真机指令限位来自 robot_calibration.h §4）。
    // CONV_B 基于一次 L 形目测，若转换后目标角出界（当前 thigh≈-71.8° 略超 -70°），
    // 说明测量有误差，需现场微调 CONV_B，避免硬顶机械限位。
    bool over = false;
    const char* jname[3] = {"hip", "thigh", "calf"};
    const float lo[3] = {LOWER_LIMIT_THETA1_DEG, LOWER_LIMIT_THETA2_DEG, LOWER_LIMIT_THETA3_DEG};
    const float hi[3] = {UPPER_LIMIT_THETA1_DEG, UPPER_LIMIT_THETA2_DEG, UPPER_LIMIT_THETA3_DEG};
    printf("\n[限位核对] 真机指令限位(deg) hip[%g,%g] thigh[%g,%g] calf[%g,%g]\n",
           lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]);
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < 3; j++) {
            float t = rad2deg(tgt_gs[leg * 4 + j]);
            if (t < lo[j] || t > hi[j]) {
                printf("  [WARN] %s-%s 目标 %7.1f° 超限 [%5.1f, %5.1f]\n",
                       legname[leg], jname[j], t, lo[j], hi[j]);
                over = true;
            }
        }
    }
    if (over)
        printf("[WARN] 存在超限目标角 → CONV_B 测量需微调；程序仍会执行，超限角将顶机械限位，务必盯紧电机。\n");

    printf("\n\n[INFO] 5s 插值到位...\n");
    fflush(stdout);

    const int FRAMES = 250;  // 5s @ 50Hz
    for (int f = 0; f <= FRAMES; f++) {
        float t = (float)f / FRAMES;
        for (int cp = 0; cp < 4; cp++) {
            for (int mi = 1; mi <= 3; mi++) {
                int mjx = cp * 4 + (mi - 1);
                float pos = start[mjx] + (tgt_gs[mjx] - start[mjx]) * t;
                motor_mgr.SendImpedance(cp, mi, pos, 0.0f, 250.0f, 20.0f, 0.0f);
            }
            motor_mgr.SendImpedance(cp, 4, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);  // 轮自由
        }
        if (f % 25 == 0) {
            printf("\r  [%3.0f%%] ", t * 100);
            for (int leg = 0; leg < 4; leg++)
                printf("%s(h%6.1f t%6.1f c%6.1f)  ", legname[leg],
                       rad2deg(motor_mgr.GetStatus(leg, 1).position),
                       rad2deg(motor_mgr.GetStatus(leg, 2).position),
                       rad2deg(motor_mgr.GetStatus(leg, 3).position));
            fflush(stdout);
        }
        usleep(20000);  // 50Hz
    }

    // 保持 3s，观察是否稳定
    printf("\n[INFO] 保持 3s（观察是否稳定、脚是否在默认位置）...\n");
    for (int f = 0; f < 750; f++) {
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 3; mi++) {
                int mjx = cp * 4 + (mi - 1);
                motor_mgr.SendImpedance(cp, mi, tgt_gs[mjx], 0.0f, 250.0f, 20.0f, 0.0f);
            }
        usleep(20000);
    }

    printf("\n[INFO] 到位后各腿实际角度: ");
    for (int leg = 0; leg < 4; leg++)
        printf("%s(h%6.1f t%6.1f c%6.1f)  ", legname[leg],
               rad2deg(motor_mgr.GetStatus(leg, 1).position),
               rad2deg(motor_mgr.GetStatus(leg, 2).position),
               rad2deg(motor_mgr.GetStatus(leg, 3).position));
    printf("\n\n[判读] 对照目标：是否到达且姿态像 dogurdf 默认站姿。\n");
    printf("      脚若不在默认位置/方向不对 → 转换有问题，调 CONV_*。\n");

    // 缓慢插值回到初始位置（5s），测试完不留机器在 DEFAULT 姿态
    printf("\n[INFO] 缓慢插值回到初始位置（5s）...\n");
    for (int f = 0; f <= FRAMES; f++) {
        float t = (float)f / FRAMES;
        for (int cp = 0; cp < 4; cp++) {
            for (int mi = 1; mi <= 3; mi++) {
                int mjx = cp * 4 + (mi - 1);
                float pos = tgt_gs[mjx] + (start[mjx] - tgt_gs[mjx]) * t;
                motor_mgr.SendImpedance(cp, mi, pos, 0.0f, 250.0f, 20.0f, 0.0f);
            }
            motor_mgr.SendImpedance(cp, 4, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);  // 轮自由
        }
        if (f % 25 == 0) {
            printf("\r  [%3.0f%%] ", t * 100);
            for (int leg = 0; leg < 4; leg++)
                printf("%s(h%6.1f t%6.1f c%6.1f)  ", legname[leg],
                       rad2deg(motor_mgr.GetStatus(leg, 1).position),
                       rad2deg(motor_mgr.GetStatus(leg, 2).position),
                       rad2deg(motor_mgr.GetStatus(leg, 3).position));
            fflush(stdout);
        }
        usleep(20000);
    }
    printf("\n[INFO] 已回到初始位置。\n");

    // 清理
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.DisableMotor(cp, mi);
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    printf("\n[INFO] 示例32 完成。\n");
    fflush(stdout);
}

// ================= 示例 33：IMU 链路验证（只读，不碰电机） =================
// 目的：在跑完整 RL（Example25）之前，验证 IMU 链路是否正确——
//       串口连通、帧校验通过、gyro/quat 合理，且 RL 观测里直接用的
//       base_ang_vel（= gyro，机体系 rad/s）与 projected_gravity 方向符合约定。
// 约定（与 sim2sim.py / rl_controller.h 一致）：
//   - 机体系 X+ 前 / Y+ 左 / Z+ 上；IMU 实际安装 Z_DOWN_X（Z 朝下、绕 X 翻面），
//     与 Example25 一致。
//   - projected_gravity = world2self(quat, [0,0,-1])，机器人水平放平时 ≈ (0,0,-1)。
//     （world2self 已离线验证 == 仿真 brax rotate(v, quat_inv(q))。）
// 判读：
//   1) 水平放平、静止：quat≈(1,0,0,0)、欧拉角≈0、pgr≈(0,0,-1)、gyro≈0。
//   2) 前倾：pitch>0，pgr.x 变正；右倾：roll>0，pgr.y 变负（MuJoCo 约定）。
//   3) gyro 只在转动时非零，方向与转动一致。
//   4) 放平但 pgr 明显偏离 (0,0,-1) → 安装方向/开机水平校准有问题，先别上 RL。
// 60s 窗口，1s 刷新。
void Example35_WheelFFCalibrate() {
    printf("\n========== 示例 35：轮电机前馈标定 ==========\n");
    printf("[WARN] 将使能 16 电机并起立（5s），轮子悬空。\n");
    printf("       每轮分别标定：↑/↓ 调扭矩(0.1梯度)，轮子恰好转动时按回车确认。\n");
    printf("       Ctrl+Q 退出（回位并失能）。请确保机器平稳支撑。\n\n");

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    signal(SIGINT, rl_signal_handler);
    g_rl_stop = 0;
    using clk = std::chrono::steady_clock;

    // 使能 16 电机（阻抗模式）
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.SetControlMode(cp, mi, IMPEDANCE);
    usleep(200000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.EnableMotor(cp, mi);
    usleep(300000);

    // 读初始腿位置 + 起立目标（STAND_*，真机实测站立指令角）
    float start_pos[12];
    float stand_q[12];
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < 3; j++)
            start_pos[leg * 3 + j] = motor_mgr.GetStatus(leg, j + 1).position;
        stand_q[leg * 3 + 0] = deg2rad(STAND_HIP_DEG);
        stand_q[leg * 3 + 1] = deg2rad(STAND_THIGH_DEG);
        stand_q[leg * 3 + 2] = deg2rad(STAND_CALF_DEG);
    }

    auto send_leg = [&](const float* q) {
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++)
                motor_mgr.SendImpedance(leg, j + 1, q[leg * 3 + j], 0.0f, 200.0f, 20.0f, 0.0f);
    };

    // 5s 慢插值起立（250 frames @ 50Hz），轮子 0 扭矩
    printf("[INFO] 起立中（5s，四腿支撑、轮子悬空）...\n");
    const int STAND_FRAMES = 250;
    for (int f = 0; f <= STAND_FRAMES && !g_rl_stop; f++) {
        float t = (float)f / STAND_FRAMES;
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++)
                motor_mgr.SendImpedance(leg, j + 1,
                    start_pos[leg * 3 + j] + (stand_q[leg * 3 + j] - start_pos[leg * 3 + j]) * t,
                    0.0f, 200.0f, 20.0f, 0.0f);
        for (int cp = 0; cp < 4; cp++)
            motor_mgr.SendImpedance(cp, 4, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        if (f % 50 == 0) { printf("\r  [%3.0f%%] ", t * 100); fflush(stdout); }
        usleep(20000);
    }
    printf("\n[INFO] 起立完成，轮子悬空。开始逐电机标定。\n");

    RawTerminal term;   // 进入 raw 模式（非阻塞键盘），析构自动恢复
    if (!term.ok) {
        printf("[ERROR] 无法进入 raw 终端模式（请从集成终端运行）\n");
        g_rl_stop = 1;
    }

    // 按键读取：0=无 1=UP 2=DOWN 3=ENTER 4=QUIT
    auto read_key = []() -> int {
        unsigned char c;
        if (read(STDIN_FILENO, &c, 1) != 1) return 0;
        if (c == '\n' || c == '\r') return 3;          // 回车
        if (c == 'q' || c == 'Q') return 4;            // 退出
        if (c != 0x1b) return 0;                       // 非 ESC
        unsigned char s[2];
        if (read(STDIN_FILENO, &s[0], 1) != 1) return 0;
        if (read(STDIN_FILENO, &s[1], 1) != 1) return 0;
        if (s[0] != '[') return 0;
        if (s[1] == 'A') return 1;                     // ↑
        if (s[1] == 'B') return 2;                     // ↓
        return 0;
    };

    const char* legname[4] = {"FL(C0)", "FR(C1)", "RL(C2)", "RR(C3)"};
    float ff_result[4][2] = {};   // [can][0]=正扭矩前馈, [1]=负扭矩前馈

    for (int can = 0; can < 4 && !g_rl_stop; can++) {
        printf("\n========== 标定 %s 轮电机 ==========\n", legname[can]);

        // ---- 正扭矩前馈 ----
        printf("[正扭矩] 从 0 开始，↑ 加 0.1 / ↓ 减 0.1（≥0）；轮子恰好开始转动时按回车。\n");
        float tau = 0.0f;
        bool done = false;
        while (!g_rl_stop && !done) {
            int k = read_key();
            if      (k == 1) tau += 0.1f;
            else if (k == 2) tau = std::max(0.0f, tau - 0.1f);
            else if (k == 3) done = true;
            else if (k == 4) g_rl_stop = 1;
            // 下发：当前电机 tau，其他轮 0，腿保持站立
            send_leg(stand_q);
            for (int cp = 0; cp < 4; cp++)
                motor_mgr.SendImpedance(cp, 4, 0.0f, 0.0f, 0.0f, 0.0f,
                                        cp == can ? tau : 0.0f);
            printf("\r  正扭矩=%+5.2f Nm | 轮速=%+6.3f rad/s  ", tau,
                   motor_mgr.GetStatus(can, 4).velocity);
            fflush(stdout);
            usleep(20000);
        }
        printf("\n");
        ff_result[can][0] = tau;
        for (int cp = 0; cp < 4; cp++)
            motor_mgr.SendImpedance(cp, 4, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        usleep(300000);

        // ---- 负扭矩前馈 ----
        printf("[负扭矩] 从 0 开始，↓ 减 0.1 / ↑ 加 0.1（≤0）；轮子恰好开始转动时按回车。\n");
        tau = 0.0f;
        done = false;
        while (!g_rl_stop && !done) {
            int k = read_key();
            if      (k == 2) tau -= 0.1f;
            else if (k == 1) tau = std::min(0.0f, tau + 0.1f);
            else if (k == 3) done = true;
            else if (k == 4) g_rl_stop = 1;
            send_leg(stand_q);
            for (int cp = 0; cp < 4; cp++)
                motor_mgr.SendImpedance(cp, 4, 0.0f, 0.0f, 0.0f, 0.0f,
                                        cp == can ? tau : 0.0f);
            printf("\r  负扭矩=%+5.2f Nm | 轮速=%+6.3f rad/s  ", tau,
                   motor_mgr.GetStatus(can, 4).velocity);
            fflush(stdout);
            usleep(20000);
        }
        printf("\n");
        ff_result[can][1] = tau;
        for (int cp = 0; cp < 4; cp++)
            motor_mgr.SendImpedance(cp, 4, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        usleep(300000);
    }

    // ---- 打印结果 ----
    printf("\n========== 前馈标定结果 ==========\n");
    printf("const float WHEEL_FF[4][2] = {\n");
    for (int can = 0; can < 4; can++)
        printf("    { %+6.3ff, %+6.3ff },  // %s\n",
               ff_result[can][0], ff_result[can][1], legname[can]);
    printf("};\n");

    // ---- 5s 回位 ----
    printf("\n[INFO] 回位中（5s）...\n");
    for (int f = 0; f <= STAND_FRAMES && !g_rl_stop; f++) {
        float t = (float)f / STAND_FRAMES;
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++)
                motor_mgr.SendImpedance(leg, j + 1,
                    stand_q[leg * 3 + j] + (start_pos[leg * 3 + j] - stand_q[leg * 3 + j]) * t,
                    0.0f, 200.0f, 20.0f, 0.0f);
        for (int cp = 0; cp < 4; cp++)
            motor_mgr.SendImpedance(cp, 4, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        if (f % 50 == 0) { printf("\r  [%3.0f%%] ", t * 100); fflush(stdout); }
        usleep(20000);
    }
    printf("\n[INFO] 已回位。\n");

    // 失能 16 电机
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.DisableMotor(cp, mi);
    signal(SIGINT, SIG_DFL);

    thread_mgr.stop_thread("motor_send");
    thread_mgr.stop_thread("motor_receive");
    motor_mgr.Stop();
    printf("\n[INFO] 示例35 完成。\n");
    fflush(stdout);
}

// ================= 示例 36：RL 站立循环（无手柄，对照 Example25） =================
// 与 Example25 的唯一区别：去掉手柄。cmd 恒为 {0,0,0} 原地站立。
// 目的：定位"轮电机乱转"——
//   若本示例稳定：问题在手柄（SDL 初始化干扰 / 摇杆回中漂移越过死区）。
//   若仍乱转：问题在 wheel_torque 摩擦前馈 / pos_scale / 观测链路。
void Example36_RLStandLoop() {
    printf("\n========== Example 36: RL 站立循环（USB2CAN 4 路，无手柄） ==========\n");
    printf("[INFO] 50 Hz RL 循环，cmd 固定 {0,0,0} 原地站立，4 路全走达妙 USB2CAN。\n");
    printf("[INFO] 与 Example25 唯一区别：无手柄。Ctrl+C 急停。\n\n");

    const int HZ = 50;  // 与训练一致（CONTROL_DT = 0.02 s）

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    motor_mgr.SetTransport(&Usb2CanTransport::GetInstance());   // 4 路全 USB2CAN
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 预写固件模式：腿 IMPEDANCE + 轮 SPEED（固件阻抗忽略 vel_des，轮速度控制走 SPEED）
    // 腿零扭矩预置覆盖残留避免使能瞬间冲；轮 0 速弱增益软启动（假速度偏移窗口内力矩小）。
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            motor_mgr.SetControlMode(cp, mi, IMPEDANCE);
    for (int cp = 0; cp < 4; cp++)
        motor_mgr.SetControlMode(cp, 4, SPEED);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            motor_mgr.PreEnableZeroTorque(cp, mi);
    for (int cp = 0; cp < 4; cp++)
        motor_mgr.SendSpeed(cp, 4, 0.0f, rl::WHEEL_SOFT_KVP, 0.0f);  // 轮 0 速弱增益预置
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.EnableMotor(cp, mi);
    usleep(200000);

    // IMU（可选）
    ImuDevice imu;
    imu.SetMount(ImuMount::Z_DOWN_X);
    bool imu_ok = imu.Initialize("/dev/ttyUSB0", 115200);
    if (!imu_ok)
        printf("[WARN] IMU 打开失败，gyro/quat 用默认值（机器人会失控，务必急停）\n");

    // 起立目标：A/B 验证（2026-08-27）——复用 CANET 时代真机实测站立角 STAND_*（{0,-60°,60°}），
    // 排除 urdf_to_status(DEFAULT_POSE) 转换路径引入的偏差（该转换转出 thigh -66.6°/calf 53.5°，
    // 与 STAND 不一致，疑为后腿姿态异常根源）。起立到位后再 1s 过渡到 DEFAULT_POSE（RL 基准）。
    float stand_q[12];
    for (int leg = 0; leg < 4; leg++) {
        stand_q[leg * 3 + 0] = deg2rad(STAND_HIP_DEG);
        stand_q[leg * 3 + 1] = deg2rad(STAND_THIGH_DEG);
        stand_q[leg * 3 + 2] = deg2rad(STAND_CALF_DEG);
    }
    float start_pos[12];
    for (int leg = 0; leg < 4; leg++)
        for (int j = 0; j < 3; j++)
            start_pos[leg * 3 + j] = motor_mgr.GetStatus(leg, j + 1).position;

    printf("[INFO] 起立中（10s，目标 STAND_* 真机角 {0,-60°,60°}）...\n");
    const int STAND_FRAMES = 500;  // 500 / 50 Hz = 10 s
    for (int f = 0; f <= STAND_FRAMES; f++) {
        if (g_rl_stop) break;
        float t = (float)f / STAND_FRAMES;
        for (int leg = 0; leg < 4; leg++) {
            for (int j = 0; j < 3; j++) {
                float pos = start_pos[leg * 3 + j]
                          + (stand_q[leg * 3 + j] - start_pos[leg * 3 + j]) * t;
                motor_mgr.SendImpedance(leg, j + 1, pos, 0.0f, 200.0f, 20.0f, 0.0f);
            }
            motor_mgr.SendSpeed(leg, 4, 0.0f, rl::WHEEL_SOFT_KVP, 0.0f);  // 轮 0 速弱增益（SPEED 软启动）
        }
        // 诊断：每 0.5s 打印 4 路 hip(1)/thigh(2) 的 enabled/目标/实际/扭矩。
        // 判读：某路 en=0 → SendOnce 跳过（未使能）；trq≈0 且 act 不动 → 固件未执行；
        //       tgt 一直等于 act → 目标计算异常（start_pos 读错）。
        if (f % 25 == 0) {
            printf("[STAND] t=%4.2f |", t);
            for (int cp = 0; cp < 4; cp++) {
                for (int j = 1; j <= 2; j++) {
                    MotorStatus st = motor_mgr.GetStatus(cp, j);
                    float tgt = start_pos[cp * 3 + j - 1]
                              + (stand_q[cp * 3 + j - 1] - start_pos[cp * 3 + j - 1]) * t;
                    printf(" C%d-%d en%d tgt%+.2f act%+.2f trq%+.1f",
                           cp, j, (int)st.enable, tgt, st.position, st.torque);
                }
            }
            printf("\n");
            fflush(stdout);
        }
        usleep(1000000 / HZ);
    }

    // 过渡：STAND → DEFAULT_POSE（1s），平滑进 RL，避免首帧目标跳变
    float dp_q[12];
    for (int leg = 0; leg < 4; leg++)
        for (int j = 0; j < 3; j++)
            dp_q[leg * 3 + j] = rl::urdf_to_status(rl::DEFAULT_POSE[leg * 3 + j], leg * 3 + j);
    printf("[INFO] 过渡到 DEFAULT_POSE（1s，RL 基准）...\n");
    for (int f = 0; f <= 50; f++) {
        if (g_rl_stop) break;
        float t = (float)f / 50.0f;
        for (int leg = 0; leg < 4; leg++) {
            for (int j = 0; j < 3; j++) {
                float pos = stand_q[leg * 3 + j]
                          + (dp_q[leg * 3 + j] - stand_q[leg * 3 + j]) * t;
                motor_mgr.SendImpedance(leg, j + 1, pos, 0.0f, 200.0f, 20.0f, 0.0f);
            }
            motor_mgr.SendSpeed(leg, 4, 0.0f, rl::WHEEL_SOFT_KVP, 0.0f);  // 轮 0 速弱增益（SPEED 软启动）
        }
        usleep(1000000 / HZ);
    }
    printf("[INFO] 起立完成，进入 RL 站立循环\n");

    // ---- RL 主循环（50 Hz，cmd 固定原地站立）----
    g_rl_stop = 0;
    signal(SIGINT, rl_signal_handler);

    float last_action[16] = {0.0f};
    float wheel_v_lp[4] = {0.0f, 0.0f, 0.0f, 0.0f};   // 轮速目标低通状态（SPEED 下发前平滑）
    // 向前溜车抵消（2026-08-28 重新启用）：实测 traj_v26/iteration_1100 的 wheel action
    // 仍有正向偏置（cmd=0 时均值 +0.005~+0.051 → 整体前冲，解除挂钩后速度环振荡疯转）。
    // 给向后 cmd 让策略输出抵消前冲。量级从 -0.05 起，观察前冲是否消失，负太多会变后退。
    const float CMD_BIAS_VX = -0.05f;
    float cmd[3] = {CMD_BIAS_VX, 0.0f, 0.0f};
    int step = 0;

    // 接收链路心跳检测（诊断"接收线程卡死 / SDK 回调停"）：
    //   ReceiveOnce 心跳停滞 → 报警一次并区分两类（见 MotorManager 头注释）
    int64_t last_rx_hb = -1;
    uint64_t last_rx_cnt = 0;
    bool rx_stall_warned = false;

    printf("[INFO] RL 循环启动：Ctrl+C 急停，按 q 优雅退出（失能轮 + 腿回位）\n");

    RawTerminal term;         // stdin 非阻塞，读 q 键
    bool graceful = false;    // true = 按 q 优雅退出
    while (!g_rl_stop) {
        // 0) 键盘检测：按 q 优雅退出（失能轮电机 + 腿回初始）
        if (term.ok) {
            unsigned char key;
            while (read(STDIN_FILENO, &key, 1) == 1)
                if (key == 'q' || key == 'Q') { graceful = true; g_rl_stop = 1; }
        }

        // 0.5) 接收链路心跳检测：ReceiveOnce 心跳停滞 → 报警（仅一次）
        if (!rx_stall_warned) {
            int64_t hb = MotorManager::GetReceiveHeartbeatMs();
            uint64_t cnt = Usb2CanTransport::RxCount().load();
            if (last_rx_hb >= 0 && hb == last_rx_hb) {
                bool cb_alive = (cnt > last_rx_cnt);
                printf("[WARN] 接收线程心跳停滞 %dms（RxCount=%llu，%s）→ %s\n",
                       step * 20, (unsigned long long)cnt,
                       cb_alive ? "SDK 回调仍触发" : "SDK 回调也停",
                       cb_alive ? "ReceiveOnce 卡死（日志写/锁阻塞）" : "USB2CAN 接收链路挂（URB/回调停）");
                rx_stall_warned = true;
            }
            last_rx_hb = hb;
            last_rx_cnt = cnt;
        }

        // 1) 读 16 电机（CAN order，标定后）
        float pos_can[16], vel_can[16];
        for (int cp = 0; cp < 4; cp++) {
            for (int mi = 1; mi <= 4; mi++) {
                MotorStatus st = motor_mgr.GetStatus(cp, mi);
                int mjx = cp * 4 + (mi - 1);
                pos_can[mjx] = st.position;
                vel_can[mjx] = st.velocity;
            }
        }

        // 2) CAN order -> policy order -> URDF 约定
        float pos_policy[16], vel_policy[16];
        for (int i = 0; i < 16; i++) {
            pos_policy[i] = rl::status_to_urdf(pos_can[rl::MJX_TO_POLICY[i]], i);
            vel_policy[i] = rl::status_vel_to_urdf(vel_can[rl::MJX_TO_POLICY[i]], i);
        }

        // 3) 读 IMU
        float gyro[3] = {0.0f, 0.0f, 0.0f};
        float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        if (imu_ok) {
            imu.GetGyro(gyro[0], gyro[1], gyro[2]);
            imu.GetQuat(quat[0], quat[1], quat[2], quat[3]);
        }

        // 4) 观测 -> 推理 -> 下发
        float obs[64];
        rl::build_observation(gyro, quat, pos_policy, vel_policy,
                              last_action, cmd, step, obs);
        float action[16];
        rl::mlp_forward(obs, action);

        float tau_wheel[4] = {0.0f, 0.0f, 0.0f, 0.0f};   // 诊断：记录各轮下发扭矩
        for (int cp = 0; cp < 4; cp++) {
            for (int mi = 1; mi <= 4; mi++) {
                int mjx = cp * 4 + (mi - 1);
                int p = rl::POLICY_TO_MJX[mjx];
                if (mi <= 3) {
                    float q_target = rl::urdf_to_status(rl::leg_pos_target(action[p], p), p);
                    // 扭矩前馈：JOINT_IMPEDANCE.tau_ff（重力）+ 腿摩擦前馈（库仑）
                    const JointImpedanceParam& ip = GetJointImpedance(cp, mi);
                    float q_t_urdf = rl::leg_pos_target(action[p], p);
                    float tau_pd = rl::LEG_KP * (q_t_urdf - pos_policy[p])
                                 - rl::LEG_KD * vel_policy[p];
                    float tau_ff = ip.tau_ff + rl::leg_friction_ff(tau_pd, vel_policy[p], p);
                    motor_mgr.SendImpedance(cp, mi, q_target, 0.0f,
                                            rl::LEG_KP, rl::LEG_KD, tau_ff);
                } else {
                    int w_idx = p - rl::NUM_LEG_JOINTS;
                    // 轮子：SPEED 固件速度环（2026-08-29 迁移）。固件阻抗忽略 vel_des 实测轮
                    // 不动，速度控制改走 SendSpeed(vel/kvp/ki)，固件内部 1kHz 闭环（无上位机振荡）。
                    // 目标速度一阶低通：抑制推杆/松杆 cmd 骤变导致的轮子振荡（τ≈100ms @50Hz）。
                    float tv_raw = rl::WHEEL_VEL_SCALE * action[p];
                    wheel_v_lp[w_idx] += rl::WHEEL_CMD_ALPHA * (tv_raw - wheel_v_lp[w_idx]);
                    // 轮速目标：低通后原样执行（死区已弃用，见 rl_controller.h WHEEL_CMD_DEADZONE）
                    float cmd_v = wheel_v_lp[w_idx];
                    // 站立门控：无移动指令时轮子强制静止（策略站立时后轮常有正 action，真机会精确执行成溜车）
                    if (fabsf(cmd[0]) < rl::WHEEL_CMD_MOVE_THR && fabsf(cmd[2]) < rl::WHEEL_CMD_MOVE_THR)
                        cmd_v = 0.0f;
                    motor_mgr.SendSpeed(cp, mi, cmd_v, rl::WHEEL_KVP, rl::WHEEL_KVI);
                    tau_wheel[w_idx] = rl::WHEEL_KD * (cmd_v - vel_policy[p]);  // 诊断：预估速度环扭矩
                }
            }
        }

        // 4.5) 诊断打印（每 25 步 = 0.5s 一次），按日志分类开关控制（log_control.h）
        //      RL  = action(腿+轮) + 下发扭矩  → 判断策略输出 / 控制律
        //      WHEEL = joint_pos_rel(12) + 轮速反馈  → 判断 CONV 正确性 / 轮速方向
        //      IMU  = projected_gravity + angvel     → 判断姿态观测是否污染策略
        if (step % 25 == 0) {
            LOG(LogCat::RL, "[RL %4d] cmd(%+.1f,%+.1f,%+.1f) aLeg[%+.2f %+.2f %+.2f %+.2f %+.2f %+.2f %+.2f %+.2f %+.2f %+.2f %+.2f %+.2f]\n",
                step, cmd[0], cmd[1], cmd[2],
                action[0], action[1], action[2], action[3], action[4], action[5],
                action[6], action[7], action[8], action[9], action[10], action[11]);
            LOG(LogCat::RL, "         aW[%+.2f %+.2f %+.2f %+.2f] tauW[%+.2f %+.2f %+.2f %+.2f]\n",
                action[12], action[13], action[14], action[15],
                tau_wheel[0], tau_wheel[1], tau_wheel[2], tau_wheel[3]);
            LOG(LogCat::WHEEL, "  qrel[FL%+.2f %+.2f %+.2f FR%+.2f %+.2f %+.2f RL%+.2f %+.2f %+.2f RR%+.2f %+.2f %+.2f]\n",
                obs[9], obs[10], obs[11], obs[12], obs[13], obs[14],
                obs[15], obs[16], obs[17], obs[18], obs[19], obs[20]);
            LOG(LogCat::WHEEL, "         vW[%+.2f %+.2f %+.2f %+.2f] vP[%+.2f %+.2f %+.2f %+.2f]\n",
                vel_can[3], vel_can[7], vel_can[11], vel_can[15],
                vel_policy[12], vel_policy[13], vel_policy[14], vel_policy[15]);
            LOG(LogCat::IMU, "  pgr(%+.3f,%+.3f,%+.3f) angvel(%+.2f,%+.2f,%+.2f)\n",
                obs[6], obs[7], obs[8], obs[3], obs[4], obs[5]);
            fflush(stdout);
        }

        // 4.6) RL 诊断写入 CSV（每控制步，log/rl_*.csv，供离线分析）
        MotorLogger::GetInstance().LogRL(step, cmd, obs, action, tau_wheel);

        // 5) 更新 last_action
        for (int i = 0; i < 16; i++)
            last_action[i] = action[i];

        // 6) 跌倒检测（重力投影 z > -0.34 ≈ 70° 倾斜）
        if (obs[8] > -0.34f) {
            printf("[WARN] 跌倒检测触发 (proj_grav_z=%.2f)，急停\n", obs[8]);
            break;
        }

        step++;
        usleep(1000000 / HZ);
    }

    // ---- 优雅退出（按 q）：失能轮电机，腿慢速回初始姿态 ----
    if (graceful) {
        printf("[INFO] 优雅退出：失能轮电机，腿回初始姿态（10s）...\n");
        for (int cp = 0; cp < 4; cp++)
            motor_mgr.DisableMotor(cp, 4);   // 只失能轮电机

        float cur_pos[12];   // 回位起点 = 当前腿位置
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++)
                cur_pos[leg * 3 + j] = motor_mgr.GetStatus(leg, j + 1).position;

        const int RET_FRAMES = 500;   // 10s @50Hz，慢慢回
        for (int f = 0; f <= RET_FRAMES && !g_rl_stop; f++) {
            float t = (float)f / RET_FRAMES;
            for (int leg = 0; leg < 4; leg++)
                for (int j = 0; j < 3; j++) {
                    float pos = cur_pos[leg * 3 + j]
                              + (start_pos[leg * 3 + j] - cur_pos[leg * 3 + j]) * t;
                    motor_mgr.SendImpedance(leg, j + 1, pos, 0.0f, 200.0f, 20.0f, 0.0f);
                }
            if (f % 100 == 0) { printf("\r  [%3.0f%%] ", t * 100); fflush(stdout); }
            usleep(1000000 / HZ);
        }
        printf("\n[INFO] 腿已回初始姿态。\n");
    }

    signal(SIGINT, SIG_DFL);

    // ---- 清理 ----
    printf("[INFO] 正在失能...\n");
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.DisableMotor(cp, mi);

    imu.Shutdown();
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    printf("[INFO] Example36 完成\n");
}

// ================= 示例 37：RL 遥操作（手柄前进/后退） =================
// 与 Example36 相同（起立 DEFAULT_POSE + q 优雅退出 + 诊断 + 软限位），
// 差别：加入 Xbox 手柄实时给速度命令。
//   左摇杆 Y 上推 = +vx 前进 / 下推 = -vx 后退（量程 ±1.0 m/s）
//   右摇杆 X 左推 = +wz 左转（量程 ±1.0 rad/s）
//   B/START 键优雅趴下；q 键优雅退出（失能轮 + 腿回位）；Ctrl+C 硬急停。
//   向前溜车抵消 CMD_BIAS_VX 仍叠加在 cmd[0]，手柄中位时站住不溜。
void Example37_RLTeleopControl() {
    printf("\n========== Example 37: RL 遥操作（手柄，走 MotionController） ==========\n");
    printf("[INFO] 50 Hz RL 循环。左摇杆=前进/后退，右摇杆=转向，B/START=趴下，q=优雅退出。\n\n");

    const int HZ = 50;

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    motor_mgr.SetTransport(&Usb2CanTransport::GetInstance());   // 4 路全 USB2CAN（与 Example36 一致）
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 预写固件模式：腿 IMPEDANCE + 轮 SPEED（固件阻抗忽略 vel_des，轮速度控制走 SPEED）
    // 轮 0 速弱增益预置（软启动，假速度偏移窗口内力矩小）；腿由 sendInterpFrame PD 接管。
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            motor_mgr.SetControlMode(cp, mi, IMPEDANCE);
    for (int cp = 0; cp < 4; cp++)
        motor_mgr.SetControlMode(cp, 4, SPEED);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        motor_mgr.SendSpeed(cp, 4, 0.0f, rl::WHEEL_SOFT_KVP, 0.0f);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.EnableMotor(cp, mi);
    usleep(200000);

    // IMU（可选）
    ImuDevice imu;
    imu.SetMount(ImuMount::Z_DOWN_X);
    bool imu_ok = imu.Initialize("/dev/ttyUSB0", 115200);
    if (!imu_ok)
        printf("[WARN] IMU 打开失败，gyro/quat 用默认值（机器人会失控，务必急停）\n");

    // 手柄速度命令（对齐 sim2sim.py GamepadReader；未连接时 cmd 保持 bias 站立）
    XboxController controller;
    bool pad_ok = controller.Initialize();
    if (!pad_ok)
        printf("[WARN] 手柄未连接，cmd 保持向后 bias 站立；接入后需重启生效\n");

    // ---- 运控层：起立到 DEFAULT_POSE + RL 循环 ----
    MotionController motion;
    MotionController::Config mcfg;
    mcfg.hz          = HZ;
    mcfg.max_vx      = 0.5f;   // 移动量程（用户 2026-08-30：vx 保持 0.5 保守）
    mcfg.max_wz      = 1.0f;   // 转向量程（用户 2026-08-30 满推改回 1.0 = 训练上限）
    mcfg.cmd_bias_vx = -0.05f;  // 抵消策略 wheel action 正向偏置（与 Example36 的 CMD_BIAS_VX 对齐）
    motion.setConfig(mcfg);
    motion.init(motor_mgr, imu_ok ? &imu : nullptr);

    printf("[INFO] 起立中（10s）...\n");
    if (!motion.standTo(rl::DEFAULT_POSE, 10.0f, []() { return g_rl_stop != 0; })) {
        printf("[WARN] 起立被中止，直接失能退出\n");
        signal(SIGINT, SIG_DFL);
        motion.emergencyStop();
        imu.Shutdown();
        controller.Shutdown();
        thread_mgr.stop_thread("motor_receive");
        thread_mgr.stop_thread("motor_send");
        motor_mgr.Stop();
        printf("[INFO] Example37 完成（起立中止）\n");
        return;
    }
    printf("[INFO] 起立完成，进入 RL 遥操作循环\n");

    // ---- RL 主循环（50 Hz，手柄遥操作）----
    g_rl_stop = 0;
    signal(SIGINT, rl_signal_handler);

    float cmd[3] = {0.0f, 0.0f, 0.0f};
    motion.beginRL(cmd);

    printf("[INFO] RL 循环启动：左摇杆前进/后退，B 急停，q 优雅退出\n");

    RawTerminal term;         // q 键优雅退出
    bool graceful = false;
    bool do_lie_down = false; // START 键触发优雅趴下（身体缓降着地）
    while (!g_rl_stop) {
        // 0) 键盘检测：按 q 优雅退出（失能轮 + 腿回位）
        if (term.ok) {
            unsigned char key;
            while (read(STDIN_FILENO, &key, 1) == 1)
                if (key == 'q' || key == 'Q') { graceful = true; g_rl_stop = 1; }
        }

        // 0.5) 手柄速度命令：左摇杆上推=+vx；右摇杆左推=+wz；B/START 键趴下
        if (pad_ok) {
            controller.Poll();
            const XboxState& st = controller.GetState();
            cmd[0] = -st.left_stick_y  * mcfg.max_vx + mcfg.cmd_bias_vx;
            cmd[1] =  0.0f;
            // 转向统一映射：左右同增益 1.0（用户 2026-08-30 改回，转向不到位另查原因）
            cmd[2] = -st.right_stick_x * mcfg.max_wz;
            // B/START 键：优雅趴下（Ctrl+C 仍为硬急停，轮子超速自动急停兜底）
            if (st.b || st.start) {
                printf("[INFO] 手柄 B/START 键：优雅趴下\n");
                do_lie_down = true;
                g_rl_stop = 1;
            }
            motion.setCmd(cmd);
        }

        // 1) RL 一步（读状态→obs→推理→下发→跌倒检测）
        if (!motion.rlStep()) {
            printf("[WARN] 跌倒检测触发 (proj_grav_z=%.2f)，急停\n", motion.lastGravZ());
            break;
        }

        // 2) 诊断打印（每 25 步，log_control.h 开关控制）
        int step = motion.step();
        if (step % 25 == 0) {
            const float* a   = motion.lastAction();
            const float* tw  = motion.lastTauWheel();
            const float* obs = motion.lastObs();
            LOG(LogCat::RL, "[RL %4d] cmd(%+.1f,%+.1f,%+.1f) aW[%+.2f %+.2f %+.2f %+.2f] tauW[%+.2f %+.2f %+.2f %+.2f]\n",
                step, cmd[0], cmd[1], cmd[2],
                a[12], a[13], a[14], a[15], tw[0], tw[1], tw[2], tw[3]);
            LOG(LogCat::IMU, "  pgr(%+.3f,%+.3f,%+.3f) av(%+.2f,%+.2f,%+.2f)\n",
                obs[6], obs[7], obs[8], obs[3], obs[4], obs[5]);
            fflush(stdout);
        }

        // 3) RL 诊断写入 CSV
        MotorLogger::GetInstance().LogRL(step, cmd, motion.lastObs(),
                                         motion.lastAction(), motion.lastTauWheel());

        usleep(1000000 / HZ);
    }

    // ---- 优雅退出：START=趴下 / q=腿回初始 ----
    // ⚠ 退出循环时 g_rl_stop=1（B/START/q 键设的），lieDown/returnToStart 的 is_stopped
    //   回调会立即中止（第一帧就返回 true）→ 趴下/回位不生效（024949 日志实证）。
    //   必须重置为 0 才放行；趴下/回位期间 Ctrl+C 会再次设 g_rl_stop=1 中止。
    if (do_lie_down) {
        printf("[INFO] 优雅趴下（12s 身体缓降，轮子着地）...\n");
        g_rl_stop = 0;
        motion.lieDown(12.0f, []() { return g_rl_stop != 0; });
        printf("[INFO] 已趴下，保持 2s 稳定...\n");
        sleep(2);
    } else if (graceful) {
        printf("[INFO] 优雅退出：失能轮电机，腿回初始姿态（10s）...\n");
        g_rl_stop = 0;
        motion.returnToStart(10.0f, []() { return g_rl_stop != 0; });
        printf("\n[INFO] 腿已回初始姿态。\n");
    }

    signal(SIGINT, SIG_DFL);

    // ---- 清理 ----
    printf("[INFO] 正在失能...\n");
    motion.emergencyStop();

    imu.Shutdown();
    controller.Shutdown();
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    printf("[INFO] Example37 完成\n");
}
// ================= 示例 38：执行器延迟辨识（Action Delay 测量） =================
// 目的：定出训练侧 action_delay_steps 的可信值（不靠估算，靠实测相位滞后）。
// 原理：给执行器发正弦命令，测反馈基波相对命令基波的相位滞后 φ(f)。
//       纯传输延迟在频域表现为线性相位：φ = -2πf·T_delay，T = |φ|/(2πf) 与 f 无关。
//       若 T 随 f 上升说明叠加了一阶惯性，取低频段 T 更接近纯延迟。
// 阶段A 腿位置扫频（4×calf，±0.03 rad，用 RL 的 KP/KD）：位置命令→实际位置闭环。
//       ⚠ 腿是位置闭环，滞后含 PD+机械惯性（低频即滞后），只是"上界"，非纯延迟。
// 阶段B 轮扭矩扫频（悬空，±1.0 Nm）：扭矩→轮速是开环积分环节，恒有 -90° 相移，
//       扣除 -90° 后余量 = 纯传输延迟（主判据）。死区摩擦污染低速段，取 4~6Hz 较可信。
// 关键实现点：命令与反馈用同一 steady_clock 实际时间轴（循环抖动不引入相位误差）；
//       反馈滞后为正 dphi（fit 用 sin 基准，滞后 δ 得 phase_f = 90°+ωδ）。
// 命令 50Hz（与 RL 一致，每步 20ms），最终 steps = round(T_wheel/20ms)。
void Example38_ActionDelayMeasure() {
    printf("\n========== 示例 38：执行器延迟辨识（Action Delay） ==========\n");
    printf("[WARN] 将使能 16 电机并起立到 RL 默认姿态（10s），四腿支撑、轮子悬空。\n");
    printf("       阶段A 腿位置扫频（calf ±0.03rad）→ 闭环滞后（上界）；\n");
    printf("       阶段B 轮扭矩扫频（±1.0Nm）→ 纯传输延迟（主判据）。\n");
    printf("       全程约 1 分钟。Ctrl+C 随时急停。结果给出建议 action_delay_steps。\n\n");

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) { printf("[ERROR] MotorManager 初始化失败\n"); return; }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    signal(SIGINT, rl_signal_handler);
    g_rl_stop = 0;
    const int   HZ = 50;                 // 与 RL 控制频率一致
    const float DT = 1.0f / HZ;

    auto cleanup = [&]() {
        printf("[INFO] 失能 16 电机\n");
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 4; mi++)
                motor_mgr.DisableMotor(cp, mi);
        thread_mgr.stop_thread("motor_receive");
        thread_mgr.stop_thread("motor_send");
        motor_mgr.Stop();
    };

    // ---- 使能 16 电机（阻抗模式，先写模式后使能）----
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.SetControlMode(cp, mi, IMPEDANCE);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.EnableMotor(cp, mi);
    usleep(200000);

    // ---- 起立到 DEFAULT_POSE（10s 慢插值，与 RL 循环目标一致避免跳变）----
    float stand_q[12], start_pos[12];
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < 3; j++) {
            int p = leg * 3 + j;
            stand_q[leg * 3 + j] = rl::urdf_to_status(rl::DEFAULT_POSE[p], p);
            start_pos[leg * 3 + j] = motor_mgr.GetStatus(leg, j + 1).position;
        }
    }
    printf("[INFO] 起立中（10s）...\n");
    const int STAND_FRAMES = 500;
    for (int f = 0; f <= STAND_FRAMES && !g_rl_stop; f++) {
        float t = (float)f / STAND_FRAMES;
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++)
                motor_mgr.SendImpedance(leg, j + 1,
                    start_pos[leg * 3 + j] + (stand_q[leg * 3 + j] - start_pos[leg * 3 + j]) * t,
                    0.0f, 200.0f, 20.0f, 0.0f);
        for (int cp = 0; cp < 4; cp++)
            motor_mgr.SendImpedance(cp, 4, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        if (f % 100 == 0) { printf("\r  [%3.0f%%] ", t * 100); fflush(stdout); }
        usleep(1000000 / HZ);
    }
    printf("\n[INFO] 起立完成。\n");
    if (g_rl_stop) { cleanup(); return; }

    // ---- 基波最小二乘拟合：y = a·cos(ωt) + b·sin(ωt) + c ----
    // 返回基波幅度与相位（atan2(b,a)）。窗口内信号须是稳定正弦（丢过渡段）。
    auto fit_wave = [](const float* t, const float* y, int n, float f,
                       float& amp, float& phase) -> bool {
        if (n < 8) return false;
        float ym = 0.0f;
        for (int i = 0; i < n; i++) ym += y[i];
        ym /= n;
        float w = 2.0f * 3.14159265f * f;
        float Scc = 0, Scs = 0, Sss = 0, Scy = 0, Ssy = 0;
        for (int i = 0; i < n; i++) {
            float c = cosf(w * t[i]), s = sinf(w * t[i]);
            float yd = y[i] - ym;
            Scc += c * c; Scs += c * s; Sss += s * s;
            Scy += c * yd; Ssy += s * yd;
        }
        float det = Scc * Sss - Scs * Scs;
        if (fabsf(det) < 1e-9f) return false;
        float a = (Scy * Sss - Scs * Ssy) / det;
        float b = (Scc * Ssy - Scs * Scy) / det;
        amp = sqrtf(a * a + b * b);
        phase = atan2f(b, a);
        return true;
    };
    // 归一化相位差到 [-π, π]
    auto wrap_pi = [](float p) {
        while (p > 3.14159265f)  p -= 2.0f * 3.14159265f;
        while (p < -3.14159265f) p += 2.0f * 3.14159265f;
        return p;
    };

    struct FRes { float f; float t_ms; };
    const int MAX_RES = 64;
    FRes leg_res[MAX_RES];  int leg_n = 0;
    FRes wres[MAX_RES];     int wres_n = 0;
    float tbuf[256], cbuf[256], fbuf[256];

    // ---- 实际时钟基准：命令与反馈用同一真实时间轴，循环抖动不引入相位误差 ----
    using clk = std::chrono::steady_clock;
    auto t_zero = clk::now();
    auto t_now  = [&]() {
        return std::chrono::duration<float>(clk::now() - t_zero).count();
    };

    // ============ 阶段A：腿位置扫频（4 条腿 calf，位置 PD 约束，安全） ============
    // ⚠ 腿是位置闭环：测出的滞后含 PD+机械惯性（低频即滞后），是"上界"而非纯传输延迟。
    //   纯延迟的主判据看阶段B 轮通道（开环扭矩→速度，扣除积分 -90° 后余量 = 纯延迟）。
    const float LEG_AMP     = 0.03f;                       // ±0.03 rad ≈ ±1.7°，支撑状态不晃机身
    const float LEG_FREQ[4] = {0.5f, 1.0f, 2.0f, 3.0f};
    const float LEG_DUR     = 2.5f;
    const int   NS          = (int)(LEG_DUR * HZ);         // 125 点
    printf("\n========== 阶段A：腿位置扫频（4×calf，±%.2f rad，闭环滞后/上界） ==========\n", LEG_AMP);
    for (int leg = 0; leg < 4 && !g_rl_stop; leg++) {
        float q0 = motor_mgr.GetStatus(leg, 3).position;  // 扫频基准 = 当前 calf 角
        for (int fi = 0; fi < 4 && !g_rl_stop; fi++) {
            float f = LEG_FREQ[fi];
            float w = 2.0f * 3.14159265f * f;
            for (int k = 0; k < NS; k++) {
                float t = t_now();                        // 实际时刻（命令/反馈同轴）
                tbuf[k] = t;
                cbuf[k] = LEG_AMP * sinf(w * t);
                for (int L = 0; L < 4; L++)
                    for (int j = 0; j < 3; j++)
                        motor_mgr.SendImpedance(L, j + 1,
                            (L == leg && j == 2) ? (q0 + cbuf[k]) : stand_q[L * 3 + j],
                            0.0f, rl::LEG_KP, rl::LEG_KD, 0.0f);
                fbuf[k] = motor_mgr.GetStatus(leg, 3).position - q0;
                usleep(1000000 / HZ);
            }
            int skip = (int)(0.5f / f / DT) + 2;          // 丢前 0.5 周期（低频过渡更长）
            if (skip > NS - 10) skip = NS - 10;
            float amp_c, ph_c, amp_f, ph_f;
            bool ok = fit_wave(tbuf + skip, cbuf + skip, NS - skip, f, amp_c, ph_c) &&
                      fit_wave(tbuf + skip, fbuf + skip, NS - skip, f, amp_f, ph_f);
            if (ok && amp_f > 1e-4f) {
                float dphi = wrap_pi(ph_f - ph_c);        // 滞后为正（物理）
                float tms  = dphi / w * 1000.0f;          // 等效延迟（含闭环动态，上界）
                if (tms > 0) leg_res[leg_n++] = {f, tms};
                printf("  腿%d calf  f=%4.1fHz  滞后=%+6.1f°  T_上界=%6.1f ms  amp=%.4f rad\n",
                       leg, f, dphi * 180.0f / 3.14159265f, tms, amp_f);
            } else {
                printf("  [跳过] 腿%d calf f=%4.1fHz 拟合失败/幅值过小\n", leg, f);
            }
            usleep(300000);   // 频率间静置，位姿稳定
        }
    }

    // ============ 阶段B：轮子扭矩扫频（悬空，主判据） ============
    // 扭矩→轮速是积分环节（恒滞后 -90°），总滞后扣除 -90° 后余量 = 纯传输延迟 δ。
    // 死区摩擦会污染低速段，4~6Hz 较可信。W_AMP 降到 1.0 减少轮速超限被跳过。
    const float W_AMP     = 1.0f;                          // 超过静摩擦(0.5~0.8Nm)，连续转动
    const float W_FREQ[3] = {2.0f, 4.0f, 6.0f};            // 低频下扭矩→速度幅值过大，从 2Hz 起
    const float W_DUR     = 1.5f;
    const int   WN        = (int)(W_DUR * HZ);             // 75 点
    const int   WSKIP     = 10;
    printf("\n========== 阶段B：轮扭矩扫频（±%.2f Nm，悬空，扣除积分-90°=纯延迟） ==========\n", W_AMP);
    for (int w = 0; w < 4 && !g_rl_stop; w++) {
        for (int fi = 0; fi < 3 && !g_rl_stop; fi++) {
            float f = W_FREQ[fi];
            float ww = 2.0f * 3.14159265f * f;
            bool over = false;
            for (int k = 0; k < WN; k++) {
                float t = t_now();
                tbuf[k] = t;
                cbuf[k] = W_AMP * sinf(ww * t);
                for (int L = 0; L < 4; L++)
                    for (int j = 0; j < 3; j++)
                        motor_mgr.SendImpedance(L, j + 1, stand_q[L * 3 + j],
                                                0.0f, rl::LEG_KP, rl::LEG_KD, 0.0f);
                for (int c = 0; c < 4; c++)
                    motor_mgr.SendImpedance(c, 4, 0.0f, 0.0f, 0.0f, 0.0f,
                                            c == w ? cbuf[k] : 0.0f);
                float vel = motor_mgr.GetStatus(w, 4).velocity;
                fbuf[k] = vel;
                if (fabsf(vel) > 8.0f) over = true;       // 幅值过大保护，跳过该频率
                usleep(1000000 / HZ);
            }
            if (over) {
                printf("  [跳过] 轮%d f=%4.1fHz 轮速超 8 rad/s\n", w, f);
                continue;
            }
            float amp_c, ph_c, amp_f, ph_f;
            bool ok = fit_wave(tbuf + WSKIP, cbuf + WSKIP, WN - WSKIP, f, amp_c, ph_c) &&
                      fit_wave(tbuf + WSKIP, fbuf + WSKIP, WN - WSKIP, f, amp_f, ph_f);
            if (ok && amp_f > 1e-3f) {
                float dphi = wrap_pi(ph_f - ph_c);        // 总滞后（含积分 -90°），滞后为正
                if (dphi > 1.5707963f) {                  // 总滞后必须 > 90° 才有正延迟
                    float tms = (dphi - 1.5707963f) / ww * 1000.0f;
                    if (tms > 0) wres[wres_n++] = {f, tms};
                    printf("  轮%d  f=%4.1fHz  总滞后=%+6.1f°  纯延迟 T=%6.1f ms  amp=%.3f\n",
                           w, f, dphi * 180.0f / 3.14159265f, tms, amp_f);
                } else {
                    printf("  [跳过] 轮%d f=%4.1fHz 总滞后<90°（摩擦/噪声），无效\n", w, f);
                }
            } else {
                printf("  [跳过] 轮%d f=%4.1fHz 拟合失败/幅值过小\n", w, f);
            }
            usleep(300000);
        }
    }

    // ---- 汇总 ----
    printf("\n========== 结果汇总 ==========\n");
    auto avg_t = [](const FRes* r, int n) -> float {
        if (n <= 0) return 0.0f;
        float s = 0; for (int i = 0; i < n; i++) s += r[i].t_ms;
        return s / n;
    };
    float t_leg = avg_t(leg_res, leg_n);   // 上界
    float t_w   = avg_t(wres, wres_n);     // 纯延迟（主判据）
    printf("  腿闭环滞后（上界）：%d 个有效点，平均 T_leg ≈ %.1f ms（含 PD+机械惯性，仅作上限参考）\n",
           leg_n, t_leg);
    printf("  轮纯延迟（主判据）：%d 个有效点，平均 T_wheel ≈ %.1f ms（扣除积分 -90°）\n",
           wres_n, t_w);

    float t_total = t_w > 0.0f ? t_w : t_leg;   // 主判据无效时退用腿上界
    int steps = (int)roundf(t_total / 20.0f);
    if (steps < 1) steps = 1;
    printf("\n  纯传输延迟 T_delay ≈ %.1f ms → 建议 action_delay_steps = %d（50Hz，每步 20ms）\n",
           t_total, steps);
    if (wres_n == 0 && leg_n == 0)
        printf("  ⚠ 两通道均无有效数据，请检查电机连接、幅度/频率参数\n");
    else if (wres_n == 0)
        printf("  ⚠ 轮通道无有效数据，以上界 T_leg 代替（偏保守）；建议调低 W_AMP 后重测轮通道\n");

    cleanup();
    printf("[INFO] 示例 38 完成\n");
}

// ================= 示例 51：起立 → RL 站立 → 回车趴下 =================
// 目的：完整流程验证——插值起立到 STAND，进 RL 站立循环（50Hz），按回车缓慢趴下。
// 流程：起立(10s STAND {0,-60,60}) → 过渡 DEFAULT_POSE(1s) → RL 站立循环(回车退出) →
//       趴下(12s 缓降到 LIE_DOWN，手动标定值) → 保持 2s → 失能。
// 安全：起立/趴下慢速插值、轮子 0 速弱增益 + 站立门控锁轮、跌倒检测、Ctrl+C 急停。
void Example51_StandRLThenLieDown() {
    printf("\n========== Example 51: 起立 → RL 站立 → 回车趴下 ==========\n");
    printf("[INFO] 起立后 RL 站立，按回车触发缓慢趴下（LIE_DOWN）。Ctrl+C 急停。\n\n");
    const int HZ = 50;

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) { printf("[ERROR] 初始化失败\n"); return; }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 使能：腿 IMPEDANCE（PreEnable 零扭矩）+ 轮 SPEED 0 速弱增益（软启动）
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            motor_mgr.SetControlMode(cp, mi, IMPEDANCE);
    for (int cp = 0; cp < 4; cp++)
        motor_mgr.SetControlMode(cp, 4, SPEED);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            motor_mgr.PreEnableZeroTorque(cp, mi);
    for (int cp = 0; cp < 4; cp++)
        motor_mgr.SendSpeed(cp, 4, 0.0f, rl::WHEEL_SOFT_KVP, 0.0f);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.EnableMotor(cp, mi);
    usleep(200000);

    // IMU（可选）
    ImuDevice imu;
    imu.SetMount(ImuMount::Z_DOWN_X);
    bool imu_ok = imu.Initialize("/dev/ttyUSB0", 115200);
    if (!imu_ok) printf("[WARN] IMU 打开失败，gyro/quat 用默认值\n");

    // ---- 起立到 STAND（真机角 {0,-60,60}，10s）----
    float stand_q[12];
    for (int leg = 0; leg < 4; leg++) {
        stand_q[leg * 3 + 0] = deg2rad(STAND_HIP_DEG);
        stand_q[leg * 3 + 1] = deg2rad(STAND_THIGH_DEG);
        stand_q[leg * 3 + 2] = deg2rad(STAND_CALF_DEG);
    }
    float start_pos[12];
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            start_pos[cp * 3 + mi - 1] = motor_mgr.GetStatus(cp, mi).position;
    printf("[INFO] 起立中（10s 到 STAND {0,-60,60}°）...\n");
    for (int f = 0; f <= 500; f++) {
        if (g_rl_stop) break;
        float t = (float)f / 500;
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 3; mi++) {
                float pos = start_pos[cp * 3 + mi - 1]
                          + (stand_q[cp * 3 + mi - 1] - start_pos[cp * 3 + mi - 1]) * t;
                motor_mgr.SendImpedance(cp, mi, pos, 0.0f, 200.0f, 20.0f, 0.0f);
            }
        if (f % 100 == 0) printf("  起立 %4.0f%%\n", t * 100.0f);
        usleep(1000000 / HZ);
    }

    // ---- 过渡到 DEFAULT_POSE（1s，RL 基准）----
    float dp_q[12];
    for (int leg = 0; leg < 4; leg++)
        for (int j = 0; j < 3; j++)
            dp_q[leg * 3 + j] = rl::urdf_to_status(rl::DEFAULT_POSE[leg * 3 + j], leg * 3 + j);
    printf("[INFO] 过渡到 DEFAULT_POSE（1s）...\n");
    for (int f = 0; f <= 50; f++) {
        if (g_rl_stop) break;
        float t = (float)f / 50;
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 3; mi++) {
                float pos = stand_q[cp * 3 + mi - 1]
                          + (dp_q[cp * 3 + mi - 1] - stand_q[cp * 3 + mi - 1]) * t;
                motor_mgr.SendImpedance(cp, mi, pos, 0.0f, 200.0f, 20.0f, 0.0f);
            }
        usleep(1000000 / HZ);
    }
    printf("[INFO] 已站立，进入 RL 站立循环\n");

    // ---- RL 站立循环（50Hz），回车触发退出 → 趴下 ----
    g_rl_stop = 0;
    signal(SIGINT, rl_signal_handler);
    float last_action[16] = {0.0f};
    float wheel_v_lp[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    const float CMD_BIAS_VX = -0.05f;   // 抵消策略前冲（同 Example36）
    float cmd[3] = {CMD_BIAS_VX, 0.0f, 0.0f};
    int step = 0;
    printf("[INFO] RL 站立循环中（cmd 固定 %+.2f 站立）。按回车趴下，Ctrl+C 急停\n", CMD_BIAS_VX);
    printf("  （等待回车...）\n"); fflush(stdout);

    while (!g_rl_stop) {
        // 读 16 电机（CAN 顺序）
        float pos_can[16], vel_can[16];
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 4; mi++) {
                MotorStatus st = motor_mgr.GetStatus(cp, mi);
                int mjx = cp * 4 + (mi - 1);
                pos_can[mjx] = st.position;
                vel_can[mjx] = st.velocity;
            }
        // CAN → URDF
        float pos_policy[16], vel_policy[16];
        for (int i = 0; i < 16; i++) {
            pos_policy[i] = rl::status_to_urdf(pos_can[rl::MJX_TO_POLICY[i]], i);
            vel_policy[i] = rl::status_vel_to_urdf(vel_can[rl::MJX_TO_POLICY[i]], i);
        }
        float gyro[3] = {0, 0, 0}, quat[4] = {1, 0, 0, 0};
        if (imu_ok) {
            imu.GetGyro(gyro[0], gyro[1], gyro[2]);
            imu.GetQuat(quat[0], quat[1], quat[2], quat[3]);
        }
        // obs → 推理 → 下发
        float obs[64];
        rl::build_observation(gyro, quat, pos_policy, vel_policy, last_action, cmd, step, obs);
        float action[16];
        rl::mlp_forward(obs, action);
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 4; mi++) {
                int mjx = cp * 4 + (mi - 1);
                int p = rl::POLICY_TO_MJX[mjx];
                if (mi <= 3) {
                    float q = rl::urdf_to_status(rl::leg_pos_target(action[p], p), p);
                    const JointImpedanceParam& ip = GetJointImpedance(cp, mi);
                    motor_mgr.SendImpedance(cp, mi, q, 0.0f, rl::LEG_KP, rl::LEG_KD, ip.tau_ff);
                } else {
                    int w = p - rl::NUM_LEG_JOINTS;
                    float tv = rl::WHEEL_VEL_SCALE * action[p];
                    wheel_v_lp[w] += rl::WHEEL_CMD_ALPHA * (tv - wheel_v_lp[w]);
                    float cv = wheel_v_lp[w];
                    // 站立门控：无移动指令锁轮（防溜车）
                    if (fabsf(cmd[0]) < rl::WHEEL_CMD_MOVE_THR && fabsf(cmd[2]) < rl::WHEEL_CMD_MOVE_THR)
                        cv = 0.0f;
                    motor_mgr.SendSpeed(cp, mi, cv, rl::WHEEL_KVP, rl::WHEEL_KVI);
                }
            }
        for (int i = 0; i < 16; i++) last_action[i] = action[i];
        // 跌倒检测
        if (obs[8] > -0.34f) { printf("[WARN] 跌倒检测触发 (pgr_z=%.2f)，急停\n", obs[8]); break; }
        // 回车检测（stdin 非阻塞）
        struct pollfd pfd = {STDIN_FILENO, POLLIN, 0};
        if (poll(&pfd, 1, 0) > 0) {
            char c = 0;
            if (read(STDIN_FILENO, &c, 1) == 1 && c == '\n') {
                printf("\n[INFO] 收到回车，开始缓慢趴下\n");
                break;
            }
        }
        step++;
        usleep(1000000 / HZ);
    }
    signal(SIGINT, SIG_DFL);

    // ---- 趴下（12s 缓降到 LIE_DOWN，轮 0 速弱增益保持）----
    float lie_q[12];
    for (int leg = 0; leg < 4; leg++) {
        lie_q[leg * 3 + 0] = deg2rad(LIE_DOWN_HIP_DEG);
        lie_q[leg * 3 + 1] = deg2rad(LIE_DOWN_THIGH_DEG);
        lie_q[leg * 3 + 2] = deg2rad(LIE_DOWN_CALF_DEG);
    }
    float cur_pos[12];
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            cur_pos[cp * 3 + mi - 1] = motor_mgr.GetStatus(cp, mi).position;
    printf("[INFO] 趴下中（12s 缓降到 LIE_DOWN {%.0f,%.0f,%.0f}°）...\n",
           LIE_DOWN_HIP_DEG, LIE_DOWN_THIGH_DEG, LIE_DOWN_CALF_DEG);
    for (int f = 0; f <= 600; f++) {
        if (g_rl_stop) break;
        float t = (float)f / 600;
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 3; mi++) {
                float pos = cur_pos[cp * 3 + mi - 1]
                          + (lie_q[cp * 3 + mi - 1] - cur_pos[cp * 3 + mi - 1]) * t;
                motor_mgr.SendImpedance(cp, mi, pos, 0.0f, 200.0f, 20.0f, 0.0f);
            }
        if (f % 100 == 0) printf("  趴下 %4.0f%%\n", t * 100.0f);
        usleep(1000000 / HZ);
    }
    printf("[INFO] 已趴下，保持 2s 稳定...\n");
    sleep(2);

    // ---- 清理 ----
    printf("[INFO] 失能...\n");
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.DisableMotor(cp, mi);
    imu.Shutdown();
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    printf("[INFO] 示例51 完成\n");
}

// ================= 示例 52：固定转向命令 RL（sim2real 对比用） =================
// 目的：真机跑固定 yaw=0.5 转向 5s，产出 rl_*.csv 轨迹，与 sim2sim 的 --record 输出对比
//       （同策略 v28/3000、同命令，隔离真机执行差异）。
// 流程：起立 STAND → 过渡 DEFAULT_POSE → RL 循环 cmd 固定 {0,0,0.5} 跑 250 步(5s) → 趴下失能。
// 对比数据：log/rl_*.csv（qrel/vel/act/pgr，POLICY order，与 sim2sim --record 字段对齐）。
// 安全：站立门控锁轮（防溜）、跌倒检测、Ctrl+C 急停。5s 是实测安全上限（10s 可能倒地）。
void Example52_FixedCmdYaw() {
    printf("\n========== Example 52: 固定转向命令 RL（yaw=0.5，5s，sim2real 对比） ==========\n");
    const int HZ = 50;
    const int RUN_STEPS = 250;              // 5s
    const float CMD[3] = {0.0f, 0.0f, 0.5f}; // vx=0, vy=0, yaw=0.5（对齐 sim2sim）

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) { printf("[ERROR] 初始化失败\n"); return; }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 使能：腿 IMPEDANCE + 轮 SPEED 0 速弱增益（软启动）
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            motor_mgr.SetControlMode(cp, mi, IMPEDANCE);
    for (int cp = 0; cp < 4; cp++)
        motor_mgr.SetControlMode(cp, 4, SPEED);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            motor_mgr.PreEnableZeroTorque(cp, mi);
    for (int cp = 0; cp < 4; cp++)
        motor_mgr.SendSpeed(cp, 4, 0.0f, rl::WHEEL_SOFT_KVP, 0.0f);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.EnableMotor(cp, mi);
    usleep(200000);

    ImuDevice imu;
    imu.SetMount(ImuMount::Z_DOWN_X);
    bool imu_ok = imu.Initialize("/dev/ttyUSB0", 115200);
    if (!imu_ok) printf("[WARN] IMU 打开失败，gyro/quat 用默认值\n");

    // ---- 起立到 STAND（10s）----
    float stand_q[12];
    for (int leg = 0; leg < 4; leg++) {
        stand_q[leg * 3 + 0] = deg2rad(STAND_HIP_DEG);
        stand_q[leg * 3 + 1] = deg2rad(STAND_THIGH_DEG);
        stand_q[leg * 3 + 2] = deg2rad(STAND_CALF_DEG);
    }
    float start_pos[12];
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            start_pos[cp * 3 + mi - 1] = motor_mgr.GetStatus(cp, mi).position;
    printf("[INFO] 起立中（10s 到 STAND {0,-60,60}°）...\n");
    for (int f = 0; f <= 500; f++) {
        if (g_rl_stop) break;
        float t = (float)f / 500;
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 3; mi++) {
                float pos = start_pos[cp * 3 + mi - 1]
                          + (stand_q[cp * 3 + mi - 1] - start_pos[cp * 3 + mi - 1]) * t;
                motor_mgr.SendImpedance(cp, mi, pos, 0.0f, 200.0f, 20.0f, 0.0f);
            }
        if (f % 100 == 0) printf("  起立 %4.0f%%\n", t * 100.0f);
        usleep(1000000 / HZ);
    }
    // 过渡 DEFAULT_POSE（1s）
    float dp_q[12];
    for (int leg = 0; leg < 4; leg++)
        for (int j = 0; j < 3; j++)
            dp_q[leg * 3 + j] = rl::urdf_to_status(rl::DEFAULT_POSE[leg * 3 + j], leg * 3 + j);
    printf("[INFO] 过渡到 DEFAULT_POSE（1s）...\n");
    for (int f = 0; f <= 50; f++) {
        if (g_rl_stop) break;
        float t = (float)f / 50;
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 3; mi++) {
                float pos = stand_q[cp * 3 + mi - 1]
                          + (dp_q[cp * 3 + mi - 1] - stand_q[cp * 3 + mi - 1]) * t;
                motor_mgr.SendImpedance(cp, mi, pos, 0.0f, 200.0f, 20.0f, 0.0f);
            }
        usleep(1000000 / HZ);
    }
    printf("[INFO] 已站立，进入 RL 转向循环（yaw=0.5，5s）...\n");

    // ---- RL 循环：cmd 固定 {0,0,0.5}，跑 RUN_STEPS 步 ----
    g_rl_stop = 0;
    signal(SIGINT, rl_signal_handler);
    float last_action[16] = {0.0f};
    float wheel_v_lp[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float cmd[3] = {CMD[0], CMD[1], CMD[2]};
    int step = 0;
    bool fell = false;

    for (step = 0; step < RUN_STEPS && !g_rl_stop; step++) {
        float pos_can[16], vel_can[16];
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 4; mi++) {
                MotorStatus st = motor_mgr.GetStatus(cp, mi);
                int mjx = cp * 4 + (mi - 1);
                pos_can[mjx] = st.position;
                vel_can[mjx] = st.velocity;
            }
        float pos_policy[16], vel_policy[16];
        for (int i = 0; i < 16; i++) {
            pos_policy[i] = rl::status_to_urdf(pos_can[rl::MJX_TO_POLICY[i]], i);
            vel_policy[i] = rl::status_vel_to_urdf(vel_can[rl::MJX_TO_POLICY[i]], i);
        }
        float gyro[3] = {0, 0, 0}, quat[4] = {1, 0, 0, 0};
        if (imu_ok) {
            imu.GetGyro(gyro[0], gyro[1], gyro[2]);
            imu.GetQuat(quat[0], quat[1], quat[2], quat[3]);
        }
        float obs[64];
        rl::build_observation(gyro, quat, pos_policy, vel_policy, last_action, cmd, step, obs);
        float action[16];
        rl::mlp_forward(obs, action);
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 4; mi++) {
                int mjx = cp * 4 + (mi - 1);
                int p = rl::POLICY_TO_MJX[mjx];
                if (mi <= 3) {
                    float q = rl::urdf_to_status(rl::leg_pos_target(action[p], p), p);
                    const JointImpedanceParam& ip = GetJointImpedance(cp, mi);
                    motor_mgr.SendImpedance(cp, mi, q, 0.0f, rl::LEG_KP, rl::LEG_KD, ip.tau_ff);
                } else {
                    int w = p - rl::NUM_LEG_JOINTS;
                    float tv = rl::WHEEL_VEL_SCALE * action[p];
                    wheel_v_lp[w] += rl::WHEEL_CMD_ALPHA * (tv - wheel_v_lp[w]);
                    float cv = wheel_v_lp[w];
                    // 转向命令（cmd[2]=0.5 > 阈值）放开轮控
                    if (fabsf(cmd[0]) < rl::WHEEL_CMD_MOVE_THR && fabsf(cmd[2]) < rl::WHEEL_CMD_MOVE_THR)
                        cv = 0.0f;
                    motor_mgr.SendSpeed(cp, mi, cv, rl::WHEEL_KVP, rl::WHEEL_KVI);
                }
            }
        for (int i = 0; i < 16; i++) last_action[i] = action[i];
        // 诊断：每 25 步打印轮速 + pgr（对比用）
        if (step % 25 == 0) {
            MotorStatus w0 = motor_mgr.GetStatus(0, 4);
            printf("  [step %4d] cmd(%.1f,%.1f,%.1f) vW0=%.2f pgr_z=%.2f\n",
                   step, cmd[0], cmd[1], cmd[2], w0.velocity, obs[8]);
            fflush(stdout);
        }
        if (obs[8] > -0.34f) { printf("[WARN] 跌倒检测触发 (pgr_z=%.2f)，急停\n", obs[8]); fell = true; break; }
        usleep(1000000 / HZ);
    }
    signal(SIGINT, SIG_DFL);
    printf("[INFO] 转向结束（%d 步%s），趴下...\n", step, fell ? "，中途跌倒" : "");

    // ---- 趴下（12s 缓降到 LIE_DOWN）----
    float lie_q[12];
    for (int leg = 0; leg < 4; leg++) {
        lie_q[leg * 3 + 0] = deg2rad(LIE_DOWN_HIP_DEG);
        lie_q[leg * 3 + 1] = deg2rad(LIE_DOWN_THIGH_DEG);
        lie_q[leg * 3 + 2] = deg2rad(LIE_DOWN_CALF_DEG);
    }
    float cur_pos[12];
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            cur_pos[cp * 3 + mi - 1] = motor_mgr.GetStatus(cp, mi).position;
    for (int f = 0; f <= 600; f++) {
        if (g_rl_stop) break;
        float t = (float)f / 600;
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 3; mi++) {
                float pos = cur_pos[cp * 3 + mi - 1]
                          + (lie_q[cp * 3 + mi - 1] - cur_pos[cp * 3 + mi - 1]) * t;
                motor_mgr.SendImpedance(cp, mi, pos, 0.0f, 200.0f, 20.0f, 0.0f);
            }
        if (f % 100 == 0) printf("  趴下 %4.0f%%\n", t * 100.0f);
        usleep(1000000 / HZ);
    }
    printf("[INFO] 已趴下，保持 2s...\n");
    sleep(2);

    // ---- 清理 ----
    printf("[INFO] 失能...\n");
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.DisableMotor(cp, mi);
    imu.Shutdown();
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    printf("[INFO] 示例52 完成\n");
}

// ================= 示例 53：RL 站立下重力前馈测量 =================
// 目的：进 RL 站立循环（策略控制腿，实际工作姿态），读各关节稳态 cal_torque，
//       作为 JOINT_IMPEDANCE.tau_ff 的重力前馈标定值。
// 原理：狗静止站立时电机输出扭矩 = 重力负载（动态项≈0），稳态均值即 tau_ff 需求。
//       用 RL 参数（rl::LEG_KP/KD + 当前 tau_ff）由策略控制站立，姿态更真实。
// 流程：起立 STAND → 过渡 DEFAULT_POSE → RL 站立 250 步 → 记录末 150 步(3s)
//       12 关节 cal_torque 稳态均值 → 打印建议 tau_ff → 失能。
// 注意：若狗站不稳（塌/抖），kp×err 会污染读数——需先保证站立稳定再采。
void Example53_MeasureGravityFF() {
    printf("\n========== Example 53: RL 站立下重力前馈测量 ==========\n");
    printf("[INFO] 进 RL 站立循环，记录 12 关节稳态 cal_torque（= 重力前馈需求）。\n\n");
    const int HZ = 50;
    const int WARM_STEPS = 100;   // 热身（让姿态稳定）
    const int REC_STEPS  = 1000;  // 记录 20s（更长采样，均值更稳）

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) { printf("[ERROR] 初始化失败\n"); return; }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 使能：腿 IMPEDANCE + 轮 SPEED 0 速弱增益
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            motor_mgr.SetControlMode(cp, mi, IMPEDANCE);
    for (int cp = 0; cp < 4; cp++)
        motor_mgr.SetControlMode(cp, 4, SPEED);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            motor_mgr.PreEnableZeroTorque(cp, mi);
    for (int cp = 0; cp < 4; cp++)
        motor_mgr.SendSpeed(cp, 4, 0.0f, rl::WHEEL_SOFT_KVP, 0.0f);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.EnableMotor(cp, mi);
    usleep(200000);

    ImuDevice imu;
    imu.SetMount(ImuMount::Z_DOWN_X);
    bool imu_ok = imu.Initialize("/dev/ttyUSB0", 115200);
    if (!imu_ok) printf("[WARN] IMU 打开失败，gyro/quat 用默认值\n");

    // ---- 起立到 STAND（10s）----
    float stand_q[12];
    for (int leg = 0; leg < 4; leg++) {
        stand_q[leg * 3 + 0] = deg2rad(STAND_HIP_DEG);
        stand_q[leg * 3 + 1] = deg2rad(STAND_THIGH_DEG);
        stand_q[leg * 3 + 2] = deg2rad(STAND_CALF_DEG);
    }
    float start_pos[12];
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            start_pos[cp * 3 + mi - 1] = motor_mgr.GetStatus(cp, mi).position;
    printf("[INFO] 起立中（10s 到 STAND {0,-60,60}°）...\n");
    for (int f = 0; f <= 500; f++) {
        if (g_rl_stop) break;
        float t = (float)f / 500;
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 3; mi++) {
                float pos = start_pos[cp * 3 + mi - 1]
                          + (stand_q[cp * 3 + mi - 1] - start_pos[cp * 3 + mi - 1]) * t;
                motor_mgr.SendImpedance(cp, mi, pos, 0.0f, 200.0f, 20.0f, 0.0f);
            }
        if (f % 100 == 0) printf("  起立 %4.0f%%\n", t * 100.0f);
        usleep(1000000 / HZ);
    }
    // 过渡 DEFAULT_POSE（1s）
    float dp_q[12];
    for (int leg = 0; leg < 4; leg++)
        for (int j = 0; j < 3; j++)
            dp_q[leg * 3 + j] = rl::urdf_to_status(rl::DEFAULT_POSE[leg * 3 + j], leg * 3 + j);
    printf("[INFO] 过渡到 DEFAULT_POSE（1s）...\n");
    for (int f = 0; f <= 50; f++) {
        if (g_rl_stop) break;
        float t = (float)f / 50;
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 3; mi++) {
                float pos = stand_q[cp * 3 + mi - 1]
                          + (dp_q[cp * 3 + mi - 1] - stand_q[cp * 3 + mi - 1]) * t;
                motor_mgr.SendImpedance(cp, mi, pos, 0.0f, 200.0f, 20.0f, 0.0f);
            }
        usleep(1000000 / HZ);
    }

    // ---- RL 站立循环：热身 + 记录 cal_torque ----
    g_rl_stop = 0;
    signal(SIGINT, rl_signal_handler);
    float last_action[16] = {0.0f};
    float wheel_v_lp[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    const float CMD_BIAS_VX = -0.05f;
    float cmd[3] = {CMD_BIAS_VX, 0.0f, 0.0f};
    float sum_tau[12] = {0.0f};
    int rec_cnt = 0;
    int step = 0;

    for (step = 0; step < WARM_STEPS + REC_STEPS && !g_rl_stop; step++) {
        float pos_can[16], vel_can[16];
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 4; mi++) {
                MotorStatus st = motor_mgr.GetStatus(cp, mi);
                int mjx = cp * 4 + (mi - 1);
                pos_can[mjx] = st.position;
                vel_can[mjx] = st.velocity;
            }
        float pos_policy[16], vel_policy[16];
        for (int i = 0; i < 16; i++) {
            pos_policy[i] = rl::status_to_urdf(pos_can[rl::MJX_TO_POLICY[i]], i);
            vel_policy[i] = rl::status_vel_to_urdf(vel_can[rl::MJX_TO_POLICY[i]], i);
        }
        float gyro[3] = {0, 0, 0}, quat[4] = {1, 0, 0, 0};
        if (imu_ok) {
            imu.GetGyro(gyro[0], gyro[1], gyro[2]);
            imu.GetQuat(quat[0], quat[1], quat[2], quat[3]);
        }
        float obs[64];
        rl::build_observation(gyro, quat, pos_policy, vel_policy, last_action, cmd, step, obs);
        float action[16];
        rl::mlp_forward(obs, action);
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 4; mi++) {
                int mjx = cp * 4 + (mi - 1);
                int p = rl::POLICY_TO_MJX[mjx];
                if (mi <= 3) {
                    float q = rl::urdf_to_status(rl::leg_pos_target(action[p], p), p);
                    const JointImpedanceParam& ip = GetJointImpedance(cp, mi);
                    motor_mgr.SendImpedance(cp, mi, q, 0.0f, rl::LEG_KP, rl::LEG_KD, ip.tau_ff);
                    // 记录段：累计稳态 cal_torque
                    if (step >= WARM_STEPS) {
                        sum_tau[cp * 3 + mi - 1] += motor_mgr.GetStatus(cp, mi).torque;
                    }
                } else {
                    int w = p - rl::NUM_LEG_JOINTS;
                    float tv = rl::WHEEL_VEL_SCALE * action[p];
                    wheel_v_lp[w] += rl::WHEEL_CMD_ALPHA * (tv - wheel_v_lp[w]);
                    float cv = wheel_v_lp[w];
                    if (fabsf(cmd[0]) < rl::WHEEL_CMD_MOVE_THR && fabsf(cmd[2]) < rl::WHEEL_CMD_MOVE_THR)
                        cv = 0.0f;
                    motor_mgr.SendSpeed(cp, mi, cv, rl::WHEEL_KVP, rl::WHEEL_KVI);
                }
            }
        for (int i = 0; i < 16; i++) last_action[i] = action[i];
        if (obs[8] > -0.34f) { printf("[WARN] 跌倒检测触发，急停\n"); break; }
        if (step >= WARM_STEPS) rec_cnt++;
        usleep(1000000 / HZ);
    }
    signal(SIGINT, SIG_DFL);

    // ---- 打印建议 tau_ff ----
    if (rec_cnt > 0) {
        const char* legname[4] = {"FL", "FR", "RL", "RR"};
        const char* jname[3]   = {"hip", "thigh", "calf"};
        printf("\n=== 重力前馈建议（RL 站立稳态 cal_torque 均值，%d 步）===\n", rec_cnt);
        for (int leg = 0; leg < 4; leg++) {
            printf("  %s: ", legname[leg]);
            for (int j = 0; j < 3; j++) {
                float avg = sum_tau[leg * 3 + j] / rec_cnt;
                printf("%s %+6.1f Nm  ", jname[j], avg);
            }
            printf("\n");
        }
        // 四腿平均建议
        printf("\n  → 建议 JOINT_IMPEDANCE.tau_ff（四腿平均，上层坐标系）:\n");
        for (int j = 0; j < 3; j++) {
            float a = 0;
            for (int leg = 0; leg < 4; leg++) a += sum_tau[leg * 3 + j] / rec_cnt;
            a /= 4.0f;
            printf("    %-6s tau_ff %+.1f Nm\n", jname[j], a);
        }
        printf("    （狗站得越稳，读数越接近纯重力；若塌/抖需先稳住再采）\n");
    } else {
        printf("[WARN] 未采到记录步（提前中断）\n");
    }

    // ---- 测量结束，趴下（12s 缓降到 LIE_DOWN，轮 0 速弱增益保持）----
    float lie_q[12];
    for (int leg = 0; leg < 4; leg++) {
        lie_q[leg * 3 + 0] = deg2rad(LIE_DOWN_HIP_DEG);
        lie_q[leg * 3 + 1] = deg2rad(LIE_DOWN_THIGH_DEG);
        lie_q[leg * 3 + 2] = deg2rad(LIE_DOWN_CALF_DEG);
    }
    float cur_pos[12];
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            cur_pos[cp * 3 + mi - 1] = motor_mgr.GetStatus(cp, mi).position;
    printf("\n[INFO] 测量结束，趴下中（12s 缓降到 LIE_DOWN {%.0f,%.0f,%.0f}°）...\n",
           LIE_DOWN_HIP_DEG, LIE_DOWN_THIGH_DEG, LIE_DOWN_CALF_DEG);
    for (int f = 0; f <= 600; f++) {
        if (g_rl_stop) break;
        float t = (float)f / 600;
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 3; mi++) {
                float pos = cur_pos[cp * 3 + mi - 1]
                          + (lie_q[cp * 3 + mi - 1] - cur_pos[cp * 3 + mi - 1]) * t;
                motor_mgr.SendImpedance(cp, mi, pos, 0.0f, 200.0f, 20.0f, 0.0f);
            }
        if (f % 100 == 0) printf("  趴下 %4.0f%%\n", t * 100.0f);
        usleep(1000000 / HZ);
    }
    printf("[INFO] 已趴下，保持 2s 稳定...\n");
    sleep(2);

    // ---- 失能 ----
    printf("\n[INFO] 失能...\n");
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.DisableMotor(cp, mi);
    imu.Shutdown();
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    printf("[INFO] 示例53 完成\n");
}
