# 项目事实

## 工程位置与构建
- 主工程：`/home/sysu/Desktop/Project/Bruce/EasyDogFrame/Bruce/simplify`（四足机器狗电机控制框架，CMake + C++17，分层重组后）。
- 构建：`cmake -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j$(nproc)`，产物 `bin/can_motor_app`。
- 分层结构（2026-08-26 重构）：`src/app/`（main + examples）、`src/runtime/`（线程/robot_app/motor_io）、`src/strategy/`（rl_controller/sim2real_conv/imu_device）、`src/motion/`（motion_controller）、`src/motor/`（motor_manager/ele_motor）、`src/transport/`（canet_transport/can_device/usb2can_transport）。
- 传输层：CANET TCP（`lib/CANET.h`、`lib/linux_x64/{Debug,Release}/libCANET_TCP.{a,so}`）+ **达妙 USB2CAN**（`lib/damiao_sdk/linux/x86_64/libdm_device.so`）。达妙链接需新版 libstdc++(GLIBCXX_3.4.32) + libusb(≥1.0.26)；CMakeLists **自动探测** conda lib 路径（bruce 机 `/home/bruce/miniforge3/lib`、sysu 机 `/home/sysu/miniconda3/lib`，可用 `-DCONDA_LIB_DIR` 覆盖），libusb 用绝对路径避免解析到系统老版本。

## 硬件拓扑
- 4 路总线，每路 4 个电机 = 共 16 电机：motor_id 1=髋、2=大腿、3=小腿、4=轮；tx_id=motor_id，rx_id=50+motor_id。
- 原为 4 路 CANET TCP（IP 192.168.0.178，端口 4001~4004）；重构后支持达妙 USB2CAN（Example36 全 4 路走 USB2CAN）。
- 标定矩阵 `MOTOR_CALIBRATION[4][4]` 位于 `include/motor/motor_calibration.h`（重构后路径）。⚠ FR hip pos_offset 代码为 0.611，注释与 FACT 旧值 0.78 矛盾，待核实。
- 电机环频率：`motor_receive` = 2ms（500Hz）、`motor_send` = 2ms（500Hz，原 1ms 冗余已改），`robot_calibration.h` `CONTROL_HZ=500`。轮子速度环走**固件 SPEED**（SendSpeed 1kHz 闭环），目标值由 500Hz SendOnce 下发。

## RL 部署（dogurdf 轮足策略，已部署 traj_v28）
- 策略链路（重构后路径）：`include/strategy/{policy_weights.h,mlp.h,rl_controller.h,sim2real_conv.h}` + `src/strategy/{rl_controller.cpp,sim2real_conv.cpp}`，与训练 `RL_Train/code`（权威）逐参数/逐布局一致。
- 关键常量（**对齐 v28 sim2sim.py 默认**）：64 维观测 / 16 维动作、`ACTION_SCALE=0.25`、**`LEG_KP=250`、`LEG_KD=4`、`LEG_TORQUE_LIMIT=250`**（= `RL_Train/code/src/sim2sim.py` 默认，traj_v28 训练 stiffness=250/damping=4，见 commit 07884c7；⚠ 300/10 是 V30 参数勿混淆）、**`WHEEL_KD=1.0`**（⚠ sim2sim 默认 2.0，SPEED 迁移后轮子走固件速度环 kvp/ki，此量仅诊断用）、`WHEEL_VEL_SCALE=12.5`、`WHEEL_TORQUE_LIMIT=53`、`GAIT_CYCLE=0.6`、`GAIT_OFFSET={0,0.5,0.5,0}`、`CONTROL_DT=0.02`（RL 50Hz）。⚠ 历史：曾用 LEG_KP/KD=250/40（真机标定）、LEG_TORQUE_LIMIT=150，后统一套 v28 sim2sim 默认。
- 站立稳定（2026-08-28）：`JOINT_IMPEDANCE` = hip kp300/kd10/**tau_ff=-10**、thigh kp250/kd10/**tau_ff=-5**、calf kp250/kd10/**tau_ff=+12（前腿）/ +20（后腿）**；`CMD_BIAS_VX=-0.05`（Example36 + MotionController `cmd_bias_vx`）抵消策略 wheel action 正向偏置（整体前冲）。⚠ cmd_bias_vx 绝对值必须 < 训练 turn_lin_threshold(0.1)，否则原地转向退出纯 yaw gate 变扭腿。
- 🔴 **轮子控制（2026-08-29 SPEED 迁移）**：轮子不再走阻抗前馈扭矩，改走**固件 SPEED 速度环**（`SendSpeed(vel, kvp, ki)`，固件内部 1kHz 闭环）。常量：`WHEEL_KVP=3.0`（比例）、`WHEEL_KVI=0.05`（积分，历史 0.3 在 RL 上积分过强）、`WHEEL_SOFT_KVP=0.1`（起立/回位 0 速弱增益软启动）、`WHEEL_CMD_ALPHA=0.2`（轮速目标低通 @50Hz）、`WHEEL_CMD_MOVE_THR=0.1`（移动/静止门控，站立锁轮，防策略后轮微调 action 溜车）、`WHEEL_CMD_DEADZONE=0.5`（已弃用，门控取代）。
- ✅ 权重已更新（2026-08-27）：部署 **traj_v28 / iteration_3000**（`RL_Train/code/checkpoints/dogurdf_velocity/checkpoints_20260827_044715_traj_v28/iteration_3000.pkl`），`export_policy.py` 已指向它并输出到 `include/strategy/`，Example30 离线回归通过（C++ MLP err 2.4e-06）。v28 训练 stiffness=250/damping=4（与 v26 相同；⚠ 300/10 是 V30 参数，非 v28）。
- 零位转换 `sim2real_conv.cpp`：`CONV_A` hip/thigh/calf = +1/-1/+1；`CONV_B` = hip+0.0297 / thigh **-0.9624** / calf **-1.2832** / 轮 0（代码注释已同步为新值）。
- 真机状态：RL 站立稳定 + 手柄遥操作可用（Example37）。**traj_v28 新权重已部署（含 sim2sim 默认 PD 参数）**。
- 轮子摩擦前馈（Example35 实测）：`rl::WHEEL_FF[4][2]` = FL{+0.6,-0.6} FR{+0.5,-0.4} RL{+0.8,-0.8} RR{+0.5,-0.4}，CAN2 阻力最大。⚠ `WHEEL_FF_ENABLE=false`（先关）。
- 轮子速度软限位（安全兜底，rl_controller.h）：`WHEEL_SOFT_LIMIT_ENABLE=true`，|轮速|>**5.0** rad/s 时扭矩限幅到 ±**10.0** Nm（**限幅式**，不是硬制动；⚠ 代码阈值 5.0/限幅 10.0，与 a0327c8 提交信息写的「10/7」不符，以代码为准）。
- 🔴 轮子乱转根因（2026-08-21 Example36 诊断锁定）：起立用 STAND_*(thigh-60°真机)，RL 循环目标用 DEFAULT_POSE，进入 RL 时目标跳变 → 腿猛动带轮子 → 轮速冲高 → 制动饱和 → 轮速污染策略观测 → 发散。修复：Example36 起立目标改为 `urdf_to_status(DEFAULT_POSE)`（消除跳变）。
- 日志分类开关（控制台）：`include/common/log_control.h` 的 `logctl::LOG_SWITCH[]`（SYSTEM/MOTOR/RL/IMU/WHEEL/CAN/DIAG）。log/ CSV 开关：`include/motor/motor_logger.h` 的 `LogFileSwitch`。
- Example25 手柄命令：左摇杆上推=+vx 前进、右摇杆左推=+wz 左转(CCW)，vy 恒 0，量程 vx±1.0 m/s、wz±1.0 rad/s；B 键急停；Ctrl+C 急停。
- 关节角约定：策略工作在 **URDF 约定**（默认姿态 hip=0,thigh=0.20,calf=-0.35 与 dogurdf.py NOMINAL_* 一致）；真机 GetStatus 指令角由 `CONV_A/CONV_B` 转换吸收（大腿符号相反）。新 CONV_B 下默认姿态 thigh≈-66.6° 已在限位内。
- 连杆/机身参数已按 URDF 更新：`LEG_L1=0.1308`、`LEG_L2=0.34`、`LEG_L3=0.343`、`BODY_LENGTH=0.653`、`BODY_WIDTH=0.16`（见 include/motion/robot_calibration.h）。
- 示例分工（ex_rl.cpp + ex_diag.cpp，示例到 53）：`Example25` 完整 RL 循环（50Hz、手柄、急停）；`Example30` 离线链路回归；`Example31` 零位对齐/关节范围扫描（不使能电机）；`Example32` 默认姿态验证；`Example35` 轮摩擦前馈标定；`Example36` 4 路 USB2CAN RL 站立循环；`Example37` 手柄遥操作（走 MotionController）；`Example38` 动作延迟测量；`Example44` USB2CAN Xbox 手柄控制（**当前 main.cpp 激活**）；`Example47` 整狗站立 chirp 参数辨识（sysid_all.csv）；`Example48` 轮子扭矩方向验证；`Example51` 站立后趴下；`Example52` 固定 yaw 指令；`Example53` 站立下重力前馈测量。
- IMU（维特 HWT606，`/dev/ttyUSB0`，安装 `Z_DOWN_X`）：`base_ang_vel`=gyro（机体系 rad/s）；`projected_gravity`=world2self(quat,[0,0,-1])，已离线验证与仿真 `rotate(v,quat_inv)` 等价。放平判读：pgr≈(0,0,-1)；右倾→pgr.y 变负。开机需机身水平（IMU 水平校准基准）。
- ⚠ IMU 串口环境：`/dev/ttyUSB0` 曾被 Ubuntu **brltty 盲文服务抢占**（85-brltty.rules 的 `PRODUCT==1a86/7523`）。已 `systemctl stop/disable/mask brltty` 与 `brltty-udev` 修复，勿再启用。另加 udev 规则（CH340 → MODE=0666）。
- ⚠ 代码教训：`rl::world2self`/`build_observation` 要求 quat 是**连续 float[4]**。写 IMU 读取时统一用数组 + `GetQuat(quat[0],...)`，勿用独立局部变量地址。

## 注意
- `PLAN.md`（12 电机）与 `TODO.md` 滞后于代码（实际 16 电机）；`TODO.md` 列的 Bug 1~5 多已在代码中修复。
- 零位偏移唯一真值来源是 `MOTOR_CALIBRATION[].pos_offset`；`robot_calibration.h` 直接引用它，勿再手抄字面量。
- 训练权威 = `RL_Train/code`（非 `dogurdf_sim2sim_deploy`，后者是历史快照）；sim2sim.py 用 SIM_DT=0.002/DECIMATION=10/MOTOR_DECIMATION=1（500Hz PD 子环）。
- 最新提交（2026-08-30）：a0327c8 参数辨识（Example47 整狗 chirp）+ 轮子 500Hz 闭环 + motor_send 1ms→2ms + USB2CAN recv 取空队列；6e9877b 站立稳定（JOINT_IMPEDANCE、CMD_BIAS_VX、WHEEL_KD 2→1、Example48）；8da704a 迁移 traj_v28 + 对齐 v28 PD；8b114bc **RL 轮控 SPEED 迁移 + 轮控安全层 + sim2real 对比工具（run_dual_compare.sh）+ 趴下姿态**；f56b588 tau_ff 存档 + sim2sim Ctrl+C 修复；cae611c 解决 RMA 层缺失。新增 `docs/RL_TRAINING_REFERENCE.md`（真机辨识参数/控制架构/KP/KD/tau_ff 建议）。
