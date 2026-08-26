# 项目事实

## 工程位置与构建
- 主工程：`/home/sysu/Desktop/Project/Bruce/EasyDogFrame/Bruce/simplify`（四足机器狗电机控制框架，CMake + C++17，分层重组后）。
- 构建：`cmake -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j$(nproc)`，产物 `bin/can_motor_app`。
- 分层结构（2026-08-26 重构）：`src/app/`（main + examples）、`src/runtime/`（线程/robot_app/motor_io）、`src/strategy/`（rl_controller/sim2real_conv/imu_device）、`src/motion/`（motion_controller）、`src/motor/`（motor_manager/ele_motor）、`src/transport/`（canet_transport/can_device/usb2can_transport）。
- 传输层：CANET TCP（`lib/CANET.h`、`lib/linux_x64/{Debug,Release}/libCANET_TCP.{a,so}`）+ **达妙 USB2CAN**（`lib/damiao_sdk/linux/x86_64/libdm_device.so`）。达妙链接需新版 libstdc++(GLIBCXX_3.4.32) + libusb(≥1.0.26)：`CONDA_LIB_DIR` 指向本机 conda base `/home/sysu/miniconda3/lib`（libusb 已 conda 装），libusb 用绝对路径避免解析到系统老版本。

## 硬件拓扑
- 4 路总线，每路 4 个电机 = 共 16 电机：motor_id 1=髋、2=大腿、3=小腿、4=轮；tx_id=motor_id，rx_id=50+motor_id。
- 原为 4 路 CANET TCP（IP 192.168.0.178，端口 4001~4004）；重构后支持达妙 USB2CAN（Example36 全 4 路走 USB2CAN）。
- 标定矩阵 `MOTOR_CALIBRATION[4][4]` 位于 `include/motor/motor_calibration.h`（重构后路径）。⚠ FR hip pos_offset 代码为 0.611，注释与 FACT 旧值 0.78 矛盾，待核实。
- 电机环频率（重构后）：`motor_receive` = 2ms（500Hz）、`motor_send` = 1ms（1000Hz），`robot_calibration.h` `CONTROL_HZ=500`。

## RL 部署（dogurdf 轮足策略，已部署 traj_v26）
- 策略链路（重构后路径）：`include/strategy/{policy_weights.h,mlp.h,rl_controller.h,sim2real_conv.h}` + `src/strategy/{rl_controller.cpp,sim2real_conv.cpp}`，与训练 `RL_Train/code`（权威）逐参数/逐布局一致。
- 关键常量：64 维观测 / 16 维动作、`ACTION_SCALE=0.25`、`LEG_KP=250`、**`LEG_KD=40`（⚠ 训练真值为 4.0，差 10 倍，属现场标定疑点）**、`WHEEL_KD=2`、`WHEEL_VEL_SCALE=12.5`、`LEG_TORQUE_LIMIT=150`（⚠ 训练 sim2sim 用 250）、`GAIT_CYCLE=0.6`、`GAIT_OFFSET={0,0.5,0.5,0}`、`CONTROL_DT=0.02`（RL 50Hz）。
- ✅ 权重已更新（2026-08-26）：部署 **traj_v26 / iteration_1100**（`RL_Train/code/checkpoints/dogurdf_velocity/checkpoints_20260826_182658_traj_v26/iteration_1100.pkl`），`export_policy.py` 已指向它并输出到 `include/strategy/`，Example30 离线回归通过。v26 = stage2 训练，启用了 action_delay（每关节 D~N(1.5,0.5) clip[1,2] 步 = 20-40ms，更抗总线时延）。
- 零位转换 `sim2real_conv.cpp`：`CONV_A` hip/thigh/calf = +1/-1/+1；`CONV_B` = hip+0.0297 / thigh **-0.9624** / calf **-1.2832** / 轮 0（⚠ 代码注释仍写旧值 -1.0524/-1.3832，需同步修正注释）。
- 真机状态（2026-08-22 收尾）：RL 站立稳定 + 手柄遥操作可用（Example37）。**traj_v26 新权重已部署**。
- 轮子摩擦前馈（Example35 实测）：`rl::WHEEL_FF[4][2]` = FL{+0.6,-0.6} FR{+0.5,-0.4} RL{+0.8,-0.8} RR{+0.5,-0.4}，CAN2 阻力最大。⚠ `WHEEL_FF_ENABLE=false`（先关）。
- 轮子速度软限位（安全兜底，rl_controller.h）：`WHEEL_SOFT_LIMIT_ENABLE=true`，|轮速|>**5.0** rad/s 强制制动 **7.0** Nm（旧值 15/30 已改）。
- 🔴 轮子乱转根因（2026-08-21 Example36 诊断锁定）：起立用 STAND_*(thigh-60°真机)，RL 循环目标用 DEFAULT_POSE，进入 RL 时目标跳变 → 腿猛动带轮子 → 轮速冲高 → 制动饱和 → 轮速污染策略观测 → 发散。修复：Example36 起立目标改为 `urdf_to_status(DEFAULT_POSE)`（消除跳变）。
- 日志分类开关（控制台）：`include/common/log_control.h` 的 `logctl::LOG_SWITCH[]`（SYSTEM/MOTOR/RL/IMU/WHEEL/CAN/DIAG）。log/ CSV 开关：`include/motor/motor_logger.h` 的 `LogFileSwitch`。
- Example25 手柄命令：左摇杆上推=+vx 前进、右摇杆左推=+wz 左转(CCW)，vy 恒 0，量程 vx±1.0 m/s、wz±1.0 rad/s；B 键急停；Ctrl+C 急停。
- 关节角约定：策略工作在 **URDF 约定**（默认姿态 hip=0,thigh=0.20,calf=-0.35 与 dogurdf.py NOMINAL_* 一致）；真机 GetStatus 指令角由 `CONV_A/CONV_B` 转换吸收（大腿符号相反）。新 CONV_B 下默认姿态 thigh≈-66.6° 已在限位内。
- 连杆/机身参数已按 URDF 更新：`LEG_L1=0.1308`、`LEG_L2=0.34`、`LEG_L3=0.343`、`BODY_LENGTH=0.653`、`BODY_WIDTH=0.16`（见 include/motion/robot_calibration.h）。
- 示例分工（ex_rl.cpp）：`Example25` 完整 RL 循环（50Hz、手柄、急停）；`Example30` 离线链路回归；`Example31` 零位对齐/关节范围扫描（不使能电机）；`Example32` 默认姿态验证；`Example35` 轮摩擦前馈标定；`Example36` 4 路 USB2CAN RL 站立循环（**当前 main.cpp 激活**）；`Example37` 手柄遥操作（走 MotionController）；`Example38` 动作延迟测量。
- IMU（维特 HWT606，`/dev/ttyUSB0`，安装 `Z_DOWN_X`）：`base_ang_vel`=gyro（机体系 rad/s）；`projected_gravity`=world2self(quat,[0,0,-1])，已离线验证与仿真 `rotate(v,quat_inv)` 等价。放平判读：pgr≈(0,0,-1)；右倾→pgr.y 变负。开机需机身水平（IMU 水平校准基准）。
- ⚠ IMU 串口环境：`/dev/ttyUSB0` 曾被 Ubuntu **brltty 盲文服务抢占**（85-brltty.rules 的 `PRODUCT==1a86/7523`）。已 `systemctl stop/disable/mask brltty` 与 `brltty-udev` 修复，勿再启用。另加 udev 规则（CH340 → MODE=0666）。
- ⚠ 代码教训：`rl::world2self`/`build_observation` 要求 quat 是**连续 float[4]**。写 IMU 读取时统一用数组 + `GetQuat(quat[0],...)`，勿用独立局部变量地址。

## 注意
- `PLAN.md`（12 电机）与 `TODO.md` 滞后于代码（实际 16 电机）；`TODO.md` 列的 Bug 1~5 多已在代码中修复。
- 零位偏移唯一真值来源是 `MOTOR_CALIBRATION[].pos_offset`；`robot_calibration.h` 直接引用它，勿再手抄字面量。
- 训练权威 = `RL_Train/code`（非 `dogurdf_sim2sim_deploy`，后者是历史快照）；sim2sim.py 用 SIM_DT=0.002/DECIMATION=10/MOTOR_DECIMATION=1（500Hz PD 子环）。
