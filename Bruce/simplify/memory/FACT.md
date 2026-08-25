# 项目事实

## 工程位置与构建
- 主工程：`/home/bruce/Desktop/EasyDogFrame/Bruce/simplify`（四足机器狗 CAN 电机控制框架，CMake + C++17）。
- 构建：`cmake -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j$(nproc)`，产物 `bin/can_motor_app`。
- CANET SDK 在 `lib/` 下：`lib/CANET.h`、`lib/ControlCAN.h`、`lib/linux_x64/{Debug,Release}/libCANET_TCP.{a,so}`。工程可完整编译（SDL2 用于手柄示例）。

## 硬件拓扑
- 4 路 CANET TCP（CAN0~3，IP 192.168.0.178，端口 4001~4004）。
- 每路 4 个电机 = 共 16 电机：motor_id 1=髋、2=大腿、3=小腿、4=轮；tx_id=motor_id，rx_id=50+motor_id。
- 标定矩阵 `MOTOR_CALIBRATION[4][4]` 位于 `include/motor_calibration.h`。

## RL 部署（dogurdf 轮足策略，进行中）
- 策略链路：`include/rl/{policy_weights.h,mlp.h,rl_controller.h}` + `src/rl/rl_controller.cpp`，与 `dogurdf_sim2sim_deploy/src/sim2sim.py` 逐参数/逐布局一致（64 维观测、ACTION_SCALE=0.25、LEG_KP/KD=250/4、WHEEL_KD=2、GAIT_CYCLE=0.6、GAIT_OFFSET={0,0.5,0.5,0}）。
- ✅ 权重已更新（2026-08-22）：从远程拉取 **iteration_450** 新策略（替代 iteration_1000）。`policy_weights.h`/`policy_test_ref.h` 已重新导出，Example30 离线回归通过（max err 4e-7）。对应模型 `dogurdf.xml` 质量大幅调整（torso 1.97→10.71、thigh 1.707→6.302、总重 ~14→~57kg，摩擦 0.8→1.36），几何不变，不影响 C++ 纯 PD 控制。LEG_KP=250 与训练 stiffness=250 一致。
- ✅ 真机状态（2026-08-22 收尾）：RL 站立稳定 + 手柄遥操作可用（Example37）。**iteration_450 新权重**已部署（Example30 离线回归通过）。
- FR 腿扭矩小问题：软件排查定位为 FR hip **下发位置命令偏**（RL +0.18 / E21 +0.108 rad），`MOTOR_CALIBRATION[1][0].pos_offset` 0.611→**0.78** 后 FR hip 位置恢复对称（−0.152→−0.054）；但扭矩仍偏小→确认为电机/承重层（已解决）。`pos_offset=0.78` 保留。
- 🔴→✅ 轮子发散排查（2026-08-21 修正结论）：实测确定四路轮子 pos_scale 正确配置为 **CAN0/2=-1, CAN1/3=+1**（CAN0/2 固件负扭矩=前滚、CAN1/3 固件正扭矩=前滚，接线方向不同）。曾误改 CAN0/2=+1 导致左侧反，已恢复。RL 轮子方向问题与发散根因仍待 Example25 重测确认。
- 轮子摩擦前馈（Example35 实测 2026-08-21）：`rl::WHEEL_FF[4][2]` = FL{+0.6,-0.6} FR{+0.5,-0.4} RL{+0.8,-0.8} RR{+0.5,-0.4}，CAN2 阻力最大。已集成进 `wheel_torque(action, vel, wheel_idx)`。⚠ `WHEEL_FF_ENABLE=false`（先关）。
- 轮子速度软限位（安全兜底，rl_controller.h）：`WHEEL_SOFT_LIMIT_ENABLE=true`，|轮速|>15 rad/s 强制制动 30 Nm，防冲到 48 饱和。正常移动轮速<9 rad/s 不干扰。
- 🔴 轮子乱转根因（2026-08-21 Example36 诊断锁定）：起立用 STAND_*(thigh-60°真机)，RL 循环目标用 DEFAULT_POSE(转真机 thigh≈-71.8°)，**进入 RL 时 12° 目标跳变 → 腿猛动带轮子 → 轮速冲到 37~44 rad/s → wheel_torque 制动饱和(±53)压不住 → 轮速污染策略观测 → 发散**。修复：Example36 起立目标改为 `urdf_to_status(DEFAULT_POSE)`（与 RL 一致，消除跳变）。若 qrel 仍偏大→CONV_B 需重测（Example31）。
- 日志分类开关（控制台）：`include/log_control.h` 的 `logctl::LOG_SWITCH[]`（SYSTEM/MOTOR/RL/IMU/WHEEL/CAN/DIAG），改 true/false 重编译即开关某类诊断日志。Example36 诊断已迁移（RL=action/扭矩、WHEEL=qrel+轮速、IMU=姿态）。
- log/ CSV 文件开关：`include/motor_logger.h` 的 `LogFileSwitch`（SEND/RECV/SENDCAN/XBOX/KEY），改 true/false 重编译。关闭的分类不创建文件、不写入。默认 sendcan 关（体积最大，帧级诊断才开）。
- Example25 手柄命令（对齐 sim2sim.py GamepadReader，2026-08-20 接入）：左摇杆上推=+vx 前进、右摇杆左推=+wz 左转(CCW)，vy 恒 0，量程 vx±1.0 m/s、wz±1.0 rad/s；**B 键急停**（失能）；Ctrl+C 急停。手柄在程序启动时初始化，未插则保持原地站立。
- 关节角约定：策略工作在 **URDF 约定**（默认姿态 hip=0,thigh=0.20,calf=-0.35 与 dogurdf.py NOMINAL_* 一致）；真机 GetStatus 指令角与之相差每关节符号/偏移，由 `CONV_A/CONV_B` 转换吸收（大腿符号相反；hip/thigh/calf 偏移 ≈ +1.7°/-60.3°/-79.3°）。
- ⚠ CONV_B 基于 2026-08-20 一次 L 形目测，转换后默认姿态对应真机指令角 thigh≈-71.8° 略超真机限位[-70,90]，**待 Example32 低增益真机验证后微调**。
- 连杆/机身参数已按 URDF 更新：`LEG_L1=0.1308`、`LEG_L2=0.34`、`LEG_L3=0.343`、`BODY_LENGTH=0.653`、`BODY_WIDTH=0.16`（见 robot_calibration.h §2/§3）。
- 示例分工：`Example30` 离线链路回归；`Example31` 零位对齐/关节范围扫描（不使能电机）；`Example32` 默认姿态验证（低增益慢插值）；`Example33` IMU 链路验证（只读）；`Example25` 完整 RL 循环（50Hz、Ctrl+C 急停、跌倒检测）。
- IMU（维特 HWT606，`/dev/ttyUSB0`，安装 `Z_DOWN_X`）：`base_ang_vel`=gyro（机体系 rad/s）；`projected_gravity`=world2self(quat,[0,0,-1])，已离线验证与仿真 `rotate(v,quat_inv)` 等价。`Example33` 放平判读：pgr≈(0,0,-1)、欧拉角≈0；右倾→pgr.y 变负（MuJoCo 约定）。开机需机身水平（IMU 水平校准基准）。2026-08-20 真机验证通过：水平放平 pgr=(-0.045,0.001,-0.999)。
- ⚠ IMU 串口环境：`/dev/ttyUSB0` 曾被 Ubuntu **brltty 盲文服务抢占**（85-brltty.rules 的 `PRODUCT==1a86/7523`），表现为 ttyUSB0 刚创建即被断开。已 `systemctl stop/disable/mask brltty` 与 `brltty-udev` 修复，勿再启用。另加 udev 规则 `/etc/udev/rules.d/99-ch340-serial.rules`（CH340 → MODE=0666），因 VS Code/gdb 调试进程继承旧会话组、即使加入 `dialout` 也无法获得权限；0666 使任何进程可打开串口，重启持久。
- ⚠ 代码教训：`rl::world2self`/`build_observation` 要求 quat 是**连续 float[4]**。Example33 曾误用 `&w`（独立局部变量地址）当数组传入 → pgr 错乱（(0,0,-0.669)），已改 `float quat[4]` 修复。写 IMU 读取时统一用数组 + `GetQuat(quat[0],...)`。

## 注意
- `PLAN.md`（12 电机）与 `TODO.md` 滞后于代码（实际 16 电机）；`TODO.md` 列的 Bug 1~5 多已在代码中修复。
- 零位偏移唯一真值来源是 `MOTOR_CALIBRATION[].pos_offset`；`robot_calibration.h` 直接引用它，勿再手抄字面量。
