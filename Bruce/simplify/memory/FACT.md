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
- 策略链路：`include/rl/{policy_weights.h,mlp.h,rl_controller.h}` + `src/rl/rl_controller.cpp`，与 `dogurdf_sim2sim_deploy/src/sim2sim.py` 逐参数/逐布局一致（64 维观测、ACTION_SCALE=0.25、LEG_KP/KD=150/4、WHEEL_KD=2、GAIT_CYCLE=0.6、GAIT_OFFSET={0,0.5,0.5,0}）。
- 关节角约定：策略工作在 **URDF 约定**（默认姿态 hip=0,thigh=0.20,calf=-0.35 与 dogurdf.py NOMINAL_* 一致）；真机 GetStatus 指令角与之相差每关节符号/偏移，由 `CONV_A/CONV_B` 转换吸收（大腿符号相反；hip/thigh/calf 偏移 ≈ +1.7°/-60.3°/-79.3°）。
- ⚠ CONV_B 基于 2026-08-20 一次 L 形目测，转换后默认姿态对应真机指令角 thigh≈-71.8° 略超真机限位[-70,90]，**待 Example32 低增益真机验证后微调**。
- 连杆/机身参数已按 URDF 更新：`LEG_L1=0.1308`、`LEG_L2=0.34`、`LEG_L3=0.343`、`BODY_LENGTH=0.653`、`BODY_WIDTH=0.16`（见 robot_calibration.h §2/§3）。
- 示例分工：`Example30` 离线链路回归；`Example31` 零位对齐/关节范围扫描（不使能电机）；`Example32` 默认姿态验证（低增益慢插值）；`Example25` 完整 RL 循环（50Hz、Ctrl+C 急停、跌倒检测）。当前 main 跑 Example32。

## 注意
- `PLAN.md`（12 电机）与 `TODO.md` 滞后于代码（实际 16 电机）；`TODO.md` 列的 Bug 1~5 多已在代码中修复。
- 零位偏移唯一真值来源是 `MOTOR_CALIBRATION[].pos_offset`；`robot_calibration.h` 直接引用它，勿再手抄字面量。
