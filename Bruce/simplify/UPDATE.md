# 更新记录

---

## 2026-06-07 | 电机标定系统

### 新增

- `include/motor_calibration.h`
  - 定义 `MotorCalibrationParam` 结构体：包含 `pos_scale`、`vel_scale`、`pos_offset`
  - 定义 `MOTOR_CALIBRATION[4][3]` 静态矩阵：4个CAN端口 × 3个电机的标定参数表
  - 实现 `ApplyMotorCalibration()` 函数：自动应用标定转换

### 修改

- `src/motor_drive/ele_motor.cpp`
  - 添加 `#include "motor_calibration.h"`
  - 修改 `unpack_frame()` 函数：在解包完成后调用 `ApplyMotorCalibration()` 应用标定参数

### 功能说明

**问题背景：** 12个电机由于厂家原因，位置和速度方向不一致

**解决方案：** 矩阵式标定系统

- **矩阵布局**：`MOTOR_CALIBRATION[can_port][motor_id - 1]`
  - 4个CAN端口 (CAN0~3) × 3个电机 (ID 1~3) = 12个电机

- **标定参数**：
  - `pos_scale`: 位置缩放系数 (1.0f 正常，-1.0f 反向)
  - `vel_scale`: 速度缩放系数 (1.0f 正常，-1.0f 反向)
  - `pos_offset`: 位置偏移值 (默认 0.0f)

- **工作流程**：
  ```
  原始CAN数据 → unpack_frame() → 电机状态更新 
  → ApplyMotorCalibration() → 标定后的数据
  ```

### 使用方法

1. **编辑标定矩阵** — 修改 `include/motor_calibration.h` 中的 `MOTOR_CALIBRATION` 值

   示例：
   ```cpp
   // CAN0 电机2：位置反向，速度正常
   {-1.0f, 1.0f, 0.0f},
   
   // CAN1 电机1：位置和速度都反向，有0.5的位置偏移
   {-1.0f, -1.0f, 0.5f},
   ```

2. **重新编译** — `cd build && cmake .. && make`

3. **自动应用** — 所有12个电机的位置和速度反馈自动应用标定参数

### 技术特点

✅ 编译时内联，零运行时开销

✅ 矩阵布局清晰，对应每个电机一目了然

✅ 自动应用，无需修改控制代码

✅ 支持位置反向、速度反向、位置偏移

---

## 2026-06-04 | Threadmanagement

### 修改

- `include/motor_manager.h`
  - 补充缺失的 `m_can_devices` 成员声明（`std::vector<std::unique_ptr<CanDevice>>`）
  - 补充 `#include "can_device.h"`

- `src/motor_manager.cpp`
  - 修正 `SendThreadFunc()` 中 SPEED 模式参数错误：`motor.kp` → `motor.kvp`（速度环 Kp 字段）

---

## 2026-08-17 | 冗余代码清理

### 删除

- `src/base/` 整目录：`common`、`crc16`、`log`、`md5`、`jsoncpp`、`network`、`serial`、`BleConfigLib`、`DTUCloudConfigLib`（均为死代码，应用层从未引用）。
- `data_types.h`：`MotorCommand`、`MotorConfig`、`ErrorCode` 三个死类型；`MotorStatus` 的 `ack`/`fault` 字段（从不赋值，故障改由 `error_code` 表示）。
- `ele_motor`：`unpack_cmd()`、`has_error()`、`clear_error()`（死函数）。
- `BspCan`/`CanDevice`：`CanDeviceWrapper` 中间层、`IsDeviceRunning()`、`GetReceivedFrameCount()`、`GetSentFrameCount()`、`InitAllDevices()`、`StartAllDevices()`、`StopAllDevices()`、`m_is_opened`、`m_received_count`。
- `ThreadState::ERROR` 枚举值。

### 修改

- `robot_calibration.h`：零位偏移改为直接引用 `MOTOR_CALIBRATION[0][*].pos_offset`，`MOTOR_CALIBRATION` 改为 `constexpr`，消除双份常量。
- `unpack_frame()`：复用 `unionFloat`，不再手写匿名 union。
- `CanDevice::Stop()`：改为返回 `VCI_ResetCAN` 的实际结果。
- `MotorManager`：移除残留的 `m_can_devices` 成员，设备初始化仅走 `BspCan` 单例。


---

## 2026-08-30 | 分层重组 + USB2CAN + traj_v28 + 轮控 SPEED 迁移

### 2026-08-26 框架分层重组
- `src/` 拆为 `app|runtime|strategy|motion|motor|transport` 分层；`include/` 同步分层（`bsp/`、`thread/`、`RoboTasks/`、`motor_drive/` 目录移除）。

### 2026-08-27 达妙 USB2CAN + 策略迁移
- 接入达妙 USB2CAN（`lib/damiao_sdk`，默认后端；需新版 libstdc++ + libusb，CMake 自动探测 `CONDA_LIB_DIR`）。
- 策略迁移到 traj_v28/iteration_3000（`RL_Train/code`，v28 默认 PD250/damping4）。

### 2026-08-28 参数辨识 + 站立稳定
- Example47 整狗 chirp 参数辨识；motor_send 1ms→2ms（500Hz）；USB2CAN recv 取空队列。
- 站立稳定：JOINT_IMPEDANCE（hip 300/10/tau_ff-10 等）、CMD_BIAS_VX=-0.05、WHEEL_KD 2→1。

### 2026-08-29 RL 轮控 SPEED 迁移
- 轮子从「阻抗前馈扭矩」迁移到「固件 SPEED 速度环」（`SendSpeed(vel, kvp, ki)`，1kHz 闭环）。
- 轮控安全层：WHEEL_SOFT_KVP（软启动）、WHEEL_CMD_ALPHA（轮速低通）、WHEEL_CMD_MOVE_THR（移动门控锁轮）。

### 2026-08-30
- sim2real 双开对比工具（`run_dual_compare.sh` + `tool/compare_sim2real.py`）、趴下姿态（Example51）、重力前馈测量（Example53）。

---

## 2026-09-29 | 摩擦前馈 + 权重迭代 + 全面审查修复

### 2026-09-01 摩擦前馈 + 扭矩命令限幅
- `463d230`：新增腿关节摩擦前馈 `τ_ff = fc·tanh(τ_pd/2) + fv·dq`（`include/strategy/rl_controller.h` + `src/strategy/rl_controller.cpp`：`LEG_FF_ENABLE`/`LEG_FF_TANH_K`/`LEG_FF_FC`/`LEG_FF_FV`），方向用 PD 扭矩 `tanh` 决定，避免静止时符号抖动；`include/motor/ele_motor_def.h` 新增 `TORQUE_CMD_LIMIT` 命令扭矩 clamp（当时 Hip/Thigh 110、Calf 200、Wheel 52），`MOTOR_LIMITS` 协议量程同步。
- `72200b0`：新增 Example54 吊装摩擦辨识（重力标定 + 前馈恒速扫掠 + 离线回归 b/fc±σ），配套 `tool/friction_id_offline.py`、`tool/verify_friction_ff.py`。
- `a723bb3`：注释与文档整理，对齐代码现状（数值 / 路径 / 示例号 / 量程）。

### 2026-09-03 权重同步 iteration_2100
- `ada57ff`：`weights/iteration_2100.pkl` 入库，`tool/export_policy.py` 导出参数化（`--ckpt`）并默认指向该权重，`include/strategy/policy_weights.h` + `policy_test_ref.h` 重新导出。

### 2026-09-05 稳定版存档 + 摩擦前馈高频化
- `f10a2f5`：稳定版存档 —— 摩擦辨识完成、S2R 遥测记录（`s2r_recorder`）、标定与扭矩修正。
- `5c4b30a`：摩擦前馈高频化到 500Hz（`MotorManager::SetLegTauFFOverride` + `examples_common` 的 `EnableRlFrictionFF`/`DisableRlFrictionFF`），Ex56 A/B 腿跟踪误差 −46%。

### 2026-09-07 Ex37 激活 + gait_phase 对齐训练
- `0544f6c`：RL 遥操作行走稳定，`src/app/main.cpp` 激活 `Example37_RLTeleopControl`；`tool/plot_rlrun.py` 支持 sim 记录。
- `0ee431f`：gait_phase 尾 8 通道对齐训练 —— 由交错 `[sinFL,cosFL,…]` 改为**分组** `[sin×4, cos×4]`（`src/strategy/rl_controller.cpp`，`obs[56..59]` / `obs[60..63]`）。

### 2026-09-11 ~ 09-22 权重迭代链
- `3039927`（09-11）iteration_3500；`33b672e`（09-12）iteration_4350（med_posefree_s4_v3.3）；`b61216e`（09-15）回退到 stable iteration_2100（v28_v3_resume）；`0d0257c`（09-17）iteration_5350（宽摩擦 DR，地形适应）。
- `98ef8ea`（09-22，**当前 HEAD**）：权重切换 **iteration_9754**（降低抬腿幅度）—— `weights/iteration_9754.pkl` 入库 + `tool/export_policy.py` 默认指向 + `policy_weights.h`/`policy_test_ref.h` 重新导出。
- `weights/` 最终只保留 `iteration_9754.pkl`（当前）与 `iteration_2100.pkl`（历史）；3500/4350/5350 于本次审查清理。

### 2026-09-13 Ex57 + sim2sim wheel_gate + 真机参数
- `bfadc76`：新增 Example57 单腿零位对照（验证 CONV_A/B 与物理零位）。
- `88a64e1`：sim2sim 加 `--wheel_gate`，复现真机站立锁轮门控（阈值同 `rl::WHEEL_CMD_MOVE_THR=0.1`）。
- `6546688`：真机参数调整 —— 腿重力前馈清零、轮软限位 30Nm、Ex37 量程 0.7。（⚠️ 该提交信息与 `JOINT_IMPEDANCE` 的 thigh `tau_ff` 符号不一致，见 `memory/FACT.md` 的「待现场确认」项。）

### 2026-09-29 全面审查修复（本次）
**代码（死代码清理）**
- `include/strategy/rl_controller.h` + `src/strategy/rl_controller.cpp`：删除无调用者死代码簇 `wheel_torque()`、`WHEEL_FF[4][2]`、`WHEEL_FF_ENABLE`、`WHEEL_SOFT_LIMIT_ENABLE`、`WHEEL_VEL_SOFT_LIMIT`、`WHEEL_SOFT_LIMIT_TORQUE`、`WHEEL_TORQUE_LIMIT`、`LEG_TORQUE_LIMIT`（历史数值留档于 `memory/FACT.md`）。
- `include/motor/ele_motor.h`：删除死成员 `state_mutex`（实际加锁为 `MotorManager::m_motor_mutex`）。
- `include/app/examples/ex_diag.h` + `src/app/main.cpp`：删除 `Example55_SingleLegLimitMeasure` 的声明与被注释调用（该示例从未实现，取消注释即链接失败）。
- `src/app/main.cpp`：删除未使用的全局 `static RobotApp g_app;`（其析构会访问已销毁的 `MotorManager` 单例，属静态析构顺序 UB）及随之无用的 include。

**代码（bug 修复）**
- `src/motor/ele_motor.cpp` + `include/motor/motor_calibration.h`：修复参数回帧**二次标定** —— 单寄存器回帧只标定被更新的字段，补上原先完全没做的 `MOTOR_OR_torque` 标定，日志 raw 值改为标定前捕获；新增按字段 helper `ApplyMotorCalibrationPos/Vel/Torque`。
- `src/transport/can_device.cpp`：`DWORD` 用 `%d` 打印改 `%lu`；`include/common/log_control.h`：注释里的 `/*` 触发 `-Wcomment`；`types.h`：`#endif` 注释与宏名不符。
- `src/transport/usb2can_transport.cpp`：`recv` 用 `operator[]` 会给未打开通道凭空建条目 → 改 `find`。
- `src/app/examples/examples_common.cpp`：`poll_key` 改为带缓冲的状态解析（修读方向键时字节分次到达丢键）。
- `src/motion/motion_controller.cpp`：三个插值函数在 `mm_==nullptr` 或 `total<=0` 时会崩 / 产生 NaN（加保护）。
- 未使用符号：`ex_rl.cpp` typedef、`ex_diag.cpp` `POS_LIMIT`、`ex_basic.cpp` `dt`。

**代码（注释修正，值不变）**
- `include/motion/robot_calibration.h`：θ1 上限 15°、删除"仍压在 0 边界"等与代码不符的注释、修正引用已删除 Example55 的措辞。
- ⚠️ 仍**保持原值不变、仅标注**的两处安全冲突（待现场确认）：`JOINT_IMPEDANCE[..][THIGH].tau_ff` 代码 `+5.0f`（注释 / `6546688` 写 −5）；`UPPER_LIMIT_THETA1_DEG` 代码 `15.0f`（旧注释写 +30°）。

**sim2sim 同步**
- `dogurdf_sim2sim_deploy/run_sim2sim.sh`：默认 checkpoint 由 `checkpoints/dogurdf_velocity/iteration_3000.pkl` 改为 `../weights/iteration_9754.pkl`（**与真机同一份**），支持 `SIM2SIM_CKPT=<path>` 覆盖；`checkpoints/` 内 450/3000 标注为历史存档，不再与真机同步。

**文档**
- `memory/FACT.md`、`UPDATE.md`、`memory/JOURNAL.jsonl` 与 `docs/*`、根目录指南全面同步到当前代码：权重 9754、示例 17~57 共 40 个（55 从未实现）/ 激活 Ex37、量程 120/120/200/52、轮控保护 `WHEEL_ESTOP_*`、`SIM_DT=0.005 / DECIMATION=4`、gait_phase 分组布局等。
