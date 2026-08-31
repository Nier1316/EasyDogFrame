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
