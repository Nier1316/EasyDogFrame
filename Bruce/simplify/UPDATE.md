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

---

## 2026-10-06 | 标定稳定分支 + 节拍修正 + 5350 演示策略

### 新建 `stable` 分支
- 从 `develop` 拉出 **`stable`**，作为**对外演示 / 现场稳定运行**的基线；日常实验仍在 `develop`。
- 根目录新增 **`STABLE.md`**：记录当前稳定策略 = **h52_s45_v4 / iteration_5350**、为什么选它、以及本分支包含的稳定化改动。

### Ex37 切换到 iteration_5350
- 恢复被删权重：`weights/iteration_5350.pkl` 于 2026-09-29 仓库清理时删除，
  由 git 历史 `0d0257c` 恢复，`git hash-object` 校验与原 blob `d19b3d60…` **逐字节一致**。
- 用 `tool/export_policy.py --ckpt weights/iteration_5350.pkl --out-weights include/strategy/policy_weights_5350.h --out-ref include/strategy/policy_test_ref_5350.h` 导出（形状 64-512-256-128-16，与另两套一致）。
- 新增第 3 个运行时变体 `rl::POLICY_H52S45_V4_5350 = 2`；`POLICY_VARIANT` 默认改为 **2**；
  **Example37 改为强制 5350**；Example30 扩展为**同时校验三套权重**（9754/10000/5350 全通过）。

### 控制节拍修正（500 Hz 达标）
- `src/runtime/thread_manager.cpp`：LOOP 线程由「`func(); sleep_for(interval-elapsed)`」
  改为**绝对 deadline 网格**（`next += period; sleep_until(next)`）。
  旧实现每轮多睡 ~50µs（Linux 默认 timer_slack），2ms 周期下累积成 ~486 Hz（−2.8%）；
  修正后实测 **500.000 Hz**。影响 `motor_receive` / `motor_send`（全示例生效）。
- `include/app/examples_common.h`：新增 `LoopPacer`（同款绝对节拍），
  **Example37 / Example61** 策略环改用它 → 实测 **50.000 Hz**（原 20.57 ms/步 ≈ 48.6 Hz）。
- 真机验证：`dataset_20261004_155443.csv` 实测行率 **499.92 Hz**（修复前 485.95 Hz），
  策略间隔中位 **20.002 ms**（修复前 20.574 ms）。

### Example61 按键
- `B` 键由「硬急停」改为**与 Example37 一致的优雅趴下**（`do_lie_down`），并把判断提升到每拍执行，
  使**标准对比序列进行中也能按 B/START 安全退出**；`Ctrl+C` 仍为硬急停。

---

## 2026-10-04 ~ 10-08 | PACE 复现 Phase 0 + 模型参数修正 + 反馈链路定位（补记）

> 这一段当时只写了提交信息、没进本文件，2026-10-12 整理时补记（详见 `memory/FACT.md` 对应章节）。

### PACE 复现（新增专题，`docs/PACE_*.md` + `paper_2509.06342/`）
- 方法依据 arXiv 2509.06342（IJRR 2026）：辨识 `p = [I_a, d, τ_f, q̃_b, T_d]ᵀ ∈ R^{4n+1}`（12 腿 = **49 参**），
  损失 = 时均关节位置平方误差，激励 = 固定基座 + 悬空 + **位置 chirp 20 s**，优化 = CMA-ES / 4096 并行回放。
- **Phase 0 完成**：Isaac Lab 2.3.2 vs mjlab 1.2.0 双仿真器对照；配置层 49 维上下界逐字一致、
  资产同源（URDF 字节相同）；两侧各自拟合自产数据、**五个真值全部还原**（误差 ≤0.33%）——
  但属自洽拟合，只证明实现正确、不能区分仿真器。
- **资产归因（解掉 P0 阻塞）**：上游 URDF 57.0279 kg / 96 links，MuJoCo 编译后仅 30.4490 kg（基座掉 26.58 kg），
  Isaac USD 实测 51.6357 kg / 17 bodies；**运动链质量逐项吻合**（THIGH 5.1322 相同、SHANK+FOOT 焊合等价、HIP 在场），
  差异全在被焊死的基座上 ⇒ 对固定基座辨识等价；浮动基座 RL 训练前**必须修**。
- **真机侧**：新增 `Example62_PaceChirpCollect`（12 腿同时位置 chirp，0.1→10 Hz / 20 s / 400 Hz）
  + `tool/pace_export_dataset.py` + `docs/PACE_REAL_ROBOT_PROCEDURE.md`（前置条件 / 固件补偿寄存器 / 验收判据）。
  中途修了一个真 bug：下发必须 `urdf_to_status`、记录必须 `status_to_urdf`（否则 thigh 差 ~78°）。
  轮子改为**不使能 + 胶带物理固定**（PACE 无轮式平台，轮关节无重力负载）。

### 模型与动力学参数（`dogurdf.xml`）
- `armature` 由无依据的统一 `0.01` 改为**逐关节物理基线** = 规格书转子惯量 × 减速比²：
  hip/thigh **0.0289**、calf **0.1575**、wheel **0.00396**（关节侧折算值；固件 `0x45` 是电机侧，差 N²）。
- 远端质量与 hip/thigh/calf 惯量重算后，**髋连杆改动被回退**（结论：HFE 电机装在 HAA 轴上、原 CAD 值正确）。
- 现状：总质量 **57.94 kg**（真机 57.46，+0.8%）；sim2sim 与训练模型的连杆质量已对齐。

### 反馈链路 100 ms 攒批（新发现，见 `memory/FACT.md` §反馈链路）
- 用 `log/pace/` 的 10 次采集查明：`GetStatus()` 的位置更新间隔中位数 **96.8 ms**，
  8000 拍里只有 **387 拍**有变化 ⇒ 控制环实际只看到 **~10 Hz** 的位置反馈。
- 根因定位到厂商 SDK：`libdm_device.so` 的 `usb_handle_thread_func()` 把 libusb 事件收割超时写死 **100 ms**
  （`.rodata@0x151d8`，全库仅此一处），且不提供任何可调项。

---

## 2026-10-12 | 反馈链路诊断落地 + Example62 采集修复 + 合并 stable

### 新增：反馈链路诊断工具链
- `tool/libusb_fastpoll.c`（新）：LD_PRELOAD 垫片，把上述 100 ms 事件收割超时压到 `DMCAN_POLL_US`（默认 1 ms，0 = 关闭）。
  CMake 目标 `usb_fastpoll` → `bin/libusb_fastpoll.so`；`run.sh` 自动接线；不加载即原行为。
  硬件无关实测：不加载 100.157 ms / 加载 1.193 ms / `DMCAN_POLL_US=5000` → 5.089 ms / `=0` → 100.156 ms。
- `src/app/main.cpp`：启动时**自检垫片是否真的加载**（漏写 `LD_PRELOAD`、被 `setcap` 的 `AT_SECURE` 忽略、
  被 `DMCAN_POLL_US=0` 关闭——这三种情况从外部完全看不出来）。
- `src/transport/usb2can_transport.cpp`：新增 `DMCAN_RXDIAG=1` 探针，打印 `onRecv` 到达间隔直方图，
  用于判定攒批发生在厂商栈内部还是我们自己的消费路径。
- `src/runtime/thread_manager.cpp`：`pthread_setschedparam` 失败**不再静默降级**，打印一次告警并说明开启方式。
  （实测本机 `ulimit -r` = 0 ⇒ **SCHED_FIFO 从未生效**，500 Hz 收发线程一直跑在 CFS 上；
  ⚠️ 开启请用 `limits.conf` 的 `rtprio`，**不要用 `setcap`** —— 那会让 `LD_PRELOAD` 失效、垫片白做。）
- ⚠️ **真机实测结论**：垫片加载后突发间隔**仍是 99~100 ms**（`meas` 20.1 Hz，垫片前 17.8 Hz）
  ⇒ 攒批在 libusb 之下，垫片不够；下一步按 `DMCAN_RXDIAG` 直方图决定是否换 socketcan 传输层。

### Example62 采集修复
- **PD 增益** `50/1` → `rl::LEG_KP/rl::LEG_KD`（= **250/4**，与真机部署同源）：PACE 硬规则是"辨识与部署同一套增益"。
- **起立方式**：原为"目标直接等于 `DEFAULT_POSE`"（一个 tick 到位，kp=250 下变成猛拽）⇒ 改为
  读当前位 → **线性插值 10 s**（对齐 `Example47`）→ **静置 5 s** → 取实测角作 chirp 偏置；新增该阶段越限保护。
- **节拍**：删除 `usleep(1e6/HZ)`（= 睡 2.5 ms + 循环体 0.22 ms），改绝对 deadline 等距调度。
  真机实测 **+8.6~8.9% → −0.003%**（20.00 s 整 8000 拍），`pace_export_dataset.py` 的时间基判据由"全部拒收"变为通过。
- 新增 `tool/pace_qc.py`：一条命令出体检表（拍数/周期偏差/最大单拍/`meas` 刷新率/各关节最长保持/
  无反馈通道/末行截断），`--compare A B` 对比两次运行，退出码 0 = 全过。

### 合并 stable
- `ca9486e` 把 `stable`（运行时三变体 + 5350 演示策略 + 节拍修正）合并进 `develop`。
  冲突仅 `main.cpp`（示例激活 + 顶部注释），其余（`CMakeLists.txt`/`thread_manager.cpp`/`SIM2REAL_DATA_FEEDBACK.md`）自动合并且**双方改动均保留**。
- 合并后验证：编译零告警；二进制内**三套权重俱在**；`Example30` 离线回归
  **9754 = 1.67e-6 / 10000 = 4.29e-6 / 5350 = 2.38e-6 全通过**；`weights/iteration_5350.pkl` 与 git 历史 `0d0257c` **逐字节一致**。
- 激活示例切到 **Example37**（`37ad2d2`），运行时会自报 `[策略] h52_s45_v4/iteration_5350（可前进）`。
- ⚠️ 遗留：`run_sim2sim.sh` 默认 checkpoint 仍是 `iteration_9754`，与真机（5350）不一致 ⇒
  对比须 `SIM2SIM_CKPT=../weights/iteration_5350.pkl`。

---

## 2026-10-12（续）| 新增姿态可视化工具 + 发现名义站姿不一致

### 新增 `tool/pose_view.py`（Python + MuJoCo，调试用）
- 用真实 MJCF 做**正运动学**（只 `mj_forward`、**不跑物理**），把 12 个腿关节画成**连杆骨架**：
  关掉 mesh（group 1）、显示碰撞体（group 3）。
  ⚠️ 踩到一处：MJCF 里**所有碰撞体的 `rgba` alpha 都是 0**（仿真里本来不可见），
  只切 geomgroup 会得到一副**隐形的骨架** ⇒ 工具里强制把 group3 置为不透明。
- 数据源三种，自动识别：`log/dataset_*.csv`（status/CAN 序 + IMU quat）、
  `log/recv_*.csv`（同，无 IMU）、`log/pace/chirp_*.csv`（已是 URDF/PACE 序）。
  **换算常数从 `sim2real_conv.cpp` / `dogurdf.py` 解析，不手抄**（项目规范）。
- 机身位姿用 IMU quat；⚠️ IMU **无磁力计 ⇒ 偏航基准任意**（2026-10-06 数据集首帧就是 51° 偏航），
  默认 `--imu-mode yaw0`：以首帧为基准去掉初始偏航（保留真实转向），另有 `full/tilt/none`。
- 窗口内按键：空格暂停、`N` 单步、`R` 回起点、`1..4` 切预设、`0` 隐藏机身、`Q` 退出。
- 另带 `--selftest`（不开窗口验证坐标链路，含 FK 检查）与 `--shot x.png`（离屏出图给文档用）。
- ⚠️ `log/recv_*.csv` 的 `elapsed_ms` 是**按突发打戳**的（100 ms 攒批，见 `memory/FACT.md` §反馈链路），
  37.88 s 只组出约 820 帧 ⇒ recv 回放"一跳一跳"是**数据本身如此**，不是工具的问题。

### 🔴 发现：部署侧 `DEFAULT_POSE` 与训练/sim2sim 的名义站姿不一致
- `src/strategy/sim2real_conv.cpp` 的 `DEFAULT_POSE` calf = **−0.35**；
  `dogurdf_sim2sim_deploy/src/dogurdf.py` 的 `NOMINAL_CALF` = **−0.53**（`NOMINAL_TORSO_HEIGHT` = 0.40）。
  差 **0.18 rad = 10.31°**，而 `sim2real_conv.h` 的注释写着"与 dogurdf.py 的 NOMINAL_* 一致"——**这句是错的**。
- 用新工具做 FK 实测（机身 z = 0.40 m，四腿相同）：
  calf **−0.53** → 轮心 z **113.7 mm**（= 轮半径，**刚触地**）、轮心比髋后 24.5 mm；
  calf **−0.35** → 轮心 z **63.6 mm**（**陷地 50 mm**）、轮心比髋后 59.5 mm。
- 影响：若训练侧确实用 −0.53，则 `joint_pos_rel = q − DEFAULT_POSE` 的 4 个小腿通道带
  **−0.18 rad 的恒定偏置**，`leg_pos_target` 的基点也差同样多。
- **本次只标注、不改数值**（改它会直接改变真机站姿与策略观测）。待查 repo 外的 `RL_Train/code`
  真实训练配置 + 现场确认后再定。
- 可视化复核：`tool/pose_view.py --preset default|nominal`（侧视对比图见 `tmp_s2r/figs/`）。

### 姿态可视化接上实时（UDP 姿态流）
- 新增 `include/common/pose_stream.h`（header-only）：把 **16 个 URDF 关节角 + IMU 姿态**
  用 **非阻塞 UDP** 发给 `tool/pose_view.py`（164 字节/帧，格式对 Python 端有 static_assert 兜底）。
  · **必须由本进程发**：达妙适配器同一时刻只能被一个进程占用（libusb claim 接口），
    可视化进程没法自己读 CAN。
  · **默认关闭**：`POSE_STREAM=1` 才建 socket；不开时 `send()` 只有一次 bool 判断。
  · viewer 没开也不影响机器人（UDP 无连接、丢帧无妨、永不阻塞控制环）。
  · 环境变量：`POSE_STREAM` / `POSE_HOST`(127.0.0.1) / `POSE_PORT`(45555) / `POSE_HZ`(100)。
- 接入点：`MotionController::rlStep()`（Ex25/36/37/51/61 等所有走 MotionController 的 RL 示例自动可用），
  在 50 Hz 环末尾发一帧"实际姿态 + 指令目标"。发的是 **URDF 坐标 + POLICY 序**（MuJoCo 期望的那层）。
- `tool/pose_view.py` 新增 `--udp [port]`：后台线程收流、只保留最新帧，
  界面显示 `seq / 包率 / 丢包数`；无包时回退显示预设。`--shot` 也可配合 `--udp` 实时抓帧存档。
- `run.sh` 注释里加了开启方式（`POSE_STREAM=1 ./run.sh`）。
- **端到端验证（无需机器人）**：合成发送端（直接 include pose_stream.h）+ 接收端
  → 40/40 包、0 丢、q/q_target/quat/gyro 逐位相符；`--udp --shot` 出图正常（机身按 quat 横滚）。

### 新增 `Example63_PassivePoseMonitor`：**不使能电机**的被动姿态监视

动机：**电机在未使能状态下也能响应参数读取**，所以可以做一个"绝不产生力矩"的姿态监视器，
用来安全地看姿态（尤其是零位核对这类准静态场景）。

- 做法：`ReadParam(…, MOTOR_OR_angle=0x0F)` 走"读写参数"帧（**不发任何控制/力矩帧**），
  回复由 `unpack_frame` 的参数分支解析并同样过 `ApplyMotorCalibrationPos`
  ⇒ `GetStatus()` 得到标定后的 status 角，再 `rl::status_to_urdf` 转 URDF 喂 MuJoCo。
- **只启 `motor_receive` 线程**（`ReadParam` 直接发帧，不需要 `motor_send`）。
- 轮询：500 Hz 每拍发 1 个请求、16 拍扫完全部电机 ⇒ **整帧 ≈ 31 Hz**；
  再叠加 IMU（`Z_DOWN_X`）组姿态，经 UDP 姿态流发给 `tool/pose_view.py --udp`。
- ⚠️ **安全前提**：不使能 = 无保持力矩 = **狗会软掉**，必须架起/吊起（或本就趴着）。
  本示例不会使能任何电机、不会产生运动指令。
- ⚠️ 实测踩到的坑（已加防护）：**驱动器未上电时参数请求发得出去、没人回**，
  `position` 恒为 0 ⇒ `status_to_urdf(0) == CONV_B`（hip +1.70°/thigh −55.14°/calf −73.52°），
  看起来像个"合法姿态"，viewer 分不出来。现用 `Usb2CanTransport::RxCount()` 判活：
  **本秒零回帧 ⇒ 告警并停发姿态流**。
- 真机实测（2026-10-13，适配器已插但驱动器未上电）：适配器 2 台/4 路打开成功、整帧 30 次/s、
  IMU 有；因零回帧而正确进入"停发 + 告警"分支。**待驱动器上电后再验读数是否正确**。

### 真机上电后实测 Ex63（2026-10-13）—— 流程走通，并修了两处 bug

✅ **"不使能也能读关节角"得到证实**：适配器 2 台/4 路打开、整帧 30 次/s、UDP 29.2 包/s 丢 0、
IMU 正常（roll +2.0°/pitch +6.3°），16 个关节读数稳定且**四条腿各不相同**。
全程未使能、未发任何力矩/控制帧。

修掉的两处（都是我前面引入的）：
1. **Ex63 存活判断粒度错**：原按每个扫描周期（32 ms）判断"本周期有没有回帧"，但回帧是
   **~100 ms 突发投递**的 ⇒ 32 ms 窗口里大多本来就没帧 → 误报"驱动器没上电"并把姿态流一起
   停掉（整帧从 30 掉到 10~11）。改为"**曾经**收到过回帧就持续发"，判活/告警放到每秒块里，
   并区分"从未收到"与"本秒无回帧（突发投递的正常间隙）"。
2. **`pose_view --udp` 下 `yaw0` 等于没生效**：偏航基准 q0 原来只从 CSV 源取，实时模式拿不到
   ⇒ 狗朝任意偏航站着（实测 46°）。改为用**收到的第一帧**懒初始化（viewer 与 `--shot` 两条路径）。

🔴 **查出真硬件问题**：`log/recv_*.csv` 按 (can_port, motor_id) 统计回帧 ⇒
**CAN3-M4（RR 轮）= 0 帧**，其余 15 个电机各 ~375 帧 ⇒ 该电机不回帧，viewer 里的 0.00
是"没读到"而非真位置。待查接线/上电/节点 ID。

### 新增 `tool/pose_view.sh`（包装脚本）
- 自动挑选装了 mujoco 的 python（显式 `$PY` 优先 → 本项目约定路径 → PATH），再 exec
  `pose_view.py`，省得每次手打长路径；所有参数原样透传。
- 顺带在文档里明确 **不要用 sudo**：本工具不需要特权，而 sudo 会丢 XAUTHORITY ⇒ 窗口打不开；
  且 `$PY` 为空时 `sudo $PY tool/pose_view.py` 会退化成 `sudo tool/pose_view.py` ⇒
  "command not found"（用户实际踩到）。

### 🔴 修复 FR hip 方向反了（现场确认）+ 顺带纠一个提交混入

现场用 Ex63 被动监视 + viewer 逐腿核对，用户确认：**右前(FR) hip 的观测方向是反的**
（把 FR 腿远端往机器人左侧推，URDF 读数反而变小；其余腿正常）。

**这不是显示问题**：`CONV_A/CONV_B` 同时被观测（`status_to_urdf`）与动作（`urdf_to_status`）
路径使用且精确互逆 ⇒ 策略下发的 FR hip **动作在物理上同样是反的**，而反馈自洽所以站着
（hip 名义值≈0）看不出来。

**怎么定位到的**：
- 链路 `URDF = CONV_A·(pos_scale·raw + pos_offset) + CONV_B`；hip 的 `CONV_A=+1`（四条腿同）、
  而电机层 `pos_scale` 是**对角同号**：FL −1 / FR +1 / RL +1 / RR −1。
- 该对角模式自 `57cd63b`(2026-06-13 "补充物理0位") 引入后**从未复核**（中间只有 `f10a2f5`
  把 vel_scale 同步成 pos_scale）。
- 模型侧真值（实测）：MJCF 四条 hip 的世界轴**全是 (+1,0,0)**，FK 验证「hip 正 ⇒ 该腿远端向
  机器人左侧(+y)移动」**四条腿一致**；模型本身左右严格镜像；`dogurdf.py` 四条腿共用
  `NOMINAL_HIP`。⇒ hip 的 URDF 语义必须统一，对角模式里必有错。

**改法**（`src/strategy/sim2real_conv.cpp`）：把该关节的**整个映射取反**——
`CONV_A[3]: +1→−1` 且 `CONV_B[3]: +0.0297→−0.0297`，于是 `urdf_new ≡ −urdf_old`
⇒ **零位一点不动、方向翻转**，且 `urdf_to_status` 仍精确互逆（观测与动作同步翻转）。
- ⚠ 只翻 `CONV_A` 会让零位偏 `2×CONV_B = 0.0594 rad (3.4°)`。
- ⚠ 不动电机层 `pos_scale`（那是"物理编码器正方向"的实测声明，且只改它同样挪零位）。

**验证（无需机器人）**：用**实际编译出的常量**跑 `status_to_urdf/urdf_to_status` ⇒ 仅 FR hip
从 `+0.3297` 变 `−0.3297`，其余 15 个关节逐位不变；raw 域零点四条腿全部不动；互逆误差 1.2e-7
（float 精度极限）。

**同步**：`sim2real_conv.h` 注释、`docs/SIM2REAL_DEPLOY.md`、`memory/FACT.md`；
`tool/plot_rlrun.py` 原来**硬编码** `CONV_B_HIP=0.0297, A=+1`（第二真值来源，改完必错）
⇒ 改为从 `sim2real_conv.cpp` **解析** CONV_A/CONV_B 并逐腿换算；
`motor_calibration.h` 里那条"FR hip 偏 +0.18 ⇒ 把 offset 提到 0.78"的旧评估**加了警示**：
它发生在符号被确认反了之前，很可能是符号错误的表象，**重新现场核实前不要再动 offset**。

**待办**：RL(左后) hip 未确认（电机层与 FR 同号，首要嫌疑），thigh/calf 也建议一并复核；
判据：腿远端向**左**推 ⇒ hip 读数**变大**，向**后**推 ⇒ thigh/calf 读数**变大**。

**顺带纠正**：上一提交 `56a6ade` 的 `git add -A` 把当时工作区里 `main.cpp` 的
"Ex63 激活"状态一并提交了（而 `FACT.md` 记的当前激活是 Ex37）⇒ 本次把 `main.cpp`
恢复为 **Ex37 激活**（Ex63 回到注释态），与文档一致。

### 🔴 查清 IMU 打开失败：物理层故障（不是软件）+ 新增 `tool/check_imu.py`

现象：真机上 IMU 打不开（程序打印 `[WARN] IMU 打开失败` / Ex63 提示"机身姿态按水平处理"）。

**证据链（时间线）**：
| 时间 | 事件 |
|---|---|
| 10-13 19:06:55 | `ch341 1-4:1.0: ch341-uart converter detected` → `usb 1-4: attached to ttyUSB0` |
| 10-13 19:18:10 | **`ttyUSB0: ch341-uart converter now disconnected`** / `device disconnected` |
| 10-16 20:14:49（本次开机） | 整个开机周期**无任何 ttyUSB/ch341 记录**（IMU 再没枚举成功） |
| 10-16 22:07:23–25 | 同一端口 **1-4**：`device descriptor read/64, error -71`(EPROTO) ×4 → `not accepting address` → 内核放弃 |

⇒ IMU = **CH340 USB 转串口，挂 USB 端口 1-4 → `/dev/ttyUSB0`**；**描述符阶段就失败 ⇒ udev/用户态根本没参与**。
结论：线材/接头/供电/USB 口/CH340 模块的**物理层问题**。

**排除项（都查过了）**：
- **brltty**：`brltty`/`brltty-udev` 均 `masked`+`inactive`；`/usr/lib/udev/rules.d/85-brltty.rules:293`
  确实匹配 CH340（`1a86/7523`）但 `brltty_usb_run` 标签唯一动作是
  `ENV{SYSTEMD_WANTS}+="brltty-udev.service"` ⇒ 服务已 mask = 空操作 ⇒ **排除**。
- **权限**：`bruce` 在 `dialout` 组 ✓（另：FACT.md 早前记的"CH340 udev MODE=0666 规则"其实不存在，
  但不影响）；**ModemManager**：snapd 规则对 `1a86:7523` 设了 `ID_MM_DEVICE_MANUAL_SCAN_ONLY=1` ✓。
- **软件/配置**：串口路径与波特率（`/dev/ttyUSB0` @115200）与 10-13 成功那次完全一致，代码没动。
- **CAN 适配器**：是 `ttyACM*`（cdc_acm），与 IMU 无关；内核 `cdc_acm: probe of 1-x:1.3 failed with
  error -16` 是 SDK libusb 抢占接口的 EBUSY，**每次跑程序都会打，属正常**。

**新增 `tool/check_imu.py`**（纯标准库，不依赖机器人）——为什么需要它：
- `ImuDevice::Initialize()` **只判 `open()` 成功，不验证有没有有效帧** ⇒ "IMU 有"不代表数据在流；
- IMU 缺失时 RL 示例**只打印 WARN 就继续跑**，姿态退化成单位四元数（假设机身水平）⇒ 策略平衡信号是假的。

功能：解析维特 WT 帧（`0x55` + type + 8 数据 + SUM，定长 11），统计各类帧率/校验失败/帧头外字节，
打印 quat（含 `Z_DOWN_X` 安装补偿，可 `--raw` 关闭）与 roll/pitch/yaw、gyro；设备不存在时给出
`lsusb`/`journalctl -k -f` 排查步骤；退出码可脚本化（0 正常 / 1 打不开 / 2 无有效帧 / 3 自检失败）。
带 `--selftest`：用**伪终端**灌合成帧验证解析器（自检当场抓出我两个 bug：① pty slave 默认 canonical
模式会按换行缓冲、吃掉无换行的帧；② 维特帧是**定长 11 字节**，陀螺帧 8 个数据字节（6 角速度 + 2 温度），
只填 6 个会让帧变 9 字节、后续全部错位）。

### 2026-10-17 反馈链路（T-A）机制细化 + 两个新工具

**T-A 仍成立**（CAN 回帧每 ~77~100 ms 才交付一次 ⇒ 策略的关节观测量化到 ~10 Hz），
但排除了两处原有嫌疑，并把定位收窄到 **SDK 的入站派发层**。

新工具：
- `tool/libusb_xferprobe.c` → `bin/libusb_xferprobe.so`：LD_PRELOAD 探针，打印 SDK 的
  `libusb_submit_transfer` 请求（端点/类型/长度/超时）、实测完成间隔与 `actual_length` 直方图；
  支持 `DMCAN_XFER_BYTES=<n>` 夹小 bulk IN 请求长度做 A/B。
- `Example64_FeedbackRateDiag`：**不使能电机**，全速发参数读取帧制造 RL 级入站流量，
  统计 `MotorPosUpdateCount()`（新增计数器：`current_position` 被写入的次数）⇒ 应用可见更新率。
  env：`FB_REQ_HZ`（总请求率，默认 8000 ≈ RL 的 16×500 Hz）/ `FB_SECS`。

实测结论：
1. **攒批与帧类型无关**：RL 控制帧回帧（500 帧/s）与参数读回帧（504 帧/s）**都是**
   "每时刻最多 51 帧、每秒仅 13~15 个投递时刻" ⇒ 属 SDK 入站派发层。
2. **不是 USB 传输长度**：SDK 异步 bulk IN 请求 **4096 B / timeout=0**（ep 0x81/0x83），
   但每次完成实际只带回 **~23 B（≈1 帧）**、完成间隔 **32~127 µs**；
   `DMCAN_XFER_BYTES=512` 夹小后 A/B **无差别**。
3. **也不是 100 ms 事件超时**（与既有垫片真机无效一致）；故 `libusb_fastpoll.so` 保持现状即可。
4. ⚠️ **测量陷阱（本轮踩到并纠正）**：`log/rl_*.csv` 是 `%.4f`，静止关节读数**逐位重复**
   （相邻帧 |Δ| 中位 = 0）⇒ "值不变"≠"没更新"。**判新鲜度必须用突发结构**（同一毫秒的帧数 /
   每秒投递时刻数）。用"写入次数/秒"也会误判——100 ms 一批的 50 帧照样给出 500 次/s。
   PACE 原始数据交叉验证：chirp 时 `des_*` 每拍都变（Δ 中位 2.47e-3 rad），`meas_*` 却
   **连续 40 拍（100 ms）完全相同** ⇒ 关节在动而反馈冻结，陈旧是真的。
5. 🔴 **CAN3-M1（RR hip）在 8000 请求/s 下 0 次更新**（其余 15 个电机各 ~498 Hz）⇒ 该电机
   **根本不回帧**，是独立于攒批的硬件问题（10-13 那次是 CAN3-M4，同一腿"换着电机"不回）。

### Ex63 增强：逐电机静默检测（自己点名哪个电机没回帧）

现场 symptom：`RR hip` 打印 **`+1.70°`**，而它正好等于 `CONV_B[hip] = +0.0297 rad`
—— 即 `status = 0`，**该电机根本没回帧**。但 `+1.70°` 看起来完全像个合法姿态，
肉眼无法区分"真在 1.7°"和"没数据"。

现用上一轮新增的 `MotorPosUpdateCount()`（`current_position` 被写入的次数）做**每秒增量判活**：
某电机本秒计数零增长 ⇒ 直接点名：

```
🔴 本秒无回帧的电机 1 个：CAN3-M1 ⇒ 这些通道的读数恒等于 CONV_B，**不是真实姿态**
```

现场验证（2026-10-16 23:24）：只 `CAN3-M1`（RR hip）被点名，同腿 M2/M3/M4 正常
⇒ 故障只在那个电机的接线/插头，**不是整条 CAN3 总线**（与"CAN3 上任何时刻恰好一个电机静默、
且换过（10-13 是 M4）"的间歇故障规律一致）。
