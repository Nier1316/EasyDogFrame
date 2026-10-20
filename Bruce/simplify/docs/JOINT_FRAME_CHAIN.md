# 关节角坐标系全链路：SolidWorks/CAD → 驱动器原始编码器

> 只读核对，未改动任何 `.cpp/.h/.xml`。所有结论附**文件:行号**，推断标【推断】，查不到标【未找到】。
> 相关但不同的文档：`docs/SIGN_MAP.md`（只讲"符号"）、本文（讲**符号 + 零点 + 量纲 + 层间一致性**）。
> 核对基线：`git rev-parse --short HEAD` = `08dffa1`（工作区干净）。

---

## 0. 一页速览

**一句话链路**

```
SW/CAD 几何角  --(SW 导出)-->  URDF 关节系 q_urdf  ≡MJCF qpos  --(纯置换)-->  POLICY 序
   --(CONV_A/B：±1 符号 + 常数)-->  status（GetStatus/SendImpedance 的坐标）
   --(pos_scale：±1 符号 + pos_offset：常数)-->  驱动器 raw（固件编码器）
```

**复合关系（每一层都是"乘 ±1 + 一个常数"，所以整条链可合成一个仿射式）**

```
q_urdf = A · ( s · raw + o ) + B  =  a · raw + b ,   a = A·s ,  b = A·o + B
        └─── status ───┘
A = CONV_A[j]      src/strategy/sim2real_conv.cpp:45-55
B = CONV_B[j]      src/strategy/sim2real_conv.cpp:56-63
s = pos_scale      include/motor/motor_calibration.h:40-75
o = pos_offset     include/motor/motor_calibration.h:40-75
```

**三条"零点声明"同时存在，任一条错都会让整链偏移**

| 声明 | 位置 | 含义 |
|---|---|---|
| `q=0` 的物理姿态 | `urdf/dogurdf.urdf` + `dogurdf.xml` | 模型零点（SW 导出），策略在此坐标训练 |
| `CONV_B` | `sim2real_conv.cpp:56-63` | 真机 status 零点 ↔ 模型零点 |
| `pos_offset` | `motor_calibration.h:40-75` | 驱动器 raw 零点 ↔ status 零点 |
| `raw=0` 的物理姿态 | 固件（`SetZero` → `motor_manager.cpp:261-272`） | 电机自带绝对零点，硬件事实 |

**核心判断（详见 §6.11）**：PACE 的 calf `bias≈−0.5` **不构成"某一层零点差 0.5 rad"的证据**；它更可能是
被顶到边界的、在辨识模型里不可辨识/被别处误差顶出来的补偿量。**先做 §5 的两点静态实测，再动任何一层。**

---

## 1. 链路分层、公式与应用位置

### 1.1 完整链路图（含读/写两个方向）

```
写（50 Hz 策略 → 500 Hz 发送）                                    读（500 Hz 接收 → 观测）
─────────────────────────────────────────────                    ─────────────────────────────────────────
action[16] (POLICY)                                               CAN 帧
  │ rl::leg_pos_target()                                              │ unpack_frame()            ele_motor.cpp:245
  │   q = DEFAULT_POSE[p] + 0.25·action , clip URDF 限位              │   raw  = uint_to_float(15bit, ±12.5)  :336
  ▼ rl_controller.h:132-137                                          ▼   [轮] 一阶低通 0.2        :343
q_urdf (POLICY, = MJCF 坐标)                                        raw (驱动器原始读数)
  │ rl::urdf_to_status()  sim2real_conv.h:41-43                       │ ApplyMotorCalibrationPos/Vel/Torque
  │   status = (q_urdf − B)/A                                        │   status = raw·s + o      :288-289 / 298-299 / 309-310
  ▼                                                                  ▼   （周期回帧：一次性三量）:350-351
status (POLICY)                                                    status (CAN 序, 已标定)
  │ MotorManager::SendImpedance()  motor_manager.cpp:287-301          │ MotorManager::GetStatus()  motor_manager.cpp:333-348
  │   只**存**目标，不换算                                             ▼
  ▼                                                                 status
target_position (status 坐标)                                        │ POLICY_TO_CAN 置换  rl_controller.cpp:38-43
  │ [500Hz] MotorManager::SendOnce()  motor_manager.cpp:377-504       │ rl::status_to_urdf()      sim2real_conv.h:33-35
  │   [+ 腿摩擦前馈 override，当前关闭 rl_controller.h:160]           ▼
  │ ApplyMotorCalibrationInverse()   motor_manager.cpp:453         q_urdf (POLICY)
  │   raw = (status − o)·s ,  τ ×= s                                  │ obs[9..20] = q_urdf[:12] − DEFAULT_POSE[:12]
  ▼                                                                  ▼   rl_controller.cpp:110-113
set_motor_para_bt() → float_to_uint() → CAN 帧                       obs[64] → MLP
  ele_motor.cpp:159-221 / 371-377
```

### 1.2 逐层公式与应用位置（按"层"编号）

| 层 | 变换 | 公式 | 代码 / 行号 | 何时应用 |
|---|---|---|---|---|
| **L0** SW/CAD 几何 | 无（真值） | — | — | — |
| **L1** URDF/MJCF 关节系 | q_urdf = q_mjcf（同一次 SW 导出） | 关节 `origin/axis/range` | `urdf/dogurdf.urdf:83-96,144-157,211-229,266-280`；`dogurdf.xml:107-130,136-160,166-190,196-220` | 训练时（策略的"世界"） |
| **L2** POLICY 序 | 纯索引置换，无符号无数值变换 | `pos_policy[p] = pos_can[POLICY_TO_CAN[p]]` | `rl_controller.cpp:38-46`；用法 `motion_controller.cpp:143-144`、`ex_rl.cpp:163-164` | 每次观测构建（50 Hz） |
| **L3** status ↔ URDF | 每关节 ±1 + 常数（**rad**） | `status_to_urdf: urdf = A·status + B`<br>`status_vel_to_urdf: dq_urdf = A·dq_status`<br>`urdf_to_status: status = (urdf − B)/A` | `sim2real_conv.h:33-43`；常量 `sim2real_conv.cpp:45-63` | 观测（每拍）＋动作（每拍）；**唯一一处** RL↔硬件坐标转换 |
| **L4** raw ↔ status | 每电机 ±1 + 常数（**rad**） | `status = raw·s + o`；逆 `raw = (status − o)·s`；力矩 `τ_status = raw_τ·s` | 收：`ele_motor.cpp:288-289 / 298-299 / 309-310 / 350-351`；发：`motor_manager.cpp:453`（`motor_calibration.h:220-240`） | 收：**解包 CAN 帧时**；发：**SendOnce 组帧前**（不是 `SendImpedance`） |
| **L5** 协议编码 | 线性量化 | `float_to_uint(v, v_min, v_max, bits)` | `ele_motor.cpp:371-377`；量程 `ele_motor_def.h:123-133`（p ∈ ±12.5 rad、腿 v ∈ ±3、轮 v ∈ ±48） | 每帧 |
| **L6** 固件 | 阻抗环 τ = kp(θ_t−θ) + kd(ω_t−ω) + τ_ff | — | 说明见 `motor_calibration.h:80-95` | 1 kHz（固件内） |

> **口径纠正（与你给的印象不符）**：`CONV_A/CONV_B` 的**数值定义在 `.cpp`**，`sim2real_conv.h` 只有
> `extern` 声明（`sim2real_conv.h:29-30`）。改符号要改 `src/strategy/sim2real_conv.cpp:45-63`。

### 1.3 "calibration 到底在哪个环节被应用"（你点名要的）

| 方向 | 应用点 | 行号 | 说明 |
|---|---|---|---|
| **收**（raw→status） | `unpack_frame()` 内部 | `ele_motor.cpp:288-289`（角度）、`298-299`（速度）、`309-310`（力矩） | **单寄存器回帧**（`MOTOR_OR_angle`=0x0F / velocity=0x10 / torque=0x11，`ele_motor_def.h:34-36`）——只标定"被更新的那一个量"，注释解释了历史二次标定事故 `ele_motor.cpp:267-273` |
| **收**（周期回帧） | `unpack_frame()` 末尾 | `ele_motor.cpp:350-351` | `set_motor_para_bt` 之后的周期帧三量同帧到达，整体调用 `ApplyMotorCalibration()` |
| **发**（status→raw） | `MotorManager::SendOnce()` | `motor_manager.cpp:453` | `SendImpedance/SendSpeed/SendPosition` **只存目标**（`287-301/303-315/317-331`），逆标定在 2 ms 的发送线程里做 |
| **发**（轮 IMPEDANCE 特例） | `SendOnce()` 轮分支 | `motor_manager.cpp:469-472` | 轮子自己算 `send_vel_w / send_tau_w`，**丢弃**了 453 行的结果（等价，但两处并存） |
| **发**（轮急停） | `WheelEmergencyStop/SendOnce` | `motor_manager.cpp:363-364 / 415-417` | 直接用 `vel_scale` 逆变换 |

⇒ **推论**：`GetStatus()`（`motor_manager.cpp:333-348`）返回的是**已标定的 status**，不是驱动器原始值；
驱动器原始值只出现在 `MotorLogger` 的 `raw_pos,raw_vel,raw_torque` 列（`include/common/motor_logger.h:105`，
由 `ele_motor.cpp:290-292` 等写入）与 `[PARAM]` 打印（`ele_motor.cpp:262-265`）。

### 1.4 应用顺序（时序）

1. 50 Hz 控制拍：`GetStatus`(→status) → `status_to_urdf` → obs → MLP → `leg_pos_target`(URDF) →
   `urdf_to_status` → `SendImpedance`（**存** status 目标）。
2. 500 Hz 发送拍（独立线程，2 ms）：读存下的 status 目标 →（可选摩擦前馈，加在 **status** 坐标，
   `motor_manager.cpp:446-452`，当前 `LEG_FF_ENABLE=false`）→ `ApplyMotorCalibrationInverse` →
   `float_to_uint` → CAN。⇒ **同一个目标被重发约 10 次**，位置目标在 500 Hz 上是零阶保持。
3. 500 Hz 接收拍：CAN 帧 → `unpack_frame` → 标定 → `current_position` → `GetStatus`。

⚠ 因此 `GetStatus` 的延迟 = (1/50 采样) + (2 ms 发送) + 固件环，而 `udp/CSV` 里 `des` 与 `meas` 严格同拍
记录（`ex_probe.cpp:503-517`：**先下发再读反馈**）。

### 1.5 力矩与速度的符号（与位置不是同一处）

- 速度：`vel_scale`（**必须 = pos_scale**，`motor_calibration.h:35-38` 记录了 2026-09-04 事故），
  URDF 速度再乘 `CONV_A`（`sim2real_conv.h:37-39`）。
- 力矩：反馈 `ApplyMotorCalibrationTorque` 乘 **`pos_scale`**（`motor_calibration.h:195-202`），
  下发 `τ_ff` 在逆变换里也乘 `pos_scale`（`motor_calibration.h:236-239`），
  而**落盘/工具**里的 `tau_policy[i] = CONV_A[i]·tau_can[...]`（`ex_rl.cpp:1101`、`ex_rl.cpp:2192`、
  `motion_controller.cpp:196`）。⇒ 力矩的"URDF 化"只发生在**记录**时，控制回路里不存在。

### 1.6 顺序置换（陷阱）

- **CAN 序 == MJCF 序**（每腿 hip/thigh/calf/wheel，`dogurdf.xml:20-23`、`230-246`；`i = can*4 + (mi−1)`）。
- **POLICY 序** = 12 腿 + 4 轮：`POLICY_TO_CAN = {0,1,2, 4,5,6, 8,9,10, 12,13,14, 3,7,11,15}`（`rl_controller.cpp:38-40`）。
- 旧名 `POLICY_TO_MJX` / `MJX_TO_POLICY` 的**字面含义与实际数组相反**（`rl_controller.cpp:45-46`，
  `rl_controller.h:98-101`）。新代码只用 `POLICY_TO_CAN` / `CAN_TO_POLICY`。

---

## 2. 逐关节数值表

### 2.1 MJCF/URDF 关节定义（一次导出，MJCF 与 URDF 逐项相同）

| 关节 | MJCF name（腿前缀 FL/FR/RL/RR） | axis | range (rad) | body `pos` / `quat`（父→本） |
|---|---|---|---|---|
| hip | `xx_hip_joint` | `1 0 0`（四条腿**全同向**） | −0.6 … +0.6 | `pos=±0.3265 ±0.08 0`，quat 单位 |
| thigh | `xx_thigh_joint` | `0 1 0` | −0.7 … +1.75 | `pos=0 ±0.1308 0`，`quat=0.872818 0 0.488046 0`（=rpy 1.0197） |
| calf | `xx_calf_joint` | `0 1 0` | −1.0 … +0.35 | `pos=0 0 −0.34`，`quat=0.51675 ∓1.2e−6 −0.856137 ±1.2e−6`（≈绕 y −2.0555） |
| wheel | `xx_wheel_joint` | `0 1 0` | 无（continuous） | `pos=−0.10583 ±0.045 −0.32311`，`quat=0.994159 0 −0.107924 0`（=−0.21627） |

行号：`dogurdf.xml:109/114/120/127`（FL）、`139/144/150/157`（FR）、`169/174/180/187`（RL）、`199/204/210/217`（RR）；
URDF 原文 `urdf/dogurdf.urdf:83-96 / 144-157 / 211-229 / 266-280`。
`|wheel 偏置向量| = 0.34297 = LEG_L3`（`robot_calibration.h:66` 的 0.343 即此值）。

### 2.2 标定表 + 复合关系（`q_urdf = a·raw + b`，实数值）

`A`/`B` 见 `sim2real_conv.cpp:45-63`；`s`/`o` 见 `motor_calibration.h:40-75`。
`POLICY_TO_CAN` 给出 policy 索引 p → CAN 索引 i（`rl_controller.cpp:38-40`）。

| 关节类型 | 腿 | MJCF axis / range | `CONV_A` | `CONV_B` | `pos_scale` | `pos_offset` | p → CAN i | **`a=A·s`** | **`b=A·o+B`** | **复合式** |
|---|---|---|---|---|---|---|---|---|---|---|
| **hip** | FL | (1,0,0) / −0.6…0.6 | **+1** | **+0.0297** | **−1** | 0.611 | 0 → 0 | **−1** | **+0.6407** | q = −raw + 0.6407 |
| | FR | 同上 | **−1** | **−0.0297** | **+1** | 0.611 | 3 → 4 | **−1** | **−0.6407** | q = −raw − 0.6407 |
| | RL | 同上 | **+1** | **+0.0297** | **+1** | 0.611 | 6 → 8 | **+1** | **+0.6407** | q = +raw + 0.6407 |
| | RR | 同上 | **−1** | **−0.0297** | **−1** | 0.611 | 9 → 12 | **+1** | **−0.6407** | q = +raw − 0.6407 |
| **thigh** | FL | (0,1,0) / −0.7…1.75 | **−1** | **−0.9624** | **+1** | 0.441 | 1 → 1 | **−1** | **−1.4034** | q = −raw − 1.4034 |
| | FR | 同上 | −1 | −0.9624 | **−1** | 0.441 | 4 → 5 | **+1** | **−1.4034** | q = +raw − 1.4034 |
| | RL | 同上 | −1 | −0.9624 | **+1** | 0.441 | 7 → 9 | **−1** | **−1.4034** | q = −raw − 1.4034 |
| | RR | 同上 | −1 | −0.9624 | **−1** | 0.441 | 10 → 13 | **+1** | **−1.4034** | q = +raw − 1.4034 |
| **calf** | FL | (0,1,0) / −1.0…0.35 | **+1** | **−1.2832** | **−1** | 0.211 | 2 → 2 | **−1** | **−1.0722** | q = −raw − 1.0722 |
| | FR | 同上 | +1 | −1.2832 | **+1** | 0.211 | 5 → 6 | **+1** | **−1.0722** | q = +raw − 1.0722 |
| | RL | 同上 | +1 | −1.2832 | **−1** | 0.211 | 8 → 10 | **−1** | **−1.0722** | q = −raw − 1.0722 |
| | RR | 同上 | +1 | −1.2832 | **+1** | 0.211 | 11 → 14 | **+1** | **−1.0722** | q = +raw − 1.0722 |
| **wheel** | FL | (0,1,0) / 无 | +1 | 0 | **−1** | 0 | 12 → 3 | **−1** | 0 | q = −raw |
| | FR | 同上 | +1 | 0 | **+1** | 0 | 13 → 7 | **+1** | 0 | q = +raw |
| | RL | 同上 | +1 | 0 | **−1** | 0 | 14 → 11 | **−1** | 0 | q = −raw |
| | RR | 同上 | +1 | 0 | **+1** | 0 | 15 → 15 | **+1** | 0 | q = +raw |

**读法要点**

- `pos_scale` 的镜像方向**三种关节型不同**：hip = **对角**（FL− FR+ RL+ RR−）、thigh = **FL+ FR− RL+ RR−**、
  calf/wheel = **FL− FR+ RL− RR+**（`motor_calibration.h:43-73`）。
  但 `a = A·s` 里 `CONV_A` 在 thigh/calf/wheel 上四腿同值（thigh −1、calf +1、wheel +1），所以合成的 `a`：
  thigh/calf/wheel 全是 **FL/RL = −1、FR/RR = +1**（左右镜像）；
  **hip 是"前负后正"**（FL/FR = −1，RL/RR = +1）——这是 2026-10-17 把 FR/RR 的 `CONV_A` 翻号后的结果。
- `b` 对 thigh/calf **四条腿相同**，只有 hip 的 `b` 随 `CONV_B` 翻号。
- `urdf = 0` 对应的 raw（即"模型零点落在驱动器的哪个读数"）：
  hip **`+0.6407 / −0.6407 / −0.6407 / +0.6407`**（FL/FR/RL/RR，**对角**）、
  thigh **`−1.4034 / +1.4034 / −1.4034 / +1.4034`**（左右镜像）、
  calf **`−1.0722 / +1.0722 / −1.0722 / +1.0722`**（左右镜像）。

### 2.3 三个"关键姿态"的 status / raw（可直接上台架对照）

**(a) `DEFAULT_POSE = (0, −0.03, −0.116)`（`sim2real_conv.cpp:13-21`）**

| 腿 | 期望 `GetStatus`（status） | 期望驱动器 raw |
|---|---|---|
| 四条腿**全部相同** | hip **−0.0297**, thigh **−0.9324**, calf **+1.1672** | FL: hip **+0.6407**, thigh −1.3734, calf −0.9562 |
| | | FR: hip **−0.6407**, thigh +1.3734, calf +0.9562 |
| | | RL: hip **−0.6407**, thigh −1.3734, calf −0.9562 |
| | | RR: hip **+0.6407**, thigh +1.3734, calf +0.9562 |

> **这条是最省事的现场自检**：`urdf_to_status(DEFAULT_POSE)` 后四个 hip/thigh/calf 的 status 应**完全相同**
> （thigh/calf 的 `A`/`B` 四腿同值；hip 虽在 FR/RR 上 `A`、`B` 同时取负，但 `urdf=0` 时两项抵消，status 仍相同）。
> 若某条腿不同 ⇒ 那条腿的 `CONV_A`/`pos_scale` 与其它三条不一致（或该腿未标定）。
> **但这条只验证"一致性"，不验证"绝对零点"**——四个一样错也是可能的。

**(b) 物理零位姿态（hip 连杆水平 / thigh 连杆竖直 / calf ⊥ thigh，轮轴朝前）= `Example57` 用的姿态**

理论值（`ex_diag.cpp:2744-2745, 2788-2790`，本文 §5 已独立复核）：`q_urdf = (0, −1.020, +0.168)`。
对应：

| 腿 | 期望 `GetStatus`（status） | 期望 raw（hip / thigh / calf） |
|---|---|---|
| 四条腿**相同** | hip **−0.0297**, thigh **+0.0576**, calf **+1.4512** | FL: **+0.6407** / −0.3834 / −1.2402 |
| | | FR: **−0.6407** / +0.3834 / +1.2402 |
| | | RL: **−0.6407** / −0.3834 / −1.2402 |
| | | RR: **+0.6407** / +0.3834 / +1.2402 |

**(c) `q_urdf = 0`（模型零点）**：status = `(−0.0297, −0.9624, +1.2832)`（四腿相同；thigh 因 `A=−1` 故取负）；
raw（FL/FR/RL/RR）：hip `+0.6407 / −0.6407 / −0.6407 / +0.6407`、thigh `−1.4034 / +1.4034 / −1.4034 / +1.4034`、
calf `−1.0722 / +1.0722 / −1.0722 / +1.0722`。

---

## 3. 候选"零点层"（4 层 + 1 个硬件层）与"SW 新零点该改哪一层"

### 3.1 定义、影响面

你说的"四个候选零点层"= 下表的 **Z1/Z2/Z3/Z4**；**Z5** 是硬件层（固件编码器零点/装配），
必须一起考虑，否则会把"编码器被重零过"误判成"`CONV_B` 错了"。

| # | 层 | 数值载体 | 直接影响的消费者 |
|---|---|---|---|
| **Z1** | 模型零点（SW→URDF→MJCF 的 `q=0` 姿态） | `urdf/dogurdf.urdf` 关节 `origin`；`dogurdf.xml` 同值 | 训练（`dogurdf.py`）、sim2sim、PACE 的仿真模型、`leg_kinematics` 的连杆几何、所有 FK/IK |
| **Z2** | 策略坐标的**默认姿态/动作语义** | `DEFAULT_POSE`（`sim2real_conv.cpp:13-21`）、`ACTION_SCALE`（`rl_controller.h:31`）、`JOINT_LOWER/UPPER`（`rl_controller.cpp:13-26`）、obs 布局（`rl_controller.cpp:110-123`） | 已训好的权重（**语义绑定**） |
| **Z3** | `CONV_A` / `CONV_B`（status ↔ URDF） | `sim2real_conv.cpp:45-63` | RL 观测/动作、`Example62` CSV、PACE 数据、URDF 空间日志、`tool/plot_rlrun.py`/`pose_view.py`（运行时读 cpp） |
| **Z4** | `pos_scale` / `pos_offset`（raw ↔ status） | `motor_calibration.h:40-75` | **status 的所有消费者**：`leg_kinematics`（`robot_calibration.h:106-108` 直接引用 pos_offset 作 `ZERO_OFFSET`）、`STAND_*`/`LIE_DOWN_*`（`robot_calibration.h:154-166`）、status 限位（`robot_calibration.h:112-131`）、Ex19/21/36/37/47/50/54/57/60/63、`s2r_dataset` 的坐标注释（`src/common/s2r_dataset.cpp:83`） |
| **Z5** | 固件编码器零点（`SetZero`）／装配 | 固件 + `motor_manager.cpp:261-272` | 直接改 raw 的物理含义；改后 Z4 的 offset 必须重推 |

### 3.2 哪一层绝对不能动 —— 结论：**Z1 + Z2**

代码证据（三者是同一个坐标系，任意一个改动都会让已训权重失效）：

1. `DEFAULT_POSE` 注释明确是"**策略训练默认姿态**"（`sim2real_conv.h:13-19`、`sim2real_conv.cpp:9-12`）。
2. `obs[9..20] = pos_policy[:12] − DEFAULT_POSE[:12]`（`rl_controller.cpp:110-113`）——观测里唯一的关节位置项，
   **就是 q_urdf 减 `DEFAULT_POSE`**，没有别的归一化/缩放。
3. 动作：`q_target = DEFAULT_POSE[p] + action·ACTION_SCALE`（`rl_controller.h:132-137`）——**同一个 `DEFAULT_POSE`**。
4. 限位：`JOINT_LOWER/UPPER`（`rl_controller.cpp:13-26`）= `(−0.6,−0.7,−1.0) … (0.6,1.75,0.35)`，
   与 MJCF `range`（`dogurdf.xml:109/114/120`）**逐字相同** ⇒ 限位也在 URDF/MJCF 坐标。
5. 训练侧：`obs = concat(…, joint_pos − default_pose, …)`、`pos_target = clip(default_pose + action*0.25, …)`
   （`dogurdf_sim2sim_deploy/src/sim2sim.py:193-195, 233`），`default_pose` 由 `NOMINAL_*` 生成
   （`dogurdf_sim2sim_deploy/src/robots/dogurdf.py:44-46, 84-90`）。

⇒ **Z1/Z2 就是策略的度量衡。改 Z1（MJCF/URDF 零点）或 Z2（DEFAULT_POSE/限位/obs 语义）等于把策略换成另一个
问题，必须重训。** 这一点你的判断是对的。

### 3.3 "SW 得到一个更准的零点" → 改哪一层（决策流程）

**第一步（必做）：把 SW 的几何量换成 `q_urdf` 的期望值**，公式见 §5.1。得到一组 `q_SW`。

**第二步：在真机上量两个点，分离"斜率"和"零点"。** 单点只能定"零点"，双点才能定"斜率"（§5.2）。

| 实测结果 | 结论 | 改哪里 |
|---|---|---|
| 斜率 `Δq_SW/Δraw` 的**符号**与 `a = A·s` 不符 | 符号错（该关节的 `A` 或 `s`） | 改 **Z3 的 `CONV_A`**，并把同索引 `CONV_B` **同时取负**（保零位：只翻 A 会让零位偏 `2B`，`sim2real_conv.cpp:39`）。**不要动 `pos_scale`** |
| 斜率符号对，只有零点差 `e = q_meas − q_SW` | 纯零点差 | 改 **Z3 的 `CONV_B −= e`**（推荐，见下） |
| 斜率**幅值**也不对（不是 1:1） | 该电机编码器/减速比/协议量程问题 | 那是 `pos_scale` 之外的硬件层（`ele_motor_def.h:123-133`），**不属于本链**，先查固件寄存器 |

**为什么零点优先改 Z3 而不是 Z4：**

- Z3 只被"策略侧（URDF 空间）"使用：RL 观测/动作、Ex62/PACE、`plot_rlrun.py`/`pose_view.py`。
  改它**不会**动 `leg_kinematics` 的 FK/IK、不会动 `STAND_*`/`LIE_DOWN_*` 这些已被实测调通的姿态、
  不会动 status 空间的安全限位。而且 `urdf_to_status` 是 `status_to_urdf` 的**精确逆**（`sim2real_conv.h:33-43`），
  观测与动作同步移动，**不会产生正反馈**。
- Z4 是 **status 坐标的定义本身**：`robot_calibration.h:106-108` 直接把 `pos_offset` 变成
  `ZERO_OFFSET_THETA*`（`111/124/129`），进而 `THETA*_OFFSET`（`134-136`）进 `leg_kinematics` 的每个 FK/IK；
  `STAND_*`/`LIE_DOWN_*`/限位也全是 status/deg。改 Z4 = 悄悄重定义半个工程。
- 唯一的例外：**如果换过电机 / 重跑过 `SetZero`**，那 `raw` 的物理零点本身变了，此时必须改 Z4（`pos_offset`），
  并且**不要**再改 Z3（否则等于在错误的基上又叠一层补偿，`motor_calibration.h:50-54` 记录过这个坑）。

**改完 Z3 必须连带做**：重跑 `Example57`（§5.4）复核 → 重采 `Example62` → 重导 `.pt` → 重拟合 → 重看
`s2r` 落盘日志的坐标解释。**旧的 `log/pace/*.csv` 与 `.pt` 全部作废**（它们记录的是旧 `B` 下的 URDF 值）。

---

## 4. 改每一层分别会连带影响什么（清单）

| 层 | 已训策略（obs/action 语义） | `Example62` 采集 / 已录数据 | 上机限位 / 安全检查 | PACE 拟合数据 | sim2sim 部署 |
|---|---|---|---|---|---|
| **Z1** MJCF/URDF 零点（改关节 origin/range） | ❌**直接废掉**（策略的 `q` 不再是同一个物理姿态） | 语义变了 | MJCF range 变 ⇒ 与 `JOINT_LOWER/UPPER`（`rl_controller.cpp:13-26`）失配 | 仿真模型变，全部重做 | ❌`sim2sim.py` 直接用 `dogurdf.xml`，必须重训 |
| **Z2** `DEFAULT_POSE`/`ACTION_SCALE`/限位 | ❌**废掉**（obs 与动作的锚点/尺度变了） | Ex62 的 `target_urdf` 起点变（`ex_probe.cpp:365`），但 `base` 是**实测**的（`ex_probe.cpp:412-420`）⇒ 对已录 CSV **无影响** | `leg_pos_target` 的 clip 变（`rl_controller.h:132-137`） | 无（数据不变） | 无（sim2sim 用 `NOMINAL_*`，不改 C++ 常量） |
| **Z3** `CONV_A/CONV_B` | ⚠**只影响真机**：观测与动作同步平移/翻号 ⇒ 物理姿态改变，策略本身仍"自洽"，但**物理站高/姿态会按 `A·ΔB` 平移** | Ex62 的 `des`/`meas` **都**平移 ⇒ 旧 CSV 作废、需重采或按 `A·ΔB` 整体平移后重拟合 | URDF 限位不动，但 `status` 实际范围平移（`POS_LIMIT=2.5` 是 status，`ex_probe.cpp:317`） | **需要重拟合**（模型对绝对角非线性） | 无（sim2sim 不读 CONV） |
| **Z4** `pos_scale/pos_offset` | ⚠通过 status 平移/翻号影响（等价于 Z3 的一部分，但**同时**改 status 空间的一切） | 同上，且 `Example57/19/21/…` 的物理姿态全变 | `STAND_*`/`LIE_DOWN_*`/status 限位全部要重新验证 | 同上 | 无 |
| **Z5** 固件零点（`SetZero`） | 通过 Z4 整个平移 | 旧日志全部不可比 | 全部要重标 | 要重采 | 无 |

**工程里对 URDF 角有没有 clamp（安全检查盘点）**

- `rl::leg_pos_target()` 对 **URDF** 角 clamp 到 `JOINT_LOWER/UPPER`（`rl_controller.h:132-137`）——这是 RL 路径唯一的 URDF clamp。
- `Example62`**没有**对 URDF 目标 clamp；只有 status 硬保护 `POS_LIMIT=2.5 rad`（`ex_probe.cpp:317, 389, 406, 516`）
  与 deviation 保护 `DEV_LIMIT=0.5`（`ex_probe.cpp:318, 516`）。
- `MotionController::sendInterpFrame` **无 clamp**（`motion_controller.cpp:28-37`），
  `lieDown` 直接下发 status（`motion_controller.cpp:94-99`，`robot_calibration.h:162` 已承认越限）。
- 旧 IK 路径用 **status/deg** 限位 clamp（`ex_basic.cpp:256-262, 873-877, 1274-1276`，常量来自
  `robot_calibration.h:112-131`）。
- 固件协议量程 ±12.5 rad（`ele_motor_def.h:129-132`）对关节角**不构成有意义的保护**。
- 轮子另有 `WHEEL_ESTOP_VEL`/急停（`motor_manager.cpp:385-420`）。

---

## 5. 逐关节静态验证配方（可直接套用）

### 5.1 几何 → `q_urdf` 的公式（本文从 MJCF 反推，并用 `Example57` 的独立理论值交叉验证）

把腿投影到**腿自己的矢状面**（三个关节轴都平行于机体 y 轴），量三个角：

- `α`：髋外摆角，绕机体 x 轴，**+ = 腿向机体 +y（左侧）摆**（MJCF 四条腿都是 `axis="1 0 0"`，所以四条腿同号）。
- `a2`：**thigh 连杆**的俯仰角，从"正下"向"前(+x)"为正（`a2=0` 即大腿竖直）。
- `a3`：**膝→轮心向量**的俯仰角，同样从"正下"向前为正。
- `κ = a3 − a2`：膝弯曲角（`κ=0` 膝伸直；`κ=+90°` = 小腿⊥大腿且轮轴朝前）。

```
q_hip   = α
q_thigh = −( a2 + 1.0197 )                 # 1.0197 = thigh body 的固定 rpy（dogurdf.xml:112）
q_calf  = 1.7390 − κ                        # κ 用 rad；1.7390 = 膝伸直时的 q_calf
        = 1.7390 − (a3 − a2)
```

**验证**（本文计算，脚本与 MJCF 一致）：

- `q_thigh = −1.0197` ⇒ thigh 连杆恰好竖直（俯仰 0.00000）⇒ 与 `ex_diag.cpp:2745` 的 `thigh −1.020` 一致 ✅
- `q_thigh=−1.0197, q_calf=+0.168` ⇒ thigh 与 calf 三维夹角 **90.010°** ⇒ 与 `ex_diag.cpp:2745/2789` 的 `+0.168` 一致 ✅
- 任意姿态下 `q_calf = 1.7390 − κ` 的数值残差 < 3.2e−5 rad ✅
- `κ = 0`（膝完全伸直）⇒ `q_calf = 1.739`，**超出** joint range `[−1, 0.35]` ⇒ 与历史笔记
  "calf 的 θ₃=0 零点不可达"（commit `57cd63b`，`Example20_MoveToPhysicalZero` 的打印）一致 ✅

> **注意**：`leg_kinematics` 的 `θ` **不是**这个 `q`。`leg_fk` 把 "calf 连杆"当作一条长 0.343 的直线指向轮心
> （`robot_calibration.h:60-66`），丢掉了轮偏置里的 0.045 m 侧向分量与 0.317 rad 固定转角（**结构差**），
> 并且它用的零点（`pos_offset`）与 `CONV_B` 差 ~0.5 rad（**口径差**，§6.8）。
> 因此 **不能用 `leg_fk` 的 FK 去判定零点**（详见 §6.9）。

### 5.2 两点法：把"斜率"与"零点"分离（这是关键）

对每个关节（一次一条腿，吊起/悬空）：

1. 用 SW 几何算出姿态 A 的 `q_A`（§5.1）。
2. 把腿摆到 A，稳 2 s，读 `GetStatus`（= status）⇒ `status_A`；同时从 `MotorLogger` 的 `raw_pos` 列读 `raw_A`
   （`include/common/motor_logger.h:105`）。
3. 换姿态 B（建议与 A 差 ≥0.3 rad），同法得 `q_B, status_B, raw_B`。

```
斜率： a_meas = (q_B − q_A) / (raw_B − raw_A)      # 应 ≈ ±1；与 a = CONV_A·pos_scale 比
零点： b_meas = q_A − a_meas · raw_A               # 与 b = CONV_A·pos_offset + CONV_B 比
误差： e = b_meas − b    ⇒ 若只改 Z3：CONV_B_new = CONV_B − e
```

- `a_meas` 与 `a` **符号不同** ⇒ 改 `CONV_A[i] = −CONV_A[i]` 且 `CONV_B[i] = −CONV_B[i]`。
- 符号相同、只是 `b` 偏 `e` ⇒ `CONV_B[i] −= e`（因 `q = A·status + B`，把 `B` 减 `e` 后同一位姿的
  `q_meas` 就与 `q_SW` 重合）。

### 5.3 数值示例（用现有 `CONV_*`/`pos_*`，直接抄）

**姿态 A：thigh 竖直、膝 90°、hip 水平** ⇒ `q_SW = (0, −1.020, +0.168)`：

| 量 | hip | thigh | calf |
|---|---|---|---|
| `q_urdf`（目标） | 0.000 | −1.020 | +0.168 |
| `status = (q−B)/A` | (0−0.0297)/+1 = **−0.0297** | (−1.020+0.9624)/(−1) = **+0.0576** | (0.168+1.2832)/+1 = **+1.4512** |
| `raw = (status−o)/s`，FL | (−0.0297−0.611)/(−1) = **+0.6407** | (0.0576−0.441)/+1 = **−0.3834** | (1.4512−0.211)/(−1) = **−1.2402** |
| `raw`，FR | (−0.0297−0.611)/(+1) = **−0.6407** | (0.0576−0.441)/(−1) = **+0.3834** | (1.4512−0.211)/(+1) = **+1.2402** |

**姿态 B（示例）：`a2 = −0.3`（大腿后仰 17.2°）、`κ = 1.20`、`α = 0`**：

```
q_thigh = −(−0.3 + 1.0197) = −0.7197      q_calf = 1.7390 − 1.20 = +0.5390
status_thigh = (−0.7197 + 0.9624)/(−1) = −0.2427      status_calf = 0.5390 + 1.2832 = +1.8222
raw_FL: thigh = (−0.2427 − 0.441)/+1 = −0.6837        calf = (1.8222 − 0.211)/(−1) = −1.6112
```

**斜率自检**：A→B 的 `Δq` = (+0.3003, +0.3710)，`Δstatus` = (+0.3003, +0.3710)（因 `A=±1`），
`Δraw_FL` = (−0.3003, −0.3710) ⇒ `a_meas = −1` = `A·s` ✅（FL/RL 应为 −1，FR/RR 为 +1）。

### 5.4 现成工具：`Example57_SingleLegZeroAlign`

`src/app/examples/ex_diag.cpp:2756-2865` 已经把上面的流程做完了（含"第二点验证 A 斜率"`2842-2857`），
理论值表在 `2788-2790`。**建议直接用它 + §5.1 的公式，对四条腿各跑一遍、记下 `e`**，
而不是先改代码。注释里 2026-09-12 的单腿实测残差是 hip −1.9° / thigh +3.7° / calf +4.9°
（`ex_diag.cpp:2752-2754`）——这是**全仓库唯一的现场零位实测记录**，量级只有 0.03~0.09 rad
（见 §6.11 的用法与其循环性警告）。

⚠ 但要注意它的判据是"读出的 urdf 与理论 urdf 之差"，而理论值是由 **`status=(0,0,π/2)` 这个指令**推出的，
所以它同时也把 `CONV_B` 的误差算进去了：`status=(0,0,π/2)` 按 CONV 对应的 `q` 是 `(0.0297, −0.9624, +0.2876)`，
与理论 `(0, −1.020, +0.168)` 差 `(+0.0297, +0.0576, +0.1196)` ⇒ 那 3~7° 的"残差"其实是**两种零点定义的差**，
不是独立的第三方校准。**真正独立的判据是操作者肉眼/直尺确认"连杆真的竖直/垂直了"**。

---

## 6. 不确定 / 不自洽清单（重点）

### 6.1 `CONV_B` 的注释是不是"L 形目测、待验证"？——**是，但逐字原文与你的印象略有出入**

`include/strategy/sim2real_conv.h:21-28` 逐字：

```
// 真机 GetStatus 角 ↔ URDF 角每关节映射（2026-08-20 真机 L 形测量）：
//   URDF = CONV_A[joint]*GetStatus + CONV_B[joint]（rad）
//   hip   默认 A=+1 B=+0.0297（**例外：FR hip 为 A=-1 B=-0.0297**，2026-10-13 现场确认
//         观测方向反了，修法=整个映射取反以保零位，见 sim2real_conv.cpp 注释）
//   thigh A=-1 B=-0.9624   calf A=+1 B=-1.2832   wheel A=+1 B=0
// ⚠ 基于一次 L 形目测（08-20）；**hip 的符号还叠加了电机层 pos_scale 的对角模式**
//   （FL −1 / FR +1 / RL +1 / RR −1，自 57cd63b 起未复核），FR 已于 10-13 现场纠正，
//   RL 待现场确认。判据：腿远端向机器人左侧推 ⇒ URDF 读数应变大。
```

⇒ "**基于一次 L 形目测（08-20）**"确实在（`sim2real_conv.h:26`），"**待验证**"这个字眼**没有**，原文是
"**未复核**/待现场确认"（`sim2real_conv.h:27-28`）。三点补充：

1. 三个 `CONV_B` 数值**没有任何推导记录**（没有 SW 坐标、没有姿态复现说明）⇒【未找到】来源。
   `Example57` 只做了"和它自己一致"的复核（§5.4）。
2. 头文件说"**例外：FR hip**"（单数），但 `sim2real_conv.cpp:47/53` 实际是 **FR 和 RR 都翻**（2026-10-17）。
3. `sim2real_conv.cpp:26-28` 的注释写"**已回退为 A=+1, B=+0.0297**"，而 `sim2real_conv.cpp:48/53` 的数组
   是 `−1 / −0.0297`。**同一文件内注释与代码互斥**。以数组为准（`git log` 显示 `32bdc48` 把 FR+RR 一起翻了，
   `5f8a4dc` 只翻 FR）。⇒ **头文件注释 + cpp 前 44 行的叙述都已经过期**。

### 6.2 `pos_offset`（0.611/0.441/0.211/0.0）属于哪一层、单位、怎么来的

- **层**：raw↔status（Z4），`status = raw·pos_scale + pos_offset`（`motor_calibration.h:184`）。
  注意它是**乘完 `pos_scale` 之后**才加的 ⇒ 它的坐标系是 **status**，**不是 raw**。
- **单位**：rad（`docs/MOTION_RL_KNOWLEDGE.md:153` 明写"髋 0.611、大腿 0.441、小腿 0.211（rad）"；
  `robot_calibration.h:111/124/129` 又把它转成 deg 并当 `ZERO_OFFSET`）。
  代码里 `pos_offset` 字段本身**没有单位注释**（`motor_calibration.h:19` 只有 "// 位置偏移值"），
  `MOTOR_CALIBRATION` 表里 0.611/0.441/0.211 三行也**没有解释**（`motor_calibration.h:43-45` 只有 "// Motor 1 (Hip)"）。
- **来源**：`git log` 追到 **commit `57cd63b`（2026-06-13）"补充物理0位，example20待测试"**，同一次提交里的
  `Example20_MoveToPhysicalZero` 打印给出了当时的**意图**：

  ```
  Motor 1 (Hip):   θ₁=0    → 0.611 rad   horizontal
  Motor 2 (Thigh): θ₂=0    → 0.441 rad   vertical
  Motor 3 (Calf):  θ₃=90°  → ...         90° (zero unreachable)
  ```

  即"hip 连杆水平 / thigh 连杆竖直 / calf 的 0 不可达"。**这是人手动摆位 + 读数的结果，没有任何文件记录**。
- **calf 的符号被改过一次且无说明**：`57cd63b` 引入时是 **−0.211**，两天后 **`8afc2cf`（2026-06-15
  "姿态解算完成"）改成 +0.211**，diff 里没有任何理由。⇒ **calf 的 `pos_offset` 是这条链上唯一被静默翻过号的常数**。
- **符号约定四处不一致**（这是真·不自洽，且有具体数值后果）：
  1. 电机层：`status = raw·s + o`（加）⇒ `raw = 0` 时 `status = o`；
  2. `robot_calibration.h:20-21`：`物理角 = ZERO_OFFSET + 指令角`（加）⇒ `raw = 0` 时"物理角" = `2o`
     （hip 2×0.611 = 1.222 rad = 70°，thigh 0.882，calf 0.422）；
  3. `leg_kinematics.h:48-50`：`t1=q1+off1, t2=q2+off2, t3=q3+off3`（加，与 2 一致）；
  4. 而 `57cd63b` 的 `Example20` 打印的是"θ=0 对应指令 = +0.441"（这要求 `t = q − off`，**减**）。
  ⇒ (1)(2)(3) 与 (4) 互斥。若 (4) 是当初的意图（"θ=0 时连杆竖直"），则 (2)(3) 的符号用反了，
  零点被挪了 **2·offset**；反之若 (2)(3) 对，则 (4) 只是打印措辞错了。

### 6.3 MJCF 的关节零点是否与 URDF 零点**严格相同**？——**是**（文件级可证），但"URDF 坐标 = 真机坐标"不可证

**能证明的（强证据）**：

1. `dogurdf.xml` 是从 `urdf/dogurdf.urdf` 手工搬过去的，关节段**逐项相同**：
   `pos="0 0 0"`、`axis`、`range` 三项在 16 个关节上完全一致；body 的 `pos`/`quat` 与 URDF 的
   `origin xyz/rpy` 一一对应（例：URDF `rpy="0 1.0197 0"` ↔ MJCF `quat="0.872818 0 0.488046 0"`；
   URDF `rpy="3.1416 -1.0861 -3.1416"` ↔ MJCF `quat="0.51675 0 −0.856137 0"`）。
2. `dogurdf_from_urdf.xml`（自动转换版）与 `dogurdf.xml` 的 16 个关节 `name/pos/axis/range` **完全一致**
   （本文用脚本逐行比对），差别**只**在：`torso` 没有 body/freejoint/actuator/ground
   （`dogurdf.xml:4-10` 自己写了这点）、`actuatorfrcrange`（300 vs 150/53）、`frictionloss/armature`
   （`dogurdf.xml:41-48` 是后加的 PACE 物理基线）、damping（`dogurdf.xml:51`）。
   ⇒ **零点没有差任何常数**。
3. `range` 的不对称（hip ±0.6、thigh −0.7…+1.75、calf −1.0…+0.35）在 URDF 与两个 MJCF 里**完全一样**，
   所以"用 range 不对称来推断两者差常数"这条路**走不通**（它只能告诉你 SW 的零位不是行程中点）。

**证不了的**：`q_urdf`（`status_to_urdf` 的输出）是否**真的**等于 `q_mjcf`，取决于 `CONV_B` 那次 L 形目测。
这一步**没有第三方证据**（§5.4 的 Ex57 是自证）。

**怎么用现有数据判断"是否差一个常数"（可行方案）**：

- **几何自检**：用 §5.1 的公式算 `q_SW`，与 `status_to_urdf(GetStatus)` 比（两点法，§5.2）。这是唯一能定案的。
- **交叉量**：`leg_kinematics` 的"物理角"与 `CONV_B` 两套零点在同一姿态下的差（本文算出来）：
  hip ≈ **0.581 rad**、thigh ≈ **0.498 rad**、calf **不只是常数差（斜率也不一致）**。
  见 §6.10 的推导，这是本项目里最值得追的一条线索。

### 6.4 `DEFAULT_POSE` —— **你记的值已经过期，现值为 `0 / −0.03 / −0.116`**

- 当前代码：`sim2real_conv.cpp:13-21` = `hip 0.0、thigh −0.03、calf −0.116、wheel 0`（四腿相同）。
  注释明确写了"✅ 2026-10-17：本组数值**已由训练侧真值核实**，取代此前的 thigh=0.20 / calf=−0.35"
  （`sim2real_conv.cpp:16-18`）。你记的 `+0.20 / −0.35` 就是**旧值**。
- **坐标**：URDF/策略坐标（`sim2real_conv.h:13-19`、`sim2real_conv.cpp:9-12`），经 `urdf_to_status` 下发
  （`sim2real_conv.cpp:11-12` 明确"不要在此手填真机标定角"）。
- **与 MJCF 的"站立姿态"是否一致？—— 本仓库内无法确认**：
  - `dogurdf_sim2sim_deploy/src/robots/dogurdf.py:44-46` 仍是 `NOMINAL_HIP=0 / THIGH=0.20 / CALF=−0.35`，
    `NOMINAL_TORSO_HEIGHT=0.45`；
  - 同一目录还有第二份副本 `dogurdf_sim2sim_deploy/src/dogurdf.py:43-46` = `0 / 0.20 / −0.53`，`TORSO=0.40`
    （**两份副本互相矛盾**）；
  - `dogurdf.xml:25-26` 的注释还写着 "Nominal stance is thigh=0.20 calf=-0.70"（第三个数）。
  - C++ 注释引用的"`RL_Train/code/src/robots/dogurdf.py`"**不在本仓库**【未找到】，所以 `−0.03/−0.116`
    在本仓库内**没有任何代码可以复核**。`memory/FACT.md:56-62` 记录了这个矛盾并算过：calf 差 0.18 rad
    ⇒ obs 的 4 个小腿通道会带恒定偏置。
- **`0.52 m` 站高可复现**：用 MJCF 正解，`(0,−0.03,−0.116)` 时髋→轮心竖直 = **0.407 m** ⇒ 机体高
  `0.407+0.113 = 0.520 m` ✅ **正好是"0.52 m 站高标定"**；旧值 `(0, +0.20, −0.35)` 给 **0.336 m**
  ⇒ 机体低 **71 mm**（`sim2real_conv.cpp:18` 写"约 50 mm"，量级对，数值偏小 20 mm）。

### 6.5 `Example62` 的取数与坐标（确认无隐藏换算）

- `target_urdf[i] = DEFAULT_POSE[i]`、`start_urdf = status_to_urdf(GetStatus)`（`ex_probe.cpp:361-368`）。
- 插值/静置都用 `urdf_to_status` 下发（`ex_probe.cpp:384, 402`）。
- **`base` 取静置后的实测值**（`ex_probe.cpp:412-420`），**不是** `DEFAULT_POSE` ⇒ `DEFAULT_POSE` 的错值
  **不会**污染 chirp 的中心，只影响"从哪开始摆"。
- 主循环：`des = base + AMP·sin`（URDF）→ `urdf_to_status` 下发；`meas = status_to_urdf(GetStatus)`（URDF）
  写 CSV（`ex_probe.cpp:496-517`）。⇒ **`des` 与 `meas` 在同一坐标、同拍记录**，CSV 里不存在跨坐标拼接。
- 关节顺序：`jn[] = LF_HAA/HFE/KFE, RF…, LH…, RH…`（`ex_probe.cpp:443-444`）= 按腿分组，与 CAN 序**一一对应**，
  与 PACE 的 `joint_order` 一致；导出脚本 `tool/pace_export_dataset.py` 只做时间基校验，**不做任何坐标换算**
  （`tool/pace_export_dataset.py:60-80`）。
- ⇒ **PACE 拿到的 `dof_pos/des_dof_pos` 就是本链的 `q_urdf`，一个纯 `CONV_B` 的零点错会 1:1 出现在拟合的
  `bias` 里；但反过来，`bias` 无法把误差定位到 `CONV_B` 还是 `pos_offset`**（两者在 CSV 里被加成了同一个常数）。

### 6.6 限位体系不自洽（**有安全影响**）

| 关节 | URDF/MJCF range（`dogurdf.xml:109/114/120`、`rl_controller.cpp:13-26`） | 折成 status（`(q−B)/A`） | `robot_calibration.h` 的 status 限位（`112-131`） |
|---|---|---|---|
| hip | −0.6 … +0.6 | −0.630…+0.570（FL/RL）／−0.570…+0.630（FR/RR） | **−60° … +15°**（= −1.047 … +0.262） |
| thigh | −0.7 … +1.75 | −0.262 … −2.712 | **−70° … +90°**（= −1.222 … +1.571） |
| calf | −1.0 … +0.35 | +0.283 … +1.633 | **+60° … +180°**（= +1.047 … +3.142） |

⇒ 三对区间**都不重合得很干净**；尤其 calf：`robot_calibration` 允许到 180°（status 3.142 = `q_calf` **+1.857**），
而 CAD/URDF 的上限是 **+0.35**。若 CAD 的 range 是真的，则 status 允许的上半段会让膝**反向过伸**（`κ<0`）。
**两套限位必须择一为准（建议以 CAD range 为准），否则 `example`/IK 里的 clamp 挡不住真正的越界。**

另外：`JOINT_LOWER/UPPER`（URDF，`rl_controller.cpp:13-26`）与 MJCF range 完全一致 ✅，这一套是自洽的。

### 6.7 其它过期注释（会误导改代码的人）

- `leg_kinematics.h:295`："DEFAULT_POSE (0,0.20,−0.35) cond ≈ 15" —— **旧值**。
- `ex_probe.cpp:317`："thigh 在 DEFAULT_POSE 就有 ~−1.16" —— 旧值下才对（新值 status = −0.9324）。
- `ex_rl.cpp:883`："该转换转出 thigh −66.6°/calf 53.5°" —— 旧值下才对（新值 −53.4°/66.9°）。
- `docs/SIGN_MAP.md:45,107-117`：净符号表写"hip = −1/+1/+1/−1 对角同号，唯一与其他关节不同"，
  那是 `CONV_A[hip]` 全 +1 时的结果；**当前代码 FR/RR 已翻号**，净符号变成 **−1/−1/+1/+1**（前负后正）。
- `docs/MOTION_RL_KNOWLEDGE.md:138-141` 的标定表把 `vel_scale` 全写成 +1 —— 早已改成 `= pos_scale`。
- `sim2real_conv.h:23-24` 仍写"例外：FR hip"，漏了 RR（§6.1）。

### 6.8 `leg_kinematics` 用的是哪个坐标系？—— **status（指令角）坐标，且与 `CONV` 坐标不自洽**

- 证据：`robot_calibration.h:106-108` 直接令 `HIP/THIGH/CALF_POS_OFFSET_RAD = MOTOR_CALIBRATION[0][*].pos_offset`，
  `111/124/129` 转成 `ZERO_OFFSET_THETA*_DEG`，`134-136` 转成 `THETA*_OFFSET`，
  然后在 `leg_fk/leg_ik/leg_jacobian` 里用（`leg_kinematics.h:48-50, 126-128, 213-215`）。
  `leg_kinematics.h:38` 说输入是"指令角"，`ex_sysid.cpp:359` 明写"真机指令角坐标 = leg_fk/GetStatus 同一坐标"。
  ⇒ **`leg_kinematics` 吃的是 `GetStatus` 的 status**，不是 URDF。
- **但这套"物理角"与 `CONV` 的 URDF 坐标不是同一个东西**（本文推导，见 §6.9/§6.10）：把 `leg_fk` 的两条
  连杆对应到 MJCF 的 thigh 连杆与"膝→轮心"向量后，可得

  ```
  a2（thigh 连杆俯仰） = status2 + 0.441        ⇒  q_urdf(thigh) = −status2 − 1.4607
  κ（膝弯）           = status3 + 0.211        ⇒  q_urdf(calf)  = 1.7390 − status3 − 0.211 = 1.5280 − status3
  a1（外摆）           = status1 + 0.611        ⇒  q_urdf(hip)   = ±(status1 + 0.611)  (左 + / 右 −)
  ```

  与 `CONV` 对比：

  | 关节 | `leg_kinematics` 口径 | `CONV` 口径 | 差 |
  |---|---|---|---|
  | hip | `±(status + 0.611)` | `+status + 0.0297`（FL/RL）／`−status − 0.0297`（FR/RR） | **斜率一致 ✅**，零点差 **0.581 rad** |
  | thigh | `−status − 1.4607` | `−status − 0.9624` | **斜率一致 ✅**，零点差 **0.498 rad** |
  | calf | `1.5280 − status` | `+status − 1.2832` | **斜率相反 ❌**：两式在 `status≈1.405` 处交叉，工作点附近一致（L 形姿态 `status=1.4512` 时两口径只差 **0.091 rad ≈ 5°**，正好是 Ex57 的残差量级），但**偏离工作点 ±0.25 rad 就拉开 0.5 rad** |

  ⇒ **两套零点定义在 hip/thigh 上差 ≈0.5 rad，在 calf 上连斜率都相反。** 这是本工程里最硬的一条"不自洽"。
  注意：calf 的斜率差**不会**被 RL 暴露（工作点附近两口径几乎重合），但它意味着**同一 status 值在
  `leg_kinematics` 与 `CONV` 下会给出差 0.5 rad 的 calf 角**——这正是"0.5 rad"这个数字在本工程里出现的地方（§6.11）。
- 哪一套可信？**RL 那一套（`CONV`）更可信**，理由是闭环论证：若 calf 的 `A` 真该是 −1（`leg_kinematics` 口径），
  那么策略的观测与动作在 calf 上都反号 ⇒ 正反馈 ⇒ 策略不可能走。**策略能走 ⇒ `CONV_A[calf]=+1` 是对的**
  ⇒ 反推 `leg_kinematics` 的 calf 零点/符号口径是错的（或至少与 RL 无关且过时）。
  【推断】`leg_kinematics` 那套是 2026-06 的老框架遗留，只被 Ex18/19/21/47/54 之类旧示例使用，
  没有被 URDF/RL 这条链校验过。

### 6.9 为什么**不能**用 `leg_fk` 的 FK 判定零点（重要限制）

`leg_kinematics` 的腿模型与 MJCF 有**三处结构差异**：

1. 第三条"连杆"长度用 0.343 = `|轮偏置|`（`robot_calibration.h:60-66`），但真实膝→轮心向量有 **0.045 m 侧向分量**
   （`dogurdf.xml:125`），平面模型无法表达；
2. 轮偏置在 calf 坐标系里还有 **0.317 rad 的固定转角**（`atan2(−0.10583, 0.32311)`），平面模型把它并进了
   "calf 连杆方向"；
3. 因此 `leg_fk` 的 `θ₃` 是"膝→轮心方向相对 thigh 的方向"，**不是** calf 关节的机械角
   （两者差一个 ≈0.317 rad 的常数 + 上面 §6.8 的口径差）。

**两个模型的实测差异（本文计算，DEFAULT_POSE）**：

| 量 | `leg_fk`（喂 status `−0.0297,−0.9324,+1.1672`） | MJCF（喂 `q_urdf = 0,−0.03,−0.116`） |
|---|---|---|
| 轮心（机体系，FL） | `(0.2211, **0.4729**, −0.3598)` | `(0.3011, **0.2558**, −0.4071)` |
| 差值 | **0.236 m**（侧向差 0.217 m ≈ 35° 外摆 = `pos_offset` 0.611 的口径差；前后差 0.08 m ≈ thigh 0.498 rad；上下 0.047 m） | — |

⇒ 把同一组"名义站姿"喂给两个模型，轮心差 **0.24 m**。这不是几何误差，而是**口径（datum）误差**
（§6.8 的 hip 0.581 / thigh 0.498 rad）。**结论：判断零点只能用 §5.1 的解析公式或 MJCF 直接正解，
不能用 `leg_fk` 的 FK**；也不要拿 `leg_fk` 算出来的足端位置去标定零点。

### 6.10 `pos_offset` 的"物理零点"是否在 MJCF 里成立？（交叉检验，结果：不成立）

用 §5.1 的公式 + `pos_offset` 定义的"θ=0 时 thigh 竖直 / calf 90°"来检验：

- "thigh 竖直"要求 `a2 = 0`，而 `a2 = status2 + 0.441` ⇒ 需要在 `status2 = −0.441`；但历史 `Example20`
  打印的是"θ₂=0 → 0.441 rad"，即 `status2 = +0.441`。**符号相反**（这与 §6.2 的第 4 条是同一个病）。
- "calf 的 θ₃=0"在 `leg_fk` 里对应 `status3 = −0.211`，在 `CONV` 里对应 `q_calf = −0.211−1.2832 = −1.494`
  —— **超出** URDF range `[−1, 0.35]`。⇒ 两套口径对"calf 零点"的定义差得太多，**不可能同时成立**。

### 6.11 关键矛盾：PACE 的 calf `bias` 顶在 −0.5 —— 分析与判断

**先把事实钉住**：

1. `bias` 是在 `q_urdf` 坐标里被拟合的（`Example62` 的 CSV 与 `pace_export_dataset.py` 都是 `q_urdf`，
   §6.5），且 PACE 的用法是 `q_sim = q_meas + bias`（`docs/PACE_PHASE0_PARITY.md:317`）。
2. `q_urdf = A·(s·raw + o) + B` ⇒ **任何一层**（Z3 的 `B`、Z4 的 `o`、Z5 的 raw 零点）的常数误差都以 **1:1**
   进 `bias`，**无法从拟合值反推是哪一层**。
3. **"共模"这个特征能排除什么**：四条腿同号同值（std 3e−4）说明这是**四腿共用/装配级**的常数，
   **不是**每腿的加工/装配散差（那种会有 0.01~0.05 rad 量级的散布）。
   但它**不能**区分 Z3 / Z4 / Z5——因为"四条腿都差同一个物理角"在 `q_urdf` 上就是共模的，
   无论这个常数是写在 `CONV_B`、`pos_offset` 还是机械/编码器零点里。
   ⇒ **PACE 的数据无法定位层，只能定位"这是坐标级常数"这个性质。**
4. **量级检验（反对"真差 0.5 rad"，这是本文最有力的定量论据）**：用 MJCF 正解算站高对 calf 角的灵敏度
   `d(wheel_z)/d(q_calf) = −0.2588 m/rad`（在本工程 DEFAULT_POSE 处）。⇒ 0.5 rad 的 calf 零点错
   = **站高差 0.129 m**（而且 `joint_pos_rel` 的 calf 通道会整体偏 0.5，是 `ACTION_SCALE=0.25` 的**两倍**）。
   0.13 m 的站高误差在 0.52 m 的机身上是 25%，且腿型（膝弯 106°→135°）肉眼可见
   ⇒ **与"策略可用、且用户一直在调站高（0.347→0.45→0.52）"这一事实很难共存**。
5. **"策略可用"能排除什么、不能排除什么**：
   - **能排除符号错**：任一关节的 `A·s` 反号 ⇒ 观测/动作在该关节上互斥 ⇒ 正反馈 ⇒ 立刻失稳。策略能走 ⇒
     **整条链的符号（含 calf）是对的**。
   - **不能排除常数错**：常数错只让"物理姿态"相对"策略意图"整体平移，闭环仍然自洽（观测看到的仍是
     指令值），所以能走。但它**会改变实际站高/腿型**（第 4 点）。
   - ⇒ 所以"能走"**不能**证明零点没错；但它 + 第 4 点合起来，把"真的差 0.5 rad"压到很低概率。
6. **`bias` 顶在边界本身是个信号**：`std = 0.0003`、四条腿全部压在同一根轨道上，这更像
   **"该参数在本次数据里不可辨识/被别处的更大误差顶到边界"**，而不是一个收敛的估计。
   尤其 `des` 与 `meas` 是同一坐标、同拍记录（§6.5），而 `kp=250`（`ex_probe.cpp:314`）意味着
   0.5 rad 的位置不一致会在仿真里造出 **125 N·m** 的 PD 力矩（真机只有几 N·m）——若有真 0.5 rad 的差，
   **残差会大到拟合根本不会选它**，除非别处有等量级的模型误差在抵消。

**我的判断（明确）**：

- **这不像"膝关节零点在某一层差了 0.5 rad"**。更像下面三种之一（按可能性排序）：
  1. **辨识模型的关节零点/默认姿态与部署的 `q_urdf` 不是同一个**（本仓库里 `NOMINAL_CALF` 就有 `−0.35`
     和 `−0.53` 两个值、MJCF 注释还写 `−0.70`，见 §6.4）⇒ 差一个常数是"设计如此"，不是硬件错；
  2. `bias` 在本次数据上**不可辨识**（fixed-base + 腿悬空 + 全关节同相 chirp ⇒ calf 的绝对零点与
     膝部重力矩/惯量强耦合，可被 armature/inertia 的误差顶走），顶在边界；
  3. 确实存在共模常数误差，但量级远小于 0.5（0.05~0.1 这种量级），边界值只是"最优解在可行域外"的表现。
- **不要因为 `bias` 就改任何一层。** 先做四件便宜的事：
  a) 把 `bias` 的**代价曲线**打出来（固定 bias 扫描、其余参数重拟合）——若代价在 `[−0.5,−0.3]` 平底，
     就是不可辨识，不是零点；若尖锐指 −0.5 外侧，则说明有更大的模型误差在。
  b) 检查另外 11 个 `bias`（尤其 **thigh**）：所有关节串联，thigh 的坐标/惯量误差会"漏"进 calf 的 `bias`
     （calf 的绝对角 = thigh + calf）；若只有 calf 一条腿式地顶边，才更像 calf 自己的常数。
  c) 确认 PACE 侧用的模型确实是 `dogurdf.xml`（而不是 ANYmal 或别的资产）——用错资产的零点必然是任意的。
  d) 确认 PACE 侧/任何预处理**没有**用 `leg_kinematics` 口径的"物理角"（§6.8：那套与 `q_urdf` 在 calf 上
     差一个斜率，工作点附近还看不出来，偏 0.25 rad 就差 0.5 rad）。本仓库的 Python 工具都从
     `sim2real_conv.cpp` 现读 `CONV_A/B`（`tool/plot_rlrun.py:20-50`），数据链本身是干净的
     ——所以要查的是**仓库外**的那部分脚本。
- 真要定案，只能上台架：**§5.2 的两点法 + `Example57`**，四条腿各一遍。这也是唯一能区分
  "Z3 的 `B` 错"与"Z4 的 `o` 错"的办法（单点不能，必须靠斜率）。

### 6.12 其它【未找到】

- `CONV_B`（`0.0297 / −0.9624 / −1.2832`）的**推导过程/原始测量记录**：无。
- `DEFAULT_POSE` 引用的 `RL_Train/code/src/robots/dogurdf.py`（0.52 m 站高标定）：本仓库内无此文件。
- `pos_offset` 的原始测量（谁在什么姿态下读的）：无，只有 commit message 与当时的 `Example20` 打印。
- 任何"真机 `raw` 与 CAD 角的对照表"：**全仓库没有**（`log/`、`tool/`、`docs/` 都没有）。

---

## 7. 附录 A：上台架前的 7 步核对清单

1. 吊起/腿悬空；`Example57` 选一条腿，记 `status` 与 `raw_pos`（`raw_pos` 在 recv CSV）。
2. 用 §5.1 把"肉眼确认的几何姿态"算成 `q_SW`。
3. 按 §5.2 算 `a_meas`、`b_meas`；与 `a = CONV_A·pos_scale`、`b = CONV_A·pos_offset + CONV_B` 比。
4. 重复姿态 B，确认 `a_meas` 稳定（这步最重要，单点无意义）。
5. 四条腿各做一遍，看 `b_meas − b` 是否**共模**（共模 ⇒ 层间常数；镜像 ⇒ raw/pos_scale 层）。
6. 只在这个 `e` 稳定且 > 0.05 rad 时才动 `CONV_B`（§3.3），改前先 `git stash`/分支。
7. 改后：Ex57 复核 → 重采 Ex62 → 重导 `.pt` → 重拟合 → 对比新旧 `bias`（应显著变小/离开边界）。

## 8. 附录 B：一句话索引（按"要查什么"）

| 想查 | 去哪 |
|---|---|
| 我一个关节的 `q_urdf` 对应多少 raw | `q = a·raw + b`，§2.2 表 |
| `GetStatus` 是原始值还是标定值 | 标定值（`motor_manager.cpp:343` + `ele_motor.cpp:288-289`） |
| 原始值在哪看 | `MotorLogger` 的 `raw_pos`（`motor_logger.h:105`）；`[PARAM]` 打印（`ele_motor.cpp:262-265`） |
| 标定在收还是发 | 收：`unpack_frame`（`ele_motor.cpp:288/298/309/350`）；发：`SendOnce`（`motor_manager.cpp:453`） |
| 策略坐标的锚点 | `DEFAULT_POSE`（`sim2real_conv.cpp:13-21`）+ `ACTION_SCALE`（`rl_controller.h:31`）+ obs（`rl_controller.cpp:110-123`） |
| 谁在用 status 坐标 | `robot_calibration.h:106-108`、`leg_kinematics.h:48-50`、`STAND_*/LIE_DOWN_*`、status 限位 |
| 改符号要连带改什么 | `CONV_A` 与 `CONV_B` **同时取负**；`pos_scale` 与 `vel_scale` **必须同号** |
| PACE 数据是什么坐标 | `q_urdf`（`ex_probe.cpp:496-517` + `tool/pace_export_dataset.py`） |
