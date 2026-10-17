# 符号地图：工程里所有会改变「关节量正负」的位置

> 2026-10-17 整理。起因：FR hip 的符号在 10-13 和 10-17 两次现场判断中**给出了相反结论**，
> 说明"符号"这条链上有多个层，光凭"看上去反了"很容易改错层/改错关节。
> 本文把**所有**会翻转符号或改变量纲的位置列全，并给出"哪一层该用什么方法验证"。

---

## 一、链路总览（编码器 → 策略）

```
                      ┌──────────────── ① 电机层（物理编码器方向）────────────────┐
 固件 raw  ──pos_scale/pos_offset──►  status  ──CONV_A/CONV_B──►  URDF（POLICY 序）
   ▲                                   │                              │
   │                                   │                              ├─► 策略观测 obs
   │                                   │                              └─► 渲染/落盘/工具
   └──── ApplyMotorCalibrationInverse ─┴─ 反向：URDF 目标 → status → raw（下发）
```

**净符号（位置）= `MOTOR_CALIBRATION[pos_scale]` × `CONV_A`**

- 速度：`vel_scale`（**必须等于 pos_scale**）× `CONV_A`
- 力矩：`pos_scale`（`ApplyMotorCalibrationTorque` 用它）× `CONV_A`（仅**记录**时乘）
- 加速度：工程里没有加速度通道（不涉及）

---

## 二、逐层清单

### ① 电机层（L2/L3）—— 物理编码器方向

| 位置 | 作用 | 说明 |
|---|---|---|
| `include/motor/motor_calibration.h` 的 `MOTOR_CALIBRATION[4][4]` | `pos_scale` / `vel_scale` / `pos_offset` | **每电机**一组，共 16 组；这是"物理事实"层 |
| `ApplyMotorCalibrationPos()`（同文件） | `pos = pos*pos_scale + pos_offset` | 接收方向 |
| `ApplyMotorCalibrationVel()` | `vel = vel * vel_scale` | ⚠ **vel_scale 必须 = pos_scale**（2026-09-04 事故） |
| `ApplyMotorCalibrationTorque()` | `torque = torque * pos_scale` | 力矩沿用位置符号 |
| `ApplyMotorCalibrationInverse()` | `pos = (pos − offset) * pos_scale`；`torque *= pos_scale` | 下发方向（含 `tau_ff`） |
| `src/motor/ele_motor.cpp` `unpack_frame()` | 调上面三个 | 参数回帧只标定"被更新的那个字段"；周期回帧整体标定 |

**当前取值（务必与 `pos_scale` 对照）**

| 关节 | pos_scale（FL / FR / RL / RR） | 模式 |
|---|---|---|
| hip | **−1 / +1 / +1 / −1** | 对角同号 |
| thigh | +1 / −1 / +1 / −1 | 左右镜像 |
| calf | −1 / +1 / −1 / +1 | 左右镜像 |
| wheel | −1 / +1 / −1 / +1 | 左右镜像 |

### ② 策略层（L6）—— status ↔ URDF

| 位置 | 作用 |
|---|---|
| `src/strategy/sim2real_conv.cpp` 的 `CONV_A[16]` / `CONV_B[16]` | 每关节 ±1 符号 + 偏移（共 16 个关节位） |
| `status_to_urdf()`（`sim2real_conv.h`） | `urdf = A*status + B` —— **观测**方向 |
| `status_vel_to_urdf()` | `vel_urdf = A*vel_status` |
| `urdf_to_status()` | `status = (urdf − B)/A` —— **动作**方向（与上两条精确互逆） |
| `DEFAULT_POSE[16]` | 名义站姿；`joint_pos_rel = q − DEFAULT_POSE`（是**偏移**，不是符号） |

**当前取值**：`CONV_A` 腿全部 `hip +1 / thigh −1 / calf +1`，轮 `+1`；`CONV_B` 腿
`hip +0.0297 / thigh −0.9624 / calf −1.2832`，轮 `0`。

> 2026-10-13 曾把 FR hip 改成 `A=−1, B=−0.0297`，10-17 现场复核后**已回退为 +1/+0.0297**
> （A、B 同时翻才能保零位；只翻 A 会让零位偏 2×B = 3.4°）。

### ③ 力矩记录的符号（仅落盘/工具）

| 位置 | 作用 |
|---|---|
| `src/app/examples/ex_rl.cpp:1101`、`:2192`、`src/motion/motion_controller.cpp:196` | `tau_policy[i] = CONV_A[i] * tau_can[POLICY_TO_CAN[i]]` —— 把 status 坐标力矩转成 URDF 序落盘 |
| `src/common/s2r_recorder.cpp` | 落盘列注释里也标了 `trq 腿×CONV_A(thigh=−1)` |

### ④ 前馈/叠加扭矩（**在 status 坐标**，不经 CONV_A）

| 位置 | 作用 | 关键点 |
|---|---|---|
| `MotorManager::SendOnce()` `motor_manager.cpp:~445` | 每 2 ms 调 `m_leg_ff(...)` 并把返回值**加进 `send_torque`**，随后才 `ApplyMotorCalibrationInverse` | 摩擦前馈用的是 `current_position`（status 坐标）⇒ **它的符号由电机层负责，不受 CONV_A 影响** |
| `examples_common.cpp` `EnableRlFrictionFF()` | 安装上述回调（仅 Ex25 / Ex36 调） | `leg_friction_ff()` 受 `LEG_FF_ENABLE` 控制，**当前 = false（已关）** |
| `JOINT_IMPEDANCE[..].tau_ff`（`motor_calibration.h`） | 重力前馈 | 当前全为 `0`；填值时按实测同号填，`ApplyMotorCalibrationInverse` 会自动按 pos_scale 翻转 |

### ⑤ IMU / 姿态（不是关节，但属同一类"符号层"）

| 位置 | 作用 |
|---|---|
| `ImuDevice::ParseGyroFrame()` `imu_device.cpp:~187` | `Z_DOWN_X` 安装：**`gy = −gy, gz = −gz`**（X 不变） |
| `ImuDevice::ParseQuatFrame()` `imu_device.cpp:~207` | `Z_DOWN_X`：`(w,x,y,z) → (−x, w, −z, y)`（绕 X 翻 180°） |
| `rl::world2self()`（`rl_controller.cpp:~72`） | 世界系 → 机体系；`projected_gravity = world2self(quat, [0,0,−1])` |

### ⑥ 动作 → 目标（有缩放，无符号）

| 位置 | 作用 |
|---|---|
| `rl::action_to_target()`（`rl_controller.h:~133`） | `q_target = DEFAULT_POSE[i] + action*ACTION_SCALE`（`ACTION_SCALE=0.25`） |
| `mlp_forward_clipped()`（`mlp.h`） | 新策略按训练 `action_clip=1.0` 截断到 [−1,1] |
| 轮速 | `WHEEL_VEL_SCALE = 12.5`（缩放；轮的方向由 `pos_scale` + 固件速度环决定） |

### ⑦ 工具侧（Python）

| 位置 | 说明 |
|---|---|
| `tool/plot_rlrun.py` | `status = (urdf − CONV_B)/CONV_A`，A/B **从 `sim2real_conv.cpp` 现读**（2026-10-17 改，之前是硬编码，改符号后会静默算错） |
| `tool/pose_view.py` | 解析 `DEFAULT_POSE`/`CONV_A`/`CONV_B` 生成预设；**实时流不做二次换算**（C++ 已发 URDF） |
| `tool/pace_export_dataset.py` | 直接用 CSV 里的 URDF 值，不做符号换算 |

---

## 三、净符号一览（位置）＝ pos_scale × CONV_A

| 关节 | FL | FR | RL | RR | 模式 |
|---|---|---|---|---|---|
| **hip** | **−1** | **+1** | **+1** | **−1** | **对角同号** ← 唯一与其他关节类型不同 |
| thigh | −1 | +1 | −1 | +1 | 左右镜像 |
| calf | −1 | +1 | −1 | +1 | 左右镜像 |
| wheel | −1 | +1 | −1 | +1 | 左右镜像 |

> **观察（非定论）**：thigh / calf / wheel 三者都是同一套「左右镜像」模式，**只有 hip 是对角**。
> 而 MJCF 里四条 hip 的世界轴全是 `(+1,0,0)`、`dogurdf.py` 四条腿共用同一个 `NOMINAL_HIP`。
> 这个"三类一致、一类不同"的差异值得作为**重点怀疑对象**，但**模型本身证明不了哪个符号对** ——
> 必须用下面的测量法定案。

---

## 四、怎么验证每一层（按可靠性排序）

### A. 电机层（status 坐标）—— 最可靠，需使能单个电机

给某个电机发一个**已知 +0.1 rad 的 status 目标**（阻抗模式、抬离地面），读回 `GetStatus().position`：
- 读数**增大** ⇒ 该电机 `pos_scale` 符号正确
- 减小 ⇒ `pos_scale` 反了（应改 `MOTOR_CALIBRATION[can][m].pos_scale` **并同步改 `vel_scale`**）

### B. 策略层（URDF 坐标）—— 无需使能，但**必须四腿横向比较**

单看"变大/变小"会被"哪边算左侧"搞混（FR hip 的两次相反结论就是这么来的）。正确做法：

1. `Ex63`（被动、不使能）跑起来，记下 **4 个 hip 读数**
2. 把 **四条腿的远端依次向同一个世界方向**推（例如都往机器人左侧）
3. 记录每条腿 Δ 的**符号**

**判据：4 个 Δ 的符号必须全部相同。**
- 全 `+` 或全 `−` ⇒ ✅ 一致
- 出现对角分裂（如 `+,−,−,+`）⇒ 说明其中两条腿的 `CONV_A` 与另外两条不一致
- 只有一条不同 ⇒ 那一条单独改

> 注意：**改 `CONV_A` 必须同时改同索引的 `CONV_B`（取负）**，否则零位会偏 `2×B`。

### C. 运动验证 —— 最硬，需使能

下发给某关节一个**已知 +Δ 的 URDF 目标**，看腿实际往哪边走。
模型语义（已用 MJCF + `mj_forward` 实测确认，四条腿一致）：
- **hip +** ⇒ 腿远端向**机器人左侧(+y)** 移动
- **thigh + / calf +** ⇒ 腿远端向**后(−x)** 移动

---

## 五、历史坑（改符号前先读）

1. **`vel_scale` 必须与 `pos_scale` 一致**。2026-09-04 曾漏改，导致速度反馈反向
   （Ex54 摩擦辨识 corr(dθ/dt, ω) ≈ −0.6、fc 回归为负）。
2. **`POLICY_TO_MJX` / `MJX_TO_POLICY` 两个旧名与实际语义正好相反**（2026-09-30 更名）。
   新代码一律用 `POLICY_TO_CAN` / `CAN_TO_POLICY`。
3. **摩擦前馈工作在 status 坐标**（`SendOnce` 里叠加，之后才逆标定）⇒ 它**不**乘 `CONV_A`；
   只有"力矩落盘"那一步才乘 `CONV_A`。把这两件事混起来会得出错误结论。
4. **改 `CONV_A` 只翻符号会让零位偏移 `2×CONV_B`**（hip 是 3.4°）—— 必须 A、B 同时取负。
5. **别在工具里硬编码 A/B**（`plot_rlrun.py` 曾硬编码 `CONV_B_HIP=0.0297, A=+1`，改符号后静默算错）。
6. **IMU 安装方向有两处独立的翻转**：gyro 的 y/z、quat 的 `(w,x,y,z)→(−x,w,−z,y)`；
   调姿态前先确认 `imu.SetMount()` 与实物一致。
7. **`DEFAULT_POSE` 只影响偏移，不影响符号**（2026-10-17 已更正为 `thigh −0.03 / calf −0.116`，
   对齐训练侧 0.52 m 站高标定）。

---

## 六、一页速查：改任何一个关节的符号，动哪里？

| 目标 | 改哪里 | 必须同步 |
|---|---|---|
| 只影响**真机**（观测+动作） | `CONV_A[i] = ∓1` **且** `CONV_B[i] = −CONV_B[i]` | 无（`urdf_to_status` 自动互逆） |
| 影响**所有**使用该电机的程序（含工具/标定） | `MOTOR_CALIBRATION[can][m].pos_scale` **且** `.vel_scale` | 检查 `pos_offset` 语义（status 零位会变） |
| 只影响**力矩记录** | `ex_rl.cpp` / `motion_controller.cpp` 的 `tau_policy[i] = CONV_A[i]*tau_can[...]` | 无（跟 `CONV_A` 走） |
| 只影响**IMU 姿态** | `ImuDevice::SetMount()` / `imu_device.cpp` 的两处补偿 | 无 |
