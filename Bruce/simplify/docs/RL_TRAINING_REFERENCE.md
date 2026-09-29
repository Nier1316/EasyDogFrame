# RL 重训参考文档（真机辨识参数与控制架构）

> 日期：2026-08-28
> 目的：为 dogurdf 轮足策略重新训练（sim2sim.py / Isaac Gym / MuJoCo）提供**真机实测**的
> 物理参数、控制架构、反馈特性与已知坑，使仿真模型与奖励设计对齐真实硬件，缩小 sim2real gap。
> 数据来源：Example46/47（单关节阶跃 + 整狗站立 chirp 扫描辨识）、Example36（RL 站立）日志分析。
>
> ⚠ **2026-09-29 更新（部分结论已过时）**：
> - 真机**轮子不再走上位机闭环**，已改为**固件 SPEED 速度环**（`SendSpeed(vel, kvp, ki)`，`WHEEL_KVP=3.0/WHEEL_KVI=0.05`，固件内部 1kHz）。§1/§4.1/§5 中"上位机 500Hz 闭环/阻抗扭矩控轮"的写法见各节订正。
> - 轮速**软限位已失效**（`WHEEL_SOFT_LIMIT_*` / `rl::wheel_torque()` 无调用者）；现由 `MotorManager` 的 `WHEEL_ESTOP_*`（15 rad/s 自动急停）承担。
> - 真机扭矩命令限幅现为 **Hip 120 / Thigh 120 / Calf 200 / Wheel 52 N·m**（`TORQUE_CMD_LIMIT`），§6.3 的"±150"已作废。
> - 当前部署权重 `weights/iteration_9754.pkl`，`main.cpp` 激活 **Example37_RLTeleopControl**（示例编号 17~57，共 40 个；`Example55` 从未实现已移除）。

---

## 1. 真机硬件与控制架构

| 项 | 值 | 说明 |
|---|---|---|
| 电机 | 16 = 4 CAN × 4（1=hip, 2=thigh, 3=calf, 4=wheel） | 达妙 MIT 协议，位置15bit/速度12bit/kp12bit/kd12bit/扭矩12bit |
| 传输 | 达妙 USB2CAN 双路模块 ×2（CAN0-3） | 逻辑路 idx→(设备=idx/2, 通道=idx%2) |
| 控制频率 | **CONTROL_HZ = 500**（`robot_calibration.h`） | motor_receive 2ms / motor_send 2ms |
| 策略决策 | **50Hz**（`CONTROL_DT=0.02`，与训练一致） | RL 循环 HZ=50 |
| **轮子闭环** | **固件 SPEED 速度环**（`SendSpeed`，固件内部 1kHz） | ⚠ **2026-08-29 后已改**：旧文写的"上位机 500Hz 闭环"已过时；轮子现走固件速度环 |
| 腿关节 PD | 下发固件（kp/kd），固件内部高频执行 | 策略只输出位置/速度目标 |

**架构要点**（对齐 legged_gym 的做法）：
- 50Hz 策略决策 + 高频 PD 执行是标准架构。leg 的 PD 下发给固件（kp/kd=250/4）；**轮子改走固件 SPEED 速度环**（`rl::WHEEL_KVP=3.0` / `rl::WHEEL_KVI=0.05`）。
  ⚠️ 订正：旧文"轮子没有可靠固件 PD，必须上位机 500Hz 闭环"已被推翻——2026-08-29 起轮控迁移到固件 SPEED 模式，`rl::WHEEL_KD=1.0` 仅用于不再被调用的阻抗诊断路径。
- 轮子控制模型（历史/诊断）：`τ = kd·(目标轮速 − 反馈轮速) + 前馈`，kd=0 时退化为纯扭矩；**实际真机不经过该路径**。

---

## 2. 关节参数辨识结果（Example47，整狗站立 chirp 扫描）

模型：`τ = J·θ̈ + B·θ̇ + f_c·sign(θ̇) + K_g·θ + b`
- 方法：整狗起立到 STAND_*（轮子悬空）→ 逐关节斜坡测静摩擦 → 开环 chirp（0.5→15Hz, 8s）→ 5 参数最小二乘
- 结果记录：`log/sysid_all.csv`（示例47 运行生成）

### 2.1 辨识表（2026-08-28）

| 关节 | J (kg·m²) | B (Nm·s/rad) | f_c (Nm) | K_g (Nm/rad) | b (Nm) | 可靠性 |
|---|---|---|---|---|---|---|
| FL hip (C0-1) | -0.11 | 8.6 | -0.55 | -143.9 | 33.0 | ⚠️ 位置范围仅0.027，数据差 |
| FR hip (C1-1) | 0.066 | -0.26 | -0.03 | -1.4 | **+10.5** | ✅ b 可靠 |
| RL hip (C2-1) | 0.047 | -0.72 | 0.01 | -19.7 | **+11.0** | ✅ b 可靠 |
| RR hip (C3-1) | -0.049 | 3.3 | -0.24 | -4.8 | **+9.1** | ✅ b 可靠 |
| FL thigh (C0-2) | 0.043 | -1.6 | -0.29 | 0.4 | **-5.1** | ✅ b 可靠 |
| FR thigh (C1-2) | -0.003 | 0.4 | -0.09 | 1.2 | **-3.0** | ✅ b 可靠 |
| RL thigh (C2-2) | 0.056 | -1.2 | -0.36 | 2.1 | **-2.5** | ✅ b 可靠 |
| RR thigh (C3-2) | -0.048 | 0.3 | 0.64 | 2.4 | **-2.3** | ✅ b 可靠 |
| FL calf (C0-3) | -0.035 | 1.1 | 0.07 | -10.1 | +6.2 | ⚠️ 分散 |
| FR calf (C1-3) | 0.115 | 1.7 | -0.15 | -24.1 | +25.9 | ⚠️ b 异常大 |
| RL calf (C2-3) | -0.078 | 3.5 | 0.04 | 1.5 | -8.3 | ⚠️ 分散 |
| RR calf (C3-3) | -0.027 | -1.1 | 4.63 | 4.3 | -11.2 | ⚠️ f_c/幅度异常 |

### 2.2 可靠性结论

- ✅ **b（重力偏置）**：hip ≈ **+10 Nm**、thigh ≈ **-3 Nm**（多关节一致，可靠）。
  calf 分散（+6/-8/-11，C1-3=+26 异常），需重测。
- ✅ **f_c（库仑摩擦）**：多数关节 ≈ **0.05~0.6 Nm**（C3-3 RR calf=4.6 异常，需查机械）。
- ⚠️ **K_g（重力刚度）**：hip 在 0° 附近 ≈ **-34 Nm/rad**（量级可信），其他关节分散。
- ❌ **J / B（动力学）**：受**速度反馈延迟**限制（见 §4），开环 chirp 无法可靠辨识（多数为负值）。
  建议用 CAD 估算或短时阶跃法替代。

### 2.3 静摩擦（斜坡突破扭矩）

| 关节 | 突破扭矩（相对偏置） |
|---|---|
| hip | +1.5 Nm（一致） |
| thigh | +1.5~+6 Nm（FL thigh 6.5 需查） |
| calf | +2~+12.5 Nm（RR calf 需 12.5 Nm，机械摩擦异常大，建议检查） |

---

## 3. 建议的控制参数（起立 / 通用 PD）

### 3.1 KP / KD 整定框架

```
KP（刚度）   = J · (2π·f)²         f = 目标闭环带宽（建议 8~10 Hz）
KD（阻尼）   = 2·√(KP·J)           临界阻尼
tau_ff（前馈）= 抵消重力/摩擦（消稳态误差）
```

### 3.2 建议值（基于辨识 J 量级）

| 关节 | KP | KD | tau_ff |
|---|---|---|---|
| hip | 250 | 10 | **+10 Nm**（b 值，符号需真机验证） |
| thigh | 200 | 8 | **-3 Nm** |
| calf | 300 | 10 | 待重测（C0 +6 参考） |

- 当前值：`JOINT_IMPEDANCE` kp/kd = hip 300/10、thigh/calf 250/10，tau_ff 已填 —— 代码实际为 **hip −10 / thigh +5 / calf +12（CAN0/1）、+20（CAN2/3）Nm**。
  ⚠ **thigh 是未解冲突**：`include/motor/motor_calibration.h` 代码值是 `+5.0f`，但同行注释与提交 `6546688` 的信息写 `-5`。以**代码 +5** 为准，现场起立验证后再定。
- ⚠ 符号验证：填 tau_ff 后起立，若 hip 反而塌/过冲，把 −10 翻成 +10（thigh 若表现反向，优先怀疑上述 +5/−5 冲突）。
- ⚠ **RL 循环的 `LEG_KP/KD = 250/4`（对齐 v28 训练）**；起立的 `JOINT_IMPEDANCE`（300/250）可调，两套参数勿混。

### 3.3 稳态误差的根源与对策

- 纯 PD（无前馈）稳态误差 = `(重力负载 − tau_ff) / KP`。
- 实测 hip 负载 ~30 Nm（`kp_max=500` 时误差仍有 3.4°），**必须靠 tau_ff 补偿**，不能单靠 KP。

---

## 4. 反馈链路特性（影响仿真观测建模）

| 特性 | 实测 | 对训练的意义 |
|---|---|---|
| **速度反馈延迟** | 有延迟（导致 J 辨识为负） | 训练时关节速度观测**应加延迟/低通**仿真 |
| **轮速跳变** | USB2CAN 丢帧/错帧 → 单帧 ±4~8 rad/s | 观测加**限幅/滤波**，真机已加一阶低通（alpha=0.2） |
| **接收链路** | 高负载/电机大电流下接收方向可能挂（recv 停） | 部署必须加**接收心跳检测 + 急停** |
| **静摩擦死区** | hip/thigh ~1.5 Nm，RR calf ~12.5 Nm | 动作尺度下限要大于死区，或加摩擦前馈 |

### 4.1 轮子控制参数（真机已验证）

| 参数 | 值 | 说明 |
|---|---|---|
| WHEEL_KD | 1.0 | 轮速阻尼（RL 诊断路径用；SPEED 迁移后轮走固件速度环 kvp/ki） |
| WHEEL_VEL_SCALE | 12.5 | action→目标轮速 |
| 软限位 | **🔴 已失效** | `WHEEL_SOFT_LIMIT_*`（旧值 `\|vel\|>5.0 rad/s → 扭矩夹 ±30.0 Nm`）与 `rl::wheel_torque()` 均**无调用者**（SPEED 迁移后不经过），本次代码清理已移除。旧文写的"±10.0 Nm"更是早已被 30.0 取代 |
| 轮速保护（现行） | `MotorManager` 的 `WHEEL_ESTOP_*` | `WHEEL_ESTOP_VEL=15.0 rad/s` 自动触发急停（瞬态制动）；手动 `WheelEmergencyStop()` 才保持；`WHEEL_ESTOP_GRACE_TICKS=1000`（2s 静默窗口） |
| 轮速环频率 | **固件内部 1kHz**（`SendSpeed`，`WHEEL_KVP=3.0/WHEEL_KVI=0.05`） | 上位机 50Hz 只刷新目标；⚠ 旧文"必须上位机 500Hz 闭环"已过时 |
| 轮速滤波 | 一阶低通 alpha=0.2（仅轮，`ele_motor.cpp`） | 抑制 USB2CAN 跳变 |

---

## 5. 已知坑与对策（训练/部署必读）

1. **轮子 50Hz 上位机闭环 → 疯转**：⚠ **已由 SPEED 迁移解决**——轮子现走固件速度环，上位机只按 50Hz 刷新目标轮速。旧文"轮子 PD 必须 500Hz（`SendOnce`）"是迁移前的结论。
2. **USB2CAN 接收挂**：电机大电流（满扭矩）+ 高发送负载（500Hz×16）会打挂接收方向。
   对策：发送 500Hz 对齐、关高开销日志、接收心跳检测。
3. **recv 只取 1 帧 → 观测积压滞后**：`recv` 必须一次性取空队列（对齐 CANET 批量语义）。
4. **起立稳态误差**：KP 不足 + 无 tau_ff。填 b（重力偏置）作前馈。
5. **轮速反馈跳变**：加一阶低通滤波（`ele_motor.cpp` FilterWheelVel）。

---

## 6. 对 RL 重训的具体建议

### 6.1 仿真模型参数对齐（MuJoCo / Isaac Gym）

| 仿真参数 | 填入值 | 对应真机 |
|---|---|---|
| armature（转动惯量） | hip/thigh ~0.05, calf ~0.08 kg·m² | 辨识 J（量级） |
| damping | 0.5~2 Nm·s/rad | 辨识 B（不可靠，用量级 + 阶跃调） |
| frictionloss（库仑摩擦） | 0.05~0.6 Nm（RR calf 单独查） | 辨识 f_c |
| 重力补偿 | 站立姿态 hip +10 / thigh -3 Nm | 辨识 b |
| 轮子速度环 | 固件 SPEED（WHEEL_KVP=3.0/WHEEL_KVI=0.05） | 2026-08-29 迁移；WHEEL_KD 仅诊断用 |

### 6.2 观测与动作设计

- **观测**：关节角（标定后）+ 速度（**加延迟/低通仿真**）+ 扭矩 + 轮速。
  IMU 姿态（HWT606 100Hz）已验证可用。
- **动作**：腿关节位置残差（PD 下发固件，kp/kd=250/4），轮子速度目标（固件 SPEED 速度环，kvp=3.0/ki=0.05）。
- **动作尺度下限**：需大于静摩擦死区（hip/thigh ~1.5 Nm），否则策略输出被摩擦吃掉。

### 6.3 奖励设计建议（结合真机特性）

- 能耗惩罚 `Στ²`（真机扭矩 ~±10 Nm 站立保持，过大惩罚会抑制动作）。
- 摩擦相关：避免策略在死区边缘抖振（可用动作平滑惩罚）。
- 限位/扭矩 clamp：真机 kp_max=500 / kd_max=100 / 协议量程与命令限幅 **Hip 120、Thigh 120、Calf 200、Wheel 52 N·m**（`TORQUE_CMD_LIMIT`；⚠ 旧文"±150"已作废）。

### 6.4 部署频率

- **训练决策 50Hz 不变**；真机执行由固件承担（leg 固件 PD 于 500Hz 收发帧刷新，wheel 固件 SPEED 环内部 1kHz）。
  ⚠ 旧文"部署必须 500Hz 上位机 PD（含 wheel）"已过时——SPEED 迁移后轮子不再由上位机闭环。
- 若训练 sim dt 用 0.002×decimation 保证 PD 更新频率 ≥ 部署频率，仿真更贴近。

---

## 7. 代码/文件索引

| 文件 | 内容 |
|---|---|
| `include/motion/robot_calibration.h` | CONTROL_HZ=500, STAND_*_DEG |
| `include/motor/motor_calibration.h` | MOTOR_CALIBRATION, JOINT_IMPEDANCE, MOTOR_LIMITS |
| `include/strategy/rl_controller.h` | LEG_KP/KD=250/4, WHEEL_KD=1.0（仅诊断）, WHEEL_VEL_SCALE, WHEEL_KVP/KVI, LEG_FF_FC/FV |
| `include/strategy/sim2real_conv.h/.cpp` | CONV_A/B, DEFAULT_POSE, urdf_to_status |
| `include/motor/ele_motor_def.h` | MOTOR_LIMITS 量程 + TORQUE_CMD_LIMIT（120/120/200/52） |
| `src/motor/motor_manager.cpp` | WHEEL_ESTOP_* 轮控保护 + SendOnce（2ms 收发） |
| `src/motor/ele_motor.cpp` | 轮速一阶低通滤波（alpha=0.2） |
| `src/runtime/motor_io.cpp` | motor_receive/send 线程（2ms，优先级 80） |
| `src/app/examples/ex_diag.cpp` | Example47 整狗 chirp 辨识、Example54 吊装摩擦辨识 |
| `docs/ACTION_DELAY_MEASURE.md` | 真机传输延迟实测（建议 action_delay_steps=1） |
| `weights/iteration_9754.pkl` | 当前部署权重（`tool/export_policy.py` 默认输入） |

---

## 8. 待办/待确认

- [ ] C0-1 FL hip / C1-3 FR calf / C3-3 RR calf 数据异常，需重测（可能机械摩擦或辨识被带动）。
- [ ] calf 的 tau_ff 需重测确认（当前分散 +6/-8/-11）。
- [ ] J/B 需用 CAD 或阶跃法补测（开环 chirp 受反馈延迟限制）。
- [ ] hip/thigh tau_ff 符号需真机起立验证。**thigh 存在未解冲突**：`JOINT_IMPEDANCE` 代码值 `+5.0f`，注释/提交 `6546688` 写 `−5`，以代码为准，现场确认后只改注释不改安全值。
- [ ] 若重训，用本文 §6 对齐仿真参数后跑基线，再真机回归（Example36 起立 + Example46 单电机阶跃）。
