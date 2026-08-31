# dogurdf 轮足策略 sim2real 部署交接文档

> 本文记录 dogurdf 轮足策略从训练到真机的完整部署链路、关键规格、验证结果与剩余工作。
> 数值唯一真值来源：`include/strategy/rl_controller.h`(.cpp)、`include/strategy/sim2real_conv.h`(.cpp)、`include/strategy/imu_device.h`(.cpp)。
> 训练侧权威参考：`/home/sysu/Desktop/Project/Bruce/RL_Train/code`（`src/sim2sim.py`、`src/cfg/dogurdf_config.py`）。

---

## 1. 交付概述

已把 MJX/JAX 训练的 dogurdf 轮足策略（16 DoF，对角步态转弯）部署为真机 C++ 闭环：

```
HWT606 IMU ─(115200 串口, Z朝下绕X翻)─► imu_device
                                            │ gyro + quat
16 电机反馈 ─(标定后=指令角, CAN order)─────► status_to_urdf(CONV_A/B) ─► 重排 policy order
                                            │
                                      build_observation(64维)
                                            │
                                      mlp_forward(手写 MLP)
                                            │ action(16, policy order)
                               urdf_to_status + 重排 CAN order ─► SendImpedance(16 电机)
```

**推理后端**：手写 C++ MLP（`include/strategy/mlp.h`，零依赖，不用 libtorch/onnxruntime），适配仅 CPU 的 NUC i7。**控制架构**：RL 策略 50Hz，电机收发独立线程（receive 2ms/500Hz、send 2ms/500Hz）。

---

## 2. 文件结构（分层重组后）

### 部署模块（`include/strategy/` + `src/strategy/`）
| 文件 | 作用 |
|---|---|
| `mlp.h` | 手写 MLP 前向（64→512→256→128→16，ELU） |
| `policy_weights.h` | 导出的 actor 权重（**生成物，勿手改**） |
| `policy_test_ref.h` | MLP 数值校验参考（**生成物**） |
| `rl_controller.h/.cpp` | 观测构建 / PD 控制律 / 常量 / 关节顺序映射 |
| `sim2real_conv.h/.cpp` | 真机 GetStatus 角 ↔ URDF 角转换（CONV_A/CONV_B） |
| `imu_device.h/.cpp` | HWT606 串口读取 + 安装方向变换 |

### 部署入口（`src/app/`）
- `examples/ex_rl.cpp`：Example25/30/31/32/35/36/37/38/51/52/53（RL 相关）
- `examples/ex_diag.cpp`：Example24/26-29/33/34/39-50（诊断 + USB2CAN）
- `main.cpp`：当前激活 **Example44_USB2CanXboxControl**（USB2CAN 手柄控制）

### 权重导出
- `tool/export_policy.py`：读取 `RL_Train/code/checkpoints/.../traj_v28/iteration_3000.pkl` → 写 `include/strategy/policy_weights.h` + `policy_test_ref.h`（含 numpy/flax 交叉校验）。

---

## 3. 策略规格（traj_v28 / iteration_3000）

| 项 | 值 |
|---|---|
| 观测 / 动作 | 64 维入 → 16 维出 |
| 控制频率 | RL **50 Hz**；电机 send 2ms/500Hz / receive 2ms/500Hz |
| 网络 | MLP 64→512→256→128→16，**ELU**，确定性 `act()` |
| 站姿 default pose（URDF 约定） | hip=0, thigh=0.20, calf=−0.35, wheel=0（站高 0.45 m） |
| 腿控制律（代码实际） | `τ = 250·(q_t−q) − 4·q̇`（LEG_KP=250, LEG_KD=4，对齐 v28 sim2sim 默认） |
| 轮控制律 | **固件 SPEED 速度环**（`SendSpeed(vel, kvp, ki)`，WHEEL_KVP=3.0/WHEEL_KVI=0.05） |
| 扭矩限幅 | 腿 250 N·m，轮 53 N·m |
| 关节限位 | hip ±0.6，thigh −0.7~1.75，calf −1.0~0.35（rad） |

> 部署参数已统一对齐 v28 sim2sim.py 默认（LEG_KP/KD=250/4、LEG_TORQUE_LIMIT=250）。⚠ 300/10 是 V30 参数勿混淆。

### 观测 64 维布局（`rl_controller.cpp: build_observation`）
与训练 `sim2sim.py::_build_observation` 严格一致：

| 段 | 索引 | 维 | 内容 |
|---|---|---|---|
| base_lin_vel | 0–2 | 3 | **恒 0**（训练端置零） |
| base_ang_vel | 3–5 | 3 | 机体系角速度（IMU gyro，rad/s） |
| projected_gravity | 6–8 | 3 | `world2self(quat, [0,0,-1])` |
| joint_pos_rel | 9–20 | 12 | 腿位置 − default_pose（URDF 约定） |
| joint_vel | 21–36 | 16 | 全部关节速度（含轮） |
| last_action | 37–52 | 16 | 上一控制步 action |
| command | 53–55 | 3 | `[vx, vy, wz]`（vy 恒 0） |
| gait_phase | 56–63 | 8 | `[sin(2πφ), cos(2πφ)]×4` |

gait_phase：`φ = (step·0.02/0.6 + offset) mod 1`，`offset=[0,0.5,0.5,0]`（FL,FR,RL,RR），**step 每控制步 +1、复位归 0**。观测整体 clip 到 [−100,100]。

---

## 4. 关节顺序映射（关键，勿踩坑）

真机 CAN order（per-leg）：`FL hip/thigh/calf/wheel | FR | RL | RR`；策略 policy order：**12 腿关节（FL,FR,RL,RR × hip/thigh/calf）+ 4 轮**。

```
POLICY_TO_MJX = (0,1,2,12, 3,4,5,13, 6,7,8,14, 9,10,11,15)   // 写：can[mjx] = policy[POLICY_TO_MJX[mjx]]
MJX_TO_POLICY = (0,1,2,4, 5,6,8, 9,10,12, 13,14,3, 7,11,15)   // 读：policy[i] = can[MJX_TO_POLICY[i]]
```

即：FL hip→a0/thigh→a1/calf→a2/wheel→a12；FR →a3,4,5,13；RL →a6,7,8,14；RR →a9,10,11,15。

---

## 5. 零位转换（sim2real_conv）

真机 `GetStatus` 返回**标定后指令角**（电机标定约定），策略工作在 **URDF 约定**，二者有每关节符号/偏移差异，由 `CONV_A/CONV_B` 吸收：

```
URDF = CONV_A[joint]·GetStatus + CONV_B[joint]
CONV_A: hip+1 / thigh−1 / calf+1 / wheel+1
CONV_B: hip+0.0297 / thigh−0.9624 / calf−1.2832 / wheel 0   (rad)
```

- `status_to_urdf(angle, j)`：读反馈转 URDF（位置观测）
- `status_vel_to_urdf(vel, j)`：速度只保留符号（B 是常数）
- `urdf_to_status(angle, j)`：动作下发转真机指令角

> CONV_B 值基于 2026-08-20 真机 L 形测量，代码注释与数组已一致（thigh −0.9624 / calf −1.2832）。

---

## 6. IMU（维特 HWT606）

| 项 | 值 |
|---|---|
| 串口 | `/dev/ttyUSB0` @ **115200**（CH340 芯片） |
| 输出内容 | RSW=0x0204 → 角速度(0x52) + 四元数(0x59) |
| 输出速率 | RRATE=0x09 → 100 Hz |
| 算法 | AXIS6=0x01 → **6 轴**（避电机磁场干扰磁力计） |
| 安装方向 | **Z 轴朝下、绕 X 轴翻面** → `ImuMount::Z_DOWN_X` |

安装方向变换（`imu_device.cpp` 内）：gyro `(gx, -gy, -gz)`；四元数 `q_body = (-x, w, -z, y)`。
帧格式：`0x55 | TYPE | 8×data(低字节在前) | SUM`，11 字节；角速度 `/32768*2000` °/s→rad/s，四元数 `/32768`。

---

## 7. 构建与运行

```bash
cd simplify
cmake -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j$(nproc)
# main.cpp 当前激活 Example44_USB2CanXboxControl（USB2CAN 手柄控制），直接运行：
./bin/can_motor_app
```

- **达妙 USB2CAN 链接**：`libdm_device.so` 需新版 libstdc++(GLIBCXX_3.4.32) + libusb(≥1.0.26)。CMakeLists **自动探测** conda lib 路径（bruce 机 `/home/bruce/miniforge3/lib`、sysu 机 `/home/sysu/miniconda3/lib`），可用 `-DCONDA_LIB_DIR` 覆盖；libusb 用绝对路径避免解析到系统老版本。
- 起立目标 = `urdf_to_status(DEFAULT_POSE)`（与 RL 目标一致，消除跳变）。
- **轮子走固件 SPEED 速度环**（`SendSpeed(vel, WHEEL_KVP, WHEEL_KVI)`，固件内部 1kHz 闭环），带软启动 `WHEEL_SOFT_KVP` 与移动门控 `WHEEL_CMD_MOVE_THR`（站立锁轮）。
- 手柄命令量程：vx ±1.0 m/s，wz ±1.0 rad/s（训练 yaw 范围为 ±2.0，注意差异）；`CMD_BIAS_VX=-0.05` 抵消前冲（须 < 训练 turn_lin_threshold 0.1）。

---

## 8. 数值验证结果（已通过）

| 校验项 | 结果（traj_v28） |
|---|---|
| flax vs numpy 前向（`export_policy.py`） | 9.54e-07 |
| C++ mlp_forward vs 参考 | 2.38e-06 |
| 工程编译 | ✅ 通过 |
| Example30 离线回归门（`max_err < 1e-3`） | 需上机前跑一次确认 |
| IMU 真实数据（115200） | gyro 静止≈0，重力投影模长≈1 |

---

## 9. 剩余工作

1. **CONV_B 现场微调**：基于一次 L 形目测，真机低增益验证（Example32）后微调；当前新值下默认姿态已在限位内。
2. **LEG_KD 真机复核**：代码 LEG_KD=4 对齐 v28 训练；曾评估提至 10（hip 外翻漂移）未落地，若真机 hip 仍漂移可现场试提 6~10。
3. **真机起步**：先 PD 保持站姿确认能托住，再小 yaw 指令起步，逐步加大（v28 训练含 action_delay，抗 20-60ms 总线时延，无需额外补偿）。

---

## 10. 已知坑

- **BRLTTY 抢占串口**：Linux 上 brltty 抢 CH340 导致 `/dev/ttyUSB0` 不出现，需 `systemctl stop/disable/mask brltty` 后重新插拔；udev 规则 CH340 → MODE=0666。
- **轮子阻抗前馈已废弃**（2026-08-29 SPEED 迁移）：轮速控制走固件 SPEED 速度环；阻抗模式忽略 vel_des，勿再用阻抗扭矩通道控轮。
- **WHEEL_KVI 别用 0.3**：RL 上积分过强会振荡疯转，用 0.05。
- **权重生成物路径**：`export_policy.py` 必须输出到 `include/strategy/`（`include/rl/` 已废弃，编译不引用）。
- **达妙链接环境**：依赖本机 conda 的 libstdc++/libusb，CMakeLists 自动探测，换机器可用 `-DCONDA_LIB_DIR` 覆盖。
