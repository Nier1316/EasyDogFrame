# dogurdf 轮足策略 sim2real 部署交接文档

> 本文记录 `dogurdf_sim2sim_deploy/` 训练策略移植到真机 `simplify/` 的完整链路、关键规格、验证结果与剩余工作。
> 数值唯一真值来源：`include/rl/rl_controller.h`（.cpp）与 `include/imu_device.h`（.cpp）。
> 训练侧权威参考：`dogurdf_sim2sim_deploy/README.md` 与 `dogurdf_sim2sim_deploy/src/sim2sim.py`。

---

## 1. 交付概述

已把 MJX/JAX 训练的 dogurdf 轮足策略（16 DoF，对角步态转弯）部署为真机 C++ 闭环：

```
HWT606 IMU ─(115200 串口, Z朝下绕X翻)─► imu_device
                                            │ gyro + quat
16 电机反馈 ─(标定后=指令角, CAN order)─────► 重排 policy order ─► build_observation(64维)
                                            │
                                      mlp_forward(手写 MLP)
                                            │ action(16, policy order)
                                 重排 CAN order ─► PD/扭矩 ─► SendImpedance(16 电机)
```

**推理后端**：手写 C++ MLP（零依赖，不用 libtorch/onnxruntime），适配仅 CPU 的 NUC i7。

---

## 2. 新增 / 修改文件清单

### 新增

| 文件 | 作用 |
|---|---|
| `tool/export_policy.py` | 从 flax checkpoint 导出 actor 权重 → `policy_weights.h`，含 numpy/flax 前向一致性校验 |
| `include/rl/mlp.h` | 手写 MLP 前向（64→512→256→128→16，ELU），纯头文件 |
| `include/rl/policy_weights.h` | 导出的 actor 权重（**生成物，勿手改**） |
| `include/rl/policy_test_ref.h` | MLP 数值校验用参考输入/输出（**生成物**） |
| `include/rl/rl_controller.h` | 观测构建 / 控制律 / 常量 / 关节顺序映射声明 |
| `src/rl/rl_controller.cpp` | 观测构建 + `world2self` + 常量数组定义 |
| `include/imu_device.h` | HWT606 串口读取 + 安装方向变换声明 |
| `src/imu_device.cpp` | 串口读取 + 帧解析 + 安装方向变换实现 |

### 修改

| 文件 | 改动 |
|---|---|
| `src/example.cpp` | 新增 `Example25_RLPolicyControl`（50Hz 闭环 + 起立 + 跌倒检测 + 急停） |
| `include/example.h` | 声明 `Example25_RLPolicyControl` |
| `src/main.cpp` | 注释 `Example21`、激活 `Example25` |
| `CMakeLists.txt` | `SOURCES` 追加 `src/rl/rl_controller.cpp`、`src/imu_device.cpp` |

---

## 3. 策略规格（真机必须复现）

| 项 | 值 |
|---|---|
| 观测 / 动作 | 64 维入 → 16 维出 |
| 控制频率 | **50 Hz**（控制步 0.02 s） |
| 网络 | MLP 64→512→256→128→16，**ELU**，确定性 `act()` |
| 站姿 default pose | hip=0, thigh=0.20, calf=−0.35, wheel=0（站高 0.45 m） |
| 腿控制律 | `q_t = clip(default + 0.25·a, 限位)`，`τ = 150·(q_t−q) − 4·qd` |
| 轮控制律 | `w_t = 12.5·a`，`τ = 2·(w_t − qd)`（kp=0） |
| 扭矩限幅 | 腿 150 N·m，轮 53 N·m |
| 关节限位 | hip ±0.6，thigh −0.7~1.75，calf −1.0~0.35（rad） |

### 观测 64 维布局（`rl_controller.cpp: build_observation`）

| 段 | 索引 | 维 | 内容 |
|---|---|---|---|
| base_lin_vel | 0–2 | 3 | **恒 0**（训练端置零） |
| base_ang_vel | 3–5 | 3 | 机体系角速度（IMU gyro，rad/s） |
| projected_gravity | 6–8 | 3 | `world2self(quat, [0,0,-1])` |
| joint_pos_rel | 9–20 | 12 | 腿位置 − default_pose（policy order） |
| joint_vel | 21–36 | 16 | 全部关节速度（policy order，含轮） |
| last_action | 37–52 | 16 | 上一控制步 action |
| command | 53–55 | 3 | `[vx, vy, wz]`（vy 恒 0） |
| gait_phase | 56–63 | 8 | `[sin(2πφ), cos(2πφ)]×4` |

gait_phase：`φ = (step·0.02/0.6 + offset) mod 1`，`offset=[0,0.5,0.5,0]`（FL,FR,RL,RR，对角步态），**step 每控制步 +1、复位归 0**。观测整体 clip 到 [−100,100]。

---

## 4. 关节顺序映射（关键，勿踩坑）

真机 CAN order（per-leg）：`FL hip/thigh/calf/wheel | FR | RL | RR`，等价于 MJX order。
策略 policy order：**12 腿关节（FL,FR,RL,RR 各 hip/thigh/calf）+ 4 轮**。

```
POLICY_TO_MJX = (0,1,2,12, 3,4,5,13, 6,7,8,14, 9,10,11,15)   // 写：can[mjx] = policy[POLICY_TO_MJX[mjx]]
MJX_TO_POLICY = (0,1,2,4, 5,6,8, 9,10,12, 13,14,3, 7,11,15)   // 读：policy[i] = can[MJX_TO_POLICY[i]]
```

即：FL hip→a0/thigh→a1/calf→a2/wheel→a12；FR →a3,4,5,13；RL →a6,7,8,14；RR →a9,10,11,15。

---

## 5. IMU（维特 HWT606）

| 项 | 值 |
|---|---|
| 串口 | `/dev/ttyUSB0` @ **115200**（CH340 芯片） |
| 输出内容 | RSW=0x0204 → 只输出角速度(0x52) + 四元数(0x59) |
| 输出速率 | RRATE=0x09 → 100 Hz |
| 算法 | AXIS6=0x01 → **6 轴**（避电机磁场干扰磁力计） |
| 安装方向 | **Z 轴朝下、绕 X 轴翻面** → `ImuMount::Z_DOWN_X` |

安装方向变换（`imu_device.cpp` 内实现）：
- gyro：`(gx, -gy, -gz)`
- 四元数：`q_body = (-x, w, -z, y)`

帧格式：`0x55 | TYPE | 8×data(低字节在前) | SUM`，11 字节；角速度 `/32768*2000` °/s→rad/s，四元数 `/32768`。

**配置命令**（已在模块 flash 保存，断电保持；重配可用 `imu.Configure()`）：
解锁 `FF AA 69 88 B5` → 写 `FF AA 02 04 02`(RSW) / `FF AA 03 09 00`(RRATE) / `FF AA 24 01 00`(AXIS6) → 保存 `FF AA 00 00 00`。

---

## 6. 构建与运行

```bash
cd simplify
cmake -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j$(nproc)
# main.cpp 已切到 Example25，直接运行：
./bin/can_motor_app
```

运行流程：初始化 16 电机（关节/轮都 IMPEDANCE）→ 使能 → 起立 10s 到真机站立指令角 → 50Hz RL 循环 → **Ctrl+C 急停（失能所有电机）**。

- 轮子走上位机扭矩前馈：`SendImpedance(cp,4, 0,0,0,0, τ)`（kp=kd=0），**不用固件速度环**。
- command 当前固定 `[0,0,0]`（原地站立），改 `Example25` 里 `cmd[3]` 即可行驶（vx ±1.0 m/s，wz ±1.0 rad/s）。
- 零位对齐未做前，**策略输出不会正确**，务必小指令 + 随时急停。

---

## 7. 数值验证结果（已通过）

| 校验项 | 误差 | 目标 |
|---|---|---|
| flax vs numpy 前向（`export_policy.py`） | 4.17e-07 | < 1e-5 |
| C++ MLP vs 参考输出 | 1.31e-06 | < 1e-4 |
| C++ 观测构建 vs 仿真 | 4.77e-07 | < 1e-5 |
| IMU 帧解析（gyro/quat/坏帧丢弃） | PASS | — |
| IMU 安装方向变换（Z_DOWN_X） | PASS | — |
| IMU 真实数据（115200） | gyro 静止≈0，重力投影模长≈1 | — |

---

## 8. 剩余工作

1. **零位对齐（必须做）**：`rl::DEFAULT_POSE` 现为仿真占位值（hip=0/thigh=0.20/calf=−0.35），真机 `GetStatus` 返回指令角（站立 thigh=−60°/−1.047 rad、calf=60°/+1.047 rad）。需用真机 `leg_fk` 反解仿真 default 足端 `[±0.267, ±0.2558, −0.3364]` / `[−0.386, ±0.2558, −0.3364]` 对应的真机指令角，替换 `DEFAULT_POSE`（含方向符号）。
2. **IMU 装机 + 静态标定**：水平固定到机身中心，轴对齐后水平静置，验证重力投影 ≈ `(0,0,-1)`。
3. **真机起步**：先 PD 保持站姿确认能托住，再 `cmd_vel_yaw 0.2` 小指令起步，逐步加到 1.0。

---

## 9. 已知坑

- **BRLTTY 抢占串口**：Linux 上 brltty 会抢 CH340 导致 `/dev/ttyUSB0` 不出现，需 `sudo apt remove brltty` 后重新插拔。已写 udev 规则 `99-ch340-imu.rules`（0666/plugdev）。
- **轮子固件速度环不可靠**（`MOTION_RL_KNOWLEDGE.md`）：务必走上位机扭矩前馈。
- **训练未含时延随机化**：CANET TCP + 串口时延需真机评估。
- **HWT606 满量程 2000°/s 边界**：raw=32768 恰好溢出 int16，实际角速度远离满量程，无影响。
