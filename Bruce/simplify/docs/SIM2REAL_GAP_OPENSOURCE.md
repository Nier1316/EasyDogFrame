# 开源运控框架是怎么缩小 sim2real gap 的（检索报告）

> 2026-10-02。检索范围：主要开源四足/轮足运控框架与相关论文，重点是**他们具体做了什么**（方法、数值、超参、配置文件），
> 以及**哪些能直接搬到本项目**（轮足、无足端力传感器、50 Hz 策略、500 Hz 电机环）。
>
> 出处全部给链接。标注：**【原文】** = 论文/仓库明确写的；**【推断】** = 我的判断，需自行验证。
> 配套：`docs/SIM2REAL_DATA_FEEDBACK.md`（本项目的采集与回馈方案）、`docs/CONTACT_PHASE_SIM2REAL.md` §5（gap 分类总表）。

---

## 0. 三条最值得注意的结论

1. **"识别"正在取代"随机化"**：Bjelonic 等人的 IJRR 2026 工作明确做到
   **"reliable policy transfer without randomization of dynamic parameters"** —— 靠自底向上的参数辨识
   （执行器 → 整机空中轨迹 → 地面行走）把 gap 压掉，而不是靠 DR 硬扛。这条对我们最重要，
   因为我们已经有 Ex47/54/58/59/60 的辨识工具链。
2. **轮子的正确做法可能不是"仿真去模拟固件速度环"，而是"真机改用关节级力矩+阻尼"**：
   同级别硬件的开源轮足部署（Go2W）用的是 `kp=0`、`kd=0.5`、`wheel_action_scale=5.0` 的**关节级阻尼**，
   与仿真里的 `kd·(ω_des−ω)` **同构**。详见 §2.3 —— 这可能比改仿真更省事、更彻底。
3. **最便宜、立刻可抄的一条：动作低通滤波**。LocoWheeledLegged 在训练与部署两侧都加动作低通
   （腿 `fc=5 Hz`、轮 `fc=15 Hz`，控制频率 50 Hz），目的是不让策略依赖真机跟不上的高频指令。
   我们目前只有轮子有 `WHEEL_CMD_ALPHA` 一阶低通，**腿没有**。

---

## 1. 我直接核实过的一手材料

### 1.1 Bjelonic / Tischhauser / Hutter — IJRR 2026（arXiv [2509.06342](https://arxiv.org/abs/2509.06342)）

**原文摘要要点**（我抓的是 arXiv abs 页）：
- 把 sim-to-real RL 与**永磁同步电机（PMSM）的物理能量模型**结合；
- **只需极小的参数集**就能刻画 sim2real gap；**四项紧凑奖励** + 第一性原理的能量损耗项（电损耗与机械损耗平衡）；
- 验证方式是**自底向上的动态参数辨识**：执行器 → **整机空中轨迹（in-air trajectories）** → 地面行走；
- 在 3 个平台上测试、**另部署到 10 台机器人**，**在没有对动态参数做随机化的情况下**实现可靠迁移；
- **ANYmal 的全程 CoT 降低 32%**（降到 1.27）；
- 论文注明 **"All code, models, and datasets are publicly available"**。

**【推断】对我们最直接的两点**：(a) "in-air trajectory 回放拟合"这条路他们走通了，且不依赖 DR；
(b) 他们建模的是**电机本体损耗**（PMSM 能量模型），而我们目前只做到"库仑摩擦 + 粘性"，
    没有电损耗/铜损项 —— 长时间行走的发热与效率差异可能就来自这里。

### 1.2 LocoWheeledLegged（Go2W，Isaac Lab）——[仓库](https://github.com/zhaozijie2022/LocoWheeledLegged)

**sim2real 相关措施（原文 README）**：
| 措施 | 具体做法 | 解决什么 |
|---|---|---|
| **动作低通滤波** | `JointPositionLowPassAction`，一阶/二阶可选，由控制频率与截止频率算权重 | 复杂地形上关节抖动；不让策略依赖真机跟不上的高频指令 |
| **姿态正则** | `hip_deviation_l2` / `joint_deviation_l2`：对偏离默认姿态做 L2 惩罚，尤其锁 hip | 站姿自然、别乱甩腿 |
| **action_rate 截断** | `custom_action_rate_l2_with_clip`，阈值 **7.0** | 训练稳定性 |
| **`last_action` 历史长度 = 1** | 显式设为 1 | **避免控制信号自激** ← 与我们相关（我们的 obs 里有 last_action） |
| 数值稳定性 | RayCaster `ray_hits_w` 截断、value function 双重截断 | 防 NaN/Inf |
| **部署参数对齐** | `deploy_real/configs/go2w.yaml`："**Align simulation and real parameters**" | 见 §1.3 |
| **关节顺序转换** | 原文强调："the joint order is **different** between the simulator and real robot, so **conversion is required**" | **和我们 POLICY/CAN 顺序的坑同源** |

### 1.3 `go2w.yaml`：轮足 sim↔real 对齐配置（**重点**，[原始文件](https://raw.githubusercontent.com/zhaozijie2022/LocoWheeledLegged/main/deploy_real/configs/go2w.yaml)）

```yaml
control_dt: 0.02                 # 50 Hz 策略（与我们相同）
num_obs: 57                      # 57 维观测
history_length: 6                # ← 策略带 6 帧历史（DreamWaQ 一系：同作者另有 DreamWaQ_Go2W）
actor_hidden_dims: [512, 256, 128]   # ← 与我们完全相同的 MLP 结构

kps: [50]*12 + [0]*4             # 腿 kp=50（比我们的 250 软得多）；轮 kp=0
kds: [1.0]*12 + [0.5]*4          # 腿 kd=1.0（我们 4.0）；轮 kd=0.5

# ↓ 显式的 sim/real 关节重映射表（顺序+符号差异编码成配置，而不是散在代码里）
default_real_angles: [0.0, 0.8, -1.5,  ... ] ×4 腿 + 4 轮 0
default_sim_angles:  [0.0,0.0,0.0, 0.0,0.8,0.8,0.8,0.8, -1.5,-1.5,-1.5,-1.5, 0,0,0,0]

num_actions: 16
action_scale: 0.25               # ← 与我们相同
wheel_action_scale: 5.0          # ← 我们用的是 12.5

# 动作滤波（在 50 Hz 控制频率下）
fc_leg: 5                        # 腿动作低通截止 (Hz)
fc_wheel: 15                     # 轮动作低通截止 (Hz)  ← 轮比腿"允许更快"
fs: 50

# 观测缩放
velocity_commands_scale: [1.0, 0.5, 0.523]   # 0.523 = pi/6
base_ang_vel_scale: 0.25; projected_gravity_scale: 1.0
joint_pos_scale: 1.0; joint_vel_scale: 0.05; last_action_scale: 1.0
```

**【原文】** 3 条可直接对照我们项目的差异：
1. **轮子是 `kp=0 / kd=0.5` 的关节级阻尼**（不是固件速度环），且 `wheel_action_scale=5.0`；
2. **策略带 6 帧历史**（我们只有单帧 64 维）；
3. **腿增益比我们软 5 倍**（50/1.0 vs 250/4.0）—— 这一点很值得注意：
   我们的 kp=250 是从 sim2sim 默认继承的，而**真正部署在同类硬件上的开源项目用的是 50**。

### 1.4 go2w_rl_gym —— [仓库](https://github.com/ShengqianChen/go2w_rl_gym)

**【原文】** 建立在 `legged_gym` + `rsl_rl` + `unitree_sdk2_python` 之上，轮足控制参考
`clearlab-sustech/Wheel-Legged-Gym` 与 `aCodeDog/legged-robots-manipulation`。
说明：**轮足 RL 部署的主流技术栈 = legged_gym 系**，与我们训练侧的 `RL_Train`（MJX 系）不同源。

### 1.5 Wheel-Legged-Gym（SUSTech CLEAR Lab）——[仓库](https://github.com/clearlab-sustech/Wheel-Legged-Gym)

**【原文】** 基于 legged_gym/rsl_rl；提供 `wheel_legged` / `wheel_legged_vmc` / **`wheel_legged_vmc_flat`** 三个任务；
`wheel_legged_vmc` 用 **VMC（虚拟模型控制）统一开链与闭链机构**，便于把策略部署到**闭链**轮足机器人上。
其 README 还记了一条与我们相关的工程坑：**GPU 三角网格地形下 `net_contact_force_tensor` 的接触力不可靠**，
需要改用 force sensor 并显式排除重力（`enable_forward_dynamics_forces=False`）。
**【推断】** 这条告诉我们：**别把仿真里的接触力当真值**去标定阈值 —— 与我们在
`docs/CONTACT_PHASE_SIM2REAL.md` 里"接触几乎只进奖励"的结论一致。

---

## 2. 按技术手段归类（含其它框架，待补）

> 本节由检索汇总，逐条给出处。

### 2.1 执行器建模：从"参数化模型"到"进化辨识 + 电机能量模型"

**PACE 流程**（Bjelonic 等，IJRR 2026，[arXiv 2509.06342v2](https://arxiv.org/html/2509.06342v2)，以下为我从论文正文核实的原文要点）：

| 环节 | 原文做法 |
|---|---|
| 数据采集 | 真机 **base 固定、悬空（in-air）**，**所有关节同时**做 **20~60 s 的 chirp** 激励；<br>激励加在**关节位置目标**上、由 PD 跟踪 —— 真机与仿真因此被**完全相同的参考轨迹**驱动，而不是各自自由响应，<br>"**eliminates accumulating phase error and makes direct trajectory matching well-posed**" |
| 记录 | 时间同步，采样率 **典型 400 Hz ~ 10 000 Hz** |
| 辨识参数集 | **Inertia, damping, friction, delay, bias**（惯量/阻尼/摩擦/**延迟**/偏置）—— **每关节一套**，参数集很小 |
| 优化器 | **CMA-ES**（原文强调"framework is agnostic to the specific choice of optimizer"，CMA-ES 只是实用实例）；<br>理由：摩擦、饱和、延迟带来**非凸、不可导**的损失 |
| 目标函数 | **时间平均的关节位置误差**（直接轨迹匹配），开环回放位置指令层面 |
| 仿真回放条件 | **base 刚性固定在空中**、**避免一切接触（含腿间接触）**；<br>理由：站立时基座惯量比腿/驱动惯量大 **1~2 个数量级**，空中数据才能**隔离**腿与驱动动力学 |
| 激励频段 | 理想是到 **f_policy/2（Nyquist）**；受结构限制时（ANYmal 只到 **2 Hz**；Tytan/Minimal 到 **10 Hz**），<br>**至少要覆盖运动控制器动作最高频的 2 倍**（例：1 m/s 行走取 1 Hz） |
| 宣传口径 | 3 个平台 + 另部署 **10 台**机器人；**"reliable policy transfer without randomization of dynamic parameters"**；ANYmal 全程 CoT **−32%**（→1.27） |
| 相关工作 | 点名 **Miller et al. (2025)** 与 **SPI-Active (Sobanbabu et al. 2025)** 为同一结构（用行走策略/动作先验采数据 + 分布型/信息型目标） |

**【推断】对我们的直接差距**（对照 `Example60` 与 `docs/SIM2REAL_DATA_FEEDBACK.md` 路径 3）：
我们现在是"单关节、8 s、0.5→15 Hz、离线最小二乘/一阶拟合"；论文是"**全关节同时、20~60 s、位置 chirp、开环回放 + CMA-ES**"。
三处最值得改：**① 时长与"全关节同时"**（能一次辨识出耦合项）；**② 开环位置回放**（消除相位漂移，使轨迹匹配良定）；
**③ 把 delay 与 bias 作为显式待辨识参数**（我们目前靠 Ex38 单独测延迟、Ex47 单独出 b）。

### 2.2 延迟与滤波

- **PACE 把 `delay` 直接作为待辨识参数**（见 §2.1），而不是先测后填 —— 与"分两步"相比更不易互相污染。
- **动作低通滤波**：LocoWheeledLegged（Go2W）在训练与部署**两侧**都加（腿 `fc=5 Hz`、轮 `fc=15 Hz`，`fs=50 Hz`，见 §1.3）。
  目的是**不让策略依赖真机跟不上的高频指令**。我们现在只有轮的 `WHEEL_CMD_ALPHA`，**腿没有动作低通**。
- **多模态延迟随机化**（Multi-Modal Delay Randomization，[arXiv 2109.14549](https://ar5iv.labs.arxiv.org/html/2109.14549)）：
  训练时随机化观测延迟/噪声（面向视觉+本体多模态）。**【推断】** 对无视觉的我们，价值主要在"观测延迟随机化"这一条。
- **`last_action` 历史长度 = 1**（LocoWheeledLegged 原文）用于**避免控制自激**；我们的 obs 里也有 last_action，
  如果出现自激振荡，这是第一个可试的开关。

### 2.3 轮子：速度环 vs 关节级阻尼（**已从源码核实**）

**Go2W 实物部署（[deploy 源码](https://github.com/zhaozijie2022/LocoWheeledLegged/blob/main/deploy_real/deploy_real.py)，
我核对了下载的源码）**：`low_cmd.motor_cmd[i].tau` **恒为 0**；只下发 `kp`/`kd`（`kps`/`kds` 来自 yaml）。
轮子的取值是 **`kp=0`、`kd=0.5`**，`wheel_action_scale=5.0`。
⇒ **轮子在真机上就是 `τ = kd·(ω_des − ω)` 的关节级阻尼跟踪**，与仿真里的 `kd·(w_target − qd)` **同构**。
另外他们在观测里把轮子通道**置零**（`err_obs[wheel_sim_indices]=0`、`qj_obs[wheel_sim_indices]=0`）。

**更精确的读法（重要修正）**：Unitree lowcmd 的语义是 `τ = kp(q_des−q) + kd(dq_des−dq) + τ_ff`。
**当 `kp=0` 时，这个固件 PD 环就退化成 `kd(dq_des−dq)` —— 与 Isaac Lab `IdealPDActuator` 的 `kd(q̇_des−q̇)`
是同一个方程**。所以 LocoWheeledLegged 不是"绕开固件环"，而是**把固件环的参数调成与仿真逐项相等**
（`stiffness=0.0, damping=0.5` ↔ 真机 `kp=0.0, kd=0.5, tau=0`），**因此无需辨识**。
（出处：`locowheeledlegged/assets/go2w.py` L61–70、`deploy_real/deploy_real.py` L326–361、`configs/go2w.yaml`）

**对照我们**：我们的真机轮子走**独立的 SPEED 模式**（`τ = kvp(ω_des−ω) + ki·Dt·Σ`，**带积分**），
仿真却是 `kd(ω_des−ω)`（`kd=2.0`、无积分）—— 这才是结构差异的来源：**不是"固件 vs 阻尼"，而是"PI vs 纯 P"**。
好消息是：我们的驱动 **IMPEDANCE 模式下轮子的下发分支本来就强制 `kp=0`**，形式是
`τ = kd(ω_des−ω) + τ_ff`（见 `src/motor/motor_manager.cpp` 的轮子分支：
`set_motor_para_bt(motor, 0.0f, send_vel_w, 0.0f, motor.kd, send_tau_w, IMPEDANCE)`）
—— **与仿真同构的通道一直存在**（就是被删掉的 `rl::wheel_torque()` 用的那条），
**切换成本≈改模式 + 对齐一个 `kd` 常数**。三条可选路线：

| 路线 | 做法 | 优点 | 代价 |
|---|---|---|---|
| **A：改仿真** | 仿真里把轮子建成速度伺服（`ω̇ = (ω_des−ω)/τ` + 饱和/死区），参数用 Ex60 模式 4 数据拟合 | 保留真机硬件速度环的带宽与积分（无稳态误差） | 要辨识伺服模型；伺服内部对我们是黑箱 |
| **B：改真机（Go2W 路线）** | 轮子从 SPEED 模式改到 **IMPEDANCE 模式 + `kp=0, kd=WHEEL_KD`**，`ω_des = WHEEL_VEL_SCALE·a`，`τ_ff=0`（**代码路径已存在**：`motor_manager.cpp` 的轮子分支 + 被删的 `rl::wheel_torque()`） | 与仿真**逐项同构**（都是纯 P 速度环），结构性 gap 直接消失；**无需辨识**；改动≈改模式 + 统一一个常数 | 失去积分项 ⇒ **斜坡/负载下有稳态速度误差**；失去 20 kHz 内环 ⇒ 跟踪带宽取决于 500 Hz 外环 + 24 ms 延迟 |
| **B′：先判定再选** | 先按子代理建议**实测轮速环闭环频率响应**（ω_des 扫频/chirp，记 ω），判断它是否真等价于 `kd=2.0` 一阶环 | 用数据决定走 A 还是 B，不拍脑袋 | 半天（Ex60 模式 4 已具备采集能力） |
| **C：两边都留，但把 A 做准** | 同 A，但把伺服参数当作显式模型项（含饱和/死区/延迟）一起辨识 | 最贴近真机 | 工程量最大 |

**【原文旁证】** 本项目**曾经就有**路线 B 的实现（已作为死代码删除的 `rl::wheel_torque() = kd·(WHEEL_VEL_SCALE·a − vel)`，
见 `include/strategy/rl_controller.h` 第 130 行附近的历史注释）—— 也就是说我们**从"与仿真同构"改成了"固件速度环"**。
改为速度环大概率是为了更好的跟踪与发热表现，但代价就是现在这个结构性 gap。**【推断】** 值得回头量化对比一次，
而不是默认"固件环一定更好"。

### 2.4 观测/历史/自适应
（待补：RMA / HIM / DreamWaQ / 历史长度）

### 2.5 域随机化的"少而准"
（待补：legged_gym 默认 DR 范围、静态摩擦的影响）
**【原文旁证，1.3】** Go2W 部署侧 `history_length: 6`、`num_obs: 57`（我们：单帧 64 维，无历史）。

### 2.6 真机在线自适应/微调
（待补）

### 2.4 观测/历史/自适应
（待补：RMA / HIM / DreamWaQ / 历史长度）

### 2.5 域随机化的"少而准"
（待补：legged_gym 默认 DR 范围、静态摩擦的影响）

### 2.6 真机在线自适应/微调
（待补）

---

## 3. 与本项目的对照表（已核实部分）

| 手段 | 出处 | 本项目现状 | 建议 |
|---|---|---|---|
| **全关节同时、20~60 s 位置 chirp + 开环回放 + CMA-ES**，显式辨识 delay/bias | [2509.06342](https://arxiv.org/html/2509.06342v2) §2.1 | Ex60 模式5：单关节、8 s、0.5→15 Hz；Ex47 最小二乘 | 升级 Ex60 + 新增 `tool/replay_fit.py`（已在计划里） |
| **真机轮子用关节级阻尼**（`kp=0, kd`）与仿真同构 | [Go2W deploy](https://github.com/zhaozijie2022/LocoWheeledLegged) §2.3 | 真机固件速度环 vs 仿真 `kd` | 二选一（§2.3 表），**先量化对比** |
| **动作低通**（腿 5 Hz / 轮 15 Hz @50 Hz） | 同上 §1.3 | 仅轮有 `WHEEL_CMD_ALPHA` | 低成本，可直接加腿动作低通 |
| **历史观测**（history_length 6） | 同上 §1.3 | 单帧 64 维 | 属"改策略结构"，需重训，优先级低 |
| **腿增益更软**（kp=50/kd=1 vs 我们 250/4） | 同上 §1.3 | 250/4 | 值得在 sim2sim 里做一次对照实验 |
| **`last_action` 历史=1 防自激** | 同上 §1.3 | 未显式约束 | 出现自激时的第一个开关 |
| **不靠 DR、靠辨识** | [2509.06342](https://arxiv.org/abs/2509.06342) | 同方向（Ex47/54/58/59/60） | 方向正确，继续 |
| **接触力别当真值**（GPU 网格地形下不可靠） | [Wheel-Legged-Gym](https://github.com/clearlab-sustech/Wheel-Legged-Gym) README | 与 `CONTACT_PHASE_SIM2REAL.md` 结论一致 | 保持 |

---

## 4. 未核实 / 存疑

- §1.1 我只抓了 arXiv **摘要页**，PMSM 能量模型的具体公式、四项奖励的定义、辨识用的激励信号与优化器
  尚未逐条核对（论文有 HTML 版：https://arxiv.org/html/2509.06342v2 ，代码与数据集公开）。
- §1.3 的 `num_obs: 57` 与 `history_length: 6` 我只按 yaml 原样引用，**未推断其观测分组**。
- Go2W 轮子的实际控制模式（是 `τ=action` 还是 `τ=kd·(ω_des−ω)`）需看其 `deploy_real.py` 源码确认，目前是【推断】。

---

## 附录 A：子代理检索汇总 —— 轮足 / 系统辨识 / 经典控制（**未逐条复核**）

> 下列内容由检索子代理汇总（32 个 URL），**我只复核了其中与本项目直接相关的少数几条**（见正文 §1.3/§2.3 与下面标 ✅ 的条目）。
> 标 ⚠️ 的是子代理自己标注的**未核实/推断**项。请当线索而不是结论使用。

 # 技术简报 B：轮足专用 + 系统辨识/经典控制路线如何缩小 sim2real gap
 
 > 读者：16 电机轮足（4×CAN，hip/thigh/calf/wheel）；RL 策略 50 Hz；电机收发 500 Hz；真机轮子走固件速度环（Dt=50 µs=20 kHz 内环，上位机给 ω_des + kvp/ki）；仿真用 `kd·(ω_des−ω)`（kd=2.0）；无足端力传感器；无视觉；实测腿库仑摩擦 1.6~6.2 N·m；端到端延迟 ≈24 ms。
 > 标注：**【原文】**=论文/README 明确写的；**【代码】**=我直接读了源码某行；**【推断】**=我的推理；**【未核实】**=本次没查到。行号来自本次抓取的 main/master 快照，仓库更新后会漂移。
 
 ## 1. 速览：与本项目最相关的 6 个事实
 
 | # | 事实 | 出处 |
 |---|---|---|
 | 1 | 轮足 RL 项目里轮子几乎一律是**速度伺服**而非力矩：仿真 `τ = kd·(ω_des − ω)`，`ω_des = action × vel_scale` | go2w_rl_gym、LocoWheeledLegged、Wheel-Legged-Gym 三家一致 |
 | 2 | **LocoWheeledLegged 是唯一显式对齐"真机固件速度环"与"仿真 kd 阻尼"的开源 Go2W 项目**：真机 kp=0/kd=0.5/只发 dq；仿真 wheel actuator stiffness=0/damping=0.5 | 【代码】`assets/go2w.py` L61–70、`deploy_real.py` L356–361 |
 | 3 | 三家实际轮子公式均为 `τ = 0.5·(vel_scale·a − ω)`，vel_scale=10（go2w/WLG）或 5（LocoWheeledLegged） | §2 表格 |
 | 4 | **开源轮足项目无一辨识过"轮速环"**；唯一把轮速环当辨识对象的是 ETH ANYmal-on-Wheels 的 `I_t = f(ω_des \| ω_{t−1}, ω_{t−2}, …)` 神经网络 | 【原文】[arXiv 2405.01792](https://arxiv.org/abs/2405.01792) |
 | 5 | **所有开源轮足项目的关节库仑摩擦都是 0**：URDF `dynamics` 被注释、Isaac Lab actuator `friction=0.0` | 【代码】go2w.urdf L418；`assets/go2w.py` L57/L67 |
 | 6 | 系统辨识路线里延迟典型量级 **7.5 ms（PACE/ANYmal）**，且作为**一个全局集中延迟参数**被 CMA-ES 拟合出来 | 【原文】[arXiv 2509.06342](https://arxiv.org/abs/2509.06342) |
 
 ## 2. 轮足专用开源项目逐个核对
 
 ### 2.1 `ShengqianChen/go2w_rl_gym`（Isaac Gym + legged_gym，Go2W 真机）
 
 <https://github.com/ShengqianChen/go2w_rl_gym>
 - **轮子仿真建模**：`legged_gym/envs/go2w/go2w_config.py` L100–113：`control_type='P'`、`stiffness{"foot_joint":40}`、`damping{"foot_joint":0.5}`、`action_scale=0.25`、`vel_scale=10.0`、`decimation=4`；sim dt 0.005 → **50 Hz**（与本项目一致）。`go2w_robot.py` L387–425 `_compute_torques` 把腿/轮强行拆开：`dof_err[:,wheel]=0`、`actions_scaled[:,wheel]=0`、`vel_ref[:,wheel]=actions×vel_scale`，再 `τ = p·(scaled+err) + d·(vel_ref − ω)` → **轮子 `τ = 0.5·(10a − ω)`**（P 项被置零，p=40 不起作用）。【代码】
 - **动作空间**：16 维；腿=位置增量（×0.25+default），轮=期望角速度（×10）。观测里**轮位置置零**（L225–226）；`_reward_dof_vel` 里轮速置零（L913–915）。
 - **真机下发**（`deploy/deploy_real/deploy_real_go2w.py` L343–362）：轮 `i>=12` → `q=a·0.25+q_meas`、`dq=1.0+ω_meas`、`kp=20`、`kd=0.5`、`tau=0`；腿 → `q=default+a·0.25`、`dq=0`、`kp=50`、`kd=1`。`configs/g2w.yaml`：`control_dt=0.02`、轮 `kps=20`、`kds=0.5`。【代码】
 - 【推断，基于 Unitree lowcmd 语义 `τ=kp(q_des−q)+kd(dq_des−dq)+τ_ff`】真机轮子力矩 ≈ `20·0.25·a + 0.5·1.0 = 5a + 0.5 N·m`。→ **仿真 `5a − 0.5ω` vs 真机 `5a + 0.5`：比例项一致，但真机既无 −0.5ω 速度阻尼、又多 +0.5 N·m 常值偏置，语义不等价。**
 - **一致性处理**：**没有**。URDF 里轮关节 `<dynamics damping="0.01" friction="0.01"/>` 被注释（`resources/robots/go2w/urdf/go2w.urdf` L418）→ 全 URDF 无关节摩擦；域随机化只有地面摩擦 `friction_range=[0.25,1.25]`（`base/legged_robot_config.py` L93–94），**无执行器增益/惯量/延迟随机化**；无低通、无延迟建模。**未把轮速环当辨识对象。**
 
 ### 2.2 `zhaozijie2022/LocoWheeledLegged`（Isaac Lab，Go2W sim2real）— **最值得精读**
 
 <https://github.com/zhaozijie2022/LocoWheeledLegged>（Isaac Sim 5.5.1 + Isaac Lab 2.2.1，锁定 commit `c91a125c73`）
 - **轮子仿真建模**（`locowheeledlegged/assets/go2w.py`）：腿 `DelayedPDActuatorCfg(effort_limit=23.5, velocity_limit=30.0, stiffness=50.0, damping=1.0, friction=0.0, min_delay=0, max_delay=15)` L51–60；轮 `DelayedPDActuatorCfg(stiffness=0.0, damping=0.5, friction=0.0, min_delay=0, max_delay=15)` L61–70。`sim.dt=0.005`、`decimation=4` → **50 Hz**（`config/go2w/locomotion_env_cfg.py` L553–555）。【代码】
 - 【推断】Isaac Lab `IdealPDActuator` 公式 `τ=kp(q_des−q)+kd(q̇_des−q̇)+τ_ff`（`isaaclab/actuators/actuator_pd.py` L157）→ **轮子 `τ = 0.5·(ω_des − ω)`**。
 - **动作空间 + 滤波**（`mdp/actions.py`；`locomotion_env_cfg.py` L244–265）：腿 `JointPositionLowPassAction(scale=0.25, control_frequency=50, cut_off_frequency=5.0, order=1)` → α=1−exp(−2π·5/50)≈**0.4665**；轮 `JointVelocityLowPassAction(scale=5.0, control_frequency=50, cut_off_frequency=15.0, order=1)` → α≈**0.8499**。一阶 EMA 作用在模型原始输出上，历史项为滤波后输出，`reset()` 清零。
 - **真机下发**（`deploy_real/deploy_real.py` L326–361；`configs/go2w.yaml`）：上位机**用同一 α 公式再滤一遍**（L327–332，`fc_wheel=15`、`fs=50`）；轮 `q=0.0`、`dq=action×wheel_action_scale(5.0)`、**`kp=0.0`**、`kd=kds[i]=0.5`、`tau=0`；腿 `q=default_real+a·0.25`、`dq=0`、`kp=50`、`kd=1.0`；轮关节顺序置换 `action[13]→motor12, [12]→13, [15]→14, [14]→15`（L348–356）；`control_dt=0.02`。
 - 【推断】kd=0.5 + kp=0 时真机固件环 `τ=kd(dq_des−ω)=0.5(5a−ω)`，**与仿真逐项一致**。这是该仓库最重要的对齐手法。
 - **其它一致性**：`randomize_actuator_gains` 把**全部关节（含轮）**的 stiffness/damping 按 log-uniform `[0.8,1.2]` 缩放（L447–455）；地面摩擦 `static(0.5,1.0)/dynamic(0.5,0.8)`（L380–381）、`friction_combine_mode="multiply"`。**未建模项**：`friction=0.0`（无库仑摩擦），URDF 同样只有被注释的一处 `dynamics`。【代码】
 
 ### 2.3 `clearlab-sustech/Wheel-Legged-Gym`（两足轮腿）及其它
 
 - Wheel-Legged-Gym <https://github.com/clearlab-sustech/Wheel-Legged-Gym>：`wheel_legged_config.py` L50–55 → `pos_action_scale=0.5`、**`vel_action_scale=10.0`**、`stiffness{... "wheel":0}`、**`damping{... "wheel":0.5}`**。【推断】轮子 `τ=0.5(10a−ω)`。**唯一含经典控制变体**：`envs/wheel_legged_vmc/`（VMC 虚拟模型控制，`wheel_legged_vmc.py` 32 KB）。
 - 其它（**仅核实"存在+技术栈"，细节【未核实】**）：Go2w-dreamwaq-RL <https://github.com/ShengqianChen/DreamWaQ_Go2W>（Isaac Gym 前身，含 C++ 部署 `deploy/deploy_real/cpp_go2w/Controller.cpp`）｜Tron1-IsaacGym <https://github.com/limxdynamics/tron1-rl-isaacgym>｜FLORES <https://github.com/ZhichengSong6/FLORES>（含 `unitree_guide_controller` + gazebo 速度配置）｜legged-robots-manipulation(B2W) <https://github.com/aCodeDog/legged-robots-manipulation>｜`LuanDev3/anymal_with_wheels` <https://github.com/LuanDev3/anymal_with_wheels>（ROS/Gazebo 经典栈，`aww_gazebo/config/wheels_velocities.yaml`）｜汇总索引 <https://github.com/XinLang2019/awesome-wheeled-legged>｜RC_WheelLeg <https://github.com/zeitvex/RC_WheelLeg>。
 
 ## 3. 系统辨识 / 执行器建模
 
 ### 3.1 PACE — [arXiv 2509.06342](https://arxiv.org/abs/2509.06342)（Bjelonic/Tischhauser/Hutter，IJRR 2026）
 
 <https://arxiv.org/abs/2509.06342>｜<https://arxiv.org/html/2509.06342v2>｜代码 **<https://github.com/leggedrobotics/pace-sim2real>**｜数据 DOI <https://doi.org/10.3929/ethz-c-000783505>
 - **辨识参数**：每关节 **armature 惯量 I_a**、**粘性阻尼 d**、**库仑摩擦 τ_f**、**零位偏置 q̃_b**，外加**一个全局延迟 T_d**；饱和限幅用厂商规格写死，不参与优化。【原文】
 - **激励与频段**：**固定基座、腿悬空无接触**；**chirp 加在关节位置目标层**、由 PD 跟踪，故真机与仿真被同一参考轨迹驱动；每序列 **20~60 s**；理想应覆盖到 **f_policy/2**，实际结构限制 **ANYmal 2 Hz / Tytan·Minimal 10 Hz**；单驱动台架 **0.1~10 Hz chirp @ 2.5 kHz 记录**；电流环另有 **1~1250 Hz、2 A、25 s** chirp；记录 **400 Hz~10 kHz**。【原文】
 - **数据量**：结论段明写 **~20 s** 空中、仅编码器的轨迹即足够；单驱动 30 组实验（15 开固件前馈补偿/15 关）×5 种负载。【原文】
 - **优化器**：**CMA-ES**（Nomura & Shibata 2024 实现）；理由：目标是"轨迹级、非凸、含摩擦/饱和/延迟等非光滑效应"。**并行回放**：Isaac Gym **N = 4096 个并行环境**，每环境一套参数，把**录下的关节位置目标按后续 RL 的仿真步长回放**。**目标函数**：时间平均关节位置 MSE `ℓ_e=(1/k)Σ‖q_real−q_sim‖²`（Eq.3/4）。**收敛代价**：单平台 **10~24 h**。【原文】
 - **拟合出的延迟**：**ANYmal 与 Tytan 均为 7.5 ms**；单驱动台架通信/控制延迟**微秒级可忽略**。**阻尼量级**：ANYmal **order 5 N·m·s/rad**。**PD 增益**：Tytan 辨识 `P=60 N·m/rad, D=2 N·m·s/rad`，验证改用 `145/5`。【原文】
 - **量化收益**：ANYmal **全 CoT 降低 32%，CoT=1.27**；在 ANYmal 上达到 ActuatorNet 同等保真度，但**数据更少且不需要力矩传感器**。**反直觉**：Tytan LF-HFE 拟合惯量约为预期的 **4 倍**；开固件补偿会引入负载无关虚惯量 → 辨识与部署必须同一固件模式。【原文】
 - **轮足**：**不涉及**（全文 "wheel" 仅 2 次，均在参考文献）。【原文】但方法（关节空间 + 一个集中延迟）可移植到轮速关节【推断】。
 
 ### 3.2 UAN — [arXiv 2502.10894](https://arxiv.org/abs/2502.10894)（Bridging the Sim-to-Real Gap for Athletic Loco-Manipulation）
 
 <https://arxiv.org/abs/2502.10894>｜<https://arxiv.org/html/2502.10894v1>（平台：Unitree B2 + Z1 Pro 臂）
 - **思路**：不用力矩传感器，用 RL 学**修正力矩** `δτ = π_UAN(e)`，目标是**让仿真与真机的关节编码器读数差异最小**。**网络**：2 层 MLP `[128,128]`+**ELU**，每个仿真步执行（**5 ms**），同型执行器共用一个网络。**输入**：**过去 20 步（=100 ms）**的位置误差与速度误差历史（刻意排除其它量以防过拟合惯性耦合）。【原文】
 - **数据**：方波/正弦——**一次只驱动一个执行器**，其余保持位置目标，每个执行器扫 **12 组幅值/频率 ≈ 50 s**；高斯噪声——所有关节同时给，每 **5~400 ms** 重采样，约 **5 分钟**。**训练**：Isaac Sim **4096 并行环境**、RSL-RL 的 PPO（actor 每 epoch 4 个 mini-batch、critic 全 batch）。【原文】
 - **对照**：Default / DR（随机化 PD 增益、摩擦、armature）/ ROA（正则化在线自适应）/ Actuator Net（监督式，力矩标签由电机电流估计，无法捕捉谐波减速器非线性）。**结果**：UAN 的"仿真-真机投掷距离差"最小、真机投掷距离最大（Fig.4）；Actuator Net 能捕捉滞后但在 5 分钟 rollout 上发散。**Fig.4 是柱状图，正文未给数值** →【未核实具体数值】。【原文】
 - **注意**：他们明说电机电流作为力矩代理**不可靠**（非线性摩擦、迟滞、滞后）——这正是"把速度环当辨识对象"的核心难点。
 
 ### 3.3 ANYmal ActuatorNet — [arXiv 1901.08652](https://arxiv.org/abs/1901.08652)（Science Robotics 2019）
 
 <https://arxiv.org/abs/1901.08652>｜<https://arxiv.org/html/1901.08652v1>
 - **输入**：关节**位置误差（实际−指令）与速度的历史**，取 **当前 + t−0.01 s + t−0.02 s**（约 **20 ms 窗口**）。**网络**：MLP **3 隐层 × 32 单元**、**softsign**。**精度**：验证误差 **0.7~0.8 N·m RMS**；12 关节全推理 **12.2 µs（softsign）vs 31.6 µs（tanh）**。【原文】
 - **数据量**：参数化控制器生成**正弦足端轨迹**（幅值 **5~10 cm**、频率 **1~25 Hz**），脚不断触地/离地并人为扰动；**12 执行器并行采集 < 4 分钟**；**400 Hz** 记录 → **>100 万样本**；90% 训练/10% 验证。**训练损失未详述**（监督回归）。【原文】
 - **为什么胜过参数化模型**：执行器有级联内环、不可观内部状态、非线性非光滑耗散；解析模型（Gehring 等 SEA）**近 100 个参数**且实测/数据表拿不全，ANYmal 上**至少 3 周**工作量，而 ActuatorNet 只需 <4 min 数据。**关键准则**：**历史窗口长度必须长于"所有通信延迟 + 机械响应时间"之和**；太稀（>100 Hz 动态）会失效。仿真里执行器网络占约一半算力，混合仿真 ~500K 步/s ≈ 1000× 实时。**部署**：真机载板 200 Hz / 100 Hz。【原文】
 - **后续**：PACE 提到 ANYbotics 后续版本是 **LSTM actuator model**（用作对照基线）【原文 [2509.06342](https://arxiv.org/abs/2509.06342) §3.2.2】；其数据量/结构【未核实】。
 
 ### 3.4 静摩擦 — [arXiv 2503.01255](https://arxiv.org/abs/2503.01255)（Lenovo，Saturn Lite）
 
 <https://arxiv.org/abs/2503.01255>｜<https://arxiv.org/html/2503.01255v1>
 - **为什么静摩擦是主要 gap**：常规域随机化参数空间**不含静摩擦**。作者建单关节控制论模型（Eq.12）并辨识，发现自家机器人摩擦力矩占比异常高：**Go1 的 f/τ_max = 0.13% vs Saturn Lite = 0.98%（约 7.5 倍）**；根因是防水密封圈（拆掉后静摩擦**降低 70%**）。【原文】
 - **怎么测**（Eq.13）：关节给**正弦激励 `θ*(t)=A sin(ωt)`**，**最小二乘**辨识 `I_j,B_j,b_j^c`，带正性约束 `I_j>0,B_j>0,b_j^c>0`；只测 Go1 与 Saturn Lite 的**小腿电机空载**。结果：静摩擦 **0.0481 N（Go1） vs 0.442 N（Saturn Lite）**；粘性摩擦 **0.0342 vs 0.0704 N·m·s/rad**；惯量 **0.0121 vs 0.0145 kg·m²**。【原文】
 - **怎么补**：把静摩擦按 **`[0.0,1.2]` 倍**加进域随机化（**"deception method"：不追求精确对齐，而是显著拉大随机化范围**）。两个替代方案失败：直接把辨识值加入仿真 → 机器人训练时原地不动；对已训练模型**迭代微调**引入静摩擦 → 真机剧烈抖动、站不住。【原文】
 - **完整 DR 表（Table II）**：joint armature `[0.8,1.2]`×、joint damping `[0.8,1.2]`×、joint static friction `[0.0,1.2]`×、Kp `[0.95,1.05]`×、motor strength `[0.8,1.2]`×、ground friction `[0.2,2.0]`×、payload `[-2,3]` kg、CoM `[-0.25,0.25]` m、push interval 8 s、push velocity 1 m/s。**另一条可直接抄**：`k_d` 与粘性摩擦 `B_j` 在动力学里**互相抵消**，只需随机化其一。【原文】
 - **三法对比**（RMA，Webots）：DR-无静摩擦 → 仿真最优但真机只能倒走、前进摔倒；ActuatorNet（仅用"真机倒走"数据训练，H=3 历史步，MSE 力矩损失）→ 真机平地不稳走路、**上不了楼梯**，且 Sim2Sim 直接失效；DR-含静摩擦 → 平地稳定 + 成功上下楼梯。**只测小腿电机，未涉及轮/速度控制关节**。【原文】
 
 ### 3.5 现成 actuator_net / sysid 工具（本次核实到的）
 
 - **MuJoCo sysid**：`google-deepmind/mujoco` → `python/least_squares.ipynb`（<https://github.com/google-deepmind/mujoco/blob/main/python/least_squares.ipynb>）。提供 **`mujoco.minimize.least_squares()`**（Gauss-Newton + Levenberg-Marquardt、box 约束、可自定义残差/雅可比）。notebook 明确写 **"System Identification (sysID) 是主要用例"**（决策变量=模型参数，残差=实测 vs 仿真传感器值），并称 **"完整 sysID 教程与示例将稍后提供"** → 截至 MuJoCo 3.3.0 **没有现成腿式 sysid 示例**；残差评估需 rollout，是最贵的一步。【原文】
 - **Isaac Lab 延迟执行器**：`isaac-sim/IsaacLab` → `source/isaaclab/isaaclab/actuators/actuator_pd.py`（`DelayedPDActuator` L326–367）、`actuator_cfg.py`（`DelayedPDActuatorCfg`）。**把"延迟"显式表达为物理步的整数倍**：`time_lags = randint(min_delay, max_delay+1)`，**每个 env reset 时重采样**，作用在 **joint_positions/velocities/efforts 三个设定值**上（DelayBuffer），之后才进 PD 公式。【代码】
 - **`actuator_net` 独立仓库**：搜到的 `sunzhon/actuator_net` <https://github.com/sunzhon/actuator_net> **README 404** →【未核实】。与 Hwangbo 论文配套的独立公开仓库本次**未找到**；最接近的开放实现是 `pace-sim2real` 与 UAN 的论文描述。**`mujoco_playground` 的 sysid 示例**：【未核实到】。
 
 ## 4. 经典 MPC / WBC 路线的 sim2real 手段
 
 | 项目 | 靠什么对齐 | 有没有把"轮子速度环"纳入模型 |
 |---|---|---|
 | **OCS2** <https://github.com/leggedrobotics/ocs2> | **loopshaping 框架**：`ocs2_core/include/ocs2_core/loopshaping/LoopshapingFilter.h`、`.../dynamics/LoopshapingFilterDynamics.h` 提供 LTI 状态空间 `Filter(A,B,C,D)`，把**执行器/传感器滤波与延迟动力学显式并入最优控制问题**。这是"把延迟写进模型"最正统的开源做法。【代码】 | 与机器人无关，可对任意关节（含轮）挂滤波器；**未核实**有轮足示例 |
 | **legged_control** <https://github.com/qiayuanl/legged_control>（master） | OCS2 的 NMPC + WBC；`legged_estimation/src/LinearKalmanFilter.cpp` 做状态估计；`legged_common/.../HybridJointInterface.h` 做关节力矩接口；README 称 NUC 上 **NMPC 接近 200 Hz** | **未核实到轮关节支持**（README/文件树未见 wheel/go2w）→ **没有把轮速环纳入模型** |
 | **Keep Rollin'** [arXiv 1809.03557](https://arxiv.org/abs/1809.03557) | ZMP 运动优化 + 分层 WBC，WBC 内含**非完整滚动约束** | 轮子当**滚动约束**建模，不是速度伺服 |
 | **Rolling in the Deep** [arXiv 1909.07193](https://arxiv.org/abs/1909.07193) | 在线轨迹优化，问题**拆成轮轨迹规划 + 基座轨迹规划**降低维度 | 轮子是规划变量，不是被辨识的执行器 |
 | **Whole-Body MPC** [arXiv 2010.06322](https://arxiv.org/abs/2010.06322) | 单刚体动力学 + 运动学，**把轮子当移动的地面接触点**，**轮关节速度与地面反力一起作为在线优化变量**，精确表达滚动约束 | **决策变量就是 ω**（速度级），**隐含假设轮子能瞬时达到 ω_des**；不辨识固件速度环带宽【推断】 |
 | **Cheetah-Software / Quad-SDK / unitree_guide** | 公开资料普遍做法是**仿真与真机跑同一套控制/估计代码**（同一控制环、同一估计器、同一力矩标定），用"代码同源"替代"参数辨识"。**本次未逐行核实其延迟补偿/力矩标定细节。** | **未核实**有轮足 + 轮速环建模 |
 
 > **可复用判断**：经典 MPC/WBC 路线**不给轮子建执行器模型**——它把 ω 当决策变量，靠**在线重规划**吸收"固件速度环达不到 ω_des"的误差。对本项目（无足端力传感器）这是**合法但需验证的"不补"选项**。而 RL 前馈策略没有这个重规划回路，所以需要 §6 的对齐手法。
 
 ## 5. 延迟 / 滤波
 
 **5.1 典型量级（仅列本次核实到的数字）**
 
 | 来源 | 延迟量级 | 表达方式 |
 |---|---|---|
 | PACE / ANYmal、Tytan【原文 2509.06342】 | **全局延迟 7.5 ms**，作为**一个**集中参数被 CMA-ES 拟合；单驱动台架通信+控制延迟**微秒级** | 一个加进闭环传递函数的集中纯延迟 `T_d`（Eq.1 的 lumped delay） |
 | MMDR / A1【原文 [2109.14549](https://arxiv.org/abs/2109.14549)】 | **网络推理：仅状态 4 ms；状态+视觉 40 ms**；训练时**本体感觉延迟随机化 `[0,0.04] s`**；评估环境延迟 `0.04~0.12 s` | **观测级延迟缓冲 + 线性插值**（本体感觉）／连续 k 帧内随机取一帧（视觉） |
 | ActuatorNet / ANYmal【原文 1901.08652】 | 不给具体值，给准则：**历史窗口必须长于"所有通信延迟 + 机械响应时间"之和**；实际窗口 20 ms | 用输入历史窗口"吸收"延迟，不显式建模 |
 | LocoWheeledLegged / Go2W【代码】 | `DelayedPDActuator(min_delay=0, max_delay=15)`、`sim.dt=0.005` → **随机化 0~75 ms**，每 env reset 重采样 | **物理步的整数倍**，作用于 pos/vel/effort 设定值 |
 | 本项目实测 | 端到端 **≈24 ms** | — |
 
 > 24 ms **远大于** PACE 在 ANYmal 上拟合的 7.5 ms，且已超过一个 50 Hz 策略周期（20 ms）→ 属于"必须建模"的量级。
 
 **5.2 测量方法（本次核实到的 3 类）**
 1. **把集中延迟当可辨识参数、用轨迹匹配优化出来**（PACE）：chirp 驱动 → 仿真回放同一位置目标 → CMA-ES 同时拟合 `{I_a,d,τ_f,q̃_b}` 与 `T_d`。这是**最省事、最贴合 RL 需求**的做法：得到的是"在闭环里等效的那个延迟"，而非物理链路各段之和。
 2. **分环节实测再随机化**（MMDR）：把延迟拆成处理/计算/控制延迟，只用实测推理时间做锚点，其余靠随机化覆盖。
 3. **频域辨识**（PACE）：**1~1250 Hz、2 A、25 s 电流 chirp** 估电机内环传函；位置 chirp 估整关节闭环。前提是内环带宽足够高且远离饱和（原文明确 Eq.1 是理想化线性模型、忽略饱和）。
 
 **5.3 上位机低通滤波对辨识的污染（本项目直接相关）**
 - **风险**：若辨识数据是"上位机输出 → 低通 → 电机"的闭环信号，而仿真回放时**未复现同一滤波器**，则拟合出的惯量/阻尼/延迟里混入滤波器相位滞后，参数不可迁移。
 - **PACE 规避方式**：把激励加在**关节位置目标层**，并在仿真里**回放录下的同一条位置目标**（"以相同参考轨迹驱动真机与仿真，消除累积相位误差"）→ 让 PD+滤波链路在两边**结构性一致**，而不是去辨识滤波器。【原文 [2509.06342](https://arxiv.org/abs/2509.06342) §2.1】
 - **LocoWheeledLegged 规避方式**：仿真 action term 与上位机**用同一 α 公式、同一组 `control_frequency/cut_off_frequency/order`**（腿 50/5、轮 50/15，order=1）→ **滤波器不是待辨识对象，而是共同已知项**。【代码】
 
 **5.4 在仿真里表达延迟（本次核实到的 3 种形式）**
 
 | 形式 | 出处 | 备注 |
 |---|---|---|
 | **控制/物理周期整数倍** | Isaac Lab `DelayedPDActuatorCfg(min_delay,max_delay)` | 实现最简、与 500 Hz 收发时钟天然对齐；缺点是不能连续调节非整数倍延迟 |
 | **一个连续集中纯延迟参数** | PACE 的 `T_d`（7.5 ms） | 适合"整体等效延迟"；原文说它 lump 了通信与内环延迟 |
 | **观测级缓冲 + 插值/随机选择** | MMDR（本体感觉线性插值、视觉随机取帧） | 适合延迟大于一个控制周期的情形，正对应本项目 24 ms > 20 ms |
 
 > 【未核实】没找到"用一阶滞后替代纯延迟"并给出对比数据的轮足论文；OCS2 loopshaping 的 `Filter` 提供**通用 LTI 状态空间**能力（等价于一阶/任意阶滞后），但**无配套实验对比数字**。
 
 ## 6. 单独一节：真机轮子是固件速度环、仿真是 kd 阻尼，业界怎么处理？
 
 **方案 1（最推荐）：让仿真 kd 就等于真机下发的 kd，并把真机 kp 置 0 —— LocoWheeledLegged**
 - **做了什么**：真机轮子 `q=0, dq=action×5.0, kp=0.0, kd=0.5, tau=0`；仿真轮子 `DelayedPDActuatorCfg(stiffness=0.0, damping=0.5)` + `JointVelocityAction(scale=5.0)`。
 - **【推断】为什么等价**：Unitree lowcmd 固件环语义 `τ=kp(q_des−q)+kd(dq_des−dq)+τ_ff`；`kp=0` 时退化为**纯速度环**，其数学形式与仿真 `IdealPDActuator` 的 `kd(q̇_des−q̇)` **逐项相同**。仿真里的 `damping` 不是"近似固件环"，而是"同一个方程"，无需辨识。
 - **配套**：滤波在仿真与上位机用同一 α（轮 50/15 Hz → α≈0.8499）；`damping` 按 log-uniform `[0.8,1.2]` 随机化吸收固件增益误差/温漂；`DelayedPDActuator(max_delay=15)` 覆盖 0~75 ms 延迟。
 - **出处**：【代码】<https://github.com/zhaozijie2022/LocoWheeledLegged> → `locowheeledlegged/assets/go2w.py` L61–70、`locowheeledlegged/mdp/actions.py`、`deploy_real/deploy_real.py` L326–361、`deploy_real/configs/go2w.yaml`
 - **对本项目的代价**：需先确认**固件 kvp/ki 与你仿真 kd 的对应关系**（本项目是"上位机给 ω_des + kvp/ki"），否则无法直接令 kp=0【推断】。
 
 **方案 2（最彻底）：把"轮速指令 → 力矩/电流"本身当辨识对象 —— ETH ANYmal-on-Wheels**
 - **做了什么**：轮子是准直驱、**没有可靠力矩测量**，于是**学一个神经网络把速度指令 + 历史速度读数映射到电机电流**：
   `I_t = f(ω_target | ω_{t−1}, ω_{t−2}, …)`，再 `τ_t = K_τ·GR·I_t`，并叠加摩擦模型
   `τ_t = K_τ·GR·I_t + τ_friction`，其中 **库仑** `τ_friction,C = −C₁·ω`、**粘滞/静摩擦** `τ_friction,S = −C₂·sgn(ω)`，且 **C₁、C₂ 随机化并放进特权观测**。
 - **【原文】出处**：<https://arxiv.org/abs/2405.01792>（Science Robotics 9(89), adi9641, 2024）"Modeling Actuators" 一节 Eq.4–8。
 - **要点**：他们**明确区分**腿（有 SEA 力矩测量 → ActuatorNet）与轮（无力矩测量 → 速度环网络 + 摩擦模型）。这正是本项目"轮子走固件速度环、无足端力传感器"的对应情形。
 - **代价**：需采集轮子激励数据（速度阶跃/chirp）+ 电流（或力矩估计）标签，比方案 1 重得多。
 
 **方案 3（最省事）：不建执行器模型，只把 kd 与 ω_des 关系当"共同已知项"，靠域随机化兜底**
 - **做了什么**：go2w_rl_gym 固定 `damping foot_joint=0.5`、`vel_scale=10`；Wheel-Legged-Gym 用 `vel_action_scale=10.0`、`damping wheel=0.5`；两家仿真的轮子公式都是 **`τ=0.5(10a−ω)`**。
 - **出处**：【代码】<https://github.com/ShengqianChen/go2w_rl_gym> → `go2w_config.py` L100–113、`go2w_robot.py` L404–425；<https://github.com/clearlab-sustech/Wheel-Legged-Gym> → `wheel_legged_config.py` L50–55。
 - **反面教材（勿照抄）**：go2w_rl_gym 的**真机**下发为 `q=a·0.25+q_meas, dq=1.0+ω_meas, kp=20, kd=0.5`，【推断】等价 `τ=5a+0.5 N·m`，**与它自己仿真的 `5a−0.5ω` 不等价**（少了 −0.5ω、多了常值 +0.5）。这是"仿真与真机用了两套语义"的具体案例。出处：【代码】`deploy/deploy_real/deploy_real_go2w.py` L343–362、`configs/g2w.yaml`。
 
 **方案 4（不补，但合法）：经典 MPC/WBC 把 ω 当决策变量 + 滚动约束，靠在线重规划吸收**
 - **做了什么**：`Keep Rollin'`（[arXiv 1809.03557](https://arxiv.org/abs/1809.03557)）非完整滚动约束 + 分层 WBC；`Rolling in the Deep`（[arXiv 1909.07193](https://arxiv.org/abs/1909.07193)）拆分轮/基座轨迹优化；`Whole-Body MPC`（[arXiv 2010.06322](https://arxiv.org/abs/2010.06322)）把**轮关节速度与地面反力一起**作为在线优化变量。
 - **为什么不追执行器建模**：这些控制器**每个周期都重规划**，速度环偏差被反馈回路吸收；对模型精度的依赖集中在接触/状态估计，而非前馈力矩精度。
 - **对本项目的适用性**：【推断】若你的 RL 策略仍是 50 Hz 前馈，则**没有**这个重规划回路，方案 4 不能直接替代方案 1/2。
 
 **方案 5：把速度环延迟/带宽并进 OCS2 loopshaping 的 LTI 滤波器**
 - **做了什么**：OCS2 提供 `Filter(A,B,C,D)` 与 `LoopshapingFilterDynamics`，把执行器滤波/延迟**显式并入 OCP**。
 - **出处**：【代码】<https://github.com/leggedrobotics/ocs2> → `ocs2_core/include/ocs2_core/loopshaping/LoopshapingFilter.h`、`.../dynamics/LoopshapingFilterDynamics.h`。
 - **适用**：适合把"轮速环 = 一阶/二阶 + 纯延迟"写成状态空间并放进 MPC；对纯 RL 前馈策略，等价做法是把该模型输出接到 action 上。
 
 **对本项目最直接的三处映射（【推断】）**：① 轮子先按方案 1 做等价化——你已知内环 Dt=50 µs 且上位机能给 kvp/ki，建议**实测一次轮速环闭环频率响应**（给 ω_des 扫频/chirp、记录 ω），确认是否真等价于 `kd=2.0` 的一阶环；若不等价再按方案 2 学 `ω_des → τ` 映射。② 腿库仑摩擦 1.6~6.2 N·m —— **没有**一家开源轮足项目建模过（URDF `dynamics` 被注释、Isaac Lab `friction=0.0`、go2w 域随机化只随机地面摩擦）；可照 2503.01255 用正弦激励 + 最小二乘辨识，再把静摩擦按 **`[0,1.2]` 倍** 加进域随机化（deception method），**不要**用"先训练再加摩擦微调"。③ 24 ms 延迟远超 PACE 的 7.5 ms，建议照 PACE 作为**一个集中延迟参数**一起拟合，仿真里先按"控制周期整数倍 + 随机化"实现，再考虑 MMDR 式观测级缓冲 + 插值。
 
 ## 7. 未能核实的点（**不做推测性填充**）
 
 1. **go2w_rl_gym 真机轮子命令的确切物理语义**：`τ=kp(q_des−q)+kd(dq_des−dq)` 是 Unitree SDK 的通用约定，本次**未从 Unitree 官方文档核实**；故"真机 = 5a + 0.5 N·m"是【推断】。
 2. **LocoWheeledLegged 是否做过真机 kd/固件一致性实验**：只读到代码与配置的**结构性对齐**，未见论文/README 给出对比数据。
 3. **ANYbotics LSTM actuator model**（[2509.06342](https://arxiv.org/abs/2509.06342) 用作基线）的数据量、层数、输入窗口 → 未找到公开细节。
 4. **`sunzhon/actuator_net`** 内容（README 404）；**`mujoco_playground` 的 sysid 示例**是否存在。
 5. **Cheetah-Software / Quad-SDK / unitree_guide** 的延迟补偿与力矩标定细节、是否有轮足扩展 → 未核查源码。
 6. **FLORES / Tron1 / RC_WheelLeg / anymal_with_wheels / B2W** 的轮子建模方式、动作空间、真机下发协议 → 未读源码。
 7. **"用一阶滞后替代纯延迟"在轮足上的实验对比** → 未找到带数字的公开结果。
 8. **UAN 的投掷距离具体数值**（[arXiv 2502.10894](https://arxiv.org/abs/2502.10894) Fig.4 为柱状图，正文未给数字）。
 9. **2503.01255 的 `f/τ_max` 单位**：Table III 标 `(%)` 但数值为 0.13/0.98，量纲表述可能有误 → 按原文照抄、不做换算。
 10. **本项目轮速环 kvp/ki 与仿真 kd 的定量对应关系** → 需你侧实测，公开资料无法给出。


---

## 附录 B：子代理检索汇总 —— 学习式 RL 路线（RMA / HIM / DreamWaQ / 执行器建模 / 延迟）

> 由检索子代理汇总（67 个可点击出处），它**逐个 curl/git clone 原仓库代码、pdftotext 抓 arXiv 原文后 grep 核实**，
> 并区分【原文】/【推断】/【未核实】。下列内容我**未逐条复核**（我复核了其中与轮足直接相关的少数几条，见正文 §2.3/§3）。
> ⚠️ **它同时纠正了我文档里的两处错误**（已在正文修掉）：①「总功率上限」在开源栈里**没有实现**；
> ②`legged_gym` 里没有 `action_delay_steps`/`ACTION_DELAY`，也没有我 prompt 里误写的 "legged-robots" 分支。

# 学习式（RL）四足/轮足运控框架：sim2real gap 技术简报

> 读者约束：16 电机（4×hip/thigh/calf/wheel）、策略 50 Hz、电机收发 500 Hz、**轮子为固件速度环**、**无足端力传感器**、MuJoCo 仿真。
> 阅读约定：**【原文】**＝论文/仓库里能直接指到的文字或代码；**【推断】**＝我基于代码结构做的推理；**【未核实】**＝没查到原文，宁缺勿编。
> 所有数值都尽量给到文件/行。仓库链接用 GitHub blob + `#L` 行号；arXiv 链接给 abs 页。

---

## 0. 一句话结论（给"抄谁"用）

| 你关心的 gap | 最值得抄的对象 | 需要足端力传感器？ | 需要重训策略？ |
|---|---|---|---|
| 执行器不真实（力矩-转速、连续力矩、armature） | Isaac Lab `DCMotor` + mjlab `DcMotorActuator` | 否 | 否（只改仿真/部署一致性）；若要吃到收益需重训 |
| 通信/计算延迟 | mjlab 的 actuator delay + obs delay（现成字段）；WTW 常数延迟 | 否 | 是（但只加随机化，不改网络） |
| 接触缺失/地形未知 | HIM（history encoder + 对比学习）、DreamWaQ（β-VAE 隐变量） | 否 | 是（架构级） |
| RMA 的 30 维状态含 4 个接触位 | 不要直接抄 RMA 的观测定义（见 §5） | **RMA 原文要用足端"foot sensors"** | — |
| 轮子（固件速度环） | DreamWaQ_Go2W 的 16 电机配置（Kp=0, Kd=0.5, 速度参考） | 否 | 是 |

## 1. legged_gym / Isaac Lab

### 1.1 legged_gym（Isaac Gym，ETH）【原文】

| 项 | 值 | 出处 |
|---|---|---|
| 摩擦随机化 | `friction_range = [0.5, 1.25]`，先抽 64 个 bucket 再按 env 分配（减少 env 数量带来的显存/时间开销） | [legged_robot_config.py#L121-L123](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot_config.py#L121-L123)、[legged_robot.py#L266-L276](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot.py#L266-L276) |
| 基座质量 | `randomize_base_mass` 默认 **False**；范围 `added_mass_range = [-1., 1.]`（kg，直接加到 base link） | [#L124-L125](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot_config.py#L124-L125) |
| 推力扰动 | 每 `push_interval_s = 15` s 给基座 xy 速度赋 `±max_push_vel_xy = ±1.0 m/s`（**不是力**，是直接改 root state） | [#L126-L128](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot_config.py#L126-L128)、[legged_robot.py#L414-L419](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot.py#L414-L419) |
| 观测噪声 | `noise_scales`: dof_pos 0.01、dof_vel 1.5、lin_vel 0.1、ang_vel 0.2、gravity 0.05、height 0.1；`noise_level=1.0`；实现是 **均匀分布** `(2*rand-1)*noise_scale_vec`，且噪声乘了 `obs_scales` | [#L166-L175](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot_config.py#L166-L175)、[legged_robot.py#L226](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot.py#L226)、[#L455-L478](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot.py#L455-L478) |
| 地形课程 | `move_up = 走够 env_length/2 就升 1 级；没走到"指令速度×episode×0.5"就降 1 级` | [legged_robot.py#L421-L438](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot.py#L421-L438) |
| 控制频率 | `sim.dt=0.005`，`decimation=4` → 50 Hz | [#L186-L188](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot_config.py#L186-L188) |

**重要负面结论【原文】**：legged_gym **没有** `action_delay_steps` / `DCMotor` / 功率上限。全仓没有"动作延迟"实现；执行器就是 `τ = Kp(θ_target−θ) − Kd·θ̇` 再 clip 到 URDF effort（[legged_robot.py#L353-L365](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot.py#L353-L365)）。
**关于"legged-robots 分支"【未核实】**：`leggedrobotics/legged_gym` 的分支只有 `algorithms / dev/pe / gh-pages / master`（GitHub API 查询），**没有** `legged-robots` 分支。你要找的大概是 Isaac Lab。

### 1.2 Isaac Lab

**DCMotor（你给的那条公式，逐字对上了）**【原文】[actuator_pd.py#L203-L310](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab/isaaclab/actuators/actuator_pd.py#L203-L310)：

```
τ_j,max(q̇) = clip( τ_stall·(1 − q̇/q̇_max),  −∞, τ_con )
τ_j,min(q̇) = clip( τ_stall·(−1 − q̇/q̇_max), −τ_con, ∞ )
τ_applied  = clip(τ_computed, τ_min, τ_max)
```
- 代码里 `τ_con = effort_limit`，`τ_stall = saturation_effort`，`q̇_max = velocity_limit`；
- 先把 `q̇` clip 到 `vel_at_effort_lim = velocity_limit·(1 + effort_limit/saturation_effort)`（即曲线与连续力矩交点），再算上下界；
- 配置项只有 `saturation_effort`（[actuator_pd_cfg.py#L42-L49](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab/isaaclab/actuators/actuator_pd_cfg.py#L42-L49)）。**没有总功率上限**（`grep power` 在整个 actuator 模块无命中）。
- 实例数值（可直接对照你的电机）[unitree.py#L32-L45](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_assets/isaaclab_assets/robots/unitree.py#L32-L45)、[#L161-L177](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_assets/isaaclab_assets/robots/unitree.py#L161-L177)：Go1 `effort_limit=23.7, velocity_limit=30.0, saturation_effort=23.7`；Go2 `effort_limit=23.5, saturation_effort=23.5, velocity_limit=30.0, stiffness=25.0, damping=0.5, friction=0.0`；G1 关节 `saturation_effort=180.0`、足 `80.0`。

**"总功率上限"【未核实】**：Isaac Lab / legged_gym / unitree_rl_gym / unitree_rl_mjlab 里都**没有** `Σ|τ·q̇| ≤ P_max` 的实现。我唯一核到的功率相关代码是 Parkour 里**记录**功率（`max_power_per_timestep`，[parkour legged_robot.py#L662-L664](https://github.com/ZiwenZhuang/parkour/blob/main/legged_gym/legged_gym/envs/base/legged_robot.py#L662-L664)），不是限幅。想做电功率限幅，得自己加（【推断】在 MuJoCo 里可作为 clip 层加在力矩输出后）。

**动作延迟（现成 API）**【原文】[actuator_pd_cfg.py#L52-L62](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab/isaaclab/actuators/actuator_pd_cfg.py#L52-L62)、[actuator_pd.py#L310-L365](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab/isaaclab/actuators/actuator_pd.py#L310-L365)：
- `DelayedPDActuatorCfg(min_delay: int, max_delay: int)`，单位是 **physics steps**；`reset()` 时按 env 抽 `torch.randint(low=min_delay, high=max_delay+1)`，对 position/velocity/effort 三个 `DelayBuffer` 设 lag。
- 还有 `ActuatorNetMLPCfg`：Isaac Lab 的 Go1 直接引用了 walk-these-ways 的 actuator net（`network_file=.../unitree_go1.pt, pos_scale=-1.0, vel_scale=1.0, input_order="pos_vel", input_idx=[0,1,2]`，注释写明 "taken from https://github.com/Improbable-AI/walk-these-ways"）。

**官方 velocity 任务的默认 DR（比想象中保守）**【原文】[velocity_env_cfg.py#L150-L226](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/velocity_env_cfg.py#L150-L226)：
- 地面材质：`static_friction_range=(0.8,0.8)`、`dynamic_friction_range=(0.6,0.6)`、`restitution_range=(0.0,0.0)`、`num_buckets=64` → **默认根本没随机摩擦**（是点值）；
- `add_base_mass` 操作 `add`，`(-5.0, 5.0)` kg（Go2 rough 覆盖为 `(-1.0, 3.0)`，[go2/rough_env_cfg.py#L33-L35](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/go2/rough_env_cfg.py#L30-L35)）；
- `base_com`：x/y ±0.05 m、z ±0.01 m；`reset_joints_by_scale`：joint pos `(0.5,1.5)`；`reset_base`：pose ±0.5 m / yaw ±3.14，速度 ±0.5；
- `push_robot`：`interval_range_s=(10.0,15.0)`，xy 各 ±0.5 m/s。

**DR 工具箱（按需开）**【原文】[events.py](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab/isaaclab/envs/mdp/events.py)：`randomize_rigid_body_material`(#L155)、`randomize_rigid_body_mass`(#L286)、`randomize_rigid_body_com`(#L400)、`randomize_rigid_body_collider_offsets`(#L441)、`randomize_physics_scene_gravity`(#L498，重力三轴)、`randomize_actuator_gains`(#L541，stiffness/damping)、`randomize_joint_parameters`(#L652，**joint friction + armature**)、`randomize_fixed_tendon_parameters`(#L838)。
> 对轮足很关键：轮子的 `armature`（折算转子惯量）会直接改"固件速度环"的响应，属于必须打对的量（见 §3.3）。

## 2. walk-these-ways（MIT Imitate Dogs / Margolis）

论文：[arXiv:2212.03238](https://arxiv.org/abs/2212.03238)；代码：[Improbable-AI/walk-these-ways](https://github.com/Improbable-AI/walk-these-ways)。

### 2.1 Actuator network【原文】
- 结构：MLP `in_dim=6 → 2 层 ×32 → out_dim=1`，激活 **softsign**；输入 `joint_pos_err(t,t-1,t-2)` + `joint_vel(t,t-1,t-2)`，输出力矩；Adam `lr=8e-4, eps=1e-8`，batch 128，**100 epochs**，4:1 训练/验证，导出 TorchScript 到 `resources/actuator_nets/unitree_go1.pt`。[utils.py#L78-L110](https://github.com/Improbable-AI/walk-these-ways/blob/master/scripts/actuator_net/utils.py#L78-L110)
- 仿真里替代 PD：`joint_pos_err = dof_pos − (action_scale·action + default) + motor_offsets`（[legged_robot.py#L930-L938](https://github.com/Improbable-AI/walk-these-ways/blob/master/go1_gym/envs/base/legged_robot.py#L930-L938)）；注意 `_compute_torques` 在 decimation 循环**内**每个物理步都调用（[#L74-L75](https://github.com/Improbable-AI/walk-these-ways/blob/master/go1_gym/envs/base/legged_robot.py#L74-L75)）。
- **采集方式/时长/激励信号【未核实】**：仓库只有 `train/eval/utils.py`，数据来自 `go1_gym_deploy` 真机跑次落盘的 `log.pkl`（[deployment_runner.py#L37](https://github.com/Improbable-AI/walk-these-ways/blob/main/go1_gym_deploy/utils/deployment_runner.py#L37)）；**没找到** chirp/sine sweep 专用激励脚本，论文只说"following [22] 训练 actuator network 捕捉 PD 误差与实现力矩的非理想关系"，未给采集分钟数。

### 2.2 延迟【原文 + 推断】
- 【原文】"we identify a latency of around **20 ms** in our system and model this as a **constant action delay** during simulation"（论文 §3.3，PDF 文本 L372-L373）。
- 【原文】代码里是可配的动作延迟缓冲：`lag_timesteps = 6`、`randomize_lag_timesteps = True`（[train.py#L30-L31](https://github.com/Improbable-AI/walk-these-ways/blob/master/scripts/train.py#L30-L31)、[legged_robot_config.py#L269-L270](https://github.com/Improbable-AI/walk-these-ways/blob/master/go1_gym/envs/base/legged_robot_config.py#L269-L270)），`target = lag_buffer[0] + default`（[legged_robot.py#L922-L924](https://github.com/Improbable-AI/walk-these-ways/blob/master/go1_gym/envs/base/legged_robot.py#L922-L924)）。
- 【推断】因为 `_compute_torques` 在 decimation 循环内，`lag_timesteps=6` × `sim.dt=0.005` = **约 30 ms** 动作延迟；与论文写的 20 ms 同量级但不等，两者别混着引。注意这是**固定延迟**（缓冲长度固定），不是随机化延迟。

### 2.3 观测噪声 / DR / 策略【原文】
- 噪声 scales：dof_pos 0.01、dof_vel 1.5、lin_vel 0.1、ang_vel 0.2、imu 0.1、gravity 0.05、contact_states 0.05、height 0.1（[#L382-L393](https://github.com/Improbable-AI/walk-these-ways/blob/master/go1_gym/envs/base/legged_robot_config.py#L382-L393)）。
- 代码 DR（[go1_config.py#L89-L104](https://github.com/Improbable-AI/walk-these-ways/blob/master/go1_gym/envs/go1/go1_config.py#L89-L104)）：base mass `[-1,3]` kg、friction `[0.05,4.5]`、restitution `[0,1]`、CoM `[-0.1,0.1]`、motor strength `[0.9,1.1]`；`Kp/Kd factor` **关掉**（范围备着 `[0.8,1.3]`/`[0.5,1.5]`，[#L258-L262](https://github.com/Improbable-AI/walk-these-ways/blob/master/go1_gym/envs/base/legged_robot_config.py#L258-L262)）；`motor_offset_range=[-0.05,0.05]`（[#L344](https://github.com/Improbable-AI/walk-these-ways/blob/master/go1_gym/envs/base/legged_robot_config.py#L344)）。
- 论文 Table 6：Payload `−1.0~3.0 kg`、Motor Strength `90~110 %`、**Joint Calibration `−0.02~0.02 rad`**、Ground Friction `0.40~1.00`、Restitution `0.00~1.00`、Gravity Offset `±1.0 m/s²`；**只在平地训练、不做地形随机化**（作者明说为让 MoB 研究成立）。
- 策略/估计器：policy `512-256-128` ELU，输入 **30 步历史**（obs、commands、behaviors、previous actions、4 足相位 sin 计时参考）；另有 `256-128` ELU 估计器监督学习预测**机体速度与地面摩擦**，论文自述"没分析这个估计的影响，但可视化有用"（PDF L314-L320）；控制 50 Hz，`kp=20, kd=0.5`。

**可抄点**：① "先做系统辨识再随机化"的取舍哲学（论文原话：直接辨识不变性质可以避免过度保守的 DR）；② 固定 20–30 ms 动作延迟；③ 用 30 步历史 + 监督估计速度/摩擦（不需要足端力）。

## 3. Unitree 官方：unitree_rl_gym / unitree_rl_mjlab

### 3.1 unitree_rl_gym（Isaac Gym）【原文】
- [go2_config.py](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/legged_gym/envs/go2/go2_config.py)：`decimation=4`、`action_scale=0.25`、`stiffness=20`、`damping=0.5`、`soft_dof_pos_limit=0.9`、`base_height_target=0.25`、`torques=-0.0002`、`dof_pos_limits=-10.0`；DR 继承 legged_gym（friction 0.5–1.25、15 s 推一次 ±1 m/s）。
- **全仓 `grep DCMotor` 无命中**（29 个 .py 全查）→ Unitree 的 Isaac Gym 版用的是理想 PD + URDF effort 限幅，**没有力矩-转速曲线**。
- sim2sim→sim2real 一致性做法：`deploy/deploy_mujoco/`（MuJoCo）与 `deploy/deploy_real/` 共用同一份 `config.py` + 同一段 obs 拼装代码，obs 顺序 `[ang_vel*0.25, gravity, cmd*cmd_scale, (qj−default)*dof_pos_scale, dqj*dof_vel_scale, last_action, sin_phase, cos_phase]`（[deploy_real.py#L188-L196](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_real/deploy_real.py#L188-L196)、[deploy_mujoco.py#L114-L115](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_mujoco/deploy_mujoco.py#L114-L115)）；yaml 里显式列 `control_dt=0.02 / leg_joint2motor_idx / kps / kds / default_angles / action_scale / cmd_scale / max_cmd / num_obs`（例：[configs/g1.yaml](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_real/configs/g1.yaml)；注意仓库只带 h1/g1/h1_2 的 yaml）。**没有**滤波、动作延迟、观测延迟的处理。

### 3.2 unitree_rl_mjlab（MuJoCo + mjlab）【原文】
[velocity_env_cfg.py](https://github.com/unitreerobotics/unitree_rl_mjlab/blob/main/src/tasks/velocity/velocity_env_cfg.py)：`timestep=0.005`、`decimation=4`（50 Hz）。
- 观测噪声（逐项 `UniformNoise`）：base_ang_vel `±0.2`、projected_gravity `±0.05`、joint_pos `±0.01`、joint_vel `±1.5`、height_scan `±0.1`（再 `scale=1/5.0`）；critic 额外有 base_lin_vel `±0.5`、foot_height、foot_air_time（L59-L105）。
- DR 事件（L209-L252）：`push_robot` 每 `(5.0, 6.0)` s，x/y `±0.5 m/s`、z `±0.4`、roll/pitch `±0.52 rad`、yaw `±0.78 rad`；`foot_friction` 足底 geom 摩擦 `(0.3, 1.6)` 且四足共享同值；**`encoder_bias` 编码器零位偏差 `±0.015 rad`**；`base_com` 三轴各 `±0.05 m`。
- 执行器（[go2_constants.py#L40-L66](https://github.com/unitreerobotics/unitree_rl_mjlab/blob/main/src/assets/robots/unitree_go2/go2_constants.py#L40-L66)）：hip/thigh `stiffness=20, damping=1.0, effort_limit=23.5, armature=0.01`；calf `stiffness=40, damping=2.0, effort_limit=45, armature=0.02`。G1 用 `ElectricActuator(reflected_inertia=..., velocity_limit=37/32/20/22, effort_limit=25/88/139/5)`，Kp 由 `armature·(2π·10)²`、Kd 由阻尼比 2.0 推出来（[g1_constants.py#L100-L126](https://github.com/unitreerobotics/unitree_rl_mjlab/blob/main/src/assets/robots/unitree_g1/g1_constants.py#L100-L126)）——**这套"由反射惯量 + 10 Hz 固有频率 + 阻尼比 2.0 反推 PD"的做法，对轮子的速度环特别值得抄**。
- **默认没开延迟**：`grep delay_min_lag|delay_max_lag` 在 `unitree_rl_mjlab/src` 无命中——即官方 mjlab 任务默认不做延迟随机化，但**框架支持**（见 §3.3）。

### 3.3 mjlab 框架的延迟工具（最现成、最省事）【原文】
- **执行器延迟（动作延迟）**：任何 actuator cfg 都能加 `delay_min_lag / delay_max_lag / delay_hold_prob / delay_update_period`（单位 **physics steps**，每 env 每 step 从 `[min,max]` 抽 lag）。文档原文："板载 PD 以 kHz 跑并有编码器直读，但来自策略的位置目标因推理时间与总线周期**到得晚**；actuator delay 建模这一点——**命令目标被延迟，但控制律仍看到新鲜关节状态**"，与"观测延迟（旧状态进策略）"是往返的两条腿。示例超参：`delay_min_lag=2, delay_max_lag=5, delay_hold_prob=0.3, delay_update_period=10`；换算例子"500 Hz 物理（2 ms/step）时 `delay_min_lag=2` 即 4 ms"（[docs/source/actuators.rst](https://github.com/mujocolab/mjlab/blob/main/docs/source/actuators.rst)）。
- **观测延迟**：按 obs term 配 `delay_min_lag/delay_max_lag/delay_per_env/delay_hold_prob/delay_update_period/delay_per_env_phase`（[observation_manager.py#L35-L55](https://github.com/mujocolab/mjlab/blob/main/src/mjlab/managers/observation_manager.py#L35-L55)）；底层 `DelayBuffer` 支持"均匀抽 lag / 周期重抽 / 每 env 相位错开 / hold 概率制造时间相关性"（[delay_buffer.py](https://github.com/mujocolab/mjlab/blob/main/src/mjlab/utils/buffers/delay_buffer.py)）；处理管线顺序 `compute → noise → clip → scale → delay → history`（L21）。

## 4. HIM/HIMLoco 与 DreamWaQ（纯本体感受补偿状态估计误差与地形）

### 4.1 HIM/HIMLoco
论文：[arXiv:2312.11460](https://arxiv.org/abs/2312.11460)；代码：[InternRobotics/HIMLoco](https://github.com/InternRobotics/HIMLoco)。
- 【原文】观测只有本体感受（关节编码器 + IMU）：`o_t = [指令速度, 关节位置, 关节速度, 角速度, 重力方向, 上一动作]`；**value/critic 才用特权信息**（外加力 + 地面高度）（PDF L240-L250、L406）。
- 【原文】history **H = 5**；extractor 是 3 层 MLP `512-256-128`；embedding = 显式速度 `v̂_t` + 隐式响应 `l̂_t`，`l̂_t ∈ R^16`，用 **SwAV 式对比学习**（同轨迹为正样本）对齐后继观测 `o_{t+1}`，同时用 ground truth 回归 `v̂_t`（PDF L293-L320）。
- 【原文】训练：4096 envs、rollout 100 steps、1000 rollouts ≈ 单卡 4090 **1 小时**（2000 rollouts 更好）；Table 1 里自报 200 M samples（RMA 1,280 M）。
- 【原文】DR 表（PDF Table 6）：body/link mass `0.8–1.2×`、CoM `±0.1 m`、payload `−1~3 kg`、地面摩擦 `0.2–2.75`、restitution `0–1`、motor strength `0.8–1.2×`、Kp `0.8–1.2 × 20`、Kd `0.8–1.2 × 0.5`、初始关节位置 `0.5–1.5×`、**System Delay `[0, 3Δt]`**、外力 `±30 N` 三轴。
- 【原文】地形课程：走到线性跟踪奖励的 **80%** 升级；楼梯 `5 + 18·level/9` cm、离散步高 `5 + 10·level/9` cm，比例 0.1/0.2/0.6/0.1。
- 【原文】真机对比（Table 2，各 20 次试验）：楼梯短程成功率 **100% vs RMA 60%**；长程楼梯数 **176.5±7.81 vs 75.35±19.98**；未见地形 **85% vs 45%**；可变形斜坡 **55% vs 10%**；缺台阶 **100% vs 0%**。
- 【原文】HIMLoco 代码里的延迟实现（**与论文的 [0,3Δt] 不同**）：每个策略步、每个 env 抽 `delay_steps = randint(0, decimation)`，`delayed_actions[:, i] = last_actions + (actions − last_actions)·(i >= delay_steps)`，即延迟量化到 physics step（dt=5 ms → **0~15 ms**）（[legged_robot.py#L91-L99](https://github.com/InternRobotics/HIMLoco/blob/main/legged_gym/legged_gym/envs/base/legged_robot.py#L91-L99)）；DR 代码（[legged_robot_config.py#L124-L160](https://github.com/InternRobotics/HIMLoco/blob/main/legged_gym/legged_gym/envs/base/legged_robot_config.py#L124-L160)）：payload `[-1,2]`、CoM `±0.05`、link mass `[0.9,1.1]`、friction `[0.2,1.25]`、motor strength `[0.9,1.1]`、Kp/Kd `[0.9,1.1]`、初始关节 `[0.5,1.5]`、外力 `±30 N` 每 8 s、push 每 16 s、`delay = True`。

### 4.2 DreamWaQ
论文：[arXiv:2301.10602](https://arxiv.org/abs/2301.10602)；轮足版代码：[ShengqianChen/DreamWaQ_Go2W](https://github.com/ShengqianChen/DreamWaQ_Go2W)（另有 [DreamWaQ++](https://arxiv.org/abs/2409.19709)，**未读，未核实**）。
- 【原文】CENet = context-aided estimator：从观测历史 `o_{t−H:t}`（**H = 5**）**同时**估计机体速度 `v_t`(3 维) 与隐地形上下文 `z_t`（latent）；`L_CE = L_est + L_VAE`，`L_est = MSE(ṽ,v)`，`L_VAE = MSE(õ_{t+1}, o_{t+1}) + β·D_KL`（β-VAE，[PDF L192-L220](https://arxiv.org/pdf/2301.10602)）。
- 【原文】策略输入 `o_t + v_t + z_t`，**不需要特权信息**；critic 用 `s_t = [o_t, v_t, d_t(扰动力), h_t(高度扫描)]`——即"地形想象力"只在训练时通过 VAE 重建/隐变量进入 actor。
- 【原文】DR 表（Table II）：payload `[−1,2] kg`、Kp factor `[0.9,1.1]`、Kd factor `[0.9,1.1]`、motor strength `[0.9,1.1]`、CoM shift `[−50,50] mm`、friction `[0.2,1.25]`、**System delay `[0.0,15.0] ms`**。
- 【原文】部署：CENet 与策略**同步 50 Hz** 跑在板上 Intel NUC；PD 200 Hz `Kp=28, Kd=0.7`；训练 4096 envs × 1000 iterations。
- 【原文】鲁棒性量化：最大推力 `0.714±0.096 m/s`、存活率 `82.37±2.49%`，对比 EstimatorNet 基线 `0.511±0.053` / `20.51±6.44%`（Table III）。Fig.5 给了 CENet vs EstimatorNet 的估计误差曲线（**具体数值在图里，未核实**）。
- 【原文】官方实现细节（Go2W 仓库）：actor `512-256-128`，actor 输入维 = `num_obs + 16(latent) + 3(vel)`；VAE encoder 用 history encoder，输出 `num_latent*4=64` 再分出 `latent_mu/var(16)` 与 `vel_mu/var(3)`，decoder `[64,128] → num_obs`（[actor_critic_DWAQ.py#L1-L40](https://github.com/ShengqianChen/DreamWaQ_Go2W/blob/main/rsl_rl-1.0.2/rsl_rl/modules/actor_critic_DWAQ.py#L20-L40)、[estimator.py#L8-L45](https://github.com/ShengqianChen/DreamWaQ_Go2W/blob/main/rsl_rl-1.0.2/rsl_rl/modules/estimator.py#L8-L45)、[on_policy_runner.py#L61-L70](https://github.com/ShengqianChen/DreamWaQ_Go2W/blob/main/rsl_rl-1.0.2/rsl_rl/runners/on_policy_runner.py#L61-L70)）。

### 4.3 DreamWaQ_Go2W：**16 电机 + 轮子**，最贴近你们（强烈建议细读）【原文】
- [go2w_config.py](https://github.com/ShengqianChen/DreamWaQ_Go2W/blob/main/legged_gym/legged_gym/envs/go2w/go2w_config.py)：`num_actions=16`、`num_observations=73`、`num_obs_hist=5`、`num_privileged_obs=320`、`decimation=4`；`stiffness={'hip':40,'thigh':40,'calf':40,'foot':0}`、`damping={... 'foot':0.5}`、`action_scale=0.25`、`vel_scale=10.0`、`wheel_armature_add`、reward `wheel_acc = -1e-7`；DR：payload `[-1,2]`、CoM `±0.05`、friction `[0.25,1.25]`、motor strength `[0.9,1.1]`、Kp/Kd `[0.9,1.1]`、初始关节 `[0.5,1.5]`、外力 `±30 N`。
- **轮子怎么变成速度环**（正是你们的固件速度环）：`actions_scaled[wheel]=0`、`dof_err[wheel]=0`、`vel_ref[wheel] = actions*vel_scale`，力矩 `τ = Kp·(...) + Kd·(vel_ref − q̇)`（[go2w_robot.py#L470-L490](https://github.com/ShengqianChen/DreamWaQ_Go2W/blob/main/legged_gym/legged_gym/envs/go2w/go2w_robot.py#L470-L490)）；真机 yaml 里轮子 `kps=0, kds=0.5`（[configs/g2w.yaml](https://github.com/ShengqianChen/DreamWaQ_Go2W/blob/main/deploy/deploy_real/configs/g2w.yaml)）。
- **部署一致性措施（很具体，建议照抄）**：yaml 同时给 `default_sim_angles` 与 `default_real_angles`（仿真/真机零位差异显式映射）、`joint2motor_idx`、`wheel_real_indices / wheel_sim_indices`（左右轮序不同）、`wheel_speed`、逐项 obs scale（`lin_vel_scale=2.0, ang_vel_scale=0.25, cmd_scale=[2,2,0.25], dof_err_scale=1.0, dof_vel_scale=0.05`）、`control_dt: 0.02`。
- 真机 obs 拼装与仿真一致的顺序：`[ang_vel*0.25(3), gravity(3), cmd*cmd_scale(3), (qj−default_sim_angles)(16, 轮子置 0), dqj*0.05(16), qj(16, 轮子置 0), last_action(16)] = 73`（[deploy_real_go2w_DWAQ.py#L244-L265](https://github.com/ShengqianChen/DreamWaQ_Go2W/blob/main/deploy/deploy_real/deploy_real_go2w_DWAQ.py#L244-L265)）。

## 5. RMA（Rapid Motor Adaptation）— 关键：它**依赖**足端接触

论文：[arXiv:2107.04034](https://arxiv.org/abs/2107.04034)。
- 【原文】两阶段：① base policy `π(x_t ∈ R^30, a_{t−1} ∈ R^12, z_t ∈ R^8)` 跑 100 Hz，`z_t = μ(e_t)`，`e_t ∈ R^17`（质量 3 + 电机强度 12 + 摩擦 1 + 局部地形高度 1），π 与 μ 端到端用 model-free RL 联合训；② adaptation `φ` 用 **k=50（0.5 s）** 的 `(x, a)` 历史预测 `ẑ_t`，1-D CNN，MSE 损失，**on-policy**（随机初始化 φ 后自我 rollout 采数据，类似 DAgger）。
- 【原文】异步部署：φ 约 **10 Hz** 更新 `ẑ`，π 100 Hz 消费最新 `ẑ`；论文说这个异步"对无缝部署至关重要"（并说直接吃历史的单体策略 (a) 步态不自然 (b) 板上只能跑 10 Hz）。
- ⚠️ **足端力传感器的坑（对你们最关键的一条）**【原文】：`x_t ∈ R^30` = 12 关节位置 + 12 关节速度 + roll/pitch + **4 个二值足端接触指示**；硬件段明确写 "roll and pitch from the IMU sensor and the **binarized foot contact indicators from the foot sensors**"（PDF L248-L250、L269）。→ **RMA 论文没有给出"无足端力传感器"时的替代方案**，接触位是输入的一部分；【未核实】A1 上这 4 个接触位具体如何取得（论文只写 foot sensors，我没找到"A1 是否原生带足力传感器"的权威说明）。→ 可行替代（【推断】，非原文）：用 HIM/DreamWaQ 的 history encoder 从本体感受**估计**接触概率，或去掉 4 个接触位改由速度估计器 + 隐变量承担；但**不能拿 RMA 论文的参数直接宣称等价**。
- 【原文】DR 训练/测试范围（Table I）：friction `[0.05,4.5]`/`[0.04,6.0]`；Kp `[50,60]`/`[45,65]`；Kd `[0.4,0.8]`/`[0.3,0.9]`；payload `[0,6]`/`[0,7]` kg；CoM `±0.15`/`±0.18` cm；motor strength `[0.90,1.10]`/`[0.88,1.22]`；re-sample probability `0.004`/`0.01`。
- 【原文】奖励 10 项与权重：Forward `min(vx,0.35)`、Lateral/Rotation `−‖vy‖²−‖ω_yaw‖²`、Work `−|τ·(q−q_{t−1})|`、Ground Impact `−‖f_t−f_{t−1}‖²`、Smoothness、Action Magnitude、Joint Speed、Orientation、Z Accel、Foot Slip `−‖diag(g_t)·v_f‖²`；权重 `20, 21, 0.002, 0.02, 0.001, 0.07, 0.002, 1.5, 2.0, 0.8`；惩罚系数与扰动难度用固定课程加大；**地形不设课程**（fractal octaves=2, lacunarity=2.0, gain=0.25, z-scale=0.27）；终止：高度 <0.28 m、roll >0.4 rad、pitch >0.2 rad。
- 【原文】真机结果（Fig.3）：Uneven Foam 80%（A1 自带控制器 20%，去掉 adaptation 0%）、Upward Incline 100%、Mattress 100%、Step Down-15 100%、Step Up-6 100%、Step Up-8 60%；负载扫描到 12 kg。

## 6. Parkour / Barkour / DeepMind 风格

### 6.1 Extreme Parkour【原文】[arXiv:2309.14341](https://arxiv.org/abs/2309.14341)
- **ROA（regularized online adaptation）** 单阶段自适应 + **MTS（mixture of teacher and student）** 缓解模仿学习的分布漂移；学生用 convnet-GRU 吃深度图替代 privileged scandots；`θ_obs = θ_pred if |θ_pred − d̂_w| < 0.6 else d̂_w`。
- 延迟：深度相机 `10 ± 2 Hz`，**强制常数深度延迟 0.08 s**（t_p < 0.08 就 sleep 补齐），**本体感受延迟固定 0.016 s**；深度 backbone 10 Hz + base policy 50 Hz，UDP；单卡 3090 < 20 h。
- 真机验证：每种地形每难度 **5 次试验**记录成功率，最难地形比基线高 20–80%；Table 3：本方法 `MXD 0.92±0.19`、Oracle `0.94±0.19`、Both `0.12±0.07`、Mask `0.05±0.07`。**DR 细节正文未展开、附录未核到 →【未核实】**。用仿真足端接触计数做 `r_clearance` 惩罚防踩边缘（[PDF L264](https://arxiv.org/pdf/2309.14341)），与力传感器无关。

### 6.2 Robot Parkour Learning【原文】[arXiv:2309.05665](https://arxiv.org/abs/2309.05665)
- **软/硬动力学约束地形课程**：先在可穿透（soft dynamics）约束下预训练每个技能，再用硬约束微调；特权物理信息 `e_t` 用 RMA/ROA 式采样（地形摩擦、基座 CoM、电机强度等）。
- DR 表（PDF Table 9）：Added Mass `[1.0,3.0] kg`、CoM x `[−0.05,0.15]`/y `[−0.1,0.1]`/z `[−0.05,0.05]` m、Friction `[0.5,1.0]`、Motor Strength `[0.9,1.1]`、**Forward Depth Latency `[0.2,0.26] s`**、相机位置 `0.27±0.01/0.0075±0.0025/0.033±0.0005 m`、pitch `[0,5]°`、FOV `[85,88]°`、**Proprioception Latency `[0.0375,0.0475] s`**；每技能 100 次试验 × 3 seeds 报成功率，蒸馏用 GRU + DAgger。

### 6.3 Barkour（Google DeepMind）【原文】[arXiv:2305.14654](https://arxiv.org/abs/2305.14654)
- DR 表（Table II）：Torso mass `[2.0,6.5] kg`、**Torso inertia `[40%,165%]`**、Torso 线速度扰动 `[0,1] m/s` **每 10 s**、Ground friction `[0.5,1.25]`、**Position gain `[15,20] N·m/rad`**、Damping gain `[0.5,0.75] N·m·s/rad`、**Joint static friction `[0,0.7]`**。
- **最有价值的一条经验结论**【原文】：Rudin 那套默认 DR 在 <1 m/s 够用，但**敏捷动作（跳、爬坡，>2 m/s）会出现明显 sim2real gap**；补上 **torso inertia + motor modeling + joint static friction** 才成功迁移，作者说这些"对真机迁移是 critical 的"。
- 真机：T-Motor **AK80-6**（Elmo G-SOLTWIR50/100SE2S、24 V、峰值 **12 N·m/关节**）；专家策略每障碍 70 次试验、generalist Transformer 19 次；Weave Poles **100%**（9.27±0.87 s）、A-Frame **100%**（7.95±0.73 s）、Broad Jump **38%**（2.45±0.54 s / 1.7±0.24 m/s）；蒸馏数据集 **17,636 episodes ≈ 57.58 h**。

## 7. 多模态延迟随机化 MMDR（arXiv 2109.14549）【原文】

论文：[Vision-Guided Quadrupedal Locomotion in the Wild with Multi-Modal Delay Randomization](https://arxiv.org/abs/2109.14549)（Imai, Zhang, Zhang, ... Xiaolong Wang）。

**真机测得的延迟（Table I/II，最能直接抄的量）**

| 环节 | 延迟 |
|---|---|
| 深度相机 sensor | `0.033 ± 0.004 s` |
| 关节状态 / IMU sensor（同一 SDK 进程，与执行延迟同频） | `0.0025 ± 0.001 s` |
| 网络推理（state only） | `0.004 ± 0.026 s` |
| 网络推理（state + vision，用作随机化依据） | `0.040 ± 0.009 s` |

**做法**
- 本体感受：**每个 episode 抽一个延迟**，用两个相邻状态的**线性插值**得到延迟观测（本体感受平滑），全 episode 使用 → 与真机同样的状态转移。视觉：深度相机只有 30 Hz（本体感受 ~1 kHz），帧间不连续，所以**离散**随机化——维护长度 **4k** 的深度图缓冲、切成 4 个子缓冲、**每子缓冲随机取 1 帧**堆成 4 帧输入；k=4 最好（消融 4/8/16 → 0.64/1.28/2.56 s 历史）。仿真 400 Hz、控制 25 Hz。
- 其他 DR（Table III）：Mass `[0.8,1.2]×`、Motor Friction `[0,0.05] Nms/rad`、Motor Strength `[0.8,1.2]×`、Lateral Friction `[0.5,1.25] Ns/m`、Inertia `[0.5,1.5]×`、**Proprioception Latency `[0,0.04] s`**、Kp `[40,90]`、Kd `[0.4,0.8]`；深度图随机挑 3–30 像素置为最大深度（10 m）模拟缺失。

**量化收益**
- 测试延迟 0.04–0.12 s：Moving Distance `MMDR 28.7±7.7` > `No-Delay 26.5±5.0` > `Frame-Extract 24.9±3.1` > `Interpolation 21.4±2.7` > `Fixed-Delayed 18.5±0.8` > `State-Only 2.9±0.5`（Fig.7）。
- 动态障碍环境：Moving Distance 比 No-Delay 提升近 **100%**，Collision Steps 降低 **475%**（对 Frame-Extract 340%）；训练样本效率与 No-Delay 持平（"随机化延迟几乎不损性能"）。

## 8. 真机在线自适应/微调代表工作

### 8.1 Smith et al., "Learning to Walk in the Real World with Minimal Human Effort"【原文】[arXiv:2002.08550](https://arxiv.org/abs/2002.08550)
- 机器人 Minitaur（8 个直驱电机），**观测 = 电机角度 + IMU + 前 6 个时间步的上一动作**；从非实时 Linux 工作站（Xeon E5-1650 V4）以 **约 50 Hz** 直接控制；每个控制步做 **2 次梯度更新**。
- PD 增益极低：`0.5 / 0.005`；动作额外过 **5 Hz 一阶 Butterworth 低通**（论文明确理由是"减少随机探索造成的电机磨损"）。
- 算法：**安全约束 MDP + SAC**，对熵与安全约束各用一个拉格朗日乘子做对偶梯度下降；网络 2×256 ReLU，Adam lr `3e-4`。
- 自动化 reset：多任务（前进/后退/转向）+ 学习 reset 控制器 + 工作空间边界处理；论文报"平地 **零人工 reset**"（对比先前工作 100+ 次人工 reset），挑战地形（床垫 200k steps 5.5 h / 150k steps 4.5 h）仍需 20–30 次人工 reset；两组策略 1.5 h ≈ 60k steps。
- **注意**：这是纯真机 RL（仿真只用于分析），和你们"MuJoCo 里训好再上真机"是不同的路线；但它给的"50 Hz + 5 Hz 动作低通 + 低 PD 增益 + 板载奖励"是可借鉴的安全措施。

### 8.2 其它
- RMA / HIM / DreamWaQ / Extreme Parkour 都是 **sim 训练、零样本迁移、真机不微调**（RMA 原话："deploy it in the real world without any modification or fine-tuning"）。
- 【未核实】我**没有**找到"针对轮足、在真机上做在线自适应/微调"的代表工作（只核到仿真侧的 Go2W）。

## 9. 优先级建议（针对"16 电机 + 固件速度环轮子 + 无足端力 + MuJoCo"）

**先做（不需要足端力传感器、不需要重训策略）**
1. **执行器模型**：把 Isaac Lab `DCMotor` 的 τ–ω 曲线 + 连续力矩 clamp 搬进 MuJoCo 的力矩输出层（[公式与代码](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab/isaaclab/actuators/actuator_pd.py#L203-L310)）；用 `saturation_effort(τ_stall)/velocity_limit(q̇_max)/effort_limit(τ_con)` 三个数标定你的 hip/thigh/calf/wheel。**Sim2sim 与 sim2real 都用同一份参数**，这一步不需要重训。
2. **轮子的反射惯量**：mjlab 的 `armature`（Go2 hip `0.01`、calf `0.02`）+ DreamWaQ_Go2W 的 `wheel_armature_add`；轮子是固件速度环，仿真里若不给 armature，轮速响应会过快、策略会学到假的速度环。**这是最容易漏、代价最大的一条**（【推断】）。
3. **部署一致性清单**：照抄 Go2W yaml 的 `joint2motor_idx / default_sim_angles vs default_real_angles / wheel_sim_indices / 逐项 obs scale / control_dt`，并把 obs 拼装代码在 sim2sim(MuJoCo) 与真机之间**共用同一份**（unitree_rl_gym 的 deploy_mujoco/deploy_real 就是这么做的）。

**再重训一次就能吃到的收益（仍不需要足端力）**
4. **延迟随机化**（mjlab 现成字段）：`actuator delay_min_lag/delay_max_lag`（单位=物理步）+ obs delay `delay_min_lag/max_lag/hold_prob/update_period`。数值起点：DreamWaQ `0–15 ms`、HIM 论文 `[0, 3Δt]`（50 Hz → 0–60 ms）、WTW 固定 20–30 ms、EP 深度 80 ms/本体 16 ms、Robot Parkour 本体 `37.5–47.5 ms`。你们 500 Hz 电机 → 1 物理步 = 2 ms，`delay_min_lag=0, max_lag=15` 覆盖 0–30 ms。
5. **DR 组合**（跨论文共识）：`encoder_bias ±0.015 rad`（mjlab）、motor strength `0.9–1.1`、Kp/Kd `0.9–1.1`、payload `−1~3 kg`、CoM `±0.05 m`、friction `0.2–1.25`、base mass `±1~5 kg`、外力 `±30 N`、每 5–15 s 推一次 `±0.5 m/s`。Barkour 的教训：高速/敏捷动作要额外加 **torso inertia、motor modeling、joint static friction**。
6. **估计器路线选一个**：HIM（H=5，MLP 512-256-128，latent 16 + 对比学习）或 DreamWaQ（H=5，β-VAE，latent 16 + 显式速度回归，50 Hz 板上同步跑）。两者都**只要本体感受**、都**不需要接触传感器**；HIM 的真机对比数字最全（§4.1）。

**不要直接抄**
7. RMA 的观测定义（含 4 个足端接触位 + 论文明确写了 foot sensors）；若要 RMA 式自适应，必须把接触位换成估计量并重新验证（论文未给该替代方案）。

## 10. 未核实清单（按"缺失的原文"列）

1. `leggedrobotics/legged_gym` 的 **"legged-robots 分支"不存在**（分支只有 algorithms/dev/pe/gh-pages/master，GitHub API 查询）。你需要的应是 Isaac Lab。
2. **"总功率上限"**（Σ|τ·q̇| ≤ P_max）在 Isaac Lab / legged_gym / unitree_rl_gym / unitree_rl_mjlab **都没找到实现**；legged_gym 也没有 `action_delay_steps` / `ACTION_DELAY`。我只核到 Parkour 的功率**记录**。若你手上有具体某个仓库带这个公式，需要再给线索。
3. **walk-these-ways actuator net 的数据采集时长与激励方式**（chirp/sine sweep/随机走动）在论文正文与仓库里都没写；仓库只有 `train.py/eval.py/utils.py`，数据来自真机部署落盘的 `log.pkl`。采集分钟数：未核实。
4. **RMA 在 A1 上如何取得 4 个二值接触位**：论文文字写 "foot sensors"，但没有说明传感器型号/是否原生；我无法确认 A1 是否自带足端力传感器。**这条直接决定你们能不能"照抄 RMA 观测定义"——目前判定为不能。**
5. Extreme Parkour 的 **DR 细节/附录**未核到（正文没有 DR 表）；DreamWaQ Fig.5 的**具体估计误差数值**只在图里，未核实；DreamWaQ++（2409.19709）只有标题/链接，未读。
6. Isaac Lab 的 `DelayedPDActuatorCfg` 我只确认了 API 存在与实现机制；**官方 velocity 任务是否默认启用**，我只检查了 `unitree.py / go1 / go2 / anymal_d` 的配置（未命中），**没有做全仓搜索**。
7. 我**没有**找到"轮足机器人在真机上做在线自适应/微调"的代表工作（只核到仿真侧 Go2W）。
