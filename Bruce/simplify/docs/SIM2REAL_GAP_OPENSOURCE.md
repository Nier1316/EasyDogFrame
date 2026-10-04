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
