# 在我们这条狗上复现 PACE（论文 → 可执行方案）

> 依据：Bjelonic, Tischhauser, Hutter, *Towards bridging the gap: Systematic sim-to-real transfer for
> diverse legged robots*, IJRR 2026（arXiv [2509.06342v2](https://arxiv.org/abs/2509.06342)）——
> 本文件里带 **【§x.y / Eq.(N) / Table N】** 的都是论文原文；标 **【推断】** 的是我的工程判断。
> 官方代码：<https://github.com/leggedrobotics/pace-sim2real>（Isaac Lab，Isaac Sim 5.0+）；
> 方法细节抽取见 §9 附录引用。配套：`docs/SIM2REAL_GAP_OPENSOURCE.md`、`docs/SIM2REAL_DATA_FEEDBACK.md`。

---

## 0. TL;DR

1. **PACE 的可复现内核很小**：辨识 `p = [I_a, d, τ_f, q̃_b, T_d]ᵀ ∈ R^{4n+1}`（**Eq. 2**），
   目标函数就是**时均关节位置平方误差**（**Eq. 3**），激励是**位置目标层的 chirp**、仿真用**同一串位置目标开环回放**。
2. **它的数据要求恰好是我们已经有的**：只需要**编码器位置 + 我们下发的目标位置**，
   不需要力矩传感、不需要足端力。我们的 500 Hz 数据集里 `c_pos_*` / `m_pos_*` 正好对应 `des_dof_pos` / `dof_pos`。
3. **唯一的硬冲突是仿真器**：官方代码跑在 Isaac Lab。而 PACE 的前提是"**对齐你将要训练策略的那个仿真器**"
   （仿真回放要用"**the simulation rate used later for RL**"，§2.2）—— 我们训练在 MuJoCo/MJX。
   **在 Isaac Lab 里辨识、却在 MJX 里训练，方法上不成立**（详见 §4）。
4. **推荐路线**：把官方仓库当**规范与参考实现**（参数化/目标/CMA-ES 超参/PaceDCMotor/能量奖励公式逐条照搬），
   但**在我们自己的 MuJoCo 狗上实现拟合回路**；官方仓库另外装一份用于**小规模交叉核对**。
5. **分阶段**：先做**单腿 3 关节（10 参）**打通闭环 → 再做**12 条腿关节（49 参）** →
   轮子单独处理（它不在论文的参数化里，见 §3.4）。

---

## 1. 论文方法要点（复现所需的全部）

### 1.1 参数化

| 项 | 论文 |
|---|---|
| 待辨识参数 | **每关节** `I_a`（等效电枢惯量）、`d`（粘性阻尼）、`τ_f`（库仑摩擦）、`q̃_b`（关节偏置）<br>+ **一个全局** `T_d`（集中延迟）**【Eq. 2, §2.2】** |
| 参数向量 | `p = [I_a, d, τ_f, q̃_b, T_d]ᵀ ∈ R^{4n+1}`（n = 被驱动关节数）**【Eq. 2】**；我们 n=16 → **65**，只做 12 腿 → **49**（论文 ≈49） |
| `q̃_b` 含义 | 指令位置与有效位置之间的**准静态偏置**（编码器零位、装配公差、固件补偿），在辨识与部署期间近似常数；不建模会表现为**稳态 PD 设定点偏移****【§2.2】** |
| 隐含的关节闭环模型 | `I_a q̈ + d q̇ = sat( P_τ(q̂ − q + q̃_b) − D_τ q̇ ) + τ_comp + τ_f` **【Eq. 6】** |
| 频域形式（仅用于分析） | `H_q(s) = e^{−sT_d} · P_τ / ( I_a s² + (d + D_τ)s + P_τ )` **【Eq. 1】** |
| 抽象层级 | **关节空间、端到端**；内层电流/力矩环视为**单位增益**；**固件补偿、滤波、未建模电子学不单独参数化**，而是被 `I_a / d / T_d` **隐式吸收** **【§2.2】** |
| 不含 | **饱和不作为自由参数**（"poorly conditioned and weakly identifiable from in-air trajectories"）→ 饱和用**厂商规格 + 安全限值**预先算成包络 **【§2.2, Table 1】** |

### 1.2 目标函数与仿真回放

```
ℓ_e = (1/k) Σ_{i=1..k} || q_i^real − q_{i,e}^sim ||²        (Eq. 3)
p*  = argmin_p E[ ℓ_e ]                                       (Eq. 4)
```
- **只用关节位置**；时间平均；**论文未提按关节/幅值归一化，也未提加权**（**【未找到】**）**【Eq. 3, §2.2】**。
- 仿真回放：**base 刚性固定在空中**；**按"以后 RL 用的仿真步长"回放记录的关节目标 `q̂`**；
  每个并行环境用**真机实验的 base 位姿**；**开环回放位置指令**（不是自由动力学）**【§2.1, §2.2】**。
- **为什么不用分布距离**：Table 1 写明用关节位置误差的理由是 "**Simplicity (no drift)**" **【Table 1】**。

### 1.3 数据采集协议

| 项 | 论文要求 |
|---|---|
| base | **固定、无基座运动**；仿真回放时刚性固定在空中 **【§2.1】** |
| 接触 | **避免一切接触（含腿间）** —— 站立时基座惯量比腿/驱动大 1~2 个数量级，空中数据才能隔离腿/驱动动力学 **【§2.1, App. B】** |
| 激励 | **所有关节同时**做 **20~60 s** chirp（§2.1）/ 实验中为 **20~40 s**（§3.1.1），**f0 = 0.1 Hz → 上限 10 Hz**，**加在关节位置目标层**，由 PD 跟踪 **【§2.1, §3.1.1】** |
| 为什么加在位置目标层 | 真机与仿真被**完全相同的参考轨迹**驱动，而不是各自自由响应 ⇒ "**eliminates accumulating phase error and makes direct trajectory matching well-posed**" **【§2.1】** |
| 采样 | **典型 400~10 000 Hz**，信号与测量**时间同步**；整机日志与底层控制跑 **400 Hz** **【§2.1, §3.1.1】** |
| 数据量 | **每个机器人只需约 20 s 的「仅编码器」空中数据**就能完成拟合 **【§5】** ← 门槛比想象低得多 |
| 频段 | 理想到 **f_policy/2**；受结构限制时（**ANYmal 只到 2 Hz**、Tytan/Minimal 10 Hz）**至少覆盖运动控制器动作最高频的 2 倍**（1 m/s 行走取 1 Hz）**【§2.1】** |
| PD 增益 | **没有正式/最优选法**，按实验可行性保守取；**故意用较低增益**，"更高增益并未改善辨识质量，反而把闭环极点推出可行激励带宽" **【§2.1】** |
| 对称性 | 有对称面的机器人（ANYmal 两个面）用**对称关节轨迹抵消净基座力旋量**；无对称面的机器人则**刚性约束基座**即可，执行器级辨识不变 **【§2.1】** |

### 1.4 优化器（CMA-ES）

| 项 | 值 |
|---|---|
| 库 | Python `cmaes`（Nomura & Shibata 2024）**【§2.2】** |
| 并行 | **N = 4096 个并行环境**，每个一套参数 `p_e` **【§2.2】** |
| 参数归一化 | 全部按各自上下界归一化到 **[−1, 1]** **【§3.1.2】** |
| 初值 | **零均值**（= 可行区间中心，非偏置先验）**【§3.1.2】** |
| 初始 σ | **0.5**（归一化空间）**【§3.1.2】** |
| 迭代 | Figure 7 横轴到 **100 次**；**终止准则未给**【Figure 7】** |
| 收敛时间 | **每平台 10~24 h**，硬件 **单张 RTX 3080** **【§4, §3.1.1】** |
| 单驱动小规模 | 每次拟合 **3 个参数**、**30 次运行共约 5 h** **【§3.2.1】** |
| 明确禁止 | ① **不要把 PD 增益与动力学联合优化**（存在公共缩放 `u_c` 使 `{I_a,d,P_τ,D_τ}` 同缩放而轨迹不变 ⇒ **非唯一最优**）**【Eq. 7, §2.2】**；② **不要任意加参数**（"adding parameters indiscriminately can harm identifiability"）**【§2.4】** |
| 替代方案 | "for lower-dimensional or more expensive settings, **Bayesian optimization** is an alternative" **【§2.2】** |

### 1.5 验证与收益

- **in-air**：单驱动台架（hip 执行器刚体固定、**电机轴竖直**使重力可忽略、锁定销在**已知半径**加离散质量调惯量）；
  整机用相位图（真机−仿真的位置/速度差）对比 no-model / actuator-network / PACE **【§3.2.1, §4.1.1】**。
- **on-ground**：ANYmal 全程 **CoT −32%（→1.27）**；3 个平台 + 另部署 **10 台**；**不做动态参数随机化** **【摘要】**。
- **`T_d` 实测值（Table 6）**：ANYmal **7.5 ms**、Tytan **7.5 ms**、Minimal **0.0 ms**（单驱动台架是 µs 级、可忽略）。**← 我们实测 ≈24 ms，是它们的 3 倍多。**

---

## 2. 我们已有什么 / 缺什么

| 论文需要 | 我们现状 | 判断 |
|---|---|---|
| 编码器位置 + 下发目标位置（同时间基） | ✅ `S2RDataset`：500 Hz、`m_pos_*` / `c_pos_*`、`t_ms`/`wall_ms` | **已在 400~10 000 Hz 区间内** |
| 位置目标层 PD 跟踪（IMPEDANCE） | ✅ 腿就是 IMPEDANCE + kp/kd | 直接可用 |
| 全关节**同时** chirp 20~60 s | ❌ `Example60` 模式 5 是**逐关节**、8 s | 需改成"同时激励 + 20~60 s" |
| base **刚性**固定、无基座运动 | 🟡 我们是**吊带悬吊** | ⚠️ **吊带会摆 ⇒ 必须做刚性夹具**（否则基座运动违反前提） |
| 避免一切接触（含腿间） | ✅ 吊起即可 | 轮子离地后**无重力负载**，见 §3.4 |
| 低 PD 增益 | 🟡 我们腿 kp=250/kd=4 | 论文明确"**故意低增益**" ⇒ 辨识时应降到 ~50/1（与 Go2W 部署值一致） |
| CMA-ES + 4096 并行回放 | ❌ 没有 | 需自建（MJX 可上千并行，我们训练就用 16384 envs） |
| 仿真步长 = "以后 RL 用的步长" | ✅ 我们 500 Hz 电机环 / 50 Hz 策略 | 需明确回放用哪一级（见 §5 待决） |
| `T_d` 时域实现 | ❌ 论文只给 `e^{−sT_d}`，**未给时域形式**【未找到】 | 自定（建议按步长缓冲 `N = round(T_d/dt)`）并做敏感性分析 |
| `I_a/d/τ_f/q̃_b` 的**上下界数值** | ❌ 论文只说要归一化到 [−1,1] | **【未找到】**，需从官方代码仓的配置文件补齐 |
| 能量奖励（若要一起做） | ❌ | Eq.(12) 需 `τ_j, R_j, r_j, k_{i,j}`；**`k_i` 被排除在辨识外，必须单独实测**；`k_regen` 按平台（Tytan/Minimal 0.3、ANYmal 0.0）**【Eq.12/13, Table 2】** |
| 观测含 base 线速度 | 🟡 我们 IMU **100 Hz 且无加速度输出**；sim2sim 里 base_lin_vel **恒置 0** | 论文观测是 48 维本体感受含 base 线速度 ⇒ 若要完全对齐需状态估计/腿式里程计 |

---

## 3. 与硬件的三个具体冲突

### 3.1 只有 IMPEDANCE 模式落在论文的参数化里
论文的模型是 `τ = P_τ(q̂ − q + q̃_b) − D_τ q̇`，且**内层电流/力矩环视为单位增益**。
我们的达妙驱动器有 IMPEDANCE / SPEED / POSITION 三种模式，**只有 IMPEDANCE 匹配**。
⇒ **辨识与部署都必须用 IMPEDANCE**（轮子也一样，见 §3.4）。**SPEED 模式（固件 PI 速度环）不在参数化内**，
它属于「固件补偿/内环」那一类，理论上可被 `I_a/d/T_d` 吸收，但**前提是激励与部署用同一模式**。

⚠️ **论文给了这条约束的量化后果**：固件补偿开启会引入**表观虚拟惯量**（Tytan 实测 `I_comp = 8.1e-3 kg·m²`，
与真实 armature 同量级）**【§4.1.1, §5】**。⇒ 达妙的 cogging/摩擦补偿**开或关必须在辨识与部署期间保持一致**，
否则辨识出的 `I_a` 立刻失效。这是**硬约束**，不是建议。

### 3.2 时基 500 Hz ≠ 论文的 400 Hz
论文整机日志与底层控制是 **400 Hz**【§3.1.1】。我们上位机 **500 Hz**、策略 **50 Hz**。
⇒ `T_d` **必须重新辨识，不能沿用论文的 7.5 ms**（我们已实测 ≈24 ms，**超过一个 50 Hz 控制周期**）。

### 3.3 激励带宽要先测再定
我们策略 50 Hz ⇒ Nyquist 25 Hz，但**结构上到不了**；论文 ANYmal **只到 2 Hz** 仍然成功，
因为它的运动内容只有约 1 Hz【§2.1】。
⇒ **先测我们步态/转向动作的实际频率含量**，再定 chirp 上限（不要一上来追 15 Hz）。

### 3.4 轮子：论文没有轮式平台（**【未找到】**）
轮子的 `I_a/d/τ_f` 含义需重新定义，且 `q̃_b` 的语义（重力/装配导致的准静态 PD 偏移）对**无重力负载的轮关节**
**可辨识性存疑**。轮-地接触摩擦、轮惯量/阻尼是额外自由度。
⇒ **第一轮把轮子排除在 PACE 之外**：只辨识 **12 个腿关节（49 参）**；
轮子走另一条已核实过的路线（Go2W 的 `kp=0, kd` 关节级阻尼，与仿真 `kd(ω_des−ω)` **同一个方程**，
见 `docs/SIM2REAL_GAP_OPENSOURCE.md` §2.3）。

---

## 4. "用官方原仓库"的三条路线（请选一条）

| 路线 | 做法 | 优点 | 代价 / 风险 |
|---|---|---|---|
| **A. 全迁 Isaac Lab** | 装 Isaac Sim 5.0 + Isaac Lab，把我们的狗做成 PACE task，**辨识与训练都在 Isaac Lab**，只把部署留在 C++ | 能**原封不动**跑官方 `data_collection.py` + `fit.py`；参数化/目标/CMA-ES 都是作者验证过的 | 迁移成本最大（URDF→Isaac、joint 顺序/符号、接触与执行器建模重做、训练框架从 MJX 换成 rsl_rl）；<br>我们整套 C++ 部署与 MJX 资产要长期双栈维护 |
| **B. 在 MuJoCo/MJX 上复现（推荐）** | 官方仓库当**规范**：照搬 Eq.(2)(3)(6) 的参数化与目标、CMA-ES 超参（[−1,1] 归一化 / 零均值 / σ=0.5 / N=4096）、`PaceDCMotor` 与能量奖励公式；**拟合回路建在我们自己的 MuJoCo 狗上** | 训练与辨识在**同一个仿真器**（方法上正确）；复用我们已有的 sim2sim / MJX / 数据集工具链；MJX 能上千并行 | 拟合回路要自己写（CMA-ES 库 + 并行回放 + 损失）；**上下界/population/dt 等论文未给的项**要自己定 |
| **C. 用 mjlab 移植版** | <https://github.com/fan-ziqi/pace-sim2real-mjlab>，保留同名包/脚本/数据格式/API，只换成 MuJoCo-Warp | 省掉"自己写拟合回路"；与 MJX 同族 | **不是官方**（社区移植，作者自标 experimental 未完全测试）；仍是第三方依赖 |

**【推断】我的建议：B 为主 + A 做小规模交叉核对。**
理由：PACE 成立的前提是"对齐你将要训练的那个仿真器"（§2.2 明确回放要用 RL 的仿真步长）；
我们在 MJX 训练，若在 Isaac Lab 辨识，就要额外证明"辨识出的 `I_a/d/τ_f` 能跨仿真器迁移"——这本身是个研究问题，
不是工程默认项。而官方仓库的价值（参数化、目标、CMA-ES 配置、电机模型、能量奖励）都是**规范级**的，
照搬规范不需要跑它的仿真器。
**A 的合理用法**：把官方仓库装起来，在 ANYmal-D 上跑通一次 `data_collection.py` + `fit.py`，
拿到"论文方法在标准平台上应有的结果"作为基线，再用我们的实现对齐这个基线。

---

## 5. 分阶段执行计划（路线 B）

### S0：前置与决策（0.5 天，不碰狗）
- [ ] **做刚性夹具**：把机身**固连**到支架（吊带会摆，违反"fixed base, no base motion"）。这是**唯一必须新增的硬件**。
- [ ] 决定回放的仿真步长（候选：物理步 0.002 s / 控制步 0.02 s）+ `T_d` 时域实现（建议 `N = round(T_d/dt)` 步缓冲）。
- [ ] 从官方代码仓补齐 **`I_a/d/τ_f/q̃_b/T_d` 的上下界**、population size、终止准则（论文均未给）。
- [ ] 测**步态/转向动作的频率含量**，定 chirp 上限。
- [ ] 定 PD 增益：辨识与部署一致，且**偏低**（建议先取 kp=50 / kd=1，与 Go2W 部署值一致）。

### S1：真机激励采集（1 天）
- [ ] 给 `Example60_SysIdProbe` 加**新模式 6：全关节同时位置 chirp**（12 腿同时、20~60 s、低增益、刚性固定、轮离地）。
- [ ] 同一段 chirp 采**多组**（幅值/频段不同），每组落一个 `log/dataset_*.csv`（500 Hz，含 `c_pos_*`/`m_pos_*`/IMU）。
- [ ] 记 meta：PD 增益、固件补偿配置、电池电压、温度、刚性固定方式。

### S2：仿真回放（1 天）
- [ ] 给 `dogurdf_sim2sim_deploy` 加 `--replay <dataset.csv>`：**base 刚性固定**、**按记录的 `c_pos_*` 开环回放**、
      用与 RL 相同的 PD 与力矩限幅、输出同格式的 `dof_pos` 序列。
- [ ] 先做**单腿 3 关节**的回放冒烟（10 参），确认能生成与真机同长度、同时间基的轨迹。

### S3：拟合（2~4 天）
- [ ] 写 `tool/pace_fit.py`：CMA-ES（`cmaes` 库）+ **MJX 批量回放**（每个候选一个 world），
      损失用 **Eq. 3 的时均位置平方误差**，参数归一化到 [−1,1]、零均值、σ=0.5。
- [ ] 规模阶梯：**单腿 3 关节（10 参）** → **12 腿关节（49 参）**；先小 N（64~256）验证能收敛，再上 N=4096。
- [ ] 落盘：最优 `p*`、每次迭代的 score（对齐官方 `progress.pt` 语义）、以及**每个关节的残差**。

### S4：验证（1 天）
- [ ] **in-air**：新采一组**未参与拟合**的 chirp，比较真机 vs 仿真的轨迹与**相位图**（论文 Figure 1c 的三种模型对比）。
      判据：残差显著小于"无模型（URDF-only）"与"ActuatorNet 式"基线。
- [ ] **on-ground**：用 `Example61_RLStandTurnTeleop` 的**标准对比序列**（站立/左转/右转）跑真机，
      再用同一条命令串跑 sim，最后 `python3 tool/compare_sim2real.py` 看 **τ 对比 + 最佳时移 + 补偿前后残差**。
- [ ] 量化：`gyro_z` vs `cmd_wz` 的增益比、切换段超调、以及 compare 的残差下降幅度。

### S5：重训与部署（后续）
- [ ] 把 `p*` 写进 MJX 训练的机器人配置（`armature / damping / frictionloss` + 延迟 + 编码器偏置）。
- [ ] （可选，另立任务）四项奖励 + PMSM 能量模型：需要**单独实测** `k_i`（转矩常数）、`R`、`r`、`k_regen`。

---

## 6. 风险与必须先验证的假设

1. **跨仿真器迁移**：若走路线 A/C，必须先证明辨识参数在 MJX 里同样有效。**走 B 则不存在这个问题。**
2. **吊带 vs 刚性固定**：不解决这点，S1 的数据就违反论文前提（基座运动 → 未建模的外部力旋量）。
3. **500 Hz + 24 ms 延迟**：延迟超过一个控制周期，`T_d` 的辨识与仿真实现方式会显著影响结果 —— 必须做**敏感性分析**，
   而不是只取一个点值。
4. **轮关节的可辨识性**：`q̃_b/τ_f` 对无重力负载的轮子可能不可辨识（**【推断】**）—— 所以第一轮排除轮子。
5. **饱和包络**：论文用**厂商规格 + 安全限值**预计算（不作为自由参数）。我们的 `TORQUE_CMD_LIMIT`（120/120/200/52）
   就是现成素材，但**反电动势限速**部分需要补（达妙手册有电机参数）。
6. **PD 增益不许联合优化**（Eq. 7 的非唯一性）—— 增益必须**固定、已知、且辨识=部署**。
7. **固件补偿开关必须一致**（见 §3.1 的量化后果：补偿可引入与真实 armature 同量级的表观惯量）——
   这条最容易在「辨识时图省事关掉、部署时又打开」的场景里被违反。
8. **数据门槛很低，别过度设计**：论文每个机器人只用约 20 s 空中数据就能拟合 **【§5】**；
   瓶颈不在采集量，而在**仿真器侧的并行拟合回路**与**刚性固定**。

---

## 7. 交付物清单（本项目内）

| 文件 | 作用 |
|---|---|
| `src/app/examples/ex_probe.cpp` 新模式 6 | 全关节同时位置 chirp（20~60 s，低增益，刚性固定） |
| `dogurdf_sim2sim_deploy/src/sim2sim.py --replay` | 固定 base、开环回放同一位置目标，输出 `dof_pos` |
| `tool/export_pace_data.py` | `log/dataset_*.csv` → `chirp_data.pt`（`time/dof_pos/des_dof_pos`），含时间基校验与重采样 |
| `tool/pace_fit.py` | CMA-ES + MJX 并行回放，照搬 Eq.(2)(3) 与官方超参 |
| `tool/pace_validate.py` | in-air 轨迹/相位图 + on-ground 与 `compare_sim2real.py` 联动 |
| 本文档 | 规范与步骤（论文未给的项在 §5 待决清单里） |

---

## 8. 论文未给、已从官方代码仓查到的实现细节（2026-10-02 补齐）

> 来源：`git clone --depth 1 https://github.com/leggedrobotics/pace-sim2real`（在 `/tmp/pace_repo`，**未污染本仓库**）。
> 行号对应当次克隆的 master。

### 8.1 参数上下界（**item 1 已解决**）

`source/pace_sim2real/pace_sim2real/tasks/manager_based/pace/anymal_pace_env_cfg.py:35-59`
（`bounds_params = torch.zeros((49, 2))`，注释写明 `12+12+12+12+1 = 49`，顺序与 Eq.(2) 一致）：

| 参数 | 下界 | 上界 | 单位 | 行 |
|---|---|---|---|---|
| `I_a`（armature，12 个） | `1e-5` | `1.0` | kg·m² | :53-54 |
| `d`（dof_damping，12 个） | `0.0`（默认） | `7.0` | Nm·s/rad | :55 |
| `τ_f`（friction，12 个） | `0.0`（默认） | `0.5` | — | :56 |
| `q̃_b`（bias，12 个） | `-0.1` | `0.1` | rad | :57-58 |
| `T_d`（delay，1 个） | `0.0`（默认） | `10.0` | **仿真步（整数）** | :59 |

⚠️ **注意延迟单位是"仿真步"不是秒**：10 步 × `dt=0.0025` = **上限 25 ms**。
我们实测延迟 ≈24 ms —— 在 400 Hz 下约 **9.6 步**，正好卡在这个上界附近。
**若我们用 500 Hz（dt=0.002）则 24 ms ≈ 12 步 > 10 步上界**，必须自行放宽上界或改用 400 Hz 时间基。

### 8.2 CMA-ES 配置与终止准则（**item 2 已解决**）

| 项 | 值 | 出处 |
|---|---|---|
| population size | **= `num_envs`**（论文用 4096） | `scripts/pace/fit.py:73` |
| `max_iteration` | **200** | `pace_sim2real_env_cfg.py:99` |
| `sigma` | **0.5** | 同上 :101 |
| `save_interval` | **10** | 同上 :102 |
| 终止准则 | **仅硬上限**：`finished = self.max_iteration <= self.iteration_counter`（无早停/无收敛判据） | `optim/cma_es.py:98` |
| 库 | `cmaes.CMA(mean=零, sigma, bounds=[-1,1], seed=0, population_size)` | `optim/cma_es.py:40-44` |

（注意别与 `agents/rsl_rl_ppo_cfg.py` 的 `max_iterations = 150` 混淆——那是 **PPO 训练**的配置，不是 CMA-ES。）

### 8.3 仿真步长与控制频率（**item 3 已解决**）

`anymal_pace_env_cfg.py:80-81`：
```python
self.sim.dt = 0.0025      # 400 Hz simulation
self.decimation = 1       # 400 Hz control
```
⇒ **辨识时仿真与控制都是 400 Hz**（chirp 位置目标每个物理步都更新），
`episode_length_s = 99999.0`（超长 episode）；
`PaceSim2realEnvCfg.__post_init__` 里还设了 **`articulation_props.fix_root_link = True`**（基座刚性固定）、
`render_interval = 4`（100 Hz 渲染），并注明 **action = 关节位置目标、scale = 1.0 ⇒ impedance control**。

### 8.4 `T_d` 的时域实现（**item 4 已解决**）

`utils/pace_actuator.py`（`PaceDCMotor(DCMotor)`，类文档明说 "inspired by DelayedPDActuator"）：

```python
self.torques_delay_buffer = DelayBuffer(cfg.max_delay + 1, self._num_envs, ...)
self.torques_delay_buffer.set_time_lag(cfg.max_delay, ...)
...
def compute(self, control_action, joint_pos, joint_vel):
    # ① 编码器偏置：内环看到的是"带偏置的编码器位置"
    control_action_sim = super().compute(control_action, joint_pos - self.encoder_bias, joint_vel)
    # ② 延迟作用在算出来的**力矩**上
    control_action_sim.joint_efforts = self.torques_delay_buffer.compute(control_action_sim.joint_efforts)
    return control_action_sim
```

两条关键实现细节（论文正文没写）：
1. **延迟是整数仿真步的离散缓冲，作用在"PD/电机模型算出的力矩"上**，不是作用在位置/速度目标上；
2. **`q̃_b` 的实现方式是"从送入内环的 `joint_pos` 里减去偏差"**（即控制器看到编码器帧的位置），
   而不是加到目标上。复现时必须照这个方向做，否则偏差的符号会反。

### 8.5 每条轨迹的样本数（**item 5 已解决**）

`scripts/pace/data_collection.py`：
| 项 | 值 | 行 |
|---|---|---|
| `--duration` 默认 | **20.0 s** | :19 |
| `--min_frequency` / `--max_frequency` | **0.1 / 10.0 Hz** | :17-18 |
| `sample_rate` | `1 / sim.get_physics_dt()` = **400 Hz** | :93 |
| `num_steps` | `int(duration × sample_rate)` = **8000** | :94 |
| chirp 相位 | `2π(f0·t + (f1−f0)/(2T)·t²)`（**线性 chirp**，与论文 Eq. 一致） | :99-100 |
| 记录的两个缓冲 | `dof_pos_buffer` / `dof_target_pos_buffer`，形状 `num_steps × n_joints` | :124-125 |
| 动作 | `actions = trajectory[counter % num_steps]`（**位置目标即 action**） | :133 |
| sim 侧自测用值 | `armature = 0.1`（全关节）、`time_lag = 5`（步） | :60, :64 |

⇒ `.pt` 里就是 **`time` / `dof_pos` / `dof_target_pos` 三个长度 8000 的张量**（20 s @400 Hz）。
这与我们数据集里的 `t_ms` / `m_pos_*` / `c_pos_*` **一一对应**。

### 8.6 RL 训练用的仿真器（**item 6 已解决**）

`scripts/rsl_rl/train.py:13` → `from isaaclab.app import AppLauncher`，
整仓依赖 `isaaclab`（`setup.py` / 各 cfg 均 `from isaaclab...` import）。
⇒ **辨认与训练都在 Isaac Lab（+ rsl_rl）**，与 §4 路线 A 的描述一致；
论文正文只说了单驱动辨识在 Isaac Gym，仓库实际是 Isaac Lab。

### 8.7 因此 §4 的路线选择依据更清楚了

官方仓库给出的**全部**可复用资产（参数化、上下界、目标、CMA-ES 配置、`PaceDCMotor` 的延迟/偏置实现、
数据格式）**都是与仿真器解耦的"规范"**，只有负责并行的仿真回放依赖 Isaac Lab。
⇒ 走路线 B 时，上面 8.1~8.5 每一项都可以**逐字照搬**，工作集中在"把回放接到我们的 MuJoCo/MJX 上"。
