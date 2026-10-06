# PACE Phase 0 · 步骤 1：两边"可比性"核对报告

> 2026-10-02。目的：在跑 ANYmal-D 双仿真器对照实验之前，确认 **Isaac 端**与 **mjlab 端**在
> 「资产 / 执行器 / 任务 / 参数上下界」上是否真的可比。**不可比的话，后面测出的误差差异无法归因。**
> 代码位置：`/tmp/pace_repo`（官方，commit 见下）、`/tmp/pace_mjlab`（mjlab 移植版），均为浅克隆。
> 配套：`docs/PACE_REPRODUCTION_PLAN.md` §9（Phase 0 协议）。
>
> 代码版本（本报告核对时）：官方仓 commit **`f07259c`**、mjlab 移植版 commit **`f860aac`**（均为浅克隆）。
> 复现请对齐这两个 commit，或重新 `git log --oneline -1` 记录当前值。

结论速览：**配置层几乎完全一致（逐字段核过），资产层"语义一致但描述不同"** —— 后者是唯一真正需要处理的可比性问题。

---

## 1. 配置层核对（逐字段）

### 1.1 执行器（`ANYDRIVE_PACE_ACTUATOR_CFG`）

| 字段 | 官方（Isaac） | mjlab | 结论 |
|---|---|---|---|
| `joint_names_expr` | `[".*HAA", ".*HFE", ".*KFE"]`（正则） | `JOINT_ORDER`（12 个显式名） | 语义等价；**见 §3 风险 1** |
| `saturation_effort` | 140.0 | 140.0 | ✅ |
| `effort_limit` | 89.0 | 89.0 | ✅ |
| `velocity_limit` | 8.5 | 8.5 | ✅ |
| `stiffness` (P) | 85.0 Nm/rad | 85.0 | ✅ |
| `damping` (D) | 0.6 Nm·s/rad | 0.6 | ✅ |
| `encoder_bias` | 0.0 | 0.0 | ✅ |
| `armature` | **未设置**（`None`） | **显式 0.0** | ⚠️ 等价（见 §3 风险 2） |
| `friction` | 0.0 | 0.0 | ✅ |
| `dynamic_friction` | 0.0 | 0.0 | ✅ |
| `viscous_friction` | 0.0 | 0.0 | ✅ |
| `max_delay` | 10（仿真步） | 10 | ✅ |

（出处：官方 `.../pace/anymal_pace_env_cfg.py:13-30`；mjlab `src/pace_sim2real/assets/anymal_d_asset.py:52-65`。）

### 1.2 参数上下界（49 维，顺序 = Eq.(2)）

| 参数 | 官方 | mjlab | 结论 |
|---|---|---|---|
| `I_a`（armature，1:12） | `[1e-5, 1.0]` | `bounds[:12,0]=1e-5`、`bounds[:12,1]=1.0` | ✅ |
| `d`（2:24） | `[0.0, 7.0]` | `bounds[12:24,1]=7.0`（下界默认 0） | ✅ |
| `τ_f`（24:36） | `[0.0, 0.5]` | `bounds[24:36,1]=0.5` | ✅ |
| `q̃_b`（36:48） | `[-0.1, 0.1]` | `bounds[36:48,0]=-0.1`（+ 上界，见下） | ✅ |
| `T_d`（48） | `[0.0, 10.0]` 步 | 同（`bounds[48,1]=10.0`） | ✅ |

（mjlab 出处：`tasks/manager_based/pace/anymal_pace_env_cfg.py:26-35` 的 `_anymal_d_bounds()`。
**已逐字复核完毕**，7 行赋值与官方完全一致：`[:12,0]=1e-5`、`[:12,1]=1.0`、`[12:24,1]=7.0`、
`[24:36,1]=0.5`、`[36:48,0]=-0.1`、`[36:48,1]=0.1`、`[48,1]=10.0` ⇒ **49 维上下界完全一致 ✅**）

### 1.3 任务项

| 项 | 官方 | mjlab | 结论 |
|---|---|---|---|
| `sim.dt` | 0.0025（400 Hz） | 0.0025 | ✅ |
| `decimation` | 1（400 Hz 控制） | 1 | ✅ |
| action | 关节位置目标，`scale=1.0`（→ impedance） | 同（`scale=1.0`） | ✅ |
| `max_iteration` | 200 | 200 | ✅ |
| `sigma` | 0.5 | 0.5 | ✅ |
| `episode_length_s` | 99999.0 | 1e9 | 无实质影响 |
| **`joint_order`** | `["LF_HAA","LF_HFE","LF_KFE","RF_…","LH_…","RH_…"]`（12 个，顺序见下） | **完全相同**（`JOINT_ORDER` 元组逐字一致） | ✅ **关键项通过** |
| **固定基座** | `scene.robot.spawn.articulation_props.fix_root_link = True` | `get_spec()` 文档串明写 "**fixed-base** simplified ANYmal-D MuJoCo specification" | ✅ **语义一致、机制不同**（见 §2） |

`joint_order` 两侧逐字相同（这是拟合列序的唯一定义，**不一致会让拟合彻底失效**）：
`LF_HAA, LF_HFE, LF_KFE, RF_HAA, RF_HFE, RF_KFE, LH_HAA, LH_HFE, LH_KFE, RH_HAA, RH_HFE, RH_KFE`（共 12）。

---

## 2. 资产层：**唯一需要处理的不可比项**

### 2.1 两边用的根本不是同一个描述

| | 官方（Isaac） | mjlab |
|---|---|---|
| 来源 | `from isaaclab_assets.robots.anymal import ANYMAL_D_CFG` | `src/pace_sim2real/assets/anymal_d/anymal.urdf`（自带，BSD-3） |
| 形式 | Isaac Lab 内置 **USD** | **简化 URDF**（加载时在内存里删掉 `<visual>`，只留碰撞+惯量） |

⇒ 直接跑两边对比 = 「仿真器差异 + 资产差异」混在一起，**无法归因**。这是**必须处理**的一条。

### 2.2 mjlab 侧资产的实测数字（我已实际加载验证）

用 `unitree_rl_mjlab` 环境加载该 URDF（`MjSpec.from_string` + `compile()`）：

| 量 | 实测值 | 说明 |
|---|---|---|
| `nq` / `nv` / `nbody` / `njnt` | **14 / 14 / 15 / 14** | 全部是 HINGE 关节（type 3） |
| **free joint 数** | **0** | ✅ **基座焊死 = 刚性固定**，与官方 `fix_root_link=True` 语义一致 |
| **总质量** | **30.449 kg** | ANYmal-D 名义质量约 30 kg 量级 ⇒ **惯量模型看着可信** |
| `dof_armature` | 全 **0.0** | 与 mjlab cfg 的 `armature=0.0` 一致（且 armature 本身是待拟合量，下界 1e-5） |
| 关节限位（抽样） | `LF_HAA=[-0.7854,+0.6109]`、`RF_HAA=[-0.6109,+0.7854]`（**左右镜像** ✓）<br>`LF_HFE/KFE=[-9.4248,+9.4248]`（±540°，等于不约束） | 髋限位与真实 ANYmal-D 的 ±0.785/-0.611 一致 |
| 重力 | 默认 `[0,0,-9.81]` | — |

⚠️ **14 个 HINGE 关节但有 12 个被致动**：多出的 2 个需在 Isaac 侧同样核对（可能只是结构/被动关节）。

### 2.3 Isaac 侧资产数字尚缺（下一步要跑的第一个 Isaac 任务）

需要一个**一次性脚本**加载 `ANYMAL_D_CFG` 并打印同样的量，才能做出资产 diff 表。可直接用：

```python
# 放在 /home/sysu/IsaacLab 下用 env_isaaclab 跑（首次启动会编译内核，耐心等）
from isaaclab.app import AppLauncher
app = AppLauncher({"headless": True, "enable_cameras": False}).app
import torch
from isaaclab_assets.robots.anymal import ANYMAL_D_CFG
from isaaclab.assets import Articulation
import isaaclab.sim as sim_utils
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.0025))
sim.set_camera_view([2, 2, 1.5], [0, 0, 0.8])
art = Articulation(ANYMAL_D_CFG.replace(prim_path="/World/Robot"))
sim.reset()
print("joint_names:", art.joint_names)
print("num_joints:", art.num_joints)
print("总质量: 需按 body 求和，见下")
masses = art.root_physx_view.get_masses()          # 各 body 质量
print("body 数:", masses.shape, "总质量:", float(masses.sum()))
print("关节限位:", art.data.joint_pos_limits)
app.close()
```
（具体 API 随 Isaac Lab 版本略有差异，2.3.2 下 `root_physx_view.get_masses()` 可用。）

---

## 3. 剩余风险（都不阻塞，但要知道）

1. **`joint_names_expr` 的匹配顺序**：官方用正则（匹配顺序取决于 USD 里的关节顺序），mjlab 用显式元组。
   但**拟合列序由 `joint_order` 决定、不由执行器决定**（sim buffer 按 `len(joint_order)` 建），
   所以顺序不一致**不会**污染拟合 ⇒ 低风险。仍建议在 Isaac 侧打印 `art.joint_names` 与 `joint_order` 比对一次。
2. **`armature` 的初始值**：官方不设（`None` → 不写 sim 的 armature，等价用 USD 里的值，通常 0），mjlab 显式 0.0。
   PACE 本来就要**拟合** armature，故初值影响小；但两侧"未拟合前的初始动力学"应尽量一致 ⇒ 记录为待确认。
3. **资产差异的三种处理方案**（推荐 (i)）：

| 方案 | 做法 | 代价 | 可比性 |
|---|---|---|---|
| **(i) 统一到简化 URDF（推荐）** | 用 Isaac Lab 的 URDF importer 把 `anymal.urdf` 转 USD，官方侧 `ANYMAL_D_CFG` 指向它 | 一次转换 + 校核 | **最好**：同资产、只差仿真器 |
| (ii) 统一到 Isaac 的 USD | 把 USD 转 MJCF 给 mjlab | USD→MJCF 有损、工具链别扭 | 好，但工程量大 |
| (iii) 各用原生资产 + 量化差异 | 只做 §2.2/§2.3 的 diff 表，解释时带 caveat | 最小 | 差：结论只能"带保留" |

4. **`episode_length_s`**：官方 99999.0 vs mjlab 1e9 —— 都是"超长"，无影响。
5. **交付物缺口**：官方仓的 `data/anymal_d_sim/chirp_data.pt` 与拟合结果**不随仓库分发**
   （README 让用户自己跑 `data_collection.py` 生成）⇒ Phase 0 两边都必须**自己采数据**；
   这也意味着"与论文数值对齐"只能比对 **`T_d = 7.5 ms`（3 步 @400 Hz）** 与 **score 量级 1e-1→1e-2 rad²**。

---

## 4. 步骤 1 的结论与下一步

**结论**：
- 配置层（执行器 12 字段、49 维上下界、`joint_order`、dt/decimation/action/迭代/σ）**逐项一致** ⇒ **可比**；
- **固定基座两边都做到了**（机制不同：Isaac 用 spawn 属性焊根，mjlab 用无 free joint 的固定基座 spec，实测 `free joints = 0`）；
- **唯一实质障碍是资产描述不同**（Isaac 内置 USD vs 自带简化 URDF，后者实测 30.449 kg、基座焊死、髋限位镜像正确）。

**下一步（按顺序）**：
1. 跑 §2.3 的 Isaac 侧资产脚本 → 补齐资产 diff 表（**含 `art.joint_names` 与 `joint_order` 的比对**）；
2. 依 §3 表选资产统一方案（建议 (i)：URDF importer 统一到简化 URDF）；
3. 然后才进 Phase 0 的 P0.1/P0.2（先用 `--duration 0.1 --max_iterations 1` 跑通 smoke test，再上正式拟合）。


---

## 5. 追加核对（2026-10-02 第二批）：资产来源已锁定，但发现一个 **P0 阻塞项**

### 5.1 仓库改为持久位置（`/tmp` 会被系统清理）

⚠️ 本次核对中 `/tmp/pace_repo`、`/tmp/pace_mjlab` **被系统清理掉了**（`/tmp` 是易失的）。
已改克隆到**持久目录**，后续请一律用这里：

| 仓库 | 路径 | commit |
|---|---|---|
| 官方 PACE | `/home/sysu/pace/refs/pace-sim2real` | `f07259c` |
| mjlab 移植版 | `/home/sysu/pace/refs/pace-sim2real-mjlab` | `f860aac` |
| **上游 ANYbotics 描述** | `/home/sysu/pace/refs/anymal_d_simple_description` | `d5bf4dc` |

### 5.2 ✅ 资产来源锁定：两边看的是**同一份上游模型**

- Isaac Lab 的 `ANYMAL_D_CFG` 文档串明确列出上游为
  **`ANYbotics/anymal_d_simple_description`**（`isaaclab_assets/robots/anymal.py:18`），
  USD 位于 `ISAACLAB_NUCLEUS_DIR/Robots/ANYbotics/ANYmal-D/anymal_d.usd` —— **在 NVIDIA Nucleus 云端，本地没有**。
- **mjlab 打包的那份 URDF 与上游 ANYbotics URDF 字节完全相同**：
  `diff -q` 零差异，两者都是 **59405 B**。
- ⇒ 「统一到 Isaac 的 USD」与「统一到简化 URDF」在**源头**上是同一件事，**前提是 NVIDIA 的 USD 转换忠实**
  （这条仍需 Isaac 侧实测确认，见 §5.4）。**因此很可能不需要做 USD→MJCF 转换**。
- ⚠️ 附带注意：Isaac 的 ANYmal 配置还引用一个 **ActuatorNet LSTM**（`ActuatorNets/ANYbotics/anydrive_3_lstm_jit.pt`，云端）。
  PACE 用 `PaceDCMotorCfg` 覆盖了执行器，理论上不加载它；但这是**隐藏的网络依赖**，首次 Isaac 运行要留意日志。

### 5.3 🚨 P0 阻塞项：编译出来的模型**丢了 26.6 kg**

用 `unitree_rl_mjlab` 环境把该 URDF 编译成 MuJoCo 模型，结果与 URDF 声明值严重不符：

| 量 | URDF 声明 | MuJoCo 编译后 |
|---|---|---|
| link / body 数 | 96 links（其中 **64 个有 inertial**） | **15 bodies**（fixed 关节被焊合，符合预期） |
| **总质量** | **57.0279 kg** | **30.4490 kg** ← **少了 26.58 kg** |
| 有 inertial 但未出现在编译模型里的 link | — | **50 个**（`base_inertia`、`top_shell`、`battery`、`base_LF_HAA_drive`、各类相机…） |
| 抽查焊合是否正确 | — | `LF_THIGH = 5.1322 kg` = `thigh_fixed 1.1010 + HFE_drive 2.0151 + KFE_drive 2.0151` ✅ **焊合本身是对的** |

**缺失的质量集中在"基座侧"**：`base_inertia 8.28 kg`、4×`HAA_drive 2.015 kg`、`battery 5.51 kg`、
shells/lidar/相机等（合计 ≈26.6 kg）。而**运动链上的质量（thigh/shank/pan/tilt）都在**。

**影响判断（关键）**：
- 对 **PACE 辨识（基座焊死）**：基座质量/惯量**不参与**关节动力学 ⇒ 可能**不影响**辨识结果；
- 对 **RL 训练（浮动基座）**：基座质量/惯量直接决定动力学 ⇒ **30.4 kg vs 57.0 kg 是完全不同的机器人**。
- ⇒ 在把它用于**任何浮动基座**用途之前，必须查清这是"MuJoCo URDF 解析行为"还是"模型本身的问题"。

**尚未定位根因**，下一步诊断方向（按序）：
1. 打印 MuJoCo 的 **body 1（基座）质量**，确认是否确实≈0；
2. 检查 URDF 里 `base` link 是否**没有 `<inertial>`**、以及 `base_inertia` 等是否通过 fixed 关节挂在 `base` 下；
3. 对照 mjlab 的 `get_spec()`：它是否额外做了"剔除 base 侧质量"的处理（目前看**没有**，只是删 `<visual>`）；
4. 用 MuJoCo 官方 `mj_spec` 的 URDF 解析日志/警告（编译时是否打印过 "link ... has no inertia" 之类）；
5. 若确认是解析行为，考虑改为"显式给基座写入合并后的质量惯量"或走 **MJCF 原生资产**。

### 5.4 更新后的下一步

1. **先解决 §5.3**（P0）：不改的话，mjlab 侧的一切结论都不能采信（尤其"误差对比"会被质量差异主导）；
2. 再跑 Isaac 侧资产脚本（**这次要联网下载 USD**），把 Isaac 的 body 质量/限位/关节名与上游 URDF 逐项对比
   —— 若 NVIDIA 的 USD 转换忠实，则 §5.2 的"同源"成立，两边可直接用各自原生格式；
3. 然后才进 Phase 0 的 P0.1/P0.2。


---

## 6. Isaac USD 实测结果（2026-10-05，脚本 `tool/pace_inspect_anymal_asset.py`）

运行环境：`env_isaaclab`（Isaac Sim 5.1），headless，日志 `/home/sysu/pace/logs/isaac_asset.log`。
USD 从 NVIDIA Nucleus **成功下载**（无本地副本）。

### 6.1 三份"总质量"终于对上了

| 侧 | 关节数 | body 数 | 总质量 | 说明 |
|---|---|---|---|---|
| 上游 ANYbotics URDF（声明求和） | 14 revolute + 81 fixed | 96 links | **57.0279 kg** | 含 inspection payload pan/tilt |
| **Isaac USD（NVIDIA 转换）** | **12** | **17** | **51.6357 kg** | = 57.03 − **5.39**（pan 3.19 + tilt 2.08 的关节与质量被移除） |
| mjlab（MuJoCo 编译同一 URDF） | **14** | **15** | **30.4490 kg** | = 57.03 − **26.58**（基座自身的壳/电池/HAA 驱动等被丢弃，见 §5.3） |

### 6.2 ⭐ 决定性发现：**运动链质量逐项吻合，差异全在被焊死的基座上**

| body | Isaac USD | mjlab(MuJoCo) | 判定 |
|---|---|---|---|
| `THIGH` ×4 | **5.1322** kg | **5.1322** kg | ✅ **完全相同** |
| `SHANK` ×4 | 0.4995 | 0.7960 | ✅ 等价：MuJoCo 把 fixed 的 FOOT 焊进了 SHANK（0.4995 + 0.2964 = 0.7959） |
| `FOOT` ×4 | 0.2964 | （已并入 SHANK） | ✅ 同上 |
| `HIP` ×4 | 0.3659 | 已焊入 base（4×0.3659 = 1.4636） | ✅ 质量在场 |
| **`base`** | **26.4596** | **1.4636**（只剩 4 个 HIP） | ❌ **差 25.0 kg**（Isaac 多了 base_inertia/battery/4×HAA_drive/shells） |
| `inspection_payload_pan/tilt` | **不存在**（USD 只有 12 关节） | 3.1889 / 2.0840 | ⚠️ mjlab 多出 2 个**未致动**关节 |
| 合计 | 51.6357 | 30.4490 | — |

**对 PACE 辨识的影响：可以认为两者等价 ✅**
- PACE 的基座是**焊死**的 ⇒ 基座质量/惯量**不参与关节动力学**，那 25 kg 差异与 pan/tilt 那一支（同样挂在焊死的基座上、且未被致动）**都不影响 12 个腿关节的动力学**；
- 而**真正影响关节动力学的 THIGH / SHANK+FOOT / HIP 质量在两侧逐项吻合**。

**对浮动基座用途（RL 训练）的影响：** ⚠️ mjlab 侧基座轻了 25 kg ⇒ **那是另一台机器人**，训练前必须修（§5.3）。

### 6.3 Isaac 侧实测细节（供回填与复查）

- `fix_root_link`：脚本第 [基座] 段已打印（另见 §1.3）。
- 关节限位（与上游一致）：`HAA = [-0.7854, +0.6109]`（左）/`[-0.6109, +0.7854]`（右，镜像 ✓）；
  `HFE/KFE = [-9.4248, +9.4248]`（±540°，等于不约束）。
- `base` 惯量对角 = `[0.25324, 0.00512, 0.02138]`（Isaac 侧，供与上游 URDF 的 base 惯量对照）。
- body 列表（17 个）：`base, {LF,LH,RF,RH}_HIP, {…}_THIGH, {…}_SHANK, {…}_FOOT`。

### 6.4 ⚠️ 新发现：**关节顺序不同**（但集合相同）

| | 顺序 |
|---|---|
| **Isaac USD**（`robot.joint_names`） | `LF_HAA, LH_HAA, RF_HAA, RH_HAA, LF_HFE, LH_HFE, RF_HFE, RH_HFE, LF_KFE, LH_KFE, RF_KFE, RH_KFE`（**按关节类型分组**） |
| **PACE `joint_order`** | `LF_HAA, LF_HFE, LF_KFE, RF_HAA, RF_HFE, RF_KFE, LH_…, RH_…`（**按腿分组**） |
| mjlab `JOINT_ORDER` | 与 PACE `joint_order` **逐字相同** ✅ |

集合相同（缺失/多余均为空），但**排列不同**。
⇒ 只要 PACE 的代码一切按 `joint_order` 解析 `joint_ids`，就不会出问题；
**但任何"假定资产顺序 == joint_order"的写法都会静默错位**。
**核实结果（2026-10-05）：安全，不是问题 ✅** —— 见 §8。

### 6.5 脚本侧小问题（不影响结论）

`[5] ANYMAL_D_CFG.actuators` 那段只打印出 `(class = str)` —— `replace()` 之后 `actuators` 的形态与预期不同，
未能打印出 Isaac 默认的 stiffness/damping 等。PACE 会用 `PaceDCMotorCfg` 覆盖执行器，故不阻塞；
如需，可改为从 `ANYMAL_D_CFG.__dict__` 直接取。

---

## 7. 待办（按优先级）

| # | 事项 | 依据 |
|---|---|---|
| 1 | ~~核实 PACE 如何把参数对应到关节~~ ⇒ **已完成，不是问题**（见 §8：按**关节名**查表映射，与资产顺序无关） | §6.4, §8 |
| 2 | **若要把 mjlab 用于浮动基座（RL 训练）**：修掉基座少 25 kg（补齐 base_inertia/battery/HAA_drive/shells 的合并质量），或改用与 Isaac 一致的资产 | §5.3, §6.2 |
| 3 | 决定 pan/tilt 这 2 个未致动关节如何处理（Isaac 无、mjlab 有；对辨识无影响，但会让"DoF 数"不一致） | §6.2 |
| 4 | 补跑 `[5]` 执行器默认参数（可选） | §6.5 |
| 5 | 然后进 Phase 0 的 P0.1/P0.2（先 `--duration 0.1 --max_iterations 1` 跑 smoke test） | §9.3 |

> **对"用哪个仿真器"的初步影响**：既然两侧**腿链质量逐项吻合**、且 PACE 是**固定基座**辨识，
> 那么"资产差异"对 Phase 0 的**辨识误差对比**基本不构成干扰 —— 前提是把 §7 的第 1 条（关节顺序）核实掉。
> 若最终要用 mjlab 做**训练**，则第 2 条（基座 25 kg）必须先修。


---

## 8. ✅ 第 1 项核实结果：**关节顺序差异不会导致错位**（2026-10-05）

### 8.1 参数列 → 资产索引：**按名字查表**，与资产顺序无关

`scripts/pace/fit.py:56` 与 `scripts/pace/data_collection.py:58` 都是同一句：

```python
joint_order = env_cfg.sim2real.joint_order
joint_ids   = torch.tensor([articulation.joint_names.index(name) for name in joint_order], device=...)
```

⇒ 参数向量的第 `i` 列 ↔ `joint_order[i]` ↔ **按名字找到的资产关节**。
资产内部怎么排（Isaac 按关节类型分组、mjlab 按腿分组）**完全不影响** ✅

写入时也全部带 `joint_ids`（`optim/cma_es.py:111-124`）：

```python
articulation.write_joint_armature_to_sim(self.sim_params[:, armature_idx], joint_ids=joint_ids, env_ids=env_ids)
articulation.write_joint_viscous_friction_coefficient_to_sim(self.sim_params[:, damping_idx], joint_ids=joint_ids, ...)
articulation.write_joint_friction_coefficient_to_sim(self.sim_params[:, friction_idx], joint_ids=joint_ids, ...)
articulation.write_joint_position_to_sim(initial_position + self.sim_params[:, bias_idx], joint_ids=joint_ids)
```
（另有注释说明：静态摩擦不能低于动态摩擦，所以先写 `dynamic=0` 再写 `static`，最后把 `dynamic` 补上同一值。）

### 8.2 执行器侧还有一道显式保护（**这是最容易被忽略的一步**）

`encoder_bias` / `time_lags` 是**住在执行器里的**，而执行器有自己的关节顺序
（`actuators[...].joint_indices`）。PACE 在这里做了**显式的散射映射**（`cma_es.py:126-133`）：

```python
for drive_type in articulation.actuators.keys():
    drive_indices = articulation.actuators[drive_type].joint_indices      # 执行器自己的资产索引
    if isinstance(drive_indices, slice):
        all_idx = torch.arange(joint_ids.shape[0]); drive_indices = all_idx[drive_indices]
    comparison_matrix = (joint_ids.unsqueeze(1) == drive_indices.unsqueeze(0))
    drive_joint_idx = torch.argmax(comparison_matrix.int(), dim=0)        # 拟合列 → 执行器列
    articulation.actuators[drive_type].update_encoder_bias(self.sim_params[:, bias_idx][:, drive_joint_idx])
    articulation.actuators[drive_type].update_time_lags(self.sim_params[:, delay_idx].to(torch.int))
```

⇒ `q̃b` 被正确重排到执行器顺序；`T_d` 是**全局单值**，无需映射。
**结论：这套代码对"资产顺序 ≠ joint_order"是安全的**，我们**不需要**为了顺序去改任何东西。

> 复现时唯一要照做的是：**自己实现回放时也要按名字映射**，别用位置索引。
> 另外照抄一个坑：`torch.argmax` 在"名字对不上"时会**静默返回 0**（不报错）——
> 所以仍建议像 §1.3 那样断言一次"两个集合相同"（我们已核过：缺失/多余均为空）。

### 8.3 🎁 顺带挖到：ANYmal-D 例子的**精确验收真值**

`scripts/pace/data_collection.py:59-64` 在生成 sim 激励数据前，**显式把仿真参数设成已知值**：

```python
armature = 0.1      # 12 个关节全 0.1
damping  = 4.5
friction = 0.05     # 库仑摩擦
bias     = 0.05     # 12 个
time_lag = 5        # 仿真步  (= 12.5 ms @ 400 Hz)
```

⇒ **拟合 ANYmal-D 的 sim 数据时，必须把这些值近似还原回来**，这就是 Phase 0 最硬的验收标准
（比我之前引用论文的 `T_d = 7.5 ms` 更合适 —— 那个是**真机**数据拟合出来的值，而这里是 sim 数据的**真值**）：

| 参数 | 真值 | 官方上界 | 在界内? |
|---|---|---|---|
| `I_a` | **0.1** | `[1e-5, 1.0]` | ✅ |
| `d` | **4.5** | `[0, 7.0]` | ✅ |
| `τ_f` | **0.05** | `[0, 0.5]` | ✅ |
| `q̃_b` | **0.05** | `[-0.1, 0.1]` | ✅ |
| `T_d` | **5 步 = 12.5 ms** | `[0, 10]` 步 | ✅ |

**验收判据（Phase 0 用这个，不用论文的 7.5 ms）**：
> 两侧各自的拟合结果都应落在真值附近（例如 `I_a∈[0.05,0.2]`、`d∈[3,6]`、`τ_f∈[0.02,0.1]`、
> `q̃_b∈[0.02,0.08]`、`T_d∈[4,6] 步`）。**哪一侧先复现出这五个真值，那一侧的实现就是对的**；
> 若某一侧偏差很大，先怀疑该侧 setup（而不是仿真器物理）。

⚠️ 注意一个实现细节：**同一个拟合出的 `bias` 被用了两次** ——
`write_joint_position_to_sim(initial_position + bias)`（初始状态偏移）
以及 `update_encoder_bias(bias)`（执行器看到的编码器帧偏移）。
复现时两处都要照做，否则 `q̃b` 的可辨识性会变差。

### 8.4 因此第 1 项最终结论

**关节顺序不是问题**（按名字映射 + 执行器侧显式散射），**Phase 0 的辨识误差对比可以照计划进行**。
下一步只剩：决定 pan/tilt 怎么处理（§7 第 3 项，对固定基座辨识无影响）→ 跑 smoke test → 正式拟合。


---

## 9. Phase 0 正式拟合结果（2026-10-05，两侧 N=4096）

### 9.1 运行信息

| | Isaac 端 | mjlab 端 |
|---|---|---|
| 命令 | `fit.py --task Isaac-Pace-Anymal-D-v0 --num_envs 4096 --headless` | `fit.py --task Isaac-Pace-Anymal-D-v0 --num_envs 4096 --device cuda:0` |
| 启动 → 完成 | 17:10:59 → **19:23:12**（**2 h 12 min**） | 17:14:47 → **19:07:09**（**1 h 52 min**） |
| 迭代次数 | **196**（上限 200） | **139**（提前结束，见 §9.4） |
| 退出码 / 错误 | **rc=0 / 0** | **rc=0 / 0** |
| 最优 score | **8.98e-13 rad²** | **3.60e-10 rad²** |
| 显存 | 两任务**并行**共享一张 GPU，合计仅 **9.7 / 49 GB** | 同上 |

> 说明：score 比论文 Figure 7 的 1e-1→1e-2 低若干数量级，原因是**这里拟合的是同一仿真器自己生成的数据**（自洽问题），残差可以逼近机器精度。

### 9.2 ⭐ 最终参数 vs 已知真值：**两侧都精确还原**

真值来自 `data_collection.py:59-64`（生成数据前显式设定的仿真参数）。

| 参数 | 真值 | **Isaac 拟合均值** | 误差 | **mjlab 拟合均值** | 误差 |
|---|---|---|---|---|---|
| armature `I_a` | 0.100 | **0.1000** | **0.00%** | **0.1000** | **0.00%** |
| viscous `d` | 4.500 | **4.5000** | **0.00%** | **4.5001** | **0.00%** |
| static/dynamic `τ_f` | 0.050 | **0.0500** | **0.03%** | **0.0498** | **0.33%** |
| bias `q̃_b` | 0.050 | **0.0500** | **0.00%** | **0.0501** | **0.11%** |
| delay `T_d` | 5 步 | 5.713 → **int = 5** ✅ | — | 5.459 → **int = 5** ✅ | — |

（delay 的连续坐标用 `to(torch.long)` 截断成整数仿真步后再应用，所以 5.7 / 5.5 都等价于 **5 步**。）

### 9.3 结论：**Phase 0 的"实现正确性"验证通过，但它无法区分两个仿真器**

1. **两套实现都是对的**：两个不同仿真器（PhysX vs MuJoCo-Warp）、各自拟合自己生成的数据，
   都**精确还原了全部五个真值**（误差 ≤0.33%）。这同时验证了：参数映射（§8 名字查表）、
   数据格式、CMA-ES 配置、执行器/延迟/偏置实现，两边都已正确落地。
2. **本实验不能用来选仿真器**：在"自洽拟合"下两侧**同样正确**（score 都是机器精度级）。
   **能区分仿真器的实验必须用真实数据** —— 即用我们自己狗的真机 20 s chirp，
   看哪一侧能把真机轨迹拟得更准（这才是 P0.3 的原意）。
3. **对"参数可否跨仿真器搬运"是强（但间接）证据**：两个仿真器收敛到**同一组参数**，
   说明**受物理主导的关节空间参数是仿真器无关的**。最终判据仍需**交叉回放**（把一侧的参数
   拿到另一侧回放比残差）——在**真实数据**上做才有区分力。

### 9.4 待查 / 遗留

1. **mjlab 侧为何 139 次就结束**（上限是 200）：需确认移植版是否引入了额外的收敛判据（其 `cma_es.py` 有 `epsilon` 参数）。
2. **两侧 chirp 时间网格约定不同**（官方 `linspace` vs 移植版 `arange*dt`）——
   用户决定**不补丁、按官方原样跑**，故作为**已知台架差异**保留；解读膝关节相关结果时需注意
   （目标差 mean 0.031 / max 0.313 rad）。
3. **交叉回放实验尚未做**（做在真实数据上才有意义）。
4. 过程记录：我在第 57 次迭代时曾误判"摩擦被系统性高估 5 倍"——那是**早期探索阶段**的读数，
   收敛后实际是 0.0500（误差 0.03%）。**已更正**。
