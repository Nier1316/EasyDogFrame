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
