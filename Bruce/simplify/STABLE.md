# STABLE —— 演示 / 现场稳定策略标记

> 本分支（`stable`）用于**对外演示与现场稳定运行**，不做日常实验开发（实验在 `develop`）。
> 本文件是「这条分支为什么可以拿出去跑」的记录。

## 当前稳定策略：h52_s45_v4 / iteration_5350

| 项 | 值 |
|---|---|
| 策略名 | **h52_s45_v4 / iteration_5350** |
| checkpoint | `weights/iteration_5350.pkl`（**已入库**，git 跟踪，4.9 MB） |
| 训练来源 | `checkpoints_20260917_193400_h52_s45_v4/iteration_5350.pkl` |
| 训练日期 | 2026-09-17 |
| 特点 | **宽摩擦 DR + 地形适应**，可前进 `vx` + 转向 |
| 代码变体 id | `rl::POLICY_H52S45_V4_5350 = 2`（`include/strategy/policy_set.h`） |
| 默认变体 | `POLICY_VARIANT = 2`（`include/strategy/policy_variant.h`） |
| 使用示例 | **`Example37_RLTeleopControl`**（手柄遥操作，函数内强制该变体） |
| 离线回归 | `Example30_RLPolicyLinkTest`：`mlp_forward(REF_OBS)` vs `REF_ACTION` 最大误差 **2.384e-6（通过）** |

### 为什么选 5350 作为演示策略
- 相比 `standstep_s4 / iteration_10000`（只支持站立 + 原地转向），5350 **支持前进 + 转向**，演示动作更完整。
- **宽摩擦 DR + 地形适应**，对演示现场多变地面（水泥/瓷砖/地毯）鲁棒性更好。
- 历史核查：该 checkpoint 在 2026-09-29 的仓库清理中被删除；2026-10-06 由 git 历史
  `0d0257c` 恢复，blob 哈希 `d19b3d6086311a2b3742dfa9c843355dbb53c096`，
  **与原版本逐字节一致**（`git hash-object` 校验通过）。

## 本分支一并包含的稳定化改动

| 项 | 改动 | 实测 |
|---|---|---|
| **500 Hz 控制环** | `ThreadManager` 改用绝对 deadline 网格（`next += period; sleep_until`） | **500.000 Hz**（原 486 Hz / −2.8%） |
| **50 Hz 策略环** | 新增 `LoopPacer`，Ex37/Ex61 改用 | **50.000 Hz**（原 ~48.6 Hz） |
| **Example61 B 键** | 改为优雅趴下（与 Example37 一致），不再硬急停；且跑对比序列时也可用 | — |
| **运行时多策略** | 9754 / 10000 / 5350 三套权重同时编译，示例运行时选择 | Ex30 三套均通过 |

## 使用

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)      # 产物 bin/can_motor_app
./bin/can_motor_app                 # 当前激活 Example37（iteration_5350）
```

安全：真机运行前先读 `docs/REAL_ROBOT_HANDOVER.md`（安全铁律 + T0~T8 清单）。
