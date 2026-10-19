# VS Code 快捷启动（一键跑指定示例）

> 2026-10-19。目标：**按一个键 → 选示例 → 直接跑**，不用再"改 `main.cpp` 注释 + 重编"。

## 一、快捷键

| 按键 | 作用 |
|---|---|
| **`Ctrl+Shift+B`** | 只编译（默认构建任务 `cmake-build-debug`） |
| **`F5`** | 调试（现在的默认配置会**先弹示例号选择器**），带 gdb |
| **`F6`** | **运行示例**：弹示例号选择 → `./run.sh <号>`（≈0.3 s 增量编译 + 运行） |
| **`Shift+F6`** | 从 launch.json 里挑一个调试配置 |

键位是**用户级**配置（`~/.config/Code/User/keybindings.json`，不随仓库走）：

```json
[
    { "key": "f6", "command": "workbench.action.tasks.runTask", "args": "run-example" },
    { "key": "shift+f6", "command": "workbench.action.debug.selectandstart" }
]
```

## 二、工程内配置（`.vscode/` 已在 .gitignore，属本机配置）

- `tasks.json`
  - `run-example`：`./run.sh ${input:exampleNum}`，带 `dependsOn: cmake-build-debug`
  - 顶层 `inputs.exampleNum`：`pickString` 列出常用示例号
- `launch.json`
  - 第一个配置 `调试 can_motor_app (选示例)`：`args: ["${input:exampleNum}"]` ⇒ **F5 直接弹选择器**
  - 其余配置保持原样（原始 F5 / 停在 main / 外部终端 / attach）

> 如果换了机器：把 `tasks.json` / `launch.json` / `keybindings.json` 三段配置照抄过去即可。

## 三、为什么需要"运行时选示例"

工程原本的设计是**编译期**决定跑哪个示例（`main.cpp` 里注释切换，"无命令行参数、无注册表"）。
现场频繁切换（Ex63 被动核对 / Ex62 PACE 采集 / Ex37 遥操作）很不方便，而且多次出现
"HEAD 里的激活状态与现场不一致"。现在 `main()` 支持：

```bash
./run.sh 63              # 等价于 EXAMPLE=63 ./run.sh
./run.sh --list          # 列出可用编号
EXAMPLE=64 ./bin/can_motor_app
```

**白名单（`RunExampleByNumber`）**：24 / 25 / 30 / 36 / 37 / 42 / 57 / 62 / 63 / 64。
未列入的示例仍走原来的编译期注释 ⇐ 保持原行为。

## 四、安全设计

| 示例 | 会不会使能电机 |
|---|---|
| 24 / 30 / 42 / 63 / 64 | **不会**（只读 / 离线 / 不使能） |
| 57 | 低增益摆腿 |
| **25 / 36 / 37 / 62** | **会** ⇒ 运行前打印 3 秒倒计时，**Ctrl+C 可取消**；自动化场景设 `EXAMPLE_YES=1` 跳过 |

⚠️ `F6` 选择器里也会列出使能类示例，**看清编号再回车**。
⚠️ **必须用 `./run.sh`**（它会自动 `LD_PRELOAD=bin/dmcan_fastdispatch.so`，修掉 100 ms 派发攒批；
用 F5/直接跑二进制**不会**加载它 ⇒ 反馈又变 ~10 Hz）。

## 五、常用组合

```bash
# 被动看姿态（不使能）+ 三维可视化
F6 → 63 ；另开终端 ./tool/pose_view.sh --udp

# PACE 采集（使能；狗需刚性固定、腿悬空）
F6 → 62 ；跑完 python3 tool/pace_qc.py     # 必须显示 meas 刷新率 ≈400 Hz

# 反馈链路诊断（不使能）
F6 → 64
```
