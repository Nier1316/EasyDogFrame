#!/bin/bash
# 姿态可视化启动脚本（tool/pose_view.py 的包装）
#
# 为什么需要它：pose_view.py 依赖 **mujoco**，而它只装在 conda 的某个环境里
#（本项目是 /home/bruce/miniforge3/envs/MJX）。本脚本自动找到那个环境再跑，
#  省得每次手打一长串 python 路径。
#
# 用法：
#   ./tool/pose_view.sh                       # 静态预设（default）
#   ./tool/pose_view.sh --preset nominal      # 训练侧名义站姿
#   ./tool/pose_view.sh --udp                 # 实时（配 C++ 侧 POSE_STREAM=1 ./run.sh）
#   ./tool/pose_view.sh --csv log/dataset_xxx.csv
#   ./tool/pose_view.sh --selftest            # 不开窗口，验证坐标链路
#
# ⚠️ **不要用 sudo**：
#   · 本工具只收 UDP + 渲染，不碰 CAN、不需要任何特权；
#   · sudo 会丢掉 XAUTHORITY ⇒ MuJoCo 窗口大概率打不开（报 Cannot open display）；
#   · 还可能在本工程目录里留下 root 属主的文件。
#   想指定别的 python：PY=/path/to/python ./tool/pose_view.sh --udp
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"

# 候选 python：显式 $PY 优先，其次本项目约定路径，最后 PATH 上的 python3
CANDIDATES=()
[ -n "${PY:-}" ] && CANDIDATES+=("$PY")
CANDIDATES+=(
    "$HOME/miniforge3/envs/MJX/bin/python"
    "/home/sysu/miniconda3/envs/MJX/bin/python"
    "$HOME/miniforge3/bin/python"
    python3
)

for cand in "${CANDIDATES[@]}"; do
    command -v "$cand" >/dev/null 2>&1 || continue
    if "$cand" -c 'import mujoco' >/dev/null 2>&1; then
        exec "$cand" "$HERE/pose_view.py" "$@"
    fi
done

echo "[pose_view.sh] 找不到装了 mujoco 的 python。试过：" >&2
for cand in "${CANDIDATES[@]}"; do echo "    $cand" >&2; done
echo "请显式指定：PY=/path/to/python $0 $*" >&2
exit 1
