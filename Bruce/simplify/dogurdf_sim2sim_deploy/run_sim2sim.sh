#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# dogurdf sim2sim 一键启动脚本 (native MuJoCo, CPU backend)
#
#   启动方式（任选其一，附加参数直接透传给 src/sim2sim.py）:
#     ./run_sim2sim.sh                        # headless 测试 1000 步
#     ./run_sim2sim.sh --gamepad              # 手柄实时驾驶（打开 viewer）
#     ./run_sim2sim.sh --cmd_vel_yaw 1.0 --episode_length 300
#     ./run_sim2sim.sh --save_video --video_path /tmp/turn.mp4 --cmd_vel_yaw 1.0
#
#   Python 环境：自动优先使用 conda 的 MJX 环境，否则回退到系统 python3。
#   需要的第三方库：jax numpy mujoco flax (brax可选) pygame(手柄) mediapy(录视频)
# ---------------------------------------------------------------------------
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

# --- 环境选择：conda MJX 优先 -----------------------------------------
if command -v conda >/dev/null 2>&1; then
  if conda env list 2>/dev/null | grep -q "MJX"; then
    eval "$(conda shell.bash hook)"
    conda activate MJX
  fi
fi
PY="$(command -v python || command -v python3)"

# --- checkpoint 校验 -----------------------------------------------------
# 默认用 traj_v28/iteration_3000（与真机 C++ policy_weights.h 一致，2026-08-30 对齐）
CKPT="checkpoints/dogurdf_velocity/iteration_3000.pkl"
if [ ! -f "$CKPT" ]; then
  echo "ERROR: checkpoint 不存在: $CKPT" >&2
  echo "  期望路径: $HERE/$CKPT" >&2
  exit 1
fi

echo "Python  : $PY"
echo "Checkpoint: $CKPT"

exec "$PY" src/sim2sim.py --checkpoint "$CKPT" "$@"
