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
#
#   权重：默认使用与真机 C++ 完全相同的那一份 —— ../weights/iteration_9754.pkl
#   （即 tool/export_policy.py 的默认输入，导出到 include/strategy/policy_weights.h）。
#   sim2sim 与真机必须同权重，否则 run_dual_compare.sh 的对比结论无效。
#   需要跑别的版本时：SIM2SIM_CKPT=<path.pkl> ./run_sim2sim.sh
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
# 默认 = 真机部署权重（同一文件）。sim2sim 与真机策略必须一致，否则对比无意义。
# 2026-09-29 前此处硬编码 iteration_3000，而真机已切到 9754，属版本错配，已修正。
DEFAULT_CKPT="$HERE/../weights/iteration_9754.pkl"
CKPT="${SIM2SIM_CKPT:-$DEFAULT_CKPT}"
if [ ! -f "$CKPT" ]; then
  echo "ERROR: checkpoint 不存在: $CKPT" >&2
  echo "  默认应为真机同款: $DEFAULT_CKPT" >&2
  echo "  可用 SIM2SIM_CKPT=<path.pkl> 指定其它版本。" >&2
  echo "  历史存档（不与真机同步）: $HERE/checkpoints/dogurdf_velocity/" >&2
  exit 1
fi

echo "Python  : $PY"
echo "Checkpoint: $CKPT"

exec "$PY" src/sim2sim.py --checkpoint "$CKPT" "$@"
