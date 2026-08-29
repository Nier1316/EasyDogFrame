#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# 双开 sim2real 对比：sim2sim(仿真) + 真机 Example37，同一手柄同时驱动，各自记录
#
#   sim2sim : --gamepad + --record /tmp/sim_dual_<ts>.csv（wall_ms+qpos+qrel+vel+act+pgr）
#   真机    : bin/can_motor_app（main.cpp 需启用 Example37）→ log/rl_<ts>.csv / recv_<ts>.csv
#
# 用法:
#   ./run_dual_compare.sh            # 双开，手柄驱动，Ctrl+C 停止
#
# 停止后对比（用最新时间戳日志）:
#   python3 tool/compare_sim2real.py /tmp/sim_dual_<ts>.csv log/rl_<最新>.csv log/recv_<最新>.csv
#
# 注意: 手柄双开需实测（evdev 多进程读）；真机狗会动，务必安全距离 + Ctrl+C 急停。
# ---------------------------------------------------------------------------
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

TS=$(date +%Y%m%d_%H%M%S)
SIM_LOG="/tmp/sim_dual_${TS}.csv"

echo "============================================================"
echo "[双开] sim2sim → $SIM_LOG"
echo "[双开] 真机   → log/rl_${TS}.csv + log/recv_${TS}.csv"
echo "[双开] 同一手柄同时驱动两边。Ctrl+C 一起停止。"
echo "[安全] 真机狗会动！保持安全距离，随时 Ctrl+C 急停。"
echo "============================================================"

# 启动 sim2sim（gamepad 驱动 + 记录）
( cd dogurdf_sim2sim_deploy && ./run_sim2sim.sh --gamepad --record "$SIM_LOG" ) &
SIM_PID=$!

sleep 2   # 让 sim2sim 先起（手柄就绪）

# 启动真机 Example37（main.cpp 需启用 Example37）
./bin/can_motor_app &
REAL_PID=$!

# Ctrl+C 一起停
trap 'echo; echo "[双开] 停止..."; kill $SIM_PID $REAL_PID 2>/dev/null; wait 2>/dev/null; exit 0' INT TERM

wait

echo "[双开] 结束。"
echo "[对比] 用最新日志:"
echo "  python3 tool/compare_sim2real.py $SIM_LOG log/rl_<最新>.csv log/recv_<最新>.csv"
