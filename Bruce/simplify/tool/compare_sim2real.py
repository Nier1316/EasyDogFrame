#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim2real 双开对比：sim2sim --record vs 真机 rl/recv 日志，wall_ms 时间戳对齐

用法:
    python3 tool/compare_sim2real.py <sim.csv> <real_rl.csv> [real_recv.csv]

- sim.csv:    sim2sim --record 输出（wall_ms,qpos_16,qrel,vel,act,pgr）
- real_rl.csv: 真机 log/rl_*.csv（wall_ms,cmd,qrel,vel,act,pgr，POLICY order）
- real_recv.csv（可选）: 真机 log/recv_*.csv（wall_ms,cal_pos/vel/torque，CAN order，500Hz）

对齐：两边按 wall_ms（系统时间戳）对齐，逐点对比。
对比：cmd 一致性（验证对齐质量）、轮速 vel_12..15、腿 qrel、姿态 pgr_z、
      编码器位置波动（sim qpos vs real cal_pos）。
注意：sim qpos 是 URDF 角（POLICY），real cal_pos 是真机标定角（CAN）——零点不同，
      对比波动(std/范围)而非绝对值。
"""
import csv
import sys
import bisect
import statistics as st
from collections import defaultdict


def _num(s, d=0.0):
    try:
        return float(s)
    except (TypeError, ValueError):
        return d


def load(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def col(r, k, d=0.0):
    v = r.get(k)
    return _num(v, d)


def align(sim, real):
    """按 wall_ms 对齐：sim 每点找 real 最近 wall_ms。返回 (pairs, max_dt_ms)。"""
    real_wall = [col(r, "wall_ms") for r in real]
    pairs = []
    max_dt = 0.0
    for r in sim:
        w = col(r, "wall_ms")
        i = bisect.bisect_left(real_wall, w)
        cand = [j for j in (i, i - 1) if 0 <= j < len(real)]
        if not cand:
            continue
        j = min(cand, key=lambda k: abs(real_wall[k] - w))
        dt = abs(real_wall[j] - w)
        max_dt = max(max_dt, dt)
        pairs.append((r, real[j], dt))
    return pairs, max_dt


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    sim_path, rl_path = sys.argv[1], sys.argv[2]
    recv_path = sys.argv[3] if len(sys.argv) > 3 else None

    sim = load(sim_path)
    real = load(rl_path)

    # 排序（wall_ms 单调）
    sim.sort(key=lambda r: col(r, "wall_ms"))
    real.sort(key=lambda r: col(r, "wall_ms"))

    pairs, max_dt = align(sim, real)
    print(f"sim {len(sim)} 步, real {len(real)} 步, wall_ms 对齐 {len(pairs)} 对, "
          f"最大时间差 {max_dt:.0f} ms")

    if not pairs:
        print("[ERROR] 无对齐对（检查两边 wall_ms）")
        sys.exit(1)

    # ---- 1) cmd 一致性（验证对齐质量）----
    cmd_err = [abs(col(r, "cmd_vx") - col(rr, "cmd_vx")) +
               abs(col(r, "cmd_wz") - col(rr, "cmd_wz")) for r, rr, _ in pairs]
    print(f"\n=== cmd 对齐一致性 ===\n  vx/wz 平均 |diff| = {st.mean(cmd_err):.3f} "
          f"(<0.05 说明两边命令一致，对齐可靠)")

    # ---- 2) 轮速 vel_12..15（POLICY 轮）----
    print("\n=== 轮速 vel_12..15 (rad/s) ===")
    for i in range(4):
        sv = [col(r, f"vel_{12 + i:02d}") for r, rr, _ in pairs]
        rv = [col(rr, f"vel_{12 + i:02d}") for r, rr, _ in pairs]
        print(f"  轮{i}: sim均值{st.mean(sv):+7.2f} std{st.pstdev(sv):5.2f} | "
              f"real均值{st.mean(rv):+7.2f} std{st.pstdev(rv):5.2f}")

    # ---- 3) 腿 qrel（POLICY 0..11）----
    real_qrel_cols = [k for k in real[0].keys() if k.startswith("qrel")][:12]
    print("\n=== 腿 qrel_0..11 (rad) ===")
    print("  idx  sim均值   simstd   real均值  realstd")
    for i in range(12):
        sv = [col(r, f"qrel_{i}") for r, rr, _ in pairs]
        rv = [col(rr, real_qrel_cols[i]) for r, rr, _ in pairs]
        print(f"  {i:3d} {st.mean(sv):+7.3f} {st.pstdev(sv):7.3f} "
              f"{st.mean(rv):+8.3f} {st.pstdev(rv):8.3f}")

    # ---- 4) 姿态 pgr_z ----
    sp = [col(r, "pgr_z") for r, rr, _ in pairs]
    rp = [col(rr, "pgr_z") for r, rr, _ in pairs]
    print(f"\n=== 姿态 pgr_z ===\n  sim:  均值{st.mean(sp):+.3f} std{st.pstdev(sp):.3f}\n"
          f"  real: 均值{st.mean(rp):+.3f} std{st.pstdev(rp):.3f}")

    # ---- 5) 编码器绝对位置波动（sim qpos vs real recv cal_pos）----
    if recv_path:
        recv = load(recv_path)
        recv.sort(key=lambda r: col(r, "wall_ms"))
        # 聚合：每个 wall_ms 时刻的 12 腿 cal_pos（CAN order m1..m3）
        # 取每时刻每个电机最新读数
        leg_pos = defaultdict(dict)   # wall_ms -> {mjx: cal_pos}
        for r in recv:
            w = int(col(r, "wall_ms"))
            cp = int(col(r, "can_port"))
            mi = int(col(r, "motor_id"))
            if mi <= 3:
                leg_pos[w][cp * 3 + mi - 1] = col(r, "cal_pos")
        # 对 sim 每点，找最近 wall_ms 的 recv 快照
        recv_wall = sorted(leg_pos.keys())
        print("\n=== 编码器位置波动（std, rad；零点不同比波动）===")
        print("  idx  simqposstd  realcalstd")
        for i in range(12):
            sq = [col(r, f"qpos_{i:02d}") for r, rr, _ in pairs]
            rc = []
            for r, rr, _ in pairs:
                w = int(col(r, "wall_ms"))
                j = bisect.bisect_left(recv_wall, w)
                snap = None
                for k in (j, j - 1):
                    if 0 <= k < len(recv_wall) and i in leg_pos[recv_wall[k]]:
                        snap = leg_pos[recv_wall[k]][i]
                        break
                if snap is not None:
                    rc.append(snap)
            if len(rc) > 5:
                print(f"  {i:3d} {st.pstdev(sq):8.3f}  {st.pstdev(rc):9.3f}")

    print("\n[INFO] 判读：")
    print("  · cmd 平均 |diff| < 0.05 → 对齐可靠，对比可信")
    print("  · 轮速/腿 qrel：sim vs real 方向一致、幅值相近 = sim2real 一致性好；"
          "幅值差大 → 执行差异（延迟/摩擦）")
    print("  · 编码器波动：real 明显大于 sim → 真机步态更抖（延迟/摩擦）")


if __name__ == "__main__":
    main()
