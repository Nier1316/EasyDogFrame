#!/usr/bin/env python3
"""Example62 采集的真机 CSV → PACE 的 chirp_data.pt（time / dof_pos / des_dof_pos）。

用法：
    python3 tool/pace_export_dataset.py log/pace/chirp_<ts>.csv [--rate 400] [--out DIR] [--force]

依据（arXiv 2509.06342 §2.1/§2.2）：
  · PACE 的数据就是三个张量：time(N,)、dof_pos(N,12)、des_dof_pos(N,12)，
    列序 = PACE 的 joint_order（LF/RF/LH/RH × HAA/HFE/KFE，按腿分组）。
    Example62 的 CSV 列序与之**一一对应**（本框架 CAN 序也是按腿分组），无需重排。
  · **时间基必须严格等距**（移植版明确：不匹配的真实数据会被拒绝，而不是悄悄改 delay/damping 估计）。
    故这里用理想网格 time = arange(N)/rate；CSV 里的 t_actual_s 只用于**校验实际周期偏差**。
  · 校验不通过（周期偏差 >1%、有 >3× 周期的长跳、NaN、拍数不足）默认**报错退出**；--force 可强制导出并打印警告。

需要 torch（用识别用的 conda 环境跑）：
    /home/sysu/miniconda3/envs/pace_mjlab/bin/python tool/pace_export_dataset.py ...
"""

from __future__ import annotations
import argparse, csv, sys
from pathlib import Path

JOINTS = ["LF_HAA","LF_HFE","LF_KFE","RF_HAA","RF_HFE","RF_KFE",
          "LH_HAA","LH_HFE","LH_KFE","RH_HAA","RH_HFE","RH_KFE"]

def main() -> int:
    ap = argparse.ArgumentParser(description="Example62 CSV → PACE chirp_data.pt")
    ap.add_argument("csv")
    ap.add_argument("--rate", type=float, default=400.0, help="辨识采样率 (Hz)，PACE 整机用 400")
    ap.add_argument("--out", default=None, help="输出目录（默认：CSV 同目录）")
    ap.add_argument("--force", action="store_true", help="校验不通过也导出（会打印警告）")
    a = ap.parse_args()

    src = Path(a.csv)
    if not src.exists():
        print(f"ERROR: 找不到 {src}", file=sys.stderr); return 2
    rows = list(csv.DictReader(src.open()))
    n = len(rows)
    if n < 100:
        print(f"ERROR: 有效拍数过少 ({n})", file=sys.stderr); return 2

    need = [f"des_{j}" for j in JOINTS] + [f"meas_{j}" for j in JOINTS]
    missing = [c for c in need if c not in rows[0]]
    if missing:
        print(f"ERROR: CSV 缺列: {missing[:4]}...", file=sys.stderr); return 2

    des = [[float(r[f"des_{j}"])  for j in JOINTS] for r in rows]
    pos = [[float(r[f"meas_{j}"]) for j in JOINTS] for r in rows]
    t_act = [float(r["t_actual_s"]) for r in rows]

    # ---- 时间基校验 ----
    dts = [t_act[i+1]-t_act[i] for i in range(n-1)]
    mean_dt = sum(dts)/len(dts)
    tgt = 1.0/a.rate
    dev = (mean_dt-tgt)/tgt*100.0
    worst = max(dts)
    ok = abs(dev) < 1.0 and worst < 3*tgt
    print(f"拍数={n}  实际平均周期={mean_dt:.6f}s（目标 {tgt:.6f}）偏差={dev:+.3f}%  最大跳={worst:.6f}s")
    print(f"时间基校验: {'通过' if ok else '**不通过**'}")
    if not ok and not a.force:
        print("ERROR: 时间基不达标 —— 该次采集应作废重采（或加 --force 强制导出，但 delay/damping 估计会失真）",
              file=sys.stderr)
        return 3

    try:
        import torch
    except ImportError:
        print("ERROR: 需要 torch。请用识别环境：/home/sysu/miniconda3/envs/pace_mjlab/bin/python", file=sys.stderr)
        return 4

    # 理想等距网格（PACE 要求与仿真速率一致）
    time = torch.arange(n, dtype=torch.float32) / a.rate
    out_dir = Path(a.out) if a.out else src.parent
    out = out_dir / "chirp_data.pt"
    torch.save({"time": time,
                "dof_pos": torch.tensor(pos, dtype=torch.float32),
                "des_dof_pos": torch.tensor(des, dtype=torch.float32)}, out)
    print(f"已写出 {out}  （time {tuple(time.shape)}, dof_pos ({n},12), des_dof_pos ({n},12)）")
    print(f"目标幅值范围: {min(min(r) for r in des):+.3f} ~ {max(max(r) for r in des):+.3f} rad")
    return 0

if __name__ == "__main__":
    sys.exit(main())
