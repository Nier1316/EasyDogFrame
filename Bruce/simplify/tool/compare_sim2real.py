#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim2real 双开对比：sim2sim --record vs 真机 rl/recv 日志，wall_ms 时间戳对齐

用法:
    python3 tool/compare_sim2real.py <sim.csv> <real_rl.csv> [real_recv.csv] [--dataset log/dataset_*.csv]

- sim.csv:    sim2sim --record 输出（wall_ms,qpos_16,qrel,vel,act,pgr）
- real_rl.csv: 真机 log/rl_*.csv（wall_ms,cmd,qrel,vel,act,pgr，POLICY order）
- real_recv.csv（可选）: 真机 log/recv_*.csv（wall_ms,cal_pos/vel/torque，CAN order，500Hz）
- --dataset（可选，新增）: 统一数据集 log/dataset_*.csv（m_tau_00..15，CAN order），
  供新增的 (a) τ 对比章节使用；不给则退回 recv 的 cal_torque。

对齐：两边按 wall_ms（系统时间戳）对齐，逐点对比。
对比：cmd 一致性（验证对齐质量）、轮速 vel_12..15、腿 qrel、姿态 pgr_z、
      编码器位置波动（sim qpos vs real cal_pos）。
注意：sim qpos 是 URDF 角（POLICY），real cal_pos 是真机标定角（CAN）——零点不同，
      对比波动(std/范围)而非绝对值。

[新增章节，追加在既有输出之后，不改动既有格式]
  (a) 关节力矩 τ 对比：每关节均值/std/相关系数，标出「符号相反」的关节
      （sim CSV 无 τ 列时明确提示给 sim2sim --record 加 τ 列）；
  (b) 互相关时移：对轮速/腿 qrel/pgr_z(/τ) 计算 sim↔real 互相关峰值时移(ms)，
      给出「各量最佳时移」表，并按该时移补偿后重算残差（补偿前/后对比）。
"""
import csv
import os
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
    argv = sys.argv[1:]
    dataset_path = None
    if "--dataset" in argv:                      # 新增可选参数，不改变原有位置参数行为
        k = argv.index("--dataset")
        if k + 1 >= len(argv):
            print("[ERROR] --dataset 后面要给统一数据集 CSV 路径（log/dataset_*.csv）")
            sys.exit(1)
        dataset_path = argv[k + 1]
        del argv[k:k + 2]
    if len(argv) < 2:
        print(__doc__)
        sys.exit(1)
    sim_path, rl_path = argv[0], argv[1]
    recv_path = argv[2] if len(argv) > 2 else None

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

    # ==================================================================
    # [新增] 追加章节：不改动以上任何输出与命令行行为，只在其后追加
    #   (a) τ（关节力矩）对比 + 符号相反检查
    #   (b) sim↔real 互相关时移 + 补偿前/后残差
    # ==================================================================
    appendix(pairs, sim, real, recv_path, dataset_path)


# ============================================================================
# [新增] 以下均为追加章节的实现（stdlib + numpy；numpy 缺失时优雅跳过）
# ============================================================================
# CAN -> POLICY 关节置换（POLICY_TO_MJX，见 include/strategy/rl_controller.h）
CAN_TO_POLICY = [0, 1, 2, 12, 3, 4, 5, 13, 6, 7, 8, 14, 9, 10, 11, 15]
_POLICY_TO_CAN = CAN_TO_POLICY          # can = POLICY_TO_MJX[policy]
_LEG = ("FL", "FR", "RL", "RR")
_PART = ("hip", "thigh", "calf", "wheel")


def _cname(i):
    return "%s-%s(can%02d)" % (_LEG[i // 4], _PART[i % 4], i)


def _ncol(r, k, default=float("nan")):
    v = r.get(k)
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _recv_tau_snapshots(recv_rows):
    """log/recv_*.csv -> (sorted wall list, [np.array(16) CAN 序 cal_torque])。"""
    import numpy as _np
    snap = {}
    for r in recv_rows:
        cp = int(_ncol(r, "can_port", -1))
        mi = int(_ncol(r, "motor_id", -1))
        if not (0 <= cp <= 3 and 1 <= mi <= 4):
            continue
        snap.setdefault(int(_ncol(r, "wall_ms", 0)), {})[cp * 4 + mi - 1] = _ncol(r, "cal_torque")
    walls = sorted(snap.keys())
    arrs = [_np.array([snap[w].get(i, _np.nan) for i in range(16)]) for w in walls]
    return walls, arrs


def _nearest(walls, w):
    i = bisect.bisect_left(walls, w)
    best = None
    for j in (i, i - 1):
        if 0 <= j < len(walls):
            if best is None or abs(walls[j] - w) < abs(walls[best] - w):
                best = j
    return best


def _dataset_tau(ds):
    """统一数据集 m_tau_00..15 -> (sorted wall list, [np.array(16) CAN 序])。"""
    import numpy as _np
    if not all(ds.has("m_tau_%02d" % i) for i in range(16)):
        return None, None
    w = ds.get("wall_ms")
    order = _np.argsort(w)
    cols = [_np.array(ds.get("m_tau_%02d" % i), dtype=float)[order] for i in range(16)]
    arrs = [_np.stack([c[k] for c in cols]) for k in range(ds.n)]
    return [float(w[k]) for k in order], arrs


def _sim_tau_spec(sim):
    """识别 sim CSV 的 τ 列。返回 (描述, 取值函数) 或 (None, None)。"""
    keys = list(sim[0].keys())
    t16 = ["tau_%02d" % i for i in range(16)]
    if all(k in keys for k in t16):
        return ("tau_00..15（按 POLICY 序解释）",
                lambda r: [col(r, k) for k in t16])
    if all(k in keys for k in ["tau_w%d" % i for i in range(4)]):
        return ("tau_w0..3（POLICY 轮 12..15）",
                lambda r: [col(r, "tau_w%d" % (p - 12)) if p >= 12 else float("nan")
                           for p in range(16)])
    if all(k in keys for k in ["m_tau_%02d" % i for i in range(16)]):
        return ("m_tau_00..15（按 POLICY 序解释）",
                lambda r: [col(r, "m_tau_%02d" % i) for i in range(16)])
    return (None, None)


def tau_section(pairs, sim, real, recv_rows, dataset_path, np):
    """(a) τ 对比：每关节均值/标准差/相关系数 + 符号相反标记。"""
    print("\n=== [新增] (a) 关节力矩 τ 对比 ===")
    sim_desc, sim_fn = _sim_tau_spec(sim)
    real_walls = real_arrs = None
    real_src = None
    if dataset_path:
        try:
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from delay_fit import load_dataset, DatasetError, joint_name as _jn  # noqa
            ds = load_dataset(dataset_path)
            real_walls, real_arrs = _dataset_tau(ds)
            real_src = "dataset m_tau_*（CAN 序）: %s" % dataset_path
        except Exception as e:                       # 容错：任何问题都只提示不中断
            print("  [提示] 读 --dataset 失败(%s)，改用 recv cal_torque" % e)
            real_walls = real_arrs = None
    if real_arrs is None and recv_rows:
        real_walls, real_arrs = _recv_tau_snapshots(recv_rows)
        real_src = "log/recv_*.csv 的 cal_torque（CAN 序）"
    if real_arrs is None:
        print("  [提示] 没有 real 侧 τ 数据（给 --dataset log/dataset_*.csv，或第 3 个参数传"
              " log/recv_*.csv）")

    if sim_desc is None:
        print("  [提示] sim CSV 未记录 τ 列 → 无法做 sim↔real 力矩对比。")
        print("         现有 sim 字段: %s ..." % ",".join(list(sim[0].keys())[:10]))
        print("         → 建议给 sim2sim --record 加 τ 列（在 _open_record 的 fields 里加"
              " m_tau_00..15 或 tau_00..15），本工具会自动识别。")
    if sim_desc is None and real_arrs is None:
        return
    if real_src:
        print("  real τ 来源: %s" % real_src)
    if sim_desc:
        print("  sim τ 来源: %s" % sim_desc)

    n = len(pairs)
    # real τ CAN 序 16 通道（对齐到 pairs）
    r_can = np.full((n, 16), np.nan)
    if real_arrs is not None and real_walls:
        for k, (_, rr, _) in enumerate(pairs):
            j = _nearest(real_walls, col(rr, "wall_ms"))
            if j is not None:
                r_can[k, :] = real_arrs[j]
    # sim τ → CAN 序
    s_can = np.full((n, 16), np.nan)
    if sim_fn is not None:
        for k, (r, _, _) in enumerate(pairs):
            pv = sim_fn(r)
            for p in range(16):
                s_can[k, _POLICY_TO_CAN[p]] = pv[p]

    # sim 无 τ 但有 act_* 时，用 act 作代理做同号性检查
    proxy = None
    if sim_fn is None and all("act_%02d" % i in sim[0] for i in range(12)):
        proxy = np.full((n, 12), np.nan)
        for k, (r, _, _) in enumerate(pairs):
            for p in range(12):
                proxy[k, _POLICY_TO_CAN[p]] = col(r, "act_%02d" % p)

    joints = [i for i in range(16)
              if np.isfinite(r_can[:, i]).any() or (sim_fn and np.isfinite(s_can[:, i]).any())]
    if not joints:
        print("  [提示] real τ 通道全为 NaN（recv 里可能没有对应电机），跳过逐关节对比。")
        return
    print("\n  can  关节           sim均值  simstd | real均值 realstd |  相关系数  判定")
    flags = []
    for i in joints:
        rs = r_can[:, i]
        rr_ok = np.isfinite(rs)
        if not rr_ok.any():
            continue
        rmean, rstd = float(np.nanmean(rs)), float(np.nanstd(rs))
        if sim_fn is not None:
            ss = s_can[:, i]
            m = np.isfinite(ss) & np.isfinite(rs)
            if m.sum() > 10 and np.std(ss[m]) > 1e-9 and np.std(rs[m]) > 1e-9:
                cc = float(np.corrcoef(ss[m], rs[m])[0, 1])
            else:
                cc = float("nan")
            jd = "—"
            if np.isfinite(cc) and cc < -0.2:
                jd = "!! 符号相反（量纲/坐标系错误，不是模型误差）"
                flags.append(i)
            print("  %2d  %-13s %+7.3f %7.3f | %+8.3f %7.3f | %+9.3f  %s"
                  % (i, _cname(i), float(np.nanmean(ss)), float(np.nanstd(ss)),
                     rmean, rstd, cc, jd))
        else:
            extra = ""
            if proxy is not None:
                pp = proxy[:, i]
                m = np.isfinite(pp) & np.isfinite(rs)
                if m.sum() > 10 and np.std(pp[m]) > 1e-9 and np.std(rs[m]) > 1e-9:
                    cp = float(np.corrcoef(pp[m], rs[m])[0, 1])
                    extra = " | corr(real τ, sim act)=%+.3f%s" % (
                        cp, "  ← 疑似符号相反" if cp < -0.2 else "")
            print("  %2d  %-13s %7s %7s | %+8.3f %7.3f | %9s  %s"
                  % (i, _cname(i), "—", "—", rmean, rstd, "n/a(sim无τ)", extra))
    if sim_fn is not None:
        if flags:
            print("  [结论] %d 个关节 sim↔real τ 反相: %s —— 优先查标定系数量纲/坐标系符号"
                  % (len(flags), ", ".join(_cname(i) for i in flags)))
        else:
            print("  [结论] 未发现 sim↔real τ 反相关节（相关系数均 ≥ -0.2）")
    else:
        print("  [结论] sim 未记录 τ：先给 sim2sim --record 加 τ 列，才能判定符号/量纲"
              "（上面的 act 代理只能粗略提示）")


def _xcorr_shift(s, r, dt_ms, max_lag_ms=None):
    """均匀网格上做互相关：正时移 = real 相对 sim 滞后。返回 (shift_ms, corr_peak)。"""
    import numpy as _np
    s = _np.asarray(s, float)
    r = _np.asarray(r, float)
    m = _np.isfinite(s) & _np.isfinite(r)
    if m.sum() < 20:
        return None
    s, r = s[m], r[m]
    a = s - s.mean()
    b = r - r.mean()
    if _np.std(a) <= 0 or _np.std(b) <= 0:
        return None
    n = a.size
    if max_lag_ms is None:
        max_lag_ms = min(400.0, max(2.0 * dt_ms, n * dt_ms * 0.25))
    max_lag = max(1, int(round(max_lag_ms / dt_ms)))
    lags = _np.arange(-max_lag, max_lag + 1)
    rs = _np.full(lags.size, _np.nan)
    for k, lag in enumerate(lags):
        if lag >= 0:
            x, y = a[:n - lag], b[lag:]
        else:
            x, y = a[-lag:], b[:n + lag]
        if x.size < 10:
            continue
        x = x - x.mean()
        y = y - y.mean()
        den = (x * x).sum() ** 0.5 * (y * y).sum() ** 0.5
        if den > 0:
            rs[k] = float((x * y).sum() / den)
    if not _np.isfinite(rs).any():
        return None
    j = int(_np.nanargmax(rs))
    lag = int(lags[j])
    d = 0.0
    if 0 < j < rs.size - 1 and _np.isfinite(rs[j - 1]) and _np.isfinite(rs[j + 1]):
        y0, y1, y2 = float(rs[j - 1]), float(rs[j]), float(rs[j + 1])
        den = y0 - 2 * y1 + y2
        if abs(den) > 1e-12:
            d = float(min(1.0, max(-1.0, 0.5 * (y0 - y2) / den)))
    return (lag + d) * dt_ms, float(rs[j])


def _resid(s, r):
    import numpy as _np
    s = _np.asarray(s, float)
    r = _np.asarray(r, float)
    m = _np.isfinite(s) & _np.isfinite(r)
    if m.sum() < 10:
        return float("nan"), float("nan"), 0
    s, r = s[m], r[m]
    scale = float(_np.std(r))
    if scale <= 0:
        return float("nan"), float("nan"), int(m.sum())
    rms = float(_np.sqrt(_np.mean((s - r) ** 2)))
    cc = float(_np.corrcoef(s, r)[0, 1]) if _np.std(s) > 0 else float("nan")
    return 100.0 * rms / scale, cc, int(m.sum())


def _pair_series(pairs, ks, kr):
    import numpy as _np
    return (_np.array([_ncol(r, ks) for r, _, _ in pairs]),
            _np.array([_ncol(rr, kr) for _, rr, _ in pairs]),
            _np.array([col(r, "wall_ms") for r, _, _ in pairs]))


def shift_section(pairs, sim, real, np, tau_pack=None):
    """(b) 每类量的 sim↔real 互相关时移 + 补偿前/后残差。"""
    print("\n=== [新增] (b) 互相关时移（sim ↔ real 时序对齐余量）===")
    real_qrel_cols = [k for k in real[0].keys() if k.startswith("qrel")][:12]
    if not real_qrel_cols:
        real_qrel_cols = ["qrel_%d" % i for i in range(12)]

    chan = []                                   # (组, 名字, sim键, real键)
    for i in range(4):
        chan.append(("轮速", "wheel%d(vel_%02d)" % (i, 12 + i),
                     "vel_%02d" % (12 + i), "vel_%02d" % (12 + i)))
    for i in range(12):
        chan.append(("腿 qrel", "qrel_%d" % i, "qrel_%d" % i, real_qrel_cols[i]))
    chan.append(("姿态", "pgr_z", "pgr_z", "pgr_z"))

    sim_wall = np.array([col(r, "wall_ms") for r, _, _ in pairs])
    dt = float(np.median(np.diff(sim_wall))) if sim_wall.size > 2 else 20.0
    if not np.isfinite(dt) or dt <= 0:
        dt = 20.0
    print("  sim 采样间隔(中位) = %.1f ms → 时移表的分辨率即 %.1f ms（sim2sim --record 是控制步"
          " 50Hz）" % (dt, dt))
    print("  约定：正时移 = real 相对 sim 滞后（单位 ms）；残差按 real 的 std 归一化(%)")
    print("\n  量                 最佳时移(ms)  补偿前残差%  补偿后残差%  下降  补偿前corr  补偿后corr")
    groups = {}
    for gname, label, ks, kr in chan:
        if ks not in sim[0] or kr not in real[0]:
            continue
        s, r, w = _pair_series(pairs, ks, kr)
        grid = np.arange(w[0], w[-1] + 1e-9, dt)
        if grid.size < 25:
            continue
        sg = np.interp(grid, w, s, left=np.nan, right=np.nan)
        rg = np.interp(grid, w, r, left=np.nan, right=np.nan)
        res = _xcorr_shift(sg, rg, dt)
        if res is None:
            continue
        sh, cpeak = res
        b_pct, b_corr, npts = _resid(sg, rg)
        r_shift = np.interp(grid, grid + sh, rg, left=np.nan, right=np.nan)
        a_pct, a_corr, _ = _resid(sg, r_shift)
        drop = (100.0 * (b_pct - a_pct) / b_pct) if (np.isfinite(b_pct) and b_pct > 0) else float("nan")
        print("  %-19s %+10.2f  %11.2f  %11.2f  %5.1f%% %+10.3f  %+10.3f"
              % (label, sh, b_pct, a_pct, drop, b_corr, a_corr))
        groups.setdefault(gname, []).append((label, sh, b_pct, a_pct, b_corr, a_corr))

    if not groups:
        print("  [提示] 没有可比的通道（sim/real 列名不匹配或数据太少）。")
        return
    print("\n  ---- 各量最佳时移汇总 ----")
    print("  量类        通道数  平均时移(ms)  中位时移(ms)  补偿前残差%  补偿后残差%")
    for gname, rows in groups.items():
        shs = np.array([x[1] for x in rows], float)
        b = np.array([x[2] for x in rows], float)
        a = np.array([x[3] for x in rows], float)
        print("  %-10s %5d %13.2f %13.2f %12.2f %12.2f"
              % (gname, len(rows), float(np.mean(shs)), float(np.median(shs)),
                 float(np.nanmean(b)), float(np.nanmean(a))))
    print("  [判读] 时移明显非零(>1 个采样间隔) 说明两边时序没对齐（延迟差/时钟起点差）；")
    print("         上表已按各自时移补偿后重算残差：若「补偿后残差」明显低于「补偿前」，")
    print("         则第 4/5 列的模型差异里含时序错位成分，不应全部算作模型误差。")


def appendix(pairs, sim, real, recv_path, dataset_path):
    """新增章节总入口（numpy 缺失则优雅跳过）。"""
    try:
        import numpy as np
    except ImportError:
        print("\n[INFO] 未安装 numpy，跳过新增的 τ 对比 / 互相关时移章节"
              "（pip install numpy 后重跑）")
        return
    recv_rows = None
    if recv_path:
        try:
            recv_rows = load(recv_path)
        except OSError as e:
            print("\n[提示] 读取 recv CSV 失败(%s)，τ 对比将只用 --dataset" % e)
    try:
        tau_section(pairs, sim, real, recv_rows, dataset_path, np)
    except Exception as e:                       # 新增章节任何异常都不影响既有输出
        print("\n[提示] τ 对比章节出错，已跳过: %s" % e)
    try:
        shift_section(pairs, sim, real, np)
    except Exception as e:
        print("\n[提示] 互相关时移章节出错，已跳过: %s" % e)


if __name__ == "__main__":
    main()
