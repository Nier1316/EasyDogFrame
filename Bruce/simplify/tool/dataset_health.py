#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dataset_health.py —— 统一数据集 CSV 的中文体检报告

用法:
    python3 tool/dataset_health.py log/dataset_*.csv [--no-plot] [--quiet]
    python3 tool/dataset_health.py --selftest [--tmpdir /tmp]

输出内容:
    1) 基本信息：行数 / 时长 / 实际采样率 / 表头是否符合冻结 schema / 同名 .meta.txt
    2) 时间轴：实际平均/中位/最大行间隔；>5ms 的间隔数与占比（卡顿/丢帧）；
       wall_ms 与 t_ms 是否单调；wall_ms 与 t_ms 的相对漂移
    3) 每列 NaN / 异常值检查（区分「恒 0」与「有效 0」，未轮询的 temp/vbus=0 属正常）
    4) 16 个 m_pos / m_tau 的范围与标准差
    5) c_mode 取值分布（含非法值）
    6) 命令活跃度：c_* 是否长期不变（长期不变 ⇒ 该段对辨识无用）
    7) 结论：「这段数据能用来做什么」——能否测延迟 / 拟合轮速环 / 拟合执行器
    8) matplotlib 可用时把 4 个轮子的 c_vel vs m_vel 叠图存到同目录 health_*.png

================= 冻结数据集 schema（188 列，UTF-8，逗号分隔，\\n 行尾）=================
 1. wall_ms                     系统墙钟 epoch 毫秒(int)，与 sim2sim --record / log/rl_*.csv 可直接对齐
 2. t_ms                        相对录制开始毫秒(float，3 位小数，单调递增)
 3. c_mode_00 .. c_mode_15      int：0=IMPEDANCE 1=SPEED 2=POSITION
 4. c_pos_00 .. c_pos_15        指令(按该行 c_mode_i 解释)
 5. c_vel_00 .. c_vel_15
 6. c_kp_00  .. c_kp_15
 7. c_kd_00  .. c_kd_15
 8. c_tau_00 .. c_tau_15
 9. m_pos_00 .. m_pos_15        反馈位置
10. m_vel_00 .. m_vel_15        反馈速度
11. m_tau_00 .. m_tau_15        反馈力矩
12. m_temp_00 .. m_temp_15      电机温度 °C（1Hz 轮询全部 16 个；未轮询到 = 0）
13. m_vbus_00 .. m_vbus_15      母线电压 V（1Hz 只轮询每路 CAN 的 1 号电机 i%4==0；其余 0/保持）
14. gyro_0, gyro_1, gyro_2      机体系角速度 rad/s
15. quat_w, quat_x, quat_y, quat_z   body←world，w 在前
16. cmd_vx, cmd_vy, cmd_wz      上层速度命令（站立即 0）

c_* 语义（按行内 c_mode_i 解释）:
    mode=0 IMPEDANCE：c_pos=目标位置, c_vel=目标速度, c_kp=kp, c_kd=kd, c_tau=τ_ff
    mode=1 SPEED    ：c_pos=0,        c_vel=目标速度, c_kp=kvp, c_kd=0,  c_tau=ki
    mode=2 POSITION ：c_pos=目标位置, c_vel=0,        c_kp=kvp, c_kd=kp,  c_tau=kvi

电机索引（CAN 顺序）: i = can_port*4 + (motor_id-1)
    00=FL-hip 01=FL-thigh 02=FL-calf 03=FL-wheel  ... 12=RR-hip 13=RR-thigh 14=RR-calf 15=RR-wheel
    CAN→POLICY: [0,1,2,12, 3,4,5,13, 6,7,8,14, 9,10,11,15]（见 include/strategy/rl_controller.h）

依赖：stdlib + numpy（matplotlib 可选，缺了只跳过画图）。
底层读取/阶跃检测复用同目录 delay_fit.py。
"""
import argparse
import os
import sys

try:
    import numpy as np
except ImportError:  # pragma: no cover
    sys.stderr.write("[ERROR] 需要 numpy：pip install numpy（本工具只用 stdlib + numpy）\n")
    sys.exit(2)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from delay_fit import (DatasetError, EXPECTED_NCOL, joint_name, load_dataset,
                           detect_steps, pick_signal, write_synthetic_dataset)
except ImportError as e:  # pragma: no cover
    sys.stderr.write("[ERROR] 无法导入同目录 delay_fit.py: %s\n" % e)
    sys.exit(2)

WHEELS = (3, 7, 11, 15)
LEGS = [i for i in range(16) if i % 4 != 3]


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def _nan_stats(v):
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return None
    return {"n": int(v.size), "min": float(v.min()), "max": float(v.max()),
            "mean": float(v.mean()), "std": float(v.std())}


def _is_const(v, tol=1e-9):
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return True, float("nan")
    c0 = float(v[0])
    return bool(np.all(np.abs(v - c0) <= tol)), c0


def _changes(v, tol_ratio=1e-6):
    """变化次数（相对动态范围容差）+ 唯一值数。"""
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    if v.size < 2:
        return 0, int(np.unique(v).size)
    rng = float(np.percentile(v, 99) - np.percentile(v, 1))
    tol = max(rng * tol_ratio, 1e-12)
    d = np.abs(np.diff(v))
    return int(np.sum(d > tol)), int(np.unique(np.round(v / max(tol, 1e-12))).size)


def _corr(a, b):
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    m = np.isfinite(a) & np.isfinite(b)
    a, b = a[m], b[m]
    if a.size < 8:
        return float("nan")
    a = a - a.mean()
    b = b - b.mean()
    den = np.sqrt(float((a * a).sum()) * float((b * b).sum()))
    if den <= 0:
        return float("nan")
    return float((a * b).sum() / den)


def read_meta(path):
    """同名 .meta.txt（key=value）。返回 dict 或 None。"""
    stem = os.path.splitext(path)[0]
    for cand in (stem + ".meta.txt", stem + ".meta"):
        if os.path.isfile(cand):
            d = {}
            try:
                with open(cand, "r", encoding="utf-8", errors="replace") as f:
                    for line in f:
                        line = line.strip()
                        if not line or "=" not in line:
                            continue
                        k, v = line.split("=", 1)
                        d[k.strip()] = v.strip()
            except OSError:
                return None
            return d
    return None


# ---------------------------------------------------------------------------
# 报告主体
# ---------------------------------------------------------------------------
def report(path, do_plot=True, quiet=False):
    """打印体检报告，返回 findings dict（供 selftest 断言）。"""
    print("=" * 78)
    print("数据集体检 (dataset_health.py): %s" % path)
    print("=" * 78)
    ds = load_dataset(path)
    t = ds.t
    n = ds.n
    dt = ds.dt_ms()
    f = {"path": path, "n": n, "dt_ms": dt}

    # ---- 0) meta ----
    meta = read_meta(path)
    if meta:
        keys = ("note", "start_ts", "freq", "motor_order", "imu", "dropped", "weight")
        print("\n[0] meta 文件: %s" % ", ".join(
            "%s=%s" % (k, meta[k]) for k in keys if k in meta) or "(空)")
    else:
        print("\n[0] 未找到同名 .meta.txt（建议录制器写 note/weight/dropped 便于追溯）")

    # ---- 1) 基本信息 ----
    dur_s = float(t[-1] - t[0]) / 1000.0 if n > 1 else 0.0
    rate = n / dur_s if dur_s > 0 else float("nan")
    print("\n[1] 基本信息")
    print("  行数 %d，时长 %.3f s，实际平均采样率 %.2f Hz（标称 500Hz）" % (n, dur_s, rate))
    if len(ds.header) == EXPECTED_NCOL:
        print("  表头 %d 列，符合冻结 schema" % len(ds.header))
    else:
        print("  [警告] 表头 %d 列 != 期望 %d 列" % (len(ds.header), EXPECTED_NCOL))
    for w in ds.warnings:
        print("  [警告] %s" % w)
    if ds.n_mismatch:
        print("  [警告] 有 %d 行列数与表头不符被跳过" % ds.n_mismatch)

    # ---- 2) 时间轴 ----
    print("\n[2] 时间轴")
    if n > 1:
        d = np.diff(t)
        dfin = d[np.isfinite(d)]
        big = dfin > 5.0
        print("  行间隔: 平均 %.3f ms, 中位 %.3f ms, 最大 %.3f ms"
              % (float(dfin.mean()), float(np.median(dfin)), float(dfin.max())))
        print("  >5ms 的间隔: %d 个 (%.3f%%) %s"
              % (int(big.sum()), 100.0 * big.mean(),
                 "← 卡顿/丢帧，需注意" if big.any() else "← 无卡顿"))
        if big.any():
            idx = np.where(big)[0]
            worst = np.argsort(-dfin[idx])[:5]
            print("  最严重的 5 处: %s" % ", ".join(
                "t=%.1fms 间隔%.1fms" % (t[idx[k]], dfin[idx[k]]) for k in worst))
        f["n_big_gap"] = int(big.sum())
        wall = ds.wall
        if wall is not None:
            wd = np.diff(wall)
            n_dec = int(np.sum(wd < 0))
            print("  wall_ms 单调: %s（逆序 %d 处）" % ("是" if n_dec == 0 else "否", n_dec))
            est = wall - wall[0] - t
            print("  wall_ms-t_ms 相对漂移: 均值 %.2f ms, 最大|偏差| %.2f ms"
                  % (float(np.mean(est)), float(np.max(np.abs(est)))))
            f["wall_mono"] = (n_dec == 0)
        n_dec_t = int(np.sum(d < 0))
        n_eq_t = int(np.sum(np.abs(d) < 1e-9))
        print("  t_ms 单调递增: %s（逆序 %d 处, 相等 %d 处）"
              % ("是" if (n_dec_t == 0 and n_eq_t == 0) else "否", n_dec_t, n_eq_t))
        f["t_mono"] = (n_dec_t == 0 and n_eq_t == 0)

    # ---- 3) NaN / 异常 ----
    print("\n[3] NaN / 非数值检查")
    nan_cols = []
    for name in ds.header:
        v = ds.get(name)
        if v is None:
            continue
        k = int(np.sum(~np.isfinite(v)))
        if k:
            nan_cols.append((name, k))
    if nan_cols:
        print("  含 NaN 的列 %d 个: %s" % (len(nan_cols), ", ".join(
            "%s×%d" % (a, b) for a, b in nan_cols[:12])))
        f["nan_cols"] = len(nan_cols)
    else:
        print("  无 NaN / 非数值")
        f["nan_cols"] = 0
    q = ds.get("quat_w")
    if q is not None:
        qn = np.sqrt(sum(ds.get("quat_%s" % c) ** 2 for c in ("w", "x", "y", "z")))
        qs = _nan_stats(qn)
        if qs:
            print("  四元数模长: %.4f ~ %.4f（应 ≈1）" % (qs["min"], qs["max"]))
    gx = ds.get("gyro_0")
    if gx is not None:
        gs = _nan_stats(np.concatenate([ds.get("gyro_%d" % i) for i in range(3)]))
        if gs:
            print("  gyro 范围: %.3f ~ %.3f rad/s" % (gs["min"], gs["max"]))

    # ---- 3b) temp / vbus：恒 0 vs 有效 ----
    print("\n[4] 温度 / 母线电压（未轮询时 =0 属正常）")
    print("    m_temp: 1Hz 轮询全部 16 个 → 全 0 说明轮询没生效；"
          "m_vbus: 1Hz 只轮询每路 CAN 的 1 号电机(i%4==0) → 其余 12 路 ==0 属正常")
    for base, unit in (("m_temp", "°C"), ("m_vbus", "V")):
        const0, valid, other = [], [], []
        for i in range(16):
            v = ds.get("%s_%02d" % (base, i))
            if v is None:
                continue
            const, c0 = _is_const(v)
            if const and abs(c0) < 1e-9:
                const0.append(i)
            elif const:
                other.append((i, c0))
            else:
                s = _nan_stats(v)
                valid.append((i, s))
        print("  %s: 恒 0 通道 %d 个 %s%s"
              % (base, len(const0), const0[:6], " ..." if len(const0) > 6 else ""))
        print("      %s有效(有变化)通道: %s" % (
            "  ",
            ", ".join("%02d[%.2f~%.2f]" % (i, s["min"], s["max"]) for i, s in valid) or "无"))
        if other:
            print("      恒定非 0 通道: %s" % ", ".join("%02d=%.3f" % (i, c) for i, c in other))
        if base == "m_vbus":
            polled = [i for i in range(16) if i % 4 == 0]
            miss = [i for i in polled if i in const0]
            if miss:
                print("      [警告] 本应轮询的 CAN 1 号电机通道却恒 0: %s（固件可能不支持 Vbus 回读）" % miss)
        if base == "m_temp" and len(const0) == 16:
            print("      [警告] m_temp 16 路全恒 0 → 温度轮询(PollSlowTelemetry)没生效或未使能电机")
        f["%s_const0" % base] = len(const0)
        f["%s_valid" % base] = len(valid)

    # ---- 4) m_pos / m_tau ----
    sec = [4]
    for base in ("m_pos", "m_tau"):
        sec[0] += 1
        print("\n[%d] %s 逐关节统计（CAN 顺序）" % (sec[0], base))
        if quiet:
            stats = [_nan_stats(ds.get("%s_%02d" % (base, i))) for i in range(16)]
            ok = [s for s in stats if s]
            if ok:
                print("  (quiet) 16 关节 min %.4f~%.4f, max|std| %.4f"
                      % (min(s["min"] for s in ok), max(s["max"] for s in ok),
                         max(s["std"] for s in ok)))
        else:
            print("  idx 关节         min        max        mean       std")
            for i in range(16):
                v = ds.get("%s_%02d" % (base, i))
                s = _nan_stats(v) if v is not None else None
                if s is None:
                    print("  %2d %-11s (无数据)" % (i, joint_name(i).split("(")[0]))
                    continue
                print("  %2d %-11s %10.4f %10.4f %10.4f %9.4f"
                      % (i, joint_name(i).split("(")[0], s["min"], s["max"],
                         s["mean"], s["std"]))
        if base == "m_tau":
            dead = [i for i in range(16)
                    if (lambda s: s is None or s["std"] < 1e-9)(_nan_stats(ds.get("%s_%02d" % (base, i))))]
            if dead:
                print("  [提示] 力矩恒定的关节(可能没反馈力矩/未使能): %s" % dead)

    # ---- 5) c_mode 分布 ----
    sec[0] += 1
    print("\n[%d] c_mode 取值分布（0=IMPEDANCE 1=SPEED 2=POSITION；按 16 关节×行 统计）"
          % sec[0])
    mode_cols = [ds.get("c_mode_%02d" % i) for i in range(16)]
    allm = np.concatenate([v[np.isfinite(v)] for v in mode_cols if v is not None]) \
        if any(v is not None for v in mode_cols) else np.zeros(0)
    if allm.size:
        u, c = np.unique(np.round(allm).astype(int), return_counts=True)
        print("  全体: %s" % ", ".join("mode=%d: %d (%.1f%%)" % (a, b, 100.0 * b / allm.size)
                                       for a, b in zip(u, c)))
        illegal = [(int(a), int(b)) for a, b in zip(u, c) if a not in (0, 1, 2)]
        if illegal:
            print("  [警告] 非法 mode 值: %s" % illegal)
    per = {}
    mixed = []
    for i in range(16):
        v = ds.get("c_mode_%02d" % i)
        if v is None:
            continue
        uv = set(np.unique(np.round(v[np.isfinite(v)]).astype(int)).tolist())
        per[i] = uv
        if len(uv) > 1:
            mixed.append((i, sorted(uv)))
    print("  每关节: %s" % ", ".join(
        "%02d=%s" % (i, "/".join(str(x) for x in sorted(per[i]))) for i in range(16) if i in per))
    if mixed:
        print("  录制途中切过模式的关节: %s" % ", ".join("%02d%s" % (i, m) for i, m in mixed))

    # ---- 6) 命令活跃度 ----
    sec[0] += 1
    print("\n[%d] 命令活跃度（长期不变 ⇒ 该段对辨识无用）" % sec[0])
    if not quiet:
        print("  idx 关节         c_pos变化   c_vel变化  判定        可用阶跃段")
    active, static = [], []
    step_joints = {}
    for i in range(16):
        cp = ds.get("c_pos_%02d" % i)
        cv = ds.get("c_vel_%02d" % i)
        ncp = _changes(cp)[0] if cp is not None else 0
        ncv = _changes(cv)[0] if cv is not None else 0
        cmd_col, fb_col, why = pick_signal(ds, i)
        cl = ds.get(cmd_col)
        nev = 0
        if cl is not None and np.isfinite(cl).any():
            ev = detect_steps(t, cl, dt)
            nev = len(ev)
            if nev:
                step_joints[i] = ev
        is_active = (ncp > 2 or ncv > 2)
        (active if is_active else static).append(i)
        if not quiet:
            print("  %2d %-11s %9d %11d  %-10s %d"
                  % (i, joint_name(i).split("(")[0], ncp, ncv,
                     "活跃" if is_active else "长期不变", nev))
    f["static_joints"] = static
    f["step_joints"] = sorted(step_joints.keys())
    print("  活跃关节 %d 个: %s" % (len(active), active))
    if static:
        print("  长期不变关节 %d 个（对辨识无用）: %s" % (len(static), static))

    # ---- 7) 结论 ----
    sec[0] += 1
    print("\n[%d] 结论：这段数据能用来做什么" % sec[0])
    quality = []
    if f.get("n_big_gap", 0):
        ratio = 100.0 * f["n_big_gap"] / max(1, n - 1)
        quality.append("时间轴有 %d 处 >5ms 卡顿(%.2f%%)" % (f["n_big_gap"], ratio))
    else:
        quality.append("时间轴无 >5ms 卡顿")
    if not f.get("t_mono", True):
        quality.append("t_ms 非严格单调（插值/对齐前必须处理）")
    if f["nan_cols"]:
        quality.append("%d 列含 NaN" % f["nan_cols"])
    print("  · 数据质量: %s" % "；".join(quality))

    # 延迟辨识
    usable_delay = []
    for i, ev in step_joints.items():
        if len(ev) < 2:
            continue
        cmd_col, fb_col, why = pick_signal(ds, i)
        c = ds.get(cmd_col)
        m = ds.get(fb_col)
        rs, amps = [], []
        for e in ev:
            i0 = e["idx"]
            lo = max(0, i0 - 50)
            hi = min(n, i0 + 200)
            rs.append(_corr(c[lo:hi], m[lo:hi]))
            pre = m[max(0, i0 - 50):i0]
            post = m[i0 + 10:i0 + 60]
            if pre.size >= 8 and post.size >= 8:
                mu = float(np.median(pre))
                sd = 1.4826 * float(np.median(np.abs(pre - mu)))
                if sd <= 0:
                    sd = float(np.std(pre))
                amps.append((float(np.median(post)) - mu) / max(sd, 1e-9))
        rs = [r for r in rs if np.isfinite(r)]
        amps = [a for a in amps if np.isfinite(a)]
        if rs and amps and float(np.median(rs)) > 0.4 and max(abs(a) for a in amps) > 3.0:
            usable_delay.append((i, len(ev)))
    f["delay_joints"] = [i for i, _ in usable_delay]
    if len(usable_delay) >= 1:
        print("  · 延迟辨识: 【可以】%d 个关节有 ≥2 个阶跃沿且反馈同向响应: %s"
              % (len(usable_delay), ", ".join("%02d(n=%d)" % (i, k) for i, k in usable_delay)))
        print("      用法: python3 tool/delay_fit.py <csv> --motor %d" % usable_delay[0][0])
    else:
        print("  · 延迟辨识: 【不可以】没有「阶跃沿 + 反馈响应」的通道")
        print("      建议: 录制时对目标关节下发 ≥3 个大幅阶跃（幅值 > 3σ 噪声），静止基线 100ms")

    # 轮速环
    wheel_info = []
    for i in WHEELS:
        mode = ds.get("c_mode_%02d" % i)
        cv = ds.get("c_vel_%02d" % i)
        mv = ds.get("m_vel_%02d" % i)
        if mode is None or cv is None or mv is None:
            continue
        sel = np.isfinite(mode) & (np.round(mode) == 1)
        if sel.sum() < 50:
            continue
        cs = float(np.std(cv[sel]))
        r = _corr(cv[sel], mv[sel])
        wheel_info.append((i, int(sel.sum()), cs, r))
    f["wheel_speed_rows"] = wheel_info
    ok_wheel = [w for w in wheel_info if w[2] > 0.5 and np.isfinite(w[3]) and w[3] > 0.3]
    if ok_wheel:
        print("  · 轮速环(SPEED)拟合: 【可以】%s"
              % ", ".join("can%02d(%d行,std(ω_des)=%.2f,corr=%.2f)" % w for w in ok_wheel))
        print("      用法: python3 tool/wheel_servo_fit.py <csv> --wheel %d" % (ok_wheel[0][0] // 4))
    elif wheel_info:
        print("  · 轮速环(SPEED)拟合: 【勉强】有 SPEED 行但 ω_des 激励不足: %s"
              % ", ".join("can%02d(%d行,std=%.2f,corr=%.2f)" % w for w in wheel_info))
        print("      建议: 悬空/垫起后给轮子 -25…+25 rad/s 的多级阶跃")
    else:
        print("  · 轮速环(SPEED)拟合: 【不可以】没有 c_mode_i==1 的行（轮子未走固件速度环）")

    # 执行器（腿力矩）
    act = []
    for i in LEGS:
        mode = ds.get("c_mode_%02d" % i)
        cp = ds.get("c_pos_%02d" % i)
        mt = ds.get("m_tau_%02d" % i)
        if mode is None or cp is None or mt is None:
            continue
        sel = np.isin(np.round(mode), (0, 2))
        if sel.sum() < 50:
            continue
        csd = float(np.std(cp[sel]))
        tstd = float(np.std(mt[sel]))
        if csd > 0.02 and tstd > 0.05:
            act.append((i, csd, tstd))
    f["actuator_joints"] = [a[0] for a in act]
    if act:
        print("  · 执行器(腿阻抗/位置)辨识: 【可以】%d 个关节位置激励 std>0.02rad 且力矩有波动"
              % len(act))
        print("      覆盖关节: %s" % ", ".join("can%02d(posσ=%.3f,tauσ=%.3f)" % a for a in act[:8]))
    else:
        print("  · 执行器(腿阻抗/位置)辨识: 【不可以】腿的 c_pos 基本不动或 m_tau 恒定")
        print("      建议: 让腿在站立工作区间做小幅正弦/阶跃（幅值 ≥0.05rad）再用 m_tau 回归")

    # ---- 8) 绘图 ----
    if do_plot:
        png = plot_health(path, ds)
        if png:
            print("\n[%d] 时域叠图已保存: %s" % (sec[0] + 1, png))
        else:
            print("\n[%d] 未绘图（未安装 matplotlib 或数据不足）" % (sec[0] + 1))
    return f


def plot_health(path, ds):
    """c_vel/m_vel 四个轮子的时域叠图 -> 同目录 health_*.png。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[提示] 未安装 matplotlib，跳过绘图（pip install matplotlib 可出图）")
        return None
    t = ds.t
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    for ax, i in zip(axes, WHEELS):
        cv = ds.get("c_vel_%02d" % i)
        mv = ds.get("m_vel_%02d" % i)
        if cv is None or mv is None:
            continue
        ax.plot(t / 1000.0, cv, "--", lw=1.0, label="c_vel (cmd)")
        ax.plot(t / 1000.0, mv, lw=0.9, label="m_vel (fb)")
        ax.set_ylabel("%s\nrad/s" % joint_name(i).split("(")[0])
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=7)
    axes[-1].set_xlabel("t (s)")
    fig.suptitle("dataset health: wheel cmd vs feedback  %s" % os.path.basename(path), fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    stem = os.path.splitext(os.path.basename(path))[0]
    out = os.path.join(os.path.dirname(os.path.abspath(path)), "health_%s.png" % stem)
    try:
        fig.savefig(out, dpi=110)
    except OSError as e:
        plt.close(fig)
        print("[警告] 保存图片失败: %s" % e)
        return None
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# selftest
# ---------------------------------------------------------------------------
def selftest(tmpdir="/tmp"):
    print("=========== dataset_health.py 自测 ===========")
    path = os.path.join(tmpdir, "dsh_synth_health.csv")
    path, meta, truth = write_synthetic_dataset(path)
    print("合成数据集: %s (+%s)" % (path, meta))
    f = report(path, do_plot=True, quiet=True)
    checks = [
        ("行数 > 0", f["n"] > 0),
        ("检出丢帧(>5ms 间隔) >= 2 处", f.get("n_big_gap", 0) >= 2),
        ("t_ms 严格单调", f.get("t_mono") is True),
        ("wall_ms 单调", f.get("wall_mono") is True),
        ("无 NaN 列", f["nan_cols"] == 0),
        ("temp 恒 0 通道 = 0（16 路都轮询）", f.get("m_temp_const0") == 0),
        ("vbus 有效通道 = 4（每路 CAN 1 号电机）", f.get("m_vbus_valid") == 4),
        ("vbus 恒 0 通道 = 12（未轮询）", f.get("m_vbus_const0") == 12),
        ("检出阶跃关节 >= 16", len(f.get("step_joints", [])) >= 16),
        ("可做延迟辨识的关节 >= 4", len(f.get("delay_joints", [])) >= 4),
        ("可拟合轮速环", len(f.get("wheel_speed_rows", [])) >= 4),
        ("可辨识执行器关节 >= 12", len(f.get("actuator_joints", [])) >= 12),
    ]
    ok = True
    for name, cond in checks:
        print("  [%s] %s" % ("PASS" if cond else "FAIL", name))
        ok = ok and bool(cond)
    print("=========== 自测%s ===========" % ("通过" if ok else "未通过"))
    return 0 if ok else 1


def build_parser():
    p = argparse.ArgumentParser(
        prog="dataset_health.py",
        description="统一数据集 CSV 中文体检报告（用法见模块 docstring）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n  python3 tool/dataset_health.py log/dataset_20261001_120000.csv\n"
               "  python3 tool/dataset_health.py 'log/dataset_*.csv' --no-plot\n"
               "  python3 tool/dataset_health.py --selftest\n")
    p.add_argument("csv", nargs="*", help="一个或多个统一数据集 CSV")
    p.add_argument("--no-plot", action="store_true", help="不画图")
    p.add_argument("--quiet", action="store_true", help="只输出结论（自测用）")
    p.add_argument("--selftest", action="store_true", help="生成合成数据并自测")
    p.add_argument("--tmpdir", default="/tmp", help="自测合成数据目录（默认 /tmp）")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.selftest:
        return selftest(args.tmpdir)
    if not args.csv:
        build_parser().print_help()
        print("\n[ERROR] 缺少数据集路径（例：python3 tool/dataset_health.py log/dataset_*.csv）")
        return 2
    rc = 0
    for path in args.csv:
        try:
            report(path, do_plot=not args.no_plot, quiet=args.quiet)
        except DatasetError as e:
            # report() 已经打印过文件/标题，这里只补错误行，避免重复标题
            print("[ERROR] %s" % e)
            rc = 2
    return rc


if __name__ == "__main__":
    sys.exit(main())
