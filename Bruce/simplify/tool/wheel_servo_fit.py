#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wheel_servo_fit.py —— 真机轮速环（固件速度环）离线辨识，产出可写进仿真/配置的等价参数

用法:
    python3 tool/wheel_servo_fit.py <dataset.csv> [--wheel 0] [--motor 3]
                                    [--policy-order] [--csv out.csv] [--no-plot]
    python3 tool/wheel_servo_fit.py --selftest [--tmpdir /tmp]

参数:
    --wheel W       腿序号 0..3（FL/FR/RL/RR）→ CAN 关节 i = 4*W+3（默认 0 = FL-wheel）
    --motor N       直接给 CAN 关节索引 0..15（必须是轮子 i%4==3；与 --wheel 二选一）
    --policy-order  --motor 按 POLICY 序解释（12 腿+4 轮），内部转成 CAN 序
    --csv PATH      段明细落盘
    --no-plot       不画图

估计内容（只在 c_mode_i == 1 的 SPEED 行上工作，用 c_vel_i=ω_des, c_kp_i=kvp,
c_tau_i=ki, m_vel_i=ω, m_tau_i=力矩）:
    纯延迟 T_d        —— 与 delay_fit.py 同一套方法（边沿互相关/首次越阈/一阶+延迟拟合）
    一阶时间常数 τ
    静态增益 K = 稳态 ω / ω_des（在未饱和、未死区的工作段上取）
    死区              —— |ω_des| 已有一定幅值但 ω≈0 的区间（观察值）
    饱和              —— ω_des 继续增大而 ω 封顶（观察值 + 推断的封顶转速）
    等价仿真参数      —— kd_equiv（把固件速度环折算成仿真里 kd·(ω_des−ω) 的阻尼），
                          以及 J_eff（由 m_tau vs dω/dt 回归）、粘性 b、库仑/门槛力矩
重要说明：真机是固件 1kHz 速度环（含 PI + 摩擦），仿真里是纯阻尼 kd·(ω_des−ω)，
          二者结构不同。kd_equiv 只是「让一阶响应时间常数/静态增益对上」的近似，
          已在输出里逐条标注 [拟合] / [实测统计] / [推断]。
坐标：c_* / m_* 均为标定后统一坐标，不要再乘/除任何标定系数。

================= 冻结数据集 schema（188 列，UTF-8，逗号分隔，\\n 行尾）=================
 1. wall_ms                     系统墙钟 epoch 毫秒(int)
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
12. m_temp_00 .. m_temp_15      温度 °C（1Hz 轮询全部 16 个；未轮询到 = 0）
13. m_vbus_00 .. m_vbus_15      母线电压 V（1Hz 只轮询每路 CAN 的 1 号电机 i%4==0）
14. gyro_0..gyro_2              机体系角速度 rad/s
15. quat_w,quat_x,quat_y,quat_z body←world，w 在前
16. cmd_vx,cmd_vy,cmd_wz        上层速度命令

c_* 语义（按行内 c_mode_i 解释）:
    mode=0 IMPEDANCE：c_pos=目标位置, c_vel=目标速度, c_kp=kp, c_kd=kd, c_tau=τ_ff
    mode=1 SPEED    ：c_pos=0,        c_vel=目标速度, c_kp=kvp, c_kd=0,  c_tau=ki
    mode=2 POSITION ：c_pos=目标位置, c_vel=0,        c_kp=kvp, c_kd=kp, c_tau=kvi

电机索引（CAN 顺序）: i = can_port*4 + (motor_id-1)，轮子 = i%4==3
    03=FL-wheel 07=FR-wheel 11=RL-wheel 15=RR-wheel
    CAN→POLICY: [0,1,2,12, 3,4,5,13, 6,7,8,14, 9,10,11,15]

依赖：stdlib + numpy；scipy/matplotlib 可选（缺了优雅降级）。
底层复用同目录 delay_fit.py。
"""
import argparse
import csv
import os
import sys

try:
    import numpy as np
except ImportError:  # pragma: no cover
    sys.stderr.write("[ERROR] 需要 numpy：pip install numpy（本工具只用 stdlib + numpy）\n")
    sys.exit(2)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from delay_fit import (DatasetError, POLICY_TO_CAN, joint_name, load_dataset,
                           detect_steps, analyze_events, write_synthetic_dataset)
except ImportError as e:  # pragma: no cover
    sys.stderr.write("[ERROR] 无法导入同目录 delay_fit.py: %s\n" % e)
    sys.exit(2)


# ---------------------------------------------------------------------------
# 取 SPEED 行 + 连续段
# ---------------------------------------------------------------------------
class RunView(object):
    """把 dataset 的某一段连续区间包装成 analyze_events 需要的接口。"""

    def __init__(self, t, cols):
        self.t = t
        self.cols = cols
        self.n = len(t)

    def get(self, k):
        return self.cols.get(k)


def speed_runs(ds, i, min_ms=600.0):
    """返回 c_mode_i==1 的连续运行段列表（每段一个 RunView + 原始切片索引）。"""
    mode = ds.get("c_mode_%02d" % i)
    if mode is None:
        return []
    sel = np.isfinite(mode) & (np.round(mode) == 1)
    if sel.sum() < 20:
        return []
    idx = np.where(sel)[0]
    t_all = ds.t
    dt = ds.dt_ms()
    names = ["c_vel_%02d" % i, "m_vel_%02d" % i, "m_tau_%02d" % i,
             "c_kp_%02d" % i, "c_tau_%02d" % i, "c_mode_%02d" % i]
    cols_all = {k: ds.get(k) for k in names}
    runs = []
    start = 0
    for k in range(1, idx.size + 1):
        end = (t_all[idx[k]] - t_all[idx[k - 1]] > 1.8 * dt) if k < idx.size else True
        if end:
            sl = idx[start:k]
            if sl.size >= max(20, int(min_ms / dt)):
                cols = {name: (v[sl] if v is not None else None)
                        for name, v in cols_all.items()}
                runs.append((RunView(t_all[sl] - t_all[sl][0], cols), sl))
            start = k
    return runs


# ---------------------------------------------------------------------------
# 辨识
# ---------------------------------------------------------------------------
def estimate(ds, i, use_scipy=True):
    """返回 findings dict；无 SPEED 数据返回 None。"""
    runs = speed_runs(ds, i)
    if not runs:
        return None
    dt = ds.dt_ms()
    cmd_col, fb_col = "c_vel_%02d" % i, "m_vel_%02d" % i
    tau_col = "m_tau_%02d" % i
    rows = []
    for view, sl in runs:
        c = view.get(cmd_col)
        ev = detect_steps(view.t, c, dt, max_events=64)
        rows += analyze_events(view, i, cmd_col, fb_col, ev, dt, use_scipy=use_scipy)

    if not rows:
        return {"rows": [], "runs": len(runs), "i": i}

    # 全 SPEED 行（原始时间轴）统计
    mode = ds.get("c_mode_%02d" % i)
    sel = np.isfinite(mode) & (np.round(mode) == 1)
    w_all = ds.get(fb_col)[sel]
    des_all = ds.get(cmd_col)[sel]
    tau_all = ds.get(tau_col)[sel] if ds.get(tau_col) is not None else None
    kvp_all = ds.get("c_kp_%02d" % i)[sel]
    ki_all = ds.get("c_tau_%02d" % i)[sel]

    f = {"i": i, "rows": rows, "runs": len(runs), "dt_ms": dt,
         "n_speed": int(sel.sum()),
         "des_abs_max": float(np.nanmax(np.abs(des_all))),
         "w_abs_p99": float(np.nanpercentile(np.abs(w_all), 99)),
         "w_abs_max": float(np.nanmax(np.abs(w_all))),
         "w_std": float(np.nanstd(w_all)),
         "kvp": float(np.nanmedian(kvp_all)) if np.isfinite(kvp_all).any() else float("nan"),
         "ki": float(np.nanmedian(ki_all)) if np.isfinite(ki_all).any() else float("nan"),
         "tau_peak": float(np.nanmax(np.abs(tau_all))) if tau_all is not None else float("nan"),
         "tau_std": float(np.nanstd(tau_all)) if tau_all is not None else float("nan")}

    # ---- 阶跃电平表：|ω_des| 电平 -> 稳态 |ω|（用 delay_fit 给出的 w_ss）----
    lv = {}
    for r in rows:
        d = r["after"]
        w = r.get("w_ss", float("nan"))
        if not np.isfinite(d) or abs(d) < 1e-9 or not np.isfinite(w):
            continue
        lv.setdefault(round(abs(d), 3), []).append(w)

    table = []
    for des_abs in sorted(lv.keys()):
        vals = np.asarray(lv[des_abs], float)
        vals = vals[np.isfinite(vals)]
        if vals.size == 0:
            continue
        table.append({"des_abs": float(des_abs), "n": int(vals.size),
                      "w_abs": float(np.median(np.abs(vals)))})
    f["levels"] = table

    # ---- 静态增益 K（未饱和段）----
    gains = [(x["des_abs"], x["w_abs"] / x["des_abs"]) for x in table if x["des_abs"] > 1e-6]
    K = float("nan")
    if gains:
        gmax = max(g for _, g in gains)
        good = [g for _, g in gains if g >= 0.7 * gmax]
        K = float(np.median(good)) if good else gmax
    f["K"] = K

    # ---- 死区 ----
    dz = 0.0
    dz_pts = []
    for x in table:
        if x["des_abs"] <= 1e-6 or not np.isfinite(K):
            continue
        if x["w_abs"] < 0.2 * K * x["des_abs"]:
            dz_pts.append(x["des_abs"])
    if dz_pts:
        dz = max(dz_pts)
    f["deadzone"] = dz
    f["deadzone_pts"] = dz_pts

    # ---- 饱和 ----
    sat = False
    onset = float("nan")
    slope_top = float("nan")
    if len(table) >= 2:
        a, b = table[-2], table[-1]
        if b["des_abs"] > a["des_abs"]:
            slope_top = (b["w_abs"] - a["w_abs"]) / (b["des_abs"] - a["des_abs"])
        top_gain = table[-1]["w_abs"] / max(table[-1]["des_abs"], 1e-9)
        if (np.isfinite(slope_top) and np.isfinite(K) and slope_top < 0.7 * K) or \
           (np.isfinite(K) and top_gain < 0.9 * K):
            sat = True
            onset = table[-2]["des_abs"]
    f["saturated"] = bool(sat)
    f["sat_onset"] = onset
    f["slope_top"] = slope_top

    # ---- 延迟/τ 汇总（只统计响应显著且 τ 可信的段）----
    def med(key, need_tau=False):
        v = [r[key] for r in rows
             if np.isfinite(r[key]) and r.get("resp_ok")
             and (not need_tau or r["tau_ok"])]
        if not v:
            return float("nan"), float("nan"), 0
        return float(np.median(v)), (float(np.std(v, ddof=1)) if len(v) > 1 else 0.0), len(v)

    f["td_edge"] = med("td_edge")
    f["td_thr"] = med("td_thr")
    f["td_fit"] = med("td_fit", need_tau=True)
    f["tau"] = med("tau_ms", need_tau=True)
    f["td_rec"] = f["td_fit"] if f["td_fit"][2] > 0 else med("td_med")
    f["n_seg"] = len(rows)
    f["n_resp_ok"] = sum(1 for r in rows if r.get("resp_ok"))

    # ---- m_tau vs dω/dt 回归 -> J_eff / b（分段做，避免跨段差分）----
    J = b = c0 = float("nan")
    X, Y = [], []
    if tau_all is not None and np.isfinite(tau_all).any():
        for view, sl in runs:
            mm = view.get(fb_col)
            ta = view.get(tau_col)
            if ta is None:
                continue
            wd = np.gradient(mm, dt / 1000.0)
            good = np.isfinite(wd) & np.isfinite(ta) & np.isfinite(mm)
            if good.sum() < 20:
                continue
            X.append(np.column_stack([wd[good], mm[good], np.ones(int(good.sum()))]))
            Y.append(ta[good])
    if X:
        A = np.vstack(X)
        y = np.concatenate(Y)
        # 只用 |ω̇| 较大或 |ω| 较大的样本，避免静止段噪声主导
        strong = (np.abs(A[:, 0]) > 0.15 * np.nanpercentile(np.abs(A[:, 0]), 95)) | \
                 (np.abs(A[:, 1]) > 0.15 * np.nanpercentile(np.abs(A[:, 1]), 95))
        if strong.sum() > 20:
            A, y = A[strong], y[strong]
        try:
            coef, *_ = np.linalg.lstsq(A, y, rcond=None)
            J, b, c0 = float(coef[0]), float(coef[1]), float(coef[2])
        except np.linalg.LinAlgError:
            pass
    f["J_eff"] = J
    f["b_visc"] = b
    f["c_const"] = c0

    # ---- 等价仿真参数 ----
    tau_s = f["tau"][0] / 1000.0
    f["kd_torque"] = (f["tau_peak"] / f["w_abs_p99"]) if f["w_abs_p99"] > 1e-9 else float("nan")
    f["kd_inertia"] = (J / tau_s) if (np.isfinite(J) and np.isfinite(tau_s) and tau_s > 0) else float("nan")
    f["J_over_tau"] = f["kd_inertia"]
    return f


def write_seg_csv(path, f):
    rows = f["rows"]
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh, lineterminator="\n")
        w.writerow(["can", "joint_name", "seg", "t_step_ms", "des_after", "des_before",
                    "td_edge_ms", "td_threshold_ms", "td_fit_ms", "tau_ms", "K_fit",
                    "fit_rms", "fit_method", "tau_ok", "bg"])
        for k, r in enumerate(rows):
            w.writerow([f["i"], joint_name(f["i"]), k, "%.3f" % r["t_step"],
                        "%.6g" % r["after"], "%.6g" % r["before"],
                        "%.3f" % r["td_edge"], "%.3f" % r["td_thr"],
                        "%.3f" % r["td_fit"], "%.3f" % r["tau_ms"],
                        "%.5g" % r["K"], "%.4g" % r["rms"], r["method"],
                        int(r["tau_ok"]), "%.3g" % r.get("bg", float("nan"))])


# ---------------------------------------------------------------------------
# 报告
# ---------------------------------------------------------------------------
def _fmt(t):
    if t[2] == 0 or not np.isfinite(t[0]):
        return "n/a"
    return "%.1f ± %.1f ms (n=%d)" % t


def print_report(f, quiet=False):
    i = f["i"]
    name = joint_name(i)
    print("\n==== 轮速环辨识: %s (%s) ====" % (name, "CAN 顺序"))
    print("SPEED 行数 %d，连续段 %d 个，阶跃段 %d 个；采样 dt=%.2f ms"
          % (f["n_speed"], f["runs"], f["n_seg"], f["dt_ms"]))
    print("固件速度环增益（段内中位）: kvp=%.3f, ki=%.4f" % (f["kvp"], f["ki"]))
    if not quiet:
        print("\n  段  t_step(ms)  ω_des      T_d边沿  T_d越阈  T_d拟合  τ(ms)  K     fit")
        for k, r in enumerate(f["rows"]):
            print("  %3d %11.1f %9.3f %8.1f %8.1f %8.1f %7.1f %5.3f  %s"
                  % (k, r["t_step"], r["after"], r["td_edge"], r["td_thr"],
                     r["td_fit"], r["tau_ms"], r["K"], r["method"]))

    print("\n---- 辨识结果 ----")
    print("  T_d 纯延迟(一阶拟合)  = %s   [拟合]" % _fmt(f["td_fit"]))
    print("  T_d 纯延迟(推荐)      = %s   [拟合]" % _fmt(f["td_rec"]))
    print("     互相关-边沿        = %s   [拟合]" % _fmt(f["td_edge"]))
    print("     首次越阈           = %s   [拟合]" % _fmt(f["td_thr"]))
    print("  τ  一阶时间常数       = %s   [拟合] (闭环带宽 ≈ %.2f Hz)"
          % (_fmt(f["tau"]), (1000.0 / f["tau"][0] / (2 * np.pi)) if f["tau"][2] and f["tau"][0] > 0 else float("nan")))
    print("  K  静态增益 ω/ω_des   = %.3f                 [拟合]（未饱和段中位）" % f["K"])
    print("  死区 |ω_des| 门槛     = %.3f rad/s           [实测统计]（该幅值以下 ω≈0）" % f["deadzone"])
    if f["saturated"]:
        print("  饱和                  = 已观察到             [实测统计]")
        print("    ω 封顶(99%%分位)     = %.2f rad/s (max %.2f)  [实测统计]"
              % (f["w_abs_p99"], f["w_abs_max"]))
        print("    饱和起点 ω_des ≈ %.2f rad/s（最高两级增量斜率 %.3f vs K %.3f）[推断]"
              % (f["sat_onset"], f["slope_top"], f["K"]))
    else:
        print("  饱和                  = 未观察到（最大 ω_des=%.1f, ω 最大 %.2f）"
              % (f["des_abs_max"], f["w_abs_max"]))
        if f["des_abs_max"] < 1.3 * f["w_abs_p99"] and f["w_abs_p99"] > 0:
            print("    [推断] 励磁不足：建议把 ω_des 提到 ≥ %.1f rad/s 才能判定封顶转速"
                  % (1.5 * f["w_abs_p99"]))
    print("\n---- 力矩侧（用 m_tau 回归）----")
    print("  m_tau 峰值 = %.3f, std = %.3f" % (f["tau_peak"], f["tau_std"]))
    print("  J_eff = %.5f kg·m², 粘性 b = %.5f N·m/(rad/s), 常数项 = %.4f  [拟合]"
          % (f["J_eff"], f["b_visc"], f["c_const"]))
    print("  （回归模型 m_tau = J_eff·dω/dt + b·ω + c；常数项含库仑摩擦/偏置）")

    print("\n---- 建议写进仿真/配置的等价参数 ----")
    print("  [推断] 仿真轮速阻尼 kd_equiv = J_eff/τ = %.5f N·m/(rad/s)" % f["kd_inertia"])
    print("  [实测统计] 力矩-速度粗算 τ_peak/ω_max = %.5f N·m/(rad/s)" % f["kd_torque"])
    if np.isfinite(f["kd_inertia"]) and np.isfinite(f["kd_torque"]):
        print("         两者差异说明 m_tau 里含摩擦/偏置，不是纯阻尼；建议取 %.3f~%.3f"
              % (min(f["kd_inertia"], f["kd_torque"]), max(f["kd_inertia"], f["kd_torque"])))
    print("  [推断] 若仿真要给 ω_des 加死区/饱和：死区 %.2f rad/s，饱和转速 %.2f rad/s，"
          "一阶时间常数 %.1f ms，纯延迟 %.1f ms"
          % (f["deadzone"], f["w_abs_p99"], f["tau"][0] if f["tau"][2] else float("nan"),
             f["td_rec"][0] if f["td_rec"][2] else float("nan")))
    print("\n  [注意] 真机是固件 PI 速度环（kvp=%.2f, ki=%.3f, 1kHz），仿真里是纯阻尼 "
          "kd·(ω_des−ω)；kd_equiv 只保证「一阶响应时间常数 + 静态增益」近似对上，"
          "不能复现积分项消除稳态误差的行为。"
          "\n         ⚠ 两个 kd 取值目前**不一致**，改仿真前先确认用哪个："
          "\n           · sim2sim 部署脚本 dogurdf_sim2sim_deploy/src/sim2sim.py: WHEEL_KD = 2.0（实际跑仿真的那个）"
          "\n           · 真机控制器 include/strategy/rl_controller.h: rl::WHEEL_KD = 1.0（注释记为「历史 2.0 → 1.0」）"
          "\n         若 kd_equiv 与这两者都差很多，建议在仿真里另加死区/饱和钳位，而不是只调 kd。"
          % (f["kvp"], f["ki"]))


def _cjk_font():
    """找一个可用的中文字体；找不到返回 None（画图自动退回英文标签，避免豆腐块）。"""
    try:
        from matplotlib import font_manager
        names = set(x.name for x in font_manager.fontManager.ttflist)
    except Exception:
        return None
    for cand in ("Noto Sans CJK SC", "Noto Sans CJK JP", "Source Han Sans SC",
                 "WenQuanYi Micro Hei", "WenQuanYi Zen Hei", "SimHei",
                 "Microsoft YaHei", "AR PL UMing CN", "Droid Sans Fallback"):
        if cand in names:
            return cand
    return None


def plot_servo(path, ds, f):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[提示] 未安装 matplotlib，跳过绘图（pip install matplotlib 可出图）")
        return None
    cjk = _cjk_font()
    if cjk:
        matplotlib.rcParams["font.sans-serif"] = [cjk, "DejaVu Sans"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        L = {"title": "wheel %s 阶跃响应族（实线=ω反馈, 虚线=ω_des, 颜色=阶跃幅值）",
             "obs": "实测稳态 |ω|", "ideal": "理想 K·|ω_des| (K=%.2f)",
             "dz": "死区", "sat": "饱和起点≈%.1f"}
    else:
        L = {"title": "wheel %s step-response family (solid=omega fb, dashed=omega_des)",
             "obs": "measured steady |omega|", "ideal": "ideal K*|omega_des| (K=%.2f)",
             "dz": "deadzone", "sat": "saturation onset ~%.1f"}
    i = f["i"]
    dt = f["dt_ms"]
    fig, axes = plt.subplots(2, 1, figsize=(10, 8),
                             gridspec_kw={"height_ratios": [2, 1]})
    cmap = plt.get_cmap("viridis")
    runs = speed_runs(ds, i)
    levels = sorted(set(round(abs(r["after"]), 3) for r in f["rows"]
                        if np.isfinite(r["after"])))
    lmax = max(levels) if levels else 1.0
    for view, sl in runs:
        t = view.t
        c = view.get("c_vel_%02d" % i)
        m = view.get("m_vel_%02d" % i)
        for e in detect_steps(t, c, dt, max_events=64):
            i0 = e["idx"]
            hi = min(len(t), i0 + int(200 / dt))
            lo = max(0, i0 - int(20 / dt))
            x = t[lo:hi] - (float(t[i0]) + dt)
            col = cmap(0.15 + 0.75 * min(1.0, abs(e["after"]) / max(lmax, 1e-9)))
            axes[0].plot(x, m[lo:hi], color=col, lw=1.0)
            axes[0].plot(x, c[lo:hi], color=col, lw=0.7, ls="--", alpha=0.6)
    axes[0].set_xlabel("t - t_step (ms)")
    axes[0].set_ylabel("omega (rad/s)")
    axes[0].set_title(L["title"] % joint_name(i))
    axes[0].grid(alpha=0.3)
    axes[0].set_xlim(-20, 200)

    tbl = f["levels"]
    if tbl:
        d = [x["des_abs"] for x in tbl]
        w = [x["w_abs"] for x in tbl]
        axes[1].plot(d, w, "o-", label=L["obs"])
        if np.isfinite(f["K"]):
            dd = np.linspace(0, max(d), 50)
            axes[1].plot(dd, f["K"] * dd, ":", label=L["ideal"] % f["K"])
        if f["deadzone"] > 0:
            axes[1].axvspan(0, f["deadzone"], color="red", alpha=0.12, label=L["dz"])
        if f["saturated"] and np.isfinite(f["sat_onset"]):
            axes[1].axvline(f["sat_onset"], color="orange", ls="--",
                            label=L["sat"] % f["sat_onset"])
    axes[1].set_xlabel("|omega_des| (rad/s)")
    axes[1].set_ylabel("steady |omega| (rad/s)")
    axes[1].grid(alpha=0.3)
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    stem = os.path.splitext(os.path.basename(path))[0]
    out = os.path.join(os.path.dirname(os.path.abspath(path)),
                       "wheel_servo_%s_can%02d.png" % (stem, i))
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
    print("=========== wheel_servo_fit.py 自测 ===========")
    path = os.path.join(tmpdir, "dsh_synth_wheel.csv")
    path, meta, truth = write_synthetic_dataset(path)
    print("合成数据集: %s (+%s)" % (path, meta))
    print("真值: T_d=%.1f ms, τ=%.1f ms, K=1.0, 死区=0.4 rad/s, 饱和转速=25 rad/s, J=0.010 kg·m²"
          % (truth["td_ms"], truth["tau_ms"]))
    ds = load_dataset(path)
    f = estimate(ds, 3)
    if f is None or not f["rows"]:
        print("[FAIL] 未辨识出轮速环（无 SPEED 阶跃）")
        return 1
    print_report(f, quiet=True)
    png = plot_servo(path, ds, f)
    print("\n图: %s" % png)

    checks = [
        ("T_d 误差 ≤ 5ms", abs(f["td_rec"][0] - truth["td_ms"]) <= 5.0),
        ("τ 误差 ≤ 5ms", abs(f["tau"][0] - truth["tau_ms"]) <= 5.0),
        ("K ∈ [0.8, 1.2]", 0.8 <= f["K"] <= 1.2),
        ("死区估计 ∈ [0, 1.2] rad/s", 0.0 <= f["deadzone"] <= 1.2),
        ("检出饱和", bool(f["saturated"])),
        ("ω 封顶 ≈25 rad/s (±3)", abs(f["w_abs_p99"] - 25.0) <= 3.0),
        ("J_eff ≈0.010 (误差 ≤0.005)", abs(f["J_eff"] - 0.010) <= 0.005),
        ("kd_equiv 有限且 >0", np.isfinite(f["kd_inertia"]) and f["kd_inertia"] > 0),
    ]
    ok = True
    for nm, cond in checks:
        print("  [%s] %s" % ("PASS" if cond else "FAIL", nm))
        ok = ok and bool(cond)
    print("  估计 vs 真值: T_d %.1f/%.1f ms, τ %.1f/%.1f ms, K %.3f/1.0, 死区 %.2f/0.40, "
          "ω_max %.1f/25, J %.5f/0.01000"
          % (f["td_rec"][0], truth["td_ms"], f["tau"][0], truth["tau_ms"], f["K"],
             f["deadzone"], f["w_abs_p99"], f["J_eff"]))
    print("=========== 自测%s ===========" % ("通过" if ok else "未通过"))
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(
        prog="wheel_servo_fit.py",
        description="真机轮速环（固件速度环）离线辨识（用法见模块 docstring）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n  python3 tool/wheel_servo_fit.py log/dataset_20261001_120000.csv --wheel 0\n"
               "  python3 tool/wheel_servo_fit.py log/dataset_*.csv --motor 15 --csv /tmp/w.csv\n"
               "  python3 tool/wheel_servo_fit.py --selftest\n")
    p.add_argument("csv", nargs="?", help="统一数据集 CSV（188 列，500Hz）")
    p.add_argument("--wheel", type=int, default=None, help="腿序号 0..3（FL/FR/RL/RR）→ can=4*W+3")
    p.add_argument("--motor", type=int, default=None, help="CAN 关节索引 0..15（必须是轮子 i%%4==3）")
    p.add_argument("--policy-order", action="store_true", help="--motor 按 POLICY 序解释")
    p.add_argument("--csv", dest="out_csv", default=None, help="段明细落盘 CSV")
    p.add_argument("--no-plot", action="store_true", help="不画图")
    p.add_argument("--no-scipy", action="store_true", help="禁用 scipy 精修")
    p.add_argument("--selftest", action="store_true", help="生成合成数据自测")
    p.add_argument("--tmpdir", default="/tmp", help="自测合成数据目录（默认 /tmp）")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.selftest:
        return selftest(args.tmpdir)
    if not args.csv:
        build_parser().print_help()
        print("\n[ERROR] 缺少数据集路径。")
        return 2

    if args.motor is not None and args.wheel is not None:
        print("[ERROR] --wheel 与 --motor 只能给一个。")
        return 2
    if args.motor is not None:
        can = args.motor
        if args.policy_order:
            if not (0 <= can <= 15):
                print("[ERROR] --motor 超范围: %d（POLICY 序 0..15）" % can)
                return 2
            can = POLICY_TO_CAN[can]
        if not (0 <= can <= 15):
            print("[ERROR] --motor 超范围: %d（CAN 索引 0..15）" % can)
            return 2
        i = can
    else:
        w = 0 if args.wheel is None else args.wheel
        if not (0 <= w <= 3):
            print("[ERROR] --wheel 超范围: %d（0..3）" % w)
            return 2
        i = 4 * w + 3
    if i % 4 != 3:
        print("[ERROR] 关节 can%02d (%s) 不是轮子。轮子 = i%%4==3（03/07/11/15）。"
              % (i, joint_name(i)))
        print("        轮速环只在 SPEED 模式的轮子上辨识；腿关节请用 delay_fit.py。")
        return 2

    try:
        ds = load_dataset(args.csv)
    except DatasetError as e:
        print("[ERROR] %s" % e)
        return 2
    for w in ds.warnings:
        print("[WARN] %s" % w)

    f = estimate(ds, i, use_scipy=not args.no_scipy)
    if f is None:
        print("[ERROR] 关节 %s 没有 c_mode_i==1（SPEED）的行。" % joint_name(i))
        print("        轮速环必须走 SendSpeed（固件速度环）才会记录 SPEED 行；")
        print("        若轮子仍在阻抗模式(IMPEDANCE)，请改为 SPEED 后重录。")
        return 3
    if not f["rows"]:
        print("[提示] 关节 %s 有 %d 行 SPEED 数据，但检测不到阶跃沿（ω_des 长期不变）。"
              % (joint_name(i), f["n_speed"]))
        print("        轮速环辨识需要多级阶跃（例：0 → ±5 → ±12 → ±25 → 0 rad/s，每级 ≥0.7s）。")
        return 3

    print("=" * 78)
    print("轮速环伺服辨识 (wheel_servo_fit.py): %s" % args.csv)
    print("=" * 78)
    print_report(f)
    if not args.no_plot:
        png = plot_servo(args.csv, ds, f)
        if png:
            print("\n[OK] 阶跃响应族图已保存: %s" % png)
    if args.out_csv:
        try:
            write_seg_csv(args.out_csv, f)
            print("[OK] 段明细已写入 %s" % args.out_csv)
        except OSError as e:
            print("[ERROR] 写 CSV 失败: %s" % e)
            return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
