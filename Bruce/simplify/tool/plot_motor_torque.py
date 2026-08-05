#!/usr/bin/env python3
"""电机扭矩分析绘图工具

读取 log/recv_*.csv，按 motor_id 分成 4 个子图，每图叠加 4 路 CAN。
同时输出矢量 PDF（报告/论文用）与 300 dpi PNG（快速查看），
统计量另存 CSV，便于引用具体数值而非从图上目测。

环境:
    conda activate dog          # numpy/pandas/matplotlib/scipy

用法:
    python tool/plot_motor_torque.py                    # 自动取最新 recv 日志
    python tool/plot_motor_torque.py log/recv_xxx.csv   # 指定日志
    python tool/plot_motor_torque.py --field raw        # 画标定前原始值
    python tool/plot_motor_torque.py --port 1           # 只画 CAN1
    python tool/plot_motor_torque.py --psd              # 追加功率谱子图
    python tool/plot_motor_torque.py --window 8.9 9.1   # 只看某段时间(秒)
    python tool/plot_motor_torque.py --keep-junk        # 保留上电首帧
    python tool/plot_motor_torque.py --outdir fig/      # 指定输出目录

说明:
    1kHz x 16 电机的日志有几十万行。曲线用 rasterized=True，
    点全部画进 PDF 但文件体积可控，矢量的坐标轴/文字仍可无损缩放。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")          # 无显示环境（服务器/SSH）也能出图
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator

# ---------------------------------------------------------------- 常量

# 与 include/motor_drive/ele_motor_def.h 的 MOTOR_LIMITS 保持一致
MOTOR_NAMES = {1: "Hip 髋", 2: "Thigh 大腿", 3: "Calf 小腿", 4: "Wheel 轮"}
TORQUE_LIMIT = {1: 150.0, 2: 150.0, 3: 150.0, 4: 53.0}

# 编码量程下限，用于识别上电首帧
POS_MIN = -12.5
VEL_MIN = -65.0

CAN_PORTS = 4
MOTORS_PER_CAN = 4

# 四路 CAN 配色（Tableau 色序，色盲友好度尚可）
PORT_COLORS = {0: "#d62728", 1: "#1f77b4", 2: "#2ca02c", 3: "#ff7f0e"}
PORT_LABELS = {0: "CAN0", 1: "CAN1", 2: "CAN2", 3: "CAN3"}


def setup_style():
    """科研出版常用的绘图参数"""
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Noto Sans CJK SC", "DejaVu Sans"],
        "axes.unicode_minus": False,      # 中文字体下负号会变方框
        "font.size": 10,
        "axes.labelsize": 11,
        "axes.titlesize": 11,
        "legend.fontsize": 9,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "grid.linewidth": 0.5,
        "axes.axisbelow": True,           # 网格压在数据下面
        "lines.linewidth": 0.9,
        "figure.dpi": 110,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "pdf.fonttype": 42,               # TrueType，PDF 里文字可编辑可搜索
        "ps.fonttype": 42,
    })


# ---------------------------------------------------------------- 数据

def find_latest_log(log_dir: Path) -> Path | None:
    """返回 log/ 下最新的 recv_*.csv"""
    files = sorted(log_dir.glob("recv_*.csv"), key=lambda p: p.stat().st_mtime)
    return files[-1] if files else None


def load_log(path: Path, keep_junk: bool = False) -> tuple[pd.DataFrame, int]:
    """读日志，返回 (DataFrame, 剔除的上电首帧数)

    日志尾部常被 Ctrl-C 截断成半行，用 on_bad_lines="skip" 容错。
    """
    df = pd.read_csv(
        path,
        dtype={
            "elapsed_ms": "int64", "can_port": "int8", "motor_id": "int8",
            "raw_pos": "float32", "raw_vel": "float32", "raw_torque": "float32",
            "cal_pos": "float32", "cal_vel": "float32", "cal_torque": "float32",
        },
        on_bad_lines="skip",
    )
    df = df.dropna()
    df = df[df["can_port"].between(0, CAN_PORTS - 1)
            & df["motor_id"].between(1, MOTORS_PER_CAN)]

    junk = 0
    if not keep_junk:
        # 上电首帧：数据位全 0，按 uint_to_float 解码后正好落在三个量程下限
        # （-12.5 rad / -65 rad·s⁻¹ / -t_max Nm）。不是真实读数，但幅值等于
        # 限幅，会把纵轴撑满、真实信号压成直线，必须剔除。
        lim = df["motor_id"].map(TORQUE_LIMIT).astype("float32")
        mask = (np.isclose(df["raw_pos"], POS_MIN, atol=1e-3)
                & np.isclose(df["raw_vel"], VEL_MIN, atol=1e-3)
                & np.isclose(df["raw_torque"].abs(), lim, atol=1e-3))
        junk = int(mask.sum())
        df = df[~mask]

    df = df.reset_index(drop=True)
    df["t_s"] = df["elapsed_ms"] / 1000.0
    return df, junk


def compute_stats(df: pd.DataFrame, col: str) -> pd.DataFrame:
    """按 (can_port, motor_id) 汇总扭矩统计量

    RMS 反映持续热负荷，p99 比 max 更抗单点毛刺，两者都比 max 更能说明问题。
    """
    rows = []
    for (port, motor), g in df.groupby(["can_port", "motor_id"], sort=True):
        v = g[col].to_numpy(dtype=np.float64)
        lim = TORQUE_LIMIT.get(int(motor), 150.0)
        absmax = float(np.abs(v).max()) if v.size else float("nan")
        rows.append({
            "can_port": int(port),
            "motor_id": int(motor),
            "motor_name": MOTOR_NAMES.get(int(motor), "?"),
            "n": int(v.size),
            "min": float(v.min()),
            "max": float(v.max()),
            "mean": float(v.mean()),
            "std": float(v.std(ddof=1)) if v.size > 1 else 0.0,
            "rms": float(np.sqrt(np.mean(v ** 2))),
            "p99_abs": float(np.percentile(np.abs(v), 99)),
            "abs_max": absmax,
            "limit": lim,
            "util_pct": 100.0 * absmax / lim,
        })
    return pd.DataFrame(rows)


def estimate_fs(g: pd.DataFrame) -> float:
    """从时间戳估采样率 (Hz)，用于功率谱

    日志时间戳是 1ms 分辨率的整数，同一毫秒内可能有多帧，
    所以用总时长除以点数，而不是相邻差分的中位数。
    """
    span = g["t_s"].iloc[-1] - g["t_s"].iloc[0]
    if span <= 0 or len(g) < 2:
        return float("nan")
    return (len(g) - 1) / span


# ---------------------------------------------------------------- 绘图

def plot_torque(df: pd.DataFrame, col: str, stats: pd.DataFrame,
                src_name: str, ports: list[int]) -> plt.Figure:
    """4 个子图（每个 motor_id 一张），共享时间轴，叠加各路 CAN"""
    fig, axes = plt.subplots(MOTORS_PER_CAN, 1, figsize=(11, 11),
                             sharex=True)
    field_cn = "标定后" if col == "cal_torque" else "标定前原始"

    for ax, motor in zip(axes, range(1, MOTORS_PER_CAN + 1)):
        lim = TORQUE_LIMIT.get(motor, 150.0)
        sub = df[df["motor_id"] == motor]

        for port in ports:
            g = sub[sub["can_port"] == port]
            if g.empty:
                continue
            # rasterized: 几十万点栅格化进 PDF，坐标轴与文字仍是矢量
            ax.plot(g["t_s"], g[col], color=PORT_COLORS[port],
                    linewidth=0.6, alpha=0.85, rasterized=True,
                    label=PORT_LABELS[port])

        ax.axhline(0, color="0.4", linewidth=0.8, zorder=1)

        # 只有数据真的贴近限幅时才画饱和线，否则纵轴被拉大反而看不清信号
        s = stats[stats["motor_id"] == motor]
        if not s.empty and s["util_pct"].max() >= 90.0:
            for sgn in (1, -1):
                ax.axhline(sgn * lim, color="#c00000", linestyle="--",
                           linewidth=0.9, zorder=1)
            # 靠左放，避免和右上角图例重叠
            ax.text(0.005, 0.94, f"饱和限 ±{lim:.0f} N·m",
                    transform=ax.transAxes, ha="left", va="top",
                    fontsize=8, color="#c00000")

        peak = s["abs_max"].max() if not s.empty else 0.0
        ax.set_title(f"motor{motor}  {MOTOR_NAMES.get(motor, '?')}   "
                     f"限幅 ±{lim:.0f} N·m   实测峰值 {peak:.2f} N·m "
                     f"({100.0 * peak / lim:.1f}%)", loc="left")
        ax.set_ylabel("扭矩 (N·m)")
        ax.yaxis.set_minor_locator(AutoMinorLocator())
        ax.legend(loc="upper right", ncol=len(ports), framealpha=0.9)

    axes[-1].set_xlabel("时间 (s)")
    axes[-1].xaxis.set_minor_locator(AutoMinorLocator())
    fig.suptitle(f"电机{field_cn}扭矩 — 按 motor_id 分图，叠加各路 CAN\n"
                 f"数据源 {src_name}", fontsize=12, y=0.997)
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    return fig


def plot_psd(df: pd.DataFrame, col: str, ports: list[int]) -> plt.Figure | None:
    """扭矩功率谱：时域曲线看不出尖峰里有没有固定频率的自激，频谱能

    单边 PSD，Welch 法。纵轴 dB 便于跨电机比较相对能量。
    """
    try:
        from scipy import signal
    except ImportError:
        print("[WARN] 未装 scipy，跳过功率谱（conda install scipy）")
        return None

    fig, axes = plt.subplots(MOTORS_PER_CAN, 1, figsize=(11, 11), sharex=True)

    for ax, motor in zip(axes, range(1, MOTORS_PER_CAN + 1)):
        sub = df[df["motor_id"] == motor]
        for port in ports:
            g = sub[sub["can_port"] == port]
            if len(g) < 512:
                continue
            fs = estimate_fs(g)
            if not np.isfinite(fs) or fs <= 0:
                continue
            v = g[col].to_numpy(dtype=np.float64)
            v = v - v.mean()                      # 去直流，否则 0 Hz 压过一切
            nper = min(4096, len(v))
            f, pxx = signal.welch(v, fs=fs, nperseg=nper)
            with np.errstate(divide="ignore"):
                db = 10.0 * np.log10(pxx)
            ax.plot(f, db, color=PORT_COLORS[port], linewidth=0.8,
                    label=f"{PORT_LABELS[port]} (fs≈{fs:.0f} Hz)")

        ax.set_title(f"motor{motor}  {MOTOR_NAMES.get(motor, '?')}", loc="left")
        ax.set_ylabel("PSD (dB, N·m²/Hz)")
        ax.set_xscale("log")
        ax.legend(loc="upper right", ncol=2, framealpha=0.9)

    axes[-1].set_xlabel("频率 (Hz)")
    fig.suptitle("扭矩功率谱密度（Welch，已去直流）", fontsize=12, y=0.997)
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    return fig


# ---------------------------------------------------------------- 主流程

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="电机扭矩分析绘图工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n"
               "  python tool/plot_motor_torque.py\n"
               "  python tool/plot_motor_torque.py --psd --window 8.9 9.1\n")
    p.add_argument("log", nargs="?", type=Path,
                   help="recv_*.csv 路径，省略则取 log/ 下最新的一份")
    p.add_argument("--field", choices=["cal", "raw"], default="cal",
                   help="cal=标定后(默认)，raw=标定前原始值")
    p.add_argument("--port", type=int, action="append", dest="ports",
                   help="只画指定 CAN 口，可重复，如 --port 0 --port 1")
    p.add_argument("--window", nargs=2, type=float, metavar=("T0", "T1"),
                   help="只取该时间段（秒）")
    p.add_argument("--psd", action="store_true", help="追加功率谱图")
    p.add_argument("--keep-junk", action="store_true",
                   help="保留上电首帧（默认剔除）")
    p.add_argument("--outdir", type=Path, default=None,
                   help="输出目录，默认与日志同目录")
    p.add_argument("--no-pdf", action="store_true", help="只出 PNG")
    return p


def main(argv: list[str]) -> int:
    args = build_parser().parse_args(argv[1:])
    repo = Path(__file__).resolve().parent.parent

    path = args.log
    if path is None:
        path = find_latest_log(repo / "log")
        if path is None:
            print("[ERROR] log/ 下找不到 recv_*.csv，请显式给出日志路径",
                  file=sys.stderr)
            return 1
        print(f"[INFO] 自动选用最新日志: {path}")
    if not path.is_file():
        print(f"[ERROR] 日志不存在: {path}", file=sys.stderr)
        return 1

    setup_style()
    df, junk = load_log(path, args.keep_junk)
    if junk:
        print(f"[INFO] 剔除 {junk} 个上电首帧（数据位全 0 解码成量程下限，"
              f"非真实读数），--keep-junk 可保留")
    if df.empty:
        print("[ERROR] 日志里没有有效数据", file=sys.stderr)
        return 1

    if args.ports:
        df = df[df["can_port"].isin(args.ports)]
    ports = sorted(df["can_port"].unique().tolist())

    if args.window:
        t0, t1 = sorted(args.window)
        df = df[df["t_s"].between(t0, t1)]
        if df.empty:
            print(f"[ERROR] {t0}–{t1}s 区间内没有数据", file=sys.stderr)
            return 1
        print(f"[INFO] 时间窗 {t0:.3f}–{t1:.3f} s，剩余 {len(df)} 行")

    col = f"{args.field}_torque"
    stats = compute_stats(df, col)

    outdir = args.outdir or path.parent
    outdir.mkdir(parents=True, exist_ok=True)
    stem = f"torque_{path.stem}_{args.field}"
    if args.ports:
        stem += "_can" + "".join(str(p) for p in ports)
    if args.window:
        stem += f"_{args.window[0]:g}-{args.window[1]:g}s"

    fig = plot_torque(df, col, stats, path.name, ports)
    saved = []
    png = outdir / f"{stem}.png"
    fig.savefig(png)
    saved.append(png)
    if not args.no_pdf:
        pdf = outdir / f"{stem}.pdf"
        fig.savefig(pdf)
        saved.append(pdf)
    plt.close(fig)

    if args.psd:
        pfig = plot_psd(df, col, ports)
        if pfig is not None:
            ppng = outdir / f"{stem}_psd.png"
            pfig.savefig(ppng)
            saved.append(ppng)
            if not args.no_pdf:
                ppdf = outdir / f"{stem}_psd.pdf"
                pfig.savefig(ppdf)
                saved.append(ppdf)
            plt.close(pfig)

    csv_path = outdir / f"{stem}_stats.csv"
    stats.to_csv(csv_path, index=False, float_format="%.4f")
    saved.append(csv_path)

    for f in saved:
        print(f"[INFO] 已输出 {f}  ({f.stat().st_size / 1024:.1f} KB)")

    show = stats[["can_port", "motor_id", "motor_name", "n", "min", "max",
                  "mean", "std", "rms", "p99_abs", "abs_max", "util_pct"]]
    print(f"\n扭矩统计 ({col}, N·m):")
    print(show.to_string(index=False,
                         formatters={c: "{:.2f}".format
                                     for c in ("min", "max", "mean", "std",
                                               "rms", "p99_abs", "abs_max",
                                               "util_pct")}))
    hot = stats[stats["util_pct"] >= 90.0]
    if not hot.empty:
        print("\n[WARN] 以下电机扭矩达限幅 90% 以上:")
        for _, r in hot.iterrows():
            print(f"  CAN{int(r['can_port'])} motor{int(r['motor_id'])} "
                  f"{r['motor_name']}: 峰值 {r['abs_max']:.2f} / "
                  f"限幅 {r['limit']:.0f} N·m = {r['util_pct']:.1f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
