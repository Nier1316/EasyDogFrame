#!/bin/bash
# 达妙 USB2CAN 运行入口。
# libdm_device.so 内嵌的作者机器 RUNPATH 已用 patchelf 移除（lib/dm_device.so 的 RPATH
# 指向 miniforge3/lib + damiao_sdk），程序直接跑即可，无需 LD_LIBRARY_PATH。
# ---------------------------------------------------------------------------
# LD_PRELOAD 垫片：修正达妙 SDK 写死的 libusb 事件收割超时（100 ms）
#   不加载时：CAN 回帧被攒成 ~100 ms 一批，上位机看到的关节位置只有 ~10 Hz 刷新率。
#   加载后  ：收割粒度压到 DMCAN_POLL_US（默认 1000 µs），反馈恢复高刷新率。
#   原理/证据见 tool/libusb_fastpoll.c；临时关闭：DMCAN_POLL_US=0 ./run.sh
SHIM="./bin/libusb_fastpoll.so"
if [ -f "$SHIM" ]; then
    export LD_PRELOAD="${SHIM}${LD_PRELOAD:+:$LD_PRELOAD}"
fi
# ---------------------------------------------------------------------------
exec ./bin/can_motor_app "$@"
