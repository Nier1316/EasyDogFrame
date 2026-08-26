#!/bin/bash
# 达妙 USB2CAN 运行入口。
# libdm_device.so 内嵌的作者机器 RUNPATH 已用 patchelf 移除（lib/dm_device.so 的 RPATH
# 指向 miniforge3/lib + damiao_sdk），程序直接跑即可，无需 LD_LIBRARY_PATH。
exec ./bin/can_motor_app "$@"
