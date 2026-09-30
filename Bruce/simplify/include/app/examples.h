#pragma once
// 示例聚合头：main.cpp 只需 include 本文件即可调用全部示例函数。
// 示例实现已按功能拆分（阶段3），现存 43 个（编号 17~60，1~16 已清理、55 从未实现已删除）：
//   examples_common.*   公共 helper（RawTerminal / poll_key / g_rl_stop / rl_signal_handler）
//   examples/ex_basic   基础运动示例 17~23
//   examples/ex_diag    诊断/标定/辨识示例 24, 26~29, 33, 34, 39~50, 54, 57
//   examples/ex_rl      RL/前馈/延迟辨识示例 25, 30~32, 35~38, 51~53, 56
//   examples/ex_sysid   系统辨识/通道校验 58~59（触地检测前提件：力矩通道 + 重力/质量）
//   examples/ex_probe   真机激励探针 60（500Hz 数据集录制：延迟/伺服/执行器辨识）
#include "app/examples_common.h"
#include "app/examples/ex_basic.h"
#include "app/examples/ex_diag.h"
#include "app/examples/ex_rl.h"
#include "app/examples/ex_sysid.h"
#include "app/examples/ex_probe.h"
