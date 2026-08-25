#pragma once
// 示例聚合头：main.cpp 只需 include 本文件即可调用全部示例函数。
// 示例实现已按功能拆分（阶段3）：
//   examples_common.*   公共 helper（RawTerminal / poll_key / g_rl_stop / rl_signal_handler）
//   examples/ex_basic   基础运动示例 17~23
//   examples/ex_diag    诊断/只读示例 24, 26~29, 33, 34
//   examples/ex_rl      RL/前馈/延迟辨识示例 25, 30~32, 35~38
#include "app/examples_common.h"
#include "app/examples/ex_basic.h"
#include "app/examples/ex_diag.h"
#include "app/examples/ex_rl.h"
