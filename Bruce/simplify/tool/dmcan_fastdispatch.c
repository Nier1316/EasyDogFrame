/*
 * tool/dmcan_fastdispatch.c —— 把达妙 SDK 的**入站派发线程**从 10 Hz 提速（2026-10-17）
 *
 * 【根因（已用反汇编 + .rodata 精确定位）】
 *   libdm_device.so 的 `can_bus_box::notification_thread_func()` 循环里：
 *       10318: movdqa 0x4ec0(%rip),%xmm0     # .rodata@0x151e0 = timespec{0, 100000000 ns}
 *       10325: movaps %xmm0,0x40(%rsp)       # 作为 struct timespec
 *       10330: call   nanosleep              # ← 每轮睡 **100 ms**
 *   收到的 CAN 帧由 libusb 回调（`transfer_callback`）压进队列，**由这个线程派发给我们的
 *   onRecv**。它 100 ms 才醒一次 ⇒ 上位机看到的就是"~10 Hz、每次 ~50 帧"的攒批，
 *   与实测完全吻合（`tool/libusb_xferprobe.c` 显示帧几十 µs 就到 libusb 回调）。
 *
 *   全库只有两个 nanosleep 调用点：
 *     · 0x10330  notification_thread_func → 100 ms（本垫片要夹的那个）
 *     · 0x136e9  heartbeat_thread_func    → 10 ms （**保持不动**）
 *
 * 【做法】LD_PRELOAD 拦截 `nanosleep`，**只在两个条件同时满足**时缩短：
 *     ① 调用方位于 libdm_device.so 内（用 dl_iterate_phdr 取模块地址区间 + 返回地址判断）
 *     ② 请求时长 > DMCAN_TICK_THRESHOLD_US（默认 20 ms）
 *   ⇒ 只命中派发线程的 100 ms，心跳 10 ms、以及**我们自己的 sleep/usleep 全部不受影响**。
 *
 * 【环境变量】
 *   DMCAN_TICK_US=<n>           夹到多少 µs（默认 1000 = 1 ms ⇒ 派发 ~1 kHz）
 *   DMCAN_TICK_THRESHOLD_US=<n> 只夹超过这个时长的请求（默认 20000 = 20 ms）
 *   DMCAN_FASTDISPATCH_QUIET=1  不打印统计
 *
 * 【用法】
 *   LD_PRELOAD=./bin/dmcan_fastdispatch.so ./bin/can_motor_app
 *   或在 run.sh 里自动加载；VS Code F5 可在 launch.json 的 "environment" 里加
 *   { "name": "LD_PRELOAD", "value": "${workspaceFolder}/bin/dmcan_fastdispatch.so" }
 *
 * 【验证】退出时打印"夹了几次、原请求时长分布"；配合 Example64 看每电机位置更新率
 *   是否从 ~8 Hz 提到 ~500 Hz。
 */
#define _GNU_SOURCE
#include <link.h>
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdint.h>

static int64_t now_us(void) {   /* 单调时钟，用于"进度每 10 s 报一次"的节流 */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + (int64_t)ts.tv_nsec / 1000LL;
}

typedef int (*nanosleep_fn)(const struct timespec*, struct timespec*);
static nanosleep_fn g_real = NULL;

/* libdm_device.so 的加载区间 */
static uintptr_t g_sdk_lo = 0, g_sdk_hi = 0;

static int g_quiet = -1;
static long g_tick_us = -1;
static long g_thresh_us = -1;

static int64_t g_last_report_us = 0;
static unsigned long g_clamped = 0;
static unsigned long g_passed = 0;
static long g_max_seen_us = 0;

static int quiet(void) {
    if (g_quiet < 0) {
        const char* s = getenv("DMCAN_FASTDISPATCH_QUIET");
        g_quiet = (s && *s && *s != '0') ? 1 : 0;
    }
    return g_quiet;
}

static long env_long(const char* name, long dflt) {
    const char* s = getenv(name);
    return (s && *s) ? atol(s) : dflt;
}

static long tick_us(void) {
    if (g_tick_us < 0) {
        g_tick_us = env_long("DMCAN_TICK_US", 1000);
        if (g_tick_us <= 0) g_tick_us = 1000;
    }
    return g_tick_us;
}

static long thresh_us(void) {
    if (g_thresh_us < 0) g_thresh_us = env_long("DMCAN_TICK_THRESHOLD_US", 20000);
    return g_thresh_us;
}

static int find_sdk(struct dl_phdr_info* info, size_t size, void* data) {
    (void)size; (void)data;
    const char* nm = info->dlpi_name ? info->dlpi_name : "";
    if (!strstr(nm, "libdm_device.so")) return 0;
    uintptr_t lo = (uintptr_t)info->dlpi_addr, hi = lo;
    for (int i = 0; i < info->dlpi_phnum; i++) {
        const ElfW(Phdr)* ph = &info->dlpi_phdr[i];
        if (ph->p_type != PT_LOAD) continue;
        const uintptr_t end = lo + ph->p_vaddr + ph->p_memsz;
        if (end > hi) hi = end;
    }
    g_sdk_lo = lo;
    g_sdk_hi = hi;
    return 1;   /* 找到就停 */
}

static int caller_in_sdk(void* ra) {
    if (!g_sdk_lo) dl_iterate_phdr(find_sdk, NULL);
    const uintptr_t a = (uintptr_t)ra;
    return g_sdk_lo && a >= g_sdk_lo && a < g_sdk_hi;
}

static void at_exit_report(void) {
    if (!g_clamped && !g_passed) return;
    fprintf(stderr,
            "\n[FASTDISPATCH] 拦截统计：缩短 %lu 次，原样放过 %lu 次"
            "（最大请求时长 %.1f ms；缩短到 %ld µs，阈值 %ld µs）\n",
            g_clamped, g_passed, g_max_seen_us / 1000.0, tick_us(), thresh_us());
}

__attribute__((constructor)) static void init(void) {
    atexit(at_exit_report);
    dl_iterate_phdr(find_sdk, NULL);
    /* 注意：构造时 libdm_device.so 可能还没被动态链接器加载（依赖顺序），所以这里
       **不判断是否找到**，也不打印"将不生效"（会误导）。真正的定位在首次调用时懒做。 */
    if (!quiet())
        fprintf(stderr, "[FASTDISPATCH] 已加载：把 libdm_device.so 内 >%ldµs 的 nanosleep "
                        "夹到 %ldµs（目标 = 派发线程的 100 ms 节拍）\n",
                thresh_us(), tick_us());
}

int nanosleep(const struct timespec* req, struct timespec* rem) {
    if (!g_real) {
        void* p = dlsym(RTLD_NEXT, "nanosleep");
        memcpy(&g_real, &p, sizeof(g_real));
        if (!g_real) return -1;
    }
    if (!req) return g_real(req, rem);

    const long us = (long)req->tv_sec * 1000000L + req->tv_nsec / 1000L;
    if (us > g_max_seen_us) g_max_seen_us = us;

    if (us > thresh_us() && caller_in_sdk(__builtin_return_address(0))) {
        struct timespec t;
        t.tv_sec  = tick_us() / 1000000L;
        t.tv_nsec = (tick_us() % 1000000L) * 1000L;
        if (g_clamped == 0 && !quiet())
            fprintf(stderr, "[FASTDISPATCH] ✅ 首次命中：libdm_device.so @0x%lx-0x%lx，"
                            "把请求 %ld µs 夹到 %ld µs\n",
                    (unsigned long)g_sdk_lo, (unsigned long)g_sdk_hi, us, tick_us());
        g_clamped++;
        // 进度打印改成**每 10 秒最多一次**（原先每 200 次一次 ⇒ 派发跑 1 kHz 时每秒刷 5 行）
        if (!quiet()) {
            const int64_t now = now_us();
            if (now - g_last_report_us >= 10000000LL) {
                g_last_report_us = now;
                fprintf(stderr, "[FASTDISPATCH] 已缩短 %lu 次 / 原样放过 %lu 次"
                                "（每 10 s 报一次；静默：DMCAN_FASTDISPATCH_QUIET=1）\n",
                        g_clamped, g_passed);
            }
        }
        return g_real(&t, NULL);      /* 缩短后正常睡完 ⇒ 不设 rem */
    }
    g_passed++;
    return g_real(req, rem);
}

/* 供 main() 自检（与 libusb_fastpoll 同一套 marker 机制） */
const char* dsh_dmcan_fastdispatch_marker(void) {
    return "dmcan_fastdispatch：夹 libdm_device.so 的 100 ms 派发节拍";
}
