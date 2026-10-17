/**
 * @file    libusb_fastpoll.c
 * @brief   LD_PRELOAD 垫片：修正达妙 USB2CAN SDK 的 libusb 事件收割超时（2026-10-08）
 *
 * ⚠⚠ 2026-10-17 实测结论：**本垫片对真机攒批无效，run.sh 已不再自动加载。**
 *    攒批不在这个 libusb 事件超时上，而在 SDK 的**入站派发层**——帧几十 µs 就到 libusb
 *    回调（tool/libusb_xferprobe.c 实测完成间隔 32~127 µs），却每 ~77~100 ms 才交 ~50 帧
 *    给我们的 onRecv（Example64_FeedbackRateDiag + recv 日志的突发结构实测）。
 *    详见 memory/FACT.md §反馈链路。保留本文件仅供历史参考 / 手动复现实验。
 *
 * 【要解决的问题】
 *   libdm_device.so 内部的线程 can_bus_box::usb_handle_thread_func() 循环是：
 *       while (runningFlag)
 *           libusb_handle_events_timeout_completed(ctx, &{0, 100000}, ...);
 *   那个 100 ms 的超时写死在 .rodata（VMA 0x151d8，**全库仅此一处 100000**），反汇编：
 *       dba0: movdqa 0x7628(%rip),%xmm0   # 151d0   -> timeval{0, 100000}
 *       dbac: movaps %xmm0,(%rsp)
 *       dbb5: mov    %rsp,%rsi
 *       dbbd: call   libusb_handle_events_timeout_completed
 *   ⇒ USB 完成事件（CAN 回帧）每 ~100 ms 才被收割一次。驱动器明明每 2 ms 发一帧，
 *     上位机却只能拿到 ~100 ms 一批（实测每批每路 35~50 帧，恰好是 100 ms 的量）。
 *     后果：MotorManager::GetStatus() 的位置在两次收割之间**冻住**，控制环看到的关节
 *     位置实际只有 ~10 Hz 刷新率 —— 2026-10-08 的 PACE 采集里 8000 拍只有 387 拍发生变化。
 *
 * 【本垫片做什么】
 *   拦截 libusb_handle_events_timeout_completed()，把入参 timeval 的**等待上限**压到
 *   DMCAN_POLL_US（默认 1000 µs，可用环境变量覆盖；<=0 表示完全不干预 = 原行为）。
 *   语义上完全安全：该函数的契约就是"最多等这么久，有事件就提前返回"，
 *   缩短只影响等待上限，不改变任何判定逻辑与返回值语义。
 *
 * 【用法】
 *   LD_PRELOAD=./bin/libusb_fastpoll.so ./bin/can_motor_app   # 或直接 ./run.sh（已自动接线）
 *   DMCAN_POLL_US=500  LD_PRELOAD=... ./bin/can_motor_app     # 改成 500 µs
 *   DMCAN_POLL_US=0    LD_PRELOAD=... ./bin/can_motor_app     # 关闭（等价于不加载）
 *   启动时会往 stderr 打一行 "事件收割超时 0.100000s -> 1000 µs" 以确认垫片生效。
 *   每个**不同的**超时值只报一次 —— 实测除 SDK 的 100 ms 外，libusb 内部（同步传输路径）
 *   还会传别的值（见过 60 s），那些同样被压短（有意为之：该 API 就是"最多等这么久"）。
 *
 * ⚠ 与 SCHED_FIFO 的冲突：不要给 can_motor_app 加 file capability（setcap cap_sys_nice+ep）——
 *   内核会因此置 AT_SECURE，动态链接器将**忽略 LD_PRELOAD**，垫片失效。
 *   要同时拿到实时优先级和垫片，请用 /etc/security/limits.conf 的 rtprio（见 thread_manager.cpp）。
 *
 * 【构建】CMake 目标 usb_fastpoll → ./bin/libusb_fastpoll.so
 *
 * 【刻意不依赖 libusb 头文件/库】
 *   只做指针级转发，不碰 libusb_context 的内部结构，因此既不需要 libusb 头文件也不需要
 *   链接 libusb（只需要 libdl），不会因 conda / 系统 libusb 版本差异而构建失败。
 *   libusb_context 是不透明类型，用 void* 传递在 ABI 上完全等价。
 */
#define _GNU_SOURCE
#include <dlfcn.h>        /* dlsym / dlopen / RTLD_NEXT */
#include <stdio.h>        /* snprintf */
#include <stdlib.h>       /* getenv / strtol */
#include <string.h>       /* strcmp */
#include <sys/time.h>     /* struct timeval */
#include <unistd.h>       /* write */

typedef int (*handle_events_fn)(void* ctx, struct timeval* tv, int* completed);

static const char kSym[] = "libusb_handle_events_timeout_completed";

/* 目标超时（µs）。默认 1000；DMCAN_POLL_US 可覆盖；<=0 表示不干预。只读一次环境变量。 */
static long target_poll_us(void) {
    static long cached = -1;
    if (cached < 0) {
        const char* s = getenv("DMCAN_POLL_US");
        long v = 1000;
        if (s && *s) {
            char* end = NULL;
            long parsed = strtol(s, &end, 10);
            if (end && end != s) v = parsed;
        }
        cached = (v > 0) ? v : 0;
    }
    return cached;
}

/* 供应用层自检：垫片到底有没有被加载。
   LD_PRELOAD 漏写、被 setcap 的 AT_SECURE 忽略、或 DMCAN_POLL_US=0 关掉，
   这三种情况在外部完全看不出来 —— 应用侧 dlsym(RTLD_DEFAULT, "dsh_libusb_fastpoll_marker")
   取到本符号即说明垫片已加载（返回值还能说明是否生效）。 */
const char* dsh_libusb_fastpoll_marker(void) {
    static char buf[64];
    long us = target_poll_us();
    if (us > 0) snprintf(buf, sizeof buf, "生效中，事件收割上限 %ld µs", us);
    else        snprintf(buf, sizeof buf, "已加载但被 DMCAN_POLL_US=0 关闭");
    return buf;
}

/* 只投递一次的诊断输出（用 write 直写 fd 2，避开 stdio 重入风险） */
static void note(const char* msg) {
    size_t n = strlen(msg);
    ssize_t w = write(2, msg, n);
    (void)w;
}

/* 每个**不同的**超时值只报一次（上限 8 个），用来发现所有调用者：
   实测达妙 SDK 传 100 ms；libusb 内部（同步传输路径）还会传别的值（见过 60 s）。
   那些也一并压短是**有意为之** —— 该 API 的语义就是"最多等这么久"，缩短只提高收割频率。 */
static void note_once(long sec, long usec, long to_us) {
    enum { CAP = 8 };
    static long seen_s[CAP], seen_us[CAP];
    static int  nseen = 0;
    for (int i = 0; i < nseen; i++)
        if (seen_s[i] == sec && seen_us[i] == usec) return;
    if (nseen >= CAP) return;
    seen_s[nseen] = sec;
    seen_us[nseen] = usec;
    nseen++;
    char buf[192];
    int n = snprintf(buf, sizeof buf,
                     "[libusb_fastpoll] 事件收割超时 %ld.%06lds -> %ld µs"
                     "（达妙 SDK 写死 100 ms ⇒ CAN 回帧被攒成 ~100 ms 一批）\n",
                     sec, usec, to_us);
    if (n > 0) note(buf);
}

/* dlsym 返回 void*，而 ISO C（-Wpedantic）不允许 对象指针 → 函数指针 的直接转换。
   用 memcpy 走一遍，生成的代码与直接转换完全相同，且不触发告警。 */
static handle_events_fn as_fn(void* p) {
    handle_events_fn f = NULL;
    _Static_assert(sizeof f == sizeof p, "函数指针与数据指针大小不一致，无法安全转换");
    memcpy(&f, &p, sizeof f);
    return f;
}

int libusb_handle_events_timeout_completed(void* ctx, struct timeval* tv, int* completed) {
    static handle_events_fn real = NULL;

    if (!real) {
        real = as_fn(dlsym(RTLD_NEXT, kSym));
        if (!real) {
            /* RTLD_NEXT 兜底失败时，直接按 SONAME 找已加载的 libusb（RTLD_NOLOAD 不重复加载） */
            void* h = dlopen("libusb-1.0.so.0", RTLD_NOW | RTLD_NOLOAD);
            if (h) real = as_fn(dlsym(h, kSym));
        }
        if (!real) {
            /* 解析不到真实符号：垫片无能为力。宁可显式失败，也不要静默地把事件吞掉。 */
            note("[libusb_fastpoll] 🔴 无法解析真实符号 libusb_handle_events_timeout_completed，垫片失效\n");
            _exit(127);
        }
    }

    const long us = target_poll_us();
    if (tv && us > 0 && (tv->tv_sec > 0 || (long)tv->tv_usec > us)) {
        note_once((long)tv->tv_sec, (long)tv->tv_usec, us);
        struct timeval fast;
        fast.tv_sec  = 0;
        fast.tv_usec = (suseconds_t)us;
        return real(ctx, &fast, completed);
    }
    return real(ctx, tv, completed);
}
