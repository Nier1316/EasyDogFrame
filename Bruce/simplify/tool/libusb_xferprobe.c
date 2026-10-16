/*
 * tool/libusb_xferprobe.c —— 探测达妙 SDK 的 libusb 传输参数（定位 ~100 ms 攒批发生在哪一层）
 *
 * 背景（见 memory/FACT.md §反馈链路、tool/libusb_fastpoll.c）：
 *   CAN 回帧在**应用侧**只有 ~10 Hz（策略观测中位 4~5 个控制步看到同一份关节位置），
 *   而 `log/recv_*.csv` 显示帧其实以 ~500 帧/s/电机 到达、只是**成批投递**（~40 帧/100 ms）。
 *   已排除：不是"帧没来"，也不是 `libusb_handle_events_timeout_completed` 的 100 ms 超时
 *   （夹小该超时的垫片在真机上无效）。
 *
 * 本垫片要回答的问题：**一次 bulk IN 传输被请求了多少字节？多久完成一次？**
 *   · USB bulk IN 传输**要么收满请求长度、要么收到短包**才会完成。
 *     若 SDK 请求 16 KB，而设备数据率约 160 KB/s（16 电机 × 500 Hz × ~20 B），
 *     那么一次传输正好 ~100 ms 才完成 ⇒ 上层看到的就是"100 ms 一批"。
 *   · 若请求长度很小（几百字节）却仍 ~100 ms 才完成，说明是**设备固件**在攒包，
 *     则只能换传输层（SocketCAN）。
 *
 * 用法（不需要使能电机；任何会打开传输层的示例都会触发）：
 *   LD_PRELOAD=./bin/libusb_xferprobe.so ./bin/can_motor_app
 *   DMCAN_XFER_BYTES=1024 LD_PRELOAD=./bin/libusb_xferprobe.so ./bin/can_motor_app
 *       ↑ 验证性修复：把所有 bulk IN 的请求长度夹到 1024 B ⇒ 完成频率提高 ~16 倍
 *   DMCAN_XFERPROBE_QUIET=1 只打印汇总。
 *
 * 输出：
 *   [XFER] submit ep=0x81 type=BULK len=16384 timeout=100      ← 请求长度（首见即打印）
 *   [XFER] done   ep=0x81 actual=16384 status=0                 ← 实际收到（每 N 次打印一次）
 *   [XFER-PROBE] 汇总：提交/完成次数、actual_length 分布、**完成间隔直方图**（＝真正的攒批周期）
 * 判读：完成间隔集中在 ≈100 ms ⇒ 上层 10 Hz 就是这么来的。
 */
#define _GNU_SOURCE
#include <libusb.h>   /* 配 -I${CONDA_PREFIX}/include/libusb-1.0（pkg-config 惯例） */

#include <dlfcn.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

/* ---------------- 真实函数指针 ---------------- */
static int (*real_submit_transfer)(struct libusb_transfer*) = NULL;
static int (*real_bulk_transfer)(libusb_device_handle*, unsigned char, unsigned char*,
                                 int, int*, unsigned int) = NULL;

/* dlsym 返回 void*，ISO C（-Wpedantic）不允许对象指针→函数指针直接转换：
   用 memcpy 移植（与 tool/libusb_fastpoll.c 同一套写法）。 */
typedef int (*submit_fn_t)(struct libusb_transfer*);
typedef int (*bulk_fn_t)(libusb_device_handle*, unsigned char, unsigned char*,
                         int, int*, unsigned int);

static submit_fn_t as_submit_fn(void* p) {
    submit_fn_t f = NULL;
    _Static_assert(sizeof(f) == sizeof(p), "函数指针与对象指针大小必须一致");
    memcpy(&f, &p, sizeof(f));
    return f;
}

static bulk_fn_t as_bulk_fn(void* p) {
    bulk_fn_t f = NULL;
    _Static_assert(sizeof(f) == sizeof(p), "函数指针与对象指针大小必须一致");
    memcpy(&f, &p, sizeof(f));
    return f;
}

static int probe_quiet(void) {
    static int q = -1;
    if (q < 0) {
        const char* s = getenv("DMCAN_XFERPROBE_QUIET");
        q = (s && *s && *s != '0') ? 1 : 0;
    }
    return q;
}

/* DMCAN_XFER_BYTES：把所有 bulk IN 的请求长度夹到这个值（<=0 表示不改） */
static int xfer_clamp(void) {
    static long v = -2;
    if (v == -2) {
        const char* s = getenv("DMCAN_XFER_BYTES");
        v = (s && *s) ? atol(s) : 0;
    }
    return (int)v;
}

static int64_t now_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

/* ---------------- 提交签名统计 ---------------- */
#define MAX_SIG 32
typedef struct {
    unsigned char ep;
    unsigned char type;
    int len;
    unsigned long n;
} Sig;
static Sig g_sig[MAX_SIG];
static int g_nsig = 0;
static pthread_mutex_t g_mu = PTHREAD_MUTEX_INITIALIZER;

static const char* type_name(unsigned char t) {
    switch (t) {
        case LIBUSB_TRANSFER_TYPE_CONTROL:     return "CTRL";
        case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS: return "ISO";
        case LIBUSB_TRANSFER_TYPE_BULK:        return "BULK";
        case LIBUSB_TRANSFER_TYPE_INTERRUPT:   return "INT";
        default:                               return "?";
    }
}

#define GAP_BUCKETS 24

/* ---------------- 同步 bulk IN 统计（若回帧走这条路，这里就是答案） ---------------- */
static unsigned long g_in_calls = 0;
static long   g_in_min = -1, g_in_max = 0;
static double g_in_sum = 0;
static unsigned long g_in_len_hist[24];   /* 每次返回的字节数（2 的幂桶） */
static unsigned long g_in_dur[GAP_BUCKETS]; /* 每次调用阻塞时长 */
static int64_t g_in_last_us = 0;

/* ---------------- 完成统计（异步路径的攒批周期） ---------------- */
#define MAX_TRK 128
static struct libusb_transfer* g_track[MAX_TRK];
static libusb_transfer_cb_fn   g_orig[MAX_TRK];
static int g_ntrack = 0;

static unsigned long g_done = 0, g_done_fail = 0;
static long g_act_min = -1, g_act_max = 0;
static double g_act_sum = 0;
static int64_t g_last_done_us = 0;
static unsigned long g_gap[GAP_BUCKETS];   /* 桶 i: [2^(i-1), 2^i) µs */
static unsigned long g_ep_done[256];

static void note_gap(int64_t us) {
    int i = 0;
    while (us > 1 && i < GAP_BUCKETS - 1) { us >>= 1; ++i; }
    g_gap[i]++;
}

static void summary(void) {
    pthread_mutex_lock(&g_mu);
    int64_t t = now_us();
    fprintf(stderr,
            "\n[XFER-PROBE] ================= 汇总 =================\n"
            "  提交签名（ep/类型/请求长度 → 提交次数）:\n");
    for (int i = 0; i < g_nsig; i++)
        fprintf(stderr, "    ep=0x%02x %-4s len=%-8d → %lu 次%s\n",
                g_sig[i].ep, type_name(g_sig[i].type), g_sig[i].len, g_sig[i].n,
                (g_sig[i].type == LIBUSB_TRANSFER_TYPE_BULK && (g_sig[i].ep & 0x80))
                    ? "   ← bulk IN（CAN 回帧走这里）" : "");
    if (g_in_calls) {
        fprintf(stderr,
                "  同步 bulk IN：调用 %lu 次；返回字节 min=%ld avg=%.1f max=%ld\n",
                g_in_calls, g_in_min < 0 ? 0 : g_in_min,
                (g_in_min < 0) ? 0.0 : g_in_sum / (double)g_in_calls, g_in_max);
        fprintf(stderr, "    每次返回字节数分布（2 的幂桶）:\n");
        for (int i = 0; i < 24; i++)
            if (g_in_len_hist[i])
                fprintf(stderr, "      [%7d, %7d) B : %lu 次\n",
                        i ? (1 << (i - 1)) : 0, (1 << i) - 1, g_in_len_hist[i]);
        fprintf(stderr, "    **每次 IN 调用阻塞时长**（＝两次数据交付的间隔）:\n");
        for (int i = 0; i < GAP_BUCKETS; i++)
            if (g_in_dur[i])
                fprintf(stderr, "      %7lld µs ~ %7lld µs : %lu 次%s\n",
                        (long long)(i ? (1LL << (i - 1)) : 0), (long long)((1LL << i) - 1),
                        g_in_dur[i],
                        (i >= 16) ? "   ← ≈100 ms（问题所在）"
                                  : (i <= 2 ? "   ← 连续投递（健康）" : ""));
    }
    fprintf(stderr, "  异步完成 %lu 次（失败 %lu）；actual_length min=%ld avg=%.1f max=%ld\n",
            g_done, g_done_fail, g_act_min < 0 ? 0 : g_act_min,
            g_done ? g_act_sum / (double)g_done : 0.0, g_act_max);
    fprintf(stderr, "  **完成间隔直方图**（＝上位机真正能看到数据的周期）:\n");
    for (int i = 0; i < GAP_BUCKETS; i++) {
        if (!g_gap[i]) continue;
        const char* tag = (i <= 2)  ? "   ← 连续投递（健康）"
                        : (i >= 16) ? "   ← ≈100 ms 一批（问题所在）" : "";
        fprintf(stderr, "    %7lld µs ~ %7lld µs : %8lu 次%s\n",
                (long long)(i ? (1LL << (i - 1)) : 0), (long long)((1LL << i) - 1),
                g_gap[i], tag);
    }
    if (t - g_last_done_us > 1000000)
        fprintf(stderr, "  （距最后一次完成 %.1f s —— 期间无回帧）\n",
                (t - g_last_done_us) / 1e6);
    fprintf(stderr, "[XFER-PROBE] =========================================\n");
    pthread_mutex_unlock(&g_mu);
}

static void at_exit_summary(void) {
    if (g_done || g_nsig || g_in_calls) summary();
}

/* 包一层完成回调：记录 actual_length 与完成间隔，再调用 SDK 自己的回调 */
static void probe_completion(struct libusb_transfer* t) {
    libusb_transfer_cb_fn orig = NULL;
    pthread_mutex_lock(&g_mu);
    for (int i = 0; i < g_ntrack; i++)
        if (g_track[i] == t) { orig = g_orig[i]; break; }
    const int64_t tn = now_us();
    g_done++;
    if (t->status != LIBUSB_TRANSFER_COMPLETED) g_done_fail++;
    if (g_act_min < 0 || t->actual_length < g_act_min) g_act_min = t->actual_length;
    if (t->actual_length > g_act_max) g_act_max = t->actual_length;
    g_act_sum += t->actual_length;
    if (g_last_done_us) note_gap(tn - g_last_done_us);
    g_last_done_us = tn;
    g_ep_done[t->endpoint]++;
    const int show = !probe_quiet() && (g_done <= 4 || (g_done % 200) == 0);
    const unsigned long n = g_done;
    const int alen = t->actual_length, st = (int)t->status, ep = t->endpoint;
    pthread_mutex_unlock(&g_mu);
    if (show)
        fprintf(stderr, "[XFER] done #%lu ep=0x%02x actual=%-6d status=%d\n", n, ep, alen, st);
    if (orig) orig(t);
}

/* ---------------- 拦截点 ---------------- */
int libusb_submit_transfer(struct libusb_transfer* t) {
    if (!real_submit_transfer)
        real_submit_transfer = as_submit_fn(dlsym(RTLD_NEXT, "libusb_submit_transfer"));
    if (!t) return real_submit_transfer ? real_submit_transfer(t) : -1;

    const int is_bulk_in = (t->type == LIBUSB_TRANSFER_TYPE_BULK) && (t->endpoint & 0x80);
    const int clamp = xfer_clamp();

    pthread_mutex_lock(&g_mu);
    int found = 0;
    for (int i = 0; i < g_nsig; i++)
        if (g_sig[i].ep == t->endpoint && g_sig[i].type == t->type && g_sig[i].len == t->length) {
            g_sig[i].n++;
            found = 1;
            break;
        }
    if (!found && g_nsig < MAX_SIG) {
        g_sig[g_nsig].ep = t->endpoint;
        g_sig[g_nsig].type = t->type;
        g_sig[g_nsig].len = t->length;
        g_sig[g_nsig].n = 1;
        g_nsig++;
        if (!probe_quiet())
            fprintf(stderr, "[XFER] submit ep=0x%02x %s len=%d timeout=%u%s\n",
                    t->endpoint, type_name(t->type), t->length, t->timeout,
                    is_bulk_in ? "   ← bulk IN（CAN 回帧）" : "");
    }
    /* 包一层回调用一次即可（同一个 transfer 会被反复 submit，回调已是我们就不再包） */
    if (t->callback != probe_completion && g_ntrack < MAX_TRK) {
        int dup = 0;
        for (int i = 0; i < g_ntrack; i++)
            if (g_track[i] == t) { dup = 1; break; }
        if (!dup) {
            g_track[g_ntrack] = t;
            g_orig[g_ntrack] = t->callback;
            g_ntrack++;
            t->callback = probe_completion;
        }
    }
    pthread_mutex_unlock(&g_mu);

    if (is_bulk_in && clamp > 0 && t->length > clamp) {
        static int warned = 0;
        if (!warned) {
            warned = 1;
            fprintf(stderr, "[XFER-PROBE] DMCAN_XFER_BYTES=%d：把 bulk IN 请求长度由 %d 夹到 %d\n",
                    clamp, t->length, clamp);
        }
        t->length = clamp;
    }
    return real_submit_transfer ? real_submit_transfer(t) : -1;
}

int libusb_bulk_transfer(libusb_device_handle* dev, unsigned char ep, unsigned char* data,
                         int length, int* transferred, unsigned int timeout) {
    if (!real_bulk_transfer)
        real_bulk_transfer = as_bulk_fn(dlsym(RTLD_NEXT, "libusb_bulk_transfer"));

    /* 首见即登记 (ep, 请求长度) 签名 */
    pthread_mutex_lock(&g_mu);
    int found = 0;
    for (int i = 0; i < g_nsig; i++)
        if (g_sig[i].ep == ep && g_sig[i].type == LIBUSB_TRANSFER_TYPE_BULK &&
            g_sig[i].len == length) { g_sig[i].n++; found = 1; break; }
    if (!found && g_nsig < MAX_SIG) {
        g_sig[g_nsig].ep = ep; g_sig[g_nsig].type = LIBUSB_TRANSFER_TYPE_BULK;
        g_sig[g_nsig].len = length; g_sig[g_nsig].n = 1; g_nsig++;
        if (!probe_quiet())
            fprintf(stderr, "[XFER] sync bulk ep=0x%02x len=%-6d timeout=%-4u %s\n",
                    ep, length, timeout, (ep & 0x80) ? "← IN（回帧，阻塞读）" : "← OUT（发送）");
    }
    pthread_mutex_unlock(&g_mu);

    const int64_t t0 = now_us();
    const int r = real_bulk_transfer ? real_bulk_transfer(dev, ep, data, length, transferred, timeout)
                                     : -1;
    const int64_t dt = now_us() - t0;
    if (ep & 0x80) {                       /* 只有 IN 才是回帧 */
        const int tr = transferred ? *transferred : -1;
        pthread_mutex_lock(&g_mu);
        g_in_calls++;
        if (r == 0 && tr > 0) {
            if (g_in_min < 0 || tr < g_in_min) g_in_min = tr;
            if (tr > g_in_max) g_in_max = tr;
            g_in_sum += tr;
            int b = 0; while ((1 << b) < tr && b < 15) b++;   /* 2 的幂桶 */
            g_in_len_hist[b]++;
            const int64_t tn = now_us();
            if (g_in_last_us) note_gap(tn - g_in_last_us);
            g_in_last_us = tn;
        }
        int d = 0; { int64_t v = dt; while (v > 1 && d < GAP_BUCKETS - 1) { v >>= 1; ++d; } g_in_dur[d]++; }
        pthread_mutex_unlock(&g_mu);
        if (!probe_quiet() && (g_in_calls <= 3 || (g_in_calls % 200) == 0))
            fprintf(stderr, "[XFER] IN #%lu 返回 %d B（请求 %d）耗时 %.2f ms ret=%d\n",
                    g_in_calls, tr, length, dt / 1000.0, r);
    }
    return r;
}

__attribute__((constructor)) static void probe_init(void) {
    atexit(at_exit_summary);
    if (!probe_quiet())
        fprintf(stderr, "[XFER-PROBE] 已加载：探测 libusb 传输长度/完成间隔"
                        "（DMCAN_XFER_BYTES=<n> 可夹小 bulk IN 长度）\n");
}
