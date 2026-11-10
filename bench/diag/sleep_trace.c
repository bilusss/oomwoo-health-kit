/* Diagnostic LD_PRELOAD shim: log sleeps made by Fast DDS (dds.*) threads, with a backtrace at the start of each burst.
 * Observation only; does not change sleep behaviour.  Build: gcc -shared -fPIC -O1 -o sleep_trace.so sleep_trace.c -ldl -lpthread
 * Output: $SLEEP_TRACE_DIR/sleep_<pid>.log   lines: t_mono_s tid thread_name req_ms [BACKTRACE follows] */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <execinfo.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

static int fd = -2;
static __thread double last_t = -1.0;
static __thread int in_hook = 0;

static double now_s(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec / 1e9; }

static void log_sleep(double req_ms) {
    if (in_hook) return;
    in_hook = 1;
    char name[32] = {0};
    prctl(PR_GET_NAME, name, 0, 0, 0);
    if (strncmp(name, "dds", 3) != 0 && strncmp(name, "health", 6) != 0) { in_hook = 0; return; }
    if (fd == -2) {
        const char *d = getenv("SLEEP_TRACE_DIR");
        char p[512]; snprintf(p, sizeof p, "%s/sleep_%d.log", d ? d : "/tmp", (int)getpid());
        fd = open(p, O_WRONLY | O_CREAT | O_APPEND, 0644);
    }
    if (fd >= 0) {
        double t = now_s(); char line[200];
        int n = snprintf(line, sizeof line, "%.4f %ld %s %.3f\n", t, (long)syscall(SYS_gettid), name, req_ms);
        write(fd, line, n);
        if (last_t < 0 || t - last_t > 0.05) {
            void *bt[14]; int k = backtrace(bt, 14);
            write(fd, "BACKTRACE\n", 10);
            backtrace_symbols_fd(bt, k, fd);
        }
        last_t = t;
    }
    in_hook = 0;
}

int nanosleep(const struct timespec *req, struct timespec *rem) {
    static int (*real)(const struct timespec *, struct timespec *);
    if (!real) real = dlsym(RTLD_NEXT, "nanosleep");
    log_sleep(req->tv_sec * 1e3 + req->tv_nsec / 1e6);
    return real(req, rem);
}
int clock_nanosleep(clockid_t c, int flags, const struct timespec *req, struct timespec *rem) {
    static int (*real)(clockid_t, int, const struct timespec *, struct timespec *);
    if (!real) real = dlsym(RTLD_NEXT, "clock_nanosleep");
    double ms = req->tv_sec * 1e3 + req->tv_nsec / 1e6;
    if (flags & TIMER_ABSTIME) ms = -1;
    log_sleep(ms);
    return real(c, flags, req, rem);
}
int usleep(useconds_t us) {
    static int (*real)(useconds_t);
    if (!real) real = dlsym(RTLD_NEXT, "usleep");
    log_sleep(us / 1e3);
    return real(us);
}
