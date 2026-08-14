#include "sensors.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <stdarg.h>

/* ---- tunables ---- */
#define TICK_MS            100      /* main loop period                         */
#define DEBOUNCE_N         3        /* consecutive qualifying samples to fire   */
#define DETECT_CM          15       /* object present if distance < this (cm)   */
#define PLACE_MG           200000L  /* parcel if weight rises > this (200 g)    */
#define OPEN_HOLD_MS       3000     /* dwell at +60 deg before returning (ms)   */
#define SETTLE_MS          800      /* let servo seat after returning to HOME   */
#define ERROR_COOLDOWN_MS  2000     /* pause before retrying after errors       */
#define MAX_CONSEC_ERR     5        /* sensor failures before ERROR             */
#define STATUS_EVERY_MS    1000     /* throttle heartbeat logging               */
#define BASELINE_SAMPLES   5        /* readings averaged for the weight baseline*/

enum state {
        ST_IDLE = 0,
        ST_ACTING,
        ST_ERROR,
};

static const char *state_name(enum state s)
{
        switch (s) {
        case ST_IDLE:   return "IDLE";
        case ST_ACTING: return "ACTING";
        case ST_ERROR:  return "ERROR";
        }
        return "?";
}

static volatile sig_atomic_t g_run = 1;

static void on_sigint(int sig)
{
        (void)sig;
        g_run = 0;
}

static long now_ms(void)
{
        struct timespec ts;

        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static void log_line(const char *fmt, ...)
{
        static long t0;
        va_list ap;

        if (!t0)
                t0 = now_ms();
        printf("[%6.1fs] ", (now_ms() - t0) / 1000.0);
        va_start(ap, fmt);
        vprintf(fmt, ap);
        va_end(ap);
        printf("\n");
        fflush(stdout);
}

/* Sample the load cell a few times and average, to fix a stable baseline.
 * Returns 0 / -errno; on success writes the averaged weight to *out. */
static int sample_baseline(long *out)
{
        long sum = 0;
        int  i, got = 0;

        for (i = 0; i < BASELINE_SAMPLES; i++) {
                long mg;

                if (weight_read_mg(&mg) == 0) {
                        sum += mg;
                        got++;
                }
                usleep(20 * 1000);
        }
        if (got == 0)
                return -1;
        *out = sum / got;
        return 0;
}

int main(void)
{
        enum state st = ST_IDLE;
        long state_enter = 0;
        long last_status = 0;
        long baseline_mg = 0;
        int  trig_cnt = 0, err_cnt = 0;
        int  ret;

        signal(SIGINT, on_sigint);
        signal(SIGTERM, on_sigint);

        ret = sensor_init();
        if (ret) {
                fprintf(stderr, "sensor_init failed: %d\n", -ret);
                return 1;
        }

        /* servo is at HOME after sensor_init(); establish the weight baseline */
        if (sample_baseline(&baseline_mg)) {
                fprintf(stderr, "could not sample initial weight baseline\n");
                sensor_cleanup();
                return 1;
        }

        log_line("locker ready. paddle at HOME. baseline=%ldmg (%.1fg).",
                 baseline_mg, baseline_mg / 1000.0);
        log_line("initial state: %s", state_name(st));
        state_enter = now_ms();

        while (g_run) {
                long t  = now_ms();
                int  cm = 0;
                long mg = 0;

                switch (st) {
                case ST_IDLE: {
                        int  near = 0, heavy = 0;
                        long delta;

                        ret = distance_read_cm(&cm);
                        if (ret) {
                                if (++err_cnt >= MAX_CONSEC_ERR)
                                        { st = ST_ERROR; state_enter = t; }
                                break;
                        }
                        ret = weight_read_mg(&mg);
                        if (ret) {
                                if (++err_cnt >= MAX_CONSEC_ERR)
                                        { st = ST_ERROR; state_enter = t; }
                                break;
                        }
                        err_cnt = 0;

                        delta = mg - baseline_mg;
                        near  = (cm > 0 && cm < DETECT_CM);
                        heavy = (delta > PLACE_MG);

                        /* both conditions must hold simultaneously */
                        if (near && heavy)
                                trig_cnt++;
                        else
                                trig_cnt = 0;

                        if (t - last_status > STATUS_EVERY_MS) {
                                log_line("IDLE: dist=%dcm  dW=%+ldmg (%.1fg)  [%s%s] %d/%d",
                                         cm, delta, delta / 1000.0,
                                         near ? "N" : "-", heavy ? "H" : "-",
                                         trig_cnt, DEBOUNCE_N);
                                last_status = t;
                        }

                        if (trig_cnt >= DEBOUNCE_N) {
                                log_line("parcel confirmed (dist=%dcm, dW=%.1fg) -> sweeping paddle",
                                         cm, delta / 1000.0);
                                trig_cnt = 0;
                                if (lock_open()) {   /* rotate to +60 deg (OPEN) */
                                        log_line("servo move failed -> ERROR");
                                        st = ST_ERROR; state_enter = t; break;
                                }
                                st = ST_ACTING;
                                state_enter = t;
                        }
                        break;
                }

                case ST_ACTING:
                        /* open-loop dwell at +60 deg, then return home */
                        if (t - state_enter < OPEN_HOLD_MS)
                                break;                 /* still holding OPEN */

                        if (lock_close()) {            /* unconditional return to HOME */
                                log_line("servo return failed -> ERROR");
                                st = ST_ERROR; state_enter = t; break;
                        }

                        /* let it seat, then re-baseline and go idle */
                        usleep(SETTLE_MS * 1000);
                        {
                                int pos = -1;

                                servo_present_position(&pos);
                                if (sample_baseline(&baseline_mg) == 0)
                                        log_line("returned HOME (pos=%d). new baseline=%.1fg. -> IDLE",
                                                 pos, baseline_mg / 1000.0);
                                else
                                        log_line("returned HOME (pos=%d). baseline resample failed. -> IDLE",
                                                 pos);
                        }
                        st = ST_IDLE;
                        state_enter = t;
                        last_status = t;
                        break;

                case ST_ERROR:
                        if (t - state_enter > ERROR_COOLDOWN_MS) {
                                log_line("recovering from ERROR -> HOME, IDLE");
                                err_cnt = 0;
                                trig_cnt = 0;
                                lock_close();          /* fail safe: return paddle home */
                                usleep(SETTLE_MS * 1000);
                                sample_baseline(&baseline_mg);
                                st = ST_IDLE;
                                state_enter = t;
                                last_status = t;
                        }
                        break;
                }

                usleep(TICK_MS * 1000);
        }

        log_line("shutting down: releasing torque, closing devices.");
        sensor_cleanup();
        return 0;
}
