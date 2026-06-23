// SPDX-License-Identifier: GPL-2.0
/*
 * main.c - smart parcel locker controller (polling state machine)
 *
 * Pure userspace, no hardware specifics here: every device access goes through
 * the HAL in sensors.c. The control logic is an event-driven state machine
 * driven by a fixed-period polling loop.
 *
 * Flow:
 *   STANDBY ---(someone within DETECT_CM, debounced)---> DETECTED
 *   DETECTED ---(tare scale, open door)---------------> WAIT_PARCEL
 *   WAIT_PARCEL ---(weight rises past PLACE_MG)--------> PARCEL_PLACED
 *               ---(timeout, nobody placed anything)--> close, STANDBY
 *   PARCEL_PLACED ---(record weight)------------------> LOCKING
 *   LOCKING ---(drive to locked, settle)--------------> STANDBY
 *   any sensor failure (repeated) --------------------> ERROR --> STANDBY
 *
 * Engineering points worth explaining:
 *   - Debounce: a transition needs DEBOUNCE_N consecutive qualifying samples,
 *     so a single noisy ultrasonic ping can't false-trigger.
 *   - Timeouts: every "waiting" state has a deadline and falls back safely.
 *   - Graceful shutdown: SIGINT releases torque and closes fds.
 *   - Polling (not pure IRQ) is a deliberate trade-off: simple, deterministic
 *     loop period, easy to reason about; fast enough for this application.
 */
#include "sensors.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <stdarg.h>

/* ---- tunables ---- */
#define TICK_MS              100	/* main loop period               */
#define DEBOUNCE_N           3	/* consecutive samples to confirm */
#define DETECT_CM            30	/* presence if distance < this    */
#define PLACE_MG             50000L	/* parcel if weight > this (50 g) */
#define WAIT_PARCEL_MS       15000	/* give up waiting for a parcel   */
#define LOCK_SETTLE_MS       1500	/* time for the door to seat      */
#define ERROR_COOLDOWN_MS    2000	/* pause before retrying          */
#define MAX_CONSEC_ERR       5	/* sensor failures before ERROR   */
#define STATUS_EVERY_MS      1000	/* throttle heartbeat logging     */

enum state {
	ST_STANDBY = 0,
	ST_DETECTED,
	ST_WAIT_PARCEL,
	ST_PARCEL_PLACED,
	ST_LOCKING,
	ST_ERROR,
};

static const char *state_name(enum state s)
{
	switch (s) {
	case ST_STANDBY:       return "STANDBY";
	case ST_DETECTED:      return "DETECTED";
	case ST_WAIT_PARCEL:   return "WAIT_PARCEL";
	case ST_PARCEL_PLACED: return "PARCEL_PLACED";
	case ST_LOCKING:       return "LOCKING";
	case ST_ERROR:         return "ERROR";
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

int main(void)
{
	enum state st = ST_STANDBY;
	long state_enter = 0;
	long last_status = 0;
	int  detect_cnt = 0, place_cnt = 0, err_cnt = 0;
	long parcel_mg = 0;
	int  ret;

	signal(SIGINT, on_sigint);
	signal(SIGTERM, on_sigint);

	ret = sensor_init();
	if (ret) {
		fprintf(stderr, "sensor_init failed: %d\n", -ret);
		return 1;
	}

	log_line("locker ready. door locked. waiting for activity.");
	log_line("initial state: %s", state_name(st));
	state_enter = now_ms();

	while (g_run) {
		long t = now_ms();
		int  cm = 0;
		long mg = 0;

		switch (st) {
		case ST_STANDBY: {
			ret = distance_read_cm(&cm);
			if (ret) {
				if (++err_cnt >= MAX_CONSEC_ERR)
					{ st = ST_ERROR; state_enter = t; }
				break;
			}
			err_cnt = 0;

			if (cm > 0 && cm < DETECT_CM)
				detect_cnt++;
			else
				detect_cnt = 0;

			if (t - last_status > STATUS_EVERY_MS) {
				log_line("STANDBY: distance=%dcm (%d/%d)",
					 cm, detect_cnt, DEBOUNCE_N);
				last_status = t;
			}

			if (detect_cnt >= DEBOUNCE_N) {
				log_line("presence confirmed at %dcm -> opening door",
					 cm);
				detect_cnt = 0;
				st = ST_DETECTED;
				state_enter = t;
			}
			break;
		}

		case ST_DETECTED:
			/* zero the scale, then open the door for the user */
			if (weight_tare())
				log_line("warning: tare failed");
			if (lock_open()) {
				log_line("lock_open failed -> ERROR");
				st = ST_ERROR; state_enter = t; break;
			}
			log_line("door open. waiting for parcel (timeout %ds)",
				 WAIT_PARCEL_MS / 1000);
			place_cnt = 0;
			st = ST_WAIT_PARCEL;
			state_enter = t;
			last_status = t;
			break;

		case ST_WAIT_PARCEL:
			ret = weight_read_mg(&mg);
			if (ret) {
				if (++err_cnt >= MAX_CONSEC_ERR)
					{ st = ST_ERROR; state_enter = t; }
				break;
			}
			err_cnt = 0;

			if (mg > PLACE_MG)
				place_cnt++;
			else
				place_cnt = 0;

			if (t - last_status > STATUS_EVERY_MS) {
				log_line("WAIT_PARCEL: weight=%ldmg (%d/%d)",
					 mg, place_cnt, DEBOUNCE_N);
				last_status = t;
			}

			if (place_cnt >= DEBOUNCE_N) {
				parcel_mg = mg;
				place_cnt = 0;
				st = ST_PARCEL_PLACED;
				state_enter = t;
			} else if (t - state_enter > WAIT_PARCEL_MS) {
				log_line("no parcel placed in time -> closing");
				lock_close();
				st = ST_STANDBY;
				state_enter = t;
				last_status = t;
			}
			break;

		case ST_PARCEL_PLACED:
			log_line("parcel detected: %ld mg (%.1f g) -> locking",
				 parcel_mg, parcel_mg / 1000.0);
			st = ST_LOCKING;
			state_enter = t;
			break;

		case ST_LOCKING:
			if (lock_close()) {
				log_line("lock_close failed -> ERROR");
				st = ST_ERROR; state_enter = t; break;
			}
			/* give the door time to seat before going idle */
			if (t - state_enter > LOCK_SETTLE_MS) {
				int pos = -1;

				servo_present_position(&pos);
				log_line("locked (servo pos=%d). back to standby.", pos);
				st = ST_STANDBY;
				state_enter = t;
				last_status = t;
			}
			break;

		case ST_ERROR:
			if (t - state_enter > ERROR_COOLDOWN_MS) {
				log_line("recovering from ERROR -> standby");
				err_cnt = 0;
				detect_cnt = place_cnt = 0;
				lock_close();		/* fail safe: latch the door */
				st = ST_STANDBY;
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
