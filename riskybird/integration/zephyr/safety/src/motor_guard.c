/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * See include/riskybird/motor_guard.h for why this exists.
 *
 * RELATIONSHIP TO THE FLIGHT CONTROLLER'S OWN ENVELOPE. rose_flight_controller
 * already has safety_violation() -- tilt, body rate, velocity, height, battery,
 * debounced, latching g_estop. This is NOT a replacement for it and does not
 * duplicate it. Two differences decide where each one belongs:
 *
 *   - That envelope reads the ESTIMATOR's 12-DoF state. It needs the estimator
 *     running, so it cannot protect a workload that has no estimator. This
 *     one reads the accelerometer directly and works anywhere the BMI088 does.
 *   - That envelope has NO free-fall or impact test (verified by grep over
 *     src/main.cpp, 2026-09-21). Tilt cannot catch a drop: a frame dropped flat
 *     stays flat the whole way down and only the magnitude of |a| gives it away.
 *
 * So this covers the bench workloads that drive motors with no estimator --
 * bootup_check --mode motors above all, which is what ran while the drone was
 * knocked off the bench -- and adds the two magnitude tests neither had.
 */

#include <riskybird/motor_guard.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

#include <stdio.h>

#define GUARD_STACK_SIZE 2048
/*
 * Cooperative, so it cannot be starved by the application thread. The guard
 * sleeps every cycle, so a cooperative priority costs the rest of the system
 * nothing while guaranteeing the sample actually happens on schedule -- a
 * watchdog that runs only when the thing it is watching is idle is not one.
 */
#define GUARD_PRIORITY   K_PRIO_COOP(7)

static K_THREAD_STACK_DEFINE(guard_stack, GUARD_STACK_SIZE);
static struct k_thread guard_thread;
static k_tid_t guard_tid;

static const struct pwm_dt_spec *g_motors;
static size_t g_motor_count;
static uint32_t g_period_nsec;
static struct rb_motor_guard_config g_cfg;

static volatile bool g_running;
static volatile bool g_tripped;
static volatile enum rb_guard_trip g_reason;
static volatile int32_t g_trip_accel;
static volatile uint32_t g_samples;
static volatile int32_t g_live_tilt_pct;
static volatile int32_t g_live_accel_mm;

static const char *const REASON_STR[] = {
	[RB_GUARD_OK]       = "ok",
	[RB_GUARD_TILT]     = "tilt past the bench limit",
	[RB_GUARD_FREEFALL] = "free fall -- the airframe is falling",
	[RB_GUARD_IMPACT]   = "impact",
	[RB_GUARD_IMU_LOST] = "IMU stopped answering",
};

/* Zephyr reports m/s^2 as integer + millionths. Work in mm/s^2 so the whole
 * guard is integer: g is 9807, and every threshold is a plain comparison. */
static inline int32_t to_mm_s2(const struct sensor_value *v)
{
	return (int32_t)v->val1 * 1000 + v->val2 / 1000;
}

void rb_motor_guard_kill(enum rb_guard_trip why)
{
	/* Zero every channel first, report afterwards. printf on this board is a
	 * polling 115200 console that busy-waits for milliseconds; doing it
	 * before the motors are off would put the whole console latency between
	 * the decision and the motors actually stopping. */
	for (size_t i = 0; i < g_motor_count; i++) {
		(void)pwm_set_dt(&g_motors[i], g_period_nsec, 0);
	}

	if (!g_tripped) {
		g_tripped = true;
		g_reason = why;
	}
}

static void guard_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	const struct device *accel = DEVICE_DT_GET(DT_ALIAS(bmi088_accel));
	unsigned int tilt_run = 0, shock_run = 0, imu_fail_run = 0;

	while (g_running) {
		struct sensor_value v[3];
		int32_t ax, ay, az;
		int64_t horiz2, total2;
		int32_t mag2;

		if (sensor_sample_fetch(accel) != 0 ||
		    sensor_channel_get(accel, SENSOR_CHAN_ACCEL_XYZ, v) != 0) {
			/* One missed sample is a bus hiccup; a run of them means the
			 * guard is blind, and a blind guard must not be trusted to
			 * keep motors spinning. */
			if (++imu_fail_run >= 5U) {
				rb_motor_guard_kill(RB_GUARD_IMU_LOST);
				g_trip_accel = -1;
				return;
			}
			k_msleep(g_cfg.sample_ms);
			continue;
		}
		imu_fail_run = 0;

		ax = to_mm_s2(&v[0]);
		ay = to_mm_s2(&v[1]);
		az = to_mm_s2(&v[2]);

		horiz2 = (int64_t)ax * ax + (int64_t)ay * ay;
		total2 = horiz2 + (int64_t)az * az;

		/* |a| without a sqrt in the hot path: compare squares against
		 * squared thresholds. mag2 is only computed for the report. */
		{
			int64_t ff2  = (int64_t)g_cfg.freefall_mm_s2 * g_cfg.freefall_mm_s2;
			int64_t imp2 = (int64_t)g_cfg.impact_mm_s2 * g_cfg.impact_mm_s2;

			if (total2 < ff2 || total2 > imp2) {
				if (++shock_run >= g_cfg.shock_samples) {
					/* sqrt only here, once, off the hot path. */
					int32_t m = 0;
					while ((int64_t)(m + 1) * (m + 1) <= total2) { m++; }
					g_trip_accel = m;
					rb_motor_guard_kill(total2 < ff2 ? RB_GUARD_FREEFALL
									 : RB_GUARD_IMPACT);
					return;
				}
			} else {
				shock_run = 0;
			}
		}

		/*
		 * Tilt, with no trig: the frame is past theta when
		 *   horiz^2 * 100 > |a|^2 * sin^2(theta)%
		 * Skipped while |a| is implausible, because the direction of a
		 * vector that is not gravity says nothing about attitude -- and
		 * the magnitude tests above already own that case.
		 */
		if (total2 > 0) {
			if (horiz2 * 100 > total2 * (int64_t)g_cfg.tilt_sin2_pct) {
				if (++tilt_run >= g_cfg.tilt_samples) {
					int32_t m = 0;
					while ((int64_t)(m + 1) * (m + 1) <= total2) { m++; }
					g_trip_accel = m;
					rb_motor_guard_kill(RB_GUARD_TILT);
					return;
				}
			} else {
				tilt_run = 0;
			}
		}

		/* Publish what this sample saw, so a self-test can display the
		 * guard's own view rather than inferring it from silence. */
		{
			int32_t m = 0;

			while ((int64_t)(m + 1) * (m + 1) <= total2) { m++; }
			g_live_accel_mm = m;
			g_live_tilt_pct = (total2 > 0)
				? (int32_t)((horiz2 * 100) / total2)
				: 0;
		}

		mag2 = 0;
		(void)mag2;
		g_samples++;
		k_msleep(g_cfg.sample_ms);
	}
}

int rb_motor_guard_start(const struct pwm_dt_spec *motors, size_t count,
			 uint32_t period_nsec,
			 const struct rb_motor_guard_config *cfg)
{
	const struct device *accel = DEVICE_DT_GET(DT_ALIAS(bmi088_accel));

	if (motors == NULL || count == 0U) {
		return -EINVAL;
	}
	if (!device_is_ready(accel)) {
		/* Reported, never silently skipped: "no watchdog" must be a
		 * visible outcome, not the same as "watchdog saw nothing". */
		return -ENODEV;
	}
	if (g_running) {
		return -EALREADY;
	}

	g_motors = motors;
	g_motor_count = count;
	g_period_nsec = period_nsec;

	g_cfg.tilt_sin2_pct  = (cfg && cfg->tilt_sin2_pct)  ? cfg->tilt_sin2_pct  : 50U;
	g_cfg.freefall_mm_s2 = (cfg && cfg->freefall_mm_s2) ? cfg->freefall_mm_s2 : 3900U;
	g_cfg.impact_mm_s2   = (cfg && cfg->impact_mm_s2)   ? cfg->impact_mm_s2   : 29400U;
	g_cfg.sample_ms      = (cfg && cfg->sample_ms)      ? cfg->sample_ms      : 20U;
	g_cfg.tilt_samples   = (cfg && cfg->tilt_samples)   ? cfg->tilt_samples   : 3U;
	g_cfg.shock_samples  = (cfg && cfg->shock_samples)  ? cfg->shock_samples  : 2U;

	g_tripped = false;
	g_reason = RB_GUARD_OK;
	g_trip_accel = 0;
	g_samples = 0;
	g_running = true;

	guard_tid = k_thread_create(&guard_thread, guard_stack,
				    K_THREAD_STACK_SIZEOF(guard_stack),
				    guard_entry, NULL, NULL, NULL,
				    GUARD_PRIORITY, 0, K_NO_WAIT);
	if (guard_tid == NULL) {
		g_running = false;
		return -EAGAIN;
	}
	k_thread_name_set(guard_tid, "motor_guard");
	return 0;
}

void rb_motor_guard_stop(void)
{
	g_running = false;
	if (guard_tid != NULL) {
		k_thread_abort(guard_tid);
		guard_tid = NULL;
	}
	/* Park regardless of why we are stopping. */
	for (size_t i = 0; i < g_motor_count; i++) {
		(void)pwm_set_dt(&g_motors[i], g_period_nsec, 0);
	}
}

bool rb_motor_guard_tripped(void) { return g_tripped; }
enum rb_guard_trip rb_motor_guard_reason(void) { return g_reason; }
int32_t rb_motor_guard_trip_accel(void) { return g_trip_accel; }
uint32_t rb_motor_guard_samples(void) { return g_samples; }

void rb_motor_guard_live(int32_t *tilt_pct, int32_t *accel_mm)
{
	if (tilt_pct != NULL)  { *tilt_pct = g_live_tilt_pct; }
	if (accel_mm != NULL)  { *accel_mm = g_live_accel_mm; }
}

const char *rb_motor_guard_reason_str(void)
{
	enum rb_guard_trip r = g_reason;

	if ((unsigned)r >= ARRAY_SIZE(REASON_STR) || REASON_STR[r] == NULL) {
		return "unknown";
	}
	return REASON_STR[r];
}
