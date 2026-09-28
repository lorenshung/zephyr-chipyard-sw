/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Attitude watchdog: kill every motor if the airframe stops being a drone
 * sitting still on a bench.
 *
 * WHY THIS EXISTS. The motor sweep in bootup_check drove motors with no
 * attitude supervision of any kind. On 2026-09-21 the drone was knocked off the
 * bench mid-sweep and kept driving all the way down and through whatever it hit,
 * because nothing in that path was watching. The flight controller has a tilt
 * envelope; the bench sweep had nothing, and the bench sweep is what runs most
 * often and is handled most.
 *
 * WHAT IT CATCHES, and why three conditions rather than one:
 *
 *   tilt       the frame is past a sane bench angle -- it has been picked up,
 *              tipped, or has walked off something.
 *   free fall  |a| collapses toward zero. THIS is the dropped case, and tilt
 *              alone does not catch it: a drone dropped flat stays level all
 *              the way down.
 *   impact     |a| spikes well past g. Catches the landing, so motors are
 *              already dead before the thing bounces into something else.
 *
 * WHAT IT CANNOT DO, stated plainly so nobody trusts it further than it goes:
 * this is software on the Rocket core. The SiFive PWM block is separate
 * hardware and keeps driving whatever its comparators hold, so a halted, reset
 * or crashed CPU is NOT covered -- the comparators must still be zeroed before
 * the core is halted, exactly as hardware/flight/stop.gdb does. It also cannot
 * outrun a fault faster than its sample period. It is a guard, not an interlock,
 * and the battery is still the only real kill switch.
 *
 * It is deliberately integer-only: this must link and run in a build with no
 * FPU enabled, and a guard that is compiled out of the cheap builds is a guard
 * that is absent exactly where the bench spends its time.
 */

#ifndef RISKYBIRD_MOTOR_GUARD_H_
#define RISKYBIRD_MOTOR_GUARD_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <zephyr/drivers/pwm.h>

/* Why each trip fired, for the report. */
enum rb_guard_trip {
	RB_GUARD_OK = 0,
	RB_GUARD_TILT,
	RB_GUARD_FREEFALL,
	RB_GUARD_IMPACT,
	RB_GUARD_IMU_LOST,
};

/*
 * Thresholds. Defaults are chosen for a propellers-off bench sweep, where the
 * frame should be flat and still; a flight build wants its own numbers.
 *
 * tilt_sin2_pct is sin^2(theta) as a percentage, which keeps the whole test in
 * integer arithmetic: the frame is past theta when
 *     (ax^2 + ay^2) * 100  >  (ax^2 + ay^2 + az^2) * tilt_sin2_pct
 * 50 is exactly 45 degrees. No trig, no floats, no sqrt.
 */
struct rb_motor_guard_config {
	unsigned int tilt_sin2_pct;    /* default 50 == 45 degrees */
	unsigned int freefall_mm_s2;   /* |a| below this trips; default 3900 (~0.4g) */
	unsigned int impact_mm_s2;     /* |a| above this trips; default 29400 (~3g) */
	unsigned int sample_ms;        /* default 20 (50 Hz) */
	unsigned int tilt_samples;     /* consecutive samples to trip; default 3 */
	unsigned int shock_samples;    /* for freefall/impact; default 2, faster */
};

/*
 * Start guarding. `motors` and `period_nsec` are what a trip will zero; the
 * array must outlive the guard, so pass a static one.
 *
 * Returns 0 on success. A guard that cannot start is reported to the caller
 * rather than being silently absent -- "the watchdog is running" is exactly the
 * kind of thing that must never be assumed.
 */
int rb_motor_guard_start(const struct pwm_dt_spec *motors, size_t count,
			 uint32_t period_nsec,
			 const struct rb_motor_guard_config *cfg);

/* Stop guarding. Motors are parked on the way out regardless of trip state. */
void rb_motor_guard_stop(void);

/* Has it tripped? Latched: once tripped it stays tripped until _start again. */
bool rb_motor_guard_tripped(void);

/* Why it tripped, and the |a| in mm/s^2 that did it. */
enum rb_guard_trip rb_motor_guard_reason(void);
int32_t rb_motor_guard_trip_accel(void);

/* Human-readable reason, always a valid static string. */
const char *rb_motor_guard_reason_str(void);

/*
 * Samples taken since the last start. Zero after a sweep means the guard never
 * ran -- a thread that failed to start would otherwise look identical to one
 * that ran and saw nothing wrong.
 */
uint32_t rb_motor_guard_samples(void);

/*
 * Live values, for a self-test that has to prove the guard can SEE. Without
 * these, "it did not trip" and "it is not looking" produce identical output,
 * which is the one thing a safety device must never do.
 *
 * tilt_pct is horiz^2*100/|a|^2, i.e. sin^2(theta) as a percentage -- directly
 * comparable to cfg.tilt_sin2_pct. accel_mm is |a| in mm/s^2.
 */
void rb_motor_guard_live(int32_t *tilt_pct, int32_t *accel_mm);

/* Kill now, from anywhere. Safe to call whether or not the guard is running. */
void rb_motor_guard_kill(enum rb_guard_trip why);

#endif /* RISKYBIRD_MOTOR_GUARD_H_ */
