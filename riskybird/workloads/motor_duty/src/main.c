/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * RiskyBird fixed-duty motor commander: the FPGA half of a thrust-curve bench
 * test, with no flight stack behind it.
 *
 * WHY THIS EXISTS
 * ---------------
 * Nobody has measured this airframe's thrust versus commanded duty. The 58-60 g
 * mass and the ~71 % hover duty in the docs are inherited numbers, and the
 * 0.15 break-away was measured through the FPGA's own PWM comparators -- a path
 * that no longer drives the gates at all. Every gate now belongs to the
 * ESP32-C6, which applies duty frames arriving from the FPGA on uart1
 * (workloads/esp_motors). So a thrust measurement made any other way is a
 * measurement of a path that is not the flight path.
 *
 * The flight controller can send those frames, but only as the output of a
 * closed control loop: there is no way to ask it for "motor 2 at 18.00 %, held,
 * for six seconds". This workload is that ask, and nothing else. It emits the
 * same 14-byte frames on the same wire at the same 50 Hz, so the ESP cannot
 * tell it apart from the flight controller -- which is the point: what gets
 * measured is the real actuator chain, gate FET and all.
 *
 * THE FPGA DRIVES NO GATE HERE, AND THAT IS STRUCTURAL
 * ----------------------------------------------------
 * There is no CONFIG_PWM in prj.conf, no `motors` alias in any overlay this
 * workload selects, and no <zephyr/drivers/pwm.h> in this file. pwm_sifive_init()
 * therefore never runs and never writes pwmcfg, so the four motor balls stay in
 * their power-on state for the whole session. That state is measured, not
 * assumed: hardware/pwm/run.sh Verilates the PWMTimer that the Arty shells
 * elaborate and reports "power-on: pin is low before software runs -- raw
 * comparator 100% high, pin 0% high", i.e. the harness inversion lands and the
 * pad is low while the comparator reads asserted. Same reasoning, and the same
 * words, as flight_controller-esp-motors.conf.
 *
 * SAFETY MODEL
 * ------------
 * Four independent bounds, because a thrust test is the one bench test that
 * wants propellers fitted:
 *
 *   ceiling     MOTOR_DUTY_MAX is the highest duty this image can put on the
 *               wire. It defaults to 25.00 %, which is also the ESP's own
 *               independent ceiling (ESP_MOTORS_MAX_DUTY_PCT) -- so at the
 *               default NEITHER end can produce flight thrust, and raising one
 *               without the other changes nothing at the gate.
 *   dwell       An armed step ends by itself after MOTOR_DUTY_DWELL_MS. Nothing
 *               has to be pressed to stop it; something has to be pressed to
 *               keep it going.
 *   stall cap   Below MOTOR_DUTY_BREAKAWAY a motor may simply not turn, and a
 *               stalled brushed motor on a low-side SI2302 is a resistor: it
 *               heats fast and it is the reason a break-away sweep is the most
 *               damaging test in this document. Any step below that threshold
 *               is cut after MOTOR_DUTY_STALL_MS regardless of the dwell.
 *               (There is no ESC and no locked-rotor latch on this airframe --
 *               brushed motors, low-side FETs. The old "the ESC latches until
 *               the battery is pulled" story is refuted and is NOT why this cap
 *               exists; heat is.)
 *   budget      Cumulative armed time is capped at MOTOR_DUTY_BUDGET_MS per
 *               boot. When it runs out the image latches off and says so; a
 *               board reset is the only way back. A bench session cannot drift
 *               into an hour of spinning motors.
 *
 * And behind all four, the two that do not depend on this image being correct:
 * the ESP cuts all four motors if frames stop for 120 ms (so halting the core,
 * pulling the ribbon or quitting the debugger all stop the motors), and pulling
 * the battery is the hard kill.
 *
 * CONSOLE KEYS
 * ------------
 *   1 2 3 4   toggle that motor in the selection
 *   a / n     select all four / none
 *   + -       duty +/- 1.00 %          ] [   duty +/- 5.00 %
 *   g         GO: arm the selection at the current duty for one dwell
 *   x q SPC   STOP now (also aborts a sweep)
 *   t         run the scripted step sweep (g = next step early, x = abort)
 *   ?         reprint this
 *
 * Duties are shown and entered in hundredths of a percent, the wire's own unit,
 * so nothing is rounded between the key press and the frame.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <riskybird/motor_link.h>

#if !DT_NODE_EXISTS(DT_ALIAS(esp_uart))
#error "motor_duty needs the esp-uart alias: build against a shell that elaborates \
serial@10021000 so rb appends hardware/zephyr/fpga-esp-uart.overlay."
#endif

static const struct device *const esp_link = DEVICE_DT_GET(DT_ALIAS(esp_uart));
static const struct device *const console =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

/* ---- tunables (RB_EXTRA_CFLAGS=-D...) ------------------------------------ */

/* Frame rate. 50 Hz is what main.cpp's offload path sends and is 4x the margin
 * on the receiver's 120 ms failsafe. */
#ifndef MOTOR_DUTY_HZ
#define MOTOR_DUTY_HZ 50
#endif
#define FRAME_PERIOD_MS (1000 / (MOTOR_DUTY_HZ))

/* Highest duty this image will put on the wire, in 1/10000. 2500 = 25.00 %,
 * matching the ESP's own ceiling so the two agree at the default. */
#ifndef MOTOR_DUTY_MAX
#define MOTOR_DUTY_MAX 2500
#endif

/* How long one armed step lasts before it ends by itself. */
#ifndef MOTOR_DUTY_DWELL_MS
#define MOTOR_DUTY_DWELL_MS 6000
#endif

/* Break-away duty, in 1/10000. The recorded 0.15 came from the FPGA PWM path
 * and has never been re-measured through the ESP; it is used here only to
 * decide which steps get the short stall cap, so being wrong costs a shorter
 * dwell, never a longer one. */
#ifndef MOTOR_DUTY_BREAKAWAY
#define MOTOR_DUTY_BREAKAWAY 1500
#endif

/* Dwell cap for a step below the break-away duty: long enough to see whether
 * the motor turns, short enough that a stalled one does not cook. */
#ifndef MOTOR_DUTY_STALL_MS
#define MOTOR_DUTY_STALL_MS 2000
#endif

/* Cumulative armed time per boot. */
#ifndef MOTOR_DUTY_BUDGET_MS
#define MOTOR_DUTY_BUDGET_MS 120000
#endif

/* Scripted sweep: start, step, stop (1/10000) and the cool-down between steps.
 * The cool-down is longer than the dwell on purpose -- brushed motors and their
 * FETs heat, and a thrust curve measured on a hot airframe sags against one
 * measured on a cold one. */
#ifndef MOTOR_DUTY_SWEEP_START
#define MOTOR_DUTY_SWEEP_START 500
#endif
#ifndef MOTOR_DUTY_SWEEP_STEP
#define MOTOR_DUTY_SWEEP_STEP 250
#endif
#ifndef MOTOR_DUTY_SWEEP_STOP
#define MOTOR_DUTY_SWEEP_STOP 2500
#endif
#ifndef MOTOR_DUTY_SWEEP_COOL_MS
#define MOTOR_DUTY_SWEEP_COOL_MS 9000
#endif

/*
 * THE FRAME RATE IS A SAFETY PARAMETER, NOT A PREFERENCE.
 *
 * The ESP cuts all four motors if no valid frame arrives for
 * ESP_MOTORS_TIMEOUT_MS (120 ms). A commander that sends slower than ~8 Hz
 * therefore has its motors cut BETWEEN ITS OWN FRAMES -- which does not look
 * like a failsafe from the bench, it looks like a motor that stutters, or a
 * thrust reading that sags partway through a dwell and cannot be reproduced.
 * It is the single easiest way to get confusing numbers out of this test.
 *
 * So the rate is asserted at build time rather than trusted. 20 Hz (50 ms) is
 * the floor allowed here: 2.4x margin on the timeout, which leaves room for the
 * console printfs this image makes while armed (a ~90-character status line at
 * 115200 on a polled UART is ~8 ms).
 */
BUILD_ASSERT(MOTOR_DUTY_HZ >= 20,
	     "motor_duty must send at >= 20 Hz: the ESP cuts all four motors after "
	     "120 ms without a valid frame (ESP_MOTORS_TIMEOUT_MS), so a slower "
	     "commander fails its own measurement mid-dwell.");
BUILD_ASSERT(FRAME_PERIOD_MS >= 1, "MOTOR_DUTY_HZ is too high to express as a period");
BUILD_ASSERT(MOTOR_DUTY_MAX > 0 && MOTOR_DUTY_MAX <= (int)MOTOR_LINK_DUTY_FULL,
	     "MOTOR_DUTY_MAX is in 1/10000 of full scale: 2500 = 25.00 %. It must be "
	     "within 1..10000.");
BUILD_ASSERT(MOTOR_DUTY_SWEEP_START > 0 && MOTOR_DUTY_SWEEP_STEP > 0,
	     "a sweep that starts at 0 or steps by 0 never terminates");

/* ---- state --------------------------------------------------------------- */

static uint16_t g_duty = 1000;                 /* commanded duty, 1/10000 */
static uint8_t  g_sel = 0x0F;                  /* which motors the duty applies to */
static bool     g_armed;
static bool     g_latched;                     /* budget exhausted: refuse to arm again */
static int64_t  g_arm_until;
static int64_t  g_armed_ms;                    /* cumulative, this boot */
static int64_t  g_arm_started;
static uint32_t g_frames;

/* scripted sweep */
static bool     g_sweeping;
static uint16_t g_sweep_duty;
static bool     g_sweep_cooling;
static int64_t  g_sweep_next;
static bool     g_sweep_drift_check;   /* the repeat of the first step, at the end */

/* Heartbeat while armed: the operator has to be able to see that frames are
 * still flowing, because "the motors stopped" and "the frames stopped" are
 * different faults with the same symptom. */
static int64_t  g_next_hold_print;
static uint32_t g_frames_at_arm;

static void tx(uint16_t duty[MOTOR_LINK_NMOTORS], uint8_t flags)
{
	static uint8_t seq;
	uint8_t frame[MOTOR_LINK_FRAME_LEN];

	motor_link_encode(frame, ++seq, flags, duty);
	for (unsigned i = 0; i < MOTOR_LINK_FRAME_LEN; i++) {
		uart_poll_out(esp_link, frame[i]);
	}
	g_frames++;
}

/*
 * The only place a non-zero duty is put on the wire.
 *
 * Every bound is applied here rather than at the key handler, so no command
 * path can reach the link without passing all of them -- the same argument
 * send_control() makes in the flight controller.
 */
static void send_now(void)
{
	uint16_t duty[MOTOR_LINK_NMOTORS] = { 0, 0, 0, 0 };
	uint8_t flags = 0u;

	if (g_armed && !g_latched) {
		uint16_t d = g_duty;

		if (d > (uint16_t)MOTOR_DUTY_MAX) {
			d = (uint16_t)MOTOR_DUTY_MAX;
		}
		for (unsigned i = 0; i < MOTOR_LINK_NMOTORS; i++) {
			duty[i] = (g_sel & (1u << i)) ? d : 0u;
		}
		flags = MOTOR_LINK_FLAG_ARMED;
	}
	tx(duty, flags);
}

static void disarm(const char *why)
{
	if (g_armed) {
		g_armed_ms += k_uptime_get() - g_arm_started;
		g_armed = false;
		/* Explicit, immediately, three times: "commanded off" and "link
		 * dead" are different things and the receiver should be told
		 * which one this is rather than inferring it from 120 ms of
		 * silence. Same reasoning as motors_shutdown() in main.cpp. */
		for (int i = 0; i < 3; i++) {
			send_now();
		}
		printf("MOTORDUTY OFF (%s) -- armed %d ms of the %d ms budget\n",
		       why, (int)g_armed_ms, (int)MOTOR_DUTY_BUDGET_MS);
	}
	if (g_armed_ms >= (int64_t)MOTOR_DUTY_BUDGET_MS && !g_latched) {
		g_latched = true;
		printf("MOTORDUTY BUDGET EXHAUSTED -- latched off. Reset the board to "
		       "run more steps.\n");
	}
}

static int step_limit_ms(uint16_t duty)
{
	/* Below break-away the motor may be sitting stalled, so the step is
	 * capped hard regardless of what the dwell says. */
	if (duty > 0u && duty < (uint16_t)MOTOR_DUTY_BREAKAWAY) {
		return MOTOR_DUTY_STALL_MS;
	}
	return MOTOR_DUTY_DWELL_MS;
}

static void arm(void)
{
	int limit;

	if (g_latched) {
		printf("MOTORDUTY refused: budget exhausted, reset the board\n");
		return;
	}
	if (g_sel == 0u) {
		printf("MOTORDUTY refused: no motor selected (1/2/3/4 or a)\n");
		return;
	}
	if (g_duty == 0u) {
		printf("MOTORDUTY refused: duty is 0.00 %%\n");
		return;
	}
	limit = step_limit_ms(g_duty);
	g_arm_started = k_uptime_get();
	g_arm_until = g_arm_started + limit;
	g_armed = true;
	g_frames_at_arm = g_frames;
	g_next_hold_print = g_arm_started + 1000;
	send_now();
	printf("MOTORDUTY STEP duty=%u.%02u%% sel=%c%c%c%c hold=%d ms%s\n",
	       (unsigned)(g_duty / 100u), (unsigned)(g_duty % 100u),
	       (g_sel & 1u) ? '1' : '-', (g_sel & 2u) ? '2' : '-',
	       (g_sel & 4u) ? '3' : '-', (g_sel & 8u) ? '4' : '-',
	       limit,
	       (g_duty < (uint16_t)MOTOR_DUTY_BREAKAWAY)
		       ? "  [below break-away: short cap, may not turn]" : "");
}

static void adjust(int delta)
{
	int d = (int)g_duty + delta;

	if (d < 0) { d = 0; }
	if (d > MOTOR_DUTY_MAX) { d = MOTOR_DUTY_MAX; }
	g_duty = (uint16_t)d;
	printf("MOTORDUTY duty=%u.%02u%%%s\n",
	       (unsigned)(g_duty / 100u), (unsigned)(g_duty % 100u),
	       (d == MOTOR_DUTY_MAX) ? "  (at the image ceiling)" : "");
}

static void print_keys(void)
{
	printf("\nKeys: 1 2 3 4 = toggle motor   a = all   n = none\n"
	       "      + - = +/-1.00%%   ] [ = +/-5.00%%   g = GO   x/q/SPACE = STOP\n"
	       "      t = scripted sweep %u.%02u%% -> %u.%02u%% in %u.%02u%% steps   ? = this\n",
	       (unsigned)(MOTOR_DUTY_SWEEP_START / 100), (unsigned)(MOTOR_DUTY_SWEEP_START % 100),
	       (unsigned)(MOTOR_DUTY_SWEEP_STOP / 100), (unsigned)(MOTOR_DUTY_SWEEP_STOP % 100),
	       (unsigned)(MOTOR_DUTY_SWEEP_STEP / 100), (unsigned)(MOTOR_DUTY_SWEEP_STEP % 100));
	printf("Bounds: image ceiling %u.%02u%%, dwell %d ms, sub-break-away cap %d ms,\n"
	       "        budget %d ms/boot. The ESP applies its OWN %s ceiling and cuts all\n"
	       "        four if frames stop for 120 ms.\n\n",
	       (unsigned)(MOTOR_DUTY_MAX / 100), (unsigned)(MOTOR_DUTY_MAX % 100),
	       MOTOR_DUTY_DWELL_MS, MOTOR_DUTY_STALL_MS, MOTOR_DUTY_BUDGET_MS,
	       "ESP_MOTORS_MAX_DUTY_PCT");
}

static void sweep_stop(const char *why)
{
	if (g_sweeping) {
		g_sweeping = false;
		g_sweep_cooling = false;
		printf("MOTORDUTY SWEEP END (%s)\n", why);
	}
}

static void sweep_advance(void)
{
	bool drift = false;

	if (g_sweep_duty > (uint16_t)MOTOR_DUTY_SWEEP_STOP ||
	    g_sweep_duty > (uint16_t)MOTOR_DUTY_MAX) {
		/*
		 * One extra step, and it is the most informative one in the
		 * sweep: the FIRST step again, run last.
		 *
		 * Brushed motors and their low-side FETs heat, and a hot motor
		 * makes less thrust at the same duty than a cold one. A sweep
		 * that only ever goes up cannot tell a real thrust curve from
		 * that drift -- both look like "the top of the curve sags".
		 * Repeating the opening step at the end measures the drift
		 * directly: if its thrust matches the original within the
		 * scale's resolution, the whole curve was taken cold enough to
		 * trust. If it does not, the curve is thermal, not aerodynamic,
		 * and the batch has to be redone with longer cool-downs.
		 */
		if (g_sweep_drift_check) {
			sweep_stop("all steps done");
			return;
		}
		g_sweep_drift_check = true;
		g_sweep_duty = (uint16_t)MOTOR_DUTY_SWEEP_START;
		drift = true;
	}
	g_duty = g_sweep_duty;
	if (drift) {
		printf("MOTORDUTY DRIFT CHECK -- repeating the FIRST step last. Its thrust "
		       "must match the first reading, or the curve is thermal.\n");
	}
	arm();
	if (!g_armed) {
		sweep_stop("a step was refused");
		return;
	}
	g_sweep_next = g_arm_until;
	g_sweep_cooling = false;
	g_sweep_duty = (uint16_t)(g_sweep_duty + MOTOR_DUTY_SWEEP_STEP);
}

int main(void)
{
	int64_t next_frame;

	printf("\n=====================================================\n");
	printf("RiskyBird fixed-duty motor commander (FPGA -> ESP32-C6 duty frames)\n");
	printf("  built : " __DATE__ " " __TIME__ "\n");
	printf("  wire  : %s, 14-byte motor_link frames at %d Hz\n",
	       esp_link->name, MOTOR_DUTY_HZ);
	printf("          (the ESP cuts all four after 120 ms without a valid frame --\n");
	printf("           %d Hz is %d ms per frame, so that is %d.%dx of margin)\n",
	       MOTOR_DUTY_HZ, FRAME_PERIOD_MS, 120 / FRAME_PERIOD_MS,
	       (1200 / FRAME_PERIOD_MS) % 10);
	printf("  gates : driven by the ESP (workloads/esp_motors). This image\n");
	printf("          enables no PWM driver and drives no motor ball.\n");
	printf("=====================================================\n");
	printf("PROPELLERS: this is the one bench test that wants them ON. Restrain the\n");
	printf("airframe, stay out of the prop plane, and keep the battery within reach.\n");
	/* Said out loud because the flight controller's 5 s commitment window is the
	 * first thing an operator will expect here, and expecting a bound that does
	 * not exist is worse than knowing there is none. */
	printf("NOTE: this is NOT the flight controller. There is no commitment gate and\n");
	printf("      no 5 s window in this image -- its bounds are the dwell (%d ms), the\n",
	       MOTOR_DUTY_DWELL_MS);
	printf("      sub-break-away cap (%d ms) and the %d ms per-boot budget, all above.\n",
	       MOTOR_DUTY_STALL_MS, MOTOR_DUTY_BUDGET_MS);
	printf("      Nothing has to be committed to run a step, so nothing forgets to\n");
	printf("      un-commit afterwards.\n");

	if (!device_is_ready(esp_link)) {
		printf("ERROR: esp-uart is not ready -- no frames will be sent, so the\n"
		       "       ESP failsafe keeps every motor off. Nothing can spin.\n");
		return 1;
	}
	if (!device_is_ready(console)) {
		printf("ERROR: console not ready -- refusing to run, because there would\n"
		       "       be no way to press stop.\n");
		return 1;
	}
	print_keys();

	/* One explicit disarmed, all-zero frame before anything else, so the ESP
	 * has a good frame to point at rather than inferring "off" from silence. */
	send_now();
	next_frame = k_uptime_get() + FRAME_PERIOD_MS;

	for (;;) {
		int64_t now = k_uptime_get();
		uint8_t key;

		while (uart_poll_in(console, &key) == 0) {
			switch (key) {
			case '1': case '2': case '3': case '4':
				g_sel ^= (uint8_t)(1u << (key - '1'));
				printf("MOTORDUTY sel=%c%c%c%c\n",
				       (g_sel & 1u) ? '1' : '-', (g_sel & 2u) ? '2' : '-',
				       (g_sel & 4u) ? '3' : '-', (g_sel & 8u) ? '4' : '-');
				break;
			case 'a': case 'A':
				g_sel = 0x0Fu;
				printf("MOTORDUTY sel=1234 (all)\n");
				break;
			case 'n': case 'N':
				g_sel = 0u;
				printf("MOTORDUTY sel=---- (none)\n");
				break;
			case '+': case '=':
				adjust(+100);
				break;
			case '-': case '_':
				adjust(-100);
				break;
			case ']': case '}':
				adjust(+500);
				break;
			case '[': case '{':
				adjust(-500);
				break;
			case 'g': case 'G':
				if (g_sweeping) {
					disarm("sweep step advanced early");
					sweep_advance();
				} else {
					arm();
				}
				break;
			case 't': case 'T':
				if (g_sweeping) {
					sweep_stop("restarted");
				}
				g_sweep_duty = (uint16_t)MOTOR_DUTY_SWEEP_START;
				g_sweep_drift_check = false;
				g_sweeping = true;
				printf("MOTORDUTY SWEEP START -- read the scale on each STEP "
				       "line; g = next step early, x = abort\n");
				sweep_advance();
				break;
			case 'x': case 'X': case 'q': case 'Q': case ' ':
				sweep_stop("operator stop");
				disarm("operator stop");
				break;
			case '?':
				print_keys();
				break;
			default:
				break;
			}
		}

		/* 1 Hz while armed. `frames` is the count since this step armed,
		 * so it doubles as a live check that the link is still being fed:
		 * it must climb by about MOTOR_DUTY_HZ every second. If it does
		 * not, the ESP is about to cut and the reading on the scale is
		 * not the reading you think it is. */
		if (g_armed && now >= g_next_hold_print) {
			g_next_hold_print += 1000;
			printf("MOTORDUTY HOLD duty=%u.%02u%% t=%d/%d ms frames=%u "
			       "(expect ~%d/s)\n",
			       (unsigned)(g_duty / 100u), (unsigned)(g_duty % 100u),
			       (int)(now - g_arm_started),
			       (int)(g_arm_until - g_arm_started),
			       (unsigned)(g_frames - g_frames_at_arm), MOTOR_DUTY_HZ);
		}

		if (g_armed && now >= g_arm_until) {
			disarm("dwell elapsed");
			if (g_sweeping) {
				g_sweep_cooling = true;
				g_sweep_next = now + MOTOR_DUTY_SWEEP_COOL_MS;
				printf("MOTORDUTY COOL %d ms\n", MOTOR_DUTY_SWEEP_COOL_MS);
			}
		}
		if (g_sweeping && g_sweep_cooling && now >= g_sweep_next) {
			sweep_advance();
		}
		/* Cumulative budget, checked while armed so a single very long
		 * step cannot outlive it. */
		if (g_armed && (g_armed_ms + (now - g_arm_started)) >=
				(int64_t)MOTOR_DUTY_BUDGET_MS) {
			sweep_stop("budget exhausted");
			disarm("budget exhausted");
		}

		if (now >= next_frame) {
			next_frame = now + FRAME_PERIOD_MS;
			send_now();
		}
		k_msleep(2);
	}

	return 0;
}
