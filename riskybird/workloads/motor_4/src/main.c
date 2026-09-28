/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Four-motor PWM diagnostic.
 *
 * Two ways in, because on the TE0712 drone carrier the console UART has never
 * emitted a character (docs/drone-initial-bringup.md) and a workload that can
 * only be armed over the console cannot be run on that bench at all:
 *
 *   console  's' arms at the selected percentage, 'x'/' '/'q' stops and
 *            disarms, and 0-100 changes the selected percentage.
 *   debugger writes to motor_arm_request / motor_throttle_pct over JTAG, with
 *            the outcome readable back in motor_state and motor_applied_pct.
 *
 * Uses UART1 when the motor_uart alias is defined, else the console UART.
 *
 * SAFETY. Three independent things have to be true before any output moves:
 * the request word has to carry MOTOR_ARM_MAGIC, so a stray write cannot arm
 * the machine; the throttle is clamped to MOTOR_MAX_PCT; and an armed run
 * expires after MOTOR_ARM_TIMEOUT_MS unless it is renewed. The timeout matters
 * most on the debugger path -- if the GDB session dies while the motors are
 * spinning there is otherwise nothing left to stop them.
 *
 * The outputs are active-high at the pin: WithArty200TPWM inverts the sifive
 * comparator, so pulse 0 really is off. Read that binder's comment before
 * assuming anything about polarity -- on an uninverted build every one of these
 * "stopped" calls would command full throttle instead.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/sys_io.h>
#include <stdlib.h>

static const struct pwm_dt_spec pwm_motor0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor0));
static const struct pwm_dt_spec pwm_motor1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor1));
static const struct pwm_dt_spec pwm_motor2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor2));
static const struct pwm_dt_spec pwm_motor3 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor3));

#if DT_NODE_EXISTS(DT_ALIAS(motor_uart))
#define UART_NODE DT_ALIAS(motor_uart)
#else
#define UART_NODE DT_CHOSEN(zephyr_console)
#endif
static const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);

/* 20 kHz period (matches the overlay), 50 us in nanoseconds. Above audible,
 * and at the 50 MHz peripheral clock it is 2500 counts -- well inside the
 * 16-bit comparator, so the sifive driver picks scale 0 and every one of those
 * 2500 steps is a distinct duty cycle.
 */
#define MOTOR_PERIOD_NSEC (50U * 1000U)

/* Overridable at build time with -DEXTRA_CFLAGS=-DMOTOR_MAX_PCT=<n>. */
#ifndef MOTOR_MAX_PCT
#define MOTOR_MAX_PCT 25U
#endif
#ifndef MOTOR_ARM_TIMEOUT_MS
#define MOTOR_ARM_TIMEOUT_MS 5000
#endif
/* Ceiling on motor_arm_window_ms, so the debugger can lengthen a spin but not
 * remove the ceiling on it. */
#define MOTOR_ARM_WINDOW_MAX_MS 30000U

#define MOTOR_ARM_MAGIC   0xA5A5C0DEu
/* Written back over the request once it has been acted on. Distinct from 0 so
 * "armed and counting down" is not confused with "asked to stop". */
#define MOTOR_ARM_CONSUMED 0xC0FFEE01u
#define MOTOR_STATE_INIT   0x11111111u
#define MOTOR_STATE_READY  0x2EADD1E5u
#define MOTOR_STATE_ARMED  0xA5A5A5A5u
#define MOTOR_STATE_FAILED 0xDEADBEEFu
#define MOTOR_PROBE_RUNNING 0x50B1u
#define MOTOR_PROBE_DONE    0x50B1D0E5u

/*
 * The debugger's half of the interface. Non-static and volatile so they keep
 * symbols and survive optimisation:
 *
 *   (gdb) set var motor_throttle_pct = 10
 *   (gdb) set var motor_arm_request = 0xA5A5C0DE
 *   (gdb) p motor_state
 *   (gdb) p motor_applied_pct
 *   (gdb) set var motor_arm_request = 0          # stop
 */
volatile uint32_t motor_arm_request;
volatile uint32_t motor_throttle_pct = 10;
volatile uint32_t motor_state;
volatile uint32_t motor_applied_pct;
volatile int32_t  motor_last_error;
volatile uint32_t motor_arm_expiries;
/*
 * How long one arm lasts, in milliseconds, settable from the debugger so a
 * visual check can be given longer than the default without a rebuild. Clamped
 * on use: a wild write must not be able to disable the window.
 */
volatile uint32_t motor_arm_window_ms = MOTOR_ARM_TIMEOUT_MS;
volatile uint32_t motor_updates;

/*
 * ON-CHIP WAVEFORM PROBE
 *
 * The problem this solves: every readback in the section above proves that
 * software wrote a register, and none of it proves that anything toggles. On a
 * bench nobody can look at, "the driver returned 0" and "the motor pin is
 * moving" are very different claims, and the whole point of this bring-up was
 * that the second one can be wrong while the first looks perfect.
 *
 * The sifive block exposes two things that close that gap without a scope:
 *
 *   pwms          (+0x10)      the live scaled counter, i.e. the phase
 *   pwmcfg[28+n]  (pwmcmpXip)  the live comparator output for channel n
 *
 * pwmcmpXip is not a copy of what was written -- it is the signal the harness
 * inverts to drive the package pin. Sampling (pwms, pwmcfg) pairs and binning
 * by the measured phase reconstructs one whole period of the real waveform, on
 * real silicon, at the real clock, over JTAG.
 *
 * Binning by the phase that was read, rather than by sample index, is what
 * makes this immune to the CPU's sampling loop locking to a harmonic of the
 * PWM period: an aliased loop visits fewer bins, which shows up as an empty
 * bin in motor_probe_bin_total rather than as a wrong answer.
 *
 * The pin is the inverse of pwmcmpXip, so pin-high is ip-low. That inversion is
 * the one thing here that is inferred rather than measured -- it lives in the
 * FPGA fabric past the last register -- and it is what
 * tests/test_pwm.py::test_harness_inverts_each_comparator_at_the_top_level
 * checks against the generated Verilog.
 */
/*
 * The probe reads the SiFive PWM blocks directly, and those exist only on the
 * FPGA targets. This same workload also builds for the ESP32-C6, which reaches
 * the identical four MOSFET gates through LEDC at completely different
 * addresses -- so on that target the first probe load faults before main()
 * reaches the console loop, and the image is dead on arrival:
 *
 *     mcause: 5, Load access fault
 *      mtval: 10050010          <- PWM0_BASE + PWM_S
 *
 * Measured on an esp32c6 build 2026-09-16, which is why the ESP-side motor
 * test appeared to do nothing at all: no console, no keystrokes, no motors.
 */
#define HAVE_PWM_PROBE  DT_HAS_COMPAT_STATUS_OKAY(sifive_pwm0)

#define PWM0_BASE       0x10050000UL
#define PWM1_BASE       0x10051000UL
#define PWM_CFG         0x00
#define PWM_COUNT       0x08
#define PWM_S           0x10
#define PWM_CMP(n)      (0x20 + 4U * (n))
#define PWM_IP_BIT(n)   (28U + (n))

#define PROBE_BINS      32U
#define PROBE_SAMPLES   8000U

/* Which block and comparator each motor is on. Mirrors motor_4.overlay and
 * WithArty200TPWM; tests/test_pwm.py checks all three against each other. */
static const uint32_t motor_block[4] = { 0, 0, 0, 1 };
static const uint32_t motor_cmp[4]   = { 1, 2, 3, 1 };

volatile uint32_t motor_probe_request;
volatile uint32_t motor_probe_state;
volatile uint32_t motor_probe_runs;
/* Samples per phase bin, per block. A zero bin means that phase was never
 * observed, which invalidates the duty for that block rather than biasing it. */
volatile uint32_t motor_probe_bin_total[2][PROBE_BINS];
/* Samples in that bin where the comparator output was high, per motor. */
volatile uint32_t motor_probe_bin_high[4][PROBE_BINS];
/* The answer: measured duty AT THE PIN, in per mille. */
volatile uint32_t motor_duty_permille[4];
/* How many of the PROBE_BINS phase slices were sampled at all. Fewer than
 * PROBE_BINS means the duty is an average over partial coverage. */
volatile uint32_t motor_probe_bins_used[4];
/* First bin where the comparator output goes high as phase increases; should
 * land at cmp/period, which is where the requested pulse ends. */
volatile uint32_t motor_probe_edge_bin[4];
/*
 * Evidence that the counter is actually running, which no register readback can
 * give you. Sampling pwmcount across a delay does NOT work: pwmzerocmp resets
 * it every period, so the difference between two reads 20 ms apart is a
 * difference of two values in [0, period] and means nothing.
 *
 * These come from a tight burst of phase reads instead. A running counter wraps
 * repeatedly and covers most of the period; a stopped one returns one value
 * forever, giving zero wraps and zero span.
 */
volatile uint32_t motor_probe_wraps[2];
volatile uint32_t motor_probe_phase_span[2];
volatile uint32_t motor_probe_phase_min[2];
volatile uint32_t motor_probe_phase_max[2];
/* Counts the phase advances between two back-to-back pwms reads, so the bin
 * assignment error is bounded and explicit rather than assumed to be zero. */
volatile uint32_t motor_probe_read_lag;
/* Raw register readback, for when the reconstruction disagrees with itself. */
volatile uint32_t motor_probe_cfg[2];
volatile uint32_t motor_probe_period[2];
volatile uint32_t motor_probe_cmp_readback[4];

/*
 * The breakpoint the batch readback stops on. motor_4 runs forever, so a script
 * needs a deterministic place to catch it with the results settled; this is it.
 * It carries the two state updates so there is no empty body for --gc-sections
 * to remove. See hardware/pwm/probe.gdb.
 */
__attribute__((noinline)) void motor_probe_done(void)
{
	motor_probe_runs++;
	motor_probe_state = MOTOR_PROBE_DONE;
}

static inline uint32_t pwm_base(uint32_t block)
{
	return (uint32_t)(block ? PWM1_BASE : PWM0_BASE);
}

static inline uint32_t pwm_rd(uint32_t block, uint32_t offset)
{
	return sys_read32((mem_addr_t)(pwm_base(block) + offset));
}

#define LIVENESS_SAMPLES 2000U

/*
 * Is the counter running at all? A block that is mapped but never enabled, or
 * whose clock never arrives, reads back every register perfectly and does not
 * count. Take a burst of phase samples and record how far they range and how
 * often they go backwards; only a running counter does either.
 */
static void probe_liveness(void)
{
	if (!HAVE_PWM_PROBE) {
		return;
	}

	for (uint32_t block = 0; block < 2; block++) {
		uint32_t previous = pwm_rd(block, PWM_S);
		uint32_t low = previous, high = previous, wraps = 0;

		for (uint32_t i = 1; i < LIVENESS_SAMPLES; i++) {
			uint32_t phase = pwm_rd(block, PWM_S);

			if (phase < previous) {
				wraps++;
			}
			if (phase < low) {
				low = phase;
			}
			if (phase > high) {
				high = phase;
			}
			previous = phase;
		}
		motor_probe_wraps[block] = wraps;
		motor_probe_phase_min[block] = low;
		motor_probe_phase_max[block] = high;
		motor_probe_phase_span[block] = high - low;
	}
}

static void probe_waveform(void)
{
	uint32_t period[2];

	if (!HAVE_PWM_PROBE) {
		/* Sets state to DONE and bumps the run counter, so a script
		 * breaking on it still gets a deterministic stop. */
		motor_probe_done();
		return;
	}

	motor_probe_state = MOTOR_PROBE_RUNNING;

	for (uint32_t block = 0; block < 2; block++) {
		motor_probe_cfg[block] = pwm_rd(block, PWM_CFG);
		period[block] = pwm_rd(block, PWM_CMP(0));
		motor_probe_period[block] = period[block];
		for (uint32_t bin = 0; bin < PROBE_BINS; bin++) {
			motor_probe_bin_total[block][bin] = 0;
		}
	}
	for (uint32_t motor = 0; motor < 4; motor++) {
		motor_probe_cmp_readback[motor] =
			pwm_rd(motor_block[motor], PWM_CMP(motor_cmp[motor]));
		for (uint32_t bin = 0; bin < PROBE_BINS; bin++) {
			motor_probe_bin_high[motor][bin] = 0;
		}
	}

	uint32_t lag_sum = 0, lag_samples = 0;
	uint32_t rng = 0x12345u;

	for (uint32_t sample = 0; sample < PROBE_SAMPLES; sample++) {
		/*
		 * Jitter the interval between samples.
		 *
		 * A fixed-length loop body lands on a near-integer fraction of
		 * the PWM period and then visits only those few phases forever.
		 * Measured on hardware without this, consecutive bins held 615
		 * and 1 samples, and three bins were never visited at all. The
		 * per-bin normalisation below survives that; coverage is still
		 * worth having, because a bin nobody visits carries no
		 * information about the waveform there.
		 */
		rng = rng * 1103515245u + 12345u;
		for (volatile uint32_t spin = (rng >> 16) & 0x3FU; spin > 0U; spin--) {
		}

		for (uint32_t block = 0; block < 2; block++) {
			if (period[block] == 0U) {
				continue;
			}
			/*
			 * Straddle the comparator read with two phase reads and
			 * bin at the midpoint.
			 *
			 * An MMIO load over the peripheral bus is tens of
			 * cycles, and the counter does not stop for it. Reading
			 * the phase once and then the comparator attributes an
			 * ip sample to a phase it has already left, which biases
			 * the measured duty low by the read latency -- at 2500
			 * counts per period that is a percent or two of duty,
			 * systematically, in the same direction every time. The
			 * midpoint cancels it to first order and the spread is
			 * reported as motor_probe_read_lag.
			 */
			uint32_t before = pwm_rd(block, PWM_S);
			uint32_t cfg = pwm_rd(block, PWM_CFG);
			uint32_t after = pwm_rd(block, PWM_S);

			/*
			 * Unwrap rather than discard when the counter rolls
			 * over mid-sample.
			 *
			 * Discarding those samples looks harmless and is not:
			 * the midpoint sits half a read-window ahead of
			 * `before`, so the phases just after zero can only be
			 * reached from `before` values just before the wrap --
			 * exactly the samples a drop throws away. Bin 0 then
			 * collects half the samples of every other bin, and
			 * bin 0 is where the pin is high at low duty, so the
			 * measured duty comes out low by more the smaller it
			 * is. Simulation put 10% at 86 per mille that way.
			 */
			uint32_t span = (after >= before)
				? (after - before)
				: (after + period[block] - before);
			uint32_t phase = (before + span / 2U) % period[block];

			lag_sum += span;
			lag_samples++;

			uint32_t bin = (phase * PROBE_BINS) / period[block];

			if (bin >= PROBE_BINS) {
				bin = PROBE_BINS - 1U;
			}
			motor_probe_bin_total[block][bin]++;
			for (uint32_t motor = 0; motor < 4; motor++) {
				if (motor_block[motor] != block) {
					continue;
				}
				if (cfg & (1U << PWM_IP_BIT(motor_cmp[motor]))) {
					motor_probe_bin_high[motor][bin]++;
				}
			}
		}
	}

	motor_probe_read_lag = lag_samples ? lag_sum / lag_samples : 0U;

	for (uint32_t motor = 0; motor < 4; motor++) {
		uint32_t block = motor_block[motor];
		uint32_t edge = PROBE_BINS, used = 0, permille_sum = 0;

		for (uint32_t bin = 0; bin < PROBE_BINS; bin++) {
			uint32_t total = motor_probe_bin_total[block][bin];
			uint32_t high = motor_probe_bin_high[motor][bin];

			if (total == 0U) {
				continue;
			}
			/*
			 * Average the per-bin duty rather than pooling every
			 * sample.
			 *
			 * Each bin covers an equal slice of the period, so an
			 * equally weighted mean over bins estimates the duty
			 * even when the bins hold wildly different numbers of
			 * samples. Pooling does not: the first hardware run
			 * aliased into a 615/1/614/1 pattern and
			 * reported 153 per mille for a 100 per mille request,
			 * purely because the bins where the pin was high had
			 * been sampled more often than the ones where it was
			 * not.
			 */
			permille_sum += (1000U * (total - high)) / total;
			used++;
			/* The pin is the inverse of the comparator output. */
			if (edge == PROBE_BINS && high * 2U > total) {
				edge = bin;
			}
		}
		motor_probe_edge_bin[motor] = edge;
		motor_probe_bins_used[motor] = used;
		motor_duty_permille[motor] = used ? permille_sum / used : 0U;
	}

	motor_probe_done();
}

#define THROTTLE_BUF_LEN 4
static uint8_t selected_pct = 10; /* 0-100, default 10% */
static bool armed;
static int64_t arm_deadline;
static char throttle_buf[THROTTLE_BUF_LEN];
static int throttle_buf_len;

/*
 * A stable symbol to break on when the outputs have just been commanded.
 *
 * It has to do something: an empty body leaves nothing at the call site, and
 * --gc-sections then drops the function itself, so "break motor_outputs_changed"
 * fails on an image that otherwise looks correct. The counter is also the
 * cheapest way to tell a wedged main loop from a quiet one.
 */
__attribute__((noinline)) void motor_outputs_changed(void)
{
	motor_updates++;
}

static uint8_t clamp_pct(uint32_t pct)
{
	if (pct > MOTOR_MAX_PCT) {
		return (uint8_t)MOTOR_MAX_PCT;
	}
	return (uint8_t)pct;
}

static int set_outputs(uint8_t pct)
{
	uint32_t pulse = (MOTOR_PERIOD_NSEC * (uint32_t)pct) / 100U;
	int ret;

	ret = pwm_set_dt(&pwm_motor0, MOTOR_PERIOD_NSEC, pulse);
	if (ret == 0) ret = pwm_set_dt(&pwm_motor1, MOTOR_PERIOD_NSEC, pulse);
	if (ret == 0) ret = pwm_set_dt(&pwm_motor2, MOTOR_PERIOD_NSEC, pulse);
	if (ret == 0) ret = pwm_set_dt(&pwm_motor3, MOTOR_PERIOD_NSEC, pulse);

	motor_last_error = ret;
	if (ret == 0) {
		motor_applied_pct = pct;
	}
	motor_outputs_changed();
	return ret;
}

static void arm_at(uint8_t pct)
{
	if (set_outputs(pct) != 0) {
		return;
	}
	uint32_t window = motor_arm_window_ms;

	if (window == 0U || window > MOTOR_ARM_WINDOW_MAX_MS) {
		window = MOTOR_ARM_TIMEOUT_MS;
	}
	armed = true;
	arm_deadline = k_uptime_get() + (int64_t)window;
	motor_state = MOTOR_STATE_ARMED;
}

static void disarm(void)
{
	(void)set_outputs(0);
	armed = false;
	motor_arm_request = 0;
	motor_state = MOTOR_STATE_READY;
}

static void apply_throttle_number(void)
{
	if (throttle_buf_len == 0) return;
	throttle_buf[throttle_buf_len] = '\0';
	selected_pct = clamp_pct((uint32_t)strtoul(throttle_buf, NULL, 10));
	motor_throttle_pct = selected_pct;
	if (armed) {
		arm_at(selected_pct);
		printk("Throttle %u%% (armed)\n", (unsigned int)selected_pct);
	} else {
		printk("Selected %u%% (outputs remain stopped)\n",
		       (unsigned int)selected_pct);
	}
	throttle_buf_len = 0;
}

static void process_uart_byte(uint8_t byte)
{
	if (byte >= '0' && byte <= '9') {
		if (throttle_buf_len < THROTTLE_BUF_LEN - 1) {
			throttle_buf[throttle_buf_len++] = (char)byte;
		}
		return;
	}
	/* Non-digit: apply any pending number then handle command */
	apply_throttle_number();

	switch (byte) {
	case 's':
	case 'S':
		arm_at(selected_pct);
		if (armed) {
			printk("Motors ARMED (%u%%)\n", selected_pct);
		}
		break;
	case 'x':
	case 'X':
	case ' ':
	case 'q':
	case 'Q':
		disarm();
		printk("Motors STOPPED and DISARMED\n");
		break;
	case '\r':
	case '\n':
		/* already applied in apply_throttle_number */
		break;
	default:
		break;
	}
}

/*
 * The debugger path. Arming needs the magic word, so the motors cannot start
 * because something scribbled on this address; renewing the same request keeps
 * the deadline moving, which is what makes a lost GDB session safe.
 */
static void poll_debug_request(void)
{
	uint32_t request = motor_arm_request;

	if (request == MOTOR_ARM_MAGIC) {
		uint8_t want = clamp_pct(motor_throttle_pct);

		/*
		 * Consume the request rather than leaving it standing.
		 *
		 * The earlier version treated a request that was still present
		 * as a renewal and pushed the deadline out again. That reads
		 * like a watchdog and is the opposite of one: the request word
		 * is plain memory and outlives whatever wrote it, so a debugger
		 * that detached -- or died -- left the magic sitting there and
		 * the window renewed itself every 10 ms forever. Measured on
		 * the board: armed at 10%, still armed 12 seconds later, zero
		 * expiries. Consuming it means a renewal has to be an actual
		 * write, which is the property the window was supposed to have.
		 */
		motor_arm_request = MOTOR_ARM_CONSUMED;
		arm_at(want);
		printk("Motors ARMED via debugger (%u%%)\n", want);
		return;
	}
	if (request == MOTOR_ARM_CONSUMED) {
		return;  /* armed, counting down; write the magic again to renew */
	}
	if (armed) {
		disarm();
		printk("Motors STOPPED via debugger\n");
	}
}

int main(void)
{
	uint8_t byte;

	motor_state = MOTOR_STATE_INIT;

	printk("Four-motor diagnostic: outputs start STOPPED and DISARMED\n");
	printk("Propellers off. Commands: 0-100=setpoint, s=arm, "
	       "x/space/q=stop\n");
	printk("Debugger: set motor_throttle_pct, then motor_arm_request=0x%X\n",
	       MOTOR_ARM_MAGIC);
	printk("Debugger: set motor_probe_request=1 to remeasure the live pins\n");

	if (!pwm_is_ready_dt(&pwm_motor0) || !pwm_is_ready_dt(&pwm_motor1) ||
	    !pwm_is_ready_dt(&pwm_motor2) || !pwm_is_ready_dt(&pwm_motor3)) {
		printk("Error: One or more PWM devices not ready\n");
		motor_state = MOTOR_STATE_FAILED;
		return 0;
	}

	/*
	 * A dead console is not a reason to refuse to run: on this carrier it is
	 * the expected case, and the debugger path does not need the UART.
	 */
	if (!device_is_ready(uart_dev)) {
		printk("Warning: UART not ready; debugger path only\n");
	}

	printk("Motors on channels %d,%d,%d,%d, period %u nsec, max %u%%\n",
	       pwm_motor0.channel, pwm_motor1.channel,
	       pwm_motor2.channel, pwm_motor3.channel,
	       MOTOR_PERIOD_NSEC, MOTOR_MAX_PCT);

	/* Start with motors stopped (0% throttle) */
	(void)set_outputs(0);
	motor_state = MOTOR_STATE_READY;

	/*
	 * Measure the stopped state before anything is armed. This is the
	 * safety-critical case and the one an uninverted build gets wrong: it
	 * should come out as four duties of 0, and if it comes out as 1000 the
	 * outputs are inverted the wrong way and nothing should be armed.
	 */
	probe_liveness();
	probe_waveform();
	printk("probe: phase span %u/%u, wraps %u/%u, duty %u/%u/%u/%u per mille\n",
	       (unsigned int)motor_probe_phase_span[0],
	       (unsigned int)motor_probe_phase_span[1],
	       (unsigned int)motor_probe_wraps[0],
	       (unsigned int)motor_probe_wraps[1],
	       (unsigned int)motor_duty_permille[0],
	       (unsigned int)motor_duty_permille[1],
	       (unsigned int)motor_duty_permille[2],
	       (unsigned int)motor_duty_permille[3]);

	while (1) {
		if (device_is_ready(uart_dev) &&
		    uart_poll_in(uart_dev, &byte) == 0) {
			process_uart_byte(byte);
		}
		poll_debug_request();

		if (motor_probe_request) {
			motor_probe_request = 0;
			probe_liveness();
			probe_waveform();
			printk("probe: duty %u/%u/%u/%u per mille\n",
			       (unsigned int)motor_duty_permille[0],
			       (unsigned int)motor_duty_permille[1],
			       (unsigned int)motor_duty_permille[2],
			       (unsigned int)motor_duty_permille[3]);
		}

		if (armed && k_uptime_get() > arm_deadline) {
			motor_arm_expiries++;
			disarm();
			printk("Motors STOPPED: arm window expired\n");
		}
		k_msleep(10);
	}
	return 0;
}
