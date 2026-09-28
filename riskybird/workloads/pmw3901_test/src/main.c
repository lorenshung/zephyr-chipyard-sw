/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * PMW3901 optical-flow diagnostic AND flow-calibration capture tool.
 *
 * WHAT THIS IS FOR
 * ----------------
 * FLOW_RAD_PER_COUNT (flow.c) converts a PMW3901 motion count into an angle of
 * ground-feature motion. It has never been measured on this airframe; the
 * current 0.00205 is the Crazyflie flow-deck number carried across because the
 * register init is the same. Attitude survives a wrong value because the gyro
 * compensation in main.cpp cancels the SIGN either way, but horizontal VELOCITY
 * is then wrong-scaled and position drifts -- over a 30 s hover that is what
 * ends the flight.
 *
 * The calibration is a DISPLACEMENT measurement, and the arithmetic is why:
 *
 *     ang = -dx * K / dt            (flow.c, per sample)
 *     v   =  ang * h                (main.cpp, per control tick)
 *   =>  v = -dx * K * h / dt
 *   =>  D = integral(v dt) = K * h * SUM(dx)       for a constant height h
 *   =>  K = D / (h * SUM(dx))
 *
 * dt CANCELS. So the constant can be recovered from a slow, hand-made slide
 * across a bench, with no motion rig, no timing accuracy and no sensitivity to
 * how fast the sensor is polled -- as long as every count between the start and
 * the end of the slide is summed. That is all this tool does: sum counts, and
 * report the surface quality that says whether the sum is trustworthy.
 *
 * SUMMING IS SAFE ACROSS POLL RATES. The motion burst returns the deltas
 * accumulated since the last read and clears them, so a slower poll returns
 * BIGGER deltas, not fewer counts. The only way to lose counts is to saturate
 * int16 within one interval, which cannot happen at hand-slide speeds; the
 * summary reports the largest per-sample |dx|/|dy| so that is checked rather
 * than assumed.
 *
 * MODES
 * -----
 *   IDLE      the original diagnostic: one line a second with deltaX, deltaY,
 *             SQUAL and shutter, so the sensor can be aimed and the surface
 *             judged before a run starts. Counts seen while idle are still
 *             summed and shown, so nothing is silently discarded.
 *   CAPTURE   sums every sample until stopped, then prints one greppable
 *             FLOWCAL RUN line. Per-sample printing is OFF during a run by
 *             default: printk busy-waits on this carrier (the FPGA console is
 *             polled, CONFIG_UART_INTERRUPT_DRIVEN is off), and a line per
 *             sample at 100 Hz would steal most of the read loop.
 *
 * CONSOLE KEYS
 * ------------
 *   r / SPACE   start a capture (resets the accumulator)
 *   s / ENTER   stop the capture and print the summary
 *   z           zero the accumulator without leaving the current mode
 *   v           toggle per-sample lines (for a short look; not for a run)
 *   ?           reprint the key list and the current configuration
 *
 * Nothing here drives a motor, a PWM channel or any output but the console and
 * the flow sensor's chip select, so it is safe with the battery disconnected
 * and safe with propellers fitted.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#include "pmw3901.h"
#include <zephyr/devicetree.h>

#define PMW3901_WIRING_NODE DT_NODELABEL(riskybird_pmw3901)

BUILD_ASSERT(DT_NODE_HAS_COMPAT(PMW3901_WIRING_NODE, riskybird_pmw3901),
	     "target overlay must define riskybird_pmw3901 wiring");

/*
 * Read period. 10 ms matches FLOW_PERIOD_MS in flow.c, so the instrument polls
 * the sensor exactly as the flight build does. It does NOT have to, for the
 * reason in the header comment -- the sum is rate-independent -- but keeping it
 * the same removes one difference between the number measured here and the
 * number used in flight.
 */
#ifndef FLOW_CAL_PERIOD_MS
#define FLOW_CAL_PERIOD_MS 10
#endif

/*
 * Surface-quality floor, matching FLOW_MIN_SQUAL in flow.c. Samples below it
 * are the ones the FLIGHT build throws away, so the summary reports the sum
 * both ways: over every sample, and over only the samples flight would have
 * kept. If those two disagree, the surface is not good enough to calibrate on.
 */
#ifndef FLOW_CAL_MIN_SQUAL
#define FLOW_CAL_MIN_SQUAL 19
#endif

/* IDLE heartbeat period. */
#ifndef FLOW_CAL_IDLE_MS
#define FLOW_CAL_IDLE_MS 1000
#endif

static const struct gpio_dt_spec pmw3901_cs =
	GPIO_DT_SPEC_GET(PMW3901_WIRING_NODE, cs_gpios);
static const struct gpio_dt_spec pmw3901_reset =
	GPIO_DT_SPEC_GET(PMW3901_WIRING_NODE, reset_gpios);
static const struct gpio_dt_spec pmw3901_led =
	GPIO_DT_SPEC_GET(PMW3901_WIRING_NODE, led_gpios);

static struct pmw3901_config pmw3901_cfg;
static struct pmw3901_data pmw3901_data;

static const struct device pmw3901_device = {
	.name = "PMW3901",
	.config = &pmw3901_cfg,
	.data = &pmw3901_data,
};

#define PMW3901_DEV (&pmw3901_device)

static const struct device *const console =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

/* ---- accumulator --------------------------------------------------------- */

struct acc {
	uint32_t n;            /* samples read without error */
	uint32_t err;          /* read errors */
	int32_t  sum_dx;       /* signed count sums -- THE measurement */
	int32_t  sum_dy;
	uint32_t abs_dx;       /* path length, to tell a clean slide from a shuffle */
	uint32_t abs_dy;
	int32_t  peak_dx;      /* largest per-sample magnitude: rules out int16 saturation */
	int32_t  peak_dy;
	uint32_t n_ok;         /* samples at or above the SQUAL floor */
	int32_t  ok_dx;        /* the same sums, restricted to those samples */
	int32_t  ok_dy;
	uint32_t sq_min, sq_max;
	uint64_t sq_sum;
	uint32_t sh_min, sh_max;
	uint64_t sh_sum;
	int64_t  t0_ms;
};

static struct acc run;
static struct acc idle;
static uint32_t run_id;
static bool capturing;
static bool verbose;

static void acc_reset(struct acc *a)
{
	static const struct acc empty;

	*a = empty;
	a->sq_min = 0xFFFFFFFFu;
	a->sh_min = 0xFFFFFFFFu;
	a->t0_ms = k_uptime_get();
}

static void acc_add(struct acc *a, const motionBurst_t *m)
{
	int32_t dx = m->deltaX;
	int32_t dy = m->deltaY;
	int32_t ax = (dx < 0) ? -dx : dx;
	int32_t ay = (dy < 0) ? -dy : dy;

	a->n++;
	a->sum_dx += dx;
	a->sum_dy += dy;
	a->abs_dx += (uint32_t)ax;
	a->abs_dy += (uint32_t)ay;
	if (ax > a->peak_dx) { a->peak_dx = ax; }
	if (ay > a->peak_dy) { a->peak_dy = ay; }

	if (m->squal < a->sq_min) { a->sq_min = m->squal; }
	if (m->squal > a->sq_max) { a->sq_max = m->squal; }
	a->sq_sum += m->squal;
	if (m->shutter < a->sh_min) { a->sh_min = m->shutter; }
	if (m->shutter > a->sh_max) { a->sh_max = m->shutter; }
	a->sh_sum += m->shutter;

	if (m->squal >= FLOW_CAL_MIN_SQUAL) {
		a->n_ok++;
		a->ok_dx += dx;
		a->ok_dy += dy;
	}
}

/*
 * One line per run, carrying every field the arithmetic needs. Deliberately
 * flat key=value text: tools/flow-cal.py parses exactly this, so a capture can
 * be pasted out of a console log into the solver with no transcription.
 *
 * Integers only. This console build has no floating-point printf, and a
 * silently-dropped %f in a measurement record is worse than no record at all.
 */
static void acc_report(const struct acc *a, const char *tag, uint32_t id)
{
	int64_t ms = k_uptime_get() - a->t0_ms;
	uint32_t sq_mean = a->n ? (uint32_t)(a->sq_sum / a->n) : 0u;
	uint32_t sh_mean = a->n ? (uint32_t)(a->sh_sum / a->n) : 0u;

	printf("FLOWCAL %s id=%u ms=%d n=%u err=%u "
	       "sumdx=%d sumdy=%d absdx=%u absdy=%u peakdx=%d peakdy=%d "
	       "n_ok=%u okdx=%d okdy=%d "
	       "sq_min=%u sq_max=%u sq_mean=%u sh_min=%u sh_max=%u sh_mean=%u\n",
	       tag, (unsigned)id, (int)ms, (unsigned)a->n, (unsigned)a->err,
	       (int)a->sum_dx, (int)a->sum_dy,
	       (unsigned)a->abs_dx, (unsigned)a->abs_dy,
	       (int)a->peak_dx, (int)a->peak_dy,
	       (unsigned)a->n_ok, (int)a->ok_dx, (int)a->ok_dy,
	       (unsigned)(a->n ? a->sq_min : 0u), (unsigned)a->sq_max, (unsigned)sq_mean,
	       (unsigned)(a->n ? a->sh_min : 0u), (unsigned)a->sh_max, (unsigned)sh_mean);

	if (a->n == 0u) {
		printf("     NO SAMPLES -- nothing to report.\n");
		return;
	}
	if (a->n_ok != a->n) {
		printf("     %u of %u samples were BELOW the SQUAL floor of %d and would "
		       "have been discarded in flight. Use okdx/okdy, or redo the run on "
		       "a better surface.\n",
		       (unsigned)(a->n - a->n_ok), (unsigned)a->n, FLOW_CAL_MIN_SQUAL);
	}
	if (a->peak_dx > 8000 || a->peak_dy > 8000) {
		printf("     PEAK per-sample count is near the int16 range. Slide slower "
		       "or poll faster; a saturated interval loses counts silently.\n");
	}
	if (a->err != 0u) {
		printf("     %u read error(s) during the run -- counts were lost. "
		       "REJECT this run.\n", (unsigned)a->err);
	}
}

static void print_keys(void)
{
	printf("\nKeys: r/SPACE=start capture  s/ENTER=stop+report  z=zero  "
	       "v=per-sample lines  ?=this\n");
	printf("Config: read period %d ms, SQUAL floor %d (matches flow.c), "
	       "per-sample lines %s\n\n",
	       FLOW_CAL_PERIOD_MS, FLOW_CAL_MIN_SQUAL, verbose ? "ON" : "off");
}

static int pmw3901_init_config(void)
{
	pmw3901_cfg.spi.bus =
		DEVICE_DT_GET(DT_PHANDLE(PMW3901_WIRING_NODE, spi_bus));
	/* SPI mode 3 (CPOL=1, CPHA=1) at 2 MHz -- identical to flow.c's flight setup. */
	pmw3901_cfg.spi.config.operation =
		SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL |
		SPI_MODE_CPHA | SPI_TRANSFER_MSB;
	pmw3901_cfg.spi.config.frequency = 2000000;
	pmw3901_cfg.spi.config.slave = 0;

	pmw3901_cfg.cs_gpio = pmw3901_cs;
	pmw3901_cfg.reset_gpio = pmw3901_reset;
	pmw3901_cfg.led_gpio = pmw3901_led;

	return 0;
}

int main(void)
{
	motionBurst_t motion = { 0 };
	int64_t next_idle;
	int ret;

	printf("\n=====================================================\n");
	printf("RiskyBird PMW3901 diagnostic + FLOW_RAD_PER_COUNT capture\n");
	printf("  built : " __DATE__ " " __TIME__ "\n");
	printf("=====================================================\n\n");

	(void)pmw3901_init_config();
	if (!device_is_ready(pmw3901_cfg.spi.bus)) {
		printf("ERROR: SPI device is not ready\n");
		return 1;
	}
	if (!device_is_ready(pmw3901_cfg.cs_gpio.port)) {
		printf("ERROR: GPIO device is not ready\n");
		return 1;
	}
	if (!device_is_ready(console)) {
		printf("ERROR: console device is not ready (no keys could be read)\n");
		return 1;
	}
	printf("SPI and GPIO devices ready\n");

	printf("Initializing PMW3901...\n");
	ret = pmw3901_init(PMW3901_DEV);
	if (ret != 0) {
		printf("ERROR: PMW3901 did not initialise (ret=%d). Check the CS/SPI\n"
		       "       wiring; the chip-ID pair must read 0x49 and 0xB6.\n", ret);
		return 1;
	}
	printf("PMW3901 initialised.\n");
	print_keys();

	acc_reset(&idle);
	acc_reset(&run);
	next_idle = k_uptime_get() + FLOW_CAL_IDLE_MS;

	for (;;) {
		struct acc *a = capturing ? &run : &idle;
		uint8_t key;

		ret = pmw3901_read_motion_burst(PMW3901_DEV, &motion);
		if (ret != 0) {
			a->err++;
		} else {
			acc_add(a, &motion);
			if (verbose) {
				printf("  dx=%5d dy=%5d SQUAL=%3u shutter=%5u%s\n",
				       motion.deltaX, motion.deltaY,
				       (unsigned)motion.squal, (unsigned)motion.shutter,
				       motion.motionOccured ? " [MOTION]" : "");
			}
		}

		if (!capturing && k_uptime_get() >= next_idle) {
			next_idle = k_uptime_get() + FLOW_CAL_IDLE_MS;
			printf("IDLE dx=%+5d dy=%+5d SQUAL=%3u shutter=%5u | "
			       "since zero: sumdx=%d sumdy=%d over %u samples\n",
			       motion.deltaX, motion.deltaY,
			       (unsigned)motion.squal, (unsigned)motion.shutter,
			       (int)idle.sum_dx, (int)idle.sum_dy, (unsigned)idle.n);
			if (motion.squal < FLOW_CAL_MIN_SQUAL) {
				printf("     SQUAL %u is BELOW the flight floor of %d -- flight "
				       "would discard this sample. Add texture, or change the "
				       "height, before capturing.\n",
				       (unsigned)motion.squal, FLOW_CAL_MIN_SQUAL);
			}
		}

		while (uart_poll_in(console, &key) == 0) {
			switch (key) {
			case 'r':
			case 'R':
			case ' ':
				run_id++;
				acc_reset(&run);
				capturing = true;
				printf("\nFLOWCAL START id=%u -- move NOW; press s to stop\n",
				       (unsigned)run_id);
				break;
			case 's':
			case 'S':
			case '\r':
			case '\n':
				if (capturing) {
					capturing = false;
					acc_report(&run, "RUN", run_id);
					acc_reset(&idle);
					next_idle = k_uptime_get() + FLOW_CAL_IDLE_MS;
				} else {
					acc_report(&idle, "IDLE-TOTAL", 0u);
				}
				break;
			case 'z':
			case 'Z':
				acc_reset(a);
				printf("FLOWCAL ZERO (%s)\n", capturing ? "capture" : "idle");
				break;
			case 'v':
			case 'V':
				verbose = !verbose;
				printf("per-sample lines %s\n", verbose ? "ON" : "off");
				break;
			case '?':
				print_keys();
				break;
			default:
				break;
			}
		}

		k_msleep(FLOW_CAL_PERIOD_MS);
	}

	return 0;
}
