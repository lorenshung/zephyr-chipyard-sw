/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Passive owner for the ESP32-C6 side of RiskyBird's shared motor nets.
 */

#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define MOTOR_SAFE_HOG DT_NODELABEL(riskybird_motor_safe_hog)

#define MOTOR_SAFE_SPEC(index)                                            \
	{                                                                  \
		.port = DEVICE_DT_GET(DT_PARENT(MOTOR_SAFE_HOG)),            \
		.pin = DT_GPIO_HOG_PIN_BY_IDX(MOTOR_SAFE_HOG, index),        \
		.dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(MOTOR_SAFE_HOG, index), \
	}

static const struct gpio_dt_spec motor_pins[] = {
	MOTOR_SAFE_SPEC(0),
	MOTOR_SAFE_SPEC(1),
	MOTOR_SAFE_SPEC(2),
	MOTOR_SAFE_SPEC(3),
};

static unsigned int park_esp_motor_pins(bool verbose)
{
	unsigned int safe = 0U;

	for (size_t i = 0; i < ARRAY_SIZE(motor_pins); i++) {
		int err = gpio_pin_configure_dt(&motor_pins[i], GPIO_INPUT);

		if (verbose || err != 0) {
			printk("motor%u ESP GPIO%u input+pulldown: %s (%d)\n",
			       (unsigned)i + 1U, (unsigned)motor_pins[i].pin,
			       err == 0 ? "OK" : "FAIL", err);
		}
		if (err == 0) {
			safe++;
		}
	}
	return safe;
}

/*
 * Read the gate nets back, using the parked pins as logic probes.
 *
 * Each MOTOR gate is driven by the ESP through a 47R AND by the FPGA through a
 * second 47R, with a 10k pulldown holding the SI2302 off until something drives
 * it. While this image holds the ESP side high-impedance, that same pin sits on
 * the gate and can be read -- which measures what the FPGA is actually
 * DELIVERING. No FPGA-side register read can establish that: the comparator can
 * be perfectly programmed while nothing reaches the ball.
 *
 * Sampled rather than read once, because the FPGA drives 20 kHz PWM and a
 * single read lands at an arbitrary phase. Counting highs over many samples
 * recovers the duty as it appears at the gate, so an FPGA channel driving 15%
 * should read ~150 per mille here. A channel reading 0 while its comparator
 * says otherwise is an open between the FPGA ball and the gate.
 */
#define GATE_SAMPLES 2000U

static void report_gate_levels(void)
{
	unsigned int high[ARRAY_SIZE(motor_pins)] = { 0 };

	for (unsigned int s = 0U; s < GATE_SAMPLES; s++) {
		for (size_t i = 0; i < ARRAY_SIZE(motor_pins); i++) {
			if (gpio_pin_get_dt(&motor_pins[i]) == 1) {
				high[i]++;
			}
		}
	}

	printk("MOTOR GATE duty per mille: M1=%u M2=%u M3=%u M4=%u\n",
	       high[0] * 1000U / GATE_SAMPLES, high[1] * 1000U / GATE_SAMPLES,
	       high[2] * 1000U / GATE_SAMPLES, high[3] * 1000U / GATE_SAMPLES);
}

/*
 * Test the instrument before trusting it.
 *
 * report_gate_levels() reads these four pins to decide what the FPGA is
 * delivering to each motor gate, and a whole diagnosis was built on one of its
 * readings: M4 reporting 0 while M1-M3 reported ~146 per mille. That is only
 * evidence if GPIO22's INPUT path works. Its OUTPUT path is proven -- driving it
 * spins motor 4 -- and GPIO21/20/23 clearly read correctly, but a stuck-low
 * input buffer on GPIO22 alone would produce exactly the observed result while
 * meaning nothing at all about the FPGA.
 *
 * So drive each pin high and read it back through its own input buffer. A pin
 * that reads 1 while driving 1 has a working input. A pin that reads 0 is lying,
 * and every gate measurement from it must be discarded.
 *
 * Driven for 2 ms, which is far too short to turn a brushed motor through any
 * meaningful angle, and each pin is returned to parked before the next is
 * touched -- never more than one gate live at a time.
 */
static void probe_self_test(void)
{
	printk("PROBE SELFTEST: verifying each pin can read back its own drive\n");

	for (size_t i = 0; i < ARRAY_SIZE(motor_pins); i++) {
		int rc = gpio_pin_configure_dt(&motor_pins[i],
					       GPIO_OUTPUT_HIGH | GPIO_INPUT);
		int readback = -1;

		if (rc == 0) {
			k_busy_wait(2000);   /* 2 ms: settle the node, barely move the rotor */
			readback = gpio_pin_get_dt(&motor_pins[i]);
		}

		/* Park it again before touching the next one. */
		(void)gpio_pin_configure_dt(&motor_pins[i], GPIO_INPUT);

		printk("PROBE SELFTEST M%u (GPIO%u): configure=%d readback=%d -- %s\n",
		       (unsigned)i + 1U, (unsigned)motor_pins[i].pin, rc, readback,
		       (rc != 0)        ? "CANNOT drive+read this pin; test inconclusive"
		       : (readback == 1) ? "input path OK, gate readings trustworthy"
					 : "READS LOW WHILE DRIVEN HIGH -- this pin's gate readings are WORTHLESS");
		k_msleep(250);
	}
}

int main(void)
{
	unsigned int safe;
	unsigned int tick = 0U;

	(void)park_esp_motor_pins(false);   /* park first, then self-test one pin at a time */
	probe_self_test();

	safe = park_esp_motor_pins(true);

	printk("MOTOR SAFE: %u/4 ESP drivers high-impedance with pull-downs\n", safe);
	if (safe != ARRAY_SIZE(motor_pins)) {
		printk("MOTOR SAFE FAILED -- do not arm the FPGA sweep\n");
	}

	for (;;) {
		/* 250 ms, not 5 s: the FPGA sweep drives each motor for as
		 * little as 1 s, and a slow probe would sample between pulses
		 * and report a dead gate for a working one. */
		k_sleep(K_MSEC(250));
		report_gate_levels();

		if (++tick % 20U == 0U) {
			safe = park_esp_motor_pins(false);
			printk("MOTOR SAFE heartbeat: %u/4 inputs+pulldowns\n", safe);
		}
	}

	return 0;
}
