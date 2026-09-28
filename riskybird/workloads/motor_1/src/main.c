/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Explicitly armed one-motor PWM diagnostic. The output is forced to zero at
 * boot. With propellers removed, send 's' on the console to apply 10% duty and
 * 'x', space, or 'q' to stop.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define MOTOR_PERIOD_NSEC (50U * 1000U)
#define MOTOR_TEST_PULSE_NSEC (MOTOR_PERIOD_NSEC / 10U)

static const struct pwm_dt_spec motor = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor));
static const struct device *const console =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

int main(void)
{
	uint8_t byte;

	printk("MOTOR1 diagnostic: output starts STOPPED\n");
	printk("Propellers off. Commands: s=10%% duty, x/space/q=stop\n");

	if (!pwm_is_ready_dt(&motor) || !device_is_ready(console)) {
		printk("ERROR: PWM or console device is not ready\n");
		return 1;
	}
	if (pwm_set_dt(&motor, MOTOR_PERIOD_NSEC, 0) != 0) {
		printk("ERROR: could not force MOTOR1 output low\n");
		return 1;
	}

	while (1) {
		if (uart_poll_in(console, &byte) == 0) {
			if (byte == 's' || byte == 'S') {
				if (pwm_set_dt(&motor, MOTOR_PERIOD_NSEC,
					       MOTOR_TEST_PULSE_NSEC) == 0) {
					printk("MOTOR1 START 10%%\n");
				}
			} else if (byte == 'x' || byte == 'X' || byte == ' ' ||
				   byte == 'q' || byte == 'Q') {
				(void)pwm_set_dt(&motor, MOTOR_PERIOD_NSEC, 0);
				printk("MOTOR1 STOP\n");
			}
		}
		k_msleep(10);
	}
	return 0;
}
