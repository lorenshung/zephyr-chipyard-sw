/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Explicitly armed two-motor PWM diagnostic. Both outputs start at zero.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define MOTOR_PERIOD_NSEC (50U * 1000U)
#define MOTOR_TEST_PULSE_NSEC (MOTOR_PERIOD_NSEC / 10U)

static const struct pwm_dt_spec motors[] = {
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor0)),
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor1)),
};
static const struct device *const console =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static int set_all(uint32_t pulse)
{
	for (size_t i = 0; i < ARRAY_SIZE(motors); ++i) {
		int ret = pwm_set_dt(&motors[i], MOTOR_PERIOD_NSEC, pulse);
		if (ret != 0) {
			return ret;
		}
	}
	return 0;
}

int main(void)
{
	uint8_t byte;

	printk("MOTOR1+MOTOR2 diagnostic: outputs start STOPPED\n");
	printk("Propellers off. Commands: s=10%% duty, x/space/q=stop\n");
	for (size_t i = 0; i < ARRAY_SIZE(motors); ++i) {
		if (!pwm_is_ready_dt(&motors[i])) {
			printk("ERROR: PWM %u is not ready\n", (unsigned int)i);
			return 1;
		}
	}
	if (!device_is_ready(console) || set_all(0) != 0) {
		printk("ERROR: console not ready or outputs could not be stopped\n");
		return 1;
	}

	while (1) {
		if (uart_poll_in(console, &byte) == 0) {
			if (byte == 's' || byte == 'S') {
				if (set_all(MOTOR_TEST_PULSE_NSEC) == 0) {
					printk("MOTOR1+MOTOR2 START 10%%\n");
				}
			} else if (byte == 'x' || byte == 'X' || byte == ' ' ||
				   byte == 'q' || byte == 'Q') {
				(void)set_all(0);
				printk("MOTOR1+MOTOR2 STOP\n");
			}
		}
		k_msleep(10);
	}
	return 0;
}
