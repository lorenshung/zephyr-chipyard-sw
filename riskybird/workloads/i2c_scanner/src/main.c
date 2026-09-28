/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/reboot.h>
#include <stdint.h>
#include <stdio.h>

#define I2C_NODE DT_NODELABEL(i2c0)

/*
 * The scan result is also left in plain globals, not only printed.
 *
 * On a board being brought up the console is the least trustworthy thing in the
 * system -- it is often the reason the scan is being run at all. These let a
 * debugger read the answer straight out of memory over the same JTAG link that
 * loaded the image:
 *
 *   (gdb) p i2c_scan_count
 *   (gdb) p/x i2c_scan_found
 *   (gdb) p i2c_scan_last_error
 *
 * i2c_scan_last_error is the diagnostic that separates "the bus is fine and
 * empty" from "the bus is broken": a floating or un-pulled-up bus fails every
 * address the same way, whereas a healthy bus with no devices returns a clean
 * NAK for each one.
 */
volatile uint8_t  i2c_scan_found[128];
volatile uint32_t i2c_scan_count;
volatile int32_t  i2c_scan_last_error;
volatile uint32_t i2c_scan_attempts;
volatile uint32_t i2c_scan_state;

#define I2C_SCAN_STARTED  0x11111111u
#define I2C_SCAN_FINISHED 0x5CA77EDu

/* A stable symbol to break on; noinline keeps it out of main. */
__attribute__((noinline)) void i2c_scan_done(void)
{
	i2c_scan_state = I2C_SCAN_FINISHED;
}

int main(void)
{
	const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

	i2c_scan_state = I2C_SCAN_STARTED;

	if (!device_is_ready(i2c_dev)) {
		printf("I2C: Device not ready.\n");
		i2c_scan_last_error = -ENODEV;
		i2c_scan_done();
		return 1;
	}

	printf("Starting I2C scan on %s...\n", i2c_dev->name);

	for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
		/*
		 * Try to write zero bytes. If the device ACKs, it's present.
		 */
		int ret = i2c_write(i2c_dev, NULL, 0, addr);

		i2c_scan_attempts++;
		if (ret == 0) {
			if (i2c_scan_count < ARRAY_SIZE(i2c_scan_found)) {
				i2c_scan_found[i2c_scan_count] = addr;
			}
			i2c_scan_count++;
			printf("Found device at 0x%02X\n", addr);
		} else {
			i2c_scan_last_error = ret;
		}
		k_msleep(10); /* short delay between probes */
	}

	printf("I2C scan complete: %u device(s) found in %u probes",
	       (unsigned int)i2c_scan_count, (unsigned int)i2c_scan_attempts);
	if (i2c_scan_count == 0) {
		printf(", last error %d", (int)i2c_scan_last_error);
	}
	printf("\n");

	for (uint32_t i = 0; i < i2c_scan_count; i++) {
		printf("  address 0x%02X\n", i2c_scan_found[i]);
	}

	i2c_scan_done();

	return 0;
}
