/* SPDX-License-Identifier: Apache-2.0 */

#include "rb_imu.h"
#include "rb_madgwick.h"

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>

#define RAD_TO_MILLIDEG 57295.7795f

static int32_t to_milli(float value)
{
	return (int32_t)(value * 1000.0f);
}

int main(void)
{
	struct rb_madgwick filter;
	struct rb_imu_sample sample;
	uint32_t sequence = 0;
	int64_t previous_us = 0;
	int result = rb_imu_init();

	if (result != 0) {
		printk("RB_STATE error=imu_init code=%d\n", result);
		return result;
	}

	rb_madgwick_init(&filter, (float)CONFIG_RB_STATE_BETA_MILLI / 1000.0f);
	timing_init();
	timing_start();
	printk("RB_STATE ready source=%s rate_hz=%d beta_milli=%d\n",
	       rb_imu_source_name(), CONFIG_RB_STATE_RATE_HZ,
	       CONFIG_RB_STATE_BETA_MILLI);

	while (true) {
		int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
		float dt = previous_us == 0
				   ? 1.0f / (float)CONFIG_RB_STATE_RATE_HZ
				   : (float)(now_us - previous_us) / 1000000.0f;
		previous_us = now_us;

		result = rb_imu_read(&sample);
		if (result != 0) {
			printk("RB_STATE error=imu_read code=%d seq=%u\n", result, sequence);
			k_msleep(10);
			continue;
		}

		timing_t start = timing_counter_get();
		rb_madgwick_update_imu(
			&filter,
			sample.gyro_rps[0],
			sample.gyro_rps[1],
			sample.gyro_rps[2],
			sample.accel_mps2[0],
			sample.accel_mps2[1],
			sample.accel_mps2[2],
			dt);
		timing_t end = timing_counter_get();

		if ((sequence % (CONFIG_RB_STATE_RATE_HZ / 10)) == 0U) {
			struct rb_euler euler;
			uint64_t cycles = timing_cycles_get(&start, &end);

			rb_madgwick_euler(&filter, &euler);
			printk(
				"RB_STATE seq=%u q_milli=%d,%d,%d,%d "
				"euler_mdeg=%d,%d,%d update_ns=%llu\n",
				sequence,
				to_milli(filter.q[0]),
				to_milli(filter.q[1]),
				to_milli(filter.q[2]),
				to_milli(filter.q[3]),
				(int32_t)(euler.roll * RAD_TO_MILLIDEG),
				(int32_t)(euler.pitch * RAD_TO_MILLIDEG),
				(int32_t)(euler.yaw * RAD_TO_MILLIDEG),
				(unsigned long long)timing_cycles_to_ns(cycles));
		}
		sequence++;
		k_usleep(1000000U / CONFIG_RB_STATE_RATE_HZ);
	}
	return 0;
}
