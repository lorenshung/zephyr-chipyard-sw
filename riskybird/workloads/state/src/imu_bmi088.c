/* SPDX-License-Identifier: Apache-2.0 */

#include "rb_imu.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#define BMI088_ACCEL_NODE DT_ALIAS(bmi088_accel)
#define BMI088_GYRO_NODE DT_ALIAS(bmi088_gyro)

#if !DT_NODE_EXISTS(BMI088_ACCEL_NODE)
#error "RiskyBird FPGA overlay must define the bmi088-accel alias"
#endif

#if !DT_NODE_EXISTS(BMI088_GYRO_NODE)
#error "RiskyBird FPGA overlay must define the bmi088-gyro alias"
#endif

static const struct device *const accel = DEVICE_DT_GET(BMI088_ACCEL_NODE);
static const struct device *const gyro = DEVICE_DT_GET(BMI088_GYRO_NODE);
static float gyro_bias[3];

static int read_uncalibrated(struct rb_imu_sample *sample)
{
	struct sensor_value accel_values[3];
	struct sensor_value gyro_values[3];
	int result = sensor_sample_fetch(accel);

	if (result != 0) {
		return result;
	}
	result = sensor_channel_get(accel, SENSOR_CHAN_ACCEL_XYZ, accel_values);
	if (result != 0) {
		return result;
	}
	result = sensor_sample_fetch(gyro);
	if (result != 0) {
		return result;
	}
	result = sensor_channel_get(gyro, SENSOR_CHAN_GYRO_XYZ, gyro_values);
	if (result != 0) {
		return result;
	}

	for (size_t axis = 0; axis < 3; axis++) {
		sample->accel_mps2[axis] = (float)sensor_value_to_double(&accel_values[axis]);
		sample->gyro_rps[axis] = (float)sensor_value_to_double(&gyro_values[axis]);
	}
	return 0;
}

int rb_imu_init(void)
{
	if (!device_is_ready(accel) || !device_is_ready(gyro)) {
		return -ENODEV;
	}

	for (size_t axis = 0; axis < 3; axis++) {
		gyro_bias[axis] = 0.0f;
	}

	for (int sample_index = 0; sample_index < CONFIG_RB_STATE_CALIBRATION_SAMPLES;
	     sample_index++) {
		struct rb_imu_sample sample;
		int result = read_uncalibrated(&sample);

		if (result != 0) {
			return result;
		}
		for (size_t axis = 0; axis < 3; axis++) {
			gyro_bias[axis] += sample.gyro_rps[axis];
		}
		k_msleep(5);
	}

	if (CONFIG_RB_STATE_CALIBRATION_SAMPLES > 0) {
		for (size_t axis = 0; axis < 3; axis++) {
			gyro_bias[axis] /= (float)CONFIG_RB_STATE_CALIBRATION_SAMPLES;
		}
	}
	return 0;
}

int rb_imu_read(struct rb_imu_sample *sample)
{
	int result = read_uncalibrated(sample);

	if (result == 0) {
		for (size_t axis = 0; axis < 3; axis++) {
			sample->gyro_rps[axis] -= gyro_bias[axis];
		}
	}
	return result;
}

const char *rb_imu_source_name(void)
{
	return "bmi088";
}
