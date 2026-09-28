/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RB_IMU_H
#define RB_IMU_H

struct rb_imu_sample {
	float accel_mps2[3];
	float gyro_rps[3];
};

int rb_imu_init(void);
int rb_imu_read(struct rb_imu_sample *sample);
const char *rb_imu_source_name(void);

#endif
