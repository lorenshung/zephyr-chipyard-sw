/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RB_MADGWICK_H
#define RB_MADGWICK_H

struct rb_madgwick {
	float q[4];
	float beta;
};

struct rb_euler {
	float roll;
	float pitch;
	float yaw;
};

void rb_madgwick_init(struct rb_madgwick *filter, float beta);
void rb_madgwick_update_imu(
	struct rb_madgwick *filter,
	float gx,
	float gy,
	float gz,
	float ax,
	float ay,
	float az,
	float dt);
void rb_madgwick_euler(const struct rb_madgwick *filter, struct rb_euler *euler);

#endif
