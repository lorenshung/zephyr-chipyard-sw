/* SPDX-License-Identifier: Apache-2.0 */

#include "rb_imu.h"

#include <stddef.h>

#define GRAVITY_MPS2 9.80665f
#define YAW_RATE_RPS 0.5235988f

static unsigned int replay_index;

int rb_imu_init(void)
{
	replay_index = 0;
	return 0;
}

int rb_imu_read(struct rb_imu_sample *sample)
{
	if (sample == NULL) {
		return -1;
	}

	sample->accel_mps2[0] = 0.0f;
	sample->accel_mps2[1] = 0.0f;
	sample->accel_mps2[2] = GRAVITY_MPS2;
	sample->gyro_rps[0] = 0.0f;
	sample->gyro_rps[1] = 0.0f;
	/* One second still, one second at 30 degrees/s, then repeat. */
	const unsigned int one_second = CONFIG_RB_STATE_RATE_HZ;
	sample->gyro_rps[2] =
		(replay_index % (2U * one_second)) >= one_second ? YAW_RATE_RPS : 0.0f;
	replay_index++;
	return 0;
}

const char *rb_imu_source_name(void)
{
	return "replay";
}
