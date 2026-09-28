/* SPDX-License-Identifier: Apache-2.0 */

#include "rb_madgwick.h"

#include <math.h>
#include <stddef.h>

#define RB_NORM_EPSILON 1.0e-12f
#define RB_PI_OVER_4 0.7853981634f
#define RB_THREE_PI_OVER_4 2.3561944902f

static float inverse_norm(float value)
{
	if (value <= RB_NORM_EPSILON) {
		return 0.0f;
	}
	return 1.0f / sqrtf(value);
}

/* A compact atan2 approximation is sufficient for human-readable Euler
 * telemetry. The estimator state itself remains the normalized quaternion. */
static float approximate_atan2(float y, float x)
{
	float absolute_y = y < 0.0f ? -y : y;
	float ratio;
	float angle;

	absolute_y += 1.0e-10f;
	if (x < 0.0f) {
		ratio = (x + absolute_y) / (absolute_y - x);
		angle = RB_THREE_PI_OVER_4;
	} else {
		ratio = (x - absolute_y) / (x + absolute_y);
		angle = RB_PI_OVER_4;
	}
	angle += (0.1963f * ratio * ratio - 0.9817f) * ratio;
	return y < 0.0f ? -angle : angle;
}

void rb_madgwick_init(struct rb_madgwick *filter, float beta)
{
	filter->q[0] = 1.0f;
	filter->q[1] = 0.0f;
	filter->q[2] = 0.0f;
	filter->q[3] = 0.0f;
	filter->beta = beta;
}

void rb_madgwick_update_imu(
	struct rb_madgwick *filter,
	float gx,
	float gy,
	float gz,
	float ax,
	float ay,
	float az,
	float dt)
{
	float q0 = filter->q[0];
	float q1 = filter->q[1];
	float q2 = filter->q[2];
	float q3 = filter->q[3];
	float q_dot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	float q_dot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	float q_dot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	float q_dot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
	float accel_recip = inverse_norm(ax * ax + ay * ay + az * az);

	if (accel_recip > 0.0f) {
		ax *= accel_recip;
		ay *= accel_recip;
		az *= accel_recip;

		float two_q0 = 2.0f * q0;
		float two_q1 = 2.0f * q1;
		float two_q2 = 2.0f * q2;
		float two_q3 = 2.0f * q3;
		float four_q0 = 4.0f * q0;
		float four_q1 = 4.0f * q1;
		float four_q2 = 4.0f * q2;
		float eight_q1 = 8.0f * q1;
		float eight_q2 = 8.0f * q2;
		float q0q0 = q0 * q0;
		float q1q1 = q1 * q1;
		float q2q2 = q2 * q2;
		float q3q3 = q3 * q3;

		float s0 = four_q0 * q2q2 + two_q2 * ax +
			   four_q0 * q1q1 - two_q1 * ay;
		float s1 = four_q1 * q3q3 - two_q3 * ax +
			   4.0f * q0q0 * q1 - two_q0 * ay - four_q1 +
			   eight_q1 * q1q1 + eight_q1 * q2q2 + four_q1 * az;
		float s2 = 4.0f * q0q0 * q2 + two_q0 * ax +
			   four_q2 * q3q3 - two_q3 * ay - four_q2 +
			   eight_q2 * q1q1 + eight_q2 * q2q2 + four_q2 * az;
		float s3 = 4.0f * q1q1 * q3 - two_q1 * ax +
			   4.0f * q2q2 * q3 - two_q2 * ay;
		float step_recip = inverse_norm(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);

		if (step_recip > 0.0f) {
			q_dot0 -= filter->beta * s0 * step_recip;
			q_dot1 -= filter->beta * s1 * step_recip;
			q_dot2 -= filter->beta * s2 * step_recip;
			q_dot3 -= filter->beta * s3 * step_recip;
		}
	}

	q0 += q_dot0 * dt;
	q1 += q_dot1 * dt;
	q2 += q_dot2 * dt;
	q3 += q_dot3 * dt;

	float quaternion_recip = inverse_norm(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	if (quaternion_recip == 0.0f) {
		rb_madgwick_init(filter, filter->beta);
		return;
	}
	filter->q[0] = q0 * quaternion_recip;
	filter->q[1] = q1 * quaternion_recip;
	filter->q[2] = q2 * quaternion_recip;
	filter->q[3] = q3 * quaternion_recip;
}

void rb_madgwick_euler(const struct rb_madgwick *filter, struct rb_euler *euler)
{
	const float q0 = filter->q[0];
	const float q1 = filter->q[1];
	const float q2 = filter->q[2];
	const float q3 = filter->q[3];
	float pitch_sine = 2.0f * (q0 * q2 - q3 * q1);

	if (pitch_sine > 1.0f) {
		pitch_sine = 1.0f;
	} else if (pitch_sine < -1.0f) {
		pitch_sine = -1.0f;
	}

	euler->roll = approximate_atan2(
		2.0f * (q0 * q1 + q2 * q3),
		1.0f - 2.0f * (q1 * q1 + q2 * q2));
	euler->pitch = approximate_atan2(
		pitch_sine,
		sqrtf(1.0f - pitch_sine * pitch_sine));
	euler->yaw = approximate_atan2(
		2.0f * (q0 * q3 + q1 * q2),
		1.0f - 2.0f * (q2 * q2 + q3 * q3));
}
