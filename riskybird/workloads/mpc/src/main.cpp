/* SPDX-License-Identifier: Apache-2.0 */

#include <admm.hpp>
#include <problem_data/quadrotor_50hz_params_constrained.hpp>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>

#if defined(CONFIG_RB_MPC_INPUT_ROSE)
#include <rose/rose.h>
#include <rose/rose_proto.h>
#endif

namespace {

constexpr uint8_t kHeader[] = {0xDE, 0xAD, 0xBE, 0xEF};
constexpr int kStates = NSTATES;
constexpr int kActions = NINPUTS;

TinyCache cache;
TinyWorkspace workspace;
TinySettings settings;
TinySolver solver{&settings, &cache, &workspace};

void initialize_solver()
{
	tiny_init(&solver);

	cache.rho = rho_value;
	matsetv(cache.Kinf.data, Kinf_data, cache.Kinf.outer, cache.Kinf.inner);
	transpose(cache.Kinf.data, cache.KinfT.data, NINPUTS, NSTATES);
	matsetv(cache.Pinf.data, Pinf_data, cache.Pinf.outer, cache.Pinf.inner);
	transpose(cache.Pinf.data, cache.PinfT.data, NSTATES, NSTATES);
	matsetv(
		cache.Quu_inv.data,
		Quu_inv_data,
		cache.Quu_inv.outer,
		cache.Quu_inv.inner);
	matsetv(cache.AmBKt.data, AmBKt_data, cache.AmBKt.outer, cache.AmBKt.inner);
	transpose(cache.AmBKt.data, cache.AmBKtT.data, NSTATES, NSTATES);
	matsetv(
		cache.coeff_d2p.data,
		coeff_d2p_data,
		cache.coeff_d2p.outer,
		cache.coeff_d2p.inner);

	matsetv(
		workspace.Adyn.data,
		Adyn_data,
		workspace.Adyn.outer,
		workspace.Adyn.inner);
	transpose(workspace.Adyn.data, workspace.AdynT.data, NSTATES, NSTATES);
	matsetv(
		workspace.Bdyn.data,
		Bdyn_data,
		workspace.Bdyn.outer,
		workspace.Bdyn.inner);
	transpose(workspace.Bdyn.data, workspace.BdynT.data, NSTATES, NINPUTS);
	matsetv(workspace.Q.data, Q_data, workspace.Q.outer, workspace.Q.inner);
	matsetv(workspace.Qf.data, Qf_data, workspace.Qf.outer, workspace.Qf.inner);
	matsetv(workspace.R.data, R_data, workspace.R.outer, workspace.R.inner);

	matset(workspace.u_min.data, -0.583f, workspace.u_min.outer, workspace.u_min.inner);
	matset(
		workspace.u_max.data,
		1.0f - 0.583f,
		workspace.u_max.outer,
		workspace.u_max.inner);
	matset(workspace.x_min.data, -5.0f, workspace.x_min.outer, workspace.x_min.inner);
	matset(workspace.x_max.data, 5.0f, workspace.x_max.outer, workspace.x_max.inner);
	matset(workspace.Xref.data, 0.0f, workspace.Xref.outer, workspace.Xref.inner);
	matset(workspace.Uref.data, 0.0f, workspace.Uref.outer, workspace.Uref.inner);
}

uint32_t solve(const float state[kStates], float control[kActions], int *status)
{
	// The backend matlib predates const-correct input pointers; matsetv only
	// reads its second argument.
	matsetv(workspace.x.vector[0], const_cast<float *>(state), 1, NSTATES);
	matset(workspace.y.data, 0.0f, workspace.y.outer, workspace.y.inner);
	matset(workspace.g.data, 0.0f, workspace.g.outer, workspace.g.inner);

	timing_t start = timing_counter_get();
	*status = tiny_solve(&solver);
	timing_t end = timing_counter_get();
	for (int action = 0; action < kActions; action++) {
		control[action] = workspace.u.vector[0][action];
	}

	uint64_t cycles = timing_cycles_get(&start, &end);
	uint64_t nanoseconds = timing_cycles_to_ns(cycles);
	return nanoseconds > UINT32_MAX ? UINT32_MAX
				       : static_cast<uint32_t>(nanoseconds);
}

#if defined(CONFIG_RB_MPC_INPUT_HIL)

const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

uint8_t receive_byte()
{
	uint8_t value;
	while (uart_poll_in(uart, &value) != 0) {
	}
	return value;
}

void receive_header()
{
	size_t matched = 0;
	while (matched < sizeof(kHeader)) {
		uint8_t value = receive_byte();
		matched = value == kHeader[matched] ? matched + 1
						  : (value == kHeader[0] ? 1 : 0);
	}
}

float receive_float()
{
	uint8_t bytes[sizeof(float)];
	float value;
	for (size_t index = 0; index < sizeof(bytes); index++) {
		bytes[index] = receive_byte();
	}
	memcpy(&value, bytes, sizeof(value));
	return value;
}

void send_bytes(const void *data, size_t size)
{
	const auto *bytes = static_cast<const uint8_t *>(data);
	for (size_t index = 0; index < size; index++) {
		uart_poll_out(uart, bytes[index]);
	}
}

void send_response(const float control[kActions], uint32_t nanoseconds)
{
	const uint8_t drone_id = 0;
	send_bytes(kHeader, sizeof(kHeader));
	send_bytes(&drone_id, sizeof(drone_id));
	send_bytes(control, sizeof(float) * kActions);
	send_bytes(&nanoseconds, sizeof(nanoseconds));
}

void run_hil()
{
	if (!device_is_ready(uart)) {
		return;
	}
	while (true) {
		receive_header();
		uint8_t drone_count = receive_byte();
		float state[kStates] = {};

		for (uint8_t drone = 0; drone < drone_count; drone++) {
			for (int index = 0; index < kStates; index++) {
				float value = receive_float();
				if (drone == 0) {
					state[index] = value;
				}
			}
		}
		if (drone_count == 0) {
			continue;
		}

		float control[kActions];
		int status;
		uint32_t nanoseconds = solve(state, control, &status);
		send_response(control, nanoseconds);
	}
}

#elif defined(CONFIG_RB_MPC_INPUT_ROSE)

constexpr uint32_t kRoseStateCommand = 0x12U;
constexpr uint32_t kRoseThrustCommand = 0x20U;
constexpr uint8_t kRoseStateChannel = 2U;

const struct device *const rose = DEVICE_DT_GET_ONE(ucbbar_roseadapter);

bool receive_rose_state(float state[kStates])
{
	uint32_t words[kStates];
	if (rose_request(rose, kRoseStateCommand, 0U) != 0) {
		return false;
	}
	int received = rose_recv_reqrsp(
		rose, kRoseStateChannel, words, kStates);
	if (received != kStates) {
		return false;
	}
	memcpy(state, words, sizeof(words));
	return true;
}

bool send_rose_control(const float control[kActions])
{
	if (rose_tx(rose, kRoseThrustCommand) != 0 ||
	    rose_tx(rose, sizeof(float) * kActions) != 0) {
		return false;
	}
	for (int index = 0; index < kActions; index++) {
		uint32_t word;
		memcpy(&word, &control[index], sizeof(word));
		if (rose_tx(rose, word) != 0) {
			return false;
		}
	}
	return true;
}

void run_rose()
{
	if (!device_is_ready(rose)) {
		printk("RB_MPC_ROSE error=device_not_ready\n");
		return;
	}
	printk(
		"RB_MPC_ROSE ready states=%d actions=%d horizon=%d\n",
		NSTATES,
		NINPUTS,
		NHORIZON);
	uint32_t iteration = 0;
	while (true) {
		float state[kStates];
		if (!receive_rose_state(state)) {
			printk("RB_MPC_ROSE error=state_receive\n");
			continue;
		}
		float control[kActions];
		int status;
		uint32_t nanoseconds = solve(state, control, &status);
		if (!send_rose_control(control)) {
			printk("RB_MPC_ROSE error=control_send\n");
			continue;
		}
		iteration++;
		printk(
			"RB_MPC_ROSE iter=%u status=%d z_milli=%d solve_ns=%u\n",
			iteration,
			status,
			static_cast<int>(state[2] * 1000.0f),
			nanoseconds);
	}
}

#else

int milli(float value)
{
	return static_cast<int>(value * 1000.0f);
}

constexpr float kReplayStates[][kStates] = {
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
	{0.25f, -0.10f, 0.15f, 0.02f, -0.03f, 0.01f,
	 0.05f, -0.02f, 0.01f, 0.0f, 0.0f, 0.0f},
	{-0.20f, 0.20f, -0.10f, -0.01f, 0.04f, -0.02f,
	 -0.03f, 0.04f, -0.02f, 0.0f, 0.0f, 0.0f},
};

void run_replay()
{
	uint32_t sequence = 0;
	while (true) {
		const float *state =
			kReplayStates[sequence % (sizeof(kReplayStates) / sizeof(kReplayStates[0]))];
		float control[kActions];
		int status;
		uint32_t nanoseconds = solve(state, control, &status);

		printk(
			"RB_MPC seq=%u status=%d u_milli=%d,%d,%d,%d solve_ns=%u\n",
			sequence,
			status,
			milli(control[0]),
			milli(control[1]),
			milli(control[2]),
			milli(control[3]),
			nanoseconds);
		sequence++;
		k_busy_wait(CONFIG_RB_MPC_REPLAY_PERIOD_MS * 1000);
	}
}

#endif

}  // namespace

int main()
{
	timing_init();
	timing_start();
	initialize_solver();

#if defined(CONFIG_RB_MPC_INPUT_HIL)
	run_hil();
#elif defined(CONFIG_RB_MPC_INPUT_ROSE)
	run_rose();
#else
	printk(
		"RB_MPC ready mode=replay states=%d actions=%d horizon=%d workspace_bytes=%u\n",
		NSTATES,
		NINPUTS,
		NHORIZON,
		static_cast<unsigned int>(sizeof(TinyWorkspace)));
	run_replay();
#endif
	return 0;
}
