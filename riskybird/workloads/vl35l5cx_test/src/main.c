/* SPDX-License-Identifier: Apache-2.0 */

#include <riskybird/tof.h>

#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#define I2C_NODE DT_NODELABEL(i2c0)

#if defined(CONFIG_RB_VL53L5CX_RANGING)
/*
 * The ranging device for slot n, or NULL when the target's overlay declares no
 * tof<n>. The overlays declare a VL53L5CX node only for the slots that can hold
 * one, so a slot with no alias -- the down VL53L1X, or an unused slot -- is
 * skipped by rb_tof_array_init's ranging pass instead of failing the build.
 */
#define TOF_DEVICE(n)                                                      \
	COND_CODE_1(DT_NODE_EXISTS(DT_ALIAS(tof##n)),                      \
		    (DEVICE_DT_GET(DT_ALIAS(tof##n))), (NULL))
#endif

static struct rb_tof_array tof_array;

static const struct rb_tof_array_config tof_config = {
	.i2c = DEVICE_DT_GET(I2C_NODE),
#if defined(CONFIG_RB_VL53L5CX_RANGING)
	.sensors = {
		TOF_DEVICE(0),
		TOF_DEVICE(1),
		TOF_DEVICE(2),
		TOF_DEVICE(3),
		TOF_DEVICE(4),
	},
#endif
	/*
	 * Four sensors, all gated by the ADS7128, on GPIO channels 1, 2, 3
	 * and 6. Measured rather than assumed:
	 *
	 *  - rb_tof_scan_xshut releases each unclaimed channel on its own and
	 *    probes 0x29. Only 1, 2, 3 and 6 gate anything; channels 0, 4, 5
	 *    and 7 gate nothing. The documented assumption was 1..4, and
	 *    channel 4 is empty.
	 *  - rb_tof_hold_all drives every channel low and sweeps the bus. No
	 *    VL53 part answers, so nothing here is wired directly: every ToF on
	 *    this board is behind the expander.
	 *
	 * The parts are mixed -- VL53L5CX on the side channels, the down
	 * VL53L1X on 6 -- so the kind is detected per sensor rather than
	 * declared. The board is meant to carry three (two side plus the down
	 * one); bootup_check on 2026-09-10 found only channel 1's side sensor
	 * and the down sensor answering, so a missing side sensor here is a
	 * real fault, not a config to trim.
	 */
	.sensor_count = 4,
	.xshut_channels = { 1, 2, 3, 6, RB_TOF_XSHUT_DIRECT },
	.addresses = { 0x31, 0x32, 0x33, 0x34, 0x35 },
};

#define RB_TOF_ACTIVE 4U

static void print_address_status(void)
{
	for (uint8_t index = 0U; index < RB_TOF_ACTIVE; index++) {
		uint16_t identity = 0U;
		int result = rb_tof_probe(&tof_array, index, &identity);

		printf(
			"RB_TOF sensor=%u %s addr=0x%02x part=%-8s "
			"identity=0x%04x valid=%u error=%d\n",
			index,
			tof_config.xshut_channels[index] == RB_TOF_XSHUT_DIRECT
				? "direct"
				: "xshut ",
			tof_config.addresses[index],
			rb_tof_kind_name(tof_array.kinds[index]),
			identity,
			result == 0 ? 1U : 0U,
			result);
	}
}

int main(void)
{
	printf(
		"RB_TOF ready mode=%s sensors=%u xshut=1,2,3,6 "
		"addresses=0x31,0x32,0x33,0x34\n",
		rb_tof_ranging_available() ? "ranging" : "address",
		RB_TOF_ACTIVE);

	int result = rb_tof_array_init(&tof_array, &tof_config);

	printf(
		"RB_TOF init code=%d initialized=%u of %u\n",
		result,
		tof_array.initialized_sensors,
		RB_TOF_ACTIVE);
	for (uint8_t index = 0U; index < RB_TOF_ACTIVE; index++) {
		if (tof_array.status[index] != 0) {
			printf(
				"RB_TOF sensor=%u FAILED code=%d\n",
				index,
				tof_array.status[index]);
		}
	}
	if (result != 0) {
		printf(
			"RB_TOF error=init step=%s sensor=%u\n",
			rb_tof_step_name(tof_array.last_step),
			tof_array.last_sensor);
		return result;
	}

	print_address_status();

	/*
	 * Only map the topology when something is missing. The map resets every
	 * sensor and polls each of the eight channels in turn, which takes
	 * seconds and destroys the addresses just assigned -- worth it to find
	 * a miswired channel, not worth it on a healthy boot.
	 */
	if (tof_array.initialized_sensors < RB_TOF_ACTIVE) {
		const struct device *bus = DEVICE_DT_GET(I2C_NODE);
		uint8_t kinds[8];
		uint8_t direct_kind = RB_TOF_KIND_NONE;
		uint8_t direct_addr = 0U;

		printf("RB_TOF incomplete -- mapping topology\n");
		if (rb_tof_map(bus, kinds, &direct_kind, &direct_addr) == 0) {
			for (uint8_t ch = 0U; ch < 8U; ch++) {
				printf("  channel %u -> %s\n", ch,
				       rb_tof_kind_name(kinds[ch]));
			}
			printf("  direct (all channels low) -> %s",
			       rb_tof_kind_name(direct_kind));
			if (direct_kind != RB_TOF_KIND_NONE) {
				printf(" at 0x%02x", direct_addr);
			}
			printf("\n");
		}
	}

	while (true) {
		if (!rb_tof_ranging_available()) {
			print_address_status();
			k_msleep(500);
			continue;
		}

		for (uint8_t index = 0U; index < RB_TOF_ACTIVE; index++) {
			struct rb_tof_frame frame;

			/* Not a ranging slot: absent, or the down VL53L1X. */
			if (!rb_tof_ranging_sensor(&tof_array, index)) {
				continue;
			}
			result = rb_tof_fetch(&tof_array, index, &frame);
			if (result != 0) {
				printf(
					"RB_TOF error=fetch sensor=%u code=%d\n",
					index,
					result);
				continue;
			}

			uint8_t center = (uint8_t)(
				(frame.rows / 2U) * frame.columns +
				(frame.columns / 2U));
			printf(
				"RB_TOF seq=%u host_us=%llu sensor=%u "
				"addr=0x%02x shape=%ux%u center_mm=%d "
				"center_targets=%u center_status=%u row0_mm=",
				frame.sequence,
				(unsigned long long)frame.host_timestamp_us,
				frame.sensor_index,
				frame.address,
				frame.rows,
				frame.columns,
				frame.distance_mm[center],
				frame.target_count[center],
				frame.target_status[center]);
			for (uint8_t column = 0U; column < frame.columns; column++) {
				printf(
					"%s%d",
					column == 0U ? "" : ",",
					frame.distance_mm[column]);
			}
			printf("\n");
		}
		k_msleep(500);
	}
	return 0;
}
