/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RISKYBIRD_TOF_H_
#define RISKYBIRD_TOF_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RB_TOF_SENSOR_COUNT 5U
#define RB_TOF_MAX_ZONES 64U
#define RB_TOF_VL53L5CX_ID 0xF002U
#define RB_TOF_VL53L1X_ID 0xEACCU

/*
 * Which VL53 part a sensor turned out to be. The RiskyBird array is mixed --
 * the directly-wired sensor is a VL53L1X while the XSHUT sensors need not be --
 * and the families share neither an ID register nor an address-change register,
 * so the kind is detected per sensor rather than declared.
 */
enum rb_tof_kind {
	RB_TOF_KIND_NONE = 0,
	RB_TOF_KIND_VL53L5CX,
	RB_TOF_KIND_VL53L1X,
};

/*
 * xshut_channels entry for a sensor wired straight to the bus with no XSHUT
 * line of its own. Such a sensor cannot be held in reset, so it is always live
 * at the VL53L5CX default address and would collide with every XSHUT sensor as
 * that one is released. rb_tof_array_init therefore readdresses the direct
 * sensors first, while the others are still held down.
 */
#define RB_TOF_XSHUT_DIRECT 0xFFU

/*
 * Stable RiskyBird contract for the RiskyBird ToF array: up to
 * RB_TOF_SENSOR_COUNT sensors whose XSHUT lines hang off the ADS7128 GPIO
 * expander, or that are wired directly to the bus. The drone carrier is meant
 * to carry three -- two side VL53L5CX and the down VL53L1X on GPIO6 -- and on
 * the current board only one side sensor answers. All addresses are 7-bit I2C
 * addresses.
 */
struct rb_tof_array_config {
	const struct device *i2c;
	/*
	 * How many entries of the arrays below are populated. Zero means all
	 * RB_TOF_SENSOR_COUNT of them. A board with fewer sensors than the
	 * maximum should say so rather than leave trailing slots to fail, which
	 * makes a real fault indistinguishable from an empty slot.
	 */
	uint8_t sensor_count;
	const struct device *sensors[RB_TOF_SENSOR_COUNT];
	uint8_t xshut_channels[RB_TOF_SENSOR_COUNT];
	uint8_t addresses[RB_TOF_SENSOR_COUNT];
};

/*
 * Where initialization stopped. An -EIO from rb_tof_array_init could come from
 * the expander, from a sensor that never appeared, or from the readdress that
 * follows; on a board being brought up those are entirely different faults and
 * the return code alone cannot tell them apart.
 */
enum rb_tof_step {
	RB_TOF_STEP_NONE = 0,
	RB_TOF_STEP_PIN_CFG,
	RB_TOF_STEP_GPIO_CFG,
	RB_TOF_STEP_DRIVE_CFG,
	RB_TOF_STEP_GPO_READ,
	RB_TOF_STEP_GPO_WRITE,
	RB_TOF_STEP_DIRECT_PROBE,
	RB_TOF_STEP_DIRECT_READDRESS,
	RB_TOF_STEP_ENABLE,
	RB_TOF_STEP_WAIT,
	RB_TOF_STEP_READDRESS,
	RB_TOF_STEP_VERIFY,
};

struct rb_tof_array {
	struct rb_tof_array_config config;
	uint32_t sequence;
	uint8_t gpo_shadow;
	uint8_t initialized_sensors;
	uint8_t last_step;
	uint8_t last_sensor;
	int last_error;
	uint8_t kinds[RB_TOF_SENSOR_COUNT];
	uint16_t identities[RB_TOF_SENSOR_COUNT];
	/*
	 * Per-sensor outcome. Initialization is deliberately tolerant: one
	 * absent or faulty sensor must not hide the state of the other four,
	 * which is exactly what an early return does on a board being brought
	 * up. rb_tof_array_init returns 0 if any sensor came up.
	 */
	int status[RB_TOF_SENSOR_COUNT];
	/* Slots rb_tof_array_init brought up for grid ranging (bit n = slot n). */
	uint8_t ranging_mask;
	bool ranging;
};

/* Human-readable name for array->last_step. */
const char *rb_tof_step_name(uint8_t step);

/* Human-readable name for array->kinds[i]. */
const char *rb_tof_kind_name(uint8_t kind);

/*
 * Find which ADS7128 GPO channel gates an unclaimed sensor.
 *
 * Call after rb_tof_array_init, when every sensor that came up has already been
 * moved off the default address and 0x29 is therefore free. Each candidate
 * channel is released on its own, probed at 0x29, and driven straight back
 * down, so nothing is left enabled. Channels already assigned to a sensor that
 * initialized are skipped rather than disturbed.
 *
 * Returns a bitmask of channels where a sensor answered.
 */
int rb_tof_scan_xshut(struct rb_tof_array *array, uint8_t *found_mask);

/*
 * Drive every ADS7128 GPO channel low, holding all XSHUT-gated sensors in
 * reset. Any VL53 part still answering afterwards is wired directly to the bus,
 * which is the only way to tell a directly-wired sensor from one that merely
 * happened to be released when the scan ran.
 */
int rb_tof_hold_all(const struct device *i2c);

/*
 * Map the ToF topology from scratch, making no assumption about channels.
 *
 * For each ADS7128 GPO channel 0..7: drive every channel low, release only that
 * one, and see what appears at the default address. Toggling XSHUT resets a
 * sensor, so it returns to 0x29 even if a previous run readdressed it, which
 * makes each channel testable in isolation. kinds[channel] is the detected part
 * or RB_TOF_KIND_NONE.
 *
 * direct_kind receives whatever still answers with every channel held low --
 * a sensor wired straight to the bus. direct_address receives its address.
 */
int rb_tof_map(
	const struct device *i2c,
	uint8_t kinds[8],
	uint8_t *direct_kind,
	uint8_t *direct_address);

/*
 * Read the identity registers of all three VL53 families at one address.
 *
 * The parts are not interchangeable and do not share an ID register, so a
 * device that ACKs its address says nothing about which sensor it is. Each
 * out-parameter is set to the raw reading, or left untouched if that family's
 * read failed. Expected values:
 *
 *   l5cx  0xF002   VL53L5CX  (page 0, regs 0x0000/0x0001)
 *   l1x   0xEACC   VL53L1X   (regs 0x010F/0x0110)
 *   l0x   0xEE     VL53L0X   (reg 0xC0)
 */
int rb_tof_identify(
	const struct device *i2c,
	uint8_t address,
	uint16_t *l5cx,
	uint16_t *l1x,
	uint8_t *l0x);

/*
 * Timestamp is host monotonic time immediately after the frame transfer.
 * It is not yet a sensor exposure timestamp. Distances are millimeters and
 * zones are row-major. Only rows * columns entries are valid.
 */
struct rb_tof_frame {
	uint64_t host_timestamp_us;
	uint32_t sequence;
	uint16_t identity;
	uint8_t sensor_index;
	uint8_t address;
	uint8_t rows;
	uint8_t columns;
	uint8_t zone_count;
	uint8_t target_count[RB_TOF_MAX_ZONES];
	uint8_t target_status[RB_TOF_MAX_ZONES];
	int16_t distance_mm[RB_TOF_MAX_ZONES];
};

/*
 * Hold every configured sensor in reset, then enable and assign addresses
 * cumulatively. When ranging support is compiled in, initialize the ULD for
 * all four devices after their identities and final addresses are confirmed.
 */
int rb_tof_array_init(
	struct rb_tof_array *array,
	const struct rb_tof_array_config *config);

/* Probe one sensor at its assigned address and return its raw 16-bit ID. */
int rb_tof_probe(
	const struct rb_tof_array *array,
	uint8_t sensor_index,
	uint16_t *identity);

/* Fetch one ranging grid. Returns -ENOTSUP in address-only builds. */
int rb_tof_fetch(
	struct rb_tof_array *array,
	uint8_t sensor_index,
	struct rb_tof_frame *frame);

bool rb_tof_ranging_available(void);

/*
 * Whether slot sensor_index can be ranged: it came up as a VL53L5CX and the
 * board declared a ranging device for it. Always false without
 * CONFIG_RB_VL53L5CX_RANGING.
 */
bool rb_tof_ranging_sensor(const struct rb_tof_array *array, uint8_t sensor_index);

#ifdef __cplusplus
}
#endif

#endif /* RISKYBIRD_TOF_H_ */
