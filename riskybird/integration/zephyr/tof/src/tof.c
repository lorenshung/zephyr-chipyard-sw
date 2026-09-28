/* SPDX-License-Identifier: Apache-2.0 */

#include <riskybird/tof.h>

#include <errno.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#if defined(CONFIG_RB_VL53L5CX_RANGING)
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/vl53l5cx.h>
#endif

#define ADS7128_I2C_ADDR 0x17U
#define ADS7128_CMD_REG_WRITE 0x08U
#define ADS7128_CMD_REG_READ 0x10U
#define ADS7128_PIN_CFG 0x05U
#define ADS7128_GPIO_CFG 0x07U
#define ADS7128_GPO_DRIVE_CFG 0x09U
#define ADS7128_GPO_VALUE 0x0BU

#define VL53L5CX_DEFAULT_ADDR 0x29U
#define VL53L5CX_PAGE_SELECT 0x7FFFU
#define VL53L5CX_I2C_ADDRESS 0x0004U
#define VL53L5CX_DEVICE_ID 0x0000U
#define VL53L5CX_REVISION_ID 0x0001U

/* VL53L1X: 16-bit register addressing, like the L5CX but unpaged. */
#define VL53L1X_MODEL_ID 0x010FU
#define VL53L1X_I2C_ADDRESS 0x0001U

static int ads7128_read(
	const struct device *i2c,
	uint8_t reg,
	uint8_t *value)
{
	uint8_t command[2] = { ADS7128_CMD_REG_READ, reg };

	return i2c_write_read(
		i2c,
		ADS7128_I2C_ADDR,
		command,
		sizeof(command),
		value,
		1);
}

static int ads7128_write(
	const struct device *i2c,
	uint8_t reg,
	uint8_t value)
{
	uint8_t command[3] = { ADS7128_CMD_REG_WRITE, reg, value };

	return i2c_write(i2c, command, sizeof(command), ADS7128_I2C_ADDR);
}

static int ads7128_set_bits(
	const struct device *i2c,
	uint8_t reg,
	uint8_t mask)
{
	uint8_t value;
	int result = ads7128_read(i2c, reg, &value);

	if (result != 0) {
		return result;
	}
	return ads7128_write(i2c, reg, (uint8_t)(value | mask));
}

static uint8_t sensor_count(const struct rb_tof_array *array)
{
	uint8_t count = array->config.sensor_count;

	return (count == 0U || count > RB_TOF_SENSOR_COUNT)
		       ? RB_TOF_SENSOR_COUNT
		       : count;
}

static int rb_tof_fail(
	struct rb_tof_array *array,
	enum rb_tof_step step,
	uint8_t sensor_index,
	int error)
{
	array->last_step = (uint8_t)step;
	array->last_sensor = sensor_index;
	array->last_error = error;
	return error;
}

const char *rb_tof_step_name(uint8_t step)
{
	switch (step) {
	case RB_TOF_STEP_NONE: return "none";
	case RB_TOF_STEP_PIN_CFG: return "ads7128-pin-cfg";
	case RB_TOF_STEP_GPIO_CFG: return "ads7128-gpio-cfg";
	case RB_TOF_STEP_DRIVE_CFG: return "ads7128-drive-cfg";
	case RB_TOF_STEP_GPO_READ: return "ads7128-gpo-read";
	case RB_TOF_STEP_GPO_WRITE: return "ads7128-gpo-write";
	case RB_TOF_STEP_DIRECT_PROBE: return "direct-probe-0x29";
	case RB_TOF_STEP_DIRECT_READDRESS: return "direct-readdress";
	case RB_TOF_STEP_ENABLE: return "xshut-release";
	case RB_TOF_STEP_WAIT: return "wait-for-sensor";
	case RB_TOF_STEP_READDRESS: return "readdress";
	case RB_TOF_STEP_VERIFY: return "verify";
	default: return "unknown";
	}
}

static int prepare_xshut(struct rb_tof_array *array)
{
	uint8_t mask = 0U;
	int result;

	for (uint8_t index = 0U; index < sensor_count(array); index++) {
		uint8_t channel = array->config.xshut_channels[index];

		if (channel == RB_TOF_XSHUT_DIRECT) {
			continue;
		}
		mask |= (uint8_t)(1U << channel);
	}

	result = ads7128_set_bits(array->config.i2c, ADS7128_PIN_CFG, mask);
	if (result != 0) {
		return rb_tof_fail(array, RB_TOF_STEP_PIN_CFG, 0xFFU, result);
	}
	result = ads7128_set_bits(array->config.i2c, ADS7128_GPIO_CFG, mask);
	if (result != 0) {
		return rb_tof_fail(array, RB_TOF_STEP_GPIO_CFG, 0xFFU, result);
	}
	result = ads7128_set_bits(
		array->config.i2c,
		ADS7128_GPO_DRIVE_CFG,
		mask);
	if (result != 0) {
		return rb_tof_fail(array, RB_TOF_STEP_DRIVE_CFG, 0xFFU, result);
	}

	/*
	 * Drive every channel low, not just the configured ones.
	 *
	 * Preserving unrelated outputs sounds safer but is not: a channel left
	 * high from a previous run releases a sensor that nothing here is
	 * tracking, and it sits on the default address. The first configured
	 * sensor to be released then collides with it, and the sequence
	 * readdresses whichever answers first -- so sensors appear at the wrong
	 * indices and a channel that gates nothing looks populated. Every ToF
	 * on this board is behind this expander, so holding all eight down is
	 * the only well-defined starting state.
	 */
	result = ads7128_set_bits(array->config.i2c, ADS7128_PIN_CFG, 0xFFU);
	if (result != 0) {
		return rb_tof_fail(array, RB_TOF_STEP_PIN_CFG, 0xFFU, result);
	}
	result = ads7128_set_bits(array->config.i2c, ADS7128_GPIO_CFG, 0xFFU);
	if (result != 0) {
		return rb_tof_fail(array, RB_TOF_STEP_GPIO_CFG, 0xFFU, result);
	}
	result = ads7128_set_bits(
		array->config.i2c, ADS7128_GPO_DRIVE_CFG, 0xFFU);
	if (result != 0) {
		return rb_tof_fail(array, RB_TOF_STEP_DRIVE_CFG, 0xFFU, result);
	}
	array->gpo_shadow = 0U;
	result = ads7128_write(
		array->config.i2c,
		ADS7128_GPO_VALUE,
		array->gpo_shadow);
	if (result != 0) {
		return rb_tof_fail(array, RB_TOF_STEP_GPO_WRITE, 0xFFU, result);
	}
	k_msleep(20);
	return 0;
}

static int enable_sensor(struct rb_tof_array *array, uint8_t sensor_index)
{
	array->gpo_shadow |=
		(uint8_t)(1U << array->config.xshut_channels[sensor_index]);
	int result = ads7128_write(
		array->config.i2c,
		ADS7128_GPO_VALUE,
		array->gpo_shadow);

	if (result == 0) {
		k_msleep(100);
	}
	return result;
}

/* Drive one XSHUT line back low, so a sensor that failed cannot squat on 0x29. */
static void disable_sensor(struct rb_tof_array *array, uint8_t sensor_index)
{
	array->gpo_shadow &= (uint8_t)~(
		1U << array->config.xshut_channels[sensor_index]);
	(void)ads7128_write(
		array->config.i2c,
		ADS7128_GPO_VALUE,
		array->gpo_shadow);
	k_msleep(10);
}

static int vl53l5cx_write_u8(
	const struct device *i2c,
	uint8_t address,
	uint16_t reg,
	uint8_t value)
{
	uint8_t command[3] = {
		(uint8_t)(reg >> 8),
		(uint8_t)(reg & 0xFFU),
		value,
	};

	return i2c_write(i2c, command, sizeof(command), address);
}

static int vl53l5cx_read_u8(
	const struct device *i2c,
	uint8_t address,
	uint16_t reg,
	uint8_t *value)
{
	uint8_t command[2] = {
		(uint8_t)(reg >> 8),
		(uint8_t)(reg & 0xFFU),
	};

	return i2c_write_read(
		i2c,
		address,
		command,
		sizeof(command),
		value,
		1);
}

/*
 * Identify whichever VL53 part answers at this address.
 *
 * The L5CX keeps its ID behind a page select at 0x0000/0x0001; the L1X exposes
 * a model ID at 0x010F/0x0110 with no paging. Both use 16-bit register
 * addressing, so the same read primitive serves, but the answers do not
 * overlap and a wrong guess reads meaningless bytes rather than failing -- the
 * L1X reads as 0x0129 through the L5CX's registers. Detect, never assume.
 */
static int detect_kind(
	const struct device *i2c,
	uint8_t address,
	uint8_t *kind,
	uint16_t *identity)
{
	uint8_t high;
	uint8_t low;

	/* VL53L1X first: unpaged, so it cannot disturb an L5CX's page state. */
	if (vl53l5cx_read_u8(i2c, address, VL53L1X_MODEL_ID, &high) == 0 &&
	    vl53l5cx_read_u8(i2c, address, VL53L1X_MODEL_ID + 1U, &low) == 0) {
		uint16_t raw = (uint16_t)(((uint16_t)high << 8) | low);

		if (raw == RB_TOF_VL53L1X_ID) {
			if (kind != NULL) {
				*kind = RB_TOF_KIND_VL53L1X;
			}
			if (identity != NULL) {
				*identity = raw;
			}
			return 0;
		}
	}

	if (vl53l5cx_write_u8(i2c, address, VL53L5CX_PAGE_SELECT, 0x00U) == 0 &&
	    vl53l5cx_read_u8(i2c, address, VL53L5CX_DEVICE_ID, &high) == 0 &&
	    vl53l5cx_read_u8(i2c, address, VL53L5CX_REVISION_ID, &low) == 0) {
		uint16_t raw = (uint16_t)(((uint16_t)high << 8) | low);

		(void)vl53l5cx_write_u8(
			i2c, address, VL53L5CX_PAGE_SELECT, 0x02U);
		if (raw == RB_TOF_VL53L5CX_ID) {
			if (kind != NULL) {
				*kind = RB_TOF_KIND_VL53L5CX;
			}
			if (identity != NULL) {
				*identity = raw;
			}
			return 0;
		}
		if (identity != NULL) {
			*identity = raw;
		}
	}

	if (kind != NULL) {
		*kind = RB_TOF_KIND_NONE;
	}
	return -ENODEV;
}

static int probe_address(
	const struct device *i2c,
	uint8_t address,
	uint16_t *identity)
{
	return detect_kind(i2c, address, NULL, identity);
}

static int change_address(
	const struct device *i2c,
	uint8_t kind,
	uint8_t old_address,
	uint8_t new_address)
{
	int result;

	if (kind == RB_TOF_KIND_VL53L1X) {
		/* One unpaged 8-bit write; the device answers at the new
		 * address immediately afterwards. */
		result = vl53l5cx_write_u8(
			i2c, old_address, VL53L1X_I2C_ADDRESS, new_address);
		if (result != 0) {
			return result;
		}
		k_msleep(2);
		return 0;
	}

	result = vl53l5cx_write_u8(
		i2c,
		old_address,
		VL53L5CX_PAGE_SELECT,
		0x00U);

	if (result != 0) {
		return result;
	}
	result = vl53l5cx_write_u8(
		i2c,
		old_address,
		VL53L5CX_I2C_ADDRESS,
		new_address);
	if (result != 0) {
		return result;
	}
	k_msleep(2);

	/* The new address may take effect before the page restore. */
	result = vl53l5cx_write_u8(
		i2c,
		new_address,
		VL53L5CX_PAGE_SELECT,
		0x02U);
	if (result != 0) {
		(void)vl53l5cx_write_u8(
			i2c,
			old_address,
			VL53L5CX_PAGE_SELECT,
			0x02U);
	}
	return 0;
}

const char *rb_tof_kind_name(uint8_t kind)
{
	switch (kind) {
	case RB_TOF_KIND_VL53L5CX: return "VL53L5CX";
	case RB_TOF_KIND_VL53L1X: return "VL53L1X";
	default: return "none";
	}
}

/*
 * How long to wait for a sensor to answer after its XSHUT is released.
 *
 * One constant for every release path, because they are all asking the same
 * physical question and they used to disagree: wait_for_sensor allowed 500 ms
 * and rb_tof_map 600 ms, so the topology map could call a sensor absent that
 * init had just brought up.
 *
 * 600 ms was measured to be too short on riskybird v3. rb_tof_map reported
 * `channel 1 -> none` for a VL53L5CX that rb_tof_array_init had found and
 * readdressed to 0x31 seconds earlier, in the same run -- a false negative on
 * a known-good part, from a scan whose whole purpose is deciding whether a
 * header is empty.
 *
 * Raising it alone does NOT fix that false negative -- measured: at 5000 ms the
 * map still reported `channel 1 -> none`. The cause was the probed ADDRESS, not
 * the deadline (see rb_tof_map). 1500 ms is kept anyway because the two paths
 * disagreeing was its own bug, and because the L5CX is genuinely slower to
 * answer than the L1X.
 */
#ifndef RB_TOF_RELEASE_TIMEOUT_MS
#define RB_TOF_RELEASE_TIMEOUT_MS 1500
#endif

/*
 * Bitmap of which addresses in the assignable window currently answer.
 * Bit n corresponds to VL53L5CX_DEFAULT_ADDR + n (0x29..0x35).
 */
static uint16_t window_mask(const struct device *i2c)
{
	uint16_t seen = 0U;

	for (uint8_t addr = VL53L5CX_DEFAULT_ADDR; addr <= 0x35U; addr++) {
		if (probe_address(i2c, addr, NULL) == 0) {
			seen |= (uint16_t)(1U << (addr - VL53L5CX_DEFAULT_ADDR));
		}
	}
	return seen;
}

/*
 * Wait for a NEW VL53 part to appear anywhere in the window, relative to what
 * was already answering before the channel was released.
 *
 * Differential rather than absolute, because sensors on OTHER channels stay
 * powered during the scan and keep answering at their readdressed values. An
 * absolute "does anything answer in the window" test reports every channel as
 * populated -- measured: it returned mask 0xbd (ch0 ch2 ch3 ch4 ch5 ch7) on a
 * board where those headers gate nothing, because the already-powered parts at
 * 0x31 and 0x33 answered every time. Probing only 0x29 has the opposite
 * failure and misses sensors that kept an address. Only the difference
 * attributes a sensor to the channel actually being released.
 */
static int wait_for_new(const struct device *i2c, uint16_t before)
{
	int64_t start = k_uptime_get();

	do {
		if ((window_mask(i2c) & ~before) != 0U) {
			return 0;
		}
		k_msleep(10);
	} while ((k_uptime_get() - start) < RB_TOF_RELEASE_TIMEOUT_MS);

	return -ETIMEDOUT;
}

static int wait_for_sensor(
	const struct device *i2c,
	uint8_t default_address,
	uint8_t assigned_address)
{
	int64_t start = k_uptime_get();

	do {
		if (probe_address(i2c, assigned_address, NULL) == 0 ||
		    probe_address(i2c, default_address, NULL) == 0) {
			return 0;
		}
		k_msleep(10);
	} while ((k_uptime_get() - start) < RB_TOF_RELEASE_TIMEOUT_MS);

	return -ETIMEDOUT;
}

/*
 * A directly-wired sensor cannot be reset, so its address survives until power
 * is lost. Probe the assigned address first so a warm restart is a no-op, and
 * fall back to the default address on a cold one.
 */
static int adopt_direct(struct rb_tof_array *array, uint8_t sensor_index)
{
	const struct device *i2c = array->config.i2c;
	uint8_t assigned = array->config.addresses[sensor_index];
	int result;

	uint8_t kind = RB_TOF_KIND_NONE;

	if (detect_kind(i2c, assigned, &kind,
			&array->identities[sensor_index]) == 0) {
		array->kinds[sensor_index] = kind;
		return 0;
	}
	result = detect_kind(
		i2c,
		VL53L5CX_DEFAULT_ADDR,
		&kind,
		&array->identities[sensor_index]);
	if (result != 0) {
		return rb_tof_fail(
			array, RB_TOF_STEP_DIRECT_PROBE, sensor_index, result);
	}
	array->kinds[sensor_index] = kind;
	result = change_address(i2c, kind, VL53L5CX_DEFAULT_ADDR, assigned);
	if (result != 0) {
		return rb_tof_fail(
			array, RB_TOF_STEP_DIRECT_READDRESS, sensor_index,
			result);
	}
	k_msleep(10);
	result = detect_kind(
		i2c, assigned, &kind, &array->identities[sensor_index]);
	if (result != 0) {
		return rb_tof_fail(
			array, RB_TOF_STEP_VERIFY, sensor_index, result);
	}
	array->kinds[sensor_index] = kind;
	return 0;
}

static int validate_config(const struct rb_tof_array_config *config)
{
	if (config == NULL || config->i2c == NULL) {
		return -EINVAL;
	}
	if (!device_is_ready(config->i2c)) {
		return -ENODEV;
	}

	uint8_t count = (config->sensor_count == 0U ||
			 config->sensor_count > RB_TOF_SENSOR_COUNT)
				? RB_TOF_SENSOR_COUNT
				: config->sensor_count;

	for (uint8_t index = 0U; index < count; index++) {
		uint8_t channel = config->xshut_channels[index];
		bool direct = channel == RB_TOF_XSHUT_DIRECT;

		if ((!direct && channel > 7U) ||
		    config->addresses[index] < 0x08U ||
		    config->addresses[index] > 0x77U) {
			return -EINVAL;
		}
		/*
		 * A direct sensor has no XSHUT line, so several may share the
		 * sentinel; their addresses must still be distinct.
		 */
		if (config->addresses[index] == VL53L5CX_DEFAULT_ADDR) {
			return -EINVAL;
		}
		for (uint8_t prior = 0U; prior < index; prior++) {
			if ((!direct &&
			     channel == config->xshut_channels[prior]) ||
			    config->addresses[index] == config->addresses[prior]) {
				return -EINVAL;
			}
		}
	}
	return 0;
}

int rb_tof_array_init(
	struct rb_tof_array *array,
	const struct rb_tof_array_config *config)
{
	int result;

	if (array == NULL) {
		return -EINVAL;
	}
	result = validate_config(config);
	if (result != 0) {
		return result;
	}

	memset(array, 0, sizeof(*array));
	array->config = *config;
	result = prepare_xshut(array);
	if (result != 0) {
		return result;
	}

	/*
	 * Direct sensors first, and only while every XSHUT sensor is still held
	 * down. A direct sensor sits at the default address permanently, so
	 * releasing an XSHUT sensor before moving it would put two devices on
	 * 0x29 at once.
	 */
	for (uint8_t index = 0U; index < sensor_count(array); index++) {
		if (array->config.xshut_channels[index] !=
		    RB_TOF_XSHUT_DIRECT) {
			continue;
		}
		result = adopt_direct(array, index);
		array->status[index] = result;
		if (result == 0) {
			array->initialized_sensors++;
		}
	}

	/*
	 * Keep every successfully assigned sensor enabled. Dropping XSHUT would
	 * reset its volatile address to 0x29 and collide with the next sensor.
	 * A sensor that fails is driven back down for the same reason: left
	 * released and unassigned it would sit on 0x29 and break every sensor
	 * after it.
	 */
	for (uint8_t index = 0U; index < sensor_count(array); index++) {
		uint8_t assigned = array->config.addresses[index];

		if (array->config.xshut_channels[index] ==
		    RB_TOF_XSHUT_DIRECT) {
			continue;
		}

		result = enable_sensor(array, index);
		if (result != 0) {
			array->status[index] = rb_tof_fail(
				array, RB_TOF_STEP_ENABLE, index, result);
			continue;
		}
		result = wait_for_sensor(
			array->config.i2c,
			VL53L5CX_DEFAULT_ADDR,
			assigned);
		if (result != 0) {
			array->status[index] = rb_tof_fail(
				array, RB_TOF_STEP_WAIT, index, result);
			disable_sensor(array, index);
			continue;
		}
		if (detect_kind(array->config.i2c, assigned,
				&array->kinds[index],
				&array->identities[index]) != 0) {
			result = detect_kind(
				array->config.i2c,
				VL53L5CX_DEFAULT_ADDR,
				&array->kinds[index],
				&array->identities[index]);
			if (result != 0) {
				array->status[index] = rb_tof_fail(
					array, RB_TOF_STEP_WAIT, index, result);
				disable_sensor(array, index);
				continue;
			}
			result = change_address(
				array->config.i2c,
				array->kinds[index],
				VL53L5CX_DEFAULT_ADDR,
				assigned);
			if (result != 0) {
				array->status[index] = rb_tof_fail(
					array, RB_TOF_STEP_READDRESS, index,
					result);
				disable_sensor(array, index);
				continue;
			}
			k_msleep(10);
		}
		result = detect_kind(
			array->config.i2c,
			assigned,
			&array->kinds[index],
			&array->identities[index]);
		if (result != 0) {
			array->status[index] = rb_tof_fail(
				array, RB_TOF_STEP_VERIFY, index, result);
			disable_sensor(array, index);
			continue;
		}
		array->status[index] = 0;
		array->initialized_sensors++;
	}

	if (array->initialized_sensors == 0U) {
		return array->last_error != 0 ? array->last_error : -ENODEV;
	}

#if defined(CONFIG_RB_VL53L5CX_RANGING)
	/*
	 * Only the populated slots that came up as a VL53L5CX, and only those
	 * the board declared a ranging device for. This used to walk all
	 * RB_TOF_SENSOR_COUNT slots and fail on the first NULL or unready one,
	 * which made a board with fewer than five VL53L5CX -- every board, since
	 * the down sensor is a VL53L1X -- unable to range at all, and handed the
	 * VL53L1X to the VL53L5CX firmware upload. Same tolerance as the address
	 * pass above: one bad sensor is recorded, not fatal.
	 */
	for (uint8_t index = 0U; index < sensor_count(array); index++) {
		const struct device *sensor = array->config.sensors[index];

		if (array->status[index] != 0 ||
		    array->kinds[index] != RB_TOF_KIND_VL53L5CX ||
		    sensor == NULL) {
			continue;
		}
		result = device_is_ready(sensor) ? vl53l5cx_reinit(sensor)
						 : -ENODEV;
		if (result != 0) {
			array->status[index] = result;
			continue;
		}
		array->ranging_mask |= (uint8_t)BIT(index);
	}
	array->ranging = array->ranging_mask != 0U;
	if (!array->ranging) {
		return -ENODEV;
	}
#endif

	return 0;
}

int rb_tof_hold_all(const struct device *i2c)
{
	int result;

	if (i2c == NULL) {
		return -EINVAL;
	}
	result = ads7128_set_bits(i2c, ADS7128_PIN_CFG, 0xFFU);
	if (result != 0) {
		return result;
	}
	result = ads7128_set_bits(i2c, ADS7128_GPIO_CFG, 0xFFU);
	if (result != 0) {
		return result;
	}
	result = ads7128_set_bits(i2c, ADS7128_GPO_DRIVE_CFG, 0xFFU);
	if (result != 0) {
		return result;
	}
	result = ads7128_write(i2c, ADS7128_GPO_VALUE, 0x00U);
	if (result == 0) {
		k_msleep(50);
	}
	return result;
}

int rb_tof_map(
	const struct device *i2c,
	uint8_t kinds[8],
	uint8_t *direct_kind,
	uint8_t *direct_address)
{
	int result;

	if (i2c == NULL || kinds == NULL) {
		return -EINVAL;
	}

	result = rb_tof_hold_all(i2c);
	if (result != 0) {
		return result;
	}
	k_msleep(50);

	/* With everything held down, anything left is wired directly. */
	if (direct_kind != NULL) {
		*direct_kind = RB_TOF_KIND_NONE;
	}
	for (uint8_t addr = 0x08U; addr <= 0x77U; addr++) {
		uint8_t kind = RB_TOF_KIND_NONE;

		if (detect_kind(i2c, addr, &kind, NULL) == 0) {
			if (direct_kind != NULL) {
				*direct_kind = kind;
			}
			if (direct_address != NULL) {
				*direct_address = addr;
			}
			break;
		}
	}

	for (uint8_t channel = 0U; channel < 8U; channel++) {
		uint8_t bit = (uint8_t)(1U << channel);

		kinds[channel] = RB_TOF_KIND_NONE;

		/* Every channel low, then only this one released. */
		(void)ads7128_write(i2c, ADS7128_GPO_VALUE, 0x00U);
		k_msleep(30);
		if (ads7128_write(i2c, ADS7128_GPO_VALUE, bit) != 0) {
			continue;
		}

		/*
		 * Probe the whole assignable window, not just 0x29.
		 *
		 * This function's contract used to read "toggling XSHUT resets
		 * a sensor, so it returns to 0x29 even if a previous run
		 * readdressed it". On riskybird v3 that is false, and it is
		 * the bug that made two fitted sensors look like empty
		 * headers. Measured: driving every channel low powers the ch1
		 * VL53L5CX off (it stops answering at 0x31, -ENODEV), but
		 * releasing the channel brings it back at 0x31 -- its
		 * readdressed value -- and never at 0x29. Probing only the
		 * default address therefore reported `channel 1 -> none` for a
		 * part that rb_tof_array_init had identified seconds earlier
		 * in the same run, and raising the deadline to 5000 ms did not
		 * change it.
		 *
		 * 0x29..0x35 covers the power-on default plus every address
		 * this driver hands out, so a sensor is found whether it reset
		 * or kept what it was given.
		 */
		int64_t start = k_uptime_get();

		do {
			bool seen = false;

			for (uint8_t addr = VL53L5CX_DEFAULT_ADDR;
			     addr <= 0x35U; addr++) {
				if (detect_kind(i2c, addr,
						&kinds[channel], NULL) == 0) {
					seen = true;
					break;
				}
			}
			if (seen) {
				break;
			}
			k_msleep(20);
		} while ((k_uptime_get() - start) < RB_TOF_RELEASE_TIMEOUT_MS);
	}

	(void)ads7128_write(i2c, ADS7128_GPO_VALUE, 0x00U);
	k_msleep(20);
	return 0;
}

int rb_tof_scan_xshut(struct rb_tof_array *array, uint8_t *found_mask)
{
	uint8_t mask = 0U;
	uint8_t claimed = 0U;

	if (array == NULL) {
		return -EINVAL;
	}

	for (uint8_t index = 0U; index < sensor_count(array); index++) {
		uint8_t channel = array->config.xshut_channels[index];

		if (channel != RB_TOF_XSHUT_DIRECT && array->status[index] == 0) {
			claimed |= (uint8_t)(1U << channel);
		}
	}

	for (uint8_t channel = 0U; channel < 8U; channel++) {
		uint8_t bit = (uint8_t)(1U << channel);

		if ((claimed & bit) != 0U) {
			continue;
		}

		/* Baseline with this channel still low: everything answering
		 * now belongs to some other channel and must not be credited
		 * to this one. */
		uint16_t before = window_mask(array->config.i2c);

		/* Make sure the channel really is a driven output first. */
		(void)ads7128_set_bits(array->config.i2c, ADS7128_PIN_CFG, bit);
		(void)ads7128_set_bits(array->config.i2c, ADS7128_GPIO_CFG, bit);
		(void)ads7128_set_bits(
			array->config.i2c, ADS7128_GPO_DRIVE_CFG, bit);

		array->gpo_shadow |= bit;
		if (ads7128_write(
			    array->config.i2c,
			    ADS7128_GPO_VALUE,
			    array->gpo_shadow) == 0) {
			/*
			 * Poll, and poll the whole window, and only count what
			 * is NEW. This scan decides whether a header is empty,
			 * so each of those three has bitten:
			 *
			 *  - a single probe after a fixed delay false-negatives
			 *    any sensor slower than the delay;
			 *  - probing only 0x29 misses a sensor that kept an
			 *    address from an earlier run, because toggling
			 *    XSHUT does not return these parts to the default;
			 *  - counting any answer in the window credits this
			 *    channel with sensors belonging to other channels,
			 *    which reported mask 0xbd on a board whose spare
			 *    headers gate nothing.
			 */
			if (wait_for_new(array->config.i2c, before) == 0) {
				mask |= bit;
			}
		}

		array->gpo_shadow &= (uint8_t)~bit;
		(void)ads7128_write(
			array->config.i2c,
			ADS7128_GPO_VALUE,
			array->gpo_shadow);
		k_msleep(10);
	}

	if (found_mask != NULL) {
		*found_mask = mask;
	}
	return 0;
}

int rb_tof_identify(
	const struct device *i2c,
	uint8_t address,
	uint16_t *l5cx,
	uint16_t *l1x,
	uint8_t *l0x)
{
	uint8_t high;
	uint8_t low;
	int found = 0;

	if (i2c == NULL) {
		return -EINVAL;
	}

	/* VL53L5CX: paged, 8-bit reads at 0x0000/0x0001. */
	if (l5cx != NULL &&
	    vl53l5cx_write_u8(i2c, address, VL53L5CX_PAGE_SELECT, 0x00U) == 0 &&
	    vl53l5cx_read_u8(i2c, address, VL53L5CX_DEVICE_ID, &high) == 0 &&
	    vl53l5cx_read_u8(i2c, address, VL53L5CX_REVISION_ID, &low) == 0) {
		*l5cx = (uint16_t)(((uint16_t)high << 8) | low);
		found++;
	}
	(void)vl53l5cx_write_u8(i2c, address, VL53L5CX_PAGE_SELECT, 0x02U);

	/* VL53L1X: 16-bit register addressing, model ID at 0x010F. */
	if (l1x != NULL &&
	    vl53l5cx_read_u8(i2c, address, 0x010FU, &high) == 0 &&
	    vl53l5cx_read_u8(i2c, address, 0x0110U, &low) == 0) {
		*l1x = (uint16_t)(((uint16_t)high << 8) | low);
		found++;
	}

	/* VL53L0X: 8-bit register addressing, model ID at 0xC0. */
	if (l0x != NULL) {
		uint8_t reg = 0xC0U;

		if (i2c_write_read(i2c, address, &reg, 1, &high, 1) == 0) {
			*l0x = high;
			found++;
		}
	}

	return found > 0 ? 0 : -ENODEV;
}

int rb_tof_probe(
	const struct rb_tof_array *array,
	uint8_t sensor_index,
	uint16_t *identity)
{
	if (array == NULL || sensor_index >= RB_TOF_SENSOR_COUNT) {
		return -EINVAL;
	}
	return probe_address(
		array->config.i2c,
		array->config.addresses[sensor_index],
		identity);
}

int rb_tof_fetch(
	struct rb_tof_array *array,
	uint8_t sensor_index,
	struct rb_tof_frame *frame)
{
	if (array == NULL || frame == NULL ||
	    sensor_index >= RB_TOF_SENSOR_COUNT) {
		return -EINVAL;
	}

#if defined(CONFIG_RB_VL53L5CX_RANGING)
	if (!array->ranging) {
		return -EAGAIN;
	}
	if (!rb_tof_ranging_sensor(array, sensor_index)) {
		return -ENODEV;
	}

	const struct device *sensor = array->config.sensors[sensor_index];
	struct vl53l5cx_grid grid;
	int result = sensor_sample_fetch(sensor);

	if (result != 0) {
		return result;
	}
	result = vl53l5cx_get_grid(sensor, &grid);
	if (result != 0) {
		return result;
	}

	memset(frame, 0, sizeof(*frame));
	frame->host_timestamp_us =
		k_ticks_to_us_floor64(k_uptime_ticks());
	frame->sequence = array->sequence++;
	frame->identity = RB_TOF_VL53L5CX_ID;
	frame->sensor_index = sensor_index;
	frame->address = array->config.addresses[sensor_index];
	frame->zone_count =
		grid.resolution == 64U ? 64U : 16U;
	frame->rows = frame->zone_count == 64U ? 8U : 4U;
	frame->columns = frame->rows;

	for (uint8_t zone = 0U; zone < frame->zone_count; zone++) {
		frame->target_count[zone] = grid.nb_target_detected[zone];
		frame->target_status[zone] = grid.target_status[zone];
		frame->distance_mm[zone] = grid.distance_mm[zone];
	}
	return 0;
#else
	(void)sensor_index;
	return -ENOTSUP;
#endif
}

bool rb_tof_ranging_sensor(const struct rb_tof_array *array, uint8_t sensor_index)
{
	return array != NULL && sensor_index < RB_TOF_SENSOR_COUNT &&
	       (array->ranging_mask & BIT(sensor_index)) != 0U;
}

bool rb_tof_ranging_available(void)
{
#if defined(CONFIG_RB_VL53L5CX_RANGING)
	return true;
#else
	return false;
#endif
}
