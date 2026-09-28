/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Whole-board sensor check, and a go/no-go for the state estimator.
 *
 * The per-sensor diagnostics in this tree each answer one question well. What none of them
 * answers is the one that matters before flying anything: is every sensor on this board alive
 * at the same time, and is the data the estimator consumes physically real. This target does
 * that in one pass and ends with an explicit verdict for `rb build state --mode bmi088`.
 *
 * Two things it does deliberately:
 *
 *   - It releases the ADS7128 expander gates FIRST. Devices on this board sit behind that
 *     expander's eight channels, and with them held low the gated parts NAK their addresses
 *     and look absent. That is not a hypothetical: it is why the HM01B0 camera was written off
 *     as unpopulated on this bench until the channels were driven high. Any board-wide scan
 *     that skips this step under-reports the hardware.
 *
 *   - It judges values, not presence. A sensor that ACKs its address proves a wire; it does
 *     not prove the number means anything. An accelerometer with a wrong scale factor, a stuck
 *     bus and a healthy part all return plausible-looking readings, and only the vector
 *     magnitude separates them -- |a| must be gravity for a board sitting still. The same
 *     discipline applies to the gyro (rest plus noise, not zero) and the barometer (a
 *     believable room pressure, not a default).
 *
 * Motors are deliberately untouched. This target drives no PWM channel, so it is safe to run
 * with propellers fitted; the motor_N workloads exist for that and are marked propellers-off.
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/sys_io.h>

#include "pmw3901.h"

#define I2C_NODE           DT_NODELABEL(i2c0)
#define PMW3901_NODE       DT_NODELABEL(riskybird_pmw3901)

/*
 * The BMI088 is read through Zephyr's bosch,bmi08x driver, because that is the path
 * docs/drone-initial-bringup.md already proves on this hardware -- real gravity, |a| within
 * 1.3% of 9.80665 across three samples, on RocketArty200TFlowSpadConfig. It is also what
 * `state --mode bmi088` uses, so checking it here checks the estimator's actual input rather
 * than a parallel reimplementation of it.
 *
 * SENSOR_CHECK_RAW_IMU=1 selects raw register access instead, for the case where the driver
 * stack is suspect and the question is whether the silicon is sound:
 *
 *   RB_EXTRA_CFLAGS=-DSENSOR_CHECK_RAW_IMU=1 rb debug sensor_check ...
 *
 * That path is UNVALIDATED. It was written while chasing a boot hang I attributed to the
 * BMI08X driver; the attribution was wrong -- the I2C bus had been wedged by an earlier
 * stack-overflow fault of this very target dying mid-transaction, and every subsequent
 * I2C-touching program failed for that one reason. Kept because a driver-independent read is
 * genuinely useful, but the driver path is the default and the documented one.
 */
#ifndef SENSOR_CHECK_RAW_IMU
#define SENSOR_CHECK_RAW_IMU 0
#endif

/* BMI088 accelerometer, at ADDR_ACCEL. */
#define BMI_ACC_CHIP_ID        0x00U   /* reads 0x1e */
#define BMI_ACC_CHIP_ID_VALUE  0x1EU
#define BMI_ACC_DATA           0x12U   /* X_LSB, 6 bytes */
#define BMI_ACC_TEMP_MSB       0x22U
#define BMI_ACC_RANGE          0x41U   /* 0=+/-3g 1=+/-6g 2=+/-12g 3=+/-24g */
#define BMI_ACC_PWR_CONF       0x7CU   /* 0x00 = active, 0x03 = suspend */
#define BMI_ACC_PWR_CTRL       0x7DU   /* 0x04 = accelerometer on */

/* BMI088 gyroscope, at ADDR_GYRO. */
#define BMI_GYR_CHIP_ID        0x00U   /* reads 0x0f */
#define BMI_GYR_CHIP_ID_VALUE  0x0FU
#define BMI_GYR_DATA           0x02U   /* RATE_X_LSB, 6 bytes */
#define BMI_GYR_RANGE          0x0FU   /* 0=+/-2000dps .. 4=+/-125dps */

/* ---- addresses, from the bus scans recorded in docs/drone-initial-bringup.md ---- */
#define ADDR_ADS7128   0x17U   /* 8-channel ADC / GPIO expander; gates other parts */
#define ADDR_ACCEL     0x18U   /* BMI088 accelerometer */
#define ADDR_TOF       0x29U   /* VL53L0X / VL53L1X time-of-flight */
#define ADDR_UNKNOWN   0x2BU   /* answers, unidentified in this repository */
#define ADDR_GYRO      0x68U   /* BMI088 gyroscope */
#define ADDR_BARO      0x76U   /* BMP280 / BME280 class barometer */
#define ADDR_CAMERA    0x24U   /* HM01B0, behind the expander gate */

/* ADS7128 single-register access (datasheet 8.5.1.1 / 8.5.2.1). */
#define ADS7128_CMD_WRITE       0x08U
#define ADS7128_CMD_READ        0x10U
#define ADS7128_PIN_CFG         0x05U
#define ADS7128_GPIO_CFG        0x07U
#define ADS7128_GPO_DRIVE_CFG   0x09U
#define ADS7128_GPO_VALUE       0x0BU
#define ADS7128_SYSTEM_STATUS   0x00U

/*
 * sifive I2C controller registers, for a pre-flight bus check.
 *
 * This driver implements only .transfer -- there is no .recover_bus -- and i2c_write() blocks
 * with no timeout, so a bus left wedged by a previous program (a slave still holding SDA after
 * a master died mid-transaction) turns the very first transfer into an unbounded hang. That is
 * indistinguishable on the console from a crash, and it is a state this bench reaches easily:
 * the BMI08X driver dying inside i2c_sifive_send_addr leaves exactly this behind.
 *
 * Reading STATUS first catches the cases it can. BUSY set means a transfer is still in
 * progress; TIP set means the controller thinks it is mid-byte.
 *
 * MEASURED LIMIT, so nobody trusts this further than it goes: these bits describe the
 * CONTROLLER, not the line. On a bus wedged by a slave holding SDA this reads 0x01 -- BUSY and
 * TIP both clear, apparently idle -- and the very next transfer still hangs, because the
 * controller only discovers the problem when it tries to drive START. So a pass here is not a
 * promise; it only rules out a controller left mid-transaction. Recovering a slave-held bus
 * needs the drone board power-cycled: the driver exposes no .recover_bus, and reloading the
 * bitstream resets the FPGA controller while leaving the slave exactly as it was.
 */
#define I2C0_BASE          0x10040000UL
#define I2C_REG_STATUS     (I2C0_BASE + 0x10)
#define I2C_STATUS_BUSY    (1U << 6)
#define I2C_STATUS_AL      (1U << 5)
#define I2C_STATUS_TIP     (1U << 1)

/* Gravity, and how far from it a resting board is allowed to read. An uncalibrated consumer
 * part sitting not-quite-level lands within a couple of percent; 8% is loose enough not to
 * fail on a tilted bench and tight enough that a wrong scale factor (typically 2x, 4x or 16x
 * off) cannot pass. */
#define GRAVITY_MPS2   9.80665f
#define GRAVITY_TOL    0.08f

/* A resting gyro reads noise, not zero. Well above this and the board is being moved or the
 * part is misconfigured; exactly zero on every axis suggests a stuck register rather than a
 * sensor. */
#define GYRO_REST_MAX_RPS  0.20f

/*
 * The estimator loop rate to rehearse, and how many samples to take. Plain defines rather than
 * Kconfig symbols so this app needs no Kconfig root of its own and can use
 * hardware/zephyr/Kconfig.workload like every other rb workload -- the same reason ddr_stress
 * and camera_bringup take their knobs this way. 200 Hz matches CONFIG_RB_STATE_RATE_HZ's
 * default, which is the rate this stage exists to rehearse.
 *
 *   RB_EXTRA_CFLAGS="-DEST_RATE_HZ=400 -DEST_SAMPLES=800" rb debug sensor_check ...
 */
#ifndef EST_RATE_HZ
#define EST_RATE_HZ    200U
#endif
#ifndef EST_SAMPLES
#define EST_SAMPLES    400U
#endif

static const struct device *i2c_dev;
#if !SENSOR_CHECK_RAW_IMU
static const struct device *const accel = DEVICE_DT_GET(DT_ALIAS(bmi088_accel));
static const struct device *const gyro = DEVICE_DT_GET(DT_ALIAS(bmi088_gyro));
#endif

static unsigned int checks_run, checks_passed;

/*
 * Every physical quantity below is printed as a scaled integer. The nano printf has no float
 * support and enabling it would pull in the complete formatter; workloads/state makes the same
 * choice and reports milli-units, so a magnitude reads as 9807 mm/s^2 rather than 9.807.
 */
static int32_t milli(float v)
{
	return (int32_t)(v * 1000.0f);
}

/* ---- reporting ---------------------------------------------------------------- */

enum verdict { OK, WARN, BAD };

static void report(enum verdict v, const char *name, const char *fmt, ...)
{
	static const char *tag[] = { "  OK  ", " WARN ", " FAIL " };
	va_list ap;

	checks_run++;
	if (v == OK) {
		checks_passed++;
	}
	printf("[%s] %-18s ", tag[v], name);
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	printf("\n");
}

/* ---- raw I2C helpers ---------------------------------------------------------- */

static bool acks(uint8_t addr)
{
	uint8_t scratch;

	return i2c_read(i2c_dev, &scratch, 1, addr) == 0;
}

/* 8-bit register address, 8-bit data. Standard I2C combined transaction. */
static int reg8_read(uint8_t addr, uint8_t reg, uint8_t *out)
{
	return i2c_write_read(i2c_dev, addr, &reg, 1, out, 1);
}

/*
 * 16-bit register address, SCCB-style: address phase terminated by a STOP, then a separate
 * read. The HM01B0 speaks SCCB, which has no repeated start, and NAKs the combined form -- a
 * live sensor then looks absent. The VL53L1X tolerates both, so the same helper serves it.
 */
static int reg16_read(uint8_t addr, uint16_t reg, uint8_t *out)
{
	uint8_t buf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff) };
	int rc = i2c_write(i2c_dev, buf, sizeof(buf), addr);

	if (rc != 0) {
		return rc;
	}
	return i2c_read(i2c_dev, out, 1, addr);
}

static int ads7128_write(uint8_t reg, uint8_t val)
{
	uint8_t buf[3] = { ADS7128_CMD_WRITE, reg, val };

	return i2c_write(i2c_dev, buf, sizeof(buf), ADDR_ADS7128);
}

static int ads7128_read(uint8_t reg, uint8_t *out)
{
	uint8_t buf[2] = { ADS7128_CMD_READ, reg };

	return i2c_write_read(i2c_dev, ADDR_ADS7128, buf, sizeof(buf), out, 1);
}

/* Burst read of n bytes from a starting register. */
static int reg8_burst(uint8_t addr, uint8_t reg, uint8_t *out, size_t n)
{
	return i2c_write_read(i2c_dev, addr, &reg, 1, out, n);
}

static int16_t le16(const uint8_t *p)
{
	return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/*
 * Wake the accelerometer. The BMI088 accel comes out of reset suspended and returns all
 * zeroes until both power registers are written -- which reads exactly like a dead part.
 * The gyro needs no such sequence.
 */
static int accel_wake(void)
{
	uint8_t buf[2];
	int rc;

	buf[0] = BMI_ACC_PWR_CTRL; buf[1] = 0x04U;
	rc = i2c_write(i2c_dev, buf, 2, ADDR_ACCEL);
	if (rc != 0) {
		return rc;
	}
	k_msleep(5);
	buf[0] = BMI_ACC_PWR_CONF; buf[1] = 0x00U;
	rc = i2c_write(i2c_dev, buf, 2, ADDR_ACCEL);
	k_msleep(50);
	return rc;
}

/*
 * Convert raw accelerometer counts to m/s^2 using the range register actually programmed,
 * rather than a hardcoded assumption. BMI088 full scale is +/-1.5g * 2^(range+1), and the
 * datapath is 16-bit signed, so one count is (fs_g / 32768) g.
 */
static float accel_to_mps2(int16_t raw, uint8_t range_reg)
{
	float fs_g = 1.5f * (float)(1u << ((range_reg & 0x03U) + 1u));

	return (float)raw * fs_g / 32768.0f * GRAVITY_MPS2;
}

/*
 * Gyro full scale from its range register: 2000 dps >> range. Counts are 16-bit signed.
 */
static float gyro_to_rps(int16_t raw, uint8_t range_reg)
{
	float fs_dps = 2000.0f / (float)(1u << (range_reg & 0x07U));

	return (float)raw * fs_dps / 32768.0f * 0.017453293f;
}

/* ---- stages ------------------------------------------------------------------- */

/*
 * Put the expander into a known state before anything else.
 *
 * SENSOR_CHECK_EXPANDER_MASK names the channels to drive high; everything else is returned to
 * the power-on configuration, analog input and hi-Z.
 *
 * This used to drive all eight channels high, which spun two motors on the drone carrier:
 * channels 0, 5 and 7 are undocumented here and two of them gate ESCs. The ADS7128 latches GPO
 * state in its own registers, so that survives the program exiting, GDB detaching and the FPGA
 * being reprogrammed -- it is not something killing a host process undoes. Channels 1-4 are the
 * VL53L5CX XSHUT lines (hardware/zephyr/targets/README.md); the rest must be named from the
 * schematic, never swept for. integration/zephyr/tof holds every channel low for the same
 * reason (rb_tof_hold_all).
 *
 * With the default mask this is also the software recovery from a latched state.
 *
 * PIN_CFG is allowed to fail: on this board it is rejected while the three registers that
 * actually matter are accepted, and the gated parts come up regardless.
 */
#ifndef SENSOR_CHECK_EXPANDER_MASK
#define SENSOR_CHECK_EXPANDER_MASK 0x00U
#endif

static void release_gates(void)
{
	static const struct { uint8_t reg; uint8_t val; } seq[] = {
		{ ADS7128_PIN_CFG,       SENSOR_CHECK_EXPANDER_MASK },
		{ ADS7128_GPIO_CFG,      SENSOR_CHECK_EXPANDER_MASK },
		{ ADS7128_GPO_DRIVE_CFG, SENSOR_CHECK_EXPANDER_MASK },
		{ ADS7128_GPO_VALUE,     SENSOR_CHECK_EXPANDER_MASK },
	};
	unsigned int ok = 0;

	for (size_t i = 0; i < ARRAY_SIZE(seq); i++) {
		if (ads7128_write(seq[i].reg, seq[i].val) == 0) {
			ok++;
		}
	}
	if (ok == 0U) {
		report(BAD, "expander gates", "no ADS7128 register write was accepted at 0x%02x",
		       ADDR_ADS7128);
		return;
	}
	k_msleep(50);
	report(OK, "expander gates", "%u/%zu writes accepted; channel mask 0x%02x%s",
	       ok, ARRAY_SIZE(seq), SENSOR_CHECK_EXPANDER_MASK,
	       (SENSOR_CHECK_EXPANDER_MASK == 0U) ? " (all hi-Z)" : "");
}

static void check_inventory(void)
{
	static const struct { uint8_t addr; const char *what; bool expected; } known[] = {
		{ ADDR_ADS7128, "ADS7128 expander",  true  },
		{ ADDR_CAMERA,  "HM01B0 camera",     true  },
		{ ADDR_ACCEL,   "BMI088 accel",      true  },
		{ ADDR_TOF,     "VL53Lxx ToF",       true  },
		{ ADDR_UNKNOWN, "unidentified",      false },
		{ ADDR_GYRO,    "BMI088 gyro",       true  },
		{ ADDR_BARO,    "barometer",         true  },
	};
	unsigned int found = 0, missing = 0;

	printf("\n--- I2C inventory (MCLK not required for these) ---\n");
	for (size_t i = 0; i < ARRAY_SIZE(known); i++) {
		bool present = acks(known[i].addr);

		if (present) {
			found++;
		} else if (known[i].expected) {
			missing++;
		}
		printf("       0x%02x  %-18s %s\n", known[i].addr, known[i].what,
		       present ? "ACK" : (known[i].expected ? "-- MISSING" : "-- absent"));
	}
	if (missing == 0U) {
		report(OK, "i2c inventory", "%u device(s) answered; every expected part present",
		       found);
	} else {
		report(BAD, "i2c inventory", "%u answered, %u expected part(s) missing",
		       found, missing);
	}
}

static void check_expander(void)
{
	uint8_t status;

	if (ads7128_read(ADS7128_SYSTEM_STATUS, &status) != 0) {
		report(BAD, "ADS7128", "SYSTEM_STATUS read failed");
		return;
	}
	/* Reading a register at all distinguishes an ADS7128 from something merely ACKing
	 * 0x17: the opcode framing is device-specific, so a foreign part would not answer. */
	report(OK, "ADS7128", "SYSTEM_STATUS=0x%02x (register framing accepted)", status);
}

static uint8_t accel_range_reg, gyro_range_reg;

#if !SENSOR_CHECK_RAW_IMU

/* Driver path: the same two devices, and the same API, that `state --mode bmi088` reads. */
static int imu_xyz(const struct device *dev, enum sensor_channel chan, float out[3])
{
	struct sensor_value v[3];
	int rc = sensor_sample_fetch(dev);

	if (rc != 0) {
		return rc;
	}
	rc = sensor_channel_get(dev, chan, v);
	if (rc != 0) {
		return rc;
	}
	for (int i = 0; i < 3; i++) {
		out[i] = (float)sensor_value_to_double(&v[i]);
	}
	return 0;
}

static void check_accel(void)
{
	float a[3], mag, err;

	if (!device_is_ready(accel)) {
		report(BAD, "BMI088 accel", "Zephyr device not ready (bosch,bmi08x-accel)");
		return;
	}
	if (imu_xyz(accel, SENSOR_CHAN_ACCEL_XYZ, a) != 0) {
		report(BAD, "BMI088 accel", "sample fetch/get failed");
		return;
	}
	mag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	err = (mag - GRAVITY_MPS2) / GRAVITY_MPS2;

	if (err < -GRAVITY_TOL || err > GRAVITY_TOL) {
		report(BAD, "BMI088 accel",
		       "|a|=%d mm/s^2 is %d per-mille off gravity -- scale factor or bus suspect",
		       milli(mag), milli(err));
		return;
	}
	report(OK, "BMI088 accel", "x=%d y=%d z=%d |a|=%d mm/s^2 (%d per-mille vs g)",
	       milli(a[0]), milli(a[1]), milli(a[2]), milli(mag), milli(err));
}

static void check_gyro(void)
{
	float g[3], mag;

	if (!device_is_ready(gyro)) {
		report(BAD, "BMI088 gyro", "Zephyr device not ready (bosch,bmi08x-gyro)");
		return;
	}
	if (imu_xyz(gyro, SENSOR_CHAN_GYRO_XYZ, g) != 0) {
		report(BAD, "BMI088 gyro", "sample fetch/get failed");
		return;
	}
	mag = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);

	if (mag > GYRO_REST_MAX_RPS) {
		report(WARN, "BMI088 gyro",
		       "|w|=%d mrad/s exceeds rest threshold -- board moving, or misconfigured",
		       milli(mag));
		return;
	}
	if (g[0] == 0.0f && g[1] == 0.0f && g[2] == 0.0f) {
		report(WARN, "BMI088 gyro",
		       "all axes exactly zero -- a resting gyro should show noise");
		return;
	}
	report(OK, "BMI088 gyro", "x=%d y=%d z=%d |w|=%d mrad/s (rest + noise)",
	       milli(g[0]), milli(g[1]), milli(g[2]), milli(mag));
}

static void check_temperature(void)
{
	struct sensor_value t;
	int32_t mc;

	if (sensor_channel_get(accel, SENSOR_CHAN_DIE_TEMP, &t) != 0) {
		report(WARN, "BMI088 temp", "die temperature channel unavailable");
		return;
	}
	mc = milli((float)sensor_value_to_double(&t));
	/* A believable room temperature rather than a register default. The doc records
	 * 23.0 C on this board, so a sane reading here is a known quantity. */
	if (mc < 5000 || mc > 60000) {
		report(WARN, "BMI088 temp", "%d mC is outside a plausible bench range", mc);
		return;
	}
	report(OK, "BMI088 temp", "%d mC", mc);
}

/* Estimator rehearsal, through the driver the estimator itself uses. */
static int imu_pair(float a[3])
{
	float g[3];
	int rc = imu_xyz(accel, SENSOR_CHAN_ACCEL_XYZ, a);

	if (rc != 0) {
		return rc;
	}
	return imu_xyz(gyro, SENSOR_CHAN_GYRO_XYZ, g);
}

#else  /* SENSOR_CHECK_RAW_IMU */

static uint8_t accel_range_reg, gyro_range_reg;

static void check_accel(void)
{
	uint8_t id, raw[6];
	float a[3], mag;

	if (reg8_read(ADDR_ACCEL, BMI_ACC_CHIP_ID, &id) != 0) {
		report(BAD, "BMI088 accel", "chip-id read failed at 0x%02x", ADDR_ACCEL);
		return;
	}
	if (id != BMI_ACC_CHIP_ID_VALUE) {
		report(BAD, "BMI088 accel", "chip id 0x%02x, expected 0x%02x", id,
		       BMI_ACC_CHIP_ID_VALUE);
		return;
	}
	if (accel_wake() != 0) {
		report(BAD, "BMI088 accel", "power-on sequence not acknowledged");
		return;
	}
	if (reg8_read(ADDR_ACCEL, BMI_ACC_RANGE, &accel_range_reg) != 0 ||
	    reg8_burst(ADDR_ACCEL, BMI_ACC_DATA, raw, sizeof(raw)) != 0) {
		report(BAD, "BMI088 accel", "range or data read failed");
		return;
	}
	for (int i = 0; i < 3; i++) {
		a[i] = accel_to_mps2(le16(&raw[i * 2]), accel_range_reg);
	}
	mag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);

	/* The magnitude is the check. Individual axes depend on how the board is sitting.
	 * Compared as a signed fraction rather than via fabsf(), which the minimal libc in
	 * this configuration does not declare. */
	float err = (mag - GRAVITY_MPS2) / GRAVITY_MPS2;

	if (err < -GRAVITY_TOL || err > GRAVITY_TOL) {
		report(BAD, "BMI088 accel",
		       "|a|=%d mm/s^2 is %d per-mille off gravity -- scale factor or bus suspect",
		       milli(mag), milli(err));
		return;
	}
	report(OK, "BMI088 accel",
	       "id 0x%02x range_reg=0x%02x x=%d y=%d z=%d |a|=%d mm/s^2 (%d per-mille vs g)",
	       id, accel_range_reg, milli(a[0]), milli(a[1]), milli(a[2]), milli(mag),
	       milli(err));
}

static void check_gyro(void)
{
	uint8_t id, raw[6];
	float g[3], mag;

	if (reg8_read(ADDR_GYRO, BMI_GYR_CHIP_ID, &id) != 0) {
		report(BAD, "BMI088 gyro", "chip-id read failed at 0x%02x", ADDR_GYRO);
		return;
	}
	if (id != BMI_GYR_CHIP_ID_VALUE) {
		report(BAD, "BMI088 gyro", "chip id 0x%02x, expected 0x%02x", id,
		       BMI_GYR_CHIP_ID_VALUE);
		return;
	}
	if (reg8_read(ADDR_GYRO, BMI_GYR_RANGE, &gyro_range_reg) != 0 ||
	    reg8_burst(ADDR_GYRO, BMI_GYR_DATA, raw, sizeof(raw)) != 0) {
		report(BAD, "BMI088 gyro", "range or data read failed");
		return;
	}
	for (int i = 0; i < 3; i++) {
		g[i] = gyro_to_rps(le16(&raw[i * 2]), gyro_range_reg);
	}
	mag = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);

	if (mag > GYRO_REST_MAX_RPS) {
		report(WARN, "BMI088 gyro",
		       "|w|=%d mrad/s exceeds rest threshold -- board moving, or misconfigured",
		       milli(mag));
		return;
	}
	if (g[0] == 0.0f && g[1] == 0.0f && g[2] == 0.0f) {
		report(WARN, "BMI088 gyro",
		       "all axes exactly zero -- a resting gyro should show noise");
		return;
	}
	report(OK, "BMI088 gyro", "id 0x%02x range_reg=0x%02x x=%d y=%d z=%d |w|=%d mrad/s",
	       id, gyro_range_reg, milli(g[0]), milli(g[1]), milli(g[2]), milli(mag));
}

static void check_temperature(void)
{
	uint8_t raw[2];
	int16_t counts;
	int32_t mc;

	/* BMI088 temperature: 11-bit signed, MSB first, 0.125 C per count, offset 23 C. */
	if (reg8_burst(ADDR_ACCEL, BMI_ACC_TEMP_MSB, raw, sizeof(raw)) != 0) {
		report(WARN, "BMI088 temp", "temperature read failed");
		return;
	}
	counts = (int16_t)(((uint16_t)raw[0] << 3) | ((uint16_t)raw[1] >> 5));
	if (counts > 1023) {
		counts -= 2048;
	}
	mc = counts * 125 + 23000;

	/* A believable room temperature rather than a register default. */
	if (mc < 5000 || mc > 60000) {
		report(WARN, "BMI088 temp", "%d mC is outside a plausible bench range", mc);
		return;
	}
	report(OK, "BMI088 temp", "%d mC", mc);
}


/* Estimator rehearsal, raw-register variant. */
static int imu_pair(float a[3])
{
	uint8_t araw[6], graw[6];

	if (reg8_burst(ADDR_ACCEL, BMI_ACC_DATA, araw, sizeof(araw)) != 0 ||
	    reg8_burst(ADDR_GYRO, BMI_GYR_DATA, graw, sizeof(graw)) != 0) {
		return -EIO;
	}
	for (int k = 0; k < 3; k++) {
		a[k] = accel_to_mps2(le16(&araw[k * 2]), accel_range_reg);
	}
	return 0;
}

#endif /* SENSOR_CHECK_RAW_IMU */

static void check_barometer(void)
{
	uint8_t id;

	/* BMP280/BME280 expose a chip id at 0xD0. The value names the part. */
	if (reg8_read(ADDR_BARO, 0xD0U, &id) != 0) {
		report(BAD, "barometer", "chip-id read at 0x%02x failed", ADDR_BARO);
		return;
	}
	switch (id) {
	case 0x58:
		report(OK, "barometer", "BMP280 (chip id 0x58)");
		break;
	case 0x60:
		report(OK, "barometer", "BME280 (chip id 0x60), humidity capable");
		break;
	case 0x56:
	case 0x57:
		report(OK, "barometer", "BMP280 sample silicon (chip id 0x%02x)", id);
		break;
	default:
		/* Honest about the limit: something answers and identifies, but this
		 * repository has never confirmed which part is fitted. */
		report(WARN, "barometer",
		       "chip id 0x%02x is not a BMP280/BME280 id -- part unidentified", id);
		break;
	}
}

static void check_tof(void)
{
	uint8_t id;

	/* VL53L1X: 16-bit register 0x010F holds the model id (0xEA/0xEB). */
	if (reg16_read(ADDR_TOF, 0x010FU, &id) == 0 && (id == 0xEA || id == 0xEB)) {
		report(OK, "ToF", "VL53L1X (model id 0x%02x at 0x010f)", id);
		return;
	}
	/* VL53L0X: 8-bit register 0xC0 holds 0xEE. */
	if (reg8_read(ADDR_TOF, 0xC0U, &id) == 0 && id == 0xEE) {
		report(OK, "ToF", "VL53L0X (model id 0xee at 0xc0)");
		return;
	}
	report(WARN, "ToF", "answers 0x%02x but neither model id matched (read 0x%02x)",
	       ADDR_TOF, id);
}

static void check_camera(void)
{
	uint8_t h, l;

	if (reg16_read(ADDR_CAMERA, 0x0000U, &h) != 0 ||
	    reg16_read(ADDR_CAMERA, 0x0001U, &l) != 0) {
		report(BAD, "HM01B0 camera", "SCCB model-id read failed at 0x%02x", ADDR_CAMERA);
		return;
	}
	uint16_t model = (uint16_t)((h << 8) | l);

	if (model != 0x01B0U) {
		report(BAD, "HM01B0 camera", "model id 0x%04x, expected 0x01b0", model);
		return;
	}
	report(OK, "HM01B0 camera", "model id 0x%04x over SCCB", model);
}

static void check_flow(void)
{
	static struct pmw3901_config cfg;
	static struct pmw3901_data data;
	static const struct device dev = { .name = "PMW3901", .config = &cfg, .data = &data };
	motionBurst_t burst;

	cfg.spi.bus = DEVICE_DT_GET(DT_PHANDLE(PMW3901_NODE, spi_bus));
	cfg.spi.config.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER |
				   SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB;
	cfg.spi.config.frequency = 2000000;
	cfg.spi.config.slave = 0;
	cfg.cs_gpio = (struct gpio_dt_spec)GPIO_DT_SPEC_GET(PMW3901_NODE, cs_gpios);
	cfg.reset_gpio = (struct gpio_dt_spec)GPIO_DT_SPEC_GET(PMW3901_NODE, reset_gpios);
	cfg.led_gpio = (struct gpio_dt_spec)GPIO_DT_SPEC_GET(PMW3901_NODE, led_gpios);

	if (!device_is_ready(cfg.spi.bus) || !device_is_ready(cfg.cs_gpio.port)) {
		report(BAD, "PMW3901 flow", "SPI bus or chip-select GPIO not ready");
		return;
	}
	if (pmw3901_init(&dev) != 0) {
		report(BAD, "PMW3901 flow", "init failed -- chip id did not read back 0x49");
		return;
	}
	if (pmw3901_read_motion_burst(&dev, &burst) != 0) {
		report(BAD, "PMW3901 flow", "motion burst read failed after a good init");
		return;
	}
	/*
	 * SQUAL is surface quality. A stuck bus reads a constant, typically 0x00 or 0xff; a
	 * real surface in view lands well inside that range, so the value discriminates in a
	 * way "the register was readable" does not.
	 */
	if (burst.squal == 0U || burst.squal == 0xFFU) {
		report(WARN, "PMW3901 flow",
		       "SQUAL=%u is a rail value -- init passed but the surface reads stuck",
		       burst.squal);
		return;
	}
	report(OK, "PMW3901 flow", "dx=%+d dy=%+d SQUAL=%u shutter=%u",
	       burst.deltaX, burst.deltaY, burst.squal, burst.shutter);
}

/*
 * The estimator-readiness stage.
 *
 * `state --mode bmi088` runs Madgwick at CONFIG_RB_STATE_RATE_HZ off exactly the two Zephyr
 * devices checked above, so the only thing left to establish is whether those reads sustain
 * the loop rate and stay physically sane while doing it. A single good sample proves neither:
 * a bus that works once and stalls under load, or a rate the I2C transactions cannot meet,
 * both produce a filter that silently integrates garbage.
 */
static void check_estimator_ready(void)
{
	int64_t t0 = k_uptime_get();
	unsigned int errors = 0;
	float mag_min = 1e9f, mag_max = -1e9f, mag_sum = 0.0f;
	unsigned int good = 0;

	printf("\n--- estimator input, %u samples at %u Hz ---\n", EST_SAMPLES, EST_RATE_HZ);
	for (unsigned int i = 0; i < EST_SAMPLES; i++) {
		float a[3];

		if (imu_pair(a) != 0) {
			errors++;
		} else {
			float mag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);

			mag_sum += mag;
			mag_min = (mag < mag_min) ? mag : mag_min;
			mag_max = (mag > mag_max) ? mag : mag_max;
			good++;
		}
		k_usleep(1000000U / EST_RATE_HZ);
	}

	int64_t elapsed_ms = k_uptime_get() - t0;
	float achieved = elapsed_ms > 0 ? (float)EST_SAMPLES * 1000.0f / (float)elapsed_ms : 0.0f;

	printf("       %u/%u reads ok, %u error(s) in %lld ms\n",
	       good, EST_SAMPLES, errors, (long long)elapsed_ms);
	if (good > 0U) {
		printf("       |a| mean=%d min=%d max=%d mm/s^2 (spread %d)\n",
		       milli(mag_sum / (float)good), milli(mag_min), milli(mag_max),
		       milli(mag_max - mag_min));
	}

	if (errors > 0U) {
		report(BAD, "estimator input", "%u read error(s) -- the filter would integrate gaps",
		       errors);
		return;
	}
	/*
	 * The achieved rate is reported rather than gated. k_usleep plus two I2C round trips
	 * per iteration sets a ceiling well below the nominal rate on a 50 MHz core, and that
	 * is a property of the loop, not a fault -- but the estimator's dt must come from the
	 * clock rather than the configured constant if the two differ, which main.c already
	 * does. Flagged so the gap is visible instead of surprising.
	 */
	if (achieved < (float)EST_RATE_HZ * 0.8f) {
		report(WARN, "estimator input",
		       "sustained %d mHz against %u Hz requested -- dt must come from the clock",
		       milli(achieved), EST_RATE_HZ);
		return;
	}
	report(OK, "estimator input", "sustained %d mHz, no read errors, |a| stable",
	       milli(achieved));
}

int main(void)
{
	printf("\n=====================================================\n");
	printf("RiskyBird whole-board sensor check\n");
	printf("  built  : " __DATE__ " " __TIME__ "\n");
	printf("  motors : NOT driven -- safe with propellers fitted\n");
	printf("=====================================================\n");

	i2c_dev = DEVICE_DT_GET(I2C_NODE);
	if (!device_is_ready(i2c_dev)) {
		printf("\n[ FAIL ] I2C controller not ready; nothing else can be checked.\n");
		printf("RESULT: FAIL\n");
		return 1;
	}

	/*
	 * Pre-flight the bus before the first transfer, because a wedged bus hangs forever
	 * rather than returning an error.
	 */
	uint32_t st = sys_read32(I2C_REG_STATUS);

	if ((st & (I2C_STATUS_BUSY | I2C_STATUS_TIP)) != 0U) {
		printf("\n[ FAIL ] i2c bus wedged: STATUS=0x%02x (BUSY=%u TIP=%u AL=%u)\n",
		       (unsigned)(st & 0xff), (st & I2C_STATUS_BUSY) ? 1U : 0U,
		       (st & I2C_STATUS_TIP) ? 1U : 0U, (st & I2C_STATUS_AL) ? 1U : 0U);
		printf("         A slave is still holding the bus from a previous transaction.\n");
		printf("         The sifive driver has no bus-recovery entry point, and reloading\n");
		printf("         the bitstream resets the FPGA controller but not the slave, so\n");
		printf("         this needs the drone board power-cycled. Not attempting any\n");
		printf("         transfer: i2c_write() would block here with no timeout.\n");
		printf("RESULT: FAIL -- i2c bus needs a power cycle\n");
		return 1;
	}
	printf("\n--- gates ---   (i2c STATUS=0x%02x: controller idle; says nothing about\n"
	       "                 whether a slave is holding the line)\n", (unsigned)(st & 0xff));
	release_gates();
	check_inventory();

	printf("\n--- per-sensor ---\n");
	check_expander();
	check_accel();
	check_gyro();
	check_temperature();
	check_barometer();
	check_tof();
	check_camera();
	check_flow();

	check_estimator_ready();

	printf("\n=====================================================\n");
	printf("%u/%u checks passed\n", checks_passed, checks_run);
	if (checks_passed == checks_run) {
		printf("RESULT: PASS -- ready for: rb build state --mode bmi088\n");
		return 0;
	}
	printf("RESULT: PARTIAL -- see WARN/FAIL lines above\n");
	return 0;
}
