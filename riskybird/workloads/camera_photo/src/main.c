/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Take one photograph with the HM01B0 and leave it in RAM for the debugger.
 *
 * camera_bringup answers "does the camera work at all" and prints nine stages of evidence to
 * prove it. This target assumes that question is settled and does the one thing that follows
 * from it: configure the sensor, capture a full frame, stop. It needs a DDR-backed shell --
 * a frame is ~79 KB and a scratchpad holds 32 KB -- so it is registered only for configs that
 * present real memory.
 *
 * The frame is never printed. It is left in pixels[], and hardware/ospi/capture-frame.gdb
 * copies it out over the same JTAG link that loaded the image:
 *
 *   ./rb debug camera_photo --target arty200t \
 *       --config RocketArty200TDroneFullDDRConfig --program \
 *       --program-usb-location <bitstream> --usb-location <debug> \
 *       --gdb-script hardware/ospi/capture-frame.gdb
 *   python3 hardware/ospi/frame-to-png.py camera-frame.raw 326
 *
 * WHAT THIS TARGET DOES NOT TOUCH. Not the ADS7128 expander: the camera shares the I2C bus
 * with it but is not gated by it, and writing its channels perturbs the ToF XSHUT lines for
 * no benefit. Not the sensor's TEST_PATTERN register: that is a bring-up proof, and leaving a
 * walking-1s pattern enabled turns a photograph into synthetic data.
 *
 * The PWM blocks it does touch, and only to park them -- see pwm_park().
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/sys_io.h>
#include <stdio.h>

#include "hm01b0.h"

/* ---- capture peripheral. Base and layout: ospi/OspiChipyard.scala. ------------------------ */
#define CAM_BASE        0x10080000UL
#define CAM_CTRL        (CAM_BASE + 0x00)
#define CAM_GEOM        (CAM_BASE + 0x04)
#define CAM_MCLKDIV     (CAM_BASE + 0x08)
#define CAM_FRAMECNT    (CAM_BASE + 0x10)
#define CAM_LASTWIDTH   (CAM_BASE + 0x14)
#define CAM_LASTHEIGHT  (CAM_BASE + 0x18)
#define CAM_FLAGS       (CAM_BASE + 0x1c)
#define CAM_DATA        (CAM_BASE + 0x20)
#define CAM_CAPACITY    (CAM_BASE + 0x24)
#define CAM_PIXTARGET   (CAM_BASE + 0x28)
#define CAM_CAPCOUNT    (CAM_BASE + 0x2c)
#define CAM_PCLKCNT     (CAM_BASE + 0x30)
#define CAM_FVLDCNT     (CAM_BASE + 0x34)
#define CAM_LVLDCNT     (CAM_BASE + 0x38)
#define CAM_CAPSTAT     (CAM_BASE + 0x40)

#define CTRL_ENABLE     (1U << 0)
#define CTRL_CLEAR      (1U << 4)
#define CTRL_FLUSH      (1U << 5)
#define CTRL_ARM        (1U << 6)

#define FLAG_OVERFLOW   (1U << 1)
#define DATA_VALID      (1U << 31)
#define DATA_EOF        (1U << 10)
#define DATA_PIXEL_MASK 0xffU
#define CAPSTAT_DONE    (1U << 1)

/*
 * MCLK = sysclk / (2 * (div + 1)), sysclk = 50 MHz, so div = 1 is 12.5 MHz.
 * 12.5 MHz because that is the rate this sensor has actually answered at on this hardware;
 * 6.25 MHz, which is equally inside the datasheet's 3-36 MHz range, got no ACK at all.
 */
#define CAM_MCLK_DIV    1U

/*
 * One frame at the width the core MEASURES, not the width GEOM is configured for. The sensor
 * puts two dummy pixels ahead of each active line, so lines are 326 wide where GEOM says 324.
 * Capturing 324-wide rows shears the picture one pixel per row, which looks like broken
 * hardware and is not.
 */
#ifndef CAMERA_PHOTO_WIDTH
#define CAMERA_PHOTO_WIDTH  326U
#endif
#ifndef CAMERA_PHOTO_HEIGHT
#define CAMERA_PHOTO_HEIGHT 242U
#endif
#define PHOTO_PIXELS        (CAMERA_PHOTO_WIDTH * CAMERA_PHOTO_HEIGHT)

#define CAPTURE_TIMEOUT_MS  4000

#define I2C_NODE DT_NODELABEL(i2c0)

static const struct device *i2c_dev;
static uint8_t pixels[PHOTO_PIXELS];

/*
 * Debugger handles. pixels[] is a file static and so survives main returning, but the count
 * actually drained is a block local and does not, and stopping on a source line number breaks
 * the moment this file is edited. camera_frame_pixels is how many leading bytes are valid;
 * camera_frame_ready() is the breakpoint anchor. Same names camera_bringup exports, so one
 * GDB script serves both.
 */
volatile uint32_t camera_frame_pixels;

__attribute__((noinline)) void camera_frame_ready(void)
{
	/*
	 * A side effect the optimiser cannot discard. noinline alone is not enough: with an empty
	 * body GCC deduces the function is const and deletes the call site, so the breakpoint
	 * anchor disappears from the image even though the symbol still resolves.
	 */
	__asm__ volatile("" ::: "memory");
}

/* ---- motor outputs ------------------------------------------------------------------------ */

/*
 * Park both PWM blocks before doing anything else.
 *
 * A camera target has no business with the motor pins, and on a correctly built shell it would
 * not need this: RocketArty200TDroneFullDDRConfig routes motor1-4 to E22/B20/R17/F13 from the
 * two sifive PWM blocks, and WithArty200TPWM inverts them precisely so that the reset state
 * (pwmcmp = 0, comparator output constantly high, inverted to low) means motors off before a
 * single instruction runs.
 *
 * It is here anyway, because on 2026-09-04 loading that shell spun two motors, and a workload
 * that cannot be run without watching the propellers is not a workload. Writing the parked
 * state explicitly costs eight stores and removes the dependency on the reset state being what
 * the comment says it is. If the pins really are already low this changes nothing.
 *
 * Parked means pwmcfg = 0 (counter disabled, no invert bits) and every comparator at 0. With
 * the harness inversion that is 0% duty. Do NOT set pwmcfg's own invert bits (20-23): the
 * harness has already inverted once, and inverting a second time is full throttle. That is
 * also why this writes pwmcfg wholesale rather than read-modify-write -- it guarantees those
 * bits end up clear regardless of what was there before.
 */
#define PWM0_BASE       0x10050000UL
#define PWM1_BASE       0x10051000UL
#define PWM_CFG         0x00
#define PWM_CMP0        0x20

/*
 * The comparator value that means "motor off".
 *
 * Which one that is depends on a polarity this bench has not yet measured, and the two answers
 * are opposites -- so this is a knob rather than a constant, and the default is the one that
 * changes nothing.
 *
 * The comparator output is (pwms >= pwmcmp), so the value picks the idle level directly:
 *
 *   pwmcmp = 0        always matches   -> output constantly ASSERTED   (the reset state)
 *   pwmcmp = 0xffff   never matches    -> output constantly DEASSERTED
 *
 * WithArty200TPWM inverts at the harness so that the reset state is meant to be 0% duty, which
 * makes 0 correct. But a GDB read of pwmcfg on a freshly configured part, before any
 * instruction executed, returns 0xf000_0000 on both blocks -- the four pwmcmpXip bits all set,
 * i.e. all four comparators asserted, exactly the condition that comment calls full throttle
 * without the inversion. Whether the inversion then lands is a question about the pin, not
 * about this register, and one meter reading on J1_31 / J1_36 / J1_99 / J1_100 answers it.
 *
 * If those pins measure HIGH at idle, rebuild with the opposite value and they will go low:
 *
 *   RB_EXTRA_CFLAGS=-DCAMERA_PHOTO_PWM_OFF_CMP=0xffff
 *
 * Deliberately NOT done by touching pwmcfg's invert bits: the harness has already inverted
 * once, inverting again is full throttle, and the comparator route needs no assumption about
 * which bit position sifive-blocks put that field in.
 */
#ifndef CAMERA_PHOTO_PWM_OFF_CMP
#define CAMERA_PHOTO_PWM_OFF_CMP 0x0000U
#endif

static void pwm_park(void)
{
	static const unsigned long blocks[] = { PWM0_BASE, PWM1_BASE };

	printf("[0/6] parking motor PWM outputs\n");
	/*
	 * Report what the blocks were in BEFORE parking them. This is the only cheap evidence
	 * about the state the bitstream leaves the motor pins in: a nonzero cfg or comparator
	 * here means something configured them, and all-zero means the pins were already at the
	 * reset state and any motor motion came from the harness polarity rather than from a
	 * stray register write.
	 */
	for (size_t b = 0; b < ARRAY_SIZE(blocks); b++) {
		printf("      pwm%zu before: cfg=0x%08x cmp=%08x %08x %08x %08x\n", b,
		       sys_read32(blocks[b] + PWM_CFG),
		       sys_read32(blocks[b] + PWM_CMP0), sys_read32(blocks[b] + PWM_CMP0 + 4),
		       sys_read32(blocks[b] + PWM_CMP0 + 8), sys_read32(blocks[b] + PWM_CMP0 + 12));
	}
	for (size_t b = 0; b < ARRAY_SIZE(blocks); b++) {
		sys_write32(0U, blocks[b] + PWM_CFG);
		for (unsigned int c = 0; c < 4U; c++) {
			sys_write32(CAMERA_PHOTO_PWM_OFF_CMP, blocks[b] + PWM_CMP0 + 4U * c);
		}
	}
	printf("      parked: pwmcfg=0x%08x/0x%08x, all comparators 0x%04x%s\n",
	       sys_read32(PWM0_BASE + PWM_CFG), sys_read32(PWM1_BASE + PWM_CFG),
	       CAMERA_PHOTO_PWM_OFF_CMP,
	       (CAMERA_PHOTO_PWM_OFF_CMP == 0U) ? "  (== the reset state; a no-op by design)" : "");
}

/* ---- sensor control (SCCB) ---------------------------------------------------------------- */

/*
 * SCCB has no repeated START, so the register-address write must be terminated by a STOP
 * before the read begins. i2c_write_read() emits a repeated START and a healthy HM01B0 NAKs
 * it, which is indistinguishable from an absent sensor.
 */
static int cam_reg_read(uint16_t reg, uint8_t *out)
{
	uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff) };
	int rc = i2c_write(i2c_dev, addr, sizeof(addr), HM01B0_I2C_ADDR);

	if (rc != 0) {
		return rc;
	}
	return i2c_read(i2c_dev, out, 1, HM01B0_I2C_ADDR);
}

static int cam_reg_write(uint16_t reg, uint8_t val)
{
	uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff), val };

	return i2c_write(i2c_dev, buf, sizeof(buf), HM01B0_I2C_ADDR);
}

static int fail(const char *stage, const char *detail)
{
	printf("\n[FAIL] stage=%s: %s\n", stage, detail);
	printf("RESULT: FAIL\n");
	camera_frame_ready();   /* still stop, so a partial frame can be read out */
	return 1;
}

int main(void)
{
	uint8_t id_h, id_l;
	uint16_t model_id;
	uint32_t pclk0, pclk1, captured, n = 0, word;
	int64_t deadline;

	printf("\n=====================================================\n");
	printf("RiskyBird HM01B0 photograph\n");
	printf("  frame  : %u x %u = %u bytes\n",
	       CAMERA_PHOTO_WIDTH, CAMERA_PHOTO_HEIGHT, (unsigned int)PHOTO_PIXELS);
	printf("  built  : " __DATE__ " " __TIME__ "\n");
	printf("=====================================================\n\n");

	pwm_park();

	if (sys_read32(CAM_CAPACITY) < PHOTO_PIXELS + 1U) {
		return fail("capacity", "capture buffer is smaller than one frame");
	}

	/* MCLK first: the sensor cannot answer on SCCB without its master clock. */
	sys_write32(CAM_MCLK_DIV, CAM_MCLKDIV);
	sys_write32(CTRL_ENABLE, CAM_CTRL);
	k_msleep(10);

	i2c_dev = DEVICE_DT_GET(I2C_NODE);
	if (!device_is_ready(i2c_dev)) {
		return fail("i2c-dev", "I2C controller device not ready");
	}
	if (cam_reg_read(HM01B0_REG_MODEL_ID_H, &id_h) != 0 ||
	    cam_reg_read(HM01B0_REG_MODEL_ID_L, &id_l) != 0) {
		return fail("i2c-nack", "no ACK from the sensor at 0x24 with MCLK running");
	}
	model_id = (uint16_t)((id_h << 8) | id_l);
	printf("[1/6] sensor: MODEL_ID = 0x%04x\n", model_id);
	if (model_id != HM01B0_MODEL_ID) {
		return fail("chip-id", "I2C works but this is not an HM01B0");
	}

	/* A photograph, not a test pattern. Cheap insurance against a previous run's leftovers. */
	if (cam_reg_write(HM01B0_REG_TEST_PATTERN, HM01B0_TESTPAT_OFF) != 0) {
		return fail("reg-write", "could not disable TEST_PATTERN_MODE");
	}

	if (cam_reg_write(HM01B0_REG_MODE_SELECT, HM01B0_MODE_STREAMING) != 0) {
		return fail("stream", "MODE_SELECT write was not acknowledged");
	}
	printf("[2/6] streaming enabled\n");
	k_msleep(100);

	/*
	 * PCLK is only meaningful after the sensor leaves standby -- the part powers up in standby
	 * and a perfectly healthy camera drives no pixel clock until now.
	 */
	pclk0 = sys_read32(CAM_PCLKCNT);
	k_msleep(50);
	pclk1 = sys_read32(CAM_PCLKCNT);
	printf("[3/6] PCLK: %u -> %u over 50 ms\n", pclk0, pclk1);
	if (pclk1 == pclk0) {
		return fail("pclk", "no pixel clock after MODE_SELECT=streaming");
	}
	if (sys_read32(CAM_FVLDCNT) == 0U || sys_read32(CAM_LVLDCNT) == 0U) {
		return fail("sync", "sensor is clocked but FVLD/LVLD never asserted");
	}

	/* Arm on a frame boundary so the capture is one picture, not a slice of two. */
	sys_write32(CTRL_ENABLE | CTRL_CLEAR, CAM_CTRL);
	sys_write32(CTRL_ENABLE | CTRL_FLUSH, CAM_CTRL);
	sys_write32(PHOTO_PIXELS, CAM_PIXTARGET);
	sys_write32(CTRL_ENABLE | CTRL_ARM, CAM_CTRL);
	printf("[4/6] armed for %u pixels\n", (unsigned int)PHOTO_PIXELS);

	deadline = k_uptime_get() + CAPTURE_TIMEOUT_MS;
	do {
		captured = sys_read32(CAM_CAPCOUNT);
	} while (captured < PHOTO_PIXELS && (sys_read32(CAM_CAPSTAT) & CAPSTAT_DONE) == 0U &&
		 k_uptime_get() < deadline);

	/*
	 * Stop the sensor the moment the frame is satisfied. The CDC FIFO is 1024 beats and the
	 * sensor streams at several MHz, so it refills far faster than a polling CPU drains it;
	 * leaving it running means OVERFLOW is set by surplus pixels arriving after PIXTARGET.
	 * That is data nobody asked for, not data that was lost, which is why overflow is only
	 * fatal below when the count also came up short.
	 */
	(void)cam_reg_write(HM01B0_REG_MODE_SELECT, HM01B0_MODE_STANDBY);
	sys_write32(0U, CAM_CTRL);

	if (captured < PHOTO_PIXELS) {
		printf("[5/6] captured only %u of %u pixels (flags=0x%08x)\n",
		       captured, (unsigned int)PHOTO_PIXELS, sys_read32(CAM_FLAGS));
		return fail((sys_read32(CAM_FLAGS) & FLAG_OVERFLOW) ? "overflow" : "timeout",
			    "the frame did not complete");
	}
	printf("[5/6] captured %u pixels\n", captured);

	while (n < PHOTO_PIXELS) {
		word = sys_read32(CAM_DATA);
		if ((word & DATA_VALID) == 0U) {
			break;
		}
		if (word & DATA_EOF) {
			continue;   /* in-band end-of-frame marker, not a pixel */
		}
		pixels[n++] = (uint8_t)(word & DATA_PIXEL_MASK);
	}
	camera_frame_pixels = n;

	{
		uint8_t vmin = 0xff, vmax = 0x00;

		for (uint32_t i = 0; i < n; i++) {
			if (pixels[i] < vmin) { vmin = pixels[i]; }
			if (pixels[i] > vmax) { vmax = pixels[i]; }
		}
		printf("[6/6] drained %u pixels: min=0x%02x max=0x%02x\n", n, vmin, vmax);
		printf("      geometry: measured %ux%u, configured %ux%u\n",
		       sys_read32(CAM_LASTWIDTH), sys_read32(CAM_LASTHEIGHT),
		       sys_read32(CAM_GEOM) & 0xffffU, (sys_read32(CAM_GEOM) >> 16) & 0xffffU);
		if (n == 0U) {
			return fail("drain", "capture reported done but the buffer returned no pixels");
		}
		if (vmin == vmax) {
			return fail("constant-data", "every pixel identical: the data bus is stuck");
		}
	}

	printf("\nRESULT: PASS -- %u bytes in pixels[], reshape rows at %u\n",
	       n, sys_read32(CAM_LASTWIDTH));
	camera_frame_ready();
	return 0;
}
