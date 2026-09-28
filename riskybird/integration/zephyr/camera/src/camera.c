/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * See include/riskybird/camera.h. Extracted from workloads/camera_photo, whose
 * comments explain why each step is where it is; the reasoning is kept here
 * because it is the part that is expensive to rediscover.
 */

#include <riskybird/camera.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "hm01b0.h"

/* Capture peripheral. Base and layout: ospi/OspiChipyard.scala. */
#define CAM_BASE        0x10080000UL
#define CAM_CTRL        (CAM_BASE + 0x00)
#define CAM_GEOM        (CAM_BASE + 0x04)
#define CAM_MCLKDIV     (CAM_BASE + 0x08)
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
 * MCLK = sysclk / (2 * (div + 1)). On the 50 MHz shells div = 1 is 12.5 MHz,
 * the rate this sensor has actually answered at on this hardware; 6.25 MHz,
 * equally inside the datasheet's 3-36 MHz range, got no ACK at all.
 *
 * sysclk is the fabric clock, so the same div = 1 is 10 MHz on the 40 MHz
 * accelerator shells and 8.75 MHz on the 35 MHz one -- neither tried. 12.5 MHz
 * is not reachable there; div = 0 gives 20 / 17.5 MHz. Override per build with
 * RB_EXTRA_CFLAGS=-DCAM_MCLK_DIV=<n> (C only, so RB_EXTRA_CFLAGS, not
 * RB_CMAKE_ARGS). The register is 8 bits wide (ospi/Params.scala mclkDivWidth).
 */
#ifndef CAM_MCLK_DIV
#define CAM_MCLK_DIV       1U
#endif
BUILD_ASSERT((CAM_MCLK_DIV) <= 0xffU, "CAM_MCLK_DIV is an 8-bit register field");
/* The same number SIFIVE_PERIPHERAL_CLOCK_FREQUENCY is built from. */
#define CAM_SYSCLK_HZ \
	((uint32_t)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC * CONFIG_RTC_CLOCK_DIVIDER_VALUE)
#define CAM_MCLK_HZ        (CAM_SYSCLK_HZ / (2U * ((CAM_MCLK_DIV) + 1U)))
#define CAPTURE_TIMEOUT_MS 20000

static const struct device *i2c_dev;

/*
 * SCCB needs a STOP between the register address and the read. i2c_write_read()
 * emits a repeated START and a healthy HM01B0 NAKs it, which is
 * indistinguishable from an absent sensor.
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

/*
 * OPTIONAL, OFF BY DEFAULT: program the sensor for full-resolution, unbinned,
 * unsubsampled readout instead of trusting its power-up defaults.
 *
 *   ./rb build ... with RB_EXTRA_CPPFLAGS=-DRB_CAMERA_FULL_READOUT=1
 *
 * It is a deliberate experiment and not the default because of what is already
 * known. Nothing in this repository has ever written a resolution, window,
 * binning or subsample register -- the whole init is TEST_PATTERN_MODE and
 * MODE_SELECT -- and the pictures that came out were sharp and correctly
 * exposed. A build that starts writing mode registers whose encodings are NOT
 * established on this machine (see the provenance note in hm01b0.h: the only
 * Himax datasheet here is a module datasheet that names exactly one of them)
 * can turn a working camera into a broken one, and the failure would look like
 * a hardware fault. Keeping it behind a flag means the bad outcome is always
 * one rebuild away from being undone.
 *
 * What it is FOR: the sensor demonstrably is not in the mode the software
 * assumes -- GEOM is configured 324x244 and the core measures 324 active
 * columns by roughly 324 lines -- so this pins the mode down instead of
 * inferring it. It should be a no-op on a sensor that is already in full
 * readout, and the read-back below is what proves that either way.
 *
 * GRP_PARAM_HOLD brackets the writes so the sensor cannot latch a half-written
 * configuration mid-frame. 0x0104 = 1 to hold, 0 to commit. UNVERIFIED: the
 * address and that protocol are datasheet recall, not established here. If the
 * hold write is NAKed the sequence gives up rather than writing the mode
 * registers unbracketed, because a torn configuration is harder to diagnose
 * than a refusal.
 *
 * Order is deliberate: QVGA_WIN_EN last, because it is the only one of the four
 * whose address and polarity this machine can actually confirm, so if the
 * sequence dies partway the register that is most likely to have taken effect
 * is the one whose effect is understood.
 */
#ifndef RB_CAMERA_FULL_READOUT
#define RB_CAMERA_FULL_READOUT 0
#endif

/*
 * Raise the auto-exposure target.
 *
 * MEASURED 2026-09-22, reading the live part for the first time: AE is ENABLED
 * (ae_ctrl 0x2100 = 0x01) and working correctly -- it is simply converging to
 * the target it was given, AE_TARGET 0x2101 = 0x3c = 60 DN. With black level
 * correction on and BLC_TARGET = 32, a mean of 60 leaves about 28 DN of actual
 * signal, and every frame ever captured here spans 5-49 DN above black. The
 * sensor was never underexposed by accident; nobody had ever set the target.
 *
 * There is plenty of headroom to spend: integration was 67 lines out of a
 * 562-line frame (12%), and analog gain was at its 1x minimum, so AE can reach
 * a higher target on integration time alone without adding gain noise.
 *
 * Why this is the prerequisite for the colour question rather than a cosmetic
 * improvement: a per-channel difference on a Bayer sensor is a RATIO of
 * signals above black. At 28 DN that difference lives in a handful of codes and
 * sits at the quantisation floor; at ~96 DN it is four times larger while the
 * quantisation step is unchanged. Every colour statistic computed on this
 * bench so far was measured on almost no signal.
 *
 * Default 128 is mid-scale for 8 bits, leaving ~96 DN above black and headroom
 * before clipping. Override with RB_CAMERA_AE_TARGET.
 */
#ifndef RB_CAMERA_AE_SETTLE_MS
#define RB_CAMERA_AE_SETTLE_MS 6000
#endif

#ifndef RB_CAMERA_AE_TARGET
/*
 * 64 DN, not 128.
 *
 * AE's target is a mean of the sensor's LINEAR output, and a mean of 128 there
 * is not mid-grey -- it is about a stop and a half hot, because everything
 * downstream that displays the frame applies an sRGB curve that lifts 0.25
 * linear to roughly 0.53 on screen. Measured on the same scene:
 *
 *     target 128:  18.3% of samples clipped at 255, analog gain 0x10
 *     target  64:   0.1% clipped,                   analog gain 0x00 (minimum)
 *
 * So the lower target is better twice over: the highlights survive, and AE
 * reaches the exposure with integration time alone instead of buying the last
 * stop with analog gain, which is the noisiest way to get it.
 */
#define RB_CAMERA_AE_TARGET 64
#endif

/*
 * Turn on the D8 status LED, the only light source on this board that software
 * can reach.
 *
 * It is an indicator LED on the ADS7128 expander (GPIO7, ACTIVE LOW), not an
 * illuminator, and it is not aimed at the lens. It is worth a try anyway: when
 * the airframe is face down on a bench the LED and the camera are looking at
 * the same nearby surface, and a few photons is the difference between a frame
 * that says "zero light" and one that has something in it to measure.
 *
 * MEASURED 2026-09-22 with the lens dark: the sensor emitted a constant 96 DN
 * over 99.99% of the frame, which is exactly BLC_TARGET 32 x digital gain 3 --
 * i.e. its black level amplified, i.e. no photons at all. If this LED changes
 * that number at all, there is an optical path to work with.
 *
 * Deliberately NOT reverted afterwards: an LED left on is visible to a human
 * walking past and is a far smaller problem than a dark camera nobody can
 * diagnose remotely.
 */
#define CAM_ADS7128_ADDR      0x17U
#define CAM_ADS7128_CMD_WRITE 0x08U
#define CAM_ADS7128_PIN_CFG   0x05U
#define CAM_ADS7128_GPIO_CFG  0x07U
#define CAM_ADS7128_GPO_VALUE 0x0BU
#define CAM_STATUS_LED_CH     7U

/* unused when RB_CAMERA_STATUS_LED=0 (the flight photo modes: D8 is the flight LED) */
__attribute__((unused)) static int cam_status_led_on(void)
{
	uint8_t tx[3];
	int rc;

	/* GPIO mode, then output, then drive LOW (active low = lit). */
	tx[0] = CAM_ADS7128_CMD_WRITE; tx[1] = CAM_ADS7128_PIN_CFG;
	tx[2] = (uint8_t)(1U << CAM_STATUS_LED_CH);
	rc = i2c_write(i2c_dev, tx, sizeof(tx), CAM_ADS7128_ADDR);
	if (rc != 0) {
		return rc;
	}
	tx[1] = CAM_ADS7128_GPIO_CFG;
	rc = i2c_write(i2c_dev, tx, sizeof(tx), CAM_ADS7128_ADDR);
	if (rc != 0) {
		return rc;
	}
	tx[1] = CAM_ADS7128_GPO_VALUE; tx[2] = 0x00U;   /* LOW -> LED on */
	return i2c_write(i2c_dev, tx, sizeof(tx), CAM_ADS7128_ADDR);
}

static int cam_set_ae_target(uint8_t target)
{
	int rc = cam_reg_write(HM01B0_REG_GRP_PARAM_HOLD, HM01B0_GRP_HOLD);

	if (rc != 0) {
		return rc;
	}
	rc = cam_reg_write(HM01B0_REG_AE_TARGET, target);
	/* Commit regardless: a sensor left held may never stream again, and
	 * that presents as a dead camera rather than as a failed write. */
	(void)cam_reg_write(HM01B0_REG_GRP_PARAM_HOLD, HM01B0_GRP_COMMIT);
	return rc;
}

#if RB_CAMERA_FULL_READOUT
static int cam_program_full_readout(void)
{
	static const struct { uint16_t reg; uint8_t val; } seq[] = {
		{ HM01B0_REG_X_ODD_INC,    HM01B0_X_ODD_INC_EVERY },
		{ HM01B0_REG_Y_ODD_INC,    HM01B0_Y_ODD_INC_EVERY },
		{ HM01B0_REG_BINNING_MODE, HM01B0_BINNING_OFF },
		{ HM01B0_REG_QVGA_WIN_EN,  HM01B0_QVGA_WIN_OFF },
	};
	int rc = cam_reg_write(HM01B0_REG_GRP_PARAM_HOLD, HM01B0_GRP_HOLD);

	if (rc != 0) {
		return rc;
	}
	for (int i = 0; i < (int)ARRAY_SIZE(seq); i++) {
		rc = cam_reg_write(seq[i].reg, seq[i].val);
		if (rc != 0) {
			/* Commit anyway: leaving the sensor held is worse than
			 * committing a partial write, because a held sensor may
			 * never stream again and that reads as a dead camera. */
			(void)cam_reg_write(HM01B0_REG_GRP_PARAM_HOLD,
					    HM01B0_GRP_COMMIT);
			return rc;
		}
	}
	return cam_reg_write(HM01B0_REG_GRP_PARAM_HOLD, HM01B0_GRP_COMMIT);
}
#endif

uint32_t rb_camera_capacity(void)
{
	return sys_read32(CAM_CAPACITY);
}

static int stop(const char **stage, const char **detail,
		const char *s, const char *d)
{
	*stage = s;
	*detail = d;
	return -1;
}

/*
 * Print the live exposure chain. MUST be called with the sensor STREAMING and
 * AE settled: every register here is an AE OUTPUT, and in standby they still
 * hold whatever the PREVIOUS run converged to. Reading them before
 * MODE_SELECT therefore reports the last scene the camera saw, not this one.
 * That is not hypothetical -- on this bench it printed "the sensor has NO
 * LIGHT" for a capture of a brightly lit target, because the carried-over
 * values were from a run with the lens dark.
 */
static void cam_dump_exposure(void)
{
	static const struct { uint16_t reg; const char *name; } dump[] = {
		{ HM01B0_REG_AE_CTRL,        "ae_ctrl      " },
		{ HM01B0_REG_AE_TARGET,      "ae_target    " },
		{ HM01B0_REG_AE_MIN_MEAN,    "ae_min_mean  " },
		{ HM01B0_REG_MAX_INTG_H,     "max_intg_h   " },
		{ HM01B0_REG_MAX_INTG_L,     "max_intg_l   " },
		{ HM01B0_REG_MAX_AGAIN,      "max_again    " },
		{ HM01B0_REG_MAX_DGAIN,      "max_dgain    " },
		{ HM01B0_REG_INTEGRATION_H,  "integration_h" },
		{ HM01B0_REG_INTEGRATION_L,  "integration_l" },
		{ HM01B0_REG_ANALOG_GAIN,    "analog_gain  " },
		{ HM01B0_REG_DIGITAL_GAIN_H, "dig_gain_h   " },
		{ HM01B0_REG_DIGITAL_GAIN_L, "dig_gain_l   " },
		{ HM01B0_REG_FRAME_LEN_H,    "frame_len_h  " },
		{ HM01B0_REG_FRAME_LEN_L,    "frame_len_l  " },
		{ HM01B0_REG_LINE_LEN_H,     "line_len_h   " },
		{ HM01B0_REG_LINE_LEN_L,     "line_len_l   " },
		{ HM01B0_REG_BLC_CFG,        "blc_cfg      " },
		{ HM01B0_REG_BLC_TARGET,     "blc_target   " },
	};

	uint8_t ag = 0, dgh = 0, dgl = 0;
	uint8_t ih = 0, il = 0, mih = 0, mil = 0, mag = 0;

	printk("         --- HM01B0 exposure chain ---\n");
	for (unsigned i = 0; i < ARRAY_SIZE(dump); i++) {
		uint8_t v = 0;
		int rc = cam_reg_read(dump[i].reg, &v);

		if (rc == 0) {
			printk("         %s 0x%04x = 0x%02x (%u)\n",
			       dump[i].name, dump[i].reg, v, v);
			if (dump[i].reg == HM01B0_REG_ANALOG_GAIN)    { ag = v; }
			if (dump[i].reg == HM01B0_REG_DIGITAL_GAIN_H) { dgh = v; }
			if (dump[i].reg == HM01B0_REG_DIGITAL_GAIN_L) { dgl = v; }
			if (dump[i].reg == HM01B0_REG_INTEGRATION_H)  { ih = v; }
			if (dump[i].reg == HM01B0_REG_INTEGRATION_L)  { il = v; }
			if (dump[i].reg == HM01B0_REG_MAX_INTG_H)     { mih = v; }
			if (dump[i].reg == HM01B0_REG_MAX_INTG_L)     { mil = v; }
			if (dump[i].reg == HM01B0_REG_MAX_AGAIN)      { mag = v; }
		} else {
			printk("         %s 0x%04x = NAK (%d) -- wrong address?\n",
			       dump[i].name, dump[i].reg, rc);
		}
	}

	/*
	 * AE SATURATION IS A LIGHTING REPORT, NOT A FAULT, and saying so
	 * here is the whole point of dumping these.
	 *
	 * With the gain chain pegged the sensor emits amplified black,
	 * which arrives as a nearly constant frame with a handful of
	 * distinct values. That looks exactly like a dead sensor, a
	 * broken capture path or a "synthetic pattern", and it has
	 * already been misread as all three on this bench. It is none of
	 * them: it means no photons.
	 *
	 * Measured on this hardware for contrast -- lens with a view:
	 * analog_gain 0x00, integration 67 of 562 lines. Lens dark:
	 * analog_gain 0x30 (max), digital gain 3x, and the frame comes
	 * back as 99.99% one value.
	 *
	 * No colour question can be answered in this state: a per-channel
	 * ratio needs signal above black, and there is none.
	 */
	/*
	 * Analog gain pegged is sufficient on its own. Do NOT also
	 * require the digital gain to read > 1: DIGITAL_GAIN_H is the
	 * INTEGER part and the fraction lives in DIGITAL_GAIN_L, so a
	 * genuine 1.92x reads as h=0x01, l=0xec. Testing h > 1 silently
	 * missed a fully saturated sensor on this bench.
	 */
	if (ag >= HM01B0_ANALOG_GAIN_MAX) {
		printk("         *** AE SATURATED -- analog gain at max (0x%02x), "
		       "digital gain %u+%u/256. The sensor has NO LIGHT.\n",
		       ag, dgh, dgl);
		printk("         *** This is a lighting problem, not a sensor, capture "
		       "or colour problem. Point the camera at a lit scene.\n");
	}
	{
		/*
		 * Where the loop actually stopped. "Pegged" only means no light
		 * if the loop was free to go further; if integration is sitting
		 * exactly on MAX_INTG the ceiling is what stopped it, and the
		 * headroom below is real exposure nobody is using.
		 */
		unsigned intg = ((unsigned)ih << 8) | il;
		unsigned maxi = ((unsigned)mih << 8) | mil;

		printk("         AE stopped at integration %u of MAX_INTG %u%s; "
		       "analog gain 0x%02x of MAX_AGAIN 0x%02x\n",
		       intg, maxi,
		       (maxi != 0U && intg >= maxi) ? " (AT THE CEILING)" : "",
		       ag, mag);
	}
}

int rb_camera_prepare(uint32_t want, struct rb_camera_capture *out,
		      const char **stage, const char **detail)
{
	uint8_t id_h, id_l;

	*stage = "none";
	*detail = "";

	/*
	 * Mark the diagnostics "not reached" FIRST, before any early return can
	 * fire. A zeroed struct would otherwise report readout_ok = 0, which
	 * means "all four registers read cleanly", on a capture that never got
	 * as far as opening the I2C bus -- a lie that reads as evidence.
	 */
	out->readout_ok = RB_CAMERA_READOUT_NOT_READ;
	out->x_odd_inc = out->y_odd_inc = 0xffU;
	out->binning_mode = out->qvga_win_en = 0xffU;
	out->program_ok = RB_CAMERA_PROGRAM_OFF;
	/* A caller that retries reuses `out`: never report the last try's clock. */
	out->mode_select = 0xffU;
	out->pclk_before = out->pclk_after = 0U;
	out->fvld_count = out->lvld_count = 0U;

	if (sys_read32(CAM_CAPACITY) < want + 1U) {
		return stop(stage, detail, "capacity",
			    "capture buffer is smaller than one frame");
	}

	/* MCLK first: the sensor cannot answer on SCCB without its master clock. */
	printk("         MCLK %u Hz = sysclk %u Hz / (2*(div %u + 1))\n",
	       (unsigned int)CAM_MCLK_HZ, (unsigned int)CAM_SYSCLK_HZ,
	       (unsigned int)(CAM_MCLK_DIV));
	sys_write32(CAM_MCLK_DIV, CAM_MCLKDIV);
	sys_write32(CTRL_ENABLE, CAM_CTRL);
	k_msleep(10);

	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(i2c_dev)) {
		return stop(stage, detail, "i2c-dev", "I2C controller not ready");
	}
	if (cam_reg_read(HM01B0_REG_MODEL_ID_H, &id_h) != 0 ||
	    cam_reg_read(HM01B0_REG_MODEL_ID_L, &id_l) != 0) {
		return stop(stage, detail, "i2c-nack",
			    "no ACK from the sensor at 0x24 with MCLK running");
	}
	out->model_id = (uint16_t)((id_h << 8) | id_l);
	if (out->model_id != HM01B0_MODEL_ID) {
		return stop(stage, detail, "chip-id",
			    "I2C works but this is not an HM01B0");
	}

	/*
	 * Record the readout mode before streaming starts. Read-only in the
	 * default build -- nothing here programs these, so the sensor runs on
	 * its power-up defaults and this is the only way to find out what they
	 * are. With RB_CAMERA_FULL_READOUT the write above has just happened and
	 * this becomes the read-back that says whether it took.
	 *
	 * The reason it is worth four I2C reads: the mono and Bayer-colour
	 * HM01B0 variants share model id 0x01B0, so a colour part that powers up
	 * subsampling or binning averages adjacent CFA sites and yields a frame
	 * with no recoverable mosaic -- byte-for-byte the shape of mono data.
	 * Without these values, "the picture is not colour" and "the sensor is
	 * throwing the colour away before it leaves the die" are indistinguishable.
	 *
	 * A failed read is reported, never fatal: these are diagnostics, and a
	 * sensor that will not answer them can still take a perfectly good
	 * picture.
	 */
#if RB_CAMERA_FULL_READOUT
	/*
	 * Before the probe, so the values reported below are what the sensor is
	 * ACTUALLY in after being told, not what it was told. A write that is
	 * acknowledged and then ignored is a real failure mode for an
	 * unverified register address, and only the read-back catches it.
	 */
	out->program_ok = cam_program_full_readout();
#endif

	out->readout_ok = 0;
	{
		static const uint16_t regs[4] = {
			HM01B0_REG_X_ODD_INC, HM01B0_REG_Y_ODD_INC,
			HM01B0_REG_BINNING_MODE, HM01B0_REG_QVGA_WIN_EN,
		};
		uint8_t *dst[4] = {
			&out->x_odd_inc, &out->y_odd_inc,
			&out->binning_mode, &out->qvga_win_en,
		};

		for (int i = 0; i < 4; i++) {
			int rc = cam_reg_read(regs[i], dst[i]);

			if (rc != 0) {
				out->readout_ok = rc;
				*dst[i] = 0xffU;
			}
		}
		/* Bit 0 is the whole of the documented field -- hm01b0.h
		 * quotes the datasheet. The other seven bits are undocumented
		 * here and reporting them invites reading a reserved bit as a
		 * mode change. */
		if (out->readout_ok == 0) {
			out->qvga_win_en &= HM01B0_QVGA_WIN_EN_MASK;
		}
	}

	/*
	 * A photograph, not a test pattern: cheap insurance against a previous
	 * run leaving a walking-1s pattern enabled.
	 *
	 * RB_CAMERA_TEST_PATTERN=1 turns it back on deliberately. The pattern is
	 * generated after the analog column path, so it separates a defect in
	 * the capture datapath from one in the sensor's analog readout -- an
	 * artifact present in a photograph and absent in the pattern is
	 * upstream of the bus, and cannot be a capture-core or wiring fault.
	 *
	 * RESULT, 2026-09-22: it is absent. The pattern arrives bit-exact --
	 * three distinct byte values in the whole frame, all three present on
	 * all four 2x2 phases, and every full-scale adjacent-column step lands
	 * on its exact code. So the one-phase attenuation in the photographs is
	 * UPSTREAM of the bus and no capture-core change will remove it.
	 *
	 * Do NOT read the per-2x2-phase means of a pattern frame: the pattern is
	 * a one-pixel-period column stripe, so those means measure its geometry.
	 * hm01b0.h carries the measurement.
	 */
#ifndef RB_CAMERA_TEST_PATTERN
#define RB_CAMERA_TEST_PATTERN 0
#endif
	/*
	 * D8 is the only light software controls, so it is on by default -- but
	 * it sits ON THE BOARD, inches from the lens and pointing roughly the
	 * same way. If the sensor is enclosed, covered or filmed, D8 is then the
	 * brightest thing in frame and the ONLY thing in frame, which is
	 * indistinguishable from a scene until you turn it off. Build with
	 * RB_CAMERA_STATUS_LED=0 to take the control capture that tells them
	 * apart: a bright blob that survives with the LED dark is external
	 * light; one that vanishes was always the LED.
	 */
#ifndef RB_CAMERA_STATUS_LED
#define RB_CAMERA_STATUS_LED 1
#endif
#if RB_CAMERA_STATUS_LED
	{
		int ledrc = cam_status_led_on();

		printk("         status LED (D8, the only light software controls): %s\n",
		       ledrc == 0 ? "ON" : "write failed");
	}
#else
	printk("         status LED (D8): OFF by request -- control capture, any "
	       "light in this frame came from outside the board\n");
#endif

	/*
	 * BLACK LEVEL CORRECTION OVERRIDE.
	 *
	 * BLC subtracts a dark reference and adds back BLC_TARGET, so every
	 * pixel at or below that reference leaves the sensor as exactly
	 * BLC_TARGET. Measured on this bench with the lens looking at a lit
	 * scene: 101641 of 102690 pixels came back BIT-IDENTICAL at 96, which
	 * is BLC_TARGET 32 times the 3x digital gain, with 364 at 255 and only
	 * ~55 values in between in the entire frame.
	 *
	 * A frame with NO NOISE is the finding. At maximum analog gain a live
	 * analog readout cannot produce a hundred thousand identical pixels --
	 * read noise and dark current alone would scatter them across several
	 * DN, and roughly half would land above the reference. They do not, so
	 * the dark reference BLC is subtracting sits above the signal, and the
	 * clamp -- not the scene and not the capture path -- is what flattens
	 * the picture.
	 *
	 * This tree has never written the Himax analog/BLC init block (0x1000
	 * to 0x100C), so BLC has only ever run on power-up defaults. Rather
	 * than write a register map this repository does not have, turn the
	 * clamp OFF and look: with BLC disabled the pedestal comes through raw,
	 * and noise reappearing is proof the analog path is alive and the clamp
	 * was eating the image.
	 *
	 * MEASURED, same scene, back to back:
	 *
	 *   BLC on  (0x01): range 96..255, 101641/102690 pixels BIT-IDENTICAL
	 *                   at 96, 364 at 255, ~55 intermediate values total.
	 *   BLC off (0x00): range 0..171, a graded noisy image with 55 distinct
	 *                   levels across the lit region.
	 *
	 * So OFF is the default here. Leaving it on is not a neutral choice --
	 * it is what made every frame in this investigation look like a dead
	 * sensor. Build with RB_CAMERA_BLC_CFG=0xff to leave the register
	 * alone, or =0x01 to put the clamp back deliberately.
	 */
#ifndef RB_CAMERA_BLC_CFG
#define RB_CAMERA_BLC_CFG 0x00
#endif
#if RB_CAMERA_BLC_CFG != 0xff
	{
		uint8_t back = 0xffU;
		int rc = cam_reg_write(HM01B0_REG_BLC_CFG, RB_CAMERA_BLC_CFG);

		if (rc == 0) {
			rc = cam_reg_read(HM01B0_REG_BLC_CFG, &back);
		}
		printk("         BLC_CFG forced to 0x%02x -- write %s, reads back 0x%02x%s\n",
		       (unsigned)RB_CAMERA_BLC_CFG, rc == 0 ? "ACKed" : "FAILED", back,
		       back == (uint8_t)RB_CAMERA_BLC_CFG ? "" : "  <- DID NOT STICK");
	}
#endif

	/*
	 * EXPOSURE CEILING.
	 *
	 * AE cannot expose past MAX_INTG (0x2105/06) and MAX_INTG cannot
	 * usefully exceed FRAME_LEN (0x0340/41), so the frame length is the
	 * real ceiling on how much light this sensor is allowed to collect.
	 * Measured defaults: FRAME_LEN 562 lines, MAX_INTG 340, and AE parked
	 * at integration 300 with analog gain already pegged at 0x30. The loop
	 * was out of rope, not out of scene.
	 *
	 * Line time is LINE_LEN 370 pclk at ~12.5 MHz, so 300 lines is ~8.9 ms.
	 * Stretching FRAME_LEN buys exposure in direct proportion and costs
	 * only frame rate, which a stills capture does not care about: at 4000
	 * lines a frame still completes in ~118 ms, far inside CAPTURE_TIMEOUT_MS.
	 *
	 * MEASURED, same indoor scene, back to back:
	 *
	 *   562 / 340  (the defaults):  AE pegged, analog gain 0x30, digital
	 *                               gain 3x, 99.6% of the frame at zero and
	 *                               a single 17x31 lit patch.
	 *   4000 / 3900:                AE converged -- analog gain 0x10, frame
	 *                               mean 129 against an AE target of 128,
	 *                               160 distinct levels, a sharp photograph.
	 *
	 * That one change is the difference between "the sensor has no light"
	 * and a picture. 0 leaves a register alone; set FRAME_LEN first, since
	 * raising MAX_INTG past FRAME_LEN does nothing -- the sensor cannot
	 * integrate for longer than a frame lasts.
	 *
	 * COSTS FRAME RATE: 4000 lines is ~118 ms, about 8 fps. That is free for
	 * a stills capture and wrong for a flight vision loop, which should pass
	 * a shorter FRAME_LEN and accept the noise.
	 */
#ifndef RB_CAMERA_FRAME_LEN
#define RB_CAMERA_FRAME_LEN 4000
#endif
#ifndef RB_CAMERA_MAX_INTG
#define RB_CAMERA_MAX_INTG 3900
#endif
#if RB_CAMERA_FRAME_LEN || RB_CAMERA_MAX_INTG
	{
		static const struct { uint16_t reg; uint8_t val; } ex[] = {
#if RB_CAMERA_FRAME_LEN
			{ HM01B0_REG_FRAME_LEN_H, (RB_CAMERA_FRAME_LEN >> 8) & 0xffU },
			{ HM01B0_REG_FRAME_LEN_L, RB_CAMERA_FRAME_LEN & 0xffU },
#endif
#if RB_CAMERA_MAX_INTG
			{ HM01B0_REG_MAX_INTG_H,  (RB_CAMERA_MAX_INTG >> 8) & 0xffU },
			{ HM01B0_REG_MAX_INTG_L,  RB_CAMERA_MAX_INTG & 0xffU },
#endif
		};
		int rc = cam_reg_write(HM01B0_REG_GRP_PARAM_HOLD, HM01B0_GRP_HOLD);

		for (unsigned i = 0; rc == 0 && i < ARRAY_SIZE(ex); i++) {
			rc = cam_reg_write(ex[i].reg, ex[i].val);
		}
		(void)cam_reg_write(HM01B0_REG_GRP_PARAM_HOLD, HM01B0_GRP_COMMIT);

		/* Read back: a frame-timing register that is ACKed and then
		 * ignored looks exactly like one that took, and the whole point
		 * of moving these is to know how much exposure was really
		 * bought. */
		for (unsigned i = 0; i < ARRAY_SIZE(ex); i++) {
			uint8_t back = 0xffU;

			(void)cam_reg_read(ex[i].reg, &back);
			printk("         exposure ceiling 0x%04x <- 0x%02x, reads 0x%02x%s\n",
			       ex[i].reg, ex[i].val, back,
			       back == ex[i].val ? "" : "  <- DID NOT STICK");
		}
		printk("         FRAME_LEN=%u MAX_INTG=%u requested (0 = untouched); "
		       "write %s\n", (unsigned)RB_CAMERA_FRAME_LEN,
		       (unsigned)RB_CAMERA_MAX_INTG, rc == 0 ? "ACKed" : "FAILED");
	}
#endif

	{
		int aerc = cam_set_ae_target(RB_CAMERA_AE_TARGET);

		if (aerc == 0) {
			printk("         AE target raised to %u DN (was 60 at power-up; "
			       "black level is 32, so this is ~%u DN of signal)\n",
			       (unsigned)RB_CAMERA_AE_TARGET,
			       (unsigned)(RB_CAMERA_AE_TARGET > 32
					  ? RB_CAMERA_AE_TARGET - 32 : 0));
		} else {
			printk("         AE target write FAILED (%d) -- exposure "
			       "unchanged, frames will stay dark\n", aerc);
		}
	}

	if (cam_reg_write(HM01B0_REG_TEST_PATTERN,
			  RB_CAMERA_TEST_PATTERN ? HM01B0_TESTPAT_WALKING1
						 : HM01B0_TESTPAT_OFF) != 0) {
		return stop(stage, detail, "reg-write",
			    "could not set TEST_PATTERN_MODE");
	}
	if (cam_reg_write(HM01B0_REG_MODE_SELECT, HM01B0_MODE_STREAMING) != 0) {
		return stop(stage, detail, "stream",
			    "MODE_SELECT write was not acknowledged");
	}
	/* An ACK is not a latch: read it back, so a later "pclk" failure can tell
	 * "never left standby" from "streams, but no clock reaches the core". */
	if (cam_reg_read(HM01B0_REG_MODE_SELECT, &out->mode_select) != 0) {
		out->mode_select = 0xffU;
	}
	printk("         MODE_SELECT <- 0x%02x, reads 0x%02x%s\n",
	       (unsigned)HM01B0_MODE_STREAMING, out->mode_select,
	       out->mode_select == HM01B0_MODE_STREAMING ? "" : "  <- DID NOT STICK");

	/*
	 * OPTIONAL EARLY CLOCK CHECK, off by default (the flight controller turns
	 * it on). camera_photo proves PCLK is running 100 ms after MODE_SELECT, so
	 * a sensor that shows none by then will not show any after the settle
	 * either -- and failing here costs ~150 ms instead of the whole 6 s settle,
	 * which is what makes a reset-and-retry affordable at boot.
	 */
#ifndef RB_CAMERA_EARLY_PCLK_MS
#define RB_CAMERA_EARLY_PCLK_MS 0
#endif
#if RB_CAMERA_EARLY_PCLK_MS > 0
	k_msleep(RB_CAMERA_EARLY_PCLK_MS);
	out->pclk_before = sys_read32(CAM_PCLKCNT);
	k_msleep(50);
	out->pclk_after = sys_read32(CAM_PCLKCNT);
	out->fvld_count = sys_read32(CAM_FVLDCNT);
	out->lvld_count = sys_read32(CAM_LVLDCNT);
	printk("         early PCLK %u -> %u over 50 ms, FVLD %u LVLD %u (%d ms after streaming)\n",
	       (unsigned)out->pclk_before, (unsigned)out->pclk_after,
	       (unsigned)out->fvld_count, (unsigned)out->lvld_count, RB_CAMERA_EARLY_PCLK_MS);
	if (out->pclk_after == out->pclk_before) {
		return stop(stage, detail, "pclk",
			    "no pixel clock 100 ms after MODE_SELECT=streaming (early check)");
	}
#endif
	/*
	 * AE settle. 100 ms was enough when the target was never changed --
	 * the sensor simply streamed at whatever it powered up with. Having
	 * just moved the target, the loop has to re-converge, and it does so
	 * over frames rather than instantly.
	 *
	 * Count FRAMES, not milliseconds. At FRAME_LEN 4000 a frame is ~118 ms,
	 * so 1500 ms is only 12 of them -- measured, that was enough for AE to
	 * drop digital gain but NOT to walk analog gain back down, and the frame
	 * came out with its median pixel at 255. 6000 ms is ~50 frames, and with
	 * that the frame mean lands on 129 against a target of 128. Capturing
	 * before convergence yields a frame part-way through the ramp, which
	 * looks exactly like a target that did not take.
	 */
	k_msleep(RB_CAMERA_AE_SETTLE_MS);

	/* Now, and only now, do these registers describe the frame that is about
	 * to be taken. See cam_dump_exposure(). */
	cam_dump_exposure();

	/* PCLK is only meaningful after the sensor leaves standby -- the part
	 * powers up in standby and a healthy camera drives no pixel clock until
	 * now, so checking earlier fails a working sensor. */
	out->pclk_before = sys_read32(CAM_PCLKCNT);
	k_msleep(50);
	out->pclk_after = sys_read32(CAM_PCLKCNT);
	out->fvld_count = sys_read32(CAM_FVLDCNT);
	out->lvld_count = sys_read32(CAM_LVLDCNT);
	printk("         PCLK %u -> %u over 50 ms, FVLD %u LVLD %u (after the AE settle)\n",
	       (unsigned)out->pclk_before, (unsigned)out->pclk_after,
	       (unsigned)out->fvld_count, (unsigned)out->lvld_count);
	if (out->pclk_after == out->pclk_before) {
		return stop(stage, detail, "pclk",
			    "no pixel clock after MODE_SELECT=streaming");
	}
	if (out->fvld_count == 0U || out->lvld_count == 0U) {
		return stop(stage, detail, "sync",
			    "sensor is clocked but FVLD/LVLD never asserted");
	}

	/* Streaming, clocked and in sync: the next thing is to arm, and a caller that
	 * cannot block (the flight controller) arms later, from its own loop. */
	return 0;
}

/*
 * Back to the power-up state, whatever a previous image left behind. Nothing on
 * this board power-cycles the sensor between loads (USB keeps it up with the pack
 * out), a failed prepare returns with the sensor still streaming, and `rb fly load`
 * reprograms the FPGA -- stopping MCLK -- underneath a sensor in that state. The
 * proven single-shot paths (camera_photo, bootup_check) only ever start from a
 * sensor a previous run put back in standby.
 *
 * SW_RESET 0x0103 is the first write of Himax's reference init; the value is
 * ignored. The part may NAK the write it resets on, so that is not an error; the
 * model id answering again is the test.
 */
#ifndef RB_CAMERA_RESET_WAIT_MS
#define RB_CAMERA_RESET_WAIT_MS 20
#endif
int rb_camera_sensor_reset(void)
{
	uint8_t id_h = 0, id_l = 0;

	sys_write32(CAM_MCLK_DIV, CAM_MCLKDIV);
	sys_write32(CTRL_ENABLE, CAM_CTRL);
	k_msleep(10);
	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(i2c_dev)) {
		return -1;
	}
	(void)cam_reg_write(HM01B0_REG_MODE_SELECT, HM01B0_MODE_STANDBY);
	(void)cam_reg_write(HM01B0_REG_SW_RESET, HM01B0_SW_RESET_VALUE);
	for (int i = 0; i < 10; i++) {
		k_msleep(RB_CAMERA_RESET_WAIT_MS);
		if (cam_reg_read(HM01B0_REG_MODEL_ID_H, &id_h) == 0 &&
		    cam_reg_read(HM01B0_REG_MODEL_ID_L, &id_l) == 0 &&
		    (uint16_t)((id_h << 8) | id_l) == HM01B0_MODEL_ID) {
			return 0;
		}
	}
	return -2;
}

void rb_camera_standby(void)
{
	if (i2c_dev != NULL) {
		(void)cam_reg_write(HM01B0_REG_MODE_SELECT, HM01B0_MODE_STANDBY);
	}
}

void rb_camera_arm(uint32_t want)
{
	/* Four register writes, no I2C, no printk, no sleep -- safe inside a control loop.
	 * Arming mid-frame is fine: the capture starts at the next boundary the core sees
	 * and the PNG tools re-align on the line markers. */
	sys_write32(CTRL_ENABLE | CTRL_CLEAR, CAM_CTRL);
	sys_write32(CTRL_ENABLE | CTRL_FLUSH, CAM_CTRL);
	sys_write32(want, CAM_PIXTARGET);
	sys_write32(CTRL_ENABLE | CTRL_ARM, CAM_CTRL);
}

bool rb_camera_done(uint32_t want)
{
	return sys_read32(CAM_CAPCOUNT) >= want ||
	       (sys_read32(CAM_CAPSTAT) & CAPSTAT_DONE) != 0U;
}

uint32_t rb_camera_frames(void)
{
	return sys_read32(CAM_FVLDCNT);
}

int rb_camera_finish(uint8_t *dst, uint32_t want, struct rb_camera_capture *out,
		     const char **stage, const char **detail)
{
	uint32_t captured, n = 0, word;

	/*
	 * Read the count again now that the loop has stopped. The core sets
	 * CAPCOUNT to the target and CAPSTAT.DONE in the same cycle
	 * (hardware/ospi HM01B0Capture.scala, the bounded-capture gate), so the
	 * iteration that sees DONE can hold a CAPCOUNT sampled just before the
	 * last pixel landed. Trusting that stale value failed a complete frame
	 * as "overflow -- the frame did not complete" on 2 of 4 runs on the
	 * 35 MHz Fp16At35 shell, with CAPCOUNT reading the full target
	 * afterwards. Once DONE is set the count no longer moves, and after a
	 * timeout a re-read is still short, so this cannot pass a short frame.
	 */
	captured = sys_read32(CAM_CAPCOUNT);
	out->captured = captured;

	/*
	 * Stop the sensor the moment the frame is satisfied. The CDC FIFO is
	 * 1024 beats and the sensor streams at several MHz, so it refills faster
	 * than a polling CPU drains it; leaving it running means OVERFLOW is set
	 * by surplus pixels arriving after PIXTARGET. That is data nobody asked
	 * for, not data that was lost -- which is why overflow is only fatal
	 * below when the count also came up short.
	 */
	(void)cam_reg_write(HM01B0_REG_MODE_SELECT, HM01B0_MODE_STANDBY);
	sys_write32(0U, CAM_CTRL);
	out->flags = sys_read32(CAM_FLAGS);

	if (captured < want) {
		return stop(stage, detail,
			    (out->flags & FLAG_OVERFLOW) ? "overflow" : "timeout",
			    "the frame did not complete");
	}

	while (n < want) {
		word = sys_read32(CAM_DATA);
		if ((word & DATA_VALID) == 0U) {
			break;
		}
		if (word & DATA_EOF) {
			continue;   /* in-band end-of-frame marker, not a pixel */
		}
		dst[n++] = (uint8_t)(word & DATA_PIXEL_MASK);
	}
	out->drained = n;
	out->measured_width = sys_read32(CAM_LASTWIDTH);
	out->measured_height = sys_read32(CAM_LASTHEIGHT);
	out->configured_width = sys_read32(CAM_GEOM) & 0xffffU;
	out->configured_height = (sys_read32(CAM_GEOM) >> 16) & 0xffffU;

	out->vmin = 0xff;
	out->vmax = 0x00;
	for (uint32_t i = 0; i < n; i++) {
		if (dst[i] < out->vmin) { out->vmin = dst[i]; }
		if (dst[i] > out->vmax) { out->vmax = dst[i]; }
	}
	if (n == 0U) {
		return stop(stage, detail, "drain",
			    "capture reported done but the buffer returned no pixels");
	}
	if (out->vmin == out->vmax) {
		return stop(stage, detail, "constant-data",
			    "every pixel identical: the data bus is stuck");
	}
	return 0;
}

int rb_camera_capture_frame(uint8_t *dst, uint32_t want,
			    struct rb_camera_capture *out,
			    const char **stage, const char **detail)
{
	int64_t deadline;

	if (rb_camera_prepare(want, out, stage, detail) != 0) {
		return -1;
	}
	rb_camera_arm(want);
	deadline = k_uptime_get() + CAPTURE_TIMEOUT_MS;
	while (!rb_camera_done(want) && k_uptime_get() < deadline) {
	}
	return rb_camera_finish(dst, want, out, stage, detail);
}
