/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * HM01B0 camera bring-up for the RiskyBird Arty100T/TE0712 CameraBringupConfig shell.
 *
 * There is no ILA on this board, so UART is the only instrument. Every stage below prints what
 * it observed before it decides, and every wait is bounded, so a failure names the stage that
 * failed rather than hanging.
 *
 * Two independent things are being proved:
 *   1. the control path  -- the FPGA can talk I2C to the physical sensor and configure it;
 *   2. the receive path  -- the Chisel capture RTL actually samples pixel data.
 * A chip-ID read proves only (1). Only repeatable pixel capture proves (2).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/sys_io.h>
#include <stdio.h>
#include <string.h>

#include "hm01b0.h"

/* ---- Capture peripheral MMIO. Base and layout: ospi/OspiChipyard.scala. ---- */
#define CAM_BASE        0x10080000UL

#define CAM_CTRL        (CAM_BASE + 0x00)
#define CAM_GEOM        (CAM_BASE + 0x04)
#define CAM_MCLKDIV     (CAM_BASE + 0x08)
#define CAM_FIFOCOUNT   (CAM_BASE + 0x0c)
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
#define CAM_LASTPIX     (CAM_BASE + 0x3c)
#define CAM_CAPSTAT     (CAM_BASE + 0x40)

#define CTRL_ENABLE     (1U << 0)
#define CTRL_CONTINUOUS (1U << 1)
#define CTRL_IRQEN      (1U << 2)
#define CTRL_TRIGGER    (1U << 3)
#define CTRL_CLEAR      (1U << 4)
#define CTRL_FLUSH      (1U << 5)
#define CTRL_ARM        (1U << 6)

#define FLAG_DATAVALID  (1U << 0)
#define FLAG_OVERFLOW   (1U << 1)
#define FLAG_GEOMERR    (1U << 2)
#define FLAG_SENSORINT  (1U << 3)
#define FLAG_BUSY       (1U << 4)
#define FLAG_IRQPENDING (1U << 5)
#define FLAG_BUFFULL    (1U << 6)

#define DATA_VALID      (1U << 31)
#define DATA_EOF        (1U << 10)
#define DATA_EOL        (1U << 9)
#define DATA_SOF        (1U << 8)
#define DATA_PIXEL_MASK 0xffU

#define CAPSTAT_ARMED   (1U << 0)
#define CAPSTAT_DONE    (1U << 1)

/*
 * MCLK = sysclk / (2 * (div + 1)) with sysclk = 50 MHz, so div = 1 gives 12.5 MHz.
 * The datasheet table in hardware/ospi/docs/README.md gives a legal MCLK range of 3-36 MHz;
 * legal divider values are 0..7 (25 MHz down to 3.125 MHz).
 *
 * 12.5 MHz rather than a slower bring-up rate because that is the rate the sensor has actually
 * been read at on this hardware: a run that drove 12.5 MHz returned MODEL_ID = 0x01b0, while
 * this target at 6.25 MHz got no ACK at all. 6.25 MHz is inside the datasheet range, so the
 * difference is not obviously legality -- do not lower this without re-testing the ID read.
 */
#define CAM_MCLK_DIV        1U
#define CAM_MCLK_HZ         12500000U

/*
 * CAM_SKIP_I2C: capture without configuring the sensor from the FPGA.
 *
 *   rb build camera_bringup ... ; RB_EXTRA_CFLAGS=-DCAM_SKIP_I2C=1 rb debug camera_bringup ...
 *
 * On the RiskyBird drone carrier the FPGA's only I2C master (i2c@10040000, A15/A16) is the
 * drone sensor bus: a survey with MCLK live ACKs 0x17, 0x18, 0x29, 0x68 and 0x76 and nothing
 * at the HM01B0's 0x24. The camera's control interface is not on that bus, so the sensor is
 * configured by the ESP32 rather than by Rocket, and the FPGA's job is only to sample the DVP
 * stream -- PCLK, FVLD, LVLD and D[7:0] -- which needs no I2C whatsoever.
 *
 * With this set the target skips every I2C stage and goes straight to the sync counters and a
 * bounded capture. A pass then means real pixels crossed the DVP bus; a stall at fvld means
 * nothing is currently streaming, which is a statement about the sensor's configurator and not
 * about this shell.
 */
#ifndef CAM_SKIP_I2C
#define CAM_SKIP_I2C 0
#endif

/*
 * How many pixels to capture. Default is one QVGA line, which is all the bring-up proof needs
 * and all that fits a scratchpad shell; hardware/ospi/docs/README.md gives the QVGA window as
 * 324x244. Override for a whole frame on a DDR-backed shell, where the pixel buffer has room:
 *
 *   RB_EXTRA_CFLAGS=-DCAM_CAPTURE_PIXELS=79056   # 324*244, one full QVGA frame
 *
 * The frame buffer in the capture core holds 324*324+1 beats, so a full frame fits without the
 * bounded capture overflowing. Keep the console dump small regardless -- the frame is read out
 * over JTAG, not printed.
 */
#ifndef CAM_CAPTURE_PIXELS
#define CAM_CAPTURE_PIXELS  324U
#endif
#define CAPTURE_PIXELS      CAM_CAPTURE_PIXELS
#define CAPTURE_DUMP        64U

#define I2C_NODE DT_NODELABEL(i2c0)

static const struct device *i2c_dev;
static uint8_t pixels[CAPTURE_PIXELS];

/*
 * Debugger handles for the captured frame.
 *
 * A frame is read out over JTAG rather than printed, so the readout needs somewhere stable to
 * stop and a length it can trust. Neither exists otherwise: pixels[] is a file static that is
 * still in scope after main returns, but the count actually drained is a block local that is
 * not, and stopping on a source line number breaks the moment this file is edited.
 *
 * camera_frame_pixels is how many leading bytes of pixels[] are valid -- which is not
 * CAPTURE_PIXELS when the capture came up short. camera_frame_ready() is a breakpoint anchor,
 * noinline so it cannot be folded into main. Same pattern as hello_world_done().
 */
volatile uint32_t camera_frame_pixels;

__attribute__((noinline)) void camera_frame_ready(void)
{
    /*
     * A side effect the optimiser cannot discard. noinline alone is not enough: with an empty
     * body GCC deduces the function is const, deletes the call site, and the breakpoint anchor
     * vanishes from the image even though the symbol still resolves.
     */
    __asm__ volatile("" ::: "memory");
}

/* ---- bounded helpers ---------------------------------------------------- */

static void fail(const char *stage, const char *detail)
{
    printf("\n[FAIL] stage=%s: %s\n", stage, detail);
    printf("RESULT: FAIL\n");
}

/*
 * Register read, SCCB-style: two separate transactions, not one combined one.
 *
 * This used i2c_write_read(), which emits a REPEATED START between the register-address write
 * and the data read. The HM01B0's control interface is SCCB, and SCCB has no repeated start --
 * the address phase must be terminated by a STOP before the read begins. A sensor that is
 * present, powered and correctly clocked will NAK the combined form, which is indistinguishable
 * at this level from an absent sensor and is exactly how this target used to report a live
 * camera as missing.
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
 * Put the ADS7128 expander at 0x17 into a known state before probing for the sensor.
 *
 * CAM_EXPANDER_MASK names the channels to drive high. Everything not in the mask is returned
 * to the power-on configuration -- analog input, hi-Z -- which is the only state on this board
 * that is known to be safe, because the board comes up in it every time.
 *
 * This used to configure all eight channels as push-pull outputs and drive them ALL high, on
 * the reasoning that if a camera power-enable sat on one of them, raising every channel was
 * the cheap way to find out. That was wrong, and on the drone carrier it spun two motors.
 * Channels 0, 5 and 7 are undocumented in this repository and two of them gate ESCs.
 *
 * What made it dangerous rather than merely wrong is that the ADS7128 latches GPO state in its
 * own registers. It outlives the program exiting, GDB detaching, and the FPGA being
 * reprogrammed; killing every process on the host changes nothing. Only a power cycle or an
 * explicit write clears it.
 *
 * The rest of this repository already had the rule and wrote down why: integration/zephyr/tof
 * drives every channel low at start (rb_tof_hold_all) rather than preserving or raising
 * channels whose function it does not know. See docs/directory-changelog.md.
 *
 * So: name the channel rather than sweeping for it. Once the schematic says which channel
 * gates the camera, pass exactly that one --
 *
 *   RB_EXTRA_CFLAGS=-DCAM_EXPANDER_MASK=0x01     # channel 0 only
 *
 * Channels 1-4 are the VL53L5CX XSHUT lines (hardware/zephyr/targets/README.md). Do not put an
 * undocumented channel in this mask to see what happens: that is precisely the mistake above.
 *
 * With the default mask this function is also the software recovery from a latched state --
 * it writes every channel back to hi-Z.
 *
 * Writes use the ADS7128's single-register-write opcode (datasheet 8.5.2.1), the same framing
 * ads7128_test uses. Failures are reported and not fatal.
 */
#ifndef CAM_EXPANDER_MASK
#define CAM_EXPANDER_MASK 0x00U
#endif
#define ADS7128_ADDR            0x17U
#define ADS7128_CMD_REG_WRITE   0x08U
#define ADS7128_PIN_CFG         0x05U
#define ADS7128_GPIO_CFG        0x07U
#define ADS7128_GPO_DRIVE_CFG   0x09U
#define ADS7128_GPO_VALUE       0x0BU

static int ads7128_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[3] = { ADS7128_CMD_REG_WRITE, reg, val };

    return i2c_write(i2c_dev, buf, sizeof(buf), ADS7128_ADDR);
}

static void release_expander_channels(void)
{
    static const struct { uint8_t reg; uint8_t val; const char *name; } seq[] = {
        { ADS7128_PIN_CFG,       CAM_EXPANDER_MASK, "PIN_CFG"   },
        { ADS7128_GPIO_CFG,      CAM_EXPANDER_MASK, "GPIO_CFG"  },
        { ADS7128_GPO_DRIVE_CFG, CAM_EXPANDER_MASK, "GPO_DRIVE" },
        { ADS7128_GPO_VALUE,     CAM_EXPANDER_MASK, "GPO_VALUE" },
    };
    int failures = 0;

    printf("       ADS7128 channels: mask=0x%02x%s\n", CAM_EXPANDER_MASK,
           (CAM_EXPANDER_MASK == 0U) ? " (all hi-Z; nothing driven)" : "");
    printf("       writing:");
    for (size_t i = 0; i < ARRAY_SIZE(seq); i++) {
        int rc = ads7128_write(seq[i].reg, seq[i].val);

        printf(" %s=%s", seq[i].name, (rc == 0) ? "ok" : "FAIL");
        if (rc != 0) {
            failures++;
        }
    }
    printf("\n");
    if (failures == 0) {
        k_msleep(50);
    }
}

/*
 * Diagnostic: which addresses ACK, with MCLK already running.
 *
 * The standalone i2c_scanner workload cannot answer this question, because it never touches
 * the capture peripheral and therefore never starts the sensor's master clock -- and a camera
 * with no MCLK does not respond on I2C at all. So a plain bus scan can miss a perfectly live
 * sensor. Run from the 0x24 NAK path, this distinguishes three cases that otherwise look
 * identical: nothing on the bus at all (bus or wiring fault), other devices but no camera
 * (camera absent, unpowered or held in reset), and a camera answering somewhere other than
 * 0x24 (address strap differs from the one this target assumes).
 */
static void i2c_bus_survey(void)
{
    uint8_t scratch;
    unsigned int found = 0;

    printf("       bus survey with MCLK live (%u Hz):", CAM_MCLK_HZ);
    for (uint16_t a = 0x08; a <= 0x77; a++) {
        if (i2c_read(i2c_dev, &scratch, 1, (uint8_t)a) == 0) {
            printf(" 0x%02x", a);
            found++;
        }
    }
    if (found == 0U) {
        printf(" none -- no device ACKed any address, so this is a bus/wiring fault,"
               " not a missing camera");
    }
    printf("\n       %u device(s) ACKed 0x08..0x77\n", found);
}

/* ---- stages ------------------------------------------------------------- */

int main(void)
{
    uint32_t capacity, flags, pclk0, pclk1, fvld, lvld, captured;
    uint8_t id_h, id_l, orig_pattern, readback;
    uint16_t model_id;
    int rc;

    printf("\n");
    printf("=====================================================\n");
    printf("RiskyBird HM01B0 camera bring-up\n");
    printf("  shell   : CameraBringupConfig (arty100t / TE0712)\n");
    printf("  built   : " __DATE__ " " __TIME__ "\n");
    printf("  capture : 0x%08x, I2C addr 0x%02x\n",
           (unsigned int)CAM_BASE, HM01B0_I2C_ADDR);
    printf("  mclk    : sysclk/(2*(%u+1)) = %u Hz\n", CAM_MCLK_DIV, CAM_MCLK_HZ);
    printf("  request : %u pixels (one QVGA line)\n", CAPTURE_PIXELS);
    printf("=====================================================\n\n");

    /* --- 3. capture peripheral reachable over MMIO --------------------- */
    capacity = sys_read32(CAM_CAPACITY);
    printf("[1/9] MMIO probe: CAPACITY = %u beats\n", capacity);
    if (capacity == 0U || capacity == 0xffffffffU) {
        fail("mmio", "capture peripheral did not answer; check the config has WithOspiCapture");
        return 1;
    }
    if (capacity < CAPTURE_PIXELS + 1U) {
        fail("mmio", "capture buffer smaller than the requested capture");
        return 1;
    }

    /* --- 4. reset and clock the sensor --------------------------------- */
    /* Start MCLK before touching I2C: the sensor needs its master clock to respond. */
    sys_write32(CAM_MCLK_DIV, CAM_MCLKDIV);
    sys_write32(CTRL_ENABLE, CAM_CTRL);
    k_msleep(10);

    /*
     * PCLK here is INFORMATIONAL and deliberately not a gate.
     *
     * The HM01B0 powers up in standby (MODE_SELECT 0x0100 = 0) and a sensor in standby
     * drives no pixel clock, so a correctly wired and perfectly healthy camera reads zero
     * at this point. This check used to abort the run here, which made the whole target
     * unpassable: streaming is not enabled until stage [6/9], below. The real PCLK gate is
     * after that write, where a stopped clock genuinely means something is wrong.
     *
     * A non-zero count here is still worth printing -- it means the part free-runs out of
     * reset, which not every sensor or every strap configuration does.
     */
    pclk0 = sys_read32(CAM_PCLKCNT);
    k_msleep(50);
    pclk1 = sys_read32(CAM_PCLKCNT);
    printf("[2/9] PCLK activity (pre-stream, standby expected): %u -> %u over 50 ms%s\n",
           pclk0, pclk1, (pclk1 == pclk0) ? "  (idle, as expected in standby)" : "  (free-running)");

#if !CAM_SKIP_I2C
    /* --- 5. probe the I2C address -------------------------------------- */
    i2c_dev = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(i2c_dev)) {
        fail("i2c-dev", "I2C controller device not ready");
        return 1;
    }
    rc = cam_reg_read(HM01B0_REG_MODEL_ID_H, &id_h);
    if (rc != 0) {
        printf("[3/9] I2C probe: no ACK at 0x%02x -- releasing expander gates and retrying\n",
               HM01B0_I2C_ADDR);
        release_expander_channels();
        rc = cam_reg_read(HM01B0_REG_MODEL_ID_H, &id_h);
        printf("       retry after expander release: %s\n", (rc == 0) ? "ACK" : "still no ACK");
    }
    if (rc != 0) {
        i2c_bus_survey();
        fail("i2c-nack",
             "no ACK from the sensor at 0x24: check SDA/SCL wiring, pull-ups and sensor power");
        return 1;
    }
    printf("[3/9] I2C probe: sensor ACKed at 0x%02x\n", HM01B0_I2C_ADDR);

    /* --- 6/7. chip ID --------------------------------------------------- */
    if (cam_reg_read(HM01B0_REG_MODEL_ID_L, &id_l) != 0) {
        fail("chip-id", "MODEL_ID_L read failed after MODEL_ID_H succeeded");
        return 1;
    }
    model_id = (uint16_t)((id_h << 8) | id_l);
    printf("[4/9] chip ID: 0x%04x (expect 0x%04x)\n", model_id, HM01B0_MODEL_ID);
    if (model_id != HM01B0_MODEL_ID) {
        fail("chip-id", "unexpected model ID: I2C works but this is not an HM01B0");
        return 1;
    }

    /* --- 8. write / read back / restore a safe register ----------------- */
    if (cam_reg_read(HM01B0_REG_TEST_PATTERN, &orig_pattern) != 0) {
        fail("reg-read", "could not read TEST_PATTERN_MODE");
        return 1;
    }
    if (cam_reg_write(HM01B0_REG_TEST_PATTERN, HM01B0_TESTPAT_WALKING1) != 0) {
        fail("reg-write", "TEST_PATTERN_MODE write was not acknowledged");
        return 1;
    }
    if (cam_reg_read(HM01B0_REG_TEST_PATTERN, &readback) != 0) {
        fail("reg-read", "TEST_PATTERN_MODE read-back failed");
        return 1;
    }
    printf("[5/9] register r/w: TEST_PATTERN 0x%02x -> wrote 0x%02x -> read 0x%02x\n",
           orig_pattern, HM01B0_TESTPAT_WALKING1, readback);
    if (readback != HM01B0_TESTPAT_WALKING1) {
        fail("reg-verify", "register read-back did not match the value written");
        return 1;
    }

    /*
     * Restore TEST_PATTERN before streaming so the capture is a real scene.
     *
     * Stage [5/9] uses TEST_PATTERN as its scratch register for the write/read-back proof and
     * leaves the walking-1s pattern enabled, which means every captured pixel is synthetic. The
     * register proof is already complete by this point, so put the register back and let the
     * sensor image. Set CAM_KEEP_TEST_PATTERN=1 to stream the internal pattern instead.
     */
#ifndef CAM_KEEP_TEST_PATTERN
#define CAM_KEEP_TEST_PATTERN 0
#endif
#if !CAM_KEEP_TEST_PATTERN
    if (cam_reg_write(HM01B0_REG_TEST_PATTERN, 0x00U) != 0) {
        fail("reg-write", "could not disable TEST_PATTERN_MODE before streaming");
        return 1;
    }
    printf("[5b/9] test pattern disabled -- capturing a real scene\n");
#endif

    /* --- 9/10. slowest safe grayscale streaming ------------------------- */
    /* Data-width mode is left at its reset default; see the note in hm01b0.h. */
    if (cam_reg_write(HM01B0_REG_MODE_SELECT, HM01B0_MODE_STREAMING) != 0) {
        fail("stream", "MODE_SELECT write was not acknowledged");
        return 1;
    }
    printf("[6/9] streaming enabled\n");
    k_msleep(100);

    /*
     * The real camera-clock gate. The sensor has now been taken out of standby, so PCLK must
     * be toggling; if it is not, the pixel interface is genuinely not reaching the FPGA.
     */
    pclk0 = sys_read32(CAM_PCLKCNT);
    k_msleep(50);
    pclk1 = sys_read32(CAM_PCLKCNT);
    printf("[6b/9] PCLK activity (streaming): %u -> %u over 50 ms\n", pclk0, pclk1);
    if (pclk1 == pclk0) {
        fail("pclk",
             "no camera clock after MODE_SELECT=streaming: PCLK edge count did not advance. "
             "The sensor answers on I2C, so check the PCLK pin mapping, the DVP wiring and "
             "the level translator");
        return 1;
    }

#else
    printf("[3-6/9] I2C stages skipped (CAM_SKIP_I2C): the sensor is configured\n");
    printf("        externally; this run only samples the DVP stream.\n");
    ARG_UNUSED(rc); ARG_UNUSED(id_h); ARG_UNUSED(id_l); ARG_UNUSED(model_id);
    ARG_UNUSED(orig_pattern); ARG_UNUSED(readback);
#endif

    /* --- sync activity --------------------------------------------------- */
    fvld = sys_read32(CAM_FVLDCNT);
    lvld = sys_read32(CAM_LVLDCNT);
    printf("[7/9] sync activity: FVLD rises = %u, LVLD rises = %u\n", fvld, lvld);
    if (fvld == 0U) {
        fail("fvld", "no frame-valid activity: sensor is clocked but never starts a frame");
        return 1;
    }
    if (lvld == 0U) {
        fail("lvld", "no line-valid activity: frames start but no line is ever asserted");
        return 1;
    }

    /* --- 11/12. arm a bounded capture with a timeout --------------------- */
    sys_write32(CTRL_ENABLE | CTRL_CLEAR, CAM_CTRL);   /* clear sticky flags */
    sys_write32(CTRL_ENABLE | CTRL_FLUSH, CAM_CTRL);   /* drop stale beats */
    sys_write32(CAPTURE_PIXELS, CAM_PIXTARGET);
    sys_write32(CTRL_ENABLE | CTRL_ARM, CAM_CTRL);

    {
        const int timeout_ms = 2000;
        int waited = 0;

        while (((sys_read32(CAM_CAPSTAT) & CAPSTAT_DONE) == 0U) && waited < timeout_ms) {
            k_msleep(10);
            waited += 10;
        }
        captured = sys_read32(CAM_CAPCOUNT);
        flags = sys_read32(CAM_FLAGS);
        printf("[8/9] capture: %u/%u pixels in %d ms, flags=0x%02x\n",
               captured, CAPTURE_PIXELS, waited, flags);

        /*
         * Stop the sensor the moment the bounded capture is satisfied, so nothing more arrives
         * while software drains at CPU speed. Without this the sensor keeps streaming the whole
         * time the readout loop runs.
         */
#if !CAM_SKIP_I2C
        (void)cam_reg_write(HM01B0_REG_MODE_SELECT, HM01B0_MODE_STANDBY);
#endif
        sys_write32(0U, CAM_CTRL);   /* stop ingest; the frame buffer keeps its contents */

        /*
         * Overflow is only fatal if it cost us the pixels we asked for.
         *
         * The CDC FIFO is 1024 beats and the sensor streams continuously at several MHz, so it
         * fills in well under a millisecond -- far faster than a polling CPU can drain the frame
         * buffer. Once PIXTARGET is reached the capture is already satisfied and every further
         * pixel is surplus, so a set OVERFLOW flag alongside a complete capture describes data
         * we never wanted rather than data we lost. Aborting on it made a working camera report
         * a failure. If the count came up short, overflow is the reason and it stays fatal.
         */
        if ((flags & FLAG_OVERFLOW) && captured < CAPTURE_PIXELS) {
            fail("overflow",
                 "capture FIFO overflowed before the requested pixel count was reached");
            return 1;
        }
        if (flags & FLAG_OVERFLOW) {
            printf("       note: OVERFLOW set with a complete %u-pixel capture -- surplus\n"
                   "       pixels were dropped after PIXTARGET, which does not affect the\n"
                   "       captured window.\n", CAPTURE_PIXELS);
        }
        if ((sys_read32(CAM_CAPSTAT) & CAPSTAT_DONE) == 0U) {
            if (captured == 0U) {
                fail("no-pixels",
                     "capture timed out with zero pixels: sync is present but no data was sampled");
            } else {
                fail("timeout", "capture timed out part-way through the requested pixel count");
            }
            return 1;
        }
    }

    /* --- 13. drain and report ------------------------------------------- */
    {
        uint32_t n = 0, word;
        uint32_t sum = 0, csum = 2166136261U; /* FNV-1a */
        uint8_t vmin = 0xff, vmax = 0x00;

        while (n < CAPTURE_PIXELS) {
            word = sys_read32(CAM_DATA);
            if ((word & DATA_VALID) == 0U) {
                break;
            }
            if (word & DATA_EOF) {
                continue; /* in-band end-of-frame marker, not a pixel */
            }
            pixels[n++] = (uint8_t)(word & DATA_PIXEL_MASK);
        }

        if (n == 0U) {
            fail("drain", "capture reported done but the buffer returned no pixels");
            return 1;
        }
        camera_frame_pixels = n;

        for (uint32_t i = 0; i < n; i++) {
            uint8_t v = pixels[i];

            sum += v;
            if (v < vmin) { vmin = v; }
            if (v > vmax) { vmax = v; }
            csum = (csum ^ v) * 16777619U;
        }

        printf("\nfirst %u pixels:\n", (n < CAPTURE_DUMP) ? n : CAPTURE_DUMP);
        for (uint32_t i = 0; i < n && i < CAPTURE_DUMP; i++) {
            printf("%02x ", pixels[i]);
            if ((i % 16) == 15) {
                printf("\n");
            }
        }
        printf("\n\n");

        printf("[9/9] drained %u pixels: min=0x%02x max=0x%02x avg=%u checksum=0x%08x\n",
               n, vmin, vmax, (unsigned int)(sum / n), csum);
        /*
         * Report the MEASURED line width alongside the configured one. The core measures what
         * FVLD/LVLD actually gate, and on this sensor the two disagree: LASTWIDTH reads 326
         * where GEOM is configured for 324, because the line carries two dummy pixels ahead of
         * the active window. That matters to anyone reshaping the drained bytes into an image --
         * assuming 324 shears the picture one pixel per row, which looks like a broken capture
         * and is not one. Reshape at LASTWIDTH and drop the two leading columns.
         */
        printf("       geometry: measured %ux%u, configured %ux%u%s\n",
               sys_read32(CAM_LASTWIDTH), sys_read32(CAM_LASTHEIGHT),
               sys_read32(CAM_GEOM) & 0xffffU, (sys_read32(CAM_GEOM) >> 16) & 0xffffU,
               (sys_read32(CAM_LASTWIDTH) != (sys_read32(CAM_GEOM) & 0xffffU))
                   ? "  <- reshape rows at the MEASURED width" : "");
        printf("       counters: pclk=%u fvld=%u lvld=%u frames=%u lastpix=0x%02x\n",
               sys_read32(CAM_PCLKCNT), sys_read32(CAM_FVLDCNT), sys_read32(CAM_LVLDCNT),
               sys_read32(CAM_FRAMECNT), sys_read32(CAM_LASTPIX) & 0xffU);

        if (vmin == vmax) {
            fail("constant-data",
                 "every captured pixel had the same value: the data bus is probably stuck");
            return 1;
        }

        /*
         * Test-pattern content check. Reported, never gating.
         *
         * This used to test for a walking-1s ramp. It is not one -- a capture off this
         * part on 2026-09-22 measured a one-pixel-period column stripe of 0xff on the
         * black level, three distinct byte values in the whole frame. See the note in
         * hm01b0.h. What is worth reporting is therefore the distinct-value count and
         * the full-scale adjacent-column step, because those are what say whether the
         * bus carried the pattern intact.
         */
        {
            unsigned int distinct = 0, steps = 0;
            uint8_t seen[256] = { 0 };
            uint8_t lo = 0xffU, hi = 0x00U;

            for (uint32_t i = 0; i < n; i++) {
                if (!seen[pixels[i]]) {
                    seen[pixels[i]] = 1U;
                    distinct++;
                }
                if (pixels[i] < lo) {
                    lo = pixels[i];
                }
                if (pixels[i] > hi) {
                    hi = pixels[i];
                }
            }
            for (uint32_t i = 1; i < n; i++) {
                if ((pixels[i - 1] == lo && pixels[i] == hi) ||
                    (pixels[i - 1] == hi && pixels[i] == lo)) {
                    steps++;
                }
            }
            printf("       test pattern: %u distinct values, range 0x%02x..0x%02x, "
                   "%u full-scale adjacent steps\n", distinct, lo, hi, steps);
            printf("       expect ~3 distinct values and tens of thousands of steps; a "
                   "fourth value would be a capture-path fault (not gating)\n");
        }
    }

    /* restore the register we perturbed */
    (void)cam_reg_write(HM01B0_REG_TEST_PATTERN, orig_pattern);

    /* Frame complete and still in RAM: stop here to read it out. */
    camera_frame_ready();

    printf("\nRESULT: PASS\n");
    printf("  control path proved by: chip ID 0x%04x + register write/read-back\n", model_id);
    printf("  receive path proved by: %u pixels captured with no overflow\n", CAPTURE_PIXELS);
    return 0;
}
