/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Himax HM01B0 register definitions used by the camera bring-up program.
 *
 * PROVENANCE. Two different levels of confidence live in this file and they are
 * labelled, because bring-up debugging is impossible if a wrong constant is
 * indistinguishable from a wrong wire.
 *
 *  VERIFIED IN THIS REPOSITORY
 *    - 7-bit I2C address 0x24. Recorded in hardware/ospi/docs/README.md
 *      ("Chipyard's TLI2C controller configures the HM01B0 at its default 7-bit
 *      address 0x24") and repeated in the WithArty100TI2C binder comment.
 *    - 16-bit register address, 8-bit register data; I2C at most 400 kHz.
 *      Same source, "Datasheet conformance boundary" table.
 *    - 8-bit parallel output with PCLK/FVLD/LVLD, maximum PCLK 36 MHz,
 *      MCLK 3-36 MHz. Same table.
 *
 *  VERIFIED AGAINST A DATASHEET ON THIS MACHINE. There is one, found
 *  2026-09-21 at /scratch2/loren/riskybird-dev-old/HM01B0.pdf -- "DATA SHEET
 *  (DOC No. HM01B0-ANA-00FT870-DS), Himax Imaging, Preliminary version 01,
 *  July 2021". It is the CAMERA MODULE datasheet, twenty pages, and it does
 *  NOT contain a register map; it names exactly one register. What it settles:
 *    - 0x3010[0] = 1 selects the QVGA window. Section 1.4, quoted: "The QVGA
 *      sensor window with an active resolution of 324 x 244 pixels is
 *      programmed by setting register 0x3010[0] to 1. The location of the
 *      window is fixed such that the coordinate of the first pixel read out
 *      location is 0, 0." So the polarity below is right, it is BIT 0 and not
 *      the whole byte, and the window is a CROP -- no pixel is skipped or
 *      averaged, so QVGA cannot destroy a mosaic.
 *    - Active array 324 x 324; the QVGA window is 324 x 244. Both are 324 wide.
 *      Section 1 and 1.1.
 *    - A "2x2 monochrome binning mode" exists (section 1), which is what makes
 *      BINNING_MODE worth reading -- but note it is 2x2, so it would halve the
 *      WIDTH as well, and the width this board measures is not halved.
 *    - Table 2.1 note: "HM01B0 sensor default slave address: 0x24."
 *    - I2C 400 kHz max, MCLK 3-36 MHz, PCLK 36 MHz max, 1/4/8-bit data
 *      interface, 6-bit/8-bit RAW output. Key-parameter table, section 1.3.
 *    - "Color Filter Array: Monochrome" -- section 1.3, for the HM01B0-ANA
 *      part. The colour part is a different order code, and no datasheet for
 *      one exists anywhere on this machine. That is not proof the fitted part
 *      is mono, but it is the only Himax document here and it describes a mono
 *      module.
 *
 *  FROM THE HM01B0 DATASHEET, NOT INDEPENDENTLY CONFIRMED AGAINST A COPY ON
 *  THIS MACHINE. Everything below that the module datasheet does not name --
 *  MODEL_ID, MODE_SELECT, SW_RESET, GRP_PARAM_HOLD, X/Y_ODD_INC, BINNING_MODE,
 *  TEST_PATTERN_MODE. These are the standard Himax addresses also used by the
 *  public SparkFun Edge / TensorFlow Lite Micro himax drivers. A search of this
 *  machine on 2026-09-21 found NO canonical Himax init script: not in Zephyr
 *  upstream (which ships no HM01B0 driver -- drivers/video has gc2145, imx335,
 *  mt9m114, ov2640/5640/7670/7725/9655 and nothing Himax), not in the
 *  zephyr-rose module (its himax,hm01b0 node is served by hm01b0_stub.c, which
 *  returns -ENOSYS and touches no register), not in Baremetal-IDE, and not in
 *  the other checkouts. Confirm against the real sensor datasheet before
 *  trusting a failure that implicates one of them.
 */

#ifndef HM01B0_H_
#define HM01B0_H_

/* --- verified in this repository --- */
#define HM01B0_I2C_ADDR          0x24U

/* --- datasheet values, see the provenance note above --- */
#define HM01B0_REG_MODEL_ID_H    0x0000U /* expect 0x01 */
#define HM01B0_REG_MODEL_ID_L    0x0001U /* expect 0xB0 */
#define HM01B0_MODEL_ID          0x01B0U

#define HM01B0_REG_MODE_SELECT   0x0100U /* 0 = standby, 1 = streaming */
#define HM01B0_MODE_STANDBY      0x00U
#define HM01B0_MODE_STREAMING    0x01U

#define HM01B0_REG_SW_RESET      0x0103U /* write any value to reset */
#define HM01B0_SW_RESET_VALUE    0x00U

#define HM01B0_REG_GRP_PARAM_HOLD 0x0104U /* 1 = hold, 0 = commit */
#define HM01B0_GRP_HOLD          0x01U
#define HM01B0_GRP_COMMIT        0x00U

/*
 * Readout-mode registers. Read-only unless RB_CAMERA_FULL_READOUT is defined
 * (see integration/zephyr/camera/src/camera.c), so by default whatever the
 * sensor powers up with is what it runs, and these exist so a capture can
 * REPORT its readout mode rather than assume it.
 *
 * Why they were added: the HM01B0 ships in a monochrome variant and a Bayer-CFA
 * colour variant with the SAME model id (0x01B0), so software cannot tell them
 * apart, and a colour part that is subsampling (X/Y_ODD_INC = 3) or binning
 * (BINNING_MODE != 0) averages or skips adjacent CFA sites and returns a frame
 * indistinguishable from mono data.
 *
 * MEASURED 2026-09-21, AND IT LARGELY SETTLES THIS: the frame's active width is
 * 324 -- every line is 326 LVLD beats of which the first two are 0x00 dummies,
 * in all of four captures, 315 of 315 lines each. 324 is the FULL active width
 * from the datasheet. Both mosaic-destroying modes halve it:
 *
 *    2x2 binning     324 -> 162      X_ODD_INC = 3 subsample     324 -> 162
 *
 * The width is not halved, so neither is on in the horizontal direction. Only a
 * Y-only subsample could still be hiding, and that is what Y_ODD_INC is read
 * for. The separate question of why the frames carry no mosaic is answered in
 * hardware/ospi/frame-to-png.py: one 2x2 sampling phase reads 12-18% low in raw
 * DN. The raw deficit grows with signal level, but that is only the black
 * pedestal showing through -- ABOVE black the transmission is flat at ~0.55-0.59
 * on every frame, including the pre-swap control. It is NOT a capture artifact:
 * the test pattern below arrives bit-exact on all four phases, so the
 * attenuation is upstream of the digital bus.
 *
 * Encodings are datasheet values, not established on this machine; treat a
 * surprising value as "confirm the datasheet", the same caveat the walking-1s
 * note below carries.
 */
/* Unverified addresses and encodings -- see the provenance note above. */
#define HM01B0_REG_X_ODD_INC     0x0383U /* 0x01 = every column, 0x03 = skip */
#define HM01B0_REG_Y_ODD_INC     0x0387U /* 0x01 = every row,    0x03 = skip */
#define HM01B0_REG_BINNING_MODE  0x0390U /* 0x00 = off; non-zero averages */
#define HM01B0_X_ODD_INC_EVERY   0x01U
#define HM01B0_Y_ODD_INC_EVERY   0x01U
#define HM01B0_BINNING_OFF       0x00U

/*
 * QVGA_WIN_EN is the one readout register the datasheet on this machine names,
 * and it names it exactly: 0x3010 BIT 0, 1 = the 324x244 window, 0 = the full
 * 324x324 array. Masked to bit 0 because the other seven bits are undocumented
 * here and reporting them as part of the value invites reading a reserved bit
 * as a mode change.
 */
#define HM01B0_REG_QVGA_WIN_EN   0x3010U
#define HM01B0_QVGA_WIN_EN_MASK  0x01U
#define HM01B0_QVGA_WIN_OFF      0x00U   /* full 324x324 */
#define HM01B0_QVGA_WIN_ON       0x01U   /* 324x244 crop at (0,0) */

/*
 * THE EXPOSURE CHAIN. Nothing in this repository has ever written ANY of
 * these, and nothing has ever read them back. The driver programs exactly two
 * registers -- TEST_PATTERN and MODE_SELECT -- so the sensor has run on its
 * power-up defaults since the camera was first brought up.
 *
 * That is the most likely explanation for the single most striking property of
 * every frame ever captured here: the whole image spans 5-49 DN above a ~36 DN
 * black floor. A correctly exposed 8-bit frame spans roughly 90-150 DN. At 20
 * DN of signal, per-channel colour differences are down in the quantisation
 * noise whether or not a colour filter array exists -- so "is it colour?"
 * cannot be answered from an unexposed sensor, and every colour statistic
 * computed on these frames has been measured on almost no signal.
 *
 * Addresses are the ones every published HM01B0 driver uses (Himax's own
 * reference init, and the Arduino/OpenMV/TFLite-micro ports that descend from
 * it). They are NOT verified against a register map in this repository -- the
 * only Himax document here, HM01B0-ANA-00FT870, is a 20-page module datasheet
 * with no register map and names only 0x3010. Treat a NAK or an unchanged
 * read-back as "wrong address", not as a broken sensor, exactly as the
 * provenance note above requires.
 */
#define HM01B0_REG_INTEGRATION_H 0x0202U /* coarse integration time, high byte */
#define HM01B0_REG_INTEGRATION_L 0x0203U /* coarse integration time, low byte  */
#define HM01B0_REG_ANALOG_GAIN   0x0205U /* analog gain, 0x00..0x30 in 3 dB steps */
#define HM01B0_ANALOG_GAIN_MAX   0x30U   /* AE pegs here when there is no light */
#define HM01B0_REG_DIGITAL_GAIN_H 0x020EU /* digital gain integer part */
#define HM01B0_REG_DIGITAL_GAIN_L 0x020FU /* digital gain fraction, /128 */
#define HM01B0_REG_FRAME_LEN_H   0x0340U /* frame length in lines, high */
#define HM01B0_REG_FRAME_LEN_L   0x0341U /* frame length in lines, low  */
#define HM01B0_REG_LINE_LEN_H    0x0342U /* line length in pixel clocks, high */
#define HM01B0_REG_LINE_LEN_L    0x0343U /* line length in pixel clocks, low  */
#define HM01B0_REG_AE_CTRL       0x2100U /* bit0: 1 = auto exposure enabled */
#define HM01B0_REG_AE_TARGET     0x2101U /* AE target mean, in DN */
#define HM01B0_REG_AE_MIN_MEAN   0x2102U /* AE minimum acceptable mean */
/*
 * AE CLAMPS. These bound what the AE loop is ALLOWED to reach, so they decide
 * whether "analog gain pegged at max" means the scene is dark or merely that
 * the loop ran out of rope. Read them before concluding anything about light:
 * integration stalling well below FRAME_LEN is MAX_INTG doing its job, not the
 * sensor refusing to expose.
 *
 * Addresses are from the published Himax reference register map, not from a
 * datasheet in this repository -- so they are reported, never acted on, per the
 * rule the exposure dump already follows.
 */
#define HM01B0_REG_MAX_INTG_H    0x2105U /* AE integration ceiling, high byte */
#define HM01B0_REG_MAX_INTG_L    0x2106U /* AE integration ceiling, low byte  */
#define HM01B0_REG_MAX_AGAIN     0x2107U /* AE analog-gain ceiling, full res  */
#define HM01B0_REG_MAX_DGAIN     0x210AU /* AE digital-gain ceiling           */
#define HM01B0_REG_BLC_CFG       0x1000U /* black level correction enable */
#define HM01B0_REG_BLC_TARGET    0x1003U /* black level target, in DN */

#define HM01B0_REG_TEST_PATTERN  0x0601U
#define HM01B0_TESTPAT_OFF       0x00U
#define HM01B0_TESTPAT_MODE1     0x01U
/* Old name. The pattern is not walking 1s -- see below -- but the register
 * value is unchanged, so the spelling is kept working rather than churned. */
#define HM01B0_TESTPAT_WALKING1  HM01B0_TESTPAT_MODE1

/*
 * MEASURED 2026-09-22, and it is NOT walking 1s. The expectation recorded here
 * previously -- 0x01, 0x02, 0x04 ... 0x80 repeating -- was flagged UNVERIFIED
 * and is now disproven by a capture off this part with TEST_PATTERN_MODE=1
 * confirmed on the compile line.
 *
 * What 0x0601 = 0x01 actually emits is a ONE-PIXEL-PERIOD COLUMN STRIPE of
 * 0xff laid on the black level:
 *
 *   - three distinct byte values in the entire 311x324 frame: the black level
 *     (0x24 here), the next frame's black level where the buffer wrapped
 *     (0x2e), and 0xff;
 *   - four distinct row patterns, one for even rows and one for odd, repeated;
 *   - even rows carry 108 stripes, odd rows 130, with two phase slips per odd
 *     line and a 45-pixel run of solid 0xff at the end of it.
 *
 * TWO CONSEQUENCES, both of which have already cost this investigation a round:
 *
 *  1. A power-of-two / walking-1s check on this data is meaningless. Nothing
 *     here is a power of two and nothing was ever going to be.
 *  2. PER-2x2-PHASE MEANS ON THIS PATTERN MEASURE THE PATTERN, NOT THE
 *     HARDWARE. The stripes sit exactly at the Nyquist frequency of the 2x2
 *     phase grid, so the phase means just count where the 0xff landed: even
 *     rows split 54/54 between column parities and so give two phase means
 *     equal to three decimals by construction, while odd rows split 11020/7830
 *     and so give one phase mean 27% high. That 27% is the picture, not a
 *     defect. Do not read a mosaic, a CFA or a capture fault out of it.
 *
 * What the pattern DOES prove, and it is the reason to keep capturing it: all
 * three values arrive at their exact codes on all four phases, and every one of
 * the 28092 full-scale 0x24 -> 0xff adjacent-column steps lands on 0xff exactly.
 * An affine per-phase corruption would have put a fourth value in the
 * histogram. The digital capture path is clean, including at the worst-case
 * column transition rate. hardware/ospi/colour-decide.py --pattern reports
 * exactly these two things and deliberately refuses to report phase means.
 */
#define HM01B0_WALKING1_FIRST    0x01U

/*
 * Data-width mode is deliberately NOT programmed. The sensor's reset default
 * is the 8-bit parallel output this capture core requires, so bring-up relies
 * on the reset state rather than writing a mode register whose encoding is not
 * established here. If the sensor were to come up in 1-bit or 4-bit mode the
 * pixel values would be wrong while the sync counters still looked healthy --
 * which is exactly what the per-pixel dump below is for.
 */

#endif /* HM01B0_H_ */
