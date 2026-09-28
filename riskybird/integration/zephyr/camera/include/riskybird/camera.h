/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * One HM01B0 frame, captured into a caller-supplied buffer.
 *
 * The sequence -- MCLK before SCCB, streaming before PCLK is meaningful, arm on
 * a frame boundary, stop the sensor the moment PIXTARGET is satisfied -- is
 * board knowledge, and every step of it is a way a capture can look broken
 * while the hardware is fine. It lives here so a workload that wants a picture
 * does not have to re-derive it.
 *
 * This reports rather than prints: the caller owns its own output format, and
 * bootup_check's one-line-per-stage table and camera_photo's nine stages of
 * evidence want different things from the same capture.
 *
 * NOTE ON DUPLICATION: workloads/camera_photo predates this module and still
 * carries its own copy of the sequence. It is the reference implementation the
 * camera docs quote verbatim, so it was left alone rather than refactored
 * mid-bench-session; migrating it here is the outstanding follow-up, and until
 * that happens a fix to the sequence belongs in both places.
 */

#ifndef RISKYBIRD_CAMERA_H_
#define RISKYBIRD_CAMERA_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* What the capture observed. Populated as far as the sequence got, so a failed
 * capture still says how far it got and what the core saw. */
struct rb_camera_capture {
	uint16_t model_id;
	uint32_t pclk_before;
	uint32_t pclk_after;
	uint32_t captured;        /* pixels the core counted */
	uint32_t drained;         /* pixels actually read into the buffer */
	uint32_t measured_width;  /* LASTWIDTH -- reshape rows at THIS, not GEOM */
	uint32_t measured_height;
	uint32_t configured_width;
	uint32_t configured_height;
	uint32_t flags;
	uint8_t vmin;
	uint8_t vmax;
	/* Readout mode as the sensor actually reports it, not as configured.
	 * readout_ok is 0 when every read succeeded; a non-zero value means the
	 * four bytes below are meaningless. See the note in hm01b0.h: a colour
	 * part that is binning or subsampling produces a frame with no mosaic,
	 * which looks exactly like a mono part. */
	uint8_t x_odd_inc;
	uint8_t y_odd_inc;
	uint8_t binning_mode;
	uint8_t qvga_win_en;      /* bit 0 only; see hm01b0.h */
	int readout_ok;
	/*
	 * What RB_CAMERA_FULL_READOUT did, if it was built in.
	 *   RB_CAMERA_PROGRAM_OFF      not compiled in -- the default
	 *   RB_CAMERA_PROGRAM_OK       the sequence was written and read back clean
	 *   anything else              the negative errno of the write that failed
	 * Separate from readout_ok because "the sensor would not accept the
	 * configuration" and "the sensor would not tell me its configuration" are
	 * different faults and the second is survivable.
	 */
	int program_ok;
	/*
	 * What the pixel-clock check saw, so a "pclk" failure says which of
	 * "the sensor never left standby" (mode_select != 0x01) and "it streams
	 * but nothing reaches the core" (mode_select == 0x01, counts flat) it is.
	 * mode_select is MODE_SELECT read back after the streaming write, 0xff if
	 * the read failed or was not reached; the counts are the core's FVLDCNT /
	 * LVLDCNT at the last PCLK check.
	 */
	uint8_t mode_select;
	uint32_t fvld_count;
	uint32_t lvld_count;
};

#define RB_CAMERA_PROGRAM_OFF      1    /* not built in */
#define RB_CAMERA_PROGRAM_OK       0
#define RB_CAMERA_READOUT_NOT_READ (-1000)  /* capture stopped before the probe */

/*
 * Pixels the capture core can hold in one capture -- the hard ceiling on a
 * frame, and lower than a full frame on at least one shipped shell. A caller
 * that wants "as much of the picture as fits" clamps to whole rows against
 * this rather than hardcoding a height, so a core with a bigger buffer needs
 * no code change.
 */
uint32_t rb_camera_capacity(void);

/*
 * Capture `want` pixels into `dst`. Returns 0 on success, or -1 with *stage and
 * *detail set to static strings naming where it stopped and why.
 *
 * Needs a DDR-backed shell: a full frame is ~79 KB and a scratchpad holds 32 KB.
 */
int rb_camera_capture_frame(uint8_t *dst, uint32_t want,
			    struct rb_camera_capture *out,
			    const char **stage, const char **detail);

/*
 * The same sequence, split at its seams, for a caller that cannot sit in
 * rb_camera_capture_frame() for ~6.3 s -- the flight controller taking one
 * picture mid-hover. rb_camera_capture_frame() is exactly
 * prepare -> arm -> poll done -> finish, so the two paths cannot drift.
 *
 *   rb_camera_prepare  MCLK, SCCB setup, streaming, AE settle (blocks ~6.3 s,
 *                      uses I2C). Leaves the sensor streaming and the core
 *                      enabled but NOT armed: it keeps draining, so nothing
 *                      overflows while the caller waits. Call before the
 *                      control loop starts.
 *   rb_camera_arm      four register writes; no I2C, no printk, no sleep.
 *                      Safe inside a control loop.
 *   rb_camera_done     non-blocking; true once `want` pixels are held.
 *   rb_camera_frames   frames seen so far (FVLDCNT), to arm just after one
 *                      increments. Optional.
 *   rb_camera_finish   sensor standby over I2C, then drain `want` pixels into
 *                      dst (~79k register reads). NOT for a control loop: call
 *                      it after landing.
 * Once armed and complete, the core holds the frame and discards everything
 * after it, so the sensor may keep streaming until finish(); the OVERFLOW flag
 * that sets is surplus data, not lost data.
 */
int rb_camera_prepare(uint32_t want, struct rb_camera_capture *out,
		      const char **stage, const char **detail);
void rb_camera_arm(uint32_t want);
bool rb_camera_done(uint32_t want);
uint32_t rb_camera_frames(void);
int rb_camera_finish(uint8_t *dst, uint32_t want, struct rb_camera_capture *out,
		     const char **stage, const char **detail);

/*
 * Recovery, for a caller that retries prepare (the flight controller). Neither
 * is called by rb_camera_capture_frame(), so the proven single-shot path is
 * unchanged.
 *
 *   rb_camera_sensor_reset  MCLK on, MODE_SELECT=standby, SW_RESET, then waits
 *                           for the model id to answer again. Puts the sensor
 *                           back in its power-up state whatever a previous
 *                           image left it in: nothing on this board power-cycles
 *                           it between loads, and a failed prepare leaves it
 *                           streaming. 0, or negative if it did not come back.
 *   rb_camera_standby       MODE_SELECT=standby, best effort. For a prepare
 *                           that is giving up, so the next image starts clean.
 */
int rb_camera_sensor_reset(void);
void rb_camera_standby(void);

#ifdef __cplusplus
}
#endif

#endif /* RISKYBIRD_CAMERA_H_ */
