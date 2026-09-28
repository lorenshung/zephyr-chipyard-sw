/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * RiskyBird motor link: the FPGA -> ESP32-C6 motor duty command frame.
 *
 * WHY THIS EXISTS
 * ---------------
 * FPGA ball F13 drives motor4's gate and is dead as an OUTPUT. Measured, twice,
 * with no Rocket and no Chipyard in the picture:
 *
 *   - a bare-metal Verilog bitstream driving the four pads at 4/8/12/16 % reads
 *     back at the gates as M1=40 M2=80 M3=120 M4=0 (per mille). Three exact,
 *     F13 nothing.
 *   - a second bitstream with F13 as an INPUT with an internal pullup reads a
 *     solid, stable 0 -- so the carrier's 10k gate pulldown IS reachable from
 *     the ball. The net is intact; the output driver (or the whole IOB) is not.
 *
 * There is no RTL fix for a dead output driver. The ESP32-C6 reaches every gate
 * through its own 47R and can still spin motor4 on GPIO22, so the motors move
 * to the ESP and the FPGA stops driving gates entirely. Not just motor4: ALL
 * FOUR. Every gate is a wired-OR of an FPGA 47R and an ESP 47R into a 10k
 * pulldown, so leaving three motors dual-driven would keep the contention that
 * parks a contested gate near half rail -- exactly the condition nobody wants
 * on a MOSFET gate. One owner per gate is the whole point.
 *
 * THE WIRE
 * --------
 * The existing second UART, already routed and already described by
 * hardware/zephyr/fpga-esp-uart.overlay:
 *
 *   uart1_rxd  E13  J1_96   FPGA input   <- ESP TX   (ESP GPIO16 / U0TXD)
 *   uart1_txd  F14  J1_98   FPGA output  -> ESP RX   (ESP GPIO17 / U0RXD)
 *
 * Bank 16 on the FPGA side, which is the bank this bench has proven live, and
 * unrelated to the failed F13.
 *
 * FRAME (14 bytes, fixed length, little-endian on the wire)
 * --------------------------------------------------------
 *   off  len  field
 *    0    2   magic  0xA5 0x5A
 *    2    1   seq    frame counter, wraps at 256
 *    3    1   flags  see MOTOR_LINK_FLAG_*
 *    4    8   duty   4 x uint16, 0 = off .. 10000 = 100.00 %
 *   12    2   crc    CRC-16/CCITT-FALSE over bytes 0..11
 *
 * Field widths, and why each one is what it is:
 *
 *   magic 2 bytes. One sync byte gives a 1-in-256 chance that a random byte
 *         opens a candidate frame; two gives 1-in-65536. The magic is not the
 *         validator -- the CRC is -- it is what makes resynchronisation cheap,
 *         so the CRC is evaluated only on plausible alignments.
 *
 *   seq   1 byte. The receiver needs to see gaps and reject stale/replayed
 *         frames, not to number them forever. At the 50 Hz this link runs a
 *         uint8 wraps every 5.1 s, which is ~40x the failsafe timeout -- so a
 *         wrap can never be mistaken for a stale frame that the timeout has
 *         not already caught.
 *
 *   flags 1 byte. ARMED is the important one: "motors off" is then a BIT the
 *         sender sets, not something the receiver has to infer from four duty
 *         fields all happening to be zero. One bit, covered by the CRC.
 *
 *   duty  uint16 each, in units of 1/10000 (0.01 % per step). 8 bits would give
 *         0.4 % steps, which is ~8x coarser than either actuator: the FPGA's
 *         sifive comparator resolves 1/2500 = 0.040 % at 20 kHz, and the ESP's
 *         LEDC lands on an 11-bit timer at 20 kHz from the 80 MHz PLL_DIV clock
 *         -- 2048 steps, 0.049 % (read out of pwm_led_esp32.c's resolution
 *         calculation, not assumed). A wire quantiser coarser than both
 *         actuators is a self-inflicted limit; at 0.01 % the wire is finer than
 *         both and disappears from the error budget. The extra 4 bytes per frame
 *         cost 200 B/s at 50 Hz -- 1.7 % of a 115200 link.
 *
 *   crc   CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final
 *         XOR). Catches all 1- and 2-bit errors, all odd-bit errors, and every
 *         burst up to 16 bits. The realistic corruption on a UART is a framing
 *         slip that garbles a RUN of bytes, which is precisely the burst case;
 *         a sum-8 would miss byte transpositions and a CRC-32 would cost two
 *         more bytes to protect twelve.
 *
 * LATENCY BUDGET
 * --------------
 * 14 bytes at 115200 8N1 = 140 bit times (start + 8 + stop per byte):
 *
 *   140 / 115200 = 1.215 ms on the wire.
 *
 * The control loop iterates in roughly 25-35 ms. One frame is therefore 3.5-4.9 %
 * of one iteration, and the link is driven at a fixed 50 Hz rather than once per
 * iteration so a faster loop cannot flood it: 14 B x 50 Hz = 700 B/s against the
 * 11520 B/s the line carries, i.e. 6.1 % of the link. Even with the existing
 * ~240-byte ASCII telemetry line sharing the same wire at 25 Hz (6000 B/s, 52 %)
 * the total is ~58 %, so 115200 does not have to change for this.
 *
 * Raising the baud is possible and is NOT needed. If it is ever done it must be
 * done on BOTH sides in the same commit -- `current-speed` in
 * hardware/zephyr/fpga-esp-uart.overlay and in the ESP workload's overlay -- and
 * confirmed with workloads/esp_uart, which sweeps candidate rates and reports
 * per-rate byte counts. Note the two overlays currently disagree about whether
 * 115200 was ever measured: esp_bridge.overlay says esp_uart proved it,
 * fpga-esp-uart.overlay says it is a starting point and not a measured value.
 * Until that is settled on the bench, treat 115200 as unverified.
 *
 * RESYNCHRONISATION
 * -----------------
 * The receiver keeps a 14-byte sliding window (motor_link_rx_feed below) rather
 * than a start-state machine. Every arriving byte shifts the window by one and
 * the window is tested for magic-then-CRC. Because the window always holds the
 * most recent 14 bytes, the instant the last byte of any valid frame arrives the
 * window IS that frame -- so recovery from a partial frame, a lost byte or pure
 * garbage is guaranteed within one frame, with no timeout and no special state.
 * A start-state machine that discards a whole frame on a CRC failure can swallow
 * the start of the NEXT frame; this cannot.
 *
 * The window also ignores ASCII cleanly (0xA5 0x5A is not text), which is what
 * lets binary duty frames and the existing line-oriented telemetry share one
 * wire if they are ever merged.
 *
 * This header is the single copy of the contract. It is deliberately
 * freestanding -- no Zephyr, no libc beyond stdint/stdbool -- so both ends
 * compile the same encoder, the same decoder and the same CRC.
 */

#ifndef RISKYBIRD_MOTOR_LINK_H_
#define RISKYBIRD_MOTOR_LINK_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_LINK_NMOTORS   4
#define MOTOR_LINK_MAGIC0    0xA5u
#define MOTOR_LINK_MAGIC1    0x5Au
#define MOTOR_LINK_FRAME_LEN 14u
/* Bytes 0..11 are covered by the CRC; 12..13 carry it. */
#define MOTOR_LINK_CRC_LEN   12u

/* Duty is carried as 1/10000 of full scale: 0 = off, 10000 = 100.00 %. */
#define MOTOR_LINK_DUTY_FULL 10000u

/*
 * ARMED is the receiver's permission to drive at all. A frame without it is a
 * valid, in-sequence, CRC-good frame that commands every motor OFF -- which is
 * what the FPGA sends while disarmed, estopped, past its actuation timeout, or
 * past the commitment window that bounds how long motors may run at all.
 * That matters: "off" then arrives as data on a live link, and is distinguishable
 * from the link having died (which the receiver's own timeout handles).
 */
#define MOTOR_LINK_FLAG_ARMED   0x01u
#define MOTOR_LINK_FLAG_ESTOP   0x02u   /* sender latched its emergency stop */
#define MOTOR_LINK_FLAG_TIMEOUT 0x04u   /* sender's bench actuation timeout expired */
#define MOTOR_LINK_FLAG_INHIBIT 0x08u   /* sender built with motors hard-inhibited */
#define MOTOR_LINK_FLAG_WINDOW  0x10u   /* sender's COMMITMENT WINDOW expired -- latched off */
#define MOTOR_LINK_FLAG_COMMIT  0x20u   /* sender is COMMITTED to a flight test (informational) */

/*
 * The reasons a receiver must drive nothing, as one mask, in the shared header.
 *
 * It lives here rather than in the receiver because it is part of the contract:
 * a sender that gains a new "off" reason and a receiver that has never heard of
 * it is exactly the divergence that leaves a motor spinning on a frame the
 * sender believed said stop. Adding a blocking flag above and forgetting it here
 * is the one mistake this mask makes impossible.
 *
 * MOTOR_LINK_FLAG_COMMIT is deliberately NOT in it. It reports the sender's
 * state for the operator's benefit and must never be able to permit -- or
 * forbid -- drive on its own; ARMED remains the only permission.
 */
#define MOTOR_LINK_FLAGS_BLOCK (MOTOR_LINK_FLAG_ESTOP | MOTOR_LINK_FLAG_TIMEOUT | \
				MOTOR_LINK_FLAG_INHIBIT | MOTOR_LINK_FLAG_WINDOW)

struct motor_link_cmd {
	uint8_t  seq;
	uint8_t  flags;
	uint16_t duty[MOTOR_LINK_NMOTORS];
};

/* CRC-16/CCITT-FALSE. Bitwise rather than table-driven: 8 iterations per byte,
 * 112 per frame, 5600/s at 50 Hz. A 512-byte table would buy nothing measurable
 * on either end and would be a second thing to keep identical across the link. */
static inline uint16_t motor_link_crc16(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFFu;

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)((uint16_t)data[i] << 8);
		for (int bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000u) ? (uint16_t)((uint16_t)(crc << 1) ^ 0x1021u)
					      : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

/* Clamp a normalised [0,1] duty onto the wire's 0..10000. Saturating rather
 * than wrapping: a duty that arrives above 1.0 from a controller that wound up
 * must become full scale's ceiling, never a small number via truncation. */
static inline uint16_t motor_link_duty_from_float(float duty)
{
	if (!(duty > 0.0f)) {       /* also catches NaN: every comparison is false */
		return 0u;
	}
	if (duty >= 1.0f) {
		return (uint16_t)MOTOR_LINK_DUTY_FULL;
	}
	return (uint16_t)(duty * (float)MOTOR_LINK_DUTY_FULL + 0.5f);
}

static inline void motor_link_encode(uint8_t out[MOTOR_LINK_FRAME_LEN],
				     uint8_t seq, uint8_t flags,
				     const uint16_t duty[MOTOR_LINK_NMOTORS])
{
	uint16_t crc;

	out[0] = (uint8_t)MOTOR_LINK_MAGIC0;
	out[1] = (uint8_t)MOTOR_LINK_MAGIC1;
	out[2] = seq;
	out[3] = flags;
	for (unsigned i = 0; i < MOTOR_LINK_NMOTORS; i++) {
		uint16_t d = duty[i];

		if (d > MOTOR_LINK_DUTY_FULL) {
			d = (uint16_t)MOTOR_LINK_DUTY_FULL;
		}
		out[4u + 2u * i] = (uint8_t)(d & 0xFFu);
		out[5u + 2u * i] = (uint8_t)(d >> 8);
	}
	crc = motor_link_crc16(out, MOTOR_LINK_CRC_LEN);
	out[12] = (uint8_t)(crc & 0xFFu);
	out[13] = (uint8_t)(crc >> 8);
}

/*
 * Receiver state. Zero-initialise it; there is no init call to forget.
 *
 * `noise` counts bytes that fell out of the window without ever belonging to an
 * accepted frame. On a healthy link it stays at 0 after the first frame, so a
 * rising noise count is the signature of a wrong baud rate or of something else
 * sharing the wire -- which is a different fault from `bad_crc` rising, and the
 * two need different fixes.
 */
struct motor_link_rx {
	uint8_t  win[MOTOR_LINK_FRAME_LEN];
	uint8_t  len;
	uint32_t bad_crc;
	uint32_t noise;
};

/*
 * Feed one received byte. Returns true exactly when *cmd has been filled with a
 * frame whose magic and CRC both check out.
 *
 * O(1) per byte: two compares, and a 12-byte CRC only on a window that already
 * starts with the magic.
 */
static inline bool motor_link_rx_feed(struct motor_link_rx *rx, uint8_t byte,
				      struct motor_link_cmd *cmd)
{
	uint16_t got;

	if (rx->len < MOTOR_LINK_FRAME_LEN) {
		rx->win[rx->len++] = byte;
	} else {
		/* The byte leaving the window was never part of an accepted
		 * frame, by construction: an accepted frame resets len to 0. */
		rx->noise++;
		for (unsigned i = 1; i < MOTOR_LINK_FRAME_LEN; i++) {
			rx->win[i - 1] = rx->win[i];
		}
		rx->win[MOTOR_LINK_FRAME_LEN - 1u] = byte;
	}
	if (rx->len < MOTOR_LINK_FRAME_LEN) {
		return false;
	}
	if (rx->win[0] != (uint8_t)MOTOR_LINK_MAGIC0 ||
	    rx->win[1] != (uint8_t)MOTOR_LINK_MAGIC1) {
		return false;
	}
	got = (uint16_t)rx->win[12] | (uint16_t)((uint16_t)rx->win[13] << 8);
	if (got != motor_link_crc16(rx->win, MOTOR_LINK_CRC_LEN)) {
		rx->bad_crc++;
		return false;
	}
	cmd->seq = rx->win[2];
	cmd->flags = rx->win[3];
	for (unsigned i = 0; i < MOTOR_LINK_NMOTORS; i++) {
		cmd->duty[i] = (uint16_t)rx->win[4u + 2u * i] |
			       (uint16_t)((uint16_t)rx->win[5u + 2u * i] << 8);
	}
	rx->len = 0u;   /* consumed: the next frame starts a fresh window */
	return true;
}

/*
 * Sequence relation, wrap-aware. > 0 means `seq` is newer than `prev`.
 *
 * A UART cannot reorder, so a frame that is NOT newer than the last accepted one
 * is either a replay or the tail of something the window resynchronised across.
 * Applying it would mean an older command overwriting a newer one, so the
 * receiver drops it -- see the ESP side's handling.
 */
static inline int motor_link_seq_delta(uint8_t seq, uint8_t prev)
{
	return (int)(int8_t)((uint8_t)(seq - prev));
}

#ifdef __cplusplus
}
#endif

#endif /* RISKYBIRD_MOTOR_LINK_H_ */
