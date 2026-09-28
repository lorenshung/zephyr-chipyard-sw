/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * RiskyBird motor offload: the ESP32-C6 drives all four motor gates from duty
 * frames sent by the FPGA flight controller over the existing UART link.
 *
 * WHY THE ESP OWNS THE MOTORS
 * ---------------------------
 * FPGA ball F13 drives motor4's gate and is dead as an output. Proven with a
 * bare-metal bitstream (no Rocket, no Chipyard, no PWM IP): four pads driven at
 * 4/8/12/16 % read back at the gates as M1=40 M2=80 M3=120 M4=0 per mille --
 * three exact, F13 nothing -- and a second bitstream with F13 as an INPUT with
 * an internal pullup read a solid, stable 0, i.e. the carrier's 10k gate
 * pulldown is reachable from the ball. The NET is intact; the output driver is
 * not. There is no RTL repair for that.
 *
 * The ESP reaches every gate through its own 47R, so it takes ALL FOUR, not
 * just motor4. Each gate is a wired-OR of an FPGA 47R and an ESP 47R into a 10k
 * pulldown; leaving three motors dual-driven would keep the contention that
 * parks a contested gate near half rail. One owner per gate is simpler to
 * reason about and simpler to make safe.
 *
 * THIS IMAGE REPLACES THE GATE PROBE. workloads/motor_safe parks these same
 * four pins as pulled-down inputs and reports what the FPGA delivers to each
 * gate; that is the instrument every measurement above came from. It cannot run
 * at the same time as this. Reflash motor_safe to get it back. To keep some
 * observability here, this image prints a deliberately parallel line --
 * "MOTOR CMD duty per mille" against motor_safe's "MOTOR GATE duty per mille" --
 * so the same eye reads both.
 *
 * SAFETY MODEL
 * ------------
 * The failsafe is the point of this workload, not a feature of it.
 *
 *   boot          Motors are written to 0 before the UART is even enabled, and
 *                 the failsafe deadline starts already expired. Nothing can
 *                 drive a gate until a valid frame has arrived.
 *   silence       No valid frame for ESP_MOTORS_TIMEOUT_MS -> all four to 0.
 *   CRC failure   Frame discarded, duties unchanged, deadline NOT advanced. A
 *                 stream of corrupt frames therefore ends in the timeout, which
 *                 is right: corrupt is indistinguishable from absent.
 *   sequence gap  Frame IS applied and the deadline IS advanced. A gap means
 *                 bytes were lost, not that this command is wrong -- and this
 *                 command is the newest one. Cutting motors because the link
 *                 dropped one frame would make link jitter a flight hazard.
 *                 The gap is counted and printed.
 *   stale seq     A frame not newer than the last accepted one is dropped and
 *                 does NOT advance the deadline. A UART cannot reorder, so this
 *                 is a replay or the tail of a resynchronisation, and applying
 *                 it would let an older command overwrite a newer one.
 *   not ARMED     Every motor to 0, whatever the duty fields say.
 *
 * THIS IS A NET SAFETY IMPROVEMENT OVER TODAY, concretely. Today the FPGA's
 * SiFive PWM comparators are free-running hardware: halting the Rocket core --
 * which `rb debug` does routinely -- leaves them driving the gates at whatever
 * duty was last programmed, indefinitely. Nothing stops them but a reconfigure
 * or a power cycle. With the offload, halting the core stops the frame stream
 * and the ESP cuts all four within ESP_MOTORS_TIMEOUT_MS. The same holds for a
 * Rocket crash, a reset, or a pulled ribbon.
 *
 * THREAD PRIORITIES, AND WHY WIFI CANNOT STARVE THE MOTOR THREAD
 * --------------------------------------------------------------
 * The motor thread runs at K_PRIO_COOP(0) -- the most urgent cooperative
 * priority the application can name. Cooperative means no PREEMPTIBLE thread
 * can ever take the CPU from it, so the networking stack's threads, the system
 * workqueue at its default preemptible priority, and any future telemetry
 * thread are all structurally incapable of delaying it. It gives the CPU back
 * by blocking, every pass, on a semaphore with an ESP_MOTORS_TICK_MS timeout,
 * so it also cannot starve anything else: the failsafe is evaluated at least
 * every ESP_MOTORS_TICK_MS regardless of whether a single byte arrives.
 *
 * The UART receive path is an ISR, which preempts every thread including this
 * one, so byte loss cannot be caused by thread scheduling at all. The ISR only
 * moves bytes into a ring buffer and gives a semaphore.
 *
 * Checked in the tree rather than assumed, because this is the argument the
 * whole coexistence story rests on:
 *
 *   - The esp32 WiFi driver creates its tasks through a FreeRTOS shim
 *     (modules/hal/espressif/zephyr/esp32c6/src/wifi/esp_wifi_adapter.c) that
 *     passes the blob's priority straight into k_thread_create with no sign
 *     flip. ESP-IDF priorities are non-negative and capped by
 *     CONFIG_ESP32_WIFI_MAX_THREAD_PRIORITY (default 7), so every WiFi task
 *     lands in Zephyr's PREEMPTIVE range. K_PRIO_COOP is used only by the
 *     Bluetooth adapters, never by WiFi. esp_timer is priority 3, also
 *     preemptive. The system workqueue defaults to -1. All of those are
 *     outranked by K_PRIO_COOP(0) = -16.
 *   - The networking traffic-class threads. AN EARLIER VERSION OF THIS COMMENT
 *     GOT THIS WRONG IN BOTH DIRECTIONS AND ITS ADVICE WOULD HAVE MADE THINGS
 *     WORSE, so here is what the tree actually says.
 *
 *     Default is NET_TC_THREAD_COOPERATIVE, and net_tc.c takes
 *     BASE_PRIO_RX = CONFIG_NET_TC_NUM_PRIORITIES - 1, which Kconfig defaults
 *     to NUM_COOP_PRIORITIES when cooperative. So the RX thread is
 *     K_PRIO_COOP(NUM_COOP_PRIORITIES - 1) = **-1**, not -16: it is the LOWEST
 *     cooperative priority, not the highest, and this thread already outranks
 *     it. CONFIG_NET_TC_TX_COUNT defaults to 0, so there is no TX thread at all.
 *
 *     The old advice -- turn on CONFIG_NET_TC_THREAD_PRIO_CUSTOM -- would have
 *     CREATED the collision it was meant to prevent: NET_TC_RX_THREAD_BASE_PRIO
 *     defaults to 0, so switching PRIO_CUSTOM on without also setting the bases
 *     moves the RX thread from -1 to K_PRIO_COOP(0) = -16, level with this one.
 *
 *     It would also not have helped even if the priorities HAD collided,
 *     because relative priority among cooperative threads does not govern
 *     preemption at all: a cooperative thread is never preempted by another
 *     thread, whatever its priority (see should_preempt() in kthread.h). What
 *     matters is only that a coop thread holds the CPU until it blocks. The fix
 *     that actually works is to stop them being cooperative:
 *
 *         CONFIG_NET_TC_THREAD_PREEMPTIVE=y
 *         CONFIG_SYSTEM_WORKQUEUE_PRIORITY=1
 *
 *     None of this applies to THIS image, which has no networking. It matters
 *     only if the radio is ever merged into the motor receiver.
 *   - ISRs preempt everything, and the WiFi MAC does a lot of work in interrupt
 *     context. That is the realistic source of jitter here, not scheduling.
 *
 * The backstop that does not depend on getting any of that right is a hardware
 * watchdog: wdt0 is already `okay` on this board and CONFIG_WDT_ESP32 is
 * default y, so wdt_install_timeout(WDT_FLAG_RESET_SOC) fed from this thread
 * would reset the SoC if it is ever starved -- and a reset drops the LEDC
 * outputs, leaving the gates to the carrier's 10k pulldowns. Note when adding
 * it that wdt_esp32 programs BOTH MWDT stages with the same value, so the
 * interrupt fires at T and the reset at 2T. Deliberately NOT in this first
 * version: it is not needed until WiFi is, and an unproven self-reset on a
 * motor bench is its own hazard.
 *
 * Printing is deliberately NOT done from the motor thread: the console is a USB
 * CDC endpoint that can block when no host is draining it, and a cooperative
 * thread that blocks on the console is a cooperative thread that is not running
 * the failsafe. The status thread is preemptible and reads a snapshot.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

#include <riskybird/motor_link.h>

/* ---- configuration ------------------------------------------------------- */

/* 20 kHz, matching the FPGA build and the existing motor_1..motor_4 diagnostics.
 * Above audible, and well inside what the LEDC resolves. */
#define MOTOR_PERIOD_NSEC 50000U

/*
 * Failsafe timeout. Bounded from below by the worst case the link legitimately
 * produces and from above by how long a quadcopter may hold a stale command.
 *
 *   lower: the FPGA emits at 50 Hz, but its control loop is 25-35 ms under the
 *          default controller, so the real worst interval is ~35 ms. The FPGA
 *          console is polled (CONFIG_UART_INTERRUPT_DRIVEN is off on that
 *          carrier) and an undrained console has been measured stalling that
 *          loop 30-60 ms. 35 + 60 = 95 ms is the worst benign gap.
 *   upper: the airframe's attitude time constant is of order 150 ms, so holding
 *          a stale command for 120 ms is under one time constant -- the attitude
 *          has not yet diverged far. Much beyond that and the frame is no longer
 *          a control link.
 *
 * 120 ms it is: ~3.4 nominal intervals, above the worst benign gap, below one
 * attitude time constant. Overridable for bench work with
 * RB_EXTRA_CFLAGS=-DESP_MOTORS_TIMEOUT_MS=<n>.
 */
#ifndef ESP_MOTORS_TIMEOUT_MS
#define ESP_MOTORS_TIMEOUT_MS 120
#endif

/* How often the motor thread wakes when no byte arrives. Bounds the failsafe's
 * own granularity: worst-case cut latency is TIMEOUT + TICK. */
#ifndef ESP_MOTORS_TICK_MS
#define ESP_MOTORS_TICK_MS 5
#endif

/*
 * Independent duty ceiling, applied here, after everything the FPGA did.
 *
 * The FPGA already caps duty (MOTOR_DUTY_CEILING x MOTOR_MAX_DUTY; 10 % on a
 * default bench build). This is a SECOND ceiling in the last component before
 * the gate, so a flight-configured FPGA build cannot produce flight thrust on a
 * bench that was not expecting it. 25 % matches the ceiling workloads/motor_4
 * has always used.
 *
 * THE TWO CEILINGS ARE INDEPENDENT AND THE LOWER ONE WINS. Raising only the
 * FPGA's leaves every duty clamped to 25 % here, which on a flight attempt
 * looks exactly like a thrust failure or a tuning problem and is neither. A
 * flight test raises BOTH, in the same session:
 *
 *   FPGA  RB_CMAKE_ARGS="-DMOTOR_MAX_DUTY=1.0f"
 *         RB_EXTRA_CPPFLAGS="-DMOTOR_DUTY_CEILING=0.75f"
 *   ESP   RB_EXTRA_CFLAGS=-DESP_MOTORS_MAX_DUTY_PCT=80
 *
 * (CFLAGS here because this file is C; CPPFLAGS there because main.cpp is C++.)
 * Set this one ABOVE the FPGA's so it stays a backstop against a wrong FPGA
 * build rather than the thing that shapes normal flight -- see motors_apply()
 * for why a ceiling that bites every tick is not free.
 */
#ifndef ESP_MOTORS_MAX_DUTY_PCT
#define ESP_MOTORS_MAX_DUTY_PCT 25U
#endif
BUILD_ASSERT(ESP_MOTORS_MAX_DUTY_PCT > 0U && ESP_MOTORS_MAX_DUTY_PCT <= 100U,
	     "ESP_MOTORS_MAX_DUTY_PCT is a percentage of full duty: (0, 100]");

/* ~16 frames of backlog. The ISR must never block, so the only backpressure is
 * dropping, and a ring this size means dropping only ever follows a stall
 * longer than the failsafe timeout -- by which point the motors are already
 * off and the backlog no longer matters. */
#define RX_RING_SIZE 256

#define STATUS_PERIOD_MS 250   /* same cadence as motor_safe's gate probe */

/* ---- devices ------------------------------------------------------------- */

#define FPGA_UART_NODE DT_ALIAS(fpga_uart)
#if !DT_NODE_EXISTS(FPGA_UART_NODE)
#error "esp_motors needs an fpga-uart alias (see esp_motors.overlay)"
#endif

static const struct device *const fpga_uart = DEVICE_DT_GET(FPGA_UART_NODE);

static const struct pwm_dt_spec motors[MOTOR_LINK_NMOTORS] = {
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor0)),   /* MOTOR1 GPIO21 */
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor1)),   /* MOTOR2 GPIO20 */
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor2)),   /* MOTOR3 GPIO23 */
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor3)),   /* MOTOR4 GPIO22 */
};

/* ---- shared state -------------------------------------------------------- */

static uint8_t rx_ring_buf[RX_RING_SIZE];
static struct ring_buf rx_ring;
static K_SEM_DEFINE(rx_sem, 0, 1);

/* Written by the motor thread, read by the status thread. Plain volatile
 * scalars rather than a lock: the status line is diagnostics, a torn read costs
 * one cosmetically odd line, and taking a mutex in the motor thread would let a
 * printing thread delay the failsafe. */
static volatile uint16_t g_duty[MOTOR_LINK_NMOTORS];   /* per 10000, as applied */
static volatile uint32_t g_frames;        /* accepted frames */
static volatile uint32_t g_gaps;          /* frames missing, inferred from seq */
static volatile uint32_t g_stale;         /* frames not newer than the last */
static volatile uint32_t g_failsafe_cuts;
static volatile uint32_t g_pwm_errs;
static volatile uint8_t  g_flags;
static volatile uint8_t  g_seq;
static volatile bool     g_link_up;
static volatile int32_t  g_last_gap_ms;   /* measured interval between frames */

static struct motor_link_rx g_rx;         /* motor thread only */

/* ---- actuation ----------------------------------------------------------- */

/*
 * Apply one duty vector, in units of 1/10000. Everything that reaches a gate
 * goes through here, so the ceiling below cannot be bypassed by a code path.
 *
 * Thread context only, and only this thread. pwm_led_esp32_set_cycles() takes a
 * per-device semaphore with K_FOREVER, so it is not callable from an ISR or a
 * k_timer expiry -- which is why the failsafe is a deadline this thread checks
 * rather than a timer that cuts the motors itself. Keeping this the sole caller
 * also means that semaphore is never contended, so the cooperative motor thread
 * can never block on another thread's PWM write.
 */
static void motors_apply(const uint16_t duty[MOTOR_LINK_NMOTORS])
{
	const uint32_t ceiling = (MOTOR_LINK_DUTY_FULL * ESP_MOTORS_MAX_DUTY_PCT) / 100U;
	uint32_t d[MOTOR_LINK_NMOTORS];
	uint32_t peak = 0U;

	/*
	 * ATTITUDE-PRIORITY ANTI-SATURATION, not a per-motor clamp.
	 *
	 * This used to clamp each motor to the ceiling independently, which is the
	 * wrong shape for a quadcopter and is wrong in the direction that loses the
	 * airframe. The collective (altitude) command and the roll/pitch/yaw
	 * differentials share one duty range. A per-motor clamp flattens four
	 * different commands into four EQUAL numbers as soon as the largest crosses
	 * the ceiling -- and four equal duties are zero attitude torque. The vehicle
	 * would lose attitude authority at exactly the moment it is asking for the
	 * most thrust, which on a climb is the moment it can least afford to.
	 *
	 * Subtracting the excess from ALL FOUR instead lowers the collective and
	 * leaves every difference between motors intact: a little less altitude,
	 * never less attitude. It is the same rule the FPGA's actuator_duty()
	 * applies against its own ceiling, so the two ceilings now compose -- the
	 * lower one simply bites first and the vehicle still flies level.
	 *
	 * The per-motor clamp is kept BELOW as a final, unconditional line, because
	 * "nothing above the ceiling reaches a gate" must be true by inspection and
	 * not by trusting the arithmetic above it.
	 */
	for (unsigned i = 0; i < MOTOR_LINK_NMOTORS; i++) {
		/* A frame is only CRC-good, not sane: the wire carries a uint16 and a
		 * confused sender could put 60000 in it. Fold to full scale first so
		 * `peak` is a duty and not an arbitrary number. */
		d[i] = duty[i] > MOTOR_LINK_DUTY_FULL ? MOTOR_LINK_DUTY_FULL : duty[i];
		if (d[i] > peak) {
			peak = d[i];
		}
	}
	if (peak > ceiling) {
		const uint32_t cut = peak - ceiling;

		for (unsigned i = 0; i < MOTOR_LINK_NMOTORS; i++) {
			d[i] = (d[i] > cut) ? (d[i] - cut) : 0U;
		}
	}

	for (unsigned i = 0; i < MOTOR_LINK_NMOTORS; i++) {
		uint32_t pulse;
		int rc;

		if (d[i] > ceiling) {   /* unreachable given the cut; kept as the invariant */
			d[i] = ceiling;
		}
		/* Active-high into the SI2302: pulse 0 is genuinely off. */
		pulse = (uint32_t)((uint64_t)MOTOR_PERIOD_NSEC * d[i] / MOTOR_LINK_DUTY_FULL);
		rc = pwm_set_dt(&motors[i], MOTOR_PERIOD_NSEC, pulse);
		if (rc != 0) {
			g_pwm_errs++;
		}
		g_duty[i] = (uint16_t)d[i];
	}
}

static void motors_off(void)
{
	static const uint16_t zero[MOTOR_LINK_NMOTORS] = { 0, 0, 0, 0 };

	motors_apply(zero);
}

/* ---- UART receive -------------------------------------------------------- */

static void uart_isr(const struct device *dev, void *user_data)
{
	uint8_t buf[32];

	ARG_UNUSED(user_data);

	if (!uart_irq_update(dev)) {
		return;
	}
	while (uart_irq_rx_ready(dev)) {
		int n = uart_fifo_read(dev, buf, sizeof(buf));

		if (n <= 0) {
			break;
		}
		/* A short write means the ring is full. There is nothing useful
		 * to do about that in an ISR except keep draining the FIFO,
		 * which is what this loop is -- and a ring that overflows has
		 * already been silent long enough for the failsafe to fire. */
		(void)ring_buf_put(&rx_ring, buf, (uint32_t)n);
		k_sem_give(&rx_sem);
	}
}

/* ---- the motor thread ---------------------------------------------------- */

static void handle_frame(const struct motor_link_cmd *cmd, int64_t now)
{
	static uint8_t last_seq;
	static int64_t last_ms;

	if (g_link_up) {
		int delta = motor_link_seq_delta(cmd->seq, last_seq);

		if (delta <= 0) {
			g_stale++;
			return;   /* deliberately does NOT feed the failsafe */
		}
		if (delta > 1) {
			g_gaps += (uint32_t)(delta - 1);
		}
		g_last_gap_ms = (int32_t)(now - last_ms);
	}
	last_seq = cmd->seq;
	last_ms = now;
	g_seq = cmd->seq;
	g_flags = cmd->flags;
	g_frames++;
	g_link_up = true;

	/*
	 * ARMED is the sender's permission to drive. Anything else -- estop, the
	 * sender's own bench actuation timeout, its commitment window having
	 * expired, a build with motors inhibited -- is off, and is checked here as
	 * well as by the sender so that a single mistake on one side of the link
	 * cannot spin a motor.
	 *
	 * The blocking set is MOTOR_LINK_FLAGS_BLOCK, which lives in the shared
	 * header rather than here. A sender that gains a new reason to stop and a
	 * receiver that has never heard of it is the divergence that leaves a motor
	 * spinning on a frame the sender believed said stop; keeping the mask with
	 * the flag definitions is what makes that mistake impossible to make.
	 * MOTOR_LINK_FLAG_COMMIT is deliberately outside it: it reports the
	 * sender's state for the operator and must never permit or forbid drive.
	 */
	if ((cmd->flags & MOTOR_LINK_FLAG_ARMED) == 0u ||
	    (cmd->flags & MOTOR_LINK_FLAGS_BLOCK) != 0u) {
		motors_off();
		return;
	}
	motors_apply(cmd->duty);
}

static void motor_thread(void *a, void *b, void *c)
{
	/* Starts already expired: nothing drives a gate before the first frame. */
	int64_t deadline = 0;

	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	for (;;) {
		uint8_t buf[64];
		uint32_t n;
		int64_t now;

		while ((n = ring_buf_get(&rx_ring, buf, sizeof(buf))) > 0u) {
			for (uint32_t i = 0; i < n; i++) {
				struct motor_link_cmd cmd;

				if (motor_link_rx_feed(&g_rx, buf[i], &cmd)) {
					now = k_uptime_get();
					handle_frame(&cmd, now);
					if (g_link_up) {
						deadline = now + ESP_MOTORS_TIMEOUT_MS;
					}
				}
			}
		}

		now = k_uptime_get();
		if (g_link_up && now >= deadline) {
			motors_off();
			g_link_up = false;
			g_failsafe_cuts++;
		}

		/* Yield. A cooperative thread that never blocks starves the
		 * system; blocking with a timeout is what makes the failsafe
		 * independent of any byte ever arriving again. */
		(void)k_sem_take(&rx_sem, K_MSEC(ESP_MOTORS_TICK_MS));
	}
}

/* ---- status -------------------------------------------------------------- */

/* Decode the sender's flags into something readable. A hex byte is fine for a log and useless to
 * an operator deciding whether the thing in front of them is about to spin. */
static const char *flags_str(uint8_t f, char *buf, size_t len)
{
	size_t n = 0;

	buf[0] = '\0';
	if (f & MOTOR_LINK_FLAG_ARMED)   { n += snprintk(buf + n, len - n, "ARMED "); }
	if (f & MOTOR_LINK_FLAG_COMMIT)  { n += snprintk(buf + n, len - n, "COMMITTED "); }
	if (f & MOTOR_LINK_FLAG_ESTOP)   { n += snprintk(buf + n, len - n, "ESTOP "); }
	if (f & MOTOR_LINK_FLAG_TIMEOUT) { n += snprintk(buf + n, len - n, "TIMEOUT "); }
	if (f & MOTOR_LINK_FLAG_INHIBIT) { n += snprintk(buf + n, len - n, "INHIBIT "); }
	if (f & MOTOR_LINK_FLAG_WINDOW)  { n += snprintk(buf + n, len - n, "WINDOW-EXPIRED "); }
	if (buf[0] == '\0') { (void)snprintk(buf, len, "off"); }
	return buf;
}

static void status_thread(void *a, void *b, void *c)
{
	char fbuf[80];

	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	for (;;) {
		k_msleep(STATUS_PERIOD_MS);

		/* Deliberately parallel to motor_safe's
		 * "MOTOR GATE duty per mille" so the two read the same way.
		 * GATE is what was measured at the pin; CMD is what was
		 * commanded here -- they are different claims and the words say
		 * which one this is. */
		printk("MOTOR CMD duty per mille: M1=%u M2=%u M3=%u M4=%u\n",
		       (unsigned)g_duty[0] / 10U, (unsigned)g_duty[1] / 10U,
		       (unsigned)g_duty[2] / 10U, (unsigned)g_duty[3] / 10U);
		printk("  ceil=%u%% link=%s seq=%u flags=0x%02x[%s] frames=%u gap=%dms "
		       "lost=%u stale=%u crcerr=%u noise=%u cuts=%u pwmerr=%u\n",
		       (unsigned)ESP_MOTORS_MAX_DUTY_PCT,
		       g_link_up ? "UP" : "DOWN -- MOTORS OFF (failsafe)",
		       (unsigned)g_seq, (unsigned)g_flags,
		       flags_str(g_flags, fbuf, sizeof(fbuf)), (unsigned)g_frames,
		       (int)g_last_gap_ms, (unsigned)g_gaps, (unsigned)g_stale,
		       (unsigned)g_rx.bad_crc, (unsigned)g_rx.noise,
		       (unsigned)g_failsafe_cuts, (unsigned)g_pwm_errs);
	}
}

K_THREAD_STACK_DEFINE(motor_stack, 2048);
K_THREAD_STACK_DEFINE(status_stack, 1536);
static struct k_thread motor_thread_data;
static struct k_thread status_thread_data;

int main(void)
{
	printk("\n=== RiskyBird ESP motor offload: FPGA uart -> 4x LEDC PWM ===\n");
	printk("esp_motors: MOTOR1=GPIO21 MOTOR2=GPIO20 MOTOR3=GPIO23 MOTOR4=GPIO22, "
	       "%u ns period (%u kHz), active-high\n",
	       MOTOR_PERIOD_NSEC, 1000000U / MOTOR_PERIOD_NSEC);
	printk("esp_motors: failsafe %d ms, tick %d ms, duty ceiling %u%%\n",
	       ESP_MOTORS_TIMEOUT_MS, ESP_MOTORS_TICK_MS, ESP_MOTORS_MAX_DUTY_PCT);
	printk("esp_motors: THIS IMAGE DRIVES THE GATES -- the motor_safe gate probe "
	       "is not running\n");

	for (unsigned i = 0; i < MOTOR_LINK_NMOTORS; i++) {
		if (!device_is_ready(motors[i].dev)) {
			printk("esp_motors: PWM device for motor%u not ready -- REFUSING to run\n",
			       i + 1U);
			return -ENODEV;
		}
	}

	/*
	 * Zero the outputs before anything else can command them and before the
	 * UART is enabled.
	 *
	 * The window before this line is already safe, and not by luck:
	 * pwm_led_esp32_init() configures every DT-declared channel and calls
	 * pwm_led_esp32_stop() on it -- sig_out_en = 0, idle_lv = the `inverted`
	 * property, which none of these four set -- and only THEN applies pinctrl.
	 * So each pad is driven to a static LOW from the moment the LEDC signal
	 * reaches it, with the carrier's 10k gate pulldown underneath. Duty 0 goes
	 * down that same path, so "off" here is a hard static low rather than a
	 * zero-width pulse.
	 *
	 * That is read out of the driver, not measured on this board. Step 1 of
	 * the test plan measures it, with the battery disconnected.
	 */
	motors_off();

	if (!device_is_ready(fpga_uart)) {
		printk("esp_motors: fpga-uart not ready -- motors stay off\n");
		return -ENODEV;
	}
	ring_buf_init(&rx_ring, sizeof(rx_ring_buf), rx_ring_buf);
	uart_irq_callback_user_data_set(fpga_uart, uart_isr, NULL);
	uart_irq_rx_enable(fpga_uart);
	printk("esp_motors: FPGA link up on %s, waiting for the first frame\n",
	       fpga_uart->name);

	k_thread_create(&motor_thread_data, motor_stack,
			K_THREAD_STACK_SIZEOF(motor_stack),
			motor_thread, NULL, NULL, NULL,
			K_PRIO_COOP(0), 0, K_NO_WAIT);
	k_thread_name_set(&motor_thread_data, "motors");

	k_thread_create(&status_thread_data, status_stack,
			K_THREAD_STACK_SIZEOF(status_stack),
			status_thread, NULL, NULL, NULL,
			K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
	k_thread_name_set(&status_thread_data, "status");

	return 0;
}
