/*
 * ESP link bring-up: the second peripheral UART on E13/F14.
 *
 * The full drone shells elaborate a second sifive UART at 0x1002_1000 wired to the ESP32
 * (uart1_rxd E13 = J1_96 <- ESP TX; uart1_txd F14 = J1_98 -> ESP RX). This target answers the
 * only questions that matter before anything is built on top of it: does the controller exist,
 * does it transmit, and is anything on the other end talking.
 *
 * Deliberately POLLED, not interrupt-driven. uart_poll_in/out touch only the controller's own
 * registers, so this target still works if the PLIC source in the overlay is wrong -- and a
 * console bound to the wrong source is exactly the failure this bench has hit before. Proving
 * the data path first, and the interrupt path separately, keeps the two from masking each other.
 *
 * Nothing in this repository records what baud the ESP firmware runs at, so a fixed guess would
 * turn "wrong rate" into "dead link". The listen phase therefore sweeps the plausible rates and
 * reports per-rate byte counts. The distinction that matters:
 *
 *   - bytes at exactly one rate, decoding cleanly   -> that is the ESP's rate
 *   - bytes at several rates, all garbage           -> a real signal; none of these is the rate
 *   - zero bytes at every rate                      -> no signal at all on J1_96
 *
 * A UART receiver at the wrong rate still produces bytes from a real signal. Only a line with
 * no edges yields exactly zero, which is what makes the all-zero case a wiring statement rather
 * than a configuration one.
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/sys_io.h>

/* sifive UART register map, for ground truth independent of the driver. */
#define ESP_UART_BASE   0x10021000UL
#define UART_TXDATA     (ESP_UART_BASE + 0x00)
#define UART_RXDATA     (ESP_UART_BASE + 0x04)
#define UART_TXCTRL     (ESP_UART_BASE + 0x08)
#define UART_RXCTRL     (ESP_UART_BASE + 0x0c)
#define UART_DIV        (ESP_UART_BASE + 0x18)

#define PERIPH_HZ       50000000U

#define LISTEN_MS       1500
/*
 * How much of each rate's capture to print. Raise it to read what the ESP is actually saying
 * once a rate is known to decode: RB_EXTRA_CFLAGS="-DESP_DUMP=768 -DESP_ONLY_RATE=115200".
 */
#ifndef ESP_DUMP
#define ESP_DUMP        48
#endif
#define DUMP_MAX        ESP_DUMP

#ifdef ESP_ONLY_RATE
/* Single-rate mode: skip the sweep and spend the whole window on one known-good rate. */
static const uint32_t rates[] = { ESP_ONLY_RATE };
#else
static const uint32_t rates[] = { 115200, 921600, 460800, 230400, 57600, 9600 };
#endif

static const struct device *esp;

static void show_regs(const char *when)
{
	uint32_t div = sys_read32(UART_DIV);

	printf("  regs %-8s txctrl=0x%08x rxctrl=0x%08x div=%u", when,
	       sys_read32(UART_TXCTRL), sys_read32(UART_RXCTRL), div);
	if (div != 0U) {
		printf(" -> %u baud", PERIPH_HZ / (div + 1U));
	}
	printf("\n");
}

/* Drain for a bounded window, returning how many bytes arrived. */
static uint32_t listen(uint32_t ms, uint8_t *buf, uint32_t cap)
{
	uint32_t got = 0;
	int64_t end = k_uptime_get() + ms;
	unsigned char c;

	while (k_uptime_get() < end) {
		if (uart_poll_in(esp, &c) == 0) {
			if (got < cap) {
				buf[got] = c;
			}
			got++;
		}
	}
	return got;
}

static void dump(const uint8_t *buf, uint32_t n)
{
	uint32_t shown = (n < DUMP_MAX) ? n : DUMP_MAX;

	printf("      ");
	for (uint32_t i = 0; i < shown; i++) {
		printf("%02x ", buf[i]);
	}
	printf("\n      --- as text ---\n      ");
	for (uint32_t i = 0; i < shown; i++) {
		if (buf[i] == '\n') {
			printf("\n      ");
		} else if (buf[i] >= 0x20 && buf[i] < 0x7f) {
			printf("%c", buf[i]);
		} else if (buf[i] != '\r') {
			printf(".");
		}
	}
	printf("\n");
}

/*
 * Set the baud rate by writing the divisor register directly.
 *
 * uart_configure() is not an option: the sifive driver implements no runtime-configure callback
 * and returns -ENOSYS (-88) for every rate, which silently turns a rate sweep into no sweep at
 * all. The controller's own DIV register is the whole mechanism the driver would have used --
 * div = f_periph/baud - 1 -- so writing it is equivalent and works. Everything else about the
 * port (TX/RX enables, watermarks) is left exactly as the driver set it up.
 */
static uint32_t set_rate(uint32_t baud)
{
	uint32_t div = (PERIPH_HZ / baud) - 1U;

	sys_write32(div, UART_DIV);
	/* Drop anything the FIFO latched while the divisor was mid-change. */
	for (int i = 0; i < 64; i++) {
		(void)sys_read32(UART_RXDATA);
	}
	return div;
}

int main(void)
{
	static uint8_t buf[4096];
	uint32_t total = 0;

	printf("\n=====================================================\n");
	printf("RiskyBird ESP-link UART bring-up\n");
	printf("  controller : uart1 @ 0x%08x (PLIC source 11)\n", (unsigned)ESP_UART_BASE);
	printf("  pins       : rxd E13/J1_96 <- ESP TX ; txd F14/J1_98 -> ESP RX\n");
	printf("  built      : " __DATE__ " " __TIME__ "\n");
	printf("=====================================================\n\n");

	esp = DEVICE_DT_GET(DT_ALIAS(esp_uart));
	if (!device_is_ready(esp)) {
		printf("[FAIL] uart1 device not ready: the shell has no serial@10021000,\n");
		printf("       or fpga-esp-uart.overlay was not applied.\n");
		printf("RESULT: FAIL\n");
		return 1;
	}
	printf("[1/4] device ready: %s\n", esp->name);
	show_regs("initial");

	/*
	 * Transmit before listening. uart_sifive_poll_out spins on the FIFO-full bit, so a
	 * controller whose transmitter never drains would hang here rather than return -- the
	 * fact that this stage completes is itself the proof that TX is clocked and draining.
	 */
	printf("\n[2/4] transmitting a probe on uart1\n");
	static const char probe[] = "\r\nRISKYBIRD-FPGA-PROBE\r\n";
	for (const char *s = probe; *s != '\0'; s++) {
		uart_poll_out(esp, *s);
	}
	printf("      sent %u bytes; TX drained (poll_out would spin otherwise)\n",
	       (unsigned)(sizeof(probe) - 1));
	show_regs("after tx");

	printf("\n[3/4] listening %u ms per candidate rate\n", LISTEN_MS);
	for (size_t i = 0; i < ARRAY_SIZE(rates); i++) {
		uint32_t div = set_rate(rates[i]);
		uint32_t n;

		/* Re-send the probe at each rate: if the ESP only answers when spoken to,
		 * a purely passive listen would report a dead link. */
		for (const char *s = probe; *s != '\0'; s++) {
			uart_poll_out(esp, *s);
		}
		n = listen(LISTEN_MS, buf, sizeof(buf));
		total += n;
		printf("  %7u baud (div=%4u) : %u byte(s)\n", rates[i], div, n);
		if (n > 0U) {
			dump(buf, n);
		}
	}
	set_rate(115200);

	printf("\n[4/4] verdict\n");
	if (total == 0U) {
		printf("      Nothing arrived at any rate. A receiver at the wrong rate still\n");
		printf("      emits bytes from a real signal, so zero at every rate means no\n");
		printf("      edges on J1_96 at all -- the ESP is not driving it, is not\n");
		printf("      powered, or is not running firmware that transmits.\n");
		printf("      TX is proven regardless: the probe drained.\n");
		printf("RESULT: PARTIAL -- uart1 transmits, nothing received\n");
		return 0;
	}
	printf("      %u byte(s) received. A rate that decodes to readable text is the\n", total);
	printf("      ESP's; bytes at every rate mean a real signal at none of them.\n");
	printf("RESULT: PASS -- uart1 transmits and receives\n");
	return 0;
}
