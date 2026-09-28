/*
 * Console bring-up probe for the RiskyBird FPGA shells.
 *
 * The first thing to run on a shell whose UART has never emitted a character.
 * It proves four things in order, so a partial failure still says where it
 * stopped: the image was loaded and entered, the console UART is initialised
 * and drains, the CLINT timebase advances, and the C runtime can format.
 *
 * Deliberately free of every peripheral except the console. A shell built
 * without I2C, SPI or PWM still runs this unchanged, which is what makes it
 * the right first workload on a new carrier.
 *
 * It runs a bounded number of ticks and then finishes, rather than looping
 * forever. On a board whose console is unproven, "the program reached its end"
 * has to be observable without the console: HELLO_TICKS iterations bound the
 * run, hello_world_done() gives the debugger a symbol to break on, and
 * hello_world_state is a plain word in RAM that a debugger can read back. A
 * program that never terminates cannot distinguish "still running" from
 * "wedged".
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* Overridable at build time with -DEXTRA_CFLAGS=-DHELLO_TICKS=<n>. */
#ifndef HELLO_TICKS
#define HELLO_TICKS 40
#endif

#define HELLO_STATE_ENTERED  0x11111111u
#define HELLO_STATE_TICKED   0x22222222u
#define HELLO_STATE_FINISHED 0xc0ffeeu

/*
 * Volatile and non-static so it survives optimisation and keeps a symbol. The
 * debugger reads this; nothing in the program does.
 */
volatile uint32_t hello_world_state;

/*
 * A breakpoint target. noinline keeps it from being folded into main, so
 * "break hello_world_done" is a stable way to catch the end of the program.
 */
__attribute__((noinline)) void hello_world_done(void)
{
	hello_world_state = HELLO_STATE_FINISHED;
}

int main(void)
{
	hello_world_state = HELLO_STATE_ENTERED;

	printk("hello world from RiskyBird\n");
	printk("board   %s\n", CONFIG_BOARD);
	printk("arch    %s, %u-bit\n", CONFIG_ARCH, (unsigned int)(8 * sizeof(void *)));

	/*
	 * A counted, timed loop rather than a bare banner: a console that
	 * prints once but then wedges is a different fault from one that never
	 * prints, and only repeated output separates them. k_uptime_get also
	 * fails visibly if the CLINT timebase is wrong -- the deltas come out
	 * as zero or wildly off 250 ms.
	 */
	for (uint32_t tick = 0; tick < HELLO_TICKS; tick++) {
		hello_world_state = HELLO_STATE_TICKED;
		printk("tick %u  uptime %lld ms\n", tick, k_uptime_get());
		k_msleep(250);
	}

	printk("hello world complete after %u ticks\n", (unsigned int)HELLO_TICKS);
	hello_world_done();
	return 0;
}
