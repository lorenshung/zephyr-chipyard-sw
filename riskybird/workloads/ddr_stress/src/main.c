/*
 * DDR4 stress test for the KU040 shells.
 *
 * Walks a configurable window of main memory with four patterns, each written
 * across the whole window and then read back and compared, so a fault shows up
 * as a specific address rather than a hang:
 *
 *   1. address-in-address  -- each word stores its own address. Catches stuck
 *      or swapped address lines, which a uniform pattern cannot see: aliasing
 *      two addresses still reads back "correct" data if the data is identical.
 *   2. 0x5555... / 0xAAAA... -- adjacent-bit patterns, for shorted data lines.
 *   3. inverted address-in-address -- exercises the complementary bit of every
 *      data line at every address the first pass already covered.
 *   4. LFSR pseudorandom -- a long non-repeating sequence, reproducible from
 *      the seed so a mismatch can be attributed without storing a copy.
 *
 * Throughput is reported per pass from the cycle counter. The window starts
 * well above the image so the running program is never a target.
 */

#include <stdio.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/timing/timing.h>

/*
 * Window size in MiB. Overridable at build time with
 * -DEXTRA_CFLAGS=-DDDR_STRESS_MIB=<n>; kept a plain define rather than a
 * Kconfig symbol so the app needs no Kconfig root of its own and can use
 * hardware/zephyr/Kconfig.workload like every other rb workload.
 */
#ifndef DDR_STRESS_MIB
#define DDR_STRESS_MIB 64
#endif

/* Skip the first 32 MiB: image, stacks and heap live down there. */
#define WINDOW_BASE  0x82000000UL
#define WINDOW_BYTES (DDR_STRESS_MIB * 1024UL * 1024UL)
#define WORDS        (WINDOW_BYTES / sizeof(uint64_t))

static volatile uint64_t *const mem = (volatile uint64_t *)WINDOW_BASE;

static uint64_t lfsr_next(uint64_t s)
{
	/* x^64 + x^63 + x^61 + x^60 + 1 -- maximal length over 64 bits. */
	uint64_t bit = ((s >> 63) ^ (s >> 62) ^ (s >> 60) ^ (s >> 59)) & 1ULL;
	return (s << 1) | bit;
}

static void report_mismatch(volatile uint64_t *at, uint64_t got, uint64_t want)
{
	/* Split into 32-bit halves: the nano formatter has no length modifiers. */
	printf("    MISMATCH @0x%x got %08x%08x want %08x%08x\n",
	       (unsigned)(uintptr_t)at,
	       (unsigned)(got >> 32), (unsigned)got,
	       (unsigned)(want >> 32), (unsigned)want);
}

static uint32_t report(const char *name, uint32_t start_cycles, uint32_t errors)
{
	uint32_t elapsed = k_cycle_get_32() - start_cycles;
	uint32_t ms = (uint32_t)k_cyc_to_ms_floor64(elapsed);
	uint32_t kbps = ms ? (uint32_t)((WINDOW_BYTES * 2ULL) / ms) : 0;

	printf("  %-22s %6u ms  %5u KB/s (rw)  errors: %u\n",
	       name, ms, kbps, errors);
	return errors;
}

int main(void)
{
	uint32_t total = 0, t0;

	printf("\nDDR stress: %u MiB at 0x%x, %u x 64-bit words\n",
	       (unsigned)DDR_STRESS_MIB, (unsigned)WINDOW_BASE,
	       (unsigned)WORDS);

	/* 1. address-in-address */
	t0 = k_cycle_get_32();
	for (size_t i = 0; i < WORDS; i++) {
		mem[i] = (uint64_t)(uintptr_t)&mem[i];
	}
	uint32_t errors = 0;
	for (size_t i = 0; i < WORDS; i++) {
		if (mem[i] != (uint64_t)(uintptr_t)&mem[i]) {
			if (errors < 4) {
				report_mismatch(&mem[i], mem[i],
						(uint64_t)(uintptr_t)&mem[i]);
			}
			errors++;
		}
	}
	total += report("address-in-address", t0, errors);

	/* 2. alternating adjacent-bit patterns */
	t0 = k_cycle_get_32();
	for (size_t i = 0; i < WORDS; i++) {
		mem[i] = (i & 1) ? 0xAAAAAAAAAAAAAAAAULL : 0x5555555555555555ULL;
	}
	errors = 0;
	for (size_t i = 0; i < WORDS; i++) {
		uint64_t want = (i & 1) ? 0xAAAAAAAAAAAAAAAAULL
					: 0x5555555555555555ULL;
		if (mem[i] != want) {
			if (errors < 4) {
				report_mismatch(&mem[i], mem[i], want);
			}
			errors++;
		}
	}
	total += report("0x55/0xAA alternating", t0, errors);

	/* 3. inverted address-in-address */
	t0 = k_cycle_get_32();
	for (size_t i = 0; i < WORDS; i++) {
		mem[i] = ~(uint64_t)(uintptr_t)&mem[i];
	}
	errors = 0;
	for (size_t i = 0; i < WORDS; i++) {
		if (mem[i] != ~(uint64_t)(uintptr_t)&mem[i]) {
			errors++;
		}
	}
	total += report("inverted address", t0, errors);

	/* 4. LFSR pseudorandom, replayed from the same seed to check */
	t0 = k_cycle_get_32();
	uint64_t s = 0x0123456789ABCDEFULL;
	for (size_t i = 0; i < WORDS; i++) {
		mem[i] = s;
		s = lfsr_next(s);
	}
	errors = 0;
	s = 0x0123456789ABCDEFULL;
	for (size_t i = 0; i < WORDS; i++) {
		if (mem[i] != s) {
			errors++;
		}
		s = lfsr_next(s);
	}
	total += report("LFSR pseudorandom", t0, errors);

	printf("\nDDR stress %s -- %u total errors over %u MiB x 4 passes\n",
	       total ? "FAILED" : "PASSED", total, DDR_STRESS_MIB);
	return 0;
}
