/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RB_TINYMPC_COMPAT_H_
#define RB_TINYMPC_COMPAT_H_

/*
 * Accelerated-TinyMPC expects the full C math header. Zephyr's minimal libc
 * intentionally exposes only sqrt/sqrtf. The solver's scalar code only needs
 * absolute value, so map it to the compiler builtin without adding a symbol
 * that can conflict with a later C-library declaration.
 */
#define fabs(value) __builtin_fabs(value)

#ifndef RAND_MAX
#define RAND_MAX 2147483647
extern "C" int rand(void);
#endif

#endif /* RB_TINYMPC_COMPAT_H_ */
