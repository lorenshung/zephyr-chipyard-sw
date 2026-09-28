/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Chipyard's logical SiFive SPI block has no run-time pin-multiplexer. The
 * upstream SiFive SPI driver includes the pinctrl API even when CONFIG_PINCTRL
 * is disabled, so the Rocketchip virtual SoC still needs to provide its opaque
 * pin-description type. No physical package-pin mapping is encoded here.
 */

#ifndef RISKYBIRD_CHIPYARD_PINCTRL_SOC_H_
#define RISKYBIRD_CHIPYARD_PINCTRL_SOC_H_

#include <stdint.h>

typedef uint32_t pinctrl_soc_pin_t;

#endif /* RISKYBIRD_CHIPYARD_PINCTRL_SOC_H_ */
