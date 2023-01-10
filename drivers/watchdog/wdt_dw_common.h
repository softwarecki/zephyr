/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Copyright (c) 2023 Intel Corporation
 *
 * Author: Adrian Warecki <adrian.warecki@intel.com>
 */

#ifndef ZEPHYR_DRIVERS_WATCHDOG_WDT_DW_COMMON_H_
#define ZEPHYR_DRIVERS_WATCHDOG_WDT_DW_COMMON_H_

#include <stdint.h>

int dw_wdt_check_options(const uint8_t options);

int dw_wdt_configure(const uint32_t base, const uint32_t config);

int dw_wdt_calc_period(const uint32_t base, const uint32_t clk_freq,
		       const struct wdt_timeout_cfg *config, uint32_t *period_out);

int dw_wdt_probe(const uint32_t base, const uint32_t reset_pulse_length);

int dw_wdt_disable(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_WATCHDOG_WDT_DW_COMMON_H_ */
