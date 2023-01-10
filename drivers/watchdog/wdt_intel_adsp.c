/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Copyright (c) 2023 Intel Corporation
 *
 * Author: Adrian Warecki <adrian.warecki@intel.com>
 */

#define DT_DRV_COMPAT intel_adsp_watchdog

#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys_clock.h>
#include <zephyr/math/ilog2.h>

#include "wdt_dw.h"
#include "wdt_dw_common.h"
#include "wdt_intel_adsp.h"

LOG_MODULE_REGISTER(wdt_intel_adsp, CONFIG_WDT_LOG_LEVEL);

#define DEV_NODE				DT_INST(0, DT_DRV_COMPAT)
#define WDT_INTEL_ADSP_INTERRUPT_SUPPORT	DT_NODE_HAS_PROP(DEV_NODE, interrupts)
#define WDT_INTEL_ADSP_INTERRUPT_ALLOW_RESET	0x40000000

/* Device run time data */
struct intel_adsp_wdt_dev_data {
	uint32_t device[CONFIG_MP_MAX_NUM_CPUS];
	uint32_t config;
#if WDT_INTEL_ADSP_INTERRUPT_SUPPORT
	wdt_callback_t callback;
#endif
};

/* Device constant configuration parameters */
struct intel_adsp_wdt_dev_cfg {
	uint32_t base;
	uint32_t clk_freq;
};

static int intel_adsp_wdt_setup(const struct device *dev, uint8_t options)
{
	const struct intel_adsp_wdt_dev_cfg *const dev_config = dev->config;
	struct intel_adsp_wdt_dev_data *const dev_data = dev->data;
	const bool can_reset = dev_data->config & WDT_INTEL_ADSP_INTERRUPT_ALLOW_RESET;
	unsigned int i;
	int ret;

	ret = dw_wdt_check_options(options);
	if (ret) {
		return ret;
	}

	for (i = 0; i < CONFIG_MP_MAX_NUM_CPUS; i++) {
		ret = dw_wdt_configure(dev_data->device[i], dev_data->config);
		if (ret) {
			return ret;
		}

#if WDT_INTEL_ADSP_INTERRUPT_SUPPORT
		dw_wdt_response_mode_set(dev_data->device[i], !!dev_data->callback);
#endif
		intel_adsp_wdt_reset_set(dev_config->base, i, can_reset);
	}

	return 0;
}

static int intel_adsp_wdt_install_timeout(const struct device *dev,
					  const struct wdt_timeout_cfg *config)
{
	const struct intel_adsp_wdt_dev_cfg *const dev_config = dev->config;
	struct intel_adsp_wdt_dev_data *const dev_data = dev->data;
	int ret;

#if WDT_INTEL_ADSP_INTERRUPT_SUPPORT
	dev_data->callback = config->callback;
#else
	if (config->callback) {
		LOG_ERR("Interrupt is not configured, can't set a callback.");
		return -ENOTSUP;
	}
#endif

	ret = dw_wdt_calc_period(dev_data->device[0], dev_config->clk_freq, config,
				 &dev_data->config);
	if (ret) {
		return ret;
	}

	if (config->flags & WDT_FLAG_RESET_CPU_CORE) {
		dev_data->config |= WDT_INTEL_ADSP_INTERRUPT_ALLOW_RESET;
	}

	return 0;
}

static int intel_adsp_wdt_feed(const struct device *dev, int channel_id)
{
	struct intel_adsp_wdt_dev_data *const dev_data = dev->data;

	if (channel_id >= CONFIG_MP_MAX_NUM_CPUS) {
		return -EINVAL;
	}

	dw_wdt_counter_restart(dev_data->device[channel_id]);
	return 0;
}

#if WDT_INTEL_ADSP_INTERRUPT_SUPPORT
static void intel_adsp_wdt_isr(const struct device *dev)
{
	const struct intel_adsp_wdt_dev_cfg *const dev_config = dev->config;
	struct intel_adsp_wdt_dev_data *const dev_data = dev->data;
	const uint32_t cpu = arch_proc_id();
	const uint32_t base = dev_data->device[cpu];

	if (dw_wdt_interrupt_status_register_get(base)) {
		dw_wdt_clear_interrupt(base);

		if (dev_data->callback) {
			dev_data->callback(dev, cpu);
		}
	}
}
#endif

static int intel_adsp_wdt_init(const struct device *dev)
{
	const unsigned int reset_pulse_length =
		ilog2(DT_PROP(DEV_NODE, reset_pulse_length)) - 1;
	const struct intel_adsp_wdt_dev_cfg *const dev_config = dev->config;
	struct intel_adsp_wdt_dev_data *const dev_data = dev->data;
	unsigned int i;
	int ret;

	for (i = 0; i < CONFIG_MP_MAX_NUM_CPUS; i++) {
		dev_data->device[i] = intel_adsp_wdt_pointer_get(dev_config->base, i);
		ret = dw_wdt_probe(dev_data->device[i], reset_pulse_length);
		if (ret) {
			return ret;
		}
	}

#if WDT_INTEL_ADSP_INTERRUPT_SUPPORT
	IRQ_CONNECT(DT_IRQN(DEV_NODE), DT_IRQ(DEV_NODE, priority), intel_adsp_wdt_isr,
		    DEVICE_DT_GET(DEV_NODE), DT_IRQ(DEV_NODE, sense));
	irq_enable(DT_IRQN(DEV_NODE));
#endif

	return 0;
}

/**
 * @brief Pause watchdog operation
 *
 * Sets the pause signal to stop the watchdog timing
 *
 * @param dev Pointer to the device structure
 * @param channel_id Channel identifier
 */
int intel_adsp_watchdog_pause(const struct device *dev, const int channel_id)
{
	const struct intel_adsp_wdt_dev_cfg *const dev_config = dev->config;

	if (channel_id >= CONFIG_MP_MAX_NUM_CPUS) {
		return -EINVAL;
	}

	intel_adsp_wdt_pause(dev_config->base, channel_id);
	return 0;
}

/**
 * @brief Resume watchdog operation
 *
 * Clears the pause signal to resume the watchdog timing
 *
 * @param dev Pointer to the device structure
 * @param channel_id Channel identifier
 */
int intel_adsp_watchdog_resume(const struct device *dev, const int channel_id)
{
	const struct intel_adsp_wdt_dev_cfg *const dev_config = dev->config;

	if (channel_id >= CONFIG_MP_MAX_NUM_CPUS) {
		return -EINVAL;
	}

	intel_adsp_wdt_resume(dev_config->base, channel_id);
	return 0;
}

static const struct wdt_driver_api intel_adsp_wdt_api = {
	.setup = intel_adsp_wdt_setup,
	.disable = dw_wdt_disable,
	.install_timeout = intel_adsp_wdt_install_timeout,
	.feed = intel_adsp_wdt_feed,
};

#if !(DT_NODE_HAS_PROP(DEV_NODE, clock_frequency) || DT_NODE_HAS_PROP(DEV_NODE, clocks))
#error Clock frequency not configured!
#endif

static const struct intel_adsp_wdt_dev_cfg wdt_intel_adsp_config = {
	.base = DT_REG_ADDR(DEV_NODE),
	COND_CODE_1(DT_NODE_HAS_PROP(DEV_NODE, clock_frequency),
		    (.clk_freq = DT_PROP(DEV_NODE, clock_frequency)),
		    (.clk_freq = DT_PROP_BY_PHANDLE(DEV_NODE, clocks, clock_frequency))
	),
};

static struct intel_adsp_wdt_dev_data wdt_intel_adsp_data;

DEVICE_DT_DEFINE(DEV_NODE, &intel_adsp_wdt_init, NULL, &wdt_intel_adsp_data, &wdt_intel_adsp_config,
		 POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &intel_adsp_wdt_api);
