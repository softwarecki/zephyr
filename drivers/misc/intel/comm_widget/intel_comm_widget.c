/*
 * Copyright (c) 2022 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#define DT_DRV_COMPAT		intel_adsp_communication_widget

#include "intel_comm_widget.h"

#define SB_OPCODE_CRRD 0x06
#define SB_OPCODE_CRWR 0x07

void cw_sb_write(uint32_t dest, uint32_t func, uint16_t address, uint32_t data)
{
	cw_upstream_set_attr(dest, func, SB_OPCODE_CRWR, 0, 0);
	cw_upstream_set_address16(address);
	cw_upstream_set_data(data);
	cw_upstream_do_pw();
}

static int cw_init(const struct device *dev)
{
	return 0;
}

DEVICE_DT_DEFINE(CW_DT_NODE, cw_init, NULL, NULL, NULL, POST_KERNEL, 0, NULL);
