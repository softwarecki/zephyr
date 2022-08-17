/*
 * Copyright (c) 2022 Intel Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

/* Report number of used HP-SRAM memory banks to the PMC, unit is 32 KB. */
int intel_comm_widget_pmc_send_ipc(const struct device *dev, uint16_t banks);
