/*
 * Copyright (c) 2022 Intel Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

/* Report number of used HP-SRAM memory block to the PMC, unit is 32 KB. */
void intel_comm_widget_pmc_send_ipc(const struct device *dev, uint16_t banks);
