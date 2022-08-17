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
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT		intel_adsp_communication_widget
#define LOG_DOMAIN		communication_widget
#define DT_NODE			DT_NODELABEL(ace_comm_widget)
#define CW_BASE			DT_REG_ADDR(DT_NODE)

#include "intel_comm_widget.h"
#include "pmc_interface.h"

LOG_MODULE_REGISTER(LOG_DOMAIN);

struct comm_wdgt_data {
	struct k_thread thread;
	struct k_poll_signal signal;

	uint32_t memory_usage;

	bool sending;
	struct k_sem tx_sem;

	K_KERNEL_STACK_MEMBER(stack, CONFIG_MISC_INTEL_COMM_WIDGET_THREAD_STACK_SIZE);
};

#define SB_OPCODE_CRRD 0x06
#define SB_OPCODE_CRWR 0x07

static inline struct comm_wdgt_data *cw_dev(const struct device *dev)
{
	return (struct comm_wdgt_data *)dev->data;
}

void cw_sb_write(uint32_t dest, uint32_t func, uint16_t address, uint32_t data)
{
	cw_upstream_set_attr(dest, func, SB_OPCODE_CRWR, 0, 0);
	cw_upstream_set_address16(address);
	cw_upstream_set_data(data);
	cw_upstream_do_pw();
}

void cw_isr(const struct device *dev)
{
	struct comm_wdgt_data *self = cw_dev(dev);
	LOG_DBG("CW isr");

	uint32_t us_status = sys_read32(CW_BASE + USSTS);

	if (us_status & USSTS_MSGSENT) {
		__ASSERT(self->sending, "Spurious interrupt");

		LOG_DBG("CW msgsent");
		self->sending = false;
		cw_upstream_clear_msgsent();
		k_sem_give(&self->tx_sem);
	}
}

static void comm_wdgt_thread(struct comm_wdgt_data *self)
{
	struct k_poll_event events[1] = {
		K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
					 &self->signal),
	};

	cw_upstream_enable_sent_intr(true);

	events[0].state = K_POLL_STATE_NOT_READY;
	while (1) {
		k_poll(events, 1, K_FOREVER);
		k_poll_signal_reset(&self->signal);
		events[0].state = K_POLL_STATE_NOT_READY;

		if (!cw_upstream_ready())
			continue;

		uint32_t iface = FIELD_PREP(CW_PMC_IPC_OP_CODE, CW_PMC_OPC_SRAM_CONFIG) |
				 FIELD_PREP(CW_PMC_IPC_SRAM_USED_BANKS, self->memory_usage) |
				 CW_PMC_IPC_BUSY;

		self->sending = true;
		cw_sb_write(CW_PMC_DESTID_VALUE, 0, CW_PMC_MAILBOX3_INTERFACE_ADDRESS, iface);

		/* Wait for send interrupt */
		k_sem_take(&self->tx_sem, K_FOREVER);
		/* use cw_upstream_wait_for_sent() to wait in busy loop */
	}

}

static int cw_init(const struct device *dev)
{
	struct comm_wdgt_data *self = cw_dev(dev);
	k_tid_t tid;

	memset(self, 0, sizeof(*self));

	k_sem_init(&self->tx_sem, 0, 1);
	k_poll_signal_init(&self->signal);

	IRQ_CONNECT(DT_IRQN(DT_NODE), DT_IRQ(DT_NODE, priority), cw_isr, DEVICE_DT_GET(DT_NODE),
		    DT_IRQ(DT_NODE, sense));
	irq_enable(DT_IRQN(DT_NODE));

	tid = k_thread_create(&self->thread, self->stack,
			      CONFIG_MISC_INTEL_COMM_WIDGET_THREAD_STACK_SIZE,
			      (k_thread_entry_t)comm_wdgt_thread, self, NULL, NULL,
			      CONFIG_MISC_INTEL_COMM_WIDGET_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(tid, "intel_comm_widget");

	return 0;
}

/*
 * Report number of used HP-SRAM memory block to the PMC, unit is 32 KB.
 */
void intel_comm_widget_pmc_send_ipc(const struct device *dev, uint16_t banks)
{
	struct comm_wdgt_data *self = cw_dev(dev);
	self->memory_usage = banks;
	k_poll_signal_raise(&self->signal, 0);
}

static struct comm_wdgt_data comm_wdgt_data;

DEVICE_DT_DEFINE(DT_NODE, cw_init, NULL, &comm_wdgt_data, NULL, POST_KERNEL, 0, NULL);
