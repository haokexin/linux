/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef IPC_MAILBOX_CONTROLLER_H
#define IPC_MAILBOX_CONTROLLER_H

#include "ipc_common.h"
#include "ipc_msg_manager.h"
#include "ipc_nodemanager.h"

enum rx_mode {
	IRQ_MODE = 0,
	POLL_MODE
};
#define RX_MODE enum rx_mode
#define MAX_CPU 6
#define MAX_SRC 21

struct ipc_mbox {
	struct device *dev;
	void __iomem *event_base;
	void __iomem *sem_base;
	struct ipc_mempool *pool;
	spinlock_t lock[MAX_CPU][MAX_SRC];

#ifdef ON_FPGA
	void __iomem *fpga_reset;
	void __iomem *fpga_status;
#endif
};

#define IPC_BASE_OFFSET 0x100000

// share buffer format definition
struct ipc_aligned_msg {
	struct ipc_fill_register_msg msg;
#ifdef MSG_SIZE_EXTENSION
	uint64_t payload[6];
#else
	uint64_t payload[7];
#endif
};

// ipc shared buffer
struct ipc_all_cores_register_addr // place in one page : 4096Byte
{
	struct ipc_aligned_msg addr[IPC_CORE_MAX]; // 64B*22 = 1408B
	uint64_t data_saved[IPC_CORE_MAX][4096]; // 8B * 22 * 4096 = 704 KB
};

int32_t ipc_send_data(struct ipc_client_info *client_info, enum ipc_core_e src,
		      void *data);
void * translate_address_by_system(void * addr);
void * translate_address_by_src(enum ipc_core_e cpu_id,void * addr);

#endif
