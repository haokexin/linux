/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

/*
 * IPC: Linux device driver for Black Sesame Technologies Inter Proccessor
 * Communication
 *
 */

#ifndef IPC_H
#define IPC_H

#include <linux/miscdevice.h>

#include "ipc_common.h"

struct bstipc {
	struct device *dev;
	struct miscdevice miscdev;
	struct ipc_mempool *pool;
	void __iomem *base;
	uint32_t id;
	void *private_data;
};

#endif
