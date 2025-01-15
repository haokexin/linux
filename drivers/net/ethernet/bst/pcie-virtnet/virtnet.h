/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023~2024 Black Sesame Technologies, Inc.
 *
 * Author: Xuran Yang <xuran.yang@bst.ai>
 */

#ifndef _VIRTNET_H
#define _VIRTNET_H

#include <linux/pci-epf.h>
#include <linux/types.h>

#define VNET_MAX_MTU		65536
#define VNET_SKB_REV		20

#define VNET_MAPED_BAR		2
#define VNET_USED_BAR		BAR_1
#define VNET_POLL_INTERVAL	500 /* us */

/* RC to EP int */
#define VNET_NOTIFY_EP		0

/* EP to RC int */
#define VNET_MSI_TOTAL		2
#define VNET_NOTIFY_MSI		0
#define VNET_NOTIFY_XMIT_DONE	1

#define VNET_MEM_SIZE		0x100000
#define VNET_DESC_NUM		32

/* Virtnet status flags */
#define __VNET_MAGIC		(0x25EF << 16)
#define __VNET_INIT		0
#define __VNET_POLLING		1
#define __VNET_DOWN		2
#define __VNET_MAINTEN		3
#define __VNET_WAITING		4

struct virtnet_bar {
	u32 ep_status;
	u32 rc_status;
	u32 rc_int_mask;
	u32 ep_int_mask;
	u32 ep_int_bar;
	u32 ep_int_base;
	u32 ep_int_msg;
	u32 ep_int_cnt;

	u32 rc_queue_base;
	u32 rc_desc_base;
	u32 rc_desc_num;

	u32 ep_queue_base;
	u32 ep_desc_base;
	u32 ep_desc_num;
};

/* BAR address map */
#define VNET_RC_DESC_HEAD	0x10000
#define VNET_EP_DESC_HEAD	0x20000
#define VNET_MSIX_TABLE		0x30000

#endif
