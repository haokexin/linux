// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#include <linux/cacheflush.h>
#include <asm/tlbflush.h>
#include <linux/dma-map-ops.h>
#include "usb_bst_list.h"

struct bulk_out_node *alloc_bulk_out_node(void)
{
	struct bulk_out_node *node = kmalloc(sizeof(*node), GFP_KERNEL);

	if (!node)
		return NULL;

	memset(node, 0, sizeof(*node));
	usb_init_urb(&node->urb);
	INIT_LIST_HEAD(&node->list);
	return node;
}

struct bulk_out_node *get_bulk_out_node(struct usb_virtual_device *vdev)
{
	struct bulk_out_node *node = NULL;

	spin_lock_irq(&vdev->bulk_out_lock);

	if (!list_empty(&vdev->bulk_out_list)) {
		node =
		    list_first_entry(&vdev->bulk_out_list, struct bulk_out_node,
				     list);
		list_del_init(&node->list);
	} else {
		spin_unlock_irq(&vdev->bulk_out_lock);
		node = alloc_bulk_out_node();
		if (node)
			node->vdev = vdev;
		return node;
	}

	spin_unlock_irq(&vdev->bulk_out_lock);
	return node;
}

int init_bulk_out_node(struct bulk_out_node *node, usb_bst_virsual_msg_t *pdu)
{
	void *buf = NULL;
	phys_addr_t phys_addr;
	size_t buf_len = 0;
	int ret = 0;

	phys_addr = ((u64) pdu->high_addr << 32) | pdu->low_addr;
	phys_addr = bus_to_phys(phys_addr);

	buf_len = pdu->len;

	memcpy(&(node->pdu), pdu, sizeof(usb_bst_virsual_msg_t));

	buf = memremap(phys_addr, buf_len, MEMREMAP_WB);
	if (!buf) {
		ret = -ENOMEM;
		return ret;
	}
	//ensure buf data ok
	smp_rmb();
	__aarch64_inval_dcache_range(buf, buf + buf_len);

	node->urb.transfer_dma = phys_addr;

	if (node->buf) {
		memunmap(node->buf);
		node->buf = NULL;
	}
	node->buf = buf;
	node->buf_len = buf_len;

	return 0;
}

void add_to_bulk_out_list(struct bulk_out_node *node)
{
	struct usb_virtual_device *vdev = NULL;

	if (node && node->vdev) {
		vdev = node->vdev;
		spin_lock_irq(&vdev->bulk_out_lock);
		list_add(&node->list, &vdev->bulk_out_list);
		spin_unlock_irq(&vdev->bulk_out_lock);
	}
}

void free_bulk_out_node(struct bulk_out_node *node)
{
	if (!node)
		return;
	usb_kill_urb(&node->urb);
	if (node->buf) {
		memunmap(node->buf);
		node->buf = NULL;
	}
	kfree(node);
}

void cleanup_bulk_out_list(struct usb_virtual_device *vdev)
{
	struct bulk_out_node *node, *tmp;
	LIST_HEAD(tmp_list);

	spin_lock_irq(&vdev->bulk_out_lock);
	list_for_each_entry_safe(node, tmp, &vdev->bulk_out_list, list) {
		list_del(&node->list);
		list_add_tail(&node->list, &tmp_list);
	}
	spin_unlock_irq(&vdev->bulk_out_lock);

	list_for_each_entry_safe(node, tmp, &tmp_list, list) {
		list_del(&node->list);
		free_bulk_out_node(node);
	}
}
