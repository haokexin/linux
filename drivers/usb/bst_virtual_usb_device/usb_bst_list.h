/* SPDX-License-Identifier: GPL-2.0 OR Apache 2.0
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

#ifndef __USB_BST_LIST_H__
#define __USB_BST_LIST_H__

#include "usb_bst_virt_device.h"

struct bulk_out_node {
	usb_bst_virsual_msg_t pdu;
	struct usb_virtual_device *vdev;
	struct urb urb;
	char *buf;
	size_t buf_len;
	struct list_head list;
};

struct bulk_out_node *get_bulk_out_node(struct usb_virtual_device *vdev);
void free_bulk_out_node(struct bulk_out_node *node);
void add_to_bulk_out_list(struct bulk_out_node *node);
int init_bulk_out_node(struct bulk_out_node *node, usb_bst_virsual_msg_t *pdu);
void cleanup_bulk_out_list(struct usb_virtual_device *vdev);
#endif
