// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#include "usb_bst_pdu_kfifo.h"

struct kfifo *g_msglist_bulk_out, *g_msglist_bulk_in;
spinlock_t g_bulk_out_lock, g_bulk_in_lock;

int msg_fifo_malloc(void)
{

	g_msglist_bulk_out = kmalloc(sizeof(*g_msglist_bulk_out), GFP_KERNEL);
	g_msglist_bulk_in = kmalloc(sizeof(*g_msglist_bulk_in), GFP_KERNEL);

	if (!g_msglist_bulk_out || !g_msglist_bulk_in) {
		pr_err("Failed to allocate memory for kfifo\n");
		return -ENOMEM;
	}

	if (kfifo_alloc
	    (g_msglist_bulk_out, FIFO_SIZE * sizeof(usb_bst_virsual_msg_t),
	     GFP_KERNEL) < 0) {
		pr_err("Failed to allocate memory for g_msglist_bulk_out\n");
		kfree(g_msglist_bulk_out);
		kfree(g_msglist_bulk_in);
		return -ENOMEM;
	}

	if (kfifo_alloc
	    (g_msglist_bulk_in, FIFO_SIZE * sizeof(usb_bst_virsual_msg_t),
	     GFP_KERNEL) < 0) {
		pr_err("Failed to allocate memory for g_msglist_bulk_in\n");
		kfifo_free(g_msglist_bulk_out);
		kfree(g_msglist_bulk_out);
		kfree(g_msglist_bulk_in);
		return -ENOMEM;
	}
	spin_lock_init(&g_bulk_out_lock);
	spin_lock_init(&g_bulk_in_lock);

	return 0;
}

void msg_fifo_free(void)
{

	kfifo_free(g_msglist_bulk_out);
	kfifo_free(g_msglist_bulk_in);
	kfree(g_msglist_bulk_out);
	kfree(g_msglist_bulk_in);
}

int push_pdu_to_msglist_bulk_out(usb_bst_virsual_msg_t *pdu)
{
	int ret;
	unsigned long flags;

	pr_debug("%s BASE: core_id %x command %x   high_addr %x low_addr %x offset %x len %x seqnum %x\n",
	     __func__, pdu->core_id, pdu->command, pdu->high_addr, pdu->low_addr,
	     pdu->offset, pdu->len, pdu->seqnum);
	spin_lock_irqsave(&g_bulk_out_lock, flags);
	ret = kfifo_in(g_msglist_bulk_out, (unsigned char *)pdu,
			sizeof(usb_bst_virsual_msg_t));
	spin_unlock_irqrestore(&g_bulk_out_lock, flags);

	return ret;
}

int pop_pdu_from_msglist_bulk_out(usb_bst_virsual_msg_t *pdu)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&g_bulk_out_lock, flags);
	ret = kfifo_out(g_msglist_bulk_out, (unsigned char *)pdu,
			 sizeof(usb_bst_virsual_msg_t));
	spin_unlock_irqrestore(&g_bulk_out_lock, flags);

	return ret;
}

int pdu_len_msglist_bulk_out(void)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&g_bulk_out_lock, flags);
	ret = kfifo_len(g_msglist_bulk_out);
	spin_unlock_irqrestore(&g_bulk_out_lock, flags);

	return ret;
}

int push_pdu_to_msglist_bulk_in(usb_bst_virsual_msg_t *pdu)
{
	int ret;
	unsigned long flags;

	pr_debug("%s BASE: core_id %x command %x   high_addr %x low_addr %x offset %x len %x seqnum %x\n",
	     __func__, pdu->core_id, pdu->command, pdu->high_addr, pdu->low_addr,
	     pdu->offset, pdu->len, pdu->seqnum);
	spin_lock_irqsave(&g_bulk_in_lock, flags);
	ret = kfifo_in(g_msglist_bulk_in, (unsigned char *)pdu,
			sizeof(usb_bst_virsual_msg_t));
	spin_unlock_irqrestore(&g_bulk_in_lock, flags);

	return ret;
}

int pop_pdu_from_msglist_bulk_in(usb_bst_virsual_msg_t *pdu)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&g_bulk_in_lock, flags);
	ret = kfifo_out(g_msglist_bulk_in, (unsigned char *)pdu,
			 sizeof(usb_bst_virsual_msg_t));
	spin_unlock_irqrestore(&g_bulk_in_lock, flags);

	return ret;
}

int pdu_len_msglist_bulk_in(void)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&g_bulk_in_lock, flags);
	ret = kfifo_len(g_msglist_bulk_in);
	spin_unlock_irqrestore(&g_bulk_in_lock, flags);

	return ret;
}
