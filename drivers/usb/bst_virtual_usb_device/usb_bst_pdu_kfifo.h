/* SPDX-License-Identifier: GPL-2.0 OR Apache 2.0
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

#ifndef USB_BST_PDU_KFIFO_H
#define USB_BST_PDU_KFIFO_H
#include <linux/kfifo.h>
#include <linux/slab.h>

#include "usb_bst_virt_msg.h"

#define FIFO_SIZE 256

int msg_fifo_malloc(void);
void msg_fifo_free(void);
int push_pdu_to_msglist_bulk_out(usb_bst_virsual_msg_t *pdu);
int pop_pdu_from_msglist_bulk_out(usb_bst_virsual_msg_t *pdu);
int push_pdu_to_msglist_bulk_in(usb_bst_virsual_msg_t *pdu);
int pop_pdu_from_msglist_bulk_in(usb_bst_virsual_msg_t *pdu);
int pdu_len_msglist_bulk_out(void);
int pdu_len_msglist_bulk_in(void);

#endif
