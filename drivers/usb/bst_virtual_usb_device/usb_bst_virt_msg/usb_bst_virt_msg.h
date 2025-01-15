/* SPDX-License-Identifier: GPL-2.0 OR Apache 2.0
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

#ifndef USB_BST_VIRT_MSG_H
#define USB_BST_VIRT_MSG_H

#include  "usb_datatype.h"

typedef int (*msg_sub_callback_t)(void *);
typedef int (*msg_send_callback_t)(void *);

// subscribe broadcast.
int usb_msg_sub(msg_sub_callback_t cb);
// unsubscribe broadcast.
int usb_msg_unsub(void);

int usb_msg_send(usb_bst_virsual_msg_t *pdu);
int get_system_pid(void);
#endif
