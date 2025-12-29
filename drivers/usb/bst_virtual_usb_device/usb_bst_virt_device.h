/* SPDX-License-Identifier: GPL-2.0 OR Apache 2.0
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

#ifndef USB_BST_VIRT_DEVICE_H
#define USB_BST_VIRT_DEVICE_H

#include <asm/byteorder.h>
#include <linux/kthread.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/scatterlist.h>
#include <linux/timer.h>
#include "usb_bst_virt_msg.h"
#include "usb_bst_pdu_kfifo.h"

#define USB_VIRT_EP_SUBMIT	0x01
#define USB_VIRT_EP_ACK	0x02
#define USB_VIRT_DEVICE_SUBMIT	0x03
#define USB_VIRT_DEVICE_ACK	0x04
#define USB_VIRT_CMD_MASK	0x7F

#define USB_VIRT_CMD_DIR_OUT   0x00
#define USB_VIRT_CMD_DIR_IN   0x80
#define USB_VIRT_CMD_DIR_MASK	0xFF

#define USB_VIRT_DEVICE_CMD_RESET	0x01
#define USB_VIRT_DEVICE_CMD_DISCONNECT	0x02
#define USB_VIRT_DEVICE_CMD_CONNECT	0x03
#define USB_VIRT_DEVICE_CMD_ACTIVE	0x04
#define USB_VIRT_DEVICE_CMD_ACTIVE_REPLAY	0x05
#define USB_VIRT_DEVICE_CMD_SET_SN	0x06
#define USB_VIRT_DEVICE_CMD_RECONNECT	0x07
#define USB_VIRT_DEVICE_CMD_MASK	0xFF0000

/*
 * typedef struct bst_virsual_msg {
 *   u32 core_id;
 *   u32 command;
 *   u32 high_addr;
 *   u32 low_addr;
 *   u32 offset;
 *   u32 len;
 *   u32 seqnum;
 *} usb_bst_virsual_msg_t;
 */

#define URB_MAX_NUMBER 8
#define URB_BUFFER_SIZE 0x100000
#define URB_BULK_IN_SEMA_MAX 5
#define URB_BULK_IN_ACK_TIMEOUT 30

enum urb_status {
	URB_FREE,
	URB_SUBMITTED,
	URB_WAITING_ACK,
	URB_ERROR,
};

enum device_status {
	VIRT_DEVICE_OK,
	VIRT_DEVICE_DISCONNECT,
	VIRT_DEVICE_RESET,
	VIRT_DEVICE_SUSPEND,
	VIRT_URB_ERROR,
};

struct urb_priv_entry {
	unsigned long seqnum;
	unsigned long wait_ack_jiffies;
	struct urb urb;
	char *buf;
	atomic_t status;
};
struct buf_priv_entry {
	unsigned long seqnum;
	unsigned long wait_ack_jiffies;
	char *buf;
	dma_addr_t transfer_dma;
	u32 actual_length;
	atomic_t status;
};

struct urbs_pool {
	struct urb_priv_entry *urbs;
	unsigned int urb_max_num;
	unsigned int urb_buf_size;
	struct buf_priv_entry mem_buf;
};

struct token_bucket {
	unsigned long rate;
	unsigned long capacity;
	unsigned long tokens;
	unsigned long last_update;
	wait_queue_head_t wait_queue;
};

/* Structure to hold all of our device specific stuff */
struct usb_virtual_device {
	struct usb_device *udev;	/* the usb device for this device */
	struct usb_interface *interface;	/* the interface for this device */
	//struct semaphore      limit_sem;              /* limiting the number of writes in progress */
	struct usb_anchor submitted;	/* in case we need to retract our submissions */

	int errors;		/* the last request tanked */
	//bool                  ongoing_read;           /* a read is going on */
	//spinlock_t            err_lock;               /* lock for errors */
	struct kref kref;
	struct mutex io_mutex;	/* synchronize I/O with disconnect */
	atomic_t device_state;
	wait_queue_head_t replay_wq;

	struct usb_endpoint_descriptor *bulk_in;
	struct usb_endpoint_descriptor *bulk_out;

	struct task_struct *dev_usb_rx;
	struct task_struct *dev_usb_tx;
	//wait_queue_head_t rx_waitqueue;
	wait_queue_head_t tx_waitqueue;
	// struct token_bucket tx_tb;
	struct timer_list tx_timer;
	//struct semaphore tx_sema;

	struct list_head bulk_out_list;
	spinlock_t bulk_out_lock;

	struct urbs_pool pool;
	atomic_t seqnum;	//for pdu seqnum
};
#define to_virtual_usb_dev(d) container_of(d, struct usb_virtual_device, kref)

int urb_pool_init(struct urbs_pool *pool, u32 urb_max_num, u32 urb_buf_size);
void urb_pool_free(struct urbs_pool *pool);
//void init_token_bucket(struct token_bucket *tb, unsigned long rate, unsigned long capacity);

void urb_pool_unlink(struct urbs_pool *pool);
int virt_usb_rx_loop(void *data);
int virt_usb_tx_loop(void *data);
int urb_pool_submit_urb(struct usb_virtual_device *vdev);
void usb_recv_ep_ack(struct usb_virtual_device *vdev, usb_bst_virsual_msg_t *pdu);
void usb_recv_device_ack(struct usb_virtual_device *vdev, usb_bst_virsual_msg_t *pdu);

void correct_endian_basic(usb_bst_virsual_msg_t *base, int send);
int bulk_in_trb_submit(struct usb_virtual_device *vdev,
		       struct urb_priv_entry *entry);
void virtual_usb_write_bulk_callback(struct urb *urb);
void tx_timer_callback(struct timer_list *t);
void usb_send_device_msg(struct usb_virtual_device *vdev, uint16_t cmd);
void usb_send_pid_device_msg(struct usb_virtual_device *vdev, uint16_t cmd, uint32_t pid);
void usb_send_buf_device_msg(struct usb_virtual_device *vdev, uint16_t cmd, char *buf, u32 len);

int usb_local_bulk_out_submit(struct usb_virtual_device *vdev, char *data,
			      int len);
void usb_local_send_adb_cnxn_id(struct usb_virtual_device *vdev);
void usb_local_send_adb_err(struct usb_virtual_device *vdev);
void set_system_id(int id);

#define bus_to_phys(baddr)                                                     \
((((phys_addr_t)(baddr)&0x80000000ULL) << 4) | \
((phys_addr_t)(baddr)&0x7FFFFFFFULL))

#define phys_to_bus(paddr)                                                     \
((((phys_addr_t)(paddr)&0x800000000ULL) >> 4) | \
((phys_addr_t)(paddr)&0x7FFFFFFFULL))


int usb_register_virt_dev(struct usb_interface *intf);
void usb_unregister_virt_dev(struct usb_interface *intf);

void virtual_usb_delete(struct kref *kref);
void virtual_usb_draw_down(struct usb_virtual_device *dev);

void __aarch64_clean_dcache_range(const void *base, const void *end);
void __aarch64_inval_dcache_range(const void *base, const void *end);
#endif
