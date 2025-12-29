// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/bitmap.h>

#include "usb_bst_virt_device.h"

int bulk_in_trb_submit(struct usb_virtual_device *vdev,
		       struct urb_priv_entry *entry);

struct reserve_mem_info {
	void __iomem *usb_virt_base;
	dma_addr_t usb_dma_base;
	size_t usb_virt_size;
	size_t block_size;
	unsigned long total_blocks;
	unsigned long *bitmap;
	spinlock_t usb_virt_lock;
};

static struct reserve_mem_info res_mem;

static int reserve_mem_init(int block_size)
{
/* WARNNING: 16M DDR has reserved for USB data buffer;
	* dts: usb_virt@80e000000 {reg = <0x8 0x0e000000 0x0 0x1000000>;}
	* first 8M for bulk out(set by R5), last 8M for bulk in(set by A78)
	*/
#define VIRT_USB_BULK_IN_OFFSET 0x800000
	struct device_node *np;
	struct resource res;
	unsigned long memmap_offset = VIRT_USB_BULK_IN_OFFSET;

	res_mem.block_size = block_size;
	res_mem.usb_virt_size = 0;
	spin_lock_init(&res_mem.usb_virt_lock);

	np = of_find_node_by_name(NULL, "usb_virt");
	if (!np) {
		pr_err("Could not find usb_virt node in device tree\n");
		return -ENODEV;
	}

	if (of_address_to_resource(np, 0, &res)) {
		pr_err("Could not get usb_virt resource\n");
		return -ENODEV;
	}
	//memmap_offset = resource_size(&res)/2;
	res_mem.usb_virt_size = resource_size(&res) - memmap_offset;
	res_mem.usb_dma_base = res.start + memmap_offset;
	res_mem.usb_virt_base =
	    memremap(res.start + memmap_offset, res_mem.usb_virt_size,
		     MEMREMAP_WB);
	if (!res_mem.usb_virt_base) {
		pr_err("Failed to memremap usb_virt memory\n");
		return -ENOMEM;
	}

	res_mem.total_blocks = res_mem.usb_virt_size / res_mem.block_size;
	res_mem.bitmap = kcalloc(BITS_TO_LONGS(res_mem.total_blocks), sizeof(unsigned long), GFP_KERNEL);
	if (!res_mem.bitmap) {
		pr_err("Failed to allocate bitmap\n");
		memunmap(res_mem.usb_virt_base);
		return -ENOMEM;
	}

	pr_debug("Mapped usb_virt memory: %p with size: %zu\n",
		 res_mem.usb_virt_base, res_mem.usb_virt_size);

	return 0;
}

static void reserve_mem_exit(void)
{
	kfree(res_mem.bitmap);

	if (res_mem.usb_virt_base)
		memunmap(res_mem.usb_virt_base);
}

static void *usb_alloc_buf(size_t size, dma_addr_t *dma)
{
	unsigned long flags;
	void *buf = NULL;
	unsigned long blocks_needed;
	unsigned long start_block;

	if (!res_mem.bitmap || size > res_mem.usb_virt_size) {
		pr_err
		    ("Requested size exceeds usb_virt memory size or bitmap null\n");
		return NULL;
	}

	blocks_needed = (size + res_mem.block_size - 1) / res_mem.block_size;

	spin_lock_irqsave(&res_mem.usb_virt_lock, flags);
	start_block =
	    bitmap_find_next_zero_area(res_mem.bitmap, res_mem.total_blocks, 0,
				       blocks_needed, 0);
	if (start_block < res_mem.total_blocks) {
		bitmap_set(res_mem.bitmap, start_block, blocks_needed);
		buf =
		    (void *)(res_mem.usb_virt_base +
			     start_block * res_mem.block_size);
		*dma =
		    (dma_addr_t) (start_block * res_mem.block_size +
				  res_mem.usb_dma_base);
	} else {
		pr_err("Not enough contiguous usb_virt memory available\n");
	}
	spin_unlock_irqrestore(&res_mem.usb_virt_lock, flags);

	return buf;
}

static void usb_free_buf(void *buf)
{
	unsigned long flags;
	unsigned long block_index;

	if (!buf)
		return;

	block_index =
	    ((unsigned long)buf -
	     (unsigned long)res_mem.usb_virt_base) / res_mem.block_size;

	spin_lock_irqsave(&res_mem.usb_virt_lock, flags);
	bitmap_clear(res_mem.bitmap, block_index, 1);
	spin_unlock_irqrestore(&res_mem.usb_virt_lock, flags);

}

int urb_pool_init(struct urbs_pool *pool, u32 urb_max_num, u32 urb_buf_size)
{
	int i;
	struct buf_priv_entry *mem = NULL;

	urb_max_num = urb_max_num-1;//for pool->mem_buf
	pool->urbs = kmalloc_array(urb_max_num, sizeof(struct urb_priv_entry), GFP_KERNEL);
	pool->urb_max_num = urb_max_num;
	pool->urb_buf_size = urb_buf_size;
	reserve_mem_init(urb_buf_size);
	for (i = 0; i < urb_max_num; i++) {
		struct urb_priv_entry *entry = &pool->urbs[i];

		usb_init_urb(&entry->urb);
		entry->buf = usb_alloc_buf(urb_buf_size, &entry->urb.transfer_dma);	//kmalloc(urb_buf_size, GFP_KERNEL);
		entry->seqnum = 0;
		if (entry->buf)
			atomic_set(&entry->status, URB_FREE);
	}
	mem = &pool->mem_buf;
	mem->buf = usb_alloc_buf(urb_buf_size, &mem->transfer_dma);
	mem->seqnum = 0;
	mem->actual_length = 0;
	if (mem->buf)
		atomic_set(&mem->status, URB_FREE);

	return 0;
}

static int bluk_in_buf_to_pdu(struct buf_priv_entry *entry,
			      usb_bst_virsual_msg_t *pdu)
{
	phys_addr_t phys_addr = entry->transfer_dma;

	phys_addr = phys_to_bus(phys_addr);
	pdu->high_addr = phys_addr >> 32;
	pdu->low_addr = (u32) (phys_addr & 0xffffffff);
	pdu->len = entry->actual_length;
	pdu->seqnum = entry->seqnum;
	return 0;
}

static int bluk_in_urb_to_pdu(struct urb_priv_entry *entry,
			      usb_bst_virsual_msg_t *pdu)
{
	struct urb *urb = &entry->urb;

	phys_addr_t phys_addr = urb->transfer_dma;

	phys_addr = phys_to_bus(phys_addr);
	memset(pdu, 0, sizeof(usb_bst_virsual_msg_t));
	pdu->command = USB_VIRT_EP_SUBMIT | USB_VIRT_CMD_DIR_IN;
	pdu->high_addr = phys_addr >> 32;
	pdu->low_addr = (u32) (phys_addr & 0xffffffff);
	pdu->len = urb->actual_length;
	pdu->seqnum = entry->seqnum;
	return 0;
}

int send_bluk_in_buf(struct usb_virtual_device *vdev, char *buf, u32 len, usb_bst_virsual_msg_t *pdu)
{
	struct buf_priv_entry *entry = &vdev->pool.mem_buf;

	if (atomic_read(&entry->status) != URB_FREE)
		return -1;

	atomic_set(&entry->status, URB_SUBMITTED);
	entry->seqnum = atomic_inc_return(&vdev->seqnum);
	entry->actual_length = len;
	memcpy(entry->buf, buf, len);

	bluk_in_buf_to_pdu(entry, pdu);

	//make sure access buf ok
	smp_wmb();
	__aarch64_clean_dcache_range(entry->buf,
						entry->buf +
						entry->actual_length);
	push_pdu_to_msglist_bulk_in(pdu);
	atomic_set(&entry->status, URB_WAITING_ACK);
	wake_up_interruptible(&vdev->tx_waitqueue);
	entry->wait_ack_jiffies = jiffies;
	return 0;
}

void virtual_usb_read_bulk_callback(struct urb *urb)
{
	struct usb_virtual_device *vdev;
	struct urb_priv_entry *entry;

	vdev = urb->context;
	pr_debug("%s - some read bulk callback %d\n", __func__, urb->status);

	if (atomic_read(&vdev->device_state) != VIRT_DEVICE_OK) {
		pr_debug("%s - device not in a valid state\n", __func__);
		return;
	}
	/* sync/async unlink faults aren't errors */
	entry = container_of(urb, struct urb_priv_entry, urb);

	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		      urb->status == -ECONNRESET ||
		      urb->status == -EPROTO || urb->status == -ESHUTDOWN)) {
			dev_err(&vdev->interface->dev,
				"%s - nonzero read bulk status received: %d\n",
				__func__, urb->status);
			atomic_set(&entry->status, URB_FREE);
			bulk_in_trb_submit(vdev, entry);
		}
		vdev->errors = urb->status;
	} else {
		if (urb->actual_length > 0) {
			usb_bst_virsual_msg_t pdu = { 0 };

			bluk_in_urb_to_pdu(entry, &pdu);
			//make sure access buf ok
			smp_wmb();
			__aarch64_clean_dcache_range(entry->buf,
						     entry->buf +
						     urb->actual_length);
			push_pdu_to_msglist_bulk_in(&pdu);
			atomic_set(&entry->status, URB_WAITING_ACK);
			wake_up_interruptible(&vdev->tx_waitqueue);
			entry->wait_ack_jiffies = jiffies;
		} else {
			dev_err(&vdev->interface->dev,
				"%s - read bulk received zero len : %d\n",
				__func__, urb->status);
			atomic_set(&entry->status, URB_FREE);
			bulk_in_trb_submit(vdev, entry);
		}
	}

}

int bulk_in_trb_submit(struct usb_virtual_device *vdev,
		       struct urb_priv_entry *entry)
{
	struct usb_device *udev = vdev->udev;
	int status = 0;

	if (atomic_read(&vdev->device_state) != VIRT_DEVICE_OK) {
		pr_debug("Device not in a valid state for submitting URBs\n");
		return -EIO;
	}
	if (atomic_read(&entry->status) == URB_FREE) {
		atomic_set(&entry->status, URB_SUBMITTED);
		/* initialize the urb properly */
		usb_fill_bulk_urb(&entry->urb, udev,
				usb_rcvbulkpipe(udev, vdev->bulk_in->bEndpointAddress),
				entry->buf, vdev->pool.urb_buf_size,
				virtual_usb_read_bulk_callback, vdev);
		pr_debug("urb %p urb->dma_addr %llx, bEndpointAddress %x urb->pipe %x usb_urb_dir_in(urb) %d\n",
				&entry->urb, (u64) entry->urb.transfer_dma,
				vdev->bulk_in->bEndpointAddress, entry->urb.pipe,
				usb_urb_dir_in(&entry->urb));
		entry->urb.transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		entry->seqnum = atomic_inc_return(&vdev->seqnum);
		status = usb_submit_urb(&entry->urb, GFP_ATOMIC);
		if (status) {
			entry->seqnum = atomic_dec_return(&vdev->seqnum);
			dev_err(&udev->dev, "failed to submit in-urb: %d\n",
				status);
			atomic_set(&entry->status, URB_FREE);
		}
	}
	return status;
}

int urb_pool_submit_urb(struct usb_virtual_device *vdev)
{
	struct urbs_pool *pool = &vdev->pool;
	int i = 0;

	for (i = 0; i < pool->urb_max_num; i++) {
		struct urb_priv_entry *entry = &pool->urbs[i];

		bulk_in_trb_submit(vdev, entry);
	}
	return 0;
}

void usb_recv_ep_ack(struct usb_virtual_device *vdev, usb_bst_virsual_msg_t *pdu)
{
	struct urbs_pool *pool = &vdev->pool;
	int i;

	for (i = 0; i < pool->urb_max_num; i++) {
		struct urb_priv_entry *entry = &pool->urbs[i];

		if (entry->seqnum == pdu->seqnum
		    && atomic_read(&entry->status) == URB_WAITING_ACK) {
			atomic_set(&entry->status, URB_FREE);
			bulk_in_trb_submit(vdev, entry);
			break;
		}
	}

}
void usb_recv_device_ack(struct usb_virtual_device *vdev, usb_bst_virsual_msg_t *pdu)
{
	struct urbs_pool *pool = &vdev->pool;
	struct buf_priv_entry *entry = &pool->mem_buf;

	if (entry->seqnum == pdu->seqnum
		&& atomic_read(&entry->status) == URB_WAITING_ACK) {
		atomic_set(&entry->status, URB_FREE);
	}

}


void urb_pool_unlink(struct urbs_pool *pool)
{
	int i;

	for (i = 0; i < pool->urb_max_num; i++) {
		struct urb_priv_entry *entry = &pool->urbs[i];

		if (atomic_read(&entry->status) == URB_SUBMITTED)
			usb_kill_urb(&entry->urb);
		else if (atomic_read(&entry->status) == URB_WAITING_ACK)
			atomic_set(&entry->status, URB_FREE);
	}

}

void urb_pool_free(struct urbs_pool *pool)
{
	int i;

	for (i = 0; i < pool->urb_max_num; i++) {
		struct urb_priv_entry *entry = &pool->urbs[i];

		if (entry->buf) {
			usb_free_buf(entry->buf);
			entry->buf = NULL;
		}
	}
	if (pool->mem_buf.buf) {
		usb_free_buf(pool->mem_buf.buf);
		pool->mem_buf.buf = NULL;
	}
	reserve_mem_exit();
	pool->urb_max_num = 0;
	kfree(pool->urbs);

}

void tx_timer_callback(struct timer_list *t)
{
	struct usb_virtual_device *vdev = from_timer(vdev, t, tx_timer);
	struct urbs_pool *pool = &vdev->pool;
	int i = 0;

	for (i = 0; i < pool->urb_max_num; i++) {
		struct urb_priv_entry *entry = &pool->urbs[i];

		if (atomic_read(&entry->status) == URB_WAITING_ACK) {
			if (time_is_before_jiffies(entry->wait_ack_jiffies +
				URB_BULK_IN_ACK_TIMEOUT * HZ)) {
				pr_debug("timeout urb %p urb->dma_addr %llx,urb->actual_length %x seqnum %lx  timeout %ld jiffies %ld\n",
					&entry->urb, (u64) entry->urb.transfer_dma,
					entry->urb.actual_length, entry->seqnum,
					entry->wait_ack_jiffies + URB_BULK_IN_ACK_TIMEOUT * HZ, jiffies);
				atomic_set(&entry->status, URB_FREE);
				bulk_in_trb_submit(vdev, entry);
			}
		}
	}
	if (atomic_read(&pool->mem_buf.status) == URB_WAITING_ACK) {
		if (time_is_before_jiffies(pool->mem_buf.wait_ack_jiffies +
			URB_BULK_IN_ACK_TIMEOUT * HZ)) {
			atomic_set(&pool->mem_buf.status, URB_FREE);
			pool->mem_buf.actual_length = 0;
		}
	}
	mod_timer(&vdev->tx_timer, jiffies + HZ);
}

void usb_send_pid_device_msg(struct usb_virtual_device *vdev, uint16_t cmd, uint32_t pid)
{
	usb_bst_virsual_msg_t pdu = { 0 };

	pdu.core_id = pid;
	pdu.command = cmd << 16 | USB_VIRT_DEVICE_SUBMIT | USB_VIRT_CMD_DIR_IN;
	push_pdu_to_msglist_bulk_in(&pdu);
	wake_up_interruptible(&vdev->tx_waitqueue);
}
void usb_send_device_msg(struct usb_virtual_device *vdev, uint16_t cmd)
{
	usb_send_pid_device_msg(vdev, cmd, 0);
}

void usb_send_buf_pid_device_msg(struct usb_virtual_device *vdev, uint16_t cmd, uint32_t pid, char *buf, u32 len)
{
	usb_bst_virsual_msg_t pdu = { 0 };

	pdu.core_id = pid;
	pdu.command = cmd << 16 | USB_VIRT_DEVICE_SUBMIT | USB_VIRT_CMD_DIR_IN;
	if (send_bluk_in_buf(vdev, buf, len, &pdu))
		dev_err(&vdev->interface->dev, "send device msg err\n");
}

void usb_send_buf_device_msg(struct usb_virtual_device *vdev, uint16_t cmd, char *buf, u32 len)
{
	usb_send_buf_pid_device_msg(vdev, cmd, 0, buf, len);
}
