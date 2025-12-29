// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#include <asm/byteorder.h>
#include <linux/kthread.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/scatterlist.h>
#include <linux/cacheflush.h>
#include "usb_bst_virt_device.h"
#include "usb_bst_list.h"
#include <linux/jiffies.h>
#include <linux/wait.h>
#include <linux/sched.h>

void virtual_usb_write_bulk_callback(struct urb *urb)
{

	struct bulk_out_node *node = urb->context;
	usb_bst_virsual_msg_t *pdu = &node->pdu;
	struct usb_virtual_device *vdev = node->vdev;

	pr_debug("%s %llx\n", __func__, (u64) (urb->transfer_buffer));
	//usb_free_coherent(urb->dev, urb->transfer_buffer_length,
	//                        urb->transfer_buffer, urb->transfer_dma);

	usb_unanchor_urb(urb);
	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		      urb->status == -ECONNRESET || urb->status == -ESHUTDOWN))
			dev_err(&urb->dev->dev,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);
	}

	pdu->command &= (~USB_VIRT_CMD_MASK);
	pdu->command |= USB_VIRT_EP_ACK;
	pdu->command |= USB_VIRT_CMD_DIR_OUT;
	pdu->core_id = 0;
	push_pdu_to_msglist_bulk_in(pdu);
	wake_up_interruptible(&vdev->tx_waitqueue);

	add_to_bulk_out_list(node);
}

int usb_bst_reset_device(struct usb_device *udev)
{
	int ret = 0;

	dev_dbg(&udev->dev, "device reset");
	ret = usb_lock_device_for_reset(udev, NULL);
	if (ret < 0) {
		dev_err(&udev->dev, "lock for reset\n");
		return ret;
	}
	/* try to reset the device */
	ret = usb_reset_device(udev);
	usb_unlock_device(udev);
	return ret;
}

static void usb_recv_cmd_submit(struct usb_virtual_device *vdev,
				usb_bst_virsual_msg_t *pdu)
{
	unsigned short int device_cmd =
	    ((pdu->command & USB_VIRT_DEVICE_CMD_MASK) >> 16);
	int ret = 0;

	switch (device_cmd) {
	case USB_VIRT_DEVICE_CMD_RESET:
		pr_debug("%s %d recv reset cmd\n", __func__, __LINE__);
		urb_pool_unlink(&vdev->pool);
		usb_kill_anchored_urbs(&vdev->submitted);
		cleanup_bulk_out_list(vdev);
		usb_local_send_adb_err(vdev);
		//ret = usb_bst_reset_device(vdev->udev);
		break;
	case USB_VIRT_DEVICE_CMD_DISCONNECT:
		//ret = usb_port_disable(vdev->udev);
		break;
	case USB_VIRT_DEVICE_CMD_CONNECT:
		//ret = usb_port_enable(vdev->udev);
		break;
	case USB_VIRT_DEVICE_CMD_ACTIVE_REPLAY:
		set_system_id(pdu->seqnum);
		wake_up_interruptible(&vdev->replay_wq);
		break;
	default:
		dev_err(&vdev->udev->dev,
			"%s unknown pdu cmd %x\n", __func__, pdu->command);
		break;
	}
	if (ret)
		dev_err(&vdev->udev->dev, "%s err %d\n", __func__, ret);
}

static void usb_local_bulk_out_submit_callback(struct urb *urb)
{

	pr_debug("%s %llx\n", __func__, (u64) (urb->transfer_buffer));
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			  urb->transfer_buffer, urb->transfer_dma);

	usb_unanchor_urb(urb);
	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		      urb->status == -ECONNRESET || urb->status == -ESHUTDOWN))
			dev_err(&urb->dev->dev,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);
	}
}

int usb_local_bulk_out_submit(struct usb_virtual_device *vdev, char *data,
			      int len)
{

	struct urb *urb = NULL;
	char *buf = NULL;
	int ret = 0;
	/* create a urb, and a buffer for it, and copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		ret = -ENOMEM;
		goto error;
	}

	buf = usb_alloc_coherent(vdev->udev, len, GFP_KERNEL,
				 &urb->transfer_dma);
	if (!buf) {
		ret = -ENOMEM;
		goto error;
	}
	memcpy(buf, data, len);
	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, vdev->udev,
			  usb_sndbulkpipe(vdev->udev,
					  vdev->bulk_out->bEndpointAddress),
			  buf, len, usb_local_bulk_out_submit_callback, vdev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP | URB_ZERO_PACKET;
	usb_anchor_urb(urb, &vdev->submitted);

	/* send the data out the bulk port */
	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret) {
		dev_err(&vdev->udev->dev, "failed to submit out-urb: %d\n",
			ret);
		goto error_unanchor;
	}
	usb_free_urb(urb);
	return 0;

error_unanchor:
	usb_unanchor_urb(urb);
error:
	if (urb) {
		usb_free_coherent(vdev->udev, len, buf, urb->transfer_dma);
		usb_free_urb(urb);
	}
	return ret;
}

static void usb_recv_ep_submit(struct usb_virtual_device *vdev,
			       usb_bst_virsual_msg_t *pdu)
{
	struct usb_device *udev = vdev->udev;
	struct bulk_out_node *node = NULL;
	struct urb *urb;
	int ret;

	void *buf1 = NULL;

	node = get_bulk_out_node(vdev);
	if (!node) {
		dev_err(&udev->dev, "get bulk node fail\n");
		goto err_malloc;
	}

	ret = init_bulk_out_node(node, pdu);
	if (ret) {
		dev_err(&udev->dev, "memremap err\n");
		goto err_iomap;
	}

	urb = &node->urb;

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, udev,
			  usb_sndbulkpipe(udev,
					  vdev->bulk_out->bEndpointAddress),
			  node->buf, node->buf_len,
			  virtual_usb_write_bulk_callback, node);
	pr_debug("buf1 %llx  buf_len %ld urb %p , bEndpointAddress %x urb->pipe %x\n",
	     (u64) buf1, node->buf_len, urb, vdev->bulk_out->bEndpointAddress,
	     urb->pipe);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP | URB_ZERO_PACKET;
	usb_anchor_urb(urb, &vdev->submitted);

	/* send the data out the bulk port */
	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret) {
		dev_err(&udev->dev, "failed to submit out-urb: %d\n", ret);
		goto err;
	}
	return;

err:
	usb_unanchor_urb(urb);
err_iomap:
	free_bulk_out_node(node);
err_malloc:
	return;
}

void correct_endian_basic(usb_bst_virsual_msg_t *base, int send)
{
}

/* recv a pdu */
static int usb_rx_pdu(struct usb_virtual_device *vdev)
{
	int ret;
	usb_bst_virsual_msg_t pdu;
	/* receive a pdu header */
	ret = pop_pdu_from_msglist_bulk_out(&pdu);
	if (ret <= 0)
		return -1;

	correct_endian_basic(&pdu, 0);

	pr_debug("%s BASE: core_id %x command %x   high_addr %x low_addr %x offset %x len %x seqnum %x\n",
	     __func__, pdu.core_id, pdu.command, pdu.high_addr, pdu.low_addr, pdu.offset,
	     pdu.len, pdu.seqnum);

	switch (pdu.command & 0xff) {
	case USB_VIRT_EP_SUBMIT | USB_VIRT_CMD_DIR_OUT:
		usb_recv_ep_submit(vdev, &pdu);
		break;

	case USB_VIRT_EP_ACK | USB_VIRT_CMD_DIR_IN:
		//up(&vdev->tx_sema);
		usb_recv_ep_ack(vdev, &pdu);
		break;

	case USB_VIRT_DEVICE_SUBMIT | USB_VIRT_CMD_DIR_OUT:
		usb_recv_cmd_submit(vdev, &pdu);
		break;

	case USB_VIRT_DEVICE_ACK | USB_VIRT_CMD_DIR_OUT:
		usb_recv_device_ack(vdev, &pdu);
		break;

	default:
		/* NOTREACHED */
		dev_err(&vdev->udev->dev, "unknown rx pdu cmd %x\n",
			pdu.command);
		break;
	}
	return 0;
}

/* send a pdu */
static int usb_tx_pdu(struct usb_virtual_device *vdev)
{
	int ret;
	usb_bst_virsual_msg_t pdu;
	/* receive a pdu header */
	ret = pop_pdu_from_msglist_bulk_in(&pdu);
	if (ret <= 0)
		return -1;

	pr_debug("%s BASE: core_id %x command %x   high_addr %x low_addr %x offset %x len %x seqnum %x\n",
	     __func__, pdu.core_id, pdu.command, pdu.high_addr, pdu.low_addr, pdu.offset,
	     pdu.len, pdu.seqnum);
	switch (pdu.command & 0xff) {
	case USB_VIRT_EP_SUBMIT | USB_VIRT_CMD_DIR_IN:
	case USB_VIRT_DEVICE_SUBMIT | USB_VIRT_CMD_DIR_IN:
		correct_endian_basic(&pdu, 1);
		usb_msg_send(&pdu);
		break;
	case USB_VIRT_EP_ACK | USB_VIRT_CMD_DIR_OUT:
		correct_endian_basic(&pdu, 1);
		usb_msg_send(&pdu);
		break;
	default:
		/* NOTREACHED */
		dev_err(&vdev->udev->dev, "unknown tx pdu cmd %x\n",
			pdu.command);
		break;
	}
	return 0;
}

int virt_usb_rx_loop(void *data)
{
	struct usb_virtual_device *vdev = data;

	while (!kthread_should_stop()) {
		if (pdu_len_msglist_bulk_out())
			usb_rx_pdu(vdev);
		else
			msleep(20);
	}

	return 0;
}

int virt_usb_tx_loop(void *data)
{
	struct usb_virtual_device *vdev = data;

	while (!kthread_should_stop()) {
		wait_event_interruptible(vdev->tx_waitqueue,
					 pdu_len_msglist_bulk_in() > 0
					 || kthread_should_stop());
		usb_tx_pdu(vdev);
	}

	return 0;
}
