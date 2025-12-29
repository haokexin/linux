// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/moduleparam.h>

#include "usb_bst_list.h"
#include "usb_bst_virt_device.h"

static int urb_max_number = URB_MAX_NUMBER;
static int urb_buffer_size = URB_BUFFER_SIZE;

module_param(urb_max_number, int, 0644);
MODULE_PARM_DESC(urb_max_number, "Maximum number of bulk in URBs");

module_param(urb_buffer_size, int, 0644);
MODULE_PARM_DESC(urb_buffer_size, "Buffer size for bulk in URB");

/* Define these values to match your devices */
#define USB_VIRT_VENDOR_ID	0x3415
#define USB_VIRT_PRODUCT_ID	0x0308

#define USB_VIRT_ADB_CLASS	0xFF
#define USB_VIRT_ADB_SUB_CLASS	0x42
#define USB_VIRT_ADB_PROTO	0x01

/* table of devices that work with this driver */
static const struct usb_device_id virtual_usb_table[] = {
	{ USB_DEVICE(USB_VIRT_VENDOR_ID, USB_VIRT_PRODUCT_ID) },
	{ USB_INTERFACE_INFO(USB_VIRT_ADB_CLASS, USB_VIRT_ADB_SUB_CLASS, USB_VIRT_ADB_PROTO) },
	{ }			/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, virtual_usb_table);

/**
 * virtual_device_alloc - allocate a new usb_virtual_device struct
 * @udev: usb_device of a new device
 *
 * Allocates and initializes a new usb_virtual_device struct.
 */
static struct usb_virtual_device *virtual_device_alloc(struct usb_interface
						       *interface)
{
	struct usb_virtual_device *vdev;

	/* yes, it's a new device */
	vdev = kzalloc(sizeof(struct usb_virtual_device), GFP_KERNEL);
	if (!vdev)
		return NULL;

	vdev->udev = usb_get_dev(interface_to_usbdev(interface));
	vdev->interface = usb_get_intf(interface);

	kref_init(&vdev->kref);
	mutex_init(&vdev->io_mutex);
	init_waitqueue_head(&vdev->replay_wq);
	//spin_lock_init(&vdev->err_lock);
	init_usb_anchor(&vdev->submitted);

	//init_waitqueue_head(&vdev->tx_waitq);

	//vdev->ud.eh_ops.shutdown = stub_shutdown_connection;
	//vdev->ud.eh_ops.reset    = stub_device_reset;
	//vdev->ud.eh_ops.unusable = stub_device_unusable;

	//usbip_start_eh(&vdev->ud);

	spin_lock_init(&vdev->bulk_out_lock);
	INIT_LIST_HEAD(&vdev->bulk_out_list);

	dev_dbg(&vdev->udev->dev, "register new device\n");

	return vdev;
}

static int usb_set_bulk_eps(struct usb_virtual_device *vdev,
			    struct usb_interface *interface)
{
	struct usb_endpoint_descriptor *bulk_in, *bulk_out;
	int ret;

	/* set up the endpoint information */
	/* use only the first bulk-in and bulk-out endpoints */
	ret = usb_find_common_endpoints(interface->cur_altsetting,
					&bulk_in, &bulk_out, NULL, NULL);
	if (ret) {
		dev_err(&interface->dev,
			"Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}
	vdev->bulk_in = bulk_in;
	vdev->bulk_out = bulk_out;

	return 0;

error:
	/* this frees allocated memory */
	return ret;

}

void virtual_usb_delete(struct kref *kref)
{
	struct usb_virtual_device *vdev = to_virtual_usb_dev(kref);

	usb_put_intf(vdev->interface);
	urb_pool_free(&vdev->pool);
	msg_fifo_free();
	usb_put_dev(vdev->udev);
	kfree(vdev);
}

static int device_driver_dummy_hcd(struct usb_interface *interface)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	const char		*driver_name = NULL;

	if (udev->bus->controller->driver)
		driver_name = udev->bus->controller->driver->name;
	else
		driver_name = udev->bus->sysdev->driver->name;
	if (driver_name && (strcmp(driver_name, "dummy_hcd") != 0)) {
		dev_err(&interface->dev, "not dummy device\n");
		return 0;
	}
	return 1;
}

static int virtual_usb_probe(struct usb_interface *interface,
			     const struct usb_device_id *id)
{
	struct usb_virtual_device *vdev;
	struct task_struct *dev_usb_rx = NULL;
	struct task_struct *dev_usb_tx = NULL;
	int ret;
	msg_sub_callback_t cb =
	    (msg_sub_callback_t) push_pdu_to_msglist_bulk_out;

	if (!device_driver_dummy_hcd(interface))
		return -1;
	/* allocate memory for our device state and initialize it */
	vdev = virtual_device_alloc(interface);
	if (!vdev) {
		dev_err(&interface->dev, "Could not alloc device\n");
		return -1;
	}
	ret = msg_fifo_malloc();
	if (ret)
		goto error;

	ret = usb_set_bulk_eps(vdev, interface);
	if (ret)
		goto error;

	urb_pool_init(&vdev->pool, urb_max_number, urb_buffer_size);
	urb_pool_submit_urb(vdev);
	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, vdev);

	/* we can register the device now, as it is ready */
	ret = usb_register_virt_dev(interface);
	if (ret) {
		/* something prevented us from registering this driver */
		dev_err(&interface->dev,
			"Not able to get a minor for this device.\n");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	/* let the user know what node this device is now attached to */
	dev_info(&interface->dev,
		 "USB virtual_usb device now attached to USBvirtual_usb-%d",
		 interface->minor);

	dev_usb_rx = kthread_create(virt_usb_rx_loop, vdev, "dev_usb_rx");
	if (IS_ERR(dev_usb_rx))
		goto error;

	dev_usb_tx = kthread_create(virt_usb_tx_loop, vdev, "dev_usb_tx");
	if (IS_ERR(dev_usb_tx)) {
		kthread_stop(dev_usb_rx);
		goto error;
	}
	timer_setup(&vdev->tx_timer, tx_timer_callback, 0);
	init_waitqueue_head(&vdev->tx_waitqueue);
	//sema_init(&vdev->tx_sema,URB_BULK_IN_SEMA_MAX);
	/* get task structs now */
	get_task_struct(dev_usb_rx);
	get_task_struct(dev_usb_tx);

	vdev->dev_usb_rx = dev_usb_rx;
	vdev->dev_usb_tx = dev_usb_tx;

	atomic_set(&vdev->device_state, VIRT_DEVICE_OK);
	atomic_set(&vdev->seqnum, 0);

	usb_msg_sub(cb);

	wake_up_process(vdev->dev_usb_rx);
	wake_up_process(vdev->dev_usb_tx);

	mod_timer(&vdev->tx_timer, jiffies + HZ);
	usb_local_send_adb_cnxn_id(vdev);

	return 0;

error:
	/* this frees allocated memory */
	dev_err(&interface->dev, "probe err ret %d\n", ret);
	kref_put(&vdev->kref, virtual_usb_delete);

	return ret;
}

void virtual_usb_draw_down(struct usb_virtual_device *dev)
{
	int time;

	time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&dev->submitted);
	urb_pool_unlink(&dev->pool);
}

static void virtual_usb_disconnect(struct usb_interface *interface)
{
	struct usb_virtual_device *vdev;
	int minor = interface->minor;

	usb_msg_unsub();
	vdev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);
	atomic_set(&vdev->device_state, VIRT_DEVICE_DISCONNECT);

	kthread_stop(vdev->dev_usb_rx);
	kthread_stop(vdev->dev_usb_tx);

	virtual_usb_draw_down(vdev);
	del_timer_sync(&vdev->tx_timer);
	/* give back our minor */
	usb_unregister_virt_dev(interface);

	cleanup_bulk_out_list(vdev);

	put_task_struct(vdev->dev_usb_rx);
	put_task_struct(vdev->dev_usb_tx);

	/* decrement our usage count */
	kref_put(&vdev->kref, virtual_usb_delete);

	dev_info(&interface->dev, "USB virtual_usb #%d now disconnected",
		 minor);
}

static int virtual_usb_suspend(struct usb_interface *intf, pm_message_t message)
{
	return 0;
}

static int virtual_usb_resume(struct usb_interface *intf)
{
	return 0;
}


static int virtual_usb_resume_reset(struct usb_interface *intf)
{
	struct usb_virtual_device *dev = usb_get_intfdata(intf);

	usb_local_send_adb_err(dev);
	return 0;
}

static int virtual_usb_pre_reset(struct usb_interface *intf)
{
	struct usb_virtual_device *dev = usb_get_intfdata(intf);

	mutex_lock(&dev->io_mutex);
	atomic_set(&dev->device_state, VIRT_DEVICE_RESET);
	virtual_usb_draw_down(dev);

	return 0;
}

static int virtual_usb_post_reset(struct usb_interface *intf)
{
	struct usb_virtual_device *dev = usb_get_intfdata(intf);

	/* we are sure no URBs are active - no locking needed */
	dev->errors = -EPIPE;
	atomic_set(&dev->device_state, VIRT_DEVICE_OK);
	mutex_unlock(&dev->io_mutex);

	return 0;
}

struct usb_driver virtual_usb_driver = {
	.name = "virtual_usb",
	.probe = virtual_usb_probe,
	.disconnect = virtual_usb_disconnect,
	.suspend = virtual_usb_suspend,
	.resume = virtual_usb_resume,
	.reset_resume = virtual_usb_resume_reset,
	.pre_reset = virtual_usb_pre_reset,
	.post_reset = virtual_usb_post_reset,
	.id_table = virtual_usb_table,
	.supports_autosuspend = 1,
};

module_usb_driver(virtual_usb_driver);

MODULE_LICENSE("GPL v2");
