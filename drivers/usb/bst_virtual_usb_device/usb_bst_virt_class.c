// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#include <linux/cacheflush.h>
#include <asm/tlbflush.h>
#include <linux/dma-map-ops.h>
#include "usb_bst_virt_device.h"

static int kill_process_by_name(const char *name)
{
	struct task_struct *task;

	read_lock(&tasklist_lock);
	for_each_process(task) {
		if (strcmp(task->comm, name) == 0) {
			get_task_struct(task);
			read_unlock(&tasklist_lock);

			send_sig(SIGKILL, task, 0);
			pr_debug("Killed process: %s (PID: %d)\n", name, task->pid);

			put_task_struct(task);
			return 0;
		}
	}
	read_unlock(&tasklist_lock);
	return -1;
}


extern struct usb_driver virtual_usb_driver;
int g_system_id;
/* Get a minor range for your devices from the usb maintainer */
#define USB_VIRT_MINOR_BASE	192

/* our private defines. if this grows any larger, use your own .h file */
#define MAX_TRANSFER		(PAGE_SIZE - 512)


void set_system_id(int id)
{
	g_system_id = id;
}

static int virtual_usb_open(struct inode *inode, struct file *file)
{
	struct usb_virtual_device *dev;
	struct usb_interface *interface;
	int subminor;
	int ret = 0;

	subminor = iminor(inode);

	interface = usb_find_interface(&virtual_usb_driver, subminor);
	if (!interface) {
		pr_err("%s - error, can't find device for minor %d\n",
		       __func__, subminor);
		ret = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		ret = -ENODEV;
		goto exit;
	}

	ret = usb_autopm_get_interface(interface);
	if (ret)
		goto exit;

	/* increment our usage count for the device */
	kref_get(&dev->kref);

	/* save our object in the file's private structure */
	file->private_data = dev;

exit:
	return ret;
}

static int virtual_usb_release(struct inode *inode, struct file *file)
{
	struct usb_virtual_device *dev;

	dev = file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* allow the device to be autosuspended */
	usb_autopm_put_interface(dev->interface);

	/* decrement the count on our device */
	kref_put(&dev->kref, virtual_usb_delete);
	return 0;
}


static ssize_t virtual_usb_read(struct file *file, char *buffer, size_t count,
				loff_t *ppos)
{
	struct usb_virtual_device *vdev;
	char buf[256] = { 0 };
	int ret = 0;

	vdev = file->private_data;
	mutex_lock(&vdev->io_mutex);
	if (atomic_read(&vdev->device_state) == VIRT_DEVICE_OK) {
		mutex_unlock(&vdev->io_mutex);
		g_system_id = 0;
		usb_send_device_msg(vdev, USB_VIRT_DEVICE_CMD_ACTIVE_REPLAY);
		wait_event_interruptible_timeout(vdev->replay_wq,
						 g_system_id > 0, 2 * HZ);
		switch (g_system_id) {
		case CPU_0:
			strscpy(buf, "current adb system is IVI\n", sizeof(buf));
			break;
		case CPU_4:
			strscpy(buf, "current adb system is ADAS\n", sizeof(buf));
			break;
		case CPUMP2_0:
			strscpy(buf, "current adb system is DB\n", sizeof(buf));
			break;
		default:
			strscpy(buf, "current adb system is UNKNOWN\n", sizeof(buf));
			break;
		}
		ret =
		    simple_read_from_buffer(buffer, count, ppos, buf,
					    strlen(buf));
	} else {
		mutex_unlock(&vdev->io_mutex);
		ret = -EIO;
	}
	return ret;
}

char cnxn[24] = {
	0x43, 0x4E, 0x58, 0x4E, 0x01, 0x00, 0x00, 0x01,
	0x00, 0x00, 0x10, 0x00, 0x09, 0x00, 0x00, 0x00,
	0x47, 0x66, 0x00, 0x00, 0xBC, 0xB1, 0xA7, 0xB1
};

char id[9] = { 0x68, 0x6F, 0x73, 0x74, 0x3A, 0x3A, 0x42, 0x43, 0x54 };


void usb_local_send_adb_err(struct usb_virtual_device *vdev)
{
	kill_process_by_name("adbd");
}

void usb_local_send_adb_cnxn_id(struct usb_virtual_device *vdev)
{
	usb_local_bulk_out_submit(vdev, cnxn, 24);
	usb_local_bulk_out_submit(vdev, id, 9);
}

void __aarch64_inval_dcache_range(const void *base, const void *end)
{
	unsigned int dcache_lsize;
	static unsigned int cache_info;
	const char *address;

	if (!cache_info)
		// CTR_EL0 [3:0] contains log2 of icache line size in words.
		//   CTR_EL0 [19:16] contains log2 of dcache line size in words.
		asm volatile ("mrs\t%0, ctr_el0":"=r" (cache_info));

	dcache_lsize = 4 << ((cache_info >> 16) & 0xF);

	/* Make the start address of the loop cache aligned.  */
	address = (const char *)((__UINTPTR_TYPE__) base
				 & ~(__UINTPTR_TYPE__) (dcache_lsize - 1));

	for (; address < (const char *)end; address += dcache_lsize)
		asm volatile ("dc\tcivac, %0"::"r" (address)
			       : "memory");

	asm volatile ("dsb\tsy":::"memory");
	asm volatile ("isb":::"memory");
}

void __aarch64_clean_dcache_range(const void *base, const void *end)
{
	unsigned int dcache_lsize;
	static unsigned int cache_info;
	const char *address;

	if (!cache_info)
		// CTR_EL0 [3:0] contains log2 of icache line size in words.
		//   CTR_EL0 [19:16] contains log2 of dcache line size in words.
		asm volatile ("mrs\t%0, ctr_el0":"=r" (cache_info));

	dcache_lsize = 4 << ((cache_info >> 16) & 0xF);

	/* Make the start address of the loop cache aligned.  */
	address = (const char *)((__UINTPTR_TYPE__) base
				 & ~(__UINTPTR_TYPE__) (dcache_lsize - 1));

	for (; address < (const char *)end; address += dcache_lsize)
		asm volatile ("dc\tcvac, %0"::"r" (address)
			       : "memory");

	asm volatile ("dsb\tsy":::"memory");
	asm volatile ("isb":::"memory");
}

static ssize_t virtual_usb_write(struct file *file, const char *user_buffer,
				 size_t count, loff_t *ppos)
{
	struct usb_virtual_device *vdev = NULL;
	char cmd[512] = "";
	size_t writesize = min_t(size_t, count, sizeof(cmd));

	vdev = file->private_data;

	/* verify that we actually have some data to write */
	if (count == 0)
		goto exit;

	if (copy_from_user(cmd, user_buffer, sizeof(cmd)))
		goto exit;

	if (!strncmp(cmd, "adb", 3))
		usb_send_device_msg(vdev, USB_VIRT_DEVICE_CMD_ACTIVE);
	else if (!strncmp(cmd, "cnxn", 4))
		usb_local_send_adb_cnxn_id(vdev);
	else if (!strncmp(cmd, "restart", 7))
		usb_local_send_adb_err(vdev);
	else if (!strncmp(cmd, "switch db", 9))
		usb_send_pid_device_msg(vdev, USB_VIRT_DEVICE_CMD_ACTIVE, CPUMP2_0);
	else if (!strncmp(cmd, "switch ivi", 10))
		usb_send_pid_device_msg(vdev, USB_VIRT_DEVICE_CMD_ACTIVE, CPU_0);
	else if (!strncmp(cmd, "switch adas", 11))
		usb_send_pid_device_msg(vdev, USB_VIRT_DEVICE_CMD_ACTIVE, CPU_4);
	else if (!strncmp(cmd, "sn ", 3)) {
		if (writesize > 4)
			usb_send_buf_device_msg(vdev, USB_VIRT_DEVICE_CMD_SET_SN, cmd + 3, writesize - 4);//del len[sn \n]
	} else if (!strncmp(cmd, "reconnect_sn ", 13)) {
		if (writesize > 14) {
			usb_send_buf_device_msg(vdev, USB_VIRT_DEVICE_CMD_SET_SN, cmd + 13, writesize - 14);
			mdelay(10);
			usb_send_device_msg(vdev, USB_VIRT_DEVICE_CMD_RECONNECT);
		}
	} else if (!strncmp(cmd, "reconnect", 9))
		usb_send_device_msg(vdev, USB_VIRT_DEVICE_CMD_RECONNECT);
	return writesize;
exit:
	dev_err(&vdev->udev->dev, "write error\n");
	return writesize;
}

static const struct file_operations virtual_usb_fops = {
	.owner = THIS_MODULE,
	.read = virtual_usb_read,
	.write = virtual_usb_write,
	.open = virtual_usb_open,
	.release = virtual_usb_release,
	.llseek = noop_llseek,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver virtual_usb_class = {
	.name = "virtual_usb%d",
	.fops = &virtual_usb_fops,
	.minor_base = USB_VIRT_MINOR_BASE,
};

int usb_register_virt_dev(struct usb_interface *intf)
{
	/* we can register the device now, as it is ready */
	return usb_register_dev(intf, &virtual_usb_class);
}

void usb_unregister_virt_dev(struct usb_interface *intf)
{
	/* give back our minor */
	usb_deregister_dev(intf, &virtual_usb_class);
}
