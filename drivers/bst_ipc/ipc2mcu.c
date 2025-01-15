// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <bst/ipc_interface.h>
#include <linux/platform_device.h>
#include <linux/ctype.h>
#include <linux/export.h>
#include <linux/kexec.h>
#include <linux/kmod.h>
#include <linux/kmsg_dump.h>
#include <linux/reboot.h>
#include <linux/suspend.h>
#include <linux/syscalls.h>
#include <linux/syscore_ops.h>

//extern int send_safety_usrmsg(u32 fault_code, u32 strategy_code);
struct platform_device bst_g_pdev_ipc;
int send_cpdown_ipcmsg(u32 fault_code, u32 strategy_code)
{
	int session_id;
	int ret;
	ipc_msg msg = {
		.type = IPC_MSG_TYPE_SIGNAL,
		.cmd = strategy_code,
		.data = fault_code,
	};

	session_id = ipc_init(IPC_CORE_SAFE, IPC_CORE_ARM2, &(&bst_g_pdev_ipc)->dev); /* g_pdev_ipc from bst_ddr_ecc_int.c */

	ret = ipc_send(session_id, &msg, 0);
	if (ret < 0)
		pr_err("send error\n");

	return ret;
}

static int ipc2mcu_open(struct inode *nodp, struct file *filp)
{
	return 0;
}

static ssize_t ipc2mcu_read(struct file *filep, char __user *buf, size_t len, loff_t *pos)
{
	return 0;
}

static ssize_t ipc2mcu_write(struct file *filep, const char __user *buf, size_t len, loff_t *pos)
{
	int value;

	if (copy_from_user(&value, buf, len) < 0) {
		pr_err("%s error copy_from_user\n", __func__);
		return -EINVAL;
	}

	send_cpdown_ipcmsg(0xA00D01, 3);
	pr_info("%s: val %d\n", __func__, value);
	return len;
}

static const struct file_operations ipc2mcu_fops = {
	.owner = THIS_MODULE,
	.open = ipc2mcu_open,
	.read = ipc2mcu_read,
	.write = ipc2mcu_write,
};

static struct miscdevice ipc2mcu_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ipc2mcu",
	.fops = &ipc2mcu_fops
};

static int __init ipc2mcu_init(void)
{
	int ret = misc_register(&ipc2mcu_miscdev);

	return ret;
}

device_initcall(ipc2mcu_init);
MODULE_LICENSE("GPL v2");
