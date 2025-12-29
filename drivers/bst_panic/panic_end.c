// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/kstrtox.h>
#include <linux/uaccess.h>
#include <bst/bst_common_api.h>

extern int send_dtc_to_safety_svc(u32 dtc);
static u32 panic_end_dtc = 0xaabbccdd;

static bool start_send_panic_end;

static ssize_t panic_end_read(struct file *f, char __user *b, size_t size, loff_t *ppos)
{
	return copy_to_user(b, &start_send_panic_end, sizeof(bool));
}

static ssize_t panic_end_write(struct file *f, const char __user *data, size_t len, loff_t *los)
{
	char *kdata;
	int ret;

	kdata = kzalloc(len, GFP_KERNEL);
	if (unlikely(!kdata))
		return -ENOMEM;
	ret = copy_from_user(kdata, data, len);
	if (ret < 0)
		return -EINVAL;

	if (strncmp(kdata, "0", 1) == 0)
		start_send_panic_end = false;
	else
		start_send_panic_end = true;
	kfree(kdata);

	if (start_send_panic_end) {
		send_dtc_to_safety_svc(panic_end_dtc);
	}
	
	return len;
}

static const struct proc_ops panic_end_proc_ops = {
	.proc_open	= simple_open,
	.proc_read	= panic_end_read,
	.proc_write	= panic_end_write,
};

static int __init panic_end_init(void)
{
	struct proc_dir_entry *entry;

	entry = proc_create("panic_end", S_IRUGO, NULL, &panic_end_proc_ops);
	if (unlikely(!entry))
		return -EINVAL;
	return 0;
}

module_init(panic_end_init);

MODULE_LICENSE("GPL2");
MODULE_DESCRIPTION("BST panic");
MODULE_ALIAS("bst_panic");
