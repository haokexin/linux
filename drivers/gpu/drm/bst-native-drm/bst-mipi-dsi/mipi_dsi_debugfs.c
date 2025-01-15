// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include "mipi-dsi-bst.h"
#include "mipi_dsi_hal.h"

#define DSI_DEV_NAME               "bst_dsi"
#ifdef CONFIG_DEBUG_FS
static int dsi_vpg_enable_show(struct seq_file *s, void *unused)
{
	struct dw_mipi_dsi_bst *dsi = s->private;
	u8 vpg_enable = 0;
	if(dsi->vpg_defs.vpg){
		vpg_enable = 1;
	} else{
		vpg_enable = 0;
	}
	seq_printf(s, "%d\n", vpg_enable);

	return 0;
}

static ssize_t dsi_vpg_enable_write(struct file *file,
				       const char __user *ubuf,
				       size_t len, loff_t *ppos)
{
	int ret = 0;
	u8 vpg_enable;
	struct seq_file *s = file->private_data;
	struct dw_mipi_dsi_bst *dsi = s->private;

	ret = kstrtou8_from_user(ubuf, len, 0, &vpg_enable);
	if(vpg_enable){
		dsi->vpg_defs.vpg = true;
	}
	return ret ?: len;
}

static int dsi_vpg_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, dsi_vpg_enable_show, inode->i_private);
}

static const struct file_operations dsi_vpg_enable_fops = {
	.open	   = dsi_vpg_enable_open,
	.write	  = dsi_vpg_enable_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void bst_dsi_debugfs_init(struct dw_mipi_dsi_bst *dev)
{
	struct dentry *file;
	if (!debugfs_initialized())
		return;

	dev->debugfs_root = debugfs_create_dir(DSI_DEV_NAME, NULL);
	file = debugfs_create_file("vpg_en", S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR,
			    dev->debugfs_root, dev, &dsi_vpg_enable_fops);
	if (!file)
		dev_dbg(dev->dev, "Can't create vpg_en\n");
}
#endif