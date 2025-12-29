// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <drm/drm_print.h>
#include "bst_display_global_api.h"
#include "bst_virt_drm_debugfs.h"
#include "bst_display_platform.h"

static char *virt_subdev_submodule_to_string(uint32_t subdev,
					     uint32_t submodule_id)
{
	if (IS_DC_SUBDEV_TYPE(subdev)) {
		switch (submodule_id) {
		case SUBMODULE_ID_DC_LAYER0:
			return "SUBMODULE_ID_DC_LAYER0";
		case SUBMODULE_ID_DC_LAYER1:
			return "SUBMODULE_ID_DC_LAYER1";
		case SUBMODULE_ID_DC_LAYER2:
			return "SUBMODULE_ID_DC_LAYER2";
		case SUBMODULE_ID_DC_LAYER3:
			return "SUBMODULE_ID_DC_LAYER3";
		case SUBMODULE_ID_DC_WB_LAYER:
			return "SUBMODULE_ID_DC_WB_LAYER";
		case SUBMODULE_ID_DC_COMPOSER:
			return "SUBMODULE_ID_DC_COMPOSER";
		case SUBMODULE_ID_DC_INVAILD:
			return "SUBMODULE_ID_DC_INVAILD";
		default:
			break;
		}
	}
	switch (subdev) {
	case BST_SUBDEV_eDP:
		switch (submodule_id) {
		case SUBMODULE_ID_DP_VIDEO:
			return "SUBMODULE_ID_DP_VIDEO";
		//case SUBMODULE_ID_DP_AUDIO:
		//	return "SUBMODULE_ID_DP_AUDIO";
		case SUBMODULE_ID_DP_INVAILD:
			return "SUBMODULE_ID_DP_INVAILD";
		default:
			break;
		}
		break;
	case BST_SUBDEV_DSI0:
	case BST_SUBDEV_DSI1:
		switch (submodule_id) {
		case SUBMODULE_ID_MIPI_VIDEO:
			return "SUBMODULE_ID_MIPI_VIDEO";
		case SUBMODULE_ID_MIPI_INVAILD:
			return "SUBMODULE_ID_MIPI_INVAILD";
		default:
			break;
		}
		break;
	case BST_SUBDEV_LVDS0:
	case BST_SUBDEV_LVDS1:
		switch (submodule_id) {
		case SUBMODULE_ID_LVDS_VIDEO:
			return "SUBMODULE_ID_LVDS_VIDEO";
		case SUBMODULE_ID_LVDS_INVAILD:
			return "SUBMODULE_ID_LVDS_INVAILD";
		default:
			break;
		}
		break;
	default:
		break;
	}
	return NULL;
}

static char *virt_subdev_to_string(uint32_t subdev)
{
	switch (subdev) {
	case BST_SUBDEV_NONE:
		return "BST_SUBDEV_NONE";
	case BST_SUBDEV_DC0_PIPE0:
		return "BST_SUBDEV_DC0_PIPE0";
	case BST_SUBDEV_DC0_PIPE1:
		return "BST_SUBDEV_DC0_PIPE1";
	case BST_SUBDEV_DC1_PIPE0:
		return "BST_SUBDEV_DC1_PIPE0";
	case BST_SUBDEV_DC1_PIPE1:
		return "BST_SUBDEV_DC1_PIPE1";
	case BST_SUBDEV_DC2_PIPE0:
		return "BST_SUBDEV_DC2_PIPE0";
	case BST_SUBDEV_eDP:
		return "BST_SUBDEV_eDP";
	case BST_SUBDEV_DSI0:
		return "BST_SUBDEV_DSI0";
	case BST_SUBDEV_DSI1:
		return "BST_SUBDEV_DSI1";
	case BST_SUBDEV_LVDS0:
		return "BST_SUBDEV_LVDS0";
	case BST_SUBDEV_LVDS1:
		return "BST_SUBDEV_LVDS1";
	case BST_SUBDEV_MAX:
		return "BST_SUBDEV_MAX";
	case BST_SUBDEV_INVAL:
		return "BST_SUBDEV_INVAL";
	default:
		break;
	}
	return NULL;
}

static char *client_owner_role_to_string(uint8_t role)
{
	switch (role) {
	case CLIENT_ROLE_NOT_OWNER:
		return "CLIENT_ROLE_NOT_OWNER";
	case CLIENT_ROLE_OWNER:
		return "CLIENT_ROLE_OWNER";
	case CLIENT_ROLE_INVALID:
		return "CLIENT_ROLE_INVALID";
	default:
		break;
	}
	return NULL;
}

static void client_id_to_string(uint32_t client_id, char* sys_name) {
	sys_name[0] = (client_id & 0xff000000) >> 24;
	sys_name[1] = (client_id & 0xff0000) >> 16;
	sys_name[2] = (client_id & 0xff00) >> 8;
	sys_name[3] = client_id & 0xff;
	sys_name[4] = '\0';
}

static void hexnum_to_bin(struct seq_file *s, uint32_t hex)
{
	int i;
	seq_printf(s, " 0b|");
	for (i = 31; i >= 0; i--) {
		seq_printf(s, "%2d|", (hex >> i) & 1);
	}
	seq_printf(s, "\n");
}

int bst_virt_subdev_dump_info(struct seq_file *s, uint8_t subdev) {
	struct bst_subdev_info_req request = { 0 };
	struct bst_subdev_info_result response = { 0 };
	int ret, i, j;
	char sys_name[8];

	request.want_subdev = subdev;
	ret = bst_display_glb_cmd_get_subdev_info(&request, &response);
	if (ret)
		return -EINVAL;
	seq_printf(s, "======= bst virt %s info =======\n", virt_subdev_to_string(request.want_subdev));
	seq_printf(s, "      related_sudev:%s\n", virt_subdev_to_string(response.related_subdev));
	seq_printf(s, "        exec_subdev:%s\n", virt_subdev_to_string(response.exec_subdev));
	seq_printf(s, "         client_num:%d\n", response.clist.client_num);
	for (i = 0; i < response.clist.client_num; i++) {
		client_id_to_string(response.clist.cinfo[i].client_id, sys_name);
		seq_printf(s, "         --------- Client_ID: %s ---------\n", sys_name);
		seq_printf(s, "               role:%s\n", client_owner_role_to_string(response.clist.cinfo[i].role));
		seq_printf(s, "               submodule_num:%d\n", response.clist.cinfo[i].submodule_num);
		seq_printf(s, "               privilege_flags:%#x\n", response.clist.cinfo[i].reserve);
		for (j = 0; j < SUBMODULE_IDS_MAX - 1; j++) {
			if (IS_DC_SUBDEV_TYPE(subdev)) {
				if (j < SUBMODULE_ID_DC_LAYER3)
					seq_printf(s, "               submodule_id[%d]:%s\n", j,
					   virt_subdev_submodule_to_string(subdev, response.clist.cinfo[i].submodule_ids[j]));
				else
					seq_printf(s, "               submodule_id[%d]:%s\n", j,
					   virt_subdev_submodule_to_string(subdev,
						   response.clist.cinfo[i].submodule_ids[j]));
			} else {
				seq_printf(s, "               submodule_id[%d]:%s\n",
					j, virt_subdev_submodule_to_string(subdev, response.clist.cinfo[i].submodule_ids[j]));
			}
		}
		seq_printf(s, "\n");
	}
	return 0;
}



static int virt_dc_info_show(struct seq_file *s, void *unused)
{
	struct virt_dc_dev *dev = s->private;

	seq_printf(s, "======= bst virt dc info =======\n");
	seq_printf(s, "          submodules:%d\n", dev->num_submodules);
	seq_printf(s, "    num_rich_layers:%d\n", dev->num_rich_layers);
	seq_printf(s, "            pipe id:%d\n",
		   dev->base_dev->this_pipe->pipe_id);
	seq_printf(s, "          dual_link:%d\n",
		   dev->base_dev->this_pipe->dual_link);
	seq_printf(s, "        avail_comps:\n");
	seq_printf(s,
		   "---|------------------------------------------"
		   "-----------------------------------------------------|\n");
	seq_printf(s,
		   "bit|31|30|29|28|27|26|25|24|23|22|21|20|19|18"
		   "|17|16|15|14|13|12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1| 0|\n");

	hexnum_to_bin(s, dev->base_dev->this_pipe->avail_comps);
	seq_printf(s,
		   "---|------------------------------------------"
		   "-----------------------------------------------------|\n");
	seq_printf(s, "                   --dev info--\n");
	seq_printf(s, "                 arch_id:%#x\n",
		   dev->base_dev->dev_info.arch_id);
	seq_printf(s, "                 core_id:%#x\n",
		   dev->base_dev->dev_info.core_id);
	seq_printf(s, "               core_info:%#x\n",
		   dev->base_dev->dev_info.core_info);
	seq_printf(s, "          subdev_session:%#x\n",
		   dev->base_dev->dev_info.subdev_session);
	seq_printf(s, "             platform_id:%#x\n",
		   dev->base_dev->dev_info.arch_id);
	seq_printf(s, "             device_type:%#x\n",
		   dev->base_dev->dev_info.device_type);
	seq_printf(s, "             vsync_count:%lld\n",
		   dev->vsync_count);
	seq_printf(s, "             flush_count:%lld\n",
		   dev->flush_count);
	return 0;
}

static int virt_dc_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, virt_dc_info_show, inode->i_private);
}

static const struct file_operations virt_dc_info_fops = {
	.open = virt_dc_info_open,
	.write = NULL,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t virt_dc_pattern_write(struct file *file, const char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	char buf[32];

	struct seq_file *s = file->private_data;
	struct virt_dc_dev *dev = s->private;

	memset(buf, 0, sizeof(buf));
	if (copy_from_user(buf, ubuf, count)) {
		return -EFAULT;
	}
	buf[count] = '\0';

	if (sysfs_streq(buf, "help")) {
		pr_info("echo on/off > /sys/kernel/debug/VIRT_CRTC-[x]/pattern\n");
		goto done;
	}

	if (sysfs_streq(buf, "on"))
		dev->test_mode = true;

	if (sysfs_streq(buf, "off"))
		dev->test_mode = false;
done:
	return count;
}

static int virt_dc_pattern_show(struct seq_file *s, void *unused)
{
	struct virt_dc_dev *dev = s->private;
	seq_printf(s, "%s\n", dev->test_mode ? "On" : "Off");
	return 0;
}
static int virt_dc_pattern_open(struct inode *inode, struct file *file)
{
	return single_open(file, virt_dc_pattern_show, inode->i_private);
}

static const struct file_operations virt_dc_pattern_fops = {
	.open = virt_dc_pattern_open,
	.write = virt_dc_pattern_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int virt_dc_register_show(struct seq_file *s, void *unused)
{
	struct virt_dc_dev *dev = s->private;
	dev->base_dev->funcs->debug_dump(dev->base_dev, s);
	return 0;
}
static int virt_dc_register_open(struct inode *inode, struct file *file)
{
	return single_open(file, virt_dc_register_show, inode->i_private);
}

static const struct file_operations virt_dc_registers_fops = {
	.open = virt_dc_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void virt_dc_debugfs_init(struct virt_dc_dev *dc)
{
	struct dentry *root;
	struct dentry *file;

	root = debugfs_create_dir(dc->base_dev->this_pipe->dc_crtc->base.name,
				  NULL);
	if (IS_ERR_OR_NULL(root)) {
		DRM_ERROR("Can't create debugfs root\n");
		return;
	}

	file = debugfs_create_file("info", 0644, root, dc, &virt_dc_info_fops);
	if (!file)
		DRM_ERROR("Can't create debugfs info\n");

	file = debugfs_create_file("pattern", 0644, root, dc,
				   &virt_dc_pattern_fops);
	if (!file)
		DRM_ERROR("Can't create debugfs train\n");
	file = debugfs_create_file("registers", 0644, root, dc,
				   &virt_dc_registers_fops);
	if (!file)
		DRM_ERROR("Can't create debugfs train\n");
}

static int virt_dp_dump_show(struct seq_file *s, void *unused)
{
	struct virt_dp_dev *dev = s->private;

	seq_printf(s, "======= bst virt dp info =======\n");
	seq_printf(s, "        device_type:%d\n", dev->base_dev->device_type);
	seq_printf(s, "            pipe id:%d\n",
		   dev->base_dev->this_pipe->pipe_id);
	seq_printf(s, "          dual_link:%d\n",
		   dev->base_dev->this_pipe->dual_link);
	seq_printf(s, "        avail_comps:\n");
	seq_printf(s,
		   "---|------------------------------------------"
		   "-----------------------------------------------------|\n");
	seq_printf(s,
		   "bit|31|30|29|28|27|26|25|24|23|22|21|20|19|18"
		   "|17|16|15|14|13|12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1| 0|\n");

	hexnum_to_bin(s, dev->base_dev->this_pipe->avail_comps);
	seq_printf(s,
		   "---|------------------------------------------"
		   "-----------------------------------------------------|\n");
	seq_printf(s, "                   --dev info--\n");
	seq_printf(s, "                 arch_id:%#x\n",
		   dev->base_dev->dev_info.arch_id);
	seq_printf(s, "                 core_id:%#x\n",
		   dev->base_dev->dev_info.core_id);
	seq_printf(s, "               core_info:%#x\n",
		   dev->base_dev->dev_info.core_info);
	seq_printf(s, "          subdev_session:%#x\n",
		   dev->base_dev->dev_info.subdev_session);
	seq_printf(s, "             platform_id:%#x\n",
		   dev->base_dev->dev_info.arch_id);
	seq_printf(s, "             device_type:%#x\n",
		   dev->base_dev->dev_info.device_type);

	return 0;
}
static int virt_dp_dump_open(struct inode *inode, struct file *file)
{
	return single_open(file, virt_dp_dump_show, inode->i_private);
}

static const struct file_operations virt_dp_dump_fops = {
	.open = virt_dp_dump_open,
	.write = NULL,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t virt_dp_train_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	int retval = 0, argc = 0;
	char buf[32];
	char *argv[32];
	char *data;

	struct seq_file *s = file->private_data;
	struct virt_dp_dev *dev = s->private;
	struct bst_virt_connector *v_conn =
		dev->base_dev->this_pipe->master_conn;

	memset(buf, 0, sizeof(buf));
	if (copy_from_user(buf, ubuf, count)) {
		retval = -EFAULT;
		goto done;
	}
	data = buf;
	while ((argv[argc++] = strsep(&data, " "))) {
		;
	}

	if (kstrtouint(argv[0], 10, &v_conn->rate) < 0)
		goto done;

	if (kstrtou8(argv[1], 10, &v_conn->lanes) < 0)
		goto done;

	retval = count;
done:
	return retval;
}
static int virt_dp_train_show(struct seq_file *s, void *unused)
{
	struct virt_dp_dev *dev = s->private;
	seq_printf(
		s, "     connected:%d\n",
		atomic_read(&dev->base_dev->this_pipe->master_conn->connected));
	seq_printf(s, "         lanes:%d\n",
		   dev->base_dev->this_pipe->master_conn->lanes);
	seq_printf(s, "          rate:%d\n",
		   dev->base_dev->this_pipe->master_conn->rate);
	seq_printf(s, "           bpc:%d\n",
		   dev->base_dev->this_pipe->master_conn->bpc);
	return 0;
}
static int virt_dp_train_open(struct inode *inode, struct file *file)
{
	return single_open(file, virt_dp_train_show, inode->i_private);
}

static const struct file_operations virt_dp_train_fops = {
	.open = virt_dp_train_open,
	.write = virt_dp_train_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void virt_dp_debugfs_init(struct virt_dp_dev *dp)
{
	struct dentry *root;
	struct dentry *file;

	root = debugfs_create_dir(dp->base_dev->this_pipe->master_conn->base.name, NULL);
	if (IS_ERR_OR_NULL(root)) {
		DRM_ERROR("Can't create debugfs root\n");
		return;
	}
	
	file = debugfs_create_file("dump", 0644, root, dp, &virt_dp_dump_fops);
	if (!file)
		DRM_ERROR("Can't create debugfs info\n");

	file = debugfs_create_file("train", 0644, root, dp,
				   &virt_dp_train_fops);
	if (!file)
		DRM_ERROR("Can't create debugfs train\n");
}

static int virt_lvds_info_show(struct seq_file *s, void *unused)
{
	return 0;
}
static int virt_lvds_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, virt_lvds_info_show, inode->i_private);
}

static const struct file_operations virt_lvds_info_fops = {
	.open = virt_lvds_info_open,
	.write = NULL,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void virt_lvds_debugfs_init(struct virt_lvds_dev *lvds)
{
	struct dentry *root;
	struct dentry *file;

	root = debugfs_create_dir(lvds->base_dev->this_pipe->master_conn->base.name, NULL);
	if (IS_ERR_OR_NULL(root)) {
		DRM_ERROR("Can't create debugfs root\n");
		return;
	}

	file = debugfs_create_file("info", 0644, root, lvds, &virt_lvds_info_fops);
	if (!file)
		DRM_ERROR("Can't create debugfs info\n");
}

static int virt_mipi_info_show(struct seq_file *s, void *unused)
{
	return 0;
}
static int virt_mipi_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, virt_mipi_info_show, inode->i_private);
}

static const struct file_operations virt_mipi_info_fops = {
	.open = virt_mipi_info_open,
	.write = NULL,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static void virt_mipi_debugfs_init(struct virt_mipi_dev *mipi)
{
	struct dentry *root;
	struct dentry *file;

	root = debugfs_create_dir(mipi->base_dev->this_pipe->master_conn->base.name, NULL);
	if (IS_ERR_OR_NULL(root)) {
		DRM_ERROR("Can't create debugfs root\n");
		return;
	}

	file = debugfs_create_file("info", 0644, root, mipi, &virt_mipi_info_fops);
	if (!file)
		DRM_ERROR("Can't create debugfs info\n");
}


void bst_virt_drm_debugfs_init(struct bst_virt_device *virt_dev)
{
	switch (virt_dev->device_type) {
	case DEVICE_TYPE_VIRT_DC_PIPE0:
	case DEVICE_TYPE_VIRT_DC_PIPE1:
	case DEVICE_TYPE_VIRT_DC_PIPE2:
	case DEVICE_TYPE_VIRT_DC_PIPE3:
	case DEVICE_TYPE_VIRT_DC_PIPE4:
		virt_dc_debugfs_init(virt_dev->virt_dev_data);
		break;
	case DEVICE_TYPE_VIRT_DP:
		virt_dp_debugfs_init(virt_dev->virt_dev_data);
		break;
	case DEVICE_TYPE_VIRT_LVDS0:
	case DEVICE_TYPE_VIRT_LVDS1:
	case DEVICE_TYPE_VIRT_DUAL_LVDS:
		virt_lvds_debugfs_init(virt_dev->virt_dev_data);
		break;
	case DEVICE_TYPE_VIRT_DSI0:
	case DEVICE_TYPE_VIRT_DSI1:
		virt_mipi_debugfs_init(virt_dev->virt_dev_data);
		break;
	default:
		break;
	}
	return;
}
