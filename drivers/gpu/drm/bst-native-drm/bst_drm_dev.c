// SPDX-License-Identifier: GPL-2.0
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/security.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#include <drm/drm_print.h>

#include "bst_md_csr.h"
#include "bst_dpu_csr.h"
#include "bst_drm_dev.h"
#include "bst_drm_resv_mem.h"

struct bst_str {
	char *str;
	u32 sz;
	u32 len;
};

/* return 0 on success,  < 0 on no space.
 */
__printf(2, 3)
static int bst_sprintf(struct bst_str *str, const char *fmt, ...)
{
	va_list args;
	int num, free_sz;
	int err;

	free_sz = str->sz - str->len - 1;
	if (free_sz <= 0)
		return -ENOSPC;

	va_start(args, fmt);

	num = vsnprintf(str->str + str->len, free_sz, fmt, args);

	va_end(args);

	if (num < free_sz) {
		str->len += num;
		err = 0;
	} else {
		str->len = str->sz - 1;
		err = -ENOSPC;
	}

	return err;
}

static void evt_sprintf(struct bst_str *str, u64 evt, const char *msg)
{
	if (evt)
		bst_sprintf(str, msg);
}

static void evt_str(struct bst_str *str, u64 events)
{
	if (events == 0ULL) {
		bst_sprintf(str, "None");
		return;
	}

	evt_sprintf(str, events & BST_DRM_EVENT_VSYNC, "VSYNC|");
	evt_sprintf(str, events & BST_DRM_EVENT_FLIP, "FLIP|");
	evt_sprintf(str, events & BST_DRM_EVENT_EOW, "EOW|");
	evt_sprintf(str, events & BST_DRM_EVENT_MODE, "OP-MODE|");
	evt_sprintf(str, events & BST_DRM_EVENT_URUN, "UNDERRUN|");
	evt_sprintf(str, events & BST_DRM_EVENT_OVR, "OVERRUN|");
	evt_sprintf(str, events & BST_DRM_ERR_MERR, "MERR|");
	evt_sprintf(str, events & BST_DRM_ERR_FRAMETO, "FRAMETO|");
	evt_sprintf(str, events & BST_DRM_ERR_DRIFTTO, "DRIFTTO|");
	evt_sprintf(str, events & BST_DRM_ERR_FRAMETO, "FRAMETO|");
	evt_sprintf(str, events & BST_DRM_ERR_TETO, "TETO|");
	evt_sprintf(str, events & BST_DRM_ERR_CSCE, "CSCE|");
	evt_sprintf(str, events & BST_DRM_EVENT_IBSY, "IBSY|");
	evt_sprintf(str, events & BST_DRM_EVENT_EMPTY, "EMPTY|");
	evt_sprintf(str, events & BST_DRM_EVENT_FULL, "FULL|");
	evt_sprintf(str, events & BST_DRM_ERR_AXIE, "AXIE|");
	evt_sprintf(str, events & BST_DRM_ERR_ACE0, "ACE0|");
	evt_sprintf(str, events & BST_DRM_ERR_ACE1, "ACE1|");
	evt_sprintf(str, events & BST_DRM_ERR_ACE2, "ACE2|");
	evt_sprintf(str, events & BST_DRM_ERR_ACE3, "ACE3|");
	evt_sprintf(str, events & BST_DRM_ERR_TCF, "TCF|");
	evt_sprintf(str, events & BST_DRM_ERR_TTNG, "TTNG|");
	evt_sprintf(str, events & BST_DRM_ERR_TITR, "TITR|");
	evt_sprintf(str, events & BST_DRM_ERR_TEMR, "TEMR|");
	evt_sprintf(str, events & BST_DRM_ERR_TTF, "TTF|");
	evt_sprintf(str, events & BST_DRM_ERR_CPE, "COPROC|");
	evt_sprintf(str, events & BST_DRM_ERR_ZME, "ZME|");
	evt_sprintf(str, events & BST_DRM_ERR_CFGE, "CFGE|");
	evt_sprintf(str, events & BST_DRM_ERR_TEMR, "TEMR|");

	if (str->len > 0 && (str->str[str->len - 1] == '|')) {
		str->str[str->len - 1] = 0;
		str->len--;
	}
}

static bool is_new_frame(struct bst_events *a)
{
	return (a->pipes[0] | a->pipes[1]) &
	       (BST_DRM_EVENT_FLIP | BST_DRM_EVENT_EOW);
}

void bst_print_events(struct bst_events *evts, struct drm_device *dev)
{
	u64 print_evts = 0;
	static bool en_print = true;
	struct bst_dev *mdev = dev->dev_private;
	u16 const err_verbosity = mdev->err_verbosity;
	u64 evts_mask = evts->global | evts->pipes[0] | evts->pipes[1];

	if (evts->global || is_new_frame(evts))
		en_print = true;
	if (!(err_verbosity & BST_DRM_DEV_PRINT_DISABLE_RATELIMIT) && !en_print)
		return;

	if (err_verbosity & BST_DRM_DEV_PRINT_ERR_EVENTS)
		print_evts |= BST_DRM_ERR_EVENTS;
	if (err_verbosity & BST_DRM_DEV_PRINT_WARN_EVENTS)
		print_evts |= BST_DRM_WARN_EVENTS;
	if (err_verbosity & BST_DRM_DEV_PRINT_INFO_EVENTS)
		print_evts |= BST_DRM_INFO_EVENTS;

	if (evts_mask & print_evts) {
		char msg[256];
		struct bst_str str;
		struct drm_printer p = drm_info_printer(dev->dev);

		str.str = msg;
		str.sz  = sizeof(msg);
		str.len = 0;

		bst_sprintf(&str, "gcu: ");
		evt_str(&str, evts->global);
		bst_sprintf(&str, ", pipes[0]: ");
		evt_str(&str, evts->pipes[0]);
		bst_sprintf(&str, ", pipes[1]: ");
		evt_str(&str, evts->pipes[1]);

		DRM_ERROR("err detect: %s\n", msg);
		if ((err_verbosity & BST_DRM_DEV_PRINT_DUMP_STATE_ON_EVENT) &&
		    (evts_mask & (BST_DRM_ERR_EVENTS | BST_DRM_WARN_EVENTS)))
			drm_state_dump(dev, &p);

		en_print = false;
	}
}

static int bst_register_show(struct seq_file *sf, void *x)
{
	struct bst_dev *mdev = sf->private;
	int i;

	seq_puts(sf, "\n====== bst drm register dump =========\n");

	pm_runtime_get_sync(mdev->dev);

	if (mdev->funcs->dump_register)
		mdev->funcs->dump_register(mdev, sf);

	for (i = 0; i < mdev->n_pipelines; i++)
		bst_pipeline_dump_register(mdev->pipelines[i], sf);

	pm_runtime_put(mdev->dev);

	return 0;
}

static int bst_register_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, bst_register_show, inode->i_private);
}

static const struct file_operations bst_register_fops = {
	.owner		= THIS_MODULE,
	.open		= bst_register_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static u64 get_cur_ts_us(void) {
    ktime_t now = ktime_get_real();
    u64 timestamp = ktime_to_ns(now);
    return timestamp / 1000;
}

static int bst_monitor_show(struct seq_file *sf, void *x)
{
	struct bst_dev *mdev = sf->private;
	int i;

	seq_puts(sf, "\n====== bst drm monitor =========\n");

	pm_runtime_get_sync(mdev->dev);
	for (i = 0; i < mdev->n_pipelines; i++) {
		seq_printf(sf, "pipeline[%d] cur timestamp: %lld us\n", i, get_cur_ts_us());
		seq_printf(sf, "pipeline[%d] cur active layer num: %d\n", i, mdev->cur_active_layers[i]);
		seq_printf(sf, "pipeline[%d] max active layer num: %d\n", i, mdev->max_active_layers[i]);
		seq_printf(sf, "pipeline[%d] max available layer num: %d\n", i, mdev->max_layers[i]);
		seq_printf(sf, "pipeline[%d] total frame counts: %d\n", i, mdev->frame_count[i]);
		seq_printf(sf, "pipeline[%d] total flush counts: %d\n", i, mdev->pipe_update_count[i]);
		seq_printf(sf, "pipeline[%d] total underrun counts: %d\n", i, mdev->underrun_err_count[i]);
		seq_printf(sf, "pipeline[%d] preframe output crc: %llx\n", i, mdev->output_crc[i]);
	}
	seq_printf(sf, "current commit time: %lld us\n", mdev->cur_commit_time);
	seq_printf(sf, "average commit time: %lld us\n", mdev->avg_commit_time);

	pm_runtime_put(mdev->dev);

	return 0;
}

static int bst_monitor_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, bst_monitor_show, inode->i_private);
}

static const struct file_operations bst_mointor_fops = {
	.owner		= THIS_MODULE,
	.open		= bst_monitor_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int save_wb_fb_to_file(struct bst_drm_resv_memblock* memblk,
	u32 size, const char* fname)
{
    struct file *filp;
    loff_t pos;

	filp = filp_open(fname, O_RDWR | O_CREAT, 0777);
	if (IS_ERR(filp)) {
		DRM_ERROR("cannot open the file %s, ret %ld", fname, (long)filp);
	} else {
		pos = 0;
		kernel_write(filp, memblk->vaddr, size, &pos);
		filp_close(filp, NULL);
	}

    return 0;
}

int bst_writeback_mkdir(const char *pathname, umode_t mode)
{
	struct dentry *dentry;
	struct path path;
	int error;

	error = kern_path(pathname, LOOKUP_DIRECTORY, &path);
	if (!error) {
		// printk(KERN_INFO "Directory already exists: %s\n", pathname);
		path_put(&path);
		return 0;
	}

	dentry = kern_path_create(AT_FDCWD, pathname, &path, LOOKUP_DIRECTORY);
	if (IS_ERR(dentry))
		return PTR_ERR(dentry);
	if (!IS_POSIXACL(path.dentry->d_inode))
		mode &= ~current_umask();
	error = security_path_mkdir(&path, dentry, mode);
	if (!error)
		error = vfs_mkdir(mnt_user_ns(path.mnt), path.dentry->d_inode,
				  dentry, mode);
	done_path_create(&path, dentry);
	return error;
}

static int bst_force_writeback_show(struct seq_file *sf, void *x)
{
	struct bst_dev *mdev = sf->private;
	struct dpu_wb_cfg cfg = {0};
	uint32_t buf_sz;
	uint32_t i, hsize, vsize;
        char fname[80] = {0};
	//uint32_t* raw_val;

	bst_writeback_mkdir("/mnt/drm", S_IRUGO | S_IWUSR);
	seq_puts(sf, "\n====== bst drm writeback start=========\n");
	pm_runtime_get_sync(mdev->dev);
	for (i = 0; i < mdev->n_pipelines; i++) {
		hsize = mdev->cur_cu_hsize[i];
		vsize = mdev->cur_cu_vsize[i];
		buf_sz = hsize * vsize * 3;

		mdev->wb_memblock[i] = mdev->resv_mem_ops->alloc(mdev, buf_sz, 0);
		if (!mdev->wb_memblock[i]) {
			seq_puts(sf, "\n alloc writeback buffer failed\n");
			goto out;
		}

		//raw_val = (u32 *)mdev->wb_memblock[i];
		seq_printf(sf, "\n alloc writeback buffer at pa=0x%llx,va=0x%llx\n",
			(u64)mdev->wb_memblock[i]->phys_addr,
			(u64)mdev->wb_memblock[i]->vaddr);
	    snprintf(fname, 80, "/mnt/drm/wb_fb_dpu%d_pipe%d_idx%d.RA24",
				mdev->chip.display_id, i, mdev->dump_idx[i]++);

		cfg.active_input = (i == 0 ? DPU_CU0 : DPU_CU1);
		cfg.layer_cfg.layer_en = 1;
		cfg.layer_cfg.is_va = 0;
		cfg.layer_cfg.is_yuv = 0;
		cfg.layer_cfg.layer_rot = 0;
		cfg.layer_cfg.layer_flip = 0;
		cfg.layer_cfg.pixel_format = 24; // RGB_888
		cfg.layer_cfg.hsize = hsize;
		cfg.layer_cfg.vsize = vsize;
		cfg.layer_cfg.p0_ptr = (u64)mdev->wb_memblock[i]->phys_addr;
		cfg.layer_cfg.p1_ptr = 0;
		cfg.layer_cfg.p2_ptr = 0;
		cfg.layer_cfg.p0_stride = hsize * 3;
		cfg.layer_cfg.p1_stride = 0;
		cfg.layer_cfg.num_planes = 1;

		mdev->resv_mem_ops->flush_write_buffer();
		mdev->funcs->force_writeback(mdev, &cfg, i);

		mdev->resv_mem_ops->invalid_cache(mdev->wb_memblock[i]);
		
		save_wb_fb_to_file(mdev->wb_memblock[i], buf_sz, fname);

		// line-1 begin / end
		//seq_printf(sf, "\n line-first START[0x%08x,0x%08x,0x%08x,0x%08x]\n",
		//	raw_val[0],
		//	raw_val[1],
		//	raw_val[2],
		//	raw_val[4]);
		//seq_printf(sf, "\n line-first   END[0x%08x,0x%08x,0x%08x,0x%08x]\n",
		//	raw_val[hsize - 4],
		//	raw_val[hsize - 3],
		//	raw_val[hsize - 2],
		//	raw_val[hsize - 1]);
		// line-last begin / end
		//seq_printf(sf, "\n line-last  START[0x%08x,0x%08x,0x%08x,0x%08x]\n",
		//	raw_val[hsize * (vsize-1)],
		//	raw_val[hsize * (vsize-1) + 1],
		//	raw_val[hsize * (vsize-1) + 2],
		//	raw_val[hsize * (vsize-1) + 3]);
		//seq_printf(sf, "\n line-last    END[0x%08x,0x%08x,0x%08x,0x%08x]\n",
		//	raw_val[hsize * vsize - 4],
		//	raw_val[hsize * vsize - 3],
		//	raw_val[hsize * vsize - 2],
		//	raw_val[hsize * vsize - 1]);

		mdev->resv_mem_ops->free(mdev->wb_memblock[i]);
		mdev->wb_memblock[i] = NULL;
		seq_printf(sf, "\n====== bst drm writeback done to file(%s),size(%d)=========\n",
			fname, buf_sz);
	}

out:
	pm_runtime_put(mdev->dev);

	return 0;
}

static int bst_force_writeback_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, bst_force_writeback_show, inode->i_private);
}

static const struct file_operations bst_force_writeback_fops = {
	.owner		= THIS_MODULE,
	.open		= bst_force_writeback_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int bst_poweroff_p0_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, bst_monitor_show, inode->i_private);
}

static ssize_t bst_poweroff_p0_write(struct file *filp, const char __user *user_buf,
		  size_t count, loff_t *ppos)
{
	struct bst_dev *mdev = (struct bst_dev *)file_inode(filp)->i_private;
	unsigned int disable;
	int ret;

	ret = kstrtouint_from_user(user_buf, count, 0, &disable);
	if (ret < 0)
		return ret;

	DRM_INFO("display_poweroff_pipe0: display_id:%d,core_info:0x%x,arch_id:0x%x,disable:%d\n",
		mdev->chip.display_id, mdev->chip.core_info,mdev->chip.arch_id, disable);

	mdev->funcs->force_pipe(mdev, 0, disable);
	mdev->funcs->hw_reset(mdev);

	return count;
}


static const struct file_operations bst_poweroff_p0_fops = {
	.owner		= THIS_MODULE,
	.open		= bst_poweroff_p0_open,
	.read		= seq_read,
	.write      = bst_poweroff_p0_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int bst_poweroff_p1_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, bst_monitor_show, inode->i_private);
}

static ssize_t bst_poweroff_p1_write(struct file *filp, const char __user *user_buf,
		  size_t count, loff_t *ppos)
{
	struct bst_dev *mdev = (struct bst_dev *)file_inode(filp)->i_private;
	unsigned int disable;
	int ret;

	ret = kstrtouint_from_user(user_buf, count, 0, &disable);
	if (ret < 0)
		return ret;

	DRM_INFO("display_poweroff_pipe1: display_id:%d,core_info:0x%x,arch_id:0x%x,disable:%d\n",
		mdev->chip.display_id, mdev->chip.core_info,mdev->chip.arch_id, disable);

	mdev->funcs->force_pipe(mdev, 1, disable);
	mdev->funcs->hw_reset(mdev);

	return count;
}


static const struct file_operations bst_poweroff_p1_fops = {
	.owner		= THIS_MODULE,
	.open		= bst_poweroff_p1_open,
	.read		= seq_read,
	.write      = bst_poweroff_p1_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#ifdef CONFIG_DEBUG_FS
static void bst_debugfs_init(struct bst_dev *mdev)
{
	if (!debugfs_initialized())
		return;

	if (mdev->chip.display_id == BST_DRM_DISPLAY_ID_0)
		mdev->debugfs_root = debugfs_create_dir("bst-drm-0", NULL);
	else if (mdev->chip.display_id == BST_DRM_DISPLAY_ID_1)
		mdev->debugfs_root = debugfs_create_dir("bst-drm-1", NULL);
	else if (mdev->chip.display_id == BST_DRM_DISPLAY_ID_2)
		mdev->debugfs_root = debugfs_create_dir("bst-drm-2", NULL);
	else {
		DRM_ERROR("failed to create debugfs for display_%d",
		mdev->chip.display_id - BST_DRM_DISPLAY_ID_0);
		return;
	}

	debugfs_create_file("register", 0444, mdev->debugfs_root,
			    mdev, &bst_register_fops);
	debugfs_create_x16("err_verbosity", 0664, mdev->debugfs_root,
			   &mdev->err_verbosity);
	debugfs_create_file("monitor", 0444, mdev->debugfs_root,
			    mdev, &bst_mointor_fops);
	debugfs_create_file("poweroff_p0", 0777, mdev->debugfs_root,
			    mdev, &bst_poweroff_p0_fops);
	debugfs_create_file("poweroff_p1", 0777, mdev->debugfs_root,
			    mdev, &bst_poweroff_p1_fops);
	debugfs_create_file("force_writeback", 0444, mdev->debugfs_root,
			    mdev, &bst_force_writeback_fops);

}
#endif

static ssize_t
core_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bst_dev *mdev = dev_to_mdev(dev);

	return sysfs_emit(buf, "0x%08x\n", mdev->chip.core_id);
}
static DEVICE_ATTR_RO(core_id);

static ssize_t
config_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bst_dev *mdev = dev_to_mdev(dev);
	struct bst_pipeline *pipe = mdev->pipelines[0];
	union bst_config_id config_id;
	int i;

	memset(&config_id, 0, sizeof(config_id));

	config_id.max_line_sz = pipe->layers[0]->hsize_in.end;
	config_id.n_pipelines = mdev->n_pipelines;
	config_id.n_scalers = pipe->n_scalers;
	config_id.n_layers = pipe->n_layers;
	config_id.n_richs = 0;
	for (i = 0; i < pipe->n_layers; i++) {
		if (pipe->layers[i]->layer_type == BST_DRM_FMT_RICH_LAYER)
			config_id.n_richs++;
	}
	return sysfs_emit(buf, "0x%08x\n", config_id.value);
}
static DEVICE_ATTR_RO(config_id);

static ssize_t
aclk_hz_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "aclk 800MHz defalut\n");
}
static DEVICE_ATTR_RO(aclk_hz);

static struct attribute *bst_sysfs_entries[] = {
	&dev_attr_core_id.attr,
	&dev_attr_config_id.attr,
	&dev_attr_aclk_hz.attr,
	NULL,
};

static struct attribute_group bst_sysfs_attr_group = {
	.attrs = bst_sysfs_entries,
};

static int bst_parse_pipe_dt(struct bst_pipeline *pipe)
{
	struct device_node *np = pipe->of_node;

	struct clk *clk;
	clk = of_clk_get_by_name(np, "pll_clk");
	if (IS_ERR(clk)) {
		DRM_ERROR("get pll_clk for pipeline %d failed!\n", pipe->id);
		return PTR_ERR(clk);
	}
	pipe->pll_clk = clk;



	clk = of_clk_get_by_name(np, "div_clk");
	if (IS_ERR(clk)) {
		DRM_ERROR("get div_clk for pipeline %d failed!\n", pipe->id);
		return PTR_ERR(clk);
	}
	pipe->div_clk = clk;

	clk = of_clk_get_by_name(np, "gate_pixelclk");
	if (IS_ERR(clk)) {
		DRM_ERROR("get gate_pixelclk for pipeline %d failed!\n", pipe->id);
		return PTR_ERR(clk);
	}
	pipe->gate_pxlclk = clk;


	clk = of_clk_get_by_name(np, "gate_aclk");
	if (IS_ERR(clk)) {
		DRM_ERROR("get gate_aclk for pipeline %d failed!\n", pipe->id);
		return PTR_ERR(clk);
	}
	pipe->gate_aclk = clk;


	clk = of_clk_get_by_name(np, "gate_pclk");
	if (IS_ERR(clk)) {
		DRM_ERROR("get gate_pclk for pipeline %d failed!\n", pipe->id);
		return PTR_ERR(clk);
	}
	pipe->gate_pclk = clk;

	clk = of_clk_get_by_name(np, "mux_aclk");
	if (IS_ERR(clk)) {
		DRM_ERROR("get mux_aclk for pipeline %d failed!\n", pipe->id);
		return PTR_ERR(clk);
	}
	pipe->mux_aclk = clk;

	/* enum ports */
	pipe->of_output_links[0] =
		of_graph_get_remote_node(np, BST_DRM_OF_PORT_OUTPUT, 0);
	pipe->of_output_links[1] =
		of_graph_get_remote_node(np, BST_DRM_OF_PORT_OUTPUT, 1);
	pipe->of_output_port =
		of_graph_get_port_by_id(np, BST_DRM_OF_PORT_OUTPUT);

	pipe->dual_link = pipe->of_output_links[0] && pipe->of_output_links[1];

	return 0;
}

static int bst_parse_dt(struct device *dev, struct bst_dev *mdev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *child, *np = dev->of_node;
	struct bst_pipeline *pipe;
	u32 pipe_id = U32_MAX;
	int ret = -1;

	mdev->irq  = platform_get_irq(pdev, 0);
	if (mdev->irq < 0) {
		DRM_ERROR("could not get IRQ number.\n");
		return mdev->irq;
	}
	/* Using the global cma pool */
	/* 	ret = of_reserved_mem_device_init(dev);
	 *if (ret && ret != -ENODEV)
	 *	return ret;
	 */
	ret = 0;

	for_each_available_child_of_node(np, child) {
		if (of_node_name_eq(child, "pipeline")) {
			of_property_read_u32(child, "reg", &pipe_id);
			if (pipe_id >= mdev->n_pipelines) {
				DRM_WARN("Skip the redundant DT node: pipeline-%u.\n",
					 pipe_id);
				continue;
			}
			mdev->pipelines[pipe_id]->of_node = of_node_get(child);
		}
	}

	for (pipe_id = 0; pipe_id < mdev->n_pipelines; pipe_id++) {
		pipe = mdev->pipelines[pipe_id];

		if (!pipe->of_node) {
			DRM_ERROR("Pipeline-%d doesn't have a DT node.\n",
				  pipe->id);
			return -EINVAL;
		}
		ret = bst_parse_pipe_dt(pipe);
		if (ret)
			return ret;
	}

	return 0;
}

struct bst_dev_funcs __global_dc_func;
EXPORT_SYMBOL(__global_dc_func);

struct bst_dev *bst_dev_create(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	bst_identify_func bst_identify;
	struct bst_dev *mdev;
	struct resource *res;
	int err = 0;

	bst_identify = of_device_get_match_data(dev);
	if (!bst_identify)
		return ERR_PTR(-ENODEV);

	mdev = devm_kzalloc(dev, sizeof(*mdev), GFP_KERNEL);
	if (!mdev)
		return ERR_PTR(-ENOMEM);

	mutex_init(&mdev->lock);

	mdev->dev = dev;
	mdev->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mdev->reg_base)) {
		DRM_ERROR("Map register space failed.\n");
		err = PTR_ERR(mdev->reg_base);
		mdev->reg_base = NULL;
		goto err_cleanup;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(dev, "Failed to get csr memory resource\n");
		err = -ENODEV;
		goto err_cleanup;
	}

	mdev->csr_base = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(mdev->csr_base)) {
		dev_err(dev, "Failed to map csr memory resource\n");
		err = PTR_ERR(mdev->csr_base);
		mdev->csr_base = NULL;
		goto err_cleanup;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		dev_err(dev, "Failed to get mdnoc qos memory resource\n");
		err = -ENODEV;
		goto err_cleanup;
	}

	mdev->mdnoc_qos_base = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(mdev->mdnoc_qos_base)) {
		dev_err(dev, "Failed to mdnoc qos base memory resource\n");
		err = PTR_ERR(mdev->mdnoc_qos_base);
		mdev->mdnoc_qos_base = NULL;
		goto err_cleanup;
	}

	mdev->funcs = bst_identify(mdev, &mdev->chip, &__global_dc_func);
	if (!mdev->funcs) {
		DRM_INFO("bst_identify ertry.\n");
		err = -ENODEV;
		goto err_cleanup;
	}

	mdev->funcs->init_format_table(mdev);

	err = mdev->funcs->enum_resources(mdev);
	if (err) {
		DRM_ERROR("enumerate display resource failed.\n");
		goto err_cleanup;
	}

	err = bst_parse_dt(dev, mdev);
	if (err) {
		DRM_ERROR("parse device tree failed.\n");
		goto err_cleanup;
	}

	err = bst_assemble_pipelines(mdev);
	if (err) {
		DRM_ERROR("assemble display pipelines failed.\n");
		goto err_cleanup;
	}

	dev->dma_parms = &mdev->dma_parms;
	dma_set_max_seg_size(dev, DMA_BIT_MASK(32));

	dma_set_mask(dev, DMA_BIT_MASK(36));
	dma_set_coherent_mask(dev, DMA_BIT_MASK(36));

	mdev->iommu = iommu_get_domain_for_dev(mdev->dev);
	if (!mdev->iommu)
		DRM_INFO("continue without IOMMU support!\n");

	mdev->resv_mem_ops = get_resv_mem_ops();
	if (!mdev->resv_mem_ops)
		DRM_INFO("continue without RESV CMA MEM OPS!\n");

	err = sysfs_create_group(&dev->kobj, &bst_sysfs_attr_group);
	if (err) {
		DRM_ERROR("create sysfs group failed.\n");
		goto err_cleanup;
	}

	mdev->err_verbosity = BST_DRM_DEV_PRINT_ERR_EVENTS;
	mdev->pipe_update_count[0] = 0;
	mdev->pipe_update_count[1] = 0;
	mdev->underrun_err_count[0] = 0;
	mdev->underrun_err_count[1] = 0;
	mdev->frame_count[0] = 0;
	mdev->frame_count[1] = 0;
	mdev->cur_commit_time = 0;
	mdev->avg_commit_time = 0;
	mdev->commit_time_sum = 0;
	mdev->commit_counts = 0;
	mdev->cur_active_layers[0] = 0;
	mdev->cur_active_layers[1] = 0;
	mdev->max_active_layers[0] = 0;
	mdev->max_active_layers[1] = 0;
	mdev->max_layers[0] = BST_DRM_PIPELINE_MAX_LAYERS;
	mdev->max_layers[1] = BST_DRM_PIPELINE_MAX_LAYERS;
	mdev->dump_idx[0] = 0;
	mdev->dump_idx[1] = 0;

#ifdef CONFIG_DEBUG_FS
	bst_debugfs_init(mdev);
#endif

	return mdev;
err_cleanup:
	bst_dev_destroy(mdev);
	return ERR_PTR(err);
}

void bst_dev_destroy(struct bst_dev *mdev)
{
	struct device *dev = mdev->dev;
	const struct bst_dev_funcs *funcs = mdev->funcs;
	int i;

	sysfs_remove_group(&dev->kobj, &bst_sysfs_attr_group);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(mdev->debugfs_root);
#endif

	for (i = 0; i < mdev->n_pipelines; i++) {
		bst_pipeline_destroy(mdev, mdev->pipelines[i]);
		mdev->pipelines[i] = NULL;
	}

	mdev->n_pipelines = 0;

	of_reserved_mem_device_release(dev);

	if (funcs && funcs->cleanup)
		funcs->cleanup(mdev);

	if (mdev->csr_base) {
		devm_iounmap(dev, mdev->csr_base);
		mdev->csr_base = NULL;
	}

	if (mdev->reg_base) {
		devm_iounmap(dev, mdev->reg_base);
		mdev->reg_base = NULL;
	}

	devm_kfree(dev, mdev);
}

int bst_dev_resume(struct bst_dev *mdev)
{
	mdev->funcs->enable_irq(mdev);

	if (mdev->iommu && mdev->funcs->connect_iommu)
		if (mdev->funcs->connect_iommu(mdev))
			DRM_ERROR("connect iommu failed.\n");

	return 0;
}

int bst_dev_suspend(struct bst_dev *mdev)
{
	if (mdev->iommu && mdev->funcs->disconnect_iommu)
		if (mdev->funcs->disconnect_iommu(mdev))
			DRM_ERROR("disconnect iommu failed.\n");

	mdev->funcs->disable_irq(mdev);

	return 0;
}

const struct bst_dev_funcs *
dc_identify_display_0(struct bst_dev *mdev, struct bst_chip_info *chip, const struct bst_dev_funcs* chip_func)
{
	const struct bst_dev_funcs *funcs;
	u32 product_id;

	if(!chip_func->enum_resources)
		return NULL;

	chip->core_id = readl((mdev->reg_base + (CORE_ID >> 2)));

	product_id = BSTDC_CORE_ID_PRODUCT_ID(chip->core_id);

	switch (product_id) {
	case BSTDC_C1200_PRODUCT_ID:
		funcs = chip_func;
		break;
	default:
		DRM_ERROR("Unsupported product: 0x%x\n", product_id);
		return NULL;
	}

	chip->arch_id	= readl((mdev->reg_base + (ARCH_ID >> 2)));
	chip->core_info	= readl((mdev->reg_base + (CORE_INFO >> 2)));
	chip->bus_width	= BUS_WIDTH_16_BYTES;
	chip->display_id = BST_DRM_DISPLAY_ID_0;

	return funcs;
}

const struct bst_dev_funcs *
dc_identify_display_1(struct bst_dev *mdev, struct bst_chip_info *chip,  const struct bst_dev_funcs* chip_func)
{
	const struct bst_dev_funcs *funcs;
	u32 product_id;

	if(!chip_func->enum_resources)
		return NULL;

	chip->core_id = readl((mdev->reg_base + (CORE_ID >> 2)));

	product_id = BSTDC_CORE_ID_PRODUCT_ID(chip->core_id);

	switch (product_id) {
	case BSTDC_C1200_PRODUCT_ID:
		funcs = chip_func;
		break;
	default:
		DRM_ERROR("Unsupported product: 0x%x\n", product_id);
		return NULL;
	}

	chip->arch_id	= readl((mdev->reg_base + (ARCH_ID >> 2)));
	chip->core_info	= readl((mdev->reg_base + (CORE_INFO >> 2)));
	chip->bus_width	= BUS_WIDTH_16_BYTES;
	chip->display_id = BST_DRM_DISPLAY_ID_1;

	return funcs;
}

const struct bst_dev_funcs *
dc_identify_display_2(struct bst_dev *mdev, struct bst_chip_info *chip, const struct bst_dev_funcs* chip_func)
{
	const struct bst_dev_funcs *funcs;
	u32 product_id;

	if(!chip_func->enum_resources)
		return NULL;

	chip->core_id = readl((mdev->reg_base + (CORE_ID >> 2)));

	product_id = BSTDC_CORE_ID_PRODUCT_ID(chip->core_id);

	switch (product_id) {
	case BSTDC_C1200_PRODUCT_ID:
		funcs = chip_func;
		break;
	default:
		DRM_ERROR("Unsupported product: 0x%x\n", product_id);
		return NULL;
	}

	chip->arch_id	= readl((mdev->reg_base + (ARCH_ID >> 2)));
	chip->core_info	= readl((mdev->reg_base + (CORE_INFO >> 2)));
	chip->bus_width	= BUS_WIDTH_16_BYTES;
	chip->display_id = BST_DRM_DISPLAY_ID_2;

	return funcs;
}
