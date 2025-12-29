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
#include <linux/dma-direct.h>
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/security.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#include <drm/drm_print.h>

#include "bst_virt_drm_device.h"
#include "bst_virt_dc/virt_dc_dev.h"
#include "bst_virt_dp/virt_dp_dev.h"
#include "bst_virt_lvds/virt_lvds_dev.h"
#include "bst_virt_mipi/virt_mipi_dev.h"
#include "bst_virt_drm_debugfs.h"
#include "bst_virt_drm_kms.h"
#include "bst-fw-msg/firmware_cmdsets/bst_display_cmdset_api.h"

struct bst_str {
	char *str;
	u32 sz;
	u32 len;
};

/* return 0 on success,  < 0 on no space.
 */
__printf(2, 3) static int bst_sprintf(struct bst_str *str, const char *fmt, ...)
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

	/* GLB error */
	evt_sprintf(str, events & BST_DRM_ERR_MERR, "MERR|");
	evt_sprintf(str, events & BST_DRM_ERR_FRAMETO, "FRAMETO|");

	/* DOU error */
	evt_sprintf(str, events & BST_DRM_ERR_DRIFTTO, "DRIFTTO|");
	evt_sprintf(str, events & BST_DRM_ERR_FRAMETO, "FRAMETO|");
	evt_sprintf(str, events & BST_DRM_ERR_TETO, "TETO|");
	evt_sprintf(str, events & BST_DRM_ERR_CSCE, "CSCE|");

	/* LPU errors or events */
	evt_sprintf(str, events & BST_DRM_EVENT_IBSY, "IBSY|");
	evt_sprintf(str, events & BST_DRM_EVENT_EMPTY, "EMPTY|");
	evt_sprintf(str, events & BST_DRM_EVENT_FULL, "FULL|");
	evt_sprintf(str, events & BST_DRM_ERR_AXIE, "AXIE|");
	evt_sprintf(str, events & BST_DRM_ERR_ACE0, "ACE0|");
	evt_sprintf(str, events & BST_DRM_ERR_ACE1, "ACE1|");
	evt_sprintf(str, events & BST_DRM_ERR_ACE2, "ACE2|");
	evt_sprintf(str, events & BST_DRM_ERR_ACE3, "ACE3|");

	/* LPU TBU errors*/
	evt_sprintf(str, events & BST_DRM_ERR_TCF, "TCF|");
	evt_sprintf(str, events & BST_DRM_ERR_TTNG, "TTNG|");
	evt_sprintf(str, events & BST_DRM_ERR_TITR, "TITR|");
	evt_sprintf(str, events & BST_DRM_ERR_TEMR, "TEMR|");
	evt_sprintf(str, events & BST_DRM_ERR_TTF, "TTF|");

	/* CU errors*/
	evt_sprintf(str, events & BST_DRM_ERR_CPE, "COPROC|");
	evt_sprintf(str, events & BST_DRM_ERR_ZME, "ZME|");
	evt_sprintf(str, events & BST_DRM_ERR_CFGE, "CFGE|");
	evt_sprintf(str, events & BST_DRM_ERR_TEMR, "TEMR|");

	if (str->len > 0 && (str->str[str->len - 1] == '|')) {
		str->str[str->len - 1] = 0;
		str->len--;
	}
}

static bool is_new_frame(struct bst_virt_events *a)
{
	return a->pipes &
	       (BST_DRM_EVENT_FLIP | BST_DRM_EVENT_EOW);
}

void bst_virt_print_events(struct bst_virt_events *evts, struct drm_device *dev)
{
	struct bst_super_device *sdev = dev->dev_private;
	u64 print_evts = 0;
	static bool en_print = true;
	// struct bst_dev *mdev = dev->dev_private;
	u16 const err_verbosity = sdev->err_verbosity;
	u64 evts_mask = evts->global | evts->pipes;

	/* reduce the same msg print, only print the first evt for one frame */
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
		str.sz = sizeof(msg);
		str.len = 0;

		bst_sprintf(&str, "gcu: ");
		evt_str(&str, evts->global);
		bst_sprintf(&str, ", pipes: ");
		evt_str(&str, evts->pipes);

		DRM_ERROR("err detect: %s\n", msg);
		if ((err_verbosity & BST_DRM_DEV_PRINT_DUMP_STATE_ON_EVENT) &&
		    (evts_mask & (BST_DRM_ERR_EVENTS | BST_DRM_WARN_EVENTS)))
			drm_state_dump(dev, &p);

		en_print = false;
	}
}

static int bst_crtc_read(struct seq_file *sf, void *x)
{
	struct bst_super_device *super_dev = sf->private;
	int i;
	struct bst_virt_dc_crtc *dc_crtc = super_dev->pipelines[0]->dc_crtc;
	seq_puts(sf, "\n====== bst_crtc_read =========\n");
	seq_printf(sf, "\n====== pipeline num:%d =========\n",super_dev->n_pipelines);
	pm_runtime_get_sync(super_dev->dev);
	for (i = 0; i < super_dev->n_pipelines; i++) {
		dc_crtc = super_dev->pipelines[i]->dc_crtc;
		seq_printf(sf, "pipeline%d:supported_color_formats:0x%x supported_color_depths:0x%x supports_degamma:%d supports_csc:%d supports_gamma:%d supports_dual_link:%d\n",
		i,
		dc_crtc->supported_color_formats,
		dc_crtc->supported_color_depths,
		dc_crtc->supports_degamma,
		dc_crtc->supports_csc,
		dc_crtc->supports_gamma,
		dc_crtc->supports_dual_link);
	}
	pm_runtime_put(super_dev->dev);

	return 0;
}

static int bst_crtc_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, bst_crtc_read, inode->i_private);
}

static const struct file_operations bst_crtc_fops = {
	.owner = THIS_MODULE,
	.open = bst_crtc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static int bst_drm_subdev_info_read(struct seq_file *sf, void *x)
{
	struct bst_all_subdev_topo_req request = {0};
	struct bst_all_subdev_topo result = {0};
	int i, j;
	int ret;

	ret = bst_display_glb_cmd_get_all_subdev_topo(&request, &result);
	if (ret)
		return -EINVAL;

	for (i = BST_SUBDEV_DC0_PIPE0; i < BST_SUBDEV_MAX; i++) {
		for (j = 0; j < result.num; j++) {
			if ((result.topo[j].dc_subdev == i) ||
			    (result.topo[j].conn_subdev == i)) {
				ret = bst_virt_subdev_dump_info(sf, i);
				if (ret) {
					seq_printf(sf, "Error, dump subdev(%d) info!\n", i);
					continue;
				}
			}
		}
	}
	return 0;
}
static int bst_drm_subdev_info_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, bst_drm_subdev_info_read, inode->i_private);
}

static const struct file_operations bst_drm_subdev_info_fops = {
	.owner = THIS_MODULE,
	.open = bst_drm_subdev_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

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

static int save_wb_fb_to_file(void *vaddr, u32 size, const char* fname)
{
    struct file *filp;
    loff_t pos;

	filp = filp_open(fname, O_RDWR | O_CREAT, 0777);
	if (IS_ERR(filp)) {
		DRM_ERROR("cannot open the file %s, ret %ld", fname, (long)filp);
	} else {
		pos = 0;
		kernel_write(filp, vaddr, size, &pos);
		filp_close(filp, NULL);
	}

    return 0;
}

static int bst_force_writeback_show(struct seq_file *sf, void *x)
{
	struct bst_virt_device *bst_vir_dev = sf->private;
	struct virt_dc_dev *dc_dev = (struct virt_dc_dev *)(bst_vir_dev->virt_dev_data);
	struct bst_display_wb_layer_cfg wb_cfg = {0};
	struct drm_display_mode *mode;
	struct bst_display_comm_reply reply = {0};
	struct completion force_wb;
	void *vaddr;
	dma_addr_t dma_addr;
	int ret = 0, size, align_size, timeout;
	char fname[64] = {0};

	if(!(dc_dev) || !(dc_dev->bcrtc)) {
		seq_puts(sf, "Error: crtc not enable.\n");
		return 0;
	}

	init_completion(&force_wb);
	bst_writeback_mkdir("/mnt/drm", S_IRUGO | S_IWUSR);

	pm_runtime_get_sync(bst_vir_dev->dev);
	mode = (struct drm_display_mode *)(&dc_dev->bcrtc->base.mode);
	seq_printf(sf, "crtc hdisplay(%d) vdisplay(%d)\n", mode->hdisplay, mode->vdisplay);

	wb_cfg.input_id = SUBMODULE_ID_DC_COMPOSER;
	wb_cfg.frame_mode = DC_LAYER_WB_FRAME_MODE_ONE;
	wb_cfg.precision_reduce_mode = DC_LAYER_WB_PRECISION_REDUCE_MODE_ROUNDING;
	wb_cfg.hsize = mode->hdisplay;
	wb_cfg.vsize = mode->vdisplay;
	wb_cfg.pixel_format = DC_LOCAL_FMT_RGB_888; //RGB888
	wb_cfg.p0_stride = mode->hdisplay * 3;
	wb_cfg.num_planars = 1;
	wb_cfg.layer_en = 1;
	wb_cfg.pixel_format_standard = BIT(DC_PIX_FMT_STD_TYPE_LOCAL);

	size = mode->hdisplay * mode->vdisplay * 3;
	align_size = ALIGN(size, PAGE_SIZE);
	vaddr = dma_alloc_coherent(bst_vir_dev->dev, align_size, &dma_addr, GFP_KERNEL);
    if (vaddr == NULL) {
		seq_puts(sf, "Error: alloc buf fail.\n");
        goto out;
    }

	wb_cfg.p0_ptr = dma_addr;
	ret = bst_display_dc_cmd_update_wb_layer(bst_vir_dev->subdev_session, &wb_cfg, &reply);
	if (ret || reply.base.status != DISP_COMM_REPLAY_OK)
		DRM_ERROR("wirteback layer update falied!!\n");

	dc_dev->bcrtc->force_wb_flag = 1;
	dc_dev->bcrtc->force_wb_comp = &force_wb;
	timeout = wait_for_completion_timeout(dc_dev->bcrtc->force_wb_comp, HZ);
	if (0 == timeout) {
		seq_printf(sf, "wait crtc-%d force writeback done timeout\n", drm_crtc_index(&dc_dev->bcrtc->base));
		goto out;
	}

	snprintf(fname, 64, "/mnt/drm/wb_fb_crtc%d_idx%d.RGB888",
				drm_crtc_index(&dc_dev->bcrtc->base), dc_dev->bcrtc->force_wb_index++);
	save_wb_fb_to_file(vaddr, size, fname);
	seq_printf(sf, "bst drm writeback done to file(%s),size(%d)\n", fname, size);

out:
	dma_free_coherent(bst_vir_dev->dev, align_size, vaddr, dma_addr);
	dc_dev->bcrtc->force_wb_comp = NULL;
	pm_runtime_put(bst_vir_dev->dev);

	return ret;
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


#ifdef CONFIG_DEBUG_FS
static void bst_debugfs_init(struct bst_super_device *super_dev)
{
	int i;
	char filename[32] = "force_writeback_pipe??";

	if (!debugfs_initialized())
		return;

	super_dev->debugfs_root = debugfs_create_dir(super_dev->dev->of_node->full_name, NULL);
	debugfs_create_x16("err_verbosity", 0664, super_dev->debugfs_root,
			   &super_dev->err_verbosity);
	debugfs_create_file("crtc_debug", 0444, super_dev->debugfs_root,super_dev,
			&bst_crtc_fops);
	debugfs_create_file("subdev_info", 0444, super_dev->debugfs_root,super_dev,
			&bst_drm_subdev_info_fops);
	for(i = 0; i < super_dev->n_pipelines; i ++) {
		//TODO: assume 1 pipe 1 link, if 1 pipe 2 links do not support
		sprintf(filename+strlen(filename)-2, "%02d", i);
		debugfs_create_file(filename, 0444, super_dev->debugfs_root, super_dev->subdevs[i][0],
			&bst_force_writeback_fops);
	}
}
#endif

union pipe_info_id {
	struct {
		__u32 max_line_sz : 16, n_subdevs : 4, n_layers : 4,
			n_richs : 4, pipe_idx : 4;
	};
	__u32 value;
};

static ssize_t pipe_info_id_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct bst_super_device *super_dev = dev_to_super_dev(dev);
	struct bst_virt_pipe *pipe = super_dev->pipelines[0];
	union pipe_info_id info_id;
	int i, pipe_idx, len = 0;

	for (pipe_idx = 0; pipe_idx < super_dev->n_pipelines; pipe_idx++) {
		pipe = super_dev->pipelines[pipe_idx];
		if (pipe) {
			memset(&info_id, 0, sizeof(info_id));
			info_id.max_line_sz = pipe->dc_layers[0]->hsize_in.end;
			info_id.n_subdevs = pipe->n_subdevs;
			info_id.n_layers = pipe->n_dc_layers;
			info_id.pipe_idx = pipe->pipe_id;
			info_id.n_richs = 0;
			for (i = 0; i < pipe->n_dc_layers; i++) {
				if (pipe->dc_layers[i]->layer_type ==
				    BST_DRM_FMT_RICH_LAYER)
					info_id.n_richs++;
			}
			len += snprintf(buf + len, PAGE_SIZE,
					"PIPE-%d-INFO:0x%08x\n", pipe_idx,
					info_id.value);
		}
	}

	return len;
}
static DEVICE_ATTR_RO(pipe_info_id);

static struct attribute *bst_sysfs_entries[] = {
	&dev_attr_pipe_info_id.attr,
	NULL,
};

static struct attribute_group bst_sysfs_attr_group = {
	.attrs = bst_sysfs_entries,
};


#define RICH BST_DRM_FMT_RICH_LAYER
#define SIMPLE BST_DRM_FMT_SIMPLE_LAYER
#define RICH_SIMPLE (BST_DRM_FMT_RICH_LAYER | BST_DRM_FMT_SIMPLE_LAYER)
#define RICH_WB (BST_DRM_FMT_RICH_LAYER | BST_DRM_FMT_WB_LAYER)
#define RICH_SIMPLE_WB (RICH_SIMPLE | BST_DRM_FMT_WB_LAYER)

#define Rot_0 DRM_MODE_ROTATE_0
#define Flip_H_V (DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y | Rot_0)
#define Rot_ALL_H_V (DRM_MODE_ROTATE_MASK | Flip_H_V)

#define LYT_NM BIT(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16)
#define LYT_WB BIT(AFBC_FORMAT_MOD_BLOCK_SIZE_32x8)
#define LYT_NM_WB (LYT_NM | LYT_WB)

#define AFB_TH AFBC(_TILED | _SPARSE)
#define AFB_TH_SC_YTR AFBC(_TILED | _SC | _SPARSE | _YTR)
#define AFB_TH_SC_YTR_BS AFBC(_TILED | _SC | _SPARSE | _YTR | _SPLIT)
static struct bst_format_caps layer_format_caps_table[] = {
	/*   FW_ID    |        fourcc         |   layer_types |   rots    | afbc_layouts | afbc_features */
	/* ABGR_2101010*/
	{ DC_LOCAL_FMT_ARGB_2101010, DRM_FORMAT_ARGB2101010, RICH_SIMPLE_WB, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_ABGR_2101010, DRM_FORMAT_ABGR2101010, RICH_SIMPLE_WB, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_ABGR_2101010, DRM_FORMAT_ABGR2101010, RICH_SIMPLE, Rot_ALL_H_V, LYT_NM_WB, AFB_TH_SC_YTR_BS }, /* afbc */
	{ DC_LOCAL_FMT_RGBA_1010102, DRM_FORMAT_RGBA1010102, RICH_SIMPLE_WB, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_BGRA_1010102, DRM_FORMAT_BGRA1010102, RICH_SIMPLE_WB, Flip_H_V, 0, 0 },
	/* ABGR_8888*/
	{ DC_LOCAL_FMT_ARGB_8888, DRM_FORMAT_ARGB8888, RICH_SIMPLE_WB, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_ABGR_8888, DRM_FORMAT_ABGR8888, RICH_SIMPLE_WB, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_ABGR_8888, DRM_FORMAT_ABGR8888, RICH_SIMPLE, Rot_ALL_H_V, LYT_NM_WB, AFB_TH_SC_YTR_BS }, /* afbc */
	{ DC_LOCAL_FMT_RGBA_8888, DRM_FORMAT_RGBA8888, RICH_SIMPLE_WB, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_BGRA_8888, DRM_FORMAT_BGRA8888, RICH_SIMPLE_WB, Flip_H_V, 0, 0 },
	/* XBGB_8888 */
	{ DC_LOCAL_FMT_XRGB_8888, DRM_FORMAT_XRGB8888, RICH_SIMPLE_WB, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_XBGR_8888, DRM_FORMAT_XBGR8888, RICH_SIMPLE_WB, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_RGBX_8888, DRM_FORMAT_RGBX8888, RICH_SIMPLE_WB, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_BGRX_8888, DRM_FORMAT_BGRX8888, RICH_SIMPLE_WB, Flip_H_V, 0, 0 },
	/* BGR_888 */ /* none-afbc RGB888 doesn't support rotation and flip */
	{ DC_LOCAL_FMT_RGB_888, DRM_FORMAT_RGB888, RICH_SIMPLE_WB, Rot_0, 0, 0 },
	{ DC_LOCAL_FMT_BGR_888, DRM_FORMAT_BGR888, RICH_SIMPLE_WB, Rot_0, 0, 0 },
	{ DC_LOCAL_FMT_BGR_888, DRM_FORMAT_BGR888, RICH_SIMPLE, Rot_ALL_H_V, LYT_NM_WB, AFB_TH_SC_YTR_BS }, /* afbc */
	/* BGR 16bpp */
	{ DC_LOCAL_FMT_RGBA_5551, DRM_FORMAT_RGBA5551, RICH_SIMPLE, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_ABGR_1555, DRM_FORMAT_ABGR1555, RICH_SIMPLE, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_ABGR_1555, DRM_FORMAT_ABGR1555, RICH_SIMPLE, Rot_ALL_H_V, LYT_NM_WB, AFB_TH_SC_YTR }, /* afbc */
	{ DC_LOCAL_FMT_RGB_565, DRM_FORMAT_RGB565, RICH_SIMPLE, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_BGR_565, DRM_FORMAT_BGR565, RICH_SIMPLE, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_BGR_565, DRM_FORMAT_BGR565, RICH_SIMPLE, Rot_ALL_H_V, LYT_NM_WB, AFB_TH_SC_YTR }, /* afbc */
	{ DC_LOCAL_FMT_R8, DRM_FORMAT_R8, SIMPLE, Rot_0, 0, 0 },
	/* YUV 444/422/420 8bit  */
	{ DC_LOCAL_FMT_YUV_422_P2_8, DRM_FORMAT_YUYV, RICH, Rot_ALL_H_V, LYT_NM, AFB_TH }, /* afbc */
	{ DC_LOCAL_FMT_VYUY_422_P1_8, DRM_FORMAT_YUYV, RICH, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_YVYU_422_P1_8, DRM_FORMAT_UYVY, RICH, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_YUV_420_P2_8, DRM_FORMAT_NV12, RICH, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_YUV_420_P2_8, DRM_FORMAT_YUV420_8BIT, RICH, Rot_ALL_H_V, LYT_NM, AFB_TH }, /* afbc */
	{ DC_LOCAL_FMT_YUV_420_P3_8, DRM_FORMAT_YUV420, RICH, Flip_H_V, 0, 0 },
	/* YUV 10bit*/
	{ DC_LOCAL_FMT_YUV_420_P1_10, DRM_FORMAT_X0L2, RICH, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_YUV_420_P2_10, DRM_FORMAT_P010, RICH, Flip_H_V, 0, 0 },
	{ DC_LOCAL_FMT_YUV_420_P2_10, DRM_FORMAT_YUV420_10BIT, RICH, Rot_ALL_H_V, LYT_NM, AFB_TH },  /* afbc */
};

static bool layer_format_mod_supported(const struct bst_format_caps *caps,
					 u32 layer_type, u64 modifier, u32 rot)
{
	uint64_t layout = modifier & AFBC_FORMAT_MOD_BLOCK_SIZE_MASK;

	if ((layout == AFBC_FORMAT_MOD_BLOCK_SIZE_32x8) &&
	    drm_rotation_90_or_270(rot)) {
		DRM_DEBUG_ATOMIC("DC doesn't support ROT90 for WB-AFBC.\n");
		return false;
	}

	return true;
}

static void bst_virt_init_fmt_tbl(struct bst_super_device *sdev)
{
	struct bst_format_caps_table *table = &sdev->fmt_tbl;

	table->format_caps = layer_format_caps_table;
	table->format_mod_supported = layer_format_mod_supported;
	table->n_formats = ARRAY_SIZE(layer_format_caps_table);
}

static void dc_pipe_dump_log(struct bst_virt_pipe *pipe, struct seq_file *sf)
{
}

const struct bst_virt_pipe_funcs pipe_funcs = {
	.dump_log = dc_pipe_dump_log,
};

static struct sub_dev_topo *get_subdev_topo(uint8_t subdev, struct bst_all_subdev_topo *topo) {
	int i;

	for (i = 0; i < topo->num; i++) {
		if (topo->topo[i].dc_subdev == subdev ||
		    topo->topo[i].conn_subdev == subdev) {
			return &topo->topo[i];
		}
	}
	return NULL;
}

static int bst_drm_parse_pipeline_and_components(struct device *dev,
					   struct bst_super_device *super_dev)
{
	struct bst_super_device_info *info = &super_dev->super_info;
	struct device_node *child, *np = dev->of_node;
	struct device_node *parent;
	const char *os_type;
	u32 layer_num;
	u32 virt_dc_type = 0, virt_conn_type = 0;
	u32 pipe_id = U32_MAX;
	struct bst_all_subdev_topo_req request = {0};
	struct bst_all_subdev_topo result = {0};
	int ret;
	int i;

	memset(info, 0, sizeof(struct bst_super_device_info));

	/* Using the global cma pool */
	/*
	 * ret = of_reserved_mem_device_init(dev);
	 * if (ret && ret != -ENODEV)
	 * 	return ret;
	 */

	ret = of_property_read_string(np, "os-type", &os_type);
	if (ret) {
		DRM_ERROR("os-type property is not exist!");
		goto fail;
	}
	info->client_id = 0x20202020;
	for (i = 0; i < 4; i++) {
		if (os_type[i] == 0) {
			break;
		} else {
			info->client_id &= ~(0xff << ((3 - i) * 8));
			info->client_id |= (os_type[i] << ((3 - i) * 8));
		}
	}

	info->platform_id = *(u32 *)of_device_get_match_data(dev);

	display_ipc_client_init(info->client_id, info->platform_id);

	ret = bst_display_glb_cmd_get_all_subdev_topo(&request, &result);
	if (ret)
		return -EINVAL;

	for_each_available_child_of_node(np, child) {
		u8 conn_subdev;
		struct sub_dev_topo *dev_topo = NULL;
		if (of_node_name_eq(child, "pipeline")) {
			ret = of_property_read_u32(child, "reg", &pipe_id);
			if (ret) {
				DRM_ERROR("reg value is not exits!");
				return -EINVAL;
			}

			info->pipe_np_port0[pipe_id] = of_graph_get_remote_node(of_node_get(child),
				BST_DRM_OF_PORT_OUTPUT, 0);
			info->pipe_np_port1[pipe_id] = of_graph_get_remote_node(of_node_get(child),
				BST_DRM_OF_PORT_OUTPUT, 1);

			parent = of_graph_get_remote_node(child, 0, 0);
			if (parent) {
				virt_conn_type = get_remote_node_to_virt_device(parent);
			} else {
				DRM_ERROR("get remote subdevice failed!");
				return -EINVAL;
			}
			conn_subdev = to_fw_subdev_type(virt_conn_type);
			dev_topo = get_subdev_topo(conn_subdev, &result);
			if (!dev_topo) {
				DRM_ERROR("connector subdev-%d is not valid!\n", conn_subdev);
				of_node_put(parent);
				continue;
			}
			info->device_map[pipe_id][BST_VIRT_CONN_IDX] = virt_conn_type;
			virt_dc_type = to_virt_device_type(dev_topo->dc_subdev);
			if (is_dc_device(virt_dc_type)) {
				info->device_map[pipe_id][BST_VIRT_DC_IDX] = virt_dc_type;
				info->n_pipelines++;
			} else {
				DRM_ERROR("connector type(%d) related dc type(%d) is invalid!", virt_conn_type, virt_dc_type);
				of_node_put(parent);
				continue;
			}
			of_property_read_u32(child, "layer-num", &layer_num);
			info->want_layers_num[pipe_id] = layer_num;
			of_node_put(parent);
		}
	}
	return 0;
fail:
	return ret;
}

struct bst_virt_device *
bst_virt_create_subdevice(struct device *dev,
			  struct bst_virt_platform_info *plat_info,
			  struct bst_virt_pipe *pipe)
{
	struct bst_virt_device *virt_dev;

	switch (plat_info->device_type) {
	case DEVICE_TYPE_VIRT_DC_PIPE0:
	case DEVICE_TYPE_VIRT_DC_PIPE1:
	case DEVICE_TYPE_VIRT_DC_PIPE2:
	case DEVICE_TYPE_VIRT_DC_PIPE3:
	case DEVICE_TYPE_VIRT_DC_PIPE4:
		virt_dev = bst_virt_dc_create(dev, plat_info, pipe);
		break;
	case DEVICE_TYPE_VIRT_DP:
		virt_dev = bst_virt_dp_create(dev, plat_info, pipe);
		break;
	case DEVICE_TYPE_VIRT_LVDS0:
	case DEVICE_TYPE_VIRT_LVDS1:
	case DEVICE_TYPE_VIRT_DUAL_LVDS:
		virt_dev = bst_virt_lvds_create(dev, plat_info, pipe);
		break;
	case DEVICE_TYPE_VIRT_DSI0:
	case DEVICE_TYPE_VIRT_DSI1:
		virt_dev = bst_virt_dsi_create(dev, plat_info, pipe);
		break;
	default:
		return NULL;
	}
	return virt_dev;
}

struct bst_super_device *bst_virt_dev_create(struct device *dev)
{
	struct bst_super_device *super_dev;
	struct bst_virt_device *subdev;
	struct bst_super_device_info *super_info;
	struct bst_virt_platform_info plat_info;
	uint32_t device_type;
	int err = 0, pipe_idx, subdev_idx = 0;
	int i;

	super_dev = devm_kzalloc(dev, sizeof(*super_dev), GFP_KERNEL);
	if (!super_dev)
		return ERR_PTR(-ENOMEM);

	super_dev->dev = dev;
	err = bst_drm_parse_pipeline_and_components(dev, super_dev);
	if (err)
		return NULL;

	super_info = &super_dev->super_info;
	plat_info.platform_id = super_info->platform_id;
	bst_virt_init_fmt_tbl(super_dev);

	for (pipe_idx = 0; pipe_idx < super_info->n_pipelines; pipe_idx++) {
		subdev_idx = 0;
		super_dev->pipelines[pipe_idx] = bst_virt_pipe_add(super_dev,
						sizeof(struct bst_virt_pipe),
						&pipe_funcs);
		for (i = 0; i < BST_VIRT_MAX_SUBDEV_OF_1PIPE; i++) {
			device_type = super_info->device_map[pipe_idx][i];
			if (is_dc_device(device_type)) {
				plat_info.device_type = device_type;
				plat_info.want_layer_num = super_info->want_layers_num[pipe_idx];
				subdev = bst_virt_create_subdevice(dev, &plat_info,
					super_dev->pipelines[pipe_idx]);
				if (subdev) {
					super_dev->subdevs[pipe_idx][subdev_idx] = subdev;
					super_dev->pipelines[pipe_idx]->subdevs[subdev_idx] = subdev;
					subdev->fmt_tbl = &super_dev->fmt_tbl;
					super_dev->bus_width = subdev->dev_info.bus_width;
					subdev_idx++;
				} else {
					DRM_ERROR("create subdevice(%d) for pipe%d failed.\n",
						device_type, pipe_idx);
					goto err_cleanup;
				}
				#ifdef CONFIG_DEBUG_FS
					bst_virt_drm_debugfs_init(subdev);
				#endif
			}
		}
		super_dev->pipelines[pipe_idx]->of_output_links[0] = super_info->pipe_np_port0[pipe_idx];
		super_dev->pipelines[pipe_idx]->of_output_links[1] = super_info->pipe_np_port1[pipe_idx];
		super_dev->pipelines[pipe_idx]->n_subdevs = subdev_idx;
	}

	dev->dma_parms = &super_dev->dma_parms;
	dma_set_max_seg_size(dev, DMA_BIT_MASK(32));
	dma_set_mask(dev, DMA_BIT_MASK(36));
	dma_set_coherent_mask(dev, DMA_BIT_MASK(36));

	err = sysfs_create_group(&dev->kobj, &bst_sysfs_attr_group);
	if (err) {
		DRM_ERROR("create sysfs group failed.\n");
		goto err_cleanup;
	}

	super_dev->err_verbosity = BST_DRM_DEV_PRINT_ERR_EVENTS;

#ifdef CONFIG_DEBUG_FS
	bst_debugfs_init(super_dev);
#endif

	return super_dev;

err_cleanup:
	bst_virt_dev_destroy(super_dev);
	return ERR_PTR(err);
}

void bst_virt_dev_destroy(struct bst_super_device *super_dev)
{
	struct device *dev = super_dev->dev;
	struct bst_virt_device *subdev_head;
	int subdev_idx, pipe_idx;

	sysfs_remove_group(&dev->kobj, &bst_sysfs_attr_group);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(super_dev->debugfs_root);
#endif

	for (pipe_idx = 0; pipe_idx < BST_VIRT_MAX_PIPELINES; pipe_idx++) {
		subdev_head = super_dev->subdevs[pipe_idx][0];
		if (subdev_head) {
			bst_virt_pipe_destroy(super_dev,
					      subdev_head->this_pipe);
			super_dev->pipelines[pipe_idx] = NULL;
		}
		for (subdev_idx = 0; subdev_idx < BST_VIRT_MAX_SUBDEV_OF_1PIPE;
		     subdev_idx++) {
			subdev_head = super_dev->subdevs[pipe_idx][subdev_idx];
			if (subdev_head && subdev_head->funcs &&
			    subdev_head->funcs->cleanup) {
				subdev_head->funcs->cleanup(subdev_head);
			}
		}
	}
	super_dev->n_pipelines = 0;
	of_reserved_mem_device_release(dev);
}

int bst_virt_dev_resume(struct bst_super_device *super_dev)
{
	int i, j;
	struct bst_virt_device *vdev;
	if(!super_dev){
		DRM_ERROR("super_dev is null.\n");
		return -1;
	}
	for (i = 0; i < super_dev->n_pipelines; i++) {
		for (j = 0; j < BST_VIRT_MAX_SUBDEV_OF_1PIPE; j++) {
			vdev = super_dev->subdevs[i][j];
			if (vdev && vdev->iommu && vdev->funcs->connect_iommu)
				if (vdev->funcs->connect_iommu(vdev))
					DRM_ERROR("connect iommu failed.\n");

		}
	}
	return 0;
}

int bst_virt_connector_suspend(struct bst_super_device *super_dev) {
	int i;
	struct bst_virt_device *vdev;

	if(!super_dev){
		DRM_ERROR("super_dev is null.\n");
		return -1;
	}

	for (i = 0; i < super_dev->n_pipelines; i++) {
		vdev = super_dev->subdevs[i][BST_VIRT_CONN_IDX];
		if (vdev && vdev->funcs && vdev->funcs->disable_irq)
			vdev->funcs->disable_irq(vdev);
	}

	return 0;
}

int bst_virt_connector_resume(struct bst_super_device *super_dev) {
	int i;
	struct bst_virt_device *vdev;

	if(!super_dev){
		DRM_ERROR("super_dev is null.\n");
		return -1;
	}

	for (i = 0; i < super_dev->n_pipelines; i++) {
		vdev = super_dev->subdevs[i][BST_VIRT_CONN_IDX];
		if (vdev && vdev->funcs && vdev->funcs->enable_irq)
			vdev->funcs->enable_irq(vdev);
	}

	return 0;
}

int bst_virt_dev_suspend(struct bst_super_device *super_dev)
{
	int i, j;
	struct bst_virt_device *vdev;

	if(!super_dev){
		DRM_ERROR("super_dev is null.\n");
		return -1;
	}

	for (i = 0; i < super_dev->n_pipelines; i++) {
		for (j = 0; j < BST_VIRT_MAX_SUBDEV_OF_1PIPE; j++) {
			vdev = super_dev->subdevs[i][j];
			if (vdev && vdev->iommu && vdev->funcs->connect_iommu)
				if (vdev->funcs->connect_iommu(vdev))
					DRM_ERROR("disconnect iommu failed.\n");
		}
	}
	return 0;
}

int bst_virt_dev_request_irq(struct bst_super_device *super_dev) {
	int i;
	struct bst_virt_device *vdev;

	if(!super_dev){
		DRM_ERROR("super_dev is null.\n");
		return -1;
	}

	for (i = 0; i < super_dev->n_pipelines; i++) {
		vdev = super_dev->subdevs[i][BST_VIRT_CONN_IDX];
		if (vdev && vdev->funcs && vdev->funcs->enable_irq)
			vdev->funcs->enable_irq(vdev);
	}
	return 0;
}