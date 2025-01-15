// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef _DC_DEV_H_
#define _DC_DEV_H_

#include "bst_drm_dev.h"
#include "bst_pipeline.h"
#include <linux/hrtimer.h>
#include <linux/spinlock.h>

#define DC_MAX_GLB_SCL_COEFF		4
#define DC_MAX_PIPELINE		2
#define DC_BLOCK_MAX_INPUT		9
#define DC_BLOCK_MAX_OUTPUT		5
#define DC_BLOCK_SIZE			0x0200
#define BST_BLK_INFO_BLK_TYPE(x)	(((x) & 0xFF00) >> 8)
#define BST_LAYER_PIXALPHA		0x0E4
#define DC_PALPHA_DEF_MAP		0xFFAA5500

enum dc_blk_type {
	DC_BLK_TYPE_GCU		= 0x00,
	DC_BLK_TYPE_LPU		= 0x01,
	DC_BLK_TYPE_CU			= 0x02,
	DC_BLK_TYPE_DOU		= 0x03,
	DC_BLK_TYPE_AEU		= 0x04,
	DC_BLK_TYPE_GLB_LT_COEFF	= 0x05,
	DC_BLK_TYPE_GLB_SCL_COEFF	= 0x06,
	DC_BLK_TYPE_GLB_SC_COEFF	= 0x07,
	DC_BLK_TYPE_PERIPH		= 0x08,
	DC_BLK_TYPE_LPU_TRUSTED	= 0x09,
	DC_BLK_TYPE_AEU_TRUSTED	= 0x0A,
	DC_BLK_TYPE_LPU_LAYER		= 0x10,
	DC_BLK_TYPE_LPU_WB_LAYER	= 0x11,
	DC_BLK_TYPE_CU_SPLITTER	= 0x20,
	DC_BLK_TYPE_CU_SCALER		= 0x21,
	DC_BLK_TYPE_CU_MERGER		= 0x22,
	DC_BLK_TYPE_DOU_IPS		= 0x30,
	DC_BLK_TYPE_DOU_BS		= 0x31,
	DC_BLK_TYPE_DOU_FT_COEFF	= 0x32,
	DC_BLK_TYPE_AEU_DS		= 0x40,
	DC_BLK_TYPE_AEU_AES		= 0x41,
	DC_BLK_TYPE_RESERVED		= 0xFF
};

struct dc_pipeline {
	struct bst_pipeline base;
	u32 __iomem	*lpu_addr;
	u32 __iomem	*cu_addr;
	u32 __iomem	*dou_addr;
	u32 __iomem	*disp_dou_ft_coeff_addr;
	bool disp_flush_done;
	struct completion *disp_force_eow_done;
	spinlock_t disp_force_wb_lock;
};
struct bst_crtc;
struct dc_dev {
	struct bst_dev *mdev;
	int	blocks_num;
	int	pipelines_num;
	int	rich_layers_num;
	u32	max_line_size;
	u32	max_vsize;
	u32	dual_link_support : 1;
	u32	integrate_tbu : 1;
	u32 __iomem	*disp_gcu_addr;
	u32 __iomem	*glb_scl_coeff_addr[DC_MAX_GLB_SCL_COEFF];
	u32 __iomem	*periph_addr;
	struct dc_pipeline *pipes[DC_MAX_PIPELINE];
	bool new_flush[DC_MAX_PIPELINE];
	struct bst_crtc* bcrtc[DC_MAX_PIPELINE];
	struct hrtimer flip_hrtimer_pipe0;
	struct hrtimer flip_hrtimer_pipe1;
	ktime_t framedur_ns[DC_MAX_PIPELINE];
	bool timer_inited[DC_MAX_PIPELINE];
};

struct block_header {
	u32 block_info;
	u32 pipeline_info;
	u32 input_ids[DC_BLOCK_MAX_INPUT];
	u32 output_ids[DC_BLOCK_MAX_OUTPUT];
};

#define to_dc_pipeline(x)	container_of(x, struct dc_pipeline, base)

extern const struct bst_pipeline_funcs dc_pipeline_funcs;

int dc_probe_block(struct dc_dev *dc,
		    struct block_header *blk, u32 __iomem *reg);
void dc_read_block_header(u32 __iomem *reg, struct block_header *blk);

void dc_dump(struct bst_dev *mdev, struct seq_file *sf);

#endif /* !_DC_DEV_H_ */
