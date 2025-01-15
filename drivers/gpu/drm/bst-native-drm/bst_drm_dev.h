// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
 
#ifndef _BST_DRM_DEV_H_
#define _BST_DRM_DEV_H_

#include <linux/device.h>
#include <linux/clk.h>
#include "bst_pipeline.h"
#include "bst_product.h"
#include "bst_format_color.h"

#define BST_LIMIT_USR_PLANES (4)
#define BST_BYPASS_KERNEL_QOS (1)

#define BST_DRM_EVENT_VSYNC		BIT_ULL(0)
#define BST_DRM_EVENT_FLIP		BIT_ULL(1)
#define BST_DRM_EVENT_URUN		BIT_ULL(2)
#define BST_DRM_EVENT_IBSY		BIT_ULL(3)
#define BST_DRM_EVENT_OVR		BIT_ULL(4)
#define BST_DRM_EVENT_EOW		BIT_ULL(5)
#define BST_DRM_EVENT_MODE		BIT_ULL(6)
#define BST_DRM_EVENT_FULL		BIT_ULL(7)
#define BST_DRM_EVENT_EMPTY		BIT_ULL(8)

#define BST_DRM_ERR_TETO			BIT_ULL(14)
#define BST_DRM_ERR_TEMR			BIT_ULL(15)
#define BST_DRM_ERR_TITR			BIT_ULL(16)
#define BST_DRM_ERR_CPE			BIT_ULL(17)
#define BST_DRM_ERR_CFGE			BIT_ULL(18)
#define BST_DRM_ERR_AXIE			BIT_ULL(19)
#define BST_DRM_ERR_ACE0			BIT_ULL(20)
#define BST_DRM_ERR_ACE1			BIT_ULL(21)
#define BST_DRM_ERR_ACE2			BIT_ULL(22)
#define BST_DRM_ERR_ACE3			BIT_ULL(23)
#define BST_DRM_ERR_DRIFTTO		BIT_ULL(24)
#define BST_DRM_ERR_FRAMETO		BIT_ULL(25)
#define BST_DRM_ERR_CSCE			BIT_ULL(26)
#define BST_DRM_ERR_ZME			BIT_ULL(27)
#define BST_DRM_ERR_MERR			BIT_ULL(28)
#define BST_DRM_ERR_TCF			BIT_ULL(29)
#define BST_DRM_ERR_TTNG			BIT_ULL(30)
#define BST_DRM_ERR_TTF			BIT_ULL(31)

#define BST_DRM_ERR_EVENTS	\
	(BST_DRM_EVENT_URUN	| BST_DRM_EVENT_IBSY	| BST_DRM_EVENT_OVR |\
	BST_DRM_ERR_TETO		| BST_DRM_ERR_TEMR	| BST_DRM_ERR_TITR |\
	BST_DRM_ERR_CPE		| BST_DRM_ERR_CFGE	| BST_DRM_ERR_AXIE |\
	BST_DRM_ERR_ACE0		| BST_DRM_ERR_ACE1	| BST_DRM_ERR_ACE2 |\
	BST_DRM_ERR_ACE3		| BST_DRM_ERR_DRIFTTO	| BST_DRM_ERR_FRAMETO |\
	BST_DRM_ERR_ZME		| BST_DRM_ERR_MERR	| BST_DRM_ERR_TCF |\
	BST_DRM_ERR_TTNG		| BST_DRM_ERR_TTF)

#define BST_DRM_WARN_EVENTS	\
	(BST_DRM_ERR_CSCE | BST_DRM_EVENT_FULL | BST_DRM_EVENT_EMPTY)

#define BST_DRM_INFO_EVENTS (0 \
			    | BST_DRM_EVENT_VSYNC \
			    | BST_DRM_EVENT_FLIP \
			    | BST_DRM_EVENT_EOW \
			    | BST_DRM_EVENT_MODE \
			    )

/*
 * this value range from 1 ~ 10, typical value is 2
 * user can turning this value.
 */
#define BST_DRM_HW_FLUSH_DELAY_RAITO  (2)

enum {
	BST_DRM_OF_PORT_OUTPUT		= 0,
	BST_DRM_OF_PORT_COPROC		= 1,
};

enum {
	BST_DRM_DISPLAY_ID_INVAL    = 0,
	BST_DRM_DISPLAY_ID_0		= 1,
	BST_DRM_DISPLAY_ID_1		= 2,
	BST_DRM_DISPLAY_ID_2		= 3,
};

struct bst_chip_info {
	u32 arch_id;
	u32 core_id;
	u32 core_info;
	u32 bus_width;
	u8 display_id;
};

struct dpu_layer_cfg {
	uint8_t layer_en : 1,
			is_va : 1,
			is_yuv : 1;
	uint8_t layer_rot;
	uint8_t layer_flip;
	uint8_t  pixel_format;
	uint16_t hsize;
	uint16_t vsize;
	uint64_t p0_ptr;
	uint64_t p1_ptr;
	uint64_t p2_ptr;
	uint16_t p0_stride;
	uint16_t p1_stride;
	uint8_t  num_planes;
	uint32_t yuv2rgb_coeffs[12];
};

#define DPU_CU0  (0x200)
#define DPU_CU1  (0x210)
struct dpu_wb_cfg {
	uint32_t active_input;
	struct dpu_layer_cfg layer_cfg;
};

struct bst_drm_resv_mem_ops;
struct bst_dev;

struct bst_events {
	u64 global;
	u64 pipes[BST_DRM_MAX_PIPELINES];
};
struct bst_crtc;
struct bst_dev_funcs {
	void (*init_format_table)(struct bst_dev *mdev);
	int (*enum_resources)(struct bst_dev *mdev);
	void (*cleanup)(struct bst_dev *mdev);
	int (*connect_iommu)(struct bst_dev *mdev);
	int (*disconnect_iommu)(struct bst_dev *mdev);
	irqreturn_t (*irq_handler)(struct bst_dev *mdev,
				   struct bst_events *events);
	int (*enable_irq)(struct bst_dev *mdev);
	int (*disable_irq)(struct bst_dev *mdev);
	void (*on_off_vblank)(struct bst_dev *mdev,
				int master_pipe, bool on,
				struct bst_crtc *bcrtc);
	void (*dump_register)(struct bst_dev *mdev, struct seq_file *seq);
	int (*change_opmode)(struct bst_dev *mdev, int new_mode);
	void (*flush)(struct bst_dev *mdev,
		      int master_pipe, u32 active_pipes);
	int (*force_pipe)(struct bst_dev *mdev, u8 pipe, bool disable);
	void (*hw_reset)(struct bst_dev *mdev);
	int (*force_writeback)(struct bst_dev *mdev, struct dpu_wb_cfg* cfg, u32 pipe);
};

enum {
	BST_DRM_MODE_INACTIVE	= 0,
	BST_DRM_MODE_DISP0	= BIT(0),
	BST_DRM_MODE_DISP1	= BIT(1),
	BST_DRM_MODE_DUAL_DISP	= BST_DRM_MODE_DISP0 | BST_DRM_MODE_DISP1,
};

struct bst_dev {
	struct device *dev;
	u32 __iomem   *reg_base;
	u32 __iomem   *csr_base;
	u32 __iomem   *mdnoc_qos_base;
	struct device_dma_parameters dma_parms;
	struct bst_chip_info chip;
	struct bst_format_caps_table fmt_tbl;
	struct clk *aclk;
	int irq;
	struct mutex lock;
	u32 dpmode;
	int n_pipelines;
	struct bst_pipeline *pipelines[BST_DRM_MAX_PIPELINES];
	const struct bst_dev_funcs *funcs;
	void *chip_data;
	struct iommu_domain *iommu;
	struct bst_drm_resv_mem_ops *resv_mem_ops;
	struct bst_drm_resv_memblock *wb_memblock[BST_DRM_MAX_PIPELINES];

	struct dentry *debugfs_root;
	u16 err_verbosity;
#define BST_DRM_DEV_PRINT_ERR_EVENTS BIT(0)
#define BST_DRM_DEV_PRINT_WARN_EVENTS BIT(1)
#define BST_DRM_DEV_PRINT_INFO_EVENTS BIT(2)
#define BST_DRM_DEV_PRINT_DUMP_STATE_ON_EVENT BIT(8)
#define BST_DRM_DEV_PRINT_DISABLE_RATELIMIT BIT(12)
	u32 pipe_update_count[2];
	u32 underrun_err_count[2];
	u32 frame_count[2];
	u64 cur_commit_time;
	u64 avg_commit_time;
	u64 commit_time_sum;
	u32 commit_counts;
	u32 cur_active_layers[2];
	u32 max_active_layers[2];
	u32 max_layers[2];
	u64 output_crc[2];
	u32 cur_cu_hsize[2];
	u32 cur_cu_vsize[2];
	u32 dump_idx[2];
	bool resume;
};

static inline bool
bst_product_match(struct bst_dev *mdev, u32 target)
{
	return BSTDC_CORE_ID_PRODUCT_ID(mdev->chip.core_id) == target;
}

typedef const struct bst_dev_funcs *
(*bst_identify_func)(struct bst_dev *mdev, struct bst_chip_info *chip, const struct bst_dev_funcs* chip_func);

const struct bst_dev_funcs *
dc_identify_display_0(struct bst_dev *mdev, struct bst_chip_info *chip, const struct bst_dev_funcs* chip_func);
const struct bst_dev_funcs *
dc_identify_display_1(struct bst_dev *mdev, struct bst_chip_info *chip, const struct bst_dev_funcs* chip_func);
const struct bst_dev_funcs *
dc_identify_display_2(struct bst_dev *mdev, struct bst_chip_info *chip, const struct bst_dev_funcs* chip_func);

struct bst_dev *bst_dev_create(struct device *dev);
void bst_dev_destroy(struct bst_dev *mdev);

struct bst_dev *dev_to_mdev(struct device *dev);

void bst_print_events(struct bst_events *evts, struct drm_device *dev);

int bst_dev_resume(struct bst_dev *mdev);
int bst_dev_suspend(struct bst_dev *mdev);

#endif /*_BST_DRM_DEV_H_*/
