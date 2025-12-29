// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef _BST_VIRT_DRM_DEV_H_
#define _BST_VIRT_DRM_DEV_H_

#include <linux/device.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include "bst_virt_format_color.h"
#include "bst_virt_pipeline.h"

#define BST_DRM_EVENT_VSYNC BIT_ULL(0)
#define BST_DRM_EVENT_FLIP BIT_ULL(1)
#define BST_DRM_EVENT_URUN BIT_ULL(2)
#define BST_DRM_EVENT_IBSY BIT_ULL(3)
#define BST_DRM_EVENT_OVR BIT_ULL(4)
#define BST_DRM_EVENT_EOW BIT_ULL(5)
#define BST_DRM_EVENT_MODE BIT_ULL(6)
#define BST_DRM_EVENT_FULL BIT_ULL(7)
#define BST_DRM_EVENT_EMPTY BIT_ULL(8)

#define BST_DRM_ERR_TETO BIT_ULL(14)
#define BST_DRM_ERR_TEMR BIT_ULL(15)
#define BST_DRM_ERR_TITR BIT_ULL(16)
#define BST_DRM_ERR_CPE BIT_ULL(17)
#define BST_DRM_ERR_CFGE BIT_ULL(18)
#define BST_DRM_ERR_AXIE BIT_ULL(19)
#define BST_DRM_ERR_ACE0 BIT_ULL(20)
#define BST_DRM_ERR_ACE1 BIT_ULL(21)
#define BST_DRM_ERR_ACE2 BIT_ULL(22)
#define BST_DRM_ERR_ACE3 BIT_ULL(23)
#define BST_DRM_ERR_DRIFTTO BIT_ULL(24)
#define BST_DRM_ERR_FRAMETO BIT_ULL(25)
#define BST_DRM_ERR_CSCE BIT_ULL(26)
#define BST_DRM_ERR_ZME BIT_ULL(27)
#define BST_DRM_ERR_MERR BIT_ULL(28)
#define BST_DRM_ERR_TCF BIT_ULL(29)
#define BST_DRM_ERR_TTNG BIT_ULL(30)
#define BST_DRM_ERR_TTF BIT_ULL(31)

#define BST_DRM_ERR_EVENTS                                              \
	(BST_DRM_EVENT_URUN | BST_DRM_EVENT_IBSY | BST_DRM_EVENT_OVR |  \
	 BST_DRM_ERR_TETO | BST_DRM_ERR_TEMR | BST_DRM_ERR_TITR |       \
	 BST_DRM_ERR_CPE | BST_DRM_ERR_CFGE | BST_DRM_ERR_AXIE |        \
	 BST_DRM_ERR_ACE0 | BST_DRM_ERR_ACE1 | BST_DRM_ERR_ACE2 |       \
	 BST_DRM_ERR_ACE3 | BST_DRM_ERR_DRIFTTO | BST_DRM_ERR_FRAMETO | \
	 BST_DRM_ERR_ZME | BST_DRM_ERR_MERR | BST_DRM_ERR_TCF |         \
	 BST_DRM_ERR_TTNG | BST_DRM_ERR_TTF)

#define BST_DRM_WARN_EVENTS \
	(BST_DRM_ERR_CSCE | BST_DRM_EVENT_FULL | BST_DRM_EVENT_EMPTY)

#define BST_DRM_INFO_EVENTS                                                 \
	(0 | BST_DRM_EVENT_VSYNC | BST_DRM_EVENT_FLIP | BST_DRM_EVENT_EOW | \
	 BST_DRM_EVENT_MODE)

enum {
	BST_DRM_OF_PORT_OUTPUT = 0,
	BST_DRM_OF_PORT_COPROC = 1,
};

struct bst_virt_device_info {
	u32 arch_id;
	u32 core_id;
	u32 core_info;
	u32 bus_width;
	u32 subdev_session;
	u32 platform_id;
	u32 device_type;
	bool is_owner_device;
	void *private;
};

struct bst_virt_events {
	u64 global;
	u64 pipes;
};

struct bst_virt_device {
	struct device *dev;
	uint32_t subdev_session;
	uint32_t device_type;
	struct bst_virt_device_info dev_info;
	struct bst_format_caps_table* fmt_tbl;
	struct mutex lock;
	u32 dpmode;
	u32 events_mask;
	struct bst_virt_pipe *this_pipe;
	const struct bst_virt_device_funcs *funcs;
	void *virt_dev_data;
	struct iommu_domain *iommu;
	bool first_flush;
};

struct bst_virt_platform_info {
	uint32_t device_type;
	uint32_t platform_id;
	uint32_t want_layer_num;
};

struct bst_virt_device_funcs {
	int (*probe)(struct bst_virt_device *virt_dev);
	int (*update)(struct bst_virt_device *virt_dev, void *properties);
	irqreturn_t (*irq_handler)(struct bst_virt_device *virt_dev,
				   const struct bst_display_events_status status);
	int (*enable_irq)(struct bst_virt_device *virt_dev);
	int (*disable_irq)(struct bst_virt_device *virt_dev);
	void (*on_off_vblank)(struct bst_virt_device *virt_dev, bool on,
					struct bst_crtc *bcrtc);
	int (*connect_iommu)(struct bst_virt_device *virt_dev);
	int (*disconnect_iommu)(struct bst_virt_device *virt_dev);
	void (*cleanup)(struct bst_virt_device *virt_dev);
	void (*flush)(struct bst_virt_device *virt_dev);
	void (*debug_dump)(struct bst_virt_device *virt_dev, struct seq_file *sf);
	int (*suspend)(struct bst_virt_device *virt_dev);
	int (*resume)(struct bst_virt_device *virt_dev);
};

struct bst_super_device_info {
	uint32_t device_map[BST_VIRT_MAX_PIPELINES][BST_VIRT_MAX_SUBDEV_OF_1PIPE];
	uint8_t  want_layers_num[BST_VIRT_MAX_PIPELINES];
	uint8_t n_pipelines;
	uint32_t client_id;
	uint32_t platform_id;
	struct device_node *pipe_np_port0[BST_VIRT_MAX_PIPELINES];
	struct device_node *pipe_np_port1[BST_VIRT_MAX_PIPELINES];
};

struct bst_super_device {
	struct device *dev;
	struct device_dma_parameters dma_parms;
	struct bst_virt_device
		*subdevs[BST_VIRT_MAX_PIPELINES][BST_VIRT_MAX_SUBDEV_OF_1PIPE];
	int n_pipelines;
	struct bst_virt_pipe *pipelines[BST_VIRT_MAX_PIPELINES];
	struct dentry *debugfs_root;
	struct bst_format_caps_table fmt_tbl;
	struct bst_super_device_info super_info;
	u32 bus_width;
	u16 err_verbosity;
#define BST_DRM_DEV_PRINT_ERR_EVENTS BIT(0)
#define BST_DRM_DEV_PRINT_WARN_EVENTS BIT(1)
#define BST_DRM_DEV_PRINT_INFO_EVENTS BIT(2)
#define BST_DRM_DEV_PRINT_DUMP_STATE_ON_EVENT BIT(8)
#define BST_DRM_DEV_PRINT_DISABLE_RATELIMIT BIT(12)
#define BST_VIRT_DRM_DEV_PRINT_DBG BIT(13)
};

enum VIRT_DEVICE_TYPE_e {
	DEVICE_TYPE_VIRT_NONE = 0,
	DEVICE_TYPE_VIRT_DC_PIPE0,
	DEVICE_TYPE_VIRT_DC_PIPE1,
	DEVICE_TYPE_VIRT_DC_PIPE2,
	DEVICE_TYPE_VIRT_DC_PIPE3,
	DEVICE_TYPE_VIRT_DC_PIPE4,
	DEVICE_TYPE_VIRT_DP,
	DEVICE_TYPE_VIRT_DSI0,
	DEVICE_TYPE_VIRT_DSI1,
	DEVICE_TYPE_VIRT_LVDS0,
	DEVICE_TYPE_VIRT_LVDS1,
	DEVICE_TYPE_VIRT_DUAL_LVDS,
	DEVICE_TYPE_VIRT_MAX_NUM
};

static inline int get_remote_node_to_virt_device(const struct device_node *node)
{
	int ret;
	ret = of_device_is_compatible(node, "bst,virt-dp");
	if (ret > 0)
		return DEVICE_TYPE_VIRT_DP;
	ret = of_device_is_compatible(node, "bst,virt-lvds0");
	if (ret > 0)
		return DEVICE_TYPE_VIRT_LVDS0;
	ret = of_device_is_compatible(node, "bst,virt-lvds1");
	if (ret > 0)
		return DEVICE_TYPE_VIRT_LVDS1;
	ret = of_device_is_compatible(node, "bst,virt-dsi0");
	if (ret > 0)
		return DEVICE_TYPE_VIRT_DSI0;
	ret = of_device_is_compatible(node, "bst,virt-dsi1");
	if (ret > 0)
		return DEVICE_TYPE_VIRT_DSI1;
	else
		return DEVICE_TYPE_VIRT_NONE;
}

static inline bool is_dc_device(uint32_t device_type)
{
	if ((device_type >= DEVICE_TYPE_VIRT_DC_PIPE0) &&
	    (device_type <= DEVICE_TYPE_VIRT_DC_PIPE4))
		return true;

	return false;
}

static inline bool is_valid_device(uint32_t device_type)
{
	if ((device_type > DEVICE_TYPE_VIRT_NONE) &&
	    (device_type < DEVICE_TYPE_VIRT_MAX_NUM))
		return true;

	return false;
}

static inline bool is_valid_connector(uint32_t dev_type)
{
	if ((dev_type >= DEVICE_TYPE_VIRT_DP) &&
	    (dev_type < DEVICE_TYPE_VIRT_MAX_NUM))
		return true;

	return false;
}
static inline uint32_t to_fw_subdev_type(uint32_t virt_drm_dev_type)
{
	uint32_t subdev_type;

	switch (virt_drm_dev_type) {
	case DEVICE_TYPE_VIRT_DC_PIPE0:
		subdev_type = BST_SUBDEV_DC0_PIPE0;
		break;
	case DEVICE_TYPE_VIRT_DC_PIPE1:
		subdev_type = BST_SUBDEV_DC0_PIPE1;
		break;
	case DEVICE_TYPE_VIRT_DC_PIPE2:
		subdev_type = BST_SUBDEV_DC1_PIPE0;
		break;
	case DEVICE_TYPE_VIRT_DC_PIPE3:
		subdev_type = BST_SUBDEV_DC1_PIPE1;
		break;
	case DEVICE_TYPE_VIRT_DC_PIPE4:
		subdev_type = BST_SUBDEV_DC2_PIPE0;
		break;
	case DEVICE_TYPE_VIRT_DP:
		subdev_type = BST_SUBDEV_eDP;
		break;
	case DEVICE_TYPE_VIRT_DSI0:
		subdev_type = BST_SUBDEV_DSI0;
		break;
	case DEVICE_TYPE_VIRT_DSI1:
		subdev_type = BST_SUBDEV_DSI1;
		break;
	case DEVICE_TYPE_VIRT_LVDS0:
	case DEVICE_TYPE_VIRT_DUAL_LVDS:
		subdev_type = BST_SUBDEV_LVDS0;
		break;
	case DEVICE_TYPE_VIRT_LVDS1:
		subdev_type = BST_SUBDEV_LVDS1;
		break;
	default:
		subdev_type = BST_SUBDEV_INVAL;
	}

	return subdev_type;
}

static inline uint8_t to_virt_device_type(uint8_t subdev)
{
	uint8_t virt_dev_type;

	switch (subdev) {
		case BST_SUBDEV_DC0_PIPE0:
			virt_dev_type = DEVICE_TYPE_VIRT_DC_PIPE0;
			break;
		case BST_SUBDEV_DC0_PIPE1:
			virt_dev_type = DEVICE_TYPE_VIRT_DC_PIPE1;
			break;
		case BST_SUBDEV_DC1_PIPE0:
			virt_dev_type = DEVICE_TYPE_VIRT_DC_PIPE2;
			break;
		case BST_SUBDEV_DC1_PIPE1:
			virt_dev_type = DEVICE_TYPE_VIRT_DC_PIPE3;
			break;
		case BST_SUBDEV_DC2_PIPE0:
			virt_dev_type = DEVICE_TYPE_VIRT_DC_PIPE4;
			break;
		case BST_SUBDEV_eDP:
			virt_dev_type = DEVICE_TYPE_VIRT_DP;
			break;
		case BST_SUBDEV_DSI0:
			virt_dev_type = DEVICE_TYPE_VIRT_DSI0;
			break;
		case BST_SUBDEV_DSI1:
			virt_dev_type = DEVICE_TYPE_VIRT_DSI1;
			break;
		case BST_SUBDEV_LVDS0:
			virt_dev_type = DEVICE_TYPE_VIRT_LVDS0;
			break;
		case BST_SUBDEV_LVDS1:
			virt_dev_type = DEVICE_TYPE_VIRT_LVDS1;
			break;
		default:
			virt_dev_type = DEVICE_TYPE_VIRT_NONE;
			break;
	}

	return virt_dev_type;
}

extern const struct bst_virt_device_funcs virt_shared_conn_dev_funcs;

struct bst_super_device *bst_virt_dev_create(struct device *dev);
struct bst_virt_device *bst_virt_create_subdevice(struct device *dev,
			  struct bst_virt_platform_info *plat_info,
			  struct bst_virt_pipe *pipe);
void bst_virt_dev_destroy(struct bst_super_device *super_dev);
int bst_virt_dev_suspend(struct bst_super_device *super_dev);
int bst_virt_connector_suspend(struct bst_super_device *super_dev);
int bst_virt_connector_resume(struct bst_super_device *super_dev);
int bst_virt_dev_resume(struct bst_super_device *super_dev);
void bst_virt_print_events(struct bst_virt_events *evts, struct drm_device *dev);
int bst_virt_dev_request_irq(struct bst_super_device *super_dev);
struct bst_super_device *dev_to_super_dev(struct device *dev);

#endif /*_BST_VIRT_DRM_DEV_H_*/
