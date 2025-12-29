// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef BST_DISPLAY_PLATFORM_H
#define BST_DISPLAY_PLATFORM_H

#include "bst_display_osal.h"

#ifdef BST_PLATFORM_C1200
#define BST_PLATFORM_ID      ((uint32_t)BST_PLATFORM_C1200)

//#define DISPLAY_SUPPORT_SCALE

#ifndef __KERNEL__
#include "bst_display_common.h"
#endif

#ifdef __KERNEL__
//#define ADAS_OS_MAGIC   (0x41444153)
//#define IVI_OS_MAGIC    (0x49564920)
//#define DB_OS_MAGIC     (0x44422020)
//#define REAL_OS_MAGIC   (0x5245414c)
//#define SAFE_OS_MAGIC   (0x53414645)

enum bst_subdev_type {
	BST_SUBDEV_NONE = 0x0,

	// BST_SUBDEV_DC_PIPE_START,
	BST_SUBDEV_DC0_PIPE0/*  = BST_SUBDEV_DC_PIPE_START */,
	BST_SUBDEV_DC0_PIPE1,
	BST_SUBDEV_DC1_PIPE0,
	BST_SUBDEV_DC1_PIPE1,
	BST_SUBDEV_DC2_PIPE0,
	// BST_SUBDEV_DC_PIPE_END = BST_SUBDEV_DC2_PIPE0,

	// BST_SUBDEV_CONNECTOR_START,
	BST_SUBDEV_eDP/*  = BST_SUBDEV_CONNECTOR_START */,
	BST_SUBDEV_DSI0,
	BST_SUBDEV_DSI1,
	BST_SUBDEV_LVDS0,
	BST_SUBDEV_LVDS1,
	// BST_SUBDEV_CONNECTOR_END = BST_SUBDEV_LVDS1,

	BST_SUBDEV_MAX,
	BST_SUBDEV_INVAL = 0xFF
};

#define BST_SUBDEV_DC_PIPE_START      (BST_SUBDEV_DC0_PIPE0)
#define BST_SUBDEV_DC_PIPE_END        (BST_SUBDEV_DC2_PIPE0)
#define BST_SUBDEV_CONNECTOR_START    (BST_SUBDEV_eDP)
#define BST_SUBDEV_CONNECTOR_END      (BST_SUBDEV_LVDS1)

#endif

enum conn_submodule_type {
	CONN_SUBMODULE_TYPE_NONE = 0x0,
	CONN_SUBMODULE_TYPE_VIDEO,
	CONN_SUBMODULE_TYPE_AUDIO,
	CONN_SUBMODULE_TYPE_MAX
};
#define BST_EVENT_NONE 		  0U
#define BST_EVENT_VSYNC       BIT(0)
#define BST_EVENT_FLIP        BIT(1)
#define BST_EVENT_EOW         BIT(2)
#define BST_EVENT_HOTPLUG     BIT(3)
#define BST_EVENT_UNHOTPLUG   BIT(4)
#define BST_EVENT_MAX         0xFFFFU

enum dc_submodule_type {
	DC_SUBMODULE_TYPE_NONE = 0x0,
	DC_SUBMODULE_TYPE_LAYER,
	DC_SUBMODULE_TYPE_WB_LAYER,
	DC_SUBMODULE_TYPE_TRUST_LAYER,
	DC_SUBMODULE_TYPE_COMPOSER,
	DC_SUBMODULE_TYPE_PIPE,
	DC_SUBMODULE_TYPE_MAX,
};

enum dc_submodule_id {
	SUBMODULE_ID_DC_INVAILD = 0x0,
	// SUBMODULE_ID_DC_LAYER_START,
	SUBMODULE_ID_DC_LAYER0/*  = SUBMODULE_ID_DC_LAYER_START */,
	SUBMODULE_ID_DC_LAYER1,
	SUBMODULE_ID_DC_LAYER2,
	SUBMODULE_ID_DC_LAYER3,
	// SUBMODULE_ID_DC_LAYER_END = SUBMODULE_ID_DC_LAYER3,
	SUBMODULE_ID_DC_WB_LAYER,
	SUBMODULE_ID_DC_TRUST_LAYER,
	SUBMODULE_ID_DC_COMPOSER,
	SUBMODULE_ID_DC_PIPE_STR,
	SUBMODULE_ID_DC_MAX,
};

#define SUBMODULE_ID_DC_LAYER_START (SUBMODULE_ID_DC_LAYER0)
#define SUBMODULE_ID_DC_LAYER_END   (SUBMODULE_ID_DC_LAYER3)

enum dc_scaler_id {
	SUBMODULE_ID_DC_SCALER_INVALID = 0x0,
	SUBMODULE_ID_DC_SCALER0,
	SUBMODULE_ID_DC_SCALER1,
	// SUBMODULE_ID_DC_SCALER_NUM = SUBMODULE_ID_DC_SCALER1,
};
#define SUBMODULE_ID_DC_SCALER_NUM (SUBMODULE_ID_DC_SCALER1)
#define IS_DC_SCALER_SUBM_ID(x) (((x) >= SUBMODULE_ID_DC_SCALER0) && ((x) <= SUBMODULE_ID_DC_SCALER1))

#define FIXED_DC_SUBMODULE_NUM   (2) /* WB_LAYER and COMPOSER */
#define DC_LAYER_NULL            (0xff)

#define MAX_PIPE_NUM             ((uint8_t)BST_SUBDEV_DC_PIPE_END - (uint8_t)BST_SUBDEV_DC_PIPE_START + 1U)
#define MAX_LAYER_NUM_PER_PIPE   ((uint8_t)SUBMODULE_ID_DC_LAYER_END - (uint8_t)SUBMODULE_ID_DC_LAYER_START + 1U)
#define ALL_LAYER_IDX_MASK       (0xfU)
#define MAX_CU_INPUT_NUM         ((uint8_t)MAX_LAYER_NUM_PER_PIPE + 1U) //4layer + other dc_composer

enum dp_submodule_id {
	SUBMODULE_ID_DP_INVAILD = 0x0,
	SUBMODULE_ID_DP_VIDEO,
	//SUBMODULE_ID_DP_AUDIO,
	SUBMODULE_ID_DP_MAX,
};

enum lvds_submodule_id {
	SUBMODULE_ID_LVDS_INVAILD = 0x0,
	SUBMODULE_ID_LVDS_VIDEO,
	SUBMODULE_ID_LVDS_MAX,
};

enum mipi_submodule_id {
	SUBMODULE_ID_MIPI_INVAILD = 0x0,
	SUBMODULE_ID_MIPI_VIDEO,
	SUBMODULE_ID_MIPI_MAX,
};

#define SUBMODULE_INFO_SUBMODULE_ID(x) (((x)&0x00FFU) >> 0U)
#define SUBMODULE_INFO_SUBMODULE_TYPE(x) (((x)&0xFF00U) >> 8U)
#define SUBMODULE_INFO_SUBMODULE_VERSION(x) (((x)&0xFFFF00000U) >> 16U)
#define PIPELINE_INFO_N_OUTPUTS(x) ((x)&0x000FU)
#define PIPELINE_INFO_N_VALID_INPUTS(x) (((x)&0x0F00U) >> 8)

#define SUBMODULE_IDS_MAX      (MAX3((int)SUBMODULE_ID_DC_MAX, (int)SUBMODULE_ID_DP_MAX, \
                               MAX2((int)SUBMODULE_ID_LVDS_MAX, (int)SUBMODULE_ID_MIPI_MAX)))

#define PIPE_SUBDEV_TO_IDX(pipe_subdev)		((pipe_subdev) - (uint8_t)BST_SUBDEV_DC_PIPE_START)
#define LAYER_SUBID_TO_IDX(layer_subid)		((layer_subid) - (uint8_t)SUBMODULE_ID_DC_LAYER_START)
#define LAYER_IDX_TO_SUBID(layer_idx)		((layer_idx) + (uint8_t)SUBMODULE_ID_DC_LAYER_START)

#define IS_DC_SUBDEV_TYPE(x)                                                \
	({                                                                  \
		uint8_t ret;                                                \
		if ((x) >= BST_SUBDEV_DC0_PIPE0 && (x) <= BST_SUBDEV_DC2_PIPE0) {\
			ret = true;                                         \
		}                                                             \
		else {                                                       \
			ret = false;                                        \
		}                                                             \
		ret;                                                        \
	})
#define IS_CONNECTOR_SUBDEV_TYPE(x)                                                \
	({                                                                  \
		uint8_t ret;                                                \
		if ((x) >= BST_SUBDEV_eDP && (x) < BST_SUBDEV_MAX) { \
			ret = true;                                         \
		}                                                             \
		else {                                                       \
			ret = false;                                        \
		}                                                             \
		ret;                                                        \
	})

#define IS_DC_DISP_LAYER(x) (((x) >= SUBMODULE_ID_DC_LAYER_START) && ((x) <= SUBMODULE_ID_DC_LAYER_END))

static inline bool is_rich_layer(uint8_t layer_submodule_id)
{
	if ((layer_submodule_id == (uint8_t)SUBMODULE_ID_DC_LAYER0)
		|| (layer_submodule_id == (uint8_t)SUBMODULE_ID_DC_LAYER2)) {
		return true;
	} else {
		return false;
	}
}

#endif /* BST_PLATFORM_C1200 */

#endif /* BST_DISPLAY_PLATFORM_H */
