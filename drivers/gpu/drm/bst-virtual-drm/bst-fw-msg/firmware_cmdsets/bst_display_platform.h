// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef _BST_DISPLAY_PLATFORM_H_
#define _BST_DISPLAY_PLATFORM_H_

#include "bst_display_osal.h"

#ifdef BST_PLATFORM_C1200
#define BST_PLATFORM_ID      (BST_PLATFORM_C1200)

enum bst_subdev_type {
	BST_SUBDEV_NONE = 0x0,

	BST_SUBDEV_DC_PIPE_START,
	BST_SUBDEV_DC0_PIPE0 = BST_SUBDEV_DC_PIPE_START,
	BST_SUBDEV_DC0_PIPE1,
	BST_SUBDEV_DC1_PIPE0,
	BST_SUBDEV_DC1_PIPE1,
	BST_SUBDEV_DC2_PIPE0,
	BST_SUBDEV_DC_PIPE_END = BST_SUBDEV_DC2_PIPE0,

	BST_SUBDEV_CONNECTOR_START,
	BST_SUBDEV_eDP = BST_SUBDEV_CONNECTOR_START,
	BST_SUBDEV_DSI0,
	BST_SUBDEV_DSI1,
	BST_SUBDEV_LVDS0,
	BST_SUBDEV_LVDS1,
	BST_SUBDEV_CONNECTOR_END = BST_SUBDEV_LVDS1,

	BST_SUBDEV_MAX,
	BST_SUBDEV_INVAL = 0xFF
};

enum bst_subdev_events_type {
	BST_EVENT_NONE = 0,
	BST_EVENT_VSYNC = BIT(0),
	BST_EVENT_FLIP = BIT(1),
	BST_EVENT_EOW = BIT(2),
	BST_EVENT_HOTPLUG = BIT(3),
	BST_EVENT_UNHOTPLUG = BIT(4),
	BST_EVENT_MAX = 0xFFFF,
};

#define ADAS_LINUX_OS_MAGIC  (0xABCD1234)
#define IVI_ANDROID_OS_MAGIC (0xBADC2143)
#define DB_QNX_OS_MAGIC      (0xA1B2C3D4)
#define DB_LINUX_OS_MAGIC    (0x1A2B3C4D)
#define RT_RTOS_OS_MAGIC     (0xA12BC34D)
#define SF_RTOS_OS_MAGIC     (0xA21BC43D)

enum dc_submodule_type {
	DC_SUBMODULE_TYPE_NONE = 0x0,
	DC_SUBMODULE_TYPE_LAYER,
	DC_SUBMODULE_TYPE_WB_LAYER,
	DC_SUBMODULE_TYPE_TRUST_LAYER,
	DC_SUBMODULE_TYPE_COMPOSER,
	DC_SUBMODULE_TYPE_MAX,
};

enum dp_submodule_type {
	DP_SUBMODULE_TYPE_DP_NONE = 0x0,
	DP_SUBMODULE_TYPE_DP_VIDEO,
	DP_SUBMODULE_TYPE_DP_AUDIO,
	DP_SUBMODULE_TYPE_MAX
};

enum lvds_submodule_type {
	LVDS_SUBMODULE_TYPE_NONE = 0x0,
	LVDS_SUBMODULE_TYPE_VIDEO,
	LVDS_SUBMODULE_TYPE_MAX
};

enum mipi_submodule_type {
	MIPI_SUBMODULE_TYPE_NONE = 0x0,
	MIPI_SUBMODULE_TYPE_VIDEO,
	MIPI_SUBMODULE_TYPE_MAX
};


#define FIXED_DC_SUBMODULE_NUM   (2) /* WB_LAYER and COMPOSER */
#define BST_MAX_PIPE_NUM         (5)
#define BST_MAX_1PIPE_LAYERS     (BST_C1200_MAX_1PIPE_LAYERS)
#define DC_LAYER_NULL            (0xff)
#define DC_SCALER_NULL           (0xff)
#define DC_SCALER_CH0            (0)
#define DC_SCALER_CH1            (1)
#define DC_SCALER_NUM            (2)

enum dc_submodule_id {
	SUBMODULE_ID_DC_INVAILD = 0x0,
	SUBMODULE_ID_DC_LAYER0,
	SUBMODULE_ID_DC_LAYER1,
	SUBMODULE_ID_DC_LAYER2,
	SUBMODULE_ID_DC_LAYER3,
	SUBMODULE_ID_DC_WB_LAYER,
	SUBMODULE_ID_DC_TRUST_LAYER,
	SUBMODULE_ID_DC_COMPOSER,
	SUBMODULE_ID_DC_MAX,
};

enum dp_submodule_id {
	SUBMODULE_ID_DP_INVAILD = 0x0,
	SUBMODULE_ID_DP_VIDEO,
	SUBMODULE_ID_DP_AUDIO,
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

enum client_role_type {
	CLIENT_ROLE_INVALID,
	CLIENT_ROLE_OWNER,
	CLIENT_ROLE_NOT_OWNER,
};

#define SUBMODULE_INFO_SUBMODULE_ID(x) (((x)&0x00FF) >> 0)
#define SUBMODULE_INFO_SUBMODULE_TYPE(x) (((x)&0xFF00) >> 8)
#define SUBMODULE_INFO_SUBMODULE_VERSION(x) (((x)&0xFFFF00000) >> 16)
#define PIPELINE_INFO_N_OUTPUTS(x) ((x)&0x000F)
#define PIPELINE_INFO_N_VALID_INPUTS(x) (((x)&0x0F00) >> 8)

#define MAX_CLIENT_NUM         (8)
#define SUBMODULE_IDS_MAX      (MAX3((int)SUBMODULE_ID_DC_MAX, (int)SUBMODULE_ID_DP_MAX, \
                               MAX2((int)SUBMODULE_ID_LVDS_MAX, (int)SUBMODULE_ID_MIPI_MAX)))

#define IS_DC_SUBDEV_TYPE(x)                                                \
	({                                                                  \
		uint8_t ret;                                                \
		if (x >= BST_SUBDEV_DC0_PIPE0 && x <= BST_SUBDEV_DC2_PIPE0) \
			ret = true;                                         \
		else                                                        \
			ret = false;                                        \
		ret;                                                        \
	})
#define IS_CONNECTOR_SUBDEV_TYPE(x)                                                \
	({                                                                  \
		uint8_t ret;                                                \
		if (x >= BST_SUBDEV_eDP && x < BST_SUBDEV_MAX) \
			ret = true;                                         \
		else                                                        \
			ret = false;                                        \
		ret;                                                        \
	})
#endif /* BST_PLATFORM_C1200 */

#endif /* _BST_DISPLAY_PLATFORM_H_ */
