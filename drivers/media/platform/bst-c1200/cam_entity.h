/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __C1200_CAM_ENTITY_H__
#define __C1200_CAM_ENTITY_H__

#include <linux/types.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#include "proto_isp_ipc.h"
#include "isp_video_uapi.h"

#define MAX_BIN_NAME_LEN     128
#define MAX_CAMERA_NAME_LEN  32
#define MAX_DTS_STRING_LEN   16
#define MAX_VIEWS_PER_CAMERA 3

#define EMBED_DATA_BUF_RESERVE_SIZE (64 * 1024)
#define YUV_STAT_BUF_EXPAND	    (1024)

#define IIC_OFFSET 0x1000
#define IIC_BASE   0x20000000

struct deser_hub_dev;

enum {
	BST_SUBDEV_STATE_UNKNOWN,
	BST_SUBDEV_STATE_ENABLED,
	BST_SUBDEV_STATE_BOUND,
	BST_SUBDEV_STATE_UNBIND,
};

// 0-mipi/1-HDMI input/2-FILE2FILE HDR/3-FILE2FILE SINGLE, from proto_isp_ipc.h
enum {
	BST_ISP_INPUT_MIPI = 0,
	BST_ISP_INPUT_HDMI,
	BST_ISP_INPUT_FILE2FILE_HDR,
	BST_ISP_INPUT_FILE2FILE_SINGLE,
};

enum {
	// 16 bit register addr and 16 bit data
	BST_SENSOR_RW_MODE_WORD_REG_WORD_DATA = 0,
	// 16 bit register addr and 8 bit data
	BST_SENSOR_RW_MODE_WORD_REG_BYTE_DATA,
	// 8 bit register addr and 8 bit data
	BST_SENSOR_RW_MODE_BYTE_REG_BYTE_DATA,
};

enum {
	BST_SENSOR_TYPE_OV2770_RAW = 0,
	BST_SENSOR_TYPE_OV10640_RAW = 1,
	BST_SENSOR_TYPE_AR0231_RAW = 2,
	BST_SENSOR_TYPE_ASX340_RAW = 3,
	BST_SENSOR_TYPE_AR0144_RAW = 5,
	BST_SENSOR_TYPE_OV2311_RAW = 6,
	BST_SENSOR_TYPE_IMX390_RAW = 8,
	BST_SENSOR_TYPE_AR0233_RAW = 9,
	BST_SENSOR_TYPE_YUV422 = 15,
	BST_SENSOR_TYPE_IMX424_RAW = 16,
	BST_SENSOR_TYPE_OV10652_RAW = 128,
	BST_SENSOR_TYPE_OX3C_RAW = 136,
	BST_SENSOR_TYPE_OX08B_RAW = 137,
	BST_SENSOR_TYPE_OX3F_RAW = 138,
};

struct camera_dev;

struct camera_ops {
	int (*s_register)(struct camera_dev *cam, uint16_t regaddr,
			  uint16_t regval);
	int (*g_register)(struct camera_dev *cam, uint16_t regaddr,
			  uint16_t *regval);
	int (*s_brightness)(struct camera_dev *cam, int val);
	int (*g_brightness)(struct camera_dev *cam, int *val);
	int (*s_contrast)(struct camera_dev *cam, int val);
	int (*g_contrast)(struct camera_dev *cam, int *val);
	int (*s_saturation)(struct camera_dev *cam, int val);
	int (*g_saturation)(struct camera_dev *cam, int *val);
	int (*s_hue)(struct camera_dev *cam, int val);
	int (*g_hue)(struct camera_dev *cam, int *val);
	int (*s_aec)(struct camera_dev *cam, int val);
	int (*g_aec)(struct camera_dev *cam, int *val);
	int (*s_awb)(struct camera_dev *cam, int val);
	int (*g_awb)(struct camera_dev *cam, struct v4l2_ctrl *val);
	int (*s_gamma)(struct camera_dev *cam, int val);
	int (*g_gamma)(struct camera_dev *cam, struct v4l2_ctrl *val);
};

struct camera_dev {
	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct i2c_client *i2c_client;
	struct device *dev;
	ipc_reconf_t isp_data;
	char camera_name[MAX_CAMERA_NAME_LEN];
	char algo[MAX_BIN_NAME_LEN];
	char iq[MAX_BIN_NAME_LEN];
	char pwl_lut[MAX_BIN_NAME_LEN];
	char type_name[MAX_DTS_STRING_LEN];
	int ser_alias_id;
	int sensor_id;
	int sensor_alias_id;
	int index_in_serdes;
	u32 parent_iic_address;
	// the deserializer that camera connected
	struct deser_hub_dev *deser_parent;
	atomic_t is_streaming;
	bool maxim_power_on;
	bool power_on;
	bool configured;
	int cap_buf_shift;
	int sd_state;
	uint32_t fv_polarity_low;
	uint32_t frame_valid_min;
	uint32_t trigger_gpio;
	char fpd3_mode[MAX_DTS_STRING_LEN];
	char serializer[MAX_DTS_STRING_LEN];
	struct device_node *of_node;
	struct fwnode_handle *fwnode;
	struct camera_ops ops;
	int sensor_exp;
	int row_time;
	int sensor_fps;
	int clock_frequency;
	struct isp_emd_view_info emd_view_info;
};

#endif /* __C1200_CAM_ENTITY_H__ */
