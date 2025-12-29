/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_MEDIA_DEV_H__
#define __BST_MEDIA_DEV_H__

#include <dt-bindings/media/bst-isp.h>

#include <linux/videodev2.h>
#include <media/v4l2-async.h>
#include <media/v4l2-subdev.h>

/* Encode s_stream's enable value, since we use device other than
 * vc or port as a sub device.
 */
#define STREAM_ENC(vc, enable) (((vc) << 4) | (enable))
#define STREAM_DEC_VC(val)     ((val) >> 4)
#define STREAM_DEC_EN(val)     ((val) & 0x1)
#define MAX_VC_NUM	       (32)

typedef int i2cset(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 val);
typedef int i2cget(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 *val);

struct reg_cfg {
	const u32 reg;
	const u32 val;
	const u32 delay;
};

struct reg_cfgs {
	int num;
	const struct reg_cfg *cfg;
};

#define __REG_CFGS(reg_cfg_arr)                 \
	{                                       \
		.num = ARRAY_SIZE(reg_cfg_arr), \
		.cfg = reg_cfg_arr,             \
	}

struct camera_cfg {
	u32 data_type;
	u32 fps;
	u32 row_time;
	struct reg_cfgs ser_init;
	struct reg_cfgs ser_pre_stream_on;
	struct reg_cfgs ext_init;
	struct reg_cfgs sensor_reset;
	struct reg_cfgs sensor_init;
	struct reg_cfgs sensor_pre_stream_on;
	struct reg_cfgs sensor_stream_on;
	struct reg_cfgs sensor_stream_off;
};

struct camera_cfg_set {
	unsigned long reset_delay;
	int num;
	struct camera_cfg **cfgs;
};

struct camera_dev;

struct csi_tx_dev {
	u32 phy_if;
	u32 lane_num;
	u32 lane_speed;
	u32 enable;
	struct v4l2_subdev subdev;
	struct camera_dev *cameras[MAX_VC_NUM];
	void *drv_data;
};

struct camera_dev {
	/* Sensor properties */
	u32 sensor_addr;
	u32 sensor_alias;
	u32 data_type;
	u32 sensor_fps;
	u32 row_time;
	u32 reset_delay;
	u32 reg_width;
	u32 role;
	u32 id_reg;
	u32 id_val;
	const char *name;
	const char *algo;
	const char *iq;

	/* Sensor runtime */
	const struct camera_cfg_set *cfg_set;
	const struct camera_cfg *cfg;
	bool power_on;

	struct i2c_client *i2c_client;
	struct csi_tx_dev tx_dev;
	struct device *dev;
	i2cset *i2cset;
	i2cget *i2cget;

	/* Serializer properties */
	u32 ser_type;
	u32 ser_addr;
	u32 ser_alias;
	u32 ser_fsync_tx_pin;
	bool ser_reset;
	u32 ser_reg_width;
	u32 ser_role;

	/* Serializer runtime */
	i2cset *ser_i2cset;
	i2cget *ser_i2cget;

	/* Extended device properties */
	u32 ext_addr;
	u32 ext_alias;
	u32 ext_reg_width;
	u32 ext_role;

	/* Extended device runtime */
	i2cset *ext_i2cset;
	i2cget *ext_i2cget;

	bool cfg_with_delay;
};

/* V4L2 notification event */
#define ISP_EVENT_CAMERA_DISCONNECT _IOWR('B', 1, struct camera_dev)
#define ISP_EVENT_CAMERA_CONNECT    _IOWR('B', 2, struct camera_dev)

static inline bool dt_is_raw(int dt)
{
	switch (dt) {
	case DT_RAW28:
	case DT_RAW24:
	case DT_RAW6:
	case DT_RAW7:
	case DT_RAW8:
	case DT_RAW10:
	case DT_RAW12:
	case DT_RAW14:
	case DT_RAW16:
	case DT_RAW20:
		return true;
	default:
		return false;
	}
}

#endif /* __BST_MEDIA_DEV_H__ */
