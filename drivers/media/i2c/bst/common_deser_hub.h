/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef _BST_COMMON_DESER_H_

#define _BST_COMMON_DESER_H_

#ifdef CONFIG_VIDEO_BST_C1200_ISP
#include "../../platform/bst-c1200/cam_entity.h"

#endif

#include <media/v4l2-subdev.h>
#include <media/v4l2-async.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>

#define MAX_CAMERAS_PER_SERDES 4
#define MAX_DESER_NAME_LEN 16

#define MAXIM_N_LINKS 4

/* set deser tigger info */
#define DESERIOC_SET_TRIGGER_INFO	 \
	_IOW('V', BASE_VIDIOC_PRIVATE + 20, struct deser_trigger_info)
/* get deser tigger info */
#define DESERIOC_GET_TRIGGER_INFO	 \
	_IOR('V', BASE_VIDIOC_PRIVATE + 21, struct deser_trigger_info)
/* enable deser tigger */
#define DESERIOC_ENABLE_ISP_TRIGGER \
	_IO('V', BASE_VIDIOC_PRIVATE + 22)
#define DESERIOC_ENABLE_REPILICATE \
	_IO('V', BASE_VIDIOC_PRIVATE + 23)

enum { DESER_TYPE_INVALID = 0,
		DESER_TYPE_TI954 = 1,
		DESER_TYPE_TI960 = 2,
		DESER_TYPE_MAX9286,
		DESER_TYPE_MAX9296,
		DESER_TYPE_MAX96712,
		DESER_TYPE_MAX96722,
		DESER_TYPE_MAX96724,
		DESER_TYPE_MAX96726
};

enum { SER_TYPE_INVALID = 0,
		SER_TYPE_MAX96701,
		SER_TYPE_MAX96705,
		SER_TYPE_MAX9295,
		SER_TYPE_MAX96717f,
};

enum {
	DESER_TRIGGER_MODE_NONE = 0,
	DESER_TRIGGER_MODE_INTERNAL = 1,
	DESER_TRIGGER_MODE_EXTERNAL = 2,
};

enum csi_phy_mode {
	CSI_CONFIG_DPHY = 0, // DPHY mode config
	CSI_CONFIG_CPHY = 1, // CPHY mode config
};

struct deser_hub_dev;

struct deser_channel {
	struct v4l2_async_subdev async_dev;
	struct deser_hub_dev *deser_dev;
	struct camera_dev *cam_dev;
	struct device_node *camera_node;
	struct fwnode_handle *camera_fwnode;
	int index;
	u32 csi_vc;
	bool camera_bound; // sub-dev bound
};

struct deser_trigger_info {
	int trigger_mode;
	int trigger_fps;
	int trigger_rx_gpio;
	int trigger_tx_gpio[4];
};

enum ctl_mode_t {
	NONE,
	FAD_CTL_MODE,
	FAD_LIS_MODE,
};

struct deser_hub_dev {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct bst_csi_device *csi_dev;
	struct v4l2_subdev subdev;
	struct v4l2_async_notifier notifier;
	struct deser_channel chn[MAX_CAMERAS_PER_SERDES];
	struct deser_trigger_info trig_info;
	struct miscdevice miscdev;
	struct mutex deser_mutex;
	char name[MAX_DESER_NAME_LEN];
	char ctl_level[MAX_DTS_STRING_LEN];
	enum ctl_mode_t ctl_mode;
	int type;
	int data_lanes_num;
	int speed;
	int max_port;
	int src_mask;
	int lane_speed;
	int i2c_port;
	int csi2_port;
	int num_cameras;
	int sd_state;
	bool deser_boot_flag;
	bool deser_stream_flag;
	bool frame_sync_enabled;
	bool fwd_sync_enabled;
	int (*internal_trigger_sync)(struct deser_hub_dev *hub, int camera_gpio, int fps);
	int (*external_trigger_sync)(struct deser_hub_dev *hub,
				     int camera_trigger_gpio,
				     int deser_trigger_gpio);
	int (*enter_csi_recover)(struct deser_hub_dev *hub);
	int (*exit_csi_recover)(struct deser_hub_dev *hub);
};


/* notification events */
#define MAXIM_DESER_LINK_HOTPLUG_START _IO('M', 1)
#define MAXIM_DESER_LINK_HOTPLUG_STOP  _IO('M', 2)
#define MAXIM_DESER_LINK_DISCONNECT    _IO('M', 3)
#define MAXIM_DESER_LINK_CONNECT       _IO('M', 4)

#define LINK_STATUS_UNKNOWN		(0)
#define LINK_STATUS_UNLOCK		(1)
#define LINK_STATUS_LOCKED		(2)
#define CHECK_LINK_PERIOD		(3000) /* by msecs */

enum {
	RGB888_DT = 0,
	RGB565_DT,
	RGB666_DT,
	YUV8_DT,	  /*  default */
	YUV10_DT,
	RAW8_DT,
	RAW10_DT,
	RAW12_DT,
	RAW14_DT,
};

//#define MAX96712_DEBUG
enum maxim_pads {
	MAXIM_SINK_LINK0,
	MAXIM_SINK_LINK1,
	MAXIM_SINK_LINK2,
	MAXIM_SINK_LINK3,
	MAXIM_SOURCE,
	MAXIM_N_PADS,
};

struct maxim_hub_sink {
	struct v4l2_async_subdev asd;
	struct v4l2_subdev	*sd;
	struct fwnode_handle	*fwnode;
};
// 9296 link mode(single link or spliter mode(link>1))
enum link_status {
	AUTO_LINK_MODE = 0,
	SPLITER_MODE = 1
};

enum link_mode {
	MAXIM_LINK_MODE_GMSL1 = 0,
	MAXIM_LINK_MODE_GMSL2 = 1,
	MAXIM_LINK_MODE_GMSL3 = 2
};

struct maxim_hub_priv {
	struct deser_hub_dev hub;
	struct media_pad pads[MAXIM_N_PADS];
	struct maxim_hub_sink sinks[MAXIM_N_LINKS];
	int des_addr;
	int n_links;
	int links_mask;
	long pixel_rate;
	int fsync_period;
	int pclk;
	int him;
	int hsync;
	int vsync;
	int bws;
	int dbl;
	int dt;
	int lane_speed;
	int csi2_port;
	int i2c_port;
	int timeout;
	int data_type;
	int serdes;
	int serial_i2c;
	u32 linkrx_rate[4];
	u64 crossbar;
	char cb[16];
	int ser_addr[MAXIM_N_LINKS];
	int ser_alias_addr[MAXIM_N_LINKS];
	int sensor_addr[MAXIM_N_LINKS];
	int sensor_alias_addr[MAXIM_N_LINKS];
	int eeprom_addr[MAXIM_N_LINKS];
	int eeprom_alias_addr[MAXIM_N_LINKS];
	const char *deser_type;
	int serial_type;
	enum link_mode link_mode;
	enum link_status link_status;
	enum csi_phy_mode phy_mode_cfg;
	int lock_gpio;
	int lock_irq;
	bool lock_irq_disabled;
	u8 link_unlock_map;
	struct delayed_work link_recover_work;
	u32 check_link_period;
};

int register_deser_miscdev(struct miscdevice *miscdev, int deser_index);
void dummp_trigger_info(struct device *dev, struct deser_trigger_info *info);
#endif // _BST_COMMON_DESER_H_
