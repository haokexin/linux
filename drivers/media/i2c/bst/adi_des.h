/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_ADI_DESERS_H_
#define __BST_ADI_DESERS_H_

#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <media/v4l2-async.h>
#include <media/v4l2-subdev.h>

#include <bst/media-dev.h>

/* clang-format off */
/* NOTE: Moving to deserializer's source file may be better */
#define MAX_RX_PORTS			(4)
#define MAX_TX_PORTS			(4)
#define MAX_PIPES			(8)
#define MAX_MFPS			(17)
#define MAX_PHY_CP			(2)
#define SCLK				(25000000)

#define GMSL1_CLINK_LOCK_TIME		(10000)
#define I2C_ADDR_VALID_TIME		(1000)
#define US_PER_MS			(1000)
#define CHECK_LOCK_PERIOD		(3000) /* by msecs */
#define CHECK_LOCK_RT_PRIO		(80)
#define CHECK_ERR_PERIOD		(1000) /* by msecs */
#define MK_VC_MAP(vc, dt)		(((vc) << 6) | (dt))
#define MK_I2C_MAP(addr)		((addr) << 1)

/* FSYNC */
#define FSYNC_OUTER_DES_RX_CFG		(0x83)
#define FSYNC_OUTER_SER_TX_CFG		(0x84)
#define FSYNC_OUTER_DES_RX_DISABLE	(0x81)
/* clang-format on */

struct adi_des;

struct rx_port {
	u32 id;
	u32 gmsl_ver;
	u32 rx_rate;
	u32 tx_mode; /* Pixel or tunnel */
	bool him; /* For GMSL 1*/
	bool cfg_with_delay;
	u32 enable;
	struct adi_des *des;

	struct device_node *node;
	struct fwnode_handle *remote_fwnode;
	struct camera_dev *cam;
	struct v4l2_async_subdev asd;
};

struct pipe {
	u32 id;
	u32 from_port;
	u32 from_sid;
	u32 from_vc;
	u32 to_vc;
	u32 to_csi;
	u32 enable;
};

/*
 * csi_pre_streamon:	Use this to latch des to LP11
 * is_des_setuped:	Use this to get setup state, must be implemented for multi-os scene
 */
struct des_ops {
	bool (*is_link_locked)(struct adi_des *des, int port);
	bool (*is_video_locked)(struct adi_des *des, int port);
	int (*csi_pre_streamon)(struct adi_des *des);
	int (*csi_stream)(struct adi_des *des, bool enable);
	int (*des_setup)(struct adi_des *des);
	bool (*is_des_setuped)(struct adi_des *des);
	int (*gmsl_setup)(struct adi_des *des, int port);
};

struct des_param {
	u32 des_type;
	u32 num_gmsl;
	u32 num_pipe;
	u32 num_csi;
	u32 num_i2c;
	u32 num_mfp;
	u32 t_lock;
	u32 t_i2c_wake;
	u32 gmsl_ver_up;
	u32 gmsl_ver_lo;
	u32 csi_lo;
	u32 csi_up;

	const struct reg_cfgs pre_gmsl;
	const struct reg_cfgs post_gmsl;
	const struct reg_cfgs pre_csi;
	const struct reg_cfgs post_csi;
};

struct adi_des {
	/* Common */
	struct device *dev;
	struct mutex lock;
	const struct des_param *param;
	const struct des_ops *ops;
	u32 role;
	int pdb_gpio;
	u32 csi_mode;
	u32 i2c_port;

	/* Flags */
	u32 cfg_with_delay : 1;
	u32 on		   : 1;
	u32 resume	   : 1;

	/* Routing */
	struct rx_port rx_ports[MAX_RX_PORTS];
	struct csi_tx_dev tx_ports[MAX_TX_PORTS];
	struct pipe pipes[MAX_PIPES];

	struct {
		u32 src;
		u32 dst;
	} phy_cps[MAX_PHY_CP];

	/* FSYNC */
	u32 fsync_mode;
	u32 fsync_fps;
	u32 fsync_rx_pin;
	u32 fsync_tx_pin; /* TODO: Keep to trigger other des */

	/* Hotplug */
	int lock_gpio;
	int lock_irq;
	bool lock_irq_enable;
	bool lock_disable;
	u32 check_lock_period;
	struct task_struct *check_lock_task;
	struct completion check_lock_comp;
	u32 link_en_map;
	u32 link_init_map; /* NOTE: use this to ensure links are initialized once */
	u32 video_lock_map;

	/* Safety */
	int err_gpio;
	int err_irq;
	bool err_irq_enable;
	bool err_disable;
	u32 check_err_period;
	struct task_struct *check_err_task;
	struct completion check_err_comp;

	/* V4L2 */
	struct v4l2_subdev *subdev;
	struct v4l2_async_notifier notifier;

	/* Cookies for I2C */
	struct i2c_client *i2c_client;
	struct i2c_adapter *i2c_adap;
};

int adi_des_parse_dt(struct adi_des *des);
int adi_des_verify_cfg(struct adi_des *des);
int adi_des_power_up(struct adi_des *des);
void adi_des_setup_links(struct adi_des *des);
int adi_des_init_v4l2_dev(struct adi_des *des);
int adi_des_init_lock_handler(struct adi_des *des);
void adi_des_exit_lock_handler(struct adi_des *des);
int adi_des_init_err_handler(struct adi_des *des);
void adi_des_exit_err_handler(struct adi_des *des);
int adi_des_set_pre_ser(struct adi_des *des, struct device_node *node,
			i2cset *i2cset, bool cfg_with_delay);
int adi_des_set_post_ser(struct adi_des *des, struct device_node *node,
			 i2cset *i2cset, bool cfg_with_delay);
int adi_des_set_pre_gmsl(struct adi_des *des, i2cset *i2cset);
int adi_des_set_post_gmsl(struct adi_des *des, i2cset *i2cset);
int adi_des_set_pre_csi(struct adi_des *des, i2cset *i2cset);
int adi_des_set_post_csi(struct adi_des *des, i2cset *i2cset);
int adi_des_sysfs_init(struct adi_des *des);
void adi_des_sysfs_exit(struct adi_des *des);
int adi_des_dt_to_bpp(int dt);
int adi_ser_set_alias(struct adi_des *des, int port);
int adi_ser_set_i2c_map(struct adi_des *des, int port);
int adi_ser_set_fsync(struct adi_des *des, int port);
int adi_ser_reset(struct adi_des *des, int port);

#endif
