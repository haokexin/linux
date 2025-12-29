/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_UTILS_H_
#define __BST_UTILS_H_

#include <linux/delay.h>
#include <linux/i2c.h>

#include <bst/media-dev.h>

#define I2C_OP_TRIES (5)
#define I2C_OP_DELAY (5000)
#ifdef DEFAULT_RATELIMIT_INTERVAL
#undef DEFAULT_RATELIMIT_INTERVAL
#undef DEFAULT_RATELIMIT_BURST
#endif
#define DEFAULT_RATELIMIT_INTERVAL (20 * HZ)
#define DEFAULT_RATELIMIT_BURST	   (4)

#define subdev_to_csi_tx_dev(sd) container_of(sd, struct csi_tx_dev, subdev)
#define csi_tx_dev_to_camera(tx) container_of(tx, struct camera_dev, tx_dev)
#define subdev_to_camera_dev(sd) csi_tx_dev_to_camera(subdev_to_csi_tx_dev(sd))

int i2cgetbb(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 *val);
int i2csetbb(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 val);
int i2cgetwb(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 *val);
int i2csetwb(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 val);
int i2cgetww(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 *val);
int i2csetww(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 val);
int i2cupbb(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 val,
	    u32 mask, u32 off);
int i2cupwb(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 val,
	    u32 mask, u32 off);
int i2cupww(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 val,
	    u32 mask, u32 off);
int i2cset_in_bulk(struct i2c_adapter *adap, int slave_addr, i2cset *i2cset,
		   const struct reg_cfgs *cfgs);
int i2cprobe(struct i2c_adapter *adap, int slave_addr, i2cget *i2cget);
int i2cset_from_dt(struct i2c_adapter *adap, int slave_addr, i2cset *i2cset,
		   const struct device_node *node, const char *name,
		   bool with_delay);
int csi_tx_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
			   struct v4l2_mbus_config *config);

static inline int i2cgetbbc(struct i2c_client *client, u32 reg, u32 *val)
{
	return i2cgetbb(client->adapter, client->addr, reg, val);
}

static inline int i2csetbbc(struct i2c_client *client, u32 reg, u32 val)
{
	return i2csetbb(client->adapter, client->addr, reg, val);
}

static inline int i2cgetwbc(struct i2c_client *client, u32 reg, u32 *val)
{
	return i2cgetwb(client->adapter, client->addr, reg, val);
}

static inline int i2csetwbc(struct i2c_client *client, u32 reg, u32 val)
{
	return i2csetwb(client->adapter, client->addr, reg, val);
}

static inline int i2cgetwwc(struct i2c_client *client, u32 reg, u32 *val)
{
	return i2cgetww(client->adapter, client->addr, reg, val);
}

static inline int i2csetwwc(struct i2c_client *client, u32 reg, u32 val)
{
	return i2csetww(client->adapter, client->addr, reg, val);
}

static inline int i2cupbbc(struct i2c_client *client, u32 reg, u32 val,
			   u32 mask, u32 off)
{
	return i2cupbb(client->adapter, client->addr, reg, val, mask, off);
}

static inline int i2cupwbc(struct i2c_client *client, u32 reg, u32 val,
			   u32 mask, u32 off)
{
	return i2cupwb(client->adapter, client->addr, reg, val, mask, off);
}

static inline int i2cupwwc(struct i2c_client *client, u32 reg, u32 val,
			   u32 mask, u32 off)
{
	return i2cupww(client->adapter, client->addr, reg, val, mask, off);
}

static inline int i2cprobec(struct i2c_client *client, i2cget *i2cget)
{
	return i2cprobe(client->adapter, client->addr, i2cget);
}

static inline int i2csetc_in_bulk(struct i2c_client *client, i2cset *i2cset,
				  const struct reg_cfgs *cfgs)
{
	return i2cset_in_bulk(client->adapter, client->addr, i2cset, cfgs);
}

static inline int i2csetc_from_dt(struct i2c_client *client, i2cset *i2cset,
				  const struct device_node *node,
				  const char *name, bool with_delay)
{
	return i2cset_from_dt(client->adapter, client->addr, i2cset, node, name,
			      with_delay);
}

static inline void ursleep(unsigned long delay)
{
	if (delay == 0)
		return;

	usleep_range(delay, delay + 1);
}

static const inline char *str_role(u32 role)
{
	switch (role) {
	case ROLE_MASTER:
		return "master";
	case ROLE_SLAVE:
		return "slave";
	case ROLE_AUTO:
		return "auto";
	default:
		return "unknown";
	}
}

#endif
