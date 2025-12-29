// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#include <dt-bindings/media/bst-isp.h>

#include <media/v4l2-mediabus.h>

#include <bst/media-dev.h>

#include "utils.h"

int i2csetbb(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 val)
{
	int rv;
	int tries;
	int nmsgs;
	struct i2c_msg msgs[1];
	u8 buf[2];

	msgs[0].addr = slave_addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(buf);
	msgs[0].buf = buf;

	buf[0] = reg;
	buf[1] = val;

	nmsgs = ARRAY_SIZE(msgs);
	dev_dbg(&adap->dev, "%s %d 0x%02X 0x%02X 0x%02X\n", __func__, adap->nr,
		slave_addr, reg, val);
	for (tries = 0; tries < I2C_OP_TRIES; ++tries) {
		rv = i2c_transfer(adap, msgs, nmsgs);
		if (rv == nmsgs)
			return 0;

		usleep_range(I2C_OP_DELAY, I2C_OP_DELAY * 2);
	}

	dev_err_ratelimited(&adap->dev,
			    "%s %d 0x%02X 0x%02X 0x%02X FAILED, rv: %d\n",
			    __func__, adap->nr, slave_addr, reg, val, rv);

	return (rv < 0) ? rv : -EIO;
}

int i2cgetbb(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 *val)
{
	int rv;
	int tries;
	int nmsgs;
	struct i2c_msg msgs[2];
	u8 reg_buf[1];
	u8 val_buf[1];

	msgs[0].addr = slave_addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(reg_buf);
	msgs[0].buf = reg_buf;
	msgs[1].addr = slave_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = sizeof(val_buf);
	msgs[1].buf = val_buf;

	reg_buf[0] = (u8)(reg & 0xFF);

	nmsgs = ARRAY_SIZE(msgs);
	for (tries = 0; tries < I2C_OP_TRIES; ++tries) {
		rv = i2c_transfer(adap, msgs, nmsgs);
		if (rv == nmsgs) {
			*val = val_buf[0];
			dev_dbg(&adap->dev, "%s %d 0x%02X 0x%02X -> 0x%02X\n",
				__func__, adap->nr, slave_addr, reg, *val);

			return 0;
		}
		usleep_range(I2C_OP_DELAY, I2C_OP_DELAY * 2);
	}

	dev_err_ratelimited(&adap->dev, "%s %d 0x%02X 0x%02X FAILED, rv: %d\n",
			    __func__, adap->nr, slave_addr, reg, rv);

	return (rv < 0) ? rv : -EIO;
}

int i2csetwb(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 val)
{
	int rv;
	int tries;
	int nmsgs;
	struct i2c_msg msgs[1];
	u8 buf[3];

	msgs[0].addr = slave_addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(buf);
	msgs[0].buf = buf;

	buf[0] = (u8)((reg >> 8) & 0xFF);
	buf[1] = (u8)(reg & 0xFF);
	buf[2] = (u8)(val & 0xFF);

	nmsgs = ARRAY_SIZE(msgs);
	dev_dbg(&adap->dev, "%s %d 0x%02X 0x%04X 0x%02X\n", __func__, adap->nr,
		slave_addr, reg, val);
	for (tries = 0; tries < I2C_OP_TRIES; ++tries) {
		rv = i2c_transfer(adap, msgs, nmsgs);
		if (rv == nmsgs)
			return 0;

		usleep_range(I2C_OP_DELAY, I2C_OP_DELAY * 2);
	}

	dev_err_ratelimited(&adap->dev,
			    "%s %d 0x%02X 0x%04X 0x%02X FAILED, rv: %d\n",
			    __func__, adap->nr, slave_addr, reg, val, rv);

	return (rv < 0) ? rv : -EIO;
}

int i2cgetwb(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 *val)
{
	int rv;
	int tries;
	int nmsgs;
	struct i2c_msg msgs[2];
	u8 reg_buf[2];
	u8 val_buf[1];

	msgs[0].addr = slave_addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(reg_buf);
	msgs[0].buf = reg_buf;
	msgs[1].addr = slave_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = sizeof(val_buf);
	msgs[1].buf = val_buf;

	reg_buf[0] = (u8)((reg >> 8) & 0xFF);
	reg_buf[1] = (u8)(reg & 0xFF);

	nmsgs = ARRAY_SIZE(msgs);
	for (tries = 0; tries < I2C_OP_TRIES; ++tries) {
		rv = i2c_transfer(adap, msgs, nmsgs);
		if (rv == nmsgs) {
			*val = val_buf[0];
			dev_dbg(&adap->dev, "%s %d 0x%02X 0x%04X -> 0x%02X\n",
				__func__, adap->nr, slave_addr, reg, *val);

			return 0;
		}
		usleep_range(I2C_OP_DELAY, I2C_OP_DELAY * 2);
	}

	dev_err_ratelimited(&adap->dev, "%s %d 0x%02X 0x%04X FAILED, rv: %d\n",
			    __func__, adap->nr, slave_addr, reg, rv);

	return (rv < 0) ? rv : -EIO;
}

int i2csetww(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 val)
{
	int rv;
	int tries;
	int nmsgs;
	struct i2c_msg msgs[1];
	u8 buf[4];

	msgs[0].addr = slave_addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(buf);
	msgs[0].buf = buf;

	buf[0] = (u8)((reg >> 8) & 0xFF);
	buf[1] = (u8)(reg & 0xFF);
	buf[2] = (u8)((val >> 8) & 0xFF);
	buf[3] = (u8)(val & 0xFF);

	nmsgs = ARRAY_SIZE(msgs);
	dev_dbg(&adap->dev, "%s %d 0x%02X 0x%04X 0x%04X\n", __func__, adap->nr,
		slave_addr, reg, val);
	for (tries = 0; tries < I2C_OP_TRIES; ++tries) {
		rv = i2c_transfer(adap, msgs, nmsgs);
		if (rv == nmsgs)
			return 0;

		usleep_range(I2C_OP_DELAY, I2C_OP_DELAY * 2);
	}

	dev_err_ratelimited(&adap->dev,
			    "%s %d 0x%02X 0x%04X 0x%04X FAILED, rv: %d\n",
			    __func__, adap->nr, slave_addr, reg, val, rv);

	return (rv < 0) ? rv : -EIO;
}

int i2cgetww(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 *val)
{
	int rv;
	int tries;
	int nmsgs;
	struct i2c_msg msgs[2];
	u8 reg_buf[2];
	u8 val_buf[2];

	msgs[0].addr = slave_addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(reg_buf);
	msgs[0].buf = reg_buf;
	msgs[1].addr = slave_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = sizeof(val_buf);
	msgs[1].buf = val_buf;

	reg_buf[0] = (u8)((reg >> 8) & 0xFF);
	reg_buf[1] = (u8)(reg & 0xFF);

	nmsgs = ARRAY_SIZE(msgs);
	for (tries = 0; tries < I2C_OP_TRIES; ++tries) {
		rv = i2c_transfer(adap, msgs, nmsgs);
		if (rv == nmsgs) {
			*val = (u16)(val_buf[0] << 8) | val_buf[1];
			dev_dbg(&adap->dev, "%s %d 0x%02X 0x%04X -> 0x%04X\n",
				__func__, adap->nr, slave_addr, reg, *val);

			return 0;
		}
		usleep_range(I2C_OP_DELAY, I2C_OP_DELAY * 2);
	}

	dev_err_ratelimited(&adap->dev, "%s %d 0x%02X 0x%04X FAILED, rv: %d\n",
			    __func__, adap->nr, slave_addr, reg, rv);

	return (rv < 0) ? rv : -EIO;
}

static int i2cupdate(i2cget *i2cget, i2cset *i2cset, struct i2c_adapter *adap,
		     int slave_addr, u32 reg, u32 val, u32 mask, u32 off)
{
	int rv;
	u32 rval;
	u32 wval;

	rv = i2cget(adap, slave_addr, reg, &rval);
	if (rv)
		return rv;

	wval = (rval & (~(mask << off))) | ((val & mask) << off);

	return i2cset(adap, slave_addr, reg, wval);
}

int i2cupbb(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 val,
	    u32 mask, u32 off)
{
	return i2cupdate(i2cgetbb, i2csetbb, adap, slave_addr, reg, val, mask,
			 off);
}

int i2cupwb(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 val,
	    u32 mask, u32 off)
{
	return i2cupdate(i2cgetwb, i2csetwb, adap, slave_addr, reg, val, mask,
			 off);
}

int i2cupww(struct i2c_adapter *adap, int slave_addr, u32 reg, u32 val,
	    u32 mask, u32 off)
{
	return i2cupdate(i2cgetww, i2csetww, adap, slave_addr, reg, val, mask,
			 off);
}

int i2cprobe(struct i2c_adapter *adap, int slave_addr, i2cget *i2cget)
{
	u32 val;

	return i2cget(adap, slave_addr, 0, &val);
}

int i2cset_in_bulk(struct i2c_adapter *adap, int slave_addr, i2cset *i2cset,
		   const struct reg_cfgs *cfgs)
{
	int i;
	const struct reg_cfg *cfg;

	cfg = cfgs->cfg;
	for (i = 0; i < cfgs->num; ++i) {
		int rv;

		rv = i2cset(adap, slave_addr, cfg[i].reg, cfg[i].val);
		if (cfg[i].delay > 0)
			ursleep(cfg[i].delay);
		if (rv) {
			dev_err_ratelimited(
				&adap->dev,
				"i2cset %d 0x%02X 0x%04X 0x%04X FAILED\n",
				adap->nr, slave_addr, cfg[i].reg, cfg[i].val);
			return rv;
		}
	}

	return 0;
}

int i2cset_from_dt(struct i2c_adapter *adap, int slave_addr, i2cset *i2cset,
		   const struct device_node *node, const char *name,
		   bool with_delay)
{
	struct property *prop;
	int cfg_num;
	int i;
	const __be32 *p;

	prop = of_find_property(node, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	cfg_num = prop->length / sizeof(*p);
	dev_dbg(&adap->dev, "length: %u, %lu, cfg_num: %d\n", prop->length,
		sizeof(*p), cfg_num);
	p = prop->value;
	for (i = 0; i < cfg_num - 1; i += 2) {
		u32 reg;
		u32 val;
		u32 delay;
		int rv;

		reg = be32_to_cpup(p);
		val = be32_to_cpup(p + 1);
		p += 2;
		rv = i2cset(adap, slave_addr, reg, val);
		if (rv) {
			dev_err_ratelimited(
				&adap->dev,
				"i2cset %d 0x%02X 0x%04X 0x%04X FAILED\n",
				adap->nr, slave_addr, reg, val);
			return rv;
		}
		if (with_delay) {
			delay = be32_to_cpup(p);
			ursleep(delay);
			++p;
			++i;
		}
	}

	return 0;
}

int csi_tx_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
			   struct v4l2_mbus_config *config)
{
	struct csi_tx_dev *tx_dev;

	tx_dev = (struct csi_tx_dev *)container_of(sd, struct csi_tx_dev,
						   subdev);

	if (tx_dev->phy_if == IF_CPHY)
		config->type = V4L2_MBUS_CSI2_CPHY;
	else
		config->type = V4L2_MBUS_CSI2_DPHY;

	config->bus.mipi_csi2.num_data_lanes = tx_dev->lane_num;

	return 0;
}
