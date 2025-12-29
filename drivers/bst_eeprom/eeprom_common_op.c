// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/media-entity.h>
#include <media/media-device.h>

#include "eeprom_common_op.h"


// write 8bit data to 8bit register
int i2c_write_byte_data_byte_reg(struct i2c_adapter *adap,
		uint8_t slave_address,
		uint8_t reg_offset, uint8_t value)
{
	int ret;
	struct i2c_msg msg;
	unsigned char data[2];

	msg.addr = slave_address; /* I2C address of chip */
	msg.flags = 0;
	msg.len = 2;
	msg.buf = data;
	data[0] = reg_offset; /* register num */
	data[1] = value; /* register data */
	ret = i2c_transfer(adap, &msg, 1);
	if (ret != 1) {
		printk("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}

	return 0;
}

// read 8bit data from 8bit register
int i2c_read_byte_data_byte_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint8_t reg_offset
		, uint8_t *out_value)
{
	int ret;
	uint8_t value;
	struct i2c_msg msg[2] = { {
					  .addr = slave_address,
					  .flags = 0,
					  .len = 1,
					  .buf = &reg_offset,
				  },
					{
					  .addr = slave_address,
					  .flags = I2C_M_RD,
					  .len = 1,
					  .buf = &value,
				  } };

#ifdef USE_I2CQ
	ret = i2c_transfer(adap, &msg[0], 2);
	if (ret != 2) {
		printk("i2c_transfer send error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}
#else
	ret = i2c_transfer(adap, &msg[0], 1);
	if (ret != 1) {
		printk("i2c_transfer send error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}

	ret = i2c_transfer(adap, &msg[1], 1);
	if (ret != 1) {
		printk("i2c_transfer read error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}
#endif
	if (out_value)
		*out_value = value;

	return 0;
}

// write 8bit data to 16bit register
int i2c_write_byte_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint8_t value)
{
	//u64 time_start;
	//u64 time_end;
	int ret;
	struct i2c_msg msg;
	unsigned char data[3];

	data[0] = (uint8_t)((reg_offset >> 8) & 0xff); /* register addr */
	data[1] = (uint8_t)(reg_offset & 0xff); /* register addr */
	data[2] = value;

	msg.addr = slave_address;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	//time_start = ktime_get_ns();
	ret = i2c_transfer(adap, &msg, 1);
	//time_end = ktime_get_ns();
	if (ret != 1) {
		printk("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}
	//pr_err("i2ctransfer spend time :%ld ns\n", (time_end - time_start));
	return 0;
}

// read 8bit data from 16bit register
int i2c_read_byte_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint8_t *out_value)
{
	int ret;
	uint8_t value;
	uint8_t reg_array[2];
	struct i2c_msg msg[2] = { {
					  .addr = slave_address,
					  .flags = 0,
					  .len = 2,
					  .buf = reg_array,
				  },
					{
					  .addr = slave_address,
					  .flags = I2C_M_RD,
					  .len = 1,
					  .buf = &value,
				  } };

	reg_array[0] = (uint8_t)((reg_offset >> 8) & 0xff);
	reg_array[1] = (uint8_t)(reg_offset & 0xff);
#ifdef USE_I2CQ
	ret = i2c_transfer(adap, &msg[0], 2);
	if (ret != 2) {
		printk("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}
#else
	ret = i2c_transfer(adap, &msg[0], 1);
	if (ret != 1) {
		printk("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}

	ret = i2c_transfer(adap, &msg[1], 1);
	if (ret != 1) {
		printk("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}
#endif
	if (out_value)
		*out_value = value;

	return 0;
}

// write 16bit data to 16bit register
int i2c_write_word_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint16_t value)
{
	int ret;
	struct i2c_msg msg;
	unsigned char data[4];

	data[0] = (uint8_t)((reg_offset >> 8) & 0xff); /* register addr */
	data[1] = (uint8_t)(reg_offset & 0xff); /* register addr */
	data[2] = (uint8_t)((value >> 8) & 0xff);
	data[3] = (uint8_t)(value & 0xff);
	msg.addr = slave_address; /* I2C address of chip */
	msg.flags = 0;
	msg.len = 4;
	msg.buf = data;

	ret = i2c_transfer(adap, &msg, 1);
	if (ret != 1) {
		printk("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}

	return 0;
}

// read 16bit data from 16bit register
int i2c_read_word_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint16_t *out_value)
{
	int ret;
	uint16_t value;
	uint8_t read_value[2];
	uint8_t reg_array[2];

	struct i2c_msg msg[2] = { {
					  .addr = slave_address,
					  .flags = 0,
					  .len = 2,
					  .buf = reg_array,
				  },
					{
					  .addr = slave_address,
					  .flags = I2C_M_RD,
					  .len = 2,
					  .buf = read_value,
				  } };

	reg_array[0] = (uint8_t)((reg_offset >> 8) & 0xff);
	reg_array[1] = (uint8_t)(reg_offset & 0xff);

#ifdef USE_I2CQ
	ret = i2c_transfer(adap, &msg[0], 2);
	if (ret != 2) {
		printk("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}
#else
	ret = i2c_transfer(adap, &msg[0], 1);
	if (ret != 1) {
		printk("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}

	ret = i2c_transfer(adap, &msg[1], 1);
	if (ret != 1) {
		printk("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}
#endif
	value = (uint16_t)(read_value[0] << 8 | read_value[1]);
	if (out_value)
		*out_value = value;

	return 0;
}
