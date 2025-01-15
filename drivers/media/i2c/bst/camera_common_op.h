/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_CAMERA_COMMON_OP_H_

#define __BST_CAMERA_COMMON_OP_H_

#ifdef C1200_ISP
#include "../../platform/bst-c1200/cam_entity.h"
#endif

// TI913, 933, 953, device id reg addr are all 0x00
#define TI_SER_DEVICE_ID 0x00

int parse_camera_endpoint(struct camera_dev *cam_dev,
				 struct device_node *node);

int init_camera_dev(struct camera_dev *cam_dev
	, const struct v4l2_subdev_ops *subdev_ops
	, const struct media_entity_operations *media_ops);

int is_camera_connected(struct camera_dev *cam_dev);

int set_camera_embed_offset(struct camera_dev *cam_dev);

/*
 *Judge whether current SOC model is slave model
 */
int is_slave_soc_model(struct camera_dev *camera);

// write 8bit data to 8bit register
int bst_i2c_write_byte_data_byte_reg(
	    struct i2c_adapter *adap,
		uint8_t slave_address,
		uint8_t reg_offset,
		uint8_t value);

// read 8bit data from 8bit register
int bst_i2c_read_byte_data_byte_reg(
		struct i2c_adapter *adap
		, uint8_t slave_address
		, uint8_t reg_offset
		, uint8_t *out_value);

// write 8bit data to 16bit register
int bst_i2c_write_byte_data_word_reg(
	    struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint8_t value);

// read 8bit data from 16bit register
int bst_i2c_read_byte_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint8_t *out_value);

// write 16bit data to 16bit register
int bst_i2c_write_word_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint16_t value);

// read 16bit data from 16bit register
int bst_i2c_read_word_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint16_t *out_value);

//#define USE_I2CQ

#ifdef USE_I2CQ
enum i2cq_addr_format {
	CMD_ADDR_8BIT_DATA_8BIT,
	CMD_ADDR_8BIT_DATA_16BIT,
	CMD_ADDR_16BIT_DATA_8BIT,
	CMD_ADDR_16BIT_DATA_16BIT,
};
#endif
#endif // __BST_CAMERA_COMMON_OP_H_
