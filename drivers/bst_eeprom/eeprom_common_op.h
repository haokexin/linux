/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

// write 8bit data to 8bit register
int i2c_write_byte_data_byte_reg(
	    struct i2c_adapter *adap,
		uint8_t slave_address,
		uint8_t reg_offset,
		uint8_t value);

// read 8bit data from 8bit register
int i2c_read_byte_data_byte_reg(
		struct i2c_adapter *adap
		, uint8_t slave_address
		, uint8_t reg_offset
		, uint8_t *out_value);

// write 8bit data to 16bit register
int i2c_write_byte_data_word_reg(
	    struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint8_t value);

// read 8bit data from 16bit register
int i2c_read_byte_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint8_t *out_value);

// write 16bit data to 16bit register
int i2c_write_word_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint16_t value);

// read 16bit data from 16bit register
int i2c_read_word_data_word_reg(struct i2c_adapter *adap
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
