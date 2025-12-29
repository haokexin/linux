// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 * This file contains proprietary information that is the sole intellectual
 * property of Black Sesame Technologies, Inc. and its affiliates.
 * No portions of this material may be reproduced in any
 * form without the written permission of:
 * Black Sesame Technologies, Inc. and its affiliates
 * 2255 Martin Ave. Suite D
 * Santa Clara, CA 95050
 * Copyright @2016: all right reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include "bstccgx.h"

/* Flash Row Writing, Reading, and Clearing function declarations */
static void FLASH_ROW_CLEAR(struct ccgx_upgrade_node *node, u16 row,
			    int row_width_in_bytes);
static void FLASH_ROW_WRITE(struct ccgx_upgrade_node *node, u16 row,
			    u8 *buf_ptr, int row_width_in_bytes);
__maybe_unused
static void FLASH_ROW_READ(struct ccgx_upgrade_node *node, u16 row,
			   int row_width_in_bytes);

/* State Functions */
static void start(struct ccgx_upgrade_node *node);
static void disable_pd_ports(struct ccgx_upgrade_node *node);
static void jump_to_boot(struct ccgx_upgrade_node *node);
static void enable_flashing_mode(struct ccgx_upgrade_node *node);
static void erase_metadata_row(struct ccgx_upgrade_node *node);
static void write_image(struct ccgx_upgrade_node *node);
static void validate_fw(struct ccgx_upgrade_node *node);
static void reset_fw(struct ccgx_upgrade_node *node);
static void final_check(struct ccgx_upgrade_node *node);
static void waiting_for_intr(struct ccgx_upgrade_node *node);
__maybe_unused
static void jump_to_alt_fw(struct ccgx_upgrade_node *node);
static void write_config_table(struct ccgx_upgrade_node *node);
static void error_handler(struct ccgx_upgrade_node *node);

/* Intrusive update method which disables all PD contracts and resets the device. */
static void ccgx_fw_update_intrusive(struct ccgx_upgrade_node *node);
/* Config table update method.  Disables all active pd contracs and resets the device */
static void ccgx_config_table_update(struct ccgx_upgrade_node *node);

/* Enables PDPORT */
void pdport_enable(struct regmap *regmap)
{
	u8 write_buffer[PDPORT_ENABLE_REG_BYTE_SIZE] = { 0x3 };

	regmap_raw_write(regmap, PDPORT_ENABLE_REG, write_buffer,
			 PDPORT_ENABLE_REG_BYTE_SIZE);
}

/* Disables PDPORT */
void pd_port_disable(struct regmap *regmap)
{
	u8 write_buffer_1[PDPORT_ENABLE_REG_BYTE_SIZE] = { 0x0 };

	regmap_raw_write(regmap, PDPORT_ENABLE_REG, write_buffer_1,
			 PDPORT_ENABLE_REG_BYTE_SIZE);
}

/* Checks whether PDPORT is disabled.  Returns true if all PDPORTS are disabled.  Returns false if PDPORTS are active */
bool PDPORT_DISABLE_STATUS(struct regmap *regmap)
{
	/* Returns true if all PDPORTS are disabled.  Returns false if at least one PDPORT is enabled */
	u8 read_buffer[PDPORT_ENABLE_REG_BYTE_SIZE];

	regmap_raw_read(regmap, PDPORT_ENABLE_REG, read_buffer,
			PDPORT_ENABLE_REG_BYTE_SIZE);

	if (read_buffer[0] & 0x0)
		return true;
	return false;		//else
}

/* Checks whether the response register returns a success code. */
bool RESPONSE_REGISTER_CMD_SUCCESS(struct regmap *regmap)
{
	u8 read_buffer[RESPONSE_REGISTER_REG_BYTE_SIZE];

	regmap_raw_read(regmap, RESPONSE_REGISTER_REG, read_buffer,
			RESPONSE_REGISTER_REG_BYTE_SIZE);
	if (read_buffer[0] & 0x02)
		return true;
	return false;		//else
}

/* Checks whether the response register returns a reset complete code */
bool RESPONSE_REGISTER_RESET_COMPLETE(struct regmap *regmap)
{
	u8 read_buffer[RESPONSE_REGISTER_REG_BYTE_SIZE];

	regmap_raw_read(regmap, RESPONSE_REGISTER_REG, read_buffer,
			RESPONSE_REGISTER_REG_BYTE_SIZE);
	if (read_buffer[0] & 0x80)
		return true;
	return false;		//else
}

/* Clears entire INTR register.  This includes device, port 0, and port 1 interrupts */
void CLEAR_INTR_REG(struct regmap *regmap)
{
	u8 write_buffer[INTR_REG_BYTE_SIZE] = { 0x7 };

	regmap_raw_write(regmap, INTR_REG, write_buffer, INTR_REG_BYTE_SIZE);
}

/* Checks whether the device interrupt from INTR reg is raised */
bool INTR_REG_RAISED(struct regmap *regmap)
{
	u8 read_buffer[INTR_REG_BYTE_SIZE];
	int i2c_ret;

	i2c_ret = regmap_raw_read(regmap, INTR_REG, read_buffer, INTR_REG_BYTE_SIZE);
	if (i2c_ret)
		return false;

	if (read_buffer[0] & 0x01)
		return true;
	return false;		//else
}

/* Sends command telling CCGX to jump to bootloader mode */
void JUMP_TO_BOOT_ENABLE(struct regmap *regmap)
{
	u8 write_buffer[JUMP_TO_BOOT_REG_BYTE_SIZE] = { 'J' };

	regmap_raw_write(regmap, JUMP_TO_BOOT_REG, write_buffer,
			 JUMP_TO_BOOT_REG_BYTE_SIZE);
}

/* Sends command telling CCGX to switch active FW image */
void JUMP_TO_ALT_FW(struct regmap *regmap)
{
	u8 write_buffer[JUMP_TO_BOOT_REG_BYTE_SIZE] = { 'A' };

	regmap_raw_write(regmap, JUMP_TO_BOOT_REG, write_buffer,
			 JUMP_TO_BOOT_REG_BYTE_SIZE);
}

/* Sends command telling CCGX to enable flashing mode */
void ENTER_FLASHING_MODE_ENABLE(struct regmap *regmap)
{
	u8 write_buffer[ENTER_FLASHING_MODE_REG_BYTE_SIZE] = { 'P' };

	regmap_raw_write(regmap, ENTER_FLASHING_MODE_REG, write_buffer,
			 ENTER_FLASHING_MODE_REG_BYTE_SIZE);
}

/* Sends command telling CCGX to validate either FW image 1.  Responses are sent to the response register */
void VALIDATE_FW_1(struct regmap *regmap)
{
	u8 write_buffer[VALIDATE_FW_REG_BYTE_SIZE] = { 0x1 };

	regmap_raw_write(regmap, VALIDATE_FW_REG, write_buffer,
			 VALIDATE_FW_REG_BYTE_SIZE);
}

/* Sends command telling CCGX to validate either FW image 2.  Responses are sent to the response register */
void VALIDATE_FW_2(struct regmap *regmap)
{
	u8 write_buffer[VALIDATE_FW_REG_BYTE_SIZE] = { 0x2 };

	regmap_raw_write(regmap, VALIDATE_FW_REG, write_buffer,
			 VALIDATE_FW_REG_BYTE_SIZE);
}

/* Sends command telling CCGX to reset */
void RESET_DEVICE(struct regmap *regmap)
{
	u8 write_buffer[RESET_REG_BYTE_SIZE] = { 'R', 0x1 };

	regmap_raw_write(regmap, RESET_REG, write_buffer, RESET_REG_BYTE_SIZE);
}

/* Checks whether CCGX device is currently in bootloader mode or not. */
bool IN_BOOTLOADER_MODE(struct regmap *regmap)
{
	u8 read_buffer[DEVICE_MODE_REG_BYTE_SIZE];

	regmap_raw_read(regmap, DEVICE_MODE_REG, read_buffer,
			DEVICE_MODE_REG_BYTE_SIZE);
	read_buffer[0] &= ~(0b11111100);

	if (read_buffer[0] == 0x0)
		return true;
	return false;		//else
}

/* Returns content of INTR_REG register */
u8 return_intr_register(struct regmap *regmap)
{
	u8 read_buffer[INTR_REG_BYTE_SIZE];

	regmap_raw_read(regmap, INTR_REG, read_buffer, INTR_REG_BYTE_SIZE);
	return read_buffer[0];
}

/* Returns content of silicon ID register */
u16 return_silicon_id_register(struct regmap *regmap)
{
	u8 read_buffer[SILICON_ID_REG_BYTE_SIZE];

	regmap_raw_read(regmap, SILICON_ID_REG, read_buffer,
			SILICON_ID_REG_BYTE_SIZE);
	return __le16_to_cpu(*(u16 *) read_buffer);
}

/* Returns content of device mode register */
u8 return_device_mode_register(struct regmap *regmap)
{
	u8 read_buffer[DEVICE_MODE_REG_BYTE_SIZE];

	regmap_raw_read(regmap, DEVICE_MODE_REG, read_buffer,
			DEVICE_MODE_REG_BYTE_SIZE);
	return read_buffer[0];
}

/* Returns content of response register */
u16 return_response_register(struct regmap *regmap)
{
	u8 read_buffer[RESPONSE_REGISTER_REG_BYTE_SIZE];

	regmap_raw_read(regmap, RESPONSE_REGISTER_REG, read_buffer,
			RESPONSE_REGISTER_REG_BYTE_SIZE);
	return __le16_to_cpu(*(u16 *) read_buffer);
}

void FLASH_ROW_CLEAR(struct ccgx_upgrade_node *node, u16 row,
		     int row_width_in_bytes)
{

	int timeout = 0;
	u8 write_buffer[CURRENT_FW_ROW_WIDTH_IN_BYTES] = { 0 };
	//Write flash memory in to row selection of choice
	u8 temp[FLASH_ROW_READ_WRITE_REG_BYTE_SIZE] = { 'F', 0x1, row, row >> 8 };

	dev_dbg(node->dev, "Clearing Flash Row: %i\n", row);
	/* Checks for flash row width compatibility */
	if (row_width_in_bytes != ROW_WIDTH_128_BYTES
	    && row_width_in_bytes != ROW_WIDTH_256_BYTES) {
		dev_dbg(node->dev,
			"Invalid Row Width Parameter in function %s\n", __func__);
		return;
	}
	memset(write_buffer, 0, row_width_in_bytes);
	regmap_raw_write(node->regmap, FLASH_READ_WRITE_MEMORY, write_buffer,
			 row_width_in_bytes);

	CLEAR_INTR_REG(node->regmap);

	regmap_raw_write(node->regmap, FLASH_ROW_READ_WRITE_REG, temp,
			 FLASH_ROW_READ_WRITE_REG_BYTE_SIZE);

	/* Ensures flash clear is successful\n */
	while (!RESPONSE_REGISTER_CMD_SUCCESS(node->regmap)) {
		if (timeout++ >= 50) {	//timeout = 50ms
			dev_dbg(node->dev, "Flash Row Clear Timeout\n");
			return;
		}
		mdelay(1);
	}

	CLEAR_INTR_REG(node->regmap);
}

/*******************************************************************************
 * Function Name: void FLASH_ROW_WRITE(u16 row, u8 *buf_ptr, int row_width_in_bytes)
 ********************************************************************************
 * Summary:
 *  This function writes a flash row in CCGX memory.  It writes a flash row
 *  selected by the row parameter.  Row width in bytes can be either
 *  ROW_WIDTH_128_BYTES or ROW_WIDTH_256_BYTES (128 or 256 bytes).
 *  If a flash row fails to be written an error message is printed.
 *  The buf_ptr parameter is used to point to an array of size 128 or 256 from
 *  which you'd like to write its data to flash memory.
 *
 * Parameters:
 *  u16 row
 *  u8 *buf_ptr
 *  int row_width_in_bytes
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void FLASH_ROW_WRITE(struct ccgx_upgrade_node *node, u16 row, u8 *buf_ptr,
		     int row_width_in_bytes)
{
	int timeout = 0;
	//Write flash memory in to row selection of choice
	u8 temp[FLASH_ROW_READ_WRITE_REG_BYTE_SIZE] = { 'F', 0x1, row, row >> 8 };

	dev_dbg(node->dev, "Writing Flash Row: %i\n", row);

	/* Checks for flash row width compatibility */
	if (row_width_in_bytes != ROW_WIDTH_128_BYTES
	    && row_width_in_bytes != ROW_WIDTH_256_BYTES) {
		dev_dbg(node->dev,
			"Invalid Row Width Parameter in function %s\n", __func__);
		return;
	}

	/* Write buffer data in to flash memory */
	regmap_raw_write(node->regmap, FLASH_READ_WRITE_MEMORY, buf_ptr,
			 row_width_in_bytes);

	CLEAR_INTR_REG(node->regmap);

	regmap_raw_write(node->regmap, FLASH_ROW_READ_WRITE_REG, temp,
			 FLASH_ROW_READ_WRITE_REG_BYTE_SIZE);

	/* Ensures flash write is successful\n */
	while (!RESPONSE_REGISTER_CMD_SUCCESS(node->regmap)) {
		if (timeout++ >= 50) {	//timeout = 50ms
			dev_dbg(node->dev, "Flash Row Write Timeout\n");
			return;
		}
		mdelay(1);
	}

	CLEAR_INTR_REG(node->regmap);
}

/*******************************************************************************
 * Function Name: void FLASH_ROW_READ(u16 row, int row_width_in_bytes)
 *******************************************************************************
 * Summary:
 *  This function reads from a flash row in memory.  It then prints the flash row
 *  data. Row width in bytes can be either ROW_WIDTH_128_BYTES or ROW_WIDTH_256_BYTES
 *  (128 or 256 bytes).  If flash read fails and error message is printed.
 *
 * Parameters:
 *  u16 row
 *  int row_width_in_bytes
 *
 * Return:
 *  void
 *
 ******************************************************************************/

void FLASH_ROW_READ(struct ccgx_upgrade_node *node, u16 row,
		    int row_width_in_bytes)
{
	int timeout = 0;
	/* Send command that copies flash row data in to flash memory so we can read. */
	u8 temp[FLASH_ROW_READ_WRITE_REG_BYTE_SIZE] = { 'F', 0x0, row, row >> 8 };
	u8 read_buffer[CURRENT_FW_ROW_WIDTH_IN_BYTES];

	dev_dbg(node->dev, "Reading Flash Row: %i\n", row);

	/* Checks for flash row width compatibility */
	if (row_width_in_bytes != ROW_WIDTH_128_BYTES
	    && row_width_in_bytes != ROW_WIDTH_256_BYTES) {
		dev_dbg(node->dev,
			"Invalid Row Width Parameter in function %s\n", __func__);
		return;
	}

	regmap_raw_write(node->regmap, FLASH_ROW_READ_WRITE_REG, temp,
			 FLASH_ROW_READ_WRITE_REG_BYTE_SIZE);
	mdelay(5);		//Max time required to read

	/* Ensures this command was successful */
	while (!RESPONSE_REGISTER_CMD_SUCCESS(node->regmap)) {
		if (timeout++ >= 50) {	//timeout = 50ms
			dev_dbg(node->dev, "Flash Row Read Timeout\n");
			return;
		}
		mdelay(1);
	}

	/* Read from flash memory and store in read buffer. Print out read buffer data */
	regmap_raw_read(node->regmap, FLASH_READ_WRITE_MEMORY, read_buffer,
			row_width_in_bytes);
	{
		int i = 0;

		for (i = 0; i < row_width_in_bytes; i++)
			dev_dbg(node->dev, "%x: %x\n", i, read_buffer[i]);
	}

	CLEAR_INTR_REG(node->regmap);
}

/* Start: Checks which mode the device is currently in.  If in bootloader, skips states. */
static void start(struct ccgx_upgrade_node *node)
{
	dev_dbg(node->dev, "State 1: Start State\n");

	node->device_in_bootloader_skip_states_flag = false;

	/* If device is already in Bootloader Mode */
	if (IN_BOOTLOADER_MODE(node->regmap)) {
		node->device_in_bootloader_skip_states_flag = true;
		dev_dbg(node->dev,
			"Device is already in bootloader. Skipping states\n\n");
		return;
	}

}

/* Disable PD Port: Disables all PD ports.  If PD port already disabled, skip states. */
static void disable_pd_ports(struct ccgx_upgrade_node *node)
{
	dev_dbg(node->dev, "State 2: Disable PD Ports State\n");

	node->pd_port_disabled_skip_states_flag = false;

	/* If PDPORT is already disabled */
	if (PDPORT_DISABLE_STATUS(node->regmap)) {
		node->pd_port_disabled_skip_states_flag = true;
		dev_dbg(node->dev,
			"PDPORT is already disabled. Skipping States.\n\n");
		return;
	}

	/* Sets expected response code.  Expected response code will be checked in Waiting for INTR state.*/
	/*  If a PD contract is established prior to PD PORT DISABLE resposne code returns RESET COMPLETE */
	node->expected_response_code = CMD_SUCCESS;

	/* Sets alternate expected response code.  If no PD contract is established prior to PD PORT DISABLE response code returns CMD_SUCCESS. */
	node->alt_expected_response_code = RESET_COMPLETE;

	/* Sets the error type.  In the event that an error occurs this error type is used as a method of debugging. */
	node->error_type = PDPORT_DISABLE_ERR;

	/* Disables PD Port command. */
	pd_port_disable(node->regmap);

	//Takes up to 1 second for PDPort to be disabled if CCGX is powered by PD.
	mdelay(1000);
}

/* Jump to Boot: Jumps to bootloader mode. */
static void jump_to_boot(struct ccgx_upgrade_node *node)
{
	dev_dbg(node->dev, "State 3: Jump To Boot State\n");

	/* Sets expected response code.  Expected response code will be checked in Waiting for INTR state. */
	node->expected_response_code = RESET_COMPLETE;

	/* Sets the error type.  In the event that an error occurs this error type is used as a method of debugging. */
	node->error_type = JUMP_TO_BOOT_ERR;

	/* Jump to Boot command. */
	JUMP_TO_BOOT_ENABLE(node->regmap);
}

/* Enable Flashing Mode: Enables flashing of device.  Can read/write in to flash rows. */
static void enable_flashing_mode(struct ccgx_upgrade_node *node)
{
	dev_dbg(node->dev, "State 4: Enable Flashing Mode State\n");

	/* Sets expected response code.  Expected response code will be checked in Waiting for INTR state. */
	node->expected_response_code = CMD_SUCCESS;

	/* Sets alternate expected response code.  If no PD contract is established prior to PD PORT DISABLE response code returns CMD_SUCCESS. */
	node->alt_expected_response_code = RESET_COMPLETE;

	/* Sets the error type.  In the event that an error occurs this error type is used as a method of debugging. */
	node->error_type = ENABLE_FLASHING_ERR;

	ENTER_FLASHING_MODE_ENABLE(node->regmap);
}

/* Waiting for INTR: Non blocking wait state which waits for an INTR to be raised following a CCGX command.  Once an INTR is raised, response register is checked
 * for the response code.  If it matches the node->expected_response_code variable established in the previous state that means commands in
 * the previous state were successful.  In the event that a timeout occurs, the function is non blocking, meaning that it will return to
 * the main for loop to service other functions and then return to this one to see if an INTR has been received, or if the expected response
 * code has changed
 */
static void waiting_for_intr(struct ccgx_upgrade_node *node)
{
	u16 response = 0;

	dev_dbg(node->dev, "State X: Waiting for INTR State\n");
	/* Resets waiting for INTR timeout flag */
	node->waiting_for_intr_timeout = false;

	/* Checks if at least the INTR register is raised OR if at least the INTR pin is active */
	if (!INTR_REG_RAISED(node->regmap)) {
		node->waiting_for_intr_timeout = true;
		if (node->timeout_count++ > 2000) {	// 200 ms timeout minimum
			dev_dbg(node->dev, "Waiting for INTR Timeout\n");
			node->current_state = ERROR_STATE;
			node->next_state = ERROR_STATE;
			return;
		}
		udelay(100);
		return;
	}

	/* Resets incorrect response code timeout flag */
	node->incorrect_response_code_timeout = false;

	/* Checks if response register returns the same response code as expected response */
	/* INTR Flag raised before response register is updated.  Which is why this 2nd while loop is required */
	node->timeout_count = 0;
	response = return_response_register(node->regmap);
	while ((u16) node->expected_response_code != response
	       && (u16) node->alt_expected_response_code != response) {
		if (node->timeout_count++ > 1500) {	// 1500 ms timeout
			dev_dbg(node->dev,
				"Expected response code does not match actual response code\n");
			node->incorrect_response_code_timeout = true;
			node->current_state = ERROR_STATE;
			return;
		}
		response = return_response_register(node->regmap);
		//response = node->expected_response_code;	///chen test  write reg 1
		mdelay(1);
	}

	/* If this portion of waiting_for_intr is reached without the function returning/exitting.  That means the command from the previous state was successful */
	dev_dbg(node->dev, "CMD_SUCCESS\n");
	node->waiting_for_intr_timeout = false;
}

/* Jump to Alt FW: Jumps to alternate FW image. */
static void jump_to_alt_fw(struct ccgx_upgrade_node *node)
{
	dev_dbg(node->dev, "State A: Jump to Alt FW State\n");

	/* Sets expected response code.  Expected response code will be checked in Waiting for INTR state. */
	node->expected_response_code = RESET_COMPLETE;

	/* Sets the error type.  In the event that an error occurs this error type is used as a method of debugging. */
	node->error_type = JUMP_TO_ALT_FW_ERR;

	/* Jump to ALT FW command */
	JUMP_TO_ALT_FW(node->regmap);
}

/* Erase Metadata Row: Erase metadata rows prior to updating with new metadata. */
static void erase_metadata_row(struct ccgx_upgrade_node *node)
{
	dev_dbg(node->dev, "Erasing CCG7D FW Metadata\n");
	FLASH_ROW_CLEAR(node, CCG7D_IMAGE_METADATA,
			CURRENT_FW_ROW_WIDTH_IN_BYTES);
}

/* Write Config Table: Writes config table of CCGX devices.*/
static void write_config_table(struct ccgx_upgrade_node *node)
{
	u16 row_index;
	u8 temp[CURRENT_FW_ROW_WIDTH_IN_BYTES];
	int index = 0, i = 0;
	int current_value = 0;
	size_t image_byte_size = 0;
	const u8 *data = NULL;

	if (node && node->fw) {
		image_byte_size = node->fw->size;
		data = node->fw->data;
	}
	dev_dbg(node->dev, "State: Write Configuration Table\n");
	row_index = CCG7D_CONFIG_TABLE_BEGIN;

	while (index < image_byte_size) {
		for (i = index;
		     i < current_value + CURRENT_FW_ROW_WIDTH_IN_BYTES; i++) {
			temp[i - current_value] = *(data + index);
			index++;
		}
		FLASH_ROW_WRITE(node, row_index, temp,
				CURRENT_FW_ROW_WIDTH_IN_BYTES);

		current_value = index;
		row_index++;
	}
}

/* Write Image: Writes FW image data in to flash rows of CCGX. */
static void write_image(struct ccgx_upgrade_node *node)
{
	u16 metadata_row;
	u8 temp[CURRENT_FW_ROW_WIDTH_IN_BYTES];
	int index = 0, i = 0;
	int current_value = 0;
	u16 row_index;
	size_t image_byte_size = 0;
	const u8 *data = NULL;

	if (node && node->fw) {
		image_byte_size = node->fw->size;
		data = node->fw->data;
	}

	dev_dbg(node->dev, "State 6: Write Image State\n");
	metadata_row = CCG7D_IMAGE_METADATA;
	row_index =  CCG7D_RAW_INDEX;
	row_index++;		//Write FW Image on row AFTER current row index.

	while (index < image_byte_size) {
		for (i = index;
		     i < current_value + CURRENT_FW_ROW_WIDTH_IN_BYTES; i++) {
			temp[i - current_value] = *(data + index);
			index++;
		}

		FLASH_ROW_WRITE(node, row_index, temp,
				CURRENT_FW_ROW_WIDTH_IN_BYTES);

		current_value = index;
		row_index++;
		udelay(10);
	}

	/* Write metadata row   //this is in fw.bin
	 * FLASH_ROW_WRITE(node, metadata_row,
	 *		(u8 *) CCG7D_FW_IMAGE_metadata_table,
	 *		CURRENT_FW_ROW_WIDTH_IN_BYTES);
	 */

}

/* Validate FW: CCGX validates recently updated FW images to ensure they were properly written/ is valid. */
static void validate_fw(struct ccgx_upgrade_node *node)
{
	dev_dbg(node->dev, "State 7: Validate FW State\n");

	/* Sets expected response code.  Expected response code will be checked in Waiting for INTR state. */
	node->error_type = VALIDATE_FW_ERR;

	/* Sets the error type.  In the event that an error occurs this error type is used as a method of debugging. */
	node->expected_response_code = CMD_SUCCESS;

	if (node->fw_image_to_update == FIRMWARE_2)
		VALIDATE_FW_2(node->regmap);
	else if (node->fw_image_to_update == FIRMWARE_1)
		VALIDATE_FW_1(node->regmap);
	else if (node->fw_image_to_update == SINGLE_IMAGE)
		VALIDATE_FW_1(node->regmap);
}

/* Reset FW: CCGX resets, loading in to the most recently updated and valid FW image. */
static void reset_fw(struct ccgx_upgrade_node *node)
{
	dev_dbg(node->dev, "State 8: Reset FW State\n");

	/* Sets expected response code.  Expected response code will be checked in Waiting for INTR state. */
	node->expected_response_code = RESET_COMPLETE;

	/* Sets the error type.  In the event that an error occurs this error type is used as a method of debugging. */
	node->error_type = RESET_FW_ERR;

	RESET_DEVICE(node->regmap);
}

/* Final Check: Ensures that CCGX isn't in bootloader mode after FW update.  If it is it means that FW image wasn't written properly. */
static void final_check(struct ccgx_upgrade_node *node)
{
	dev_dbg(node->dev, "State 9: Final Check State\n");
	node->waiting_for_intr_timeout = false;

	/* Checks to see if CCGX is in bootloader mode. If it is that means FW image isn't valid. */
	while (IN_BOOTLOADER_MODE(node->regmap)) {
		if (node->timeout_count++ > 200) {	//200ms timeout
			node->current_state = END_STATE;
			return;

		}
		mdelay(1);
	}

	mdelay(200);

	/* Determines if CCGX is in an active firmware image */
	if (return_device_mode_register(node->regmap) & 1 << 0)
		dev_dbg(node->dev, "Device in Firmware Image 1\n");
	else if (return_device_mode_register(node->regmap) & 1 << 1)
		dev_dbg(node->dev, "Device in Firmware Image 2\n");
	else {
		//In bootloader mode.  Go to Error State.
		node->error_type = FINAL_CHECK_ERR;
		node->current_state = ERROR_STATE;
		return;
	}

	node->update_success_flag = true;
}

/* Error Handler: Printed error messages based on error type that is defined during particular states. */
static void error_handler(struct ccgx_upgrade_node *node)
{
	dev_err(node->dev, "State X: Error State\n");

	if (node->error_type == PDPORT_DISABLE_ERR)
		dev_err(node->dev,
			"State 2 Error: Unable to disable PD port\n");
	else if (node->error_type == JUMP_TO_BOOT_ERR)
		dev_err(node->dev,
			"State 3 Error: Unable to jump to bootloader\n");
	else if (node->error_type == ENABLE_FLASHING_ERR)
		dev_err(node->dev,
			"State 4 Error: Unable to enable flashing mode\n");
	else if (node->error_type == VALIDATE_FW_ERR)
		dev_err(node->dev, "State 7 Error: FW Validation Error\n");
	else if (node->error_type == RESET_FW_ERR)
		dev_err(node->dev,
			"State 8 Error: Device Reset Unsuccessful\n");
	else if (node->error_type == FINAL_CHECK_ERR)
		dev_err(node->dev,
			"State 9 Error: Device not in FW Image 1 or 2.  Device in bootloader mode\n");
	else if (node->error_type == JUMP_TO_ALT_FW_ERR)
		dev_err(node->dev,
			"State A Error: Unable to jump to alternate FW image\n");
	else
		dev_err(node->dev, "Undefined Error\n");

}

void ccgx_fw_update_intrusive(struct ccgx_upgrade_node *node)
{
	node->fw_image_to_update = SINGLE_IMAGE;
	/* If its the initial passthrough of the function, set current state to START state */
	if (!node->initial_state_setup_complete) {
		node->current_state = START_STATE;
		node->next_state = END_STATE;
		node->initial_state_setup_complete = true;
		node->pd_port_disabled_skip_states_flag = false;
	}
	/* State Machine */
	while (node->current_state != END_STATE) {

		/* Only clears INTR REG and INTR FLAG when current state is not WAITING_FOR_INTR */
		if (node->current_state != WAITING_FOR_INTR_STATE)
			CLEAR_INTR_REG(node->regmap);

		switch (node->current_state) {

		case START_STATE:
			node->current_state = DISABLE_PD_PORTS_STATE;
			start(node);
			if (node->device_in_bootloader_skip_states_flag)
				node->current_state =
				    ENABLE_FLASHING_MODE_STATE;
			break;

		case DISABLE_PD_PORTS_STATE:
			node->current_state = WAITING_FOR_INTR_STATE;
			node->next_state = JUMP_TO_BOOT_STATE;
			disable_pd_ports(node);
			if (node->pd_port_disabled_skip_states_flag)
				node->current_state = JUMP_TO_BOOT_STATE;
			break;

		case JUMP_TO_BOOT_STATE:
			node->current_state = WAITING_FOR_INTR_STATE;
			node->next_state = ENABLE_FLASHING_MODE_STATE;
			jump_to_boot(node);
			break;

		case ENABLE_FLASHING_MODE_STATE:
			node->current_state = WAITING_FOR_INTR_STATE;
			node->next_state = ERASE_METADATA_ROW_STATE;
			enable_flashing_mode(node);
			break;

		case ERASE_METADATA_ROW_STATE:
			node->current_state = WRITE_IMAGE_STATE;
			erase_metadata_row(node);
			break;

		case WRITE_IMAGE_STATE:
			node->current_state = VALIDATE_FW_STATE;
			write_image(node);
			break;

		case VALIDATE_FW_STATE:
			node->current_state = WAITING_FOR_INTR_STATE;
			node->next_state = RESET_FW_MODE_STATE;
			validate_fw(node);
			break;

		case RESET_FW_MODE_STATE:
			node->current_state = WAITING_FOR_INTR_STATE;
			node->next_state = FINAL_CHECK_STATE;
			reset_fw(node);
			break;

		case FINAL_CHECK_STATE:
			node->current_state = END_STATE;
			final_check(node);
			if (node->waiting_for_intr_timeout)
				return;
			break;

		case WAITING_FOR_INTR_STATE:
			waiting_for_intr(node);
			if (node->waiting_for_intr_timeout
			    || node->incorrect_response_code_timeout)
				return;
			node->current_state = node->next_state;
			node->next_state = END_STATE;
			break;

		case ERROR_STATE:
			error_handler(node);
			node->current_state = END_STATE;
			break;

		default:
			dev_dbg(node->dev,
				"Undefined State Entered in Function ccgx_fw_update\n");
			node->current_state = END_STATE;
			break;
		}

		node->timeout_count = 0;
	}

	/* Function is complete, update initial state setup complete flag to re-enable function initializations */
	node->initial_state_setup_complete = false;

	dev_dbg(node->dev, "End of ccgx_fw_update\n");
}

void ccgx_config_table_update(struct ccgx_upgrade_node *node)
{
	node->fw_image_to_update = SINGLE_IMAGE;
	/* If its the initial passthrough of the function, set current state to START state */
	if (!node->initial_state_setup_complete) {
		node->current_state = START_STATE;
		node->next_state = END_STATE;
		node->initial_state_setup_complete = true;
	}
	/* State Machine */
	while (node->current_state != END_STATE) {
		/* Only clears INTR REG and INTR FLAG when current state is not WAITING_FOR_INTR */
		if (node->current_state != WAITING_FOR_INTR_STATE)
			CLEAR_INTR_REG(node->regmap);

		switch (node->current_state) {

		case START_STATE:
			node->current_state = DISABLE_PD_PORTS_STATE;
			start(node);
			if (node->device_in_bootloader_skip_states_flag)
				node->current_state =
				    ENABLE_FLASHING_MODE_STATE;
			break;

		case DISABLE_PD_PORTS_STATE:
			node->current_state = WAITING_FOR_INTR_STATE;
			node->next_state = JUMP_TO_BOOT_STATE;
			disable_pd_ports(node);
			if (node->pd_port_disabled_skip_states_flag)
				node->current_state = JUMP_TO_BOOT_STATE;
			break;

		case JUMP_TO_BOOT_STATE:
			node->current_state = WAITING_FOR_INTR_STATE;
			node->next_state = ENABLE_FLASHING_MODE_STATE;
			jump_to_boot(node);
			break;

		case ENABLE_FLASHING_MODE_STATE:
			node->current_state = WAITING_FOR_INTR_STATE;
			node->next_state = WRITE_CONFIG_TABLE_STATE;
			enable_flashing_mode(node);
			break;

		case WRITE_CONFIG_TABLE_STATE:
			write_config_table(node);
			node->current_state = RESET_FW_MODE_STATE;
			break;

		case RESET_FW_MODE_STATE:
			node->current_state = WAITING_FOR_INTR_STATE;
			node->next_state = FINAL_CHECK_STATE;
			reset_fw(node);
			break;

		case FINAL_CHECK_STATE:
			node->current_state = END_STATE;
			final_check(node);
			if (node->waiting_for_intr_timeout)
				return;
			break;

		case WAITING_FOR_INTR_STATE:
			waiting_for_intr(node);
			if (node->waiting_for_intr_timeout
			    || node->incorrect_response_code_timeout)
				return;
			node->current_state = node->next_state;
			node->next_state = END_STATE;
			break;

		case ERROR_STATE:
			error_handler(node);
			node->current_state = END_STATE;
			break;

		default:
			dev_dbg(node->dev,
				"Undefined State Entered in Function %s\n", __func__);
			node->current_state = END_STATE;
			break;
		}
		node->timeout_count = 0;
	}
	/* Function is complete, update initial state setup complete flag to re-enable function initializations */
	node->initial_state_setup_complete = false;

	dev_dbg(node->dev, "End of %s\n", __func__);
}

//todo:read fw version,compare with firmware,check firmware
static int ccg_fw_update_needed(struct ccgx_data *ctx)
{
	return 1;
}

static const char *const ccg_fw_names[] = {
	"ccg_cfg.bin",
	"ccg_fw.bin"
};

static int do_flash(struct ccgx_data *ctx, int mode)
{
	struct device *dev = ctx->dev;
	struct ccgx_upgrade_node *node;
	const struct firmware *fw;
	int err = -1;

	node = &ctx->node;
	err = request_firmware(&fw, ccg_fw_names[mode], dev);
	if (err) {
		dev_err(dev, "request ccg_fw.bin failed err=%d\n", err);
		return err;
	}
	memset(node, 0, sizeof(struct ccgx_upgrade_node));
	node->regmap = ctx->regmap;
	node->fw = fw;
	node->dev = dev;
	node->update_success_flag = false;
	do {
		if (mode == FW_IMAGE)
			ccgx_fw_update_intrusive(node);
		else if (mode == FW_CFG)
			ccgx_config_table_update(node);
		mdelay(1);
	} while (node->initial_state_setup_complete);
	if (!node->update_success_flag)
		err = -1;

	release_firmware(fw);
	return err;
}

static int ccg_fw_update(struct ccgx_data *ctx)
{
	int err = -1;

	switch (ctx->update_fw_flag) {
	case UPDATE_CFG_FLAG:
		err = do_flash(ctx, FW_CFG);
		if (!err)
			dev_info(ctx->dev, "CCG CFG update successful\n");
		else
			dev_err(ctx->dev, "CCG CFG update failed\n");
		break;
	case UPDATE_FW_FLAG:
		err = do_flash(ctx, FW_IMAGE);
		if (!err)
			dev_info(ctx->dev, "CCG FW update successful\n");
		else
			dev_err(ctx->dev, "CCG FW update failed\n");
		break;
	case UPDATE_CFG_FW_FLAG:
		err = do_flash(ctx, FW_IMAGE);
		if (err) {
			dev_err(ctx->dev, "step-1: CCG FW update failed\n");
			break;
		}
		err = do_flash(ctx, FW_CFG);
		if (!err)
			dev_info(ctx->dev, "CCG FW and CFG update successful\n");
		else
			dev_err(ctx->dev, "step-2: CCG CFG update failed\n");
		break;
	case UPDATE_NONE_FLAG:
	default:
		dev_info(ctx->dev, "CCG not update\n");
		break;
	}

	return err;
}

void ccgx_update_firmware(struct work_struct *work)
{
	struct ccgx_data *ctx =
	    container_of(work, struct ccgx_data, update_work);
	int status;

	status = ccg_fw_update_needed(ctx);
	if (status <= 0) {
		ctx->update_fw_flag = 0;
		return;
	}
	if (status > 0) {
		cancel_work_sync(&ctx->work);
		ccgx_unregister_partner(ctx);
		pm_runtime_disable(ctx->dev);
		disable_irq(ctx->irq);
		ccg_fw_update(ctx);
		enable_irq(ctx->irq);
		pm_runtime_enable(ctx->dev);
	}
	ctx->update_fw_flag = 0;
}

MODULE_DESCRIPTION("CCG7D USB Type-C PD driver");
MODULE_AUTHOR("BST Ltd.");
MODULE_LICENSE("GPL v2");

