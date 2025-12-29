/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
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

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_graph.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/usb/pd.h>
#include <linux/usb/role.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/firmware.h>

#define I2C_ADDRESS1		0x08
#define I2C_ADDRESS2		0x40
#define I2C_ADDRESS3		0x42

/* Macros for CCGX register locations and sizes */
#define DEVICE_MODE_REG (0x0000)
#define DEVICE_MODE_REG_BYTE_SIZE 1

#define SILICON_ID_REG (0x0002)
#define SILICON_ID_REG_BYTE_SIZE 2

#define RESPONSE_REGISTER_REG (0x007E)
#define RESPONSE_REGISTER_REG_BYTE_SIZE 2

#define PDPORT_ENABLE_REG (0x002C)
#define PDPORT_ENABLE_REG_BYTE_SIZE 1

#define INTR_REG (0x0006)
#define INTR_REG_BYTE_SIZE 1
#define  DEV_INT                BIT(0)
#define  PORT0_INT              BIT(1)
#define  PORT1_INT              BIT(2)
#define  AUTO_PORT0_INT         BIT(6)
#define  AUTO_PORT1_INT         BIT(5)

#define JUMP_TO_BOOT_REG (0x0007)
#define JUMP_TO_BOOT_REG_BYTE_SIZE 1

#define RESET_REG (0x0008)
#define RESET_REG_BYTE_SIZE 2

#define ENTER_FLASHING_MODE_REG (0x000A)
#define ENTER_FLASHING_MODE_REG_BYTE_SIZE 1

#define FLASH_ROW_READ_WRITE_REG (0x000C)
#define FLASH_ROW_READ_WRITE_REG_BYTE_SIZE 4

#define VALIDATE_FW_REG (0x000B)
#define VALIDATE_FW_REG_BYTE_SIZE 1

#define FLASH_READ_WRITE_MEMORY (0x0200)

/* PD Command Codes */
#define READ_PDO_DATA_PD_COMMAND (0x10)
#define READ_SOURCE_PDO_PD_COMMAND (0x33)
#define READ_SINK_PDO_PD_COMMAND (0x21)
#define PORT_DISABLE_PD_COMMAND (0x11)

#define  HPI_DEV_REG_AUTO_CMD_ADDR_P0 0x0068
#define  HPI_DEV_REG_AUTO_CMD_PARAM_P0 0x0069
#define  HPI_DEV_REG_AUTO_CMD_ADDR_P1  0x0073
#define  HPI_DEV_REG_AUTO_CMD_PARAM_P1 0x0074
#define  HPI_AUTO_CMD_STATUS_LOCATION_P0 0x0200
#define  HPI_AUTO_CMD_DATA_LOCATION_P0 0x0204
#define  HPI_AUTO_CMD_STATUS_LOCATION_P1  0x0220
#define  HPI_AUTO_CMD_DATA_LOCATION_P1 0x0224

#define HPI_GET_PORT_STATUS  11
//BIT0:3
#define PORT_NOT_CONNECT 0
#define PORT_ONLY_TYPE_C 1
#define PORT_ONLY_PD 2
#define PORT_PD_CONNECT 3
//BIT4-7
//0-> Not in active legacy mode
//1-> BC 1.2 DCP
//2-> QC 2.0 charger
//3-> QC 3.0 charger
//4-> AFC charger
//5-> Apple Power Brick
//6-> BC 1.2 CDP

//todo: here should be redefined by chip factory
#define PORT0_STATUS  0x0040
#define PORT1_STATUS  0x0042
#define  STATUS_CONNECTED           BIT(0)
#define  STATUS_ORIENTATION         BIT(1)
#define  STATUS_PORT_PD           BIT(2)
#define  STATUS_DATA_ROLE           BIT(6)
#define  STATUS_PWR_ROLE            BIT(3)
#define  STATUS_VCONN_ROLE          BIT(4)

#define USER_INT_REG  0x004F
#define PORT0_INT_STATUS     BIT(4)

/* CCGx response codes */
enum response_codes {
	CMD_NO_RESP = 0x00,
	CMD_SUCCESS = 0x02,
	FLASH_DATA_AVAILABLE = 0x03,
	CMD_INVALID = 0x05,
	CMD_FAILED = 0x06,
	FLASH_UPDATE_FAIL = 0x07,
	INVALID_FW = 0x08,
	INVALID_ARG = 0x09,
	CMD_NOT_SUPPORT = 0x0A,
	TRANSACTION_FAIL = 0x0C,
	PD_CMD_FAIL = 0x0D,
	UNDEF_ERROR = 0x0F,
	INVALID_RESP = 0x10,
	RESET_COMPLETE = 0x80
};

//define for upgrade
#define CURRENT_FW_ROW_WIDTH_IN_BYTES 256

/* Macros for row width of FW images */
#define ROW_WIDTH_128_BYTES 128
#define ROW_WIDTH_256_BYTES 256

/* Flash Row Locations */
#define CCG3_NUM_ROWS               1024
#define CCG3_IMAGE_1_METADATA       1023
#define CCG3_IMAGE_2_METADATA       1022
#define CCG3PA_IMAGE_METADATA       511
#define CCG7D_IMAGE_METADATA        511

#define CCG7D_RAW_INDEX        0X2F //CCG7D_FW_IMAGE_metadata_table[0xC6]<<8|CCG7D_FW_IMAGE_metadata_table[0xC5]<<0

#define CCG3_IMAGE_1_ROW_BEGIN      48
#define CCG3_IMAGE_2_ROW_BEGIN      512

#define CCG3PA_CONFIG_TABLE_BEGIN   50
#define CCG3_CONFIG_TABLE_1_BEGIN   50
#define CCG7D_CONFIG_TABLE_BEGIN    49

/* Config Table 2 is not a static row position.  Image 2 metadata is used to find the correct row for config table 2 */
#define CCG3_CONFIG_TABLE_2_BEGIN (((uint8) CCG3_FW_IMAGE_2_metadata_table[0x46] << 8) | ((uint8) CCG3_FW_IMAGE_2_metadata_table[0x45] << 0) + 3)

enum update_fw_flag {
	UPDATE_NONE_FLAG,
	UPDATE_CFG_FLAG,
	UPDATE_FW_FLAG,
	UPDATE_CFG_FW_FLAG,
};
/* Data type identifying which firmware image to be updating */
enum fw_device_bootmode {
	SINGLE_IMAGE,
	FIRMWARE_1 = 1,
	FIRMWARE_2 = 2
};

/* Data type used to identify which state of the update process microcontroller and CCGX are currently in */
enum fw_update_state {
	START_STATE,
	DISABLE_PD_PORTS_STATE,
	JUMP_TO_BOOT_STATE,
	ENABLE_FLASHING_MODE_STATE,
	ERASE_METADATA_ROW_STATE,
	WRITE_IMAGE_STATE,
	VALIDATE_FW_STATE,
	RESET_FW_MODE_STATE,
	FINAL_CHECK_STATE,
	WAITING_FOR_INTR_STATE,
	JUMP_TO_ALT_FW_STATE,
	WRITE_CONFIG_TABLE_STATE,
	END_STATE,
	ERROR_STATE
};

/* Data type used to identify which error is currently active */
enum fw_update_error {
	NO_ERR,
	PDPORT_DISABLE_ERR,
	JUMP_TO_BOOT_ERR,
	ENABLE_FLASHING_ERR,
	VALIDATE_FW_ERR,
	JUMP_TO_ALT_FW_ERR,
	RESET_FW_ERR,
	FINAL_CHECK_ERR
};

enum fw_mode {
	FW_CFG,
	FW_IMAGE = 1,
};

struct ccgx_upgrade_node {
	struct regmap *regmap;	//only copy from ccgx_data
	struct device *dev;	//only copy from ccgx_data
	const struct firmware *fw;
	bool initial_state_setup_complete;
	bool pd_port_disabled_skip_states_flag;
	bool device_in_bootloader_skip_states_flag;
	bool waiting_for_intr_timeout;
	bool incorrect_response_code_timeout;
	/* Response code data type which keeps track of what response code should be expected following state changes and an INTR */
	enum response_codes expected_response_code;
	/* Specific States may produce two different expected response codes.  This alternate response code is used to cover the 2nd expected response. */
	enum response_codes alt_expected_response_code;
	/* FW Update State data types which track current and next states */
	enum fw_update_state current_state;
	enum fw_update_state next_state;
	/* FW Update Error type which keeps track of current errors */
	enum fw_update_error error_type;
	enum fw_device_bootmode fw_image_to_update;
	int timeout_count;
	bool update_success_flag;
};

struct ccgx_data {
	struct i2c_client *tcpc_client;
	int irq;
	struct work_struct work;
	struct work_struct update_work;
	struct workqueue_struct *workqueue;
	struct mutex lock;
	struct device *dev;
	struct regmap *regmap;

	struct usb_role_switch *role_sw;
	struct typec_port *port;
	struct typec_partner *partner;
	struct typec_capability caps;
	void __iomem *phy_tca_base;
	int cur_orientation;

	int update_fw_flag;
	struct ccgx_upgrade_node node;

	//for test
	u32 irq_test_data;
};

void ccgx_update_firmware(struct work_struct *work);
void ccgx_unregister_partner(struct ccgx_data *ctx);

