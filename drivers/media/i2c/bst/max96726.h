/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __MAXIM_96726_DESER__
#define __MAXIM_96726_DESER__

#include "common_deser_hub.h"
#include <linux/miscdevice.h>
#include <linux/videodev2.h>

#define MAXIM_96726_VIDEO_LOCK 0x8
#define MAXIM_STREAM_ENABLE_MASK	0x01
#define MAXIM_STREAM_SUB_PORT_MASK	0xf0
#define EXT_TRIGGER_DES_RX_DISABLE 0x81
#define EXT_TRIGGER_SER_TX_CFG 0x84

#define MAXIM_DESER_I2C_RETRY_TIMES 3 /* number of read/write retries */
#define MAXIM_DESER_DETECT_TIMES 5	  /* number of detect maxim deser */
#define MAXIM_DETECT_DELAY_MS       20
#define EXT_TRIGGER_DES_RX_CFG 0x83

int max96726_video_connected(struct deser_hub_dev *hub, int index);

int max96712_reg_write(struct deser_hub_dev *hub, uint16_t reg, uint8_t value);

int max96712_reg_read(struct deser_hub_dev *hub, uint16_t reg, uint8_t *value);

int write_reg(struct i2c_client *client, uint16_t reg, int val);

int write_register(struct i2c_adapter *adap, uint8_t slave_address,
			uint16_t reg_offset, uint8_t value);

int parse_camera_serdes(struct deser_hub_dev *maxim_deser_hub,
			struct device_node *remote_ep, int index);

int ser_write(struct deser_hub_dev *hub, uint8_t addr, uint8_t reg, uint8_t value);

#endif // __MAXIM_96726_DESER__
