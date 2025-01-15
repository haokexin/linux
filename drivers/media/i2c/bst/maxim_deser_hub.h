/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __MAXIM_DESER_HUB__
#define __MAXIM_DESER_HUB__

#include "common_deser_hub.h"
#include <linux/miscdevice.h>
#include <linux/videodev2.h>
#define MAXIM_ID_REG			0x1e
#define MAX9286_ID			0x40
#define MAX96705_ID			0x41
#define MAX96705_ADDR			0x40
#define MAX96705_BROADCAST		0x45
#define MAX96701_ID			0x45
#define MAX96701_ADDR			0x40
#define MAX_SER_ADDR			0x40
#define MAXIM_DESER_I2C_RETRY_TIMES 3 /* number of read/write retries */
#define MAXIM_DESER_DETECT_TIMES 5	  /* number of detect maxim deser */
#define MAXIM_DETECT_DELAY_MS       20

#define MAXIM_STREAM_ENABLE_MASK	0x01
#define MAXIM_STREAM_SUB_PORT_MASK	0xf0
#define MAXIM_VIDEO_GSML2_LOCK_A 0x1a
#define MAXIM_VIDEO_GSML2_LOCK_B 0xa
#define MAXIM_VIDEO_GSML2_LOCK_C 0xb
#define MAXIM_VIDEO_GSML2_LOCK_D 0xc
#define EXT_TRIGGER_DES_RX_CFG 0x83
#define EXT_TRIGGER_DES_RX_DISABLE 0x81
#define EXT_TRIGGER_SER_TX_CFG 0x84

/*
 *	if connected return 1 ,otherwise return 0
 */
int is_gmsl2_video_connected(struct deser_hub_dev *hub, int index);

void config_ser_reg_group(uint16_t group[][2], int len,
			  struct deser_hub_dev *hub, int reg);

int write_register(struct i2c_adapter *adap, uint8_t slave_address, uint16_t reg_offset, uint8_t value);

int ser_word_write(struct deser_hub_dev *hub, uint8_t addr, uint16_t reg, uint8_t value);

int ser_word_read(struct deser_hub_dev *hub, uint8_t addr, uint16_t reg, uint8_t *value);

int ser_write(struct deser_hub_dev *hub, uint8_t addr, uint8_t reg, uint8_t value);

int reg8_read(struct i2c_client *client, u8 reg, u8 *val);

int write_reg(struct i2c_client *client, uint16_t reg, int val);

int max9286_reg_write(struct deser_hub_dev *hub, uint8_t reg, uint8_t value);
/* read reg for max96712 and max96722*/
int max96712_reg_read(struct deser_hub_dev *hub, uint16_t reg, uint8_t *value);
/* write reg for max96712 and max96722*/
int max96712_reg_write(struct deser_hub_dev *hub, uint16_t reg, uint8_t value);

int max9296_reg_write(struct deser_hub_dev *hub, uint16_t reg, uint8_t value);

void max967XX_replicate_mode(struct deser_hub_dev *hub);
//External trigger func
int maxim_deser_hub_set_internal_frame_sync(struct deser_hub_dev *maxim_hub_dev,
											int trigger_gpio, int fps);
//Internal trigger func
int maxim_deser_hub_set_external_frame_sync(struct deser_hub_dev *deser_hub,
											int camera_trigger_gpio, int deser_trigger_gpio);

int maxim_hub_mipi_output(struct maxim_hub_priv *priv, bool enable);

int parse_camera_serdes(struct deser_hub_dev *maxim_deser_hub,
			struct device_node *remote_ep, int index);

#endif // __MAXIM_DESER_HUB__
