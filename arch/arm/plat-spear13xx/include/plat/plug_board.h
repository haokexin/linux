/*
 * arch/arm/mach-spear13xx/include/mach/plug_board.h
 *
 * spear13xx machine family plug board header file
 *
 * Copyright (C) 2011 ST Microelectronics
 * Pratyush Anand <pratyush.anand@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_PLUG_BOARD_H
#define __MACH_PLUG_BOARD_H

#include <linux/amba/bus.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <plat/i2c.h>

/* name of an individual plug-board is limited to 10 chars */
#define PB_NAME_LIMIT	10

/* max plug-boards which can be requested via bootargs is limited to 50 */
#define MAX_REQ_PB	50

#ifdef DEBUG
#undef DEBUG
#define DEBUG 1
#endif

struct plug_board_info {
	struct platform_device **pdevs;
	u8 pcnt;
	struct amba_device **adevs;
	u8 acnt;
	struct spi_board_info **spi_devs;
	u8 spi_cnt;
	struct i2c_dev_info **i2c_devs;
	u8 i2c_cnt;
};

struct plug_board {
	struct pmx_dev **pmx_devs;
	struct amba_device **rm_adevs;
	struct amba_device **add_adevs;
	struct platform_device **rm_pdevs;
	struct platform_device **add_pdevs;
	struct spi_board_info **rm_spi_devs;
	struct spi_board_info **add_spi_devs;
	struct i2c_dev_info **rm_i2c_devs;
	struct i2c_dev_info **add_i2c_devs;
	u32 pmx_cnt;
	u32 rm_acnt;
	u32 add_acnt;
	u32 rm_pcnt;
	u32 add_pcnt;
	u32 rm_spi_cnt;
	u32 add_spi_cnt;
	u32 rm_i2c_cnt;
	u32 add_i2c_cnt;
	void (*pb_init)(void);

	struct list_head node;
	char name[PB_NAME_LIMIT];
};

enum skip_device_type {
	SKIP_AMBA_DEVICE,
	SKIP_PLAT_DEVICE,
	SKIP_SPI_DEVICE,
	SKIP_I2C_DEVICE
};

#define INIT_PB(pb_name, pb)						\
	do {								\
		if (strlen(#pb_name) > PB_NAME_LIMIT) {			\
			pr_err("Error: name choosen for plug board is "	\
				"more than 10 chars, use a smaller "	\
				"name instead\n");			\
			continue;					\
		}							\
									\
		pb = kmalloc(sizeof(struct plug_board), GFP_KERNEL);	\
		if (!pb) {						\
			pr_err("Error allocating memory for pb: %s\n",	\
				#pb_name);				\
			continue;					\
		}							\
									\
		strcpy(pb->name, #pb_name);				\
		pb->pmx_devs = (struct pmx_dev **) pb_name##_pb_pmx_devs;		\
		pb->rm_adevs = (struct amba_device **) pb_name##_pb_rm_adevs;		\
		pb->add_adevs = (struct amba_device **) pb_name##_pb_add_adevs;		\
		pb->rm_pdevs = (struct platform_device **) pb_name##_pb_rm_pdevs;	\
		pb->add_pdevs = (struct platform_device **) pb_name##_pb_add_pdevs;	\
		pb->rm_spi_devs = (struct spi_board_info **) pb_name##_pb_rm_spi_devs;	\
		pb->add_spi_devs = (struct spi_board_info **) pb_name##_pb_add_spi_devs;\
		pb->rm_i2c_devs = (struct i2c_dev_info **) pb_name##_pb_rm_i2c_devs;	\
		pb->add_i2c_devs = (struct i2c_dev_info **) pb_name##_pb_add_i2c_devs;	\
		pb->pmx_cnt = ARRAY_SIZE(pb_name##_pb_pmx_devs);	\
		pb->rm_acnt = ARRAY_SIZE(pb_name##_pb_rm_adevs);	\
		pb->add_acnt = ARRAY_SIZE(pb_name##_pb_add_adevs);	\
		pb->rm_pcnt = ARRAY_SIZE(pb_name##_pb_rm_pdevs);	\
		pb->add_pcnt = ARRAY_SIZE(pb_name##_pb_add_pdevs);	\
		pb->rm_spi_cnt = ARRAY_SIZE(pb_name##_pb_rm_spi_devs);	\
		pb->add_spi_cnt = ARRAY_SIZE(pb_name##_pb_add_spi_devs); \
		pb->rm_i2c_cnt = ARRAY_SIZE(pb_name##_pb_rm_i2c_devs);	\
		pb->add_i2c_cnt = ARRAY_SIZE(pb_name##_pb_add_i2c_devs); \
		pb->pb_init = pb_name##_pb_init;			\
									\
		pr_info("Adding plug board: %s\n", #pb_name);		\
	} while (0)

bool spear_pb_present(void);
char *get_spear_pb(void);
int __init spear_pb_init(struct plug_board_info *pb_info,
		int (*make_pb_list)(struct list_head *pb_list));
#endif
