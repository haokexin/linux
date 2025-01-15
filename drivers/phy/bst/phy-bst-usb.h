/* SPDX-License-Identifier: GPL-2.0+
 *
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 * phy driver for BST usb
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

#ifndef _PHY_BST_USB_H
#define _PHY_BST_USB_H
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#define USB3PHY_REG_1           0x14
#define FSEL_POS            0
#define FSEL_MSK            (0x3f<<FSEL_POS)
#define MPLLMULTIPLIER_POS  6
#define MPLLMULTIPLIER_MSK  (0x7f<<MPLLMULTIPLIER_POS)
#define SSCREFCLKSEL_POS    23
#define SSCREFCLKSEL_MSK    (0x1ff<<SSCREFCLKSEL_POS)

#define USB3PHY_REG_2           0x18
#define SSCEN               BIT(0)
#define REFUSEPAD           BIT(5)
#define REFSSPEN            BIT(6)

#define USB3PHY_REG_4           0x20
#define REFCLKDIV2          BIT(2)
#define REFCLKSEL_POS       0
#define REFCLKSEL_MSK       (0x3<<REFCLKSEL_POS)

#define USB3PHY_REG_5           0x60

#define USB3PHY_LOCAL_RST       0x68
#define PHYSWRSTN           BIT(0)
#define CTRLSWRSTN          BIT(1)

#define USB2PHY_PWR_CTRL        0x10
#define COMMONONN           BIT(0)
#define PHYRESET            BIT(1)

#define USB31_CRM_CTRL        0x0
#define CSR_CTRL_SW_RST           BIT(0)
#define CSR_U3PHY_SW_RST          BIT(1)
#define CSR_U2PHY_SW_RST          BIT(2)
#define CSR_DIV_SW_RST            BIT(3)

#define USB31PHY_REG_1        0x4
#define CSR_PHY_REF_USE_PAD  BIT(9)


#define DWC_USB31_DEBUG_SEL  0x22c
#define CSR_PHY31_GATE_EN    BIT(1)

#define USB31_HOST_CTRL0        0x64
#define CSR_HOST_FORCE_GEN1_SPEED           BIT(14)

#define PHY_TCA_OFFSET 0x800

enum usb_mode_enum {
	USB_MODE_UNKNOWN,
	USB_MODE_USB2,
	USB_MODE_USB3,
	USB_MODE_USB31,
};

struct bst_usb {
	//struct usb_phy                phy;
	void __iomem *phy_base;
	struct device *dev;
	enum usb_mode_enum usb_mode;
	int external_clk;
	struct reset_control *reset;
	bool allow_cr_test;
	int vboost;
	bool host_force_gen1_speed;
//typec phy
	bool is_typec_phy;
	void __iomem *phy_tca_base;
	struct typec_switch *typec_switch;
	struct typec_mux *typec_mux;
	u32 cur_orientation;

};

int usb_typec_phy_probe(struct bst_usb *ctx);
int usb_typec_phy_remove(struct bst_usb *ctx);
int usb_typec_phy_init(struct bst_usb *ctx);
u16 read_cr_reg(void __iomem *usb_phy_reg_addr, u16 addr);
u32 write_cr_reg(void __iomem *usb_phy_reg_addr, u16 addr, u16 data);
u32 wait_reg_bit(void __iomem *reg, u8 bit_start, u8 bit_len, u32 value);
void usb31_phy_use_sram_direct_init(void __iomem *usb_phy_base_addr);

int bst_sysfs_cr_create(struct device *dev);
void bst_sysfs_cr_remove(struct device *dev);
int set_orientation(struct bst_usb *ctx, unsigned int orientation);

#endif
