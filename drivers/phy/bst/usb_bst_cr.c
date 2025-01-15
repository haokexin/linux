// SPDX-License-Identifier: GPL-2.0+
/*
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

#include "phy-bst-usb.h"
#include "usb_phy_sram.c"
#define read_bit32(addr, bit_len, bit_start)    \
	((readl(addr)>>(bit_start))&((1<<(bit_len))-1))

u32 wait_reg_bit(void __iomem *reg, u8 bit_start, u8 bit_len, u32 value)
{
	u32 reg_value = 0;
	u32 count = 0;
	u32 ret = 0;

	do {
		reg_value = read_bit32(reg, bit_len, bit_start);
		if (count >= 10000) {
			ret = 1;
			break;
		}
		count++;
	} while (reg_value != value);
	return ret;
}

u32 write_cr_reg(void __iomem *usb_phy_reg_addr, u16 addr, u16 data)
{
	u32 cr_data = 0;
	u32 tmp_data = 0;

	cr_data = (data | addr << 16);
	writel(cr_data, usb_phy_reg_addr + 0x20);
	tmp_data = readl(usb_phy_reg_addr + 0x304);
	tmp_data ^= (1 << 1);	//csr_phy_cr_para_wr_en bit1
	writel(tmp_data, usb_phy_reg_addr + 0x304);
	return wait_reg_bit(usb_phy_reg_addr + 0x210, 0, 1, 0);
}

u16 read_cr_reg(void __iomem *usb_phy_reg_addr, u16 addr)
{
	u32 cr_data = 0;
	u32 tmp_data = 0;

	cr_data = (addr << 16);
	writel(cr_data, usb_phy_reg_addr + 0x20);
	tmp_data = readl(usb_phy_reg_addr + 0x304);
	tmp_data ^= (1 << 2);	//csr_phy_cr_para_rd_en bit2
	writel(tmp_data, usb_phy_reg_addr + 0x304);
	wait_reg_bit(usb_phy_reg_addr + 0x210, 0, 1, 0);
	tmp_data = readl(usb_phy_reg_addr + 0x210);
	return (tmp_data & 0xffff0000) >> 16;
}

void sram_fw_write_cr_reg(void __iomem *usb_phy_reg_addr)
{
	u32 i = 0;
	u16 reg = 0;
	u32 arraysize = ARRAY_SIZE(sram_fw); //sizeof(sram_fw) / sizeof(sram_fw[0]);

	for (i = 0; i < arraysize; i++) {
		write_cr_reg(usb_phy_reg_addr, sram_fw[i].addr,
			     sram_fw[i].data);
	}
	reg = read_cr_reg(usb_phy_reg_addr, 0x22);
	reg &= (~0xff);
	reg |= 1 << 7;
	reg |= 4 << 4;
	reg |= 1 << 3;
	reg |= 1 << 0;
	write_cr_reg(usb_phy_reg_addr, 0x22, reg);

}

void usb31_phy_use_sram_direct_init(void __iomem *usb_phy_base_addr)
{				//figure 5-59
	u32 reg = 0;
	//sram_ext_load bit10 =1  sram_bypass bit11=1
	reg = readl(usb_phy_base_addr + 0x4);
	reg |= (0x1 << 11) | (0x1 << 10);
	writel(reg, usb_phy_base_addr + 0x4);

	reg = readl(usb_phy_base_addr + 0x0);
	reg &= ~(0x3 << 1);	//bit1: csr_u3phy_rst_n  bit2: csr_u2phy_rst_n internal usb reset
	writel(reg, usb_phy_base_addr + 0x0);

	wait_reg_bit(usb_phy_base_addr + 0x200, 0, 1, 1);	//wait sram_init_done 0x200 bit0 == 1

	//write cr
	sram_fw_write_cr_reg(usb_phy_base_addr);
	//sram_ext_load bit10 =0
	reg = readl(usb_phy_base_addr + 0x4);
	reg &= ~(0x1 << 10);
	writel(reg, usb_phy_base_addr + 0x4);

}


MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("Black Sesame Technologies USB phy driver");
MODULE_LICENSE("GPL v2");
