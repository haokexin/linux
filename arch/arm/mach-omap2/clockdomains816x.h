/*
 * arch/arm/mach-omap2/clockdomains816x.h
 *
 * TI816X Clock Domain data.
 *
 * Copyright (C) 2010 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_CLOCKDOMAINS816X_H
#define __ARCH_ARM_MACH_OMAP2_CLOCKDOMAINS816X_H

#include <plat/clockdomain.h>

#include "cm.h"
#include "cm-regbits-816x.h"

#if defined(CONFIG_ARCH_TI816X)

/*
 * TODO:
 * - Add other domains as required
 * - Fill up associated powerdomans (especially ALWON powerdomains are NULL at
 *   the moment
 * - Consider dependencies across domains (probably not applicable till now)
 */

static struct clockdomain alwon_l3_slow_816x_clkdm = {
	.name		  = "alwon_l3_slow_clkdm",
	.pwrdm		  = { .name = "alwon_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_ALWON_L3_SLOW_CLKSTCTRL,
	.clktrctrl_mask	  = TI816X_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain alwon_ethernet_816x_clkdm = {
	.name		  = "alwon_ethernet_clkdm",
	.pwrdm		  = { .name = "alwon_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_ETHERNET_CLKSTCTRL,
	.clktrctrl_mask	  = TI816X_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain mmu_816x_clkdm = {
	.name		  = "mmu_clkdm",
	.pwrdm		  = { .name = "alwon_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_MMU_CLKSTCTRL,
	.clktrctrl_mask	  = TI816X_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain mmu_cfg_816x_clkdm = {
	.name		  = "mmu_cfg_clkdm",
	.pwrdm		  = { .name = "alwon_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_MMUCFG_CLKSTCTRL,
	.clktrctrl_mask	  = TI816X_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain active_gem_816x_clkdm = {
	.name		  = "active_gem_clkdm",
	.pwrdm		  = { .name = "active_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_ACTIVE_GEM_CLKSTCTRL,
	.clktrctrl_mask	  = TI816X_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain ivahd0_816x_clkdm = {
	.name		  = "ivahd0_clkdm",
	.pwrdm		  = { .name = "ivahd0_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_IVAHD0_CLKSTCTRL,
	.clktrctrl_mask	  = TI816X_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain ivahd1_816x_clkdm = {
	.name		  = "ivahd1_clkdm",
	.pwrdm		  = { .name = "ivahd1_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_IVAHD1_CLKSTCTRL,
	.clktrctrl_mask	  = TI816X_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain ivahd2_816x_clkdm = {
	.name		  = "ivahd2_clkdm",
	.pwrdm		  = { .name = "ivahd2_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_IVAHD2_CLKSTCTRL,
	.clktrctrl_mask	  = TI816X_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain sgx_816x_clkdm = {
	.name		  = "sgx_clkdm",
	.pwrdm		  = { .name = "sgx_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_SGX_CLKSTCTRL,
	.clktrctrl_mask	  = TI816X_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain default_l3_med_816x_clkdm = {
	.name		  = "default_l3_med_clkdm",
	.pwrdm		  = { .name = "default_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_DEFAULT_L3_MED_CLKSTCTRL,
	.clktrctrl_mask	  = TI816X_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain default_ducati_816x_clkdm = {
	.name		  = "default_ducati_clkdm",
	.pwrdm		  = { .name = "default_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_DEFAULT_DUCATI_CLKSTCTRL,
	.clktrctrl_mask	  = TI816X_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain default_pcie_816x_clkdm = {
	.name		  = "default_pcie_clkdm",
	.pwrdm		  = { .name = "default_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_DEFAULT_PCI_CLKSTCTRL,
	.clktrctrl_mask	  = TI816X_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain default_usb_816x_clkdm = {
	.name		  = "default_usb_clkdm",
	.pwrdm		  = { .name = "default_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_DEFAULT_L3_SLOW_CLKSTCTRL, /* FIXME */
	.clktrctrl_mask	  = TI816X_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

#endif

#endif
