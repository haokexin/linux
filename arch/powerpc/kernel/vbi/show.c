/*
 * show.c - vbi PowerPC show routines
 *
 * Copyright (c) 2008 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <vbi/vbi.h>

/*
 * vbi_disp_status_regs - display registers from a EREG_SET
 *
 * This routine display the contents of the PowerPC emulated status registers
 * structure on the console.
 *
 */
void vbi_disp_status_regs(void)
{
    struct vb_status *p = VBI_STATUS_ADDR_GET();

#ifndef CPU
#error CPU is undefined, needs to be PPC85XX or similar.
#endif

	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_disp_status_regs);
		return;
	}
#if CPU == PPC85XX /* don't know how to handle cpu variant */
    printk("  srr0/srr1:          0x%08x 0x%08x\n",
		VB_STATUS_REGS_ACCESS (p, srr0) ,
		VB_STATUS_REGS_ACCESS (p, srr1));

    printk("  mcsrr0/mcsrr1:      0x%08x 0x%08x\n",
		VB_STATUS_REGS_ACCESS (p, mcsrr0) ,
		VB_STATUS_REGS_ACCESS (p, mcsrr1));

    printk("  mcsr/mcar:          0x%08x 0x%08x\n",
		VB_STATUS_REGS_ACCESS (p, mcsr) ,
		VB_STATUS_REGS_ACCESS (p, mcar));

    printk("  dear/esr:           0x%08x 0x%08x\n",
		VB_STATUS_REGS_ACCESS (p, dear) ,
		VB_STATUS_REGS_ACCESS (p, esr));

    printk("  svr/pvr:            0x%08x 0x%08x\n",
		VB_STATUS_REGS_ACCESS (p, svr) ,
		VB_STATUS_REGS_ACCESS (p, pvr));

    printk("  hid0/hid1:          0x%08x 0x%08x\n",
		VB_STATUS_REGS_ACCESS (p, hid0) ,
		VB_STATUS_REGS_ACCESS (p, hid1));

    printk("  l1csr0/l1csr1:      0x%08x 0x%08x\n",
		VB_STATUS_REGS_ACCESS (p, l1csr0) ,
		VB_STATUS_REGS_ACCESS (p, l1csr1));

    printk("  bucsr:              0x%08x\n",
		VB_STATUS_REGS_ACCESS (p, bucsr));
#endif
}

/*
 * vbi_disp_ctrl_regs - display registers from a EREG_SET
 *
 * This routine display the contents of the PowerPC emulated status registers
 * structure on the console.
 *
 */

void vbi_disp_ctrl_regs(void)
{
    struct vb_control *p = VBI_CNTRL_ADDR_GET();

	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_disp_ctrl_regs);
		return;
	}
#if CPU == PPC85XX /* don't know how to handle cpu variant */
    printk("  srr0/srr1:           0x%08x 0x%08x\n",
		VB_CONTROL_REGS_ACCESS (p, srr0),
		VB_CONTROL_REGS_ACCESS (p, srr1));

    printk("  cr/r0:               0x%08x 0x%08x\n",
		VB_CONTROL_REGS_ACCESS (p, cr),
		VB_CONTROL_REGS_ACCESS (p, r0));
#endif
}
