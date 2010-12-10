/* show.c - vbi ARM show routines */

/*
 * Copyright (c) 2010 Wind River Systems, Inc.
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

/*******************************************************************************
 *
 * vbi_disp_status_regs - display registers from a vb_status
 *
 * This routine display the contents of the ARM emulated status registers
 * structure on the console.
 *
 */

void vbi_disp_status_regs(void)
    {
    struct vb_status *p = VBI_STATUS_ADDR_GET();

    printk ("  dfsr/ifsr:            0x%08x 0x%08x\n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, dfsr) ,
        (unsigned int)VB_STATUS_REGS_ACCESS (p, ifsr));

    printk ("  dfar/ifar:            0x%08x 0x%08x\n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, dfar) ,
        (unsigned int)VB_STATUS_REGS_ACCESS (p, ifar));

    printk ("  mode/intVector:       0x%08x 0x%08x\n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, mode) ,
        (unsigned int)VB_STATUS_REGS_ACCESS (p, intVector));

    printk ("  cpRights/cpGrants:    0x%08x 0x%08x\n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, coprocRights) ,
        (unsigned int)VB_STATUS_REGS_ACCESS (p, coprocGrants));

    printk ("  User Mode spsr:       0x%08x \n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[0].spsr));

    printk ("  User Mode sp/lr:      0x%08x 0x%08x\n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[0].sp) ,
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[0].lr));

    printk ("  FIQ Mode spsr:        0x%08x \n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[1].spsr));

    printk ("  FIQ Mode sp/lr:       0x%08x 0x%08x\n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[1].sp) ,
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[1].lr));

    printk ("  IRQ Mode spsr:        0x%08x \n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[2].spsr));

    printk ("  IRQ Mode sp/lr:       0x%08x 0x%08x\n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[2].sp) ,
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[2].lr));

    printk ("  SVC Mode spsr:        0x%08x \n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[3].spsr));

    printk ("  SVC Mode sp/lr:       0x%08x 0x%08x\n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[3].sp) ,
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[3].lr));

    printk ("  MON Mode spsr:        0x%08x \n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[6].spsr));

    printk ("  MON Mode sp/lr:       0x%08x 0x%08x\n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[6].sp) ,
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[6].lr));

    printk ("  Abort Mode spsr:      0x%08x \n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[7].spsr));

    printk ("  Abort Mode sp/lr:     0x%08x 0x%08x\n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[7].sp) ,
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[7].lr));

    printk ("  Undef Mode spsr:      0x%08x \n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[0x0B].spsr));

    printk ("  Undef Mode sp/lr:     0x%08x 0x%08x\n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[0x0B].sp) ,
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[0x0B].lr));

    printk ("  SYS Mode spsr:        0x%08x \n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[0x0F].spsr));

    printk ("  SYS Mode sp/lr:       0x%08x 0x%08x\n",
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[0x0F].sp) ,
        (unsigned int)VB_STATUS_REGS_ACCESS (p, modeSpecificReg[0x0F].lr));


       }

/*******************************************************************************
 *
 * vbi_disp_ctrl_regs - display registers from a vb_control
 *
 * This routine display the contents of the ARM emulated status registers
 * structure on the console.
 *
 */

void vbi_disp_ctrl_regs(void)
    {
    struct vb_control * p = VBI_CNTRL_ADDR_GET();

    printk ("  cpsr/spsr:            0x%08x 0x%08x\n",
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, cpsr) ,
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, spsr));

    printk ("  r0/r1:                0x%08x 0x%08x\n",
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, r0) ,
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, r1));

    printk ("  r2/r3:                0x%08x 0x%08x\n",
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, r2) ,
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, r3));

    printk ("  r4/r5:                0x%08x 0x%08x\n",
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, r4) ,
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, r5));

    printk ("  r6/r7:                0x%08x 0x%08x\n",
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, r6) ,
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, r7));

    printk ("  r8/r9:                0x%08x 0x%08x\n",
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, r8) ,
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, r9));

    printk ("  r10/r11:              0x%08x 0x%08x\n",
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, r10) ,
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, r11));

    printk ("  r12/sp:               0x%08x 0x%08x\n",
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, r12) ,
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, sp));

    printk ("  lr/pc:                0x%08x 0x%08x\n",
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, lr) ,
        (unsigned int)VB_CONTROL_REGS_ACCESS (p, pc));

    }
