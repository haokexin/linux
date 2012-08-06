/*
 * linux/arch/arm/plat-spear/cpu_suspend.S
 *
 * SPEAR6xx/3xx specific functions that will call the kernel handlers to
 * put the latch the cpu specific configurations on the DDR before
 * moving into sleep.
 * The functions are used in power management.
 *
 * Copyright (C) 2010 ST MicroElectronics
 * Deepak Sikri <deepak.sikri@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/suspend.h>

.text
/*
 * spear_cpu_suspend()
 * Put the CPU into suspend mode.
 */
ENTRY(spear_cpu_suspend)
	stmfd	sp!, {r4-r12, lr}		@ save registers on stack
	ldr	r8, =IO_ADDRESS(SPEAR_SRAM_STACK_PA)
	stmia	r8!, {r0}
	ldr	r3, =resume_with_mmu
	bl	cpu_suspend

	/*
	 * Do the necessary platform specific power management
	 * from the SRAM.
	 */
	ldr	r3, =IO_ADDRESS(SPEAR_SRAM_STACK_PA)
	ldmia	r3!, {r0}
	bl	spear_sys_suspend

resume_with_mmu:
	ldmfd	sp!, {r4-r12, pc}		@ restore regs and return
	nop
	.ltorg
