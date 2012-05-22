/*
 * arch/arm/mach-spear13xx/include/mach/suspend.h
 *
 * Sleep mode defines for SPEAr13xx machine family
 *
 * Copyright (C) 2010 ST Microelectronics
 * AUTHOR : Deepak Sikri <deepak.sikri@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_SUSPEND_H
#define __MACH_SUSPEND_H

#include <mach/hardware.h>

#ifndef __ASSEMBLER__
extern void spear1340_sleep_mode(suspend_state_t state, unsigned long *saveblk);
extern unsigned int spear1340_sleep_mode_sz;
extern void spear13xx_sleep_mode(suspend_state_t state, unsigned long *saveblk);
extern unsigned int spear13xx_sleep_mode_sz;
extern void spear_wakeup(void);
extern unsigned int spear_wakeup_sz;
extern int spear_cpu_suspend(suspend_state_t, long);
extern void spear_clocksource_resume(void);
extern void spear_clocksource_suspend(void);
extern int spear_pcie_suspend(void);
extern int spear_pcie_resume(void);

#endif

/*
 * Define this only if you want cpu + bus matrix to be off during
 * suspend to ram i.e. echo mem > /sys/power/state
 */
/* #define CPU_PWR_DOMAIN_OFF */

/* SRAM related defines*/
#define SRAM_STACK_STRT_OFF	0x800
#define SRAM_STACK_SCR_OFFS	0x900
#define SPEAR_START_SRAM	SPEAR13XX_SYSRAM1_BASE
#define SPEAR_LIMIT_SRAM	(SPEAR_START_SRAM + SZ_4K - 16)
#define SPEAR_END_SRAM		(SPEAR_START_SRAM + SZ_4K - 4)
#define SPEAR_SRAM_START_PA	SPEAR_START_SRAM
#define SPEAR_SRAM_STACK_L2	(SPEAR_START_SRAM + SRAM_STACK_STRT_OFF - 0x30)
#define SPEAR_SRAM_STACK_PA	(SPEAR_START_SRAM + SRAM_STACK_STRT_OFF)
#define SPEAR_SRAM_SCR_REG	(SPEAR_START_SRAM + SRAM_STACK_SCR_OFFS)
#define SRAM_SCRATCH_PA		(SPEAR13XX_SYS_LOCATION)
/* SPEAr subsystem physical addresses */
#define MPMC_BASE_PA		SPEAR13XX_MPMC_BASE
#define MISC_BASE_PA		SPEAR13XX_MISC_BASE
#define GPIO_START_PA		SPEAR13XX_GPIO0_BASE
#define GPIO_START_UPD_PA	SPEAR13XX_UPD_BASE
#define UART_BASE_PA		SPEAR13XX_UART_BASE

#define DISABLE_I_C_M_V	0x1805
#define MISC_PLL_OFFS	0x214
#define MPMC_REG_END	0xff0
#define SRAM_SCR_REG	0xffc
#define PLL_VAL1	0x060a
#define PLL_VAL2	0x060e
#define PLL_VAL3	0x0606

#define	MODE_IRQ_32	0x12
#define	MODE_SVC_32	0x13
#define	MODE_ABT_32	0x17
#define	MODE_UND_32	0x1B
#define	MODE_SYS_32	0x1F
#define	MODE_BITS	0x1F

#ifdef __ASSEMBLER__
.macro	io_v2p, pa, va, tmp
	ldr	\tmp, =0xfff
	bic	\pa, \va, \tmp

	/*
	 * Following code uses VA to PA Translation Registers to
	 * translate the virtual address provided by a general-purpose
	 * register and store the corresponding physical address in the
	 * PA Register.
	 */
	mcr	p15, 0, \pa, c7, c8, 1
	mrc	p15, 0, \pa, c7, c4, 0
	bic	\pa, \pa, \tmp
	and	\tmp, \va, \tmp
	orr	\pa, \pa, \tmp

.endm

.macro	io_p2v, pa, va, tmp

	ldr	\tmp, =0xfff00000
	and	\va, \pa, \tmp
	lsr	\va, \va, #4

	ldr	\tmp, =0xffff
	and	\tmp, \pa, \tmp
	orr	\va, \va, \tmp

	ldr	\tmp, =0xf0000000
	orr	\va, \va, \tmp
.endm
#endif
#endif /* __MACH_SUSPEND_H */
