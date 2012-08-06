/*
 * arch/arm/plat-spear/pll_clk.S
 *
 * SPEAR3xx and SPEAR6xx specific functions that will run in
 * cache. These funstions intend to configure the PLL.
 *
 * Copyright (ST) 2010 Deepak Sikri <deepak.sikri@.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/hardware.h>
#include <mach/suspend.h>

#if defined(CONFIG_ARCH_SPEAR3XX) || defined(CONFIG_ARCH_SPEAR6XX)
.text
ENTRY(vco_set_rate)
	stmfd	sp!, {r0-r12, lr}

	/* Lock down the TLB entry to the current victim */
	mrc	p15, 0, r3, c10, c0, 0	/* read the lockdown register */
	orr	r3, r3, #1		/* set the preserved bit */
	mcr	p15, 0, r3, c10, c0, 0	/* write to the lockdown register */

	/*
	 * set r4 to the value of the address to be locked down.
	 * Invalidate the TLB single entry in TLB to ensure that
	 * the locked address is not already in TLB.
	 * MPMC, System Controller & Miscellaneous register address
	 * are locked down below.
	 */

	ldr	r4, MPMC_BASE_VA
	/* Invalidate the MPMC virtual address in TLB. */
	mcr	p15, 0, r4, c8, c7, 1
	/* TLB will miss and entry will be reloaded */
	ldr	r4, [r4]
	/* read the lockdown register (victim will have rloaded) */
	mrc	p15, 0, r3, c10, c0, 0

	ldr	r4, SYS_CTRL_BASE_VA
	/* Invalidate the System controller virtual address in TLB */
	mcr	p15, 0, r4, c8, c7, 1
	/* TLB will miss and entry will be reloaded */
	ldr	r4, [r4]
	/* read the lockdown register (victim will have rloaded) */
	mrc	p15, 0, r3, c10, c0, 0

	ldr	r4, MISC_BASE_VA
	/* Invalidate the Miscellaneous registers virtual address in TLB */
	mcr	p15, 0, r4, c8, c7, 1
	/* TLB will miss and entry will be reloaded */
	ldr	r4, [r4]
	/* read the lockdown register (victim will have rloaded) */
	mrc	p15, 0, r3, c10, c0, 0

	/* clear preserve bit */
	bic	r3, r3, #1
	/* write to the lockdown register */
	mcr	p15, 0, r3, c10, c0, 0

	ldr	r7, MPMC_BASE_VA
	ldr	r8, SYS_CTRL_BASE_VA
	ldr	r6, MISC_BASE_VA
	/* Prefetch certain instructions in the cache. */
	adr	r4, cache_prefetch_start1
	adr	r5, cache_prefetch_end1
	mvn	r3, #0x1F
	ands	r4, r3, r4
	/* Lock Instructions in i-cache */
fetch_loop:
	/*
	 * copy a cache-line-sized block of main memory to a cache
	 * line in the I-cache.
	 */
	mcr	p15, 0, r4, c7, c13, 1
	cmp	r4, r5
	addls	r4, r4, #0x20
	bls	fetch_loop
cache_prefetch_start1:
	/* Put SDRAM in self-refresh mode */
	ldr	r3, [r7, #0x1c]
	/* Clear START bit(24) of MEMCTL_GP_03 register in MPMC */
	ldr	r4, =0x1000000
	bic	r3, r3, r4
	str	r3, [r7, #0x1c]

	ldr	r3, [r7, #0xe4]
	ldr	r4, =0xffff0000
	/* Latch the current self refresh time */
	mov	r9, r3
	/* Increase the self refresh exit time */
	bic	r3, r3, r4
	ldr	r4, =0xffff
	/* Program the SDRAM self refresh exit time on read command */
	orr	r3, r3, r4, LSL #16
	str	r3, [r7, #0xe4]

	ldr	r3, [r7, #0x1c]
	/* Set the SREFRESH bit(16) */
	ldr	r4, =0x10000
	orr	r3, r3, r4
	str	r3, [r7, #0x1c]

	/* Put the system in slow mode, use system controller */
	ldr	r3, [r8]
	bic	r3, r3, #0x7
	/* Set the apt mode bits(2:0) in SCCTRL register */
	orr	r3, r3, #0x2
	str	r3, [r8]

wait_till_slow_mode:
	ldr	r3, [r8]
	and	r3, r3, #0x78	/* Wait for the mode to be updated */
	cmp	r3, #0x10	/* Poll the SCCTRL register status bits (6:3) */
	bne wait_till_slow_mode

	/*
	 * reprogram the m(r0), p(r2), n(r1) values in the PLL
	 * control registers (PLL_FRQ register in misc space).
	 */
	ldr	r3, [r6, #0x0c]
	bic	r3, r3, #0x00ff
	/* Program the PLL post divisor: p */
	orr	r3, r3, r2
	str	r3, [r6, #0x0c]

	ldr	r3, [r6, #0x0c]
	ldr	r4, =0xffff0000
	bic	r3, r3, r4
	bic	r3, r3, #0x0700
	/* Program the PLL pre divisor: n */
	orr	r3, r3, r1, LSL #8
	/* Program the PLL feedback divisor: m */
	orr	r3, r3, r0, LSL #24
	str	r3, [r6, #0x0c]

	/* Move the system in Normal mode, use system controller */
	ldr	r3, [r8, #0x0]
	ldr	r4, =0xfffffff8
	/* Set the apt mode bits(2:0) in SCCTRL register */
	and	r3, r3, r4
	orr	r3, r3, #0x4
	str	r3, [r8, #0x0]

wait_till_normal_mode:
	ldr	r3, [r8, #0x0]
	and	r3, r3, #0x78
	cmp	r3, #0x20	/* Poll the SCCTRL register status bits (6:3) */
	bne wait_till_normal_mode

	/* Exit DDR-SDRAM from self-refresh mode */
	ldr	r10, MPMC_BASE_VA
	/* Clear the SREFRESH bit(16) */
	ldr	r3, [r10, #0x1c]
	ldr	r4, =0x10000
	bic	r3, r3, r4
	str	r3, [r10, #0x1c]
	/* Restore the SDRAM self refresh exit time on read command */
	mov	r3, r9
	str	r3, [r7, #0xe4]
	/* Begin the command processing in controller */
	ldr	r4, =0x1000000
	orr	r3, r3, r4
	/* Set START bit(24) of MEMCTL_GP_03 register in MPMC*/
	str	r3, [r10, #0x1c]

	ldmfd	sp!, {r0-r12, pc}

/* This is the end of the code to be copied */

SYS_CTRL_BASE_VA :
	.word IO_ADDRESS(SYS_CTRL_BASE_PA)
MPMC_BASE_VA :
	.word IO_ADDRESS(MPMC_BASE_PA)
MISC_BASE_VA :
	.word IO_ADDRESS(MISC_BASE_PA)
cache_prefetch_end1 :

#elif defined(CONFIG_ARCH_SPEAR13XX)
.text
ENTRY(vco_set_rate)
#endif
