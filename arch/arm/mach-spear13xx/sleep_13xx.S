/*
 * linux/arch/arm/mach-spear13xx/sleep_13xx.S
 *
 * SPEAR13xx specific functions that will run in internal SRAM.
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

#define DDR_PLL_SREFRESH
#define MPMC_RESET
#define PWRDOMAINS_ON
#define GPIO_DDR_ISOLATE

/*
 * spear13xx_sleep_mode()
 * Forces SPEAr into sleep
 * r0: state
 */
.text
ENTRY(spear13xx_sleep_mode)
	mov	r10, r1		/* latch the address of cpu_resume */
	mov	r11, r0		/* latch the suspend state */

	ldr	r8, =IO_ADDRESS(SPEAR_SRAM_STACK_PA)
	/* Store Context ID, User R/W thread, secure or NS VBAR */
	mrc	p15, 0, r2, c10, c2, 0
	mrc	p15, 0, r3, c13, c0, 1
	mrc	p15, 0, r4, c13, c1, 2
	mrc	p15, 0, r5, c13, c2, 3
	mrc	p15, 0, r6, c13, c3, 4
	mrc	p15, 0, r7, c12, c0, 0
	stmia	r8!, {r2-r7}

	/* Store the Current CPSR */
	mrs	r0, cpsr
	stmia	r8!, {r0}

	/* Move to IRQ Mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_IRQ_32)
	msr	cpsr_c, r1
	mrs	r1, spsr

	/* Saving FIQ stack pointer on to Stack in DDR */
	stmia	r8!, {r1, r13, r14}

	/* Move to svc mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_SVC_32)
	msr	cpsr_c, r1
	mrs	r1, spsr
	stmia	r8!, {r1, r13, r14}	/* Saving IRQ stack pointer */

	/* Move to ABT mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_ABT_32)
	msr	cpsr_c, r1
	mrs	r1, spsr
	stmia	r8!, {r1, r13, r14}	/* Saving IRQ stack pointer */

	/* Move to undefined mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_UND_32)
	msr	cpsr_c, r1
	mrs	r1, spsr
	stmia	r8!, {r1, r13, r14}	/* Saving IRQ stack pointer */

	/* Move to sys_32 mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_SYS_32)
	msr	cpsr_c, r1
	mrs	r1, spsr
	stmia	r8!, {r1, r13, r14}	/* Saving IRQ stack pointer */

	/* Move back to svc mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_SVC_32)
	msr	cpsr_c, r1

	/* Latch the physical address of cpu_resume on to SRAM */
	io_v2p	r0, r10, r1
	stmia	r8!, {r0}

	/* Extract the physical address to jump to */
	adr	r0, mmu_off
	io_v2p	r2, r0, r1

	/* Disable MMU */
	mrc	p15, 0, r0, c1, c0, 0
	ldr	r1, =DISABLE_I_C_M_V
	bic	r0, r0, r1
	mcr	p15, 0, r0, c1, c0, 0
	/* Move the Physical address into PC */
	bx	r2
	nop
mmu_off:
	/* Put the DDR in self refresh mode */
	ldr	r6, =MISC_BASE_PA

	/* Program MPMC Control Status register in Misc Space */
	ldr	r0, [r6, #0x334]
	/* Set srefresh_enter bit(2) */
	orr	r0, r0, #0x4
	str	r0, [r6, #0x334]
wait_till_srefresh_on_r0:
	ldr	r0, [r6, #0x334]
	/* check for cke_status bit(13) */
	tst	r0, #0x2000
	beq	wait_till_srefresh_on_r0

#ifdef GPIO_DDR_ISOLATE
	/* enable gpio clock */
	ldr	r0, [r6, #0x274]
	orr	r0, r0, #0x00800000
	str	r0, [r6, #0x274]

	ldr	r6, =GPIO_START_PA
	/* Set the Direction of GPIO-2 as output */
	ldr	r0, [r6, #0x400]
	orr	r0, r0, #0xc
	str	r0, [r6, #0x400]

	/* Set the value of GPIO-2/3 */
	ldr	r0, [r6, #0x3fc]
	orr	r0, r0, #0xc
	str	r0, [r6, #0x3fc]
	ldr	r6, =MISC_BASE_PA
#endif

#ifdef MPMC_RESET
	ldr	r7, =MPMC_BASE_PA
	/* Stop the MPMC controller */
	ldr	r0, [r7, #0x2c]
	bic	r0, r0, #0x1000000
	str	r0, [r7, #0x2c]

	/* reset the MPMC controller */
	ldr	r1, [r6, #0x280]
	orr	r1, r1, #0x3
	str	r1, [r6, #0x280]

	/* Stop the MPMC clock */
	ldr	r1, [r6, #0x278]
	bic	r1, r1, #0x3
	str	r1, [r6, #0x278]
#endif
	/* Put the system in slow mode */
	ldr	r0, [r6, #0x200]
	bic	r0, r0, #0x4
	/* Set the apt mode bits(2:0) in SCCTRL register */
	orr	r0, r0, #0x2
	str	r0, [r6, #0x200]	/* System is now in slow mode */
wait_till_slow_mode:
	ldr	r0, [r6, #0x200]
	/* Wait for the mode to be updated */
	and	r0, r0, #0xF0000
	/* Poll the SCCTRL register status bits (6:3) */
	cmp	r0, #0xA0000
	bne wait_till_slow_mode

	/*
	 * Put the all the system pll's to off state
	 * The loop of count 3 is provided below to
	 * switch off the pll-1/2/3.
	 * r1 contains the offset for the pll control
	 * registers in the misc space.
	 * DDR pll-4 requires different processing.
	 */
	ldr	r1, =MISC_PLL_OFFS
	ldr	r2, =0x0	/* PLL Counter 1, 2, 3, 4 */
swoff_pll:
	ldr	r0, [r6, r1]
	/* Clear pll_enable bit(1) of PLL1_CTR register in Misc registers */
	bic	r0, r0, #0x02
	str	r0, [r6, r1]
	add	r1, #0xc
	add	r2, #0x1
	cmp	r2, #0x3	/* Switch off pll-1/2/3 */
	bne	swoff_pll

#ifdef DDR_PLL_SREFRESH
	/* Switch off pll-4 */
	ldr	r0, [r6, r1]
	/* Clear pll_enable bit(2) of PLL1_CTR register in Misc registers */
	bic	r0, r0, #0x04
	str	r0, [r6, r1]
#endif
#ifdef PWRDOMAINS_ON
	cmp	r11, #0x1
	beq	standby_mode

	/* Store the Resume Physical Address on Stack Secure location */
	adr	r0, spear_sleep_restore
	ldr	r3, =SRAM_SCR_REG
	ldr	r6, =SPEAR_SRAM_START_PA
	str	r0, [r6, r3]

	/* Switch on certan sections as suggested by HW team */
	/* Let the USB Host clock be on */
	ldr	r6, =MISC_BASE_PA
	ldr	r0, [r6, #0x274]
	orr	r0, r0, #0xE00
	str	r0, [r6, #0x274]
	ldr	r0, [r6, #0x100]

	/*
	 * Switch off the power domains.
	 * Clear the ack bit
	 */
	bic	r0, r0, #0xc000
	str	r0, [r6, #0x100]

	bic	r0, r0, #0x1000
	str	r0, [r6, #0x100]

wait_ack1:
	ldr	r0, [r6, #0x100]
	tst	r0, #0x4000
	beq	wait_ack1

	/* Clear the ack bit */
	bic	r0, r0, #0xc000
	str	r0, [r6, #0x100]

	bic	r0, r0, #0x0800
	str	r0, [r6, #0x100]
wait_ack2:
	ldr	r0, [r6, #0x100]
	tst	r0, #0x4000
	beq	wait_ack2

	/* Clear the ack bit */
	bic	r0, r0, #0xc000
	str	r0, [r6, #0x100]

#ifdef CPU_PWR_DOMAIN_OFF
	bic	r0, r0, #0x2400
	str	r0, [r6, #0x100]
wait_ack3:
	ldr	r0, [r6, #0x100]
	tst	r0, #0x4000
	beq	wait_ack3
#endif
#endif

standby_mode:
	wfi				@ wait for interrupt
	nop
spear_sleep_restore:
	nop
	ldr	r6, =MISC_BASE_PA
	ldr	r1, =MISC_PLL_OFFS

#ifdef DDR_PLL_SREFRESH
	/* Switch on PLL-4, strobe the pll also */
	ldr	r0, [r6, #0x238]
	ldr	r0, =PLL_VAL1
	str	r0, [r6, #0x238]
	ldr	r0, =PLL_VAL2
	str	r0, [r6, #0x238]
	ldr	r0, =PLL_VAL3
	str	r0, [r6, #0x238]
	ldr	r0, =PLL_VAL2
	str	r0, [r6, #0x238]
pll_lock_4:
	/* Set the pll_lock bit(0) in PLLX_CTR register in misc space*/
	ldr	r5, [r6, #0x238]
	and	r5, r5, #0x1
	/* Wait for pll lock status */
	cmp	r5, #0x1
	bne	pll_lock_4
#endif

	ldr	r2, =0x0	/* PLL Counter 1, 2, 3, 4 */
swon_pll_1_3:
	/* Switch on Pll-1/2/3 */
	ldr	r0, [r6, r1]
	orr	r0, r0, #0x2
	str	r0, [r6, r1]
pll_lock_1_3:
	/* Set the pll_lock bit(0) in PLLX_CTR register in misc space*/
	ldr	r5, [r6, r1]
	and	r5, r5, #0x1
	/* Wait for pll lock status */
	cmp	r5, #0x1
	bne	pll_lock_1_3

	/* Loop for all the pll's */
	add	r1, #0xc
	add	r2, #0x1
	cmp	r2, #0x3	/* Switch on till pll-3 */
	bne	swon_pll_1_3

	/* Put the system in normal mode */
	ldr	r0, [r6, #0x200]
	bic	r0, r0, #0x7
	/* Set the apt mode bits(2:0) in SCCTRL register */
	orr	r0, r0, #0x4
	str	r0, [r6, #0x200]	/* System is now in slow mode */
wait_till_normal_mode:
	ldr	r0, [r6, #0x200]
	/* Wait for the mode to be updated */
	and	r0, r0, #0xF0000
	/* Poll the SCCTRL register status bits (6:3) */
	cmp	r0, #0xf0000
	bne wait_till_normal_mode

	ldr	r6, =SPEAR_SRAM_START_PA
#ifdef MPMC_RESET
	ldr	r7, =MISC_BASE_PA
	/* Enable the MPMC clock */
	ldr	r1, [r7, #0x278]
	orr	r1, r1, #0x3
	str	r1, [r7, #0x278]
	/* Provide Software Reset to MPMC */
	ldr	r1, [r7, #0x280]
	orr	r1, r1, #0x3
	str	r1, [r7, #0x280]
	/* Enable MPMC back */
	bic	r1, r1, #0x3
	str	r1, [r7, #0x280]
#endif
	/* Restore the MPMC registers */
	ldr	r7, =MPMC_BASE_PA
	mov	r0, #0
	ldr	r3, =MPMC_REG_END
mpmc_restore_regs:
	ldr	r1, [r6, r3]
	cmp	r0, #0x2c
	bne	ddf
	nop
	nop
	bic	r1, r1, #0x1000000
ddf:
	str	r1, [r7, r0]
	sub	r3, r3, #4
	add	r0, r0, #4
	cmp	r0, #0x320
	bne	mpmc_restore_regs

	/* Enable MPMC */
	ldr	r1, [r7, #0x2c]
	orr	r1, r1, #0x1000000
	str	r1, [r7, #0x2c]
	dsb
	isb

#ifdef GPIO_DDR_ISOLATE
	/* Disable MPMC reset */
	ldr	r6, =MISC_BASE_PA
	mov	r0, #0
	str	r0, [r6, #0x280]

	ldr	r7, =GPIO_START_PA
	/* Set the Direction of GPIO-2 as output */
	ldr	r0, [r7, #0x400]
	orr	r0, r0, #0xc
	str	r0, [r7, #0x400]

	/* Set the value of GPIO-2 DDR CLKEN*/
	ldr	r0, [r7, #0x3fc]
	bic	r0, r0, #0x7
	str	r0, [r7, #0x3fc]

	mov	r0, #0
ddr_out_srefresh:
	/* Add some delay */
	add	r0, r0, #0x1
	cmp	r0, #0x60000
	bne	ddr_out_srefresh
#endif
	/* Move the DDR out of self refresh mode */
	ldr	r7, =MPMC_BASE_PA
	ldr	r8, =SPEAR_SRAM_STACK_PA

	/* Clearance is required in the mpmc space */
	ldr	r1, [r7, #0x2c]
	/*
	 * Clear bit srefresh bit (2) of MPMC_11 register
	 * The misc wrapper does not works fine by itself till
	 * this bit is also cleared.
	 */
	bic	r1, r1, #0x10000

	/* Program MPMC Control Status register in Misc Space */
	ldr	r0, [r6, #0x334]
	/* Clear srefresh_enter bit(2) */
	bic	r0, r0, #0x4
	str	r0, [r6, #0x334]
	str	r1, [r7, #0x2c]

wait_for_srefresh_exit_r0:
	ldr	r0, [r6, #0x334]
	tst	r0, #0x2000
	bne	wait_for_srefresh_exit_r0

	/* Store Context ID, User R/W thread, secure or NS VBAR */
	ldmia	r8!, {r2-r7}
	mcr	p15, 0, r2, c10, c2, 0
	mcr	p15, 0, r3, c13, c0, 1
	mcr	p15, 0, r4, c13, c1, 2
	mcr	p15, 0, r5, c13, c2, 3
	mcr	p15, 0, r6, c13, c3, 4
	mcr	p15, 0, r7, c12, c0, 0

	ldmia	r8!, {r4}
	msr	cpsr, r4
	mov	r0, r4

	/* Move to IRQ mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_IRQ_32)
	msr	cpsr_c, r1
	ldmia	r8!, {r1, r13, r14}
	msr	spsr, r1

	/* Move to svc mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_SVC_32)
	msr	cpsr_c, r1
	ldmia	r8!, {r1, r13, r14}
	msr	spsr, r1

	/* Move to ABT mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_ABT_32)
	msr	cpsr_c, r1
	ldmia	r8!, {r1, r13, r14}
	msr	spsr, r1

	/* Move to undefined mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_UND_32)
	msr	cpsr_c, r1
	ldmia	r8!, {r1, r13, r14}
	msr	spsr, r1

	/* Move to sys_32 mode */
	orr	r1, r0, #(MODE_SYS_32)
	msr	cpsr_c, r1
	ldmia	r8!, {r0, r13, r14}
	msr	spsr_cxsf, r0

	/*
	 * Invalidate all instruction caches to PoU
	 */
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0
	mcr	p15, 0, r0, c7, c5, 6	@ Invalidate branch predictor array
	mcr	p15, 0, r0, c7, c5, 7
	mcr	p15, 0, r0, c8, c5, 0	@ Invalidate instruction TLB
	mcr	p15, 0, r0, c8, c6, 0	@ Invalidate data TLB

	msr	cpsr, r4
	ldmia	r8!, {r0}
	mov	pc, r0
	nop
	.ltorg
	.align
ENTRY(spear13xx_sleep_mode_sz)
	.word	. - spear13xx_sleep_mode
