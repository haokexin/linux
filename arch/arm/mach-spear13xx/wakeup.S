/*
 * linux/arch/arm/mach-spear13xx/wakeup.S
 *
 * SPEAR13xx specific functions that will run in internal SRAM.
 * The functions are used in power management for resume.
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

#define	SRAM_SCR_REG	0xffc
#define ASSOC_MASK	0x3ff
#define NUMSET_MASK	0x7fff

.text
ENTRY(spear_wakeup)
	mrc	p15, 0, r0, c0, c0, 5
	and	r0, r0, #0xf
	cmp	r0, #0x0
	bne	cpux_sleep
back:
	bl	cache_invalidate
	nop
	ldr	r3, =SRAM_SCR_REG
	ldr	r6, =SPEAR_SRAM_START_PA
	ldr	r0, [r6, r3]
	bx	r0
cpux_sleep:
	/* Core-2 executed this code */
	ldr	r7, =SRAM_SCRATCH_PA
	ldr	r1, =0xFFFFFFFF
	str	r1, [r7]
	dsb
check:
	wfe
	ldr	r1, [r7]
	tst	r1, #3
	bne	check
	bl	cache_invalidate

	ldr	r7, =SRAM_SCRATCH_PA
	ldr	r1, [r7]
	bx	r1

/* L1 invalidate */
cache_invalidate:
	/* Data Cache Invalidate */
	/* read clidr */
	mrc	p15, 1, r0, c0, c0, 1
	/* extract loc from clidr */
	ands	r3, r0, #0x7000000
	/* left align loc bit field */
	mov	r3, r3, lsr #23
	/* if loc is 0, then no need to clean */
	beq	finished
	/* start clean at cache level 0 */
	mov	r10, #0
loop1:
	/* work out 3x current cache level */
	add	r2, r10, r10, lsr #1
	/* extract cache type bits from clidr*/
	mov	r1, r0, lsr r2
	/* mask of the bits for current cache only */
	and	r1, r1, #7
	/* see what cache we have at this level */
	cmp	r1, #2
	/* skip if no cache, or just i-cache */
	blt	skip
	/* select current cache level in cssr */
	mcr	p15, 2, r10, c0, c0, 0
	/* isb to sych the new cssr&csidr */
	isb
	/* read the new csidr */
	mrc	p15, 1, r1, c0, c0, 0
	/* extract the length of the cache lines */
	and	r2, r1, #7
	/* add 4 (line length offset) */
	add	r2, r2, #4
	ldr	r4, =ASSOC_MASK
	/* find maximum number on the way size */
	ands	r4, r4, r1, lsr #3
	/* find bit position of way size increment */
	clz	r5, r4
	ldr	r7, =NUMSET_MASK
	/* extract max number of the index size*/
	ands	r7, r7, r1, lsr #13
loop2:
	mov	r9, r4
	/* create working copy of max way size*/
loop3:
	/* factor way and cache number into r11 */
	orr	r11, r10, r9, lsl r5
	/* factor index number into r11 */
	orr	r11, r11, r7, lsl r2
	/*clean & invalidate by set/way */
	mcr	p15, 0, r11, c7, c6, 2
	/* decrement the way*/
	subs	r9, r9, #1
	bge	loop3
	/*decrement the index */
	subs	r7, r7, #1
	bge	loop2
skip:
	add	r10, r10, #2
	/* increment cache number */
	cmp	r3, r10
	bgt	loop1
finished:
	/* Switch back to cache level 0 */
	mov	r10, #0
	/* select current cache level in cssr */
	mcr	p15, 2, r10, c0, c0, 0
	dsb
	isb
	mov	r0, #0
	/* L1 Invalidation complete */

	/* Disable SMP, prefetch */
	mcr	p15, 0, r0, c1, c0, 1
	dsb
	/* Instruction cache Invalidate */
	mcr	p15, 0, r0, c7, c5, 0
	/* Invalidate branch pred array */
	mcr	p15, 0, r0, c7, c5, 6
	/* Invalidate unified TLB */
	mcr	p15, 0, r0, c8, c7, 0
	mcr	p15, 0, r0, c8, c5, 0
	mcr	p15, 0, r0, c8, c6, 0
	/* memory barrier to sync up the things */
	mcr	p15, 0, r0, c7, c10, 4
	dsb
	isb
	mov	r0, #0x1800
	mcr	p15, 0, r0, c1, c0, 0
	dsb
	isb

	bx	lr
	.ltorg
	.align

ENTRY(spear_wakeup_sz)
	.word	. - spear_wakeup
	.text
