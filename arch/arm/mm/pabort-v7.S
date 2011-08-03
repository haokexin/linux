#include <linux/linkage.h>
#include <asm/assembler.h>
#ifdef CONFIG_WRHV
#include <asm/asm-offsets.h>
#endif

/*
 * Function: v7_pabort
 *
 * Params  : r0 = address of aborted instruction
 *
 * Returns : r0 = address of abort
 *	   : r1 = IFSR
 *
 * Purpose : obtain information about current prefetch abort.
 */

	.align	5
ENTRY(v7_pabort)
#ifdef CONFIG_WRHV
	ldr	r1, =wr_status			@ ptr to hv status region
	ldr	r1, [r1]
	ldr	r0, [r1, #STAT_SPACE_IFAR]	@ get ifar
	ldr	r1, [r1, #STAT_SPACE_IFSR]	@ get ifsr
#else
	mrc	p15, 0, r0, c6, c0, 2		@ get IFAR
	mrc	p15, 0, r1, c5, c0, 1		@ get IFSR
#endif
	mov	pc, lr
ENDPROC(v7_pabort)
