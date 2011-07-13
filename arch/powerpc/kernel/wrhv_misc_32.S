/*
 * Low level asm functions for guest implementation on powerpc
 * 
 * Copyright (c) 2009 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */

#include <linux/sys.h>
#include <asm/unistd.h>
#include <asm/errno.h>
#include <asm/reg.h>
#include <asm/page.h>
#include <asm/cache.h>
#include <asm/cputable.h>
#include <asm/mmu.h>
#include <asm/ppc_asm.h>
#include <asm/thread_info.h>
#include <asm/asm-offsets.h>
#include <asm/processor.h>
#include <vbi/interface.h>
#include <vbi/syscalls.h>
#include <asm/wrhv.h>

	.text

	.align	5
_GLOBAL(wrhv_int_lock)
	WRHV_INT_LOCK(r4,r5)
	blr

_GLOBAL(wrhv_int_lvl_get)
	WRHV_INT_LVL_GET(r3)
	blr

_GLOBAL(wrhv_int_unlock)
	WRHV_INT_UNLOCK(r3,r4)
	blr

/*
 * Write any modified data cache blocks out to memory.
 * Does not invalidate the corresponding data cache lines
 *
 * paravirt_clean_dcache_range(unsigned long start, unsigned long stop)
 */
#ifdef CONFIG_PARAVIRT_DCACHE_CLEAN
_GLOBAL(paravirt_clean_dcache_range)
	/*
	 * vbi_flush_dcache (void *start_addr, void *end_addr)
	 */
	#li	r5, 8
	subf	r4,r3,r4
	addi	r4, r4, 1
	bl	vbi_flush_dcache
	blr
#endif

_GLOBAL(paravirt__flush_dcache_icache)
	b	vb__flush_dcache_icache

_GLOBAL(paravirt_flush_dcache_range)
	b	vb_flush_dcache_range

_GLOBAL(paravirt__flush_icache_range)
	b	vb__flush_icache_range

_GLOBAL(paravirt__flush_dcache_icache_phys)
	b	vb__flush_dcache_icache_phys


