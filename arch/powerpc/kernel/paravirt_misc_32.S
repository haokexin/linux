/*
 *  misc operations, for paravirt implementation.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 *
 */

#include <asm/ppc_asm.h>
#include <asm/asm-offsets.h>

_GLOBAL(paravirt_clean_dcache_range)
.weak paravirt_clean_dcache_range
		b	native_clean_dcache_range

_GLOBAL(paravirt__flush_dcache_icache)
.weak paravirt__flush_dcache_icache
        b       native__flush_dcache_icache

_GLOBAL(paravirt_flush_dcache_range)
.weak paravirt_flush_dcache_range
        b       native_flush_dcache_range

_GLOBAL(paravirt__flush_icache_range)
.weak paravirt__flush_icache_range
        b       native__flush_icache_range

_GLOBAL(paravirt__flush_dcache_icache_phys)
.weak paravirt__flush_dcache_icache_phys
        b       native__flush_dcache_icache_phys
