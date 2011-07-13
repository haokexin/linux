/*
 * Contains the definition of registers common to all PowerPC variants.
 * If a register definition has been changed in a different PowerPC
 * variant, we will case it in #ifndef XXX ... #endif, and have the
 * number used in the Programming Environments Manual For 32-Bit
 * Implementations of the PowerPC Architecture (a.k.a. Green Book) here.
 */

#ifndef _ASM_POWERPC_REG_PARAVIRT_H
#define _ASM_POWERPC_REG_PARAVIRT_H
#ifdef __KERNEL__

/* default native macros */
#define PARAVIRT_MFSPR_SPRG3(a) mfspr a,SPRN_SPRG3 

/* pickup individual hypervisor specific regs */
#ifdef CONFIG_WRHV
#include <asm/reg_wrhv.h>
#endif

#endif /* __KERNEL__ */
#endif /* _ASM_POWERPC_REG_PARAVIRT_H */
