/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2, or (at your option) any
 *  later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  Copyright (C) 2009 Wind River Systems, Inc.
 *
 * Contains the definition of registers common to all PowerPC variants.
 * If a register definition has been changed in a different PowerPC
 * variant, we will case it in #ifndef XXX ... #endif, and have the
 * number used in the Programming Environments Manual For 32-Bit
 * Implementations of the PowerPC Architecture (a.k.a. Green Book) here.
 */

#ifndef _ASM_POWERPC_REG_WRHV_H
#define _ASM_POWERPC_REG_WRHV_H
#ifdef __KERNEL__

#include <linux/stringify.h>
#include <asm/cputable.h>
#include <vbi/interface.h>
#include <vbi/syscalls.h>

/* macro used in on entry_32.S */
#define PARAVIRT_ENABLE_MSR_EE      WRHV_INT_UNLOCK(r10,r11)
#define PARAVIRT_DISABLE_MSR_EE     WRHV_INT_LOCK(r10,r3)

/* macro used in misc.S */
#undef PARAVIRT_MFSPR_SPRG3
#define PARAVIRT_MFSPR_SPRG3(a)  WRHV_MFSPRG3(a)

/* macro used in arch/powerpc/kernel/traps.c */
#define PARAVIRT_DISABLE_INST_COMPLETION       do{ } while (0)
#define PARAVIRT_CLEAR_INST_COMPLETION                 do{ } while (0)

#ifdef __ASSEMBLY__
#define wrhv_supervisor (0xF0002000 +VB_CONTROL_RESERVED7)
.extern var(wrhv_sprg3)
.extern var(wrhv_user)
.extern var(wrhv_pir)

#ifdef CONFIG_SMP
/*temporary solution for MFSPRG3,PIR ,wrhv-reserverd:0xf0002068 */
#define WRHY_SPRG3 (0xF0002000 + VB_CONTROL_RESERVED8)
#define WRHV_MFSPRG3(rd)	\
	lis	rd,WRHY_SPRG3@ha;	\
	lwz	rd,WRHY_SPRG3@l(rd)

#define WRHV_MTSPRG3(rs,tmpr)	\
	lis	tmpr,WRHY_SPRG3@ha;             \
	stw	rs,WRHY_SPRG3@l(tmpr)

#else

#if defined(CONFIG_PPC85xx_VT_MODE)

#define WRHV_MFSPRG3(rd)			\
	mfspr   rd, SPRN_SPRG3

#define WRHV_MTSPRG3(rs, tmpr)			\
	mtspr   SPRN_SPRG3, rs

#ifdef CONFIG_SMP
#define WRHV_MFPIR(rd)				\
	mfspr   rd, SPRN_PIR
#endif

#else

#define WRHV_MFSPRG3(rd)                        \
	lis	rd,wrhv_sprg3@ha;               \
	lwz	rd,wrhv_sprg3@l(rd)

#define WRHV_MTSPRG3(rs,tmpr)                   \
	lis	tmpr,wrhv_sprg3@ha;             \
	stw	rs,wrhv_sprg3@l(tmpr)
#endif

#ifdef CONFIG_SMP
#define WRHV_MFPIR(rd)				\
	lis	rd,wrhv_pir@ha;		\
	lwz	rd,wrhv_pir@l(rd);
#endif

#endif

#ifdef CONFIG_PPC85xx_VT_MODE
#define WRHV_INT_LOCK(tmpr1,tmpr2)			\
	wrteei	0;
#else
#define WRHV_INT_LOCK(tmpr1,tmpr2)                      \
	li	tmpr2,-1;                               \
	lis	tmpr1,wr_control@ha;                   \
	lwz	tmpr1,wr_control@l(tmpr1);             \
	stw	tmpr2,VB_CONTROL_INT_DISABLE(tmpr1)
#endif

#ifdef CONFIG_PPC85xx_VT_MODE
#define WRHV_INT_UNLOCK(tmpr1,tmpr2)			\
	wrteei	1;					\
	lis	tmpr1,wr_status@ha;			\
	lwz	tmpr1,wr_status@l(tmpr1);		\
	lwz	tmpr1,VB_STATUS_INT_PENDING(tmpr1);	\
	cmpwi	0,tmpr1,0;				\
	beq	1f;					\
	mr	tmpr2,r0;				\
	lis	r0,VBI_SYS_int_enable@h;		\
	ori	r0,r0,VBI_SYS_int_enable@l;		\
	sc	1;					\
	mr	r0,tmpr2;				\
1:
#else
#define WRHV_INT_UNLOCK(tmpr1,tmpr2)                    \
	lis	tmpr1,wr_control@ha;                   \
	lwz	tmpr1,wr_control@l(tmpr1);             \
	li	tmpr2,0;                                \
	stw	tmpr2,VB_CONTROL_INT_DISABLE(tmpr1);    \
	lis	tmpr1,wr_status@ha;                    \
	lwz	tmpr1,wr_status@l(tmpr1);              \
	lwz	tmpr1,VB_STATUS_INT_PENDING(tmpr1);     \
	cmpwi	0,tmpr1,0;                              \
	beq	1f;                                     \
	mr	tmpr1,r0;                               \
	lis	r0,VBI_SYS_int_enable@h;                \
	ori	r0,r0,VBI_SYS_int_enable@l;             \
	sc;                                             \
	mr	r0,tmpr1;                               \
1:
#endif

#ifdef CONFIG_PPC85xx_VT_MODE
#define WRHV_INT_LVL_GET(rd)				\
	li	r4,1;					\
	mfmsr	rd;					\
	rlwinm.	rd,rd,0,16,16;      /* test EE bit */	\
	bne	1f;					\
	li	r4,0;					\
1:	mr	rd,r4                                  
#else
#define WRHV_INT_LVL_GET(rd)                            \
	lis	rd,wr_control@ha;                      \
	lwz	rd,wr_control@l(rd);                   \
	lwz	rd,VB_CONTROL_INT_DISABLE(rd)
#endif

#ifndef CONFIG_PPC85xx_VT_MODE
#define WRHV_FIX_MSR(msr,tmpr)                                  \
	rlwinm	msr,msr,0,18,15; /* Clear EE & PR bits */       \
	WRHV_INT_LVL_GET(tmpr);                         \
	cmpwi	0,tmpr,0;                                       \
	bne	1f;                                             \
	ori	msr,msr,MSR_EE;                                 \
1:	lis	tmpr,wrhv_supervisor@ha;                        \
	lwz	tmpr,wrhv_supervisor@l(tmpr);                   \
	cmpwi	0,tmpr,0;                                       \
	bne	2f;                                             \
	ori	msr,msr,MSR_PR;                                 \
2:

#define WRHV_LOAD_MSR(msr,tmpr1,tmpr2)                          \
	li	tmpr2,0;                                        \
	rlwinm.	tmpr1,msr,0,16,16;      /* test EE bit */       \
	bne	1f;                     /* IT unlocked? */      \
	li	tmpr2,-1;                                       \
1:	lis	tmpr1,wr_control@ha;                           \
	lwz	tmpr1,wr_control@l(tmpr1);                     \
	stw	tmpr2,VB_CONTROL_NEW_INT_DISABLE(tmpr1);        \
	stw	msr,VB_CONTROL_SRR1(tmpr1);                     \
	li	tmpr2,1;                                        \
	rlwinm.	tmpr1,msr,0,17,17;      /* test PR bit */       \
	beq	2f;                     /* priv. mode? */       \
	li	tmpr2,0;                                        \
2:	WRHV_SET_SUP_MODE(tmpr1,tmpr2)

#define WRHV_FIX_MSR2(msr,tmpr)                         \
	rlwinm	msr,msr,0,18,15; /* Clear EE & PR bits */       \
	lis	tmpr,wr_status@ha;                             \
	lwz	tmpr,wr_status@l(tmpr);                        \
	lwz	tmpr,VB_STATUS_OLD_INT_DISABLE(tmpr);           \
	cmpwi	0,tmpr,0;                                       \
	bne	1f;                                             \
	ori	msr,msr,MSR_EE;                                 \
1:	lis	tmpr,wrhv_supervisor@ha;                        \
	lwz	tmpr,wrhv_supervisor@l(tmpr);                   \
	cmpwi	0,tmpr,0;                                       \
	bne	2f;                                             \
	ori	msr,msr,MSR_PR;                                 \
2:

#define WRHV_SET_SUP_MODE(tmpr,rs)                              \
	lis	tmpr,wrhv_supervisor@ha;                        \
	stw	rs,wrhv_supervisor@l(tmpr)

#define WRHV_SUP_MODE_GET(rd)                                   \
	lis	rd,wrhv_supervisor@ha;                          \
	lwz	rd,wrhv_supervisor@l(rd)
#endif
#else	/* __ASSEMBLY__ */
#define	__PPC_SPR(spr)	(((spr >> 5) & 0x1f) | ((spr & 0x1f) << 5))
#ifdef CONFIG_PPC85xx_VT_MODE
#define WRHV_MFSPR_NO_PERMISSION(rn)					\
({									\
	unsigned long code = 0x7c0002a6 |				\
		(__PPC_SPR(rn) << 11) | __PPC_RS(4);  			\
	unsigned long value;						\
	asm volatile (							\
		"mr 3, %1\n"						\
		"mfspr 4," __stringify(rn) "\n"				\
		"mr %0, 4\n"						\
		:"=r" (value)						\
		:"r" (code)						\
		:"r3", "r4"						\
	);								\
	value;								\
})

#define WRHV_MFSPR(rn)					\
({							\
	unsigned long value;				\
	asm volatile (					\
		"mfspr %0," __stringify(rn)		\
		:"=r" (value)				\
	);						\
	value;						\
})

#define wrhv_mfspr(rn)					\
({							\
	unsigned long value;				\
	switch (rn) {					\
	case SPRN_DBCR0:				\
	case SPRN_DBSR:					\
	case SPRN_IAC1:					\
	case SPRN_IAC2:					\
	case SPRN_DAC1:					\
	case SPRN_DAC2:					\
	case SPRN_DBCR1:				\
	case SPRN_DBCR2:				\
		value = WRHV_MFSPR_NO_PERMISSION(rn);	\
		break;					\
	case SPRN_TLB1CFG:				\
		/* FIXME: dump this fixed value per datasheet since
		 * currently the hypervisor doesn't emulate this register.
		 */					\
		value = 0x401bc040;			\
		break;					\
	default:					\
		value = WRHV_MFSPR(rn);			\
	}						\
	value;						\
})
#else
extern unsigned int wrhv_mfspr(unsigned int sprn);
#endif
#define mfspr(rn)	wrhv_mfspr(rn)
#endif /* __ASSEMBLY__ */

#endif /* __KERNEL__ */
#endif /* _ASM_POWERPC_REG_WRHV_H */
