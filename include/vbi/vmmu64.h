/*
 * vmmu64.h - hypervisor 64 bit virtual MMU structure definitions
 *
 * Copyright 2010 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 */
#ifndef __VBI_VMMU64_H
#define __VBI_VMMU64_H

/*
 *
 * VMMU64 format - 64-bit Virtual MMU format for Guest OS
 * 
 * 64-bit Virtual MMU format for Guest OS 
 * 
 * For a 64-bit virtual address, there would need to be a 6 level page table,
 * with levels labelled as:
 * 
 * LX 	level x 	12 bits
 * LY 	level y 	12 bits
 * L1 	level 1 	10 bits
 * L2 	level 2 	 9 bits
 * L3 	level 3 	 9 bits
 * PO 	pageoffset 	12 bits
 *                        --------
 *                         64 bits
 *                        --------
 * 
 * In typical usage scenario 40-bits is sufficient (Linux for example). So
 * we leave it as a configurable parameter:
 * 
 * 	WRHV_INCLUDE_VMMU64_USE_40BITS
 * 
 * If the above feature is enabled in archConfig.h, then it'll only use 40-bits
 * as described below:
 * 
 * The vmmu64 virtual address space is restricted to 40 bits and is decoded using 
 * a level-1/level-2/level-3 page table.  The virtual address is decoded as follows:
 * 
 * 
 *                                           40-bit Virtual Address
 *      +--------------------------------------------------------------------+
 *      |      L1 offset       | L2 offset |  L3 offset |  Page offset       |
 *      +--------------------------------------------------------------------+
 * 		10 bits       9 bits        9 bits        12 bits
 *                |               |              |        
 *                |               |              |
 *  +-------------+               |              +---------+
 *  |                             |                        |
 *  |                             |                        |
 *  |           L1 Table          |            L2 Table    |      L3 Table
 *  |    1023 +----------+        |     511 +----------+   |  511 +----------+
 *  |         |          |        |         |          |   |      |          |
 *  |         |          |        |         |          |   |      |          |
 *  |         |          |        |         |----------|   |      |----------|
 *  |         |          |        |   +---->|   L3 ptr |---+----->|   PTE    | 8 
 *  |         |          |        |   |     |----------| 28       |----------|byte
 *  |         |          | 19     |   |     |          | bits     |          |PTE
 *  |         |----------| bits   |   |     |          |          |          |
 *  +-------->|  L2 ptr  |--------+---+     |          |          |          |
 *            |----------|                  |          |          |          |
 *            |          |                  |          |          |          |
 *            |          |                  |          |          |          |
 *          0 +----------+                0 +----------+        0 +----------+
 *            2 page (8KB)                   1 page (4KB)          1 page (4KB) 
 *           1024 L2 pointers               512 L3 pointers        512 PTE entries
 * 
 * 
 * 
 * (VMMU64_PTE is arch specific, following describes the VMMU64_PTE for mips64)  
 * 
 * Each page table entry is 8 bytes and uses the following format:
 * 
 * 
 * word 0 (64-bits):
 * 
 *   6     6                   5                   4               3
 *   3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *  |Fill                                   |PFN                    |
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * 
 * 
 *     3                   2                   1
 *   1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *  |PFN                                    |C C C D V G| user-bits |
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * 
 * 
 * 		PFN        - page frame number
 * 		C          - Cache Coherency bit
 * 		D	   - Dirty bit
 * 		V	   - Valid bit
 * 		G	   - Global bit
 * 		u	   - user-bits
 * 
 * NOTE:
 * 
 * Hypervisor would not know the guest is using 64-bit VMMU unless
 * the configuration file, specifically mentions the GuestOS type 
 * of "Application Unprivileged 64" or "Application Privileged 64" 
 * or any of the WRLinux versions.
 *
 * Unless the Virtual Board configuration file indicates the Guest OS
 * type, hypervisor would not know that the guest is using 64-bit
 * vmmu, in which case, it would default to using 32-bit VMMU and
 * would cause unpredictable behaviour (most likely crash).
 *
 */
 
#ifndef	_ASMLANGUAGE

#if (CPU == MIPSI64R2) 
typedef union vmmu64_pte		/* vmmu64 pte format */
{
	struct {
		uint64_t fill:20;		
		uint64_t pfn:32;
		uint64_t coherent:3;
		uint64_t dirty:1;
		uint64_t valid:1;
		uint64_t global:1;
		uint64_t usr5:1;
		uint64_t usr4:1;
		uint64_t usr3:1;
		uint64_t usr2:1;
		uint64_t usr1:1;
		uint64_t usr0:1;
	} field;
	uint64_t data64;
} VMMU64_PTE;
#else
/* for all other arches, for now placeholder */
typedef uint64_t VMMU64_PTE;
#endif

/* Effective Address Definition */

typedef union vmmu64EffectiveAddr 	/* effective Address structure */
{
	struct {
		uint64_t fill:24;			/* reserved */
		uint64_t l1index:10;		/* Level 1 Index (1K) */
		uint64_t l2index:9;		/* Level 2 Index (512) */
		uint64_t l3index:9;		/* Level 3 Index (512) */
		uint64_t po:12;			/* Page Offset (4K) */
	} field;
	uint64_t addr;
} VMMU64_EFFECTIVE_ADDR;

typedef union vmmu64RealAddress		/* Real Address Structure */
{
	struct {                      	/* Bit field description */
		uint64_t rpn:52;           	/* Real Page Number */
		uint64_t po:12;            	/* Page Offset */
	}field;
	uint64_t realAddr;            	/* Real Address */
} VMMU64_REAL_ADDRESS;


#ifndef WRHV_INCLUDE_VMMU64_USE_40BITS
/* Level-0 descriptor definition */

typedef union vmmu64_level_0_desc	/* Level 0 descriptor format */
{
	struct {                      	/* Bit field desciption */
		uint64_t l1ba:52;          	/* Level 1 table Base Address */
		uint64_t reserved:12;      	/* Reserved */
	} field;
	uint64_t l0desc;               	/* Level 0 descriptor */
} VMMU64_LEVEL_0_DESC;
#endif

/* Level-1 descriptor definition */

typedef union vmmu64_level_1_desc	/* Level 1 descriptor format */
{
	struct {                      	/* Bit field desciption */
		uint64_t l2ba:52;          	/* Level 2 table Base Address */
		uint64_t reserved:12;      	/* Reserved */
	} field;
	uint64_t l1desc;               	/* Level 1 descriptor */
} VMMU64_LEVEL_1_DESC;


/* Level-2 descriptor definition */

typedef union vmmu64_level_2_desc	/* Level 2 descriptor format */
{
	struct {                      	/* Bit field desciption */
		uint64_t l3ba:52;          	/* Level 3 table Base Address */
		uint64_t reserved:12;      	/* Reserved */
	} field;
	uint64_t l2desc;               	/* Level 3 descriptor */
} VMMU64_LEVEL_2_DESC;


/* Level-3 descriptor definition */

typedef union vmmu64_level_3_desc	/* Level 2 descriptor format */
{
	VMMU64_PTE pte;			/* a full PTE entry */
} VMMU64_LEVEL_3_DESC;

/* VMMU configuration system call paramter */

typedef struct vmmu64_config
{
	uint64_t	addr; 
	uint32_t	flush_type;
	uint32_t	asid;
	uint32_t	vmmu_handle; 
} VMMU64_CONFIG;

#endif /* _ASMLANGUAGE */


/* VMMU page table structure */

#ifdef WRHV_INCLUDE_VMMU64_4LEVEL_PAGE_TABLE
#define	VMMU64_LX_ENTRIES	4096 	/* top 12 bits of address	*/
#define	VMMU64_LX_SIZE		8	/* table size in pages (32KB)	*/
#define	VMMU64_LY_ENTRIES	4096 	/* next 12 bits of address	*/
#define	VMMU64_LY_SIZE		8	/* table size in pages (32KB)	*/
#endif
#define	VMMU64_L1_ENTRIES	1024	/* next 10 bits of address	*/
#define	VMMU64_L1_SIZE		2	/* table size in pages (8KB)	*/
#define	VMMU64_L2_ENTRIES	512	/* next 9 bits of address	*/
#define	VMMU64_L2_SIZE		1	/* table size in pages (4KB)	*/
#define	VMMU64_L3_ENTRIES	512	/* next 9 bits of address	*/
#define	VMMU64_L3_SIZE		1	/* table size in pages (4KB)	*/

#define VMMU64_TLB_FLUSH_ALL    0
#define VMMU64_TLB_FLUSH_NONE   1
#define VMMU64_TLB_FLUSH_ASID   2
#define VMMU64_TLB_FLUSH_ADDR   3

#ifndef	VMMU64_PAGE_SIZE
#define	VMMU64_PAGE_SIZE	4096	/* always use a 4KB page size */
#define	VMMU64_RPN_SHIFT	12
#endif
#define	NV64PAGES(x)		((x)/VMMU64_PAGE_SIZE)

/* address to level-1 table offset */

#define	VMMU64_L1_INDEX(v)	((((uint64_t)(v)) >> 30) & 0x3ff)

/* address to level-2 table offset */

#define	VMMU64_L2_INDEX(v)	((((uint64_t)(v)) >> 21) & 0x1ff)

/* address to level-3 table offset */

#define	VMMU64_L3_INDEX(v)	((((uint64_t)(v)) >> 12) & 0x1ff)

/* offset within page */

#define	VMMU64_PAGE_OFFSET(v)	(((uint64_t)(v)) & 0xfff)

/* address to logical block number */

#define	VMMU64_ADDR_TO_LBA(v)	(((uint64_t)(v)) >> VMMU64_RPN_SHIFT)
#define	VMMU64_LBA_TO_ADDR(v)	(((uint64_t)(v)) << VMMU64_RPN_SHIFT)

/* arch PTE commands */
#define VMMU64_PTE_INVALID	0
#define VMMU64_PTE_VALID	1
#define VMMU64_PTE_ATTR_SET	2

#endif  /* __VBI_VMMU64_H */
