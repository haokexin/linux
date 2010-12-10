/*
 * vmmu.h - hypervisor virtual MMU structure definitions
 *
 * Copyright 2007 Wind River Systems, Inc.
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

#ifndef _VBI_VMMU_H
#define _VBI_VMMU_H

/*
 *
 * VMMU format - 32-bit Virtual MMU format for Guest OS
 * 
 * 32-bit Virtual MMU format for Guest OS 
 * 
 * SYNOPSIS
 *
 * 
 * 
 * The vmmu virtual address space is restricted to 32 bits and is decoded using
 * a level-1/level-2 page table.  The virtual address is decoded as follows:
 * 
 * 
 *                           32-bit Virtual Address
 *         +---------------------------------------------------------+
 *         |      L1 offset       | L2 offset |    Page offset       |
 *         +---------------------------------------------------------+
 * 		11 bits           9 bits           12 bits
 *                   |                 |
 *                   |                 |
 *     +-------------+                 |
 *     |                               |
 *     |                               |
 *     |           L1 Table            |            L2 Table
 *     |    2047 +----------+          |      511 +----------+
 *     |         |          |          |          |          |
 *     |         |          |          |          |          |
 *     |         |          |          |          |----------|
 *     |         |          |          |   +----->|    PTE   | 8 byte PTE
 *     |         |          |          |   |      |----------|
 *     |         |          |          |   |      |          |
 *     |         |----------| 20 bits  |   |      |          |
 *     +-------->|  L2 ptr  |----------+---+      |          |
 *               |----------|                     |          |
 *               |          |                     |          |
 *               |          |                     |          |
 *             0 +----------+                   0 +----------+
 *                2 page (8KB)                    1 page (4KB)
 *              2048 L2 pointers                 512 PTE entries
 * 
 * 
 * 
 * Each page table entry is 8 bytes (2 words) and uses the following format:
 * 
 * 
 * word 0 (32-bits):
 * 	
 * 	  0 1            7 8           15 1 1 1 1 2 2 2 2 24   26 27  31
 * 	                                  6 7 8 9 0 1 2 3               
 * 	 +-+--------------+--------------+-+-+-+-+-+-+-+-+-------+------+
 * 	 |V|  Hypervisor  |   Reserved   |U|U|U|U|U|U|U|U| ERPN  | ATTR |
 * 	 | |   Reserved   |              |0|1|2|3|4|5|6|7|       |      |
 * 	 +-+--------------+--------------+-+-+-+-+-+-+-+-+-------+------+
 * 
 * 		V          - valid bit
 * 		Hypervisor - reserved for use by hypervisor
 * 		U0-U7      - user defined attributes
 * 		ERPN       - extended real page number bits
 * 		ATTR       - page attributes
 * 
 * 
 * word 1 (32-bits):
 * 
 * 	  0                                19 20      23 2 2 2 2 2 2 3 3
 * 	                                                 4 5 6 7 8 9 0 1
 * 	 +-----------------------------------+----------+-+-+-+-+-+-+-+-+
 * 	 |                RPN                | Reserved |R|C|U|S|U|S|U|S|
 * 	 |                                   |          | | |X|X|W|W|R|R|
 * 	 +-----------------------------------+----------+-+-+-+-+-+-+-+-+
 * 
 * 		RPN        - real page number
 * 		R          - page referenced bit
 * 		C          - page changed bit
 * 		SX,SW,SR   - supervisor mode protection bits
 * 		UX,UW,UR   - user mode protection bits
 * 
 * 
 * Above page table bits are numbered as per PPC(IBM) format MSB bit as 0, 
 * The same entry can be represented using LSB bit as 0, as in mips64 arch:
 * 
 * word 0 (32-bits):
 * 	
 *           3 30          24 23          16 1 1 1 1 1 1 9 8 7     5 4    0
 *           1                               5 4 3 2 1 0
 * 	 +-+--------------+--------------+-+-+-+-+-+-+-+-+-------+------+
 * 	 |V|  Hypervisor  |   Reserved   |U|U|U|U|U|U|U|U| ERPN  | ATTR |
 * 	 | |   Reserved   |              |0|1|2|3|4|5|6|7|       |      |
 * 	 +-+--------------+--------------+-+-+-+-+-+-+-+-+-------+------+
 * 
 * 		V          - valid bit
 * 		Hypervisor - reserved for use by hypervisor
 * 		U0-U7      - user defined attributes
 * 		ERPN       - extended real page number bits
 * 		ATTR       - page attributes
 * 
 * 
 * word 1 (32-bits):
 * 
 *           31                               12 11       8 7 6 5 4 3 2 1 0
 * 	 +-----------------------------------+----------+-+-+-+-+-+-+-+-+
 * 	 |                RPN                | Reserved |R|C|U|S|U|S|U|S|
 * 	 |                                   |          | | |X|X|W|W|R|R|
 * 	 +-----------------------------------+----------+-+-+-+-+-+-+-+-+
 * 
 * 		RPN        - real page number
 * 		R          - page referenced bit
 * 		C          - page changed bit
 * 		SX,SW,SR   - supervisor mode protection bits
 * 		UX,UW,UR   - user mode protection bits
 * 
 */

#ifndef	_ASMLANGUAGE

/* Page Table Entry Definition */

typedef union vmmu_pte		/* vmmu pte format */
{
	struct			/* Bit field description */
	{
		/* word 0 */
		u_int v:1;		/* valid bit */
		u_int hy:7;		/* reserved for use by hypervisor */
		u_int rsvd1:8;		/* reserved */
		u_int u0:1;		/* user attribute 0 */
		u_int u1:1;		/* user attribute 1 */
		u_int u2:1;		/* user attribute 2 */
		u_int u3:1;		/* user attribute 3 */
		u_int u4:1;		/* user attribute 4 */
		u_int u5:1;		/* user attribute 5 */
		u_int u6:1;		/* user attribute 6 */
		u_int u7:1;		/* user attribute 7 */
		u_int erpn:3;		/* extended real page number bits */
		u_int w:1;		/* write thru/back */
		u_int i:1;		/* cache inhibited */
		u_int m:1;		/* memory coherent */
		u_int g:1;		/* memory guarded  */
		u_int e:1;		/* little endian bit */

		/* word 1 */
		u_int rpn:20;		/* real page number */
		u_int rsvd2:4;		/* reserved */
		u_int r:1;		/* page referenced bit */
		u_int c:1;		/* page changed bit */
		u_int ux:1;		/* user execute protection */
		u_int sx:1;		/* supervisor execute protection */
		u_int uw:1;		/* user write protection */
		u_int sw:1;		/* supervisor write protection */
		u_int ur:1;		/* user read protection */
		u_int sr:1;		/* supervisor read protection */
	} field;

	struct
	{
		u_int word0;		/* word 0 */
		u_int word1;		/* word 1 */
	} words;
} VMMU_PTE;

/* Effective Address Definition */

typedef union vmmuEffectiveAddr /* effective Address structure */
{
	struct
        {
		u_int l1index:11;	/* Level 1 Index (2K) */
		u_int l2index:9;	/* Level 2 Index (512) */
		u_int po:12;		/* Page Offset (4K) */
        } field;
	uint32_t addr;
} VMMU_EFFECTIVE_ADDR;

/* Real Address Definition */

typedef union vmmuRealAddress	/* Real Address Structure */
{
	struct			/* Bit field description */
	{
		u_int rpn:20;	/* Real Page Number */
		u_int po:12;	/* Page Offset */
	}field;
	uint32_t realAddr;	/* Real Address */
} VMMU_REAL_ADDRESS;

/* Level-1 descriptor definition */

typedef union vmmu_level_1_desc	/* Level 1 descriptor format */
{
	struct			/* Bit field desciption */
	{
		u_int l2ba:20;		/* Level 2 table Base Address */
		u_int reserved:10;	/* Reserved */
		u_int b:1;		/* Block translation */
		u_int v:1;		/* Segment Valid bit */
        } field;
	u_int l1desc;		/* Level 1 descriptor */
} VMMU_LEVEL_1_DESC;

/* Level-2 descriptor definition */

typedef union vmmu_level_2_desc	/* Level 2 descriptor format */
{
	VMMU_PTE pte;		/* a full PTE entry */
} VMMU_LEVEL_2_DESC;

/* Level-2 table pointer definition */

typedef union vmmu_level_2_tbl_ptr /* Level 2 Table pointer structure */
{
	struct		/* Bit field description */
	{
		u_int l2tb:20;		/* Level 2 Table Base */
		u_int l2index:9;	/* Level 2 table Index */
		u_int reserved:3;	/* Reserved */
        } field;
	VMMU_LEVEL_2_DESC *pL2Desc;	/* Level 2 descriptor table pointer */
} VMMU_LEVEL_2_TBL_PTR;


/* VMMU configuration system call paramter */

typedef struct vmmuConfig
{
	/*
	 * typecast addr to (VMMU_LEVEL_1_DESC *)
	 * (void *) in 64-bit hypervisor becomes a 64-bit
	 * value causing incorrect values set for addr
	 * defining it as uint32_t would provide
	 * backward compatibility to existing 32-bit Guest OS
	 * and still support new 64-bit hypervisors
	 * with 32-bit Guests
	 */
	uint32_t	addr;
	uint32_t	flush_type;
	uint32_t	asid;
	uint32_t	vmmu_handle;
} VMMU_CONFIG;

#endif /* _ASMLANGUAGE */

#define VMMU_TLB_FLUSH_ALL	0
#define VMMU_TLB_FLUSH_NONE	1
#define VMMU_TLB_FLUSH_ASID	2
#define VMMU_TLB_FLUSH_ADDR	3

#ifndef	VMMU_PAGE_SIZE
#define	VMMU_PAGE_SIZE	4096	/* always use a 4KB page size */
#define	VMMU_RPN_SHIFT	12
#endif
#define	NVPAGES(x)		((x)/VMMU_PAGE_SIZE)

/* VMMU protection attributes */

#define	VMMU_PROT_SUPV_READ	0x00000001	/* supervisor read allowed    */
#define	VMMU_PROT_USER_READ	0x00000002	/* user read allowed	      */
#define	VMMU_PROT_SUPV_WRITE	0x00000004	/* supervisor write allowed   */
#define	VMMU_PROT_USER_WRITE	0x00000008	/* user write allowed	      */
#define	VMMU_PROT_SUPV_EXECUTE	0x00000010	/* supervisor execute allowed */
#define	VMMU_PROT_USER_EXECUTE	0x00000020	/* user execute allowed	      */
#define	VMMU_PROT_COW_INVALID	0x00000040	/* copy-on-write invalid      */
#define	VMMU_PROT_COW_VALID	0x00000080	/* copy-on-write valid	      */



#define	VMMU_PROT_USER_RWX	VMMU_PROT_USER_READ | \
				VMMU_PROT_USER_WRITE | \
				VMMU_PROT_USER_EXECUTE \

#define	VMMU_PROT_USER_RW	VMMU_PROT_USER_READ | \
				VMMU_PROT_USER_WRITE

#define	VMMU_PROT_USER_RX	VMMU_PROT_USER_READ | \
				VMMU_PROT_USER_EXECUTE

#define	VMMU_PROT_SUPV_RWX	VMMU_PROT_SUPV_READ | \
				VMMU_PROT_SUPV_WRITE | \
				VMMU_PROT_SUPV_EXECUTE \

#define	VMMU_PROT_SUPV_RW	VMMU_PROT_SUPV_READ | \
				VMMU_PROT_SUPV_WRITE

#define	VMMU_PROT_SUPV_RX	VMMU_PROT_SUPV_READ | \
				VMMU_PROT_SUPV_EXECUTE

/* VMMU cache attributes */
#define	VMMU_CACHE_LE		0x00000001	/* cache little endian	*/
#define	VMMU_CACHE_GUARDED	0x00000002	/* cache guarded	*/
#define	VMMU_CACHE_COHERENT	0x00000004	/* cache coherency	*/
#define	VMMU_CACHE_INHIBIT	0x00000008	/* cache inhibit	*/
#define	VMMU_CACHE_WRITETHROUGH	0x00000010	/* cache write through	*/
#define	VMMU_CACHE_COPYBACK	0x00000000	/* cache copy back	*/

/* VMMU page table structure */
#define	VMMU_L1_ENTRIES	2048	/* top 11 bits of address	*/
#define	VMMU_L1_SIZE	2	/* table size in pages (8KB)	*/
#define	VMMU_L2_ENTRIES	512	/* middle 9 bits of address	*/
#define	VMMU_L2_SIZE	1	/* table size in pages (4KB)	*/

/* address to level-1 table offset */
#define	VMMU_L1_INDEX(v)	(((u_int)(v)) >> 21)

/* address to level-2 table offset */
#define	VMMU_L2_INDEX(v)	((((u_int)(v)) >> 12) & 0x1ff)

/* offset within page */
#define	VMMU_PAGE_OFFSET(v)	(((u_int)(v)) & 0xfff)

/* address to logical block number */
#define	VMMU_ADDR_TO_LBA(v)	(((u_int)(v)) >> VMMU_RPN_SHIFT)
#define	VMMU_LBA_TO_ADDR(v)	(((u_int)(v)) << VMMU_RPN_SHIFT)

/* bit masks for PTE fields */
/* word 0 */
#define	VMMU_PTE_ATTR_MASK	0x0000001f	/* page attributes */
#define	VMMU_PTE_ERPN_MASK	0x000000e0	/* extended real page number */
#define	VMMU_PTE_VALID_MASK	0x80000000	/* pte valid */
#define	VMMU_PTE_SUPER_MASK	0x00008000	/* U0, bit 16 */

/* word 1 */
#define	VMMU_PTE_PERM_MASK	0x0000003f	/* page permissions */
#define	VMMU_PTE_CHG_MASK	0x00000040	/* page changed bit */
#define	VMMU_PTE_REF_MASK	0x00000080	/* page referenced bit */
#define	VMMU_PTE_RPN_MASK	0xfffff000	/* real page number */

#endif  /* _VBI_VMMU_H */
