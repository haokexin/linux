/*
 * fp_library.c
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C)2024Black Sesame Technologies. All Rights Reserved.
 */
#include <fp_library.h>
#include <asm/io.h>

#ifdef __FP_OS__
#include <linux/module.h>
#include <linux/slab.h>
#endif



/**********************************************************************
 * Function Name  : fp_malloc
 * Description    : allocate memory for requested size in kernel space
 * Inputs
 *   Parameters   : unsigned int
 * Outputs        :
 *   Parameters   : -
 *   Returns      : returns start address of allocated pointer
 * Changes        :
 ********************************************************************/
void *
fp_malloc(unsigned int uiSize)
{
#ifdef __FP_OS__
	void *p;

	p = kmalloc(uiSize, __GFP_DMA|GFP_ATOMIC);
	return p;
#else
	return FP_NULL;
#endif
}

/**********************************************************************
 * Function Name  : fp_free
 * Description    : free the requested memory in kernel space
 * Inputs
 *   Parameters   : void *
 * Outputs        :
 *   Parameters   : -
 *   Returns      : -
 * Changes        :
 ********************************************************************/
void
fp_free(void *free)
{
#ifdef __FP_OS__
	if (free != NULL)
		kfree(free);
#endif
	return;
}

/**********************************************************************
 * Function Name  : fp_free
 * Description    : copy the data byte by byte with given len
 * Inputs
 *   Parameters   : char *, char *, int
 * Outputs        :
 *   Parameters   : char * - copied data into dst pointer
 *   Returns      : -
 * Changes        :
 ********************************************************************/
void
fp_bcopy(register char *src, register char *dst, int len)
{
	if (src == FP_NULL || dst == FP_NULL)
		return;

	if (dst < src) {
		while (len--)
			*dst++ = *src++;
	} else {
		char *lasts = src + (len - 1);
		char *lastd = dst + (len - 1);

		while (len--)
			*(char *)lastd-- = *(char *)lasts--;
	}
}

/**********************************************************************
 * Function Name  : fp_memcpy
 * Description    : helper routine for fp_bcopy
 * Inputs
 *   Parameters   : char *, char *, int
 * Outputs        :
 *   Parameters   : char * - copied data into dst pointer
 *   Returns      : void *
 * Changes        :
 ********************************************************************/
void *
fp_memcpy(void *dst , const void *src, unsigned int len)
{
	fp_bcopy((char *)src, (char *)dst, len);

	return dst;
}


/********************************************************************
 * Function Name  : fp_memcmp
 * Description    : compare given no of bytes in str1 and str2
 * Inputs
 *   Parameters   : void *, void *, int
 * Outputs        :
 *   Parameters   :
 *   Returns      : int
 * Changes        :
 ********************************************************************/
int
fp_memcmp(const void *str1 , const void *str2, unsigned int count)
{
	register const unsigned char *s1 = (const unsigned char *)str1;
	register const unsigned char *s2 = (const unsigned char *)str2;

	while (count-- > 0) {
		if (*s1++ != *s2++)
			return s1[-1] < s2[-1] ? -1 : 1;
	}

	return 0;
}

/**********************************************************************
 * Function Name  : fp_memset
 * Description    : fills the given len of bytes contained by val in dest
 * Inputs
 *   Parameters   : void *, int, int
 * Outputs        :
 *   Parameters   : void *
 *   Returns      : returns filled data pointer
 * Changes        :
 ********************************************************************/
void *
fp_memset(void *dest, int val, unsigned int len)
{
	register unsigned char *ptr = (unsigned char *)dest;

	while (len-- > 0)
		*ptr++ = val;
	return dest;
}

/**********************************************************************
 * Function Name  : fp_printf
 * Description    : print the data in standard output file
 * Inputs
 *   Parameters   : const char *, variable no of arguments
 * Outputs        :
 *   Parameters   :
 *   Returns      : returns no of character printed in output
 * Changes        :
 ********************************************************************/
int
nkprintf(const char *fmt, ...)
{
#ifdef __FP_OS__
	va_list args;
	int r;

	va_start(args, fmt);
	r = vprintk(fmt, args);
	va_end(args);

	return r;
#else
	return 0;
#endif
}

/**********************************************************************
 * Function Name  : fp_SwapBytes
 * Description    : swap the first n no of bytes pointed by pv
 * Inputs
 *   Parameters   : void *, unsigned int
 * Outputs        :
 *   Parameters   : void *
 *   Returns      : -
 * Changes        :
 ********************************************************************/
void fp_SwapBytes(void *pv, unsigned int n)
{
	char *p = pv;
	unsigned int  lo, hi;
	for (lo = 0, hi = n-1; hi > lo; lo++, hi--) {
		char tmp = p[lo];
		p[lo] = p[hi];
		p[hi] = tmp;
	}
}

#define fp_SWAP(x) fp_SwapBytes(&x, sizeof(x));

/**********************************************************************
 * Function Name  : fp_convert_le2be
 * Description    : convert little-endian to big-endian
 * Inputs
 *   Parameters   : char *, unsigned int
 * Outputs        :
 *   Parameters   : char *
 *   Returns      : -
 * Changes        :
 ********************************************************************/
void
fp_convert_le2be(char *data, unsigned int size)
{
	int i, cnt;
	unsigned int *data_ptr = (unsigned int *)data;

	cnt  = size >> 2;
	if (size & 0x3)
		cnt++;
	for (i = 0; i < cnt; i++)
		fp_SwapBytes(&data_ptr[i], 4);
	return;
}

int
fp_exact_cmp(unsigned int pkt, unsigned int reg)
{
	if (pkt == reg)
		return 1;
	return -1;
}

int
fp_mask_cmp(unsigned int pkt, unsigned int reg, unsigned int mask)
{
	if ((pkt | mask) == (reg | mask))
		return 1;
	return -1;
}

int
fp_range_cmp(unsigned int pkt, unsigned int reg, unsigned int range_value)
{
	if ((pkt >= reg) && (pkt < (reg + range_value)))
		return 1;
	return -1;
}

/*Get System time in seconds*/
unsigned long fp_getseconds(void)
{
#ifdef __FP_OS__
	return get_seconds();
#else
	return 0;
#endif
}

/*****************************************************************
 * Function Name  : RegRead()
 * Description    : Hardware register read function return register
 *                  read value for given address.
 * Inputs
 *   Parameters   : UINT
 * Outputs        :
 *   Parameters   : -
 *   Returns      : UINT
 * Changes        :
 ****************************************************************/
UINT32
CSR_REG_READ(UCHAR *baseaddr, UINT32 offset)
{
	volatile u32 read_val;
//	volatile UINT *reg_addr;

//	reg_addr  = (UINT *)(ULONG)(baseaddr + offset);
//	read_val  = *reg_addr;
	read_val = ioread32(baseaddr + offset);
	return read_val;
}
//EXPORT_SYMBOL_GPL(CSR_REG_READ);

/******************************************************************
 * Function Name  : RegWrite()
 * Description    : Hardware register write function Write given value
 *                  in given address.
 * Inputs
 *   Parameters   : UINT, UINT
 * Outputs        :
 *   Parameters   : -
 *   Returns      : -
 * Changes        :
 *****************************************************************/
void
CSR_REG_WRITE(UCHAR *baseaddr, UINT32 offset, UINT32 value)
{

	iowrite32(value, baseaddr + offset);
	return;
#if 0
#ifndef __FP_OS__
	reg_addr  = (UINT *)(ULONG)(baseaddr + offset);
	*reg_addr = (UINT)value;
#else
	if (fp_allow_access_to_csr) {

		if (fp_allow_access_to_hif1_csr) {
			if (offset >= HIF1_START_OFFSET &&
					offset <= HIF1_END_OFFSET) {
				reg_addr  = (UINT *)(ULONG)(baseaddr + offset);
				*reg_addr = (UINT) value;
				return;
			}
		}

#if 0
		if (fp_allow_access_to_hif2_csr) {
			if (offset >= HIF2_START_OFFSET &&
					offset <= HIF2_END_OFFSET) {
				reg_addr  = (UINT *)(ULONG)(baseaddr + offset);
				*reg_addr = (UINT) value;
				return;
			}
		}
#endif

		if (fp_allow_access_to_emac1_csr) {
			if (offset >= EMAC1_START_OFFSET &&
					offset <= EMAC2_END_OFFSET) {
				reg_addr  = (UINT *)(ULONG)(baseaddr + offset);
				*reg_addr = (UINT) value;
				return;
			}
		}

		if (fp_allow_access_to_emac2_csr) {
			if (offset >= EMAC2_START_OFFSET &&
					offset <= EMAC2_END_OFFSET) {
				reg_addr  = (UINT *)(ULONG)(baseaddr + offset);
				*reg_addr = (UINT) value;
				return;
			}
		}

		if (fp_allow_access_to_bmu_csr) {
			if (offset >= BMU1_START_OFFSET &&
					offset <= BMU1_END_OFFSET) {
				reg_addr  = (UINT *)(ULONG)(baseaddr + offset);
				*reg_addr = (UINT) value;
				return;
			}
		}

		if (fp_allow_access_to_tlite_csr) {
			if (offset >= TLITE_START_OFFSET &&
					offset <= TLITE_END_OFFSET) {
				reg_addr  = (UINT *)(ULONG)(baseaddr + offset);
				*reg_addr = (UINT) value;
				return;
			}
		}

		if (fp_allow_access_to_classhw_csr) {
			if (offset >= CLASS_HW_START_OFFSET &&
					offset <= CLASS_HW_END_OFFSET) {
				reg_addr  = (UINT *)(ULONG)(baseaddr + offset);
				*reg_addr = (UINT) value;
				return;
			}
		}
	}
#endif
#endif
	return;
}
//EXPORT_SYMBOL_GPL(CSR_REG_WRITE);

#ifdef __FP_OS__
void
dump_me(volatile unsigned char *src, int len)
{
	int i, k;
	int j;
	volatile unsigned char *ptr;

	j = len % 8;
	k = len / 8;
	for (i = 0; i < k; i++) {
		ptr = src + (i * 8);
		FP_DEBUG(DBGP_FEAT_LIB, FP_LOG_INFO,
			"0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x,"
			"0x%02x, 0x%02x\n", ptr[0], ptr[1], ptr[2], ptr[3],
			ptr[4], ptr[5], ptr[6], ptr[7]);
	}
	ptr = src + (i * 8);

	for (i = 0; i < j; i++)
		FP_DEBUG(DBGP_FEAT_LIB, FP_LOG_INFO, "0x%02x ", ptr[i]);

	FP_DEBUG(DBGP_FEAT_LIB, FP_LOG_INFO, "\n");
}

/**********************************************************************
 * Function Name  : SwapBytes
 * Description    :
 * Inputs
 *   Parameters   : void *, size in bytes
 * Outputs        :
 *   Parameters   : void * - swaped bytes
 *   Returns      : -
 * Changes        :
 *********************************************************************/
void SwapBytes(void *pv, size_t n)
{
	char *p = pv;
	size_t lo, hi;
	for (lo = 0, hi = n - 1; hi > lo; lo++, hi--) {
		char tmp = p[lo];
		p[lo] = p[hi];
		p[hi] = tmp;
	}
}

#define SWAP(x) SwapBytes(&x, sizeof(x));
/**********************************************************************
 * Function Name  : convert_le2be_64
 * Description    : convert given data from little-endian to big-endian
 * Inputs           with 32b bit word swap
 *   Parameters   : char * , int
 * Outputs        :
 *   Parameters   : char *
 *   Returns      :
 * Changes        :
 *********************************************************************/
void
convert_le2be_64(char *data, unsigned int size)
{
	int i, cnt;
	unsigned long long  *data_ptr = (unsigned long long *)data;

	cnt  = size >> 3;
	if (size & 0x7)
		cnt++;
	/* swap the lsb bytes as msb vice-versa */
	for (i = 0; i < cnt; i++)
		SwapBytes(&data_ptr[i], 8);
	return;
}

/**********************************************************************
 * Function Name  : convert_le2be
 * Description    : convert given data from little-endian to big-endian
 * Inputs
 *   Parameters   : char * , int
 * Outputs        :
 *   Parameters   : char *
 *   Returns      :
 * Changes        :
 *********************************************************************/
void
convert_le2be(char *data, unsigned int size)
{
	int i, cnt;
	unsigned int *data_ptr = (unsigned int *)data;

	cnt  = size >> 2;
	if (size & 0x3)
		cnt++;
	/* swap the lsb bytes as msb vice-versa */
	for (i = 0; i < cnt; i++)
		SwapBytes(&data_ptr[i], 4);
	return;
}

/*****************************************************************
 * Function Name  : RegRead()
 * Description    : Hardware register read function return register
 *                  read value for given address.
 * Inputs
 *   Parameters   : UINT
 * Outputs        :
 *   Parameters   : -
 *   Returns      : UINT
 * Changes        :
 ****************************************************************/
UINT
RegRead(UINT RegAddr)
{
	volatile UINT read_val;
	volatile UINT *reg_addr;

	reg_addr  = (UINT *)(ULONG) RegAddr;
	read_val  = *reg_addr;
	return read_val;
}

/******************************************************************
 * Function Name  : RegWrite()
 * Description    : Hardware register write function Write given value
 *                  in given address.
 * Inputs
 *   Parameters   : UINT, UINT
 * Outputs        :
 *   Parameters   : -
 *   Returns      : -
 * Changes        :
 *****************************************************************/
void
RegWrite(UINT RegAddr, UINT RegData)
{
	volatile UINT *reg_addr;
	reg_addr  = (UINT *)(ULONG) RegAddr;
	*reg_addr = (UINT) RegData;
}


#endif
