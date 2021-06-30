/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IMG PVDEC pixel Registers
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 */

#ifndef _MEM_IO_H
#define _MEM_IO_H

#include <linux/types.h>

#include "reg_io2.h"

#define RND_TO_WORDS(size) ((((size) + 3) / 4) * 4)

#define MEMIO_CHECK_ALIGNMENT(vpmem)        \
	IMG_ASSERT((vpmem))

#define MEMIO_READ_FIELD(vpmem, field) \
	((((*((field ## _TYPE *)(((unsigned long)(vpmem)) + field ## _OFFSET))) & \
	   field ## _MASK) >> field ## _SHIFT))

#define MEMIO_READ_TABLE_FIELD(vpmem, field, tabidx) \
	((((*((field ## _TYPE *)(((unsigned long)(vpmem)) + field ## _OFFSET + \
				 (field ## _STRIDE * (tabidx))))) & field ## _MASK) >> \
	  field ## _SHIFT)) \

#define MEMIO_READ_REPEATED_FIELD(vpmem, field, repidx, type) \
	({ \
		type __repidx = repidx; \
		((((*((field ## _TYPE *)(((unsigned long)(vpmem)) + field ## _OFFSET))) & \
		   (field ## _MASK >> ((__repidx) * field ## _SIZE))) >> \
		  (field ## _SHIFT - ((__repidx) * field ## _SIZE)))); }) \

#define MEMIO_READ_TABLE_REPEATED_FIELD(vpmem, field, tabidx, repidx, type) \
	({ \
		type __repidx = repidx; \
		((((*((field ## _TYPE *)(((unsigned long)(vpmem)) + field ## _OFFSET + \
					 (field ## _STRIDE * (tabidx))))) & (field ## _MASK >> \
									     ((__repidx) * \
									      field ## _SIZE))) >> \
		  (field ## _SHIFT - \
		   (( \
			    __repidx) \
		    * field ## _SIZE)))); }) \

#define MEMIO_WRITE_FIELD(vpmem, field, value, type) \
	do { \
		type __vpmem = vpmem; \
		MEMIO_CHECK_ALIGNMENT(__vpmem); \
		(*((field ## _TYPE *)(((unsigned long)(__vpmem)) + \
				      field ## _OFFSET))) = \
			(field ## _TYPE)(((*((field ## _TYPE *)(((unsigned long)(__vpmem)) + \
								field ## _OFFSET))) & \
					  ~(field ## _TYPE)field ## _MASK) | \
					 (field ## _TYPE)(((value) << field ## _SHIFT) & \
							  field ## _MASK)); \
	} while (0) \

#define MEMIO_WRITE_FIELD_LITE(vpmem, field, value, type) \
	do { \
		type __vpmem = vpmem; \
		MEMIO_CHECK_ALIGNMENT(__vpmem); \
		(*((field ## _TYPE *)(((unsigned long)(__vpmem)) + \
				      field ## _OFFSET))) = \
			((*((field ## _TYPE *)(((unsigned long)(__vpmem)) + \
					       field ## _OFFSET))) | \
			 (field ## _TYPE)(((value) << field ## _SHIFT))); \
	} while (0) \

#define MEMIO_WRITE_TABLE_FIELD(vpmem, field, tabidx, value, vp_type, ta_type) \
	do { \
		vp_type __vpmem = vpmem; \
		ta_type __tabidx = tabidx; \
		MEMIO_CHECK_ALIGNMENT(__vpmem); \
		IMG_ASSERT(((__tabidx) < field ## _NO_ENTRIES) || \
			   (field ## _NO_ENTRIES == 0)); \
		(*((field ## _TYPE *)(((unsigned long)(__vpmem)) + field ## _OFFSET + \
				      (field ## _STRIDE * (__tabidx))))) =  \
			((*((field ## _TYPE *)(((unsigned long)(__vpmem)) + field ## _OFFSET + \
					       (field ## _STRIDE * (__tabidx))))) & \
			 (field ## _TYPE) ~field ## _MASK) | \
			(field ## _TYPE)(((value) << field ## _SHIFT) & field ## _MASK); \
	} while (0) \

#define MEMIO_WRITE_REPEATED_FIELD(vpmem, field, repidx, value, vp_type, re_type) \
	do { \
		vp_type __vpmem = vpmem; \
		re_type __repidx = repidx; \
		MEMIO_CHECK_ALIGNMENT(__vpmem); \
		IMG_ASSERT((__repidx) < field ## _NO_REPS); \
		(*((field ## _TYPE *)(((unsigned long)(__vpmem)) + \
				      field ## _OFFSET))) = \
			((*((field ## _TYPE *)(((unsigned long)(__vpmem)) + \
					       field ## _OFFSET))) & \
			 (field ## _TYPE) ~(field ## _MASK >> ((__repidx) * field ## _SIZE)) | \
			 (field ## _TYPE)(((value) << (field ## _SHIFT - \
						       ((__repidx) * field ## _SIZE))) & \
					  (field ## _MASK >> \
					   ((__repidx) \
					    * field ## _SIZE)))); \
	} while (0) \

#define MEMIO_WRITE_TABLE_REPEATED_FIELD(vpmem, field, tabidx, repidx, value, vp_type, ta_type, \
					 re_type) \
	do { \
		vp_type __vpmem = vpmem; \
		ta_type __tabidx = tabidx; \
		re_type __repidx = repidx; \
		MEMIO_CHECK_ALIGNMENT(__vpmem); \
		IMG_ASSERT(((__tabidx) < field ## _NO_ENTRIES) || \
			   (field ## _NO_ENTRIES == 0)); \
		IMG_ASSERT((__repidx) < field ## _NO_REPS); \
		(*((field ## _TYPE *)(((unsigned long)(__vpmem)) + field ## _OFFSET + \
				      (field ## _STRIDE * (__tabidx))))) = \
			((*((field ## _TYPE *)(((unsigned long)(__vpmem)) + field ## _OFFSET + \
					       (field ## _STRIDE * (__tabidx))))) & \
			 (field ## _TYPE) ~(field ## _MASK >> \
					    ((__repidx) * field ## _SIZE))) | \
			(field ## _TYPE)(((value) << \
					  ( \
						  field ## _SHIFT - \
						  ((__repidx) * field ## _SIZE))) & \
					 ( \
						 field ## _MASK >> ((__repidx) * field ## _SIZE)));\
	} while (0) \

#endif /* _MEM_IO_H */
