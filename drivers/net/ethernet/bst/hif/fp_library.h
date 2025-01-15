/*
 * fp_library.h
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C)2024Black Sesame Technologies. All Rights Reserved.
 */

#ifndef _FP_LIBRARY_H_
#define _FP_LIBRARY_H_
#include <linux/kernel.h>
//#include <stddef.h>
#include "fp_types.h"
//#include "fp_global_addr_map.h"
//#include "fp_macros.h"
//#include "fp_pktbuff.h"
//#include "fp_ether.h"
#ifdef __FP_OS__
#include <stddef.h>
#endif
//#include "class_hw_table_csr.h"
#define FP_NULL (void *)0

#define FP_TRUE    0
#define FP_FALSE  -1

extern void fp_macLe2BeCopy(char *src, char *dst);
extern char fp_macLe2BeCmp(char *src, char *dst);

void *fp_malloc(UINT uiSize);
void  fp_free(void *Free);
void *fp_memcpy(void *Dest , const void *Src, UINT uiSize);
int   fp_memcmp(const void *Dest , const void *Src, UINT uiSize);
void *fp_memset(void *s, int c, UINT uiSize);
UCHAR *fp_ether_ntoa(UCHAR *SourceMac);
void fp_convert_le2be(char *,unsigned int);

void
dump_me(volatile unsigned char *src, int len);

/* HW register read function */
extern UINT32 CSR_REG_READ(UCHAR *baseaddr, UINT32 offset);

/* HW register write function */
extern void CSR_REG_WRITE(UCHAR *baseaddr, UINT32 offset, UINT32 value);

/* fpath trace levels */
#define FP_LOG_EMERG       0       /* system is unusable */
#define FP_LOG_ALERT       1       /* action must be taken immediately */
#define FP_LOG_CRIT        2       /* critical conditions */
#define FP_LOG_ERR         3       /* error conditions */
#define FP_LOG_WARNING     4       /* warning conditions */
#define FP_LOG_NOTICE      5       /* normal but significant condition */
#define FP_LOG_INFO        6       /* informational */
#define FP_LOG_DEBUG       7       /* debug-level messages */

/* fpath trace types */
#define FP_TRACE_INIT                0x1
#define FP_TRACE_CLEANUP             0x2
#define FP_TRACE_L2_PARSING          0x4
#define FP_TRACE_L3_PARSING          0x8
#define FP_TRACE_L4_PARSING          0x10
#define FP_TRACE_ROUTING             0x20
#define FP_TRACE_BRIDGING            0x40
#define FP_TRACE_NF_HOOKS            0x80
#define FP_TRACE_CONFIG              0x100
#define FP_TRACE_LINUX               0x200
#define FP_TRACE_OTHER               0x400
#define FP_TRACE_ALL                 0xFFFFFFFF
/* fpath trace macros and data structures */
typedef struct fpath_trace_s {

    UINT     trace_flags;
    UINT     trace_level;

} fpath_trace_t;

extern fpath_trace_t fpath_trace;

#define FPATH_TRACE(type, trace_level, fmt_str)

extern UCHAR *fp_baseAddr;
#ifdef __FP_OS__

#define FP_MAX_DBGP 32
#define FP_MAX_DBGP_LEN 32
extern UCHAR fp_dbgp_arr[FP_MAX_DBGP][FP_MAX_DBGP_LEN];
extern UINT fp_dbgp_flags;
extern UINT fp_trace_level;

extern USHORT fp_allow_access_to_csr;
extern USHORT fp_allow_access_to_hif1_csr;
extern USHORT fp_allow_access_to_hif2_csr;
extern USHORT fp_allow_access_to_emac1_csr;
extern USHORT fp_allow_access_to_emac2_csr;
extern USHORT fp_allow_access_to_bmu_csr;
extern USHORT fp_allow_access_to_tlite_csr;
extern USHORT fp_allow_access_to_classhw_csr;
extern USHORT fp_allow_access_to_ltc_csr;

enum fp_dbgp_features {
        DBGP_FEAT_HIF     = 0x1,
        DBGP_FEAT_BRIDGE  = 0x2,
        DBGP_FEAT_ROUTE   = 0x4,
        DBGP_FEAT_FWD_PE  = 0x8,
        DBGP_FEAT_TMU     = 0x10,
        DBGP_FEAT_CLI     = 0x20,
        DBGP_FEAT_L2CLI   = 0x40,
        DBGP_FEAT_L3CLI   = 0x80,
        DBGP_FEAT_L2API   = 0x100,
        DBGP_FEAT_L3API   = 0x200,
        DBGP_FEAT_IOCTL   = 0x400,
        DBGP_FEAT_LIB     = 0x800,
        DBGP_FEAT_ERR     = 0x1000,

        /*
         * This is an execution time feature, so max number of allowed
         * groups are 0-31. You may add your features before this comment.
         */
        DBGP_FEAT_MAX = 0x80000000
};

#define FP_DEBUG(feat, level, msg, args...) if(feat & fp_dbgp_flags){ \
    int count = 0; \
    for (; count < 32; count++) { \
        if ((1 << count) & feat) { \
            break; \
        } \
    } \
    if (level <= fp_trace_level) \
    printk("%s: " msg,fp_dbgp_arr[count], ## args); \
};
#else
#define FP_DEBUG(type, debug_level, msg, args...)
#endif

#endif /*End of _FP_LIBRARY_H_ File */
