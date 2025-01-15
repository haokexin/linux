#ifndef _BSTIPC_CFG_H
#define _BSTIPC_CFG_H

#define _POSIX_C_SOURCE
#ifdef _POSIX_C_SOURCE
#include <linux/types.h>
#else

#ifdef __INT8_TYPE__
typedef __INT8_TYPE__ int8_t;
#endif

#ifdef __INT16_TYPE__
typedef __INT16_TYPE__ int16_t;
#endif
#ifdef __INT32_TYPE__
typedef __INT32_TYPE__ int32_t;
#endif

#ifdef __INT64_TYPE__
//typedef __INT64_TYPE__ int64_t;
#endif

#ifdef __UINT8_TYPE__
typedef __UINT8_TYPE__ uint8_t;
#endif
#ifdef __UINT16_TYPE__
typedef __UINT16_TYPE__ uint16_t;
#endif
#ifdef __UINT32_TYPE__
typedef __UINT32_TYPE__ uint32_t;
#endif
#ifdef __UINT64_TYPE__
//typedef __UINT64_TYPE__ uint64_t;
#endif

#ifndef NULL
#ifndef __cplusplus
#define NULL ((void *)0)
#else /* C++ */
#define NULL 0
#endif
#endif

#ifndef __cplusplus

#define bool	_Bool
#if defined __STDC_VERSION__ && __STDC_VERSION__ > 201710L
#define true	((_Bool)+1u)
#define false	((_Bool)+0u)
#else
#define true	1
#define false	0
#endif

#else /* __cplusplus */

/* Supporting _Bool in C++ is a GCC extension.  */
#define _Bool	bool

#endif /* __cplusplus */

#endif // _POSIX_C_SOURCE

#include <linux/types.h>
#include <linux/io.h>
//#define IPC_TRNAS_DEBUG 0
#ifdef IPC_TRNAS_DEBUG
#include <stdio.h>
#else
#define printf printk
#endif

// bstipc state management enable
#define IPC_STATE_MGT_ENABLE
#define IPC_FLT_MGT_ENABLE

// msgbx raw msg format definition
typedef struct ipc_hw_raw_msg_header
{
    uint32_t pid : 8;
    uint32_t cid : 8;
    uint32_t len : 4;
    uint32_t is_sec : 1;
    uint32_t is_32b : 1;
    uint32_t is_eof : 1;
    uint32_t resh : 1;
    uint32_t sid : 4;
    uint32_t fid : 4;
    uint32_t cmd : 8;
    uint32_t typ : 4;
    uint32_t tok : 4;
    uint32_t idx : 4;
    uint32_t res : 8;
    uint32_t ver : 4;
} rw_msg_header;

typedef struct ipc_hw_raw_msg
{
    rw_msg_header header;
    uint64_t payload[4];
} rw_msg;

#define MACROS_TABLE                                                                                                   \
    X_MACRO(CPU_0, ((1U << 4) | 0U))                                                                                   \
    X_MACRO(CPU_1, ((1U << 4) | 1U))                                                                                   \
    X_MACRO(CPU_2, ((1U << 4) | 2U))                                                                                   \
    X_MACRO(CPU_3, ((1U << 4) | 3U))                                                                                   \
    X_MACRO(CPU_4, ((1U << 4) | 4U))                                                                                   \
    X_MACRO(CPU_5, ((1U << 4) | 5U))                                                                                   \
    X_MACRO(CPU_6, ((1U << 4) | 6U))                                                                                   \
    X_MACRO(CPU_7, ((1U << 4) | 7U))                                                                                   \
    X_MACRO(CPUMP2_0, ((2U << 4) | 0U))                                                                                \
    X_MACRO(CPUMP2_1, ((2U << 4) | 1U))                                                                                \
    X_MACRO(ISPCV_0, ((3U << 4) | 0U))                                                                                 \
    X_MACRO(ISPCV_1, ((3U << 4) | 1U))                                                                                 \
    X_MACRO(ISPCV_2, ((3U << 4) | 2U))                                                                                 \
    X_MACRO(ISPCV_3, ((3U << 4) | 3U))                                                                                 \
    X_MACRO(ISPCV_4, ((3U << 4) | 4U))                                                                                 \
    X_MACRO(NET_0, ((4U << 4) | 0U))                                                                                   \
    X_MACRO(DMA_0, ((5U << 4) | 0U))                                                                                   \
    X_MACRO(DMA_1, ((5U << 4) | 1U))                                                                                   \
    X_MACRO(SWITCH_0, ((6U << 4) | 0U))                                                                                \
    X_MACRO(SWITCH_1, ((6U << 4) | 1U))                                                                                \
    X_MACRO(SWITCH_2, ((6U << 4) | 2U))                                                                                \
    X_MACRO(SWITCH_3, ((6U << 4) | 3U))                                                                                \
    X_MACRO(SWITCH_4, ((6U << 4) | 4U))                                                                                \
    X_MACRO(SWITCH_5, ((6U << 4) | 5U))                                                                                \
    X_MACRO(SECURE_0, ((7U << 4) | 0U))                                                                                \
    X_MACRO(SECURE_1, ((7U << 4) | 1U))                                                                                \
    X_MACRO(SAFETY_0, ((8U << 4) | 0U))                                                                                \
    X_MACRO(SAFETY_1, ((8U << 4) | 1U))                                                                                \
    X_MACRO(REALTIME_0, ((9U << 4) | 0U))                                                                              \
    X_MACRO(REALTIME_1, ((9U << 4) | 1U))                                                                              \
    X_MACRO(REALTIME_2, ((9U << 4) | 2U))                                                                              \
    X_MACRO(REALTIME_3, ((9U << 4) | 3U))                                                                              \
    X_MACRO(REALTIME_4, ((9U << 4) | 4U))                                                                              \
    X_MACRO(REALTIME_5, ((9U << 4) | 5U))                                                                              \
    X_MACRO(MEDIA_0, ((10U << 4) | 0U))

enum ipc_msg_pid_e
{
#define X_MACRO(a, b) a = b,
    MACROS_TABLE
#undef X_MACRO
};

enum ipc_msg_fid_e
{
    DEF = 0,
    F1 = 1,
    F2 = 2,
    F3 = 3,
    F4 = 4, 
    F5 = 5,
    F6 = 6,
    F7 = 7
};

#endif