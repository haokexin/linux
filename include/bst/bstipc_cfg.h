/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is also distributed under the terms of the BSD 3-Clause
 * License.
 *
 * Copyright (C) 2023 Black Sesame Technologies. Inc.
 */

#ifndef _BSTIPC_CFG_H
#define _BSTIPC_CFG_H

#if defined(__aarch64__) || defined(__x86_64__)
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/io.h>
#define IPC_RTE_KERNEL
#elif defined(__unix__) || defined(__posix__)
#include <stdint.h>
#include <stdbool.h>
#define IPC_RTE_POSIX
#endif
#elif defined(__arm__) || defined(__XTENSA__) || defined(__riscv)
#include <stdint.h>
#include <stdbool.h>
#define IPC_RTE_BAREMETAL
#define BAREMETAL_VERSION_TRUNCATE
#else
#error "unsupported architecture. Please add includes for stdint.h and stdbool.h"
#endif

#define IPC_STATE_MGT_ENABLE
#define IPC_FLT_MGT_ENABLE

#if defined(__riscv)
#define _Atomic
#define IPC_NO_STRING
#define IPC_NO_BYTE_BUFFER
#define IPC_NO_DEBUG
#define IPC_NO_FIRE_AND_FORGET
#define IPC_NO_ATOMIC
#define IPC_SHARED_SERIALIZER
// add your own atomic function macros
#endif


#ifdef CONFIG_MSGBOX_DEBUG_FS
extern bool msgbox_debug_has_en(void);
#else
static inline bool msgbox_debug_has_en(void)
{
#ifdef IPC_LOG_SUPPORT
	return true;
#else
	return false;
#endif
}
#endif
extern u8 msgbx_get_start_pid(void);

#if defined(__clang__)
// Clang atomic function macros
#define ATOMIC_LOAD(ptr, order) __c11_atomic_load((ptr), (order))
#define ATOMIC_STORE(ptr, val, order) __c11_atomic_store((ptr), (val), (order))
#define ATOMIC_EXCHANGE(ptr, val, order)                                       \
	__c11_atomic_exchange((ptr), (val), (order))
#define ATOMIC_COMPARE_EXCHANGE_STRONG(ptr, expected, desired, success,        \
				       failure)                                \
	__c11_atomic_compare_exchange_strong((ptr), (expected), (desired),     \
					     (success), (failure))
#define ATOMIC_COMPARE_EXCHANGE_WEAK(ptr, expected, desired, success, failure) \
	__c11_atomic_compare_exchange_weak((ptr), (expected), (desired),       \
					   (success), (failure))
#define ATOMIC_FETCH_ADD(ptr, val, order)                                      \
	__c11_atomic_fetch_add((ptr), (val), (order))
#define ATOMIC_FETCH_SUB(ptr, val, order)                                      \
	__c11_atomic_fetch_sub((ptr), (val), (order))
#define ATOMIC_FETCH_AND(ptr, val, order)                                      \
	__c11_atomic_fetch_and((ptr), (val), (order))
#define ATOMIC_FETCH_OR(ptr, val, order)                                       \
	__c11_atomic_fetch_or((ptr), (val), (order))
#define ATOMIC_FETCH_XOR(ptr, val, order)                                      \
	__c11_atomic_fetch_xor((ptr), (val), (order))

#elif defined(__GNUC__) || defined(__GNUG__)
// GCC atomic function macros
#define ATOMIC_LOAD(ptr, order) __atomic_load_n((ptr), ((order)))
#define ATOMIC_STORE(ptr, val, order) __atomic_store_n((ptr), (val), (order))
#define ATOMIC_EXCHANGE(ptr, val, order)                                       \
	__atomic_exchange_n((ptr), (val), (order))
#define ATOMIC_COMPARE_EXCHANGE_STRONG(ptr, expected, desired, success,        \
				       failure)                                \
	__atomic_compare_exchange_n((ptr), (expected), (desired), 0,           \
				    (success), (failure))
#define ATOMIC_COMPARE_EXCHANGE_WEAK(ptr, expected, desired, success, failure) \
	__atomic_compare_exchange_n((ptr), (expected), (desired), 1,           \
				    (success), (failure))
#define ATOMIC_FETCH_ADD(ptr, val, order)                                      \
	__atomic_fetch_add((ptr), (val), (order))
#define ATOMIC_FETCH_SUB(ptr, val, order)                                      \
	__atomic_fetch_sub((ptr), (val), (order))
#define ATOMIC_FETCH_AND(ptr, val, order)                                      \
	__atomic_fetch_and((ptr), (val), (order))
#define ATOMIC_FETCH_OR(ptr, val, order)                                       \
	__atomic_fetch_or((ptr), (val), (order))
#define ATOMIC_FETCH_XOR(ptr, val, order)                                      \
	__atomic_fetch_xor((ptr), (val), (order))
#define ATOMIC_ADD_FETCH(ptr, val, order)                                      \
	__atomic_add_fetch((ptr), (val), (order))
#define ATOMIC_SUB_FETCH(ptr, val, order)                                      \
	__atomic_sub_fetch((ptr), (val), (order))
#define ATOMIC_AND_FETCH(ptr, val, order)                                      \
	__atomic_and_fetch((ptr), (val), (order))
#define ATOMIC_OR_FETCH(ptr, val, order)                                       \
	__atomic_or_fetch((ptr), (val), (order))
#define ATOMIC_XOR_FETCH(ptr, val, order)                                      \
	__atomic_xor_fetch((ptr), (val), (order))
#else
#error "Unsupported compiler"
#endif

/* log management
 */
#define MSGBX_MODULE_NAME "Msgbx_Proto"
/* if your RTE don't belong to define above, you could add your own print api for IPC_LOG function
 */
/* log level definition
 * note: when you define 0, which means you do not need any log.
 */
enum log_level_e {
	LOG_LEVEL_NONE = 0,
	LOG_LEVEL_ERR = 1,
	LOG_LEVEL_WARNING,
	LOG_LEVEL_INFO,
	LOG_LEVEL_DEBUG,
};
#define G_LOG_LEVEL LOG_LEVEL_INFO

#define MSGBX_UCHAR_MAX 255U
#define ALIGN_SIZE (sizeof(size_t))
#define HALF_ALIGN ((ALIGN_SIZE) / 2)
#define ONES ((size_t)-1 / MSGBX_UCHAR_MAX)
#define HIGHS (ONES * (MSGBX_UCHAR_MAX / 2 + 1))
#define HASZERO(x) (((x)-ONES) & ~(x)&HIGHS)

// msgbx raw msg format definition
struct _rw_msg_header_t {
	uint32_t pid : 8;
	uint32_t cid : 8;
	uint32_t len : 4;
	uint32_t is_64b : 1;
	uint32_t is_sec : 1;
	uint32_t is_eof : 1;
	uint32_t resh : 1;
	uint32_t sid : 4;
	uint32_t fid : 4;
	uint32_t cmd : 8;
	uint32_t typ : 4;
	uint32_t res : 4;
	uint32_t idx : 4;
	uint32_t tok : 8;
	uint32_t ver : 4;
};
#define rw_msg_header_t struct _rw_msg_header_t

struct _rw_msg_t {
	rw_msg_header_t header;
	uint64_t payload[4];
};
#define rw_msg_t struct _rw_msg_t

enum ipc_msg_pid_e {
	CPU_0 = ((1U << 4) | 0U),
	CPU_1 = ((1U << 4) | 1U),
	CPU_2 = ((1U << 4) | 2U),
	CPU_3 = ((1U << 4) | 3U),
	CPU_4 = ((1U << 4) | 4U),
	CPU_5 = ((1U << 4) | 5U),
	CPU_6 = ((1U << 4) | 6U),
	CPU_7 = ((1U << 4) | 7U),
	CPUMP2_0 = ((2U << 4) | 0U),
	CPUMP2_1 = ((2U << 4) | 1U),
	ISPCV_0 = ((3U << 4) | 0U),
	ISPCV_1 = ((3U << 4) | 1U),
	ISPCV_2 = ((3U << 4) | 2U),
	ISPCV_3 = ((3U << 4) | 3U),
	ISPCV_4 = ((3U << 4) | 4U),
	NET_0 = ((4U << 4) | 0U),
	DMA_0 = ((5U << 4) | 0U),
	DMA_1 = ((5U << 4) | 1U),
	SWITCH_0 = ((6U << 4) | 0U),
	SWITCH_1 = ((6U << 4) | 1U),
	SWITCH_2 = ((6U << 4) | 2U),
	SWITCH_3 = ((6U << 4) | 3U),
	SWITCH_4 = ((6U << 4) | 4U),
	SWITCH_5 = ((6U << 4) | 5U),
	SECURE_0 = ((7U << 4) | 0U),
	SECURE_1 = ((7U << 4) | 1U),
	SAFETY_0 = ((8U << 4) | 0U),
	SAFETY_1 = ((8U << 4) | 1U),
	REALTIME_0 = ((9U << 4) | 0U),
	REALTIME_1 = ((9U << 4) | 1U),
	REALTIME_2 = ((9U << 4) | 2U),
	REALTIME_3 = ((9U << 4) | 3U),
	REALTIME_4 = ((9U << 4) | 4U),
	REALTIME_5 = ((9U << 4) | 5U),
	MEDIA_0 = ((10U << 4) | 0U),
};

enum ipc_msg_fid_e {
	DEF = 0,
	F1 = 1,
	F2 = 2,
	F3 = 3,
	F4 = 4,
	F5 = 5,
	F6 = 6,
	F7 = 7
};

// error code
enum ipc_err_code_e {
	RESULT_SUCCESS = 0,
	RESULT_ERROR = 1,
	ERR_TRANS_INIT_FAIL = 10,
	ERR_TRANS_INIT_RECV_REGISTER_FAIL,
	ERR_TRANS_INIT_START_FAIL,
	ERR_TRANS_END_REGISTER_FAIL,
	ERR_CREATE_SES_OUT_RANGE,
	ERR_CREATE_SES_ROLE_INVALID,
	ERR_CREATE_SES_FAIL,
	ERR_SET_FIL_RULE_FAIL,
	ERR_SES_IS_INVALID,
	ERR_PID_IS_INVALID,
	ERR_CID_IS_INVALID,
	ERR_CID_EQUAL_PID,
	ERR_TYP_IS_INVALID,
	ERR_SEND_MSG_FAIL,
	ERR_RECV_MSG_FAIL,
	ERR_SES_CLOSE_FAIL,
	ERR_DEINIT_FAIL,
	ERR_REGISTER_METHOD_REPEATE,
	ERR_ERR_HANDLE_FAIL,
	ERR_DES_IS_OFFLINE,
	ERR_FASTPAH_GET_TIMEOUT,
	ERR_APP_PARAM = 50,
	ERR_APP_SERDES,
	ERR_APP_TOK,
	ERR_APP_START,
	ERR_APP_STOP,
	ERR_APP_TIMEOUT,
	ERR_APP_UNKNOWN_CMD,
	ERR_APP_EXPORT_REG_MAP_SIZE_ERR,
};

/**
 * @brief Get the UUID from the header
 *
 * This function calculates the UUID from the given header by combining the
 * PID, SID, and FID fields.
 *
 * @param header The header containing the PID, SID, and FID fields
 * @return The calculated UUID
 */
static inline uint16_t ipc_msg_get_uuid(rw_msg_header_t header)
{
	return (header.pid << 8) | (header.sid << 4) | (header.fid);
}

/**
 * @brief Set memory to a specific value
 *
 * This function sets the memory pointed to by 'dst' to the specified 'val'
 * for 'count' number of times.
 *
 * @param dst Pointer to the memory to be set
 * @param val The value to set the memory to, should be in the range of
 * int8_t.
 * @param count The number of times to set the memory
 * @return Pointer to the memory after it has been set
 */
static inline void *ipc_memset(void *dst, int val, uint32_t count)
{
	if (dst) {
		uint8_t *ret = (uint8_t *)dst;

		while (count--)
			*ret++ = (char)val;
	}
	return dst;
}

#ifndef IPC_NO_STRING
/**
 * @brief Get the length of string.
 *
 * This function get the length of string pointed by 's'.
 *
 * @param s Pointer to the string
 * @return The length of the string, exclude '\0'.
 */
static inline size_t ipc_strlen(const char *s)
{
	const char *a = s;
#ifdef __GNUC__
	typedef size_t __attribute__((__may_alias__)) word;
	const word *w;

	for (; (uintptr_t)s % ALIGN_SIZE; s++)
		if (!*s)
			return s - a;
	for (w = (const word *)s; !HASZERO(*w); w++)
		;
	s = (const char *)w;
#endif
	for (; *s; s++)
		;
	return s - a;
}
#endif

#if defined (IPC_RTE_KERNEL)
#ifndef CONFIG_MSGBOX_DEBUG_FS
#define IPC_LOG_ERR(format, ...)                                               \
	do {                                                                   \
		pr_err("[%s][%s]:" format "\n", MSGBX_MODULE_NAME, __func__,   \
		       ##__VA_ARGS__);                                         \
	} while (false)

#define IPC_LOG_WARNING(format, ...)                                           \
	do {                                                                   \
		pr_warn("[%s][%s]:" format "\n", MSGBX_MODULE_NAME, __func__,  \
			##__VA_ARGS__);                                        \
	} while (false)

#define IPC_LOG_INFO(format, ...)                                              \
	do {                                                                   \
		pr_info("[%s][%s]:" format "\n", MSGBX_MODULE_NAME, __func__,  \
			##__VA_ARGS__);                                        \
	} while (false)

#define IPC_LOG_DEBUG(format, ...)                                             \
	do {                                                                   \
		pr_debug("[%s][%s]:" format "\n", MSGBX_MODULE_NAME, __func__, \
			    ##__VA_ARGS__);                                    \
	} while (false)
#else
#define IPC_LOG(level, format, ...)
#define IPC_LOG_ERR(format, ...) \
	do { \
		if (likely(msgbox_debug_has_en())) \
			pr_err("[D][%s]: %s %d:" format "\n", \
				MSGBX_MODULE_NAME, __func__, __LINE__, \
				##__VA_ARGS__); \
	} while (false)
#define IPC_LOG_WARNING(format, ...) \
	do { \
		if (likely(msgbox_debug_has_en())) \
			pr_warn("[D][%s]: %s %d:" format "\n", \
				MSGBX_MODULE_NAME, __func__, __LINE__, \
				##__VA_ARGS__); \
	} while (false)
#define IPC_LOG_INFO(format, ...) \
	do { \
		if (likely(msgbox_debug_has_en())) \
			pr_info("[D][%s]: %s %d:" format "\n", \
				MSGBX_MODULE_NAME, __func__, __LINE__, \
				##__VA_ARGS__); \
	} while (false)
#define IPC_LOG_DEBUG(format, ...) \
	do { \
		if (likely(msgbox_debug_has_en())) \
			pr_info("[D][%s]: %s %d:" format "\n", \
				MSGBX_MODULE_NAME, __func__, __LINE__, \
				##__VA_ARGS__); \
	} while (false)
#endif
#else
#define IPC_LOG_ERR(format, ...)                                               \
	do {                                                                   \
		if (G_LOG_LEVEL >= LOG_LEVEL_ERR)                              \
			IPC_LOG("[E][%s][%s]:" format "\n", MSGBX_MODULE_NAME, \
				__func__, ##__VA_ARGS__);                      \
	} while (false)

#define IPC_LOG_WARNING(format, ...)                                           \
	do {                                                                   \
		if (G_LOG_LEVEL >= LOG_LEVEL_WARNING)                          \
			IPC_LOG("[W][%s][%s]:" format "\n", MSGBX_MODULE_NAME, \
				__func__, ##__VA_ARGS__);                      \
	} while (false)

#define IPC_LOG_INFO(format, ...)                                              \
	do {                                                                   \
		if (G_LOG_LEVEL >= LOG_LEVEL_INFO)                             \
			IPC_LOG("[I][%s][%s]:" format "\n", MSGBX_MODULE_NAME, \
				__func__, ##__VA_ARGS__);                      \
	} while (false)

#define IPC_LOG_DEBUG(format, ...)                                             \
	do {                                                                   \
		if (G_LOG_LEVEL >= LOG_LEVEL_DEBUG)                            \
			IPC_LOG("[D][%s][%s]:" format "\n", MSGBX_MODULE_NAME, \
				__func__, ##__VA_ARGS__);                      \
	} while (false)
#endif
#endif
//\ No newline at end of file
