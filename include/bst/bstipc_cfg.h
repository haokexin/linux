/* SPDX-License-Identifier: GPL-2.0 OR Apache 2.0
 *
 * Copyright (c) 2024 Black Sesame Technologies
 *
 * This program is also distributed under the terms of the Apache 2.0
 * License.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// Note: The following configurations are fixed and should NOT be modified.
#ifndef _BSTIPC_CFG_H
#define _BSTIPC_CFG_H

#include "msgbx_config.h"

struct _rw_msg_t {
	rw_msg_header_t header;
	uint64_t payload[4];
};
#define rw_msg_t struct _rw_msg_t

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
	ERR_CREATE_SES_ALLOC_FAIL,
	ERR_APP_PARAM = 50,
	ERR_APP_SERDES,
	ERR_APP_TOK,
	ERR_APP_START,
	ERR_APP_STOP,
	ERR_APP_TIMEOUT,
	ERR_APP_UNKNOWN_CMD,
	ERR_APP_EXPORT_REG_MAP_SIZE_ERR,
	ERR_APP_SERVER_IS_NOT_AVAIL = 404,
	ERR_APP_SERVER_IS_FULL = 500,
};

enum _ipc_msg_type_t {
	MSGBX_MSG_TYPE_INVALID = 0,
	MSGBX_MSG_TYPE_METHOD,
	MSGBX_MSG_TYPE_REPLY,
	MSGBX_MSG_TYPE_BROADCAST,
	MSGBX_MSG_TYPE_PROTOCOL,
	MSGBX_MSG_TYPE_USERDEFINED = 10,
	MSGBX_MSG_TYPE_HARDWARE = 15,
	MSGBX_MSG_TYPE_MAX = 16,
};
#define ipc_msg_type_t enum _ipc_msg_type_t

enum _ipc_ses_role_t {
	MSGBX_SES_ROLE_INVALID = 0,
	MSGBX_SES_ROLE_CLIENT = 1,
	MSGBX_SES_ROLE_SERVER = 2,
	MSGBX_SES_ROLE_FASTPATH = 3,
	MSGBX_SES_ROLE_MAX
};
#define ipc_ses_role_t enum _ipc_ses_role_t

#define QUERY_INFO_HAS_AVAIL_RECV_MSG 0
#define QUERY_INFO_DST_STS_OFFLINE 1
#define QUERY_INFO_DST_STS_ONLINE 2
#define QUERY_INFO_DST_STS_CHANGED 3

struct _sts_endmap_t {
	uint64_t hi_map;
	uint64_t lo_map;
};
#define sts_endmap_t struct _sts_endmap_t

struct _msgbox_debug_info_t {
	uint8_t role;
	uint8_t cid;
	uint8_t ccid;
	uint8_t send_fail_cnt;
	uint32_t send_msg_cnt;
	uint32_t send_rw_msg_cnt;
	uint32_t recv_rw_msg_cnt;
	uint32_t in_rw_msg_cnt;
	uint32_t recv_msg_1_cnt;
	uint32_t recv_msg_2_cnt;
	uint32_t res;
	uint64_t send_start_time;
	uint64_t send_end_time;
	uint64_t get_msg_time;
	uint64_t max_send_time;
};
#define msgbox_debug_info_t struct _msgbox_debug_info_t

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
#ifndef IPC_RTE_BAREMETAL
	if (dst)
		memset(dst, val, count);
	return dst;
#else
	if (dst) {
		uint8_t *ret = (uint8_t *)dst;

		while (count--)
			*ret++ = (char)val;
	}
	return dst;
#endif
}

static inline void *ipc_memcpy(void* __restrict dest, const void* __restrict src, size_t n)
{
#ifndef IPC_RTE_BAREMETAL
	if (dest)
		memcpy(dest, src, n);
	return dest;
#else
    uint8_t* d = (uint8_t*)dest;
    const uint8_t* s = (const uint8_t*)src;

    const size_t word_size = sizeof(uintptr_t);
    const size_t align_mask = word_size - 1;

    const uintptr_t d_addr = (uintptr_t)d;
    const uintptr_t s_addr = (uintptr_t)s;

    if ((d_addr & align_mask) == (s_addr & align_mask)) {
        while ((d_addr & align_mask) && n > 0) {
            *d++ = *s++;
            n--;
        }

        uintptr_t* d_word = (uintptr_t*)d;
        const uintptr_t* s_word = (const uintptr_t*)s;
        size_t num_words = n / word_size;

        for (size_t i = 0; i < num_words / 4; i++) {
            d_word[0] = s_word[0];
            d_word[1] = s_word[1];
            d_word[2] = s_word[2];
            d_word[3] = s_word[3];
            d_word += 4;
            s_word += 4;
            num_words -= 4;
        }

        while (num_words--) {
            *d_word++ = *s_word++;
        }

        d = (uint8_t*)d_word;
        s = (const uint8_t*)s_word;
        n %= word_size;
    }

    while (n--) {
        *d++ = *s++;
    }

    return dest;
#endif
}

#if defined(IPC_RTE_KERNEL)
#define IPC_LOG_ERR(format, ...)                                               \
	do {                                                                   \
		if (msgbox_log_level_get() >= LOG_LEVEL_ERR) \
			pr_err("[%s][%s]:" format "\n", MSGBX_MODULE_NAME, __func__,   \
		       ##__VA_ARGS__);                                         \
	} while (false)

#define IPC_LOG_WARNING(format, ...)                                           \
	do {                                                                   \
		if (msgbox_log_level_get() >= LOG_LEVEL_WARNING) \
			pr_warn("[%s][%s]:" format "\n", MSGBX_MODULE_NAME, __func__,  \
				##__VA_ARGS__);                                        \
	} while (false)

#define IPC_LOG_INFO(format, ...)                                              \
	do {                                                                   \
		if (msgbox_log_level_get() >= LOG_LEVEL_INFO) \
			pr_info("[%s][%s]:" format "\n", MSGBX_MODULE_NAME, __func__,  \
				##__VA_ARGS__);                                        \
	} while (false)

#define IPC_LOG_DEBUG(format, ...)                                             \
	do {                                                                   \
		if (msgbox_log_level_get() >= LOG_LEVEL_DEBUG) \
			pr_info("[%s][%s]:" format "\n", MSGBX_MODULE_NAME, __func__, \
				##__VA_ARGS__);                                       \
	} while (false)
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
