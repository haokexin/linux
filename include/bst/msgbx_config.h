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

#ifndef _BST_MSGBX_H
#define _BST_MSGBX_H

#if defined(__aarch64__) || defined(__x86_64__)
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/io.h>
#include <linux/string.h>
#include <uapi/linux/sched/types.h>
#define IPC_RTE_KERNEL
#elif defined(__unix__) || defined(__posix__)
	#include <stdint.h>
	#include <stdbool.h>
	#include <string.h>
	#ifndef IPC_RTE_BAREMETAL
		#define IPC_RTE_POSIX
	#endif
#endif
#elif defined(__arm__) || defined(__XTENSA__) || defined(__riscv)
#include <stdint.h>
#include <stdbool.h>
#ifndef IPC_RTE_RTOS
	#define IPC_RTE_BAREMETAL
	#define BAREMETAL_VERSION_TRUNCATE
#endif
#else
#error "unsupported architecture. Please add includes for stdint.h and stdbool.h"
#endif

#if defined(CONFIG_ARCH_BSTC1200)
#define MSGBX_HW_TYPE_C1200
#include "msgbx_hw_c1200.h"
#elif defined(CONFIG_ARCH_BSTA2000)
#define MSGBX_HW_TYPE_A2000
#include "msgbx_hw_a2000.h"
// #define MULTI_DIE_HW_VERSION
#endif

#if defined(__clang__)
// Clang atomic function macros
#define ATOMIC_LOAD(ptr, order) __c11_atomic_load((ptr), (order))
#define ATOMIC_STORE(ptr, val, order) __c11_atomic_store((ptr), (val), (order))
#define ATOMIC_FETCH_ADD(ptr, val, order)                                      \
	__c11_atomic_fetch_add((ptr), (val), (order))
#define ATOMIC_FETCH_OR(ptr, val, order)                                       \
	__c11_atomic_fetch_or((ptr), (val), (order))
#define ATOMIC_FETCH_XOR(ptr, val, order)                                      \
	__c11_atomic_fetch_xor((ptr), (val), (order))
#elif defined(__GNUC__) || defined(__GNUG__)
// GCC atomic function macros
#define ATOMIC_LOAD(ptr, order) __atomic_load_n((ptr), ((order)))
#define ATOMIC_STORE(ptr, val, order) __atomic_store_n((ptr), (val), (order))
#define ATOMIC_FETCH_ADD(ptr, val, order)                                      \
	__atomic_fetch_add((ptr), (val), (order))
#define ATOMIC_FETCH_OR(ptr, val, order)                                       \
	__atomic_fetch_or((ptr), (val), (order))
#define ATOMIC_FETCH_XOR(ptr, val, order)                                      \
	__atomic_fetch_xor((ptr), (val), (order))
#else
#error "Unsupported compiler"
#endif

#define IPC_STATE_MGT_ENABLE
#define IPC_FLT_MGT_ENABLE
#define IPC_NO_DEBUG

/* log level definition
 * note: when you define 0, which means you do not need any log.
 */
#define MSGBX_MODULE_NAME "Msgbx_Proto"
#if defined(IPC_RTE_POSIX) || defined(IPC_RTE_BAREMETAL)
#include <stdio.h>
#define IPC_LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#endif
// if your RTE don't belong to define above, you could add your own print api for IPC_LOG function

enum log_level_e {
	LOG_LEVEL_NONE = 0,
	LOG_LEVEL_ERR = 1,
	LOG_LEVEL_WARNING,
	LOG_LEVEL_INFO,
	LOG_LEVEL_DEBUG,
};
#define G_LOG_LEVEL LOG_LEVEL_WARNING

// app_layer configuration
#define IPC_MAX_SUB_MSG_NUM 16
#define IPC_PAYLOAD_SIZE 32U

#define IPC_MAX_SUBSCRIPTION 34U
#define IPC_TOKEN_NUM 16U
#define IPC_TOKEN_GC_TIMES 2U
#define IPC_MAX_DATA_SIZE (IPC_MAX_SUB_MSG_NUM * IPC_PAYLOAD_SIZE)
#define IPC_OBJ_BUF_SIZE (IPC_MAX_DATA_SIZE / 4)

// trans_layer configuration
#define SESSION_MSG_BUFFER_COUNT 512
#define SESSION_PACKET_MSG_BUFFER_COUNT 248

// kernel spec get cpuid
uint8_t get_cpuid_by_endid(uint8_t end_id);
#define CPUID_ERR 0xff

#define ENDID_TO_CPUID(_end_id) ({\
	uint8_t _cpuid = get_cpuid_by_endid(_end_id);\
\
	if (_cpuid == CPUID_ERR) \
		return -1; \
	_cpuid; \
})

#ifdef CONFIG_MSGBOX_DEBUG_FS
extern int msgbox_log_level_get(void);
#else
static inline int msgbox_log_level_get(void)
{
#ifdef IPC_LOG_SUPPORT
	return G_LOG_LEVEL;
#else
	return 0;
#endif
}
#endif

#endif
