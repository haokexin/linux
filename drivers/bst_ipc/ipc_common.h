/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef IPC_COMMON_H
#define IPC_COMMON_H

#include <linux/types.h>
#include <linux/io.h>
#include <bst/ipc_interface.h>

// ipc configuration definition
// use to define ipc log format
#define IPC_LOG_SUPPORT

// use to apu sample initiate
// #define ENABLE_APU_SAMPLE

// use to support ipc message dump for ipcctl
// #define MSG_DUMP

// use to support set same resource core and destination core function
// #define ENABLE_SRC_DST_SAME

// use to support 64bit message extending to 128bit
// #define MSG_SIZE_EXTENSION

// use to dump ipc message hash map
// #define DUMP_MSG_HASH_TABLE

// use to support ipc resource core definition
#define IPC_SRC_DEFINITION_SUPPORT

#define IPC_INFO_MAX_NUM 4096

// max count definition
#define MSG_CMD_MAX	     255
#define MSG_CMD_MAX_IDX	     256
#define REGISTER_MAP_MAX     256
#define REGISTER_MAP_MAX_IDX     22
#define SUBSCRIPTION_MAP_MAX 22

// ipc cache coherency command define
#define ipc_isb() do {__asm__ __volatile__("isb" : : : "memory"); } while (false)
#define ipc_dsb() do {__asm__ __volatile__("dsb sy" : : : "memory"); } while (false)
#define ipc_dmb() do {__asm__ __volatile__("dmb sy" : : : "memory"); } while (false)

// ipc log definition
extern int32_t sysfs_get_log_level(void);

#ifdef IPC_LOG_SUPPORT
#define IPC_LOG(level, format, ...)                                          \
	do {                                                                 \
		if (sysfs_get_log_level() >= LOG_LEVEL_INFO)                 \
			pr_info("[%s]: %s %d: " format "\n", IPC_DRIVER_NAME, \
			       __func__, __LINE__, ##__VA_ARGS__);       \
	} while (false)

/* print info */
#define IPC_INFO_PRINT(format, ...)                          \
	do {                                                 \
		pr_info(format "\n", ##__VA_ARGS__); \
	} while (false)

/*3*/
#define IPC_LOG_ERR(format, ...)                                        \
	do {                                                            \
		if (sysfs_get_log_level() >= LOG_LEVEL_ERR)             \
			pr_info("[E][%s]: %s %d:" format "\n",  \
			       IPC_DRIVER_NAME, __func__, __LINE__, \
			       ##__VA_ARGS__);                          \
	} while (false)

/*4*/
#define IPC_LOG_WARNING(format, ...)                                       \
	do {                                                               \
		if (sysfs_get_log_level() >= LOG_LEVEL_WARNING)            \
			pr_info("[W][%s]: %s %d:" format "\n", \
			       IPC_DRIVER_NAME, __func__, __LINE__,    \
			       ##__VA_ARGS__);                             \
	} while (false)

/*"6"*/
#define IPC_LOG_INFO(format, ...)                                       \
	do {                                                            \
		if (sysfs_get_log_level() >= LOG_LEVEL_INFO)            \
			pr_info("[I][%s]: %s %d:" format "\n", \
			       IPC_DRIVER_NAME, __func__, __LINE__, \
			       ##__VA_ARGS__);                          \
	} while (false)

/*"7"*/
#define IPC_LOG_DEBUG(format, ...)                                       \
	do {                                                             \
		if (sysfs_get_log_level() >= LOG_LEVEL_DEBUG)            \
			pr_info("[D][%s]: %s %d:" format "\n", \
			       IPC_DRIVER_NAME, __func__, __LINE__,  \
			       ##__VA_ARGS__);                           \
	} while (false)

#else
#define IPC_LOG(level, format, ...)
#define IPC_LOG_ERR(format, ...)
#define IPC_LOG_WARNING(format, ...)
#define IPC_LOG_INFO(format, ...)
#define IPC_LOG_DEBUG(format, ...)
#endif // IPC_LOG_SUPPORT

/**
 * 64 bit and less is supported
 * sample usage:  find_bit_zero(0xafffffff0, 64, 0)
 * @return lowest bit of 0 bit
 */
int32_t find_bit_zero(uint64_t number, int32_t bit_num, unsigned int bit_offset);

/**
 * 64 bit and less is supported
 * sample usage:  find_1_bit(0xafffffff0, 64, 0)
 *  @return lowest bit of 1 bit
 */
int32_t find_1_bit(uint64_t number, int32_t bit_num, unsigned int bit_offset);

static inline int get_leading_zero_num(size_t value)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
	int num;
#pragma GCC diagnostic pop

	__asm__("clz %0, %1" : "=r"(num) : "r"(value) :);

	return num;
}

static inline int get_msb_bit1_index(size_t value)
{
	return ((sizeof(value) << 3) - 1 - get_leading_zero_num(value));
}

static inline int get_1bit_count(uint64_t value)
{
	uint64_t count;

	for (count = 0; value; count++)
		value &= value - 1;
	return count;
}

static inline int get_all_1bit(uint64_t map, uint8_t id[])
{
	int cnt = 0;

	while (map > 0) {
		int next_psn = __builtin_ffsll(map) - 1;

		map &= (map - 1);
		id[cnt++] = next_psn;
	}
	return cnt;
}

// ipc driver diagnostic config
enum log_level_e {
	LOG_LEVEL_ERR,
	LOG_LEVEL_WARNING,
	LOG_LEVEL_INFO,
	LOG_LEVEL_DEBUG,
};

struct diag_info_send_err {
	int32_t ipc_no_ready;
	int32_t session_invalid;
	int32_t no_ACK;
	int32_t queue_full;
};

struct diag_info_recv_err {
	int32_t ipc_no_ready;
	int32_t session_invalid;
	int32_t queue_empty;
	int32_t timeout;
};

struct diag_info {
	int32_t session_id;
	enum ipc_core_e src;
	enum ipc_core_e dst;
	int32_t num_of_send_msg;
	int32_t num_of_recv_msg;
	struct diag_info_send_err send_err;
	struct diag_info_recv_err recv_err;
};

#define SESSION_NUM 64 // bitmap used

extern enum ipc_core_e ipc_channel[IPC_CORE_MAX];
extern struct diag_info diagnose_info[SESSION_NUM];
extern struct platform_device *g_ipc_platform_dev; // read only variable
extern struct ipc_all_cores_register_addr *g_ipc_all_cores_register_addr;
extern struct ipc_all_cores_register_addr *g_ipc_all_cores_register_addr_uaddr;
extern struct ipc_memblock *g_ipc_memblock;

#ifdef MSG_DUMP
extern bool save_flag[IPC_CORE_MAX];
extern uint32_t save_cnt[IPC_CORE_MAX];
#endif

extern bool get_ipc_init_status(void);

#endif // IPC_COMMON_H
