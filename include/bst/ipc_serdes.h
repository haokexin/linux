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

#ifndef IPC_SERDES_H
#define IPC_SERDES_H
#include "bstipc_cfg.h"
#ifdef __cplusplus
extern "C" {
#endif

// macro definition
#define IPC_MAX_SUB_MSG_NUM CONFIG_IPC_MAX_SUB_MSG_NUM
#define IPC_PAYLOAD_SIZE 32U
#define IPC_CEIL(X, A) (((X) + (A)-1) / (A))
#define IPC_CHECK_ALIGN(x) ((x) > 0 && ((x) & ((x)-1)) == 0)

struct _serdes_t {
	rw_msg_header_t header;
	uint8_t *ptr;
	uint32_t index;
	uint32_t pos;
	uint32_t rcv_index;
	uint64_t recv_start_time;
	uint64_t recv_end_time;
	uint64_t recv_get_time;
	rw_msg_t msg_pool[IPC_MAX_SUB_MSG_NUM];
};
#define serdes_t struct _serdes_t

/**
 * @brief Initialize the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @return int32_t Returns 0 on success, -1 on failure.
 */
static inline int32_t ipc_ser_init(serdes_t *serdes)
{
	if (!serdes)
		return -1;
	serdes->index = 0;
	serdes->pos = 0;
	serdes->ptr = (uint8_t *)(serdes->msg_pool[serdes->index].payload);
	return 0;
}

/**
 * @brief Move to the next message in the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @return int32_t Returns 0 on success, -1 on failure.
 */
static inline int32_t ipc_serdes_next_msg(serdes_t *serdes)
{
	if (!serdes)
		return -1;
	if (serdes->index == IPC_MAX_SUB_MSG_NUM - 1)
		return 0;

	++serdes->index;
	serdes->ptr = (uint8_t *)(serdes->msg_pool[serdes->index].payload);
	serdes->pos = 0;
	return 0;
}

/**
 * @brief Set the header of the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @param header The header to set.
 * @return int32_t Returns 0 on success, -1 on failure.
 */
static inline int32_t ipc_ser_set_header(serdes_t *serdes,
					 rw_msg_header_t header)
{
	if (!serdes)
		return -1;
	serdes->header = header;
	return 0;
}

static inline bool isAligned(const void *ptr)
{
	return ((uintptr_t)ptr % ALIGN_SIZE) == 0;
}

static inline bool isAligned4(const void *ptr)
{
	return ((uintptr_t)ptr % 4) == 0;
}

static inline bool isAligned2(const void *ptr)
{
	return ((uintptr_t)ptr % 2) == 0;
}

/**
 * @brief Put data into the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @param data Pointer to the data to put.
 * @param size The size of the data.
 * @return int32_t Returns 0 on success, -1 on failure.
 */
static inline int32_t ipc_ser_put(serdes_t *serdes, const uint8_t *data,
				  uint32_t size)
{
	// return ipc_ser_put_align(serdes, data, size, 1);
	uint32_t i = 0;

	if (!serdes || !serdes->ptr || !data ||
	    size > (IPC_MAX_SUB_MSG_NUM - serdes->index) * IPC_PAYLOAD_SIZE -
			    serdes->pos)
		return -1;

	if ((uintptr_t)serdes->ptr % 4 != (uintptr_t)data % 4)
		IPC_LOG_DEBUG("%s: serdes align %lu, data align %lu", __func__,
			      (uintptr_t)serdes->ptr % 4, (uintptr_t)data % 4);

	while (i < size) {
		if ((uintptr_t)serdes->ptr % ALIGN_SIZE == 0 &&
		    (uintptr_t)data % ALIGN_SIZE == 0 &&
		    size >= i + ALIGN_SIZE) {
			*(size_t *)serdes->ptr = *(size_t *)data;
			serdes->ptr += ALIGN_SIZE;
			serdes->pos += ALIGN_SIZE;
			data += ALIGN_SIZE;
			i += ALIGN_SIZE;
#ifdef __LP64__
		} else if (isAligned4(serdes->ptr) && isAligned4(data) &&
			   size >= i + 4) {
			*(uint32_t *)serdes->ptr = *(uint32_t *)data;
			serdes->ptr += 4;
			serdes->pos += 4;
			data += 4;
			i += 4;
#endif
		} else if (isAligned2(serdes->ptr) && isAligned2(data) &&
			   size >= i + 2) {
			*(uint16_t *)serdes->ptr = *(uint16_t *)data;
			serdes->ptr += 2;
			serdes->pos += 2;
			data += 2;
			i += 2;
		} else {
			*(serdes->ptr++) = *data++;
			++serdes->pos;
			++i;
		}

		if (serdes->pos == IPC_PAYLOAD_SIZE)
			ipc_serdes_next_msg(serdes);
	}
	return 0;
}

/**
 * @brief Put 8 bits data into the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @param data Pointer to the data to put.
 * @return int32_t Returns 0 on success, -1 on failure.
 */
static inline int32_t ipc_ser_put_8(serdes_t *serdes, const uint8_t *data)
{
	if (!serdes || !serdes->ptr || !data ||
	    sizeof(uint8_t) >
		    (IPC_MAX_SUB_MSG_NUM - serdes->index) * IPC_PAYLOAD_SIZE -
			    serdes->pos)
		return -1;

	*(serdes->ptr++) = *data;
	++serdes->pos;
	if (serdes->pos == IPC_PAYLOAD_SIZE)
		ipc_serdes_next_msg(serdes);

	return 0;
}

/**
 * @brief Put 16 bits data into the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @param data Pointer to the data to put.
 * @return int32_t Returns 0 on success, -1 on failure.
 */
static inline int32_t ipc_ser_put_16(serdes_t *serdes, const uint16_t *data)
{
	uint8_t align = sizeof(uint16_t);
	uint32_t offset = 0;

	if (!serdes || !serdes->ptr || !data ||
	    serdes->index >= IPC_MAX_SUB_MSG_NUM)
		return -1;

	offset = IPC_CEIL(serdes->pos, align) * align;
	if (offset >= IPC_PAYLOAD_SIZE) {
		if (serdes->index + 1 == IPC_MAX_SUB_MSG_NUM)
			return -1;
		ipc_serdes_next_msg(serdes);
	} else {
		serdes->pos = offset;
		serdes->ptr = (uint8_t *)serdes->msg_pool[serdes->index].payload +
		      offset;
	}

	if ((uintptr_t)serdes->ptr % 2 != (uintptr_t)data % 2)
		IPC_LOG_DEBUG("%s: serdes align %lu, data align %lu", __func__,
			      (uintptr_t)serdes->ptr % 2, (uintptr_t)data % 2);
	*(uint16_t *)serdes->ptr = *data;
	serdes->ptr += align;
	serdes->pos += align;
	if (serdes->pos == IPC_PAYLOAD_SIZE)
		ipc_serdes_next_msg(serdes);

	return 0;
}

/**
 * @brief Put 32 bits data into the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @param data Pointer to the data to put.
 * @return int32_t Returns 0 on success, -1 on failure.
 */
static inline int32_t ipc_ser_put_32(serdes_t *serdes, const uint32_t *data)
{
	uint8_t align = sizeof(uint32_t);
	uint32_t offset = 0;

	if (!serdes || !serdes->ptr || !data)
		return -1;

	offset = IPC_CEIL(serdes->pos, align) * align;
	if (offset >= IPC_PAYLOAD_SIZE) {
		if (serdes->index + 1 == IPC_MAX_SUB_MSG_NUM)
			return -1;
		ipc_serdes_next_msg(serdes);
	} else {
		serdes->pos = offset;
		serdes->ptr = (uint8_t *)serdes->msg_pool[serdes->index].payload +
		      offset;
	}

	if ((uintptr_t)serdes->ptr % 4 != (uintptr_t)data % 4)
		IPC_LOG_DEBUG("%s: serdes align %lu, data align %lu", __func__,
			      (uintptr_t)serdes->ptr % 4, (uintptr_t)data % 4);
	*(uint32_t *)serdes->ptr = *data;
	serdes->ptr += align;
	serdes->pos += align;
	if (serdes->pos == IPC_PAYLOAD_SIZE)
		ipc_serdes_next_msg(serdes);

	return 0;
}

/**
 * @brief Put 64 bits data into the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @param data Pointer to the data to put.
 * @return int32_t Returns 0 on success, -1 on failure.
 */
static inline int32_t ipc_ser_put_64(serdes_t *serdes, const uint64_t *data)
{
	uint8_t align = sizeof(uint64_t);
	uint32_t offset = 0;

	if (!serdes || !serdes->ptr || !data)
		return -1;

	offset = IPC_CEIL(serdes->pos, align) * align;
	if (offset >= IPC_PAYLOAD_SIZE) {
		if (serdes->index + 1 == IPC_MAX_SUB_MSG_NUM)
			return -1;
		ipc_serdes_next_msg(serdes);
	} else {
		serdes->pos = offset;
		serdes->ptr = (uint8_t *)serdes->msg_pool[serdes->index].payload +
		      offset;
	}

	if ((uintptr_t)serdes->ptr % 8 != (uintptr_t)data % 8)
		IPC_LOG_DEBUG("%s: serdes align %lu, data align %lu", __func__,
			      (uintptr_t)serdes->ptr % 8, (uintptr_t)data % 8);
	*(uint64_t *)serdes->ptr = *data;
	serdes->ptr += align;
	serdes->pos += align;
	if (serdes->pos == IPC_PAYLOAD_SIZE)
		ipc_serdes_next_msg(serdes);

	return 0;
}

/**
 * @brief Put data into the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @param data Pointer to the data to put.
 * @param size The size of the data.
 * @param align The alignment of the data.
 * @return int32_t Returns 0 on success, -1 on failure.
 */
static inline int32_t ipc_ser_put_align(serdes_t *serdes, const uint8_t *data,
					uint32_t size, uint8_t align)
{
	uint32_t offset = 0;

	if (!serdes || !serdes->ptr || !IPC_CHECK_ALIGN(align))
		return -1;
	// check size and data.
	// return 0 when size is 0.
	if (size == 0)
		return 0;
	if (!data)
		return -1;

	offset = IPC_CEIL(serdes->pos, align) * align;
	if ((IPC_MAX_SUB_MSG_NUM - serdes->index) * IPC_PAYLOAD_SIZE -
		    serdes->pos < size)
		return -1;

	if (offset >= IPC_PAYLOAD_SIZE) {
		if (serdes->index + 1 == IPC_MAX_SUB_MSG_NUM)
			return -1;
		ipc_serdes_next_msg(serdes);
	} else {
		serdes->pos = offset;
		serdes->ptr = (uint8_t *)serdes->msg_pool[serdes->index].payload +
		      offset;
	}

	if ((uintptr_t)serdes->ptr % align != (uintptr_t)data % align)
		IPC_LOG_DEBUG("%s: serdes align %lu, data align %lu", __func__,
			      (uintptr_t)serdes->ptr % ALIGN_SIZE,
			      (uintptr_t)data % ALIGN_SIZE);

    if (serdes->pos % ALIGN_SIZE == 0 && isAligned(data)) {
        while (size >= ALIGN_SIZE) {
			*(size_t *)serdes->ptr = *(size_t *)data;
			serdes->ptr += ALIGN_SIZE;
			serdes->pos += ALIGN_SIZE;
			data += ALIGN_SIZE;
			size -= ALIGN_SIZE;
            if (serdes->pos == IPC_PAYLOAD_SIZE)
                ipc_serdes_next_msg(serdes);
        }
    }
#ifdef __LP64__
    if (isAligned4(serdes->ptr) && isAligned4(data)) {
        while (size >= 4) {
			*(uint32_t *)serdes->ptr = *(uint32_t *)data;
			serdes->ptr += 4;
			serdes->pos += 4;
			data += 4;
			size -= 4;
            if (serdes->pos == IPC_PAYLOAD_SIZE)
                ipc_serdes_next_msg(serdes);
        }
    }
#endif
    if (isAligned2(serdes->ptr) && isAligned2(data)) {
        while (size >= 2) {
			*(uint16_t *)serdes->ptr = *(uint16_t *)data;
			serdes->ptr += 2;
			serdes->pos += 2;
			data += 2;
			size -= 2;
            if (serdes->pos == IPC_PAYLOAD_SIZE)
                ipc_serdes_next_msg(serdes);
        }
    }
    while (size > 0) {
        *(serdes->ptr++) = *data++;
        ++serdes->pos;
        --size;
		if (serdes->pos == IPC_PAYLOAD_SIZE)
			ipc_serdes_next_msg(serdes);
	}
	return 0;
}

#ifndef IPC_NO_STRING
/**
 * @brief Put a string into the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @param data Pointer to the string to put.
 * @return int32_t Returns 0 on success, -1 on failure.
 */
static inline int32_t ipc_ser_put_string(serdes_t *serdes, const char *data)
{
	if (!serdes || !data)
		return -1;

	return ipc_ser_put_align(serdes, (const uint8_t *)data, ipc_strlen(data) + 1, 1);
}
#endif

/**
 * @brief Finish the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @return int32_t Returns 0 on success, -1 on failure.
 */
static inline int32_t ipc_ser_finish(serdes_t *serdes)
{
	if (!serdes)
		return -1;
	if (serdes->index >= IPC_MAX_SUB_MSG_NUM)
		return -1;
	for (uint32_t i = 0; i <= serdes->index; ++i) {
		serdes->msg_pool[i].header = serdes->header;
		serdes->msg_pool[i].header.len = IPC_PAYLOAD_SIZE / 8U;
		serdes->msg_pool[i].header.idx = i;
		serdes->msg_pool[i].header.is_eof = 0U;
	}
	if (serdes->pos == 0) {
		if (serdes->index == 0){
			serdes->msg_pool[serdes->index].header.len = 0;
			serdes->msg_pool[serdes->index].header.is_eof = 1;
		} else {
			serdes->msg_pool[serdes->index - 1].header.is_eof = 1;
		}
	} else {
		serdes->msg_pool[serdes->index].header.len =
			(serdes->pos - 1) / 8U + 1;
		serdes->msg_pool[serdes->index].header.is_eof = 1U;
	}

	return 0;
}

/**
 * @brief Initialize the deserialization of the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @return int32_t Returns 0 on success, -1 on failure.
 */
static inline int32_t ipc_des_init(serdes_t *serdes)
{
	rw_msg_header_t zero = { 0 };

	if (!serdes)
		return -1;
	serdes->index = 0;
	serdes->rcv_index = 0;
	serdes->pos = 0;
	serdes->ptr = (uint8_t *)(serdes->msg_pool[serdes->index].payload);
	serdes->header = zero;
	serdes->msg_pool[0].header = zero;
	return 0;
}

/**
 * @brief Move to the first message in the deserialization of the serdes
 * structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @return int32_t Returns 0 on success, -1 on failure.
 */
static inline int32_t ipc_des_move_first(serdes_t *serdes)
{
	if (!serdes)
		return -1;
	serdes->index = 0;
	serdes->pos = 0;
	serdes->msg_pool[0] = serdes->msg_pool[serdes->rcv_index];
	serdes->rcv_index = 0;
	return 0;
}

/**
 * @brief Get the current message in the deserialization of the serdes
 * structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @return rw_msg_t* Pointer to the current message, or NULL if serdes is
 * NULL.
 */
static inline rw_msg_t *ipc_des_get_current_msg(serdes_t *serdes)
{
	if (!serdes)
		return NULL;

	return &serdes->msg_pool[serdes->rcv_index];
}

/**
 * @brief Validate the current message in the deserialization of the serdes
 * structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @return int32_t Returns 0 on success, -1 on failure.
 */
static inline int32_t ipc_des_validate_msg(serdes_t *serdes)
{
	rw_msg_t *msg = NULL;

	if (!serdes)
		return -1;

	msg = ipc_des_get_current_msg(serdes);
	// check if index not match.
	if (msg->header.idx == 0) {
		serdes->header = msg->header;
		if (serdes->rcv_index != 0)
			(void)ipc_des_move_first(serdes);
	} else {
		if (msg->header.idx != serdes->rcv_index ||
		    msg->header.cid != serdes->header.cid ||
		    msg->header.cmd != serdes->header.cmd ||
		    msg->header.fid != serdes->header.fid ||
		    msg->header.pid != serdes->header.pid ||
		    msg->header.sid != serdes->header.sid ||
		    msg->header.tok != serdes->header.tok ||
		    msg->header.typ != serdes->header.typ) {
			IPC_LOG_WARNING(
				"msg validate fail, cmd %d message idx not match: get %d, expect %d",
				msg->header.cmd, msg->header.idx,
				serdes->header.idx);
			return -2;
		}
	}

	// check if end of frame.
	if (!msg->header.is_eof) {
		if (msg->header.len == IPC_PAYLOAD_SIZE / 8U)
			++serdes->rcv_index;
		return -1;
	}

	serdes->index = 0;
	serdes->pos = 0;
	serdes->ptr = (uint8_t *)serdes->msg_pool[0].payload;

	return 0;
}

/**
 * @brief Get data from the deserialization of the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @param data Pointer to store the data.
 * @param size The size of the data to get.
 * @return int32_t Returns bytes got(equals to size) on success, -1 on
 * failure.
 */
#if !defined(__riscv)
static inline int32_t ipc_des_get(serdes_t *serdes, uint8_t *data,
				  uint32_t size)
{
	uint32_t total_size = 0;
	uint32_t used = 0;
	uint32_t rest = 0;
	uint32_t i = 0;

	if (!serdes || !data || serdes->rcv_index >= IPC_MAX_SUB_MSG_NUM)
		return -1;

	total_size = serdes->rcv_index * IPC_PAYLOAD_SIZE +
		     serdes->msg_pool[serdes->rcv_index].header.len * 8U;
	used = serdes->index * IPC_PAYLOAD_SIZE + serdes->pos;
	rest = total_size - used;
	if (used >= total_size || size > rest)
		return -1;
	serdes->ptr = (uint8_t *)serdes->msg_pool[serdes->index].payload +
		      serdes->pos;

	if ((uintptr_t)serdes->ptr % 4 != (uintptr_t)data % 4)
		IPC_LOG_DEBUG("%s: serdes align %lu, data align %lu", __func__,
			      (uintptr_t)serdes->ptr % 4, (uintptr_t)data % 4);
	while (i < size) {
		if (isAligned(serdes->ptr) && isAligned(data) &&
		    size >= i + ALIGN_SIZE) {
			*(size_t *)data = *(size_t *)serdes->ptr;
			serdes->ptr += ALIGN_SIZE;
			serdes->pos += ALIGN_SIZE;
			data += ALIGN_SIZE;
			i += ALIGN_SIZE;
#ifdef __LP64__
		} else if (isAligned4(serdes->ptr) && isAligned4(data) &&
			   size >= i + 4) {
			*(uint32_t *)data = *(uint32_t *)serdes->ptr;
			serdes->ptr += 4;
			serdes->pos += 4;
			data += 4;
			i += 4;
#endif
		} else if (isAligned2(serdes->ptr) && isAligned2(data) &&
			   size >= i + 2) {
			*(uint32_t *)data = *(uint32_t *)serdes->ptr;
			serdes->ptr += 2;
			serdes->pos += 2;
			data += 2;
			i += 2;
		} else {
			*data++ = *(serdes->ptr++);
			++serdes->pos;
			++i;
		}

		if (serdes->pos == IPC_PAYLOAD_SIZE)
			ipc_serdes_next_msg(serdes);
	}
	return size;
}

/**
 * @brief Get 8 bits data from the deserialization of the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @param data Pointer to store the data.
 * @param size The size of the data to get.
 * @return int32_t Returns bytes got(equals to size) on success, -1 on
 * failure.
 */
static inline int32_t ipc_des_get_8(serdes_t *serdes, uint8_t *data)
{
	uint32_t total_size = 0;
	uint32_t used = 0;
	uint32_t rest = 0;
	uint8_t align = sizeof(uint8_t);

	if (!serdes || !data || serdes->rcv_index >= IPC_MAX_SUB_MSG_NUM)
		return -1;

	total_size = serdes->rcv_index * IPC_PAYLOAD_SIZE +
		     serdes->msg_pool[serdes->rcv_index].header.len * 8U;
	used = serdes->index * IPC_PAYLOAD_SIZE + serdes->pos;
	rest = total_size - used;
	if (used >= total_size || align > rest)
		return -1;

	serdes->ptr = (uint8_t *)serdes->msg_pool[serdes->index].payload +
		      serdes->pos;
	*data = *(serdes->ptr++);
	++serdes->pos;
	if (serdes->pos == IPC_PAYLOAD_SIZE)
		ipc_serdes_next_msg(serdes);

	return align;
}

/**
 * @brief Get 16 bits data from the deserialization of the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @param data Pointer to store the data.
 * @param size The size of the data to get.
 * @return int32_t Returns bytes got(equals to size) on success, -1 on
 * failure.
 */
static inline int32_t ipc_des_get_16(serdes_t *serdes, uint16_t *data)
{
	uint32_t total_size = 0;
	uint32_t used = 0;
	uint32_t rest = 0;
	uint32_t offset = 0;
	uint8_t align = sizeof(uint16_t);

	if (!serdes || !data || serdes->rcv_index >= IPC_MAX_SUB_MSG_NUM)
		return -1;

	offset = IPC_CEIL(serdes->pos, align) * align;
	total_size = serdes->rcv_index * IPC_PAYLOAD_SIZE +
		     serdes->msg_pool[serdes->rcv_index].header.len * 8U;
	used = serdes->index * IPC_PAYLOAD_SIZE + offset;
	rest = total_size - used;
	if (used >= total_size || align > rest)
		return -1;

	serdes->pos = offset;
	if (serdes->pos == IPC_PAYLOAD_SIZE)
		ipc_serdes_next_msg(serdes);

	serdes->ptr = (uint8_t *)serdes->msg_pool[serdes->index].payload +
		      serdes->pos;
	if ((uintptr_t)serdes->ptr % 2 != (uintptr_t)data % 2)
		IPC_LOG_DEBUG("%s: serdes align %lu, data align %lu", __func__,
			      (uintptr_t)serdes->ptr % 2, (uintptr_t)data % 2);
	*data = *(uint16_t *)serdes->ptr;
	serdes->ptr += align;
	serdes->pos += align;
	if (serdes->pos == IPC_PAYLOAD_SIZE)
		ipc_serdes_next_msg(serdes);

	return align;
}

/**
 * @brief Get 32 bits data from the deserialization of the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @param data Pointer to store the data.
 * @param size The size of the data to get.
 * @return int32_t Returns bytes got(equals to size) on success, -1 on
 * failure.
 */
static inline int32_t ipc_des_get_32(serdes_t *serdes, uint32_t *data)
{
	uint32_t total_size = 0;
	uint32_t used = 0;
	uint32_t rest = 0;
	uint32_t offset = 0;
	uint8_t align = sizeof(uint32_t);

	if (!serdes || !data || serdes->rcv_index >= IPC_MAX_SUB_MSG_NUM)
		return -1;

	offset = IPC_CEIL(serdes->pos, align) * align;
	total_size = serdes->rcv_index * IPC_PAYLOAD_SIZE +
		     serdes->msg_pool[serdes->rcv_index].header.len * 8U;
	used = serdes->index * IPC_PAYLOAD_SIZE + offset;
	rest = total_size - used;
	if (used >= total_size || align > rest)
		return -1;

	serdes->pos = offset;
	if (serdes->pos == IPC_PAYLOAD_SIZE)
		ipc_serdes_next_msg(serdes);

	serdes->ptr = (uint8_t *)serdes->msg_pool[serdes->index].payload +
		      serdes->pos;
	if ((uintptr_t)serdes->ptr % 4 != (uintptr_t)data % 4)
		IPC_LOG_DEBUG("%s: serdes align %lu, data align %lu", __func__,
			      (uintptr_t)serdes->ptr % 4, (uintptr_t)data % 4);
	*data = *(uint32_t *)serdes->ptr;
	serdes->ptr += align;
	serdes->pos += align;
	if (serdes->pos == IPC_PAYLOAD_SIZE)
		ipc_serdes_next_msg(serdes);

	return align;
}

/**
 * @brief Get data from the deserialization of the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @param data Pointer to store the data.
 * @param align The alignment of data.
 * @return int32_t Returns bytes got(equals to size) on success, -1 on
 * failure.
 */
static inline int32_t ipc_des_get_align(serdes_t *serdes, uint8_t *data,
					uint32_t size, uint8_t align)
{
	uint32_t total_size = 0;
	uint32_t used = 0;
	uint32_t rest = 0;
	uint32_t offset = 0;
	uint32_t i = 0;

	if (!serdes || !data || serdes->rcv_index >= IPC_MAX_SUB_MSG_NUM ||
	    !IPC_CHECK_ALIGN(align))
		return -1;

	offset = IPC_CEIL(serdes->pos, align) * align;
	total_size = serdes->rcv_index * IPC_PAYLOAD_SIZE +
		     serdes->msg_pool[serdes->rcv_index].header.len * 8U;
	used = serdes->index * IPC_PAYLOAD_SIZE + offset;
	rest = total_size - used;
	if (used >= total_size || size > rest)
		return -1;

	serdes->pos = offset;
	if (serdes->pos == IPC_PAYLOAD_SIZE)
		ipc_serdes_next_msg(serdes);

	serdes->ptr = (uint8_t *)serdes->msg_pool[serdes->index].payload +
		      serdes->pos;
	if ((uintptr_t)serdes->ptr % align != (uintptr_t)data % align)
		IPC_LOG_DEBUG("%s: serdes align %lu, data align %lu", __func__,
			      (uintptr_t)serdes->ptr % align,
			      (uintptr_t)data % align);

	while (i < size) {
		if ((uintptr_t)serdes->ptr % ALIGN_SIZE == 0 &&
		    (uintptr_t)data % ALIGN_SIZE == 0 &&
		    size >= i + ALIGN_SIZE) {
			*(size_t *)data = *(size_t *)serdes->ptr;
			serdes->ptr += ALIGN_SIZE;
			serdes->pos += ALIGN_SIZE;
			data += ALIGN_SIZE;
			i += ALIGN_SIZE;
#ifdef __LP64__
		} else if (isAligned4(serdes->ptr) && isAligned4(data) &&
			   size >= i + 4) {
			*(uint32_t *)data = *(uint32_t *)serdes->ptr;
			serdes->ptr += 4;
			serdes->pos += 4;
			data += 4;
			i += 4;
#endif
		} else if (isAligned2(serdes->ptr) && isAligned2(data) &&
			   size >= i + 2) {
			*(uint32_t *)data = *(uint32_t *)serdes->ptr;
			serdes->ptr += 2;
			serdes->pos += 2;
			data += 2;
			i += 2;
		} else {
			*data++ = *(serdes->ptr++);
			++serdes->pos;
			++i;
		}

		if (serdes->pos == IPC_PAYLOAD_SIZE)
			ipc_serdes_next_msg(serdes);
	}
	return size;
}

/**
 * @brief Get 64 bits data from the deserialization of the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @param data Pointer to store the data.
 * @param size The size of the data to get.
 * @return int32_t Returns bytes got(equals to size) on success, -1 on
 * failure.
 */
static inline int32_t ipc_des_get_64(serdes_t *serdes, uint64_t *data)
{
	uint32_t total_size = 0;
	uint32_t used = 0;
	uint32_t rest = 0;
	uint32_t offset = 0;
	uint8_t align = sizeof(uint64_t);

	if (!serdes || !data || serdes->rcv_index >= IPC_MAX_SUB_MSG_NUM)
		return -1;

	offset = IPC_CEIL(serdes->pos, align) * align;
	total_size = serdes->rcv_index * IPC_PAYLOAD_SIZE +
		     serdes->msg_pool[serdes->rcv_index].header.len * 8U;
	used = serdes->index * IPC_PAYLOAD_SIZE + offset;
	rest = total_size - used;
	if (used >= total_size || align > rest)
		return -1;

	serdes->pos = offset;
	if (serdes->pos == IPC_PAYLOAD_SIZE)
		ipc_serdes_next_msg(serdes);

	serdes->ptr = (uint8_t *)serdes->msg_pool[serdes->index].payload +
		      serdes->pos;
	if ((uintptr_t)serdes->ptr % ALIGN_SIZE != (uintptr_t)data % ALIGN_SIZE)
		IPC_LOG_DEBUG("%s: serdes align %lu, data align %lu", __func__,
			      (uintptr_t)serdes->ptr % ALIGN_SIZE,
			      (uintptr_t)data % ALIGN_SIZE);
	*data = *(uint64_t *)serdes->ptr;
	serdes->ptr += align;
	serdes->pos += align;
	if (serdes->pos == IPC_PAYLOAD_SIZE)
		ipc_serdes_next_msg(serdes);

	return align;
}

/**
 * @brief Get a string from the deserialization of the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @param data Pointer to store the string.
 * @param size The size of the string to get.
 * @return int32_t Returns num of bytes got on success, -1 on failure.
 */
static inline int32_t ipc_des_get_string(serdes_t *serdes, char *data,
					 uint32_t size)
{
	uint32_t total_size = 0;
	uint32_t used = 0;
	uint32_t rest = 0;
	uint32_t old_index = 0;
	uint32_t old_pos = 0;
	uint8_t *old_ptr = NULL;
	uint32_t len = 0;

	if (!serdes)
		return -1;

	total_size = serdes->rcv_index * IPC_PAYLOAD_SIZE +
		     serdes->msg_pool[serdes->rcv_index].header.len * 8U;
	used = serdes->index * IPC_PAYLOAD_SIZE + serdes->pos;
	rest = total_size - used;
	if (used >= total_size)
		return -1;

	serdes->ptr = (uint8_t *)serdes->msg_pool[serdes->index].payload +
		      serdes->pos;
	if ((uintptr_t)serdes->ptr % ALIGN_SIZE != (uintptr_t)data % ALIGN_SIZE)
		IPC_LOG_DEBUG("%s: serdes align %lu, data align %lu", __func__,
			      (uintptr_t)serdes->ptr % ALIGN_SIZE,
			      (uintptr_t)data % ALIGN_SIZE);

	old_index = serdes->index;
	old_pos = serdes->pos;
	old_ptr = serdes->ptr;

	while (len < rest) {
		if ((uintptr_t)serdes->ptr % ALIGN_SIZE == 0 &&
		    !HASZERO(*(const size_t *)serdes->ptr)) {
			len += ALIGN_SIZE;
			if (data && len < size) {
				if (isAligned(data)) {
					*(size_t *)data =
						*(const size_t *)serdes->ptr;
#ifdef __LP64__
				} else if (isAligned4(data)) {
					*(uint32_t *)data =
						*(const uint32_t *)serdes->ptr;
					*(uint32_t *)(data + 4) = *(
						const uint32_t *)(serdes->ptr +
								  4);
#endif
				} else if (isAligned2(data)) {
					*(uint16_t *)data =
						*(const uint16_t *)serdes->ptr;
					*(uint16_t *)(data + 2) = *(
						const uint16_t *)(serdes->ptr +
								  2);
					*(uint16_t *)(data + 4) = *(
						const uint16_t *)(serdes->ptr +
								  4);
					*(uint16_t *)(data + 6) = *(
						const uint16_t *)(serdes->ptr +
								  6);
				} else {
					for (int i = 0; i < ALIGN_SIZE; ++i)
						data[i] = serdes->ptr[i];
				}
				data += ALIGN_SIZE;
			}
			serdes->ptr += ALIGN_SIZE;
			serdes->pos += ALIGN_SIZE;
		} else {
			if (*serdes->ptr == '\0')
				break;
			++len;
			if (data && len < size) {
				*data = *serdes->ptr;
				++data;
			}
			++serdes->ptr;
			++serdes->pos;
		}

		if (serdes->pos == IPC_PAYLOAD_SIZE)
			ipc_serdes_next_msg(serdes);
	}
	// this is for '\0'
	++len;
	++serdes->ptr;
	++serdes->pos;
	if (serdes->pos == IPC_PAYLOAD_SIZE)
		ipc_serdes_next_msg(serdes);
	if (data && len <= rest && len <= size) {
		*data = '\0';
		return len;
	}

	serdes->index = old_index;
	serdes->pos = old_pos;
	serdes->ptr = old_ptr;

	if (len > rest)
		return -3;
	if (data && len > size)
		return -2;

	return len;
}
#endif
static inline int32_t ipc_des_get_all(serdes_t *serdes, uint8_t *data)
{
	uint64_t *d1 = (uint64_t *)data;
	uint64_t *d2 = d1 + 1;
	uint64_t *d3 = d2 + 1;
	uint64_t *d4 = d3 + 1;
	int32_t i = 0;

	for (; i <= serdes->rcv_index; ++i) {
		*d1 = serdes->msg_pool[i].payload[0];
		*d2 = serdes->msg_pool[i].payload[1];
		*d3 = serdes->msg_pool[i].payload[2];
		*d4 = serdes->msg_pool[i].payload[3];
		d1 += 4;
		d2 += 4;
		d3 += 4;
		d4 += 4;
	}
	return serdes->rcv_index * 32 +
	       serdes->msg_pool[serdes->rcv_index].header.len * 8;
}

#ifndef IPC_NO_DEBUG
/**
 * @brief Print the contents of the serdes structure.
 *
 * @param serdes Pointer to the serdes structure.
 * @return int32_t Returns the number of messages printed.
 */
static inline int32_t ipc_serdes_print(serdes_t *serdes)
{
	if (!serdes)
		return -1;
	for (int i = 0; i < IPC_MAX_SUB_MSG_NUM; ++i) {
		for (int j = 0; j < IPC_PAYLOAD_SIZE / 8; ++j) {
			IPC_LOG_INFO("msg %d, payload %d : %0llx", i, j,
				     serdes->msg_pool[i].payload[j]);
		}
	}

	return 0;
}
#endif

#ifdef __cplusplus
}
#endif

#endif
