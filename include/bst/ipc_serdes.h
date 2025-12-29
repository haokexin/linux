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

#ifndef IPC_SERDES_H
#define IPC_SERDES_H
#include "bstipc_cfg.h"
#ifdef __cplusplus
extern "C" {
#endif

// macro definition
#define IPC_CEIL(X, A) (((X) + (A)-1) / (A))
#define IPC_CHECK_ALIGN(x) ((x) > 0 && ((x) & ((x)-1)) == 0)

#define MSGBX_UCHAR_MAX 255U
#define ALIGN_SIZE (sizeof(size_t))
#define HALF_ALIGN ((ALIGN_SIZE) / 2)
#define ONES ((size_t)-1 / MSGBX_UCHAR_MAX)
#define HIGHS (ONES * (MSGBX_UCHAR_MAX / 2 + 1))
#define HASZERO(x) (((x)-ONES) & ~(x)&HIGHS)
#define IPC_NO_DEBUG

#if defined(__riscv)
#define _Atomic
#define IPC_NO_STRING
#define IPC_NO_BYTE_BUFFER
#define IPC_NO_FIRE_AND_FORGET
#define IPC_NO_ATOMIC
#define IPC_SHARED_SERIALIZER
// add your own atomic function macros
#endif

struct _serdes_t {
	rw_msg_header_t header;
	uint8_t *ptr;
	uint32_t index;
	uint32_t pos;
	uint32_t rcv_index;
	uint32_t res;
	uint64_t recv_start_time;
	uint64_t recv_end_time;
	uint64_t recv_get_time;
	rw_msg_t msg_pool[IPC_MAX_SUB_MSG_NUM];
};
#define serdes_t struct _serdes_t

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
#ifndef IPC_RTE_BAREMETAL
	if (s)
		return strlen(s);
	return 0;
#else
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
#endif
}
#endif

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

	if (((uintptr_t)serdes->ptr % 2 == 0) && ((uintptr_t)data % 2 == 0))
		*(uint16_t *)serdes->ptr = *data;
	else
		ipc_memcpy(serdes->ptr, data, align);

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

	if (((uintptr_t)serdes->ptr % 4 == 0) && ((uintptr_t)data % 4 == 0))
		*(uint32_t *)serdes->ptr = *data;
	else
		ipc_memcpy(serdes->ptr, data, align);

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

	if (((uintptr_t)serdes->ptr % 8 == 0) && ((uintptr_t)data % 8 == 0))
		*(uint64_t *)serdes->ptr = *data;
	else
		ipc_memcpy(serdes->ptr, data, align);

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
	uint32_t serdes_index = 0;
	uint32_t serdes_pos = 0;
	uint8_t *ptr = NULL;
	uint32_t head_size = 0;
	uint32_t tail_size = 0;
	uint32_t body_num = 0;

	if (!serdes || !serdes->ptr || !IPC_CHECK_ALIGN(align))
		return -1;
	// check size and data.
	// return 0 when size is 0.
	if (size == 0)
		return 0;
	if (!data)
		return -1;

	serdes_index = serdes->index;
	serdes_pos = serdes->pos;
	offset = IPC_CEIL(serdes_pos, align) * align;
	if ((IPC_MAX_SUB_MSG_NUM - serdes_index) * IPC_PAYLOAD_SIZE -
		    serdes_pos < size)
		return -1;

	if (offset >= IPC_PAYLOAD_SIZE) {
		if (serdes_index + 1 == IPC_MAX_SUB_MSG_NUM)
			return -1;
		ipc_serdes_next_msg(serdes);
	} else {
		serdes->pos = offset;
		serdes->ptr = (uint8_t *)serdes->msg_pool[serdes->index].payload +
		      offset;
	}
	serdes_pos = serdes->pos;
	ptr = serdes->ptr;

	// calculate the head size, tail size and body num.
	// |--head_size--|--body_num * IPC_PAYLOAD_SIZE--|--tail_size--|
	head_size = IPC_PAYLOAD_SIZE - serdes_pos;
	head_size = (head_size < size) ? head_size : size;
	tail_size = (size - head_size) % IPC_PAYLOAD_SIZE;
	body_num = (size - head_size) / IPC_PAYLOAD_SIZE;

	// copy head data.
	(void)ipc_memcpy(ptr, data, head_size);

	// copy body data.
	ptr = (uint8_t *)serdes->msg_pool[serdes_index + 1].payload;
	data += head_size;
	for (uint32_t i = 0; i < body_num; ++i) {
		(void)ipc_memcpy(ptr, data, IPC_PAYLOAD_SIZE);
		ptr += sizeof(rw_msg_t);
		data += IPC_PAYLOAD_SIZE;
	}

	// copy tail data.
	if (tail_size > 0)
		(void)ipc_memcpy(ptr, data, tail_size);

	// update serdes->pos and serdes->index and serdes->ptr.
	serdes->index += (serdes_pos + size) / IPC_PAYLOAD_SIZE;
	serdes->pos = (serdes_pos + size) % IPC_PAYLOAD_SIZE;
	if (serdes->index == IPC_MAX_SUB_MSG_NUM && serdes->pos == 0) {
		serdes->pos = IPC_PAYLOAD_SIZE;
		serdes->index = IPC_MAX_SUB_MSG_NUM - 1;
	}
	serdes->ptr = (uint8_t *)serdes->msg_pool[serdes->index].payload + serdes->pos;
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
	rw_msg_t *msg = NULL;
	rw_msg_header_t serdes_header = { 0 };
	uint32_t serdes_index = 0;
	if (!serdes)
		return -1;
	serdes_index = serdes->index;
	if (serdes_index >= IPC_MAX_SUB_MSG_NUM)
		return -1;
	msg = serdes->msg_pool;
	serdes_header = serdes->header;
	serdes_header.is_eof = 0U;
	serdes_header.len = IPC_PAYLOAD_SIZE / 8U;
	for (uint32_t i = 0; i <= serdes_index; ++i, ++msg) {
		serdes_header.idx = i;
		msg->header = serdes_header;
	}
	--msg;
	if (serdes->pos == 0) {
		if (serdes_index == 0){
			msg->header.len = 0;
			msg->header.is_eof = 1;
		} else {
			serdes->msg_pool[serdes_index - 1].header.is_eof = 1;
		}
	} else {
		msg->header.len = (serdes->pos - 1) / 8U + 1;
		msg->header.is_eof = 1;
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

static inline int32_t ipc_des_get_all(serdes_t *serdes, uint8_t *data)
{
	int32_t i = 0;
	uint8_t *src = (uint8_t *)serdes->msg_pool[0].payload;
	for (; i <= serdes->rcv_index; ++i) {
		ipc_memcpy(data, src, IPC_PAYLOAD_SIZE);
		data += IPC_PAYLOAD_SIZE;
		src += sizeof(rw_msg_t);
	}

	return serdes->rcv_index * IPC_PAYLOAD_SIZE +
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
			IPC_LOG_INFO("msg %d, payload %d : %0lx", i, j,
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
