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

/**
 * @file ipc_app_client_utils.h
 * @brief Utilities for IPC application layer client.
 *
 * This file contains the common definitions and functions used in the IPC
 * application layer.
 */

#ifndef IPC_APP_LAYER_COMMON_H
#define IPC_APP_LAYER_COMMON_H

#include "bstipc_cfg.h"
#include "ipc_serdes.h"
#include "ipc_lflist_siso.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Structure representing the version information.
 */
struct _ipc_inf_version_t {
	uint8_t major; /**< Major version number. */
	uint8_t minor; /**< Minor version number. */
};
#define ipc_inf_version_t struct _ipc_inf_version_t

/**
 * @brief Structure representing a byte buffer.
 */
struct _byte_buffer_t {
	uint8_t *data; /**< Pointer to the data buffer. */
	uint32_t size; /**< Size of the data buffer. */
};
#define byte_buffer_t struct _byte_buffer_t

/**
 * @brief Structure representing extended information.
 */
struct _ext_info_t {
	uint16_t uuid; /**< UUID. */
	uint16_t res[3]; /**< Reserved fields. */
	uint64_t timestamp; /**< Timestamp. */
	uint64_t res2; /**< Reserved field. */
};
#define ext_info_t struct _ext_info_t

/**
 * @brief Structure representing the deserialization buffer.
 */
struct _des_buf_t {
	int8_t data_buf[IPC_MAX_DATA_SIZE]; /**< Data buffer. */
	int8_t obj_buf[IPC_OBJ_BUF_SIZE]; /**< Object buffer. */
	uint16_t data_buf_offset; /**< Offset in the data buffer. */
	uint16_t unavail_data_size; /**< Unavailable data size. */
	uint16_t obj_buf_offset; /**< Offset in the object buffer. */
	uint16_t unavail_obj_size; /**< Unavailable object size. */
	rw_msg_header_t header; /**< Message header. */
	uint64_t timestamp; /**< Timestamp. */
};
#define des_buf_t struct _des_buf_t

#ifdef IPC_RTE_BAREMETAL
#define DECL_TASK(task)
#define DECL_MUTEX(mtx)
#define DECL_SEM(sem)
#define IPC_MUTEX_INIT(mtx)
#define IPC_MUTEX_LOCK(mtx)
#define IPC_MUTEX_UNLOCK(mtx)
#define IPC_MUTEX_DESTROY(mtx)
#define IPC_SEM_INIT(sem, val)
#define IPC_SEM_RESET(sem)
#define IPC_SEM_WAIT(sem)
#define IPC_SEM_TIMED_WAIT(sem, timeout)
#define IPC_SEM_POST(sem)
#define IPC_SEM_DESTROY(sem)
#define IPC_YIELD
#else
#if defined IPC_RTE_POSIX
#define DECL_TASK(task)	pthread_t task;
#define DECL_MUTEX(mtx)	pthread_mutex_t mtx;
#define DECL_SEM(sem)	sem_t sem;
#define IPC_MUTEX_INIT(mtx)	do { \
	(void)pthread_mutex_init(mtx, NULL); \
} while (0)
#define IPC_MUTEX_LOCK(mtx)	do { \
	(void)pthread_mutex_lock(mtx); \
} while (0)
#define IPC_MUTEX_UNLOCK(mtx)	do { \
	(void)pthread_mutex_unlock(mtx); \
} while (0)
#define IPC_MUTEX_DESTROY(mtx)	do { \
	(void)pthread_mutex_destroy(mtx); \
} while (0)
#define IPC_SEM_INIT(sem, val)	do { \
	(void)sem_init(sem, 0, val); \
} while (0)
#define IPC_SEM_RESET(sem)	do { \
	while (sem_trywait(sem) == 0); \
} while (0)
#define IPC_SEM_WAIT(sem) 	sem_wait(sem)
#define IPC_SEM_TIMED_WAIT(sem, timeout) do { \
	struct timespec ts; \
	timespec_get(&ts, TIME_UTC); \
	ts.tv_sec += (time_t)timeout / 1000; \
	ts.tv_nsec += (long)(timeout % 1000) * 1000000; \
	if (ts.tv_nsec > 1000000000) { \
		++ts.tv_sec; \
		ts.tv_nsec -= 1000000000; \
	} \
	ret = sem_timedwait(sem, &ts); \
	if (ret < 0 && errno == ETIMEDOUT) \
		ret = -ERR_APP_TIMEOUT; \
} while (0)	
#define IPC_SEM_POST(sem) do { \
	(void)sem_post(sem); \
} while (0)
#define IPC_SEM_DESTROY(sem) 	do { \
	(void)sem_destroy(sem); \
} while (0)
#define IPC_YIELD sched_yield()
#elif defined IPC_RTE_KERNEL
#define DECL_TASK(task)	struct task_struct *task;
#define DECL_MUTEX(mtx)	struct mutex mtx;
#define DECL_SEM(sem)	struct semaphore sem;
#define IPC_MUTEX_INIT(mtx)	do { \
	mutex_init(mtx); \
} while (0)
#define IPC_MUTEX_LOCK(mtx)	do { \
	(void)mutex_lock(mtx); \
} while (0)
#define IPC_MUTEX_UNLOCK(mtx)	do { \
	(void)mutex_unlock(mtx); \
} while (0)
#define IPC_MUTEX_DESTROY(mtx)	do { \
	(void)mutex_destroy(mtx); \
} while (0)
#define IPC_SEM_INIT(sem, val)	do { \
	(void)sema_init(sem, val); \
} while (0)
#define IPC_SEM_RESET(sem)	do { \
	((sem)->count) = 0; \
} while (0)
#define IPC_SEM_WAIT(sem) 	down(sem)
#define IPC_SEM_TIMED_WAIT(sem, timeout) do { \
	ret = down_timeout(sem, msecs_to_jiffies(timeout_ms)); \
	if (ret == -ETIME) \
		ret = -ERR_APP_TIMEOUT; \
} while (0)
#define IPC_SEM_POST(sem) 	do { \
	up(sem); \
} while (0)
#define IPC_SEM_DESTROY(sem)
#define IPC_YIELD schedule()
#elif defined IPC_RTE_RTOS
#define DECL_TASK(task) task_handle_t task;
#define DECL_MUTEX(mtx)	mutex_osal_t mtx;
#define DECL_SEM(sem) sem_osal_t sem;
#define IPC_MUTEX_INIT(mtx) do { \
	(void)CreateResource(mtx); \
} while (0)
#define IPC_MUTEX_LOCK(mtx) do { \
	(void)GetResource(mtx); \
} while (0)
#define IPC_MUTEX_UNLOCK(mtx) do { \
	(void)ReleaseResource(mtx); \
} while (0)
#define IPC_MUTEX_DESTROY(mtx) do { \
	(void)DeleteResource(mtx); \
} while (0)
#define IPC_SEM_INIT(sem, val)	do { \
	(void)SemCountCreate(sem, val, 32); \
} while (0)
#define IPC_SEM_RESET(sem)
#define IPC_SEM_WAIT(sem) do { \
	(sem)->wait_ms = 0;		\
	SemCountWait(sem); \
} while (0)
#define IPC_SEM_TIMED_WAIT(sem, timeout) do { \
	(sem)->wait_ms = timeout;	\
	ret = SemCountWait(sem); \
} while (0)
#define IPC_SEM_POST(sem) do { \
	(void)SemCountPost(sem); \
} while (0)
#define IPC_SEM_DESTROY(sem) do { \
	(void)SemCountDelete(sem); \
} while (0)
#endif
#endif

#define COM_DATA_ELE                                                        \
	uint8_t pid;                                                           	\
	uint8_t fid;                                                           	\
	uint8_t sid;                                                            \
	uint8_t handle;                                                         \
	bool initialized;                                                       \
	_Atomic uint8_t token;                                                  \
	uint8_t res[2];                                                         \
	bst_msg_queue_t *queue;													\
	des_buf_t des_buf;                                                      \
	serdes_t serializer;                                                    \
	serdes_t deserializer;                                                  \
	ext_info_t info;                                                        \
	_Atomic bool bRunning;                                                  \
	DECL_TASK(route_task)                                                   \
	DECL_MUTEX(send_mtx)

// #define KERNEL_2_USER(ptr, k_base, u_base) kernel_2_user((ptr), (k_base), (u_base))
#define KERNEL_2_USER(ptr, k_base, u_base) \
(ptr) ? ((uintptr_t)(k_base) == (uintptr_t)(u_base) ? (ptr) \
		: (void*)((uintptr_t)(ptr) - (uintptr_t)(k_base) + (uintptr_t)(u_base))) \
		: NULL;
/**
 * @brief Callback function for availability change.
 *
 * @param avail Availability status.
 */
typedef void (*avail_changed_callback_t)(bool avail, void *ext);

/**
 * Callback function for broadcast sub/unsub method.
 *
 * @param err The error code returned by the server.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 */
typedef void (*broadcast_sub_unsub_callback_t)(int32_t err, void *ext,
					       const ext_info_t *info);

/**
 * @brief Clear the deserialization buffer.
 *
 * This function clears the data and object buffers in the deserialization
 * buffer.
 *
 * @param buf Pointer to the deserialization buffer.
 */
static inline void clear_des_buf(des_buf_t *buf)
{
	if (buf) {
		buf->data_buf_offset = 0;
		buf->obj_buf_offset = 0;
	}
}

/**
 * @brief Allocate memory in the object buffer.
 *
 * This function allocates memory in the object buffer of the deserialization
 * buffer.
 *
 * @param buf Pointer to the deserialization buffer.
 * @param size Size of the memory to allocate.
 * @param align Alignment of the memory to allocate.
 * @return Pointer to the allocated memory, or NULL if allocation fails.
 */
static inline int8_t *alloc_obj(des_buf_t *buf, uint32_t size, uint8_t align)
{
	int8_t *ret = NULL;
	uint32_t offset = 0;

	if (!buf || size == 0)
		return NULL;

	if (align == 0)
		align = 1;
	else if (align > 8)
		align = 8;
	else
		align = IPC_CHECK_ALIGN(align) ? align : IPC_CEIL(align, 2) * 2;
	offset = IPC_CEIL(buf->obj_buf_offset, align) * align;

	if (offset + size > IPC_OBJ_BUF_SIZE - buf->unavail_obj_size ||
	    offset >= IPC_OBJ_BUF_SIZE - buf->unavail_obj_size)
		return NULL;

	ret = &buf->obj_buf[offset];
	buf->obj_buf_offset = offset + size;
	return ret;
}

/**
 * @brief Allocate memory in the data buffer.
 *
 * This function allocates memory in the data buffer of the deserialization
 * buffer.
 *
 * @param buf Pointer to the deserialization buffer.
 * @param size Size of the memory to allocate.
 * @param align Alignment of the memory to allocate.
 * @return Pointer to the allocated memory, or NULL if allocation fails.
 */
static inline int8_t *alloc_data(des_buf_t *buf, uint32_t size, uint8_t align)
{
	int8_t *ret = NULL;
	uint32_t offset = 0;

	if (!buf || size == 0)
		return NULL;

	if (align == 0)
		align = 1;
	else if (align > 8)
		align = 8;
	else
		align = IPC_CHECK_ALIGN(align) ? align : IPC_CEIL(align, 2) * 2;
	offset = IPC_CEIL(buf->data_buf_offset, align) * align;

	if (offset + size > IPC_MAX_DATA_SIZE - buf->unavail_data_size ||
	    offset >= IPC_MAX_DATA_SIZE - buf->unavail_data_size)
		return NULL;
	ret = &buf->data_buf[offset];
	buf->data_buf_offset = offset + size;
	return ret;
}

#ifndef IPC_NO_BYTE_BUFFER
/**
 * @brief Serialize a byte buffer.
 *
 * This function serializes a byte buffer using the given serdes structure.
 *
 * @param ser Pointer to the serdes structure.
 * @param in Byte buffer to serialize.
 * @return 0 on success, -1 on failure.
 */
static inline int32_t serialize_byte_buffer(serdes_t *ser,
					    const byte_buffer_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, &in->size);
	if (ret >= 0 && in->size > 0)
		ret = ipc_ser_put_align(ser, in->data, in->size, 1);

	return ret;
}

/**
 * @brief Deserialize a byte buffer.
 *
 * This function deserializes a byte buffer using the given serdes structure
 * and deserialization buffer.
 *
 * @param buf Pointer to the deserialization buffer.
 * @param out Pointer to the byte buffer to deserialize into.
 * @return 0 on success, -1 on failure.
 */
static inline int32_t deserialize_byte_buffer(des_buf_t *buf,
					      byte_buffer_t *out)
{
	uint32_t *size_ptr = NULL;

	size_ptr =
		(uint32_t *)alloc_data(buf, sizeof(uint32_t), sizeof(uint32_t));
	if (!size_ptr)
		return -1;

	if (*size_ptr == 0) {
		out->size = 0;
		out->data = NULL;
		return 0;
	}

	out->size = *size_ptr;
	out->data = (uint8_t *)alloc_data(buf, *size_ptr, 4);

	return out->data ? 0 : -1;
}
#endif

#ifndef IPC_NO_STRING
/**
 * @brief Serialize a string.
 *
 * This function serializes a string using the given serdes structure.
 *
 * @param ser Pointer to the serdes structure.
 * @param in String to serialize.
 * @return 0 on success, -1 on failure.
 */
static inline int32_t serialize_string(serdes_t *ser, const char *in)
{
	return ipc_ser_put_string(ser, in);
}

/**
 * @brief Deserialize a string.
 *
 * This function deserializes a string using the given serdes structure and
 * deserialization buffer.
 *
 * @param buf Pointer to the deserialization buffer.
 * @param out Pointer to the string to deserialize into.
 * @return 0 on success, -1 on failure.
 */
static inline int32_t deserialize_string(des_buf_t *buf, char **out)
{
	uint32_t len = 0;

	*out = (char *)&buf->data_buf[buf->data_buf_offset];
	len = ipc_strlen(*out) + 1;
	if (buf->data_buf_offset + len > IPC_MAX_DATA_SIZE)
		return -1;

	buf->data_buf_offset += len;
	return len;
}
#endif

static inline int32_t deserialize_8(des_buf_t *buf, uint8_t *out)
{
	uint8_t *ptr = NULL;

	ptr = (uint8_t *)alloc_data(buf, 1, 1);
	if (!ptr)
		return -1;
	*out = *ptr;
	return 0;
}

static inline int32_t deserialize_16(des_buf_t *buf, uint16_t *out)
{
	uint16_t *ptr = NULL;

	ptr = (uint16_t *)alloc_data(buf, 2, 2);
	if (!ptr)
		return -1;
	*out = *ptr;
	return 0;
}

static inline int32_t deserialize_32(des_buf_t *buf, uint32_t *out)
{
	uint32_t *ptr = NULL;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = *ptr;
	return 0;
}

static inline int32_t deserialize_64(des_buf_t *buf, uint64_t *out)
{
	uint64_t *ptr = NULL;

	ptr = (uint64_t *)alloc_data(buf, 8, 8);
	if (!ptr)
		return -1;
	*out = *ptr;
	return 0;
}

#ifdef __cplusplus
}
#endif

#endif
