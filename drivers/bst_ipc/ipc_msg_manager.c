// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/hashtable.h>

#include "ipc_common.h"
#include "ipc_msg_manager.h"

#define IPC_DRIVER_NAME "ipc_msg_manager"
#define MSG_BIT_NUM	10
#define MSG_TOKEN_MAX	65534
#define MSG_SENT_QUEUE_MAX (2<<MSG_BIT_NUM)

/********************* local variables ***************************/
static DECLARE_RWSEM(sent_msg_hash_rwsem);
static atomic_t token_count = ATOMIC_INIT(1);
static DECLARE_HASHTABLE(msg_hash, MSG_BIT_NUM);
static DEFINE_SPINLOCK(token_lock);
static uint32_t sent_queue_count;

int32_t send_msg_init(void)
{
	hash_init(msg_hash);
	sent_queue_count = 0;
	return 0;
}

int32_t send_msg_destroy(void)
{
	return 0;
}

int32_t show_all_msgs(void)
{
	int32_t i = 0;
	struct ipc_drv_msg *obj = NULL;

	down_read(&sent_msg_hash_rwsem);
	for (i = 0; i < HASH_SIZE(msg_hash); ++i) {
		if (!hlist_empty(&msg_hash[i])) {
			IPC_LOG_INFO("bucket[%d]=> ", i);
			hlist_for_each_entry(obj, &msg_hash[i], node) {
				IPC_LOG_INFO("token : %d, ", obj->msg.token);
			}
		}
		IPC_LOG_INFO(
			"-----------------bucket %d end--------------------",
			i);
	}
	up_read(&sent_msg_hash_rwsem);
	return 0;
}

// sent message store in hash map
int32_t send_msg_in(struct ipc_drv_msg *msg)
{
	if (!msg)
		return -1;

	IPC_LOG_INFO("%s, msg token = %d", __func__, msg->msg.token);
	down_write(&sent_msg_hash_rwsem);
	if (sent_queue_count >= MSG_SENT_QUEUE_MAX) {
		IPC_LOG_WARNING("send msg in overrun");
		up_write(&sent_msg_hash_rwsem);
		return -1;
	}
	hash_add(msg_hash, &msg->node, msg->msg.token);
	sent_queue_count++;
	up_write(&sent_msg_hash_rwsem);
#ifdef DUMP_MSG_HASH_TABLE
	show_all_msgs();
#endif
	return 0;
}

int32_t send_msg_out(uint32_t token, struct ipc_drv_msg **msg)
{
	struct ipc_drv_msg *obj = NULL;

	IPC_LOG_INFO("%s, token = %d", __func__, token);
#ifdef DUMP_MSG_HASH_TABLE
	show_all_msgs();
#endif

	if (msg && token > 0) {
		down_write(&sent_msg_hash_rwsem);
		hash_for_each_possible(msg_hash, obj, node, token) {
			if (obj->msg.token == token) {
				*msg = obj;
				hash_del(&obj->node);
				if (sent_queue_count > 0)
					sent_queue_count--;
				up_write(&sent_msg_hash_rwsem);
				return 0;
			}
		}
		up_write(&sent_msg_hash_rwsem);
	}
	if (msg)
		*msg = NULL;

	IPC_LOG_INFO("%s fail", __func__);
	return -1;
}

int32_t send_msg_get(uint32_t token, struct ipc_drv_msg **msg)
{
	struct ipc_drv_msg *obj = NULL;

	IPC_LOG_INFO("%s, token = %d", __func__, token);
#ifdef DUMP_MSG_HASH_TABLE
	show_all_msgs();
#endif

	if (msg && token > 0) {
		down_write(&sent_msg_hash_rwsem);
		// TODO hash_for_each_possible_safe
		hash_for_each_possible(msg_hash, obj, node, token) {
			if (obj->msg.token == token) {
				*msg = obj;
				up_write(&sent_msg_hash_rwsem);
				return 0;
			}
		}
		up_write(&sent_msg_hash_rwsem);
	}
	if (msg)
		*msg = NULL;

	IPC_LOG_INFO("%s fail", __func__);
	return -1;
}

uint32_t ipc_msg_get_an_available_token(void)
{
	unsigned long flags;
	int32_t token;

	spin_lock_irqsave(&token_lock, flags);
	token = atomic_read(&token_count);
	atomic_inc(&token_count);
	if (atomic_read(&token_count) > MSG_TOKEN_MAX)
		atomic_set(&token_count, 1);

	spin_unlock_irqrestore(&token_lock, flags);
	return token;
}

int32_t sent_msg_clear(uint32_t session_id)
{
	int32_t i = 0;

	down_write(&sent_msg_hash_rwsem);
	for (i = 0; i < HASH_SIZE(msg_hash); ++i) {
		if (!hlist_empty(&msg_hash[i])) {
			struct ipc_drv_msg *obj = NULL;

			hlist_for_each_entry(obj, &msg_hash[i], node) {
				if (obj->session_id == session_id)
					hash_del(&obj->node);
			}
		}
	}
	up_write(&sent_msg_hash_rwsem);

	return 0;
}
