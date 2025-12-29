// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/rwsem.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <bst/ipc_interface.h>
#include "ipc_common.h"
#include "ipc_msg_manager.h"
#include "ipc_nodemanager.h"
#include "ipc_regs.h"
#include "ipc_mailbox_controller.h"
#include "ipc_session.h"

#define IPC_DRIVER_NAME			"ipc_session"
#define EACH_SESSION_RECV_MSG_FIFO_SIZE 2048

/********************* local variables ***************************/
static struct ipc_session *session_map[SESSION_NUM]; // index of this array is
						     // session id
static uint64_t bitmap = 1; // session_map occupation status
static DEFINE_SPINLOCK(bitmap_lock);

// note: session id start from num 1
static int32_t request_a_session_id(void)
{
	int32_t ret;
	unsigned long flags;

	spin_lock_irqsave(&bitmap_lock, flags);
	ret = find_bit_zero(bitmap, 64, 0);
	if (ret >= 0)
		bitmap |= (1ULL << ret);
	spin_unlock_irqrestore(&bitmap_lock, flags);
	return ret;
}

IPC_SESSION_STATUS get_session_status(uint32_t session_id)
{
	struct ipc_session *session;

	if (!ipc_session_valid(session_id)) {
		pr_info("%s,session id %d is invalid", __func__,session_id);
		return SESSION_STATE_NULL;
	}
	session = session_map[session_id];

	return session->status;
}

int32_t set_session_status(uint32_t session_id, IPC_SESSION_STATUS status)
{
	struct ipc_session *session;

	if (!ipc_session_valid(session_id)) {
		IPC_LOG_INFO("session id %d is invalid", session_id);
		return -1;
	}

	session = session_map[session_id];
	session->status = status;
	return 0;
}

// register session function
int32_t ipc_register_session(enum ipc_core_e src, enum ipc_core_e dst)
{
	struct ipc_session *session;
	struct kfifo recv_msg_fifo;
	int32_t valid_id;
	struct ipc_client_info *client_info = NULL;
	int ret;
	unsigned long flags;

	IPC_LOG_INFO("%s, src = %d, dst = %d", __func__, src, dst);
	valid_id = request_a_session_id();
	IPC_LOG_INFO("valid_id = %d", valid_id);
	if (valid_id < 0 || valid_id >= SESSION_NUM) {
		IPC_LOG_ERR(
			"all session ids are consumed, please close one first");
		return -1;
	}

	// get client via dst
	ret = ipc_get_node_of_coreid(dst, &client_info);
	if (ret < 0) {
		IPC_LOG_ERR("ipc get client by dst(%d) failed", dst);
		return IPC_INIT_ERR_INVALID_PARAM;
	}

	// alloc session mem
	session = devm_kzalloc(&g_ipc_platform_dev->dev, sizeof(*session),
			       GFP_KERNEL);
	if (!session) {
		IPC_LOG_ERR("session devm_kzalloc failed");
		// clear bitmap
		spin_lock_irqsave(&bitmap_lock, flags);
		bitmap &= ~(1ull << valid_id);
		spin_unlock_irqrestore(&bitmap_lock, flags);
		return -ENOMEM;
	}

	// alloc message fifo mem
	ret = kfifo_alloc(&recv_msg_fifo,
			  sizeof(struct ipc_drv_msg) *
				  EACH_SESSION_RECV_MSG_FIFO_SIZE,
			  GFP_KERNEL);
	session->id = valid_id;
	session->status = SESSION_INIT;
	session->recv_msg_fifo = recv_msg_fifo;
	session->src = src;
	session->dest = dst;
	session->waiting_reply_msg_token = -1;
	IPC_LOG_INFO("current->pid = %d", current->pid);
	session->pid_num = current->pid;
	session->cl_info = client_info;

	mutex_init(&session->session_mutex);

	init_completion(&session->tx_complete);
	init_completion(&session->rx_complete);

	session_map[session->id] = session;
	return valid_id;
}

bool ipc_session_valid(uint32_t id)
{
	if (id >= SESSION_NUM || id <= 0)
		return false;

	if (session_map[id] == NULL)
		return false;
	else
		return true;
}

struct ipc_session *ipc_get_session_by_coreid(enum ipc_core_e coreid)
{
	uint32_t cnt;

	for (cnt = 0; cnt < SESSION_NUM; cnt++) {
		if (session_map[cnt] != NULL) {
			if (session_map[cnt]->dest == coreid)
				return session_map[cnt];
		}
	}

	return NULL;
}

struct ipc_session *get_session_by_id(uint32_t id)
{
	IPC_LOG_INFO(" enter!, id = %d", id);
	if (id >= SESSION_NUM || id <= 0) {
		IPC_LOG_ERR("session_id %d is invalid", id);
		return NULL;
	}

	return session_map[id];
}

struct ipc_session *ipc_session_by_pid(pid_t pid)
{
	int32_t cnt;

	for (cnt = 0; cnt < SESSION_NUM; cnt++) {
		if (session_map[cnt] != NULL) {
			if (session_map[cnt]->pid_num == pid)
				return session_map[cnt];
		}
	}

	return NULL;
}

int32_t ipc_session_msg_in(uint32_t session_id, struct ipc_drv_msg msg)
{
	int32_t ret = -1;
	struct ipc_session *session;

	IPC_LOG_INFO("session msg in!, session_id = %d", session_id);
	if (!ipc_session_valid(session_id)) {
		IPC_LOG_INFO("session id %d is invalid", session_id);
		return -1;
	}

	session = session_map[session_id];

	if (session->status == SESSION_DESTROY) {
		IPC_LOG_WARNING("session is destroy, msg in fail");
		return -1;
	}

	if (!kfifo_is_full(&session->recv_msg_fifo)) {
		ret = kfifo_in(&session->recv_msg_fifo, &msg,
			       sizeof(struct ipc_drv_msg));
	} else {
		IPC_LOG_WARNING("fifo is overrun");
		return -1;
	}
	return ret;
}

int32_t ipc_session_msg_out(uint32_t session_id, struct ipc_drv_msg *recv_msg)
{
	struct ipc_session *session;

	IPC_LOG_INFO("%s, session_id = %d", __func__, session_id);

	if (!ipc_session_valid(session_id)) {
		IPC_LOG_INFO("session id %d is invalid", session_id);
		return -1;
	}

	session = session_map[session_id];

	if (session->status == SESSION_DESTROY) {
		IPC_LOG_INFO("session %d is destroy, out msg fail", session_id);
		return -1;
	}

	if (kfifo_out(&session->recv_msg_fifo, recv_msg,sizeof(struct ipc_drv_msg)) == 0) {
		diagnose_info[session->id].recv_err.queue_empty += 1;
		IPC_LOG_WARNING("get msg out from session %d fail",
				session->id);
		return -1;
	}

	return 0;
}

int32_t ipc_session_destroy_by_id(uint32_t session_id)
{
	struct ipc_session *session;
	unsigned long flags;

	IPC_LOG_INFO(" destroy session id %d", session_id);
	if (!ipc_session_valid(session_id)) {
		IPC_LOG_WARNING("session id %d is invalid", session_id);
		return -1;
	}

	session = session_map[session_id];
	// session status update
	session->status = SESSION_DESTROY;

	// clear completion to release wait
	complete(&session->tx_complete);
	complete(&session->rx_complete);


	kfifo_free(&session->recv_msg_fifo);

	// qnx bugfix merge #30964
	// close session need clear sent_msg_queue to avoid token invert overlap issue
	sent_msg_clear(session_id);

	// kasan bugfix:
	// root cause: when some session call ipc_recv() to wait for message at
	// wait_for_completion(), simultaneously, this session id call ipc_close()
	// to release session.  At this point, function will complete all completion
	// to release thread blocking, then free session memory and set map to
	// NULL. In ipc_recv(), After releasing completion wait, driver would
	// check session status and re-access session memory. It would lead to
	// memory access issue.
	// FIX: using status label signals session status change, and add some
	// delay before memory free and session map change to NULL. It would ensure
	// session destroy status to whoever use session pointer.

	msleep(100);
	devm_kfree(&g_ipc_platform_dev->dev, session);

	session_map[session_id] = NULL;
	IPC_LOG_INFO("free session success!");

	// clear bitmap
	spin_lock_irqsave(&bitmap_lock, flags);
	bitmap &= ~(1ull << session_id);
	spin_unlock_irqrestore(&bitmap_lock, flags);

	return 0;
}

int32_t ipc_session_destroy_by_pid(pid_t id)
{
	int ret = 0;
	struct ipc_session *session = NULL;

	IPC_LOG_INFO(" destroy session by pid %d", id);

	session = ipc_session_by_pid(id);
	if (session == NULL) {
		IPC_LOG_WARNING("get session fail by pid %d", id);
		return -1;
	}

	ret = ipc_session_destroy_by_id(session->id);

	if (ret < 0) {
		IPC_LOG_WARNING("free session %d fail", session->id);
		return -1;
	}
	return 0;
}

bool ipc_get_dst_by_session_id(uint32_t session_id, uint32_t *src, uint32_t *dst)
{
	struct ipc_session *session;

	if (!ipc_session_valid(session_id)) {
		IPC_LOG_INFO("session id %d is invalid", session_id);
		return false;
	}

	session = session_map[session_id];
	*src = session->src;
	*dst = session->dest;

	return true;
}

