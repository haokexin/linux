// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <bst/ipc_interface.h>
#include <linux/device.h>
#include <linux/of.h>
#include <asm/memory.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/rwsem.h>

#include "ipc_communication_manager.h"
#include "ipc_session.h"
#include "ipc_msg_manager.h"
#include "ipc_common.h"
#include "ipc_mailbox_controller.h"
#include "ipc_nodemanager.h"

// macro
#define IPC_DRIVER_NAME	 "ipc_interface"
#define SEND_MAX_TIMEOUT 200

/********************* global variables ***************************/
struct diag_info diagnose_info[SESSION_NUM]; // array's index is session
					     // id

/********************* local variables ***************************/
static DEFINE_SPINLOCK(subscription_lock);
static DEFINE_SPINLOCK(register_lock);
static DEFINE_SPINLOCK(count_lock);

int32_t ipc_init(enum ipc_core_e dest_core_id, enum ipc_core_e src_core_id,
		 void *arg)
{
	int32_t session_id = -1;
	struct diag_info _info;

	if (!get_ipc_init_status()) {
		IPC_LOG_ERR("init %d request ipc is not ready", dest_core_id);
		return IPC_INIT_ERR;
	}

	IPC_LOG_INFO("enter, ---dst = %d, src = %d", dest_core_id, src_core_id);
	if (dest_core_id < IPC_CORE_ARM0 || dest_core_id >= IPC_CORE_MAX) {
		IPC_LOG_WARNING("dest_core_id %d is invalid", dest_core_id);
		return IPC_INIT_ERR_INVALID_PARAM;
	}

	// NOTE:currently, src definition is not used.
#ifdef IPC_SRC_DEFINITION_SUPPORT
	if (src_core_id < IPC_CORE_ARM0 || src_core_id > IPC_CORE_DB1) {
		IPC_LOG_WARNING("src_core_id is invalid");
		return IPC_INIT_ERR_INVALID_PARAM;
	}
#endif

	// NOTE:currently, same src and dst do not support.
#ifndef ENABLE_SRC_DST_SAME
	if (dest_core_id == src_core_id) {
		IPC_LOG_WARNING("dest_core_id = src_core_id = %d",
				src_core_id);
		return IPC_INIT_ERR_INVALID_PARAM;
	}
#endif

	session_id = ipc_register_session(src_core_id, dest_core_id);
	if (session_id < 0) {
		IPC_LOG_WARNING("ipc_register_session return : %d", session_id);
		return IPC_INIT_ERR_SESSION;
	}

	// create ipc session diagnostic information
	_info.session_id = session_id;
	_info.src = src_core_id;
	_info.dst = dest_core_id;
	_info.num_of_send_msg = 0;
	_info.num_of_recv_msg = 0;
	_info.send_err.ipc_no_ready = 0;
	_info.send_err.session_invalid = 0;
	_info.send_err.no_ACK = 0;
	_info.send_err.queue_full = 0;
	_info.recv_err.ipc_no_ready = 0;
	_info.recv_err.session_invalid = 0;
	_info.recv_err.queue_empty = 0;
	_info.recv_err.timeout = 0;
	diagnose_info[_info.session_id] = _info;

	IPC_LOG_INFO("%s success, session = %d", __func__, session_id);
	return session_id;
}
EXPORT_SYMBOL(ipc_init);



int32_t ipc_send(int32_t session_id, ipc_msg *msg, int32_t timeout, ...)
{
	int32_t ret = 0;
	uint64_t wait = 0;
	struct ipc_drv_msg *ipc_drv_msg = NULL;
	bool sent_msg_store_flag = false;
	uint32_t src, dst;
	bool reply_flag = false;
	int package_flag = 0;
	int package_size = 0;
	va_list ap_ptr;
	struct ipc_fill_register_msg fill_msg;
	struct ipc_session *p_session;
	unsigned long flags;


	if (!get_ipc_init_status()) {
		IPC_LOG_ERR("session %d send ipc is not ready", session_id);
		return IPC_SEND_ERR;
	}

	// input session id check
	if (!ipc_get_dst_by_session_id(session_id, &src, &dst)) {
		IPC_LOG_WARNING("session %d get dst fail", session_id);
		return IPC_SEND_ERR_INVALID_PARAM;
	}

	// message token management, only actively send message need add token
	if (msg->type == IPC_MSG_TYPE_METHOD ||
	    msg->type == IPC_MSG_TYPE_SIGNAL) {
		msg->token = ipc_msg_get_an_available_token();
		IPC_LOG_INFO("token = %d", msg->token);
	}

	// input parameter parsing for compatibility
	// NOTE: reply flag default value is true for compatibility
	va_start(ap_ptr, timeout);
	// this version only support:
	//  1. a flag means whether this call need reply
	reply_flag = va_arg(ap_ptr, int) == 1 ? true : false;
	va_end(ap_ptr);
	IPC_LOG_INFO(
		"session %d send msg reply flag = %d, package_flag = %d, package_size = %d",
		session_id, reply_flag, package_flag, package_size);

	// fill message in ipc
	fill_msg.long_param = msg->data;
	fill_msg.short_param = msg->token;
	fill_msg.cmd = msg->cmd;
	fill_msg.ack = package_size; // special use for package msg
	fill_msg.wakeup = package_flag; // special use for package msg
	fill_msg.type = msg->type;

#ifdef MSG_SIZE_EXTENSION
	fill_msg.long_data = msg->long_data;
#endif

	// sending message pre-processing:
	// NOTE: only method message which need to reply would store in sent
	// message map
	IPC_LOG_INFO("send_msg->type = %d", fill_msg.type);
	if (msg->type == IPC_MSG_TYPE_METHOD && reply_flag) {
		ipc_drv_msg = devm_kzalloc(&g_ipc_platform_dev->dev,
					   sizeof(*ipc_drv_msg), GFP_KERNEL);
		if (!ipc_drv_msg) {
			IPC_LOG_WARNING(
				"session %d sent msg store alloc mem error",
				session_id);
			return IPC_SEND_ERR;
		}
		ipc_drv_msg->msg.type = msg->type;
		ipc_drv_msg->msg.cmd = msg->cmd;
		ipc_drv_msg->msg.data = msg->data;
		ipc_drv_msg->msg.token = msg->token;
		ipc_drv_msg->session_id = session_id;
		ipc_drv_msg->dst = dst;
		ipc_drv_msg->src = src;
#ifdef MSG_SIZE_EXTENSION
		ipc_drv_msg->msg.long_data = msg->long_data;
#endif
		// put sent msg in hashtable for dispatching recv msg to session
		// later
		send_msg_in(ipc_drv_msg);
		sent_msg_store_flag = true;
	}

	// ipc message sending
//	set_session_status(session_id, SESSION_SENDING);
	
	p_session = get_session_by_id(session_id);
	if (IS_ERR_OR_NULL(p_session)) {

		
		if (sent_msg_store_flag){

			struct ipc_drv_msg *ipc_drv_msg_ptr;

			ret = send_msg_out(fill_msg.short_param, &ipc_drv_msg_ptr);
			if (ret < 0) {
				
				if(ipc_drv_msg)
					devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg);

				IPC_LOG_WARNING("token %d find sent msg failed",fill_msg.short_param);
				return ret;
			}
			if (IS_ERR_OR_NULL(ipc_drv_msg_ptr)) {

				if(ipc_drv_msg)
					devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg);

				IPC_LOG_WARNING("sender %d sent a message with invalid token %d",src, fill_msg.short_param);
				return ret;
			}

			devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg_ptr);
		}


		IPC_LOG_WARNING("session %d is invalid", session_id);
		return IPC_SEND_ERR_INVALID_PARAM;
	}

	mutex_lock(&p_session->session_mutex);
	ret = ipc_drv_send(src, dst, session_id,
			   (void *)&fill_msg, sizeof(fill_msg));
	if (ret) {
		IPC_LOG_WARNING("session %d ipc_drv_send fail, ret = %d",
				session_id, ret);

		if (sent_msg_store_flag){

			struct ipc_drv_msg *ipc_drv_msg_ptr;

			ret = send_msg_out(fill_msg.short_param, &ipc_drv_msg_ptr);
			if (ret < 0) {
				
				if(ipc_drv_msg)
					devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg);

				IPC_LOG_WARNING("token %d find sent msg failed",fill_msg.short_param);
				mutex_unlock(&p_session->session_mutex);
				return ret;
			}
			if (IS_ERR_OR_NULL(ipc_drv_msg_ptr)) {

				if(ipc_drv_msg)
					devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg);

				IPC_LOG_WARNING("sender %d sent a message with invalid token %d",src, fill_msg.short_param);
				mutex_unlock(&p_session->session_mutex);
				return ret;
			}

			devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg_ptr);
		}

		mutex_unlock(&p_session->session_mutex);
		return ret;
	}

	// wait sending result
	wait = msecs_to_jiffies(SEND_MAX_TIMEOUT);
	set_session_status(session_id, SESSION_WAIT_SEND);
	
	ret = wait_for_completion_timeout(&p_session->tx_complete, wait);
	if (ret == 0) { // timeout
		diagnose_info[session_id].send_err.no_ACK += 1;


		IPC_LOG_WARNING(
			"session %d no ack with %d  %d with in 200 milliseconds",
			session_id, dst,fill_msg.long_param);

		if(get_session_status(session_id) == SESSION_WAIT_SEND) {
			if (sent_msg_store_flag){
				struct ipc_drv_msg *ipc_drv_msg_ptr;

				ret = send_msg_out(fill_msg.short_param, &ipc_drv_msg_ptr);
				if (ret < 0) {
					if(ipc_drv_msg)
						devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg);

					IPC_LOG_WARNING("token %d find sent msg failed",fill_msg.short_param);
					mutex_unlock(&p_session->session_mutex);
					return IPC_SEND_ERR_TIMEOUT;
				}
				if (IS_ERR_OR_NULL(ipc_drv_msg_ptr)) {

					if(ipc_drv_msg)
						devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg);

					IPC_LOG_WARNING("sender %d sent a message with invalid token %d",src, fill_msg.short_param);
					mutex_unlock(&p_session->session_mutex);
					return IPC_SEND_ERR_TIMEOUT;
				}
				
				devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg_ptr);
			}

			IPC_LOG_ERR("session status:%d\n",get_session_status(session_id));

			mutex_unlock(&p_session->session_mutex);
			return IPC_SEND_ERR_TIMEOUT;
		}
	}

	// session status check
	if (get_session_status(session_id) == SESSION_DESTROY ||
	    get_session_status(session_id) == SESSION_STATE_NULL) {
		diagnose_info[session_id].send_err.session_invalid += 1;
		IPC_LOG_INFO("session %d is destroy", session_id);
		if (sent_msg_store_flag){
			struct ipc_drv_msg *ipc_drv_msg_ptr;

			ret = send_msg_out(fill_msg.short_param, &ipc_drv_msg_ptr);
			if (ret < 0) {
				if(ipc_drv_msg)
					devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg);

				IPC_LOG_WARNING("token %d find sent msg failed",fill_msg.short_param);
				mutex_unlock(&p_session->session_mutex);
				return IPC_RECV_ERR_INVALID_PARAM;
			}
			if (IS_ERR_OR_NULL(ipc_drv_msg_ptr)) {
				if(ipc_drv_msg)
					devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg);

				IPC_LOG_WARNING("sender %d sent a message with invalid token %d",src, fill_msg.short_param);
				mutex_unlock(&p_session->session_mutex);
				return IPC_RECV_ERR_INVALID_PARAM;
			}
			
			devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg_ptr);
		}
		mutex_unlock(&p_session->session_mutex);
		return IPC_RECV_ERR_INVALID_PARAM;
	}

	mutex_unlock(&p_session->session_mutex);


	// session diagnostic information update
	spin_lock_irqsave(&count_lock, flags);
	diagnose_info[session_id].num_of_send_msg += 1;
	spin_unlock_irqrestore(&count_lock, flags);
	return 0;
}

EXPORT_SYMBOL(ipc_send);

int32_t ipc_send_sync(int32_t session_id, ipc_msg *msg)
{
	int ret;
	struct ipc_client_info *client;
	struct ipc_fill_register_msg send_msg;
	struct ipc_session *session;
	uint32_t src;
	uint32_t dst;
	struct ipc_drv_msg *ipc_drv_msg = NULL;
	//int try_count = 5;

	session = get_session_by_id(session_id);
	if (!session) {
		IPC_LOG_WARNING("%s: Failed to get session for %d", __func__,
				session_id);
		return -EINVAL;
	}


	src = session->src;
	dst = session->dest;
	memset(&send_msg, 0, sizeof(send_msg));
	send_msg.type = msg->type;
	send_msg.long_param = msg->data;
	send_msg.cmd = msg->cmd;

	if (msg->type == IPC_MSG_TYPE_METHOD) {

		msg->token = ipc_msg_get_an_available_token();
		send_msg.short_param = msg->token;
		
		ipc_drv_msg = devm_kzalloc(&g_ipc_platform_dev->dev,
						sizeof(*ipc_drv_msg), GFP_KERNEL);
		if (!ipc_drv_msg) {
			IPC_LOG_WARNING(
				"session %d sent msg store alloc mem error",
				session_id);
			return IPC_SEND_ERR;
		}

		ipc_drv_msg->msg.type = msg->type;
		ipc_drv_msg->msg.cmd = msg->cmd;
		ipc_drv_msg->msg.data = msg->data;
		ipc_drv_msg->msg.token = msg->token;
		ipc_drv_msg->session_id = session_id;
		ipc_drv_msg->dst = dst;
		ipc_drv_msg->src = src;

		send_msg_in(ipc_drv_msg);
	}

	ret = ipc_node_parse(dst, &client);
	if (ret < 0) {
		pr_debug("%s: can not parse client for %d", __func__, dst);

		if(msg->type == IPC_MSG_TYPE_METHOD){

			struct ipc_drv_msg *ipc_drv_msg_ptr;

			ret = send_msg_out(send_msg.short_param, &ipc_drv_msg_ptr);
			if (ret < 0) {
				if(ipc_drv_msg)
					devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg);
				IPC_LOG_WARNING("token %d find sent msg failed",send_msg.short_param);
				return -ENOENT;
			}
			if (IS_ERR_OR_NULL(ipc_drv_msg_ptr)) {
				if(ipc_drv_msg)
					devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg);

				IPC_LOG_WARNING("sender %d sent a message with invalid token %d",src, send_msg.short_param);
				return -ENOENT;
			}
			
			devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg_ptr);
		}

		return -ENOENT;
	}

	ret = ipc_send_data(client, src, &send_msg);
	if(ret == -1){
		if(msg->type == IPC_MSG_TYPE_METHOD){
			
				struct ipc_drv_msg *ipc_drv_msg_ptr;

				IPC_LOG_WARNING("token %d find sent msg failed",send_msg.short_param);
				
				ret = send_msg_out(send_msg.short_param, &ipc_drv_msg_ptr);
				if (ret < 0) {
					if(ipc_drv_msg)
						devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg);

					IPC_LOG_WARNING("token %d find sent msg failed",send_msg.short_param);
					return -ENOENT;
				}
				if (IS_ERR_OR_NULL(ipc_drv_msg_ptr)) {
					if(ipc_drv_msg)
						devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg);

					IPC_LOG_WARNING("sender %d sent a message with invalid token %d",src, send_msg.short_param);
					return -ENOENT;
				}
				
				devm_kfree(&g_ipc_platform_dev->dev, ipc_drv_msg_ptr);
			}
			
		return -ENOENT;
	}


	return 0;
}
EXPORT_SYMBOL(ipc_send_sync);

int32_t ipc_recv(int32_t session_id, ipc_msg *msg, int32_t timeout)
{
	int8_t ret = 0;
	uint64_t wait;
	struct ipc_drv_msg ipc_drv_msg;
	struct completion *completion;
	struct ipc_session *p_sess;

	if (!get_ipc_init_status()) {
		// diagnose_info[session_id].recv_err.ipc_no_ready += 1;
		IPC_LOG_ERR("session %d recv ipc is not ready", session_id);
		return IPC_RECV_ERR;
	}

	p_sess = get_session_by_id(session_id);
	if (IS_ERR_OR_NULL(p_sess)) {
		IPC_LOG_WARNING("session %d is invalid", session_id);
		return IPC_RECV_ERR_INVALID_PARAM;
	}

	IPC_LOG_INFO("%s, session id= %d, dest = %d", __func__, session_id,
		     p_sess->dest);


	// use rx_complete means all message receive from resource core
	completion = &p_sess->rx_complete;
	// update session status
	//set_session_status(session_id, SESSION_WAIT_RECEIVE);

	if (timeout == -1) {
		wait_for_completion_interruptible(completion);
	} else {
		wait = msecs_to_jiffies(timeout);
		ret = wait_for_completion_interruptible_timeout(completion, wait);
		if (ret == 0) { // timeout
			diagnose_info[session_id].recv_err.timeout += 1;
			IPC_LOG_INFO("session %d no received type = %d with in %d ms",
					session_id, msg->type, timeout);


			return IPC_RECV_ERR_TIMEOUT;
		}
	}

	// session status check
	if (get_session_status(session_id) == SESSION_DESTROY ||
	    get_session_status(session_id) == SESSION_STATE_NULL) {
		// diagnose_info[session_id].recv_err.session_invalid += 1;
		IPC_LOG_INFO("session %d is destroy", session_id);
		return IPC_RECV_ERR_INVALID_PARAM;
	}

	// get message from session queue
	ret = ipc_session_msg_out(session_id, &ipc_drv_msg);
	if (ret < 0) {
		IPC_LOG_INFO("session %d get msg fail", session_id);
		return IPC_RECV_ERR_GET_MSG_FAIL;
	}
	msg->type = ipc_drv_msg.msg.type;
	msg->cmd = ipc_drv_msg.msg.cmd;
	msg->token = ipc_drv_msg.msg.token;
	msg->data = ipc_drv_msg.msg.data;
#ifdef MSG_SIZE_EXTENSION
	msg->long_data = ipc_drv_msg.long_data;
#endif
	msg->timestamp = ipc_drv_msg.msg.timestamp;

	IPC_LOG_INFO(
		"ipc recv msg from session %d type = %d, cmd = %d, token = %d, data = %x",
		session_id, msg->type, msg->cmd, msg->token, msg->data);
	// session diagnostic information update
	diagnose_info[session_id].num_of_recv_msg += 1;
	return 0;
}
EXPORT_SYMBOL(ipc_recv);

int32_t ipc_close(int32_t session_id)
{
	uint32_t dst, src;
	unsigned long flags;
	uint8_t cnt;
	int8_t ret;

	if (!get_ipc_init_status()) {
		IPC_LOG_ERR("session %d close ipc is not ready", session_id);
		return IPC_CLOSE_ERR;
	}

	// clear subscription and register map
	if (!ipc_get_dst_by_session_id(session_id, &src, &dst)) {
		IPC_LOG_WARNING("session %d get dst fail", session_id);
		return IPC_CLOSE_ERR;
	}

	spin_lock_irqsave(&subscription_lock, flags);
	for (cnt = 0; cnt < MSG_CMD_MAX; cnt++)
		signal_map[dst][cnt] &= ~(1ull << session_id);

	spin_unlock_irqrestore(&subscription_lock, flags);

	spin_lock_irqsave(&register_lock, flags);
	if (dst < IPC_CORE_MAX) {
		for (cnt = 0; cnt < MSG_CMD_MAX; cnt++) {
			if (register_list[dst][cnt] == session_id)
				register_list[dst][cnt] = 0; // map clear to 0
		}
	}
	spin_unlock_irqrestore(&register_lock, flags);

	// session destroy and recycle all resource in this session
	ret = ipc_session_destroy_by_id(session_id);

	//clean diag info
	diagnose_info[session_id].session_id = -1;
	diagnose_info[session_id].src = 0;
	diagnose_info[session_id].dst = 0;
	diagnose_info[session_id].num_of_send_msg = -1;
	diagnose_info[session_id].num_of_recv_msg = -1;
	diagnose_info[session_id].send_err.ipc_no_ready = -1;
	diagnose_info[session_id].send_err.session_invalid = -1;
	diagnose_info[session_id].send_err.no_ACK = -1;
	diagnose_info[session_id].send_err.queue_full = -1;
	diagnose_info[session_id].recv_err.ipc_no_ready = -1;
	diagnose_info[session_id].recv_err.session_invalid = -1;
	diagnose_info[session_id].recv_err.queue_empty = -1;
	diagnose_info[session_id].recv_err.timeout = -1;
	return ret;
}
EXPORT_SYMBOL(ipc_close);

int32_t ipc_signal_subscribe(int32_t session_id, uint32_t cmd)
{
	unsigned long flags;
	uint32_t dst, src;
	uint8_t subscribe_id[64];
	uint8_t count;
	int cnt = 0;

	// input parameter check
	if (!get_ipc_init_status()) {
		IPC_LOG_ERR("session %d subscribe ipc is not ready",
			    session_id);
		return IPC_INIT_ERR;
	}

	// session_id validity check
	if (!ipc_session_valid(session_id)) {
		IPC_LOG_WARNING("session_id %d is invalid ", session_id);
		return IPC_INIT_ERR;
	}

	// cmd validity check
	if (cmd > MSG_CMD_MAX) {
		IPC_LOG_WARNING("cmd %d is invalid", cmd);
		return IPC_SUBSCRIBE_ERR;
	}

	if (!ipc_get_dst_by_session_id(session_id, &src, &dst)) {
		IPC_LOG_WARNING("session %d get dst fail", session_id);
		return IPC_SUBSCRIBE_ERR;
	}

	// signal subscription map update
	spin_lock_irqsave(&subscription_lock, flags);

	signal_map[dst][cmd] |= (1ull << session_id);

	spin_unlock_irqrestore(&subscription_lock, flags);

	count = get_all_1bit(signal_map[dst][cmd], subscribe_id);
	// dump signal subscription map
	IPC_LOG_INFO(
		"session in subscription map with cmd = %d and dst = %d is : ",
		cmd, dst);

	for (cnt = 0; cnt < count; cnt++)
		IPC_LOG_INFO("session_id = %d", subscribe_id[cnt]);

	return 0;
}
EXPORT_SYMBOL(ipc_signal_subscribe);

int32_t ipc_method_register(int32_t session_id, uint32_t cmd)
{
	uint32_t dst, src;
	unsigned long flags;

	// input parameter check
	if (!get_ipc_init_status()) {
		IPC_LOG_ERR("session %d request ipc is not ready", session_id);
		return IPC_METHOD_REGISTER_ERR;
	}

	// session_id validity check
	if (!ipc_session_valid(session_id)) {
		IPC_LOG_WARNING("session_id %d is invalid ", session_id);
		return IPC_METHOD_REGISTER_INVALID_PARAM;
	}

	// cmd validity check
	if (cmd > MSG_CMD_MAX) {
		IPC_LOG_WARNING("session %d cmd %d is invalid", session_id,
				cmd);
		return IPC_METHOD_REGISTER_INVALID_PARAM;
	}

	// uniqueness check
	if (!ipc_get_dst_by_session_id(session_id, &src, &dst)) {
		IPC_LOG_WARNING("session %d get dst fail", session_id);
		return IPC_METHOD_REGISTER_INVALID_PARAM;
	}

	// this restriction is temporary, dst number validity checking
	if (src > IPC_CORE_DB1) {
		IPC_LOG_WARNING("session %d dst is invalid", session_id);
		return IPC_METHOD_REGISTER_INVALID_PARAM;
	}

	if (register_list[dst][cmd] > 0) {
		IPC_LOG_WARNING(
			"dst %d's cmd %d is registered by session %d, do not support update",
			dst, cmd, register_list[dst][cmd]);
		return IPC_METHOD_REGISTER_REPETITION;
	}

	// method register map update
	spin_lock_irqsave(&register_lock, flags);

	register_list[dst][cmd] = session_id;

	spin_unlock_irqrestore(&register_lock, flags);

	// dump method register map
	IPC_LOG_INFO(
		"session %d in register map with cmd = %d and dst = %d is : ",
		session_id, cmd, dst);
	return 0;
}
EXPORT_SYMBOL(ipc_method_register);

