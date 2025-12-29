// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*
 * BSTN: Linux device driver for Black Sesame Technologies Neural Network IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    bstn_msg_manager.c
 * @brief   This is the source file of the message manager part of BSTN driver.
 *          It contains function definitions of message handling as well as
 *          initialization and cleanup of the message manager.
 * @notes   An exchange is the entire process of ARM sending the request and
 *          getting its response from DSP side. This idea is proposed to support
 *          matching requests and responses for multithreaded nonblocking
 *          request sending and receiving.
 */

#include "bstn.h"

//#define _MSG_DUMP_

#define _CMD_FW_FIRST_MSG_ 1
#define _CMD_FUNC_SAFETY_ERR_ 32
#define _ICP_RECV_TIMEOUT_MS_ 50

#define RT_CMD_TOP_NET_BIST_ERR_MASK 2
#define RT_CMD_LITE_NET_BIST_ERR_MASK 1
#define TOP_NET_BIST_ERROR_CODE 0xF62304
#define LITE_NET_BIST_ERROR_CODE 0xF62404

//from msgbox for bstn debugging
extern uint8_t bstn_print_log_flag;
extern uint64_t bstn_max_recv_time;
extern uint64_t bstn_max_send_time;

void disp_req_callback(const uint32_t rep, const net_ErrorEnum_t err, void *ext,
		       const ext_info_t *info)
{
	struct bstn_rsp_msg *msg = NULL;
	struct bstn_device *pbstn = ext;
	struct bstn_exchange_node *exchange;
	uint64_t cost_tm = ktime_get_raw();
	cost_tm = (cost_tm - info->timestamp) / 1000;
	if (cost_tm > bstn_max_recv_time) {
		bstn_max_recv_time = cost_tm;
	}

	if ((bstn_print_log_flag) && (cost_tm > 1000)) {
		BSTN_STAGE_PRINTK("msgbox recv cost %llu us, max %llu us", 
			cost_tm, bstn_max_recv_time);
	}

	//BSTN_STAGE_PRINTK("%s,%d: rep:0x%08x, msg:%p, pbstn:%p, exchange:%p", __FILE__, __LINE__, rep, msg, pbstn, exchange);

	msg = bus_to_kern(pbstn, rep, pbstn->fw_manager.assigned_mem);
	if (msg->nid >= BSTN_EXCHANGE_NODE_NUM) {
		BSTN_STAGE_PRINTK("invalid response:0x%x, nid:%d", rep,
				  msg->nid);
		return;
	}
	//BSTN_STAGE_PRINTK("%s,%d: msg:%p", __FILE__, __LINE__, msg);
	//BSTN_STAGE_PRINTK("nid:%d", msg->nid);

	exchange = &pbstn->msg_manager.exchange_nodes[msg->nid];
	//BSTN_STAGE_PRINTK("%s,%d: exchange:%p", __FILE__, __LINE__, msg);
	exchange->rsp_buf = msg;

	//copy the response message
	//BSTN_STAGE_PRINTK("exchange node nid:%d received", exchange->nid);
	msg->rsp = exchange->rsp_buf->rsp;

	complete(&exchange->complete);
}

/*
 * @func    _rsp_recv for ipc
 * @brief   This functions is a wrapper function which reives a response
 *          message. It is used to abstract the lower level message receiving
 *          mechanism.
 * @params  pbstn - the pointer to the BSTN device
 * @return  0 - success
 *          error code - failure
 */
static struct bstn_rsp_msg *_rsp_recv(struct bstn_device *pbstn)
{
	struct bstn_rsp_msg *msg;
	dsp_ptr pmsg;

	BSTN_TRACE_PRINTK("enter");
	//loop till receive a real response message
	do {
		ipc_msg ipcmsg;

		if (ipc_recv(pbstn->msg_manager.ipc_session_id, &ipcmsg, -1) <
		    0) {
			pmsg = 0;
		} else {
			// TODO, Refactor me
			if (unlikely(IPC_MSG_CMD_BOOTDONE == ipcmsg.cmd)) {
				BSTN_TRACE_PRINTK("ipc recv bootdone");
				complete(&pbstn->msg_manager.ipc_boot_complete);
				msg = NULL;
				continue;
			} else {
				BSTN_TRACE_PRINTK("ipc recv data 0x%x",
						  ipcmsg.data);
				pmsg = ipcmsg.data;
			}
		}

		//if nothing is received until current timeout, return nothing
		if (pmsg == 0) {
			msg = NULL;
			BSTN_STAGE_PRINTK("receive nothing");
			break;
		}
		//suppress simple debugging messages
		else if ((pmsg < phys_to_bus(pbstn->mem_manager.rmem_base)) ||
			 (pmsg >= (phys_to_bus(pbstn->mem_manager.rmem_base) +
				   pbstn->mem_manager.rmem_size))) {
			BSTN_STAGE_PRINTK("debugging response: 0x%x", pmsg);
			msg = NULL;
		} else {
			//response buffers should be allocated within the buffer assigned to DSP firmware
			msg = bus_to_kern(pbstn, pmsg,
					  pbstn->fw_manager.assigned_mem);
			//suppress debugging prints from DSP
			if (msg->nid == (uint16_t)-1) {
				char *rt_print_buf = bus_to_kern(
					pbstn, msg->rsp.status,
					pbstn->fw_manager.assigned_mem);

				BSTN_STAGE_PRINTK("fw dbg msg: %s",
						  rt_print_buf);

				ipcmsg.data = -1;
				ipc_send(pbstn->msg_manager.ipc_session_id,
					 &ipcmsg, 0);

				msg = NULL;
			} else if (msg->nid >= BSTN_EXCHANGE_NODE_NUM) {
				BSTN_STAGE_PRINTK(
					"invalid response:0x%x, nid:%d", pmsg,
					msg->nid);
				msg = NULL;
			} else {
				BSTN_TRACE_PRINTK("nid:%d", msg->nid);
			}
		}
	} while (msg == NULL);
	BSTN_TRACE_PRINTK("exit");
	return msg;
}

/*
 * @func    bstn_msg_receiver for ipc
 * @brief   This is the receiver worker thread function which keeps receiving
 *          and posting DSP messages.
 * @params  args - the pointer to arguments which is a pointer to the BSTN
 *          device here
 * @return  void
 */
static int bstn_msg_receiver(void *args)
{
	struct bstn_device *pbstn = args;
	struct bstn_exchange_node *exchange;
	struct sched_param param;

	param.sched_priority = MAX_RT_PRIO - 1;
	sched_setscheduler(current, SCHED_FIFO, &param);
	do {
		struct bstn_rsp_msg *msg = _rsp_recv(pbstn);
		if (msg == NULL) {
			while (!kthread_should_stop()) {
				msleep(1);
			}
			return 0;
		} else {
			exchange = &pbstn->msg_manager.exchange_nodes[msg->nid];
			exchange->rsp_buf = msg;
			complete(&exchange->complete);
		}
	} while (1);
	return 0;
}

/*
 * @func    _req_send
 * @brief   This functions is a wrapper function which sends the request
 *          message. It is used to abstract the lower level message sending
 *          mechanism.
 * @params  pbstn - the pointer the BSTN device
 *          req - the pointer to the request to be send
 * @return  0 - success
 *          error code - failure
 */
static int _req_send(struct bstn_device *pbstn, struct bstn_req_msg *req)
{
	dsp_ptr pmsg = kern_to_bus(pbstn, req, pbstn->msg_manager.req_bufs);
	int ret;

	BSTN_TRACE_PRINTK("enter");
	//BSTN_STAGE_PRINTK("%s,%d: pmsg:0x%08x", __FILE__, __LINE__, pmsg);

	if (bstn_msg_interface == BSTN_MSG_INTERFACE_IPC) {
		ipc_msg msg = { .data = pmsg, .type = IPC_MSG_TYPE_SIGNAL };
		ret = ipc_send(pbstn->msg_manager.ipc_session_id, &msg, 0);
	} else {
		uint64_t start_tm = ktime_get_raw(), cost_tm = 0;
		ret = pbstn->msg_manager.msgbx_client->net_client.disp_req_async(
			pmsg, disp_req_callback, pbstn, NULL);
		cost_tm = (ktime_get_raw() - start_tm) / 1000;
		if (cost_tm > bstn_max_send_time) {
			bstn_max_send_time = cost_tm;
		}
		if ((bstn_print_log_flag) && (cost_tm > 1000)) {
			BSTN_STAGE_PRINTK("msgbox send cost %llu us, max %llu us", 
				cost_tm, bstn_max_send_time);
		}
	}

	BSTN_TRACE_PRINTK("exit");
	return ret;
}

/*
 * @func    bstn_sw_bister
 * @brief   This is the sw_bist worker thread function which keeps posting
 * sw_bist to DSP.
 * @params  args - the pointer to arguments which is a pointer to the BSTN
 *          device here
 * @return  void
 */
static int bstn_sw_bister(void *args)
{
	struct bstn_device *pbstn = args;
	struct bsnn_msg_exchange msg = { 0 };
	int ret = 0;
	msg.req.opcode = RT_CMD_SW_BIST;
	msg.rsp.status = -1;
	msg.req.pdata = 0;

	/* wait bstn ready */
	// fprintf(stderr, "bstn_sw_bister is running.\n");
	BSTN_TRACE_PRINTK("bstn_sw_bister");

	while (1) {
		if (kthread_should_stop()) {
			return 0;
		}
		mutex_lock(&pbstn->mutex);
		if (pbstn->msg_manager.bstn_r5msg_enable) {
			// exchange msg with r5
			ret = bstn_msg_exchange(pbstn, &msg);
			if (ret <= 0) {
				BSTN_DEV_ERR(&pbstn->pdev->dev,
					     "sw_bist msg error: %d", ret);
			}
		}
		mutex_unlock(&pbstn->mutex);

		msleep(BSTN_SW_BIST_PERIOD_MS);
	}

	return 0;
}

static void on_dst_changed(bool flag, void *ext)
{
	struct bstn_device *pbstn = ext;
	if (flag) {
		BSTN_TRACE_PRINTK("dst is online\n");
	} else {
		BSTN_TRACE_PRINTK("dst is offline\n");
	}
	complete(&pbstn->msg_manager.ipc_boot_complete);
}

/*
 * @func    bstn_msg_is_bootdone
 * @brief   This function tries to check whether it has received bootdone
 *          message from DSP
 * @params  pbstn - the pointer to the BSTN device
 * @return  0 - if bootdone message not received
 *          1 - if bootdone message received
 */
bool bstn_msg_is_bootdone(struct bstn_device *pbstn)
{
#if 1
	int time;
	time = wait_for_completion_timeout(
		&pbstn->msg_manager.ipc_boot_complete,
		msecs_to_jiffies(BSTN_RSP_TIMEOUT_MS));
	if (!time) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "wait for bootdone message timeout");
		return false;
	}
#else
	msleep(1000);
#endif
	return true;
}

/*
 * @func    bstn_msg_exchange
 * @brief   This function tries to take a free exchange node from the free list
            to send the request message. Then it waits for the response and copy
            it from the exchange node, so this is a blocking call.
 * @params  pbstn - the pointer to the BSTN device
 *          msg - the request message to be posted as well as the space where
            the response should be saved to.
 * @return  the time left after waiting for the response - success;
 *          0 - timeout;
 *          error code - other failure;
 */
int bstn_msg_exchange(struct bstn_device *pbstn, struct bsnn_msg_exchange *msg)
{
	static uint16_t sIdx = 0;

	int ret = 0;
	struct bstn_exchange_node *exchange = NULL;
	void __iomem *reg_base = NULL;
	uint32_t reg_val = 0;

	BSTN_TRACE_PRINTK("enter");

	// wait for available exchange node
	while (1) {
		mutex_lock(&pbstn->msg_manager.flist_lock);
		// prioritize software BIST request queue over regular request queue
		if (msg->req.opcode == RT_CMD_SW_BIST) {
			exchange = pbstn->msg_manager.sw_bist_flist;
			if (exchange != NULL) {
				Q_REMOVE_HEAD(&pbstn->msg_manager.sw_bist_flist,
					      link);
				mutex_unlock(&pbstn->msg_manager.flist_lock);
				break;
			}
		} else {
			exchange = pbstn->msg_manager.flist;
			if (exchange != NULL) {
				Q_REMOVE_HEAD(&pbstn->msg_manager.flist, link);
				mutex_unlock(&pbstn->msg_manager.flist_lock);
				break;
			}
		}
		mutex_unlock(&pbstn->msg_manager.flist_lock);
		schedule();
	}

	//set up this exchange node
	exchange->req_buf->req = msg->req;
	exchange->req_buf->target_net = 0;

	exchange->req_buf->req.flag = sIdx;
#ifdef CONFIG_BST_C1200_IVI
	exchange->req_buf->req.flag |= (RT_HOST_IVI << 24);
#elif defined(CONFIG_BST_C1200_DB)
	exchange->req_buf->req.flag |= (RT_HOST_DB << 24);
#elif defined(CONFIG_BST_C1200_ADAS)
	exchange->req_buf->req.flag |= (RT_HOST_ADAS << 24);
#else
	exchange->req_buf->req.flag |= (RT_HOST_ERR << 24);
	BSTN_DEV_ERR(&pbstn->pdev->dev, "host id invalid");
#endif

	/* BSTN_STAGE_PRINTK("%s,%d:%08x, %08x, %08x", __FILE__, __LINE__,
			exchange->req_buf->req.flag,
			exchange->req_buf->req.opcode,
			exchange->req_buf->req.pdata);*/

	//send the request message
	_req_send(pbstn, exchange->req_buf);
	BSTN_TRACE_PRINTK("exchange node nid:%d sent", exchange->nid);

	//wait for the response
	ret = wait_for_completion_timeout(
		&exchange->complete, msecs_to_jiffies(BSTN_RSP_TIMEOUT_MS));
	if (ret > 0) {
		//copy the response message
		BSTN_TRACE_PRINTK("exchange node nid:%d received",
				  exchange->nid);
		msg->rsp = exchange->rsp_buf->rsp;
		if (msg->rsp.status != 0) {
			BSTN_DEV_ERR(&pbstn->pdev->dev, "message rsp error %d",
				     msg->rsp.status);
			reg_base = ioremap(0x50020000, 0x100); // NET TOP
			reg_val = readl_relaxed(reg_base + 0x0);
			if (reg_val & 0x30000000) {
				reg_val = readl_relaxed(reg_base + 0x10);
				if (reg_val != 0) {
					BSTN_DEV_ERR(&pbstn->pdev->dev,
						     "bstn fatal register 0x%x",
						     reg_val);
					if ((reg_val & (~0x5c)) != 0) {
						BSTN_DEV_ERR(
							&pbstn->pdev->dev,
							"Hardware ran into a problem and needs to reboot");
					}
				}
				iounmap(reg_base);

				reg_base = ioremap(0x50000000, 0x100);
				reg_val = readl_relaxed(reg_base + 0x28);
				if (reg_val) {
					BSTN_DEV_ERR(
						&pbstn->pdev->dev,
						"bstn net_busy register 0x%x",
						reg_val);
				}

				reg_val = readl_relaxed(reg_base + 0xc);
				if ((reg_val & 0x1) == 0) {
					BSTN_DEV_ERR(
						&pbstn->pdev->dev,
						"net_engine_status register 0x%x",
						reg_val);
				}

				reg_val = readl_relaxed(reg_base + 0x4c);
				if ((reg_val & 0x1) == 0) {
					BSTN_DEV_ERR(
						&pbstn->pdev->dev,
						"net_header_status register 0x%x",
						reg_val);
				}
			}

			iounmap(reg_base);
		}

	} else {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "frame idx:%04x, exchange node nid:%d failed",
				sIdx,
				exchange->nid);
		/* while(1); */

		reg_base = ioremap(0x50020000, 0x100); // NET TOP
		reg_val = readl_relaxed(reg_base + 0x0);
		if (reg_val & 0x30000000) {
			reg_val = readl_relaxed(reg_base + 0x10);
			if (reg_val != 0) {
				BSTN_DEV_ERR(&pbstn->pdev->dev,
					     "bstn fatal register 0x%x",
					     reg_val);
				if ((reg_val & (~0x5c)) != 0) {
					BSTN_DEV_ERR(
						&pbstn->pdev->dev,
						"Hardware ran into a problem and needs to reboot");
				}
			}
			iounmap(reg_base);

			reg_base = ioremap(0x50000000, 0x100);
			reg_val = readl_relaxed(reg_base + 0x28);
			if (reg_val) {
				BSTN_DEV_ERR(&pbstn->pdev->dev,
					     "bstn net_busy register 0x%x",
					     reg_val);
			}

			reg_val = readl_relaxed(reg_base + 0xc);
			if ((reg_val & 0x1) == 0) {
				BSTN_DEV_ERR(&pbstn->pdev->dev,
					     "net_engine_status register 0x%x",
					     reg_val);
			}

			reg_val = readl_relaxed(reg_base + 0x4c);
			if ((reg_val & 0x1) == 0) {
				BSTN_DEV_ERR(&pbstn->pdev->dev,
					     "net_header_status register 0x%x",
					     reg_val);
			}
		}

		iounmap(reg_base);

		bstn_soft_reset = 1;
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "wait_for_completion_interruptible_timeout");
		return -ETIMEDOUT;
	}
	sIdx++;

	//add the exchange into the free list
	mutex_lock(&pbstn->msg_manager.flist_lock);
	if (msg->req.opcode == RT_CMD_SW_BIST) {
		Q_INSERT_TAIL(&pbstn->msg_manager.sw_bist_flist, exchange,
			      link);
	} else {
		Q_INSERT_TAIL(&pbstn->msg_manager.flist, exchange, link);
	}
	mutex_unlock(&pbstn->msg_manager.flist_lock);

	/* if sw_bist failled and sw_bist msg timeout, send msg to r5 */
	if ((exchange->req_buf->req.opcode == RT_CMD_SW_BIST) &&
	    (0 != msg->rsp.status)) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "sw_bist error %d, send msg to r5!",
			     msg->rsp.status);
		//...
	}

	BSTN_TRACE_PRINTK("exit");
	return ret;
}

/*
 * @func    bstn_msg_manager_init
 * @brief   This function initializes the exchange control block and all exchange
 *          nodes.
 * @params  pbstn - the pointer to the BSTN device
 * @return  0 - success
 *          error code - failure
 */
int bstn_msg_manager_init(struct bstn_device *pbstn)
{
	int ret = 0;
	int i;
	int msgbox_endid;

	// init the message manager itself
	pbstn->msg_manager.msg_sw_bister_task = NULL;
	pbstn->msg_manager.bist_thread_start = 0;
	pbstn->msg_manager.bstn_r5msg_enable = 0;
	pbstn->msg_manager.flist = NULL;
	pbstn->msg_manager.sw_bist_flist = NULL;
	pbstn->msg_manager.ipc_session_id = -1;

	// msgbox_endid for msgbox cpu client to firmware dsp server
	ret = device_property_read_u32(&pbstn->pdev->dev, "msgbox-endid",
				       &msgbox_endid);
	if (ret == -EINVAL || ret == -ENODATA) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "no msgbox-endid property, ret %d", ret);
		return ret;
	} else if (ret < 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "invalid msgbox-endid, ret %d",
			     ret);
		return ret;
	}
	pbstn->msg_manager.msgbx_data.com_data.pid = msgbox_endid;
	BSTN_STAGE_PRINTK("msgbox-endid: 0x%x", msgbox_endid);

	init_completion(&pbstn->msg_manager.ipc_boot_complete);

	// msgbox_endid for msgbox cpu client to firmware dsp server
	ret = device_property_read_u32(&pbstn->pdev->dev, "msgbox-endid",
				       &msgbox_endid);
	if (ret == -EINVAL || ret == -ENODATA) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "no msgbox-endid property, ret %d", ret);
		return ret;
	} else if (ret < 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "invalid msgbox-endid, ret %d",
			     ret);
		return ret;
	}
	pbstn->msg_manager.msgbx_data.com_data.pid = msgbox_endid;
	BSTN_STAGE_PRINTK("msgbox-endid: 0x%x", msgbox_endid);

	if (bstn_msg_interface == BSTN_MSG_INTERFACE_IPC) {
		pbstn->msg_manager.ipc_session_id = ipc_init(
			IPC_CORE_NET, IPC_CORE_ARM2, &pbstn->pdev->dev);
		if (pbstn->msg_manager.ipc_session_id < 0) {
			BSTN_DEV_ERR(&pbstn->pdev->dev,
				     "ipc_init failed, ret %d",
				     pbstn->msg_manager.ipc_session_id);
			return pbstn->msg_manager.ipc_session_id;
		}
		BSTN_STAGE_PRINTK("ipc_init OK");
	} else {
		ipc_inf_version_t version;

		pbstn->msg_manager.msgbx_client =
			bstn_client_init(&pbstn->msg_manager.msgbx_data);
		if (pbstn->msg_manager.msgbx_client == NULL) {
			BSTN_DEV_ERR(&pbstn->pdev->dev,
				     "bstn_client_init failed");
			return -EFAULT;
		}
		version = pbstn->msg_manager.msgbx_client->net_client.version();
		ret = pbstn->msg_manager.msgbx_client->net_client
			      .register_avail_changed(on_dst_changed, pbstn);
		BSTN_STAGE_PRINTK(
			"msgbx_client init OK.version: major %d, minor %d.",
			version.major, version.minor);
	}

	//alignment TBD
	pbstn->msg_manager.req_bufs = pbstn->mem_manager.ops->alloc(
		pbstn, BSTN_EXCHANGE_NODE_NUM * sizeof(struct bstn_req_msg), 0,
		0);
	if (pbstn->msg_manager.req_bufs == NULL) {
		if (bstn_msg_interface == BSTN_MSG_INTERFACE_IPC) {
			ipc_close(pbstn->msg_manager.ipc_session_id);
		} else {
			bstn_client_destroy();
		}
		BSTN_DEV_ERR(
			&pbstn->pdev->dev,
			"bstn_msg_manager_init: cannot allocate exchange message buffers");
		return -ENOMEM;
	}
	BSTN_STAGE_PRINTK("req_bufs @ dma:0x%llx",
			  pbstn->msg_manager.req_bufs->dma_addr);

	//intialize exchange nodes
	for (i = 0; i < BSTN_EXCHANGE_NODE_NUM; i++) {
		pbstn->msg_manager.exchange_nodes[i].nid = i;
		init_completion(&pbstn->msg_manager.exchange_nodes[i].complete);
		if (i == BSTN_EXCHANGE_NODE_NUM - 1) {
			Q_INSERT_TAIL(&pbstn->msg_manager.sw_bist_flist,
				      &pbstn->msg_manager.exchange_nodes[i],
				      link);
		} else {
			Q_INSERT_TAIL(&pbstn->msg_manager.flist,
				      &pbstn->msg_manager.exchange_nodes[i],
				      link);
		}
		pbstn->msg_manager.exchange_nodes[i].req_buf =
			(struct bstn_req_msg *)
				pbstn->msg_manager.req_bufs->kern_addr +
			i;
		pbstn->msg_manager.exchange_nodes[i].req_buf->nid = i;
	}

	mutex_init(&pbstn->msg_manager.flist_lock);

	// start the msgbx receiver thread
	if (bstn_msg_interface == BSTN_MSG_INTERFACE_IPC) {
		pbstn->msg_manager.msg_receiver_task = kthread_run(
			bstn_msg_receiver, pbstn, "msg_receiver_task");
		if (IS_ERR(pbstn->msg_manager.msg_receiver_task)) {
			ipc_close(pbstn->msg_manager.ipc_session_id);
			BSTN_DEV_ERR(
				&pbstn->pdev->dev,
				"bstn_msg_manager_init: kthread_create receiver failed!");
			return PTR_ERR(pbstn->msg_manager.msg_receiver_task);
		}
		BSTN_STAGE_PRINTK("kthread_create ipc receiver task OK");
	} else {
		ret = pbstn->msg_manager.msgbx_client->start();
		if (ret < 0) {
			bstn_client_destroy();
			BSTN_DEV_ERR(&pbstn->pdev->dev,
				     "msgbx_client->start failed ret %d!", ret);
			return ret;
		}
		BSTN_STAGE_PRINTK("msgbx_client start OK");
	}

	return 0;
}

/*
 * @func    bstn_msg_start_sw_bist_thread
 * @brief   This function start the sw bist thrad.
 * @params  void
 * @return  0 - success
 *          error code - failure
 */
int bstn_msg_start_sw_bist_thread(struct bstn_device *pbstn)
{
	// create the sw_bist thread
	BSTN_STAGE_PRINTK("create sw_bist thread ...");
	if (pbstn->msg_manager.msg_sw_bister_task == NULL) {
		pbstn->msg_manager.msg_sw_bister_task = kthread_run(
			bstn_sw_bister, pbstn, "msg_sw_bister_task");
		if (IS_ERR(pbstn->msg_manager.msg_sw_bister_task)) {
			BSTN_DEV_ERR(
				&pbstn->pdev->dev,
				"bstn_msg_start_sw_bist_thread: kthread_create failed!");
			return PTR_ERR(pbstn->msg_manager.msg_sw_bister_task);
		}
		BSTN_STAGE_PRINTK("bstn_sw_bister task created");
	} else {
		BSTN_STAGE_PRINTK("bstn_sw_bister task is already created");
	}
	return 0;
}

/*
 * @func    bstn_msg_manager_exit
 * @brief   This function frees the allocated request buffers of exchange nodes
 *          during cleanup.
 * @params  pbstn - the pointer to the BSTN device
 * @return  void
 */
void bstn_msg_manager_exit(struct bstn_device *pbstn)
{
	if (pbstn->msg_manager.ipc_session_id >= 0) {
		ipc_close(pbstn->msg_manager.ipc_session_id);
		pbstn->msg_manager.ipc_session_id = -1;
	}

	if (pbstn->msg_manager.msgbx_client) {
		pbstn->msg_manager.msgbx_client->stop();
		bstn_client_destroy();
		pbstn->msg_manager.msgbx_client = NULL;
		BSTN_STAGE_PRINTK("bstn_client_destroy OK");
	}

	if (pbstn->msg_manager.msg_sw_bister_task != NULL) {
		kthread_stop(pbstn->msg_manager.msg_sw_bister_task);
		pbstn->msg_manager.msg_sw_bister_task = NULL;
		BSTN_STAGE_PRINTK("msg_sw_bister_task kthread stop OK");
	}

	if (pbstn->msg_manager.msg_receiver_task != NULL) {
		kthread_stop(pbstn->msg_manager.msg_receiver_task);
		pbstn->msg_manager.msg_receiver_task = NULL;
		BSTN_STAGE_PRINTK("msg_receiver_task kthread stop OK");
	}

	if (pbstn->msg_manager.req_bufs) {
		pbstn->mem_manager.ops->free(pbstn->msg_manager.req_bufs);
		pbstn->msg_manager.req_bufs = NULL;
	}
	if (pbstn->msg_manager.flist) {
		mutex_destroy(&pbstn->msg_manager.flist_lock);
	}

	return;
}

/*
 * @func    bstn_msg_psm_enabled_status
 * @brief   send to safety and get psm status from safety lib.
 * @params  pbstn - the pointer to the BSTN device
 * @return  
 */
int bstn_msg_psm_enabled_status(struct bstn_device *pbstn)
{
    uint8_t blockid_in = 0xb8;
    uint8_t blockid_out = 0;
    int ret = 0;
    uint32_t psm_id_out[4] = {0};
    net_safety_UInt32Array4_t *psm_id = NULL;
    net_safety_ErrorEnum_t err = 0;
    ret = pbstn->msg_manager.msgbx_client->net_safety_client.fusaenable_method_sync(blockid_in, &blockid_out, &psm_id, &err, 5000, NULL);
    if (ret < 0 || err != 0) {
        BSTN_DEV_ERR(&pbstn->pdev->dev,
                "fusaenable_method_sync failed,ret: %d, err: %d", ret, err);
        return -EFAULT;
    }

    if (psm_id != NULL) {
        for (int i = 0; i < 4; i ++)
            psm_id_out[i] = (* psm_id)[i];
    }
    BSTN_STAGE_PRINTK(
    "psm_id_out[0]: %02x. psm_id_out[1]: %02x. psm_id_out[2]: %02x. psm_id_out[3]: %02x.",
    psm_id_out[0], psm_id_out[1], psm_id_out[2], psm_id_out[3]);

    return ret;
}
