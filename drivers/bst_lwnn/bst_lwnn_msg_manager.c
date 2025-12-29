// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*!
 * bst_lwnn: Linux device driver for Black Sesame Technologies Computer Vision IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    bst_lwnn_msg_manager.c
 * @brief   This is the source file of the message manager part of bst_lwnn
 * driver. It contains function definitions of message handling as well as
 *          initialization and exit of the message manager.
 * @note    As the index of the DSP in the metadata array is not the same as
 *          the index of the DSP in the CV subsystem, the array index is
 *          referred as DSP id to differentiate from the hardware DSP index.
 */

#include "bst_lwnn.h"

extern int has_cv_dsp2_iommu_map;
extern int has_cv_dsp3_iommu_map;

int disp_run_sync(int dsp, lwnn_client_t *msgbx_client,
		  struct bst_lwnn_msg_xchg *msg_xchg)
{
	int ret = 0;
	uint32_t perf_us = 0;

	switch (dsp) {
	case 0: {
		cvdsp0_ErrorEnum_t err = 0;
		ret = msgbx_client->cvdsp0_client.disp_run_sync(
			msg_xchg->req.opcode, msg_xchg->req.pdata,
			&msg_xchg->rsp.status, &perf_us, &err,
			BST_LWNN_RSP_TIMEOUT_MS, NULL);
		break;
	}
	case 1: {
		cvdsp1_ErrorEnum_t err = 0;
		ret = msgbx_client->cvdsp1_client.disp_run_sync(
			msg_xchg->req.opcode, msg_xchg->req.pdata,
			&msg_xchg->rsp.status, &perf_us, &err,
			BST_LWNN_RSP_TIMEOUT_MS, NULL);
		break;
	}
	case 2: {
		cvdsp2_ErrorEnum_t err = 0;
		ret = msgbx_client->cvdsp2_client.disp_run_sync(
			msg_xchg->req.opcode, msg_xchg->req.pdata,
			&msg_xchg->rsp.status, &perf_us, &err,
			BST_LWNN_RSP_TIMEOUT_MS, NULL);
		break;
	}
	case 3: {
		cvdsp3_ErrorEnum_t err = 0;
		ret = msgbx_client->cvdsp3_client.disp_run_sync(
			msg_xchg->req.opcode, msg_xchg->req.pdata,
			&msg_xchg->rsp.status, &perf_us, &err,
			BST_LWNN_RSP_TIMEOUT_MS, NULL);
		break;
	}
	default:
		ret = -2;
		break;
	}

	return ret;
}

/*!
 * @brief       This function sends a message with the specified 4-byte data to
 *              the target DSP.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @param[in]   dsp The LWNN DSP id
 * @param[in]   data The data to be sent
 * @return      0 - success
 *              Error code - failure
 */
int bst_lwnn_msg_send(struct bst_lwnn *pbst_lwnn, int dsp, uint32_t data)
{
	ipc_msg msg;

	msg.data = data;
	msg.type = IPC_MSG_TYPE_SIGNAL;
	return ipc_send(pbst_lwnn->msg_manager.dsps[dsp].ipc_session_id, &msg,
			0);
}

/*!
 * @brief       This function receives a response from the target DSP with the
 *              specified timeout.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @param[in]   dsp The LWNN DSP id
 * @param[in]   data The pointer to the received data field
 * @param[in]   timeout The timeout in ms(-1 means waiting forever)
 * @return      0 - success
 *              Error code - failure
 */
int bst_lwnn_msg_recv(struct bst_lwnn *pbst_lwnn, int dsp, uint32_t *data,
		      int timeout)
{
	int ret;
	ipc_msg msg = { 0 };

	do {
		ret = ipc_recv(pbst_lwnn->msg_manager.dsps[dsp].ipc_session_id,
			       &msg, timeout);
		if (ret < 0) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "msg recv timeout");
			return ret;
		} else if (msg.data !=
			   dma_to_bus(pbst_lwnn->fw_manager.dsps[dsp]
					      .assigned_mem->dma_addr)) {
			BST_LWNN_STAGE_PRINTK(
				"DSP debugging: %s",
				(char *)bus_to_kern(pbst_lwnn, msg.data,
						    pbst_lwnn->fw_manager
							    .dsps[dsp]
							    .assigned_mem));
		} else {
			*data = msg.data;
			return 0;
		}
	} while (1);
}

/*!
 * @brief       This is the worker thread function which handles the request
 *              sending and response receiving for one DSP
 * @param[in]   args The pointer to the message controller of the target DSP
 * @return      0
 */
static int bst_lwnn_worker(void *args)
{
	struct bst_lwnn_dsp_msg_ctl *cur = args;
	struct bst_lwnn *pbst_lwnn = cur->pbst_lwnn;
	int dsp = cur->dsp;
	struct sched_param param;
	struct bst_lwnn_xchg *xchg;
	struct bst_lwnn_req *req_buf =
		&((struct bst_lwnn_req *)
			  pbst_lwnn->msg_manager.req_bufs->kern_addr)[dsp];
	dsp_ptr req_buf_bus_addr = kern_to_bus(pbst_lwnn, req_buf,
					       pbst_lwnn->msg_manager.req_bufs);
	uint32_t rsp;

	BST_LWNN_STAGE_PRINTK("lwnn_worker, opcode is %d, pdata is 0x%0x",
			      req_buf->opcode, req_buf->pdata);
	param.sched_priority = MAX_RT_PRIO - 1;
	sched_setscheduler(current, SCHED_FIFO, &param);
	pbst_lwnn->msg_manager.dsps[dsp].state = BST_LWNN_MSG_ONLINE;
	do {
		//wait for new work
		wait_for_completion_interruptible(
			&pbst_lwnn->msg_manager.dsps[dsp].work_sem);
		if (pbst_lwnn->msg_manager.dsps[dsp].state ==
		    BST_LWNN_MSG_STOP) {
			xchg = NULL;
			goto worker_exit;
		}
		//take a request from the work list
		mutex_lock(&pbst_lwnn->msg_manager.dsps[dsp].wl_lock);
		xchg = container_of(
			pbst_lwnn->msg_manager.dsps[dsp].work_list.next,
			struct bst_lwnn_xchg, link);
		list_del(pbst_lwnn->msg_manager.dsps[dsp].work_list.next);
		mutex_unlock(&pbst_lwnn->msg_manager.dsps[dsp].wl_lock);
		//copy the request
		*req_buf = xchg->xchg->req;
		//send request and wait for response (the current request owner thread becomes the reaper of this kthread if failing)
		if (bst_lwnn_msg_send(pbst_lwnn, dsp, req_buf_bus_addr) < 0) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "fail to send the request!");
			xchg->result = XCHG_STATUS_REAPER;
		} else {
			if (bst_lwnn_msg_recv(pbst_lwnn, dsp, &rsp,
					      BST_LWNN_RSP_TIMEOUT_MS) < 0) {
				BST_LWNN_DEV_ERR(
					&pbst_lwnn->pdev->dev,
					"fail to receive the response!");
				xchg->result = XCHG_STATUS_REAPER;
			} else {
				//copy the response
				xchg->xchg->rsp =
					*((struct bst_lwnn_rsp *)bus_to_kern(
						pbst_lwnn, rsp,
						pbst_lwnn->fw_manager.dsps[dsp]
							.assigned_mem));
				xchg->result = XCHG_STATUS_SUCCESS;
			}
		}

		//update the current DSP workload/status
		if (xchg->result == XCHG_STATUS_SUCCESS) {
			mutex_lock(&pbst_lwnn->msg_manager.worker_lock);
			pbst_lwnn->msg_manager.dsps[dsp].workload--;
			mutex_unlock(&pbst_lwnn->msg_manager.worker_lock);
			complete(&xchg->complete);
		} else {
			complete(&xchg->complete);
			goto worker_exit;
		}
	} while (1);

worker_exit:
	//if this is a failure exit
	if (xchg != NULL) {
		ipc_close(pbst_lwnn->msg_manager.dsps[dsp].ipc_session_id);
		pbst_lwnn->msg_manager.dsps[dsp].state = BST_LWNN_MSG_OFFLINE;
		mutex_lock(&pbst_lwnn->msg_manager.worker_lock);
		pbst_lwnn->dsp_online[dsp] = 0;
		mutex_unlock(&pbst_lwnn->msg_manager.worker_lock);
		BST_LWNN_STAGE_PRINTK("take down DSP %d", dsp);
		if (!bst_lwnn_check_online(pbst_lwnn)) {
			mutex_lock(&pbst_lwnn->mutex);
			pbst_lwnn->state = BST_LWNN_ERROR;
			mutex_unlock(&pbst_lwnn->mutex);
			BST_LWNN_STAGE_PRINTK("all DSP(s) down");
		}
		//handle the rest requests in the work list
		while (1) {
			mutex_lock(&pbst_lwnn->msg_manager.dsps[dsp].wl_lock);
			if (pbst_lwnn->msg_manager.dsps[dsp].work_list.next !=
			    &pbst_lwnn->msg_manager.dsps[dsp].work_list) {
				struct bst_lwnn_xchg *cur;

				cur = container_of(pbst_lwnn->msg_manager
							   .dsps[dsp]
							   .work_list.next,
						   struct bst_lwnn_xchg, link);
				list_del(pbst_lwnn->msg_manager.dsps[dsp]
						 .work_list.next);
				mutex_unlock(&pbst_lwnn->msg_manager.dsps[dsp]
						      .wl_lock);
				cur->result = XCHG_STATUS_FAILURE;
				complete(&cur->complete);
			} else {
				mutex_unlock(&pbst_lwnn->msg_manager.dsps[dsp]
						      .wl_lock);
				break;
			}
		}
	}

	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
	}
	BST_LWNN_STAGE_PRINTK("The worker thread of DSP %d exits", dsp);
	return 0;
}

/*!
 * @brief           This function dispatches a request to one DSP using IPC
 * @param[in]       pbst_lwnn The bst_lwnn driver
 * @param[in,out]   xchg The pointer to the message exchange information
 * @return          0 - success
 *                  Error code - failure
 */
int bst_lwnn_msg_xchg_ipc(struct bst_lwnn *pbst_lwnn,
			  struct bst_lwnn_msg_xchg *msg_xchg)
{
	int i;
	int ret;
	struct bst_lwnn_xchg xchg = { 0 };
	int target = msg_xchg->target_dsp;

	if (target >= pbst_lwnn->dsp_num) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "invalid target DSP(target=%d)!", target);
		return -EINVAL;
	}

	if (target >= 0 && !pbst_lwnn->dsp_online[target]) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "warnning: target (%d)"
				 "is offline,change to other dsp!",
				 target);
		target = -1;
	}
	//initialize the exchange
	xchg.xchg = msg_xchg;
	init_completion(&xchg.complete);

	//assign the exchange work to the DSP
	mutex_lock(&pbst_lwnn->msg_manager.worker_lock);
	//if a target DSP is specified
	if (target >= 0) {
		if (pbst_lwnn->dsp_online[target]) {
			if (pbst_lwnn->msg_manager.dsps[target].workload <
			    BST_LWNN_MAX_WORKLOAD) {
				mutex_lock(&pbst_lwnn->msg_manager.dsps[target]
						    .wl_lock);
				list_add_tail(&xchg.link,
					      &pbst_lwnn->msg_manager
						       .dsps[target]
						       .work_list);
				mutex_unlock(
					&pbst_lwnn->msg_manager.dsps[target]
						 .wl_lock);
				mutex_unlock(
					&pbst_lwnn->msg_manager.worker_lock);
			} else {
				mutex_unlock(
					&pbst_lwnn->msg_manager.worker_lock);
				BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
						 "target DSP at full load!");
				ret = -EBUSY;
				goto msg_xchg_end;
			}
		} else {
			mutex_unlock(&pbst_lwnn->msg_manager.worker_lock);
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "target DSP not working!");
			ret = -EFAULT;
			goto msg_xchg_end;
		}
	} else { //otherwise, distribute based on the current workload of all available DSPs
		int min = BST_LWNN_MAX_WORKLOAD;

		for (i = 0; i < pbst_lwnn->dsp_num; i++) {
			if (pbst_lwnn->dsp_online[i] &&
			    pbst_lwnn->msg_manager.dsps[i].workload < min) {
				min = pbst_lwnn->msg_manager.dsps[i].workload;
				target = i;
			}
		}
		if (target >= 0) {
			pbst_lwnn->msg_manager.dsps[target].workload++;
			//add the request work into the work list of the target DSP
			mutex_lock(
				&pbst_lwnn->msg_manager.dsps[target].wl_lock);
			list_add_tail(
				&xchg.link,
				&pbst_lwnn->msg_manager.dsps[target].work_list);
			mutex_unlock(
				&pbst_lwnn->msg_manager.dsps[target].wl_lock);
			mutex_unlock(&pbst_lwnn->msg_manager.worker_lock);
		} else {
			mutex_unlock(&pbst_lwnn->msg_manager.worker_lock);
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "all DSPs at full load!");
			ret = -EBUSY;
			goto msg_xchg_end;
		}
	}

	BST_LWNN_TRACE_PRINTK("add the xchg to the work list of DSP %d",
			      target);
	complete(&pbst_lwnn->msg_manager.dsps[target].work_sem);

	//wait for exchange completion
	wait_for_completion(&xchg.complete);
	if (xchg.result != XCHG_STATUS_SUCCESS) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "message exchange failed for DSP %d!", target);
		//reap the worker thread if the current thread is the reaper
		if (xchg.result == XCHG_STATUS_REAPER) {
			kthread_stop(
				pbst_lwnn->msg_manager.dsps[target].worker);
		}
		ret = -EFAULT;
	} else {
		ret = 0;
	}

msg_xchg_end:
	return ret;
}

/*!
 * @brief           This function dispatches a request to one DSP using MSGBOX
 * @param[in]       pbst_lwnn The bst_lwnn driver
 * @param[in,out]   xchg The pointer to the message exchange information
 * @return          0 - success
 *                  Error code - failure
 */
int bst_lwnn_msg_xchg_msgbox(struct bst_lwnn *pbst_lwnn,
			     struct bst_lwnn_msg_xchg *msg_xchg)
{
	int i;
	int ret;
	int target = msg_xchg->target_dsp;

	if (target >= pbst_lwnn->dsp_num) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "invalid target DSP(target=%d)!", target);
		return -EINVAL;
	}
	BST_LWNN_TRACE_PRINTK("target %d specified", target);
	// if a target DSP is specified
	if (target >= 0) {
		if (pbst_lwnn->dsp_online[target]) {
		} else {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "target DSP %d not working!", target);
			ret = -EFAULT;
			goto msg_xchg_end;
		}
	} else {
		int min = BST_LWNN_MAX_WORKLOAD;

		mutex_lock(&pbst_lwnn->msg_manager.worker_lock);
		for (i = 0; i < pbst_lwnn->dsp_num; i++) {
			if (pbst_lwnn->dsp_online[i] &&
			    pbst_lwnn->msg_manager.dsps[i].workload < min) {
				min = pbst_lwnn->msg_manager.dsps[i].workload;
				target = i;
			}
		}
		mutex_unlock(&pbst_lwnn->msg_manager.worker_lock);
	}

	if (target >= 0) {
		mutex_lock(&pbst_lwnn->msg_manager.worker_lock);
		pbst_lwnn->msg_manager.dsps[target].workload++;
		mutex_unlock(&pbst_lwnn->msg_manager.worker_lock);
	} else {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "all DSPs at full load!");
		ret = -EBUSY;
		goto msg_xchg_end;
	}

	BST_LWNN_TRACE_PRINTK("target %d msg begin send", target);

	ret = disp_run_sync(pbst_lwnn->dsp_indices[target],
			    pbst_lwnn->msg_manager.msgbx_client, msg_xchg);
	if (ret < 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "message exchange failed for target %d!",
				 target);
		ret = -EFAULT;
	} else {
		ret = 0;
		mutex_lock(&pbst_lwnn->msg_manager.worker_lock);
		pbst_lwnn->msg_manager.dsps[target].workload--;
		mutex_unlock(&pbst_lwnn->msg_manager.worker_lock);
		BST_LWNN_TRACE_PRINTK("exchange node received");
	}

msg_xchg_end:
	return ret;
}

/*!
 * @brief           This function dispatches a request to one DSP.
 * @param[in]       pbst_lwnn The bst_lwnn driver
 * @param[in,out]   xchg The pointer to the message exchange information
 * @return          0 - success
 *                  Error code - failure
 */
int bst_lwnn_msg_xchg(struct bst_lwnn *pbst_lwnn,
		      struct bst_lwnn_msg_xchg *msg_xchg)
{
	if (bst_lwnn_msg_interface == BST_LWNN_MSG_INTERFACE_IPC) {
		return bst_lwnn_msg_xchg_ipc(pbst_lwnn, msg_xchg);
	} else {
		return bst_lwnn_msg_xchg_msgbox(pbst_lwnn, msg_xchg);
	}
}

bool bst_lwnn_msg_is_bootdone(struct bst_lwnn *pbst_lwnn, int target)
{
#if 0
	int time;
	time =
	    wait_for_completion_timeout(&pbst_lwnn->msg_manager.dsps[target].
					ipc_boot_complete,
					msecs_to_jiffies(3000));
	if (!time) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "wait for target %d bootdone message timeout",
				 target);
		return false;
	}
#else
	msleep(200);
#endif
	return true;
}

/*!
 * @brief       This is the initailization function of the message manager.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @return      0 - success
 *              Error code - failure
 */
int bst_lwnn_msg_manager_init(struct bst_lwnn *pbst_lwnn)
{
	int i;
	struct bst_lwnn_dsp_msg_ctl *cur;
	char worker_name[10];
	dma_addr_t iova;
#if 1
	phys_addr_t res_paddr;
	resource_size_t res_size;
	struct iova_domain *iovad;
	struct iova *iova_resv;
	unsigned long shift;
#endif

	//bypass fwmem phys addr
	for (i = 0; i < min(pbst_lwnn->dsp_num, BST_LWNN_MAX_DSP_NUM); i++) {
		if (pbst_lwnn->mem_manager.enable_smmu) {
			if (i == 2 && has_cv_dsp2_iommu_map != 0) {
				continue;
			}
			if (i == 3 && has_cv_dsp3_iommu_map != 0) {
				continue;
			}

			iova = pbst_lwnn->mem_manager.ops->iommu_bypass(
				pbst_lwnn,
				pbst_lwnn->fw_manager.dsps[i].fwmem_size,
				PAGE_SIZE,
				pbst_lwnn->fw_manager.dsps[i].fwmem_phys_addr,
				IOMMU_READ | IOMMU_WRITE | IOMMU_PRIV);

			pbst_lwnn->fw_manager.dsps[i].fwmem_iova = iova;

			if (i == 2) {
				has_cv_dsp2_iommu_map = 1;
			}
			if (i == 3) {
				has_cv_dsp3_iommu_map = 1;
			}
		}
	}

#if 1
	if (pbst_lwnn->mem_manager.enable_smmu) {
		// TODO, dts configuration
		res_paddr = 0x816000000;
		res_size = 0x02000000;
		iova = pbst_lwnn->mem_manager.ops->iommu_bypass_iova(
			pbst_lwnn, res_size, PAGE_SIZE, (phys_addr_t)res_paddr,
			0x6b400000,
			IOMMU_READ | IOMMU_WRITE | IOMMU_PRIV);

		pbst_lwnn->fw_manager.res_bypass[0].paddr = res_paddr;
		pbst_lwnn->fw_manager.res_bypass[0].size = res_size;
		pbst_lwnn->fw_manager.res_bypass[0].iova = iova;
		BST_LWNN_TRACE_PRINTK(
			"iova bypass: pa 0x%llx iova 0x%llx size 0x%llx",
			res_paddr, iova, res_size);

		res_paddr = 0x50000000;
		res_size = 0x10000000;
		iova = pbst_lwnn->mem_manager.ops->iommu_bypass(
			pbst_lwnn, res_size, PAGE_SIZE, (phys_addr_t)res_paddr,
			IOMMU_READ | IOMMU_WRITE | IOMMU_PRIV); // cv reserved

		pbst_lwnn->fw_manager.res_bypass[1].paddr = res_paddr;
		pbst_lwnn->fw_manager.res_bypass[1].size = res_size;
		pbst_lwnn->fw_manager.res_bypass[1].iova = iova;
		BST_LWNN_TRACE_PRINTK(
			"iova bypass: pa 0x%llx iova 0x%llx size 0x%llx",
			res_paddr, iova, res_size);

		iovad = pbst_lwnn->mem_manager.iovad;
		shift = iova_shift(iovad);

		iova_resv = reserve_iova(iovad, 0x00000000 >> shift,
					 0x80000000 >> shift);
		if (!iova_resv) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "failed to reserve IOVA");
			return -ENOMEM;
		}
		pbst_lwnn->mem_manager.iova_resv_dummy = iova_resv;
		BST_LWNN_STAGE_PRINTK(
			"reserve_iova low pfn 0x%lx, high pfn 0x%lx",
			iova_resv->pfn_lo, iova_resv->pfn_hi);
	}
#endif

	memcpy(worker_name, "bst_lwnn", 8);
	pbst_lwnn->msg_manager.req_bufs = pbst_lwnn->mem_manager.ops->alloc(
		pbst_lwnn, sizeof(struct bst_lwnn_req) * pbst_lwnn->dsp_num, 0,
		0);
	if (pbst_lwnn->msg_manager.req_bufs == NULL) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "request buffer allocation failed");
		return -ENOMEM;
	}
	mutex_init(&pbst_lwnn->msg_manager.worker_lock);

	for (i = 0; i < pbst_lwnn->dsp_num; i++) {
		if (pbst_lwnn->dsp_online[i]) {
			cur = &pbst_lwnn->msg_manager.dsps[i];
			cur->pbst_lwnn = pbst_lwnn;
			cur->dsp = i;
			cur->workload = 0;
			init_completion(&cur->work_sem);
			cur->work_list = (struct list_head)LIST_HEAD_INIT(
				cur->work_list);
			mutex_init(&cur->wl_lock);
		}
	}

	if (bst_lwnn_msg_interface == BST_LWNN_MSG_INTERFACE_IPC) {
		for (i = 0; i < pbst_lwnn->dsp_num; i++) {
			if (pbst_lwnn->dsp_online[i]) {
				cur = &pbst_lwnn->msg_manager.dsps[i];
				cur->ipc_session_id = ipc_init(
					IPC_CORE_CV0 +
						pbst_lwnn->dsp_indices[i],
					IPC_CORE_ARM0 +
						pbst_lwnn->fw_manager.dsps[i]
							.ipc_src_core,
					&pbst_lwnn->pdev->dev);
				if (cur->ipc_session_id < 0) {
					pbst_lwnn->dsp_online[i] = 0;
					BST_LWNN_DEV_ERR(
						&pbst_lwnn->pdev->dev,
						"ipc_init failed for DSP %d, ret %d",
						i, cur->ipc_session_id);
				} else {
					BST_LWNN_STAGE_PRINTK(
						"bst_lwnn dsp %d ipc init succeeded",
						i);
					worker_name[8] = '0' + i;
					worker_name[9] = '\0';
					cur->worker = kthread_run(
						bst_lwnn_worker,
						&pbst_lwnn->msg_manager.dsps[i],
						worker_name);
					if (IS_ERR(cur->worker)) {
						pbst_lwnn->dsp_online[i] = 0;
						ipc_close(cur->ipc_session_id);
						BST_LWNN_DEV_ERR(
							&pbst_lwnn->pdev->dev,
							"kthread_run failed for DSP %d!",
							i);
						continue;
					}
					BST_LWNN_STAGE_PRINTK(
						"kthread_run run successfully for DSP %d",
						i);
				}
				cur->state = BST_LWNN_MSG_ONLINE;
			}
		}
		if (!bst_lwnn_check_online(pbst_lwnn)) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "kthread_run failed for all DSPs");
			return -EFAULT;
		}
		BST_LWNN_STAGE_PRINTK("worker creation OK");
	} else {
		pbst_lwnn->msg_manager.msgbx_client =
			lwnn_client_init(&pbst_lwnn->msg_manager.msgbx_data);
		if (pbst_lwnn->msg_manager.msgbx_client == NULL) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "lwnn_client_init failed");
			return -EFAULT;
		}

		if (pbst_lwnn->msg_manager.msgbx_client->start() < 0) {
			lwnn_client_destroy();
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "msg client start failed!");
			return -EFAULT;
		}
		BST_LWNN_STAGE_PRINTK("lwnn_client_init and start OK");

		if (!bst_lwnn_check_online(pbst_lwnn)) {
			pbst_lwnn->msg_manager.msgbx_client->stop();
			lwnn_client_destroy();
			BST_LWNN_DEV_ERR(
				&pbst_lwnn->pdev->dev,
				"bst_lwnn_check_online failed for all DSPs");
			return -EFAULT;
		}
		BST_LWNN_STAGE_PRINTK("msgbx_client start OK");
	}

	return 0;
}

/*!
 * @brief       This is the variables initailization function in bst_lwnn probe
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @return      void
 */
void bst_lwnn_msg_manager_probe_init(struct bst_lwnn *pbst_lwnn)
{
	int i;
	for (i = 0; i < pbst_lwnn->dsp_num; i++) {
		pbst_lwnn->msg_manager.dsps[i].ipc_session_id = -1;
		pbst_lwnn->msg_manager.dsps[i].worker = NULL;
	}
	pbst_lwnn->msg_manager.msgbx_client = NULL;
	pbst_lwnn->msg_manager.req_bufs = NULL;
}

/*
 * @func    bst_lwnn_msg_manager_cleanup
 * @brief   This function cleans up the resources allocated in the message
 *          manager initailization for subsequential failure of DSPs or
 *          even the driver during the entire initialization.
 * @params  pbst_lwnn The bst_lwnn driver
 * @return  void
 */
void bst_lwnn_msg_manager_cleanup(struct bst_lwnn *pbst_lwnn)
{
	int i;

	for (i = 0; i < pbst_lwnn->dsp_num; i++) {
		if (!pbst_lwnn->dsp_online[i] &&
		    pbst_lwnn->msg_manager.dsps[i].state ==
			    BST_LWNN_MSG_ONLINE) {
			pbst_lwnn->msg_manager.dsps[i].state =
				BST_LWNN_MSG_STOP;
			if (bst_lwnn_msg_interface ==
			    BST_LWNN_MSG_INTERFACE_IPC) {
				if (pbst_lwnn->msg_manager.dsps[i]
					    .ipc_session_id >= 0) {
					ipc_close(pbst_lwnn->msg_manager.dsps[i]
							  .ipc_session_id);
				}
				pbst_lwnn->msg_manager.dsps[i].ipc_session_id =
					-1;
				complete(&pbst_lwnn->msg_manager.dsps[i]
						  .work_sem);
				kthread_stop(
					pbst_lwnn->msg_manager.dsps[i].worker);
				pbst_lwnn->msg_manager.dsps[i].worker = NULL;
			}
		}
	}
	return;
}

/*!
 * @brief       This is the exit function of the message manager.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @return      Void
 */
void bst_lwnn_msg_manager_exit(struct bst_lwnn *pbst_lwnn)
{
	int i;

	if (bst_lwnn_msg_interface == BST_LWNN_MSG_INTERFACE_IPC) {
		for (i = 0; i < pbst_lwnn->dsp_num; i++) {
			if (pbst_lwnn->dsp_online[i] &&
			    pbst_lwnn->msg_manager.dsps[i].state ==
				    BST_LWNN_MSG_ONLINE) {
				pbst_lwnn->msg_manager.dsps[i].state =
					BST_LWNN_MSG_STOP;
				if (pbst_lwnn->msg_manager.dsps[i]
					    .ipc_session_id >= 0) {
					ipc_close(pbst_lwnn->msg_manager.dsps[i]
							  .ipc_session_id);
				}
				pbst_lwnn->msg_manager.dsps[i].ipc_session_id =
					-1;
				complete(&pbst_lwnn->msg_manager.dsps[i]
						  .work_sem);
				kthread_stop(
					pbst_lwnn->msg_manager.dsps[i].worker);
				pbst_lwnn->msg_manager.dsps[i].worker = NULL;
			}
		}
	} else {
		for (i = 0; i < pbst_lwnn->dsp_num; i++) {
			if (pbst_lwnn->dsp_online[i] &&
			    pbst_lwnn->msg_manager.dsps[i].state ==
				    BST_LWNN_MSG_ONLINE) {
				pbst_lwnn->msg_manager.dsps[i].state =
					BST_LWNN_MSG_STOP;
			}
		}
		if (NULL != pbst_lwnn->msg_manager.msgbx_client) {
			pbst_lwnn->msg_manager.msgbx_client->stop();
			pbst_lwnn->msg_manager.msgbx_client = NULL;
			lwnn_client_destroy();
		}
		if (NULL != pbst_lwnn->msg_manager.req_bufs) {
			pbst_lwnn->mem_manager.ops->free(
				pbst_lwnn->msg_manager.req_bufs);
			pbst_lwnn->msg_manager.req_bufs = NULL;
		}
	}

	return;
}

/*!
 * @brief       This function sends to safety and get psm status from safety lib
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @return      0-success
 */
int bst_lwnn_msg_psm_enabled_status(struct bst_lwnn *pbst_lwnn)
{
	uint8_t blockid_in = 0xb7;
	uint8_t blockid_out = 0;
	int ret = 0;
	uint32_t psm_id_out[4] = { 0 };
	cvdsp_safety_UInt32Array4_t *psm_id = NULL;
	cvdsp_safety_ErrorEnum_t err = 0;
	ret = pbst_lwnn->msg_manager.msgbx_client->cvdsp_safety_client
		      .fusaenable_method_sync(blockid_in, &blockid_out, &psm_id,
					      &err, 5000, NULL);
	if (ret < 0 || err != 0) {
		BST_LWNN_DEV_ERR(
			&pbst_lwnn->pdev->dev,
			"fusaenable_method_sync failed,ret: %d, err: %d", ret,
			err);
		return -EFAULT;
	}

	if (psm_id != NULL) {
		for (int i = 0; i < 4; i++)
			psm_id_out[i] = (*psm_id)[i];
	}
	BST_LWNN_STAGE_PRINTK(
		"psm_id_out[0]: %02x. psm_id_out[1]: %02x. psm_id_out[2]: %02x. psm_id_out[3]: %02x.",
		psm_id_out[0], psm_id_out[1], psm_id_out[2], psm_id_out[3]);

	return ret;
}
