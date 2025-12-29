// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*!
 * bst_lwnn: Linux device driver for Black Sesame Technologies Computer Vision IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    bst_lwnn_msg_manager.h
 * @brief   This is the header file of the message manager part in bst_lwnn
 * driver. It contains the structure definitions of the message manager and
 *          declarations of message handling functions as well as initialization
 *          and exit functions of the message manager.
 */

#ifndef BST_LWNN_MSG_MANAGER_H
#define BST_LWNN_MSG_MANAGER_H

#include "msgbx/lwnn_client.h"

#define BST_LWNN_MAX_WORKLOAD 10
#define BST_LWNN_RSP_TIMEOUT_MS (1000 * 10)

struct bst_lwnn_xchg {
	enum {
		XCHG_STATUS_SUCCESS = 0,
		XCHG_STATUS_FAILURE,
		XCHG_STATUS_REAPER
	} result;
	struct bst_lwnn_msg_xchg *xchg;
	struct completion complete;
	struct list_head link; // work list link
};

struct bst_lwnn_dsp_msg_ctl {
	struct bst_lwnn *pbst_lwnn;
	int dsp;
	enum {
		BST_LWNN_MSG_OFFLINE = 0,
		BST_LWNN_MSG_ONLINE,
		BST_LWNN_MSG_STOP,
	} state;
	int workload;
	int ipc_session_id;
	struct task_struct *worker;
	struct completion work_sem;
	struct mutex wl_lock;
	struct list_head work_list;
};

struct bst_lwnn_msg_manager {
	lwnn_client_data_t msgbx_data;
	lwnn_client_t *msgbx_client;

	struct mutex worker_lock;
	struct bst_lwnn_dsp_msg_ctl dsps[BST_LWNN_MAX_DSP_NUM];
	struct bst_lwnn_memblock *req_bufs;
};

int bst_lwnn_msg_send(struct bst_lwnn *pbst_lwnn, int dsp, uint32_t data);
int bst_lwnn_msg_recv(struct bst_lwnn *pbst_lwnn, int dsp, uint32_t *data,
		      int timeout);
int bst_lwnn_msg_xchg(struct bst_lwnn *pbst_lwnn,
		      struct bst_lwnn_msg_xchg *msg_xchg);
int bst_lwnn_msg_manager_init(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_msg_manager_probe_init(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_msg_manager_cleanup(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_msg_manager_exit(struct bst_lwnn *pbst_lwnn);
bool bst_lwnn_msg_is_bootdone(struct bst_lwnn *pbst_lwnn, int target);
int bst_lwnn_msg_psm_enabled_status(struct bst_lwnn *pbst_lwnn);

#endif
