/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_ISP_MSG_H__
#define __BST_ISP_MSG_H__

#include <bst/media-dev.h>

#include "isp_core.h"
#include "isp_video.h"
#include "isp_proto_base.h"

#define ISP_MSG_TX_TRY_TIMES	  (10)
#define ISP_MSG_TX_TRY_DELAY	  (1000) /* by us */
#define ISP_MSG_NOACK_TRY_TIMES	  (5)
#define ISP_MSG_ACK_TIMEOUT	  (200) /* by ms */

struct msgbx_rx_msg {
	u32 data;
	u64 timestamp;
	struct list_head queue_node;
};

int isp_msg_init_ipc(struct isp_device *isp);
void isp_msg_exit_ipc(struct isp_device *isp);
int isp_msg_init_msgbox(struct isp_device *isp);
void isp_msg_exit_msgbox(struct isp_device *isp);
int isp_msg_resume(struct isp_device *isp);

int isp_msg_tx(struct isp_device *isp, struct media_command *mc, bool ack);

/* Device related */
int isp_msg_dev_start(struct isp_device *isp);
int isp_msg_rw_addr(struct isp_device *isp);

/* Channel related */
int isp_msg_channel_set_cfg(struct isp_channel *channel);
int isp_msg_channel_tx_algo(struct isp_channel *channel,
			    const struct isp_file *file);
int isp_msg_channel_tx_iq(struct isp_channel *channel,
			  const struct isp_file *file);

/* Video related */
int isp_msg_video_open(struct isp_video *video);
int isp_msg_video_close(struct isp_video *video);
int isp_msg_video_open_raw(struct isp_video *video);
int isp_msg_video_close_raw(struct isp_video *video);
int isp_msg_video_plugout(struct isp_video *video);
int isp_msg_video_plugin(struct isp_video *video);
int isp_msg_video_txbuf(struct isp_video *video, u32 *dma_addrs);

#endif /* __BST_ISP_MSG_H__ */
