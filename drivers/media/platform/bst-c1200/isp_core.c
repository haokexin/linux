// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"

#include <linux/cacheflush.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kobject.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/vmalloc.h>
#include <linux/iommu.h>
#include <linux/iova.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/reset.h>

#include <bst/ipc_interface.h>

#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>

#include <linux/coreip/proto_api_common.h>

#include "../../i2c/bst/maxim_deser_hub.h"
#include "isp_core.h"
#include "isp_fw_loader.h"
#include "isp_hw.h"
#include "isp_sysfile.h"
#include "isp_video.h"
#include "proto_isp_ipc.h"

#include <linux/kthread.h>
#include <linux/string.h>
#include "video_server.h"

static inline int send_msg_async(struct c1200_isp_device *isp, uint32_t msg);

#define MAX_KTHREAD_MAX_TRY 5

// isp msg payload size, 256Byte alignment
#define ISP_MSG_PAYLOAD_SIZE	  0x100
#define ISP_MSG_PAYLOAD_SIZE_MASK 0xff

#define IPC_SEND_RETRY_TIMES 10

struct bin_info_t {
	char filename[MAX_BIN_NAME_LEN];
	u8 *payload_vaddr;
	u32 payload_paddr;
	int size;
	int camera_mask; // maybe used by multi cameras
};

struct isp_algo_bin_t {
	struct bin_info_t algo_bin[MAX_ISP_CHANNEL];
	int file_num;
};

static struct isp_algo_bin_t s_isp_algobin;
static struct isp_misc_device *globle_isp_misc_device;

static int send_next_cfg_to_fw(struct c1200_isp_device *isp);
static int send_next_alg_to_fw(struct c1200_isp_device *isp);
static int send_next_iq_to_fw(struct c1200_isp_device *isp);
static int send_start_msg_to_fw(struct c1200_isp_device *isp);
static int get_next_fmt_from_fw(struct c1200_isp_device *isp);

static struct c1200_isp_device *g_isp;

static int setup_msg_area(struct c1200_isp_device *isp)
{
	uint32_t init_phy_base_low;
	uint32_t init_phy_base_high;
	uint64_t init_phy_base;
	uint64_t cmdp_phy_base;
	uint64_t slab_phy_base;
	tSoneInit *pInit;

	void *pSlab;
	tSoneCmdp *pCmdp;

	uint32_t traceMask = 0xffffffff;
	int cmdp_size;
	int initp_size;
	uint32_t addr_start;
	int partition_total_size;
	dma_addr_t partition_dma_addr;
	char *partition_vaddr;
	u8 *payload_vaddr;
	uint64_t payload_base;
	uint32_t payload_paddr;
	u32 addr_offset;

	addr_start = 0;
	init_phy_base = 0;
	init_phy_base_low = 0;
	init_phy_base_high = 0;

	// 1KB alignment
	// to do: 4KB alignment
	initp_size = (sizeof(tSoneInit) + SONE_PART_ALIGN_MASK) &
		     (~SONE_PART_ALIGN_MASK);
	pr_err("init start = 0x%llx, initp_size = 0x%lx, align size = 0x%x\n",
	       init_phy_base, sizeof(tSoneInit), initp_size);

	addr_start += initp_size;
	cmdp_phy_base = addr_start;
	cmdp_size = (sizeof(tSoneCmdp) + SONE_PART_ALIGN_MASK) &
		    (~SONE_PART_ALIGN_MASK);
	pr_err("cmdp_phy_base = 0x%llx, cmdp_size align size = 0x%x\n",
	       cmdp_phy_base, cmdp_size);

	addr_start += cmdp_size;
	// for buf, 1MB alignment
	addr_start = (addr_start + SONE_BUF_ADDR_ALIGN_MASK) &
		     (~SONE_BUF_ADDR_ALIGN_MASK);
	slab_phy_base = addr_start;
	addr_start += SONE_MEDIA_SLAB_BUFSIZE;
	addr_start = (addr_start + SONE_BUF_ADDR_ALIGN_MASK) &
		     (~SONE_BUF_ADDR_ALIGN_MASK);

	/* Add media_command for ISP device, not for video */
	cmdp_size =
		((sizeof(struct media_command) * ISP_MAX_MEDIA_COMMAND_NUM) +
		 SONE_PART_ALIGN_MASK) &
		(~SONE_BUF_ADDR_ALIGN_MASK);
	addr_start += cmdp_size;

	partition_total_size = addr_start - init_phy_base;
	partition_dma_addr = isp->init_paddr;
	partition_vaddr = isp->init_vaddr;
	pr_err("dma_alloc_coherent partition_total_size = 0x%x paddr = 0x%lx vaddr = 0x%lx\n",
	       partition_total_size, (unsigned long)partition_dma_addr,
	       (unsigned long)partition_vaddr);
	isp->config_media_cmd =
		(struct media_command *)(partition_vaddr +
					 partition_total_size - cmdp_size);

	init_phy_base += (unsigned long)partition_dma_addr;
	cmdp_phy_base += (unsigned long)partition_dma_addr;
	slab_phy_base += (unsigned long)partition_dma_addr;
	init_phy_base_low = (init_phy_base & LOW_32_BIT_MASK);
	init_phy_base_high = (init_phy_base & HIGH_32_BIT_MASK) >> 32;

	isp->init_paddr = init_phy_base;
	isp->cmdp_paddr = cmdp_phy_base;
	isp->slab_paddr = slab_phy_base;

	// setup init partition
	pInit = (tSoneInit *)partition_vaddr;
	isp->init_vaddr = pInit;
	setup_init_parti(pInit, init_phy_base_low, init_phy_base_high,
			 ISP_PLATFORM, ISP_PLATFORM, traceMask, "C1200", 'A');

	// setup cmdp partition
	pCmdp = (tSoneCmdp *)(partition_vaddr +
			      (cmdp_phy_base - init_phy_base));
	isp->cmdp_vaddr = pCmdp;
	setup_cmdp_parti(pInit, pCmdp, cmdp_phy_base);

	// setup slab partition
	pSlab = (void *)(partition_vaddr + (slab_phy_base - init_phy_base));
	isp->slab_vaddr = pSlab;
	setup_slab_parti(pInit, pSlab, (slab_phy_base & LOW_32_BIT_MASK));

	payload_vaddr = (u8 *)pCmdp->ch[DRV_CH_INDEX].cqueue.payload_cmd;
	addr_offset = (uint64_t)(payload_vaddr) - (uint64_t)(isp->cmdp_vaddr);
	payload_paddr = (addr_offset + isp->cmdp_paddr) & LOW_32_BIT_MASK;
	isp->payload_end_vaddr = payload_vaddr + SONE_MEDIA_PAYLOAD_MEMSIZE;
	isp->payload_end_paddr = payload_paddr + SONE_MEDIA_PAYLOAD_MEMSIZE;

	payload_base = (uint64_t)payload_vaddr;
	payload_base = ((payload_base + ISP_MSG_PAYLOAD_SIZE_MASK) &
			(~ISP_MSG_PAYLOAD_SIZE_MASK));
	isp->config_payload_vaddr = (u8 *)payload_base;
	isp->config_align_size =
		(sizeof(ipc_reconf_t) + ISP_MSG_PAYLOAD_SIZE_MASK) &
		(~ISP_MSG_PAYLOAD_SIZE_MASK);

	isp->algobin_next_vaddr = isp->config_payload_vaddr +
				  (isp->config_align_size * MAX_ISP_CHANNEL);

	addr_offset = (uint64_t)(isp->config_payload_vaddr) -
		      (uint64_t)(isp->cmdp_vaddr);
	isp->config_payload_paddr =
		(0x80000000 + (addr_offset + isp->cmdp_paddr)) &
		LOW_32_BIT_MASK;

	addr_offset = (uint64_t)(isp->algobin_next_vaddr) -
		      (uint64_t)(isp->cmdp_vaddr);
	isp->algobin_next_paddr =
		(0x80000000 + (addr_offset + isp->cmdp_paddr)) &
		LOW_32_BIT_MASK;

	addr_offset =
		(uint64_t)(isp->config_media_cmd) - (uint64_t)(isp->cmdp_vaddr);
	isp->media_cmd_paddr = (0x80000000 + (addr_offset + isp->cmdp_paddr)) &
			       LOW_32_BIT_MASK;
	pr_err("%s: config_payload_paddr = 0x%x algobin_next_paddr = 0x%x media_cmd_paddr = 0x%x\n",
	       __func__, isp->config_payload_paddr, isp->algobin_next_paddr,
	       isp->media_cmd_paddr);
	return 0;
}

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_device_ops isp_media_ops = {
	.link_notify = v4l2_pipeline_link_notify,
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

static void isp_cleanup_videos(struct c1200_isp_device *isp)
{
	int i;

	for (i = 0; i < MAX_ISP_CHANNEL; i++) {
		c1200_isp_video_cleanup(&isp->channels[i].views_video);
		c1200_isp_video_cleanup(&isp->channels[i].raw_video);
	}
}

#pragma GCC diagnostic pop

int send_cmd_to_fw(struct c1200_isp_device *isp, struct _ipc_msg *msg)
{
	int ret;
	int retry_times;

	retry_times = IPC_SEND_RETRY_TIMES;
	mutex_lock(&isp->ipc_tx_mutex);

	if (isp->state == ISPDRV_STATE_SHUTDOWN) {
		dev_warn(isp->dev, "isp state is shutdown, should not send cmd to fw\n");
		return 0;
	}

	do {
		if (!(isp->use_ipc))
			ret = send_msg_async(isp, msg->data);
		else
			ret = ipc_send_sync(isp->ipc_session_id, msg);

		if (ret)
			usleep_range(1000, 1100);
		retry_times--;
	} while (ret && retry_times > 0);
	if (ret)
		dev_err_ratelimited(
			isp->dev,
			"%s: ipc_send error, ret: %d, retry times: %d\n",
			__func__, ret, IPC_SEND_RETRY_TIMES);
	else {
		isp->ipc_tx_count++;
		dev_dbg(isp->dev, "%s: sent paddr: 0x%08X\n", __func__,
			msg->data);
	}
	mutex_unlock(&isp->ipc_tx_mutex);

	return ret;
}

static int channel_cfg_list[MAX_ISP_CHANNEL] = { -1 };
static int channel_alg_list[MAX_ISP_CHANNEL] = { -1 };
static int channel_iq_list[MAX_ISP_CHANNEL] = { -1 };

static inline int is_channel_use_alg(struct c1200_isp_device *isp, int index)
{
	return isp->channels[index].cam_dev->algo[0];
}

static inline int is_channel_use_iq(struct c1200_isp_device *isp, int index)
{
	return isp->channels[index].cam_dev->iq[0];
}

static inline int is_channel_cfg_left(struct c1200_isp_device *isp)
{
	return (isp->cfg_count < isp->cfg_num);
}

static inline int is_channel_alg_left(struct c1200_isp_device *isp)
{
	return (isp->alg_count < isp->alg_num);
}

static inline int is_channel_iq_left(struct c1200_isp_device *isp)
{
	return (isp->iq_count < isp->iq_num);
}

static inline int is_camera_info_left(struct c1200_isp_device *isp)
{
	return (isp->get_cfg_count < isp->cfg_num);
}

static int next_channel_cfg_index(struct c1200_isp_device *isp)
{
	int index;
	int i;

	if (isp->cfg_count >= isp->cfg_num)
		return -1;

	if (channel_cfg_list[0] < 0) {
		index = 0;
		for (i = 0; i < MAX_ISP_CHANNEL; i++) {
			if (isp->channels[i].enable) {
				channel_cfg_list[index] = i;
				index++;
			}
		}
	}

	return channel_cfg_list[isp->cfg_count];
}

static int next_channel_alg_index(struct c1200_isp_device *isp)
{
	int index;
	int i;

	if (isp->alg_count >= isp->alg_num)
		return -1;

	if (channel_alg_list[0] < 0) {
		index = 0;
		for (i = 0; i < MAX_ISP_CHANNEL; i++) {
			if (isp->channels[i].enable &&
			    is_channel_use_alg(isp, i)) {
				channel_alg_list[index] = i;
				index++;
			}
		}
	}

	return channel_alg_list[isp->alg_count];
}

static int next_channel_iq_index(struct c1200_isp_device *isp)
{
	int index;
	int i;

	if (isp->iq_count >= isp->iq_num)
		return -1;

	if (channel_iq_list[0] < 0) {
		index = 0;
		for (i = 0; i < MAX_ISP_CHANNEL; i++) {
			if (isp->channels[i].enable &&
			    is_channel_use_iq(isp, i)) {
				channel_iq_list[index] = i;
				index++;
			}
		}
	}

	return channel_iq_list[isp->iq_count];
}

static int ispdrv_enter_cfg_state(struct c1200_isp_device *isp)
{
	send_next_cfg_to_fw(isp);
	isp->state = ISPDRV_STATE_SCFG;
	return 0;
}

static int ispdrv_enter_alg_state(struct c1200_isp_device *isp)
{
	send_next_alg_to_fw(isp);
	isp->state = ISPDRV_STATE_SALG;
	return 0;
}

static int ispdrv_enter_iq_state(struct c1200_isp_device *isp)
{
	send_next_iq_to_fw(isp);
	isp->state = ISPDRV_STATE_SIQ;
	return 0;
}

static int ispdrv_enter_start_state(struct c1200_isp_device *isp)
{
	send_start_msg_to_fw(isp);
	isp->state = ISPDRV_STATE_START;
	return 0;
}

static int ispdrv_enter_work_state(struct c1200_isp_device *isp)
{
	atomic_set(&(isp->FW_config_done), 1);
	complete_all(&isp->FW_start_completion);
	isp->state = ISPDRV_STATE_WORK;
	return 0;
}

static void ispdrv_state_error(struct c1200_isp_device *isp, uint16_t message)
{
	dev_err_ratelimited(isp->dev, "unexpected command %d in state %d\n",
			    message, isp->state);
}

static int ispdrv_get_cfg_state(struct c1200_isp_device *isp)
{
	get_next_fmt_from_fw(isp);
	isp->state = ISPDRV_STATE_GET_SCFG;
	return 0;
}

static int get_isp_entity_index(struct c1200_isp_device *isp,
				struct media_command *msg)
{
	int isp_entity_index;

	switch (isp->core_status) {
	case 0b110:
		if (msg->cmd_hdr.src[2] == '1' || msg->cmd_hdr.src[3] >= '8')
			isp_entity_index = (msg->cmd_hdr.src[2] - '0') * 10 +
					   (msg->cmd_hdr.src[3] - '0') - 8;
		else
			isp_entity_index = (msg->cmd_hdr.src[2] - '0') * 10 +
					   (msg->cmd_hdr.src[3] - '0');
		break;
	case 0b101:
		if (msg->cmd_hdr.src[2] == '1' || msg->cmd_hdr.src[3] >= '8')
			isp_entity_index = (msg->cmd_hdr.src[2] - '0') * 10 +
					   (msg->cmd_hdr.src[3] - '0') - 4;
		else
			isp_entity_index = (msg->cmd_hdr.src[2] - '0') * 10 +
					   (msg->cmd_hdr.src[3] - '0');
		break;
	default:
		isp_entity_index = (msg->cmd_hdr.src[2] - '0') * 10 +
				   (msg->cmd_hdr.src[3] - '0');
		break;
	}

	return isp_entity_index;
}

static int work_message_handler(struct c1200_isp_device *isp,
				struct media_command *msg, int64_t ktime)
{
	int isp_entity_index;
	uint16_t cmd_main = msg->cmd_hdr.hdr_info.cmd_type_main;
	uint16_t cmd_minor = msg->cmd_hdr.hdr_info.cmd_type_minor;
	uint32_t *tick = (uint32_t *)msg->cmd_hdr.dst;
	int line_length;

	if ((msg->cmd_hdr.src[0] == 'i') && (msg->cmd_hdr.src[1] == 'i')) {
		isp_entity_index = get_isp_entity_index(isp, msg);
		if (isp_entity_index < 0) {
			dev_err_ratelimited(isp->dev,
					    "isp_entity_index = %d, error\n",
					    isp_entity_index);
			return -1;
		}
		isp->kthread_status = KTHREAD_STATUS_COPY_MSG;
		line_length = (msg->cmd_hdr.magic.magic[0] << 8) +
			      msg->cmd_hdr.magic.magic[1];
		if (cmd_minor == MINOR_ISP_RAW_BUF_DONE) {
			copy_msg_to_video_cache(
				&(isp->channels[isp_entity_index].raw_video),
				cmd_main, cmd_minor, *tick, ktime,
				msg->user_cmd_data, 4, line_length);
		} else {
			copy_msg_to_video_cache(
				&(isp->channels[isp_entity_index].views_video),
				cmd_main, cmd_minor, *tick, ktime,
				msg->user_cmd_data, 4, line_length);
		}
		isp->kthread_status = KTHREAD_STATUS_COPY_COMPLETE;
	}

	return 0;
}

static int stream_message_handler(struct c1200_isp_device *isp,
				  struct media_command *msg)
{
	int isp_entity_index;

	if (!((msg->cmd_hdr.src[0] == 'i') && (msg->cmd_hdr.src[1] == 'i'))) {
		dev_err_ratelimited(isp->dev, "%s: Unknown message %c%c\n",
				    __func__, msg->cmd_hdr.src[0],
				    msg->cmd_hdr.src[1]);
		return -1;
	}

	isp_entity_index = get_isp_entity_index(isp, msg);
	if (isp_entity_index < 0) {
		dev_err_ratelimited(isp->dev,
				    "%s: Invalid isp_entity_index: %d\n",
				    __func__, isp_entity_index);
		return -1;
	}

	dev_dbg(isp->dev, "complete video %02d stream op\n", isp_entity_index);
	complete(&isp->channels[isp_entity_index].views_video.stream_comp);

	return 0;
}

static int set_camera_embed_offset(struct camera_dev *cam_dev)
{
	int32_t size_view0 = 0;
	int32_t size_view1 = 0;
	int32_t emd_zone1_size = 0;
	// only support NV12 format
	if (View0_NV12_Fmt == cam_dev->isp_data.viewinfo[0].viewFmt)
		size_view0 = ((cam_dev->isp_data.viewinfo[0].width *
					   cam_dev->isp_data.viewinfo[0].height) *
					  3) >>
					 1;

	if (View1_NV12_Fmt == cam_dev->isp_data.viewinfo[1].viewFmt)
		size_view1 = ((cam_dev->isp_data.viewinfo[1].width *
					   cam_dev->isp_data.viewinfo[1].height) *
					  3) >>
					 1;

	cam_dev->emd_view_info.isp_embed_info.emd_line_start[0] = cam_dev->isp_data.embeddedInfo[0].line_start;
	cam_dev->emd_view_info.isp_embed_info.emd_line_num[0] = cam_dev->isp_data.embeddedInfo[0].line_end;
	emd_zone1_size = cam_dev->emd_view_info.isp_embed_info.emd_line_num[0] *
					 cam_dev->isp_data.rawinfo.width;

	cam_dev->emd_view_info.isp_embed_info.emd_zone_offset[0][0] =
		YUV_STAT_BUF_EXPAND + size_view0;
	cam_dev->emd_view_info.isp_embed_info.emd_zone_offset[0][1] =
		YUV_STAT_BUF_EXPAND + size_view0 + emd_zone1_size;
	cam_dev->emd_view_info.isp_embed_info.emd_zone_offset[1][0] =
		YUV_STAT_BUF_EXPAND + size_view1;
	cam_dev->emd_view_info.isp_embed_info.emd_zone_offset[1][1] =
		YUV_STAT_BUF_EXPAND + size_view1 + emd_zone1_size;
	cam_dev->emd_view_info.isp_embed_info.emd_zone_offset[2][0] = 0;
	cam_dev->emd_view_info.isp_embed_info.emd_zone_offset[2][1] = 0;

	return 0;
}

static int c1200_parse_camera_info(struct c1200_isp_device *isp,
				   struct media_command *cmd)
{
	struct camera_dev *cam_dev;
	u8 *payload_vaddr;
	isp_ld_reconf_t *config_cmd;
	struct c1200_isp_video *video;
	int video_sn;

	cam_dev =
		isp->channels[channel_cfg_list[isp->get_cfg_count - 1]].cam_dev;
	config_cmd = (isp_ld_reconf_t *)(&cmd->user_cmd_data[0]);
	payload_vaddr = config_cmd->payloadAddr - isp->config_payload_paddr +
					isp->config_payload_vaddr;
	memcpy(&(cam_dev->isp_data), payload_vaddr, sizeof(ipc_reconf_t));
	set_camera_embed_offset(cam_dev);
	isp->channels[channel_cfg_list[isp->get_cfg_count - 1]].camera_raw_width =
		isp->channels[channel_cfg_list[isp->get_cfg_count - 1]].cam_dev->isp_data.rawinfo.width;
	isp->channels[channel_cfg_list[isp->get_cfg_count - 1]].camera_raw_height =
		isp->channels[channel_cfg_list[isp->get_cfg_count - 1]].cam_dev->isp_data.rawinfo.height;
	video = &(isp->channels[channel_cfg_list[isp->get_cfg_count - 1]].views_video);
	update_video_from_camera_config(video, isp->channels[channel_cfg_list[isp->get_cfg_count - 1]].cam_dev);
	video = &(isp->channels[channel_cfg_list[isp->get_cfg_count - 1]].raw_video);
	video->is_pdns = false;
	video->is_raw_video = true; // raw video means not processed by// ISP, include yuyv input
	video_sn = channel_cfg_list[isp->get_cfg_count - 1] + MAX_ISP_CHANNEL; // 12 ~ 23
	video->isp = isp;
	video->enabled = true; // will be updated from config
	video->chn_index = channel_cfg_list[isp->get_cfg_count - 1];
	video->video_index = video_sn;
	video->channel = &(isp->channels[channel_cfg_list[isp->get_cfg_count - 1]]);
	update_raw_video_from_camera_config(
		video,
		isp->channels[channel_cfg_list[isp->get_cfg_count - 1]].cam_dev);

	return 0;
}

static bool is_valid_cmd_addr(struct c1200_isp_device *isp, void *mc)
{
	tSoneCmdp *cmdp;
	void *fmc_start;
	size_t fmc_size;

	cmdp = (tSoneCmdp *)isp->cmdp_vaddr;
	fmc_start = (void *)cmdp->ch[FW_CH_INDEX].cqueue.c0;
	fmc_size = sizeof(cmdp->ch[FW_CH_INDEX].cqueue.c0);
	if (mc < fmc_start || mc >= (fmc_start + fmc_size))
		return false;

	return true;
}

static int ispdrv_message_handler(struct c1200_isp_device *isp,
				  struct media_command *cmd,
				  int64_t ipc_timestamp)
{
	int channel_id = 0;
	uint16_t message = cmd->cmd_hdr.hdr_info.cmd_type_minor;

	if (!is_valid_cmd_addr(isp, cmd)) {
		dev_err(isp->dev,
			"Illegal message, DMA: 0x%08X, CPU: 0x%016llX\n",
			isp_cmd_pa(isp, cmd), (long long)cmd);
		return -1;
	}

	dev_dbg(isp->dev, "recv command 0x%x in state %d\n", message,
		isp->state);

	switch (isp->state) {
	// place ISPDRV_STATE_WORK at first to enhance efficiency
	case ISPDRV_STATE_WORK:
		if (message == MINOR_ISP_GET_IQINFO ||
		    message == MINOR_ISP_SET_IQINFO ||
		    message == MINOR_ISP_SET_VIEWINFO) {
			isp_get_iqinfo_t *ctrlinfo =
				(isp_get_iqinfo_t *)&(cmd->user_cmd_data[0]);
			switch (message) {
			case MINOR_ISP_GET_IQINFO:
				pr_info("MINOR_ISP_GET_IQINFO RECV: sensor[%d] item[%d]: value:[%d]\n",
					ctrlinfo->sensorIndex, ctrlinfo->iqItem,
					ctrlinfo->iqVal);
				channel_id = ctrlinfo->sensorIndex;
				if (channel_id >= MAX_ISP_CHANNEL) {
					pr_err("invaild channel_id\n");
					return 1;
				}
				isp->ctrl_get_val = ctrlinfo->iqVal;
				isp->isp_ctrl_status = channel_id;
				wake_up(&isp->ctrl_recv_wq);
				break;
			case MINOR_ISP_SET_IQINFO:
				pr_info("MINOR_ISP_SET_IQINFO ACK: sensor[%d] item[%d]\n",
					ctrlinfo->sensorIndex,
					ctrlinfo->iqItem);
				break;
			case MINOR_ISP_SET_VIEWINFO:
				pr_info("MINOR_ISP_SET_VIEWINFO Frimware ACK\n");
				break;
			}
		} else if (message == MINOR_ISP_I2C_BUS_CTRL) {
			isp_i2c_bus_ctrl_t *i2c_ctrl;

			i2c_ctrl =
				(isp_i2c_bus_ctrl_t *)&(cmd->user_cmd_data[0]);
			dev_dbg(isp->dev, "%s: status: %u, i2cBusRelease: %u\n",
				__func__, i2c_ctrl->status,
				i2c_ctrl->i2cBusRelease);
			if (i2c_ctrl->status == 0)
				isp->i2c_ctrl = i2c_ctrl->i2cBusRelease;
			complete_all(&isp->i2c_ctrl_completion);
		} else if (message == MINOR_ISP_CAM_OPEN ||
			   message == MINOR_ISP_CAM_CLOSE) {
			stream_message_handler(isp, cmd);
		} else if (message == MINOR_SYNC_ISP_VIEW_FRAME_DONE ||
			   message == MINOR_ISP_RAW_BUF_DONE) {
			unsigned int follow_pack_num;
			unsigned int i;
			tSoneCmdp *cmdp;
			struct media_command *queue_tail;

			follow_pack_num = cmd->cmd_hdr.hdr_info.follow_pack_num;
			if (follow_pack_num)
				dev_dbg(isp->dev,
					"ISP MSG: follow_pack_num: %u\n",
					follow_pack_num);
			i = 0;
			cmdp = (tSoneCmdp *)isp->cmdp_vaddr;
			queue_tail =
				&cmdp->ch[FW_CH_INDEX].cqueue.c0[0] +
				ARRAY_SIZE(cmdp->ch[FW_CH_INDEX].cqueue.c0);
			do {
				if (cmd >= queue_tail) {
					dev_dbg(isp->dev,
						"ISP MSG: cmd %p exceed tail %p, rewind\n",
						cmd, queue_tail);
					cmd = &cmdp->ch[FW_CH_INDEX]
						       .cqueue.c0[0];
				}
				dev_dbg(isp->dev,
					"ISP MSG: %d CMD SRC: %c%c%c%c, Userdata: 0x%08X, 0x%08X, 0x%08X, 0x%08X\n",
					i, cmd->cmd_hdr.src[0],
					cmd->cmd_hdr.src[1],
					cmd->cmd_hdr.src[2],
					cmd->cmd_hdr.src[3],
					cmd->user_cmd_data[0],
					cmd->user_cmd_data[1],
					cmd->user_cmd_data[2],
					cmd->user_cmd_data[3]);
				work_message_handler(isp, cmd, ipc_timestamp);
				i++;
				cmd++;
			} while (i <= follow_pack_num);
		} else {
			dev_err_ratelimited(
				isp->dev,
				"unexpected message 0x%x in state %d\n",
				message, isp->state);
		}
		break;
	case ISPDRV_STATE_WAIT:
		if (message == MINOR_BOOT_DONE) {
			uint32_t fw_cmd = cmd->user_cmd_data[0];
			uint8_t chip_type = fw_cmd & 0xFF;
			uint8_t core_status = (fw_cmd >> 8) & 0xFF;

			dev_info(isp->dev, "fw_cmd: 0x%08X\n", fw_cmd);
			dev_info(isp->dev, "chip_type: 0x%02X\n", chip_type);
			dev_info(isp->dev, "core_status: 0x%02X\n",
				 core_status);

			isp->core_status = core_status;
			atomic_set(&(isp->FW_boot_done), 1);
			if (is_channel_cfg_left(isp))
				ispdrv_enter_cfg_state(isp);
			else if (is_camera_info_left(isp))
				ispdrv_get_cfg_state(isp);
			else
				ispdrv_enter_start_state(isp);
		} else {
			ispdrv_state_error(isp, message);
		}
		break;
	case ISPDRV_STATE_SCFG:
		if (message == MINOR_ISP_BOOTLD_RECONF) {
			if (is_channel_cfg_left(isp))
				send_next_cfg_to_fw(isp);
			else if (is_channel_alg_left(isp))
				ispdrv_enter_alg_state(isp);
			else if (is_channel_iq_left(isp))
				ispdrv_enter_iq_state(isp);
			else if (is_camera_info_left(isp))
				ispdrv_get_cfg_state(isp);
			else
				ispdrv_enter_start_state(isp);
		} else {
			ispdrv_state_error(isp, message);
		}
		break;
	case ISPDRV_STATE_SALG:
		if (message == MINOR_ISP_BOOTLD_ALGO_BIN) {
			if (is_channel_alg_left(isp))
				send_next_alg_to_fw(isp);
			else if (is_channel_iq_left(isp))
				ispdrv_enter_iq_state(isp);
			else if (is_camera_info_left(isp))
				ispdrv_get_cfg_state(isp);
			else
				ispdrv_enter_start_state(isp);
		} else {
			ispdrv_state_error(isp, message);
		}
		break;
	case ISPDRV_STATE_SIQ:
		if (message == MINOR_ISP_BOOTLD_IQ_BIN) {
			if (is_channel_iq_left(isp))
				send_next_iq_to_fw(isp);
			else if (is_camera_info_left(isp))
				ispdrv_get_cfg_state(isp);
			else
				ispdrv_enter_start_state(isp);
		} else {
			ispdrv_state_error(isp, message);
		}
		break;
	case ISPDRV_STATE_GET_SCFG:
		if (message == MINOR_GET_CAMERA_INFO) {
			c1200_parse_camera_info(isp, cmd);
			if (is_camera_info_left(isp))
				get_next_fmt_from_fw(isp);
			else
				ispdrv_enter_start_state(isp);
		} else {
			ispdrv_state_error(isp, message);
		}
		break;
	case ISPDRV_STATE_START:
		if (message == MINOR_ISP_START) {
			dev_info(isp->dev,
				 "ISPDRV_STATE_START, MINOR_ISP_START\n");
			ispdrv_enter_work_state(isp);
		} else {
			ispdrv_state_error(isp, message);
		}
		break;
	case ISPDRV_STATE_SHUTDOWN:
		dev_warn(isp->dev, "Recv command 0x%x in state shutdown, drop it\n", message);
		break;
	}
	
	return 0;
}

static int isp_kthread_recv_msg(void *data)
{
	struct c1200_isp_device *isp = (struct c1200_isp_device *)data;
	ipc_msg msg;
	int32_t ret;
	struct media_command *media_msg;
	int32_t timeout;
	atomic_set_release(&isp->running, 1);

	while (atomic_read(&isp->running)) {
		isp->kthread_status = KTHREAD_STATUS_RECV_MSG;
		timeout = -1;

		ret = ipc_recv(isp->ipc_session_id, &msg, timeout);
		if (ret < 0) {
			if (get_isp_streamon_count(isp) != 0) {
				dev_err(isp->dev,
					"ipc_recv failed!!!, ret = %d\n", ret);
			}
			continue;
		}
		isp->ipc_rx_count++;
		isp->kthread_status = KTHREAD_STATUS_RECV_COMPLETE;
		media_msg = isp_cmd_va(isp, msg.data);
		ispdrv_message_handler(isp, media_msg, msg.timestamp);
	}

	dev_info(isp->dev, "%s: exit\n", __func__);

	return 0;
}

static int send_start_msg_to_fw(struct c1200_isp_device *isp)
{
	u32 media_cmd_paddr;
	struct media_command *media_cmd;
	isp_start_t *start_cmd;
	ipc_msg msg;
	int ret;

	media_cmd = isp_get_media_cmd(isp);
	media_cmd_paddr = isp_cmd_pa(isp, media_cmd);
	pr_err("start media_cmd_paddr = 0x%08X, vaddr:0x%016llX\n",
	       media_cmd_paddr, (long long)media_cmd);
	memset(media_cmd, 0, sizeof(struct media_command));
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_START;
	start_cmd = (isp_start_t *)(&media_cmd->user_cmd_data[0]);
	start_cmd->reconfDDRBase = isp->fbuf_paddr;
	start_cmd->reconfDDRSize = isp->fbuf_psize;
	start_cmd->txMsgMode = AttachMsg_TimeOut;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = media_cmd_paddr;
	ret = send_cmd_to_fw(isp, &msg);

	return 0;
}

static int send_config_to_fw(struct c1200_isp_device *isp,

			     struct camera_dev *cam_dev, int chn_index)
{
	u32 media_cmd_paddr;
	struct media_command *media_cmd;
	u8 *payload_vaddr;
	u32 payload_paddr;
	isp_ld_reconf_t *config_cmd;
	ipc_msg msg;
	int ret;

	media_cmd = isp_get_media_cmd(isp);
	media_cmd_paddr = isp_cmd_pa(isp, media_cmd);

	payload_vaddr = isp->config_payload_vaddr +
			(isp->config_align_size * chn_index);
	payload_paddr = isp->config_payload_paddr +
			(isp->config_align_size * chn_index);
	memcpy(payload_vaddr, &(cam_dev->isp_data), sizeof(ipc_reconf_t));
	memset(media_cmd, 0, sizeof(struct media_command));
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_BOOTLD_RECONF;
	config_cmd = (isp_ld_reconf_t *)(&media_cmd->user_cmd_data[0]);
	config_cmd->sensorIndex = cam_dev->isp_data.sensorIndex;
	config_cmd->payloadAddr = payload_paddr;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = media_cmd_paddr;
	ret = send_cmd_to_fw(isp, &msg);

	return ret;
}

static int get_config_from_fw(struct c1200_isp_device *isp,
			      struct camera_dev *cam_dev, int chn_index)
{
	u32 media_cmd_paddr;
	struct media_command *media_cmd;
	u8 *payload_vaddr;
	u32 payload_paddr;
	isp_ld_reconf_t *config_cmd;
	ipc_msg msg;
	int ret;

	media_cmd = isp->config_media_cmd;
	media_cmd_paddr = isp->media_cmd_paddr;

	payload_vaddr = isp->config_payload_vaddr +
			(isp->config_align_size * chn_index);
	payload_paddr = isp->config_payload_paddr +
			(isp->config_align_size * chn_index);
	memcpy(payload_vaddr, &(cam_dev->isp_data), sizeof(ipc_reconf_t));
	memset(media_cmd, 0, sizeof(struct media_command));
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_GET_CAMERA_INFO;
	config_cmd = (isp_ld_reconf_t *)(&media_cmd->user_cmd_data[0]);
	config_cmd->sensorIndex = cam_dev->isp_data.sensorIndex;
	config_cmd->payloadAddr = payload_paddr;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = media_cmd_paddr;
	ret = send_cmd_to_fw(isp, &msg);

	return ret;
}

static int send_next_cfg_to_fw(struct c1200_isp_device *isp)
{
	int index;

	index = next_channel_cfg_index(isp);
	if (index >= 0) {
		send_config_to_fw(isp, isp->channels[index].cam_dev, index);
		isp->cfg_count++;
	}

	return 0;
}

static int get_next_fmt_from_fw(struct c1200_isp_device *isp)
{
	int index;

	index = channel_cfg_list[isp->get_cfg_count];
	if (!isp->channels[index].enable)
		return -1;

	if (index >= 0) {
		get_config_from_fw(isp, isp->channels[index].cam_dev, index);
		isp->get_cfg_count++;
	}

	return 0;
}

static struct bin_info_t *get_bin_info(int chn_index, const char *filename)
{
	int i;
	int ret;

	for (i = 0; i < s_isp_algobin.file_num; i++) {
		ret = strncmp(filename, s_isp_algobin.algo_bin[i].filename,
			      MAX_BIN_NAME_LEN);
		if (ret == 0) {
			s_isp_algobin.algo_bin[i].camera_mask |= BIT(chn_index);
			pr_debug("find, mask = 0x%x, name = %s, addr = 0x%x\n",
				 s_isp_algobin.algo_bin[i].camera_mask,
				 s_isp_algobin.algo_bin[i].filename,
				 s_isp_algobin.algo_bin[i].payload_paddr);
			return &s_isp_algobin.algo_bin[i];
		}
	}

	return NULL;
}

static int add_bin_info_to_array(struct bin_info_t *bin_info, int chn_index,
				 const char *filename)
{
	struct bin_info_t *info;
	int index;

	if (s_isp_algobin.file_num >= MAX_ISP_CHANNEL) {
		pr_err("algo file exceeds MAX_ISP_CHANNEL, error\n");
		return -1;
	}
	index = s_isp_algobin.file_num;
	info = &s_isp_algobin.algo_bin[index];
	info->payload_vaddr = bin_info->payload_vaddr;
	info->payload_paddr = bin_info->payload_paddr;
	info->size = bin_info->size;
	strscpy(info->filename, filename, MAX_BIN_NAME_LEN);
	info->camera_mask |= BIT(chn_index);

	s_isp_algobin.file_num++;

	return 0;
}

static int send_algo_bin_to_fw(struct c1200_isp_device *isp, int chn_index,
			       const char *filename)
{
	int ret;
	const struct firmware *firmware_p;
	u32 media_cmd_paddr;
	struct media_command *media_cmd;
	isp_ld_algo_bin_t *config_cmd;
	ipc_msg msg;
	u32 file_start_paddr;
	u8 *file_start;
	u8 *file_end;
	int file_size;
	char full_name[MAX_BIN_NAME_LEN];

	struct bin_info_t *bin_info;

	bin_info = get_bin_info(chn_index, filename);
	if (bin_info != NULL) {
		file_start_paddr = bin_info->payload_paddr;
	} else {
		struct bin_info_t info;

		memset(full_name, 0, MAX_BIN_NAME_LEN);
		snprintf(full_name, MAX_BIN_NAME_LEN, "isp/algo/%s", filename);
		pr_debug("full_name = %s\n", full_name);
		ret = request_firmware(&firmware_p, full_name, isp->dev);
		if (ret) {
			pr_err("there is no %s could be used\n", full_name);
			return -1;
		}

		file_start = isp->algobin_next_vaddr;
		file_start_paddr = isp->algobin_next_paddr;
		file_size = firmware_p->size + 1; // one byte for eof
		if ((file_start + file_size) > isp->payload_end_vaddr) {
			pr_err("bin file %s size = %d exceed\n", full_name,
			       file_size);
			return -1;
		}

		memcpy(file_start, firmware_p->data, firmware_p->size);
		file_end = file_start + firmware_p->size;
		*file_end = 0xa;

		info.payload_vaddr = file_start;
		info.payload_paddr = file_start_paddr;
		info.size = file_size;
		add_bin_info_to_array(&info, chn_index, filename);
		file_size = (file_size + ISP_MSG_PAYLOAD_SIZE_MASK) &
			    (~ISP_MSG_PAYLOAD_SIZE_MASK);
		isp->algobin_next_vaddr += file_size;
		isp->algobin_next_paddr += file_size;

		release_firmware(firmware_p);
	}

	media_cmd = isp_get_media_cmd(isp);
	media_cmd_paddr = isp_cmd_pa(isp, media_cmd);
	memset(media_cmd, 0, sizeof(struct media_command));
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_BOOTLD_ALGO_BIN;
	config_cmd = (isp_ld_algo_bin_t *)(&media_cmd->user_cmd_data[0]);
	config_cmd->sensorIndex = isp->channels[chn_index].isp_chn_id;
	config_cmd->payloadAddr = file_start_paddr;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = media_cmd_paddr;
	ret = send_cmd_to_fw(isp, &msg);

	return 0;
}

static int send_iq_bin_to_fw(struct c1200_isp_device *isp, int chn_index,
			     const char *name)
{
	int ret;
	const struct firmware *fw;
	struct media_command *media_cmd;
	isp_ld_iq_bin_t *iq_bin;
	ipc_msg msg;
	char path[MAX_BIN_NAME_LEN];

	memset(path, 0, MAX_BIN_NAME_LEN);
	snprintf(path, MAX_BIN_NAME_LEN, "isp/algo/%s", name);

	ret = request_firmware(&fw, path, isp->dev);
	if (ret) {
		pr_err("request firmware %s failed\n", name);
		return -1;
	}

	if ((isp->config_payload_vaddr + fw->size) > isp->payload_end_vaddr) {
		pr_err("%s size %ld, exceed\n", name, fw->size);
		return -1;
	}
	memcpy(isp->config_payload_vaddr, fw->data, fw->size);

	media_cmd = isp_get_media_cmd(isp);
	memset(media_cmd, 0, sizeof(struct media_command));
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_BOOTLD_IQ_BIN;
	iq_bin = (isp_ld_iq_bin_t *)(media_cmd->user_cmd_data);
	iq_bin->sensorIndex = isp->channels[chn_index].isp_chn_id;
	iq_bin->payloadAddr = isp->config_payload_paddr;
	iq_bin->payloadSize = fw->size;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = isp_cmd_pa(isp, media_cmd);

	pr_debug("%s, channel %d, path %s, size %ld\n", __func__, chn_index,
		 path, fw->size);

	send_cmd_to_fw(isp, &msg);

	release_firmware(fw);
	return 0;
}

static int send_next_alg_to_fw(struct c1200_isp_device *isp)
{
	int index = next_channel_alg_index(isp);

	if (index >= 0) {
		const char *filename = isp->channels[index].cam_dev->algo;

		send_algo_bin_to_fw(isp, index, filename);
		isp->alg_count++;
	}

	return 0;
}

static int send_next_iq_to_fw(struct c1200_isp_device *isp)
{
	int index = next_channel_iq_index(isp);

	if (index >= 0) {
		const char *filename = isp->channels[index].cam_dev->iq;

		send_iq_bin_to_fw(isp, index, filename);
		isp->iq_count++;
	}

	return 0;
}

static int isp_channel_get_port_info(struct c1200_isp_device *isp_dev,
				     struct device_node *core_dt, int core_id)
{
	struct device_node *ep = NULL;
	struct device_node *remote_port = NULL;
	struct device_node *remote_node = NULL;
	struct device_node *port = NULL;
	int sn;
	int i;

	struct bst_isp_channel *channel;

	for (i = 0; i < MAX_CHANNEL_PER_CORE; i++) {
		sn = core_id * MAX_CHANNEL_PER_CORE + i;
		channel = &isp_dev->channels[sn];
		channel->sn = sn;
		channel->isp_core_id = core_id;
		channel->index_in_core = i;
		channel->isp_chn_id = ((core_id << 2) | i);
		channel->isp = isp_dev;
		port = of_graph_get_port_by_id(core_dt, i);
		if (port == NULL)
			continue;

		channel->of_node = port;
		channel->fwnode = of_fwnode_handle(port);
		dev_dbg(isp_dev->dev,
			"channel %d , port = %s, fwnode = 0x%lx\n", sn,
			port->full_name, (unsigned long)channel->fwnode);

		ep = of_get_child_by_name(port, "endpoint");
		if (ep) {
			int ret;
			int reg;
			int id;

			remote_port = of_graph_get_remote_port(ep);
			if (!remote_port) {
				pr_err("no valid remote port\n");
				channel->enable = false;
				continue;
			}
			channel->remote_fwnode = of_fwnode_handle(remote_port);
			dev_dbg(isp_dev->dev,
				"channel %d , remote port = %s, remote_fwnode = 0x%lx\n",
				sn, remote_port->full_name,
				(unsigned long)channel->remote_fwnode);

			ret = of_property_read_u32(remote_port, "reg", &reg);
			if (ret < 0) {
				dev_err(isp_dev->dev, "remote reg error\n");
				return -EINVAL;
			}
			remote_node = of_graph_get_remote_port_parent(ep);
			// of_node_put(endpoint_node);
			if (!remote_node) {
				pr_err("no valid remote node\n");
				return -EINVAL;
			}
			dev_dbg(isp_dev->dev, "channel %d , remote_node = %s\n",
				sn, remote_node->full_name);

			ret = of_property_read_u32(remote_node, "id", &id);
			if (ret < 0) {
				dev_err(isp_dev->dev, "remote id error\n");
				return -EINVAL;
			}
			channel->enable = true;
			channel->remote_mipi_id = id;
			channel->remote_mipi_vc_index = reg;

			if (isp_dev->csi_asd[id].mipi_connected == 0) {
				isp_dev->csi_asd[id].mipi_connected++;
				isp_dev->total_subdev++;
				isp_dev->csi_asd[id].mipi_fwnode =
					of_fwnode_handle(remote_node);
			} else if (isp_dev->csi_asd[id].mipi_fwnode ==
				   of_fwnode_handle(remote_node)) {
				isp_dev->csi_asd[id].mipi_connected++;
			} else {
				pr_err("error, same id, but not same node\n");
				return -1;
			}

			isp_dev->core_channel_num[core_id]++;

			dev_dbg(isp_dev->dev,
				"remote node name = %s, remote reg = %d, id = %d, fwnode = 0x%lx\n",
				remote_node->full_name, reg, id,
				(unsigned long)isp_dev->csi_asd[id].mipi_fwnode);
		}
	}

	return isp_dev->core_channel_num[core_id];
}

static int parse_dt(struct c1200_isp_device *isp_dev)
{
	int ret;
	int num_channels;
	u32 core_id;
	struct device_node *node = isp_dev->dev->of_node;
	struct device_node *core_dt = NULL;

	if (!node)
		return -EINVAL;

	isp_dev->use_ipc = of_property_read_bool(node, "use-ipc");
	if (isp_dev->use_ipc)
		pr_info("isp message use IPC enable\n");
	else
		pr_info("isp message use MSGBOX enable\n");

	isp_dev->rstc =
		devm_reset_control_get_shared(isp_dev->dev, "isp-reset");
	if (IS_ERR(isp_dev->rstc)) {
		dev_err(isp_dev->dev, "reset is not defined\n");
		return -EINVAL;
	}

	isp_dev->fw_bin = ISP_FW_BIN_PATH;
	isp_dev->fw_slab = ISP_FW_SLAB_PATH;
	of_property_read_string(node, "fw-bin", &isp_dev->fw_bin);
	of_property_read_string(node, "slab", &isp_dev->fw_slab);
	pr_info("isp fw bin name: %s\n", isp_dev->fw_bin);
	for_each_child_of_node(node, core_dt) {
		if (!core_dt->name || of_node_cmp(core_dt->name, "core"))
			continue;
		ret = of_property_read_u32(core_dt, "id", &core_id);
		if (ret < 0)
			return -EINVAL;

		num_channels =
			isp_channel_get_port_info(isp_dev, core_dt, core_id);
		if (num_channels < 0) {
			pr_err("isp_channel_get_port_info error\n");
			return -1;
		}
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Media Operations
 */
static const struct media_entity_operations isp_entity_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int isp_dev_notify_bound(struct v4l2_async_notifier *async_notifier,
				struct v4l2_subdev *sd,
				struct v4l2_async_subdev *asd)
{
	struct bst_csi_device *pcsi_dev;
	struct csi_async_dev *pcsi_async;
	struct c1200_isp_device *pisp_dev;
	struct bst_isp_channel *isp_channel;
	int i;

	pisp_dev =
		container_of(async_notifier, struct c1200_isp_device, notifier);
	if (sd->fwnode == pisp_dev->remote_hdmi) {
		pisp_dev->hdmi_cam =
			container_of(sd, struct camera_dev, subdev);
	} else {
		pcsi_dev = container_of(sd, struct bst_csi_device, subdev);
		pcsi_async = container_of(asd, struct csi_async_dev, async_dev);
		pcsi_dev->sd_state = BST_SUBDEV_STATE_BOUND;
		pcsi_async->csi_dev = pcsi_dev;

		for (i = 0; i < MAX_ISP_CHANNEL; i++) {
			isp_channel = &pisp_dev->channels[i];
			pr_debug(
				"remote_mipi_id = %d, vc_index = %d, pcsi_dev->csi_id = %d\n",
				isp_channel->remote_mipi_id,
				isp_channel->remote_mipi_vc_index,
				pcsi_dev->csi_id);

			if (isp_channel->remote_mipi_id == pcsi_dev->csi_id) {
				int vc_index;

				vc_index = isp_channel->remote_mipi_vc_index;
				isp_channel->csi_channel =
					&(pcsi_dev->csi_vc[vc_index]);
				pr_debug("isp_channel->csi_channel = 0x%p\n",
					 isp_channel->csi_channel);
			} else {
				continue;
			}
		}
	}

	return 0;
}

static void isp_dev_notify_unbind(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_subdev *asd)
{
}

static int isp_dev_notify_complete(struct v4l2_async_notifier *async_notifier)
{
	struct c1200_isp_device *isp_dev;
	int ret;

	ret = v4l2_device_register_subdev_nodes(async_notifier->v4l2_dev);
	isp_dev =
		container_of(async_notifier, struct c1200_isp_device, notifier);

	return 0;
};

static const struct v4l2_async_notifier_operations isp_dev_async_ops = {
	.bound = isp_dev_notify_bound,
	.unbind = isp_dev_notify_unbind,
	.complete = isp_dev_notify_complete,
};

static int init_isp_one_channel(struct bst_isp_channel *isp_channel, int index)
{
	int ret;

	isp_channel->is_hdmi = false;
	isp_channel->pads[ISP_CHANNEL_SINK_PAD].flags = MEDIA_PAD_FL_SINK;
	isp_channel->pads[ISP_CHANNEL_SOURCE_NORMAL].flags =
		MEDIA_PAD_FL_SOURCE;
	isp_channel->pads[ISP_CHANNEL_SOURCE_PDNS].flags = MEDIA_PAD_FL_SOURCE;
	atomic_set(&isp_channel->is_streaming, 0);
	isp_channel->entity.function = MEDIA_ENT_F_IO_V4L;
	isp_channel->entity.ops = &isp_entity_media_ops;

	ret = media_entity_pads_init(&isp_channel->entity, ISP_CHANNEL_PAD_NUM,
				     isp_channel->pads);
	if (ret < 0)
		return ret;

	return 0;
}

static int init_isp_channel_devs(struct c1200_isp_device *isp_dev)
{
	int i;
	struct bst_isp_channel *isp_channel;

	for (i = 0; i < MAX_ISP_CHANNEL; i++) {
		isp_channel = &isp_dev->channels[i];
		if (!isp_channel->enable) {
			dev_info(isp_dev->dev,
				 "%s channel %d not enabled, skip\n", __func__,
				 i);
			continue;
		}
		init_isp_one_channel(isp_channel, i);
	}

	return 0;
}

static void c1200_isp_v4l2_dev_notify(struct v4l2_subdev *sd,
				      unsigned int notification, void *arg)
{
	struct c1200_isp_device *isp;
	struct camera_dev *cam_dev;
	struct bst_isp_channel *isp_chn;
	int bus_index = -1;
	int i;

	if (sd == NULL)
		return;

	cam_dev = (struct camera_dev *)arg;
	if (cam_dev == NULL) {
		dev_dbg(isp->dev, "cam_dev is NULL\n");
		return;
	}
	bus_index = cam_dev->i2c_client->adapter->nr;
	isp = container_of(sd->v4l2_dev, struct c1200_isp_device, v4l2_dev);
	if (isp->state != ISPDRV_STATE_WORK) {
		dev_dbg(isp->dev, "ISP is not in work state, break\n");
		return;
	}
	dev_dbg(isp->dev, "%s: sd: %s, notification: %u, cam_dev: 0x%pK\n",
		__func__, sd->name, notification, cam_dev);

	switch (notification) {
	case MAXIM_DESER_LINK_HOTPLUG_START:
		send_i2c_ctrl_msg_to_fw(isp, bus_index, I2C_BUS_ARM);
		break;
	case MAXIM_DESER_LINK_HOTPLUG_STOP:
		send_i2c_ctrl_msg_to_fw(isp, bus_index, I2C_BUS_ISP);
		break;
	case MAXIM_DESER_LINK_DISCONNECT:
		for (i = 0; i < ARRAY_SIZE(isp->channels); i++) {
			dev_info(isp->dev, "%s: cam_dev[%d]: 0x%pK\n", __func__,
				 i, isp->channels[i].cam_dev);
			if (isp->channels[i].cam_dev == cam_dev) {
				dev_info(isp->dev,
					 "%s: found disconnected cam_dev\n",
					 __func__);
				isp->channels[i].cam_dev->power_on = false;
				isp_chn = &isp->channels[i];
				notify_fw_cam_plugout(&isp_chn->views_video);
			}
		}
		break;
	case MAXIM_DESER_LINK_CONNECT:
		for (i = 0; i < ARRAY_SIZE(isp->channels); i++) {
			dev_info(isp->dev, "%s: cam_dev[%d]: 0x%pK\n", __func__,
				 i, isp->channels[i].cam_dev);
			if (isp->channels[i].cam_dev == cam_dev) {
				dev_info(isp->dev,
					 "%s: found connected cam_dev\n",
					 __func__);
				isp->channels[i].cam_dev->power_on = true;
				isp_chn = &isp->channels[i];
				cam_dev->subdev.ops->core->s_power(
					&cam_dev->subdev, 1);
				notify_fw_cam_plugin(&isp_chn->views_video);
			}
		}
		break;
	default:
		break;
	}
}

static int init_isp_dev(struct c1200_isp_device *isp_dev)
{
	int i;
	int ret;

	isp_dev->media_dev.dev = isp_dev->dev;
	strscpy(isp_dev->media_dev.model, "BST C1200 ISP",
		sizeof(isp_dev->media_dev.model));
	isp_dev->media_dev.ops = &isp_media_ops;
	media_device_init(&isp_dev->media_dev);
	ret = media_device_register(&isp_dev->media_dev);
	if (ret < 0) {
		dev_err(isp_dev->dev,
			"%s: Media device registration failed (%d)\n", __func__,
			ret);
		return -1;
	}

	isp_dev->v4l2_dev.mdev = &isp_dev->media_dev;

	snprintf(isp_dev->v4l2_dev.name, sizeof(isp_dev->v4l2_dev.name),
		 "c1200_isp_v4l2");

	ret = v4l2_device_register(isp_dev->dev, &isp_dev->v4l2_dev);
	if (ret < 0) {
		dev_err(isp_dev->dev,
			"%s: V4L2 device registration failed (%d)\n", __func__,
			ret);
		return -1;
	}

	isp_dev->v4l2_dev.notify = c1200_isp_v4l2_dev_notify;

	v4l2_async_nf_init(&isp_dev->notifier);
	isp_dev->notifier.ops = &isp_dev_async_ops;

	for (i = 0; i < MAX_MIPI_DEVICE_NUM; i++) {
		isp_dev->csi_asd[i].isp_parent = isp_dev;
		if (isp_dev->csi_asd[i].mipi_fwnode == NULL) {
			pr_debug("i = %d, mipi not connected\n", i);
			continue;
		}

		isp_dev->csi_asd[i].async_dev.match_type =
			V4L2_ASYNC_MATCH_FWNODE;
		isp_dev->csi_asd[i].async_dev.match.fwnode =
			isp_dev->csi_asd[i].mipi_fwnode;
		__v4l2_async_nf_add_subdev(&isp_dev->notifier,
					   &(isp_dev->csi_asd[i].async_dev));
	}

	if (isp_dev->remote_hdmi != NULL) {
		isp_dev->hdmi_async.match_type = V4L2_ASYNC_MATCH_FWNODE;
		isp_dev->hdmi_async.match.fwnode = isp_dev->remote_hdmi;
		__v4l2_async_nf_add_subdev(&isp_dev->notifier,
					   &(isp_dev->hdmi_async));
	}
	ret = v4l2_async_nf_register(&isp_dev->v4l2_dev, &(isp_dev->notifier));
	if (ret < 0)
		dev_err(isp_dev->dev,
			"v4l2_async_nf_register register failed\n");

	return 0;
}

static int init_isp_one_channel_videos(struct c1200_isp_device *isp_dev,
				       struct bst_isp_channel *isp_channel,
				       int chn_id)
{
	int ret;
	struct c1200_isp_video *video;

	// init views_video
	video = &(isp_channel->views_video);
	isp_channel_init_video(isp_dev, isp_channel, VIEW_VIDEO, chn_id);
	ret = c1200_isp_video_register(video, &isp_dev->v4l2_dev, VIEW_VIDEO);
	if (ret < 0)
		dev_info(isp_dev->dev, "%s register view video failed\n",
			 __func__);

	// init raw video
	video = &(isp_channel->raw_video);
	isp_channel_init_video(isp_dev, isp_channel, RAW_VIDEO, chn_id);
	ret = c1200_isp_video_register(video, &isp_dev->v4l2_dev, RAW_VIDEO);
	if (ret < 0) {
		dev_err(isp_dev->dev, "%s register raw video failed\n",
			__func__);
		return -1;
	}

	return 0;
}

static int init_isp_videos(struct c1200_isp_device *isp_dev)
{
	int i;
	struct bst_isp_channel *isp_channel;
	int payload_size;

	payload_size = ISP_CTRL_PAYLOAD_SIZE * MAX_ISP_CHANNEL + 1;
	payload_size = (payload_size + ISP_MSG_PAYLOAD_SIZE_MASK) &
		       (~ISP_MSG_PAYLOAD_SIZE_MASK);
	if ((isp_dev->algobin_next_vaddr + payload_size) >
	    isp_dev->payload_end_vaddr) {
		pr_err("ipc_iqinfo_t size %d, exceed\n", payload_size);
		return -1;
	}

	for (i = 0; i < MAX_ISP_CHANNEL; i++) {
		isp_channel = &isp_dev->channels[i];
		isp_channel->ctrl_payload_vaddr =
			isp_dev->algobin_next_vaddr + i * ISP_CTRL_PAYLOAD_SIZE;
		isp_channel->ctrl_payload_paddr =
			isp_dev->algobin_next_paddr + i * ISP_CTRL_PAYLOAD_SIZE;

		if (!isp_channel->enable) {
			pr_debug("%s channel %d not enabled, skip\n", __func__,
				 i);
			continue;
		}
		init_isp_one_channel_videos(isp_dev, isp_channel, i);
	}

	isp_dev->algobin_next_vaddr += payload_size;
	isp_dev->algobin_next_paddr += payload_size;

	return 0;
}

static int calc_core0_cameras(struct c1200_isp_device *isp)
{
	int i;
	int count = 0;

	for (i = 0; i < MAX_CHANNEL_PER_CORE; i++) {
		if (isp->channels[i].cam_dev) {
			if (isp->channels[i].enable)
				count++;
		}
	}

	return count;
}

int isp_power_subdevs(struct c1200_isp_device *isp, int enable)
{
	v4l2_device_call_all(&isp->v4l2_dev, 0, core, s_power, enable);

	return 0;
}

static int update_isp_channel_status(struct c1200_isp_device *isp)
{
	int i;
	int count;
	struct bst_isp_channel *isp_chn;

	for (i = 0; i < MAX_ISP_CHANNEL; i++) {
		isp_chn = &isp->channels[i];
		pr_err("isp_chn->enable = %d, isp_chn->csi_channel = 0x%p\n",
		       isp_chn->enable, isp_chn->csi_channel);
		if (isp_chn->enable) {
			if (isp_chn->csi_channel &&
			    isp_chn->csi_channel->cam_dev) {
				isp->total_channel++;
				isp_chn->cam_dev =
					isp_chn->csi_channel->cam_dev;

				if (isp_chn->cam_dev) {
					isp_chn->camera_raw_width =
						isp_chn->cam_dev->isp_data
							.rawinfo.width;
					isp_chn->camera_raw_height =
						isp_chn->cam_dev->isp_data
							.rawinfo.height;
				}

				if (!isp_chn->cam_dev->power_on)
					isp->cfg_plugout_num++;

				isp_chn->cam_dev->isp_data.mipiSensorIndex =
					isp_chn->csi_channel->csi_chn_id;
				isp_chn->cam_dev->isp_data.sensorIndex =
					isp_chn->isp_chn_id;
				isp->cfg_num++;
				if (is_channel_use_alg(isp, i))
					isp->alg_num++;
				if (is_channel_use_iq(isp, i))
					isp->iq_num++;
			} else {
				isp_chn->enable = false;
			}
		}
	}

	count = calc_core0_cameras(isp);
	if ((count == 0) && (isp->hdmi_cam != NULL))
		isp->hdmi_detected = true;

	isp->core0_active_cam = count;

	if (isp->hdmi_detected) {
		isp->channels[0].cam_dev = isp->hdmi_cam;
		isp->channels[0].enable = true;
		isp->channels[0].is_hdmi = true;
		isp->total_channel++;
		isp->cfg_num++;
	}

	pr_err("cfg_num %d, alg_num %d unplug_num %d\n", isp->cfg_num,
	       isp->alg_num, isp->cfg_plugout_num);

	return 0;
}

static int update_camera_status(struct c1200_isp_device *isp)
{
	int i;
	struct bst_csi_device *pcsi_dev;

	for (i = 0; i < MAX_MIPI_DEVICE_NUM; i++) {
		pcsi_dev = isp->csi_asd[i].csi_dev;
		if (pcsi_dev)
			update_camera_status_in_csi(pcsi_dev);
	}
	update_isp_channel_status(isp);

	return 0;
}

int send_i2c_ctrl_msg_to_fw(struct c1200_isp_device *isp, int bus_index,
			    int i2c_bus_ctrler)
{
	u32 media_cmd_paddr;
	struct media_command *media_cmd;
	isp_i2c_bus_ctrl_t *i2c_ctrl_cmd;
	ipc_msg msg;
	int ret = 10000;
	unsigned long timeout;

	if (isp->state != ISPDRV_STATE_WORK)
		return 0;

	media_cmd = isp_get_media_cmd(isp);
	media_cmd_paddr = isp_cmd_pa(isp, media_cmd);
	memset(media_cmd, 0, sizeof(struct media_command));
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_I2C_BUS_CTRL;
	i2c_ctrl_cmd = (isp_i2c_bus_ctrl_t *)(&media_cmd->user_cmd_data[0]);

	if (bus_index == -1) {
		pr_err("bus_index is not correct!");
		return -EINVAL;
	}

	i2c_ctrl_cmd->i2cBusBase = bus_index * IIC_OFFSET + IIC_BASE;
	i2c_ctrl_cmd->i2cBusRelease = i2c_bus_ctrler; // 0:isp  1:arm
	i2c_ctrl_cmd->sensorBits = 0xff; // affect all videos' sensors

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = media_cmd_paddr;
	reinit_completion(&(isp->i2c_ctrl_completion));
	ret = send_cmd_to_fw(isp, &msg); // send ipc message to fw
	timeout = wait_for_completion_io_timeout(&(isp->i2c_ctrl_completion),
						 msecs_to_jiffies(1000));
	if (timeout == 0) {
		dev_err(isp->dev, "%s: timeout to wait I2C ctrl\n", __func__);
		return -EIO;
	}

	return ret;
}

void isp_boot_fw_api(struct c1200_isp_device *isp)
{
	int ret;
	int timeout;

	mutex_lock(&(isp->isp_mutex));
	if (atomic_read(&(isp->FW_config_done)) == 0) {
		timeout = 0;
		if (atomic_read(&(isp->FW_load_started)) == 0) {
			if (atomic_read(&(isp->FW_load_started)) == 0) {
				ret = bst_load_isp_fw(isp->fw_bin, isp->fw_slab,
						      isp);
				if (ret != 0) {
					dev_err(isp->dev,
						"bst_load_isp_fw failed\n");
					atomic_set(&(isp->FW_load_started), -1);
					mutex_unlock(&(isp->isp_mutex));
					return;
				}
				bst_start_isp_fw(isp);
				atomic_set(&(isp->FW_load_started), 1);
			}
		}
		dev_info(isp->dev, "Wait FW boot");
		timeout = wait_for_completion_io_timeout(
			&(isp->FW_start_completion), msecs_to_jiffies(10000));
		dev_info(isp->dev, "Wait FW boot done");
		if (timeout == 0) {
			dev_err(isp->dev, "open timeout");
			atomic_set(&(isp->FW_load_started), 0);
			mutex_unlock(&(isp->isp_mutex));
			return;
		}
	} else {
		dev_err(isp->dev, "Has been boot!");
	}
	mutex_unlock(&(isp->isp_mutex));
}

int isp_get_isp_id_by_video_id(struct c1200_isp_device *isp, int video_id)
{
	int i;

	for (i = 0; i < MAX_ISP_CHANNEL; i++) {
		struct c1200_isp_video *video = &isp->channels[i].views_video;

		if (video->enabled && video->device_index == video_id)
			return video->chn_index;
	}

	dev_err(isp->dev, "[ISP_FEED]: %s, failed to find video", __func__);
	return -1;
}

int isp_feed_drv_query_buff(struct c1200_isp_device *isp,
			    struct feed_drv_querybuffers *qry)
{
	int i = 0;
	int j = 0;
	int isp_ch_id = 0;
	int buf_count = 0;
	int view_index = 0;
	struct c1200_isp_video *video;
	struct vb2_queue *vb2_q;
	struct vb2_buffer *vb2_buf;
	struct vb2_plane *vb2_plane;
	struct vb2_dc_buf *vb2_dc_buf;

	dev_dbg(isp->dev, "[ISP_FEED]: video%d query buf information",
		qry->video_id);

	isp_ch_id = isp_get_isp_id_by_video_id(isp, qry->video_id);
	if (isp_ch_id >= MAX_ISP_CHANNEL || isp_ch_id < 0) {
		dev_err(isp->dev, "[ISP_FEED]: %s, error chid[%d]", __func__,
			isp_ch_id);
		return -EINVAL;
	}

	video = &isp->channels[isp_ch_id].views_video;
	if (!video) {
		dev_err(isp->dev, "[ISP_FEED]: c1200_isp_video is NULL");
		return -EINVAL;
	}

	if (!video->enabled) {
		dev_err(isp->dev, "[ISP_FEED]: video disabled");
		return -EINVAL;
	}

	vb2_q = video->queue;
	if (vb2_q == NULL) {
		qry->buf_num = 0;
		return 0;
	}

	buf_count = vb2_q->num_buffers;
	qry->buf_num = buf_count;
	for (i = 0; i < buf_count; i++) {
		vb2_buf = vb2_q->bufs[i];
		if (!vb2_buf) {
			dev_err(isp->dev, "[ISP_FEED]: vb2_buffer is NULL");
			return -EINVAL;
		}

		qry->bufs[i].index = i;

		for (j = 0; j < ISP_CHANNEL_VIEW_NUM; j++) {
			vb2_plane = &vb2_buf->planes[j];
			if (!vb2_plane) {
				dev_err(isp->dev,
					"[ISP_FEED]: vb2_plane is NULL");
				return -EINVAL;
			}

			if (vb2_plane->length == 0) {
				dev_dbg(isp->dev, "[ISP_FEED]: skip plane %d",
					j);
				continue;
			}

			vb2_dc_buf = (struct vb2_dc_buf *)vb2_plane->mem_priv;
			if (!vb2_dc_buf) {
				dev_err(isp->dev,
					"[ISP_FEED]: vb2_dc_buf is NULL");
				return -EINVAL;
			}
			view_index = video->current_views_list[j];
			qry->bufs[i].bytesused[view_index] = vb2_plane->length;
			qry->bufs[i].size[view_index] = vb2_dc_buf->size;
			if (isp->iommud)
				qry->bufs[i].paddr[view_index] = iommu_iova_to_phys(isp->iommud, vb2_dc_buf->dma_addr);
			else
				qry->bufs[i].paddr[view_index] = vb2_dc_buf->dma_addr;

			dev_dbg(isp->dev,
				"[ISP_FEED]: video%d, buf%d-%d, index = %d, size = %d, bytesused = %d, paddr = 0x%llx",
				qry->video_id, i, view_index,
				qry->bufs[i].index,
				qry->bufs[i].size[view_index],
				qry->bufs[i].bytesused[view_index],
				qry->bufs[i].paddr[view_index]);
		}
	}

	return 0;
}

int isp_feed_drv_query_free_buf(struct c1200_isp_device *isp,
				struct feed_drv_query_free_buf *qry)
{
	int ret = 0;
	unsigned int isp_ch_id = 0;
	struct isp_buffer *wait_buf;
	struct vb2_v4l2_buffer *wait_v4l2_buffer;
	struct vb2_buffer *wait_vb2_buffer;
	struct c1200_isp_video *video;

	dev_dbg(isp->dev, "[ISP_FEED]: video%d query wait buf", qry->video_id);

	isp_ch_id = isp_get_isp_id_by_video_id(isp, qry->video_id);
	if (isp_ch_id >= MAX_ISP_CHANNEL) {
		dev_err(isp->dev, "[ISP_FEED]: %s, error chid[%d]", __func__,
			isp_ch_id);
		return -EINVAL;
	}

	video = &isp->channels[isp_ch_id].views_video;
	if (!video) {
		dev_err(isp->dev, "[ISP_FEED]: c1200_isp_video is NULL");
		return -EINVAL;
	}

	while (1) {
		mutex_lock(&video->wait_queue_lock);
		if (!list_empty(&video->wait_buf_queue)) {
			wait_buf = list_first_entry(&video->wait_buf_queue,
						    struct isp_buffer, node);
			mutex_unlock(&video->wait_queue_lock);
			break;
		}
		mutex_unlock(&video->wait_queue_lock);
		reinit_completion(&video->feed_wait_buf_completion);
		if (qry->timeout_ms == 0) {
			return -ETIMEDOUT;
		} else if (qry->timeout_ms < 0) {
			wait_for_completion_io(
				&video->feed_wait_buf_completion);
		} else {
			ret = wait_for_completion_io_timeout(
				&video->feed_wait_buf_completion,
				msecs_to_jiffies(qry->timeout_ms));
			if (ret == 0) {
				dev_err(isp->dev, "[ISP_FEED]: wait timeout");
				return -ETIMEDOUT;
			}
		}
	}

	wait_v4l2_buffer = &wait_buf->vb;
	wait_vb2_buffer = &wait_v4l2_buffer->vb2_buf;
	qry->buf_index = wait_vb2_buffer->index;

	dev_dbg(isp->dev,
		"[ISP_FEED]: wait buf = 0x%p, index = %d, addr = 0x%08x, 0x%08x",
		wait_buf, qry->buf_index, wait_buf->dma[0], wait_buf->dma[1]);

	return 0;
}

int isp_feed_drv(struct c1200_isp_device *isp,
		 struct feed_drv_control *feed_drv_ctrl)
{
	int i = 0;
	struct c1200_isp_video *video;
	struct isp_buffer *wait_buf;
	int isp_ch_id;

	for (i = 0; i < MAX_ISP_CHANNEL; ++i) {
		if (!(feed_drv_ctrl->video_bitmap & (1 << i)))
			continue;

		dev_dbg(isp->dev, "[ISP_FEED]: video%d feed wait buf", i);

		isp_ch_id = isp_get_isp_id_by_video_id(isp, i);
		if (isp_ch_id >= MAX_ISP_CHANNEL || isp_ch_id < 0) {
			dev_err(isp->dev, "[ISP_FEED]: %s, error chid[%d]",
				__func__, isp_ch_id);
			return -EINVAL;
		}

		video = &isp->channels[isp_ch_id].views_video;
		if (!video) {
			dev_err(isp->dev,
				"[ISP_FEED]: c1200_isp_video is NULL");
			return -EINVAL;
		}

		mutex_lock(&video->wait_queue_lock);
		if (!list_empty(&video->wait_buf_queue)) {
			wait_buf = list_first_entry(&video->wait_buf_queue,
						    struct isp_buffer, node);
			dev_dbg(isp->dev,
				"[ISP_FEED]: wait buf = 0x%p, addr = 0x%08x, 0x%08x",
				wait_buf, wait_buf->dma[0], wait_buf->dma[1]);
			list_del(&wait_buf->node);
			mutex_unlock(&video->wait_queue_lock);
		} else {
			mutex_unlock(&video->wait_queue_lock);
			return -ENODEV;
		}

		if (wait_buf->vb.vb2_buf.state != VB2_BUF_STATE_ACTIVE)
			return -ENODEV;

		wait_buf->cycle_count = 0;
		wait_buf->vb.vb2_buf.timestamp =
			feed_drv_ctrl->bufs[i].timestamp * 1000;
		wait_buf->vb.sequence = feed_drv_ctrl->bufs[i].sequence;

		vb2_buffer_done(&wait_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
		dev_dbg(isp->dev, "[ISP_FEED]: timestamp = %llu, seq = %u",
			wait_buf->vb.vb2_buf.timestamp, wait_buf->vb.sequence);
	}

	return 0;
}

static long isp_misc_ioctl(struct file *pfile, unsigned int cmd,
			   unsigned long args)
{
	int ret = 0;
	struct c1200_isp_device *isp = container_of(
		globle_isp_misc_device, struct c1200_isp_device, misc_device);

	mutex_lock(&isp->misc_ioctl_mutex);

	switch (cmd) {
	case IOC_FEED_ENABLE:
		{
			isp->data_mode = DATA_MODE_FEED_DRIVER;
			if (isp->qry_buf_info == NULL) {
				isp->qry_buf_info = devm_kzalloc(
					isp->dev, sizeof(*isp->qry_buf_info),
					GFP_KERNEL);
			}
			if (isp->qry_free_buf_info == NULL) {
				isp->qry_free_buf_info = devm_kzalloc(
					isp->dev, sizeof(*isp->qry_buf_info),
					GFP_KERNEL);
			}
			if (isp->feed_control_info == NULL) {
				isp->feed_control_info = devm_kzalloc(
					isp->dev,
					sizeof(*isp->feed_control_info),
					GFP_KERNEL);
			}

			break;
		}
	case IOC_FEED_DRV_QUERY_BUFF:
		{
			if (isp->data_mode != DATA_MODE_FEED_DRIVER) {
				dev_err(isp->dev,
					"[ISP_FEED]: data mode is not feed mode");
				ret = -EINVAL;
				break;
			}

			ret = copy_from_user(
				isp->qry_buf_info,
				(struct feed_drv_querybuffers __user *)args,
				sizeof(struct feed_drv_querybuffers));
			if (ret) {
				pr_err("%s, IOC_FEED_DRV_QUERY_BUFF copy_from_user failed!\n",
				       __func__);
				ret = -EFAULT;
				break;
			}

			ret = isp_feed_drv_query_buff(isp, isp->qry_buf_info);
			if (ret)
				break;

			ret = copy_to_user(
				(struct feed_drv_querybuffers __user *)args,
				isp->qry_buf_info,
				sizeof(struct feed_drv_querybuffers));
			if (ret) {
				pr_err("%s, IOC_FEED_DRV_QUERY_BUFF copy_to_user failed!\n",
				       __func__);
				ret = -EFAULT;
				break;
			}

			break;
		}
	case IOC_FEED_DRV_QUERY_FREE_BUFF:
		{
			if (isp->data_mode != DATA_MODE_FEED_DRIVER) {
				dev_err(isp->dev,
					"[ISP_FEED]: data mode is not feed mode");
				ret = -EINVAL;
				break;
			}

			ret = copy_from_user(
				isp->qry_free_buf_info,
				(struct feed_drv_query_free_buf __user *)args,
				sizeof(struct feed_drv_query_free_buf));
			if (ret) {
				pr_err("%s, IOC_FEED_DRV_QUERY_FREE_BUFF copy_from_user failed!\n",
				       __func__);
				ret = -EFAULT;
				break;
			}

			ret = isp_feed_drv_query_free_buf(
				isp, isp->qry_free_buf_info);
			if (ret)
				break;

			ret = copy_to_user(
				(struct feed_drv_query_free_buf __user *)args,
				isp->qry_free_buf_info,
				sizeof(struct feed_drv_query_free_buf));
			if (ret) {
				pr_err("%s, IOC_FEED_DRV_QUERY_FREE_BUFF copy_to_user failed!\n",
				       __func__);
				ret = -EFAULT;
				break;
			}

			break;
		}
	case IOC_FEED_DRV:
		{
			if (isp->data_mode != DATA_MODE_FEED_DRIVER) {
				dev_err(isp->dev,
					"[ISP_FEED]: data mode is not feed mode");
				ret = -EINVAL;
				break;
			}

			ret = copy_from_user(
				isp->feed_control_info,
				(struct feed_drv_control __user *)args,
				sizeof(struct feed_drv_control));
			if (ret) {
				pr_err("%s, IOC_FEED_DRV copy_from_user failed!\n",
				       __func__);
				ret = -EFAULT;
				break;
			}

			ret = isp_feed_drv(isp, isp->feed_control_info);
			if (ret)
				break;

			break;
		}

	default:
		return -EINVAL;
	}

	mutex_unlock(&isp->misc_ioctl_mutex);
	return ret;
}

static const struct file_operations isp_misc_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = isp_misc_ioctl,
};

static struct miscdevice isp_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "isp_misc",
	.fops = &isp_misc_fops,
};

static void receive_isp_msg(const uint32_t msgAddr, const uint64_t context,
			    const ext_info_t *info)
{
	struct c1200_isp_device *isp;
	uint32_t msg_phy_addr;
	uint32_t msg_addr_offset;
	uint32_t *msg;
	struct media_command *media_msg;

	isp = g_isp;
	dev_dbg(isp->dev, "%s: msg: 0x%08X, timestamp: %llu\n", __func__,
		msgAddr, info->timestamp);
	/* analysis ipc message and get media msg */
	msg_phy_addr = msgAddr - 0x80000000;
	msg_addr_offset = msg_phy_addr - (isp->init_paddr & LOW_32_BIT_MASK);
	media_msg = (struct media_command *)(isp->init_vaddr + msg_addr_offset);

	dev_dbg(isp->dev,
		"%s: init_paddr: 0x%08llX, offset: 0x%08X, init_vaddr: 0x%016llX, msg_vaddr: 0x%016llX\n",
		__func__, isp->init_paddr, msg_addr_offset,
		(long long)isp->init_vaddr, (long long)media_msg);

	msg = (uint32_t *)(isp->init_vaddr + msg_addr_offset);

	dev_dbg(isp->dev,
		"%s: media_cmd contents: 0x%04X 0x%04X 0x%04X 0x%04X\n",
		__func__, msg[0], msg[1], msg[2], msg[3]);

	ispdrv_message_handler(isp, media_msg, info->timestamp);
}

static int32_t on_arm2isp_sub(uint8_t pid, uint8_t fid, uint8_t sid)
{
	pr_crit("Recevie heartbeat subscription: pid %d, fid %d, sid %d\n", pid,
		fid, sid);

	return 0;
}

static int32_t on_arm2isp_unsub(uint8_t pid, uint8_t fid, uint8_t sid)
{
	pr_crit("Recevie heartbeat unsubscription: pid %d, fid %d, sid %d\n",
		pid, fid, sid);

	return 0;
}

static inline int send_msg_async(struct c1200_isp_device *isp, uint32_t msg)
{
	int ret;

	if (!isp->msg_server) {
		dev_err_ratelimited(isp->dev, "%s: invalid msg server\n",
				    __func__);
		return -1;
	}

	/* NOTE: This returns successful send num */
	ret = isp->msg_server->video_server.arm2isp(msg);
	if (ret <= 0) {
		dev_err_ratelimited(isp->dev,
				    "Failed to send message to FW, ret: %d\n",
				    ret);
		return -1;
	}

	return 0;
}

static int isp_run_msg_server(struct c1200_isp_device *isp)
{
	int ret;
	ipc_inf_version_t version;

	isp->msg_server = isp_msgbx_server_init(&isp->msg_server_data);
	if (!isp->msg_server) {
		dev_err(isp->dev, "Failed to init msgbx server\n");
		return -1;
	}

	version = isp->msg_server->video_server.version();
	dev_info(isp->dev, "Interface version: %d.%d\n", version.major,
		 version.minor);

	ret = isp->msg_server->video_server.register_isp2arm(receive_isp_msg);
	if (ret < 0) {
		dev_err(isp->dev, "Failed to register isp2arm\n");
		return ret;
	}

	// register sub and unsub callback
	ret = isp->msg_server->video_server.register_arm2isp_subcribed(
		on_arm2isp_sub);
	if (ret < 0) {
		dev_err(isp->dev, "Failed to register on_arm2isp_sub\n");
		return ret;
	}

	ret = isp->msg_server->video_server.register_arm2isp_unsubcribed(
		on_arm2isp_unsub);
	if (ret < 0) {
		dev_err(isp->dev, "Failed to register on_arm2isp_unsub\n");
		return ret;
	}

	ret = isp->msg_server->start();
	if (ret < 0) {
		dev_err(isp->dev, "Failed to start msg server\n");
		return ret;
	}

	return 0;
}

static int isp_exit_msg_server(struct c1200_isp_device *isp)
{
	int ret;

	if (!isp->msg_server) {
		dev_err(isp->dev, "No valid msg server to stop\n");
		return -1;
	}

	ret = isp->msg_server->stop();
	if (ret < 0) {
		dev_err(isp->dev, "Failed to stop msg server\n");
		return ret;
	}

	ret = isp_msgbx_server_destroy();
	if (ret < 0) {
		dev_err(isp->dev, "Failed to destroy msg server\n");
		return ret;
	}

	return 0;
}

static int of_mem_region_to_resource(struct device_node *node, unsigned int idx,
				     struct resource *res)
{
	int rv;
	struct device_node *region;

	region = of_parse_phandle(node, "memory-region", idx);
	if (!region)
		return -EINVAL;

	rv = of_address_to_resource(region, 0, res);
	of_node_put(region);

	return rv;
}

static int setup_reg_region(struct c1200_isp_device *isp)
{
	struct device *dev;
	struct platform_device *pdev;
	struct resource *iomem;

	dev = isp->dev;
	pdev = isp->pdev;

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem) {
		dev_err(dev, "Failed to get IORESOURCE_MEM 0\n");
		goto err_reg0;
	}
	dev_info(dev, "Reg region 0 start: 0x%08llX, end: 0x%08llX\n",
		 iomem->start, iomem->end);
	isp->ctrl = devm_ioremap_resource(dev, iomem);
	if (IS_ERR(isp->ctrl)) {
		dev_err(dev, "Failed to map ctrl base: %ld\n",
			PTR_ERR(isp->ctrl));
		goto err_reg0;
	}

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!iomem) {
		dev_err(dev, "Failed to get IORESOURCE_MEM 1\n");
		goto err_reg1;
	}
	dev_info(dev, "Reg region 1 start: 0x%08llX, end: 0x%08llX\n",
		 iomem->start, iomem->end);
	isp->pram = devm_ioremap_resource(dev, iomem);
	if (IS_ERR(isp->pram)) {
		dev_err(dev, "Failed to map pram base: %ld\n",
			PTR_ERR(isp->pram));
		goto err_reg1;
	}
	memset_io(isp->pram, 0, resource_size(iomem));

	/* If IOMMU is enabled, we should map IPC registers for ISP firmware */
	if (isp->iommud && isp->use_ipc) {
		int rv;
		size_t size;

		iomem = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		if (!iomem) {
			dev_err(dev, "Failed to get IORESOURCE_MEM 2\n");
			goto err_reg2;
		}
		dev_info(dev, "Reg region 2 start: 0x%08llX, end: 0x%08llX\n",
			 iomem->start, iomem->end);
		size = resource_size(iomem);
		rv = iommu_map(isp->iommud, iomem->start, iomem->start, size,
			       IOMMU_READ | IOMMU_WRITE);
		if (rv) {
			dev_err(dev,
				"Failed to map 0x%08llX, size: %lu, rv: %d\n",
				iomem->start, size, rv);
			goto err_reg2;
		}
	}

	return 0;

err_reg2:
	devm_iounmap(dev, isp->pram);
err_reg1:
	devm_iounmap(dev, isp->ctrl);
err_reg0:
	return -1;
}

static int setup_mem_region(struct c1200_isp_device *isp)
{
	int rv;
	struct device *dev;
	struct device_node *node;
	struct resource iomem_msg;
	struct resource iomem_buf;

	dev = isp->dev;
	node = isp->dev->of_node;

	rv = of_mem_region_to_resource(node, 0, &iomem_msg);
	if (rv) {
		dev_err(dev, "No memory assigned for message exchange\n");
		goto err_mem_msg;
	}
	dev_info(dev, "Reserved 0x%08llX - 0x%08llX for message exchange\n",
		 iomem_msg.start, iomem_msg.end);
	isp->init_paddr = iomem_msg.start;
	isp->init_vaddr = devm_memremap(dev, iomem_msg.start,
					resource_size(&iomem_msg), MEMREMAP_WC);
	if (IS_ERR(isp->init_vaddr)) {
		dev_err(dev, "Failed to map message region: %ld\n",
			PTR_ERR(isp->init_vaddr));
		goto err_mem_msg;
	}
	memset(isp->init_vaddr, 0, resource_size(&iomem_msg));

	rv = of_mem_region_to_resource(node, 1, &iomem_buf);
	if (rv) {
		dev_err(dev, "No memory assigned for reserved buffer\n");
		goto err_mem_buf;
	}
	dev_info(dev, "Reserved 0x%08llX - 0x%08llX for firmware\n",
		 iomem_buf.start, iomem_buf.end);
	isp->fbuf_paddr = isp_phys_to_bus(iomem_buf.start);
	isp->fbuf_psize = resource_size(&iomem_buf);

	/* If IOMMU is enabled, we should map memories for ISP firmware */
	if (isp->iommud) {
		size_t size;

		size = resource_size(&iomem_msg);
		rv = iommu_map(isp->iommud, isp_phys_to_bus(iomem_msg.start),
			       iomem_msg.start, size, IOMMU_READ | IOMMU_WRITE);
		if (rv) {
			dev_err(dev,
				"Failed to map 0x%08llX, size: %lu, rv: %d\n",
				iomem_msg.start, size, rv);
			goto err_map_msg;
		}

		size = resource_size(&iomem_buf);
		rv = iommu_map(isp->iommud, isp_phys_to_bus(iomem_buf.start),
			       iomem_buf.start, size, IOMMU_READ | IOMMU_WRITE);
		if (rv) {
			dev_err(dev,
				"Failed to map 0x%08llX, size: %lu, rv: %d\n",
				iomem_buf.start, size, rv);
			goto err_map_buf;
		}

		if (isp->use_ipc) {
			struct resource iomem_ipc;

			rv = of_mem_region_to_resource(node, 2, &iomem_ipc);
			if (rv) {
				dev_err(dev, "No memory assigned for IPC\n");
				goto err_mem_ipc;
			}
			dev_info(dev, "Reserved 0x%08llX - 0x%08llX for IPC\n",
				 iomem_ipc.start, iomem_ipc.end);
			size = resource_size(&iomem_ipc);
			rv = iommu_map(isp->iommud,
				       isp_phys_to_bus(iomem_ipc.start),
				       iomem_ipc.start, size,
				       IOMMU_READ | IOMMU_WRITE);
			if (rv) {
				dev_err(dev,
					"Failed to map 0x%08llX, size: %lu, rv: %d\n",
					iomem_ipc.start, size, rv);
				goto err_map_ipc;
			}
		}

		dma_set_mask_and_coherent(isp->dev, DMA_BIT_MASK(32));
		dma_set_max_seg_size(isp->dev, DMA_BIT_MASK(32));
	} else {
		/* No SMMU, we should has memory pool to allocate buffer */
		rv = of_reserved_mem_device_init_by_idx(dev, node, 2);
		if (rv) {
			dev_err(dev,
				"No memory assigned for frame buffer: %d\n",
				rv);
			goto err_mem_ipc;
		}
		dma_set_mask_and_coherent(isp->dev, DMA_BIT_MASK(64));
		dma_set_max_seg_size(isp->dev, UINT_MAX);
	}

	return 0;

err_map_ipc:
err_mem_ipc:
	if (isp->iommud)
		iommu_unmap(isp->iommud, isp_phys_to_bus(iomem_buf.start),
			    resource_size(&iomem_buf));
err_map_buf:
	if (isp->iommud)
		iommu_unmap(isp->iommud, isp_phys_to_bus(iomem_msg.start),
			    resource_size(&iomem_msg));
err_map_msg:
err_mem_buf:
	devm_iounmap(dev, isp->init_vaddr);
err_mem_msg:
	return -1;
}

static void finalize_reg_region(struct c1200_isp_device *isp)
{
	devm_iounmap(isp->dev, isp->pram);
	devm_iounmap(isp->dev, isp->ctrl);
}

static void finalize_mem_region(struct c1200_isp_device *isp)
{
	devm_memunmap(isp->dev, isp->init_vaddr);
	if (!isp->iommud)
		of_reserved_mem_device_release(isp->dev);
}

static int isp_probe(struct platform_device *pdev)
{
	struct c1200_isp_device *isp;
	struct device *dev;
	int rv;

	dev = &pdev->dev;
	isp = devm_kzalloc(dev, sizeof(*isp), GFP_KERNEL);
	if (!isp)
		return -ENOMEM;

	isp->dev = dev;
	isp->pdev = pdev;
	isp->iommud = iommu_get_domain_for_dev(dev);
	if (isp->iommud)
		dev_info(dev, "Use SMMU");

	rv = parse_dt(isp);
	if (rv) {
		dev_err(dev, "Failed to parse device tree\n");
		return -EINVAL;
	}

	// reset_control_deassert(isp->rstc);
	rv = setup_reg_region(isp);
	if (rv) {
		dev_err(dev, "Failed to setup reg region\n");
		return -EIO;
	}

	rv = setup_mem_region(isp);
	if (rv) {
		dev_err(dev, "Failed to setup mem region\n");
		goto err_setup_mem;
	}

	globle_isp_misc_device = &isp->misc_device;

	isp->data_mode = DATA_MODE_NORMAL;
	mutex_init(&isp->isp_mutex);
	mutex_init(&isp->ipc_tx_mutex);
	mutex_init(&isp->misc_ioctl_mutex);
	memcpy(isp->revision, C1200_ISP_REVISION, sizeof(C1200_ISP_REVISION));
	isp->state = ISPDRV_STATE_WAIT;
	init_completion(&isp->FW_start_completion);
	init_completion(&isp->i2c_ctrl_completion);
	init_waitqueue_head(&isp->ctrl_recv_wq);
	platform_set_drvdata(pdev, isp);

	init_isp_channel_devs(isp);
	init_isp_dev(isp);

	rv = setup_msg_area(isp);
	if (rv) {
		dev_err(dev, "Failed to setup message area\n");
		goto err_setup_msg;
	}
	isp_power_subdevs(isp, 1);
	isp_fsync_work(isp);
	// must after init_isp_dev
	// because of must after v4l2_async_nf_register
	update_camera_status(isp);
	// must after firmware init, because comand array needs comand address
	init_isp_videos(isp);
	// register as misc device
	rv = misc_register(&isp_misc);
	if (rv < 0) {
		pr_err("misc register failed\n");
		return -1;
	}
#ifdef ISP_SYSFS
	isp_sysfs_init(isp);
#endif

	if (!(isp->use_ipc)) {
		if (isp_run_msg_server(isp)) {
			dev_err(isp->dev, "Failed to run msg server\n");
			return -1;
		}
	} else {
		isp->ipc_session_id =
			ipc_init(IPC_CORE_ISP, IPC_CORE_ARM3, &pdev->dev);
		if (isp->ipc_session_id < 0) {
			dev_err(&pdev->dev, "ipc_init(IPC_CORE_ARM0) failed\n");
			return -1;
		}

		isp->kthread_isp =
			kthread_run(isp_kthread_recv_msg, isp, "isp-recv-msg");

		if (IS_ERR(isp->kthread_isp)) {
			dev_err(dev, "create kthread recv error\n");
			return -1;
		}
	}

	g_isp = isp;

	return 0;

err_setup_msg:
	finalize_mem_region(isp);
err_setup_mem:
	finalize_reg_region(isp);

	return -1;
}

/*
 * isp_remove - Remove ISP platform device
 * @pdev: Pointer to ISP platform device
 *
 * Always returns 0.
 */
static int isp_remove(struct platform_device *pdev)
{
	struct c1200_isp_device *isp = platform_get_drvdata(pdev);

	if (!(isp->use_ipc))
		isp_exit_msg_server(isp);

	v4l2_async_nf_unregister(&isp->notifier);
	v4l2_async_nf_cleanup(&isp->notifier);

	return 0;
}

static void isp_shutdown(struct platform_device *pdev)
{
	struct c1200_isp_device *isp;

	isp = platform_get_drvdata(pdev);

	isp->state = ISPDRV_STATE_SHUTDOWN;
	atomic_set_release(&isp->running, 0);
	/* We reset all here to support reboot */
	isp_power_subdevs(isp, 0);
	reset_control_reset(isp->rstc);
}

static const struct of_device_id c1200_isp_of_table[] = {
	{ .compatible = "bst,c1200-isp" },
	{},
};
MODULE_DEVICE_TABLE(of, c1200_isp_of_table);

static struct platform_driver c1200_isp_driver = {
	.probe = isp_probe,
	.remove = isp_remove,
	.shutdown = isp_shutdown,
	.driver = {
		.name = "bst-isp",
		.of_match_table = c1200_isp_of_table,
	},
};

static int __init c1200_isp_init(void)
{
	platform_driver_register(&c1200_isp_driver);

	return 0;
}

static void __exit c1200_isp_exit(void)
{
	platform_driver_unregister(&c1200_isp_driver);
}

late_initcall(c1200_isp_init);
module_exit(c1200_isp_exit);

MODULE_VERSION(ISP_VIDEO_DRIVER_VERSION);
MODULE_DESCRIPTION("BST C1200 ISP driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
