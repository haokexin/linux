/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_ISP_CORE_H__
#define __BST_ISP_CORE_H__

#include <linux/completion.h>
#include <linux/dma-direct.h>
#include <linux/iommu.h>
#include <linux/iova.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <uapi/linux/ispdev.h>

#include <bst/media-dev.h>
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
#include <bst/smmu_safety_map.h>
#include <linux/bst_samphore.h>
#endif

#include "csi_rx.h"
#include "isp_hw.h"
#include "isp_video.h"
#include "isp_proto_base.h"

#ifdef CONFIG_BST_IPC_MSGBX
#include "msgbox/isp_msgbx_client.h"
#endif

#define ISP_DRIVER_NAME	      "bst-isp"
#define ISP_DRIVER_VERSION    "3.0.0"
#define LOW_32_BIT_MASK	      (0xFFFFFFFF)
#define ISP_MSG_PAYLOAD_ALIGN (256)
/* Multi OS */
#define MAX_CLIENTS	      (3)

#define ISP_RATELIMIT_INTERVAL (MAX_ISP_CHANNEL * HZ)
#define ISP_RATELIMIT_BURST    (MAX_ISP_CHANNEL)

enum cache_mode {
	CM_NONE,
	CM_HARDWARE,
	CM_SOFTWARE,
};

enum fw_stage {
	FS_UNUSED,
	FS_LOADED,
	FS_BOOTED,
	FS_RUNNING,
};

struct isp_device;

struct isp_ops {
	int (*msg_tx)(struct isp_device *isp, struct media_command *cmd);
	void (*msg_rx)(struct isp_device *isp, u32 msg_dma, u64 ts);
};

struct isp_msg {
	struct media_command *mc;
	u64 ts;
};

/*
 * @cid:	Unique channel ID, will be (CSI ID*VC num) + VC for CSI link
 * @enabled:	Whether channel is enabled, this is set when bound is completed
 * @on:		Whether channel is on, this is set when stream is on.
 * @csi_id:	The CSI device ID of this isp channel linked
 * @csi_vc:	The CSI device VC of this isp channel linked
 */
struct isp_channel {
	/* Common */
	struct mutex lock;
	int cid;
	struct isp_device *isp;
	ipc_reconf_t *cfg;

	/* Flags */
	u8 enabled : 1;
	u8 on	   : 1;

	const struct isp_file *algo;
	const struct isp_file *iq;

	struct isp_video views_video; // video supports multi view
	struct isp_video raw_video;

	/* Camera */
	struct camera_dev *cam_dev;

	/* Remote endpoint */
	struct csi_channel *csi_channel;
	int csi_id;
	int csi_vc;
};

/**
 * @fwnode:	Used by MATCH_FWNODE
 * @csi_dev:	Bound to csi_device
 * @used_ports:	How much ports are used.
 */
struct csi_async_dev {
	struct v4l2_async_subdev async_dev;
	struct fwnode_handle *fwnode;
	struct csi_device *csi_dev;
	int used_ports;
};

/**
 * struct isp_file - ISP bin file related to RAW camera
 * @path:	The path related to system's firmware directory
 * @dma:	DMA address the file be loaded
 * @size:	File size by byte
 */
struct isp_file {
	char path[PATH_MAX];
	dma_addr_t dma;
	size_t size;
};

struct isp_trigger {
	u32 gpio;
	u32 period;
	u32 polarity;
	u32 width;
	u32 num;
	u32 curr;
	u64 *ts;
	struct hrtimer timer;
	ktime_t next;
	u64 interval;
	bool running;
	u32 cpu;
};

struct isp_shared {
	/* Algo and IQ files management */
	struct {
		struct isp_file files[MAX_ISP_CHANNEL * 2];
		dma_addr_t next_dma; /* Next file will be loaded here */
		int num;
	} file;

	/* Firmware specific */
	struct {
		u32 version;
		u32 scm_id;
		u32 build_date;
		enum fw_stage stage;

		union {
			u8 psm_core[MAX_ISP_CORE];
			u32 psm_all;
		};
	} fw;
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	struct {
		u32 uid;
		struct media_command cmds[SONE_MEDIA_QSIZE];
	} mcs[MAX_CLIENTS];
#endif
};

struct isp_device {
	/* Common */
	struct mutex lock;
	struct device *dev;
	struct platform_device *pdev;
	enum cache_mode cache_mode;
	u32 role;
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	u32 sem_master;
	u32 sem_bank;
	u32 sem_id;
	struct bst_samphore *hwlock;
	u32 uid;
#endif

	/* Operations */
	struct isp_ops ops;

	/* Video, Channel */
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct isp_channel channels[MAX_ISP_CHANNEL];
	struct csi_async_dev csi_asd[MAX_CSI_DEVICE_NUM];

	/* Feed */
	bool feed_mode;

	union {
		struct feed_query_buffers qry_bufs;
		struct feed_query_free_buf qry_free_buf;
		struct feed_control feed_ctrl;
	};

	/* Communication */
	struct {
		void *init_va;
		phys_addr_t init_pa;
		dma_addr_t init_dma;
		size_t init_size;

		void *cmdp_va;
		void *slab_va;
		void *conf_va;
		size_t conf_size;

		struct media_command *cmds;
		int cmd_size;
		int cmd_index;

		struct mutex tx_lock;
		struct completion tx_comp;
		unsigned long tx_all;
		unsigned long tx_done;
		unsigned long tx_fail;
		unsigned long rx_all;
		unsigned long rx_bad;
		unsigned long rx_good;
	} msg;

	struct task_struct *async_msg_task;
	DECLARE_KFIFO(async_msg_fifo, struct isp_msg, SONE_MEDIA_QSIZE);
	struct completion async_msg_comp;
	bool use_ipc;
#ifdef CONFIG_BST_IPC
	/* IPC specific */
	struct {
		u32 cpu;
		int32_t sid;
		struct task_struct *rx_task;
		phys_addr_t msg_pa;
		dma_addr_t msg_dma;
		size_t msg_size;
	} ipc;
#endif

#ifdef CONFIG_BST_IPC_MSGBX
	/* MsgBox specific */
	struct {
		bool subscribed;
		isp_msgbx_client_t *client;
		isp_msgbx_client_data_t data;

		struct {
			struct mutex lock;
			struct list_head head;
		} rx_queue;
		struct completion avail_comp;
		struct completion rx_comp;
		struct task_struct *rx_task;
	} msgbx;
#endif

	struct {
		const char *bin;
		const char *slab;
		struct completion boot_comp;
		u32 cfg_index; /* Current config channel index */
		struct mutex cfg_lock;
		void *rsv_va;
		phys_addr_t rsv_pa;
		dma_addr_t rsv_dma;
		size_t rsv_size;
	} fw;
	struct isp_shared *shared;

	/* Read/Write memory by FW */
	struct {
		u32 addr;
		u32 value;
		u8 flag;
	} rw_addr_msg;

	/* Hardware Control */
	void __iomem *ctrl;
	void __iomem *pram;

	struct {
		unsigned long start;
		unsigned long size;
	} outer_reg[OUTER_REG_GROUP];
	struct reset_control *rstc;

	/* IOMMU support */
	struct iommu_domain *iommud;

	/* MISC device for private control */
	struct miscdevice miscdev;
	struct mutex miscdev_lock;

	/* Debug facilities */
	struct isp_trigger trigger;

	union {
		u32 flags;

		struct {
			u32 merge_msg	: 1; /* Merge buf and CAM_OPEN */
			u32 cfg_updated : 1;
			u32 rsv		: 30;
		};
	};
};

#define timer_to_trigger(timer) container_of(timer, struct isp_trigger, timer)
#define trigger_to_isp(trigger) \
	container_of(trigger, struct isp_device, trigger)

#define isp_msg_addr_cpu(isp, dma_addr) \
	((void *)((isp)->msg.init_va) + (dma_addr) - (isp)->msg.init_dma)
#define isp_msg_addr_dma(isp, cpu_addr)                   \
	((u32)((void *)(cpu_addr) + (isp)->msg.init_dma - \
	       (void *)((isp)->msg.init_va)))

static inline void *isp_buf_dma_to_va(struct isp_device *isp, u32 dma_addr)
{
	phys_addr_t pa;

	if (isp->iommud)
		pa = isp->iommud->ops->iova_to_phys(isp->iommud, dma_addr);
	else
		pa = dma_to_phys(isp->dev, dma_addr);

	return phys_to_virt(pa);
}

static inline bool is_reserved_buf(struct isp_device *isp, dma_addr_t dma_addr)
{
	dma_addr_t rsv_dma_start = isp->fw.rsv_dma;
	dma_addr_t rsv_dma_end = rsv_dma_start + isp->fw.rsv_size;

	return ((dma_addr >= rsv_dma_start) && (dma_addr < rsv_dma_end));
}

static inline struct isp_device *v4l2_to_isp_dev(struct v4l2_device *v4l2_dev)
{
	return container_of(v4l2_dev, struct isp_device, v4l2_dev);
}

static inline void isp_power_peer_devs(struct isp_device *isp, int on)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(isp->csi_asd); ++i)
		if (isp->csi_asd[i].csi_dev)
			v4l2_subdev_call(&isp->csi_asd[i].csi_dev->subdev, core,
					 s_power, on);
}

static inline void isp_reset_peer_devs(struct isp_device *isp)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(isp->csi_asd); ++i)
		if (isp->csi_asd[i].csi_dev)
			v4l2_subdev_call(&isp->csi_asd[i].csi_dev->subdev, core,
					 reset, 1);
}

static inline bool isp_fw_is_running(struct isp_device *isp)
{
	return (isp->shared->fw.stage == FS_RUNNING);
}

void isp_update_channels_cfg(struct isp_device *isp);
const char *isp_str_fw_stage(enum fw_stage stage);
const char *isp_str_role(u32 role);
bool isp_is_mapped_addr(struct isp_device *isp, dma_addr_t addr);

#endif /* __BST_ISP_CORE_H__ */
