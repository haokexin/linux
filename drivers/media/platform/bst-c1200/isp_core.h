/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __C1200_ISP_CORE_H__
#define __C1200_ISP_CORE_H__

#include <linux/clk-provider.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-memops.h>

#include <bst/ipc_interface.h>

#include "cam_entity.h"
#include "csi2_rx.h"
#include "isp_video.h"

#include "isp_msgbx_server.h"

#define ISP_SYSFS

#define SONE_MEDIA_FBUF_TOTAL_SIZE 0x8000000
#define ISP_CORE_NUM		   4
#define ISP_CHANNEL_VIEW_NUM	   3
#define ISP_CHANNEL_BUF_NUM	   8
#define ISP_CHANNEL_VIEW_NUM_PDNS  1
#define MAX_CHANNEL_PER_CORE	   4
#define HDMI_DTS_PORT_NUM	   4

// C1200 supports max 4 mipi devices
#define MAX_MIPI_DEVICE_NUM 4
#define MAX_ISP_SUB_DEVICE  5

#define MAX_ISP_CHANNEL	   16
#define MAX_ISP_MIPI_VC	   16
#define C1200_ISP_REVISION "1.0.0"

#define ISP_PLATFORM ("linux-isp")
#define ISP_DDR_BASE 0xA1000000

#define LOW_32_BIT_MASK	 0xFFFFFFFF
#define HIGH_32_BIT_MASK 0xFFFFFFFF00000000

#define ISP_INIT_PARTI_SIZE sizeof(tSoneInit)
#define ISP_CMDP_PARTI_SIZE sizeof(tSoneCmdp)

#define ISP_PRE_FW_TICK_ON_KTIME_NS 500 // each fw tick spend on ktime_ns
#define ISP_CTRL_PAYLOAD_SIZE	    40 // ioctrl paload size by byte
/* This is used by isp device, not for video */
#define ISP_MAX_MEDIA_COMMAND_NUM   10

#define IPC_RETRY_TIMES (3)

struct trigger_info {
	int trigger_mode;
	int mipi_id;
	int internal_trigger_fps;
	int camera_trigger_gpio_port;
	int deser_trigger_gpio_port;
	int target_freq;
};

struct camera_info {
	uint8_t camera_name[MAX_CAMERA_NAME_LEN];
	uint8_t mipi_vc_index;
	uint8_t isp_channel_index;
	uint8_t connected;
	uint8_t reserve;
};

struct isp_misc_device {
	struct camera_info cam_info[MAX_ISP_CHANNEL];
};

enum {
	INTERNAL_TRIGGER_MODE = 0,
	EXTERNAL_TRIGGER_MODE
};

enum i2c_bus_ctrl_t {
	I2C_BUS_ISP = 0, // isp controls i2c bus
	I2C_BUS_ARM = 1, // arm controls i2c bus
};

struct i2c_ctrl_info {
	int video_id;
	enum i2c_bus_ctrl_t bus_ctrler;
};

struct vb2_dc_buf {
	struct device *dev;
	void *vaddr;
	unsigned long size;
	void *cookie;
	dma_addr_t dma_addr;
	unsigned long attrs;
	enum dma_data_direction dma_dir;
	struct sg_table *dma_sgt;
	struct frame_vector *vec;

	/* MMAP related */
	struct vb2_vmarea_handler handler;
	refcount_t refcount;
	struct sg_table *sgt_base;

	/* DMABUF related */
	struct dma_buf_attachment *db_attach;
};

struct ispbuff_user {
	int32_t index;
	int32_t fd[ISP_CHANNEL_VIEW_NUM];
	uint64_t paddr[ISP_CHANNEL_VIEW_NUM];
	int32_t size[ISP_CHANNEL_VIEW_NUM];
	int32_t bytesused[ISP_CHANNEL_VIEW_NUM];
	uint64_t timestamp;
	uint64_t sequence;
	uint32_t exposure_time;
	int32_t reserved[8];
};

struct feed_drv_querybuffers {
	uint32_t video_id; /* Required for query */
	uint32_t buf_num; /* Output */
	struct ispbuff_user bufs[ISP_CHANNEL_BUF_NUM]; /* Output */
};

struct feed_drv_query_free_buf {
	uint32_t video_id; /* Required for query */
	/* -1 means blocked until get a free buffer, 0: non-block */
	int32_t timeout_ms;
	uint32_t buf_index; /* Output */
};

struct feed_drv_control {
	uint32_t video_bitmap;
	struct ispbuff_user bufs[MAX_ISP_CHANNEL];
	uint32_t reserved[3];
};

/* Feed function */
#define IOC_FEED_ENABLE _IOW('f', 0x10, int)
#define IOC_FEED_DRV_QUERY_FREE_BUFF \
	_IOWR('f', 0x11, struct feed_drv_query_free_buf)
#define IOC_FEED_DRV_QUERY_BUFF _IOWR('f', 0x12, struct feed_drv_querybuffers)
#define IOC_FEED_DRV		_IOW('f', 0x13, struct feed_drv_control)

enum {
	ISP_STATUS_BOOT_FW = 0,
	ISP_STATUS_FW_BOOT_DONE
};

enum {
	ISP_CHANNEL_SINK_PAD = 0,
	ISP_CHANNEL_SOURCE_NORMAL = 1,
	ISP_CHANNEL_SOURCE_PDNS = 2,
	ISP_CHANNEL_PAD_NUM,
};

enum ispdrv_state {
	ISPDRV_STATE_WAIT = 0, // wait firmware boot
	ISPDRV_STATE_SCFG, // send firmware config
	ISPDRV_STATE_SALG, // send firmware algo bin
	ISPDRV_STATE_SIQ, // send firmware iq bin
	ISPDRV_STATE_START, // start firmware
	ISPDRV_STATE_WORK, // work with firmware
	ISPDRV_STATE_GET_SCFG,
	ISPDRV_STATE_SHUTDOWN,
};

struct c1200_isp_device;

struct bst_isp_channel {
#if defined(CONFIG_MEDIA_CONTROLLER)
	struct media_entity entity;
#endif
	struct media_pad pads[ISP_CHANNEL_PAD_NUM];
	struct c1200_isp_video views_video; // video supports multi view
	struct c1200_isp_video raw_video;
	struct c1200_isp_device *isp;
	struct bst_csi_channel *csi_channel;
	struct camera_dev *cam_dev;
	int sn;
	int camera_raw_width;
	int camera_raw_height;
	int isp_core_id;
	int index_in_core;
	int isp_chn_id;
	int remote_mipi_id; // the mipi device id of this isp channel linked
	int remote_mipi_vc_index; // mipi vc index of this isp channel linked to
	bool enable;
	bool is_hdmi;
	atomic_t is_streaming;
	struct device_node *of_node;
	struct fwnode_handle *fwnode;
	struct fwnode_handle *remote_fwnode;
	uint32_t ctrl_payload_paddr;
	u8 *ctrl_payload_vaddr;
};

struct csi_async_dev {
	struct v4l2_async_subdev async_dev;
	struct fwnode_handle *mipi_fwnode;
	struct bst_csi_device *csi_dev;
	struct c1200_isp_device *isp_parent;
	int mipi_index;
	int mipi_connected;
};

/*
 * struct c1200_isp_device - ISP device structure.
 * @dev: Device pointer specific to the C1200 ISP.
 * @revision: Stores current ISP module revision.
 * @stat_lock: Spinlock for handling statistics
 * @isp_mutex: Mutex for serializing requests to ISP.
 * @stop_failure: Indicates that an entity failed to stop.
 * @ref_count: Reference count for handling multiple ISP requests.
 * @isp_slots: ISP supports 12 sensors totally, assume 12 virtual isp slots
 *
 * This structure is used to store the BST C1200 ISP Information.
 */
struct c1200_isp_device {
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct media_device media_dev;
	struct bst_isp_channel channels[MAX_ISP_CHANNEL];
	struct csi_async_dev csi_asd[MAX_MIPI_DEVICE_NUM];
	struct device *dev;
	struct platform_device *pdev;
	char revision[16];
	struct v4l2_async_subdev hdmi_async;
	struct device_node *hdmi_node;
	struct fwnode_handle *hdmi_handle;
	struct fwnode_handle *remote_hdmi;
	struct camera_dev *hdmi_cam;
	uint32_t fw_pack_version;
	uint32_t fw_svn_version;
	uint32_t fw_build_date;
	const char *fw_slab;
	const char *fw_bin;
	struct isp_misc_device misc_device;
	enum ispdrv_state state;
	int hdmi_detected;
	int core0_active_cam;
	int cfg_num;
	int alg_num;
	int iq_num;
	int cfg_count;
	int alg_count;
	int iq_count;
	int capture_conf_num;
	int capture_conf_count;
	int cfg_plugout_num;
	int cfg_plugout_count;
	int get_cfg_count;
	atomic_t FW_load_started;
	atomic_t FW_boot_done;
	atomic_t FW_config_done;
	struct completion FW_start_completion;
	struct completion i2c_ctrl_completion;
	int i2c_ctrl; // 0:isp control i2c bus 1: isp release i2c bus
	// GlbCfg   *FW_glb_cfg;
	struct wait_queue_head ctrl_recv_wq;
	uint32_t ctrl_get_val;
	int isp_ctrl_status;
	/* ISP Obj */
	struct mutex isp_mutex; /* For handling ref_count field */
	struct mutex ipc_tx_mutex; /* For IPC TX*/
	bool stop_failure;
	int core_channel_num[ISP_CORE_NUM];
	int core_camera_num[ISP_CORE_NUM];
	int64_t ipc_session_id;
	uint64_t init_paddr;
	uint64_t cmdp_paddr;
	uint64_t slab_paddr;
	uint64_t fbuf_paddr;
	uint64_t fbuf_psize;
	void *init_vaddr;
	void *cmdp_vaddr;
	void *slab_vaddr;
	uint8_t isp_client_registed;
	isp_msgbx_server_t *msg_server;
	isp_msgbx_server_data_t msg_server_data;
	struct task_struct *kthread_isp; // for isp message receiving
	uint64_t ipc_rx_count;
	uint64_t ipc_tx_count;
	u8 *config_payload_vaddr;
	u8 *algobin_next_vaddr;
	u8 *payload_end_vaddr;
	int media_cmd_index;
	struct media_command *config_media_cmd;
	uint32_t config_payload_paddr;
	int config_align_size;
	uint32_t algobin_next_paddr;
	uint32_t payload_end_paddr;
	uint32_t media_cmd_paddr;
	int kthread_status;
	// int mipi_connected[MAX_MIPI_DEVICE_NUM];
	// struct fwnode_handle *mipi_fwnode[MAX_MIPI_DEVICE_NUM];
	int total_subdev;
	int total_channel;
	int config_count;
	atomic_t streamon_count;
	bool use_ipc;
	atomic_t is_update_unplug_video;
	uint8_t core_status;
	struct mutex misc_ioctl_mutex;
	dataMode_e data_mode;
	struct feed_drv_querybuffers *qry_buf_info;
	struct feed_drv_query_free_buf *qry_free_buf_info;
	struct feed_drv_control *feed_control_info;

	/* Firmware related */

	/* Hareware Control */
	void __iomem *ctrl;
	void __iomem *pram;
	struct reset_control *rstc;

	/* IOMMU support */
	struct iommu_domain *iommud;

	atomic_t running;
};

enum {
	KTHREAD_STATUS_NONE = 0,
	KTHREAD_STATUS_RECV_MSG,
	KTHREAD_STATUS_RECV_COMPLETE,
	KTHREAD_STATUS_COPY_MSG,
	KTHREAD_STATUS_COPY_COMPLETE,
};

enum {
	ISP_SET_VIEW_STATUS_NONE = 0,
	ISP_SET_VIEW_STATUS_WAIT = 1,
	ISP_SET_VIEW_STATUS_DONE = 2,
};

// todo: add more color
// C1200_ISP_COLOR_RAW8,
// C1200_ISP_COLOR_RAW10,
// C1200_ISP_COLOR_RAW12,
// C1200_ISP_COLOR_YUV422, (YUYV and so on)
// C1200_ISP_COLOR_YUV444
enum {
	C1200_ISP_COLOR_YUV420 = 0, // Y, U, V
	C1200_ISP_COLOR_NV12,
	C1200_ISP_COLOR_NV21,
	C1200_ISP_COLOR_YUYV,
	C1200_ISP_COLOR_RGB888,
	C1200_ISP_COLOR_RAW8,
	C1200_ISP_COLOR_RAW10,
	C1200_ISP_COLOR_RAW12,
	C1200_ISP_COLOR_RAW16,
	C1200_ISP_COLOR_Y_ONLY,
	C1200_ISP_COLOR_MAX,
};

#define isp_cmd_va(isp, pa) \
	((void *)((isp)->config_media_cmd) + (pa) - (isp)->media_cmd_paddr)
#define isp_cmd_pa(isp, va)                                 \
	((uint32_t)((void *)(va) + (isp)->media_cmd_paddr - \
		    (void *)((isp)->config_media_cmd)))

int ispdrv_enter_debug_state(struct c1200_isp_device *isp);

int ispdrv_leave_debug_state(struct c1200_isp_device *isp);

void c1200_isp_print_status(struct c1200_isp_device *isp);

int send_cmd_to_fw(struct c1200_isp_device *isp, ipc_msg *msg);

int isp_power_subdevs(struct c1200_isp_device *isp, int enable);

int isp_update_views_from_fw(struct c1200_isp_device *isp);

void isp_boot_fw_api(struct c1200_isp_device *isp);

/*
 * Convert physical address which CPU see to ISP hardware side
 * @pa: Physical from CPU side
 *
 * Returns physical address from ISP hardware
 */
static inline u32 isp_phys_to_bus(phys_addr_t pa)
{
	if (pa >= 0xC00000000)
		return (pa - 0xB40000000);
	else
		return (pa - 0x780000000);
}

/*
 * Convert physical address which ISP hardware see to CPU side
 * @pa: Physical from ISP hardware side
 *
 * Returns physical address from CPU side
 */
static inline phys_addr_t isp_bus_to_phys(u32 bus_addr)
{
	if (bus_addr >= 0xC0000000)
		return ((phys_addr_t)bus_addr + 0xB40000000);
	else
		return ((phys_addr_t)bus_addr + 0x780000000);
}

static inline struct media_command *
isp_get_media_cmd(struct c1200_isp_device *isp)
{
	struct media_command *pcmd;
	tSoneCmdp *cmdp;

	cmdp = (tSoneCmdp *)isp->cmdp_vaddr;
	mutex_lock(&isp->ipc_tx_mutex);
	pcmd = cmdp->ch[DRV_CH_INDEX].cqueue.c0 + isp->media_cmd_index;
	++isp->media_cmd_index;
	isp->media_cmd_index %= ARRAY_SIZE(cmdp->ch[DRV_CH_INDEX].cqueue.c0);
	mutex_unlock(&isp->ipc_tx_mutex);

	return pcmd;
}

static inline void isp_inc_streamon_count(struct c1200_isp_device *isp)
{
	atomic_inc(&(isp->streamon_count));
}

static inline void isp_dec_streamon_count(struct c1200_isp_device *isp)
{
	atomic_dec(&(isp->streamon_count));
}

static inline int get_isp_streamon_count(struct c1200_isp_device *isp)
{
	int ret;

	ret = atomic_read(&(isp->streamon_count));

	return ret;
}

#define pack_chars_to_int(a, b, c, d) ((a) | (b << 8) | (c << 16) | (d << 24))

int send_i2c_ctrl_msg_to_fw(struct c1200_isp_device *isp, int bus_index,
			    int i2c_bus_ctrler);
#endif /* __C1200_ISP_CORE_H__ */
