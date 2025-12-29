/* SPDX-License-Identifier: GPL-2.0 */
/* virt-touchscreen driver for BST C1200
 * This file contains proprietary information that is the sole intellectual
 * property of Black Sesame Technologies, Inc. and its affiliates.
 * No portions of this material may be reproduced in any
 * form without the written permission of:
 * Black Sesame Technologies, Inc. and its affiliates
 * 2255 Martin Ave. Suite D
 * Santa Clara, CA 95050
 * Copyright @2016: all right reserved.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif /* CONFIG_DEBUG_FS */
#include <touch_virt_bst.h>
#ifdef SUPPORT_CALIBRATION
#include <touch_virt_bst_cdev.h>
#endif
#include "touchclient.h"
#include "touch_packet_statistic_cli.h"
#ifdef TOUCH_STATISTICS_PACKET_TIMESTAMP_TOOL_ENABLE
#include "touch_packet_statistic_tool.h"
#endif

#define RINGBUF_NUM_PER_SCREEN 200
#define RECONNECT_RETRY_MAX_COUNT 6
/* need wait some seconds for reconnect work, if the touch devices of server is not ready */
#define RECONNET_WORK_DELAY_MS (10 * 1000) // 10 seconds

/* 1k align */
#define RINGBUF_OFS_PER_SCREEN ALIGN((RINGBUF_NUM_PER_SCREEN * MAX_POINT * ALIGN(sizeof(point_data_t), 2)), 0x400)
#define ARRAY_LENGTH(x)            (sizeof(x) / sizeof((x)[0]))

/* configure from sbl */
#define DDR_INTERLEAVING_ON

#define MSGBOX_USE_FIXED_ARRAY

/** TYPE_B_PROTOCOL
 *  Open to enable the multi-touch (MT) protocol
 */
#define REPORT_INPUT_EVENT
/** report pointinfo debug
 * #define REPORT_POINTINFO_DEBUG
 * #define REPROT_POINTINFO_DEBUG_MSGINX
 */
 #define REPORT_POINTINFO_DEBUG
 #define REPROT_POINTINFO_DEBUG_MSGINX

#ifdef REPORT_INPUT_EVENT
#define TYPE_B_PROTOCOL
#define REPORT_TOUCH_WIDTH
/** Other point report function
 * #define REPORT_INPUT_MT_DROP_UNUSED
 * #define REPORT_SWAP_XY
 * #define REPORT_FLIP_X
 * #define REPORT_FLIP_Y
 */
#endif

#ifdef SUPPORT_CALIBRATION

#define BST_TOUCH_IOCTL_MAGIC 'T'

#define BST_TOUCH_GET_CALIBRATION _IOR(BST_TOUCH_IOCTL_MAGIC, 1, calibration_info_t *)
#define BST_TOUCH_SET_CALIBRATION _IOW(BST_TOUCH_IOCTL_MAGIC, 2, calibration_info_t *)
#define BST_TOUCH_GET_SCREEN_ID _IOR(BST_TOUCH_IOCTL_MAGIC, 3, uint32_t *)
#define BST_TOUCH_GET_CONNECT_STATUS _IOR(BST_TOUCH_IOCTL_MAGIC, 4, uint32_t *)
#define BST_TOUCH_GET_HWINFO _IOR(BST_TOUCH_IOCTL_MAGIC, 5, hw_info_t *)
#define BST_TOUCH_GET_POINTINFO _IOR(BST_TOUCH_IOCTL_MAGIC, 6, point_info_t *)

#if (KERNEL_VERSION(5, 9, 0) <= LINUX_VERSION_CODE) || \
	defined(HAVE_UNLOCKED_IOCTL)
#define USE_UNLOCKED_IOCTL
#endif
#endif

/** Codes for common object classifications, for point_data.tool_type */
enum object_classification {
	LIFT = 0,
	FINGER = 1,
	GLOVED_OBJECT = 2,
	STYLUS = 3,
	ERASER = 4,
	SMALL_OBJECT = 5,
	PALM = 6,
	EDGE_TOUCHED = 8,
	HOVER_OBJECT = 9,
	NOP = -1,
};

static uint32_t bst_ts_debug_level = 0;
module_param_named(debug_level, bst_ts_debug_level, uint, S_IRUGO | S_IWUSR);

enum {
	TOUCH_DBG_LVL_NONE = 0,
	TOUCH_DBG_LVL_1 = BIT(0),
	TOUCH_DBG_LVL_2 = BIT(1),
	TOUCH_DBG_LVL_3 = BIT(2),
	TOUCH_DBG_LVL_4 = BIT(3),
	TOUCH_DBG_LVL_MAX = 0xff,
};

#define BST_TS_DEBUG(level, fmt, args...) \
	do { \
		if (bst_ts_debug_level & (level)) \
			pr_info(fmt, ##args); \
	} while (0)

#define BST_TS_DEBUG_LVL(dev, level, fmt, args...) \
	do { \
		if (bst_ts_debug_level & (level)) \
			dev_info(dev, fmt, ##args); \
	} while (0)

#define BST_TS_DEBUG_LVL1(dev, fmt, args...) \
	BST_TS_DEBUG_LVL(dev, TOUCH_DBG_LVL_1, fmt, ##args)

#define BST_TS_DEBUG_LVL2(dev, fmt, args...) \
	BST_TS_DEBUG_LVL(dev, TOUCH_DBG_LVL_2, fmt, ##args)

#ifdef CONFIG_DEBUG_FS
static struct dentry *bst_ts_debugfs_dir = NULL;
#endif
struct tc_err_info_t {
	int errcode;
	char *errmsg;
};

static struct tc_err_info_t tc_err_info[] = {
	{TC_IPC_ERROR_RET(SERVER_FAIL), "failed server"},
	{TC_IPC_ERROR_RET(ERROR_COMMON), "failed common"},
	{TC_IPC_ERROR_RET(ERROR_SCREEN_ID_INVALID), "invalid screen id"},
	{TC_IPC_ERROR_RET(ERROR_SCREEN_NUM_OVERLIMIT), "over limit screen num"},
	{TC_IPC_ERROR_RET(ERROR_CLIENT_NUM_OVERLIMIT), "over limit client num"},
	{TC_IPC_ERROR_RET(ERROR_RESEVERD), "reserved error"},
};

enum client_status_t
{
	CLIENT_STATUS_NONE,
	CLIENT_STATUS_START,
	CLIENT_STATUS_SUBSCRIPTION_REQUEST,
	CLIENT_STATUS_SUBSCRIPTION_TIMEOUT,
	CLIENT_STATUS_SUBSCRIPTION_SUSS,
	CLIENT_STATUS_REQUEST_RESOURCE_SUSS,
	CLIENT_STATUS_EINVALID
};

enum srv_status_t
{
	SRV_STATUS_NONE,
	SRV_STATUS_OFFLINE,
	SRV_STATUS_ONLINE,
	SRV_STATUS_EINVALID
};

/**
 * Read the 32-bit value (referred to as old) stored at location pointed by p. Compute (status == cmp) ?  new :
 *   old and store result at location pointed by &status. The function returns old.
 */
#define TS_UPDATE_TO_NEXT_STATUS(status, old, new) \
	do { \
		atomic_cmpxchg(&(status), old, new); \
	} while (0)
#define TS_SET_STATUS(status, val) \
	do { \
		atomic_set(&(status), val); \
	} while (0)
#define TS_GET_STATUS(status) \
	atomic_read(&(status))

struct client_info_t
{
	uint32_t client_id;
	int request_screen_num; // configure screen from device tree;
	uint32_t request_screen_id[MAX_SCREEN]; // configure screen from device tree;
	struct platform_device *pdev;
	struct bst_ts_data **ts_data;

	/* msgbox */
	atomic_t status;
	atomic_t srv_status; /**service status: online and offline */
	tsclient_data_t ins;
	tsclient_t *ts_client;
	int reconnect_retry_count;
	struct delayed_work reconnect_work;
	struct task_struct *wait_task; // continue initial until server become ready
	struct wait_queue_head subscription_waitqueue;

#ifdef USE_SHAREMEM_POINTINFO
	bool use_shmem;
	phys_addr_t phys_addr; /* phy_addr */
	void *shmem_vaddr;
	size_t shmem_size;
#endif
};

static struct client_info_t *touchclient_info;
static des_buf_t touch_des_buf;

static int bst_ts_init(struct client_info_t *info);

#ifdef DDR_INTERLEAVING_ON
#define paddr_64_to_paddr_32(addr) _paddr_64_to_paddr_32(addr, true)
#else
#define paddr_64_to_paddr_32(addr) _paddr_64_to_paddr_32(addr, false)
#endif

static int bst_touch_request_resouce(const uint32_t client_id, const uint32_t screen_id,
	hw_info_t *info);

static __inline__ char *errcode_msg(int errcode)
{
	int i;
	for (i  = 0; i < ARRAY_LENGTH(tc_err_info); i++) {
		if (errcode == tc_err_info[i].errcode) {
			return tc_err_info[i].errmsg;
		}
	}
	return "unknow";
}

static uint8_t calculate_checksum(const uint8_t *data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; ++i) {
        checksum += data[i];
    }
    return checksum;
}

#ifdef USE_SHAREMEM_POINTINFO

/**
 * @brief Initialize shared memory ring buffer.
 *
 * @param buffer Pointer to the ring buffer structure.
 * @param addr   Address of the shared memory.
 * @param size   Size of the ring buffer.
 */
static void init_ring_buffer(ring_buffer *buffer, void *addr, int size) {
    buffer->data = (char *)addr;
    buffer->size = size;
    buffer->head = 0;
    buffer->tail = 0;
}

/**
 * @brief Read data from shared memory ring buffer.
 *
 * @param buffer Pointer to the ring buffer structure.
 * @param offset Offset in the ring buffer to start reading from.
 * @param data   Pointer to the buffer to store the read data.
 * @param size   Number of bytes to read.
 * @return Number of bytes read, or -EINVAL if offset is out of range.
 */
static int read_from_ring_buffer(ring_buffer *buffer, int offset, char *data, int size) {
	int space_left;
    int bytes_to_copy;

    if (offset < 0 || offset >= buffer->size) {
        // offset is out of range, invalid operation
        return -EINVAL;
    }

	bytes_to_copy = size < buffer->size ? size : buffer->size;
    space_left = buffer->size - offset;
    if (space_left >= bytes_to_copy) {
        memcpy(data, buffer->data + offset, bytes_to_copy);
    } else {
        memcpy(data, buffer->data + offset, space_left);
        memcpy(data + space_left, buffer->data, bytes_to_copy - space_left);
    }
    return bytes_to_copy;
}
#endif

/**
 * @brief Convert a 64-bit physical address to a 32-bit physical address for r5.
 *
 * @param paddr        64-bit physical address.
 * @param interleaving Whether interleaving is on or off.
 * @return 32-bit physical address.
 */
static uint32_t _paddr_64_to_paddr_32(phys_addr_t paddr, bool interleaving)
{
	uint32_t paddr_32bit;

	if (interleaving) {
		if (paddr < 0xc40000000 && paddr >= 0xc00000000) {
			paddr_32bit = paddr - 0xb40000000;
		} else if (paddr < 0x840000000 && paddr >= 0x800000000) {
			paddr_32bit = paddr - 0x780000000;
		} else {
			paddr_32bit = 0;
		}
	} else {
		if (paddr < 0x880000000 && paddr >= 0x800000000) {
			paddr_32bit = paddr - 0x780000000;
		} else {
			paddr_32bit = 0;
		}
	}
	return paddr_32bit;
}

/**
 * @brief Get bst_ts_data structure for the registered screen ID.
 *
 * The usage scope is after the client successfully requests
 * the server hardware information, represents the real hardware status.
 * Otherwise NULL is returned.
 * @param screen_id The ID of the screen.
 * @return Pointer to bst_ts_data structure, or NULL if not found.
 */
static struct bst_ts_data *bst_ts_data_get_by_registered_screen_id(uint32_t screen_id)
{
	int inx;
	struct client_info_t *info = touchclient_info;
	struct bst_ts_data *ts_data;

	for (inx = 0; inx < info->request_screen_num; inx++) {
		ts_data = info->ts_data[inx];
		if(ts_data && screen_id == ts_data->hwinfo.screen_id)
			return ts_data;
	}
	return NULL;
}

/**
 * @brief Get bst_ts_data structure for the requesting screen ID.
 *
 * This interface only obtains bst_ts_data, and has nothing to do with whether
 * the server's hardware information is obtained.
 * @param screen_id The requested screen ID.
 * @return Pointer to bst_ts_data structure, or NULL if not found.
 */
static struct bst_ts_data *bst_ts_data_get_by_requesting_screen_id(uint32_t screen_id)
{
	int inx;
	struct client_info_t *info = touchclient_info;
	struct bst_ts_data *ts_data;

	for (inx = 0; inx < info->request_screen_num; inx++) {
		ts_data = info->ts_data[inx];
		if(ts_data && screen_id == ts_data->requested_screen_id)
			return ts_data;
	}
	return NULL;
}

/**
 * @brief Wait for subscription to finish with a timeout.
 *
 * @param client_info Pointer to the client info structure.
 * @param timeout_ms  Timeout in milliseconds.
 * @return 0 on success, -ERESTARTSYS if interrupted, -ETIMEDOUT if timed out.
 */
static int wait_for_subscription_finish(struct client_info_t *client_info, uint32_t timeout_ms) {
    unsigned long timeout = msecs_to_jiffies(timeout_ms);
    int ret;

    while (TS_GET_STATUS(client_info->status) != CLIENT_STATUS_SUBSCRIPTION_SUSS) {
        if (timeout) {
            ret = wait_event_interruptible_timeout(client_info->subscription_waitqueue,
				TS_GET_STATUS(client_info->status) == CLIENT_STATUS_SUBSCRIPTION_SUSS,
                                                   timeout);
            if (ret == -ERESTARTSYS) {
                // Restarted due to signal, handle accordingly
                return -ERESTARTSYS;
            } else if (ret == 0) {
                // Timeout
                return -ETIMEDOUT;
            }
        } else {
            // Wait indefinitely
            wait_event_interruptible(client_info->subscription_waitqueue, TS_GET_STATUS(client_info->status) == CLIENT_STATUS_SUBSCRIPTION_SUSS);
        }
    }

    return 0;  // Success
}

/**
 * @brief  Callback for when touch info is received.
 *         Report touched events to the input subsystem.
 *
 * @param screen_id       The screen ID.
 * @param locinfo_offset  Offset in the shared memory for location info.
 * @param locinfo_size    Size of the location info.
 * @param locinfo_chksum  Checksum of the location info.
 * @param ext             Pointer to extended data.
 * @param info            Pointer to extended info.
 */
 #ifdef USE_SHAREMEM_POINTINFO
 static void on_touch_info_received(
				const uint32_t screen_id,
				const uint32_t locinfo_offset,
				const uint32_t locinfo_size,
				const uint32_t locinfo_chksum,
				void *ext,
				const ext_info_t *info
				)
{
	int ret;
#else
static void on_touch_info_received(
				const uint32_t screen_id,
				const point_info_t locinfo,
				void *ext,
				const ext_info_t *info
				)
{
#endif
	unsigned int idx, input_slot_idx;
	unsigned int x;
	unsigned int y;
	int wx;
	int wy;
	unsigned int status;
	unsigned int touch_count;
	uint8_t calculated_checksum;
#ifdef REPORT_INPUT_EVENT
	struct input_dev *input_dev;
#endif
#ifdef REPORT_POINTINFO_DEBUG
	unsigned int lift_count;
#endif
	unsigned int max_objects;
	point_data_t *object_data;
	struct bst_ts_data *ts;
	point_info_t *plocinfo;
#if (defined REPORT_SWAP_XY) || (defined REPORT_FLIP_X) || (defined REPORT_FLIP_X)
	struct touchscreen_properties *prop;
#endif

	STATISTICS_UPDATE_PACKET_COORD_LOOP(screen_id);
	STATISTICS_UPDATE_PACKET_COORD_TIMESTAMP(screen_id, TIMESTAMP_PACKET_COORD_RECV_MSGBOX_FROM_SRV);

	ts = bst_ts_data_get_by_registered_screen_id(screen_id);
	if(!ts) {
		pr_err("%s: get invalid screen_id:0x%x from server", __func__, screen_id);
		return;
	}

#ifdef USE_SHAREMEM_POINTINFO
	plocinfo = &ts->locinfo;

	if (!ts->buffer.size || !ts->buffer.data) {
		pr_err("Screen(%d-0x%x): failed to check buffer of point info, is shared memory initialized?\n",
			ts->requested_screen_inx, screen_id);
		return;
	}
	if (locinfo_size > sizeof(ts->locinfo)) {
		pr_err("Screen(%d-0x%x): out of data size (%d > max(%ld))\n",
			ts->requested_screen_inx, screen_id,
			locinfo_size, sizeof(ts->locinfo));
		return;
	}

	mutex_lock(&ts->tp_buffer_mutex);
	ret = read_from_ring_buffer(&ts->buffer, locinfo_offset, (char *)&ts->locinfo, locinfo_size);
	if(ret <= 0) {
		pr_err("Screen(%d-0x%x): failed to read pointinfo from ring buffer (in paddr: 0x%llx offset: 0x%x)",
			ts->requested_screen_inx, screen_id, ts->shmem_paddr, locinfo_offset);
		mutex_unlock(&ts->tp_buffer_mutex);
		return;
	}
	mutex_unlock(&ts->tp_buffer_mutex);

	if (screen_id != plocinfo->screen_id) {
		pr_err("The retrieved screen_id (0x%x) from shared memory and the msgbox's screen_id (0x%x) do not match.",
			plocinfo->screen_id, screen_id);
		return;
	}

	calculated_checksum = calculate_checksum((uint8_t *)&ts->locinfo, locinfo_size);
	//pr_debug("The offset:0x%x size:0x%x calculated_checksum (0x%x)  checksum(0x%x) ", locinfo_offset, locinfo_size, calculated_checksum, locinfo_chksum);

	if (calculated_checksum != locinfo_chksum) {
		pr_err("The calculated checksum (0x%x) of data from shared memory and the msgbox's checksum (0x%x) do not match.",
			calculated_checksum, locinfo_chksum);
		return;
	}
#else
	plocinfo = &locinfo;
#endif

#ifdef REPORT_INPUT_EVENT
	input_dev = ts->input_dev;
	if (input_dev == NULL) {
		pr_err("Not found input_dev in screen_id: 0x%08x!\n", screen_id);
		return;
	}
#endif

	if (plocinfo->slots > MAX_POINT) {
		dev_err(&input_dev->dev, "Out of max num touch slots defined, in server slots: %d screen_id: 0x%08x\n",
			plocinfo->slots, plocinfo->screen_id);
		return;
	}
#if (defined REPORT_SWAP_XY) || (defined REPORT_FLIP_X) || (defined REPORT_FLIP_X)
	prop = &ts->prop;
#endif

	//mutex_lock(&ts->tp_event_mutex);

	max_objects = plocinfo->slots;//or  plocinfo->point_lists.size
#ifdef MSGBOX_USE_FIXED_ARRAY
	object_data = plocinfo->point_lists;
#else
	object_data = plocinfo->point_lists.data;
#endif
	touch_count = 0;
#ifdef REPORT_POINTINFO_DEBUG
	lift_count = 0;
#endif

	for (idx = 0; idx < max_objects; idx++) {
		if (ts->hwinfo.transport_proto == TRANSPORT_PROTOTYPE_CHANGE) {
			/* change protocol, LIFT status only will be reported once, so we not need to check last LIFT status, just use it */
			status = object_data[idx].tool_type;
		} else {
			if (ts->prev_obj_status[idx] == LIFT &&
					object_data[idx].tool_type == LIFT)
				status = NOP;
			else
				status = object_data[idx].tool_type;
		}

		if (ts->hwinfo.transport_proto == TRANSPORT_PROTOTYPE_CHANGE)
			input_slot_idx = object_data[idx].tracking_id;
		else
			input_slot_idx = idx;

		switch (status) {
		case LIFT:
#ifdef REPORT_INPUT_EVENT
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(input_dev, input_slot_idx);
			if (ts->hwinfo.vendor == VENDOR_ID_HIMAX) {
				/* optional: */
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 0);
				input_report_abs(input_dev, ABS_MT_PRESSURE, 0);
			}
			input_mt_report_slot_state(input_dev,
					MT_TOOL_FINGER, 0);
#endif
#else
			pr_debug("LIFT %d\n", input_slot_idx);
#endif
#ifdef REPORT_POINTINFO_DEBUG
			lift_count++;
#endif
			break;
		case FINGER:
		case GLOVED_OBJECT:
			x = object_data[idx].x_pos;
			y = object_data[idx].y_pos;
			wx = object_data[idx].touch_major;
			wy = object_data[idx].touch_minor;

#ifdef REPORT_SWAP_XY
			if (prop->swap_x_y) {
				x = x ^ y;
				y = x ^ y;
				x = x ^ y;
			}
#endif
#ifdef REPORT_FLIP_X
			if (prop->max_x)
				x = prop->max_x - x;
#endif
#ifdef REPORT_FLIP_Y
			if (prop->max_y)
				y = prop->max_y - y;
#endif
#ifdef REPORT_INPUT_EVENT
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(input_dev, input_slot_idx);
			input_mt_report_slot_state(input_dev,
					MT_TOOL_FINGER, 1);
#endif
			input_report_key(input_dev, BTN_TOUCH, 1);
			input_report_key(input_dev, BTN_TOOL_FINGER, 1);
			input_report_abs(input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
#ifdef REPORT_TOUCH_WIDTH
			input_report_abs(input_dev,
					ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(input_dev,
					ABS_MT_TOUCH_MINOR, min(wx, wy));
			if (ts->hwinfo.vendor == VENDOR_ID_HIMAX) {
				input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, max(wx, wy));
				input_report_abs(input_dev, ABS_MT_PRESSURE, max(wx, wy));
			}
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(input_dev);
#endif
#endif
#ifdef REPORT_POINTINFO_DEBUG
			BST_TS_DEBUG_LVL2(&input_dev->dev, "%s (%d/0x%08x) Finger %d: pressure: %s, x=%d, y=%d w_major=%d, w_minor=%d\n",
				(ts->hwinfo.name[0] == '\0') ? "" : (char *) ts->hwinfo.name,
				ts->requested_screen_inx, plocinfo->screen_id,
				input_slot_idx,
				object_data[idx].pressure == 0 ? "free" : "push", x, y, wx, wy);
#endif

			touch_count++;
			break;
		default:
			break;
		}

		if (ts->hwinfo.transport_proto == TRANSPORT_PROTOTYPE_FULL)
			ts->prev_obj_status[idx] = object_data[idx].tool_type;
	}

	if (touch_count == 0) {
#ifdef REPORT_INPUT_EVENT
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_report_key(input_dev, BTN_TOOL_FINGER, 0);
#ifdef REPORT_POINTINFO_DEBUG
	BST_TS_DEBUG_LVL1(&input_dev->dev, "LIFT ALL");
#endif
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(input_dev);
#endif
#else
	pr_info("LIFT ALL\n");
#endif
	}

#ifdef REPORT_INPUT_EVENT
#ifdef REPORT_INPUT_MT_DROP_UNUSED
	input_mt_sync_frame(input_dev);// flags:INPUT_MT_DROP_UNUSED
#endif
	input_sync(input_dev);
#endif

STATISTICS_UPDATE_PACKET_COORD_TIMESTAMP(screen_id, TIMESTAMP_PACKET_COORD_SEND_EVENT_TO_INPUT_SYNC);
STATISTICS_UPDATE_PACKET_COORD_DATA(screen_id, plocinfo->point_lists[0].distance, NULL, 0);

#ifdef REPORT_POINTINFO_DEBUG
#ifdef REPROT_POINTINFO_DEBUG_MSGINX
	BST_TS_DEBUG_LVL1(&input_dev->dev, "%s (%d/0x%08x), msginx:%d transport_prototype:%s transport_slot_num:%d, touch_count:%d lift_count:%d\n",
		(ts->hwinfo.name[0] == '\0') ? "" : (char *) ts->hwinfo.name,
		ts->requested_screen_inx, plocinfo->screen_id,
		plocinfo->point_lists[0].distance,
		(ts->hwinfo.transport_proto == TRANSPORT_PROTOTYPE_CHANGE) ? "change" :
		(ts->hwinfo.transport_proto == TRANSPORT_PROTOTYPE_FULL) ? "full" :
		"unknown", plocinfo->slots, touch_count, lift_count);
#ifdef SUPPORT_PACKET_STATISTICS
	/* statistics related */
	ts->send_packets_count = plocinfo->point_lists[0].distance;
#endif
#else
	BST_TS_DEBUG_LVL1(&input_dev->dev, "%s (%d/0x%08x), transport_prototype:%s transport_slot_num:%d, touch_count:%d lift_count:%d\n",
		(ts->hwinfo.name[0] == '\0') ? "" : (char *) ts->hwinfo.name,
		ts->requested_screen_inx, plocinfo->screen_id,
		(ts->hwinfo.transport_proto == TRANSPORT_PROTOTYPE_CHANGE) ? "change" :
		(ts->hwinfo.transport_proto == TRANSPORT_PROTOTYPE_FULL) ? "full" :
		"unknown", plocinfo->slots, touch_count, lift_count);
#endif
#endif
#ifdef SUPPORT_PACKET_STATISTICS
	ts->received_packets_count++;
#endif
	//mutex_unlock(&ts->tp_event_mutex);

	return;
}

/**
 * @brief Callback for when touch subscription reply is received.
 *
 * @param err  Error code.
 * @param ext  Pointer to extended data.
 * @param info Pointer to extended info.
 */
static void on_broadcast_touch_sub_reply(int32_t err, void *ext,
					       const ext_info_t *info)
{
	struct client_info_t *client_info = (struct client_info_t *)ext;
	/*
	 * uuid = (header.pid << 8) | (header.sid << 4) | (header.fid);
	 * sid and fid from send-its for IPC_MSG_TYPE_REPLY. Refer to dispatch_message() of server
	 */
    if (err == 0) {
		if(client_info) {
			if (TS_GET_STATUS(client_info->status) < CLIENT_STATUS_REQUEST_RESOURCE_SUSS)
				TS_SET_STATUS(client_info->status, CLIENT_STATUS_SUBSCRIPTION_SUSS);
			wake_up_interruptible(&client_info->subscription_waitqueue);
		}
		pr_debug("Subscribe touchscreen success (uuid=0x%04x).\n", info->uuid);
	} else {
		pr_err("Subscribe touchscreen fail (uuid=0x%04x). Ret is %d.\n", info->uuid, err);
	}
}

/**
 * @brief Callback for when touch unsubscription reply is received.
 *
 * @param err  Error code.
 * @param ext  Pointer to extended data.
 * @param info Pointer to extended info.
 */
static void on_broadcast_touch_unsub_reply(int32_t err, void *ext,
					       const ext_info_t *info)
{
	struct client_info_t *client_info = (struct client_info_t *)ext;
	/*
	 * uuid = (header.pid << 8) | (header.sid << 4) | (header.fid);
	 * sid and fid from send-its for IPC_MSG_TYPE_REPLY. Refer to dispatch_message() of server
	 */
    if (err == 0) {
		if(client_info) {
			TS_SET_STATUS(client_info->status, CLIENT_STATUS_NONE);
			wake_up_interruptible(&client_info->subscription_waitqueue);
		}
		pr_info("Unsubscribe touchscreen success (uuid=0x%04x).\n", info->uuid);
	} else {
		pr_err("Unsubscribe touchscreen fail (uuid=0x%04x). Ret is %d.\n", info->uuid, err);
	}
}

/**
 * @brief Callback for reconnection work.
 */
static void client_reconnect_work(struct work_struct *work)
{
    struct client_info_t *client_info = container_of(to_delayed_work(work), struct client_info_t, reconnect_work);
    bst_ts_client_t *client = &client_info->ts_client->bst_touch_client;
    struct bst_ts_data *ts;
	bool is_retry_reconnect = false;
	bool is_rework = false;
    int ret, i;

	pr_debug("Running async client reconnect work\n");
	// re-subscribe
    if ((TS_GET_STATUS(client_info->status) == CLIENT_STATUS_SUBSCRIPTION_TIMEOUT) ||
        ((TS_GET_STATUS(client_info->srv_status) == SRV_STATUS_OFFLINE) &&
         (TS_GET_STATUS(client_info->status) >= CLIENT_STATUS_SUBSCRIPTION_SUSS))) {

        ret = client->location_info_sub(on_touch_info_received, (void *)client,
                                        NULL, on_broadcast_touch_sub_reply, (void *)client_info);

        TS_UPDATE_TO_NEXT_STATUS(client_info->status, CLIENT_STATUS_SUBSCRIPTION_TIMEOUT, CLIENT_STATUS_SUBSCRIPTION_REQUEST);
        if (!ret) {
            pr_debug("Send subscribe message succeeded (async). ret = %d\n", ret);
		} else {
			pr_debug("Send subscribe message failed (async). ret = %d\n", ret);
			goto retry;
		}
    }

	if (client_info->ts_data) {
		for (i = 0; i < client_info->request_screen_num; i++) {
			ts = client_info->ts_data[i];
			if (ts && ts->hwinfo.connected == false) {
				is_retry_reconnect = true;
				break;
			}
		}
	}

	/* re-request resource for each screen */
    if (((is_retry_reconnect == true) || (TS_GET_STATUS(client_info->srv_status) == SRV_STATUS_OFFLINE)) &&
        (TS_GET_STATUS(client_info->status) >= CLIENT_STATUS_REQUEST_RESOURCE_SUSS)) {

        for (i = 0; i < client_info->request_screen_num; i++) {
            ts = client_info->ts_data[i];
			if (!ts) {
				continue;
			}
			if (ts->hwinfo.connected == true) {
				continue;
			}
            ret = bst_touch_request_resouce(client_info->client_id, client_info->request_screen_id[i], &ts->hwinfo);
            if (ret < 0) {
                pr_err("Failed to request resource for screen-id: 0x%08x (client-id: 0x%08x). ret = %d\n",
                       client_info->request_screen_id[i], client_info->client_id, ret);
				goto retry;
			}

			if (ts->hwinfo.connected == false)
				is_rework = true;
        }
    }

	TS_SET_STATUS(client_info->srv_status, SRV_STATUS_ONLINE);

	if (is_rework) {
		goto retry;
	}

	return;
retry:
	if (client_info->reconnect_retry_count < RECONNECT_RETRY_MAX_COUNT) {
		/* retry to connect to server */
		schedule_delayed_work(&client_info->reconnect_work, msecs_to_jiffies(RECONNET_WORK_DELAY_MS));
		client_info->reconnect_retry_count++;
	}
}

/**
 * @brief Callback for when destination changed.
 *
 * @param flag Indicator of whether destination is online or offline.
 * @param ext  Pointer to extended data.
 */
static void on_dst_changed(bool flag, void* ext)
{
	struct client_info_t *client_info = (struct client_info_t *)ext;
	struct bst_ts_data *ts;
    int i;

	if (!client_info) {
		pr_err("%s: Invalid client_info\n", __func__);
		return;
	}

	if (flag) {
		pr_info("Touch server is online\n");
		schedule_delayed_work(&client_info->reconnect_work, msecs_to_jiffies(RECONNET_WORK_DELAY_MS));
	} else {
		pr_info("Touch server is offline\n");
		cancel_delayed_work_sync(&client_info->reconnect_work);
		/* Setting connected status of touchsreen to false */
		if (client_info->ts_data) {
			for (i = 0; i < client_info->request_screen_num; i++) {
				ts = client_info->ts_data[i];
				if (!ts) {
					continue;
				}
				ts->hwinfo.connected = false;
			}
		}
		TS_SET_STATUS(client_info->srv_status, SRV_STATUS_OFFLINE);
	}

	return;
}

/**
 * @brief Request touchscreen resource from server.
 *
 * @param client_id  Client ID.
 * @param screen_id  Screen ID.
 * @param info       Pointer to hardware info structure to store received data.
 * @return 0 on success, negative error code on failure.
 */
static int bst_touch_request_resouce(const uint32_t client_id, const uint32_t screen_id,
                              hw_info_t *info)
{
	int ret;
	uint64_t client_uuid;
#ifdef USE_SHAREMEM_POINTINFO
	struct bst_ts_data *ts;
#endif
	bst_ts_ErrorEnum_t err;
	request_info_t req_info;
	struct client_info_t *tc_info = touchclient_info;
	tsclient_t *ts_client = tc_info->ts_client;
	bst_ts_client_t *client;
	hw_info_t *server_hwinfo;

	if(!ts_client || !info)
		return -EINVAL;

	client = &touchclient_info->ts_client->bst_touch_client;
	if(!client)
		return -EINVAL;

	req_info.screen_id = screen_id;
#ifdef USE_SHAREMEM_POINTINFO
	ts = bst_ts_data_get_by_requesting_screen_id(screen_id);
	if(!ts) {
		pr_err("%s: get invalid screen_id:0x%x", __func__, screen_id);
		return -EINVAL;
	}
	/* send share memory paddr to service */
	if (tc_info->use_shmem) {
		req_info.shmem_paddr = ts->shmem_paddr;
		req_info.shmem_size = ts->shmem_size;
	} else {
		req_info.shmem_paddr = 0ULL;
		req_info.shmem_size = 0UL;
	}
#else
	req_info.shmem_paddr = 0ULL;
	req_info.shmem_size = 0UL;
#endif

	ret = client->client_request_location_init_sync(client_id, &req_info, &client_uuid, &server_hwinfo, &err, 1000,
				&touch_des_buf);
	if (ret < 0 || !server_hwinfo) {
		if (ret < 0)
			pr_err("Failed to request touch(client_id:0x%x screen_id:0x%x) resource. (communication failure? ret=%d).\n", client_id, screen_id, ret);
		else
			pr_err("Failed to request touch(client_id:0x%x screen_id:0x%x) resource. (reason: %s (errcode=%d)).\n", client_id, screen_id, errcode_msg(err), err);
		return -EINVAL;
	}
	memcpy(info, server_hwinfo, sizeof(hw_info_t)); //for Generator Version: francaidl 3a7f767 msgbx_ipc 001bddd

	if (screen_id != info->screen_id) {
		pr_err("The retrieved screen_id(0x%x) and the requesting screen_id(0x%x) do not match for client-id(0x%x).\n",
			info->screen_id, screen_id, client_id);
		return -EINVAL;
	}

	pr_debug("%s: client_id:0x%x request touch(screen_id:0x%x) resource get client_uuid:0x%llx "
		"touchscreen(screen_id:0x%x vendor:%d product:%d versions:%d x_max:%d y_max:%d touch_num_max:%d name:%s phy:%s) err:%d\n",
		__func__, client_id, screen_id, client_uuid,
		info->screen_id, info->vendor, info->product, info->versions,
		info->x_max, info->y_max, info->touch_num_max, info->name, info->phys, err);

	if (info->connected == true)
		pr_info("Touchscreen(screen_id:0x%x phy:%s) is connected.\n", info->screen_id, info->phys);
	else
		pr_debug("Touchscreen(screen_id:0x%x phy:%s) is not connected.\n", info->screen_id, info->phys);

	return err;
}

#ifdef SUPPORT_CALIBRATION
/**
 * @brief Get touch calibration information for the specified screen ID.
 *
 * @param screen_id The ID of the screen for which calibration information is requested.
 * @param cali_info Pointer to the structure where the calibration information will be stored.
 * @return int Returns 0 on success, or a negative error code on failure.
 */
static int bst_touch_get_calibration(uint32_t screen_id, calibration_info_t *cali_info)
{
	int ret;
	bst_ts_client_t *client;
	bst_ts_ErrorEnum_t err;
	struct client_info_t *tc_info = touchclient_info;
	tsclient_t *ts_client = tc_info->ts_client;
	calibration_info_t *server_cali_info;

	if(!ts_client || !cali_info)
		return -EINVAL;

	client = &touchclient_info->ts_client->bst_touch_client;
	if(!client)
		return -EINVAL;

	ret = client->get_touch_calibration_sync(screen_id, &server_cali_info, &err, 1000,
				&touch_des_buf);
	if (ret < 0 || !server_cali_info) {
		if (ret < 0)
			pr_err("Failed to get touch(screen_id:0x%x) calibration. (communication failure? ret=%d).", screen_id, ret);
		else
			pr_err("Failed to get touch(screen_id:0x%x) calibration. (reason: %s (errcode=%d)).", screen_id, errcode_msg(err), err);
		return -EINVAL;
	}
	memcpy(cali_info, server_cali_info, sizeof(calibration_info_t)); //for Generator Version: francaidl 3a7f767 msgbx_ipc 001bddd

	return 0;
}

/**
 * @brief Set touch calibration information for a specific screen
 *
 * @param screen_id Identifier of the screen
 * @param cali_info Pointer to the calibration information
 * @return 0 on success, -EINVAL on failure
 */
static int bst_touch_set_calibration(uint32_t screen_id, calibration_info_t *cali_info)
{
	int ret;
	bst_ts_client_t *client;
	bst_ts_ErrorEnum_t err;
	struct client_info_t *tc_info = touchclient_info;
	tsclient_t *ts_client = tc_info->ts_client;

	if(!ts_client || !cali_info)
		return -EINVAL;

	client = &touchclient_info->ts_client->bst_touch_client;
	if(!client)
		return -EINVAL;

	ret = client->set_touch_calibration_sync(screen_id, cali_info, &err, 1000,
				&touch_des_buf);
	if (ret < 0) {
		if (ret < 0)
			pr_err("Failed to set touch(screen_id:0x%x) calibration. (communication fail? ret=%d).", screen_id, ret);
		else
			pr_err("Failed to set touch(screen_id:0x%x) calibration. (reason: %s (errcode=%d)).", screen_id, errcode_msg(err), err);
		return -EINVAL;
	}

	return 0;
}

#ifdef USE_UNLOCKED_IOCTL
static long bst_touch_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#else
static int bst_touch_ioctl(struct inode *inp, struct file *filp,
	unsigned int cmd, unsigned long arg)
#endif
{
	struct bst_ts_data *ts = file->private_data;
	uint32_t screen_id;
	uint16_t connected;
	int ret;

	if (!ts)
		return -EINVAL;

	switch (cmd) {
	case BST_TOUCH_GET_POINTINFO:
#ifdef USE_SHAREMEM_POINTINFO
		mutex_lock(&ts->tp_buffer_mutex);
#endif
		if (copy_to_user((void __user *)arg, &ts->locinfo, sizeof(point_info_t))) {
#ifdef USE_SHAREMEM_POINTINFO
			mutex_unlock(&ts->tp_buffer_mutex);
#endif
			return -EFAULT;
		}
#ifdef USE_SHAREMEM_POINTINFO
		mutex_unlock(&ts->tp_buffer_mutex);
#endif
		return 0;
	default:
		break;
	}

	mutex_lock(&ts->mutex);
	switch (cmd) {
	case BST_TOUCH_GET_CALIBRATION:
	{
		calibration_info_t cali_info;
		screen_id = ts->hwinfo.screen_id;
		ret = bst_touch_get_calibration(screen_id, &cali_info);
		if (ret != 0) {
			mutex_unlock(&ts->mutex);
			return -EFAULT;
		}
		if (copy_to_user((void __user *)arg, &cali_info, sizeof(calibration_info_t))) {
			mutex_unlock(&ts->mutex);
			return -EFAULT;
		}
		break;
	}
	case BST_TOUCH_SET_CALIBRATION:
	{
		calibration_info_t cali_info;
		screen_id = ts->hwinfo.screen_id;
		if (copy_from_user(&cali_info, (void __user *)arg, sizeof(calibration_info_t))) {
			mutex_unlock(&ts->mutex);
			return -EFAULT;
		}
		ret = bst_touch_set_calibration(screen_id, &cali_info);
		if (ret != 0) {
			mutex_unlock(&ts->mutex);
			return -EFAULT;
		}
		break;
	}
	case BST_TOUCH_GET_SCREEN_ID:
		screen_id = ts->hwinfo.screen_id;
		if (put_user(screen_id, (uint32_t __user *)arg)) {
			mutex_unlock(&ts->mutex);
			return -EFAULT;
		}
		break;
	case BST_TOUCH_GET_CONNECT_STATUS:
		connected = ts->hwinfo.connected;
		if (put_user(connected, (uint16_t __user *)arg)) {
			mutex_unlock(&ts->mutex);
			return -EFAULT;
		}
		break;
	case BST_TOUCH_GET_HWINFO:
		if (copy_to_user((void __user *)arg, &ts->hwinfo, sizeof(hw_info_t))) {
			mutex_unlock(&ts->mutex);
			return -EFAULT;
		}
		break;
	default:
		mutex_unlock(&ts->mutex);
		return -ENOTTY;
	}
	mutex_unlock(&ts->mutex);
	return 0;
}

static struct file_operations bst_touch_fops = {
    .owner = THIS_MODULE,
#ifdef USE_UNLOCKED_IOCTL
	.unlocked_ioctl = bst_touch_ioctl,
#ifdef USE_COMPAT_IOCTL
	.compat_ioctl = NULL, //bst_touch_compat_ioctls,
#endif
#else
	.ioctl = bst_touch_ioctl,
#endif
};
#endif /* SUPPORT_CALIBRATION */

/**
 * @brief Configure the input device for BST touch.
 *
 * @param ts Pointer to the BST touch data structure.
 * @return 0 on success, negative error code on failure.
 */
static int bst_configure_input_dev(struct bst_ts_data *ts)
{
	int error;
	//int i;
	struct input_dev *input_dev = NULL;

	ts->max_touch_num = ts->hwinfo.touch_num_max;
	ts->prop.max_x = ts->hwinfo.x_max;
	ts->prop.max_y = ts->hwinfo.y_max;

	input_dev = devm_input_allocate_device(&ts->pdev->dev);
	if (!input_dev) {
		dev_err(&ts->pdev->dev, "Failed to allocate input device.");
		return -ENOMEM;
	}
	if (ts->hwinfo.name[0] != '\0') {
		/* snprintf(ts->name, sizeof(ts->name), "%s%s",
			ts->hwinfo.name,
			ts->hwinfo.connected ? "[connected]" : "");
		input_dev->name = ts->name; */
		input_dev->name = ts->hwinfo.name;
	} else {
		input_dev->name = "BST Virt TouchScreen";
	}
	if (ts->hwinfo.phys[0] != '\0')
		input_dev->phys = ts->hwinfo.phys;
	else
		input_dev->phys = "input/ts";
	snprintf(ts->uniq, sizeof(ts->uniq), "0x%08x", ts->hwinfo.screen_id);
	if (ts->uniq[0] != '\0')
		input_dev->uniq = ts->uniq;

	input_dev->id.bustype = BUS_I2C; //BUS_VIRTUAL

	if (ts->hwinfo.vendor)
		input_dev->id.vendor = ts->hwinfo.vendor;
	else
		input_dev->id.vendor = 0x0416;
	if (ts->hwinfo.product)
		input_dev->id.product = ts->hwinfo.product;
	else
		input_dev->id.product = 0x1001;
	if (ts->hwinfo.versions)
		input_dev->id.version = ts->hwinfo.versions;
	else
		input_dev->id.version = 0x1001;
	//if (ts->pdev)
	//	input_dev->dev.parent = ts->pdev->dev.parent;
	input_set_drvdata(input_dev, ts);

	/* Refer to Documentation/input/event-codes.rst */
	set_bit(EV_SYN, input_dev->evbit);
	/*
	* Only BTN_TOUCH == 1: TOUCHSCREEN
	* BTN_TOUCH == 1 AND BTN_TOOL_FINGER == 1: TOUCHPAD
	* Refer to Documentation/input/event-codes.rst
	*/
	set_bit(BTN_TOOL_FINGER, input_dev->keybit);

	input_set_capability(input_dev, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(input_dev, EV_ABS, ABS_MT_POSITION_Y);

	dev_info(&input_dev->dev, "%s: screen-%d(0x%08x) min_x %d, max_x %d, min_y %d, max_y %d\n", __func__,
		ts->requested_screen_inx, ts->hwinfo.screen_id,
		ts->hwinfo.x_min, ts->hwinfo.x_max,
		ts->hwinfo.y_min, ts->hwinfo.y_max);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ts->prop.max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ts->prop.max_y, 0, 0);

#ifdef REPORT_TOUCH_WIDTH
	if (ts->hwinfo.vendor == VENDOR_ID_HIMAX) {
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     ts->hwinfo.pressure_min,
			     ts->hwinfo.pressure_max,
			     0, 0);
#ifdef TYPE_B_PROTOCOL
		input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     ts->hwinfo.pressure_min,
			     ts->hwinfo.pressure_max,
			     0, 0);
		input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,
			     ts->hwinfo.pressure_min, //abs_width_min: 0
			     ts->hwinfo.pressure_max, //abs_width_max: 200
			     0, 0);
#endif
	} else {
		//input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	}
#endif

#ifdef REPORT_INPUT_MT_DROP_UNUSED
	error = input_mt_init_slots(input_dev, ts->max_touch_num,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
#else
	error = input_mt_init_slots(input_dev, ts->max_touch_num,
				    INPUT_MT_DIRECT);
#endif
	if (error) {
		dev_err(&input_dev->dev,
			"Failed to initialize MT slots: %d", error);
		goto err_register_input;
	}

	error = input_register_device(input_dev);
	if (error) {
		dev_err(&input_dev->dev,
			"Failed to register input device: %d", error);
		goto err_mt_init_slots;
	}
	ts->input_dev = input_dev;

	return 0;

err_mt_init_slots:
	input_mt_destroy_slots(input_dev);

err_register_input:
	input_free_device(input_dev);

	return error;
}

/**
 * @brief Remove the input device for BST touch.
 *
 * @param ts Pointer to the BST touch data structure.
 */
static void bst_remove_input_dev(struct bst_ts_data *ts)
{
	struct input_dev *input_dev;

	if (!ts || !ts->input_dev)
		return;

	input_dev = ts->input_dev;
	input_unregister_device(input_dev);
	ts->input_dev = NULL;
	return;
}

/**
 * @brief Thread function to initialize touch client.
 *
 * @param data Pointer to the client info structure.
 * @return 0 on success, negative error code on failure.
 */
static int thread_ts_init(void *data)
{
	struct client_info_t *info = (struct client_info_t *)data;

	if(!info)
		return -EINVAL;
	wait_event_interruptible(info->subscription_waitqueue, ((TS_GET_STATUS(info->status) == CLIENT_STATUS_SUBSCRIPTION_SUSS) || kthread_should_stop()));
	if(TS_GET_STATUS(info->status) == CLIENT_STATUS_SUBSCRIPTION_SUSS)
		bst_ts_init(info);
	info->wait_task = NULL;
	return 0;
}

/**
 * @brief Configure the message box for touch client.
 *
 * @param info Pointer to the client info structure.
 * @return 0 on success, negative error code on failure.
 */
static int bst_configure_msgbox(struct client_info_t *info)
{
	int32_t ret;
	tsclient_t *touchclient;
	bst_ts_client_t *client;
	ipc_inf_version_t version;
	static int initialize;

	if (initialize)
		return 0;
	initialize = 1;

	if(!info)
		return -EINVAL;

	TS_SET_STATUS(info->status, CLIENT_STATUS_NONE);

	info->ins.com_data.pid = PID;
	touchclient = touchclient_init(&info->ins);
	if (!touchclient)
    {
        pr_err("Init touch client failed.\n");
        return -EINVAL;
    }
	info->ts_client = touchclient;
    client = &touchclient->bst_touch_client;

    // get version
    version = client->version();
    pr_debug("Interface version: major %d, minor %d.\n", version.major, version.minor);

    // subscribe broadcast
	init_waitqueue_head(&info->subscription_waitqueue);

	// register server available changed callback
	client->register_avail_changed(on_dst_changed, (void *)info);
    // start touch_client
	ret = touchclient->start();
    if (ret < 0)
    {
        pr_err("Start touch client failed!\n");
        return -2;
    }else{
		pr_debug("Start touch client success!\n");
	}

	TS_SET_STATUS(info->status, CLIENT_STATUS_START);

	if (TS_GET_STATUS(info->status) == CLIENT_STATUS_START) {
		ret = client->location_info_sub(on_touch_info_received, (void *)client, NULL, on_broadcast_touch_sub_reply, (void *)info);
		if (ret < 0)
		{
			pr_err("Send subscribe message failed. ret = %d\n", ret);
			ret = -3;
			goto err_server_failed;
		}
		TS_UPDATE_TO_NEXT_STATUS(info->status, CLIENT_STATUS_START, CLIENT_STATUS_SUBSCRIPTION_REQUEST);
		pr_debug("Send subscribe message succeeded. ret = %d\n", ret);
	}

	ret = wait_for_subscription_finish(info, 100);
	if (ret < 0) {
		if(ret == -ETIMEDOUT)
			pr_warn("Subscription timeout!\n");
		/* continue initialization until server is ok */
		TS_UPDATE_TO_NEXT_STATUS(info->status, CLIENT_STATUS_SUBSCRIPTION_REQUEST, CLIENT_STATUS_SUBSCRIPTION_TIMEOUT);
		info->wait_task = kthread_run(thread_ts_init, (void *)info, "ts_wait_thread");
	}

    return 0;

err_server_failed:
	touchclient->stop();

	return ret;
}

/**
 * @brief Remove the message box for touch client.
 * 
 * @param info Pointer to the client info structure.
 * @return 0 on success, negative error code on failure.
 */
static int bst_remove_msgbox(struct client_info_t *info)
{
	int ret;
	tsclient_t *touchclient;
	bst_ts_client_t *client;

	if(!info)
		return -EINVAL;

	touchclient = info->ts_client;
	client = &touchclient->bst_touch_client;

	if (unlikely(info->wait_task)) {
		kthread_stop(info->wait_task);
		info->wait_task = NULL;
	}
	// unsubscribe broadcast
	ret = client->location_info_unsub(on_broadcast_touch_unsub_reply, (void *)NULL);
	if (ret < 0)
		pr_err("Send unsubscribe message failed. ret = %d\n", ret);
	else
		pr_info("Send unsubscribe message succeeded. ret = %d\n", ret);

	// stop touch_client
	ret = touchclient->stop();
	if (ret < 0)
		pr_err("Stop touch client failed!\n");
	else
		pr_info("Stop touch client success!\n");

	ret = touchclient_destroy();
	if (ret < 0)
		pr_err("Destroy touch client failed!\n");
	else
		pr_info("Destroy touch client success!\n");

	return 0;
}


#ifdef CONFIG_DEBUG_FS
static int bst_ts_debugfs_help_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "debug_level: 0-off, 1-point count, 2-pointinfo 3-pointinfo&count\n");
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(bst_ts_debugfs_help);

#define MAX_CMD_LEN 64
#define RESULT_BUF_SIZE 128
static char gcmd_result_buf[RESULT_BUF_SIZE];
static char *gcmd_result_ptr = gcmd_result_buf;

/**
 * @brief Display the touch shell help information.
 */
static void shell_touch_server_help(struct seq_file *s)
{
    seq_printf(s,
             "\nUsage: touch <command> [parameter]\n"
             "  touch help\n"
             "  touch show\n"
             "  touch get_route\n"
             "  touch set_route <value>\n"
             "        - <value>:\n"
             "          0 -	LOCATION_SENDTO_NONE\n"
             "          1 -	LOCATION_SENDTO_TOUCHMANAGER\n"
             "          2 -	LOCATION_SENDTO_CLIENTS\n"
    );

    seq_printf(s,
             "  touch get_tsapp_debug\n"
             "  touch set_tsapp_debug <value>\n"
             "        - <value>:\n"
             "          0 -	disable debug\n"
             "          1 -	enable debug\n"
    );

    seq_printf(s,
             "  touch get_tsdev_debug\n"
             "  touch set_tsdev_debug <value>\n"
             "        - <value>:\n"
             "          0 -	disable debug\n"
             "          1 -	enable debug for point count\n"
             "          2 -	enable debug for pointinfo\n"
             "          3 -	enable debug for pointinfo&count\n"
    );

    seq_printf(s,
             "  touch get_tsvirt_debug\n"
             "  touch set_tsvirt_debug <value>\n"
             "        - <value>:\n"
             "          0 -	disable debug\n"
             "          1 -	enable debug\n"
    );

    seq_printf(s,
             "  touch set_pkt_statistics <value>\n"
             "        - <value>:\n"
             "          0 -	disable packet statistics\n"
             "          1 -	enable packet statistics\n"
             "  touch reset_pkt_statistics\n"
    );
}

static int bst_ts_debugfs_cmd_show(struct seq_file *s, void *unused)
{
	shell_touch_server_help(s);
	return 0;
}

static int bst_ts_debugfs_cmd_open(struct inode *inode, struct file *file)
{
	return single_open(file, bst_ts_debugfs_cmd_show, inode->i_private);
}

static ssize_t bst_ts_debugfs_cmd_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	int len;
	char cmd[MAX_CMD_LEN];
	int ret;
	char *result_str;
	bst_touch_ErrorEnum_t err;
	//struct client_info_t *info = file->private_data;
	struct seq_file *m = (struct seq_file *)file->private_data;
	struct client_info_t *info = m ? ((struct client_info_t *) m->private) : NULL;
	tsclient_t *ts_client = info ? info->ts_client : NULL;
	bst_ts_client_t *client = ts_client ? &ts_client->bst_touch_client : NULL;

	if (!client)
		return -EINVAL;

	if (count >= sizeof(cmd))
		return -EINVAL;

	if (copy_from_user(cmd, buf, count))
		return -EFAULT;

	cmd[count] = '\0';

	ret = client->touch_debug_cmd_sync(cmd, &result_str, &err, 3000, &touch_des_buf);
	if (ret < 0 || !result_str) {
		if (ret < 0)
			pr_err("Failed to execute command: %s, (communication failure? ret=%d)", cmd, ret);
		else
			pr_err("Failed to execute command: %s, (reason: %s (errcode=%d)).", cmd, errcode_msg(err), err);
		return -EINVAL;
	}
	pr_info("%s\n", result_str);
	len = strlen(result_str) + 1 > RESULT_BUF_SIZE ? RESULT_BUF_SIZE : strlen(result_str) + 1;
	gcmd_result_buf[RESULT_BUF_SIZE - 1] = '\0';
	// memset(gcmd_result_buf, 0, sizeof(gcmd_result_buf));
	memcpy(gcmd_result_buf, result_str, len);

	return count;
}

static const struct file_operations bst_ts_debugfs_cmd_fops = {
	.owner = THIS_MODULE,
	.open = bst_ts_debugfs_cmd_open,
	//.open = simple_open,
	.write = bst_ts_debugfs_cmd_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int bst_ts_debugfs_info_show(struct seq_file *s, void *unused)
{
	struct client_info_t *info = s->private;
	hw_info_t *touch_hwinfo;
	struct input_handle *handle;
	struct input_dev *dev;
	int inx;

	if (!info)
		return 0;

	seq_printf(s, "== Requested screeninfo of current client ==\n");
	seq_printf(s, "Client ID: 0x%x\n", info->client_id);
	seq_printf(s, "Requested screen num: %d\n", info->request_screen_num);
	for (inx = 0; inx < info->request_screen_num; inx++) {
		seq_printf(s, "    Requested screen-[%d](0x%08x)\n", inx, info->request_screen_id[inx]);
	}

	seq_printf(s, "\n== Touchscreen hardware info ==\n");
	for (inx = 0; inx < info->request_screen_num; inx++) {
		touch_hwinfo = &info->ts_data[inx]->hwinfo;
		dev = info->ts_data[inx]->input_dev;
        seq_printf(s, "Touchscreen-[%d](0x%08x)\n", inx, touch_hwinfo->screen_id);
		seq_puts(s, "    handlers:");
		list_for_each_entry(handle, &dev->h_list, d_node)
			seq_printf(s, "%s ", handle->name);
		seq_putc(s, '\n');
        seq_printf(s, "    name:%s\n", touch_hwinfo->name);
        seq_printf(s, "    phys:%s\n", touch_hwinfo->phys);
        seq_printf(s, "    vendor:%d\n", touch_hwinfo->vendor);
        seq_printf(s, "    product:%d\n", touch_hwinfo->product);
        seq_printf(s, "    x_max:%d\n", touch_hwinfo->x_max);
        seq_printf(s, "    y_max:%d\n", touch_hwinfo->y_max);
        seq_printf(s, "    pressure_max:%d\n", touch_hwinfo->pressure_max);
        seq_printf(s, "    transport_prototype:%s\n",
                 (touch_hwinfo->transport_proto == TRANSPORT_PROTOTYPE_CHANGE) ? "change" :
                 (touch_hwinfo->transport_proto == TRANSPORT_PROTOTYPE_FULL)   ? "full" :
                                                                                     "unknown");
        seq_printf(s, "    connected:%d\n\n", touch_hwinfo->connected);
    }

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(bst_ts_debugfs_info);

static int bst_ts_buf_show(struct seq_file *s, void *unused)
{
	char *buf = (char *)s->private;

	seq_printf(s, "%s\n", buf ? buf : "");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(bst_ts_buf);

static int bst_ts_debugfs_init(struct client_info_t *info)
{

#ifdef SUPPORT_PACKET_STATISTICS
	int i;
	struct input_dev *input_dev;
	struct dentry *input_dir;
#endif

	if (!info)
		return -EINVAL;

	bst_ts_debugfs_dir = debugfs_create_dir("bst_ts", NULL);
	if (!bst_ts_debugfs_dir)
		return -EINVAL;

	debugfs_create_file("info", S_IRUGO, bst_ts_debugfs_dir, info, &bst_ts_debugfs_info_fops);
	debugfs_create_file("help", S_IRUGO, bst_ts_debugfs_dir, NULL, &bst_ts_debugfs_help_fops);
	debugfs_create_file("cmd", (S_IWUSR | S_IRUGO), bst_ts_debugfs_dir, info, &bst_ts_debugfs_cmd_fops);
	debugfs_create_str("cmd_result", S_IRUGO, bst_ts_debugfs_dir, (char **)&gcmd_result_ptr);
	debugfs_create_x32("client_id", S_IRUGO, bst_ts_debugfs_dir, (uint32_t *)&info->client_id);
	debugfs_create_u8("request_screen_num", S_IRUGO, bst_ts_debugfs_dir, (uint8_t *)&info->request_screen_num);
	debugfs_create_u8("debug_level", (S_IWUSR | S_IRUGO), bst_ts_debugfs_dir, (uint8_t *)&bst_ts_debug_level);
#ifdef TOUCH_STATISTICS_PACKET_TIMESTAMP_TOOL_ENABLE
	debugfs_create_multi_touch_packet_timestamp_tool(bst_ts_debugfs_dir);
#endif

#ifdef SUPPORT_PACKET_STATISTICS
	for (i = 0; i < info->request_screen_num; i++) {
		if (!info->ts_data[i] || !info->ts_data[i]->input_dev)
			continue;

		input_dev = info->ts_data[i]->input_dev;
		input_dir = debugfs_create_dir(dev_name(&input_dev->dev), bst_ts_debugfs_dir);

		debugfs_create_u8("connected_status", S_IRUGO, input_dir, (uint8_t *)&info->ts_data[i]->hwinfo.connected);
		debugfs_create_u16("x_max", S_IRUGO, input_dir, (uint16_t *)&info->ts_data[i]->hwinfo.x_max);
		debugfs_create_u16("y_max", S_IRUGO, input_dir, (uint16_t *)&info->ts_data[i]->hwinfo.y_max);
		debugfs_create_x32("screen_id", S_IRUGO, input_dir, (uint32_t *)&info->ts_data[i]->hwinfo.screen_id);
		debugfs_create_u32("send_packets_count", S_IRUGO, input_dir, (uint32_t *)&info->ts_data[i]->send_packets_count);
		debugfs_create_u32("received_packets_count", (S_IWUSR | S_IRUGO), input_dir, (uint32_t *)&info->ts_data[i]->received_packets_count);
		debugfs_create_file("name", S_IRUGO, input_dir, (void *)info->ts_data[i]->hwinfo.name, &bst_ts_buf_fops);
		debugfs_create_file("phys", S_IRUGO, input_dir, (void *)&info->ts_data[i]->hwinfo.phys, &bst_ts_buf_fops);
#ifdef TOUCH_STATISTICS_PACKET_TIMESTAMP_TOOL_ENABLE
		debugfs_create_single_touch_packet_timestamp_tool(input_dir, i);
#endif
	}
#endif
	return 0;
}

static void bst_ts_debugfs_exit(void)
{
	if (bst_ts_debugfs_dir) {
		debugfs_remove_recursive(bst_ts_debugfs_dir);
		bst_ts_debugfs_dir = NULL;
	}
}

#endif

/**
 * @brief Parse device tree to get client ID and screen IDs.
 *
 * @param dev          Pointer to the device structure.
 * @param client_info  Pointer to the client info structure.
 * @return 0 on success, negative error code on failure.
 */
static int bst_ts_parse_dt(struct device *dev, struct client_info_t *client_info)
{
	int i;
	uint32_t pval;
	int num_screens;
	struct device_node *node = dev_of_node(dev);

	if(!node || !client_info)
		return -EINVAL;

	// Parse client-id property
	if (of_property_read_u32(node, "client-id", &pval)) {
		pr_err("Failed to read client-id property\n");
		return -EINVAL;
	}
	client_info->client_id = pval;

	pr_info("client-id: 0x%x\n", client_info->client_id);

	// Parse screens-id property
	num_screens = of_property_count_elems_of_size(node, "screens-id", sizeof(u32));
	if (num_screens < 0) {
		pr_err("Failed to read screens-id property\n");
		return -EINVAL;
	}

	if (num_screens > MAX_SCREEN) {
		pr_err("Number of screens (%d) exceeds the maximum limit (%d)\n", num_screens, MAX_SCREEN);
		return -EINVAL;
	}
	client_info->request_screen_num = num_screens;

	for (i = 0; i < num_screens; i++) {
		if (of_property_read_u32_index(node, "screens-id", i, &pval)) {
			pr_err("Failed to read screens-id[%d] property\n", i);
			return -EINVAL;
		}
		client_info->request_screen_id[i] = pval;
	}

	return 0;
}

/**
 * @brief Initialize the touch client info and configure each screen.
 *
 * @param info  Pointer to the client info structure.
 * @return 0 on success, negative error code on failure.
 */
static int bst_ts_init(struct client_info_t *info)
{
	int ret, i;
	bool is_retry_reconnect = false;
	struct bst_ts_data *ts;
	struct platform_device *pdev;

	if(!info || !info->pdev)
		return -EINVAL;

	pdev = info->pdev;
	ret = bst_ts_parse_dt(&pdev->dev, info);
	if (ret < 0)
		return -EINVAL;

	// Allocate memory for info->ts_data
    info->ts_data = devm_kzalloc(&pdev->dev, info->request_screen_num * sizeof(struct bst_ts_data *), GFP_KERNEL);
    if (!info->ts_data) {
        return -ENOMEM;
	}

	for (i = 0; i < info->request_screen_num; i++) {
		ts = devm_kzalloc(&pdev->dev, sizeof(*ts), GFP_KERNEL);
		if (!ts) {
			ret = -ENOMEM;
			goto fail_alloc;
		}

		ts->pdev = pdev;
		ts->requested_screen_inx = i;
		ts->requested_screen_id = info->request_screen_id[i];
#ifdef SUPPORT_PACKET_STATISTICS
		ts->send_packets_count = 0;
		ts->received_packets_count = 0;
#endif
		info->ts_data[i] = ts;

#ifdef USE_SHAREMEM_POINTINFO
		mutex_init(&ts->tp_buffer_mutex);
		if (info->use_shmem && info->shmem_vaddr) {
			ts->shmem_size = RINGBUF_OFS_PER_SCREEN;
			ts->shmem_paddr = (phys_addr_t) paddr_64_to_paddr_32(info->phys_addr) + (i * ts->shmem_size);
			ts->shmem_vaddr = (void *)((phys_addr_t)info->shmem_vaddr + (i * ts->shmem_size));
			pr_debug("Screen (%d-0x%x) allocated coherent memory (vaddr: 0x%0llX, paddr: 0x%0llX size: %zu)\n",
				i, ts->requested_screen_id, (u64)ts->shmem_vaddr, ts->shmem_paddr, ts->shmem_size);

			init_ring_buffer(&ts->buffer, ts->shmem_vaddr, ts->shmem_size);
		}
#endif

		ret = bst_touch_request_resouce(info->client_id, info->request_screen_id[i], &ts->hwinfo);
		if (ret < 0) {
			ret = -EINVAL;
			goto fail_alloc;
		}
		if (ts->hwinfo.connected == false) {
			is_retry_reconnect = true;
		}
		// register input device for each screen
		ret = bst_configure_input_dev(ts);
		if (ret < 0) {
			ret = -EINVAL;
			goto fail_alloc;
		}

#ifdef SUPPORT_CALIBRATION
		ret = bst_touch_cdev_init(ts, &bst_touch_fops);
		if (ret < 0) {
			ret = -EINVAL;
			goto fail_alloc;
		}
#endif
	}

#ifdef TOUCH_STATISTICS_PACKET_TIMESTAMP_ENABLE
	// Initialize touch packet timestamp statistics for debug
	statistics_packet_timestamp_init(&pdev->dev, info->client_id, (void **)info->ts_data, info->request_screen_num);
#endif

#ifdef CONFIG_DEBUG_FS
	(void) bst_ts_debugfs_init(info);
#endif

	TS_SET_STATUS(info->status, CLIENT_STATUS_REQUEST_RESOURCE_SUSS);
	if (is_retry_reconnect) {
		schedule_delayed_work(&info->reconnect_work, msecs_to_jiffies(RECONNET_WORK_DELAY_MS));
	}
	pr_info("Touchscreen %s driver registered\n", dev_name(&pdev->dev));
	return 0;

fail_alloc:
    // Handle memory allocation failure
    for (i = 0; i < info->request_screen_num; i++) {
        if (info->ts_data[i]) {
#ifdef SUPPORT_CALIBRATION
			bst_touch_cdev_remove(info->ts_data[i]);
#endif
			bst_remove_input_dev(info->ts_data[i]);
            devm_kfree(&pdev->dev, info->ts_data[i]);
            info->ts_data[i] = NULL;
        }
    }
    devm_kfree(&pdev->dev, info->ts_data);
    info->ts_data = NULL;

    return ret;
}

/**
 * @brief Remove the touch client and free allocated resources.
 *
 * @param info  Pointer to the client info structure.
 */
static void bst_ts_remove(struct client_info_t *info)
{
	int i;
	struct platform_device *pdev = info->pdev;;

    for (i = 0; i < info->request_screen_num; i++) {
        if (info->ts_data[i]) {
#ifdef SUPPORT_CALIBRATION
			bst_touch_cdev_remove(info->ts_data[i]);
#endif
			bst_remove_input_dev(info->ts_data[i]);
            devm_kfree(&pdev->dev, info->ts_data[i]);
            info->ts_data[i] = NULL;
        }
    }
    devm_kfree(&pdev->dev, info->ts_data);
    info->ts_data = NULL;
}

/**
 * @brief Probe function for the platform driver.
 *
 * @param pdev  Pointer to the platform device structure.
 * @return 0 on success, negative error code on failure.
 */
static int bst_ts_drv_probe(struct platform_device *pdev)
{
	int rc;
	struct client_info_t *info;
#ifdef USE_SHAREMEM_POINTINFO
	dma_addr_t dma_paddr;
	//struct reserved_mem *rmem;
#endif

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	touchclient_info = info;
	atomic_set(&info->status, 0);
	atomic_set(&info->srv_status, 0);
	info->reconnect_retry_count = 0;
	INIT_DELAYED_WORK(&info->reconnect_work, client_reconnect_work);

	rc = bst_configure_msgbox(info);
	if (rc < 0) {
		pr_err("Failed to initialize msgbox client daemon!\n");
		goto err_init;
	}

	info->pdev = pdev;
	platform_set_drvdata(pdev, info);

#ifdef USE_SHAREMEM_POINTINFO
	info->use_shmem = false;
	/*
	rmem = of_reserved_mem_lookup(pdev->dev.of_node);
	if (!rmem) {
		pr_err("Failed to find reserved mem\n");
		goto err_msgbox;
	}
	*/
	/* Initialize reserved memory resources */
	rc = of_reserved_mem_device_init(&pdev->dev);
	if (rc && rc != -ENODEV) {
		pr_err("Failed to reserve memory\n");
		goto err_msgbox;
	}

	// dma_set_coherent_mask(&pdev->dev, 0xFFFFFFFF);
	#ifdef MSGBOX_USE_FIXED_ARRAY
	info->shmem_size = (MAX_SCREEN * RINGBUF_NUM_PER_SCREEN * sizeof(point_info_t));
	#else
	info->shmem_size = (MAX_SCREEN * RINGBUF_NUM_PER_SCREEN * MAX_POINT * sizeof(point_data_t));
	#endif
	/* Allocate memory */
	info->shmem_vaddr = dma_alloc_coherent(&pdev->dev, info->shmem_size,
		&dma_paddr, GFP_KERNEL);

	if (!info->shmem_vaddr) {
		rc = -ENOMEM;
		pr_err("Failed to allocate DMA memory (size: %zu)\n", info->shmem_size);
		goto err_of_reserved;
	}
	info->phys_addr = dma_to_phys(&pdev->dev, dma_paddr);
	info->use_shmem = true;

	pr_debug("Allocated coherent memory (vaddr: 0x%0llX, paddr: 0x%0llX size: %zu aligned size: %zu)\n",
		(u64)info->shmem_vaddr, info->phys_addr, info->shmem_size, PAGE_ALIGN(info->shmem_size));
#endif

	if (TS_GET_STATUS(info->status) == CLIENT_STATUS_SUBSCRIPTION_SUSS) {
		rc = bst_ts_init(info);
		if (rc < 0) {
#ifdef USE_SHAREMEM_POINTINFO
			goto err_alloc_coherent;
#else
			goto err_msgbox;
#endif
		}
	}
	pr_info("Touchscreen %s driver probed", dev_name(&pdev->dev));

    return 0;
#ifdef USE_SHAREMEM_POINTINFO
err_alloc_coherent:
	dma_free_coherent(&pdev->dev, info->shmem_size, info->shmem_vaddr, phys_to_dma(&pdev->dev, info->phys_addr));
err_of_reserved:
	of_reserved_mem_device_release(&pdev->dev);
#else
err_alloc_coherent:
#endif
err_msgbox:
	if (info->ts_client && info->ts_client->stop)
		info->ts_client->stop();
err_init:
	devm_kfree(&pdev->dev, info);
	return rc;
}

/**
 * @brief Remove function for the platform driver.
 *
 * @param pdev  Pointer to the platform device structure.
 * @return 0 on success, negative error code on failure.
 */
static int bst_ts_drv_remove(struct platform_device *pdev)
{
	struct client_info_t *info = platform_get_drvdata(pdev);

#ifdef TOUCH_STATISTICS_PACKET_TIMESTAMP_ENABLE
	statistics_packet_timestamp_uninit(&pdev->dev, info->client_id, (void **)info->ts_data, info->request_screen_num);
#endif
#ifdef CONFIG_DEBUG_FS
	bst_ts_debugfs_exit();
#endif
#ifdef USE_SHAREMEM_POINTINFO
	dma_free_coherent(&pdev->dev, info->shmem_size, info->shmem_vaddr, phys_to_dma(&pdev->dev, info->phys_addr));
	of_reserved_mem_device_release(&pdev->dev);
#endif
	bst_ts_remove(info);
	bst_remove_msgbox(info);
	devm_kfree(&pdev->dev, info);
	touchclient_info = NULL;
	pr_info("Touchscreen %s driver removed\n", dev_name(&pdev->dev));
    return 0;
}

#ifdef CONFIG_PM
static int bst_ts_suspend(struct device *dev) {
	struct client_info_t *info = dev_get_drvdata(dev);

	if (!info->ts_client)
		return 0;

	/* optional */
	//if (info->status >= CLIENT_STATUS_START)
	//	info->ts_client->stop();

	return 0;
}

static int bst_ts_resume(struct device *dev)
{
	struct client_info_t *info = dev_get_drvdata(dev);
	struct bst_ts_data *ts;
	bst_ts_client_t *client;
	int ret;
	int i;

	if (!info->ts_client)
		return 0;

	/* optional */
	//if (info->status >= CLIENT_STATUS_START)
	//	info->ts_client->start();

	/* re-subscribe to server */
	if (TS_GET_STATUS(info->status) >= CLIENT_STATUS_SUBSCRIPTION_SUSS) {
		client = &info->ts_client->bst_touch_client;
		ret = client->location_info_sub(on_touch_info_received, (void *)client, NULL, on_broadcast_touch_sub_reply, (void *)NULL);
		if (ret < 0) {
			pr_err("Send subscribe message failed. ret = %d\n", ret);
		}
		pr_info("Send subscribe message succeeded. ret = %d\n", ret);
	}

	/* re-request resource for each screen */
	if (TS_GET_STATUS(info->status) >= CLIENT_STATUS_REQUEST_RESOURCE_SUSS) {
		for (i = 0; i < info->request_screen_num; i++) {
			ts = info->ts_data[i];
			ret = bst_touch_request_resouce(info->client_id, info->request_screen_id[i], &ts->hwinfo);
			if (ret < 0)
				pr_err("Failed to request resource for screen-id: 0x%08x (client-id: 0x%08x). ret = %d\n", info->request_screen_id[i], info->client_id, ret);
		}
	}

	return 0;
}

static const struct dev_pm_ops bst_ts_dev_pm_ops = {
	.suspend = bst_ts_suspend,
	.resume = bst_ts_resume,
	//.freeze = bst_ts_suspend,
	//.restore = bst_ts_resume,
};
#endif

static const struct of_device_id bst_ts_of_match[] = {
    {.compatible = "bst,virt-ts", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bst_ts_of_match);

static struct platform_driver bst_ts_driver = {
    .driver = {
        .name = "bst-virt-ts",
        .of_match_table = of_match_ptr(bst_ts_of_match),
#ifdef CONFIG_PM
        .pm = &bst_ts_dev_pm_ops,
#endif
    },
    .probe = bst_ts_drv_probe,
    .remove = bst_ts_drv_remove,
};

module_platform_driver(bst_ts_driver);

MODULE_AUTHOR("Pengcheng Xue");
MODULE_DESCRIPTION("BST virtual Touchscreen Driver");
MODULE_LICENSE("GPL v2");
