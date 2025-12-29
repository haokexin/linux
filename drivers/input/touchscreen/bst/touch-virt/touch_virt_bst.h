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

#ifndef _TOUCH_VIRT_BST_H_
#define _TOUCH_VIRT_BST_H_

/* using sharememory for pointinfo */
#define USE_SHAREMEM_POINTINFO
#define SUPPORT_CALIBRATION
#define SUPPORT_PACKET_STATISTICS

#ifdef USE_SHAREMEM_POINTINFO
#include <linux/of_reserved_mem.h>
#include <linux/dma-mapping.h>
#include <linux/dma-direct.h>
#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#ifdef SUPPORT_CALIBRATION
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/version.h>
#endif
#include "touchclient.h"


#ifdef CONFIG_BST_C1200_ADAS
#define PID CPU_4
#elif defined(CONFIG_BST_C1200_IVI)
#define PID CPU_0
#elif defined(CONFIG_BST_C1200_DB)
#define PID CPUMP2_0
#else
#define PID CPUMP2_0
#endif

#define TC_IPC(n) (BST_TOUCH_##n)
#define TC_IPC_RET(n) (BST_TOUCH_##n)
#define TC_IPC_ERROR_RET(n) NEGATIVE(BST_TOUCH_##n)
#define NEGATIVE(n) ((n) > 0 ? -(n) : (n))

#define MAX_SCREEN			TC_IPC(MAX_SCREEN)
#define MAX_CLIENT			TC_IPC(MAX_CLIENT)
#define MAX_POINT			TC_IPC(MAX_POINT_NUM)
#define SCREENID_LVDS0			TC_IPC(SCREENID_LVDS0)
#define SCREENID_LVDS1			TC_IPC(SCREENID_LVDS1)
#define SCREENID_DSI0			TC_IPC(SCREENID_DSI0)
#define SCREENID_DSI1			TC_IPC(SCREENID_DSI1)
#define SCREENID_EDP			TC_IPC(SCREENID_EDP)
#define VENDOR_ID_SYNA			TC_IPC(VENDOR_ID_SYNA)
#define VENDOR_ID_HIMAX			TC_IPC(VENDOR_ID_HIMAX)
#define PRODUCT_ID_SYNA_DEF		TC_IPC(PRODUCT_ID_SYNA_DEF)
#define PRODUCT_ID_HIMAX_DEF	TC_IPC(PRODUCT_ID_HIMAX_DEF)
#define PRODUCT_ID_HIMAX_83192A	TC_IPC(PRODUCT_ID_HIMAX_83192A)
#define PRODUCT_ID_HIMAX_83192D	TC_IPC(RODUCT_ID_HIMAX_83192D)
#define PRODUCT_ID_HIMAX_83193A	TC_IPC(PRODUCT_ID_HIMAX_83193A)
#define PRODUCT_ID_HIMAX_83180A	TC_IPC(PRODUCT_ID_HIMAX_83180A)
#define VERSION_ID_SYNA_DEF		TC_IPC(VERSION_ID_SYNA_DEF)
#define VERSION_ID_HIMAX_DEF	TC_IPC(VERSION_ID_HIMAX_DEF)
/** 
 * transport protocol: 
 * TRANSPORT_PROTO_FULL: 
 * 	- use full pointinfo with all fields, 
 * 	- full pointinfo contains all fields,
 * TRANSPORT_PROTO_CHANGE: 
 * 	- use partial pointinfo with only changed fields,
 * 	- change when using partial pointinfo with only changed fields
 *
 * Note: 
 * TRANSPORT_PROTO_FULL is used by default,
 * and it is considered to use TRANSPORT_PROTO_CHANGE only when necessary.
 */
#define TRANSPORT_PROTOTYPE_FULL	TC_IPC(TRANSPORT_PROTOTYPE_FULL)
#define TRANSPORT_PROTOTYPE_CHANGE	TC_IPC(TRANSPORT_PROTOTYPE_CHANGE)

typedef bst_touch_hw_info_t hw_info_t;
typedef bst_touch_point_info_t point_info_t;
typedef bst_touch_point_data_t point_data_t;
typedef bst_touch_request_info_t request_info_t;
typedef bst_touch_calibration_info_t calibration_info_t;
typedef touchclient_data_t tsclient_data_t;
typedef touchclient_t tsclient_t;
typedef bst_touch_client_t bst_ts_client_t;
typedef bst_touch_ErrorEnum_t bst_ts_ErrorEnum_t;

#ifdef USE_SHAREMEM_POINTINFO
typedef struct {
    char *data;
    int size;
    int head;
    int tail;
} ring_buffer;
#endif

struct bst_ts_data {
	int requested_screen_inx;
	uint32_t requested_screen_id;
	/* char name[64]; */
	char uniq[32]; /* @uniq: unique identification code for the device (if device has it) */
#ifdef USE_SHAREMEM_POINTINFO
	point_info_t locinfo;
	struct mutex tp_buffer_mutex;
	ring_buffer buffer;
	phys_addr_t shmem_paddr; /* pointinfo buffer paddr */
	void *shmem_vaddr; /* pointinfo buffer paddr */
	size_t shmem_size;
#endif
	hw_info_t hwinfo;//from server
	struct input_dev *input_dev;
	struct platform_device *pdev;//parent platform dev
    unsigned int max_touch_num;
	unsigned char prev_obj_status[MAX_POINT];
	struct touchscreen_properties prop;
#ifdef SUPPORT_CALIBRATION
	struct mutex mutex;
	/* Stuff related to cdev interface */
	dev_t devt;
#endif
#ifdef SUPPORT_PACKET_STATISTICS
	/* statistics related */
	uint32_t send_packets_count;
	uint32_t received_packets_count;
#endif
};

#endif /* _TOUCH_VIRT_BST_H_ */