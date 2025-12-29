// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <ipc_trans_common.h>
#include "linux/types.h"
#include "usb_datatype.h"
#include "usb_client.h"
#include "UsbClient.h"
#include "usb_bst_virt_msg.h"

#define USB_INFO(fmt, ...) \
	pr_info("[%s] [%s:%d]" fmt, "usb_cmd", __func__, __LINE__, ##__VA_ARGS__)
#define USB_ERR(fmt, ...) \
	pr_err("[%s] [%s:%d]" fmt, "usb_cmd", __func__, __LINE__, ##__VA_ARGS__)
#define USB_DEBUG(fmt, ...) \
	pr_debug("[%s] [%s:%d]" fmt, "usb_cmd", __func__, __LINE__, ##__VA_ARGS__)

static UsbClient_data_t __usb_client_data;
static UsbClient_t *__pclient;

// releated to SESSION_PACKET_MSG_BUFFER_COUNT of R5, default to 8 for now
static int msg_max = 8;
// timeout for waitting time to send msg, default to 10 seconds
static int msg_timeout = 10;

module_param(msg_max, int, 0444);
MODULE_PARM_DESC(msg_max, "Maximum number of msg send,include ack and bulk in");
module_param(msg_timeout, int, 0444);
MODULE_PARM_DESC(msg_timeout, "msg send number reset timeout");

struct flow_control {
	wait_queue_head_t wait_queue;
	atomic_t msg_send_max;
};

static struct flow_control g_fc;

void try_send_msg(struct flow_control *fc)
{
	if (wait_event_interruptible_timeout(fc->wait_queue,
					     atomic_read(&fc->msg_send_max) > 0,
					     msg_timeout * HZ)) {
		atomic_dec(&fc->msg_send_max);
	} else {
		USB_ERR
		    ("Timeout occurred, msg_send_max did not become greater than 0.\n");
		atomic_set(&fc->msg_send_max, msg_max - 1);
	}
}

void msg_send_callback(struct flow_control *fc)
{
	atomic_inc(&fc->msg_send_max);
	wake_up(&fc->wait_queue);
}

static void usb_proxy_method_async_callback(const usb_bst_virsual_msg_t *
					    fw_cmd_msg,
					    const usb_ErrorEnum_t err,
					    void *ext, const ext_info_t *info)
{
	struct flow_control *fc = (struct flow_control *)ext;

	if (err != USB_NO_ERROR)
		USB_ERR("%s fail. ret is %d.\n", __func__, err);

	msg_send_callback(fc);
}

static void usb_proxy_event_sub_reply(int32_t err, void *ext,
				      const ext_info_t *info)
{
	if (err == 0)
		USB_DEBUG("Subscribe  event success.\n");
	else
		USB_ERR("Subscribe event fail. ret is %d.\n", err);
}

static void usb_proxy_event_unsub_reply(int32_t err, void *ext,
					const ext_info_t *info)
{
	if (err == 0)
		USB_DEBUG("Unsubscribe  event success.\n");
	else
		USB_ERR("Unsubscribe event fail. ret is %d.\n", err);
}

static void on_usb_proxy_event_triggered(const usb_bst_virsual_msg_t *
					 fw_cmd_msg, void *ext,
					 const ext_info_t *info)
{
	msg_sub_callback_t cb = ext;

	USB_DEBUG("Receive usb_proxy_event broadcast.\n");

	cb((void *)fw_cmd_msg);
}

// subscribe broadcast.
int usb_msg_sub(msg_sub_callback_t cb)
{
	int ret = 0;

	if (!__pclient) {
		USB_ERR("Client is NULL ,return\n");
		return -1;
	}
	ret = __pclient->usb_client.usb_proxy_event_sub(on_usb_proxy_event_triggered, cb, NULL,
				usb_proxy_event_sub_reply, NULL);
	if (ret < 0)
		USB_ERR("Client: send subscribe message fail. ret = %d\n", ret);

	return ret;
}
EXPORT_SYMBOL(usb_msg_sub);
// unsubscribe broadcast.
int usb_msg_unsub(void)
{
	int ret = 0;

	if (!__pclient) {
		USB_ERR("Client is NULL ,return\n");
		return -1;
	}
	ret = __pclient->usb_client.usb_proxy_event_unsub(usb_proxy_event_unsub_reply, NULL);
	if (ret < 0)
		USB_ERR("Client: send subscribe message fail. ret = %d\n", ret);

	return ret;
}
EXPORT_SYMBOL(usb_msg_unsub);

int usb_msg_send(usb_bst_virsual_msg_t *pdu)
{
	int ret;

	if (!__pclient) {
		USB_ERR("Client is NULL ,return\n");
		return -1;
	}
	if (!pdu->core_id)
		pdu->core_id = __usb_client_data.com_data.pid;
	try_send_msg(&g_fc);
	ret =
	    __pclient->usb_client.usb_proxy_method_async(pdu,
							 usb_proxy_method_async_callback,
							 &g_fc, NULL);
	if (ret < 0) {
		USB_ERR("send method usb_proxy_method_async failed. ret is %d\n", ret);
		msg_send_callback(&g_fc);
	}
	return ret;
}
EXPORT_SYMBOL(usb_msg_send);

static int msgbx_pid_map(const char *type)
{
	u32 value;

	if (!strcmp(type, "CPU_0")) {
		value = CPU_0;
	} else if (!strcmp(type, "CPU_1")) {
		value = CPU_1;
	} else if (!strcmp(type, "CPU_2")) {
		value = CPU_2;
	} else if (!strcmp(type, "CPU_3")) {
		value = CPU_3;
	} else if (!strcmp(type, "CPU_4")) {
		value = CPU_4;
	} else if (!strcmp(type, "CPU_5")) {
		value = CPU_5;
	} else if (!strcmp(type, "CPU_6")) {
		value = CPU_6;
	} else if (!strcmp(type, "CPU_7")) {
		value = CPU_7;
	} else if (!strcmp(type, "CPUMP2_0")) {
		value = CPUMP2_0;
	} else if (!strcmp(type, "CPUMP2_1")) {
		value = CPUMP2_1;
	} else {
		USB_ERR("msgbox pid type is wrong!");
		return -EINVAL;
	}

	return value;
}

static int usb_bst_virt_msg_parse_dt(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *pid_type;
	int ret = 0;

	ret = of_property_read_string(dev->of_node, "msgbox-pid", &pid_type);
	if (ret) {
		USB_ERR("%pOF: invalid 'msgbx-pid' property: %d\n",
			dev->of_node, ret);
		return -EINVAL;
	}

	__usb_client_data.com_data.pid = msgbx_pid_map(pid_type);
	return ret;
}
int get_system_pid(void)
{
	return __usb_client_data.com_data.pid;
}
EXPORT_SYMBOL(get_system_pid);

int usb_client_main_loop(void *arg)
{
	ipc_inf_version_t version;
	int32_t ret = 0;

	__pclient = UsbClient_init(&__usb_client_data);
	if (!__pclient) {
		USB_ERR("init client fail.\n");
		return -1;
	}
	// get version
	version = __pclient->usb_client.version();
	USB_DEBUG("usb Interface version: major %d, minor %d.\n",
		  version.major, version.minor);

	ret = __pclient->start();
	if (ret < 0) {
		USB_ERR("usb Client: start client failed!\n");
		return -2;
	}

	return 0;
}

static int usb_ipc_client_destory(void)
{
	int ret = 0;

	USB_DEBUG("usb ipc client stop !\n");
	if (!__pclient) {
		USB_ERR("Client is NULL ,return\n");
		return 0;
	}
	wake_up_all(&g_fc.wait_queue);
	ret = __pclient->stop();
	if (ret < 0)
		USB_ERR("usb ipc client stop failed!\n");

	USB_DEBUG("usb ipc client destory !\n");

	ret = UsbClient_destroy();
	if (ret < 0) {
		USB_ERR("destory client fail.\n");
		return -3;
	}

	return 0;
}

static int usb_bst_virt_msg_probe(struct platform_device *pdev)
{
	//static struct task_struct *g_ipc_tid;
	int ret;

	ret = usb_bst_virt_msg_parse_dt(pdev);
	if (ret < 0)
		USB_ERR("probe get dts fail,use default\n");

	init_waitqueue_head(&g_fc.wait_queue);
	atomic_set(&g_fc.msg_send_max, msg_max);

	usb_client_main_loop(NULL);

	return 0;
}

static int usb_bst_virt_msg_remove(struct platform_device *pdev)
{
	usb_ipc_client_destory();
	return 0;
}

static const struct of_device_id drv_dt_ids[] = {
	{.compatible = "bst,bst-virt-usb" },
	{ /* end node */  },
};

MODULE_DEVICE_TABLE(of, drv_dt_ids);

static struct platform_driver usb_bst_virt_msg_driver = {
	.probe = usb_bst_virt_msg_probe,
	.remove = usb_bst_virt_msg_remove,
	.driver = {
		   .name = "bst-virt-usb",
		   .of_match_table = drv_dt_ids,
		    },
};

module_platform_driver(usb_bst_virt_msg_driver);
MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("BST virt-usb-device-msg Driver");
MODULE_LICENSE("GPL v2");
