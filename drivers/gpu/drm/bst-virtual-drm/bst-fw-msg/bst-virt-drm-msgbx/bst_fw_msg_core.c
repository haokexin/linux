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
#include "bst_display_platform.h"
#include "bst_display_global_api.h"
#include "display-client/src-gen/display_client.h"
#include "DisplayClient.h"

struct fw_msg_events {
	disp_event_callback_t cb;
	uint32_t subdev;
	uint8_t sub_status;
};

#define WAIT_CMD_TIMEOUT (33)  // wait N ms will timeout
static DisplayClient_data_t  __display_client_data;
static DisplayClient_t *__pclient = NULL;
static struct fw_msg_events display_events[BST_SUBDEV_MAX - 1];

static void dc_pipe0_triggered(const display_event_status_t *status, void *ext, const ext_info_t *info)
{
	struct bst_display_events_status evts = {
		.client_id = status->client_id, .events = status->events_type
	};
	DISP_DBG("Receive dc pipe0 broadcast. status is %#x\n", evts.events);
	if (display_events[BST_SUBDEV_DC0_PIPE0 - 1].cb)
		display_events[BST_SUBDEV_DC0_PIPE0 - 1].cb(evts, ext);
}

static void dc_pipe1_triggered(const display_event_status_t *status, void *ext, const ext_info_t *info)
{
	struct bst_display_events_status evts = {
		.client_id = status->client_id, .events = status->events_type
	};
	DISP_DBG("Receive dc pipe1 broadcast. status is %#x\n", evts.events);
	if (display_events[BST_SUBDEV_DC0_PIPE1 - 1].cb)
		display_events[BST_SUBDEV_DC0_PIPE1 - 1].cb(evts, ext);
}

static void dc_pipe2_triggered(const display_event_status_t *status, void *ext, const ext_info_t *info)
{
	struct bst_display_events_status evts = {
		.client_id = status->client_id, .events = status->events_type
	};
	DISP_DBG("Receive dc pipe2 broadcast. status is %#x\n", evts.events);
	if (display_events[BST_SUBDEV_DC1_PIPE0 - 1].cb)
		display_events[BST_SUBDEV_DC1_PIPE0 - 1].cb(evts, ext);
}

static void dc_pipe3_triggered(const display_event_status_t *status, void *ext, const ext_info_t *info)
{
	struct bst_display_events_status evts = {
		.client_id = status->client_id, .events = status->events_type
	};
	DISP_DBG("Receive dc pipe3 broadcast status is %#x\n", evts.events);
	if (display_events[BST_SUBDEV_DC1_PIPE1 - 1].cb)
		display_events[BST_SUBDEV_DC1_PIPE1 - 1].cb(evts, ext);
}

static void dc_pipe4_triggered(const display_event_status_t *status, void *ext, const ext_info_t *info)
{
	struct bst_display_events_status evts = {
		.client_id = status->client_id, .events = status->events_type
	};
	DISP_DBG("Receive dc pipe4 broadcast. status is %#x\n", evts.events);
	if (display_events[BST_SUBDEV_DC2_PIPE0 - 1].cb)
		display_events[BST_SUBDEV_DC2_PIPE0 - 1].cb(evts, ext);
}

static void edp_triggered(const display_event_status_t *status, void *ext, const ext_info_t *info)
{
	struct bst_display_events_status evts = {
		.client_id = status->client_id, .events = status->events_type
	};
	DISP_DBG("Receive edp event broadcast. status is %#x\n",
		  status->events_type);
	if (display_events[BST_SUBDEV_eDP - 1].cb)
		display_events[BST_SUBDEV_eDP - 1].cb(evts, ext);
}

static void dsi0_triggered(const display_event_status_t *status, void *ext, const ext_info_t *info)
{
	struct bst_display_events_status evts = {
		.client_id = status->client_id, .events = status->events_type
	};
	DISP_DBG("Receive dsi0 event broadcast. status is %#x\n",
		  status->events_type);
	if (display_events[BST_SUBDEV_DSI0 - 1].cb)
		display_events[BST_SUBDEV_DSI0 - 1].cb(evts, ext);
}

static void dsi1_triggered(const display_event_status_t *status, void *ext, const ext_info_t *info)
{
	struct bst_display_events_status evts = {
		.client_id = status->client_id, .events = status->events_type
	};
	DISP_DBG("Receive dsi1 event broadcast. status is %#x\n",
		  status->events_type);
	if (display_events[BST_SUBDEV_DSI1 - 1].cb)
		display_events[BST_SUBDEV_DSI1 - 1].cb(evts, ext);
}

static void lvds0_triggered(const display_event_status_t *status, void *ext, const ext_info_t *info)
{
	struct bst_display_events_status evts = {
		.client_id = status->client_id, .events = status->events_type
	};
	DISP_DBG("Receive lvds0 event broadcast. status is %#x\n",
	       status->events_type);
	if (display_events[BST_SUBDEV_LVDS0 - 1].cb)
		display_events[BST_SUBDEV_LVDS0 - 1].cb(evts, ext);
}

static void lvds1_triggered(const display_event_status_t *status, void *ext, const ext_info_t *info)
{
	struct bst_display_events_status evts = {
		.client_id = status->client_id, .events = status->events_type
	};
	DISP_DBG("Receive lvds1 event broadcast. status is %#x\n",
	       status->events_type);
	if (display_events[BST_SUBDEV_LVDS1 - 1].cb)
		display_events[BST_SUBDEV_LVDS1 - 1].cb(evts, ext);
}

static void fw_msg_events_sub_reply(int32_t err, void *ext,
				const ext_info_t *info)
{
	if (err == 0)
		DISP_INFO("Subscribe subdev(%u) event success.\n",
			  *(uint32_t *)ext);
	else
		DISP_ERR("Subscribe subdev(%d) event fail. ret is %d.\n",
			 *(uint32_t *)ext, err);
}
static void fw_msg_events_unsub_reply(int32_t err, void *ext,
				const ext_info_t *info)
{
	if (err == 0)
		DISP_INFO("Unsubscribe subdev(%u) event success.\n",
			  *(uint32_t *)ext);
	else
		DISP_ERR("Unsubscribe subdev(%d) event fail. ret is %d.\n",
			 *(uint32_t *)ext, err);
}

int fw_msg_events_sub(uint32_t subdev, disp_event_callback_t cb, void *ext)
{
	int32_t ret = 0;

	display_events[subdev - 1].subdev = subdev;
	switch (subdev) {
	case BST_SUBDEV_DC0_PIPE0:
		ret = __pclient->display_client.dc_pipe0_event_sub(dc_pipe0_triggered,
						 (void *)ext, NULL,
						 fw_msg_events_sub_reply, &display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_DC0_PIPE1:
		ret = __pclient->display_client.dc_pipe1_event_sub(
			dc_pipe1_triggered, (void *)ext, NULL,
			fw_msg_events_sub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_DC1_PIPE0:
		ret = __pclient->display_client.dc_pipe2_event_sub(
			dc_pipe2_triggered, (void *)ext, NULL,
			fw_msg_events_sub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_DC1_PIPE1:
		ret = __pclient->display_client.dc_pipe3_event_sub(
			dc_pipe3_triggered, (void *)ext, NULL,
			fw_msg_events_sub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_DC2_PIPE0:
		ret = __pclient->display_client.dc_pipe4_event_sub(
			dc_pipe4_triggered, (void *)ext, NULL,
			fw_msg_events_sub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_eDP:
		ret = __pclient->display_client.edp_event_sub(
			edp_triggered, (void *)ext, NULL, fw_msg_events_sub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_DSI0:
		ret = __pclient->display_client.dsi0_event_sub(
			dsi0_triggered, (void *)ext, NULL, fw_msg_events_sub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_DSI1:
		ret = __pclient->display_client.dsi1_event_sub(
			dsi1_triggered, (void *)ext, NULL, fw_msg_events_sub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_LVDS0:
		ret = __pclient->display_client.lvds0_event_sub(
			lvds0_triggered, (void *)ext, NULL, fw_msg_events_sub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_LVDS1:
		ret = __pclient->display_client.lvds1_event_sub(
			lvds1_triggered, (void *)ext, NULL, fw_msg_events_sub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	default:
		break;
	}
	if (ret < 0) {
		DISP_ERR("send subscribe message fail. ret = %d\n", ret);
		display_events[subdev - 1].cb = NULL;
	} else {
		display_events[subdev - 1].cb = cb;
		display_events[subdev - 1].sub_status = 1;
	}
	return ret;
}
int fw_msg_events_unsub(uint32_t subdev)
{
	int32_t ret = 0;

	if (!display_events[subdev - 1].sub_status) {
		DISP_ERR("Error, Subdev(%d) not subscribed!\n", subdev);
		return -1;
	}
	switch (subdev) {
	case BST_SUBDEV_DC0_PIPE0:
		ret = __pclient->display_client.dc_pipe0_event_unsub(
			fw_msg_events_unsub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_DC0_PIPE1:
		ret = __pclient->display_client.dc_pipe1_event_unsub(
			fw_msg_events_unsub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_DC1_PIPE0:
		ret = __pclient->display_client.dc_pipe2_event_unsub(
			fw_msg_events_unsub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_DC1_PIPE1:
		ret = __pclient->display_client.dc_pipe3_event_unsub(
			fw_msg_events_unsub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_DC2_PIPE0:
		ret = __pclient->display_client.dc_pipe4_event_unsub(
			fw_msg_events_unsub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_eDP:
		ret = __pclient->display_client.edp_event_unsub(
			fw_msg_events_unsub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_DSI0:
		ret = __pclient->display_client.dsi0_event_unsub(
			fw_msg_events_unsub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_DSI1:
		ret = __pclient->display_client.dsi1_event_unsub(
			fw_msg_events_unsub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_LVDS0:
		ret = __pclient->display_client.lvds0_event_unsub(
			fw_msg_events_unsub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	case BST_SUBDEV_LVDS1:
		ret = __pclient->display_client.lvds1_event_unsub(
			fw_msg_events_unsub_reply,
			(void *)&display_events[subdev - 1].subdev);
		break;
	default:
		break;
	}
	if (ret < 0) {
		DISP_ERR("subdev(%d) send Unsubscribe message fail. ret = %d\n",
			 subdev, ret);
	}
	display_events[subdev - 1].cb = NULL;
	display_events[subdev - 1].sub_status = 0;

	return ret;
}

static int msgbx_pid_map(const char* type)
{
	u32 value;

	if (!strcmp(type, "CPU_0")) {
		value = CPU_0;
	} else if(!strcmp(type, "CPU_1")) {
		value = CPU_1;
	} else if(!strcmp(type, "CPU_2")) {
		value = CPU_2;
	} else if(!strcmp(type, "CPU_3")) {
		value = CPU_3;
	} else if(!strcmp(type, "CPU_4")) {
		value = CPU_4;
	} else if(!strcmp(type, "CPU_5")) {
		value = CPU_5;
	} else if(!strcmp(type, "CPU_6")) {
		value = CPU_6;
	} else if(!strcmp(type, "CPU_7")) {
		value = CPU_7;
	} else if(!strcmp(type, "CPUMP2_0")) {
		value = CPUMP2_0;
	} else if(!strcmp(type, "CPUMP2_1")) {
		value = CPUMP2_1;
	} else {
		DISP_ERR("msgbox pid type is wrong!");
		return -EINVAL;
	}

	return value;
}

static int bst_fw_msg_parse_dt(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *pid_type;
	int ret = 0;

	ret = of_property_read_string(dev->of_node, "msgbx-pid", &pid_type);
	if (ret) {
		DISP_ERR("%pOF: invalid 'msgbx-pid' property: %d\n",
				dev->of_node, ret);
		return -EINVAL;
	}

	__display_client_data.com_data.pid = msgbx_pid_map(pid_type);
	return ret;
}

int display_client_main_loop(void *arg)
{
	int32_t ret = 0;
	uint8_t count;

	for (count = 0; count < ARRAY_SIZE(display_events); count++) {
		display_events[count].cb = NULL;
		display_events[count].subdev = BST_SUBDEV_NONE;
		display_events[count].sub_status = 0;
	}

	__pclient = DisplayClient_init(&__display_client_data);
	if (!__pclient) {
		DISP_ERR("init client fail.\n");
		return -1;
	}
	ret = __pclient->start();
	if (ret < 0) {
		DISP_ERR("Display Client: start client failed!\n");
		return -2;
	}

	return 0;
}

struct task_struct *start_display_client(void)
{
	return kthread_create(display_client_main_loop, NULL,
			      "ipc-display-client-msgbox");
}

int ipc_build_msg_cmd(struct fw_msg_data *msg_data)
{
	des_buf_t buffer     = {0};
	display_bst_display_cmd_head_t fw_cmd_msg = { 0 };
	display_ErrorEnum_t err = { 0 };
	display_Array_Uint32_t user_cmd_data = { 0 };
	display_Array_Uint32_t user_ack_data = { 0 };
	int32_t ret;
	static unsigned int timeout_cnt = 0;

	if (!msg_data) {
		DISP_ERR("msg_data is null !\n");
		return -1;
	}

	fw_cmd_msg.client_id = msg_data->client_id;
	fw_cmd_msg.subdev_session = msg_data->subdev_session;
	fw_cmd_msg.cmdset = msg_data->cmdset;
	fw_cmd_msg.cmdid = msg_data->cmdid;
	user_cmd_data.data = msg_data->user_cmd_data;
	user_cmd_data.size = sizeof(msg_data->user_cmd_data) / sizeof(uint32_t);
	user_ack_data.data = msg_data->user_ack_data;
	user_ack_data.size = sizeof(msg_data->user_ack_data) / sizeof(uint32_t);
	if(!__pclient){
		DISP_ERR("__pclient is null !\n");
		return -1;
	}
	ret = __pclient->display_client.fw_msg_send_sync(user_cmd_data, &fw_cmd_msg,
					       &user_ack_data, &err, WAIT_CMD_TIMEOUT, &buffer);
	if(ret == -ERR_APP_TIMEOUT){
		DISP_INFO("ret is %d disp send time out,cnt:%d\n", ret,timeout_cnt++);
		return ret;
	} else if (ret < 0) {
		DISP_ERR("send method fw_msg_send_with_sync failed. ret is %d\n", ret);
		return ret;
	}
	memcpy(msg_data->user_ack_data, user_ack_data.data, user_ack_data.size * sizeof(uint32_t));

	return 0;
}

int ipc_client_destory(void)
{
	int ret = 0;

	DISP_INFO("display ipc client stop !\n");

	ret = __pclient->stop();
	if (ret < 0)
		DISP_ERR("display ipc client stop failed!\n");

	DISP_INFO("display ipc client destory !\n");

	ret = DisplayClient_destroy();
	if (ret < 0) {
		DISP_ERR("destory client fail.\n");
		return -3;
	}
	return 0;
}

int transfer_fw_msg(struct fw_msg_data *msg_data)
{
	int ret;

	ret = ipc_build_msg_cmd(msg_data);
	if (ret < 0) {
		if(ret != -ERR_APP_TIMEOUT)
			DISP_ERR("ipc_build_msg_cmd failed, ret = %d", ret);
		return ret;
	}

	return 0;
}

static int bst_fw_msg_probe(struct platform_device *pdev)
{
	static struct task_struct *g_ipc_tid;
	int ret;

	ret = bst_fw_msg_parse_dt(pdev);
	if (ret < 0) {
		return -EINVAL;
	}

	g_ipc_tid = start_display_client();
	wake_up_process(g_ipc_tid);

	return 0;
}

static int bst_fw_msg_remove(struct platform_device *pdev)
{
	ipc_client_destory();
	return 0;
}

static const struct of_device_id drv_dt_ids[] = {
	{ .compatible = "bst,bst-display-firmware-msgbx" },
	{ /* end node */ },
};
MODULE_DEVICE_TABLE(of, drv_dt_ids);

static struct platform_driver bst_fw_msg_driver = {
	.probe = bst_fw_msg_probe,
	.remove = bst_fw_msg_remove,
	.driver = {
		.name = "disp-fw-msg",
		.of_match_table = drv_dt_ids,
	},
};

module_platform_driver(bst_fw_msg_driver);
MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("BST Display-FW-MSG Driver");
MODULE_LICENSE("GPL v2");
