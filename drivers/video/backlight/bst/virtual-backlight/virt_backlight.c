// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#include <asm/memory.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include "backlight-client/BacklightClient.h"
#include "virt_bl_protocol.h"
#include "virt_backlight.h"

#define VIRT_BL_IPC_TIMEOUT_MS 3000

#define BUILD_VIRT_BL_MSG(_msg, _cmd_id, _data, _data_len)  \
    _msg.cmd_id = (_cmd_id);                                \
    _msg.msg_len = (_data_len);                             \
    _msg.content.data = (unsigned int*)(_data);             \
    _msg.content.size = ((_data_len)/sizeof(unsigned int))

#define BUILD_MSG_REQUEST_HEAD(_req, _client_id, _screen_id)  \
_req.client_id = (_client_id);                                \
_req.screen_id = (_screen_id)


struct virt_bl_broadcast_data {
    struct work_struct work;
    unsigned int cmd_id;
    unsigned int msg_len;
    unsigned char context_data[0];
};

/* data for ipc */
static BacklightClient_data_t ipc_client_data;
static BacklightClient_t *ipc_client;
static des_buf_t ipc_des_buf;
static bool ipc_server_ready;
/* end for ipc*/

static struct list_head virt_bl_dev_list;
static struct mutex virt_bl_dev_list_lock;

static int s_ref_cnt = 0;


static void bst_bl_brightness_change_notify(void *msg, unsigned int len)
{
    struct virt_bl_brightness_change_evt *event;
    struct virt_bl_resource *res;

    event = (struct virt_bl_brightness_change_evt *)msg;
    mutex_lock(&virt_bl_dev_list_lock);
	list_for_each_entry(res, &virt_bl_dev_list, entry) {
		if (res->client_id == event->client_id && res->screen_id == event->screen_id) {
            res->change_evt(res->priv, event->brightness);
        }
	}
    mutex_unlock(&virt_bl_dev_list_lock);
}

static void bst_bl_broadcast_work(struct work_struct *work)
{
    struct virt_bl_broadcast_data *work_data = container_of(work, struct virt_bl_broadcast_data, work);

    switch (work_data->cmd_id) {
        case ID_CMD_VIRT_BL_BRIGHTNESS_UPDATE:
            bst_bl_brightness_change_notify(work_data->context_data, work_data->msg_len);
            break;
        default :
            pr_err("unknow backlight broadcast msg:%d\n", work_data->cmd_id);
            break;
    }

    kfree(work_data);
}

static void bst_bl_broadcast_callback_async(
				const backlight_virt_bl_msg_t evt,
				void *ext,
				const ext_info_t *info)
{
    struct virt_bl_broadcast_data *work_data;
    unsigned int msg_len;

    msg_len = evt.content.size*sizeof(unsigned int);
    work_data = kmalloc(sizeof(struct virt_bl_broadcast_data) + msg_len , GFP_KERNEL);

    work_data->msg_len = msg_len;
    work_data->cmd_id = evt.cmd_id;
    memcpy(work_data->context_data, evt.content.data, msg_len);
    INIT_WORK(&work_data->work, bst_bl_broadcast_work);
    schedule_work(&work_data->work);
}

static int bst_bl_send_msg(unsigned int cmd_id,
                            void *req, unsigned int req_len,
                            void *rsp, unsigned int *rsp_len)
{
    des_buf_t *buffer;
    backlight_virt_bl_msg_t req_msg;
    backlight_virt_bl_msg_t rsp_msg;
    backlight_ErrorEnum_t err;
    unsigned int msg_len;
    int ret;

    buffer = kmalloc(sizeof(des_buf_t), GFP_KERNEL);
    if (!buffer) {
        pr_err("error! malloc des_buf_t size:%ld failed!\n", sizeof(des_buf_t));
        return -ENOMEM;
    }
    BUILD_VIRT_BL_MSG(req_msg, cmd_id, req, req_len);
    ret = ipc_client->backlight_client.virt_bl_request_sync(
            req_msg, &rsp_msg, &err, VIRT_BL_IPC_TIMEOUT_MS, buffer);
    if (ret || err) {
        pr_err("send backlight ipc message failed, ret:%d err:%d\n", ret, err);
        kfree(buffer);
        return ERR_VIRT_BL_IPC_ERR;
    }

    msg_len = rsp_msg.content.size*sizeof(unsigned int);
    memcpy(rsp, rsp_msg.content.data, *rsp_len > msg_len ? msg_len : *rsp_len);
    *rsp_len = msg_len;
    kfree(buffer);
    return 0;
}

struct virt_bl_resource * bst_bl_request_resource(unsigned int client_id,
                            unsigned int screen_id,
                            bst_bl_brightness_change_event_f evt,
                            void *priv)
{
    struct virt_bl_resource *new_res;
    struct virt_bl_request_resource_req req;
    struct virt_bl_request_resource_rsp rsp;
    unsigned int resp_len = sizeof(rsp);
    int ret;

    new_res = kzalloc(sizeof(*new_res), GFP_KERNEL);
    if (!new_res) {
        return NULL;
    }

    BUILD_MSG_REQUEST_HEAD(req, client_id, screen_id);
    ret = bst_bl_send_msg(ID_CMD_VIRT_BL_REQUEST_RESOURCE_REQ,
                            &req,
                            sizeof(req),
                            &rsp,
                            &resp_len);
    if (ret ) {
        pr_err("send backlight ipc message failed:%d\n", ret);
        kfree(new_res);
        return NULL;
    }

    if (rsp.err) {
        pr_err("received backlight server ret err:%d\n", rsp.err);
        kfree(new_res);
        return NULL;
    }

    new_res->client_id = client_id;
    new_res->screen_id = screen_id;
    new_res->change_evt = evt;
    new_res->priv       = priv;
    new_res->max_brightness = rsp.hwinfo.max_brightness;
    memset(new_res->hw_name, 0, BL_CAPACITY_NAME_LEN);
    strncpy(new_res->hw_name, rsp.hwinfo.name, BL_CAPACITY_NAME_LEN-1);

    mutex_lock(&virt_bl_dev_list_lock);
    list_add(&new_res->entry, &virt_bl_dev_list);
    mutex_unlock(&virt_bl_dev_list_lock);
    return new_res;
}

int bst_bl_release_resource(struct virt_bl_resource *res)
{
    struct virt_bl_release_resource_req req;
    struct virt_bl_release_resource_rsp rsp;
    unsigned int resp_len = sizeof(rsp);
    int ret;

    BUILD_MSG_REQUEST_HEAD(req, res->client_id, res->screen_id);
    ret = bst_bl_send_msg(ID_CMD_VIRT_BL_RELEASE_RESOURCE_REQ,
                            &req,
                            sizeof(req),
                            &rsp,
                            &resp_len);
    if (ret ) {
        pr_err("send backlight ipc message failed:%d\n", ret);
    }

    mutex_lock(&virt_bl_dev_list_lock);
    list_del(&res->entry);
    mutex_unlock(&virt_bl_dev_list_lock);
    return 0;
}

int bst_bl_set_brightness(struct virt_bl_resource *res, unsigned int brightness)
{
    struct virt_bl_set_brightness_req req;
    struct virt_bl_set_brightness_rsp rsp;
    unsigned int resp_len = sizeof(rsp);
    int ret;

    req.brightness = brightness;
    BUILD_MSG_REQUEST_HEAD(req, res->client_id, res->screen_id);
    ret = bst_bl_send_msg(ID_CMD_VIRT_BL_SET_BRIGHTNESS_REQ,
                            &req,
                            sizeof(req),
                            &rsp,
                            &resp_len);
    if (ret) {
        pr_err("send backlight ipc message failed:%d\n", ret);
        return ERR_VIRT_BL_IPC_ERR;
    }

    if (rsp.err) {
        pr_err("received backlight server ret err:%d\n", rsp.err);
        return ERR_VIRT_BL_SERVER_FAILED;
    }
    return 0;
}

int bst_bl_get_brightness(struct virt_bl_resource *res, unsigned int *brightness)
{
    struct virt_bl_get_brightness_req req;
    struct virt_bl_get_brightness_rsp rsp;
    unsigned int resp_len = sizeof(rsp);
    int ret;

    BUILD_MSG_REQUEST_HEAD(req, res->client_id, res->screen_id);
    ret = bst_bl_send_msg(ID_CMD_VIRT_BL_GET_BRIGHTNESS_REQ,
                            &req,
                            sizeof(req),
                            &rsp,
                            &resp_len);
    if (ret) {
        pr_err("send backlight ipc message failed:%d\n", ret);
        return ERR_VIRT_BL_IPC_ERR;
    }

    if (rsp.err) {
        pr_err("received backlight server ret err:%d\n", rsp.err);
        return ERR_VIRT_BL_SERVER_FAILED;
    }
    *brightness = rsp.brightness;
    return 0;
}

int bst_bl_declare_resource(struct virt_bl_resource *res)
{
    struct virt_bl_request_resource_req req;
    struct virt_bl_request_resource_rsp rsp;
    unsigned int resp_len = sizeof(rsp);
    int ret;

    BUILD_MSG_REQUEST_HEAD(req, res->client_id, res->screen_id);
    ret = bst_bl_send_msg(ID_CMD_VIRT_BL_REQUEST_RESOURCE_REQ,
                            &req,
                            sizeof(req),
                            &rsp,
                            &resp_len);
    if (ret) {
        pr_err("send backlight ipc message failed:%d\n", ret);
        return ERR_VIRT_BL_IPC_ERR;
    }

    if (rsp.err) {
        pr_err("received backlight server ret err:%d\n", rsp.err);
        return ERR_VIRT_BL_SERVER_FAILED;
    }

    return 0;
}

static void on_server_status_changed(bool flag, void *ext)
{
    pr_err("backlight server %s!\n", flag ? "online" : "offline"); 
    if (flag) {
        ipc_client->backlight_client.virt_bl_broadcast_sub(
                                            bst_bl_broadcast_callback_async,
                                            NULL,
                                            &ipc_des_buf,
                                            NULL,
                                            NULL);
    }
    ipc_server_ready = true;
}

static int bst_backlight_internal_init(void)
{
    int ret;

    mutex_init(&virt_bl_dev_list_lock);
    INIT_LIST_HEAD(&virt_bl_dev_list);
#ifdef CONFIG_BST_C1200_ADAS
    ipc_client_data.com_data.pid = CPU_4;
#elif defined(CONFIG_BST_C1200_IVI)
    ipc_client_data.com_data.pid = CPU_0;
#else
    ipc_client_data.com_data.pid = CPUMP2_0;
#endif
	ipc_client = BacklightClient_init(&ipc_client_data);
	if (!ipc_client) {
        pr_err("init backlight client fail.\n");
        return -EINVAL;
    }

    ipc_client->backlight_client.register_avail_changed(on_server_status_changed, NULL);
	ret = ipc_client->start();
    if (ret < 0) {
        pr_err("start backlight client failed!:%d\n", ret);
        return -EINVAL;
    }
    pr_info("start backlight client!\n");
    return 0;
}

int bst_backlight_init(void)
{
    int retry = 500;
    int ret;

    s_ref_cnt++;
    if (s_ref_cnt == 1) {
        ret = bst_backlight_internal_init();
        if (ret) {
            s_ref_cnt--;
            return ret;
        }
    }

    while ((!ipc_server_ready) && (retry-- > 0)) {
        msleep(10);
    }
    if (!ipc_server_ready) {
        pr_err("error, backlight server not ready!\n");
        return -EINVAL;
    }
    return 0;
}

static void bst_backlight_internal_deinit(void)
{
    ipc_server_ready = false;
    ipc_client->stop();
    BacklightClient_destroy();
}

void bst_backlight_deinit(void)
{
    if (s_ref_cnt <= 0) {
        return;
    }
    s_ref_cnt--;
    if (s_ref_cnt != 0) {
        return;
    }
    bst_backlight_internal_deinit();
}
