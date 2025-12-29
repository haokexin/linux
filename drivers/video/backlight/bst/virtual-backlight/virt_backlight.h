// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef _VIRT_BACKLIGHT_H
#define _VIRT_BACKLIGHT_H

#define ERR_VIRT_BL_NO_ERR  0
#define ERR_VIRT_BL_INVALID_PARA -1
#define ERR_VIRT_BL_IPC_ERR -2
#define ERR_VIRT_BL_SERVER_FAILED -3

#define BL_CAPACITY_NAME_LEN 20

struct bst_bl_capacity_info {
    unsigned int screen_id;
    unsigned int max_brightness;
    char name[BL_CAPACITY_NAME_LEN];
};

typedef void (*bst_bl_brightness_change_event_f)(void *priv, unsigned int brightness);

struct virt_bl_resource {
    struct list_head entry;
    unsigned int client_id;
    unsigned int screen_id;
    bst_bl_brightness_change_event_f change_evt;
    void *priv;

    //hardware info
    char hw_name[BL_CAPACITY_NAME_LEN];
    unsigned int max_brightness;
};

//申请背光资源
struct virt_bl_resource* bst_bl_request_resource(unsigned int client_id,
                            unsigned int screen_id,
                            bst_bl_brightness_change_event_f evt,
                            void *priv);

int bst_bl_release_resource(struct virt_bl_resource *res);

//调整背光值
int bst_bl_set_brightness(struct virt_bl_resource *res, unsigned int brightness);

//查询背光值
int bst_bl_get_brightness(struct virt_bl_resource *res, unsigned int *brightness);

int bst_bl_declare_resource(struct virt_bl_resource *res);

//初始化
int bst_backlight_init(void);

//deinit
void bst_backlight_deinit(void);

#endif