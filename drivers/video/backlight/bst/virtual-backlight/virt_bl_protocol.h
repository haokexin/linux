// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef _VIRT_BL_PROTOCOL_H
#define _VIRT_BL_PROTOCOL_H

#define ID_CMD_VIRT_BL_REQUEST_RESOURCE_REQ 1
#define ID_CMD_VIRT_BL_REQUEST_RESOURCE_RSP 2
#define ID_CMD_VIRT_BL_RELEASE_RESOURCE_REQ 3
#define ID_CMD_VIRT_BL_RELEASE_RESOURCE_RSP 4
#define ID_CMD_VIRT_BL_SET_BRIGHTNESS_REQ 5
#define ID_CMD_VIRT_BL_SET_BRIGHTNESS_RSP 6
#define ID_CMD_VIRT_BL_GET_BRIGHTNESS_REQ 7
#define ID_CMD_VIRT_BL_GET_BRIGHTNESS_RSP 8

#define ID_CMD_VIRT_BL_BRIGHTNESS_UPDATE 20

struct virt_bl_request_resource_req {
    unsigned int client_id;
    unsigned int screen_id;
};

struct virt_bl_capacity_info {
    unsigned int screen_id;
    unsigned int max_brightness;
#define VIRT_BL_NAME_LEN 20
    char name[VIRT_BL_NAME_LEN];
};

struct virt_bl_request_resource_rsp {
    unsigned int err;
    struct virt_bl_capacity_info hwinfo; 
};

struct virt_bl_release_resource_req {
    unsigned int client_id;
    unsigned int screen_id;
};

struct virt_bl_release_resource_rsp {
    unsigned int err;
};

struct virt_bl_set_brightness_req {
    unsigned int client_id;
    unsigned int screen_id;
    unsigned int brightness;
};

struct virt_bl_set_brightness_rsp {
    unsigned int err;
};

struct virt_bl_get_brightness_req {
    unsigned int client_id;
    unsigned int screen_id;
};

struct virt_bl_get_brightness_rsp {
    unsigned int err;
    unsigned int brightness;
};

struct virt_bl_brightness_change_evt {
    unsigned int client_id;
    unsigned int screen_id;
    unsigned int brightness;
};

#endif