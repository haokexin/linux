// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef BST_DISPLAY_GLOBAL_API_H
#define BST_DISPLAY_GLOBAL_API_H

#include "bst_display_cmdset_api.h"

enum glb_cmdid_type {
	GLB_CMD_INVALED = 0x00,
	GLB_CMD_PROBE_SUBDEV = 0x01,
	GLB_CMD_IS_VALID_TOPO = 0x02,
	GLB_CMD_GET_ALL_SUBDEV_TOPO = 0x03,
	GLB_CMD_GET_SUBDEV_INFO = 0x05,
};

enum client_role_type {
	CLIENT_ROLE_INVALID,
	CLIENT_ROLE_OWNER,
	CLIENT_ROLE_NOT_OWNER,
};

#define MAX_CLIENT_NUM		(5U)

#define PRIVILEGE_NONE               (1U << 0U)
#define PRIVILEGE_SLAVE_COMPOSER_SET (1U << 1U)
#define PRIVILEGE_SLAVE_VM_SET       (1U << 2U)
#define PRIVILEGE_SLAVE_CONN_SET     (1U << 3U)
#define PRIVILEGE_SLAVE_WB_SET       (1U << 4U)

#define PRIVILEGE_ALL (\
		PRIVILEGE_SLAVE_COMPOSER_SET | \
		PRIVILEGE_SLAVE_VM_SET | \
		PRIVILEGE_SLAVE_CONN_SET | \
		PRIVILEGE_SLAVE_WB_SET)

#define SUBDEV_PROBE_INFO_MAX_SIZE	(16*4)

struct bst_subdev_probe_response {
	reply_base base;
	uint32_t exec_subdev;
	uint32_t subdev_session;
	//uint32_t status;
	uint32_t probed_info_size;
	// attach different subdev info
	// such as struct virt_dc_probe_info, struct virt_dp_probe_info ...
	uint32_t probed_info[SUBDEV_PROBE_INFO_MAX_SIZE / 4];
	// 1: means this client control the shared resource.
	// 0: means this client can't control but something to create local objects.
	uint8_t is_owner;
};

struct bst_subdev_probe_request {
	uint32_t want_subdev;
	uint32_t want_layer_num;
	uint32_t want_info_size;
};

struct bst_subdev_info_req {
	uint32_t want_subdev;
};

struct bst_all_subdev_topo_req {
	uint32_t reserve;
};

struct sub_dev_topo {
	uint8_t dc_subdev;
	uint8_t conn_subdev;
	uint32_t dc_subdev_session;
	uint32_t conn_subdev_session;
};

struct bst_all_subdev_topo {
	reply_base base;
	uint8_t num;
	struct sub_dev_topo topo[MAX_PIPE_NUM];
};

struct client_info {
	uint32_t client_id;
	uint8_t role;
	uint8_t submodule_num;
	uint8_t submodule_ids[SUBMODULE_IDS_MAX];
	uint32_t reserve;
};

struct client_list {
	uint8_t client_num;
	struct client_info cinfo[MAX_CLIENT_NUM];
};

struct bst_subdev_info_result {
	reply_base base;
	uint8_t related_subdev;
	uint8_t exec_subdev;
	struct client_list clist;
};

enum dev_dump_type {
	DC_REGISTER_START   = 0x0000,
	DC_REGISTER_MAX     = 0x003F,
	DC_MONITOR          = 0x0040,
};

struct bst_display_dev_dump {
	uint8_t type;
	uint32_t shmem_paddr_high;
	uint32_t shmem_paddr_low;
};

struct bst_display_dc_probed_info {
	uint32_t arch_id;
	uint16_t max_hsize;
	uint16_t max_vsize;
	uint8_t bus_width;
	uint8_t num_rich_layers;
	uint8_t supported_color_mgmts;
	uint8_t supported_layer_types;
	uint8_t supported_smmu_types;
	uint8_t supported_link_types;
	uint8_t scaler_num;
	uint8_t num_outputs;
	uint8_t num_submodules;
	uint8_t submodule_ids[SUBMODULE_ID_DC_MAX];
};

struct bst_display_topology_status {
	reply_base base;
	//uint8_t topology_status;
};

struct bst_display_events_status {
	uint32_t client_id; /* Distinguish which guestOS */
	uint32_t events; 
};

enum {
	RGB = 0U,
	YCBCR420 = 1U,
	YCBCR422 = 2U,
	YCBCR444 = 3U,
	YONLY = 4U,
	RAW = 5U
};

enum { ITU601 = 1, ITU709 = 2 };

enum { CEA = 1, VESA = 2 };

enum {
	COLOR_DEPTH_INVALID = 0,
	COLOR_DEPTH_6 = 6,
	COLOR_DEPTH_8 = 8,
	COLOR_DEPTH_10 = 10,
	COLOR_DEPTH_12 = 12,
	COLOR_DEPTH_16 = 16
};


enum {
	VCEA = 0,
	CVT = 1,
	DMT = 2,
	USER = 3,
	INVALID = 0xFF
};

enum {
	SCREEN_TIMING_ID0 = 0,
	SCREEN_TIMING_ID1 = 1,
	SCREEN_TIMING_ID2 = 2,
	SCREEN_TIMING_ID3 = 3,
	SCREEN_TIMING_ID4 = 4,
	SCREEN_TIMING_EDID = 0xFF,
};

struct screen_state {
	uint8_t  timing_id;
	uint32_t screen_ppi;    // * 100
	uint32_t hszie_mm;
	uint32_t vszie_mm;
	uint32_t hszie_pix;
	uint32_t vszie_pix;
	uint8_t  pix_format;
	uint8_t  pix_bpc;
};

#define MAX_DTD_ID_NUM 5
struct bst_display_dp_probed_info {
	uint32_t arch_id;
	uint8_t num_submodules;
	uint8_t num_outputs;
	uint8_t audio_support;
	uint16_t refresh_rate;
	uint8_t display_protocol;
	uint8_t video_timing_nums;
	uint8_t video_format; /* RGB ~ RAW */
	uint8_t rate;
	uint8_t bpc;
	uint8_t lanes;
	bool connected;
	uint8_t colorimetry;
	uint8_t dynamic_range;
	uint8_t submodule_ids[SUBMODULE_ID_DP_MAX];
	struct screen_state preferred_screen;
};

#define DISP_OUTPUT_O_LVDS		0
#define DISP_OUTPUT_E_LVDS		1
#define DISP_OUTPUT_DUAL_LVDS	2
#define DISP_OUTPUT_INVALID_LVDS	3

enum{
	LVDS_VESA_30 = 0,
	LVDS_JEIDA_30,
	LVDS_FORMAT3_30,
	LVDS_VESA_24,
	LVDS_JEIDA_24,
	LVDS_FORMAT3_24,
	LVDS_VESA_18,
	LVDS_JEIDA_18,
	LVDS_LINEAR_12,
	LVDS_NOLINEAR_12,
};

struct bst_display_lvds_probed_info {
	uint32_t arch_id;
	uint8_t num_submodules;
	uint8_t num_outputs;
	uint16_t refresh_rate;
	uint8_t display_protocol;
	uint8_t video_timing_nums;
	uint8_t color_mapping;   /* LVDS_VESA_30 ~ LVDS_JEIDA_24*/
	uint8_t output_mode;     /* DISPLAY_OUTPUT_O_LVDS ~ DISPLAY_OUTPUT_DUAL_LVDS*/
	uint8_t pixel_merge;
	uint8_t bpc;
	uint8_t video_format;
	uint8_t submodule_ids[SUBMODULE_ID_LVDS_MAX];
	struct screen_state preferred_screen;
};

enum mipi_dsi_pixel_format {
	MIPI_DSI_FMT_RGB888,
	MIPI_DSI_FMT_RGB666,
	MIPI_DSI_FMT_RGB666_PACKED,
	MIPI_DSI_FMT_RGB565,
	MIPI_DSI_FMT_RGB101010,
};

struct bst_display_mipi_probed_info {
	uint32_t arch_id;
	uint8_t num_submodules;
	uint8_t num_outputs;
	uint16_t refresh_rate;
	uint8_t display_protocol;
	uint8_t video_timing_nums;
	uint8_t lanes;
	uint8_t channel;
	uint8_t format;
	uint8_t bpc;
	uint16_t mode_flags;
	uint8_t submodule_ids[SUBMODULE_ID_MIPI_MAX];
	struct screen_state preferred_screen;
};

struct bst_display_topology_info {
	uint32_t dc_subdev_session;
	uint32_t conn_subdev_session;
};

/*  Detailed Timing Descriptions(DTD) */
struct dtd {
	uint16_t pixel_repetition_input;
	unsigned int pixel_clock; /* pixelclock in Hz */
	uint8_t interlaced; /* 1 for interlaced, 0 progressive */
	uint16_t h_active;
	uint16_t h_blanking;
	uint16_t h_image_size;
	uint16_t h_sync_offset; /* hactive front porch */
	uint16_t h_sync_pulse_width;
	uint8_t h_sync_polarity;
	uint16_t v_active;
	uint16_t v_blanking;
	uint16_t v_image_size;
	uint16_t v_sync_offset; /* vactive front porch */
	uint16_t v_sync_pulse_width;
	uint8_t v_sync_polarity;
};
struct video_timing {
	uint8_t display_protocol;
	uint8_t video_timing_id;
	struct dtd video_info;
};

typedef void (*disp_event_callback_t)(
				const struct bst_display_events_status status,
				void *ext);
int bst_display_glb_cmd_probe_subdev(
				struct bst_subdev_probe_request *request,
				struct bst_subdev_probe_response *response);
int bst_display_glb_cmd_get_subdev_info(
				struct bst_subdev_info_req *request,
				struct bst_subdev_info_result *response);
int bst_display_glb_cmd_get_all_subdev_topo(
				struct bst_all_subdev_topo_req *request,
				struct bst_all_subdev_topo *response);
int bst_display_glb_cmd_is_valid_topology(
				struct bst_display_topology_info *topo_info,
				struct bst_display_topology_status *topo_status);
int bst_display_glb_cmd_subscribe_events(
				uint32_t subdev,
				disp_event_callback_t cb, void *data);
int bst_display_glb_cmd_unsubscribe_events(
				uint32_t subdev);
int __attribute__((weak)) fw_msg_events_sub(uint32_t subdev, disp_event_callback_t cb, void *ext);
int __attribute__((weak)) fw_msg_events_unsub(uint32_t subdev);
int transfer_fw_msg(struct fw_msg_data *msg_data);
#endif /* BST_DISPLAY_GLOBAL_API_H */
