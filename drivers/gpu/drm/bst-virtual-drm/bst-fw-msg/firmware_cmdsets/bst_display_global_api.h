// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef __BST_DISPLAY_GLOBAL_API_H__
#define __BST_DISPLAY_GLOBAL_API_H__

#include "bst_display_platform.h"

// cmdset definition
enum {
	BST_DISPLAY_GLB_SUBDEV = 1,
	BST_DISPLAY_DC_SUBDEV = 2,
	BST_DISPLAY_DP_SUBDEV = 3,
	BST_DISPLAY_DSI_SUBDEV = 4,
	BST_DISPLAY_LVDS_SUBDEV = 5,
};

enum glb_cmdid_type {
	GLB_CMD_INVALED = 0x00,
	GLB_CMD_PROBE_SUBDEV = 0x01,
	GLB_CMD_IS_VALID_TOPO = 0x02,
	GLB_CMD_GET_EDID = 0x3,
	GLB_CMD_GET_CUR_VM = 0x04,
	GLB_CMD_GET_SUBDEV_INFO = 0x05,
};

#define PRIVILEGE_NONE (1 << 0)
#define PRIVILEGE_SLAVE_COMPOSER_SET (1 << 1)
#define PRIVILEGE_SLAVE_VM_SET (1 << 2)
#define PRIVILEGE_SLAVE_CONN_SET (1 << 3)
#define PRIVILEGE_SLAVE_WB_SET  (1 << 4)

#define PRIVILEGE_ALL (\
		PRIVILEGE_SLAVE_COMPOSER_SET | \
		PRIVILEGE_SLAVE_VM_SET | \
		PRIVILEGE_SLAVE_CONN_SET | \
		PRIVILEGE_SLAVE_WB_SET)

#define MAX_USR_DATA (32 * 4)
#define MAX_ACK_DATA (42 * 4)
#define CMD_SYNC_MDDE (0)
#define CMD_ASYNC_MDDE (1)

struct fw_msg_data {
	uint32_t client_id;
	uint32_t subdev_session;
	uint32_t cmdset;
	uint32_t cmdid;
	uint32_t size_cmd;
	uint32_t size_ack;
	uint32_t user_cmd_data[MAX_USR_DATA / 4];
	uint32_t user_ack_data[MAX_ACK_DATA / 4];
	uint32_t sync_mode;
};

#define SUBDEV_PROBE_STATUS_OK (0)
#define SUBDEV_PROBE_STATUS_FAIL (1)
struct bst_subdev_probe_response {
	uint32_t client_id;
	uint32_t platform_id;
	uint32_t exec_subdev;
	uint32_t subdev_session;
	uint32_t status;
	uint32_t probed_info_size;
	// attach different subdev info
	// such as struct virt_dc_probe_info, struct virt_dp_probe_info ...
	uint32_t probed_info[16];
	// 1: means this client control the shared resource.
	// 0: means this client can't control but something to create local objects.
	uint8_t is_owner;
};

struct bst_subdev_probe_request {
	uint32_t client_id;
	uint32_t platform_id;
	uint32_t want_subdev;
	uint32_t want_layer_num;
	uint32_t want_info_size;
};

struct bst_subdev_info_req {
	uint32_t client_id;
	uint32_t platform_id;
	uint32_t want_subdev;
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
	uint8_t related_subdev;
	uint8_t exec_subdev;
	struct client_list clist;
};

#define SUBMODULE_MAX_INPUT 9
#define SUBMODULE_MAX_OUTPUT 5

struct bst_display_submodule_req {
	uint32_t client_id;
	uint32_t submodule_id;
};

struct bst_display_submodule_header {
	uint32_t client_id;
	uint32_t submodule_info;
	uint32_t pipeline_info;
	uint16_t input_ids[SUBMODULE_MAX_INPUT];
	uint16_t output_ids[SUBMODULE_MAX_OUTPUT];
	uint8_t input_id_num;
	uint8_t output_id_num;
};

enum dev_dump_type {
	DC_REGISTER_START   = 0x0000,
	DC_REGISTER_MAX     = 0x003F,
	DC_MONITOR          = 0x0040,
};

struct bst_display_dev_dump {
	uint32_t client_id;
	uint8_t type;
	uint32_t shmem_paddr_high;
	uint32_t shmem_paddr_low;
};

#define DISP_COMM_REPLAY_OK   (0x0)
#define DISP_COMM_REPLAY_BUSY (0x1)
#define DISP_COMM_REPLAY_FAILED (0x2)

struct bst_display_comm_reply {
	uint32_t client_id;
	uint8_t status;
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
	uint8_t max_scaler_num;
	uint8_t num_outputs;
	uint8_t num_submodules;
	uint8_t submodule_ids[SUBMODULE_ID_DC_MAX];
};

#define DISP_TOPO_STATUS_OK (0x1)
#define DISP_TOPO_STATUS_MISMATCH (0x2)

struct bst_display_topology_status {
	uint32_t client_id;
	uint32_t platform_id;
	uint8_t topology_status;
};

struct bst_display_events_status {
	uint32_t client_id; /* Distinguish which guestOS */
	uint32_t events; 
};

enum {
	RGB = 0,
	YCBCR420 = 1,
	YCBCR422 = 2,
	YCBCR444 = 3,
	YONLY = 4,
	RAW = 5
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
	uint8_t connected;
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

struct bst_display_submodule_disable {
	uint32_t client_id;
	uint8_t submodule_type;
	uint8_t submodule_id;
};

struct bst_display_topology_info {
	uint32_t client_id;
	uint32_t platform_id;
	uint32_t dc_subdev_session;
	uint32_t conn_subdev_session;
};
/*  Detailed Timing Descriptions(DTD) */
struct dtd {
	uint16_t pixel_repetition_input;
	uint32_t pixel_clock; /* pixelclock in KHz */
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
enum {
	EDID_BLOCK_TOP,
	EDID_BLOCK_BOTTOM,
	EDID_EXT_BLOCK1_TOP,
	EDID_EXT_BLOCK1_BOTTOM,
	EDID_EXT_BLOCK2_TOP,
	EDID_EXT_BLOCK2_BOTTOM,
};
#define DEFAULT_EDID_BUFLEN   256
#define EDID_BLOCK_BUFLEN     64
#define EDID_PREFERRED_MODE_OFFSET 0x36
#define EDID_PREFERRED_MODE_LEN    0x12
struct bst_display_edid_req {
	uint32_t client_id;
	uint32_t subdev_session;
	uint8_t type;
};

struct bst_display_edid_info {
	uint32_t client_id;
	uint8_t edid[EDID_BLOCK_BUFLEN];
};

struct bst_display_vm_setting {
	uint32_t client_id;
	uint32_t platform_id;
	uint16_t refresh_rate;
	struct video_timing timing;
};

struct bst_display_vm_req {
	uint32_t platform_id;
	uint32_t client_id;
	uint8_t video_timing_id;
	uint32_t subdev_session;
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
int bst_display_glb_cmd_is_valid_topology(
				struct bst_display_topology_info *topo_info,
				struct bst_display_topology_status *topo_status);
int bst_display_glb_cmd_get_edid(
				struct bst_display_edid_req *edid,
				struct bst_display_edid_info *info);
int bst_display_glb_cmd_get_cur_video_mode(
				struct bst_display_vm_req *req,
				struct bst_display_vm_setting *info);
int bst_display_glb_cmd_subscribe_events(
				uint32_t subdev,
				disp_event_callback_t cb, void *data);
int bst_display_glb_cmd_unsubscribe_events(
				uint32_t subdev);

int transfer_fw_msg(struct fw_msg_data *msg_data);
#endif /* __BST_DISPLAY_GLOBAL_API_H__ */
