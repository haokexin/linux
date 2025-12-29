// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef BST_DISPLAY_DC_CMDSETS_H
#define BST_DISPLAY_DC_CMDSETS_H

#include "bst_display_cmdset_api.h"
#include "bst_display_global_api.h"
#include "bst_display_platform.h"

enum dc_cmdid {
	DC_CMD_INVALED = 0x00,
	DC_CMD_PROBE_SUBMODULE,
	DC_CMD_GET_LAYER_INFO,
	DC_CMD_GET_COMPOSER_INFO,
	DC_CMD_DO_FLUSH,
	DC_CMD_UPDATE_COMPOSER,
	DC_CMD_UPDATE_LAYER,
	DC_CMD_UPDATE_WB_LAYER,
	DC_CMD_UPDATE_LAYER_SCALER,
	DC_CMD_UPDATE_COEFFS_TABLE,
	DC_CMD_DISABLE_SUBMODULE,
	DC_CMD_DUMP_DEBUG_INFO,
	DC_CMD_SET_LAYER_PLANEID
};

enum dc_composer_ips_dither_mode {
	DC_COMPOSER_IPS_DITHER_MODE_UNKNOW,
	DC_COMPOSER_IPS_DITHER_MODE_OFF,
	DC_COMPOSER_IPS_DITHER_MODE_ON,
};

struct bst_display_composer_ips_cfg {
	uint8_t gamma_changed;
	uint8_t ctm_color_lut_changed;
	uint8_t base_cfg_changed;
	uint8_t color_format, color_depth;
	uint16_t hsize, vsize;
	uint8_t dither_mode;
};

#define MODE_BLEND_PIXEL_NONE	0U
#define MODE_BLEND_PREMULTI		1U

struct bst_display_layer_compose_cfg {
	uint16_t hsize, vsize;
	uint16_t hoffset, voffset;
	uint8_t pixel_blend_mode;
	uint8_t layer_alpha;
};

struct bst_display_composer_zpos_cfg {
	uint8_t valid_input_num;
	uint8_t active_inputs_mask;
	/* store the hw_layer_id by z-order.
	   the low idx of valid_input_ids[] means the low level. */
	uint8_t valid_input_ids[MAX_CU_INPUT_NUM];
};

struct bst_display_composer_video_mode {
	uint8_t timing_changed;
	uint8_t enable;
	uint8_t dual_link;
	struct dtd timing;
};

struct bst_display_composer_cfg {
	struct bst_display_composer_zpos_cfg compiz;
	struct bst_display_composer_video_mode video_mode;
	struct bst_display_composer_ips_cfg ips;
};

enum dc_layer_rotate_type {
	DC_LAYER_ROTATE_TYPE_UNKNOW,
	DC_LAYER_ROTATE_TYPE_0,
	DC_LAYER_ROTATE_TYPE_90,
	DC_LAYER_ROTATE_TYPE_180,
	DC_LAYER_ROTATE_TYPE_270,
};

enum dc_layer_reflect_type {
	DC_LAYER_REFLECT_TYPE_UNKNOW,
	DC_LAYER_REFLECT_TYPE_NONE,
	DC_LAYER_REFLECT_TYPE_X,
	DC_LAYER_REFLECT_TYPE_Y,
};

enum dc_layer_type {
	DC_LAYER_TYPE_UNKNOW,
	DC_LAYER_TYPE_RICH,
	DC_LAYER_TYPE_SIMPLE,
	DC_LAYER_TYPE_WIRTEBACK,
};

enum dc_link_type {
	DC_LINK_TYPE_UNKNOW,
	DC_LINK_TYPE_SINGLE,
	DC_LINK_TYPE_SPLIT,
	DC_LINK_TYPE_SIDE_BY_SIDE,
};

enum dc_smmu_type {
	DC_SMMU_TYPE_UNKNOW,
	DC_SMMU_TYPE_BYPASS,
	DC_SMMU_TYPE_STAGE1,
	DC_SMMU_TYPE_STAGE2,
};

enum dc_color_mgmt_type {
	DC_COLOR_MGMT_TYPE_UNKNOW,
	DC_COLOR_MGMT_TYPE_CTM,
	DC_COLOR_MGMT_TYPE_GAMMA,
};

enum dc_afbc_mode_type {
	DC_AFBC_MODE_TYPE_UNKNOW,
	DC_AFBC_MODE_TYPE_ENABLE,
	DC_AFBC_MODE_TYPE_LOSSLESS_YUV,
	DC_AFBC_MODE_TYPE_BLOCK_SPLIT,
	DC_AFBC_MODE_TYPE_WIDE_BLOCK,
	DC_AFBC_MODE_TYPE_TILED_HEADER,
};

#define DC_YUV2RGB_COEFFS 12
#define DC_RGB2YUV_COEFFS 12
#define DC_LR_CHI422_BILINEAR 0
#define DC_LR_CHI422_REPLICATION 1
#define DC_LR_CHI420_JPEG 2
#define DC_LR_CHI420_MPEG 3

enum dc_pixel_format_standard {
	DC_PIX_FMT_STD_TYPE_UNKNOW,
	DC_PIX_FMT_STD_TYPE_LOCAL,
	DC_PIX_FMT_STD_TYPE_FOURCC,
};

enum {
    DC_LOCAL_FMT_ARGB_2101010,
    DC_LOCAL_FMT_ABGR_2101010,
    DC_LOCAL_FMT_RGBA_1010102,
    DC_LOCAL_FMT_BGRA_1010102,
    DC_LOCAL_FMT_ARGB_8888 = 8,
    DC_LOCAL_FMT_ABGR_8888,
    DC_LOCAL_FMT_RGBA_8888,
    DC_LOCAL_FMT_BGRA_8888,
    DC_LOCAL_FMT_XRGB_8888 = 16,
    DC_LOCAL_FMT_XBGR_8888,
    DC_LOCAL_FMT_RGBX_8888,
    DC_LOCAL_FMT_BGRX_8888,
    DC_LOCAL_FMT_RGB_888 = 24,
    DC_LOCAL_FMT_BGR_888,
    DC_LOCAL_FMT_RGBA_5551 = 32,
    DC_LOCAL_FMT_ABGR_1555,
    DC_LOCAL_FMT_RGB_565,
    DC_LOCAL_FMT_BGR_565,
    DC_LOCAL_FMT_R8,
    DC_LOCAL_FMT_YUV_422_P2_8 = 41,
    DC_LOCAL_FMT_VYUY_422_P1_8,
    DC_LOCAL_FMT_YVYU_422_P1_8,
    DC_LOCAL_FMT_YUV_420_P2_8 = 46,
    DC_LOCAL_FMT_YUV_420_P3_8,
    DC_LOCAL_FMT_YUV_420_P1_10 = 54,
    DC_LOCAL_FMT_YUV_420_P2_10,
	DC_LOCAL_FMT_MAX,
};

enum dc_color_encoding {
	DC_COLOR_YCBCR_BT601,
	DC_COLOR_YCBCR_BT709,
	DC_COLOR_YCBCR_BT2020,
	DC_COLOR_ENCODING_MAX,
};

enum dc_color_range {
	DC_COLOR_YCBCR_LIMITED_RANGE,
	DC_COLOR_YCBCR_FULL_RANGE,
	DC_COLOR_RANGE_MAX,
};

enum ctm_color_lut_standard {
	DC_CTM_COLOR_STD_TYPE_UNKNOW,
	DC_CTM_COLOR_STD_TYPE_ITUR,
	DC_CTM_COLOR_STD_TYPE_USER,
};

enum dc_layer_crop_type {
	DC_LAYER_CROP_TYPE_NORMAL,
	DC_LAYER_CROP_TYPE_AFBC,
	DC_LAYER_CROP_TYPE_SCALER,
};

struct bst_display_crop_cfg {
	uint8_t update_flag;
	uint16_t crop_type;
	uint32_t crop_left;
	uint32_t crop_right;
	uint32_t crop_top;
	uint32_t crop_bottom;
};

enum dc_layer_trust_mode {
	DC_LAYER_TRUST_MODE_UNKNOW,
	DC_LAYER_TRUST_MODE_TZC,
	DC_LAYER_TRUST_MODE_SEC_SMMU,
};

enum dc_layer_scaler_method {
	DC_LAYER_SCALER_METHOD_UNKNOW,
	DC_LAYER_SCALER_METHOD_POLYPHASE_FIR,
	DC_LAYER_SCALER_METHOD_NEAREST_NEIGHBOR,
};

struct bst_display_scaler_cfg {
	//uint32_t client_id;
	//uint8_t fw_layer_id;
	uint8_t en_scaling;
	uint8_t en_alpha;
	uint8_t en_img_enh;
	uint8_t alpha_scaling_method;
	uint8_t rgb_scaling_method;
	uint16_t hsize_in;
	uint16_t vsize_in;
	uint16_t hsize_out;
	uint16_t vsize_out;
	uint16_t total_hsize_in;
	uint16_t total_vsize_in;
	uint16_t total_hsize_out;
	uint16_t left_crop;
	uint16_t right_crop;
};

struct bst_display_trust_layer_cfg {
	uint8_t enable;
	uint8_t trust_mode;
	uint8_t trust_prot_nasid;
	uint8_t trust_nprot_nasid;
};

struct bst_display_layer_cfg {
	uint8_t fw_layer_id;
	uint8_t layer_en;
	uint8_t layer_rotate;
	uint8_t layer_reflect;
	uint8_t is_va;
	uint16_t afbc_mode;
	uint8_t pixel_format_standard;
	uint32_t pixel_format;
	uint32_t hsize;
	uint32_t vsize;
	uint64_t p0_ptr;
	uint64_t p1_ptr;
	uint64_t p2_ptr;
	uint16_t p0_stride;
	uint16_t p1_stride;
	uint8_t num_planars;
	uint8_t ctm_color_lut_standard;
	uint8_t ctm_color_encoding;
	uint8_t ctm_color_range;
	uint8_t ctm_color_lut_changed;
	struct bst_display_layer_compose_cfg cin;
	struct bst_display_crop_cfg crop;
	struct bst_display_trust_layer_cfg trust_cfg;
	struct bst_display_scaler_cfg scale;
};

struct bst_display_composer_request {
	uint32_t reserve;
};

enum dc_layer_wb_precison_mode {
	DC_LAYER_WB_PRECISION_REDUCE_MODE_UNKNOW,
	DC_LAYER_WB_PRECISION_REDUCE_MODE_ROUNDING,
	DC_LAYER_WB_PRECISION_REDUCE_MODE_TRUNCATION,
};

enum dc_layer_wb_frame_mode {
	DC_LAYER_WB_FRAME_MODE_UNKNOW,
	DC_LAYER_WB_FRAME_MODE_ONE,
	DC_LAYER_WB_FRAME_MODE_CONTINUOUS,
};

struct bst_display_wb_layer_cfg {
	uint8_t input_id;
	uint8_t precision_reduce_mode;
	uint8_t frame_mode;

	uint8_t layer_en;
	uint8_t is_va;
	uint8_t pixel_format_standard;
	uint8_t pixel_format;
	uint8_t num_planars;
	uint16_t hsize;
	uint16_t vsize;
	uint16_t p0_stride;
	uint16_t p1_stride;
	uint64_t p0_ptr;
	uint64_t p1_ptr;

	uint8_t ctm_color_lut_standard;
	uint8_t ctm_color_encoding;
	uint8_t ctm_color_range;
	uint8_t ctm_color_lut_changed;
};

struct bst_display_layer_req {
	uint8_t fw_layer_id;
};

struct bst_display_layer_info {
	reply_base base;
	uint32_t default_layer_alpha;
	uint8_t default_zpos;
	uint8_t supported_color_mgmts;
	uint8_t supported_layer_types;
	uint8_t supported_rotates;
	uint8_t supported_reflects;
	uint8_t supported_pix_fmt_std_types;
	uint8_t supported_ctm_color_std_types;
#ifdef DISPLAY_SUPPORT_SCALE
	uint8_t supported_scale;
	uint8_t max_downscale_ratio;
	uint8_t max_upscale_ratio;
	uint16_t max_scaler_hsize;
	uint16_t max_scaler_vsize;
	//uint8_t available_scaler_channel;
#endif
	uint32_t max_line_size;
	uint32_t max_yuv_line_size;
};

struct bst_display_composer_info {
	reply_base base;
	uint32_t supported_color_formats;
	uint32_t supported_color_depths;
	uint8_t supports_degamma;
	uint8_t supports_csc;
	uint8_t supports_gamma;
	uint8_t supports_dual_link;
};

enum {
	BST_DC_COEFFS_TYPE_INVALID = 0,
	BST_DC_COEFFS_TYPE_OUT_GAMMA,
	BST_DC_COEFFS_TYPE_OUT_CTM,
	BST_DC_COEFFS_TYPE_LAYER_CTM,
	BST_DC_COEFFS_TYPE_SCALER_ENHANCE,
	BST_DC_COEFFS_TYPE_SCALER_HORIZ,
	BST_DC_COEFFS_TYPE_SCALER_VERTI,
	BST_DC_COEFFS_TYPE_MAX,
};

#define BST_DC_N_CTM_COEFFS 12
#define BST_DC_N_GAMMA_COEFFS 65
#define BST_DC_N_SCALER_ENHANCE_COEFFS 9
#define BST_DC_N_SCALER_HV_COEFFS 96

struct bst_display_coeffs_cfg {
	uint32_t submodule_id;
	uint8_t coeffs_type;
	uint8_t coeffs_num;
	uint16_t coeffs_table[BST_DC_N_GAMMA_COEFFS];
};

struct bst_display_plane_ids {
	uint32_t num;
	uint32_t plane_ids[MAX_LAYER_NUM_PER_PIPE];
};

struct bst_display_flush_cfg {
	bool is_trust;
	bool test_mode;
};

int bst_display_dc_cmd_disable_submodule(uint32_t dc_session,
				struct bst_display_submodule_req *submodule_req,
				struct bst_display_comm_reply* reply);
int bst_display_dc_cmd_dump_debug_info(uint32_t dc_session,
				struct bst_display_dev_dump* dump_cfg,
				struct bst_display_comm_reply* reply);
int bst_display_dc_cmd_do_flush(uint32_t dc_session,
				struct bst_display_flush_cfg *flush_cfg,
				struct bst_display_comm_reply* reply);
int bst_display_dc_cmd_probe_submodule(uint32_t dc_session,
				struct bst_display_submodule_req *submodule_req,
				struct bst_display_submodule_header *submodule_head);
int bst_display_dc_cmd_update_composer(uint32_t dc_session,
				struct bst_display_composer_cfg *composer_cfg,
				struct bst_display_comm_reply* reply);
int bst_display_dc_cmd_update_wb_layer(uint32_t dc_session,
				struct bst_display_wb_layer_cfg *wb_lcfg,
				struct bst_display_comm_reply* reply);
int bst_display_dc_cmd_update_layer(uint32_t dc_session,
				struct bst_display_layer_cfg *lcfg,
				struct bst_display_comm_reply* reply);
int bst_display_dc_cmd_update_coeffs_table(uint32_t dc_session,
				struct bst_display_coeffs_cfg *coeffs_cfg,
				struct bst_display_comm_reply* reply);
int bst_display_dc_cmd_get_layer_info(uint32_t dc_session,
				struct bst_display_layer_req *layer_req,
				struct bst_display_layer_info *linfo);
int bst_display_dc_cmd_get_composer_info(uint32_t dc_session,
				struct bst_display_composer_request *cfg,
				struct bst_display_composer_info *info);
int bst_display_dc_cmd_set_plane_ids(uint32_t dc_session,
				struct bst_display_plane_ids *plane_ids,
				struct bst_display_comm_reply* reply);

#endif /* BST_DISPLAY_DC_CMDSET_H */
