// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include <drm/drm_writeback.h>
#include <drm/drm_print.h>
#include "bst_display_dc_cmdset.h"
#include "bst_display_platform.h"
#include "bst_virt_drm_framebuffer.h"
#include "bst_virt_pipeline.h"
#include "virt_dc_dev.h"

#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>

static void get_resources_id(uint32_t submodule_info, uint8_t num_layers,
			     uint8_t min_fw_layer_id, uint32_t *comp_id)
{
	uint32_t logic_id = SUBMODULE_INFO_SUBMODULE_ID(submodule_info);

	switch (SUBMODULE_INFO_SUBMODULE_TYPE(submodule_info)) {
	case DC_SUBMODULE_TYPE_LAYER:
		logic_id %= (num_layers + 1);
		logic_id += BST_VIRT_COMPONENT_DC_LAYER0;
		logic_id -= min_fw_layer_id;
		break;
	case DC_SUBMODULE_TYPE_WB_LAYER:
		logic_id = BST_VIRT_COMPONENT_DC_WB_LAYER;
		break;
	case DC_SUBMODULE_TYPE_COMPOSER:
		logic_id = BST_VIRT_COMPONENT_DC_CRTC;
		break;
	default:
		logic_id = 0xFFFFFFFF;
		break;
	}

	if (comp_id)
		*comp_id = logic_id;
}

static uint32_t get_valid_inputs(struct bst_display_submodule_header *submodule,
				 uint8_t min_fw_layer_id)
{
	uint32_t valid_inputs = 0, comp_id;
	int i;

	for (i = 0; i < PIPELINE_INFO_N_VALID_INPUTS(submodule->pipeline_info); i++) {
		get_resources_id(submodule->input_ids[i], MAX_LAYER_NUM_PER_PIPE,
				 min_fw_layer_id, &comp_id);
		if (comp_id == 0xFFFFFFFF)
			continue;
		valid_inputs |= BIT(comp_id);
	}

	return valid_inputs;
}

static void to_rot_ctrl(uint32_t rot, struct bst_display_layer_cfg *cfg)
{
	switch (rot & DRM_MODE_ROTATE_MASK) {
	case DRM_MODE_ROTATE_0:
		cfg->layer_rotate = DC_LAYER_ROTATE_TYPE_0;
		break;
	case DRM_MODE_ROTATE_90:
		cfg->layer_rotate = DC_LAYER_ROTATE_TYPE_90;
		break;
	case DRM_MODE_ROTATE_180:
		cfg->layer_rotate = DC_LAYER_ROTATE_TYPE_180;
		break;
	case DRM_MODE_ROTATE_270:
		cfg->layer_rotate = DC_LAYER_ROTATE_TYPE_270;
		break;
	}

	if (rot & DRM_MODE_REFLECT_X)
		cfg->layer_reflect = DC_LAYER_REFLECT_TYPE_X;
	if (rot & DRM_MODE_REFLECT_Y)
		cfg->layer_reflect = DC_LAYER_REFLECT_TYPE_Y;
}

static u32 to_fw_afbc_mode(u64 modifier)
{
	u32 afbc_mode = BIT(DC_AFBC_MODE_TYPE_ENABLE);

	if (!modifier)
		return 0;

	if ((modifier & AFBC_FORMAT_MOD_BLOCK_SIZE_MASK) ==
	    AFBC_FORMAT_MOD_BLOCK_SIZE_32x8)
		afbc_mode |= BIT(DC_AFBC_MODE_TYPE_WIDE_BLOCK);

	if (modifier & AFBC_FORMAT_MOD_YTR)
		afbc_mode |= BIT(DC_AFBC_MODE_TYPE_LOSSLESS_YUV);
	if (modifier & AFBC_FORMAT_MOD_SPLIT)
		afbc_mode |= BIT(DC_AFBC_MODE_TYPE_BLOCK_SPLIT);
	if (modifier & AFBC_FORMAT_MOD_TILED)
		afbc_mode |= BIT(DC_AFBC_MODE_TYPE_TILED_HEADER);

	return afbc_mode;
}

static u8 to_fw_color_encoding(u8 color_encoding)
{
	u8 fw_color_encoding = 0xff;

	if (color_encoding == DRM_COLOR_YCBCR_BT601)
		fw_color_encoding = DC_COLOR_YCBCR_BT601;
	if (color_encoding == DRM_COLOR_YCBCR_BT709)
		fw_color_encoding = DRM_COLOR_YCBCR_BT709;
	if (color_encoding == DRM_COLOR_YCBCR_BT2020)
		fw_color_encoding = DRM_COLOR_YCBCR_BT2020;

	return fw_color_encoding;
}

static u8 to_fw_color_range(u8 color_range)
{
	u8 fw_color_range = DC_COLOR_YCBCR_FULL_RANGE;

	if (color_range == DRM_COLOR_YCBCR_LIMITED_RANGE)
		fw_color_range = DC_COLOR_YCBCR_LIMITED_RANGE;

	return fw_color_range;
}

static int dc_layer_validate(struct bst_virt_component *c,
			   struct bst_virt_component_state *state)
{
	struct bst_virt_layer_state *st = to_layer_st(state);
	struct bst_virt_layer *layer = to_layer(c);
	struct drm_plane_state *plane_st;
	struct drm_framebuffer *fb;
	uint32_t fourcc, line_sz, max_line_sz;

	plane_st =
		drm_atomic_get_new_plane_state(state->obj.state, state->plane);
	fb = plane_st->fb;
	fourcc = fb->format->format;

	if (drm_rotation_90_or_270(st->rot))
		line_sz = st->vsize - st->afbc_crop.afbc_crop_t - st->afbc_crop.afbc_crop_b;
	else
		line_sz = st->hsize - st->afbc_crop.afbc_crop_l - st->afbc_crop.afbc_crop_r;

	if (fb->modifier) {
		if ((fb->modifier & AFBC_FORMAT_MOD_BLOCK_SIZE_MASK) ==
		    AFBC_FORMAT_MOD_BLOCK_SIZE_32x8)
			max_line_sz = layer->line_sz;
		else
			max_line_sz = layer->line_sz / 2;

		if (line_sz > max_line_sz) {
			DRM_ERROR(
				"afbc request line_sz: %d exceed the max afbc line_sz: %d.\n",
				line_sz, max_line_sz);
			return -EINVAL;
		}
	}

	if (fourcc == DRM_FORMAT_YUV420_10BIT && line_sz > 2046 &&
	    (st->afbc_crop.afbc_crop_l % 4)) {
		DRM_ERROR(
			"YUV420_10BIT input_hsize: %d exceed the max size 2046.\n",
			line_sz);
		return -EINVAL;
	}

	if (fourcc == DRM_FORMAT_X0L2 && line_sz > 2046 && (st->addr[0] % 16)) {
		DRM_ERROR(
			"X0L2 input_hsize: %d exceed the max size 2046.\n",
			line_sz);
		return -EINVAL;
	}

	return 0;
}

static void dc_layer_update(struct bst_virt_component *c,
			  struct bst_virt_component_state *state)
{
	struct bst_virt_layer_state *st = to_layer_st(state);
	struct bst_virt_layer *layer = to_layer(c);
	struct drm_plane_state *plane_st = state->plane->state;
	struct drm_framebuffer *fb = plane_st->fb;
	const struct drm_format_info *info = fb->format;
	struct bst_fb *bfb = to_bfb(fb);
	uint32_t subdev_session = c->subdev_session;
	struct bst_display_layer_cfg layer_cfg;
	struct bst_display_comm_reply reply = { 0 };
	struct virt_dc_dev *dc = c->pipe->subdevs[BST_VIRT_DC_IDX]->virt_dev_data;
	u8 i = 0, count = 0;
	int ret;
	int block_h;
	uint32_t fw_layer_id = c->fw_id;
	// const s32 *yuv2rgb_coeffs;
	u8 use_fourcc_std = true;
	// u8 use_user_cmt_lut = false;

	memset(&layer_cfg, 0, sizeof(struct bst_display_layer_cfg));
	if (info->num_planes > 2)
		layer_cfg.p2_ptr = st->addr[2];

	if (info->num_planes > 1) {
		block_h = drm_format_info_block_height(info, 1);
		layer_cfg.p1_stride = fb->pitches[1] * block_h;
		layer_cfg.p1_ptr = st->addr[1];
	}
	block_h = drm_format_info_block_height(info, 0);
	layer_cfg.p0_stride = fb->pitches[0] * block_h;
	layer_cfg.p0_ptr = st->addr[0];
	if (use_fourcc_std) {
		layer_cfg.pixel_format_standard =
			layer->supported_pix_fmt_stds & BIT(DC_PIX_FMT_STD_TYPE_FOURCC);
		layer_cfg.pixel_format = fb->format->format;
	}
	layer_cfg.num_planars = info->num_planes;
	layer_cfg.afbc_mode = to_fw_afbc_mode(fb->modifier);

	layer_cfg.crop.crop_type = DC_LAYER_CROP_TYPE_NORMAL;
	if (fb->modifier) {
		layer_cfg.crop.crop_type = DC_LAYER_CROP_TYPE_AFBC;
		layer_cfg.crop.crop_left = st->afbc_crop.afbc_crop_l;
		layer_cfg.crop.crop_right = st->afbc_crop.afbc_crop_r;
		layer_cfg.crop.crop_top = st->afbc_crop.afbc_crop_t;
		layer_cfg.crop.crop_bottom = st->afbc_crop.afbc_crop_b;
		if (fb->modifier & AFBC_FORMAT_MOD_TILED)
			layer_cfg.p1_ptr = st->addr[0] + bfb->offset_payload;
		else
			layer_cfg.p1_ptr = st->addr[0] + bfb->afbc_size - 1;
	}

	if (memcmp(&st->afbc_crop_old, &st->afbc_crop, sizeof(st->afbc_crop))) {
		layer_cfg.crop.update_flag = true;
		st->afbc_crop_old = st->afbc_crop;
	}

	if (fb->format->is_yuv) {
		// if(use_user_cmt_lut ) {
		// 	memset(&reply, 0, sizeof(reply));
		// 	yuv2rgb_coeffs = bst_select_yuv2rgb_coeffs(
		// 					plane_st->color_encoding,
		// 					plane_st->color_range);
		// 	ctm_coeffs_cfg.client_id = c->client_id;
		// 	ctm_coeffs_cfg.coeffs_type = BST_DC_COEFFS_TYPE_LAYER_CTM;
		// 	ctm_coeffs_cfg.coeffs_num = BST_DC_N_CTM_COEFFS;
		// 	coeffs_size = sizeof(ctm_coeffs_cfg.coeffs_table);
		// 	memcpy(&ctm_coeffs_cfg.coeffs_table[0], &yuv2rgb_coeffs[0], coeffs_size);
		// 	ret = bst_display_dc_cmd_update_coeffs_table(subdev_session, &ctm_coeffs_cfg, &reply);
		// 	if (ret || reply.status != DISP_COMM_REPLAY_OK)
		// 		DRM_ERROR(":%s update layer ctm table falied!!\n", __func__);
		// 	layer_cfg.ctm_color_lut_standard =
		// 		layer->supported_ctm_lut_stds & BIT(DC_CTM_COLOR_STD_TYPE_USER);
		// } else {
			layer_cfg.ctm_color_encoding = to_fw_color_encoding(plane_st->color_encoding);
			layer_cfg.ctm_color_range = to_fw_color_range(plane_st->color_range);
			layer_cfg.ctm_color_lut_standard =
				layer->supported_ctm_lut_stds & BIT(DC_CTM_COLOR_STD_TYPE_ITUR);
		// }
		layer_cfg.ctm_color_lut_changed = true;
	}

	memset(&reply, 0, sizeof(reply));
	layer_cfg.fw_layer_id = fw_layer_id;
	layer_cfg.layer_en = true;
	to_rot_ctrl(st->rot, &layer_cfg);
	layer_cfg.hsize = st->hsize;
	layer_cfg.vsize = st->vsize;
	layer_cfg.cin.hsize = st->cin.hsize;
	layer_cfg.cin.vsize = st->cin.vsize;
	layer_cfg.cin.hoffset = st->cin.hoffset;
	layer_cfg.cin.voffset = st->cin.voffset;
	layer_cfg.cin.pixel_blend_mode = (st->cin.pixel_blend_mode == DRM_MODE_BLEND_PIXEL_NONE) ?
				MODE_BLEND_PIXEL_NONE : MODE_BLEND_PREMULTI;
	layer_cfg.cin.layer_alpha = st->cin.layer_alpha;

	if (!(dc->enabled_layers_map & (fw_layer_id - SUBMODULE_ID_DC_LAYER_START))) {
		dc->enabled_layers_map |= (fw_layer_id - SUBMODULE_ID_DC_LAYER_START);
	}
	for (i = 0; i < MAX_LAYER_NUM_PER_PIPE; i++) {
		if (dc->enabled_layers_map & BIT(i)) {
			count++;
		}
		if ((count > 1) && (layer_cfg.hsize >= 3840)) {
			DRM_WARN_ONCE("Warning, 4K uses more than one hw layer!!\n");
		}
	}

	if (0) {  // trust layer function not implemented now
		layer_cfg.trust_cfg.enable = true;
		layer_cfg.trust_cfg.trust_mode = DC_LAYER_TRUST_MODE_TZC;
		layer_cfg.trust_cfg.trust_prot_nasid = 0xA;
		layer_cfg.trust_cfg.trust_nprot_nasid = 0xA;
	}

#ifdef DISPLAY_SUPPORT_SCALE
	if (st->scaler.en_scaling) {
		layer_cfg.scale.en_scaling = 1;
		layer_cfg.scale.en_alpha = st->scaler.en_alpha;
		layer_cfg.scale.en_img_enh = st->scaler.en_img_enhancement;
		layer_cfg.scale.hsize_in = st->scaler.hsize_in;
		layer_cfg.scale.vsize_in = st->scaler.vsize_in;
		layer_cfg.scale.hsize_out = st->scaler.hsize_out;
		layer_cfg.scale.vsize_out = st->scaler.vsize_out;
		layer_cfg.scale.total_hsize_in = st->scaler.total_hsize_in;
		layer_cfg.scale.total_vsize_in = st->scaler.total_vsize_in;
		layer_cfg.scale.total_hsize_out = st->scaler.total_hsize_out;
		layer_cfg.scale.left_crop = st->scaler.left_crop;
		layer_cfg.scale.right_crop = st->scaler.right_crop;
		layer_cfg.scale.alpha_scaling_method = DC_LAYER_SCALER_METHOD_POLYPHASE_FIR;
		layer_cfg.scale.rgb_scaling_method = DC_LAYER_SCALER_METHOD_POLYPHASE_FIR;
	} else {
		layer_cfg.scale.en_scaling = 0;
	}
#endif

	ret = bst_display_dc_cmd_update_layer(subdev_session, &layer_cfg, &reply);
	if (ret || reply.base.status != DISP_COMM_REPLAY_OK)
		DRM_ERROR("layer update falied!!\n");
}

static void dc_layer_disable(struct bst_virt_component *c)
{
	struct bst_display_submodule_req submodule_req = { 0 };
	uint32_t subdev_session = c->subdev_session;
	struct bst_display_comm_reply reply = { 0 };
	struct virt_dc_dev *dc = c->pipe->subdevs[BST_VIRT_DC_IDX]->virt_dev_data;
#ifdef DISPLAY_SUPPORT_SCALE
	struct bst_virt_component_state *c_st = priv_to_comp_st(c->obj.state);
	struct bst_virt_layer_state *st = to_layer_st(c_st);
#endif
	int ret;

	//submodule_req.submodule_type = DC_SUBMODULE_TYPE_LAYER;
	submodule_req.submodule_id = c->fw_id;
	if (dc->enabled_layers_map & (c->fw_id - SUBMODULE_ID_DC_LAYER_START)) {
		dc->enabled_layers_map &= ~(c->fw_id - SUBMODULE_ID_DC_LAYER_START);
	}
#ifdef DISPLAY_SUPPORT_SCALE
	st->scaler.en_scaling = 0;
	//if (memcmp(&st->scaler_old, &st->scaler, sizeof(st->scaler))) {
	//	st->scaler_old = st->scaler;
	//	remove_scaler_from_slot(to_layer(c));
	//}
#endif
	ret = bst_display_dc_cmd_disable_submodule(subdev_session, &submodule_req, &reply);
	if (!ret && reply.base.status == DISP_COMM_REPLAY_OK)
		DRM_DEBUG_ATOMIC("layer%d disable ok!!\n", submodule_req.submodule_id);
	else
		DRM_ERROR("layer%d disable falied!!\n", submodule_req.submodule_id);
}

static void dc_layer_dump(struct bst_virt_component *c, struct seq_file *sf)
{
}

static const struct bst_virt_component_funcs dc_layer_funcs = {
	.validate = dc_layer_validate,
	.update = dc_layer_update,
	.disable = dc_layer_disable,
	.dump_log = dc_layer_dump,
};

static int dc_layer_init(struct virt_dc_dev *dc,
		       struct bst_display_submodule_header *submodule)
{
	struct bst_virt_component *comp = NULL;
	struct bst_virt_layer *layer;
	uint32_t pipe_id = dc->base_dev->this_pipe->pipe_id;
	uint32_t submodule_type = SUBMODULE_INFO_SUBMODULE_TYPE(submodule->submodule_info);
	uint32_t fw_layer_id = SUBMODULE_INFO_SUBMODULE_ID(submodule->submodule_info);
	uint32_t subdev_session = dc->base_dev->dev_info.subdev_session;
	struct bst_display_layer_info fw_layer_info = { 0 };
	struct bst_display_layer_req fw_layer_req = { 0 };
	uint32_t layer_id = 0, min_fw_layer_id = dc->min_fw_layer_id;
	int ret;

	if (submodule_type != DC_SUBMODULE_TYPE_LAYER) {
		DRM_ERROR("Failed add layer for wrong submodule type:%d\n",
			  submodule_type);
		return PTR_ERR(comp);
	}

	get_resources_id(submodule->submodule_info, MAX_LAYER_NUM_PER_PIPE, min_fw_layer_id,
			 &layer_id);
	comp = bst_virt_component_add(dc->base_dev->this_pipe, dc->base_dev,
				      sizeof(*layer), layer_id, fw_layer_id,
				      &dc_layer_funcs, 0,
				      get_valid_inputs(submodule, min_fw_layer_id), 1,
				      "VIRT_LAYER-%d", layer_id);
	if (IS_ERR(comp)) {
		DRM_ERROR("Failed to add layer component\n");
		return PTR_ERR(comp);
	}

	layer = to_layer(comp);
	fw_layer_req.fw_layer_id = fw_layer_id;
	ret = bst_display_dc_cmd_get_layer_info(subdev_session, &fw_layer_req,
				       &fw_layer_info);
	if (ret) {
		DRM_ERROR("Failed to get layer info from FW\n");
		return -1;
	}

	layer->layer_type = fw_layer_info.supported_layer_types & BIT(DC_LAYER_TYPE_RICH) ?
				    BST_DRM_FMT_RICH_LAYER : BST_DRM_FMT_SIMPLE_LAYER;
	layer->line_sz = fw_layer_info.max_line_size;
	layer->yuv_line_sz = fw_layer_info.max_yuv_line_size;
	set_range(&layer->hsize_in, 4, fw_layer_info.max_line_size);
	set_range(&layer->vsize_in, 4, dc->max_vsize);

	layer->supported_rots = 0;
	if (fw_layer_info.supported_rotates & BIT(DC_LAYER_ROTATE_TYPE_0))
		layer->supported_rots |= DRM_MODE_ROTATE_0;
	if (fw_layer_info.supported_rotates & BIT(DC_LAYER_ROTATE_TYPE_90))
		layer->supported_rots |= DRM_MODE_ROTATE_90;
	if (fw_layer_info.supported_rotates & BIT(DC_LAYER_ROTATE_TYPE_180))
		layer->supported_rots |= DRM_MODE_ROTATE_180;
	if (fw_layer_info.supported_rotates & BIT(DC_LAYER_ROTATE_TYPE_270))
		layer->supported_rots |= DRM_MODE_ROTATE_270;
	if (fw_layer_info.supported_reflects & BIT(DC_LAYER_REFLECT_TYPE_X))
		layer->supported_rots |= DRM_MODE_REFLECT_X;
	if (fw_layer_info.supported_reflects & BIT(DC_LAYER_REFLECT_TYPE_Y))
		layer->supported_rots |= DRM_MODE_REFLECT_Y;

	layer->supported_pix_fmt_stds = fw_layer_info.supported_pix_fmt_std_types;
	layer->supported_ctm_lut_stds = fw_layer_info.supported_ctm_color_std_types;

	layer->init_zpos = fw_layer_info.default_zpos;
#ifdef DISPLAY_SUPPORT_SCALE
	set_range(&layer->scaler_hsize, 4, fw_layer_info.max_scaler_hsize);
	set_range(&layer->scaler_vsize, 4, fw_layer_info.max_scaler_vsize);
	layer->max_downscaling = fw_layer_info.max_downscale_ratio;
	layer->max_upscaling = fw_layer_info.max_upscale_ratio;
	layer->supported_scale = fw_layer_info.supported_scale;
#endif

	DRM_DEBUG("DC_DEV(session_id:%x) add CRTC%d_LAYER%d ok\n", subdev_session,
		 pipe_id, layer_id);
	DRM_DEBUG(
		"		layer_type:%s, max_size(%d X %d),rot_support:0x%x",
		layer->layer_type == BST_DRM_FMT_RICH_LAYER ? "RICH" : "SIMPLE",
		layer->line_sz, dc->max_vsize, layer->supported_rots);
	DRM_DEBUG(
		"		default_alpha:0x%x,init_zpos:%d,scaler support[%d], range[down-%d~up-%d]\n",
		fw_layer_info.default_layer_alpha, layer->init_zpos,
		layer->supported_scale, layer->max_downscaling, layer->max_upscaling);

	return 0;
}

static void dc_wb_layer_update(struct bst_virt_component *c,
			  struct bst_virt_component_state *state)
{
	struct bst_virt_layer_state *st = to_layer_st(state);
	struct drm_connector_state *conn_st = state->wb_conn->state;
	// struct bst_fb *bfb = to_bfb(conn_st->writeback_job->fb);
	struct bst_display_wb_layer_cfg wb_lcfg = {0};
	struct drm_framebuffer *fb = conn_st->writeback_job->fb;
	const struct drm_format_info *info = fb->format;
	uint32_t subdev_session = c->subdev_session;
	int32_t block_h = drm_format_info_block_height(info, 0);
	struct bst_display_comm_reply reply = { 0 };
	int ret;
	struct bst_virt_layer *layer = to_layer(c);
	u8 use_fourcc_std = true;

	if (info->num_planes > 2) {
		DRM_ERROR("wirteback format=[%d],num_planes[%d] unsupport !\n",
				fb->format->format, info->num_planes);
		return;
	}

	if (info->num_planes > 1) {
		block_h = drm_format_info_block_height(info, 1);
		wb_lcfg.p1_stride = fb->pitches[1] * block_h;
		wb_lcfg.p1_ptr = st->addr[1];
	}
	wb_lcfg.p0_stride = fb->pitches[0] * block_h;
	wb_lcfg.p0_ptr = st->addr[0];
	if (use_fourcc_std) {
		wb_lcfg.pixel_format_standard =
			layer->supported_pix_fmt_stds & BIT(DC_PIX_FMT_STD_TYPE_FOURCC);
		wb_lcfg.pixel_format = fb->format->format;
	}
	// else {
	// 	wb_lcfg.layer.pixel_format_standard =
	// 		layer->supported_pix_fmt_stds & BIT(DC_PIX_FMT_STD_TYPE_LOCAL);
	// 	wb_lcfg.layer.pixel_format = bfb->format_caps->fw_id;
	// }
	wb_lcfg.layer_en = true;
	wb_lcfg.hsize = st->hsize;
	wb_lcfg.vsize = st->vsize;

	wb_lcfg.input_id = DC_SUBMODULE_TYPE_COMPOSER;
	wb_lcfg.precision_reduce_mode = DC_LAYER_WB_PRECISION_REDUCE_MODE_TRUNCATION;
	wb_lcfg.frame_mode = DC_LAYER_WB_FRAME_MODE_ONE;

	ret = bst_display_dc_cmd_update_wb_layer(subdev_session, &wb_lcfg, &reply);
	if (ret || reply.base.status != DISP_COMM_REPLAY_OK)
		DRM_ERROR("wirteback layer update falied!!\n");
}

static void dc_wb_layer_disable(struct bst_virt_component *c)
{
	struct bst_display_submodule_req submodule_req = { 0 };
	uint32_t subdev_session = c->subdev_session;
	struct bst_display_comm_reply reply = { 0 };
	int ret;

	//submodule_req.submodule_type = DC_SUBMODULE_TYPE_WB_LAYER;
	submodule_req.submodule_id = c->fw_id;

	ret = bst_display_dc_cmd_disable_submodule(subdev_session, &submodule_req, &reply);
	if (!ret && reply.base.status == DISP_COMM_REPLAY_OK)
		DRM_DEBUG("wirteback layer disable ok!!\n");
	else
		DRM_ERROR("wirteback layer disable falied!!\n");
}

static void dc_wb_layer_dump(struct bst_virt_component *c, struct seq_file *sf)
{
}

static const struct bst_virt_component_funcs dc_wb_layer_funcs = {
	.update		= dc_wb_layer_update,
	.disable	= dc_wb_layer_disable,
	.dump_log	= dc_wb_layer_dump,
};

static int dc_wb_layer_init(struct virt_dc_dev *dc,
				struct bst_display_submodule_header *submodule)
{
	struct bst_virt_component *comp = NULL;
	struct bst_virt_layer *wb_layer;
	uint32_t pipe_id = dc->base_dev->this_pipe->pipe_id;
	uint32_t submodule_type = SUBMODULE_INFO_SUBMODULE_TYPE(submodule->submodule_info);
	uint32_t fw_layer_id = SUBMODULE_INFO_SUBMODULE_ID(submodule->submodule_info);
	uint32_t subdev_session = dc->base_dev->dev_info.subdev_session;
	struct bst_display_layer_info fw_layer_info = { 0 };
	struct bst_display_layer_req fw_layer_req = { 0 };
	uint32_t layer_id = 0, min_fw_layer_id = dc->min_fw_layer_id;
	int ret;

	if (!dc->base_dev->dev_info.is_owner_device)
		return 0;

	if (submodule_type != DC_SUBMODULE_TYPE_WB_LAYER) {
		DRM_ERROR("Failed add layer for wrong submodule type:%d\n",
			  submodule_type);
		return PTR_ERR(comp);
	}

	get_resources_id(submodule->submodule_info, MAX_LAYER_NUM_PER_PIPE, min_fw_layer_id,
			 &layer_id);
	comp = bst_virt_component_add(dc->base_dev->this_pipe, dc->base_dev,
					sizeof(*wb_layer), layer_id, fw_layer_id,
					&dc_wb_layer_funcs, 1,
					get_valid_inputs(submodule, min_fw_layer_id), 0,
					"VIRT_WB_LAYER-0");

	if (IS_ERR(comp)) {
		DRM_ERROR("Failed to add layer component\n");
		return PTR_ERR(comp);
	}

	fw_layer_req.fw_layer_id = fw_layer_id;
	ret = bst_display_dc_cmd_get_layer_info(subdev_session,
					&fw_layer_req, &fw_layer_info);
	if (ret) {
		DRM_ERROR("Failed to get layer info from FW\n");
		return -1;
	}
	wb_layer = to_layer(comp);
	wb_layer->layer_type = fw_layer_info.supported_layer_types
			& BIT(DC_LAYER_TYPE_WIRTEBACK) ?
			BST_DRM_FMT_WB_LAYER : 0;

	wb_layer->line_sz = fw_layer_info.max_line_size;
	wb_layer->yuv_line_sz = fw_layer_info.max_yuv_line_size;
	set_range(&wb_layer->hsize_in, 64, fw_layer_info.max_line_size);
	set_range(&wb_layer->vsize_in, 64, dc->max_vsize);

	DRM_DEBUG("DC_DEV(session_id:%x) add CRTC%d_WB_LAYER ok\n", subdev_session,
		 pipe_id);

	return 0;
}

static uint16_t aspect_gcd(int h, int v)
{
    int temp;
    while (v != 0)
    {
        temp = v;
        v = h % v;
        h = temp;
    }
    return h;
}
 /* The horizontal and vertical timings are defined per the following diagram.
 *
 * :: drm_display_mode
 *
 *
 *          Active                 Front           Sync           Back
 *         Region                 Porch                          Porch
 * <-----------------------><----------------><-------------><-------------->
 *   //////////////////////|
 *  ////////////////////// |
 * //////////////////////  |..................               ................
 *                                            _______________
 * <----- [hv]display ----->
 * <------------- [hv]sync_start ------------>
 * <--------------------- [hv]sync_end --------------------->
 * <-------------------------------- [hv]total ----------------------------->
 *
 * :: dtd timing
 *         Active                 Front                  Sync          Back
 *         Region                 Porch                                Porch
 * <-------------------><-------------------><---------------- ---><-------->
 *   //////////////////|
 *  ////////////////// |
 * /////////////////// |...................                        ..........
 *                                         ________________________
 * <--- [hv]_active --->
 *                     <-[hv]_sync_offset-><-[hv]_sync_pulse_width->
 *                     <-------------------------- [hv]_blanking ----------->
 */
static void dc_crtc_update(struct bst_virt_component *c,
			  struct bst_virt_component_state *state)
{
	struct drm_crtc_state *crtc_st = state->crtc->state;
	struct bst_virt_dc_crtc_state *st = to_dc_crtc_st(state);
	uint32_t subdev_session = c->subdev_session;
	struct drm_display_mode *mode = &crtc_st->adjusted_mode;
	struct bst_display_composer_cfg composer_cfg = {0};
	struct bst_display_comm_reply reply = {0};
	int ret;
	int16_t ratio = 0;
	struct bst_display_coeffs_cfg gamma_cfg = { 0 };
	struct bst_display_coeffs_cfg ctm_cfg = { 0 };
	uint16_t coeffs_size = 0, i = 0;
	unsigned long changed_active_inputs;
	bool update_flag = false;
	struct bst_virt_device *subdev;

	subdev = (struct bst_virt_device *)(c->base_dev);
	if (crtc_st->color_mgmt_changed) {
		if (!subdev->dev_info.is_owner_device) {
			DRM_ERROR(":%s Color change is not supported!\n", __func__);
		} else {
			if (crtc_st->gamma_lut) {
				gamma_cfg.submodule_id = SUBMODULE_ID_DC_COMPOSER;
				gamma_cfg.coeffs_type = BST_DC_COEFFS_TYPE_OUT_GAMMA;
				gamma_cfg.coeffs_num = BST_DC_N_GAMMA_COEFFS;
				coeffs_size = sizeof(gamma_cfg.coeffs_table);
				memcpy(&gamma_cfg.coeffs_table[0], &st->fgamma_coeffs[0], coeffs_size);
				ret = bst_display_dc_cmd_update_coeffs_table(subdev_session, &gamma_cfg, &reply);
				if (ret || reply.base.status != DISP_COMM_REPLAY_OK)
					DRM_ERROR(":%s update gamma table falied!!\n", __func__);
				memset(&reply, 0, sizeof(reply));
				composer_cfg.ips.gamma_changed = true;
				update_flag = true;
			}

			if (crtc_st->ctm) {
				ctm_cfg.submodule_id = SUBMODULE_ID_DC_COMPOSER;
				ctm_cfg.coeffs_type = BST_DC_COEFFS_TYPE_OUT_CTM;
				ctm_cfg.coeffs_num = BST_DRM_N_CTM_COEFFS;
				// coeffs_size = sizeof(ctm_cfg.coeffs_table);
				memcpy(&ctm_cfg.coeffs_table[0], &st->ctm_coeffs[0], BST_DRM_N_CTM_COEFFS*sizeof(u32));
				ret = bst_display_dc_cmd_update_coeffs_table(subdev_session, &ctm_cfg, &reply);
				if (ret || reply.base.status != DISP_COMM_REPLAY_OK)
					DRM_ERROR(":%s update ctm table falied!!\n", __func__);
				memset(&reply, 0, sizeof(reply));
				composer_cfg.ips.ctm_color_lut_changed = true;
				update_flag = true;
			}
		}
	}

	if (crtc_st->mode_changed) {
		update_flag = true;
		composer_cfg.video_mode.timing_changed = true;
		composer_cfg.video_mode.enable = crtc_st->enable;
		composer_cfg.video_mode.dual_link = c->pipe->dual_link;
		ratio = aspect_gcd(mode->crtc_hdisplay, mode->crtc_vdisplay);
		composer_cfg.video_mode.timing.pixel_clock = mode->clock;
		composer_cfg.video_mode.timing.interlaced = 0;
		composer_cfg.video_mode.timing.h_image_size = mode->crtc_hdisplay / ratio;
		composer_cfg.video_mode.timing.h_active = mode->crtc_hdisplay;
		composer_cfg.video_mode.timing.h_sync_offset = mode->crtc_hsync_start - mode->crtc_hdisplay;
		composer_cfg.video_mode.timing.h_sync_pulse_width = mode->crtc_hsync_end - mode->crtc_hsync_start;
		composer_cfg.video_mode.timing.h_blanking = mode->crtc_htotal - mode->crtc_hdisplay;
		composer_cfg.video_mode.timing.h_sync_polarity = mode->flags & DRM_MODE_FLAG_PHSYNC ? 1 : 0;
		composer_cfg.video_mode.timing.v_image_size = mode->crtc_vdisplay / ratio;
		composer_cfg.video_mode.timing.v_active = mode->crtc_vdisplay;
		composer_cfg.video_mode.timing.v_sync_offset = mode->crtc_vsync_start - mode->crtc_vdisplay;
		composer_cfg.video_mode.timing.v_sync_pulse_width = mode->crtc_vsync_end - mode->crtc_vsync_start;
		composer_cfg.video_mode.timing.v_blanking = mode->crtc_vtotal - mode->crtc_vdisplay;
		composer_cfg.video_mode.timing.v_sync_polarity = mode->flags & DRM_MODE_FLAG_PVSYNC ? 1 : 0;

		composer_cfg.ips.base_cfg_changed = true;
		composer_cfg.ips.hsize = st->hsize;
		composer_cfg.ips.vsize = st->vsize;
		composer_cfg.ips.color_depth = st->color_depth;
		composer_cfg.ips.dither_mode = DC_COMPOSER_IPS_DITHER_MODE_ON;
		switch(st->color_format){
			case DRM_COLOR_FORMAT_RGB444:
				composer_cfg.ips.color_format = BST_DC_OUT_COLOR_FORMAT_RGB444;
			break;
			case DRM_COLOR_FORMAT_YCBCR444:
				composer_cfg.ips.color_format = BST_DC_OUT_COLOR_FORMAT_YCRCB444;
			break;
			case DRM_COLOR_FORMAT_YCBCR422:
				composer_cfg.ips.color_format = BST_DC_OUT_COLOR_FORMAT_YCRCB422;
			break;
			case DRM_COLOR_FORMAT_YCBCR420:
				composer_cfg.ips.color_format = BST_DC_OUT_COLOR_FORMAT_YCRCB420;
			break;
			default:
				composer_cfg.ips.color_format = BST_DC_OUT_COLOR_FORMAT_RGB444;
			break;
		}
	}

	changed_active_inputs = state->changed_active_inputs;
	if (0 != changed_active_inputs) {
		for_each_set_bit(i, &changed_active_inputs, 5) {
			composer_cfg.compiz.valid_input_ids[i] = state->inputs[i].component->fw_id;
		}
		composer_cfg.compiz.active_inputs_mask = state->changed_active_inputs;
		composer_cfg.compiz.valid_input_num = hweight32(changed_active_inputs);

		update_flag = true;
	}

	if (update_flag)
	{
		ret = bst_display_dc_cmd_update_composer(subdev_session, &composer_cfg, &reply);
		if (ret || reply.base.status != DISP_COMM_REPLAY_OK)
			DRM_ERROR("UNC:%s falied!!\n", __func__);
	}
}

static void dc_crtc_disable(struct bst_virt_component *c)
{
	struct bst_display_submodule_req submodule_req = { 0 };
	uint32_t subdev_session = c->subdev_session;
	struct bst_display_comm_reply reply = { 0 };
	int ret;

	//submodule_req.submodule_type = DC_SUBMODULE_TYPE_COMPOSER;
	submodule_req.submodule_id = c->fw_id;

	ret = bst_display_dc_cmd_disable_submodule(subdev_session, &submodule_req, &reply);
	if (!ret && reply.base.status == DISP_COMM_REPLAY_OK)
		DRM_INFO("dc_crtc disable ok!!\n");
	else
		DRM_ERROR("dc_crtc disable falied!!\n");
}

static void dc_crtc_dump(struct bst_virt_component *c, struct seq_file *sf)
{
}

static const struct bst_virt_component_funcs dc_crtc_funcs = {
	.update = dc_crtc_update,
	.disable = dc_crtc_disable,
	.dump_log = dc_crtc_dump,
};

#if 0
static void dc_shared_crtc_disable(struct bst_virt_component *c)
{
}

static void dc_shared_crtc_update(struct bst_virt_component *c,
			  struct bst_virt_component_state *state)
{
	uint32_t subdev_session = c->subdev_session;
	struct bst_display_composer_cfg composer_cfg = {0};
	struct bst_display_comm_reply reply = {0};
	unsigned long changed_active_inputs;
	int ret, i = 0;

	changed_active_inputs = state->changed_active_inputs;
	if (0 == changed_active_inputs) {
		return;
	}

	for_each_set_bit(i, &changed_active_inputs, 5) {
		composer_cfg.compiz.valid_input_ids[i] = state->inputs[i].component->fw_id;
	}
	composer_cfg.compiz.active_inputs_mask = state->changed_active_inputs;
	composer_cfg.compiz.valid_input_num = hweight32(changed_active_inputs);

	ret = bst_display_dc_cmd_update_composer(subdev_session, &composer_cfg, &reply);
	if (ret || reply.base.status != DISP_COMM_REPLAY_OK)
		DRM_ERROR("UNC:%s falied!!\n", __func__);
}
static const struct bst_virt_component_funcs dc_shared_crtc_funcs = {
	.update = dc_shared_crtc_update,
	.disable = dc_shared_crtc_disable,
	.dump_log = dc_crtc_dump,
};
#endif

static int dc_crtc_init(struct virt_dc_dev *dc,
		       struct bst_display_submodule_header *submodule)
{
	struct bst_virt_component *comp = NULL;
	struct bst_virt_dc_crtc *dc_crtc;
	uint32_t pipe_id = dc->base_dev->this_pipe->pipe_id;
	uint32_t fw_id = SUBMODULE_INFO_SUBMODULE_ID(submodule->submodule_info);
	uint32_t comp_id = 0, min_fw_layer_id = dc->min_fw_layer_id;
	struct bst_display_composer_request cfg = {0};
	struct bst_display_composer_info composer_info = { 0 };
	uint32_t subdev_session = dc->base_dev->dev_info.subdev_session;

	get_resources_id(submodule->submodule_info, MAX_LAYER_NUM_PER_PIPE, min_fw_layer_id,
			 &comp_id);
	//if (dc->base_dev->dev_info.is_owner_device)
	{
		comp = bst_virt_component_add(dc->base_dev->this_pipe, dc->base_dev,
						sizeof(*dc_crtc), comp_id, fw_id,
						&dc_crtc_funcs, submodule->input_id_num,
						get_valid_inputs(submodule, min_fw_layer_id), 1,
						"VIRT_CRTC-%d", pipe_id);
	}
	#if 0
	else {
		comp = bst_virt_component_add(dc->base_dev->this_pipe, dc->base_dev,
						sizeof(*dc_crtc), comp_id, fw_id,
						&dc_shared_crtc_funcs, submodule->input_id_num,
						get_valid_inputs(submodule, min_fw_layer_id), 1,
						"VIRT_CRTC(shared)-%d", pipe_id);
	}
	#endif
	if (IS_ERR(comp)) {
		DRM_ERROR("Failed to add dc_crtc component\n");
		return PTR_ERR(comp);
	}

	dc_crtc = to_dc_crtc(comp);
	bst_display_dc_cmd_get_composer_info(subdev_session, &cfg, &composer_info);
	dc_crtc->supported_color_depths = composer_info.supported_color_depths;
	dc_crtc->supported_color_formats = composer_info.supported_color_formats;
	dc_crtc->supports_csc = composer_info.supports_csc;
	dc_crtc->supports_gamma = composer_info.supports_gamma;
	dc_crtc->supports_dual_link = composer_info.supports_dual_link;

	DRM_DEBUG("dc_crtc_init: supported_color_formats:0x%x supported_color_depths:0x%x\n",
		dc_crtc->supported_color_formats,
		dc_crtc->supported_color_depths);
	DRM_DEBUG("dc_crtc_init: supports_degamma:%d supports_csc:%d supports_gamma:%d supports_dual_link:%d\n",
		dc_crtc->supports_degamma,
		dc_crtc->supports_csc,
		dc_crtc->supports_gamma,
		dc_crtc->supports_dual_link);

	return 0;
}

int virt_dc_init_submodule(struct virt_dc_dev *dc,
		       struct bst_display_submodule_header *submodule)
{
	int err = 0;

	switch (SUBMODULE_INFO_SUBMODULE_TYPE(submodule->submodule_info)) {
	case DC_SUBMODULE_TYPE_LAYER:
		err = dc_layer_init(dc, submodule);
		break;
	case DC_SUBMODULE_TYPE_WB_LAYER:
		err = dc_wb_layer_init(dc, submodule);
		break;
	case DC_SUBMODULE_TYPE_COMPOSER:
		err = dc_crtc_init(dc, submodule);
		break;
	default:
		DRM_ERROR("Unknown submodule (submodule_info: 0x%x) is found\n",
			  submodule->submodule_info);
		err = -EINVAL;
		break;
	}

	return err;
}