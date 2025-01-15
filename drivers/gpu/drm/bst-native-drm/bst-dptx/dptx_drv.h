// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef __DPTX_DRIVER_H__
#define __DPTX_DRIVER_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/irq.h>
#include <drm/drm_device.h>
#include <drm/drm_connector.h>
#include <drm/drm_drv.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/display/drm_dp_helper.h>
#include <drm/drm_fixed.h>
#include <drm/display/drm_dp_mst_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include "drm_dp_helper_additions.h"
#include <../bst_drm_dev.h>

#define DPTX_RECEIVER_CAP_SIZE 0x100
#define DPTX_SDP_NUM	       0x10
#define DPTX_SDP_LEN	       0x9
#define DPTX_SDP_SIZE	       (9 * 4)
#define DPTX_COMBO_PHY
#define PARSE_EST_TIMINGS_FROM_BYTE3
#define ROUND_UP_TO_NEAREST(numToRound, mult) \
	((((numToRound + (mult)-1) / (mult)) * (mult)))

#include "avgen.h"
#include "dptx_reg.h"
#include "dptx_dbg.h"

struct dptx;

#define DPTX_MAX_LINK_RATE  DPTX_PHYIF_CTRL_RATE_HBR3
#define DPTX_MAX_LINK_LANES 4
#define DPTX_DEFAULT_LINK_RATE	DPTX_MAX_LINK_RATE
#define DPTX_DEFAULT_LINK_LANES DPTX_MAX_LINK_LANES
#define DPTX_MAX_STREAM_NUMBER 4

/**
 * struct dptx_link - The link state.
 * @status: Holds the sink status register values.
 * @trained: True if the link is successfully trained.
 * @rate: The current rate that the link is trained at.
 * @lanes: The current number of lanes that the link is trained at.
 * @preemp_level: The pre-emphasis level used for each lane.
 * @vswing_level: The vswing level used for each lane.
 */
struct dptx_link {
	u8 status[DP_LINK_STATUS_SIZE];
	bool trained;
	bool bypass_training;
	u8 rate;
	u8 lanes;
	u8 preemp_level[4];
	u8 vswing_level[4];
};

enum established_timings {
	DMT_640x480_60hz,
	DMT_800x600_60hz,
	DMT_1024x768_60hz,
	NONE
};

/**
 * struct dptx_aux - The aux state
 * @sts: The AUXSTS register contents.
 * @data: The AUX data register contents.
 * @event: Indicates an AUX event ocurred.
 * @abort: Indicates that the AUX transfer should be aborted.
 */
struct dptx_aux {
	u32 sts;
	u32 data[4];
	atomic_t abort;
};

struct sdp_header {
	u8 HB0;
	u8 HB1;
	u8 HB2;
	u8 HB3;
} __packed;

struct sdp_full_data {
	u8 en;
	u32 payload[9];
	u8 blanking;
	u8 cont;
} __packed;

struct adaptive_sync_sdp_data {
	struct sdp_header header;
	u8 payload[32];
	u8 size;
};

#define DPTX_HDCP_REG_DPK_CRC_ORIG 0x331c1169
#define DPTX_HDCP_MAX_AUTH_RETRY   10

struct hdcp_aksv {
	u32 lsb;
	u32 msb;
};

struct hdcp_dpk {
	u32 lsb;
	u32 msb;
};

struct hdcp_params {
	struct hdcp_aksv aksv;
	struct hdcp_dpk dpk[40];
	u32 enc_key;
	u32 crc32;
	u8 auth_fail_count;
	int hdcp13_is_en;
	int hdcp22_is_en;
};


#define INIT_FIELD(f) INIT_FIELD_CFG(field_##f, cfg_##f)
#define INIT_FIELD_CFG(f, conf)                                                \
	({                                                                     \
		dptx->f = devm_regmap_field_alloc(dptx->dev, dptx->regs[DPTX], \
						  variant->conf);              \
		if (IS_ERR(dptx->f))                                           \
			dev_warn(dptx->dev, "Ignoring regmap field" #f "\n");  \
	})

enum mem_res_enum { DPTX = 0, DPTX_CSR, DPTX_APG, PHYIF, MAX_MEM_IDX };
enum irq_res_enum { MAIN_FUNC_IRQ = 0, MAIN_PARITY_IRQ = 0, MAX_IRQ_IDX };

/**
 * struct dptx - The representation of the DP TX core
 * @mutex: dptx mutex
 * @base: Base address of the registers
 * @irq: IRQ number
 * @max_rate: The maximum rate that the controller supports
 * @max_lanes: The maximum lane count that the controller supports
 * @dev: The struct device
 * @root: The debugfs root
 * @regset: The debugfs regset
 * @vparams: The video params to use
 * @aparams: The audio params to use
 * @hparams: The HDCP params to use
 * @waitq: The waitq
 * @shutdown: Signals that the driver is shutting down and that all
 *	    operations should be aborted.
 * @c_connect: Signals that a HOT_PLUG or HOT_UNPLUG has occurred.
 * @sink_request: Signals the a HPD_IRQ has occurred.
 * @rx_caps: The sink's receiver capabilities.
 * @edid: The sink's EDID.
 * @sdp_list: The array of SDP elements
 * @aux: AUX channel state for performing an AUX transfer.
 * @link: The current link state.
 * @multipixel: Controls multipixel configuration. 0-Single, 1-Dual, 2-Quad.
 */
struct dptx {
	struct mutex mutex; /* generic mutex for dptx */
	struct {
		u8 multipixel;
		u8 streams;
		bool gen2phy;
		bool dsc;
	} hwparams;
	void __iomem *base[MAX_MEM_IDX];
	int irq[MAX_IRQ_IDX];
	struct clk *pxlclk;
	u8 max_rate;
	u8 max_lanes;
	bool ycbcr420;
	u8 streams;
	bool mst;
	bool enhance_frame_en;
	int active_mst_links;
	int active_mst_vc_payload;
	struct drm_dp_mst_topology_mgr mst_mgr;
	bool cr_fail;
	u8 multipixel;
	u8 bstatus;
	bool force_hpd;
	bool ssc_en;
	bool fec;
	bool dsc;
	bool edp;
	bool adaptive_sync;
	bool adaptive_sync_sdp;
	bool dummy_dtds_present;
	enum established_timings selected_est_timing;
	struct device *dev;
	struct dentry *root;
	struct debugfs_regset32 *regset[MAX_MEM_IDX];

	struct video_params vparams;
	struct audio_params aparams;
	struct audio_short_desc audio_desc;
	struct hdcp_params hparams;

	wait_queue_head_t waitq;

	atomic_t shutdown;
	atomic_t c_connect;
	atomic_t sink_request;

	u8 rx_caps[DPTX_RECEIVER_CAP_SIZE];

	u8 *edid;
	u8 *edid_second;
#define DPTX_DEFAULT_EDID_BUFLEN 128

	struct sdp_full_data sdp_list[DPTX_SDP_NUM];
	struct dptx_aux aux;
	struct dptx_link link;
	struct drm_device *drm;
	struct drm_connector *connector;
	struct drm_simple_display_pipe pipe;

#define XRES_DEF 640
#define YRES_DEF 480
#define XRES_MAX 8192
#define YRES_MAX 8192

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs;
	struct dw_debugfs_hwv *debugfs_hwv;
#endif

	struct regmap *regs[MAX_MEM_IDX];
	struct clkmng_regfields *clkmng_fields;
	struct rstmng_regfields *rstmng_fields;
	struct ag_regfields *ag_fields;
	struct vg_regfields *vg_fields;
	struct device *host_dpu;
	struct regmap_field *field_version_number;
	struct regmap_field *field_version_type;
	struct regmap_field *field_vendor_id;
	struct regmap_field *field_device_id;
	struct regmap_field *field_hdcp_select;
	struct regmap_field *field_audio_select;
	struct regmap_field *field_phy_used;
	struct regmap_field *field_sdp_reg_bank_size;
	struct regmap_field *field_fpga_en;
	struct regmap_field *field_dpk_romless;
	struct regmap_field *field_dpk_8bit;
	struct regmap_field *field_sync_depth;
	struct regmap_field *field_num_streams;
	struct regmap_field *field_mp_mode;
	struct regmap_field *field_dsc_en;
	struct regmap_field *field_edp_en;
	struct regmap_field *field_fec_en;
	struct regmap_field *field_gen2_phy;
	struct regmap_field *field_phy_type;
	struct regmap_field *field_adsync_en;
	struct regmap_field *field_psr_ver;
	struct regmap_field *field_scramble_dis;
	struct regmap_field *field_enhance_framing_en;
	struct regmap_field *field_default_fast_link_train_en;
	struct regmap_field *field_scale_down_mode;
	struct regmap_field *field_force_hpd;
	struct regmap_field *field_disable_interleaving;
	struct regmap_field *field_sel_aux_timeout_32ms;
	struct regmap_field *field_debug_control;
	struct regmap_field *field_sr_scale_down;
	struct regmap_field *field_bs_512_scale_down;
	struct regmap_field *field_enable_mst_mode;
	struct regmap_field *field_enable_fec;
	struct regmap_field *field_enable_edp;
	struct regmap_field *field_initiate_mst_act_seq;
	struct regmap_field *field_enhance_framing_with_fec_en;
	struct regmap_field *field_controller_reset;
	struct regmap_field *field_phy_soft_reset;
	struct regmap_field *field_hdcp_module_reset;
	struct regmap_field *field_audio_sampler_reset;
	struct regmap_field *field_aux_reset;
	struct regmap_field *field_video_reset;
	struct regmap_field *field_audio_sampler_reset_stream1;
	struct regmap_field *field_audio_sampler_reset_stream2;
	struct regmap_field *field_audio_sampler_reset_stream3;
	struct regmap_field *field_aux_cdr_state;
	struct regmap_field *field_aux_cdr_clock_cycle;
	struct regmap_field *field_video_stream_enable;
	struct regmap_field *field_video_mapping_ipi_en;
	struct regmap_field *field_video_mapping;
	struct regmap_field *field_pixel_mode_select;
	struct regmap_field *field_enable_dsc;
	struct regmap_field *field_encryption_enable;
	struct regmap_field *field_stream_type;
	struct regmap_field *field_bcb_data_stuffing_en;
	struct regmap_field *field_rcr_data_stuffing_en;
	struct regmap_field *field_gy_data_stuffing_en;
	struct regmap_field *field_bcb_stuff_data;
	struct regmap_field *field_rcr_stuff_data;
	struct regmap_field *field_gy_stuff_data;
	struct regmap_field *field_vsync_in_polarity;
	struct regmap_field *field_hsync_in_polarity;
	struct regmap_field *field_de_in_polarity;
	struct regmap_field *field_r_v_blank_in_osc;
	struct regmap_field *field_i_p;
	struct regmap_field *field_hblank_video_config1; /* hblank */
	struct regmap_field *field_hactive_video_config1; /* hactive */
	struct regmap_field *field_vactive_video_config2; /* vactive */
	struct regmap_field *field_vblank_video_config2; /* vblank */
	struct regmap_field *field_h_sync_width_video_config3; /* h_sync_width */
	struct regmap_field *field_v_sync_width_video_config4; /* v_sync_width */
	struct regmap_field *field_average_bytes_per_tu;
	struct regmap_field *field_init_threshold;
	struct regmap_field *field_average_bytes_per_tu_frac;
	struct regmap_field *field_enable_3d_frame_field_seq;
	struct regmap_field *field_init_threshold_hi;
	struct regmap_field *field_hstart;
	struct regmap_field *field_vstart;
	struct regmap_field *field_mvid;
	struct regmap_field *field_misc0;
	struct regmap_field *field_nvid;
	struct regmap_field *field_misc1;
	struct regmap_field *field_hblank_interval;
	struct regmap_field *field_mvid_cust_en;
	struct regmap_field *field_mvid_out_clr_mode;
	struct regmap_field *field_mvid_cust_den;
	struct regmap_field *field_mvid_cust_mod;
	struct regmap_field *field_mvid_cust_quo;
	struct regmap_field *field_audio_inf_select;
	struct regmap_field *field_audio_data_in_en;
	struct regmap_field *field_audio_data_width;
	struct regmap_field *field_hbr_mode_enable;
	struct regmap_field *field_num_channels;
	struct regmap_field *field_audio_mute;
	struct regmap_field *field_audio_packet_id;
	struct regmap_field *field_audio_timestamp_version_num;
	struct regmap_field *field_audio_clk_mult_fs;
	struct regmap_field *
		field_en_audio_timestamp_sdp_vertical_ctrl; /* en_audio_timestamp_sdp */
	struct regmap_field *
		field_en_audio_stream_sdp_vertical_ctrl; /* en_audio_stream_sdp */
	struct regmap_field *field_en_vertical_sdp_n;
	struct regmap_field *field_en_128bytes_sdp_1;
	struct regmap_field *field_disable_ext_sdp;
	struct regmap_field *
		field_fixed_priority_arbitration_vertical_ctrl; /* fixed_priority_arbitration */
	struct regmap_field *
		field_en_audio_timestamp_sdp_horizontal_ctrl; /* en_audio_timestamp_sdp */
	struct regmap_field *
		field_en_audio_stream_sdp_horizontal_ctrl; /* en_audio_stream_sdp */
	struct regmap_field *field_en_horizontal_sdp_n;
	struct regmap_field *
		field_fixed_priority_arbitration_horizontal_ctrl; /* fixed_priority_arbitration */
	struct regmap_field *field_audio_timestamp_sdp_status;
	struct regmap_field *field_audio_stream_sdp_status;
	struct regmap_field *field_sdp_n_tx_status;
	struct regmap_field *field_manual_mode_sdp;
	struct regmap_field *field_audio_timestamp_sdp_status_en;
	struct regmap_field *field_audio_stream_sdp_status_en;
	struct regmap_field *field_sdp_status_en;
	struct regmap_field *field_sdp_16b_bytes_reqd_vblank_ovr;
	struct regmap_field *field_sdp_16b_bytes_reqd_hblank_ovr;
	struct regmap_field *field_sdp_32b_bytes_reqd_vblank_ovr;
	struct regmap_field *field_sdp_32b_bytes_reqd_hblank_ovr;
	struct regmap_field *field_sdp_128b_bytes_reqd_vblank_ovr;
	struct regmap_field *field_sdp_128b_bytes_reqd_hblank_ovr;
	struct regmap_field *field_tps_sel;
	struct regmap_field *field_phyrate;
	struct regmap_field *field_phy_lanes;
	struct regmap_field *field_xmit_enable;
	struct regmap_field *field_phy_busy;
	struct regmap_field *field_ssc_dis;
	struct regmap_field *field_phy_powerdown;
	struct regmap_field *field_phy_width;
	struct regmap_field *field_edp_phy_rate;
	struct regmap_field *field_lane0_tx_preemp;
	struct regmap_field *field_lane0_tx_vswing;
	struct regmap_field *field_lane1_tx_preemp;
	struct regmap_field *field_lane1_tx_vswing;
	struct regmap_field *field_lane2_tx_preemp;
	struct regmap_field *field_lane2_tx_vswing;
	struct regmap_field *field_lane3_tx_preemp;
	struct regmap_field *field_lane3_tx_vswing;
	struct regmap_field *field_custom80b_0;
	struct regmap_field *field_custom80b_1;
	struct regmap_field *field_custom80b_2;
	struct regmap_field *field_num_sr_zeros;
	struct regmap_field *field_aux_len_req;
	struct regmap_field *field_i2c_addr_only;
	struct regmap_field *field_aux_addr;
	struct regmap_field *field_aux_cmd_type;
	struct regmap_field *field_aux_status;
	struct regmap_field *field_aux_m;
	struct regmap_field *field_aux_reply_received;
	struct regmap_field *field_aux_timeout;
	struct regmap_field *field_aux_reply_err;
	struct regmap_field *field_aux_bytes_read;
	struct regmap_field *field_sink_disconnect_while_active;
	struct regmap_field *field_aux_reply_err_code;
	struct regmap_field *field_aux_state;
	struct regmap_field *field_aux_data0;
	struct regmap_field *field_aux_data1;
	struct regmap_field *field_aux_data2;
	struct regmap_field *field_aux_data3;
	struct regmap_field *field_aux_250us_cnt_limit;
	struct regmap_field *field_aux_2000us_cnt_limit;
	struct regmap_field *field_aux_100000us_cnt_limit;
	struct regmap_field *field_tx0_in_generic_bus;
	struct regmap_field *field_tx0_hp_prot_en;
	struct regmap_field *field_tx0_bypass_eq_calc;
	struct regmap_field *field_tx1_in_generic_bus;
	struct regmap_field *field_tx1_hp_prot_en;
	struct regmap_field *field_tx1_bypass_eq_calc;
	struct regmap_field *field_tx2_in_generic_bus;
	struct regmap_field *field_tx2_hp_prot_en;
	struct regmap_field *field_tx2_bypass_eq_calc;
	struct regmap_field *field_tx3_in_generic_bus;
	struct regmap_field *field_tx3_hp_prot_en;
	struct regmap_field *field_tx3_bypass_eq_calc;
	struct regmap_field *field_tx0_out_generic_bus;
	struct regmap_field *field_tx1_out_generic_bus;
	struct regmap_field *field_tx2_out_generic_bus;
	struct regmap_field *field_tx3_out_generic_bus;
	struct regmap_field *field_combo_phy_ovr;
	struct regmap_field *field_combo_phy_ovr_mpll_multiplier;
	struct regmap_field *field_combo_phy_ovr_mpll_div_multiplier;
	struct regmap_field *field_combo_phy_ovr_mpll_tx_clk_div;
	struct regmap_field *field_combo_phy_ovr_mpll_ssc_freq_cnt_init;
	struct regmap_field *field_combo_phy_ovr_mpll_ssc_freq_cnt_peak;
	struct regmap_field *field_combo_phy_ovr_mpll_ssc_freq_cnt_ovrd_en;
	struct regmap_field *field_combo_phy_ovr_mpll_div_clk_en;
	struct regmap_field *field_combo_phy_ovr_mpll_word_div2_en;
	struct regmap_field *field_combo_phy_ovr_mpll_init_cal_disable;
	struct regmap_field *field_combo_phy_ovr_mpll_pmix_en;
	struct regmap_field *field_combo_phy_ovr_mpll_v2i;
	struct regmap_field *field_combo_phy_ovr_mpll_cp_int;
	struct regmap_field *field_combo_phy_ovr_mpll_cp_prop;
	struct regmap_field *field_combo_phy_ovr_mpll_ssc_up_spread;
	struct regmap_field *field_combo_phy_ovr_mpll_ssc_peak;
	struct regmap_field *field_combo_phy_ovr_mpll_ssc_stepsize;
	struct regmap_field *field_combo_phy_ovr_mpll_fracn_cfg_update_en;
	struct regmap_field *field_combo_phy_ovr_mpll_fracn_en;
	struct regmap_field *field_combo_phy_ovr_mpll_fracn_den;
	struct regmap_field *field_combo_phy_ovr_mpll_fracn_quot;
	struct regmap_field *field_combo_phy_ovr_mpll_fracn_rem;
	struct regmap_field *field_combo_phy_ovr_mpll_freq_vco;
	struct regmap_field *field_combo_phy_ovr_ref_clk_mpll_div;
	struct regmap_field *field_combo_phy_ovr_mpll_div5_clk_en;
	struct regmap_field *field_combo_phy_ovr_tx0_term_ctrl;
	struct regmap_field *field_combo_phy_ovr_tx1_term_ctrl;
	struct regmap_field *field_combo_phy_ovr_tx2_term_ctrl;
	struct regmap_field *field_combo_phy_ovr_tx3_term_ctrl;
	struct regmap_field *field_combo_phy_ovr_tx_eq_g1;
	struct regmap_field *field_combo_phy_ovr_tx_eq_main_g1;
	struct regmap_field *field_combo_phy_ovr_tx_eq_post_g1;
	struct regmap_field *field_combo_phy_ovr_tx_eq_pre_g1;
	struct regmap_field *field_combo_phy_ovr_tx_eq_g2;
	struct regmap_field *field_combo_phy_ovr_tx_eq_main_g2;
	struct regmap_field *field_combo_phy_ovr_tx_eq_post_g2;
	struct regmap_field *field_combo_phy_ovr_tx_eq_pre_g2;
	struct regmap_field *field_combo_phy_ovr_tx_eq_g3;
	struct regmap_field *field_combo_phy_ovr_tx_eq_main_g3;
	struct regmap_field *field_combo_phy_ovr_tx_eq_post_g3;
	struct regmap_field *field_combo_phy_ovr_tx_eq_pre_g3;
	struct regmap_field *field_combo_phy_ovr_tx_eq_g4;
	struct regmap_field *field_combo_phy_ovr_tx_eq_main_g4;
	struct regmap_field *field_combo_phy_ovr_tx_eq_post_g4;
	struct regmap_field *field_combo_phy_ovr_tx_eq_pre_g4;
	struct regmap_field *field_combo_phy_ovr_tx_eq_g5;
	struct regmap_field *field_combo_phy_ovr_tx_eq_main_g5;
	struct regmap_field *field_combo_phy_ovr_tx_eq_post_g5;
	struct regmap_field *field_combo_phy_ovr_tx_eq_pre_g5;
	struct regmap_field *field_combo_phy_ovr_tx_eq_g6;
	struct regmap_field *field_combo_phy_ovr_tx_eq_main_g6;
	struct regmap_field *field_combo_phy_ovr_tx_eq_post_g6;
	struct regmap_field *field_combo_phy_ovr_tx_eq_pre_g6;
	struct regmap_field *field_combo_phy_ovr_tx_eq_g7;
	struct regmap_field *field_combo_phy_ovr_tx_eq_main_g7;
	struct regmap_field *field_combo_phy_ovr_tx_eq_post_g7;
	struct regmap_field *field_combo_phy_ovr_tx_eq_pre_g7;
	struct regmap_field *field_combo_phy_ovr_tx_eq_g8;
	struct regmap_field *field_combo_phy_ovr_tx_eq_main_g8;
	struct regmap_field *field_combo_phy_ovr_tx_eq_post_g8;
	struct regmap_field *field_combo_phy_ovr_tx_eq_pre_g8;
	struct regmap_field *field_combo_phy_ovr_tx0_en;
	struct regmap_field *field_combo_phy_ovr_tx0_vboost_en;
	struct regmap_field *field_combo_phy_ovr_tx0_iboost_lvl;
	struct regmap_field *field_combo_phy_ovr_tx0_clk_rdy;
	struct regmap_field *field_combo_phy_ovr_tx0_invert;
	struct regmap_field *field_combo_phy_ovr_tx1_en;
	struct regmap_field *field_combo_phy_ovr_tx1_vboost_en;
	struct regmap_field *field_combo_phy_ovr_tx1_iboost_lvl;
	struct regmap_field *field_combo_phy_ovr_tx1_clk_rdy;
	struct regmap_field *field_combo_phy_ovr_tx1_invert;
	struct regmap_field *field_combo_phy_ovr_tx2_en;
	struct regmap_field *field_combo_phy_ovr_tx2_vboost_en;
	struct regmap_field *field_combo_phy_ovr_tx2_iboost_lvl;
	struct regmap_field *field_combo_phy_ovr_tx2_clk_rdy;
	struct regmap_field *field_combo_phy_ovr_tx2_invert;
	struct regmap_field *field_combo_phy_ovr_tx3_en;
	struct regmap_field *field_combo_phy_ovr_tx3_vboost_en;
	struct regmap_field *field_combo_phy_ovr_tx3_iboost_lvl;
	struct regmap_field *field_combo_phy_ovr_tx3_clk_rdy;
	struct regmap_field *field_combo_phy_ovr_tx3_invert;
	struct regmap_field *field_hpd_event;
	struct regmap_field *field_aux_reply_event;
	struct regmap_field *field_hdcp_event;
	struct regmap_field *field_aux_cmd_invalid;
	struct regmap_field *field_sdp_event_stream0;
	struct regmap_field *field_audio_fifo_overflow_stream0;
	struct regmap_field *field_video_fifo_overflow_stream0;
	struct regmap_field *field_video_fifo_underflow_stream0;
	struct regmap_field *field_sdp_event_stream1;
	struct regmap_field *field_audio_fifo_overflow_stream1;
	struct regmap_field *field_video_fifo_overflow_stream1;
	struct regmap_field *field_video_fifo_underflow_stream1;
	struct regmap_field *field_sdp_event_stream2;
	struct regmap_field *field_audio_fifo_overflow_stream2;
	struct regmap_field *field_video_fifo_overflow_stream2;
	struct regmap_field *field_video_fifo_underflow_stream2;
	struct regmap_field *field_sdp_event_stream3;
	struct regmap_field *field_audio_fifo_overflow_stream3;
	struct regmap_field *field_video_fifo_overflow_stream3;
	struct regmap_field *field_video_fifo_underflow_stream3;
	struct regmap_field *field_dsc_event;
	struct regmap_field *field_hpd_event_en;
	struct regmap_field *field_aux_reply_event_en;
	struct regmap_field *field_hdcp_event_en;
	struct regmap_field *field_aux_cmd_invalid_en;
	struct regmap_field *field_sdp_event_en_stream0;
	struct regmap_field *field_audio_fifo_overflow_en_stream0;
	struct regmap_field *field_video_fifo_overflow_en_stream0;
	struct regmap_field *field_video_fifo_underflow_en_stream0;
	struct regmap_field *field_sdp_event_en_stream1;
	struct regmap_field *field_audio_fifo_overflow_en_stream1;
	struct regmap_field *field_video_fifo_overflow_en_stream1;
	struct regmap_field *field_video_fifo_underflow_en_stream1;
	struct regmap_field *field_sdp_event_en_stream2;
	struct regmap_field *field_audio_fifo_overflow_en_stream2;
	struct regmap_field *field_video_fifo_overflow_en_stream2;
	struct regmap_field *field_video_fifo_underflow_en_stream2;
	struct regmap_field *field_sdp_event_en_stream3;
	struct regmap_field *field_audio_fifo_overflow_en_stream3;
	struct regmap_field *field_video_fifo_overflow_en_stream3;
	struct regmap_field *field_video_fifo_underflow_en_stream3;
	struct regmap_field *field_dsc_event_en;
	struct regmap_field *field_hpd_irq;
	struct regmap_field *field_hpd_hot_plug;
	struct regmap_field *field_hpd_hot_unplug;
	struct regmap_field *field_hpd_unplug_err;
	struct regmap_field *field_hpd_status;
	struct regmap_field *field_hpd_state;
	struct regmap_field *field_hpd_timer;
	struct regmap_field *field_hpd_irq_en;
	struct regmap_field *field_hpd_plug_en;
	struct regmap_field *field_hpd_unplug_en;
	struct regmap_field *field_hpd_unplug_err_en;
	struct regmap_field *field_enable_hdcp;
	struct regmap_field *field_enable_hdcp_13;
	struct regmap_field *field_encryptiondisable;
	struct regmap_field *field_hdcp_lock;
	struct regmap_field *field_bypencryption;
	struct regmap_field *field_cp_irq;
	struct regmap_field *field_dpcd12plus;
	struct regmap_field *field_hdcpengaged;
	struct regmap_field *field_substatea;
	struct regmap_field *field_statea;
	struct regmap_field *field_stater;
	struct regmap_field *field_stateoeg;
	struct regmap_field *field_statee;
	struct regmap_field *field_hdcp_capable;
	struct regmap_field *field_repeater;
	struct regmap_field *field_hdcp13_bstatus;
	struct regmap_field *field_hdcp2_booted;
	struct regmap_field *field_hdcp2_state;
	struct regmap_field *field_hdcp2_sink_cap_check_complete;
	struct regmap_field *field_hdcp2_capable_sink;
	struct regmap_field *field_hdcp2_authentication_success;
	struct regmap_field *field_hdcp2_authentication_failed;
	struct regmap_field *field_hdcp2_re_authentication_req;
	struct regmap_field *field_ksvaccessint_clr; /* ksvaccessint */
	struct regmap_field
		*field_auxrespdefer7times_clr; /* auxrespdefer7times */
	struct regmap_field *field_auxresptimeout_clr; /* auxresptimeout */
	struct regmap_field *field_auxrespnack7times_clr; /* auxrespnack7times */
	struct regmap_field
		*field_ksvsha1calcdoneint_clr; /* ksvsha1calcdoneint */
	struct regmap_field *field_hdcp_failed_clr; /* hdcp_failed */
	struct regmap_field *field_hdcp_engaged_clr; /* hdcp_engaged */
	struct regmap_field *field_hdcp2_gpioint_clr; /* hdcp2_gpioint */
	struct regmap_field *field_ksvaccessint_stat; /* ksvaccessint */
	struct regmap_field
		*field_auxrespdefer7times_stat; /* auxrespdefer7times */
	struct regmap_field *field_auxresptimeout_stat; /* auxresptimeout */
	struct regmap_field *field_auxrespnack7times_stat; /* auxrespnack7times */
	struct regmap_field
		*field_ksvsha1calcdoneint_stat; /* ksvsha1calcdoneint */
	struct regmap_field *field_hdcp_failed_stat; /* hdcp_failed */
	struct regmap_field *field_hdcp_engaged_stat; /* hdcp_engaged */
	struct regmap_field *field_hdcp2_gpioint_stat; /* hdcp2_gpioint */
	struct regmap_field *field_ksvaccessint_msk; /* ksvaccessint */
	struct regmap_field
		*field_auxrespdefer7times_msk; /* auxrespdefer7times */
	struct regmap_field *field_auxresptimeout_msk; /* auxresptimeout */
	struct regmap_field *field_auxrespnack7times_msk; /* auxrespnack7times */
	struct regmap_field
		*field_ksvsha1calcdoneint_msk; /* ksvsha1calcdoneint */
	struct regmap_field *field_hdcp_failed_msk; /* hdcp_failed */
	struct regmap_field *field_hdcp_engaged_msk; /* hdcp_engaged */
	struct regmap_field *field_hdcp2_gpioint_msk; /* hdcp2_gpioint */
	struct regmap_field *field_ksvmemrequest;
	struct regmap_field *field_ksvmemaccess;
	struct regmap_field *field_ksvlistprocessupd;
	struct regmap_field *field_ksvsha1swstatus;
	struct regmap_field *field_ksvsha1status;
	struct regmap_field *field_hdcpreg_bksv0;
	struct regmap_field *field_hdcpreg_bksv1;
	struct regmap_field *field_oanbypass;
	struct regmap_field *field_hdcpreg_an0;
	struct regmap_field *field_hdcpreg_an1;
	struct regmap_field *field_odpk_decrypt_enable;
	struct regmap_field *field_idpk_data_index;
	struct regmap_field *field_idpk_wr_ok_sts;
	struct regmap_field *field_hdcpreg_seed;
	struct regmap_field *field_dpk_data_0; /* dpk_data */
	struct regmap_field *field_dpk_data_1; /* dpk_data */
	struct regmap_field *field_hdcp2gpiooutsts;
	struct regmap_field *field_hdcp2gpiooutchngsts;
	struct regmap_field *field_dpk_crc;
	struct regmap_field *field_vg_swrst;
	struct regmap_field *field_odepolarity;
	struct regmap_field *field_ohsyncpolarity;
	struct regmap_field *field_ovsyncpolarity;
	struct regmap_field *field_oip;
	struct regmap_field *field_ocolorincrement;
	struct regmap_field *field_ovblankoscillation;
	struct regmap_field *field_ycc_422_mapping;
	struct regmap_field *field_ycc_pattern_generation;
	struct regmap_field *field_pixel_repetition;
	struct regmap_field *field_bits_per_comp;
	struct regmap_field *field_ycc_420_mapping;
	struct regmap_field *field_internal_external_gen;
	struct regmap_field *field_pattern_mode;
	struct regmap_field *field_hactive_vg_config2; /* hactive */
	struct regmap_field *field_hblank_vg_config2; /* hblank */
	struct regmap_field *field_h_front_porch;
	struct regmap_field *field_h_sync_width_vg_config3; /* h_sync_width */
	struct regmap_field *field_vactive_vg_config4; /* vactive */
	struct regmap_field *field_vblank_vg_config4; /* vblank */
	struct regmap_field *field_v_front_porch;
	struct regmap_field *field_v_sync_width_vg_config5; /* v_sync_width */
	struct regmap_field *field_td_structure;
	struct regmap_field *field_td_enable;
	struct regmap_field *field_td_frameseq;
	struct regmap_field *field_ipi_enable;
	struct regmap_field *field_ipi_select;
	struct regmap_field *field_ram_addr_start;
	struct regmap_field *field_start_write_ram;
	struct regmap_field *field_write_ram_data;
	struct regmap_field *field_ram_stop_addr;
	struct regmap_field *field_vg_cb_width;
	struct regmap_field *field_vg_cb_height;
	struct regmap_field *field_vg_cb_colora_lsb;
	struct regmap_field *field_vg_cb_color_a_msb;
	struct regmap_field *field_vg_cb_color_b_lsb;
	struct regmap_field *field_vg_cb_color_b_msb;
	struct regmap_field *field_ag_swrst;
	struct regmap_field *field_hbren;
	struct regmap_field *field_audiosource_clockmultiplier;
	struct regmap_field *field_i2s_wordwidth;
	struct regmap_field *field_audio_source;
	struct regmap_field *field_nlpcm_en;
	struct regmap_field *field_spdiftxdata;
	struct regmap_field *field_audio_use_lut;
	struct regmap_field *field_audio_use_counter;
	struct regmap_field *field_audio_counter_offset;
	struct regmap_field *field_incleft;
	struct regmap_field *field_incright;
	struct regmap_field *field_iec_copyright;
	struct regmap_field *field_iec_cgmsa;
	struct regmap_field *field_iec_nlpcm;
	struct regmap_field *field_iec_categorycode;
	struct regmap_field *field_iec_sourcenumber;
	struct regmap_field *field_iec_pcm_audio_mode;
	struct regmap_field *field_iec_channelnumcl0_3; /* iec_channelnumcl0 */
	struct regmap_field *field_iec_channelnumcr0_3; /* iec_channelnumcr0 */
	struct regmap_field *field_iec_samp_freq;
	struct regmap_field *field_iec_clkaccuracy;
	struct regmap_field *field_iec_word_length;
	struct regmap_field *field_iec_origsampfreq;
	struct regmap_field *field_iec_channelnumcl0_5; /* iec_channelnumcl0 */
	struct regmap_field *field_iec_channelnumcr0_5; /* iec_channelnumcr0 */
	struct regmap_field *field_iec_channelnumcl1;
	struct regmap_field *field_iec_channelnumcr1;
	struct regmap_field *field_iec_channelnumcl2;
	struct regmap_field *field_iec_channelnumcr2;
	struct regmap_field *field_iec_channelnumcl2a;
	struct regmap_field *field_iec_channelnumcr2a;
	struct regmap_field *field_userdata_cl0;
	struct regmap_field *field_userdata_cr0;
	struct regmap_field *field_userdata_cl1;
	struct regmap_field *field_userdata_cr1;
	struct regmap_field *field_userdata_cl2;
	struct regmap_field *field_userdata_cr2;
	struct regmap_field *field_userdata_cl3;
	struct regmap_field *field_userdata_cr3;
	struct regmap_field *field_validity_bit_cl0;
	struct regmap_field *field_validity_bit_cr0;
	struct regmap_field *field_validity_bit_cl1;
	struct regmap_field *field_validity_bit_cr1;
	struct regmap_field *field_validity_bit_cl2;
	struct regmap_field *field_validity_bit_cr2;
	struct regmap_field *field_validity_bit_cl3;
	struct regmap_field *field_validity_bit_cr3;
};


int dptx_core_init(struct dptx *dptx);
void dptx_init_hwparams(struct dptx *dptx);
bool dptx_check_dptx_id(struct dptx *dptx);
void dptx_core_init_phy(struct dptx *dptx);
int dptx_core_program_ssc(struct dptx *dptx, bool sink_ssc);
bool dptx_sink_enabled_ssc(struct dptx *dptx);
void dptx_enable_ssc(struct dptx *dptx);

int dptx_core_deinit(struct dptx *dptx);
void dptx_soft_reset(struct dptx *dptx, u32 bits);
void dptx_soft_reset_all(struct dptx *dptx);
void dptx_phy_soft_reset(struct dptx *dptx);

irqreturn_t dptx_irq(int irq, void *dev);
irqreturn_t dptx_threaded_irq(int irq, void *dev);

void dptx_global_intr_en(struct dptx *dp);
void dptx_global_intr_dis(struct dptx *dp);

void dptx_phy_set_lanes(struct dptx *dptx, unsigned int num);
unsigned int dptx_phy_get_lanes(struct dptx *dptx);
void dptx_phy_set_rate(struct dptx *dptx, unsigned int rate);
unsigned int bst_phy_get_rate(struct dptx *dptx);
int dptx_phy_wait_busy(struct dptx *dptx, unsigned int lanes);
void dptx_phy_set_pre_emphasis(struct dptx *dptx, unsigned int lane,
			       unsigned int level);
void dptx_phy_set_vswing(struct dptx *dptx, unsigned int lane,
			 unsigned int level);
void dptx_phy_set_pattern(struct dptx *dptx, unsigned int pattern);
void dptx_phy_enable_xmit(struct dptx *dptx, unsigned int lane, bool enable);

int dptx_phy_rate_to_bw(unsigned int rate);
int dptx_bw_to_phy_rate(unsigned int bw);
int dptx_lanes_to_dpcd_lanes(unsigned int lanes, bool enhance_frame_en);

#define DPTX_AUX_TIMEOUT 2000

int dptx_read_bytes_from_i2c(struct dptx *dptx, unsigned int device_addr,
			     u8 *bytes, u32 len);

int dptx_i2c_address_only(struct dptx *dptx, unsigned int device_addr);

int dptx_write_bytes_to_i2c(struct dptx *dptx, unsigned int device_addr,
			    u8 *bytes, u32 len);

int __dptx_read_dpcd(struct dptx *dptx, u32 addr, u8 *byte);
int __dptx_write_dpcd(struct dptx *dptx, u32 addr, u8 byte);

int __dptx_read_bytes_from_dpcd(struct dptx *dptx, unsigned int reg_addr,
				u8 *bytes, u32 len);

int __dptx_write_bytes_to_dpcd(struct dptx *dptx, unsigned int reg_addr,
			       u8 *bytes, u32 len);

#ifndef DPTX_DEBUG_DPCD_CMDS
static inline int dptx_read_dpcd(struct dptx *dptx, u32 addr, u8 *byte)
{
	__dptx_read_dpcd(dptx, addr, byte);
}

static inline int dptx_write_dpcd(struct dptx *dptx, u32 addr, u8 byte)
{
	__dptx_write_dpcd(dptx, addr, byte);
}

static inline int dptx_read_bytes_from_dpcd(struct dptx *dptx,
					    unsigned int reg_addr, u8 *bytes,
					    u32 len)
{
	return __dptx_read_bytes_from_dpcd(dptx, reg_addr, bytes, len);
}

static inline int dptx_write_bytes_to_dpcd(struct dptx *dptx,
					   unsigned int reg_addr, u8 *bytes,
					   u32 len)
{
	return __dptx_write_bytes_to_dpcd(dptx, reg_addr, bytes, len);
}

#else
#define dptx_read_dpcd(_dptx, _addr, _byteptr)                        \
	({                                                            \
		int _ret = __dptx_read_dpcd(_dptx, _addr, _byteptr);  \
		dptx_dbg(dptx, "%s: DPCD Read %s(0x%03x) = 0x%02x\n", \
			 __func__, #_addr, _addr, *(_byteptr));       \
		_ret;                                                 \
	})

#define dptx_write_dpcd(_dptx, _addr, _byte)                           \
	({                                                             \
		int _ret;                                              \
		dptx_dbg(dptx, "%s: DPCD Write %s(0x%03x) = 0x%02x\n", \
			 __func__, #_addr, _addr, _byte);              \
		_ret = __dptx_write_dpcd(_dptx, _addr, _byte);         \
		_ret;                                                  \
	})

char *__bytes_str(u8 *bytes, unsigned int n);

#define dptx_read_bytes_from_dpcd(_dptx, _addr, _b, _len)                   \
	({                                                                  \
		int _ret;                                                   \
		char *_str;                                                 \
		_ret = __dptx_read_bytes_from_dpcd(_dptx, _addr, _b, _len); \
		_str = __bytes_str(_b, _len);                               \
		dptx_dbg(dptx,                                              \
			 "%s: Read %llu bytes from %s(0x%02x) = [ %s ]\n",  \
			 __func__, (u64)_len, #_addr, _addr, _str);         \
		_ret;                                                       \
	})

#define dptx_write_bytes_to_dpcd(_dptx, _addr, _b, _len)                    \
	({                                                                  \
		int _ret;                                                   \
		char *_str = __bytes_str(_b, _len);                         \
		dptx_dbg(dptx,                                              \
			 "%s: Writing %llu bytes to %s(0x%02x) = [ %s ]\n", \
			 __func__, (u64)_len, #_addr, _addr, _str);         \
		_ret = __dptx_write_bytes_to_dpcd(_dptx, _addr, _b, _len);  \
		_ret;                                                       \
	})

#endif

int dptx_set_link_configs(struct dptx *dptx, u8 rate, u8 laness);
int dptx_link_training(struct dptx *dptx);
int dptx_fast_link_training(struct dptx *dptx);
int dptx_link_check_status(struct dptx *dptx);
int dptx_disconnect_link(struct dptx *dptx);
int dptx_xmit_enable(struct dptx *dptx, bool enable);
static inline u32 __dptx_read_reg(struct dptx *dp, char const *func, int line,
				  struct regmap *regm, u32 reg)
{
	u32 val;

	regmap_read(regm, reg, &val);
#ifdef DPTX_DEBUG_REG
	dptx_dbg(dp, "%s:%d: READ: addr=0x%05x data=0x%08x\n", func, line, reg,
		 val);
#endif

	return val;
}

#define dptx_read_reg(_dptx, _regm, _reg) \
	({ __dptx_read_reg(_dptx, __func__, __LINE__, _regm, _reg); })

static inline void __dptx_write_reg(struct dptx *dp, char const *func, int line,
				    struct regmap *regm, u32 reg, u32 val)
{
#ifdef DPTX_DEBUG_REG
	dptx_dbg(dp, "%s:%d: WRITE: addr=0x%05x data=0x%08x\n", func, line, reg,
		 val);
#endif

	regmap_write(regm, reg, val);
}

#define dptx_write_reg(_dptx, _regm, _reg, _val) \
	({ __dptx_write_reg(_dptx, __func__, __LINE__, _regm, _reg, _val); })

static inline u32 __dptx_read_regfield(struct dptx *dp, char const *func,
				       int line, struct regmap_field *reg_field)
{
	u32 val;

	regmap_field_read(reg_field, &val);

#ifdef DPTX_DEBUG_REG
	dptx_dbg(dp, "%s:%d: READ: reg=0x%05x mask=0x%05x\n", func, line,
		 reg_field->reg, reg_field->mask);
	dptx_dbg(dp, "%s:%d: READ: shift=0x%05x data=0x%08x\n", func, line,
		 reg_field->shift, val);
#endif

	return val;
}

#define dptx_read_regfield(_dptx, _reg_field) \
	({ __dptx_read_regfield(_dptx, __func__, __LINE__, _reg_field); })

static inline void __dptx_write_regfield(struct dptx *dp, char const *func,
					 int line,
					 struct regmap_field *reg_field,
					 u32 val)
{
#ifdef DPTX_DEBUG_REG
	dptx_dbg(dp, "%s:%d: WRITE: reg=0x%05x mask=0x%05x\n", func, line,
		 reg_field->reg, reg_field->mask);
	dptx_dbg(dp, "%s:%d: WRITE: shift=0x%05x data=0x%08x\n", func, line,
		 reg_field->shift, val);
#endif

	regmap_field_force_write(reg_field, val);
}

#define dptx_write_regfield(_dptx, _reg_field, _val)                         \
	({                                                                   \
		__dptx_write_regfield(_dptx, __func__, __LINE__, _reg_field, \
				      _val);                                 \
	})

int dptx_regmap_fields_init(struct dptx *dptx);

#define dptx_wait(_dptx, _cond, _timeout)                             \
	({                                                            \
		int __retval;                                         \
		__retval = wait_event_interruptible_timeout(          \
			_dptx->waitq,                                 \
			((_cond) || (atomic_read(&_dptx->shutdown))), \
			msecs_to_jiffies(_timeout));                  \
		if (atomic_read(&_dptx->shutdown)) {                  \
			__retval = -ESHUTDOWN;                        \
		} else if (!__retval) {                               \
			__retval = -ETIMEDOUT;                        \
		}                                                     \
		__retval;                                             \
	})

void dptx_notify(struct dptx *dptx);
void dptx_notify_shutdown(struct dptx *dptx);
void dptx_debugfs_init(struct dptx *dptx);
void dptx_debugfs_exit(struct dptx *dptx);
void dptx_fill_sdp(struct dptx *dptx, struct sdp_full_data *data);
void dptx_init_hdcp_keys(struct dptx *dptx);
struct dptx *dptx_get_handle(void);

int dptx_aux_rw_bytes(struct dptx *dptx, bool rw, bool i2c, u32 addr, u8 *bytes,
		      unsigned int len);
int dptx_read_bytes_from_i2c(struct dptx *dptx, u32 device_addr, u8 *bytes,
			     u32 len);
int dptx_write_bytes_to_i2c(struct dptx *dptx, u32 device_addr, u8 *bytes,
			    u32 len);
void dptx_write_hdcp22_test_keys(struct dptx *dptx);
void dptx_audio_sdp_en(struct dptx *dptx);
void dptx_audio_timestamp_sdp_en(struct dptx *dptx);

#define AUDIO_TAG	1
#define VIDEO_TAG	2
#define EDID_TAG_MASK	GENMASK(7, 5)
#define EDID_TAG_SHIFT	5
#define EDID_SIZE_MASK	GENMASK(4, 0)
#define EDID_SIZE_SHIFT 0
#define ET1_800x600_60hz BIT(0)
#define ET1_800x600_56hz BIT(1)
#define ET1_640x480_75hz BIT(2)
#define ET1_640x480_72hz BIT(3)
#define ET1_640x480_67hz BIT(4)
#define ET1_640x480_60hz BIT(5)
#define ET1_720x400_88hz BIT(6)
#define ET1_720x400_70hz BIT(7)
#define ET2_1280x1024_75hz BIT(0)
#define ET2_1024x768_75hz  BIT(1)
#define ET2_1024x768_70hz  BIT(2)
#define ET2_1024x768_60hz  BIT(3)
#define ET2_1024x768_87hz  BIT(4)
#define ET2_832x624_75hz   BIT(5)
#define ET2_800x600_75hz   BIT(6)
#define ET2_800x600_72hz   BIT(7)
#define ET3_1152x870_75hz BIT(7)

int dptx_read_edid(struct dptx *dptx);
int dptx_check_edid(struct dptx *dptx);

#endif
