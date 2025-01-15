// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include "dptx_drv.h"

int dptx_regmap_fields_init(struct dptx *dptx)
{
	const struct struct_variant *variant = &local_variant;
	INIT_FIELD(version_number);
	INIT_FIELD(version_type);
	INIT_FIELD(vendor_id);
	INIT_FIELD(device_id);
	INIT_FIELD(hdcp_select);
	INIT_FIELD(audio_select);
	INIT_FIELD(phy_used);
	INIT_FIELD(sdp_reg_bank_size);
	INIT_FIELD(fpga_en);
	INIT_FIELD(dpk_romless);
	INIT_FIELD(dpk_8bit);
	INIT_FIELD(sync_depth);
	INIT_FIELD(num_streams);
	INIT_FIELD(mp_mode);
	INIT_FIELD(dsc_en);
	INIT_FIELD(edp_en);
	INIT_FIELD(fec_en);
	INIT_FIELD(gen2_phy);
	INIT_FIELD(phy_type);
	INIT_FIELD(adsync_en);
	INIT_FIELD(psr_ver);
	INIT_FIELD(scramble_dis);
	INIT_FIELD(enhance_framing_en);
	INIT_FIELD(default_fast_link_train_en);
	INIT_FIELD(scale_down_mode);
	INIT_FIELD(force_hpd);
	INIT_FIELD(disable_interleaving);
	INIT_FIELD(sel_aux_timeout_32ms);
	INIT_FIELD(debug_control);
	INIT_FIELD(sr_scale_down);
	INIT_FIELD(bs_512_scale_down);
	INIT_FIELD(enable_mst_mode);
	INIT_FIELD(enable_fec);
	INIT_FIELD(enable_edp);
	INIT_FIELD(initiate_mst_act_seq);
	INIT_FIELD(enhance_framing_with_fec_en);
	INIT_FIELD(controller_reset);
	INIT_FIELD(phy_soft_reset);
	INIT_FIELD(hdcp_module_reset);
	INIT_FIELD(audio_sampler_reset);
	INIT_FIELD(aux_reset);
	INIT_FIELD(video_reset);
	INIT_FIELD(audio_sampler_reset_stream1);
	INIT_FIELD(audio_sampler_reset_stream2);
	INIT_FIELD(audio_sampler_reset_stream3);
	INIT_FIELD(aux_cdr_state);
	INIT_FIELD(aux_cdr_clock_cycle);
	INIT_FIELD(video_stream_enable);
	INIT_FIELD(video_mapping_ipi_en);
	INIT_FIELD(video_mapping);
	INIT_FIELD(pixel_mode_select);
	INIT_FIELD(enable_dsc);
	INIT_FIELD(encryption_enable);
	INIT_FIELD(stream_type);
	INIT_FIELD(bcb_data_stuffing_en);
	INIT_FIELD(rcr_data_stuffing_en);
	INIT_FIELD(gy_data_stuffing_en);
	INIT_FIELD(bcb_stuff_data);
	INIT_FIELD(rcr_stuff_data);
	INIT_FIELD(gy_stuff_data);
	INIT_FIELD(vsync_in_polarity);
	INIT_FIELD(hsync_in_polarity);
	INIT_FIELD(de_in_polarity);
	INIT_FIELD(r_v_blank_in_osc);
	INIT_FIELD(i_p);
	INIT_FIELD(hblank_video_config1); /* hblank */
	INIT_FIELD(hactive_video_config1); /* hactive */
	INIT_FIELD(vactive_video_config2); /* vactive */
	INIT_FIELD(vblank_video_config2); /* vblank */
	INIT_FIELD(h_sync_width_video_config3); /* h_sync_width */
	INIT_FIELD(v_sync_width_video_config4); /* v_sync_width */
	INIT_FIELD(average_bytes_per_tu);
	INIT_FIELD(init_threshold);
	INIT_FIELD(average_bytes_per_tu_frac);
	INIT_FIELD(enable_3d_frame_field_seq);
	INIT_FIELD(init_threshold_hi);
	INIT_FIELD(hstart);
	INIT_FIELD(vstart);
	INIT_FIELD(mvid);
	INIT_FIELD(misc0);
	INIT_FIELD(nvid);
	INIT_FIELD(misc1);
	INIT_FIELD(hblank_interval);
	INIT_FIELD(mvid_cust_en);
	INIT_FIELD(mvid_out_clr_mode);
	INIT_FIELD(mvid_cust_den);
	INIT_FIELD(mvid_cust_mod);
	INIT_FIELD(mvid_cust_quo);
	INIT_FIELD(audio_inf_select);
	INIT_FIELD(audio_data_in_en);
	INIT_FIELD(audio_data_width);
	INIT_FIELD(hbr_mode_enable);
	INIT_FIELD(num_channels);
	INIT_FIELD(audio_mute);
	INIT_FIELD(audio_packet_id);
	INIT_FIELD(audio_timestamp_version_num);
	INIT_FIELD(audio_clk_mult_fs);
	INIT_FIELD(en_audio_timestamp_sdp_vertical_ctrl); /* en_audio_timestamp_sdp */
	INIT_FIELD(en_audio_stream_sdp_vertical_ctrl); /* en_audio_stream_sdp */
	INIT_FIELD(en_vertical_sdp_n);
	INIT_FIELD(en_128bytes_sdp_1);
	INIT_FIELD(disable_ext_sdp);
	INIT_FIELD(fixed_priority_arbitration_vertical_ctrl); /* fixed_priority_arbitration */
	INIT_FIELD(en_audio_timestamp_sdp_horizontal_ctrl); /* en_audio_timestamp_sdp */
	INIT_FIELD(en_audio_stream_sdp_horizontal_ctrl); /* en_audio_stream_sdp */
	INIT_FIELD(en_horizontal_sdp_n);
	INIT_FIELD(fixed_priority_arbitration_horizontal_ctrl); /* fixed_priority_arbitration */
	INIT_FIELD(audio_timestamp_sdp_status);
	INIT_FIELD(audio_stream_sdp_status);
	INIT_FIELD(sdp_n_tx_status);
	INIT_FIELD(manual_mode_sdp);
	INIT_FIELD(audio_timestamp_sdp_status_en);
	INIT_FIELD(audio_stream_sdp_status_en);
	INIT_FIELD(sdp_status_en);
	INIT_FIELD(sdp_16b_bytes_reqd_vblank_ovr);
	INIT_FIELD(sdp_16b_bytes_reqd_hblank_ovr);
	INIT_FIELD(sdp_32b_bytes_reqd_vblank_ovr);
	INIT_FIELD(sdp_32b_bytes_reqd_hblank_ovr);
	INIT_FIELD(sdp_128b_bytes_reqd_vblank_ovr);
	INIT_FIELD(sdp_128b_bytes_reqd_hblank_ovr);
	INIT_FIELD(tps_sel);
	INIT_FIELD(phyrate);
	INIT_FIELD(phy_lanes);
	INIT_FIELD(xmit_enable);
	INIT_FIELD(phy_busy);
	INIT_FIELD(ssc_dis);
	INIT_FIELD(phy_powerdown);
	INIT_FIELD(phy_width);
	INIT_FIELD(edp_phy_rate);
	INIT_FIELD(lane0_tx_preemp);
	INIT_FIELD(lane0_tx_vswing);
	INIT_FIELD(lane1_tx_preemp);
	INIT_FIELD(lane1_tx_vswing);
	INIT_FIELD(lane2_tx_preemp);
	INIT_FIELD(lane2_tx_vswing);
	INIT_FIELD(lane3_tx_preemp);
	INIT_FIELD(lane3_tx_vswing);
	INIT_FIELD(custom80b_0);
	INIT_FIELD(custom80b_1);
	INIT_FIELD(custom80b_2);
	INIT_FIELD(num_sr_zeros);
	INIT_FIELD(aux_len_req);
	INIT_FIELD(i2c_addr_only);
	INIT_FIELD(aux_addr);
	INIT_FIELD(aux_cmd_type);
	INIT_FIELD(aux_status);
	INIT_FIELD(aux_m);
	INIT_FIELD(aux_reply_received);
	INIT_FIELD(aux_timeout);
	INIT_FIELD(aux_reply_err);
	INIT_FIELD(aux_bytes_read);
	INIT_FIELD(sink_disconnect_while_active);
	INIT_FIELD(aux_reply_err_code);
	INIT_FIELD(aux_state);
	INIT_FIELD(aux_data0);
	INIT_FIELD(aux_data1);
	INIT_FIELD(aux_data2);
	INIT_FIELD(aux_data3);
	INIT_FIELD(aux_250us_cnt_limit);
	INIT_FIELD(aux_2000us_cnt_limit);
	INIT_FIELD(aux_100000us_cnt_limit);
	INIT_FIELD(tx0_in_generic_bus);
	INIT_FIELD(tx0_hp_prot_en);
	INIT_FIELD(tx0_bypass_eq_calc);
	INIT_FIELD(tx1_in_generic_bus);
	INIT_FIELD(tx1_hp_prot_en);
	INIT_FIELD(tx1_bypass_eq_calc);
	INIT_FIELD(tx2_in_generic_bus);
	INIT_FIELD(tx2_hp_prot_en);
	INIT_FIELD(tx2_bypass_eq_calc);
	INIT_FIELD(tx3_in_generic_bus);
	INIT_FIELD(tx3_hp_prot_en);
	INIT_FIELD(tx3_bypass_eq_calc);
	INIT_FIELD(tx0_out_generic_bus);
	INIT_FIELD(tx1_out_generic_bus);
	INIT_FIELD(tx2_out_generic_bus);
	INIT_FIELD(tx3_out_generic_bus);
	INIT_FIELD(combo_phy_ovr);
	INIT_FIELD(combo_phy_ovr_mpll_multiplier);
	INIT_FIELD(combo_phy_ovr_mpll_div_multiplier);
	INIT_FIELD(combo_phy_ovr_mpll_tx_clk_div);
	INIT_FIELD(combo_phy_ovr_mpll_ssc_freq_cnt_init);
	INIT_FIELD(combo_phy_ovr_mpll_ssc_freq_cnt_peak);
	INIT_FIELD(combo_phy_ovr_mpll_ssc_freq_cnt_ovrd_en);
	INIT_FIELD(combo_phy_ovr_mpll_div_clk_en);
	INIT_FIELD(combo_phy_ovr_mpll_word_div2_en);
	INIT_FIELD(combo_phy_ovr_mpll_init_cal_disable);
	INIT_FIELD(combo_phy_ovr_mpll_pmix_en);
	INIT_FIELD(combo_phy_ovr_mpll_v2i);
	INIT_FIELD(combo_phy_ovr_mpll_cp_int);
	INIT_FIELD(combo_phy_ovr_mpll_cp_prop);
	INIT_FIELD(combo_phy_ovr_mpll_ssc_up_spread);
	INIT_FIELD(combo_phy_ovr_mpll_ssc_peak);
	INIT_FIELD(combo_phy_ovr_mpll_ssc_stepsize);
	INIT_FIELD(combo_phy_ovr_mpll_fracn_cfg_update_en);
	INIT_FIELD(combo_phy_ovr_mpll_fracn_en);
	INIT_FIELD(combo_phy_ovr_mpll_fracn_den);
	INIT_FIELD(combo_phy_ovr_mpll_fracn_quot);
	INIT_FIELD(combo_phy_ovr_mpll_fracn_rem);
	INIT_FIELD(combo_phy_ovr_mpll_freq_vco);
	INIT_FIELD(combo_phy_ovr_ref_clk_mpll_div);
	INIT_FIELD(combo_phy_ovr_mpll_div5_clk_en);
	INIT_FIELD(combo_phy_ovr_tx0_term_ctrl);
	INIT_FIELD(combo_phy_ovr_tx1_term_ctrl);
	INIT_FIELD(combo_phy_ovr_tx2_term_ctrl);
	INIT_FIELD(combo_phy_ovr_tx3_term_ctrl);
	INIT_FIELD(combo_phy_ovr_tx_eq_g1);
	INIT_FIELD(combo_phy_ovr_tx_eq_main_g1);
	INIT_FIELD(combo_phy_ovr_tx_eq_post_g1);
	INIT_FIELD(combo_phy_ovr_tx_eq_pre_g1);
	INIT_FIELD(combo_phy_ovr_tx_eq_g2);
	INIT_FIELD(combo_phy_ovr_tx_eq_main_g2);
	INIT_FIELD(combo_phy_ovr_tx_eq_post_g2);
	INIT_FIELD(combo_phy_ovr_tx_eq_pre_g2);
	INIT_FIELD(combo_phy_ovr_tx_eq_g3);
	INIT_FIELD(combo_phy_ovr_tx_eq_main_g3);
	INIT_FIELD(combo_phy_ovr_tx_eq_post_g3);
	INIT_FIELD(combo_phy_ovr_tx_eq_pre_g3);
	INIT_FIELD(combo_phy_ovr_tx_eq_g4);
	INIT_FIELD(combo_phy_ovr_tx_eq_main_g4);
	INIT_FIELD(combo_phy_ovr_tx_eq_post_g4);
	INIT_FIELD(combo_phy_ovr_tx_eq_pre_g4);
	INIT_FIELD(combo_phy_ovr_tx_eq_g5);
	INIT_FIELD(combo_phy_ovr_tx_eq_main_g5);
	INIT_FIELD(combo_phy_ovr_tx_eq_post_g5);
	INIT_FIELD(combo_phy_ovr_tx_eq_pre_g5);
	INIT_FIELD(combo_phy_ovr_tx_eq_g6);
	INIT_FIELD(combo_phy_ovr_tx_eq_main_g6);
	INIT_FIELD(combo_phy_ovr_tx_eq_post_g6);
	INIT_FIELD(combo_phy_ovr_tx_eq_pre_g6);
	INIT_FIELD(combo_phy_ovr_tx_eq_g7);
	INIT_FIELD(combo_phy_ovr_tx_eq_main_g7);
	INIT_FIELD(combo_phy_ovr_tx_eq_post_g7);
	INIT_FIELD(combo_phy_ovr_tx_eq_pre_g7);
	INIT_FIELD(combo_phy_ovr_tx_eq_g8);
	INIT_FIELD(combo_phy_ovr_tx_eq_main_g8);
	INIT_FIELD(combo_phy_ovr_tx_eq_post_g8);
	INIT_FIELD(combo_phy_ovr_tx_eq_pre_g8);
	INIT_FIELD(combo_phy_ovr_tx0_en);
	INIT_FIELD(combo_phy_ovr_tx0_vboost_en);
	INIT_FIELD(combo_phy_ovr_tx0_iboost_lvl);
	INIT_FIELD(combo_phy_ovr_tx0_clk_rdy);
	INIT_FIELD(combo_phy_ovr_tx0_invert);
	INIT_FIELD(combo_phy_ovr_tx1_en);
	INIT_FIELD(combo_phy_ovr_tx1_vboost_en);
	INIT_FIELD(combo_phy_ovr_tx1_iboost_lvl);
	INIT_FIELD(combo_phy_ovr_tx1_clk_rdy);
	INIT_FIELD(combo_phy_ovr_tx1_invert);
	INIT_FIELD(combo_phy_ovr_tx2_en);
	INIT_FIELD(combo_phy_ovr_tx2_vboost_en);
	INIT_FIELD(combo_phy_ovr_tx2_iboost_lvl);
	INIT_FIELD(combo_phy_ovr_tx2_clk_rdy);
	INIT_FIELD(combo_phy_ovr_tx2_invert);
	INIT_FIELD(combo_phy_ovr_tx3_en);
	INIT_FIELD(combo_phy_ovr_tx3_vboost_en);
	INIT_FIELD(combo_phy_ovr_tx3_iboost_lvl);
	INIT_FIELD(combo_phy_ovr_tx3_clk_rdy);
	INIT_FIELD(combo_phy_ovr_tx3_invert);
	INIT_FIELD(hpd_event);
	INIT_FIELD(aux_reply_event);
	INIT_FIELD(hdcp_event);
	INIT_FIELD(aux_cmd_invalid);
	INIT_FIELD(sdp_event_stream0);
	INIT_FIELD(audio_fifo_overflow_stream0);
	INIT_FIELD(video_fifo_overflow_stream0);
	INIT_FIELD(video_fifo_underflow_stream0);
	INIT_FIELD(sdp_event_stream1);
	INIT_FIELD(audio_fifo_overflow_stream1);
	INIT_FIELD(video_fifo_overflow_stream1);
	INIT_FIELD(video_fifo_underflow_stream1);
	INIT_FIELD(sdp_event_stream2);
	INIT_FIELD(audio_fifo_overflow_stream2);
	INIT_FIELD(video_fifo_overflow_stream2);
	INIT_FIELD(video_fifo_underflow_stream2);
	INIT_FIELD(sdp_event_stream3);
	INIT_FIELD(audio_fifo_overflow_stream3);
	INIT_FIELD(video_fifo_overflow_stream3);
	INIT_FIELD(video_fifo_underflow_stream3);
	INIT_FIELD(dsc_event);
	INIT_FIELD(hpd_event_en);
	INIT_FIELD(aux_reply_event_en);
	INIT_FIELD(hdcp_event_en);
	INIT_FIELD(aux_cmd_invalid_en);
	INIT_FIELD(sdp_event_en_stream0);
	INIT_FIELD(audio_fifo_overflow_en_stream0);
	INIT_FIELD(video_fifo_overflow_en_stream0);
	INIT_FIELD(video_fifo_underflow_en_stream0);
	INIT_FIELD(sdp_event_en_stream1);
	INIT_FIELD(audio_fifo_overflow_en_stream1);
	INIT_FIELD(video_fifo_overflow_en_stream1);
	INIT_FIELD(video_fifo_underflow_en_stream1);
	INIT_FIELD(sdp_event_en_stream2);
	INIT_FIELD(audio_fifo_overflow_en_stream2);
	INIT_FIELD(video_fifo_overflow_en_stream2);
	INIT_FIELD(video_fifo_underflow_en_stream2);
	INIT_FIELD(sdp_event_en_stream3);
	INIT_FIELD(audio_fifo_overflow_en_stream3);
	INIT_FIELD(video_fifo_overflow_en_stream3);
	INIT_FIELD(video_fifo_underflow_en_stream3);
	INIT_FIELD(dsc_event_en);
	INIT_FIELD(hpd_irq);
	INIT_FIELD(hpd_hot_plug);
	INIT_FIELD(hpd_hot_unplug);
	INIT_FIELD(hpd_unplug_err);
	INIT_FIELD(hpd_status);
	INIT_FIELD(hpd_state);
	INIT_FIELD(hpd_timer);
	INIT_FIELD(hpd_irq_en);
	INIT_FIELD(hpd_plug_en);
	INIT_FIELD(hpd_unplug_en);
	INIT_FIELD(hpd_unplug_err_en);
	INIT_FIELD(enable_hdcp);
	INIT_FIELD(enable_hdcp_13);
	INIT_FIELD(encryptiondisable);
	INIT_FIELD(hdcp_lock);
	INIT_FIELD(bypencryption);
	INIT_FIELD(cp_irq);
	INIT_FIELD(dpcd12plus);
	INIT_FIELD(hdcpengaged);
	INIT_FIELD(substatea);
	INIT_FIELD(statea);
	INIT_FIELD(stater);
	INIT_FIELD(stateoeg);
	INIT_FIELD(statee);
	INIT_FIELD(hdcp_capable);
	INIT_FIELD(repeater);
	INIT_FIELD(hdcp13_bstatus);
	INIT_FIELD(hdcp2_booted);
	INIT_FIELD(hdcp2_state);
	INIT_FIELD(hdcp2_sink_cap_check_complete);
	INIT_FIELD(hdcp2_capable_sink);
	INIT_FIELD(hdcp2_authentication_success);
	INIT_FIELD(hdcp2_authentication_failed);
	INIT_FIELD(hdcp2_re_authentication_req);
	INIT_FIELD(ksvaccessint_clr); /* ksvaccessint */
	INIT_FIELD(auxrespdefer7times_clr); /* auxrespdefer7times */
	INIT_FIELD(auxresptimeout_clr); /* auxresptimeout */
	INIT_FIELD(auxrespnack7times_clr); /* auxrespnack7times */
	INIT_FIELD(ksvsha1calcdoneint_clr); /* ksvsha1calcdoneint */
	INIT_FIELD(hdcp_failed_clr); /* hdcp_failed */
	INIT_FIELD(hdcp_engaged_clr); /* hdcp_engaged */
	INIT_FIELD(hdcp2_gpioint_clr); /* hdcp2_gpioint */
	INIT_FIELD(ksvaccessint_stat); /* ksvaccessint */
	INIT_FIELD(auxrespdefer7times_stat); /* auxrespdefer7times */
	INIT_FIELD(auxresptimeout_stat); /* auxresptimeout */
	INIT_FIELD(auxrespnack7times_stat); /* auxrespnack7times */
	INIT_FIELD(ksvsha1calcdoneint_stat); /* ksvsha1calcdoneint */
	INIT_FIELD(hdcp_failed_stat); /* hdcp_failed */
	INIT_FIELD(hdcp_engaged_stat); /* hdcp_engaged */
	INIT_FIELD(hdcp2_gpioint_stat); /* hdcp2_gpioint */
	INIT_FIELD(ksvaccessint_msk); /* ksvaccessint */
	INIT_FIELD(auxrespdefer7times_msk); /* auxrespdefer7times */
	INIT_FIELD(auxresptimeout_msk); /* auxresptimeout */
	INIT_FIELD(auxrespnack7times_msk); /* auxrespnack7times */
	INIT_FIELD(ksvsha1calcdoneint_msk); /* ksvsha1calcdoneint */
	INIT_FIELD(hdcp_failed_msk); /* hdcp_failed */
	INIT_FIELD(hdcp_engaged_msk); /* hdcp_engaged */
	INIT_FIELD(hdcp2_gpioint_msk); /* hdcp2_gpioint */
	INIT_FIELD(ksvmemrequest);
	INIT_FIELD(ksvmemaccess);
	INIT_FIELD(ksvlistprocessupd);
	INIT_FIELD(ksvsha1swstatus);
	INIT_FIELD(ksvsha1status);
	INIT_FIELD(hdcpreg_bksv0);
	INIT_FIELD(hdcpreg_bksv1);
	INIT_FIELD(oanbypass);
	INIT_FIELD(hdcpreg_an0);
	INIT_FIELD(hdcpreg_an1);
	INIT_FIELD(odpk_decrypt_enable);
	INIT_FIELD(idpk_data_index);
	INIT_FIELD(idpk_wr_ok_sts);
	INIT_FIELD(hdcpreg_seed);
	INIT_FIELD(dpk_data_0); /* dpk_data */
	INIT_FIELD(dpk_data_1); /* dpk_data */
	INIT_FIELD(hdcp2gpiooutsts);
	INIT_FIELD(hdcp2gpiooutchngsts);
	INIT_FIELD(dpk_crc);
	INIT_FIELD(vg_swrst);
	INIT_FIELD(odepolarity);
	INIT_FIELD(ohsyncpolarity);
	INIT_FIELD(ovsyncpolarity);
	INIT_FIELD(oip);
	INIT_FIELD(ocolorincrement);
	INIT_FIELD(ovblankoscillation);
	INIT_FIELD(ycc_422_mapping);
	INIT_FIELD(ycc_pattern_generation);
	INIT_FIELD(pixel_repetition);
	INIT_FIELD(bits_per_comp);
	INIT_FIELD(ycc_420_mapping);
	INIT_FIELD(internal_external_gen);
	INIT_FIELD(pattern_mode);
	INIT_FIELD(hactive_vg_config2); /* hactive */
	INIT_FIELD(hblank_vg_config2); /* hblank */
	INIT_FIELD(h_front_porch);
	INIT_FIELD(h_sync_width_vg_config3); /* h_sync_width */
	INIT_FIELD(vactive_vg_config4); /* vactive */
	INIT_FIELD(vblank_vg_config4); /* vblank */
	INIT_FIELD(v_front_porch);
	INIT_FIELD(v_sync_width_vg_config5); /* v_sync_width */
	INIT_FIELD(td_structure);
	INIT_FIELD(td_enable);
	INIT_FIELD(td_frameseq);
	INIT_FIELD(ipi_enable);
	INIT_FIELD(ipi_select);
	INIT_FIELD(ram_addr_start);
	INIT_FIELD(start_write_ram);
	INIT_FIELD(write_ram_data);
	INIT_FIELD(ram_stop_addr);
	INIT_FIELD(vg_cb_width);
	INIT_FIELD(vg_cb_height);
	INIT_FIELD(vg_cb_colora_lsb);
	INIT_FIELD(vg_cb_color_a_msb);
	INIT_FIELD(vg_cb_color_b_lsb);
	INIT_FIELD(vg_cb_color_b_msb);
	INIT_FIELD(ag_swrst);
	INIT_FIELD(hbren);
	INIT_FIELD(audiosource_clockmultiplier);
	INIT_FIELD(i2s_wordwidth);
	INIT_FIELD(audio_source);
	INIT_FIELD(nlpcm_en);
	INIT_FIELD(spdiftxdata);
	INIT_FIELD(audio_use_lut);
	INIT_FIELD(audio_use_counter);
	INIT_FIELD(audio_counter_offset);
	INIT_FIELD(incleft);
	INIT_FIELD(incright);
	INIT_FIELD(iec_copyright);
	INIT_FIELD(iec_cgmsa);
	INIT_FIELD(iec_nlpcm);
	INIT_FIELD(iec_categorycode);
	INIT_FIELD(iec_sourcenumber);
	INIT_FIELD(iec_pcm_audio_mode);
	INIT_FIELD(iec_channelnumcl0_3); /* iec_channelnumcl0 */
	INIT_FIELD(iec_channelnumcr0_3); /* iec_channelnumcr0 */
	INIT_FIELD(iec_samp_freq);
	INIT_FIELD(iec_clkaccuracy);
	INIT_FIELD(iec_word_length);
	INIT_FIELD(iec_origsampfreq);
	INIT_FIELD(iec_channelnumcl0_5); /* iec_channelnumcl0 */
	INIT_FIELD(iec_channelnumcr0_5); /* iec_channelnumcr0 */
	INIT_FIELD(iec_channelnumcl1);
	INIT_FIELD(iec_channelnumcr1);
	INIT_FIELD(iec_channelnumcl2);
	INIT_FIELD(iec_channelnumcr2);
	INIT_FIELD(iec_channelnumcl2a);
	INIT_FIELD(iec_channelnumcr2a);
	INIT_FIELD(userdata_cl0);
	INIT_FIELD(userdata_cr0);
	INIT_FIELD(userdata_cl1);
	INIT_FIELD(userdata_cr1);
	INIT_FIELD(userdata_cl2);
	INIT_FIELD(userdata_cr2);
	INIT_FIELD(userdata_cl3);
	INIT_FIELD(userdata_cr3);
	INIT_FIELD(validity_bit_cl0);
	INIT_FIELD(validity_bit_cr0);
	INIT_FIELD(validity_bit_cl1);
	INIT_FIELD(validity_bit_cr1);
	INIT_FIELD(validity_bit_cl2);
	INIT_FIELD(validity_bit_cr2);
	INIT_FIELD(validity_bit_cl3);
	INIT_FIELD(validity_bit_cr3);

	return 0;
}