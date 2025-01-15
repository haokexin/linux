// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef __DPTX_CSR_H__
#define __DPTX_CSR_H__

void dptx_csr_reset(struct dptx *dptx);
void dptx_csr_func_irq_dis_all(struct dptx *dptx);
void dptx_csr_func_irq_en_dptx(struct dptx *dptx);
void dptx_csr_func_irq_en_trng(struct dptx *dptx);
void dptx_csr_func_irq_en_kpf(struct dptx *dptx);
void dptx_csr_func_irq_en_tca(struct dptx *dptx);
void dptx_csr_func_irq_en_aux_timeout(struct dptx *dptx);
void dptx_csr_func_irq_en_hdcp_hpi(struct dptx *dptx);
void dptx_csr_force_hpd(struct dptx *dptx, bool force) ;
int32_t dptx_csr_set_video_ctrl(struct dptx *dptx);
int dptx_init_remote_source(struct dptx *dptx);
#endif