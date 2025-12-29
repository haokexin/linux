// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#include <linux/io.h>

#include "bst_drm_dev.h"
#include "bst_md_csr.h"


static inline void __iomem * dev_to_csr_base(struct device *dev)
{
    struct bst_dev *mdev = dev_to_mdev(dev);
    return mdev->csr_base;
}

u32 bst_md_csr_read(struct device *dev, u32 reg)
{
    void __iomem *csr_base = dev_to_csr_base(dev);
    return readl(csr_base + reg);
}

void bst_md_csr_write(struct device *dev, u32 reg, u32 val)
{
    void __iomem *csr_base = dev_to_csr_base(dev);
    writel(val, csr_base + reg);
}

static inline u32 build_mux_sel_value(u32 disp_id, u32 pipe_id, u32 link_id)
{
    return (disp_id << 2) | (pipe_id << 1) | (link_id);
}

static void bst_disp_mux_sel(struct device *dev, u32 reg, u32 disp_id, u32 pipeline_id, u32 link_id)
{
    void __iomem *csr_base = dev_to_csr_base(dev);
    u32 selected;
    u32 val;

    selected = build_mux_sel_value(disp_id, pipeline_id, link_id);

    val = selected << MD_CSR_DISP_SEL_SHIFT;
    writel(val, csr_base + reg);
}

void bst_disp_vout_mux_sel(struct device *dev, u32 disp_id, u32 pipeline_id, u32 link_id)
{
    return bst_disp_mux_sel(dev, MD_CSR_VOUT_CFG, disp_id, pipeline_id, link_id);
}

void bst_disp_edp_mux_sel(struct device *dev, u32 disp_id, u32 pipeline_id, u32 link_id)
{
    return bst_disp_mux_sel(dev, MD_CSR_EDP_CFG, disp_id, pipeline_id, link_id);
}

void bst_disp_lvds_mux_sel(struct device *dev, u32 lvds_n, u32 disp_id, u32 pipeline_id, u32 link_id)
{
    u32 reg = MD_CSR_LVDS0_CFG + lvds_n*0x4;
    return bst_disp_mux_sel(dev, reg, disp_id, pipeline_id, link_id);
}

void bst_disp_dsi_mux_sel(struct device *dev, u32 dsi_n, u32 disp_id, u32 pipeline_id, u32 link_id)
{
    u32 reg = MD_CSR_DSI0_CFG + dsi_n*0x4;
    return bst_disp_mux_sel(dev, reg, disp_id, pipeline_id, link_id);
}
