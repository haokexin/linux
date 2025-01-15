// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include "mipi_dsi_hal.h"

void bst_dsi_write(struct dw_mipi_dsi_bst *dsi, u32 reg, u32 val)
{
	if(!dsi)
	{
		DRM_ERROR("dsi is null!!\n");
		return ;
	}
	writel(val, dsi->dsi_base + reg);
}

void bst_dsi_set(struct dw_mipi_dsi_bst *dsi, u32 reg, u32 mask)
{
	if(!dsi)
	{
		DRM_ERROR("dsi is null!!\n");
		return ;
	}
	writel(bst_dsi_read(dsi, reg) | mask, dsi->dsi_base + reg);
}

u32 bst_dsi_read(struct dw_mipi_dsi_bst *dsi, u32 reg)
{
	if(!dsi)
	{
		DRM_ERROR("dsi is null!!\n");
		return -1;
	}
	return readl(dsi->dsi_base + reg);
}

u32 bst_mipi_csr_read(struct dw_mipi_dsi_bst *dsi, u32 reg)
{
	u32 val=0,ret=0;
	if(!dsi)
	{
		DRM_ERROR("dsi is null!!\n");
		return -1;
	}
	ret = regmap_read(dsi->csr_regmap, reg, &val);
	if (ret)
		DRM_ERROR("failed to read: %d\n", ret);
	return val;
}
/**
 * read and set the reg
 * @param dsi pointer to structure holding the DSI Host core information
 * @param reg reiser offset
 * @param mask which bit you want to change
 */
void bst_mipi_csr_set(struct dw_mipi_dsi_bst *dsi, unsigned int reg, unsigned int mask)
{
	if(!dsi)
	{
		DRM_ERROR("dsi is null!!\n");
		return ;
	}
	regmap_write(dsi->csr_regmap, reg,bst_mipi_csr_read(dsi, reg) | mask);
}
/**
 * write the reg,ignore read process
 * @param dsi pointer to structure holding the DSI Host core information
 * @param reg reiser offset
 * @param mask which bit you want to change
 */
void bst_dsi_csr_write(struct dw_mipi_dsi_bst *dsi,u32 reg, u32 val)
{
	int ret =0;
	if(!dsi)
	{
		DRM_ERROR("dsi is null!!\n");
		return ;
	}
	ret = regmap_write(dsi->csr_regmap,reg,val);
	if (ret)
		DRM_ERROR("csr write error!\n");
}

/**
 * Write a bit field o a 32-bit word to the DSI Host core
 * @param dev pointer to structure holding the DSI Host core information
 * @param reg_address register offset in core
 * @param data to be written to register
 * @param shift bit shift from the left (system is BIG ENDIAN)
 * @param width of bit field
 */
void mipi_dsih_write_part(struct dw_mipi_dsi_bst *dsi, u32 reg_address, u32 data, unsigned char shift, unsigned char width)
{
	u32 mask = (1 << width) - 1;
	u32 temp = bst_dsi_read(dsi, reg_address);

	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;
	bst_dsi_write(dsi, reg_address, temp);
}

/**
 * remap dsi format en
 * @param dsi pointer to structure holding the DSI Host core information
 * @return none
 */
void mipi_dsi_channel(struct dw_mipi_dsi_bst *dsi,u8 dsi_id){
	u32 val=0,val_eco=0;
	if(dsi->format==MIPI_DSI_FMT_RGB888){
		/* rgb24 */
		val = dsi_id ? BIT(13) : BIT(15);
	}
	val_eco = dsi_id ? DSI1_DATA_VID_EN_ECO : DSI0_DATA_VID_EN_ECO;
	bst_mipi_csr_set(dsi, DISPLAY2MIPI_MAP, val);
	bst_mipi_csr_set(dsi, DSICSITX_RESERVD3_ECO, val_eco);
}

/**
 * reset mipi dsi
 * @param dsi pointer to structure holding the DSI Host core information
 * @return none
 */
void bst_mipi_dsi_reset(struct dw_mipi_dsi_bst *dsi)
{
	u32 value = LOCAL_CSITX_PRESETN | LOCAL_DSI1_PRESETN | LOCAL_DSI0_PRESETN | LOCAL_DSI_CSITX_HRESETN | \
	LOCAL_DSI_CSITX_DMAC_RST_N | LOCAL_DSI1_DPIPCLK_MUX_RSTN | LOCAL_DSI0_DPIPCLK_MUX_RSTN | LOCAL_ISP_RST_N | CSITX_ALL_RST_N;
	bst_mipi_csr_set(dsi,DSI_CSITX_CLK_SEL, value);
}
