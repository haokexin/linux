// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include "mipi_dsi_hal.h"

#define dphy4txtester_DIG_testcode_x_reg 0x0
#define dphy4txtester_DIG_RDWR_TX_SYS_0  0x1
#define dphy4txtester_DIG_RDWR_TX_SYS_1  0x2
#define FREQUENCY_DIVISION               0x2
#define FCLKIN                           25
#define dphy4txtester_DIG_RDWR_TX_CB_0   0x1aa
#define dphy4txtester_DIG_RDWR_TX_SLEW_0     0x26b
#define dphy4txtester_DIG_RDWR_TX_SLEW_7     0x272
//Select value of cb_v400
#define dphy4txtester_DIG_RDWR_TX_CB_0_200mv 0xE3
#define dphy4txtester_DIG_RDWR_TX_CB_0_300mv 0xE7
#define dphy4txtester_DIG_RDWR_TX_CB_0_450mv 0xEf
struct dphy_parameter_map {
	unsigned int max_mbps;
	u8 hsfreqrange;
};

/* The table is based on 25MHz DPHY pll reference clock. */
static const struct dphy_parameter_map dppa_map[] = {
            {  99, 0b0010000},
            { 149, 0b0100000},
            { 199, 0b0110001},
            { 249, 0b0000011},
            { 299, 0b0110011},
            { 349, 0b0010100},
            { 399, 0b0110101},
            { 449, 0b0000101},
            { 499, 0b0010110},
            { 549, 0b0100110},
            { 599, 0b0110111},
            { 649, 0b0000111},
            { 699, 0b0011000},
            { 749, 0b0101000},
            { 799, 0b0111001},
            { 849, 0b0001001},
            { 899, 0b0011001},
            { 949, 0b0101001},
            { 999, 0b0111010},
            {1049, 0b0001010},
            {1099, 0b0011010},
            {1149, 0b0101010},
            {1199, 0b0111011},
            {1249, 0b0001011},
            {1299, 0b0011011},
            {1349, 0b0101011},
            {1399, 0b0111100},
            {1449, 0b0001100},
            {1499, 0b0011100},
            {1549, 0b0101100},
            {1599, 0b0111101},
            {1649, 0b0001101},
            {1699, 0b0011101},
            {1749, 0b0101110},
            {1799, 0b0111110},
            {1849, 0b0001110},
            {1899, 0b0011110},
            {1949, 0b0101111},
            {1999, 0b0111111},
            {2049, 0b0001111},
            {2099, 0b1000000},
            {2149, 0b1000001},
            {2199, 0b1000010},
            {2249, 0b1000011},
            {2299, 0b1000100},
            {2349, 0b1000101},
            {2399, 0b1000110},
            {2459, 0b1000111},
            {2499, 0b1001000},
            {2500, 0b1001001},
};

static int max_mbps_to_parameter(unsigned int max_mbps)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dppa_map); i++)
		if (dppa_map[i].max_mbps >= max_mbps)
			return i;

	return -EINVAL;
}

static int vco_caculate(int lane_mbps){
    int fout = lane_mbps/2;

    if(lane_mbps > 2500){
        DRM_ERROR("%s line:%d lane_mbps:%d!!",__FUNCTION__,__LINE__,lane_mbps);
        return -EINVAL;
    }

    if(fout >= 1000){
        return 0;
    } else if(fout >= 500){
        return 0x1;
    } else if(fout >= 250){
        return 0x2;
    } else if(fout >= 125){
        return 0x3;
    } else if(fout >= 62){
        return 0x4;
    }else{
        return 0x5;
    }
}
int cfg_dphy_signals(struct dw_mipi_dsi_bst *dsi, int dphy_sel, int lane_mbps)
{
    unsigned int wdata,lb_dsi_m = 0;
    unsigned int div_factor = 0;
    int vco = 0;
    unsigned int fout = lane_mbps/2;
    wdata = ((0x0A << 13) | (0x20 << 5) | (0x1 << 4) | (FREQUENCY_DIVISION-1));
    bst_dsi_csr_write(dsi, (0x10 * dphy_sel), wdata);
    vco = vco_caculate(lane_mbps);
    if(vco < 0){
        DRM_ERROR("%s line:%d vco:%d!!",__FUNCTION__,__LINE__,vco);
        return vco;
    }
    div_factor = 2 << vco;
    lb_dsi_m = fout*(2*div_factor*FREQUENCY_DIVISION)/FCLKIN;
    DRM_INFO("lane_mbps:%d lb_dsi_m:0x%x div_factor:%d vco:%d !!",lane_mbps,lb_dsi_m,div_factor,vco);
    wdata = ((lb_dsi_m << 18) | (vco << 15) |(0x7 << 12) | (0xA << 6) | (0x8));
    bst_dsi_csr_write(dsi, 0x04 + (0x10 * dphy_sel), wdata);
    wdata = ((0x1 << 5) | (0x0));
    bst_dsi_csr_write(dsi, 0x08 + (dphy_sel * 0x10), wdata);

    wdata = ((0x1 << 5)| (0x1 << 1) | (0x0));
    bst_dsi_csr_write(dsi, 0x08 + (dphy_sel * 0x10), wdata);

    wdata = ((0x1 << 5) | (0x0));
    bst_dsi_csr_write(dsi, 0x08 + (dphy_sel * 0x10), wdata);

    wdata = ((0x11 << 4) | (0x0));
    bst_dsi_csr_write(dsi, 0x08 + (dphy_sel * 0x10), wdata);

    wdata = ((0x1 << 5) | (0x0));
    bst_dsi_csr_write(dsi, 0x08 + (dphy_sel * 0x10), wdata);

    return 0;
}

// static int dphy_write_data(struct dw_mipi_dsi_bst *dsi,uint16_t dphy_addr, uint8_t wdata)
// {
//     uint8_t high_addr = dphy_addr >> 8;
//     uint8_t low_addr = dphy_addr & 0xff;
//     uint32_t ret_val=0;
//     DRM_INFO("high_addr:0x%x low_addr:0x%x\n",high_addr,low_addr);
//     bst_dsi_write(dsi, DSI_PHY_TST_CTRL0, 0);
//     bst_dsi_write(dsi, DSI_PHY_TST_CTRL1, 0x10000);
//     bst_dsi_write(dsi, DSI_PHY_TST_CTRL0, 2);
//     bst_dsi_write(dsi, DSI_PHY_TST_CTRL0, 0);
//     bst_dsi_write(dsi, DSI_PHY_TST_CTRL1, 0);
//     bst_dsi_write(dsi, DSI_PHY_TST_CTRL1, high_addr);
//     bst_dsi_write(dsi, DSI_PHY_TST_CTRL0, 2);

//     bst_dsi_write(dsi, DSI_PHY_TST_CTRL0, 0);
//     bst_dsi_write(dsi, DSI_PHY_TST_CTRL1,(0x1 << 16) | low_addr);
//     bst_dsi_write(dsi, DSI_PHY_TST_CTRL0, 2);
//     bst_dsi_write(dsi, DSI_PHY_TST_CTRL0, 0);
//     bst_dsi_write(dsi, DSI_PHY_TST_CTRL1, wdata);
//     bst_dsi_write(dsi, DSI_PHY_TST_CTRL0, 2);
//     bst_dsi_write(dsi, DSI_PHY_TST_CTRL0, 0);

//     ret_val = bst_dsi_read(dsi, DSI_PHY_TST_CTRL1);
//     DRM_INFO("val:0x%x dphy_addr:0x%x\n",ret_val,dphy_addr);
//     return 0;
// }

int dphy_write_control(struct dw_mipi_dsi_bst *dsi, uint8_t testcode, uint8_t testwrite)
{
    bst_dsi_write(dsi, DSI_PHY_TST_CTRL1, (0x000100 << 8) | testcode);
    bst_dsi_write(dsi, DSI_PHY_TST_CTRL0, 2);
    bst_dsi_write(dsi, DSI_PHY_TST_CTRL0, 0);
    bst_dsi_write(dsi, DSI_PHY_TST_CTRL1, testwrite);
    bst_dsi_write(dsi, DSI_PHY_TST_CTRL0, 2);
    bst_dsi_write(dsi, DSI_PHY_TST_CTRL0, 0);
    return 0;
}

int dphy_rate_swtch(struct dw_mipi_dsi_bst *dsi, int lane_mbps)
{
    int ret = 0, i = 0;
    ret = dphy_write_control(dsi,dphy4txtester_DIG_testcode_x_reg,0);
    ret = dphy_write_control(dsi,dphy4txtester_DIG_RDWR_TX_SYS_0,0x20);

	i = max_mbps_to_parameter(lane_mbps);
	if (i < 0) {
		DRM_ERROR("failed to get parameter for %dmbps clock\n",lane_mbps);
		return i;
	}

    dphy_write_control(dsi,dphy4txtester_DIG_RDWR_TX_SYS_1,
        dppa_map[i].hsfreqrange);
    //dphy_write_data(dsi,dphy4txtester_DIG_RDWR_TX_CB_0,dphy4txtester_DIG_RDWR_TX_CB_0_450mv);
    return ret;
}
