// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/delay.h>
#include <linux/phylink.h>
#include <linux/workqueue.h>
#include <linux/mdio.h>
#include <linux/pcs/pcs-bst.h>

#define XPCS_NUM                1
#define PUBLIC_CONFIG_XPCS      0
#define XPCS_START		0
#define XPCS_END		1
#define XPCS_PHYID_START	0
#define XPCS_PHYID_END		1
#define XPCS_PHYID0		0
#define XPCS_PHYID1		1

#undef DEBUG
#ifdef DEBUG
#define print_dbg(fmt, args...)					\
	printk(fmt, ##args)
#else
#define print_dbg(fmt, args...)					\
do {									\
	if (0)								\
		printk(fmt, ##args); \
} while (0)
#endif

void __iomem *pcs_addr;

static u16 xpcs_read_indirect(u16 phyid, u16 pcs, u32 reg)
{
	//phyid: 0-1 pcs:0-1
	void __iomem *base_addr = pcs_addr + (phyid * 0x2000) + (0x1000 * pcs);
	void __iomem *addr = base_addr + 0x3fc;
	u64 offset_addr, data;

	offset_addr = (reg >> 10) & 0x1fff; //reg[22:10]
	writel(offset_addr, addr);
	wmb();
print_dbg("%s line %d write reg 0x%x addr 0x%llx offset 0x%llx\n", __func__, __LINE__, reg, (u64)addr, offset_addr);
	udelay(10);
	offset_addr = reg & 0x3ff; //reg[9:0]
	data = readl(base_addr + offset_addr);
print_dbg("%s line %d read data 0x%llx base_addr 0x%llx offset 0x%llx\n", __func__, __LINE__, data, (u64)base_addr, offset_addr);
	rmb();
	return (data & 0xffff);
}

static void xpcs_write_indirect(u16 phyid, u16 pcs, u32 reg, u16 data)
{
	//phyid: 0-1 pcs:0-1
	void __iomem *base_addr = pcs_addr + (phyid * 0x2000) + (0x1000 * pcs);
	void __iomem *addr = base_addr + 0x3fc;
	u64 offset_addr;

	offset_addr = (reg >> 10) & 0x1fff; //reg[22:10]
	writel(offset_addr, addr);
	wmb();
print_dbg("%s line %d write reg 0x%x addr 0x%llx offset 0x%llx\n", __func__, __LINE__, reg, (u64)addr, offset_addr);
	udelay(10);

	offset_addr = reg & 0x3ff; //reg[9:0]
	writel(data, base_addr + offset_addr);
	wmb();
	udelay(10);
	/*print_dbg("%s line %d write data 0x%x base_addr 0x%x offset 0x%x\n", __func__, __LINE__, data, base_addr, offset_addr);
	tmp = xpcs_read_indirect(phyid, pcs, reg);
	print_dbg("%s line %d **** wr reg 0x%x  rd data 0x%x\n", __func__, __LINE__, reg, tmp);*/
}

static int bstmac_sbd_int = 0;
static void bstmac_xpcs_phy_sgmii_per_pcs(u16 phyid, u16 pcs, int speed)
{
    u16 data;
	int i;

    	/*1.In Backplane Ethernet PCS configurations, program bit [12] (AN_EN) of 
	SR_AN_CTRL Register to 0 and bit [12] (CL37_BP) of 
	VR_XS_PCS_DIG_CTRL1 Register to 1. (SKIP pcs-mode:10g-base-r)*/
	//2:Disable Clause 37 auto-negotiation by programming bit [12] (AN_ENABLE) of SR_MII_CTRL Register to 0 (in case it is already enabled).
	data = xpcs_read_indirect(phyid, pcs, 0x7c0000);
	data &= (~((1 << 12) & 0xffff));
	xpcs_write_indirect(phyid, pcs, 0x7c0000, data);
	udelay(30);

	/* 3.program various fields of VR_MII_AN_CTRL Register appropriately as follows:
	 * Program PCS_MODE to 2’b10
         * Program TX_CONFIG to 1 (PHY side SGMII) or 0 (MAC side SGMII) based on your requirement
	 * Program MII_AN_INTR_EN to 1, to enable auto-negotiation complete interrupt
	 * If TX_CONFIG is set to 1 and bit [0] of VR_MII_DIG_CTRL1 Register is set to 0, 
	 * program SGMII_LINK_STS to indicate the link status to the MAC side SGMII.
	 * Program MII_CTRL to 0 or 1, as per your requirement.
	 */
	data = xpcs_read_indirect(phyid, pcs, 0x7e0004);
	data &= (~((1 << 3) | (1 << 0) | (3 << 1)) & 0xffff); //TX_CONFIG :0 MAC side
	//if (bstmac_sbd_int)
		data |= (1 << 0) & 0xffff;

	data |= ((0x2 << 1) & 0xffff); //10:SGMII
	data &= (~(1 << 8) & 0xffff); //4bit MII
	//data |= ((1 << 8) & 0xffff); //8bit MII

	xpcs_write_indirect(phyid, pcs, 0x7e0004, data);
         
	//6 and 9 about auto-negotiation
    	/*6.If DWC_xpcs is configured as MAC-side SGMII in step 11, program bit[9] of 
	 *VR_MII_DIG_CTRL1 Register to 1, for DWC_xpcs to automatically
	 */
	data = xpcs_read_indirect(phyid, pcs, 0x7e0000);
	data |= ((1 << 9) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0x7e0000, data);

	if (speed == 2500) {
		/*7 (Required only for 2.5G SGMII) Program VR_MII_LINK_TIMER_CTRL to 16'h07A1 
		*so that link timer runs for 1.6ms
		*/
		data = xpcs_read_indirect(phyid, pcs, 0x7e0028);
		data = 0x7a1;
		xpcs_write_indirect(phyid, pcs, 0x7e0028, data);
		/*8 (Required only for 2.5G SGMII) Program bit [3] (CL37_TMR_OVR_RIDE) of 
		*VR_MII_DIG_CTRL1 Register to 1
		*/
		data = xpcs_read_indirect(phyid, pcs, 0x7e0000);
		data |= ((1 << 3) & 0xffff);
		xpcs_write_indirect(phyid, pcs, 0x7e0000, data);
	}

	//9.Enable CL37 Auto-negotiation, by programming bit [12] of the SR_MII_CTRL Register to 1.
	data = xpcs_read_indirect(phyid, pcs, 0x7c0000);
	data |= ((1 << 12) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0x7c0000, data);
	if (bstmac_sbd_int) {
		i = 100;
		while (i--) {
			if (!bstmac_sbd_int)
				break;
			udelay(10);
		}
		if (0 >= i)
			print_dbg("%s line %d sbd int timeout\n", __func__, __LINE__);
		else {
			data = xpcs_read_indirect(phyid, pcs, 0x7e0008); //clear int
			data &= (~(1 << 0) & 0xffff);
			xpcs_write_indirect(phyid, pcs, 0x7e0008, data);
		}
	} else 
		mdelay(10);
}

#define USXGMII_AUTO_AN_10M	((~(1 << 5)) | (~(1 << 6)) | (~(1 << 13)))
#define USXGMII_AUTO_AN_100M 	((~(1 << 5)) | (~(1 << 6)) | (1 << 13))
#define USXGMII_AUTO_AN_1000M 	((~(1 << 5)) | (1 << 6) | (~(1 << 13)))
#define USXGMII_AUTO_AN_10G 	((~(1 << 5)) | (1 << 6) | (1 << 13))
#define USXGMII_AUTO_AN_2P5G 	((1 << 5) | (~(1 << 6)) | (~(1 << 13)))
#define USXGMII_AUTO_AN_5G 	((1 << 5) | (~(1 << 6)) | (1 << 13))

#if 0
static int bstmac_xpcs_usxgmii_recfg(u16 phyid, u16 pcs, int speed)
{
	u16 data, link, duplex, link_speed, eee, eee_clk;
	int i;

	//7.Program bit [4] (SUPRESS_LOS_DET) and bit [6] (RX_DT_EN_CTL) of VR_XS_PCS_DEBUG_CTRL register to 1 (as LOS from PHY can be unreliable)
	data = xpcs_read_indirect(phyid, pcs, 0xe0014);
	data |= (((1 << 4) | (1 << 6)) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0xe0014, data);
	//8.Poll for RX_valID_0 bit of VR_XS_PMA_RX_LSTS register to indicate 1
	udelay(10);
	i = 100;
	while (i--) {
		data = xpcs_read_indirect(phyid, pcs, 0x60080);
		if (data & ((1 << 12) & 0xffff))
			break;
		udelay(10);
	}
	if (!i)
		return -3;
	//9.Program RX_AD_REQ bit of VR_XS_PMA_MP_12G_16G_25G_RX_EQ_CTRL4 register to 1.
	data = xpcs_read_indirect(phyid, pcs, 0x60170);
	data |= (1 << 12) & 0xffff;
	xpcs_write_indirect(phyid, pcs, 0x60170, data);
	//10.Poll for RX_ADPT_ACK bit of VR_XS_PMA_MP_12G_16G_25G_MISC_STS register to indicate 1.
	udelay(10);
	i = 100;
	while (i--) {
		data = xpcs_read_indirect(phyid, pcs, 0x60260);
		if (data & ((1 << 12) & 0xffff))
			break;
		udelay(10);
	}
	if (!i)
		return -4;
	//11.Program RX_AD_REQ bit of VR_XS_PMA_MP_12G_16G_25G_RX_EQ_CTRL4 register to 0
	data = xpcs_read_indirect(phyid, pcs, 0x60170);
	data &= (~((1 << 12) & 0xffff));
	xpcs_write_indirect(phyid, pcs, 0x60170, data);
	/* 12.Program various bits of VR_MII_AN_CTRL Register as follows: 
	 * MII_AN_INTR_EN to 1, to enable auto-negotiation complete interrupt (optional step). 
	 * MII_CTRL to 0 or 1, based on your MAC capability. 
	 * TX_CONFIG to 1 (PHY side USXGMII) or 0 (MAC side USXGMII) based on your requirement.
	 * If TX_CONFIG is set to 1, program SGMII_LINK_STS bit to a suitable 
	 * value to indicate the link status to the MAC side of USXGMII link.
	 */
	data = xpcs_read_indirect(phyid, pcs, 0x7e0004);
	data &= (~(1 << 8) & 0xffff); //4bit MII
	data &=(~(1 << 3) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0x7e0004, data);

	/*13.(Optional step) Duration of link timer can be changed (default setting corresponds to 1.6ms) by programming 
	 *VR_MII_LINK_TIMER_CTRL Register suitably and by setting bit [3] of VR_MII_DIG_CTRL1 Register to 1.
         */

	/*15.Enable Clause 37 auto-negotiation by programming bit [12] of SR_MII_CTRL Register to 1. 
	 *(If interrupt has been enabled) After the completion of auto-negotiation, 
	 *DWC_xpcs generates an interrupt on sbd_intr_o port
         */
        data = xpcs_read_indirect(phyid, pcs, 0x7c0000);
	data |= ((1 << 12) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0x7c0000, data);
	udelay(10);
	
	/*16.Read the auto-negotiation status register (VR_MII_AN_INTR_STS Register). Bit[0] is set to indicate 
	 *that auto-negotiation is complete. Bits [14:8] indicate the link-speed, duplex mode, EEE capability 
	 *and EEE clock-stop capability indicated by the link partner (PHY chip).
	 */
        data = xpcs_read_indirect(phyid, pcs, 0x7e0008);
	i = 100;
	while (i--) {
		if (data & 0x1)
			break;
		udelay(10);
		data = xpcs_read_indirect(phyid, pcs, 0x7e0008);
	}
	if (!i)
		return -5;
	eee_clk = (data >> 8) & 0x1;
	eee = (data >> 9) & 0x1;
	link_speed = (data >> 10) & 0x7;
	duplex = (data >> 13) & 0x1;
	link = (data >> 14) & 0x1;
	
	//17.Clear the Interrupt by writing 0 to bit [0] of VR_MII_AN_INTR_STS Register.
	data &= (~(1 << 0) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0x7e0008, data);
	
	/*18.Program SS13, SS6 and SS5 bits of SR_MII_CTRL Register to configure DWC_xpcs to the USXGMII speed 
	 *mode (for port 0) indicated by PHY in step 16. The values programmed to these bits will reflect in the output port 
	 *xpcs_usxg_speed_o. This step is required only if DWC_xpcs is configured as MAC-side USXGMII.
	 */
        data = xpcs_read_indirect(phyid, pcs, 0x7c0000);
	switch (link_speed) {
	case 0:
		data |= USXGMII_AUTO_AN_10M;
		break;
	case 1:
		data |= USXGMII_AUTO_AN_100M;
		break;
	case 2:
		data |= USXGMII_AUTO_AN_1000M;
		break;
	case 3:
		data |= USXGMII_AUTO_AN_10G;
		break;
	case 4:
		data |= USXGMII_AUTO_AN_2P5G;
		break;
	case 5:
		data |= USXGMII_AUTO_AN_5G;
		break;
	default:
		data |= USXGMII_AUTO_AN_10M;
		break;
	}
	xpcs_write_indirect(phyid, pcs, 0x7c0000, data);
	
	/*19.Wait for some time (say 1 microsecond) so that XGMII clocks (clk_xgmii_tx_i/clk_xgmii_rx_i) get stabilized 
	 *at the desired frequencies.
         */
        mdelay(1);

	//20.Program USRA_RST bit (bit [10]) of VR_XS_PCS_DIG_CTRL1 Register to 1 and wait for it to get self-cleared.
	data = xpcs_read_indirect(phyid, pcs, 0xe0000);
	data |= ((1 <<10) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0xe0000, data);
	udelay(10);
	i = 100;
	while (i--) {
		data = xpcs_read_indirect(phyid, pcs, 0xe0000);
		if (!(data & ((1 << 10) & 0xffff)))
			break;
		udelay(10);
	}
	if (!i) {
		return -6;
	}

        return 0;
}
#endif

static int xpcs_link_up(struct mdio_xpcs_args *xpcs, int speed,
			phy_interface_t interface)
{
#if 0
        if (interface == PHY_INTERFACE_MODE_SGMII)
	        bstmac_xpcs_phy_sgmii_per_pcs(XPCS_PHYID0, PUBLIC_CONFIG_XPCS, 1000);
        
        if (interface == PHY_INTERFACE_MODE_XGMII)
	        bstmac_xpcs_usxgmii_recfg(XPCS_PHYID0, 0, speed);//step7-20	
#endif
	return 0;
}

#define USXGMII_INIT_10G	(10000)
#define USXGMII_INIT_5G		(5000)

static void bstmac_xpcs_10g_baser_phy(u16 phyid, u16 pcs, u16 init_speed)
{
	u16 data;

	//VR_XS_PMA_MP_12G_16G_25G_MPLL_CMN_CTRL MPLLB_SEL_0=0
	data = xpcs_read_indirect(phyid, pcs, 0x601c0);
	data &= (~(1 << 4) & 0xffff); 
	xpcs_write_indirect(phyid, pcs, 0x601c0, data);
	//VR_XS_PMA_MP_12G_16G_25G_REF_CLK_CTRL REF_RANGE=6 REF_CLK_DIV2=0 REF_MPLLA_DIV=1
	data = xpcs_read_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x60244);
	data &= (~(1 << 2) & 0xffff);
	data |= (((1 << 10) | (6 << 3)) & 0xffff);
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x60244, data);
	//VR_XS_PMA_MP_25G_MPLLA_CTRL0 MPLLA_W_CLK_DIV=3 MPLLA_MULTIPLIER=8’d132
	data = xpcs_read_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601c4);
	data |= (((3 << 12) | (132 << 0)) & 0xffff);
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601c4, data);
	//VR_XS_PMA_MP_25G_MPLLA_CTRL1 FRAC_EN=0 FB_DIV4_EN=1 BW_TH=2’d3
	data = xpcs_read_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601c8);
	data &= (~(1 << 8) & 0xffff);
	data |= (((1 << 10) | (3 << 12)) & 0xffff);
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601c8, data);
	//VR_XS_PMA_MP_25G_MPLLA_CTRL2 MPLLA_DIV16P5_CLK_EN=1 MPLLA_TX_CLK_DIV=1
	data = xpcs_read_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601cc);
	data |= (((1 << 8) | (1 << 11)) & 0xffff);
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601cc, data);
	//VR_XS_PMA_MP_25G_MPLLA_CTRL3 MPLLA_FRACN_QUOT=0
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601dc, 0);
	//VR_XS_PMA_MP_25G_MPLLA_CTRL4 MPLLA_FRACN_REM=0
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601e4, 0);
	//VR_XS_PMA_MP_25G_MPLLA_CTRL5 MPLLA_FRACN_DEN=0
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601ec, 0);
	//VR_XS_PMA_MP_25G_MPLLA_BW_LO_CTRL MPLLA_BW_LO=16’d22
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601f8, 22);
	//VR_XS_PMA_MP_25G_MPLLA_BW_HI_CTRL MPLLA_BW_HI=16’d22
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601f4, 22);
	//VR_XS_PMA_MP_12G_16G_25G_VCO_CAL_LD0 VCO_LD_val_0=13’d1386
	data = xpcs_read_indirect(phyid, pcs, 0x60248);
	data |= (1386 << 0) & 0xffff;
	xpcs_write_indirect(phyid, pcs, 0x60248, data);
	//VR_XS_PMA_MP_16G_25G_VCO_CAL_REF0 VCO_REF_LD_0=7’d21
	data = xpcs_read_indirect(phyid, pcs, 0x60258);
	data |= (21 << 0) & 0xffff;
	xpcs_write_indirect(phyid, pcs, 0x60258, data);
	//VR_XS_PMA_MP_12G_16G_25G_MISC_CTRL0 RX_VREF_CTRL=5
	data = xpcs_read_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x60240);
	data |= (5 << 8) & 0Xffff;
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x60240, data);
	//VR_XS_PMA_MP_16G_25G_MISC_CTRL2 SUP_MISC=0
	data = xpcs_read_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x60270);
	data &= (~(0xff << 0) & 0Xffff);
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x60270, data);
	//VR_XS_PMA_MP_12G_16G_25G_TX_GENCTRL1 VBOOST_LVL=5 VBOOST_EN_0=1
	data = xpcs_read_indirect(phyid, pcs, 0x600c4);
	data |= (((5 << 8) | (1 << 4))& 0Xffff);
	xpcs_write_indirect(phyid, pcs, 0x600c4, data);
	//VR_XS_PMA_MP_12G_16G_25G_RX_GENCTRL1 RX_DIV16P5_CLK_EN_0=1
	data = xpcs_read_indirect(phyid, pcs, 0x60144);
	data &= ((1 << 12) & 0Xffff);
	xpcs_write_indirect(phyid, pcs, 0x60144, data);
	//VR_XS_PMA_MP_12G_16G_25G_TX_RATE_CTRL TX0_RATE=1 for 10 / TX0_RATE=2 for 5
	data = xpcs_read_indirect(phyid, pcs, 0x600d0);
	if (init_speed == USXGMII_INIT_10G)
		data |= 1;
	else
		data |= 2;
	xpcs_write_indirect(phyid, pcs, 0x600d0, data);
	//VR_XS_PMA_MP_12G_16G_25G_RX_RATE_CTRL RX0_RATE=1 for 10 / RX0_RATE=2 for 5
	data = xpcs_read_indirect(phyid, pcs, 0x60150);
	if (init_speed == USXGMII_INIT_10G)
		data |= 1;
	else
		data |= 2;
	xpcs_write_indirect(phyid, pcs, 0x60150, data);
	//VR_XS_PMA_MP_25G_TX_WIDTH_CTRL TX0_WIDTH=3’b011 (20-bit)
	data = xpcs_read_indirect(phyid, pcs, 0x60118);
	data |= 3;
	xpcs_write_indirect(phyid, pcs, 0x60118, data);
	//VR_XS_PMA_MP_25G_RX_WIDTH_CTRL RX0_WIDTH=3’b011 (20-bit)
	data = xpcs_read_indirect(phyid, pcs, 0x602c0);
	data |= 3;
	xpcs_write_indirect(phyid, pcs, 0x602c0, data);
	//VR_XS_PMA_MP_16G_25G_RX_EQ_CTRL5 RX0_ADPT_MODE=2’d3
	data = xpcs_read_indirect(phyid, pcs, 0x60174);
	data |= ((3 << 4) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0x60174, data);
	//VR_XS_PMA_MP_16G_25G_RX_EQ_CTRL0 CLTE_BOOST_0=5’d14 CTLE_POLE_0=2’d2 VGA1_GAIN=3’d6 VGA2_GAIN=3’d6 for 10
	//VR_XS_PMA_MP_16G_25G_RX_EQ_CTRL0 CLTE_BOOST_0=5’d20 CTLE_POLE_0=2’d0 VGA1_GAIN=3’d7 VGA2_GAIN=3’d7 for 5
	data = xpcs_read_indirect(phyid, pcs, 0x60160);
	if (init_speed == USXGMII_INIT_10G)
		data |= (((6 << 12) | (6 << 8) | (2 << 5) | (14 << 0)) & 0xffff);
	else {
		data |= (((7 << 12) | (7 << 8) | (20 << 0)) & 0xffff);
		data &= (~(3 << 5) & 0xffff);
	}
	xpcs_write_indirect(phyid, pcs, 0x60160, data);
	//VR_XS_PMA_MP_25G_RX_AFE_RATE_CTRL RX0_AFE_RATE=3’d4 for 10 / RX0_AFE_RATE=3’d6 for 5
	data = xpcs_read_indirect(phyid, pcs, 0x602c4);
	if (init_speed == USXGMII_INIT_10G)
		data |= ((4 << 0) & 0xffff);
	else
		data |= ((6 << 0) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0x602c4, data);
	//VR_XS_PMA_MP_16G_25G_RX_IQ_CTRL0 RX0_DELTA_IQ=4’d6 for 10 / RX0_DELTA_IQ=4’d0 for 5
	data = xpcs_read_indirect(phyid, pcs, 0x601ac);
	if (init_speed == USXGMII_INIT_10G)
		data |= ((6 << 8) & 0xffff);
	else
		data &= (~(0xf << 8) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0x601ac, data);
	//VR_XS_PMA_MP_12G_16G_25G_DFE_TAP_CTRL0 DFE_TAP1_0=8’d15 for 10 / DFE_TAP1_0=8’d0 for 5
	data = xpcs_read_indirect(phyid, pcs, 0x60178);
	if (init_speed == USXGMII_INIT_10G)
		data |= 0xff;
	else
		data &= (~(0xff) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0x60178, data);
	//VR_XS_PMA_MP_25G_RX_VCO_CFG0 RX0_VCO_CFG=12’d768
	data = xpcs_read_indirect(phyid, pcs, 0x602d0);
	data |= ((768 << 0) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0x602d0, data);
	//VR_XS_PMA_MP_16G_25G_RX_GENCTRL4 RX_DFE_BYP_0=0
	data = xpcs_read_indirect(phyid, pcs, 0x601a0);
	data &= ((~(1 << 8)) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0x601a0, data);
	//VR_XS_PMA_MP_16G_25G_RX_PPM_CTRL0 RX0_CDR_PPM_MAX=5’d18
	data = xpcs_read_indirect(phyid, pcs, 0x60194);
	data |= ((18 << 0) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0x60194, data);
}

static int bstmac_xpcs_usxgmii_pcs(u16 phyid, u16 pcs, u16 init_speed)
{
	u16 data;
	int i;

	//1.Write 4'b0000 to bits [3:0] of SR_XS_PCS_CTRL2 register to switch DWC_xpcs to BASE-R mode.
	data = xpcs_read_indirect(phyid, pcs, 0xc001c);
        data &= (~0xf);
        xpcs_write_indirect(phyid, pcs, 0xc001c, data);
	//2.Select the appropriate USXGMII mode by programming USXG_MODE field of VR_XS_PCS_KR_CTRL register to suitable values
	data = xpcs_read_indirect(phyid, pcs, 0xe001c);
	if (init_speed == USXGMII_INIT_10G)
		data &= ((~0x7) << 10) & 0xffff;
	else
		data |= ((1 << 10) & 0xffff); //5G
	xpcs_write_indirect(phyid, pcs, 0xe001c, data);
	//3.Program bit [9] (USXG_EN) of VR_XS_PCS_DIG_CTRL1 register to 1 to enable USXGMII Mode inside DWC_xpcs.
	data = xpcs_read_indirect(phyid, pcs, 0xe0000);
	data |= ((1 << 9) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0xe0000, data);

	//4.Configure the PHY to operate at 10.3125Gbps/5.15625Gbps rate by programming various registers as shown in Table 12-1
	bstmac_xpcs_10g_baser_phy(phyid, pcs, init_speed);

	//5.Initiate the Vendor specific software reset by writing 1’b1 to the VR_RST bit [15] of the VR_XS_PCS_DIG_CTRL1 register.
	data = xpcs_read_indirect(phyid, PUBLIC_CONFIG_XPCS, 0xe0000);
	data |= ((1 << 15) & 0xffff);
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0xe0000, data);
	//6.Wait for bit [15] of the VR_XS_PCS_DIG_CTRL1 register to get cleared.
	udelay(10);
	i = 100;
	while(i--) {
		data = xpcs_read_indirect(phyid, PUBLIC_CONFIG_XPCS, 0xe0000);
		if (!(data & 0x8000))
			break;
		udelay(10);
	}
	if (!i)
		return -1;

	return 0;
}

//10G/5g usxgmii
static int bstmac_xpcs_init_usxgmii(u16 phyid, u16 init_speed)
{
	int ret, pcs;
	
        for (pcs = 0; pcs < XPCS_NUM; pcs++) {
                ret = bstmac_xpcs_usxgmii_pcs(phyid, pcs, init_speed);
                if (ret)
                        return -1;
        }
	return 0;
}

static void bstmac_xpcs_1000basex_vr_step7(u16 phyid, u16 pcs)
{
        u16 data;

        //7.Program bit [4] (SUPRESS_LOS_DET) and bit [6] (RX_DT_EN_CTL) of VR_XS_PCS_DEBUG_CTRL register to 0
	data = xpcs_read_indirect(phyid, pcs, 0xe0014);
	data &= (~((1 << 4) | (1 << 6)) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0xe0014, data);
}

static int bstmac_xpcs_sram_init(u16 phyid, u16 pcs)
{
	int i = 1000;
	u16 data;

	while (i--) {
		data = xpcs_read_indirect(phyid, pcs, 0x6026c);
		udelay(10);
		if ((data & 0x1) ==  0x1) //step4:Poll bit [0] (INIT_DN) of VR_XS_PMA_MP_12G_16G_25G_SRAM register till it becomes 1.
			break;
	}
	if (i < 0)
		return -1;
	//step6:Program bit [1] (EXT_LD_DN) of VR_XS_PMA_MP_12G_16G_25G_SRAM register to 1.
	data |= 0x2;
	xpcs_write_indirect(phyid, pcs, 0x6026c, data);
	//step7:Read SR_XS_PCS_CTRL1 register and wait till the 15th bit of these registers is read as 0
	//data = xpcs_read_indirect(phyid, pcs, 0xc0000);
	//data |= 0x8000;
	//xpcs_write_indirect(phyid, pcs, 0xc0000, data);
	udelay(10);
	i = 3000;
	while (i--) {
		data = xpcs_read_indirect(phyid, pcs, 0xc0000);
		if (!(data & 0x8000))
			break;
		udelay(100);
	}
	if (i < 0)
		return -2;
	print_dbg("%s line %d *********************************0xc0000 rd cnt %d\n", __func__, __LINE__, i);
	return 0;
}

static int bstmac_xpcs_init_pcs(u16 phyid, u16 pcs)
{	
	int i = 100;
	u16 data;

	//step8:Program bit [4] (SUPRESS_LOS_DET) and bit [6] (RX_DT_EN_CTL) of VR_XS_PCS_DEBUG_CTRL register to 1
	data = xpcs_read_indirect(phyid, pcs, 0xe0014);
	data |= ((1 << 4) | (1 << 6)) & 0xffff;
	xpcs_write_indirect(phyid, pcs, 0xe0014, data);

	//step9:Poll for RX_valID_0 bit of VR_XS_PMA_RX_LSTS register to indicate 1
	udelay(10);
	i = 100;
	while (i--) {
		data = xpcs_read_indirect(phyid, pcs, 0x60080);
		if (data & ((1 << 12) & 0xffff))
			break;
		udelay(10);
	}
	if (!i)
		return -3;
	//step10:Program RX_AD_REQ bit of VR_XS_PMA_MP_12G_16G_25G_RX_EQ_CTRL4 register to 1.
	data = xpcs_read_indirect(phyid, pcs, 0x60170);
	data |= (1 << 12) & 0xffff;
	xpcs_write_indirect(phyid, pcs, 0x60170, data);
	//step11:Poll for RX_ADPT_ACK bit of VR_XS_PMA_MP_12G_16G_25G_MISC_STS register to indicate 1.
	udelay(10);
	i = 3000;
	while (i--) {
		data = xpcs_read_indirect(phyid, pcs, 0x60260);
		if (data & ((1 << 12) & 0xffff))
			break;
		udelay(100);
	}
	if (!i)
		return -4;
	//step12: Program RX_AD_REQ bit of VR_XS_PMA_MP_12G_16G_25G_RX_EQ_CTRL4 register to 0
	data = xpcs_read_indirect(phyid, pcs, 0x60170);
	data &= (~((1 << 12) & 0xffff));
	xpcs_write_indirect(phyid, pcs, 0x60170, data);

	return 0;
}


static int bstmac_xpcs_common_init(u16 phyid)
{
	int ret = 0;
	u16 pcs_start = XPCS_START, pcs_end = XPCS_END;

	ret = bstmac_xpcs_sram_init(phyid, PUBLIC_CONFIG_XPCS);
	if (ret) {
		print_dbg("%s line %d sram_init ret %d fail [phyid:pcs %d:%d]\n", __func__, __LINE__, ret, phyid, PUBLIC_CONFIG_XPCS);
		return -1;
	}

    	print_dbg("%s %d\n",__func__,__LINE__);
    	for(pcs_start = XPCS_START;pcs_start < pcs_end; pcs_start++) {
		print_dbg("%s %d pcs%d start\n",__func__,__LINE__, pcs_start);
		ret = bstmac_xpcs_init_pcs(phyid, pcs_start);
		if (ret) {
			print_dbg("%s line %d init_pcs ret %d fail [phyid:pcs %d:%d]\n", __func__, __LINE__, ret, phyid, pcs_start);
			return -1;
		}
		mdelay(20);
		print_dbg("%s %d pcs%d end\n",__func__,__LINE__, pcs_start);
	}
	print_dbg("%s %d\n",__func__,__LINE__);
	return 0;
}

static int bstmac_xpcs_1000basex_vr(u16 phyid, u16 pcs)
{
	u16 data;
	int i;

	//5.Initiate the Vendor specific software reset by writing 1’b1 to the VR_RST bit [15] of the VR_XS_PCS_DIG_CTRL1 register.
	data = xpcs_read_indirect(phyid, pcs, 0xe0000);
	data |= ((1 << 15) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0xe0000, data);
	//6.Wait for bit [15] of the VR_XS_PCS_DIG_CTRL1 register to get cleared.
	udelay(10);
	i = 3000;
	while(i--) {
		data = xpcs_read_indirect(phyid, pcs, 0xe0000);
		if (!(data & 0x8000))
			break;
		udelay(100);
	}
	if (i < 0) {
                print_dbg("%s line %d wait timeout\n", __func__, __LINE__);
        	return -1;
        }
	print_dbg("%s line %d ######################################0xe0000 rd cnt %d\n", __func__, __LINE__, i);
	return 0;
}


static void bstmac_xpcs_1000basex_pcs(u16 phyid, u16 pcs)
{
	u16 data;
	
	//1.Program 4’b0001 to bits [3:0] of SR_XS_PCS_CTRL2 register
	data = xpcs_read_indirect(phyid, pcs, 0xc001c);
	data &= (~0xf);
	data |= 0x1;
	xpcs_write_indirect(phyid, pcs, 0xc001c, data);

	//2.Program bit [2] (EN_2_5G_MODE) of VR_XS_PCS_DIG_CTRL1 to 0.
	//3. Program bit [9] (USXG_EN) of VR_XS_PCS_DIG_CTRL1 register to 0
	data = xpcs_read_indirect(phyid, pcs, 0xe0000);
	data &= (~((1 << 2) | (1 << 9)) & 0xffff);
	xpcs_write_indirect(phyid, pcs, 0xe0000, data);	
}

static void bstmac_xpcs_1000basex_phy(u16 phyid)
{
	u16 data;
	u16 pcs_start = XPCS_START;

	for (pcs_start = XPCS_START; pcs_start < XPCS_END; pcs_start++) {
		//VR_XS_PMA_MP_12G_16G_25G_MPLL_CMN_CTRL     MPLL_SEL_0 = 0
		data = xpcs_read_indirect(phyid, pcs_start, 0x601c0);
		data &= (~(1 << 4) & 0xffff); 
		xpcs_write_indirect(phyid, pcs_start, 0x601c0, data);
	}
	//VR_XS_PMA_MP_12G_16G_25G_REF_CLK_CTRL REF_RANGE=6 REF_CLK_DIV2=0 REF_MPLLA_DIV=1
	data = xpcs_read_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x60244);
	data &= (~((7 << 10) | (7 << 3) | (1 << 2)) & 0xffff);
	data |= (((1 << 10) | (6 << 3)) & 0xffff);
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x60244, data);
	//VR_XS_PMA_MP_25G_MPLLA_CTRL0 MPLLA_W_CLK_DIV=1 MPLLA_MULTIPLIER= 8’d128
	data = xpcs_read_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601c4);
	data &= (~((3 << 12) | (0xfff))) & 0xffff;
	data |= (((1 << 12) | (128 << 0)) & 0xffff);
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601c4, data);
	//VR_XS_PMA_MP_25G_MPLLA_CTRL1 FRAC_EN=0 FB_DIV4_EN=1 BW_TH=2’d3
	data = xpcs_read_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601c8);
	data &= (~(1 << 8) & 0xffff);
	data |= (((1 << 10) | (3 << 12)) & 0xffff);
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601c8, data);
	//VR_XS_PMA_MP_25G_MPLLA_CTRL2 MPLLA_DIV16P5_CLK_EN=0 MPLLA_TX_CLK_DIV=4
	data = xpcs_read_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601cc);
	//data &= (~((1 << 8) | (7 << 11)) & 0xffff);
	data &= (~(7<<11) & 0xffff);
	data |= ((1 << 8)) & 0xffff;
	data |= ((4 << 11) & 0xffff);
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601cc, data);
	//VR_XS_PMA_MP_25G_MPLLA_CTRL3 MPLLA_FRACN_QUOT=0
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601dc, 0);
	//VR_XS_PMA_MP_25G_MPLLA_CTRL4 MPLLA_FRACN_REM=0
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601e4, 0);
	//VR_XS_PMA_MP_25G_MPLLA_CTRL5 MPLLA_FRACN_DEN=0
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601ec, 0);
	//VR_XS_PMA_MP_25G_MPLLA_BW_LO_CTRL MPLLA_BW_LO=16’d38
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601f8, 38);
	//VR_XS_PMA_MP_25G_MPLLA_BW_HI_CTRL MPLLA_BW_HI=16’d38
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x601f4, 38);
	for (pcs_start = XPCS_START; pcs_start < XPCS_END; pcs_start++) {
		//VR_XS_PMA_MP_12G_16G_25G_VCO_CAL_LD0 VCO_LD_val_0=13’d1344
		data = xpcs_read_indirect(phyid, pcs_start, 0x60248);
		data &= (~0x1ff);
		data |= (1344 << 0) & 0xffff;
		xpcs_write_indirect(phyid, pcs_start, 0x60248, data);
		//VR_XS_PMA_MP_16G_25G_VCO_CAL_REF0 VCO_REF_LD_0=7’d21
		data = xpcs_read_indirect(phyid, pcs_start, 0x60258);
		data &= (~0x7f);
		data |= (21 << 0) & 0xffff;
		xpcs_write_indirect(phyid, pcs_start, 0x60258, data);
	}
	//VR_XS_PMA_MP_12G_16G_25G_MISC_CTRL0 RX_VREF_CTRL=5
	data = xpcs_read_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x60240);
	data |= (5 << 8) & 0Xffff;
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x60240, data);
	//VR_XS_PMA_MP_16G_25G_MISC_CTRL2 SUP_MISC=0
	data = xpcs_read_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x60270);
	data &= (~(0xff << 0) & 0Xffff);
	xpcs_write_indirect(phyid, PUBLIC_CONFIG_XPCS, 0x60270, data);
	for (pcs_start = XPCS_START; pcs_start < XPCS_END; pcs_start++) {
		//VR_XS_PMA_MP_12G_16G_25G_RX_GENCTRL1 RX_DIV16P5_CLK_EN_0=0
		data = xpcs_read_indirect(phyid, pcs_start, 0x60144);
		data &= (~(1 << 12) & 0Xffff);
		xpcs_write_indirect(phyid, pcs_start, 0x60144, data);
		//VR_XS_PMA_MP_12G_16G_25G_TX_RATE_CTRL TX0_RATE=2
		data = xpcs_read_indirect(phyid, pcs_start, 0x600d0);
		data &= (~7);
		data |= 2;
		xpcs_write_indirect(phyid, pcs_start, 0x600d0, data);
		//VR_XS_PMA_MP_12G_16G_25G_RX_RATE_CTRL RX0_RATE=4
		data = xpcs_read_indirect(phyid, pcs_start, 0x60150);
		data &= (~7);
		data |= 4;
		xpcs_write_indirect(phyid, pcs_start, 0x60150, data);
		//VR_XS_PMA_MP_25G_TX_WIDTH_CTRL TX0_WIDTH=3’b001 (10-bit)
		data = xpcs_read_indirect(phyid, pcs_start, 0x60118);
		data &= (~7 & 0xffff);
		data |= 1;
		xpcs_write_indirect(phyid, pcs_start, 0x60118, data);
		//VR_XS_PMA_MP_25G_RX_WIDTH_CTRL RX0_WIDTH=3’b001 (10-bit)
		data = xpcs_read_indirect(phyid, pcs_start, 0x602c0);
		data &= (~7 & 0xffff);
		data |= 1;
		xpcs_write_indirect(phyid, pcs_start, 0x602c0, data);
		//VR_XS_PMA_MP_12G_16G_25G_TX_GENCTRL1 VBOOST_EN_0=0 VBOOST_LVL=5
		data = xpcs_read_indirect(phyid, pcs_start, 0x600c4);
		data &= (~(1 << 4) & 0xffff);
		data |= (5 << 8) & 0xffff;
		xpcs_write_indirect(phyid, pcs_start, 0x600c4, data);
		//VR_XS_PMA_MP_16G_25G_RX_EQ_CTRL5 RX0_ADPT_MODE=2’d0
		data = xpcs_read_indirect(phyid, pcs_start, 0x60174);
		data &= (~(3 << 4) & 0xffff);
		xpcs_write_indirect(phyid, pcs_start, 0x60174, data);
		//VR_XS_PMA_MP_16G_25G_RX_EQ_CTRL0 CLTE_BOOST_0=5’d15 CTLE_POLE_0=2’d0 VGA1_GAIN=3’d4 VGA2_GAIN 3’d4
		data = xpcs_read_indirect(phyid, pcs_start, 0x60160);
		data &= (~((3 << 5) | (0x1f) | (7 << 12) | (7 << 8)) & 0xffff);
		data |= (((15 << 0) | (4 << 12) | (4 << 8)) & 0xffff);
		xpcs_write_indirect(phyid, pcs_start, 0x60160, data);
		//VR_XS_PMA_MP_25G_RX_AFE_RATE_CTRL RX0_AFE_RATE=3’d7
		data = xpcs_read_indirect(phyid, pcs_start, 0x602c4);
		data |= ((7 << 0) & 0xffff);
		xpcs_write_indirect(phyid, pcs_start, 0x602c4, data);
		//VR_XS_PMA_MP_16G_25G_RX_IQ_CTRL0 RX0_DELTA_IQ=4’d0
		data = xpcs_read_indirect(phyid, pcs_start, 0x601ac);
		data &= (~(0xf << 8) & 0xffff);
		xpcs_write_indirect(phyid, pcs_start, 0x601ac, data);
		//VR_XS_PMA_MP_12G_16G_25G_DFE_TAP_CTRL0 DFE_TAP1_0=8’d0
		data = xpcs_read_indirect(phyid, pcs_start, 0x60178);
		data &= (~(0xff << 0) & 0xffff);
		xpcs_write_indirect(phyid, pcs_start, 0x60178, data);
		//VR_XS_PMA_MP_25G_RX_VCO_CFG0 RX0_VCO_CFG=12’d256
		data = xpcs_read_indirect(phyid, pcs_start, 0x602d0);
		data &= (~(0xfff));
		data |= ((256 << 0) & 0xffff);
		xpcs_write_indirect(phyid, pcs_start, 0x602d0, data);
		//VR_XS_PMA_MP_16G_25G_RX_GENCTRL4 RX_DFE_BYP_0=1
		data = xpcs_read_indirect(phyid, pcs_start, 0x601a0);
		data |= ((1 << 8) & 0xffff);
		xpcs_write_indirect(phyid, pcs_start, 0x601a0, data);
		//VR_XS_PMA_MP_16G_25G_RX_PPM_CTRL0 RX0_CDR_PPM_MAX=5’d18
		data = xpcs_read_indirect(phyid, pcs_start, 0x60194);
		data &= (~(0x1f));
		data |= ((18 << 0) & 0xffff);
		xpcs_write_indirect(phyid, pcs_start, 0x60194, data);
	}
}

#if 0
static bool bstmac_xpcs_per_sgmii_pcs_is_link(u16 phyid, u16 pcs)
{
        u16 data;

        data = xpcs_read_indirect(phyid, pcs, 0x7e0008);
        if (data & (1 << 4))
                return true;

        return false;
}
#endif
static void bstmac_xpcs_init_1000basex(u16 phyid, u16 pcs)
{       
	print_dbg("%s %d\n",__func__,__LINE__);
    	bstmac_xpcs_1000basex_pcs(phyid, pcs);
	print_dbg("%s %d\n",__func__,__LINE__);
}

static void bstmac_xpcs_init_1000basex_2(u16 phyid)
{
	//Configure the PHY to operate at 1.25 Gbps rate by programming various registers as shown in Table 12-2	
	bstmac_xpcs_1000basex_phy(phyid);

    	print_dbg("%s %d\n",__func__,__LINE__);
}

static int bstmac_xpcs_init_1000basex_3(u16 phyid)
{
	int ret = 0;
	u16 pcs_start = XPCS_START;

	print_dbg("%s %d\n",__func__,__LINE__);
	ret = bstmac_xpcs_1000basex_vr(phyid, PUBLIC_CONFIG_XPCS);
	if (ret) {
		print_dbg("%s %d ret %d\n",__func__,__LINE__, ret);
		return ret;
	}
	print_dbg("%s %d\n",__func__,__LINE__);
	for (pcs_start = XPCS_START; pcs_start < XPCS_END; pcs_start++) {
		print_dbg("%s %d\n",__func__,__LINE__);
		bstmac_xpcs_1000basex_vr_step7(phyid, XPCS_START);
		print_dbg("%s %d\n",__func__,__LINE__);
	}

	return 0;
}

static int bstmac_xpcs_init_1g_sgmii(u16 phyid)
{
	u16 pcs_start = XPCS_START, pcs_end = XPCS_END;

	for (pcs_start = XPCS_START;pcs_start < pcs_end; pcs_start++) {	
		print_dbg("%s %d pcs%d start\n",__func__,__LINE__, pcs_start);
		print_dbg("%s %d\n",__func__,__LINE__);
		bstmac_xpcs_init_1000basex(phyid, pcs_start);
		print_dbg("%s %d\n",__func__,__LINE__);
		print_dbg("%s %d pcs%d end\n",__func__,__LINE__, pcs_start);
	}
	
	bstmac_xpcs_init_1000basex_2(phyid);
	
	bstmac_xpcs_init_1000basex_3(phyid);

	for (pcs_start = XPCS_START;pcs_start < pcs_end; pcs_start++) {
		print_dbg("%s %d pcs%d start\n",__func__,__LINE__, pcs_start);
		bstmac_xpcs_phy_sgmii_per_pcs(phyid, pcs_start, 1000);
		print_dbg("%s %d pcs%d end\n",__func__,__LINE__, pcs_start);
	}    

    return 0;
}

static int bstmac_xpcs_probe(struct mdio_xpcs_args *xpcs, phy_interface_t interface)
{
	int ret = -ENODEV;

	pcs_addr = ioremap(0x217c0000, 0x4000);
	if (!pcs_addr)
		return -ENOMEM;
	print_dbg("%s line %d pcs_addr 0x%llx\n", __func__, __LINE__, (u64)pcs_addr);

        //bypass epp:1.xgmac:phy0 pcs0 2.sw-gmac:phy1 pcs0
        if (interface == PHY_INTERFACE_MODE_SGMII) {
	        ret = bstmac_xpcs_common_init(xpcs->addr);
                ret |= bstmac_xpcs_init_1g_sgmii(xpcs->addr);
        }

        if (interface == PHY_INTERFACE_MODE_XGMII)
                ret = bstmac_xpcs_init_usxgmii(xpcs->addr, 10000);
	
	mdelay(10);
	iounmap(pcs_addr);
	
	return ret;
}

static struct mdio_xpcs_ops xpcs_ops = {
	.link_up = xpcs_link_up,
	.probe = bstmac_xpcs_probe,
};

struct mdio_xpcs_ops *bst_xpcs_get_ops(void)
{
	return &xpcs_ops;
}
EXPORT_SYMBOL_GPL(bst_xpcs_get_ops);

MODULE_LICENSE("GPL v2");
