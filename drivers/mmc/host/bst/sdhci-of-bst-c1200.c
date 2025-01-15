// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
 
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/i2c.h>
#include <linux/arm-smccc.h>

#include "../sdhci-pltfm.h"
#include "bst-sdhci.h"

struct dwcmshc_priv {
	struct clk *bus_clk;
	uint32_t channel;
	uint32_t  phy_crm_reg_base;
};

#define REG_SDMMC_SOFTRST_SEL (0x30002000+0x184)
#define SDMMC_SOFTRST_SEL0 0x20
#define SDMMC_SOFTRST_SEL1 0x10

//#define REG_SD_EMMC_SEL 0x33000064 // bit0 for sd0/emmc0  bit1 for sd1/emmc1  (0:sd  1:emmc)
#define BST_SDMMC_VER_ID	    0x3138302A
#define SDHCI_VENDOR_PTR_R	    0xE8
#define SYS_CTRL_SDEMMC_DIV_CTRL    0x3300003C
#define SYS_CTRL_SDEMMC_CTRL_EN_CLR 0x33000068
#define TOP_IO_CFG_REG_R_IO_CFG_41  0x33001154
#define TOP_IO_CFG_REG_R_IO_CFG_42  0x33001158

#define MBIU_CTRL	       0x510
#define BURST_INCR16_EN	       BIT(3)
#define BURST_INCR8_EN	       BIT(2)
#define BURST_INCR4_EN	       BIT(1)
#define BURST_EN	       (BURST_INCR16_EN | BURST_INCR8_EN | BURST_INCR4_EN)
/* Synopsys vendor specific registers */
#define reg_offset_addr_vendor (sdhci_readw(host, SDHCI_VENDOR_PTR_R))
#define SDHC_MHSC_VER_ID_R     (reg_offset_addr_vendor)
#define SDHC_MHSC_VER_TPYE_R   (reg_offset_addr_vendor + 0X4)
#define SDHC_MHSC_CTRL_R       (reg_offset_addr_vendor + 0X8)
#define SDHC_MBIU_CTRL_R       (reg_offset_addr_vendor + 0X10)
#define SDHC_EMMC_CTRL_R       (reg_offset_addr_vendor + 0X2C)
#define SDHC_BOOT_CTRL_R       (reg_offset_addr_vendor + 0X2E)
#define SDHC_GP_IN_R	       (reg_offset_addr_vendor + 0X30)
#define SDHC_GP_OUT_R	       (reg_offset_addr_vendor + 0X34)
#define SDHC_AT_CTRL_R	       (reg_offset_addr_vendor + 0X40)
#define SDHC_AT_STAT_R	       (reg_offset_addr_vendor + 0X44)

#define SDHC_SW_TUNE_EN 0x00000010

#define SOC_PMM_REG_BASEADDR                    0x30001000
#define SC_PMM_REG_OFFSET(n)                    (0x04*n)

#define LOCAL_RST_CFG  							0x00
#define CLK_GATE_CFG 							0x04
#define SDEMMC_CRM_BCLK_DIV_CTRL 				0x08
#define SDEMMC_CRM_RX_CLK_CTRL                  0X14
#define SDEMMC_CRM_TIMER_DIV_CTRL 				0x0C
#define SDEMMC_CRM_CQE_CLK_DIV_CTRL 			0x10
#define SDEMMC_CRM_ECC_STATUS 					0x18
#define SDEMMC_CRM_VOL_CTRL 					0x1C
#define SDEMMC_CRM_LED_CTRL 					0x20
#define SDEMMC_CRM_PARITY                       0x24
#define SDEMMC_CRM_FREQ_CHECK_EN                0x28
#define SDEMMC_CRM_RREQ_INTR                    0x2c
#define SDEMMC_CRM_TXCLK_FREQ_REF_CLKCNT        0x30
#define SDEMMC_CRM_TXCLK_RREQ_HIGHCLKCNT        0x34
#define SDEMMC_CRM_TXCLK_RREQ_THRESHOLD         0x38
#define SDEMMC_CRM_TXCLK_RREQ_HIGHCLK_PPM       0x3c
#define SDEMMC_CRM_RXCLK_RREQ_REF_CLKCNT        0x40
#define SDEMMC_CRM_RXCLK_RREQ_HIGHCLKCNT        0x44
#define SDEMMC_CRM_RXCLK_RREQ_THRESHOLD         0x48
#define SDEMMC_CRM_RXCLK_RREQ_HIGHCLK_PPM       0x4c
#define SDEMMC_CRM_TIMERCLK_RREQ_REF_CLKCNT     0x50
#define SDEMMC_CRM_TIMERCLK_RREQ_HIGHCLKCNT     0x54
#define SDEMMC_CRM_TIMERCLK_RREQ_THRESHOLD      0x58
#define SDEMMC_CRM_TIMERCLK_RREQ_HIGHCLK_PPM    0x5c
#define SDEMMC_CRM_CQECLK_RREQ_REF_CLKCNT       0x60
#define SDEMMC_CRM_CQECLK_RREQ_HIGHCLKCNT       0x64
#define SDEMMC_CRM_CQECLK_RREQ_THRESHOLD        0x68
#define SDEMMC_CRM_CQECLK_RREQ_HIGHCLK_PPM      0x6c
#define SDEMMC_CRM_BCLK_RREQ_REF_CLKCNT         0x70
#define SDEMMC_CRM_BCLK_RREQ_HIGHCLKCNT         0x74
#define SDEMMC_CRM_BCLK_RREQ_THRESHOLD          0x78
#define SDEMMC_CRM_BCLK_RREQ_HIGHCLK_PPM        0x7c

/* MMCM DRP */
#define SDHC_MMCM_DIV_REG  0x1020
#define DIV_REG_100_MHZ	   0x1145
#define DIV_REG_200_MHZ	   0x1083
#define SDHC_MMCM_CLKFBOUT 0x1024
#define CLKFBOUT_100_MHZ   0x0000
#define CLKFBOUT_200_MHZ   0x0080
#define SDHC_CCLK_MMCM_RST 0x00000001
#define DRIVER_NAME	   "sdhci_bst"

/* I2C frame. */
#define BST_ADDRESS_BASE    0x08U /* I2C device base address */
#define BST_COMM_FRAME_SIZE 0x03U /* Length of the communication frame */
#define BST_FRAME_SIZE	    0x04U /* Length of the complete I2C frame */
#define BST_READ_FRAME_LENGTH \
	0x01U /* Length of the data frame for I2C read command. */
#define BST_RX_SIZE 0x02U /* Length of the received I2C data frame */

/* CRC polynomial. */
#define BST_CRC_TBL_SIZE 256U /* Size of CRC table. */
#define BST_CRC_POLYNOM	 0x1DU /* CRC polynom. */
#define BST_CRC_INIT	 0xFFU /* CRC initial value. */



#define SDHCI_DUMP_BST(f, x...) \
	pr_err("%s: " DRIVER_NAME ": " f, mmc_hostname(host->mmc), ##x)
#define SD_3_3V 0
#define SD_1_8V 1


void sdhci_bst_print_vendor(struct sdhci_host *host)
{
	SDHCI_DUMP_BST("============ SDHCI VENDOR REGISTER DUMP ===========\n");

	SDHCI_DUMP_BST("VER_ID:  0x%08x | VER_TPYE:  0x%08x\n",
		       sdhci_readl(host, SDHC_MHSC_VER_ID_R),
		       sdhci_readl(host, SDHC_MHSC_VER_TPYE_R));
	SDHCI_DUMP_BST("MHSC_CTRL:  0x%08x |MBIU_CTRL:  0x%08x\n",
		       sdhci_readw(host, SDHC_MHSC_CTRL_R),
		       sdhci_readw(host, SDHC_MBIU_CTRL_R));
	SDHCI_DUMP_BST("EMMC_CTRL:  0x%08x | BOOT_CTRL: 0x%08x\n",
		       sdhci_readl(host, SDHC_EMMC_CTRL_R),
		       sdhci_readw(host, SDHC_BOOT_CTRL_R));
	SDHCI_DUMP_BST("GP_IN:   0x%08x | GP_OUT: 0x%08x\n",
		       sdhci_readl(host, SDHC_GP_IN_R),
		       sdhci_readb(host, SDHC_GP_OUT_R));
	SDHCI_DUMP_BST("AT_CTRL:     0x%08x | AT_STAT:  0x%08x\n",
		       sdhci_readb(host, SDHC_AT_CTRL_R),
		       sdhci_readb(host, SDHC_AT_STAT_R));
}
EXPORT_SYMBOL_GPL(sdhci_bst_print_vendor);

static u32 bst_read_phys_bst(u32 phys_addr)
{
	u32 phys_addr_page = phys_addr & 0xFFFFE000;
	u32 phys_offset = phys_addr & 0x00001FFF;
	u32 map_size = phys_offset + sizeof(u32);
	u32 ret = 0xDEADBEEF;
	void *mem_mapped = ioremap(phys_addr_page, map_size);

	if (mem_mapped != NULL) {
		ret = (u32)ioread32(((u8 *)mem_mapped) + phys_offset);
		iounmap(mem_mapped);
	}

	return ret;
}

static void bst_write_phys_bst(u32 phys_addr, u32 value)
{
	u32 phys_addr_page = phys_addr & 0xFFFFE000;
	u32 phys_offset = phys_addr & 0x00001FFF;
	u32 map_size = phys_offset + sizeof(u32);
	void *mem_mapped = ioremap(phys_addr_page, map_size);

	if (mem_mapped != NULL) {
		iowrite32(value, ((u8 *)mem_mapped) + phys_offset);
		iounmap(mem_mapped);
	}
}

static unsigned int bst_get_max_clock(struct sdhci_host *host)
{
	return host->mmc->f_max;
}

static unsigned int bst_get_min_clock(struct sdhci_host *host)
{
	return host->mmc->f_min;
}


typedef union {
	struct {
		u32 rx_revert:1;
		u32 rx_clk_sel_sec:1;
		u32 rx_clk_div:4;		
		u32 rx_clk_phase_inner:2;
		u32 rx_clk_sel_first:1;
		u32 rx_clk_phase_out:2;	
		u32 rx_clk_en:1;	
		u32 res0:20;					
	} bit;
	u32 reg;
} rx_ctrl_u;



typedef union {
	struct {
		u32 res0:16;
		u32 SC_SDMMC0_PVDD18POCSD0:2;
		u32 SC_SDMMC0_PVDD18POCSD1:2;
		u32 SC_SDMMC0_PVDD18POCSD2:2;
		u32 SC_SDMMC1_PVDD18POCSD0:2;
		u32 SC_SDMMC1_PVDD18POCSD1:2;
		u32 SC_SDMMC1_PVDD18POCSD2:2;	
		u32 res1:4;					
	} bit;
	u32 reg;
} sdmmc_iocfg_u;


void sdhci_enable_bst_clk(struct sdhci_host *host, unsigned int clk)
{
	struct sdhci_pltfm_host *pltfm_host;	
	struct dwcmshc_priv *priv;
#define default_max_freq 200000ul
	unsigned int div;
	u32 val;
	rx_ctrl_u rx_reg;

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);
	if (clk == 0) {
		div = clk;
	} else if (clk > default_max_freq) {
		div = clk / 1000;
		div = default_max_freq / div;
	} else if (clk < 1500) {
		div = clk;
	} else {
		div = default_max_freq * 100;
		div = div / clk;
		div /= 100;
	}

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk &= ~SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	clk &= ~SDHCI_CLOCK_PLL_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);




	//bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL,0x00000b13);//bit10:sdemmc_div_ctrl_en = 0

	val = bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_TIMER_DIV_CTRL);
	val &= ~(1<<8);
	bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_TIMER_DIV_CTRL,val);

	val = bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_TIMER_DIV_CTRL);
	val &= ~(0xff);
	val |=0x20;
    bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_TIMER_DIV_CTRL,  val);//bit0-7:sdemmc_timer_div_ctrl = div //800/32=25M

	val = bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_TIMER_DIV_CTRL);
	val |= 1<<8;
    bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_TIMER_DIV_CTRL,val);//bit8:sdemmc_timer_div_ctrl_en = 1


	val = bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL);
	val &= ~(1<<11);
	bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL,val);

	if(priv->channel == 1){
		#if 0
		rx_reg.reg = bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL);
		//inner
		// rx_reg.bit.rx_revert = 1;
		// rx_reg.bit.rx_clk_sel_sec = 1;
		// rx_reg.bit.rx_clk_div = 4;
		// rx_reg.bit.rx_clk_phase_inner = 2;
		// rx_reg.bit.rx_clk_sel_first = 1;
		// rx_reg.bit.rx_clk_phase_out = 2;
		//out pad

		rx_reg.bit.rx_revert = 0;
		rx_reg.bit.rx_clk_sel_sec = 1;
		rx_reg.bit.rx_clk_div = 4;
		rx_reg.bit.rx_clk_phase_inner = 2;
		rx_reg.bit.rx_clk_sel_first = 0;
		rx_reg.bit.rx_clk_phase_out = 2;


		bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL,  rx_reg.reg);//bit0-7:sdemmc_timer_div_ctrl = div //800/32=25M
		#endif


		val = bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL);
		val &= ~(0x7ff);
		val |=0x00000b13;
		bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL,  val);//bit0-7:sdemmc_timer_div_ctrl = div //800/32=25M
	}
	else{
		rx_reg.reg = bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL);
		//inner
		// rx_reg.bit.rx_revert = 1;
		// rx_reg.bit.rx_clk_sel_sec = 1;
		// rx_reg.bit.rx_clk_div = 4;
		// rx_reg.bit.rx_clk_phase_inner = 2;
		// rx_reg.bit.rx_clk_sel_first = 1;
		// rx_reg.bit.rx_clk_phase_out = 2;
		//out pad

		rx_reg.bit.rx_revert = 0;
		rx_reg.bit.rx_clk_sel_sec = 1;
		rx_reg.bit.rx_clk_div = 4;
		rx_reg.bit.rx_clk_phase_inner = 2;
		rx_reg.bit.rx_clk_sel_first = 0;
		rx_reg.bit.rx_clk_phase_out = 2;


		bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL,  rx_reg.reg);//bit0-7:sdemmc_timer_div_ctrl = div //800/32=25M

	}
	val = bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL);
	val |= 1<<11;
    bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL,val);//bit8:sdemmc_timer_div_ctrl_en = 1


	bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_BCLK_DIV_CTRL,
       bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_BCLK_DIV_CTRL)&(~0x0400));


    bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_BCLK_DIV_CTRL, 
	(bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_BCLK_DIV_CTRL)&(~ 0x03ff))|div);//bit0-9:sdemmc_div_ctrl = div


    bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_BCLK_DIV_CTRL, 
	bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_BCLK_DIV_CTRL)|(0x0400));//bit10:sdemmc_div_ctrl_en = 1


	sdhci_writew(host, (div&0xff)<<8, SDHCI_CLOCK_CONTROL);

	sdhci_writew(host, (div&0xff)<<8, SDHCI_CLOCK_CONTROL);
	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk |= SDHCI_CLOCK_PLL_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
}

void sdhci_set_bst_clock(struct sdhci_host *host, unsigned int clock)
{


	// host->mmc->actual_clock = 0;


	
	
	if (clock == 0)
		return;
	// if (host->quirks2 & SDHCI_QUIRK2_CLK_FROM_DTS) {
		// if (pltfm_host->clock > clock) {
			// clk = bst_sdhci_calc_clk(host, clock,
						 // &host->mmc->actual_clock);
		// } else {
			// host->mmc->actual_clock = pltfm_host->clock;
			// clk = host->mmc->actual_clock / 1000;
		// }
	// } else {
		// clk = bst_sdhci_calc_clk(host, clock, &host->mmc->actual_clock);
	// }
	
	sdhci_enable_bst_clk(host, clock);
}

static void sdhci_bst_reset(struct sdhci_host *host, u8 mask)
{

	if (host->mmc->caps2 & MMC_CAP2_NO_SD) {
		sdhci_writew(host,
			     sdhci_readw(host, SDHC_EMMC_CTRL_R) & (~BIT(2)),
			     SDHC_EMMC_CTRL_R);
		sdhci_reset(host, mask);
		udelay(10);
		sdhci_writew(host, sdhci_readw(host, SDHC_EMMC_CTRL_R) | BIT(2),
			     SDHC_EMMC_CTRL_R);
	} else
		sdhci_reset(host, mask);
}

static void sdhci_bst_timeout(struct sdhci_host *host, struct mmc_command *cmd)
{
	sdhci_writeb(host, 0xE, SDHCI_TIMEOUT_CONTROL);
}

static void sdhci_bst_set_power(struct sdhci_host *host, unsigned char mode,
				unsigned short vdd)
{
	// if (!IS_ERR(host->mmc->supply.vmmc)) {
	// 	struct mmc_host *mmc = host->mmc;

	// 	mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, vdd);
	// }
	sdhci_set_power(host, mode, vdd);
	sdhci_writeb(host, 0xF, SDHCI_POWER_CONTROL);
	sdhci_writew(host, (sdhci_readw(host, MBIU_CTRL) & (~0xf)) | BURST_EN,
		     MBIU_CTRL);
}

// static void sdhci_bst_voltage_switch(struct sdhci_host *host)
// {
	
// }




static int	bst_sdhci_execute_tuning(struct sdhci_host *host, u32 opcode)
{

	int ret = 0;
	struct sdhci_pltfm_host *pltfm_host;
	struct dwcmshc_priv *priv;

	unsigned int clk = 0, timeout,i=0;
	int error;
	int start0 = -1,end0 = -1,best = 0,start1 = -1,end1 = -1,flag = 0;
	
	
	
	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);
	

	
	for(i=0;i<SDHCI_TUNING_COUNT;i++){
		
		bst_write_phys_bst(priv->phy_crm_reg_base + 0x88,0x1234abcd);//protected write opened
		bst_write_phys_bst(priv->phy_crm_reg_base + 0x94,(1ul<<i)-1);

		
		timeout = 20;
		while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
			& SDHCI_CLOCK_INT_STABLE)) {
			if (timeout == 0) {
				printk("%s: Internal clock never stabilised.\n",
					   __func__);
				return -EBUSY;
			}
			timeout--;
			udelay(1000);
		}
	
		ret = mmc_send_tuning(host->mmc,opcode,&error);
		if(ret != 0){
			flag  = 1;
		}else{
			if(flag == 0){
				if(start0 == -1){
					start0 = i;
				}
				end0 = i;
			}else{
				if(start1 == -1){
					start1 = i;
				}
				end1 = i;
			}
		}
	}

	
	best = end0 - start0 >= end1 - start1 ? ((end0 - start0)>>1)+start0:((end1 - start1)>>1) + start1;

	if(best < 0)
		best = 0;
	


	// printk("end0:%d start0:%d addr:%x,opcode:%x\n",end0,start0,priv->phy_crm_reg_base,opcode);
	// printk("end1:%d start1:%d addr:%x,opcode:%x\n",end1,start1,priv->phy_crm_reg_base,opcode);
	//printk("tuning best:%d %lx\n",best,(1ul<<best)-1);

	bst_write_phys_bst(priv->phy_crm_reg_base+0x94,(1ul<<best)-1);
	timeout = 20;
	
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
		& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			printk("%s: Internal clock never stabilised.\n",
				   __func__);
			return -EBUSY;
		}
		timeout--;
		udelay(1000);
	}


	
	return 0;
}

#if 0
extern unsigned long bst_sip_special_address_rw(u_int64_t x1, u_int64_t x2, u_int64_t x3);

void bst_set_pinctrl_iolength(struct sdhci_host *host,struct mmc_ios *ios){
	sdmmc_iocfg_u sdmmc_reg;
	struct sdhci_pltfm_host *pltfm_host;
	struct dwcmshc_priv *priv;



	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);
	
	
	if(priv->channel == 0){
		return;
	}

	sdmmc_reg.reg = bst_sip_special_address_rw(0x300011c0,0,0);
	
	//printk("bst_set_pinctrl_iolength:%x\n",sdmmc_reg.reg.);
	switch(ios->signal_voltage){

		case MMC_SIGNAL_VOLTAGE_330:{
			
			sdmmc_reg.bit.SC_SDMMC1_PVDD18POCSD0 = 0;
			sdmmc_reg.bit.SC_SDMMC1_PVDD18POCSD1 = 0;
			sdmmc_reg.bit.SC_SDMMC1_PVDD18POCSD2 = 0;

			bst_sip_special_address_rw(0x300011c0,sdmmc_reg.reg,1);

			break;
		}
		case MMC_SIGNAL_VOLTAGE_180:{
			sdmmc_reg.bit.SC_SDMMC1_PVDD18POCSD0 = 0x2;
			sdmmc_reg.bit.SC_SDMMC1_PVDD18POCSD1 = 0x2;
			sdmmc_reg.bit.SC_SDMMC1_PVDD18POCSD2 = 0x2;

			bst_sip_special_address_rw(0x300011c0,sdmmc_reg.reg,1);
			break;
		}
		case MMC_SIGNAL_VOLTAGE_120:{

			break;
		}
		default:{

			break;
		}
	}

	//sdmmc_reg.reg = bst_sip_special_address_rw(0x300011c0,0,0);

	//printk("ios->signal_voltag:%d\n",ios->signal_voltage);
	//printk("ios->bst_set_pinctrl_iolength:%x\n",sdmmc_reg.reg);
	return;
}
#endif

static const struct sdhci_ops sdhci_dwcmshc_ops = {
	.set_clock = sdhci_set_bst_clock,
	.set_bus_width = sdhci_set_bus_width,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
	.get_min_clock = bst_get_min_clock,
	.get_max_clock = bst_get_max_clock,
	.reset = sdhci_bst_reset,
	.set_power = sdhci_bst_set_power,
	.set_timeout = sdhci_bst_timeout,
	.platform_execute_tuning = bst_sdhci_execute_tuning,
	//.set_pinctrl_iolength = bst_set_pinctrl_iolength,
	//.voltage_switch = sdhci_bst_voltage_switch,
};
static const struct sdhci_pltfm_data sdhci_dwcmshc_pdata = {
	.ops = &sdhci_dwcmshc_ops,
	.quirks = SDHCI_QUIRK_DELAY_AFTER_POWER |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN |
		  SDHCI_QUIRK_INVERTED_WRITE_PROTECT,
	.quirks2 = SDHCI_QUIRK2_BROKEN_DDR50 | SDHCI_QUIRK2_CLK_FROM_DTS |
		   SDHCI_QUIRK2_BROKEN_HS200 | SDHCI_QUIRK2_TUNING_WORK_AROUND,
};




#ifdef CONFIG_PM_SLEEP

static int dwcmshc_suspend(struct device *dev){
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);
	int ret;


	ret = sdhci_suspend_host(host);
	if (ret)
		return ret;


	if (!IS_ERR(pltfm_host->clk)){
		clk_disable_unprepare(pltfm_host->clk);
	}

	if (!IS_ERR(priv->bus_clk)) {
		clk_disable_unprepare(priv->bus_clk);
	}

	return ret;
}


static int dwcmshc_resume(struct device *dev){
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);
	int ret;

	if (!IS_ERR(pltfm_host->clk)){

		ret = clk_prepare_enable(pltfm_host->clk);
		if (ret)
			return ret;

	}

	if (!IS_ERR(priv->bus_clk)) {
		ret = clk_prepare_enable(priv->bus_clk);
		if (ret)
			return ret;
	}



	return sdhci_resume_host(host);
}

static SIMPLE_DEV_PM_OPS(dwcmshc_pmops, dwcmshc_suspend, dwcmshc_resume);


#endif


static int dwcmshc_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	struct dwcmshc_priv *priv;
	int err;


	host = sdhci_pltfm_init(pdev, &sdhci_dwcmshc_pdata,
				sizeof(struct dwcmshc_priv));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);


	
#if 1
	pltfm_host->clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(pltfm_host->clk)) {
		err = PTR_ERR(pltfm_host->clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", err);
		goto err_clk;
	}
	
	if(!IS_ERR(pltfm_host->clk)){
		err = clk_prepare_enable(pltfm_host->clk);
		if (err)
			goto err_clk;
	}

	priv->bus_clk = devm_clk_get(&pdev->dev, "bus");
	if (IS_ERR(priv->bus_clk)){
		err = PTR_ERR(priv->bus_clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", err);
		goto err_clk;
	}

	if(!IS_ERR(priv->bus_clk)){
		err = clk_prepare_enable(priv->bus_clk);
		if (err)
			goto err_clk;
	}
#endif
	
	err = mmc_of_parse(host->mmc);
	if (err)
	 	goto err_clk;

	sdhci_get_of_property(pdev);
	device_property_read_u32(&pdev->dev, "port", &priv->channel);
	device_property_read_u32(&pdev->dev, "mmc_crm_reg_base", &priv->phy_crm_reg_base);
	//pr_err("%s: priv->channel =%d priv->phy_crm_reg_base=0x%x \n",__func__, priv->channel,priv->phy_crm_reg_base);
    // bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_CQE_CLK_DIV_CTRL,
	// bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_CQE_CLK_DIV_CTRL) &(~ 0x0100));//bit8:sdemmc_timer_div_ctrl_en = 0
    // bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_CQE_CLK_DIV_CTRL,
	// (bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_CQE_CLK_DIV_CTRL)&(~0xff))|0x20);//bit0-7:sdemmc_timer_div_ctrl = div //800/32=25M
    // bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_CQE_CLK_DIV_CTRL,
	// bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_CQE_CLK_DIV_CTRL) | 0x0100 );//bit8:sdemmc_timer_div_ctrl_en = 1
    //     /*default timer is 25m*/
    // bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_TIMER_DIV_CTRL, 
	// bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_TIMER_DIV_CTRL) &(~ 0x0100));//bit8:sdemmc_timer_div_ctrl_en = 0
    // bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_TIMER_DIV_CTRL, 
	// (bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_TIMER_DIV_CTRL)&(~0xff))|0x20 );//bit0-7:sdemmc_timer_div_ctrl = div //800/32=25M
    // bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_TIMER_DIV_CTRL,
	// bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_TIMER_DIV_CTRL) | 0x0100);//bit8:sdemmc_timer_div_ctrl_en = 1

	// bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_VOL_CTRL,
	// bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_VOL_CTRL) | 0x80);//sdemmc_host_reg_vol_stable

	if (sdhci_readl(host, SDHC_MHSC_VER_ID_R) != BST_SDMMC_VER_ID) {
		dev_err(&pdev->dev, "%s wrong ver id\n", __func__);
		goto err_clk;
	}



	err = sdhci_add_host(host);
	if (err)
		goto err_clk;

	return 0;

err_clk:
	if(!IS_ERR(pltfm_host->clk)){
		clk_disable_unprepare(pltfm_host->clk);
	}


	if(!IS_ERR(priv->bus_clk)){
		clk_prepare_enable(priv->bus_clk);
	}
	sdhci_pltfm_free(pdev);
	return err;
}

static int dwcmshc_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);

	sdhci_remove_host(host, 0);

	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(priv->bus_clk);

	sdhci_pltfm_free(pdev);

	return 0;
}

static const struct of_device_id sdhci_dwcmshc_dt_ids[] = {
	{ .compatible = "bst,dwcmshc-sdhci" },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_dwcmshc_dt_ids);

static struct platform_driver sdhci_dwcmshc_driver = {
	.driver	= {
		.name	= "sdhci-dwcmshc",
		.of_match_table = sdhci_dwcmshc_dt_ids,
		#ifdef CONFIG_PM_SLEEP
		.pm = &dwcmshc_pmops,
		#endif
	},
	.probe	= dwcmshc_probe,
	.remove	= dwcmshc_remove,
};
module_platform_driver(sdhci_dwcmshc_driver);

MODULE_DESCRIPTION("SDHCI platform driver for BST DWC MSHC");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");