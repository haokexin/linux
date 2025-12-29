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
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include "../sdhci-pltfm.h"
#include "bst-sdhci.h"
#include <linux/reset.h>

#include "SdemmcClient.h"


struct dwcmshc_priv {
	struct clk *bus_clk;
	struct reset_control	*rst;
	uint32_t channel;
	uint32_t  phy_crm_reg_base;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_1_8v;
	struct pinctrl_state *pinctrl_3_3v;
	uint32_t psm_id;
};

struct monitor_param_s 
{
    unsigned int ref_clkcnt;
    unsigned int freq_highclkcnt;
    unsigned int freq_threshold;
};

#ifndef CONFIG_SECOND_KERNEL
static SdemmcClient_t *m_client = NULL;
#endif
static SdemmcClient_data_t m_data={0};

#define BST_SDMMC_VER_ID	    0x3138302A
#define SDHCI_VENDOR_PTR_R	    0xE8

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
/* MMCM DRP */
#define SDHC_MMCM_DIV_REG  0x1020
#define DIV_REG_100_MHZ	   0x1145
#define DIV_REG_200_MHZ	   0x1083
#define SDHC_MMCM_CLKFBOUT 0x1024
#define CLKFBOUT_100_MHZ   0x0000
#define CLKFBOUT_200_MHZ   0x0080
#define SDHC_CCLK_MMCM_RST 0x00000001
#define DRIVER_NAME	   "sdhci_bst"




#define SDHCI_DUMP_BST(f, x...) \
	pr_err("%s: " DRIVER_NAME ": " f, mmc_hostname(host->mmc), ##x)
#define SD_3_3V 0
#define SD_1_8V 1

#ifdef CONFIG_SECOND_KERNEL
extern u8 msgbx_get_start_pid(void);
#endif

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


static int clk_monitor_param_cal(unsigned int freq, unsigned int ref, struct monitor_param_s* pClkMonitor, unsigned int ppm)
{
    unsigned int i;
    unsigned long long u64val;

    for(i=63; i>0; i--) {
        u64val = i;
        u64val *=freq;
        u64val *=100;
        if((u64val/ref) < 65536) {
            break;
        }
    }

    if(i==0) {
        pr_err("%s: invalid freq = %d, ref = %d\r\n", __func__, freq, ref);
        return -1;
    }

    pClkMonitor->ref_clkcnt = i;
    u64val = pClkMonitor->ref_clkcnt;
    u64val *=freq;
    u64val *=100;

    pClkMonitor->freq_highclkcnt = u64val/ref;

    u64val = pClkMonitor->freq_highclkcnt;
    u64val *= ppm;
    u64val += 1000000 -1;

    pClkMonitor->freq_threshold = u64val/1000000;

//    pr_info("jun, %s: freq %d, ref %d, ref_clkcnt 0x%x, freq_highclkcnt 0x%x, freq_threshold 0x%x\r\n", __func__, freq, ref, pClkMonitor->ref_clkcnt, pClkMonitor->freq_highclkcnt, pClkMonitor->freq_threshold);

    return 0;
}


#define CLOCK_MONITOR_PPM 600
#define CLOCK_MONITOR_FREQ 200000000

static void clk_monitor_sdemmc(u32 crm_base, int status, unsigned int clk)
{
    struct monitor_param_s sClkMonitor;
    struct monitor_param_s *pClkMonitor = &sClkMonitor;
	int ret = 0;
	void *mem_mapped = NULL;
	u32 val;

	mem_mapped = ioremap(crm_base, 0x1000);
	if (mem_mapped == NULL) {
		pr_err("%s: ioremap fail \r\n", __func__);
		return;
	}

	if (status) {
		ret = clk_monitor_param_cal(CLOCK_MONITOR_FREQ, CLOCK_MONITOR_FREQ, &sClkMonitor, CLOCK_MONITOR_PPM);	//200M
		if (ret) {
			pr_err("%s: clk_monitor_param_cal fail \r\n", __func__);
			iounmap(mem_mapped);
			return;
		}

		/*txclk freq monitor*/
		iowrite32(pClkMonitor->ref_clkcnt, ((u8 *)mem_mapped) + SDEMMC_CRM_TXCLK_FREQ_REF_CLKCNT);
		iowrite32(pClkMonitor->freq_highclkcnt, ((u8 *)mem_mapped) + SDEMMC_CRM_TXCLK_RREQ_HIGHCLKCNT);
		iowrite32(pClkMonitor->freq_threshold, ((u8 *)mem_mapped) + SDEMMC_CRM_TXCLK_RREQ_THRESHOLD);

		val = (u32)ioread32(((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN);
		val = (val&(~(0x10)))|(0x10);
		iowrite32(val, ((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN);
	} else {
		val = (u32)ioread32(((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN);
		val = (val&(~(0x10)))|(0x00);
		iowrite32(val, ((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN);
	}

	if (status) {
		ret = clk_monitor_param_cal(CLOCK_MONITOR_FREQ, CLOCK_MONITOR_FREQ, &sClkMonitor, CLOCK_MONITOR_PPM);	//200M
		if (ret) {
			pr_err("%s: clk_monitor_param_cal fail \r\n", __func__);
			iounmap(mem_mapped);
			return;
		}
		/*rxclk freq monitor*/
		iowrite32(pClkMonitor->ref_clkcnt, ((u8 *)mem_mapped) + SDEMMC_CRM_RXCLK_RREQ_REF_CLKCNT);
		iowrite32(pClkMonitor->freq_highclkcnt, ((u8 *)mem_mapped) + SDEMMC_CRM_RXCLK_RREQ_HIGHCLKCNT);
		iowrite32(pClkMonitor->freq_threshold, ((u8 *)mem_mapped) + SDEMMC_CRM_RXCLK_RREQ_THRESHOLD);

		val = (u32)ioread32(((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN);	//rxclk_freq_check_en
		val = (val&(~(0x08)))|(0x08);
		iowrite32(val, ((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN);
	} else {
		val = (u32)ioread32(((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN); //rxclk_freq_check_en disable
		val = (val&(~(0x08)))|(0x00);
		iowrite32(val, ((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN);
	}

	if (status) {	//not enable timerclk monitor
		ret = clk_monitor_param_cal(CLOCK_MONITOR_FREQ, CLOCK_MONITOR_FREQ, &sClkMonitor, CLOCK_MONITOR_PPM);	//timer clk 25M
		if (ret) {
			pr_err("%s: clk_monitor_param_cal fail \r\n", __func__);
			iounmap(mem_mapped);
			return;
		}

		/*timerclk freq monitor*/
		iowrite32(pClkMonitor->ref_clkcnt, ((u8 *)mem_mapped) + SDEMMC_CRM_TIMERCLK_RREQ_REF_CLKCNT);
		iowrite32(pClkMonitor->freq_highclkcnt, ((u8 *)mem_mapped) + SDEMMC_CRM_TIMERCLK_RREQ_HIGHCLKCNT);
		iowrite32(pClkMonitor->freq_threshold, ((u8 *)mem_mapped) + SDEMMC_CRM_TIMERCLK_RREQ_THRESHOLD);

		val = (u32)ioread32(((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN); 	//timerclk_freq_check_en
		val = (val&(~(0x04)))|(0x04);
		iowrite32(val, ((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN);
	} else {
		val = (u32)ioread32(((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN); //timerclk_freq_check_en disable
		val = (val&(~(0x04)))|(0x00);
		iowrite32(val, ((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN);		
	}


	if (0) {	//not use cqe clk , not enable cqe monitor
		ret = clk_monitor_param_cal(CLOCK_MONITOR_FREQ, CLOCK_MONITOR_FREQ, &sClkMonitor, CLOCK_MONITOR_PPM);	//200M
		if (ret) {
			pr_err("%s: clk_monitor_param_cal fail \r\n", __func__);
			iounmap(mem_mapped);
			return;
		}

		/*cqeclk freq monitor*/
		iowrite32(pClkMonitor->ref_clkcnt, ((u8 *)mem_mapped) + SDEMMC_CRM_CQECLK_RREQ_REF_CLKCNT);
		iowrite32(pClkMonitor->freq_highclkcnt, ((u8 *)mem_mapped) + SDEMMC_CRM_CQECLK_RREQ_HIGHCLKCNT);
		iowrite32(pClkMonitor->freq_threshold, ((u8 *)mem_mapped) + SDEMMC_CRM_CQECLK_RREQ_THRESHOLD);

		val = (u32)ioread32(((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN); 	//cqeclk_freq_check_en
		val = (val&(~(0x02)))|(0x02);
		iowrite32(val, ((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN);
	} else {
		val = (u32)ioread32(((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN); 	//cqeclk_freq_check_en disable
		val = (val&(~(0x02)))|(0x00);
		iowrite32(val, ((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN);
	}

	if (status) {
		ret = clk_monitor_param_cal(clk, CLOCK_MONITOR_FREQ, &sClkMonitor, CLOCK_MONITOR_PPM);	//400K->52M->200M
		if (ret) {
			pr_err("%s: clk_monitor_param_cal fail \r\n", __func__);
			iounmap(mem_mapped);
			return;
		}
		/*bclk freq monitor*/
		iowrite32(pClkMonitor->ref_clkcnt, ((u8 *)mem_mapped) + SDEMMC_CRM_BCLK_RREQ_REF_CLKCNT);
		iowrite32(pClkMonitor->freq_highclkcnt, ((u8 *)mem_mapped) + SDEMMC_CRM_BCLK_RREQ_HIGHCLKCNT);
		iowrite32(pClkMonitor->freq_threshold, ((u8 *)mem_mapped) + SDEMMC_CRM_BCLK_RREQ_THRESHOLD);

		val = (u32)ioread32(((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN); 	//bclk_freq_check_en
		val = (val&(~(0x01)))|(0x01);
		iowrite32(val, ((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN);
	} else {
		val = (u32)ioread32(((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN); 	//bclk_freq_check_en disable
		val = (val&(~(0x01)))|(0x00);
		iowrite32(val, ((u8 *)mem_mapped) + SDEMMC_CRM_FREQ_CHECK_EN);
	}

	iounmap(mem_mapped);
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
	unsigned int clk_monitor;

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

	clk_monitor = clk;
	val = bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_FREQ_CHECK_EN);
	val = val&(~(0x1f));
	bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_FREQ_CHECK_EN,val);

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
	val |=0x20;		//800/32=25M
//	val |=0x4;		// 800/4=200M
    bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_TIMER_DIV_CTRL,  val);//bit0-7:sdemmc_timer_div_ctrl = div //800/32=25M

	val = bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_TIMER_DIV_CTRL);
	val |= 1<<8;
    bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_TIMER_DIV_CTRL,val);//bit8:sdemmc_timer_div_ctrl_en = 1


	val = bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL);
	val &= ~(1<<11);
	bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL,val);

	if(priv->channel == 1){
	
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
		rx_reg.bit.rx_clk_phase_inner = 1;
		rx_reg.bit.rx_clk_sel_first = 0;
		rx_reg.bit.rx_clk_phase_out = 2;


		bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL,  rx_reg.reg);//bit0-7:sdemmc_timer_div_ctrl = div //800/32=25M

		//val = bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL);
		//val &= ~(0x7ff);
		//val |= 0x00000b13;
		//bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_RX_CLK_CTRL,  val);//bit0-7:sdemmc_timer_div_ctrl = div //800/32=25M
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

	if (clk_monitor == CLOCK_MONITOR_FREQ) {
		clk_monitor_sdemmc(priv->phy_crm_reg_base, priv->psm_id & (1U<<0), clk_monitor);
	}
}

void sdhci_set_bst_clock(struct sdhci_host *host, unsigned int clock)
{

	if (clock == 0)
	return;

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
	sdhci_set_power(host, mode, vdd);
	sdhci_writeb(host, 0xF, SDHCI_POWER_CONTROL);
	sdhci_writew(host, (sdhci_readw(host, MBIU_CTRL) & (~0xf)) | BURST_EN,
		     MBIU_CTRL);
}

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
	
	bst_write_phys_bst(priv->phy_crm_reg_base + SDEMMC_CRM_REG_WR_PROTECT,SDEMMC_CRM_REG_WR_PROTECT_MAGIC);//protected write opened
	
	for(i=0;i<SDHCI_TUNING_COUNT;i++){		
		bst_write_phys_bst(priv->phy_crm_reg_base + SDEMMC_CRM_DELAY_CHAIN_SEL,(1ul<<i)-1);		
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
	


	//printk("end0:%d start0:%d addr:%x,opcode:%x\n",end0,start0,priv->phy_crm_reg_base,opcode);
	//printk("end1:%d start1:%d addr:%x,opcode:%x\n",end1,start1,priv->phy_crm_reg_base,opcode);
	//pr_err("channel :%d ,tuning best:%d %lx\n",priv->channel,best,(1ul<<best)-1);

	bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_DELAY_CHAIN_SEL,(1ul<<best)-1);
	timeout = 20;
	bst_write_phys_bst(priv->phy_crm_reg_base + SDEMMC_CRM_REG_WR_PROTECT,0);//protected write close
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


static void sdhci_bst_voltage_switch(struct sdhci_host *host)
{
	struct mmc_ios *ios = &host->mmc->ios;
	struct sdhci_pltfm_host *pltfm_host;
	struct dwcmshc_priv *priv;
	int count=100;
	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);	
	if (IS_ERR(priv->pinctrl )) {
    	pr_err("no pinctrl\n");
    	return;
	}
	if( (host->mmc->caps2 & MMC_CAP2_NO_SDIO) &&  (host->mmc->caps2 & MMC_CAP2_NO_SD)) {
			pinctrl_select_state(priv->pinctrl, priv->pinctrl_1_8v);
			bst_write_phys_bst(priv->phy_crm_reg_base + SDEMMC_CRM_VOL_CTRL,0x1<<7);//vol stable power on
			//pr_debug("no need to voltage switch\n");
			return;
	}

	pr_debug("channel :%d ios->signal_voltag:%d timing:%d host->mmc->caps=0x%x\n",priv->channel ,ios->signal_voltage,ios->timing ,host->mmc->caps);	
	usleep_range(5000, 5500);



	switch(ios->signal_voltage){

		case MMC_SIGNAL_VOLTAGE_330:{	
			sdhci_writeb(host, SDHCI_POWER_330|SDHCI_POWER_ON, SDHCI_POWER_CONTROL);
			if (IS_ERR(priv->pinctrl_3_3v)) {
				pr_err("error find sd_pvdd3-3 pinmux \n");
				return ;
			}		
			pinctrl_select_state(priv->pinctrl, priv->pinctrl_3_3v);
			break;
		}
		case MMC_SIGNAL_VOLTAGE_180:{	

			sdhci_writeb(host, SDHCI_POWER_180|SDHCI_POWER_ON, SDHCI_POWER_CONTROL);
			if (IS_ERR(priv->pinctrl_1_8v)) {
				pr_err("error find sd_pvdd1-8 pinmux \n");
				return ;
			}

			pinctrl_select_state(priv->pinctrl, priv->pinctrl_1_8v);
			while( ! (bst_read_phys_bst(priv->phy_crm_reg_base+0x1c)&(0x1))&&(count))
			{
				usleep_range(10, 15);
				count --;
			}
			if(count <1)
			{
				pr_err("uhs voltage not stable");
			}

			break;
		}
		case MMC_SIGNAL_VOLTAGE_120:{

			break;
		}
		default:{

			break;
		}
	}
	bst_write_phys_bst(priv->phy_crm_reg_base + SDEMMC_CRM_VOL_CTRL,0x1<<7);//vol stable power on

}
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
	.voltage_switch = sdhci_bst_voltage_switch,
};
static const struct sdhci_pltfm_data sdhci_dwcmshc_pdata = {
	.ops = &sdhci_dwcmshc_ops,
	.quirks = SDHCI_QUIRK_DELAY_AFTER_POWER |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN |
		  SDHCI_QUIRK_INVERTED_WRITE_PROTECT,
        .quirks2 = SDHCI_QUIRK2_BROKEN_DDR50 | SDHCI_QUIRK2_TUNING_WORK_AROUND|SDHCI_QUIRK2_ACMD23_BROKEN,
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


#ifndef CONFIG_SECOND_KERNEL
static void sdhci_bst_config_ecc(struct dwcmshc_priv *priv, int status)
{
	u32 val;
	val = bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_ECC_STATUS);
//	pr_info ("jun: %s, ecc status ori: 0x%x \n", __func__, val);
	if (status) {
		val |= (1<<4);
	} else {
		val &= ~(1<<4);
	}
	bst_write_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_ECC_STATUS,val);
	val = bst_read_phys_bst(priv->phy_crm_reg_base+SDEMMC_CRM_ECC_STATUS);
//	pr_info ("jun: %s, ecc status cur: 0x%x \n", __func__, val);

}

static int sdhci_get_psmid_from_safety(uint8_t block_id_in ,uint8_t *block_id_out, uint32_t *psm_id_out){
	int ret = -1;
	int i = 0;
	uint8_t blockid = 0;
    sdemmc_UInt32Array4_t *psm_id=NULL;
	sdemmc_ErrorEnum_t err=0;

	if(!m_client)
		return ret;

	ret = m_client->sdemmc_client.fusaenable_method_sync(block_id_in,&blockid,&psm_id,&err,500,NULL);
    if(ret < 0|| err != 0){
        pr_err("%s,%d ret is %d,err = %d.", __func__, __LINE__,ret,(int)err);
        return -2;
    }

    *block_id_out = blockid;
    if(psm_id != NULL){
        for(i=0;i<4;i++)
            psm_id_out[i]=(*psm_id)[i];
    }

	return 0;
}

#endif

static int dwcmshc_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	struct dwcmshc_priv *priv;
	int err;

#ifndef CONFIG_SECOND_KERNEL
	int status = 1;
    ipc_inf_version_t version = {0};
	int ret=0;

    uint8_t block_id_in;
    uint8_t block_id_out;
    uint32_t psm_id_out[4];
#endif


	host = sdhci_pltfm_init(pdev, &sdhci_dwcmshc_pdata,
				sizeof(struct dwcmshc_priv));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);


	priv->rst = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (IS_ERR(priv->rst))
		return PTR_ERR(priv->rst);

	reset_control_assert(priv->rst);
	udelay(10);
	reset_control_deassert(priv->rst);


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
	
	err = mmc_of_parse(host->mmc);
	if (err)
	 	goto err_clk;

	sdhci_get_of_property(pdev);
	device_property_read_u32(&pdev->dev, "port", &priv->channel);
	device_property_read_u32(&pdev->dev, "mmc_crm_reg_base", &priv->phy_crm_reg_base);

#ifdef CONFIG_SECOND_KERNEL
	m_data.com_data.pid = msgbx_get_start_pid();
#endif

#ifndef CONFIG_SECOND_KERNEL
	m_client = SdemmcClient_init(&m_data);
	if (!m_client) {
		pr_err("SdemmcClient_init init client fail\n");
		return ret;
	}
        
	// client start
	ret = m_client->start();
    if(ret) {
        pr_err("SdemmcClient_init start failed\n");
		return ret;
    }
	// get version
	version = m_client->sdemmc_client.version();

    block_id_in = 0x6e;

    ret = sdhci_get_psmid_from_safety(block_id_in ,&block_id_out,psm_id_out);
    if(ret != 0) {
		pr_err("get psm_id from safety fail, set psm_id to 0xf\n");
		psm_id_out[0] = 0xf;      
    }

//	pr_info("jun, %s , psm_id: 0x%x \n", __func__, psm_id_out[0]);
	//mem ecc 
	status =  psm_id_out[0] & (1U<<3);
	sdhci_bst_config_ecc(priv, status);

	//disable clk monitor
	priv->psm_id = psm_id_out[0] & (~(1<<0));
#else
	priv->psm_id = 0;
#endif

	if(( host->mmc->caps & MMC_CAP_UHS) || ((host->mmc->caps2 & MMC_CAP2_NO_SDIO) &&  (host->mmc->caps2 && MMC_CAP2_NO_SD)))
	{


		priv->pinctrl = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(priv->pinctrl )) {
        dev_err(&pdev->dev, "error get pinmux\n");
        return 0;
		}
		priv->pinctrl_3_3v = pinctrl_lookup_state(priv->pinctrl, "sd_pvdd3-3");
		if (IS_ERR(priv->pinctrl_3_3v)) {
			dev_err(&pdev->dev, "error find sd_pvdd3-3 pinmux \n");
					return 0;
		}

		priv->pinctrl_1_8v = pinctrl_lookup_state(priv->pinctrl, "sd_pvdd1-8");
		if (IS_ERR(priv->pinctrl_1_8v)) {
			dev_err(&pdev->dev, "error find sd_pvdd1-8 pinmux \n");
					return 0;	
		}		
  	}





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
