// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2016 Linaro
 * Author: Christoffer Dall <christoffer.dall@linaro.org>
 */

#include <linux/cpu.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/scmi_protocol.h>
#include <linux/reset-controller.h>
#include <dt-bindings/reset/bst-resets-scmi.h>

struct reset_ {
    char name[64];
    int id;
    struct reset_controller_dev * rcdev;
};

struct reset_ greset[]={
    [RST_CMN_FMU_SW]={
        .name="rst_cmn_fmu_sw",
    },
    [RST_XGMAC_SW]={
        .name="rst_xgmac_sw",
    },
    [RST_CLK_MONITOR_SW]={
        .name="rst_clk_monitor_sw",
    },
    [RST_CPU0_MP4_SW]={
        .name="rst_cpu0_mp4_sw",
    },
    [RST_CPU1_MP4_SW]={
        .name="rst_cpu1_mp4_sw",
    },
    [RST_CPU_PERIP_SW]={
        .name="rst_cpu_perip_sw",
    },
    [RST_EDP_SW]={
        .name="rst_edp_sw",
    },
	[RST_HIFI_DSP_SW]={
        .name="rst_hifi_dsp_sw",
    },
	[RST_MEDIA_DMA_SW]={
        .name="rst_media_dma_sw",
    },
	[RST_MIPI_DSI_CSITX_SW]={
        .name="rst_mipi_dsi_csitx_sw",
    },
	[RST_MATRIX_SW]={
        .name="rst_matrix_sw",
    },
	[RST_SOC_DMA_SW]={
        .name="rst_soc_dma_sw",
    },   
	[RST_PCIE_SW]={
        .name="rst_pcie_sw",
    },
	[RST_UFS_SW]={
        .name="rst_ufs_sw",
    },
	[RST_PLL_SW]={
        .name="rst_pll_sw",
    },
	[RST_CPU_MP2_SW]={
        .name="rst_cpu_mp2_sw",
    },
	[RST_GPU_G78AE_SP_SW]={
        .name="rst_gpu_g78ae_sp_sw",
    },
	[RST_DB_DMA_SW]={
        .name="rst_db_dma_sw",
    },
	[RST_ISP_SW]={
        .name="rst_isp_sw",
    },
    [RST_MIPI0_CSIRX_SW]={
        .name="rst_mipi0_csirx_sw",
    },
    [RST_MIPI1_CSIRX_SW]={
        .name="rst_mipi1_csirx_sw",
    },
    [RST_MIPI2_CSIRX_SW]={
        .name="rst_mipi2_csirx_sw",
    },
    [RST_CV_SW]={
        .name="rst_cv_sw",
    },
    [RST_NET_SW]={
        .name="rst_net_sw",
    },
    [RST_CS_DMA_SW]={
        .name="rst_cs_dma_sw",
    },
	[RST_GPU_G78AE_SW]={
        .name="rst_gpu_g78ae_sw",
    },
	[RST_DISPLAY0_SW]={
        .name="rst_display0_sw",
    },
	[RST_DISPLAY1_SW]={
        .name="rst_display1_sw",
    },
	[RST_DISPLAY2_SW]={
        .name="rst_display2_sw",
    },
	[RST_CODEC0_SW]={
        .name="rst_codec0_sw",
    },   
	[RST_CODEC1_SW]={
        .name="rst_codec1_sw",
    },
	[RST_LVDS_0_SW]={
        .name="rst_lvds_0_sw",
    },
	[RST_JIAYU_SW]={
        .name="rst_jiayu_sw",
    },
	[RST_USB3_0_SW]={
        .name="rst_usb3_0_sw",
    },
	[RST_USB3_1_SW]={
        .name="rst_usb3_1_sw",
    },
	[RST_SDEMMC0_SW]={
        .name="rst_sdemmc0_sw",
    },
	[RST_SDEMMC1_SW]={
        .name="rst_sdemmc1_sw",
    },
	[RST_SOC_LSP0_SW]={
        .name="rst_soc_lsp0_sw",
    },
	[RST_SOC_LSP1_SW]={
        .name="rst_soc_lsp1_sw",
    },
	[RST_LPDDR5_0_SW]={
        .name="rst_lpddr5_0_sw",
    },
	[RST_LPDDR5_1_SW]={
        .name="rst_lpddr5_1_sw",
    },
	[RST_LSP0_I2C0_SMBUS_WCLK_SW]={
        .name="rst_lsp0_i2c0_smbus_wclk_sw",
    },
    [RST_LSP0_I2C1_SMBUS_WCLK_SW]={
        .name="rst_lsp0_i2c1_smbus_wclk_sw",
    },
    [RST_LSP0_I2C0_WCLK_SW]={
        .name="rst_lsp0_i2c0_wclk_sw",
    },
    [RST_LSP0_I2C1_WCLK_SW]={
        .name="rst_lsp0_i2c1_wclk_sw",
    },
    [RST_LSP0_I2C2_WCLK_SW]={
        .name="rst_lsp0_i2c2_wclk_sw",
    },
    [RST_LSP0_I2C3_WCLK_SW]={
        .name="rst_lsp0_i2c3_wclk_sw",
    },
    [RST_LSP0_SSI_M_WCLK_SW]={
        .name="rst_lsp0_ssi_m_wclk_sw",
    },
	[RST_LSP0_UART0_WCLK_SW]={
        .name="rst_lsp0_uart0_wclk_sw",
    },
	[RST_LSP0_UART1_WCLK_SW]={
        .name="rst_lsp0_uart1_wclk_sw",
    },
	[RST_LSP0_WDT0_WCLK_SW]={
        .name="rst_lsp0_wdt0_wclk_sw",
    },
	[RST_LSP0_WDT1_WCLK_SW]={
        .name="rst_lsp0_wdt1_wclk_sw",
    },
	[RST_LSP0_GPIO0_DBCLK_SW]={
        .name="rst_lsp0_gpio0_dbclk_sw",
    },   
	[RST_LSP0_GPIO1_DBCLK_SW]={
        .name="rst_lsp0_gpio1_dbclk_sw",
    },
	[RST_LSP0_GPIO2_DBCLK_SW]={
        .name="rst_lsp0_gpio2_dbclk_sw",
    },
	[RST_LSP0_GPIO3_DBCLK_SW]={
        .name="rst_lsp0_gpio3_dbclk_sw",
    },
	[RST_LSP0_I3C0_WCLK_SW]={
        .name="rst_lsp0_i3c0_wclk_sw",
    },
	[RST_LSP0_I3C1_WCLK_SW]={
        .name="rst_lsp0_i3c1_wclk_sw",
    },
	[RST_LSP0_XRAY_WCLK_SW]={
        .name="rst_lsp0_xray_wclk_sw",
    },
	[RST_LSP0_I2S_M0_WCLK_SW]={
        .name="rst_lsp0_i2s_m0_wclk_sw",
    },
	[RST_LSP0_I2S_M1_WCLK_SW]={
        .name="rst_lsp0_i2s_m1_wclk_sw",
    },
	[RST_LSP0_TIMER0_WCLK_SW]={
        .name="rst_lsp0_timer0_wclk_sw",
    },
	[RST_LSP0_TIMER1_WCLK_SW]={
        .name="rst_lsp0_timer1_wclk_sw",
    },
    [RST_LSP0_TIMER2_WCLK_SW]={
        .name="rst_lsp0_timer2_wclk_sw",
    },
	[RST_LSP0_TIMER3_WCLK_SW]={
        .name="rst_lsp0_timer3_wclk_sw",
    },
	[RST_LSP0_TIMER4_WCLK_SW]={
        .name="rst_lsp0_timer4_wclk_sw",
    },
	[RST_LSP0_TIMER5_WCLK_SW]={
        .name="rst_lsp0_timer5_wclk_sw",
    },
	[RST_LSP0_TIMER6_WCLK_SW]={
        .name="rst_lsp0_timer6_wclk_sw",
    },
	[RST_LSP0_TIMER7_WCLK_SW]={
        .name="rst_lsp0_timer7_wclk_sw",
    },
	[RST_LSP0_SSI_S_WCLK_SW]={
        .name="rst_lsp0_ssi_s_wclk_sw",
    },
	[RST_LSP0_SPDIF0_WCLK_SW]={
        .name="rst_lsp0_spdif0_wclk_sw",
    },
	[RST_LSP0_SPDIF1_WCLK_SW]={
        .name="rst_lsp0_spdif1_wclk_sw",
    },
	[RST_LSP1_I2C0_SMBUS_WCLK_SW]={
        .name="rst_lsp1_i2c0_smbus_wclk_sw",
    },
    [RST_LSP1_I2C1_SMBUS_WCLK_SW]={
        .name="rst_lsp1_i2c1_smbus_wclk_sw",
    },
    [RST_LSP1_I2C0_WCLK_SW]={
        .name="rst_lsp1_i2c0_wclk_sw",
    },
    [RST_LSP1_I2C1_WCLK_SW]={
        .name="rst_lsp1_i2c1_wclk_sw",
    },
    [RST_LSP1_I2C2_WCLK_SW]={
        .name="rst_lsp1_i2c2_wclk_sw",
    },
    [RST_LSP1_I2C3_WCLK_SW]={
        .name="rst_lsp1_i2c3_wclk_sw",
    },
    [RST_LSP1_SSI_M_WCLK_SW]={
        .name="rst_lsp1_ssi_m_wclk_sw",
    },
	[RST_LSP1_UART0_WCLK_SW]={
        .name="rst_lsp1_uart0_wclk_sw",
    },
	[RST_LSP1_UART1_WCLK_SW]={
        .name="rst_lsp1_uart1_wclk_sw",
    },
	[RST_LSP1_WDT0_WCLK_SW]={
        .name="rst_lsp1_wdt0_wclk_sw",
    },
	[RST_LSP1_WDT1_WCLK_SW]={
        .name="rst_lsp1_wdt1_wclk_sw",
    },
	[RST_LSP1_GPIO0_DBCLK_SW]={
        .name="rst_lsp1_gpio0_dbclk_sw",
    },   
	[RST_LSP1_GPIO1_DBCLK_SW]={
        .name="rst_lsp1_gpio1_dbclk_sw",
    },
	[RST_LSP1_GPIO2_DBCLK_SW]={
        .name="rst_lsp1_gpio2_dbclk_sw",
    },
	[RST_LSP1_GPIO3_DBCLK_SW]={
        .name="rst_lsp1_gpio3_dbclk_sw",
    },
	[RST_LSP1_I3C0_WCLK_SW]={
        .name="rst_lsp1_i3c0_wclk_sw",
    },
	[RST_LSP1_I3C1_WCLK_SW]={
        .name="rst_lsp1_i3c1_wclk_sw",
    },
	[RST_LSP1_XRAY_WCLK_SW]={
        .name="rst_lsp1_xray_wclk_sw",
    },
	[RST_LSP1_I2S_M0_WCLK_SW]={
        .name="rst_lsp1_i2s_m0_wclk_sw",
    },
	[RST_LSP1_I2S_M1_WCLK_SW]={
        .name="rst_lsp1_i2s_m1_wclk_sw",
    },
	[RST_LSP1_TIMER0_WCLK_SW]={
        .name="rst_lsp1_timer0_wclk_sw",
    },
	[RST_LSP1_TIMER1_WCLK_SW]={
        .name="rst_lsp1_timer1_wclk_sw",
    },
    [RST_LSP1_TIMER2_WCLK_SW]={
        .name="rst_lsp1_timer2_wclk_sw",
    },
	[RST_LSP1_TIMER3_WCLK_SW]={
        .name="rst_lsp1_timer3_wclk_sw",
    },
	[RST_LSP1_TIMER4_WCLK_SW]={
        .name="rst_lsp1_timer4_wclk_sw",
    },
	[RST_LSP1_TIMER5_WCLK_SW]={
        .name="rst_lsp1_timer5_wclk_sw",
    },
	[RST_LSP1_TIMER6_WCLK_SW]={
        .name="rst_lsp1_timer6_wclk_sw",
    },
	[RST_LSP1_TIMER7_WCLK_SW]={
        .name="rst_lsp1_timer7_wclk_sw",
    },
	[RST_LSP1_SSI_S_WCLK_SW]={
        .name="rst_lsp1_ssi_s_wclk_sw",
    },
	[RST_LSP1_SPDIF0_WCLK_SW]={
        .name="rst_lsp1_spdif0_wclk_sw",
    },
	[RST_LSP1_SPDIF1_WCLK_SW]={
        .name="rst_lsp1_spdif1_wclk_sw",
    },
   [RST_LSP0_TIMER_PCLK_SW]={
        .name="rst_lsp0_timer_pclk_sw",
    },
	[RST_LSP1_TIMER_PCLK_SW]={
        .name="rst_lsp1_timer_pclk_sw",
    },
};


static int reset_ops_set(void *data, u64 val){

    struct reset_ * reset = (struct reset_ *)data;

    const struct reset_control_ops *reset_ops = reset->rcdev->ops;

   // printk("reset id:%d val:%lld\n",reset->id,val);

    if(val == 0){
            reset_ops->assert(reset->rcdev,reset->id);
    }else if(val == 1) {
            reset_ops->deassert(reset->rcdev,reset->id);
    }else{
        reset_ops->reset(reset->rcdev,reset->id);
    }

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(debugfs_reset_fops, NULL, reset_ops_set, "0x%016llx\n");

void reset_debug_init(struct reset_controller_dev *rcdev){

    struct dentry		*debugfs_dir;
    struct dentry		*debugfs_sec_dir;
    int i = 0;

    debugfs_dir = debugfs_create_dir("reset",NULL);
    if(debugfs_dir){
        for(i=0;i<sizeof(greset)/sizeof(struct reset_);i++){
            debugfs_sec_dir = debugfs_create_dir(greset[i].name,debugfs_dir);
            greset[i].id=i;
            greset[i].rcdev = rcdev;
            debugfs_create_file("reset", 0444, debugfs_sec_dir, &greset[i],&debugfs_reset_fops);
        }
    }
}

EXPORT_SYMBOL_GPL(reset_debug_init);

void reset_debug_destroy(void){

}
