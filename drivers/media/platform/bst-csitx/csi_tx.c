// SPDX-License-Identifier: GPL-2.0+
/*
 *    Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/dmaengine.h>
#include <linux/device.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/dma-buf.h>
#include <linux/cdev.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <linux/ktime.h>

#include "csi_tx.h"
#include "csitx_video.h"

// #define NEED_RX_CONFIG 1

#ifdef NEED_RX_CONFIG
#include "csi_cdphy.h"
#endif

#define DRIVER_NAME "csi-tx"
#define CSITX_CDEV 1

#ifdef CSITX_CDEV
dma_addr_t dmaaddr[4] = {0};
#define CSITX_DMA_BASE 0x24E00000

#define CSITX_MAJOR 176
#define CSITX_MINOR 0

#define CSITX_DMA_ADDR                    201
#define CSITX_CTRL_IOCTL                202
#define CSITX_TIMMING_IOCTL         203

static struct class *csitx_class;
static struct bst_csitx_device *s_txdev;
#endif

#define IS_IDI_SRC(id)                                                                                                                 \
    ((id) == CSITX_IDI_0 || CSITX_IDI_1 == (id) || CSITX_IDI_2 == (id))

static unsigned long last_time_us = 0;
static unsigned long curr_time_us = 0;
static unsigned long time_used = 0;

static int csitx_intr_times = 0;

static int debug;
module_param(debug, int, 0644);

#define dprintk(level, fmt, arg...)					\
	do {								\
		if (debug >= level)					\
			pr_info("[%s] : " fmt, __func__,		\
				## arg);				\
	} while (0)

void getVideoParams(unsigned int vic, videoParam_t *_vic_str)
{
    switch (vic) {
    case 5: //1920x1080 16:9
    {
        _vic_str->hactive = 1920;
        _vic_str->hblank = 1787;
        _vic_str->hfront = 1067;
        _vic_str->hsync = 10;
        _vic_str->hback = 710;
        _vic_str->vactive = 1080;
        _vic_str->vblank = 45;
        _vic_str->vfront = 4;
        _vic_str->vsync = 5;
        _vic_str->vback = 36;
        _vic_str->vpol = 1;
        _vic_str->hpol = 1;
        break;
    }
    case 6: //1920x1080 16:9    60 fps
    {
        _vic_str->hactive = 1920;
        _vic_str->hblank = 7010;//15645
        _vic_str->hfront = 5990;//15625
        _vic_str->hsync = 10;
        _vic_str->hback = 1010;
        _vic_str->vactive = 1080;
        _vic_str->vblank = 45;
        _vic_str->vfront = 4;
        _vic_str->vsync = 5;
        _vic_str->vback = 36;
        _vic_str->vpol = 1;
        _vic_str->hpol = 1;
        break;
    }
    case 7: //1920x1280 4:3
    {
        _vic_str->hactive = 1920;
        _vic_str->hblank = 13380;
        _vic_str->hfront = 13370;
        _vic_str->hsync = 5;
        _vic_str->hback = 5;
        _vic_str->vactive = 1280;
        _vic_str->vblank = 30;
        _vic_str->vfront = 8;
        _vic_str->vsync = 10;
        _vic_str->vback = 12;
        _vic_str->vpol = 1;
        _vic_str->hpol = 1;
        break;
    }
    case 8: //1920x1280 4:3
    {
        _vic_str->hactive = 1920;
        _vic_str->hblank = 13380;
        _vic_str->hfront = 13370;
        _vic_str->hsync = 5;
        _vic_str->hback = 5;
        _vic_str->vactive = 1280;
        _vic_str->vblank = 30;
        _vic_str->vfront = 8;
        _vic_str->vsync = 10;
        _vic_str->vback = 12;
        _vic_str->vpol = 1;
        _vic_str->hpol = 1;
        break;
    }
    case 9:  // RGB888 3840x2160P60  (after 60fps pass , the 40fps to be ajusted.)
    {
	_vic_str->hblank = 1440;
	_vic_str->hfront = 1056;
	_vic_str->hsync = 88;
	_vic_str->hback = 296;
	_vic_str->hactive = 3840;
	_vic_str->vblank = 90;
	_vic_str->vfront = 8;
	_vic_str->vsync = 10;
	_vic_str->vback = 72;
	_vic_str->vpol = 1;
	_vic_str->hpol = 1;
	_vic_str->vactive = 2160;
    }
    }
};

static inline void csitx_dev_iowrite32(struct bst_csitx_device *pcsitx_dev, u32 reg, u32 val)
{
    iowrite32(val, pcsitx_dev->csidev_base + reg);
}

static inline u32 csitx_dev_ioread32(struct bst_csitx_device *pcsitx_dev, u32 reg)
{
    u32 read_val = 0;

    read_val = ioread32(pcsitx_dev->csidev_base + reg);

    return read_val;
}

static inline void csitx_switch_iowrite32(struct bst_csitx_device *pcsitx_dev, u32 reg, u32 val)
{
    iowrite32(val, pcsitx_dev->switch_base + reg);
}

static inline u32 csitx_switch_ioread32(struct bst_csitx_device *pcsitx_dev, u32 reg)
{
    u32 read_val = 0;

    read_val = ioread32(pcsitx_dev->switch_base + reg);

    return read_val;
}

static inline void csitx_dphy_iowrite32(struct bst_csitx_device *pcsitx_dev, u32 reg, u32 val)
{
    iowrite32(val, pcsitx_dev->dphy_base + reg);
}

static inline u32 csitx_dphy_ioread32(struct bst_csitx_device *pcsitx_dev, u32 reg)
{
    u32 read_val = 0;

    read_val = ioread32(pcsitx_dev->dphy_base + reg);

    return read_val;
}

static inline void csitx_dma_iowrite32(struct bst_csitx_device *pcsitx_dev, u32 reg, u32 val)
{
    iowrite32(val, pcsitx_dev->dma_base + reg);
}

static inline u32 csitx_dma_ioread32(struct bst_csitx_device *pcsitx_dev, u32 reg)
{
    u32 read_val = 0;

    read_val = ioread32(pcsitx_dev->dma_base + reg);

    return read_val;
}

void csitx_dmac_int_enable(struct bst_csitx_device *pcsitx_dev, int enable)
{
    if (enable)
        iowrite32(0xFFFFFFDF, pcsitx_dev->tx_int_msk);
    else
        iowrite32(0xFFFFFFFF, pcsitx_dev->tx_int_msk);
}

static void mipi_csitx_dphy_write_control(struct bst_csitx_device *pcsitx_dev, u32 test_code, u32 test_write)
{
    //pr_err("%s enter %d", __func__, __LINE__);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_PHY0_TST_CTRL1, 0x100 << 8 | test_code);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_PHY0_TST_CTRL0, 0x2);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_PHY0_TST_CTRL0, 0x0);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_PHY0_TST_CTRL1, test_write);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_PHY0_TST_CTRL0, 0x2);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_PHY0_TST_CTRL0, 0x0);
    //pr_err("%s enter %d", __func__, __LINE__);
}


static int csitx_device_init_config(struct bst_csitx_device *pcsitx_dev)
{
    unsigned int regval, test02;
    unsigned int phy_if = 0;

    pr_err("%s enter %d", __func__, __LINE__);
    regval = csitx_dev_ioread32(pcsitx_dev, CSI2_HW_VERSION);
    dev_dbg(pcsitx_dev->dev, "hw version : %x\n", regval);

    switch (pcsitx_dev->num_lanes) {
    case 1:
        phy_if = DATA_1_LANE;
        break;
    case 2:
        phy_if = DATA_2_LANE;
        break;
    case 3:
        phy_if = DATA_3_LANE;
        break;
    case 4:
        phy_if = DATA_4_LANE;
        break;
    }
    phy_if |= 0x43 << 8;
    csitx_dev_iowrite32(pcsitx_dev, CSI2_PHY_IF_CFG, phy_if);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_PHY_ULPS_CTRL, 0);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_CLKMGR_CFG, 0x107);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_COMBO_PHY_MODE, 0);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_PHY_SWITCH_TIME, 0x40020);
    /* Initialize DPHY */
    switch (pcsitx_dev->lane_speed) {
    case 2500:
        test02 = 0x49;
        break;
    case 2400:
        test02 = 0x47;
        break;
    case 2000:
        test02 = 0xf;
        break;
    case 1500:
        test02 = 0x2c;
        break;
    case 1200:
        test02 = 0xb;
        break;
    case 1000:
        test02 = 0xa;
        break;
    case 800:
        test02 = 0x9;
        break;
    default:
        test02 = 0x47;
        break;
    }
    mipi_csitx_dphy_write_control(pcsitx_dev, 0x00, 0x0);
    mipi_csitx_dphy_write_control(pcsitx_dev, 0x01, 0x20);
    mipi_csitx_dphy_write_control(pcsitx_dev, 0x02, test02);

    /* Initialize the Controller */
    csitx_dev_iowrite32(pcsitx_dev, CSI2_INT_MASK_N_VPG, 0xFFFFFFFF);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_INT_MASK_N_IDI, 0xFFFFFFFF);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_INT_MASK_N_IPI, 0xFFFFFFFF);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_INT_MASK_N_PHY, 0xFFFFFFFF);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_INT_MASK_N_IDI_VCX, 0xFFFFFFFF);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_INT_MASK_N_DIAG0, 0xFFFFFFFF);

    csitx_dev_iowrite32(pcsitx_dev, CSI2_INT_ST_VPG, 0x3);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_INT_ST_IDI, 0x99);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_INT_ST_IPI, 0xb);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_INT_ST_PHY, 0xa);
    pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

static int csitx_device_reset(struct bst_csitx_device *pcsitx_dev)
{
    unsigned int regval;

    pr_err("%s enter %d", __func__, __LINE__);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_RESETN, 0xf);
    udelay(300);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_PHY_RSTZ, 0x1);
    udelay(300);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_PHY_RSTZ, 0x3);
    udelay(300);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_PHY_RSTZ, 0x7);
    udelay(1000);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_RESETN, 0xf);
    udelay(300);
    regval = csitx_dev_ioread32(pcsitx_dev, CSI2_PHY_RSTZ);
    regval = csitx_dev_ioread32(pcsitx_dev, CSI2_RESETN);
    csitx_dmac_int_enable(pcsitx_dev, 0);
    pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

static int csitx_device_config_tolp11(struct bst_csitx_device *pcsitx_dev)
{
    unsigned int regval;

    pr_err("%s enter %d", __func__, __LINE__);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_PHY_RSTZ, PHY_UNSHUTDOWNZ | PHY_UNRSTZ | PHY_ENABLECLK | PHY_FORCEPLL |
             PHY_FORCETXSTOPMODE);
    regval = csitx_dev_ioread32(pcsitx_dev, CSI2_PHY_STATUS);
    udelay(300);
    regval = csitx_dev_ioread32(pcsitx_dev, CSI2_PHY_STATUS);
    udelay(300);
    pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

static int csitx_device_start_send(struct bst_csitx_device *pcsitx_dev)
{
    unsigned int regval;

    pr_err("%s enter %d", __func__, __LINE__);
    /* set continus clock*/
    csitx_dev_iowrite32(pcsitx_dev, CSI2_LPCLK_CTRL, LPCLK_UNCONT);
    /* open deskew    */
    regval = csitx_dev_ioread32(pcsitx_dev, CSI2_PHY_CAL);
    regval |= (1 << 0);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_PHY_CAL, regval);
    udelay(100);
    /* close deskew    */
    regval = csitx_dev_ioread32(pcsitx_dev, CSI2_PHY_CAL);
    regval &= ~(1 << 0);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_PHY_CAL, regval);
    udelay(100);
    pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

static int csitx_device_vpg_setup(struct bst_csitx_device *pcsitx_dev)
{
    unsigned int regval;
    videoParam_t *disp_timing;

    disp_timing = kmalloc(sizeof(videoParam_t), GFP_KERNEL);
    pr_err("%s enter %d", __func__, __LINE__);
    udelay(1000);
    getVideoParams(pcsitx_dev->timming_index, disp_timing);
    /* First of all, off VPG */
    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_CTRL, 0);
    /* vpg mode colorbar */
    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_MODE_CFG, 0x0);

    /* raw14 datatype 0x2d */
    regval = (0x24 & GENMASK(5, 0));// | BIT(8);
    regval |= (0 & GENMASK(1, 0)) << 6;
    regval |= (0 & GENMASK(4, 2)) << 12;
    //regval |= (FRAME_INC_ONE_MODE | LINE_ONE_MODE);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_PKT_CFG, regval);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_PKT_SIZE, disp_timing->hactive);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_HSA_TIME, disp_timing->hsync);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_HBP_TIME, disp_timing->hback);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_HLINE_TIME, DISP_H_TOTAL(disp_timing));

    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_VSA_LINES, disp_timing->vsync);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_VBP_LINES, disp_timing->vback);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_VFP_LINES, disp_timing->vfront);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_ACT_LINES, disp_timing->vactive);

    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_MAX_FRAME_NUM, 0x100);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_START_LINE_NUM, 1);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_STEP_LINE_NUM, 1);
    /* Frame blanking period */
    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_BK_LINES, 0x0);
    /* Video pattern generator enable signal. */
    csitx_dev_iowrite32(pcsitx_dev, CSI2_VPG_CTRL, 1);

    kfree(disp_timing);
    pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

static unsigned int get_ipi_pkt_cfg(struct bst_csitx_device *pcsitx_dev, u32 vc_id)
{
    u32 ipi_pkt_cfg = 0x0;

    ipi_pkt_cfg |= (pcsitx_dev->data_type) & GENMASK(5, 0);
    /* [7:6] - ipi_vc[1:0] */
    ipi_pkt_cfg |= (vc_id & GENMASK(1, 0)) << 6;
    ipi_pkt_cfg |= FRAME_INC_ONE_MODE | LINE_ONE_MODE;
    /* [14:12] - ipi_vc[4:2] */
    //ipi_pkt_cfg |= ((vc_id & GENMASK(4, 2)) >> 2) << 12;
    ipi_pkt_cfg |= (pcsitx_dev->ipi_mode) << 16;
    /* line synchronization packets mode */
    //ipi_pkt_cfg |= BIT(8);
    return ipi_pkt_cfg;
}

static int csitx_device_multi_ipi_setup(struct bst_csitx_device *pcsitx_dev, int channel)
{
    videoParam_t *disp_timing;
    u32 hsa_hbp_time, lp_time, line;

    // pr_err("%s enter %d", __func__, __LINE__);
    if ((pcsitx_dev->vc_enable & 0xe) == 0)
        return 0;

    disp_timing = kmalloc(sizeof(videoParam_t), GFP_KERNEL);
    getVideoParams(pcsitx_dev->timming_index, disp_timing);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_MT_IPI_CFG, 0x1);//0 no timming

    csitx_dev_iowrite32(pcsitx_dev, CSI2_MT_IPI_DF_TIME, disp_timing->hactive + disp_timing->hback + disp_timing->hfront +
            disp_timing->hsync);
    /* IPI0 */

    if (pcsitx_dev->vc_enable & BIT(channel))
        csitx_dev_iowrite32(pcsitx_dev, CSI2_MT_IPI_TRANS_BASE(channel), channel + 1);

    hsa_hbp_time = (disp_timing->hsync + disp_timing->hback) & GENMASK(18, 0);
    lp_time = disp_timing->hactive + 10;//DIV_ROUND_UP(disp_timing->hactive + 6, 4) + 10;

    if (pcsitx_dev->vc_enable & BIT(channel)) {
        csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_HSA_HBP_TIME_BASE(channel), hsa_hbp_time);
        csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_LP_TIME_BASE(channel), lp_time);
    }

    /* IPI2-4 lines */
    line = disp_timing->vsync + disp_timing->vactive + disp_timing->vback +
        disp_timing->vfront;

    if ((channel > 0) && (pcsitx_dev->vc_enable & BIT(channel))) {
        /* IPI2-4 */
        csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_PKT_CFG_BASE(channel), get_ipi_pkt_cfg(pcsitx_dev, channel));
        /* IPI2-4 VIDEO Config */
        csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_PIXELS_BASE(channel), disp_timing->hactive);
        csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_MAX_FRAME_NUM_BASE(channel), 0x100);
        /* IPI2-4 start line num */
        csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_START_LINE_NUM_BASE(channel), 0x1);
        /* IPI2-4 start step line num */
        csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_STEP_LINE_NUM_BASE(channel), 0x1);
        // pr_err("%s %d CSI2_IPI_LINES_BASE %x line %d",__func__, __LINE__, CSI2_IPI_LINES_BASE(channel), line);
        csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_LINES_BASE(channel), line);
        csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_DATA_SEND_START_BASE(channel), disp_timing->hactive);
    }
    kfree(disp_timing);
    // pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

static int csitx_device_ipi_setup(struct bst_csitx_device *pcsitx_dev)
{
    videoParam_t *disp_timing;
    u32 lines, val, tmp_val;
    u32 ipi_data_timing, ppi_div_ipi, ppi_data_timing;

    // pr_err("%s enter %d", __func__, __LINE__);
    disp_timing = kmalloc(sizeof(videoParam_t), GFP_KERNEL);

    udelay(1000);
    getVideoParams(pcsitx_dev->timming_index, disp_timing);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_PKT_CFG, get_ipi_pkt_cfg(pcsitx_dev, 0));
    csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_PIXELS, (disp_timing->hactive) & GENMASK(16, 0));

    csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_MAX_FRAME_NUM, 0x40);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_START_LINE_NUM, 0x1);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_STEP_LINE_NUM, 0x1);
    lines = disp_timing->vsync + disp_timing->vactive + disp_timing->vback + disp_timing->vfront;
    csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_LINES, lines);

    val = disp_timing->hactive;
    if ((pcsitx_dev->ipi_mode == STREAM_MODE_CUT_THROUGH) &&
            (pcsitx_dev->data_type == CSI_FORMAT_RGB888)) {
        ipi_data_timing = disp_timing->hactive;
        ppi_div_ipi = (1 * 8 * 1000) / 6667;
        ppi_data_timing =    ppi_div_ipi * ((6 + ipi_data_timing * 3) / (8 * 4 / 8));

        if (ipi_data_timing - ppi_data_timing <= 12)
            val = 12 + ppi_div_ipi * 10;
        if (ipi_data_timing - ppi_data_timing > 12)
            val = ipi_data_timing - ppi_data_timing + ppi_div_ipi * 10;
        val += 10;
    }

    csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_DATA_SEND_START, val);

    tmp_val = val;
    val = (disp_timing->hsync + disp_timing->hback) * 6667 / 8000;
    // pr_err("%s enter %d    CSI2_IPI_HSA_HBP_PPI_TIME val 0x%x", __func__, __LINE__, val);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_HSA_HBP_PPI_TIME, val);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_HLINE_PPI_TIME, 0);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_VSA_LINES, disp_timing->vsync);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_VBP_LINES, disp_timing->vback);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_VFP_LINES, disp_timing->vfront);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_ACT_LINES, disp_timing->vactive);

    /* Frame Blank */
    csitx_dev_iowrite32(pcsitx_dev, CSI2_IPI_FB_LINES, 0x40);

    csitx_dev_iowrite32(pcsitx_dev, CSI2_CL_BCM_SYNC, 0x3);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_CL_BCM23ACK_PPIAPB, 0x99);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_CL_BCM23ACK_IDIAPB, 0xb);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_CL_BCM23ACK_IPIAPB, 0xa);
    csitx_dev_iowrite32(pcsitx_dev, CSI2_DATA_SCRAMBLING, 0);
    kfree(disp_timing);
    // pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

static u32 csitx_switch_ipi_src_sel(u32 id)
{
    u32 ipi_src_sel = CSITX_DMA_SRC;

    switch (id) {
    case CSITX_DMA_SRC:
        ipi_src_sel = CSITX_DMA_SRC;
        break;
    case CSITX_IDI_0:
        ipi_src_sel = CSITX_IDI_0;
        break;
    case CSITX_IDI_1:
        ipi_src_sel = CSITX_IDI_1;
        break;
    case CSITX_IDI_2:
        ipi_src_sel = CSITX_IDI_2;
        break;
    case CSITX_ISP_DVP:
        ipi_src_sel = CSITX_ISP_DVP;
        break;
    default:
        pr_err("ipi src sel error!\n");
        break;
    }
    return ipi_src_sel;
}

static u32 mipi_csitx_idi_pixel_byte(u32 data_type)
{
    u32 pixel_bytes = 3;

    switch (data_type) {
    case CSI_FORMAT_RAW8:
    case CSI_FORMAT_RAW10:
    case CSI_FORMAT_RAW12:
    case CSI_FORMAT_RAW14:
    case CSI_FORMAT_RAW16:
    case CSI_FORMAT_YUV422_8:
        return 1;
    case CSI_FORMAT_RAW20:
        return 2;
    case CSI_FORMAT_YUV422_10:
        return 1;
    default:
        pr_err("data_type error!\n");
        break;
    }
    return pixel_bytes;
}

static u32 get_pixel_bytes(struct bst_csitx_device *pcsitx_dev, u32 ipi_src)
{
    u32 pixel_bytes = 3;

    switch (ipi_src) {
    case CSITX_DMA_SRC:
        if (pcsitx_dev->data_type == CSI_FORMAT_RGB888)
            pixel_bytes = 3;
        else
            pixel_bytes = 2;
        break;
    case CSITX_IDI_0:
    case CSITX_IDI_1:
    case CSITX_IDI_2:
        if (pcsitx_dev->data_type == CSI_FORMAT_YUV422_10)
            pixel_bytes = 2;
        else if (pcsitx_dev->data_type == CSI_FORMAT_RAW20)
            pixel_bytes = 2;
        else
            pixel_bytes = mipi_csitx_idi_pixel_byte(pcsitx_dev->data_type);
        break;
    case CSITX_ISP_DVP:
        if (pcsitx_dev->data_type == CSI_FORMAT_RGB888)
            pixel_bytes = 3;
        else
            pixel_bytes = 2;
        break;
    default:
        pr_err("[%s]csitx_type error!\n", __func__);
        break;
    }
    return pixel_bytes;
}

static int csitx_switch_config_ipi(struct bst_csitx_device *pcsitx_dev, u32 ipi_src, u32 ipi_id, videoParam_t *disp_timing)
{
    u32 valid_byte = 16, pixel_bytes = 3;
    u32 val, data_num, ram_h_size, ram_v_size;

    //pr_err("%s enter %d", __func__, __LINE__);
    if (IS_IDI_SRC(ipi_src)) {
        csitx_switch_iowrite32(pcsitx_dev, IDI_DECODE_HSIZE_BASE(ipi_id), DIV_ROUND_UP(disp_timing->hactive, 3) << 16);
        csitx_switch_iowrite32(pcsitx_dev, IDI_DECODE_VSIZE_BASE(ipi_id), disp_timing->vactive << 16);
    }
    switch (ipi_src) {
    case CSITX_IDI_0:
    case CSITX_IDI_1:
    case CSITX_IDI_2:
        valid_byte = 8;
        break;
    default:
        valid_byte = 16;
        break;
    }
    pixel_bytes = get_pixel_bytes(pcsitx_dev, ipi_src);
    val = disp_timing->hactive * pixel_bytes;
    ram_h_size = DIV_ROUND_UP(val, valid_byte);
    ram_v_size = disp_timing->vactive;

    csitx_switch_iowrite32(pcsitx_dev, IPI_VEDIO_NUM_BASE(ipi_id), ram_h_size << 16 | ram_v_size);
    data_num = DIV_ROUND_UP(val, 16);
    csitx_switch_iowrite32(pcsitx_dev, IPI_STRIDE_PITCH_BASE(ipi_id), data_num << 16 | data_num);
    csitx_switch_iowrite32(pcsitx_dev, IPI_WRITE_RESERVD2_ECO_BASE(ipi_id), 0x0);

    if (pcsitx_dev->data_type == CSI_FORMAT_RAW20)
        csitx_switch_iowrite32(pcsitx_dev, IPI_PIXEL_RESERVD1_ECO_BASE(ipi_id), 0x1);
    else
        csitx_switch_iowrite32(pcsitx_dev, IPI_PIXEL_RESERVD1_ECO_BASE(ipi_id), 0x2);

    // regval = csitx_switch_ioread32(pcsitx_dev,IPI_STREAM_MODE_REG);
    // regval |= (pcsitx_dev->ipi_mode << 1);
    // regval |= BIT(0);
    // if (pcsitx_dev->data_type == CSI_FORMAT_YUV422_8)
    //     regval |= BIT(3);
    // csitx_switch_iowrite32(pcsitx_dev, IPI_STREAM_MODE_REG, regval);
    // pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

static u32 mipi_csitx_ipi_config_data_type(struct bst_csitx_device *pcsitx_dev, u32 ipi_src_sel, videoParam_t *vic_str)
{
    u32 ram_th = 0;
    u32 pixel_byte = 2, valid_byte = 16;
    u32 ipi_config_val = 0;

    switch (pcsitx_dev->data_type) {
    case CSI_FORMAT_RGB888: {
        if (ipi_src_sel == CSITX_DMA_SRC) {
            ipi_config_val = BIT(5) | BIT(0);
            pixel_byte = 3;
            valid_byte = 16;
        } else if (IS_IDI_SRC(ipi_src_sel))
            ipi_config_val = BIT(0);
        else if (ipi_src_sel == CSITX_ISP_DVP) {
            ipi_config_val = BIT(12) | BIT(0);
            pixel_byte = 3;
            valid_byte = 9;
        }
    } break;
    case CSI_FORMAT_YUV422_8: {
        if (ipi_src_sel == CSITX_DMA_SRC) {
            ipi_config_val =
                (pcsitx_dev->uv_switch ? BIT(6) : BIT(4)) | BIT(0);
            pixel_byte = 2;
            valid_byte = 16;
        } else if (IS_IDI_SRC(ipi_src_sel)) {
            ipi_config_val = BIT(0) | BIT(7);
            pixel_byte = 2;
            valid_byte = 8;
        } else if (ipi_src_sel == CSITX_ISP_DVP) {
            ipi_config_val = BIT(15) | BIT(13) | BIT(0) |
                     (pcsitx_dev->uv_switch ? BIT(11) : 0);
            pixel_byte = 2;
            valid_byte = 16;
        }
    } break;
    case CSI_FORMAT_YUV422_10: {
        if (ipi_src_sel == CSITX_DMA_SRC) {
            ipi_config_val =
                (pcsitx_dev->uv_switch ? BIT(6) : BIT(4)) | BIT(0);
            pixel_byte = 2;
            valid_byte = 16;
        } else if (IS_IDI_SRC(ipi_src_sel)) {
            ipi_config_val = BIT(0) | BIT(8);
            pixel_byte = 2;
            valid_byte = 8;
        } else if (ipi_src_sel == CSITX_ISP_DVP) {
            ipi_config_val = BIT(15) | BIT(13) | BIT(0) |
                     (pcsitx_dev->uv_switch ? BIT(11) : 0);
            pixel_byte = 2;
            valid_byte = 16;
        }
    } break;
    case CSI_FORMAT_YUV420_8: {
        if (ipi_src_sel == CSITX_DMA_SRC)
            ipi_config_val = BIT(0);
        else if (IS_IDI_SRC(ipi_src_sel))
            ipi_config_val = BIT(0);
        else if (ipi_src_sel == CSITX_ISP_DVP) {
            ipi_config_val = BIT(15) | BIT(14) | BIT(0) |
                     (pcsitx_dev->uv_switch ? BIT(11) : 0);
            pixel_byte = 2;
            valid_byte = 16;
        }
    } break;
    case CSI_FORMAT_RAW8:
    case CSI_FORMAT_RAW10:
    case CSI_FORMAT_RAW12:
    case CSI_FORMAT_RAW14:
    case CSI_FORMAT_RAW16: {
        if (ipi_src_sel == CSITX_DMA_SRC)
            ipi_config_val = BIT(0);
        else if (IS_IDI_SRC(ipi_src_sel)) {
            ipi_config_val = BIT(0) | BIT(9);
            pixel_byte = 1;
            valid_byte = 8;
        } else if (ipi_src_sel == CSITX_ISP_DVP)
            ipi_config_val = BIT(0);
    } break;
    case CSI_FORMAT_RAW20: {
        if (ipi_src_sel == CSITX_DMA_SRC)
            ipi_config_val = BIT(0);
        else if (IS_IDI_SRC(ipi_src_sel)) {
            ipi_config_val = BIT(0) | BIT(10);
            pixel_byte = 2;
            valid_byte = 8;
        } else if (ipi_src_sel == CSITX_ISP_DVP)
            ipi_config_val = BIT(0);
    } break;
    default:
        break;
    }
    /* line and a half pixel and start transfer */
    ram_th = vic_str->hactive * pixel_byte * 3 / (2 * valid_byte);
    /* [29:17] - ram depth */
    ipi_config_val |= (ram_th << 17);
    /* parity_type */
    ipi_config_val |= BIT(1);

    return ipi_config_val;
}

static int csitx_switch_config_video(struct bst_csitx_device *pcsitx_dev, u32 ipi_src, u32 ipi_id, videoParam_t *disp_timing)
{
    unsigned int regval;
    // pr_err("%s enter %d", __func__, __LINE__);

    regval = csitx_switch_ioread32(pcsitx_dev, IPI_STREAM_MODE_OFFSET(ipi_id) + IPI_STREAM_MODE_REG);
    regval |= (pcsitx_dev->ipi_mode << 1);
    if (IS_IDI_SRC(ipi_src)) {
        if (pcsitx_dev->data_type == CSI_FORMAT_RAW16)
            regval |= (0x2C << 4);
        else
            regval |= (pcsitx_dev->data_type << 4);
    }
    regval |= BIT(0);
    if (pcsitx_dev->data_type == CSI_FORMAT_YUV422_8)
        regval |= BIT(3);
    csitx_switch_iowrite32(pcsitx_dev, IPI_STREAM_MODE_OFFSET(ipi_id) + IPI_STREAM_MODE_REG, regval);

    /* Frame count */
    csitx_switch_iowrite32(pcsitx_dev, IPI_FRAME_COUNT_OFFSET(ipi_id) + IPI_FRAME_COUNT_REG, 0x1);
    /* fifo threshode reg */
    csitx_switch_iowrite32(pcsitx_dev, IPI_FIFO_THRESHODE_OFFSET(ipi_id) + IPI_FIFO_THRESHODE_REG, 300);

    csitx_switch_iowrite32(pcsitx_dev, IPI_H_SYNC_CFG_OFFSET(ipi_id) + IPI_H_SYNC_CFG, disp_timing->hsync);
    csitx_switch_iowrite32(pcsitx_dev, IPI_FP_ACTIVE_OFFSET(ipi_id) + IPI_H_ACTIVE_CFG, disp_timing->hactive);

    csitx_switch_iowrite32(pcsitx_dev, IPI_H_BP_CFG_OFFSET(ipi_id) + IPI_H_BP_CFG, disp_timing->hback);
    csitx_switch_iowrite32(pcsitx_dev, IPI_FP_ACTIVE_OFFSET(ipi_id) + IPI_H_FP_CFG, disp_timing->hfront);

    csitx_switch_iowrite32(pcsitx_dev, IPI_V_SYNC_CFG_OFFSET(ipi_id) + IPI_V_SYNC_CFG, disp_timing->vsync);
    csitx_switch_iowrite32(pcsitx_dev, IPI_FP_ACTIVE_OFFSET(ipi_id) + IPI_V_ACTIVE_CFG, disp_timing->vactive);

    csitx_switch_iowrite32(pcsitx_dev, IPI_FP_ACTIVE_OFFSET(ipi_id) + IPI_V_FP_CFG, disp_timing->vfront);
    csitx_switch_iowrite32(pcsitx_dev, IPI_V_BP_CFG_OFFSET(ipi_id) + IPI_V_BP_CFG, disp_timing->vback);

    regval = 0;//csitx_switch_ioread32(pcsitx_dev, IPI_CONTROL_OFFSET(ipi_id) + IPI_CONTROL_REG);
    regval |= mipi_csitx_ipi_config_data_type(pcsitx_dev, ipi_src, disp_timing);
    pr_err("%s    %d    IPI_CONTROL val %x", __func__, __LINE__, regval);
    csitx_switch_iowrite32(pcsitx_dev, IPI_CONTROL_OFFSET(ipi_id) + IPI_CONTROL_REG, regval);
    // pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

static u32 mipi_csitx_clk_sel(u32 src)
{
    u32 clk_sel = 0x0;

    /* [15:10] */
    switch (src) {
    case CSITX_SRC_NONE:
        pr_err("select --- CSITX_SRC_NONE ---\r\n");
        break;
    case CSITX_DMA_SRC:
        clk_sel = 0x0;
        break;
    case CSITX_IDI_0:
        clk_sel = 0x12;
        break;
    case CSITX_IDI_1:
        clk_sel = 0x16;
        break;
    case CSITX_IDI_2:
        clk_sel = 0x1a;
        break;
    case CSITX_ISP_DVP:
        clk_sel = 0x15;
        break;
    default:
        pr_err("clk sel not exist!\n");
        break;
    }
    return clk_sel << 10;
}

static int csitx_switch_config(struct bst_csitx_device *pcsitx_dev)
{
    videoParam_t *disp_timing = kmalloc(sizeof(videoParam_t), GFP_KERNEL);
    int i = 0;
    unsigned int regval;

    // pr_err("%s enter %d", __func__, __LINE__);
    if (!disp_timing) {
        pr_err("Malloc Failed!\n");
        return -1;
    }
    getVideoParams(pcsitx_dev->timming_index, disp_timing);

    csitx_switch_iowrite32(pcsitx_dev, SOURCE_CONTROL, csitx_switch_ipi_src_sel(CSITX_DMA_SRC));

    regval = csitx_switch_ioread32(pcsitx_dev, DMA_ST_START);
    for (i = 0; i < 4; i++)
        if (pcsitx_dev->vc_enable & BIT(i)) {
            regval |= BIT(i);
            csitx_switch_config_ipi(pcsitx_dev, CSITX_DMA_SRC, i, disp_timing);
            csitx_switch_config_video(pcsitx_dev, CSITX_DMA_SRC, i, disp_timing);
        }

    pr_err("%s %d DMA_ST_START val %x", __func__, __LINE__, regval);
    csitx_switch_iowrite32(pcsitx_dev, DMA_ST_START, regval);
    csitx_switch_iowrite32(pcsitx_dev, DMA_FRAMEDONE_INTR_MASK, 0);
    csitx_switch_iowrite32(pcsitx_dev, IPI_SYNC_POLARITY, 0);

    kfree(disp_timing);
    // pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

static int csitx_dphy_config(struct bst_csitx_device *pcsitx_dev, u32 ipi_src)
{
    unsigned int regval;
    int dphy_sel = 0;

    pr_err("%s enter %d", __func__, __LINE__);
    regval = csitx_dphy_ioread32(pcsitx_dev, DSI_CSITX_ADDR_START0);
    regval = csitx_dphy_ioread32(pcsitx_dev, DSI_CSITX_ADDR_START1);
    regval = csitx_dphy_ioread32(pcsitx_dev, DSI_CSITX_ADDR_START2);
    regval = csitx_dphy_ioread32(pcsitx_dev, DSI_CSITX_ADDR_END0);
    regval = csitx_dphy_ioread32(pcsitx_dev, DSI_CSITX_ADDR_END1);
    regval = csitx_dphy_ioread32(pcsitx_dev, DSI_CSITX_ADDR_END2);
    regval = csitx_dphy_ioread32(pcsitx_dev, LOCAL_CTRL_CSITX_REG0);
    /* [7]:CSITX D-PHY0 */
    regval |= BIT(7);        // csitx use dphy0
    /*val &= ~BIT(0); */
    csitx_dphy_iowrite32(pcsitx_dev, LOCAL_CTRL_CSITX_REG0, regval);

    regval = ((0x0A << 13) | (0x20 << 5) | (0x1 << 4) | (0x1));
    csitx_dphy_iowrite32(pcsitx_dev, 0x10 * dphy_sel, regval);
    switch (pcsitx_dev->lane_speed) {
    case 800:
        regval = ((0x200 << 18) | (0x10 << 12) | (0xA << 6) | (0x8));
        break;
    case 950:
        regval = ((0x130 << 18) | (0xF << 12) | (0xA << 6) | (0x8));
        break;
    case 1000:
        regval = ((0x140 << 18) | (0xF << 12) | (0xA << 6) | (0x8));
        break;
    case 1200:
        regval = ((0x180 << 18) | (0xF << 12) | (0xA << 6) | (0x8));
        break;
    case 1250:
        regval = ((0x190 << 18) | (0xF << 12) | (0xA << 6) | (0x8));
        break;
    case 1500:
        regval = ((0x1E0 << 18) | (0xF << 12) | (0xA << 6) | (0x8));
        break;
    case 2000:
        regval = ((0x140 << 18) | (0x7 << 12) | (0xA << 6) | (0x8));
        break;
    case 2400:
        regval = ((0x180 << 18) | (0x7 << 12) | (0xA << 6) | (0x8));
        break;
    case 2500:
        regval = ((0x190 << 18) | (0x7 << 12) | (0xA << 6) | (0x8));
        break;
    }
    csitx_dphy_iowrite32(pcsitx_dev, 0x10 * dphy_sel + 0x04, regval);

    regval = ((0x1 << 5) | (0x0));
    csitx_dphy_iowrite32(pcsitx_dev, 0x10 * dphy_sel + 0x08, regval);
    udelay(0x5);

    regval = ((0x1 << 5) | (0x1 << 1) | (0x0));
    csitx_dphy_iowrite32(pcsitx_dev, 0x10 * dphy_sel + 0x08, regval);

    regval = ((0x1 << 5) | (0x0));
    csitx_dphy_iowrite32(pcsitx_dev, 0x10 * dphy_sel + 0x08, regval);

    regval = ((0x1 << 5) | (0x1 << 4) | (0x0));
    csitx_dphy_iowrite32(pcsitx_dev, 0x10 * dphy_sel + 0x08, regval);

    regval = ((0x1 << 5) | (0x0));
    csitx_dphy_iowrite32(pcsitx_dev, 0x10 * dphy_sel + 0x08, regval);

    regval = csitx_dphy_ioread32(pcsitx_dev, DSI_CSITX_CLK_SEL);
    csitx_dphy_iowrite32(pcsitx_dev, DSI_CSITX_CLK_SEL, 0x3ff | mipi_csitx_clk_sel(ipi_src));

    udelay(300);
    pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

int csitx_dmac_stop(struct bst_csitx_device *pcsitx_dev)
{
    pr_err("%s enter %d", __func__, __LINE__);
    mutex_lock(&pcsitx_dev->config_lock);
    pcsitx_dev->open_vc--;
    if (pcsitx_dev->open_vc == 0)
        csitx_device_reset(pcsitx_dev);
    else
        pr_err("%s %d not close first!!", __func__, __LINE__);
    mutex_unlock(&pcsitx_dev->config_lock);
    pr_err("%s enter %d", __func__, __LINE__);

    return 0;
}

#ifdef NEED_RX_CONFIG

static int csitx_mipirx_config(struct bst_csitx_device *pcsitx_dev)
{
    struct bst_csi_device *csi_dev;

    csi_dev = kmalloc(sizeof(struct bst_csi_device), GFP_KERNEL);
    pr_err("%s enter %d", __func__, __LINE__);

    csi_dev->dev = pcsitx_dev->dev;
    csi_dev->ctrl_base = rx2_ctrl_base;
    csi_dev->top_base = rx2_top_base;
    csi_dev->phy_mode_cfg = 0;
    csi_dev->num_lanes = pcsitx_dev->num_lanes;
    csi_dev->lane_speed = pcsitx_dev->lane_speed;

    csi_cdphy_config_lanes(csi_dev);
    kfree(csi_dev);
    pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}
#endif

static void csitx_dma_channel_config(struct bst_csitx_device *pcsitx_dev, int channel, dma_addr_t dmaaddr, u32 imgSize, bool use_int)
{
    u32 sar_l, sar_h;
    u32 blk_size = 0;

    sar_l = dmaaddr & 0xffffffff;
    sar_h = (dmaaddr >> 32) & 0xffffffff;

    blk_size = imgSize / 16 - 1;
    csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CHX_SARL + CSITX_DMAC_CHX_BASE(channel), sar_l);
    csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CHX_SARH + CSITX_DMAC_CHX_BASE(channel), sar_h);

    csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CHX_BLK_TSL + CSITX_DMAC_CHX_BASE(channel), blk_size);
    if (use_int)
        csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CHX_CFGL + CSITX_DMAC_CHX_BASE(channel), 0x0);
    else
        csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CHX_CFGL + CSITX_DMAC_CHX_BASE(channel), 0x1);
    csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CHX_CFGH + CSITX_DMAC_CHX_BASE(channel), 0x7800001);

    csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CHX_CTLL + CSITX_DMAC_CHX_BASE(channel), 0x86400);
    csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CHX_CTLH + CSITX_DMAC_CHX_BASE(channel), 0x40ff8dc3);
    if (use_int)
        csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CHX_INTENABLE + CSITX_DMAC_CHX_BASE(channel), 0xffffffef);
    else
        csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CHX_INTENABLE + CSITX_DMAC_CHX_BASE(channel), 0x0);
    csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CHX_INTSIGENABLE + CSITX_DMAC_CHX_BASE(channel), 0xffffffef);
}

static void csitx_dma_dmac_enable(struct bst_csitx_device *pcsitx_dev, u32 chan_enable)
{
    u32 start_reg = 0;

    start_reg = chan_enable << 8 | chan_enable;
    csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CFGREG, 0x3);
    csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CHENREG, start_reg);
}

static void csitx_dma_dmac_disable(struct bst_csitx_device *pcsitx_dev, u32 chan_disable)
{
    u32 start_reg = 0;

    start_reg = chan_disable << 8 | chan_disable;
    csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CHENREG + 4, start_reg);
    udelay(300);
    csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CFGREG, 0x0);
}

void csitx_dma_dmac_send(struct bst_csitx_device *pcsitx_dev)
{
    int i = 0;
    static dma_addr_t last_addr[4] = {0};
    u32 start_channel = 0;

    dprintk(7, "timing_index: %d\n", pcsitx_dev->timming_index);

	dprintk(7, "	pcsitx_dev->lane_speed:%d\n", pcsitx_dev->lane_speed);
    for (i = 0; i < 4; i++) {
        if (!(pcsitx_dev->vc_enable & BIT(i)))
            continue;
        start_channel |= BIT(i);
        spin_lock_irq(&pcsitx_dev->tx_video[i]->queued_lock);
        if (!list_empty(&pcsitx_dev->tx_video[i]->queued_bufs)) {
            struct csitx_dma_buffer *buf;

            buf = list_first_entry(&pcsitx_dev->tx_video[i]->queued_bufs,
                                struct csitx_dma_buffer, queue);
            list_del(&buf->queue);
            last_addr[i] = vb2_dma_contig_plane_dma_addr(&buf->buf.vb2_buf, 0);
            // pr_err("%s %d last_addr%d %lld",__func__,__LINE__, i, last_addr[i]);
            vb2_buffer_done(&buf->buf.vb2_buf, VB2_BUF_STATE_DONE);
        }
        spin_unlock_irq(&pcsitx_dev->tx_video[i]->queued_lock);

        if (last_addr[i] == 0)
            continue;
	dprintk(7, "pcsitx_dev->tx_video[i]->format.sizeimage: %u \n", pcsitx_dev->tx_video[i]->format.sizeimage);
        csitx_dma_channel_config(pcsitx_dev, i, last_addr[i], pcsitx_dev->tx_video[i]->format.sizeimage, true);
    }
    csitx_dma_dmac_enable(pcsitx_dev, start_channel);
}

int csitx_dmac_config(struct bst_csitx_device *pcsitx_dev)
{
    int i = 0;

    pr_err("%s enter %d", __func__, __LINE__);

    mutex_lock(&pcsitx_dev->config_lock);
    pcsitx_dev->open_vc++;
    csitx_dmac_int_enable(pcsitx_dev, 0);
    //need dphy reset & mipi rate config first
    if (pcsitx_dev->open_vc == pcsitx_dev->vc_num) {
        csitx_dphy_config(pcsitx_dev, CSITX_DMA_SRC);
        csitx_switch_config(pcsitx_dev);
        csitx_device_init_config(pcsitx_dev);

        if (pcsitx_dev->vc_enable & BIT(0))
            csitx_device_ipi_setup(pcsitx_dev);

        for (i = 0; i < 4; i++)
            if (pcsitx_dev->vc_enable & BIT(i))
                csitx_device_multi_ipi_setup(pcsitx_dev, i);

        csitx_device_reset(pcsitx_dev);
        csitx_device_config_tolp11(pcsitx_dev);
        //need mipi rx config here
#ifdef NEED_RX_CONFIG
        csitx_mipirx_config(pcsitx_dev);
#endif
        csitx_device_start_send(pcsitx_dev);
        if(dmaaddr[0] == 0)
            csitx_dma_dmac_send(pcsitx_dev);
        csitx_dmac_int_enable(pcsitx_dev, 1);
    } else
        pr_err("%s %d not open untill all video open !!! ", __func__, __LINE__);

    mutex_unlock(&pcsitx_dev->config_lock);
    // csitx_dmac_setup(pcsitx_dev);
    pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

static int bst_csitx_parse_dt(struct bst_csitx_device *pcsitx_dev)
{
    struct device_node *node = pcsitx_dev->dev->of_node;
    int ret;
    int vc_num = 0;
    int lane_speed;
    int lane_num;
    struct resource *iomem;

    if (!node)
        return -EINVAL;

    ret = of_property_read_u32(node, "lane-speed", &lane_speed);
    if (ret) {
        dev_err(pcsitx_dev->dev, "csitx find lane-speed error\n");
        return -2;
    }
    pcsitx_dev->lane_speed = lane_speed;

    ret = of_property_read_u32(node, "lane-num", &lane_num);
    if (ret) {
        dev_err(pcsitx_dev->dev, "csitx find lane-speed error\n");
        return -2;
    }
    pcsitx_dev->num_lanes = lane_num;

    dev_dbg(pcsitx_dev->dev, "num_lanes = %d\n", pcsitx_dev->num_lanes);

    ret = of_property_read_u32(node, "vc-num", &vc_num);
    if (ret) {
        dev_err(pcsitx_dev->dev, "csitx find vc-num error\n");
        return -2;
    }
    pcsitx_dev->vc_num = vc_num;

    dev_dbg(pcsitx_dev->dev, "num_vc = %d\n", pcsitx_dev->vc_num);

    iomem = platform_get_resource(pcsitx_dev->pdev, IORESOURCE_MEM, 0);
    if (IS_ERR_OR_NULL(iomem)) {
        ret = PTR_ERR_OR_ZERO(iomem);
        dev_err(pcsitx_dev->dev, "get IORESOURCE_MEM 0 return %d", ret);
    } else {
        dev_info(pcsitx_dev->dev,
             "memory region 0 start: 0x%08llX, end: 0x%08llX\n",
             iomem->start, iomem->end);
        pcsitx_dev->csidev_base =
            devm_ioremap_resource(pcsitx_dev->dev, iomem);
        if (IS_ERR_OR_NULL(pcsitx_dev->csidev_base)) {
            ret = PTR_ERR_OR_ZERO(pcsitx_dev->csidev_base);
            dev_err(pcsitx_dev->dev,
                "Failed to remap csitx device base: %d\n", ret);
        }
    }

    iomem = platform_get_resource(pcsitx_dev->pdev, IORESOURCE_MEM, 1);
    if (IS_ERR_OR_NULL(iomem)) {
        ret = PTR_ERR_OR_ZERO(iomem);
        dev_err(pcsitx_dev->dev, "get IORESOURCE_MEM 1 return %d", ret);
    } else {
        dev_info(pcsitx_dev->dev,
             "memory region 1 start: 0x%08llX, end: 0x%08llX\n",
             iomem->start, iomem->end);
        pcsitx_dev->switch_base =
            devm_ioremap_resource(pcsitx_dev->dev, iomem);
        if (IS_ERR_OR_NULL(pcsitx_dev->switch_base)) {
            ret = PTR_ERR_OR_ZERO(pcsitx_dev->switch_base);
            dev_err(pcsitx_dev->dev,
                "Failed to remap csitx switch base: %d\n", ret);
            return ret;
        }
    }

    iomem = platform_get_resource(pcsitx_dev->pdev, IORESOURCE_MEM, 2);
    if (IS_ERR_OR_NULL(iomem)) {
        ret = PTR_ERR_OR_ZERO(iomem);
        dev_err(pcsitx_dev->dev, "get IORESOURCE_MEM 2 return %d", ret);
    } else {
        dev_info(pcsitx_dev->dev,
             "memory region 2 start: 0x%08llX, end: 0x%08llX\n",
             iomem->start, iomem->end);
        pcsitx_dev->dphy_base =
            devm_ioremap_resource(pcsitx_dev->dev, iomem);
        if (IS_ERR_OR_NULL(pcsitx_dev->dphy_base)) {
            ret = PTR_ERR_OR_ZERO(pcsitx_dev->dphy_base);
            dev_err(pcsitx_dev->dev,
                "Failed to remap csitx dphy base: %d\n", ret);
            return ret;
        }
    }

    iomem = platform_get_resource(pcsitx_dev->pdev, IORESOURCE_MEM, 3);
    if (IS_ERR_OR_NULL(iomem)) {
        ret = PTR_ERR_OR_ZERO(iomem);
        dev_err(pcsitx_dev->dev, "get IORESOURCE_MEM 2 return %d", ret);
    } else {
        dev_info(pcsitx_dev->dev,
             "memory region 3 start: 0x%08llX, end: 0x%08llX\n",
             iomem->start, iomem->end);
        pcsitx_dev->dma_base =
            devm_ioremap_resource(pcsitx_dev->dev, iomem);
        if (IS_ERR_OR_NULL(pcsitx_dev->dma_base)) {
            ret = PTR_ERR_OR_ZERO(pcsitx_dev->dma_base);
            dev_err(pcsitx_dev->dev,
                "Failed to remap media dma base: %d\n", ret);
            return ret;
        }
    }
    return 0;
}


#ifdef CSITX_CDEV
// struct dma_chan *dma_chan[4];
// struct dma_async_tx_descriptor *dma_desc[4];

static int csitx_vpg_config(struct bst_csitx_device *pcsitx_dev)
{
    pr_err("%s enter %d", __func__, __LINE__);
    //need dphy reset & mipi rate config first
    csitx_dphy_config(pcsitx_dev, CSITX_SRC_NONE);
    csitx_device_init_config(pcsitx_dev);
    csitx_device_reset(pcsitx_dev);
    csitx_device_config_tolp11(pcsitx_dev);
    //need mipi rx config here
#ifdef NEED_RX_CONFIG
    csitx_mipirx_config(pcsitx_dev);
#endif
    csitx_device_start_send(pcsitx_dev);
    csitx_device_vpg_setup(pcsitx_dev);
    //dmac init here
    pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

static void csitx_dma_disable(void)
{
    int i = 0;
    u32 start_channel = 0;

    for (i = 0; i < s_txdev->vc_num ; i++)
        start_channel |= BIT(i);

    csitx_dma_dmac_disable(s_txdev, start_channel);
}

static void csitx_dma_enable(void)
{
    int i = 0;
    u32 start_channel = 0;
    videoParam_t *disp_timing = (videoParam_t *)kmalloc(sizeof(videoParam_t), GFP_KERNEL);

    getVideoParams(s_txdev->timming_index, disp_timing);
    printk(KERN_ERR"timing_index: %d\n", s_txdev->timming_index);

	printk(KERN_ERR"	_vic_str->hblank  : %u\n", disp_timing->hblank  );
	printk(KERN_ERR"	_vic_str->hfront  : %u\n", disp_timing->hfront  );
	printk(KERN_ERR"	_vic_str->hsync   : %u\n", disp_timing->hsync   );
	printk(KERN_ERR"	_vic_str->hback   : %u\n", disp_timing->hback   );
	printk(KERN_ERR"	_vic_str->hactive : %u\n", disp_timing->hactive );
	printk(KERN_ERR"	_vic_str->vblank  : %u\n", disp_timing->vblank  );
	printk(KERN_ERR"	_vic_str->vfront  : %u\n", disp_timing->vfront  );
	printk(KERN_ERR"	_vic_str->vsync   : %u\n", disp_timing->vsync   );
	printk(KERN_ERR"	_vic_str->vback   : %u\n", disp_timing->vback   );
	printk(KERN_ERR"	_vic_str->vpol    : %d\n", disp_timing->vpol    );
	printk(KERN_ERR"	_vic_str->hpol    : %d\n", disp_timing->hpol    );
	printk(KERN_ERR"	_vic_str->vactive : %u\n", disp_timing->vactive );

	printk(KERN_ERR"	pcsitx_dev->lane_speed:%d\n", s_txdev->lane_speed);
    for (i = 0; i < s_txdev->vc_num; i++){
        start_channel |= BIT(i);
        if(s_txdev->data_type == CSI_FORMAT_YUV422_8)
            csitx_dma_channel_config(s_txdev, i, dmaaddr[i], disp_timing->hactive*disp_timing->vactive*2, false);
        else
            csitx_dma_channel_config(s_txdev, i, dmaaddr[i], disp_timing->hactive*disp_timing->vactive*3, false);
    }
    csitx_dma_dmac_enable(s_txdev, start_channel);
    kfree(disp_timing);
}

int csitx_open(struct inode *inode, struct file *filp)
{
    pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

int csitx_release(struct inode *inode, struct file *filp)
{
    pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

static ssize_t csitx_read(struct file *filp, char __user *buf, size_t size, loff_t *poss)
{
    printk("frame total: %d\n", csitx_intr_times);
    printk("time used : %lu us, fps: %lu\n", time_used, 60 * 1000 / (time_used / 1000));
    return 0;
}

static ssize_t csitx_write(struct file *filp, const char __user *buf, size_t size, loff_t *poss)
{
    pr_err("%s enter %d", __func__, __LINE__);
    return 0;
}

static long csitx_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    unsigned long long dma_phyaddr;
    int i = 0;
    int val = 0;
    int ret = 0;

    switch (cmd) {
    case CSITX_DMA_ADDR:
        ret = copy_from_user(&dma_phyaddr, (unsigned long long *)arg, 8);
        if (ret) {
            pr_err("copy_from_user error!!");
            return 0;
        }
        pr_err("IOCTL dma_phyaddr: 0x%llx", dma_phyaddr);
        for (i = 0; i < 4; i++)
            dmaaddr[i] = dma_phyaddr;
    break;
    case CSITX_CTRL_IOCTL:
        ret = copy_from_user(&val, (unsigned int *)arg, 4);
        if (ret) {
            pr_err("copy_from_user error!!");
            return 0;
        }
        pr_err("IOCTL CSITX_CTRL_IOCTL: 0x%x", val);
        switch (val) {
        case 0://close tx
            mutex_lock(&s_txdev->open_lock);
            s_txdev->open_count --;
            if(s_txdev->open_count == 0){
                if(s_txdev -> tx_mode == 1)
                {
                    for(i = 0; i < s_txdev->vc_num; i++)
                        csitx_dmac_stop(s_txdev);
                    csitx_dma_disable();
                }
                csitx_device_reset(s_txdev);
                s_txdev->open_count = 0;
            }
            mutex_unlock(&s_txdev->open_lock);
        break;
        case 1://dma send
            mutex_lock(&s_txdev->open_lock);
            if(s_txdev->open_count == 0){
                s_txdev -> tx_mode = 1;
                s_txdev->vc_enable =  0x1;
                s_txdev->ipi_mode = 0;
                s_txdev->data_type = CSI_FORMAT_YUV422_8;
                for(i = 0; i < s_txdev->vc_num; i++)
                    csitx_dmac_config(s_txdev);
                csitx_dma_enable();            
            }
            s_txdev->open_count ++;
            mutex_unlock(&s_txdev->open_lock);
        break;
        case 2://vpg send
            mutex_lock(&s_txdev->open_lock);
            if(s_txdev->open_count == 0){
                s_txdev -> tx_mode = 0;
                s_txdev->vc_enable =  0x1;
                s_txdev->ipi_mode = 0;
                s_txdev->data_type = CSI_FORMAT_RGB888;
                csitx_vpg_config(s_txdev);
            }
            s_txdev->open_count ++;
            mutex_unlock(&s_txdev->open_lock);
        break;
        case 3://rgb888 dma send
            mutex_lock(&s_txdev->open_lock);
            if(s_txdev->open_count == 0){
                s_txdev->tx_mode = 1;
                s_txdev->vc_enable =  0x1;
                s_txdev->ipi_mode = 0;
                s_txdev->data_type = CSI_FORMAT_RGB888;
                for(i = 0; i < s_txdev->vc_num; i++)
                    csitx_dmac_config(s_txdev);
                csitx_dma_enable();            
            }
            s_txdev->open_count ++;
            mutex_unlock(&s_txdev->open_lock);
        break;
        }
    break;
    case CSITX_TIMMING_IOCTL:
        ret = copy_from_user(&val, (unsigned int *)arg, 4);
        if (ret) {
            pr_err("copy_from_user error!!");
            return 0;
        }
        pr_err("IOCTL CSITX_TIMMING_IOCTL: 0x%x", val);
        s_txdev->timming_index = val;
    break;
    }
    return 0;
}

static const struct file_operations csitx_fops = {
    .owner = THIS_MODULE,
    .write = csitx_write,
    .read = csitx_read,
    .open = csitx_open,
    .release = csitx_release,
    .unlocked_ioctl = csitx_ioctl,
};
#endif

static inline void csitx_dma_irq_disable(struct bst_csitx_device *pcsitx_dev)
{
    u32 val;

    val = csitx_dma_ioread32(pcsitx_dev, CSITX_DMAC_CFGREG);
    val &= ~0x2;
    csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CFGREG, val);
}

static inline void csitx_dma_irq_enable(struct bst_csitx_device *pcsitx_dev)
{
    u32 val;

    val = csitx_dma_ioread32(pcsitx_dev, CSITX_DMAC_CFGREG);
    val |= 0x2;
    csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CFGREG, val);
}

static unsigned long get_time_us(void) {
    ktime_t kt = ktime_get_real();
    unsigned long time_in_us = ktime_to_us(kt);

    return time_in_us;
}

static irqreturn_t bst_csitx_interrupt(int irq, void *dev_id)
{
    struct bst_csitx_device *pcsitx_dev = dev_id;
    unsigned short vc_done = 0;
    u32 regval;
    u32 i;

    csitx_intr_times += 1;

    if ((csitx_intr_times % 60) == 0) {
        curr_time_us = get_time_us();

        time_used = curr_time_us - last_time_us;
        last_time_us = curr_time_us;

    }
    /* Disable DMAC interrupts. We'll enable them after processing channels */
    //csitx_dma_irq_disable(pcsitx_dev);
    csitx_dmac_int_enable(pcsitx_dev, 0);
    //dev_err(chip->dev, "bst_axi_dma_csi_interrupt enter\n");

    /* Poll, clear and process every channel interrupt status */
    for (i = 0; i < 4; i++) {
        if (pcsitx_dev->vc_enable & BIT(i)) {
            regval = csitx_dma_ioread32(pcsitx_dev, CSITX_DMAC_CHX_INTSTATUS + CSITX_DMAC_CHX_BASE(i));
            if (regval & 0x2)
                vc_done |= BIT(i);
        }
    }

    if (pcsitx_dev->vc_enable == vc_done) {
        tasklet_schedule(&pcsitx_dev->task);
        for (i = 0; i < 4; i++) {
            if (pcsitx_dev->vc_enable & BIT(i))
                csitx_dma_iowrite32(pcsitx_dev, CSITX_DMAC_CHX_INTCLR + CSITX_DMAC_CHX_BASE(i), 0x2);
        }
    }
        /* Re-enable interrupts */
        // csitx_dma_irq_enable(pcsitx_dev);
    csitx_dmac_int_enable(pcsitx_dev, 1);
    return IRQ_HANDLED;
}

static void dma_complete(struct tasklet_struct *t)
{
    struct bst_csitx_device *pcsitx_dev = from_tasklet(pcsitx_dev, t, task);

    // dev_err(pcsitx_dev->dev, "dma_complete start send!!\n");
    csitx_dma_dmac_send(pcsitx_dev);
}


static int c1200_csitx_probe(struct platform_device *pdev)
{
    struct bst_csitx_device *pcsitx_dev;
    struct device *dev = &pdev->dev;
    int i = 0;
    struct bst_csitx_video *pcsitx_video;
    // struct reset_control *rst_contrl = NULL;
    int ret;

    pcsitx_dev = devm_kzalloc(dev, sizeof(struct bst_csitx_device), GFP_KERNEL);
    if (!pcsitx_dev)
        return -ENOMEM;

    pcsitx_dev->pdev = pdev;
    pcsitx_dev->dev = dev;
    pcsitx_dev->open_vc = 0;
    pcsitx_dev->open_count = 0;
    //mutex_init(&pcsitx_dev->mutex);

    ret = platform_get_irq(pdev, 0);
        if (ret < 0)
            return ret;
    pcsitx_dev->csitx_irq = ret;
    ret = devm_request_irq(pcsitx_dev->dev, pcsitx_dev->csitx_irq, bst_csitx_interrupt,
         IRQF_SHARED, DRIVER_NAME, pcsitx_dev);
    if (ret)
        return ret;

    tasklet_setup(&pcsitx_dev->task, dma_complete);
    bst_csitx_parse_dt(pcsitx_dev);

    pcsitx_dev->rst = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
    if (IS_ERR(pcsitx_dev->rst))
        return PTR_ERR(pcsitx_dev->rst);

    reset_control_reset(pcsitx_dev->rst);

    pcsitx_dev->aclk = devm_clk_get_optional(&pdev->dev, "aclk");
    if (IS_ERR(pcsitx_dev->aclk)) 
        return PTR_ERR(pcsitx_dev->aclk);

    clk_prepare_enable(pcsitx_dev->aclk);

    pcsitx_dev->hclk = devm_clk_get_optional(&pdev->dev, "hclk");
    if (IS_ERR(pcsitx_dev->hclk)) 
        return PTR_ERR(pcsitx_dev->hclk);

    clk_prepare_enable(pcsitx_dev->hclk);

    // csitx_sysfs_init(pcsitx_dev);
    platform_set_drvdata(pdev, pcsitx_dev);
    dma_set_mask_and_coherent(pcsitx_dev->dev, DMA_BIT_MASK(64));
    mutex_init(&pcsitx_dev->config_lock);
    mutex_init(&pcsitx_dev->open_lock);

    pcsitx_dev->tx_int_msk = ioremap(CSITX_INT_MASK, 4);

#ifdef CSITX_CDEV
    cdev_init(&pcsitx_dev->csitx_cdev, &csitx_fops);
    pcsitx_dev->csitx_cdev.owner = THIS_MODULE;
    s_txdev = pcsitx_dev;
    ret = cdev_add(&pcsitx_dev->csitx_cdev, MKDEV(CSITX_MAJOR, CSITX_MINOR), 1);
    if (ret) {
        dev_err(dev, "cdev_add() failed\n");
        return -1;
    }
    device_create(csitx_class, dev, MKDEV(CSITX_MAJOR, CSITX_MINOR), NULL, "%s", DRIVER_NAME);
#endif
    ret = v4l2_device_register(dev, &pcsitx_dev->v4l2_dev);
    if (ret < 0) {
        dev_err(dev, "V4L2 device registration failed (%d)\n", ret);
        return ret;
    }
    for (i = 0; i < 4; i++) {
        pcsitx_video = devm_kzalloc(dev, sizeof(struct bst_csitx_video), GFP_KERNEL);
        if (!pcsitx_video)
            return -ENOMEM;
        pcsitx_dev->tx_video[i] = pcsitx_video;
        csitx_video_init(pcsitx_dev, pcsitx_video, i);
    }

    csitx_dmac_int_enable(pcsitx_dev, 0);

    // reset dma -> csitx.
    csitx_dma_disable();
    csitx_device_reset(s_txdev);

    return 0;
}

static int c1200_csitx_remove(struct platform_device *pdev)
{
    int i = 0;
    struct bst_csitx_device *pcsitx_dev = platform_get_drvdata(pdev);

    for (i = 0; i < 4; i++) {
        if (pcsitx_dev->tx_video[i])
            csitx_video_cleanup(pcsitx_dev->tx_video[i]);
    }
    v4l2_device_unregister(&pcsitx_dev->v4l2_dev);
    mutex_destroy(&pcsitx_dev->config_lock);
    mutex_destroy(&pcsitx_dev->open_lock);
    return 0;
}

static struct platform_device_id c1200_csitx_id_table[] = {
    { .name = "c1200_csi_tx", .driver_data = 0 },
    {},
};
MODULE_DEVICE_TABLE(platform, c1200_csitx_id_table);

static const struct of_device_id c1200_csitx_of_table[] = {
    { .compatible = "bst,c1200_csi_tx" },
    {},
};
MODULE_DEVICE_TABLE(of, c1200_csitx_of_table);


#ifdef CONFIG_PM_SLEEP
static int c1200_csitx_suspend(struct device *dev)
{
    int i;

    mutex_lock(&s_txdev->open_lock);
    if (s_txdev->open_count != 0) {  // csitx is running.
        if (s_txdev->tx_mode == 1) {
            for (i = 0; i < s_txdev->vc_num; i++)
                csitx_dmac_stop(s_txdev);
            csitx_dma_disable();
        }
        csitx_device_reset(s_txdev);
    }
    mutex_unlock(&s_txdev->open_lock);
    return 0;
}

static int c1200_csitx_resume(struct device *dev)
{
    return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(c1200_csitx_pm_ops, c1200_csitx_suspend, c1200_csitx_resume);

static struct platform_driver c1200_csi_tx_driver = {
    .probe    = c1200_csitx_probe,
    .remove = c1200_csitx_remove,
    .id_table = c1200_csitx_id_table,
    .driver = {
        .name = "c1200-csi-tx",
        .of_match_table = c1200_csitx_of_table,
	.pm	= &c1200_csitx_pm_ops,
    },
};


#ifdef CSITX_CDEV
static int __init csitx_module_init(void)
{
    dev_t devt;
    int retval;

    csitx_class = class_create(THIS_MODULE, "csitx");

    devt = MKDEV(CSITX_MAJOR, CSITX_MINOR);
    retval = register_chrdev_region(devt, 1, "csi_tx");
    if (retval < 0)
        return retval;

    retval = platform_driver_register(&c1200_csi_tx_driver);
    if (retval)
        goto failed;

    return retval;

failed:
    unregister_chrdev_region(devt, 1);

    return retval;
}

static void __exit csitx_module_cleanup(void)
{
    dev_t devt = MKDEV(CSITX_MAJOR, CSITX_MINOR);

    class_destroy(csitx_class);
    platform_driver_unregister(&c1200_csi_tx_driver);
    unregister_chrdev_region(devt, 1);
}

module_init(csitx_module_init);
module_exit(csitx_module_cleanup);
#else
module_platform_driver(c1200_csi_tx_driver);
#endif

MODULE_DESCRIPTION("BST C1200 CSI TX driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
