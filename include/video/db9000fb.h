#ifndef __DB9000FB_H__
#define __DB9000FB_H__

/*
 * linux/drivers/video/db9000fb.h
 *    -- Digital Blocks DB9000 LCD Controller Frame Buffer Device
 *  Copyright (C) 2010 Digital Blocks, Inc.
 *  Based on pxafb.h
 *  Copyright (C) 1999 Eric A. Thomas.
 *  Copyright (C) 2004 Jean-Frederic Clere.
 *  Copyright (C) 2004 Ian Campbell.
 *  Copyright (C) 2004 Jeff Lackey.
 *   Based on sa1100fb.c Copyright (C) 1999 Eric A. Thomas
 *  which in turn is
 *   Based on acornfb.c Copyright (C) Russell King.
 *
 *  2001-08-03: Cliff Brake <cbrake@acclent.com>
 *	 - ported SA1100 code to PXA
 *  2010-05-01: Guy Winter <gwinter@digitalblocks.com>
 *  - ported pxafb code to DB9000
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

#include <linux/byteorder/little_endian.h>
#include <linux/fb.h>
#include <asm/setup.h>
#include <mach/bitfield.h>

#define to_db9000fb(info)	container_of(info, struct db9000fb_info, fb)
#define TO_INF(ptr, member) container_of(ptr, struct db9000fb_info, member)

/*
 * LCD Controller Registers and Bits Definitions
 */
/* LCD Controller Control Register 1 */
#define DB9000_CR1	(0x000)
/* Horizontal Timing Register */
#define DB9000_HTR	(0x008)
/* Vertical Timing Register 1 */
#define DB9000_VTR1	(0x00C)
/* Vertical Timing Register 2 */
#define DB9000_VTR2	(0x010)
/* Pixel Clock Timing Register */
#define DB9000_PCTR	(0x014)
/* Interrupt Status Register */
#define DB9000_ISR	(0x018)
/* Interrupt Mask Register */
#define DB9000_IMR	(0x01C)
/* Interrupt Vector Register */
#define DB9000_IVR	(0x020)
/* Interrupt Scan Compare Register */
#define DB9000_ISCR	(0x024)
/* DMA Base Address Register */
#define DB9000_DBAR	(0x028)
/* DMA Current Address Register */
#define DB9000_DCAR	(0x02C)
/* DMA End Address Register */
#define DB9000_DEAR	(0x030)
/* PWM Frequency Register */
#define DB9000_PWMFR	(0x034)
/* PWM Duty Cycle Register */
#define DB9000_PWMDCR	(0x038)
/* DMA Frame Descriptor Branch Address Register */
#define DB9000_DFBAR	(0x03C)
/* DMA Frame Descriptor Last Address Register */
#define DB9000_DFLAR	(0x040)
/* DMA Horizontal and Vertical Timing Extension Register */
#define DB9000_HVTER	(0x044)
/* Core Identification Register */
#define DB9000_CIR	(0x1FC)
/* Palette Data Words */
#define DB9000_PALT	(0x200)
/* Palette Data Words - 2nd Port/2 Port TFT */
#define DB9000_PALT_2P	(0x400)

/* Overlay Window Registers  */
#define DB9000_NUM_OW	(16)
/* Overlay Window Enable Register */
#define DB9000_OWER	(0x600)

/* Overlay Window 0 Registers  */
/* Overlay Window X Start/End Register 0 */
#define DB9000_OWXSER0	(0x604)
/* Overlay Window Y Start/End Register 0 */
#define DB9000_OWYSER0	(0x608)
/* Overlay Window DMA BAR 0 */
#define DB9000_OWDBAR0	(0x60C)
/* Overlay Window DMA Current Addr Register 0 */
#define DB9000_OWDCAR0	(0x610)

/* Overlay Window 1 Registers  */
/* Overlay Window X Start/End Register 1 */
#define DB9000_OWXSER1	(0x614)
/* Overlay Window Y Start/End Register 1 */
#define DB9000_OWYSER1	(0x618)
/* Overlay Window DMA BAR 1 */
#define DB9000_OWDBAR1	(0x61C)
/* Overlay Window DMA Current Addr Register 1 */
#define DB9000_OWDCAR1	(0x620)

/* Multiple Memory Reads Request Register */
#define DB9000_MRR	(0xFFC)

/* Control Register 1, Offset 0x000 */
/* LCD Controller Enable */
#define DB9000_CR1_ENB	(1 << 0)
/* LCD Power Enable */
#define DB9000_CR1_LPE	(1 << 1)
/* LCD Bits per Pixel */
#define DB9000_CR1_BPP(x)  (((x) & 0x7) << 2)
/* RGB or BGR Format */
#define DB9000_CR1_RGB	(1 << 5)
/* Big or Little Endian Pixel Ordering */
#define DB9000_CR1_EPO	(1 << 6)
/* Big or Little Endian Byte Ordering  */
#define DB9000_CR1_EBO	(1 << 7)
/* Data Enable Polarity */
#define DB9000_CR1_DEP	(1 << 8)
/* Pixel Clock Polarity */
#define DB9000_CR1_PCP	(1 << 9)
/* Horizontal Sync Polarity */
#define DB9000_CR1_HSP	(1 << 10)
/* Vertical Sync Polarity */
#define DB9000_CR1_VSP	(1 << 11)
/* Output Pixel Select */
#define DB9000_CR1_OPS(x)	(((x) & 0x7) << 12)
/* Palette Load Source */
#define DB9000_CR1_PSS	(1 << 15)
/* FIFO DMA Request Words */
#define DB9000_CR1_FDW(x)	(((x) & 0x3) << 16)
/* LCD 1 or Port Select */
#define DB9000_CR1_LPS		(1 << 18)
/* Frame Buffer 24bpp Packed Word */
#define DB9000_CR1_FBP		(1 << 19)
/* DMA End Address Enable */
#define DB9000_CR1_DEE		(1 << 20)
/* DMA Frame Descriptor Re-start */
#define DB9000_CR1_DFR		(1 << 21)
/* DMA Frame Descriptor Branch Address Enable */
#define DB9000_CR1_DFB		(1 << 22)
/* DMA Frame Descriptor Enable */
#define DB9000_CR1_DFE		(1 << 23)
/* 1 bit per pixel */
#define DB9000_CR1_BPP_1bpp	(0)
/* 2 bits per pixel */
#define DB9000_CR1_BPP_2bpp	(1)
/* 4 bits per pixel */
#define DB9000_CR1_BPP_4bpp	(2)
/* 8 bits per pixel */
#define DB9000_CR1_BPP_8bpp	(3)
/* 16 bits per pixel */
#define DB9000_CR1_BPP_16bpp	(4)
/* 18 bits per pixel */
#define DB9000_CR1_BPP_18bpp	(5)
/* 14 bits per pixel */
#define DB9000_CR1_BPP_24bpp	(6)
/*  Pixel clock Rising-Edge */
#define DB9000_CR1_PixRsEdg	(DB9000_CR1_PCP*0)
/*  Pixel clock Falling-Edge */
#define DB9000_CR1_PixFlEdg	(DB9000_CR1_PCP*1)

/* Horizontal Timing Register, Offset 0x008 */
/* Horizontal Front Porch */
#define DB9000_HTR_HFP(x)	(((x) & 0xff) << 0)
/* Pixels per Line */
#define DB9000_HTR_PPL(x)	(((x) & 0xff) << 8)
/* Horizontal Back Porch */
#define DB9000_HTR_HBP(x)	(((x) & 0xff) << 16)
/* Horizontal Sync Width */
#define DB9000_HTR_HSW(x)	(((x) & 0xff) << 24)

/* Vertical Timing Register 1, Offset 0x00C */
/* Vertical Sync Width */
#define DB9000_VTR1_VSW(x)	(((x) & 0xff) << 0)
/* Vertical Front Porch */
#define DB9000_VTR1_VFP(x)	(((x) & 0xff) << 8)
/* Vertical Back Porch */
#define DB9000_VTR1_VBP(x)	(((x) & 0xff) << 16)

/* Vertical and Horizontal Timing Extension Register, Offset 0x044 */
/* Horizontal Front Porch Extension */
#define DB9000_HVTER_HFPE(x)	((((x) >> 8) & 0x3) << 0)
/* Horizontal Back Porch Extension */
#define DB9000_HVTER_HBPE(x)	((((x) >> 8) & 0x3) << 4)
/* Vertical Front Porch Extension */
#define DB9000_HVTER_VFPE(x)	((((x) >> 8) & 0x3) << 8)
/* Vertical Back Porch Extension */
#define DB9000_HVTER_VBPE(x)	((((x) >> 8) & 0x3) << 12)

/* DB9000 Revisions */
#define DB9000_REVISION_1_0		(0x00)
#define DB9000_REVISION_1_1		(0x01)
#define DB9000_REVISION_1_2		(0x02)
#define DB9000_REVISION_1_3		(0x03)
#define DB9000_REVISION_1_4		(0x04)
#define DB9000_REVISION_1_5		(0x05)
#define DB9000_REVISION_1_6		(0x06)
#define DB9000_REVISION_1_7		(0x07)
#define DB9000_REVISION_1_8		(0x08)
#define DB9000_REVISION_1_9		(0x09)
#define DB9000_REVISION_1_10		(0x0A)
#define DB9000_REVISION_1_11		(0x0B)
#define DB9000_REVISION_1_12		(0x0C)
#define DB9000_REVISION_1_13		(0x0D)
#define DB9000_REVISION_1_14		(0x0E)

/* Vertical Timing Register 2, Offset 0x010 */
/* Lines Per Panel */
#define DB9000_VTR2_LPP(x)	(((x) & 0xfff) << 0)

/* Pixel Clock Timing Register, Offset 0x014 */
/* Pixel Clock Divider */
#define DB9000_PCTR_PCD(x)	(((x) & 0xff) << 0)
/* Pixel Clock Divider Bypass */
#define DB9000_PCTR_PCB		(1 << 8)
/* Pixel Clock Input Select */
#define DB9000_PCTR_PCI		(1 << 9)
/* clock reset select */
#define DB9000_PCTR_PCR		(1 << 10)

/* Interrupt Status Register, Offset 0x018 */
#define DB9000_ISR_OFU	(1 << 0) /* Output FIFO Underrun */
#define DB9000_ISR_OFO	(1 << 1) /* Output FIFO Overrun */
#define DB9000_ISR_IFU	(1 << 2) /* Input FIFO Underrun */
#define DB9000_ISR_IFO	(1 << 3) /* Input FIFO Overrun */
#define DB9000_ISR_FER	(1 << 4) /* OR of OFU, OFO, IFU, IFO */
#define DB9000_ISR_MBE	(1 << 5) /* Master Bus Error */
#define DB9000_ISR_VCT	(1 << 6) /* Vertical Compare Triggered */
#define DB9000_ISR_BAU	(1 << 7) /* DMA Base Address Register Update to CAR */
#define DB9000_ISR_LDD	(1 << 8) /* LCD Controller Disable Done */
/* #ifdef CONFIG_AXI_BUS */
#define DB9000_ISR_ABL	(1 << 9) /* AXI Master - Read Burst Length Error */
#define DB9000_ISR_ARI	(1 << 10) /* AXI Master - Return ID Error */
#define DB9000_ISR_ARS	(1 << 11) /* AXI Master - Response Signal Error */
/* #endif */
#define DB9000_ISR_FBE	(1 << 12) /* Frame Descriptor - Bus Error */
#define DB9000_ISR_FNC	(1 << 13) /* Frame Descriptor - Node Complete */
#define DB9000_ISR_FLC	(1 << 14) /* Frame Descriptor - List Complete */

/* Interrupt Mask Register, Offset 0x01C */
#define DB9000_ISR_OFUM	(1 << 0)  /* Output FIFO Underrun - Mask */
#define DB9000_ISR_OFOM	(1 << 1)  /* Output FIFO Overrun - Mask */
#define DB9000_ISR_IFUM	(1 << 2)  /* Input FIFO Underrun - Mask */
#define DB9000_ISR_IFOM	(1 << 3)  /* Input FIFO Overrun - Mask */
#define DB9000_ISR_FERM	(1 << 4)  /* OR of OFU, OFO, IFU, IFO - Mask */
#define DB9000_ISR_MBEM	(1 << 5)  /* Master Bus Error - Mask */
#define DB9000_ISR_VCTM	(1 << 6)  /* Vertical Compare Triggered - Mask */
/* DMA Base Address Register Update to CAR - Mask */
#define DB9000_ISR_BAUM	(1 << 7)
#define DB9000_ISR_LDDM	(1 << 8)  /* LCD Controller Disable Done - Mask */
/* #ifdef CONFIG_AXI_BUS */
/* AXI Master - Read Burst Length Error - Mask */
#define DB9000_ISR_ABLM	(1 << 9)
/* AXI Master - Return ID Error - Mask */
#define DB9000_ISR_ARIM	(1 << 10)
/* AXI Master - Response Signal Error - Mask */
#define DB9000_ISR_ARSM	(1 << 11)
/* #endif */
#define DB9000_ISR_FBEM	(1 << 12) /* Frame Descriptor - Bus Error - Mask */
#define DB9000_ISR_FNCM	(1 << 13) /* Frame Descriptor - Node Complete - Mask */
#define DB9000_ISR_FLCM	(1 << 14) /* Frame Descriptor - List Complete - Mask */

/* Interrupt Scan Compare Register, Offset 0x024 */
#define DB9000_ISCR_VSC(x)		((x) & 0x7)

/* PWM Frequency Register, Offset 0x034 */
#define DB9000_PWMFR_PWM_FCD(x)	(((x) & 0xfffff) << 0)
#define DB9000_PWMFR_PWM_FCE	(1 << 20)
#define DB9000_PWMFR_PWM_FCI	(1 << 21)

/* PWM Duty Cycle Register, Offset 0x038 */
#define DB9000_PWMDCR_DCR(x)	((x) & 0xff)

/* Multiple Memory Reads Request Register, offset 0xFFC */
#define DB9000_MRR_MRR(x)	(((x) & 0x3) << 0)
#define DB9000_MRR_DEAR_MRR(x)	((x) & 0xFFFFFFFC)
#define DB9000_MRR_OUTST_0	0x0
#define DB9000_MRR_OUTST_2	0x1
#define DB9000_MRR_OUTST_4	0x2

/* End of Register description */


/*
 * These are the lcd controller states & actions for set_ctrlr_state
 */
#define C_DISABLE		(0)
#define C_ENABLE		(1)
#define C_DISABLE_CLKCHANGE	(2)
#define C_ENABLE_CLKCHANGE	(3)
#define C_REENABLE		(4)
#define C_DISABLE_PM		(5)
#define C_ENABLE_PM		(6)
#define C_STARTUP		(7)

#define LCD_PCLK_EDGE_RISE	(0 << 9)
#define LCD_PCLK_EDGE_FALL	(1 << 9)


#define DB9000FB_NAME	"CLCD-DB9000"

/*
 * Minimum X and Y resolutions
 */
#define MIN_XRES	16
#define MIN_YRES	64

#define NUM_OF_FRAMEBUFFERS 2
#define PALETTE_SIZE	(128 * 4)
#define PANEL_MAX_XRES 1920
#define PANEL_MAX_YRES 1080
#define PANEL_MAX_BPP 32


/* DB9000 LCD DMA Frame descriptor */
struct db9000fb_dma_descriptor {
	struct {
		u32	fdnav:1,
			fdna:30;
	};
	struct {
		u32	dbar_dear_ld_en:1,
			overlay_win_load:16,
			fnc:1,
			flc:1,
			fd_ower:12,
			fd_ower_ld_en:1;
	};
	u32 fd_dbar;
	u32 fd_dear;
	u32 fd_ow_dbar[DB9000_NUM_OW];
};

enum {
	PAL_NONE	= -1,
	PAL_STATIC	= 0,
	PAL_IN_FB	= 1,
/*	PAL_OV2		= 2, */
	PAL_MAX,
};

enum {
	DMA_BASE	= 0,
	DMA_DESCRIPTOR	= 1,
	DMA_MAX,
};

struct db9000fb_frame_buff {
	unsigned char *frame_buff;
	unsigned char *pixel_data_start;
};

/*
 * This structure describes the machine which we are running on.
 * It is set in linux/arch/arm/mach-spear13xx and used in the probe routine
 * of linux/drivers/video/db9000fb.c
 */
struct db9000fb_ctrl_info {
	u8	bpp;
	u32	cr1;
	u32	pctr;
	u32	dear;
	u32	pwmfr;
	u_int		cmap_greyscale : 1,
			depth:8,
			unused:23;
};

/*
 * This structure describes the machine which we are running on.
 * It is set in linux/arch/arm/mach-spear13xx/machine_name.c
 * and used in the probe routine of linux/drivers/video/db9000fb.c
 */
struct db9000fb_mach_info {
	struct fb_videomode *modes;
	struct db9000fb_ctrl_info *ctrl_info;
	unsigned int num_modes;

	unsigned int	lcd_conn;
	unsigned long	mem_size;
	unsigned long frame_buf_base;
	char		*def_mode;
	u_int		fixed_modes : 1,
			cmap_inverse:1,
			cmap_static:1,
			acceleration_enabled:1,
			unused:28;
	void (*clcd_mux_selection) (bool);
	struct clk *bus_clk;
	struct clk *pixel_clk;
	/* ignore_cpufreq_notification is > 0 if cpu and clcd uses different pll */
	unsigned int ignore_cpufreq_notification;
};

struct db9000fb_info {
	struct fb_info		fb;
	struct device		*dev;
	struct platform_device *pdev;
	struct clk		*clk;
	struct clk		*bus_clk;
	struct clk		*pixel_clk;
	struct db9000fb_dma_descriptor	*f_descriptor;

	void __iomem		*mmio_base;
	void __iomem		*misc_io_base;
	size_t			dma_buff_size;
	dma_addr_t		dma_buff_phys;
	unsigned long		frame_base;
	bool			clk_enabled;

	atomic_t		usage;
	/*
	 * These are the addresses we mapped
	 * the framebuffer memory region to.
	 */
	/* raw memory addresses */
	dma_addr_t		map_dma; /* physical */
	u_char			*map_cpu; /* virtual */
	u_int			map_size;
	unsigned long		hsync_time;
	unsigned long		cmap[16];
/* virtual address of frame buffer */
	void __iomem		*video_mem;
/* physical address of frame buffer */
	unsigned long		video_mem_phys;
/* size of the frame buffer */
	size_t			video_mem_size;
	size_t			video_mem_size_used;
/* virtual address of palette memory */
	u16			*palette_cpu;
	u_int			palette_size;
	int			palette_mode;
	u_int			cmap_inverse:1,
				cmap_static:1,
				unused:30;

/* Local images/copies of device registers */
	u32			reg_cr1;
	u32			reg_htr;
	u32			reg_hvter;
	u32			reg_vtr1;
	u32			reg_vtr2;
	u32			reg_pctr;
	u32			reg_isr;
	u32			reg_imr;
	u32			reg_ivr;
	u32			reg_iscr;
	u32			reg_dbar;
	u32			reg_dcar;
	u32			reg_dear;
	u32			reg_pwmfr;
	u32			reg_pwmdcr;
	u32			reg_dfbar;
	u32			reg_dflar;
	u32			reg_cir;

	u32			palette[PALETTE_SIZE/4];

	/*	unsigned long	hsync_time; */

	u_char			state;
	u_char			old_state;
	u_char			task_state;
	struct mutex		ctrlr_lock;
	wait_queue_head_t	ctrlr_wait;
	struct work_struct	task;

	struct completion	disable_done;

#ifdef CONFIG_CPU_FREQ
	struct notifier_block	freq_transition;
	struct notifier_block	freq_policy;
#endif
	/* Completion - for PAN display alignment with VSYNC/BAU event */
	struct completion vsync_notifier;
	void (*setup_gpio)(bool);
#ifdef CONFIG_BACKLIGHT_DB9000_LCD
	struct backlight_device *backlight;
	u8 bl_power;
#endif
	u16 db9000_rev;
	/* ignore_cpufreq_notification is > 0 if cpu and clcd uses different pll */
	unsigned int ignore_cpufreq_notification;
};
#endif /* __DB9000FB_H__ */
