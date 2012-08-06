/*
 * SPEAr camera platform initialization
 *
 * Copyright (C) 2011 ST Microelectronics
 * Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_CAMIF_H
#define __PLAT_CAMIF_H

#include <linux/platform_device.h>

/* camif video buffer offset */
#define CAMIF_MEM_BUFFER	0x3000

/* camif synchronization methods */
enum camif_synhcro {
	EXTERNAL_SYNC = 0,	/* hsync/vsync are used to recover data */
	EMBEDDED_SYNC,
};

/* camif polarity settings : common for hsync, vsync and pixclk */
enum camif_polarity {
	ACTIVE_LOW = 0,
	ACTIVE_HIGH,
};

/* camif capture modes */
enum camif_capture_modes {
	NO_CAPTURE = 0,
	PHOTO_MODE_1,
	PHOTO_MODE_2,
	PHOTO_MODE_3,
	PHOTO_MODE_4,
	PHOTO_MODE_8,
	PHOTO_MODE_16,
	PHOTO_MODE_32,
	VIDEO_MODE_ALL_FRAMES,
	VIDEO_MODE_1_FRAME_ON_2,
	VIDEO_MODE_1_FRAME_ON_4,
	VIDEO_MODE_1_FRAME_ON_8,
	VIDEO_MODE_1_FRAME_ON_16,
	VIDEO_MODE_1_FRAME_ON_32,
	VIDEO_MODE_1_FRAME_ON_64,
	VIDEO_MODE_1_FRAME_ON_128,
};

/* camif supported storage swappings */
enum camif_dma_burst_size {
	BURST_SIZE_1 = 0,
	BURST_SIZE_4,
	BURST_SIZE_8,
	BURST_SIZE_16,
	BURST_SIZE_32,
	BURST_SIZE_64,
	BURST_SIZE_128,
	BURST_SIZE_256,
};

/* camif dma channel types */
enum camif_channel_type {
	EVEN_CHANNEL,
	BOTH_ODD_EVEN_CHANNELS,
};

/**
 * struct camif_config_data - configuration specific params for
 * camif
 *
 * @sync_type: embedded or external sync
 * @vsync_polarity: ploarity of vsync signal
 * @hsync_polarity: polarity of hsync signal
 * @pclk_polarity: polarity of pixel clock signal
 * @capture_mode: photo or video mode
 * @burst_size: DMA burst size
 * (should be programmed consistently with the DMA IP burst size used)
 * @channel: DMA channel type to use
 */
struct camif_config_data {
	enum camif_synhcro sync_type;
	enum camif_polarity vsync_polarity;
	enum camif_polarity hsync_polarity;
	enum camif_polarity pclk_polarity;
	enum camif_capture_modes capture_mode;
	enum camif_dma_burst_size burst_size;
	enum camif_channel_type channel;
	/* callback to reset the CAMIF module */
	void (*camif_module_enable)(int, bool);
};

/**
 * struct camif_controller - platform_data for camif controller devices
 *
 * @enable_dma: if true enables DMA driven transfers
 * @dma_even_param: parameter to locate an DMA channel for even lines
 * @dma_odd_param: parameter to locate a DMA channel for odd lines
 * @config: configuration parameter for this camif instance
 */
struct camif_controller {
	bool (*dma_filter)(struct dma_chan *chan, void *filter_param);
	void *dma_even_param;
	void *dma_odd_param;
	struct camif_config_data *config;
};

#endif /* __PLAT_CAMIF_H */
