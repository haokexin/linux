/*
 * arch/arm/plat-spear/include/plat/adc.h
 *
 * ADC definitions for SPEAr platform
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_ADC_H
#define __PLAT_ADC_H

#include <linux/types.h>
#include <linux/spear_adc_usr.h>

#ifdef CONFIG_ARCH_SPEAR13XX
#define ADC_DMA_MAX_COUNT	2048
#else
#include <asm/hardware/pl080.h>

#define ADC_DMA_MAX_COUNT	PL080_CONTROL_TRANSFER_SIZE_MASK
#endif

struct dma_chan;
struct adc_plat_data {
	struct adc_config config;
	void *dma_data;
	bool (*dma_filter)(struct dma_chan *chan, void *filter_param);
};
#endif /* __PLAT_ADC_H */
