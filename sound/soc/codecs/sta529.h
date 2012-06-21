/*
 * ASoC machine driver for spear platform
 *
 * sound/soc/codecs/sta529.h -- spear ALSA Soc Audio driver Header file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef STA529_H
#define STA529_H

/* STA529 Register offsets */
#define	 STA529_FFXCFG0		0x00
#define	 STA529_FFXCFG1		0x01
#define	 STA529_MVOL		0x02
#define	 STA529_LVOL		0x03
#define	 STA529_RVOL		0x04
#define	 STA529_TTF0		0x05
#define	 STA529_TTF1		0x06
#define	 STA529_TTP0		0x07
#define	 STA529_TTP1		0x08
#define	 STA529_S2PCFG0		0x0A
#define	 STA529_S2PCFG1		0x0B
#define	 STA529_P2SCFG0		0x0C
#define	 STA529_P2SCFG1		0x0D
#define	 STA529_PLLCFG0		0x14
#define	 STA529_PLLCFG1		0x15
#define	 STA529_PLLCFG2		0x16
#define	 STA529_PLLCFG3		0x17
#define	 STA529_PLLPFE		0x18
#define	 STA529_PLLST		0x19
#define	 STA529_ADCCFG		0x1E /*mic_select*/
#define	 STA529_CKOCFG		0x1F
#define	 STA529_MISC		0x20
#define	 STA529_PADST0		0x21
#define	 STA529_PADST1		0x22
#define	 STA529_FFXST		0x23
#define	 STA529_PWMIN1		0x2D
#define	 STA529_PWMIN2		0x2E
#define	 STA529_POWST		0x32

#define STA529_CACHEREGNUM	0x33 /*total num of reg. in sta529*/

#define SPEAR_PCM_RATES		(SNDRV_PCM_RATE_8000 | \
				SNDRV_PCM_RATE_11025 | \
				SNDRV_PCM_RATE_16000 | \
				SNDRV_PCM_RATE_22050 | \
				SNDRV_PCM_RATE_32000 | \
				SNDRV_PCM_RATE_44100 | \
				SNDRV_PCM_RATE_48000)
#define SPEAR_PCM_FORMAT	SNDRV_PCM_FMTBIT_S16_LE
#define	S2PC_VALUE		0x98
#define CLOCK_OUT		0x60
#define LEFT_J_DATA_FORMAT	0x00
#define I2S_DATA_FORMAT		0x02
#define RIGHT_J_DATA_FORMAT	0x04
#define RIGHT_J_DATA_FORMAT	0x04
#define CODEC_MUTE_VAL		0x80
#define NUM_OF_MSG		2
#define POWER_STBY		(0x1 << 6)

#endif /* end of codec header file */
