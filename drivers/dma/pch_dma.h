/**
 * @file pch_dma.h
 *
 * @brief
 * This file declares the constants & functions used by the
 *  PCH_DMA_CONTROLLER driver.
 *
 * @version 0.90
 * @section
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307, USA.
 *
 * <hr>
 */

/*
 * History:
 * Copyright (C) 2008 OKI SEMICONDUCTOR Co., LTD.
 *
 *
 * created:
 *	OKISEMI 04/14/2010
 *
 */

#ifndef __PCH_DMA_H__
#define __PCH_DMA_H__

/* Constant used to denote the mode */
#define DMA_ONE_SHOT_MODE			0x2U
#define DMA_SCATTER_GATHER_MODE		0x1U

/* Constant used to denote the access size */
#define PCH_DMA_SIZE_TYPE_8BIT		(0x3U << 12)
#define PCH_DMA_SIZE_TYPE_16BIT		(0x2U << 12)
#define PCH_DMA_SIZE_TYPE_32BIT		0x0U

/* Constant used to denote the transfer direction */
#define PCH_DMA_DIR_OUT_TO_IN		0x4
#define PCH_DMA_DIR_IN_TO_OUT		0x0

/* Constant used to denote the transfer status as ACCESS */
#define PCH_DMA_END				0
#define PCH_DMA_ABORT			(-1)

/* Bits to be sit as LSB2 bits of descriptor address. */
#define DMA_DESC_END_WITH_INTERRUPT	0x00000001UL
#define DMA_DESC_FOLLOW_WITH_INTERRUPT	0x00000003UL
#define DMA_DESC_END_WITHOUT_INTERRUPT	0x00000000UL
#define DMA_DESC_FOLLOW_WITHOUT_INTERRUPT	0x00000002UL

/* The maximun transfer count */
#define PCH_DMA_8BIT_COUNT_MAX		0x3FF
#define PCH_DMA_16BIT_COUNT_MAX		0x3FF
#define PCH_DMA_32BIT_COUNT_MAX		0x7FF

/* DMA bus address for 32bit */
typedef u32 dma_addr32_t;

/**
 * enum pch_channel_request_id - Constant used to denote the channel
 *				 request type.
 * @PCH_DMA_TX_DATA_REQ0	Transmission channel 0.
 * @PCH_DMA_RX_DATA_REQ0	Reception channel 0.
 * @PCH_DMA_TX_DATA_REQ1	Transmission channel 1.
 * @PCH_DMA_RX_DATA_REQ1	Reception channel 1.
 * @PCH_DMA_TX_DATA_REQ2	Transmission channel 2.
 * @PCH_DMA_RX_DATA_REQ2	Reception channel 2.
 * @PCH_DMA_TX_DATA_REQ3	Transmission channel 3.
 * @PCH_DMA_RX_DATA_REQ3	Reception channel 3.
 * @PCH_DMA_TX_DATA_REQ4	Transmission channel 4.
 * @PCH_DMA_RX_DATA_REQ4	Reception channel 4.
 * @PCH_DMA_TX_DATA_REQ5	Transmission channel 5.
 * @PCH_DMA_RX_DATA_REQ5	Reception channel 5.
 *
 * These constants are used by other modules to make the DMA module aware of the
 * channel type it requires.
 */
enum pch_channel_request_id {
	PCH_DMA_TX_DATA_REQ0 = 1,
	PCH_DMA_RX_DATA_REQ0,
	PCH_DMA_TX_DATA_REQ1,
	PCH_DMA_RX_DATA_REQ1,
	PCH_DMA_TX_DATA_REQ2,
	PCH_DMA_RX_DATA_REQ2,
	PCH_DMA_TX_DATA_REQ3,
	PCH_DMA_RX_DATA_REQ3,
	PCH_DMA_TX_DATA_REQ4,
	PCH_DMA_RX_DATA_REQ4,
	PCH_DMA_TX_DATA_REQ5,
	PCH_DMA_RX_DATA_REQ5
};

/**
 * struct pch_dma_mode_param - Format for specifying the mode characteristics of
 *				a channel.
 * @trans_direction	Direction of Transfer(IN to OUT or OUT to IN).
 * @dma_size_typ		Type of DMA Transfer size (8bit, 16bit or 32bit).
 * @dma_trans_mode	Mode of Transfer (ONE_SHOT_MODE or SCATTER_GATHER_MODE).
 *
 * This structure is used by other modules to make the DMA module aware of the
 * channel mode characteristics.
 */
struct pch_dma_mode_param {
	u16 trans_direction;
	u16 dma_size_typ;
	u16 dma_trans_mode;
};

/**
 * struct pch_dma_desc - Format for specifying the descriptors.
 * @inside_addr	Inside address
 * @outside_addr	Outside address
 * @size		Size
 * @next_desc		Next Descriptor address
 *
 * This structure is used by other modules to make the DMA module aware of the
 * channel descriptors in SCATTER_GATHER_MODE.
 */
struct pch_dma_desc {
	u32 inside_addr;
	u32 outside_addr;
	u32 size;
	u32 next_desc;
};

extern s32 pch_request_dma(struct pci_dev *dev, s32 dreq);
extern s32 pch_free_dma(s32 channel);
extern s32 pch_set_dma_mode(s32 channel, struct pch_dma_mode_param mode_param);
extern s32 pch_set_dma_addr(s32 channel, dma_addr32_t iaddr,
				dma_addr32_t oaddr);
extern s32 pch_set_dma_count(s32 channel, u32 count);
extern s32 pch_set_dma_desc(s32 channel, struct pch_dma_desc *start,
			    struct pch_dma_desc *end);
extern s32 pch_enable_dma(s32 channel);
extern s32 pch_disable_dma(s32 channel);
extern s32 pch_dma_set_callback(s32 channel,
				void (*pch_dma_cbr) (int value,
						     unsigned long data1),
				u32 data);
#endif
