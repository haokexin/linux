/*
 * fp_bd_api.c - Buffer description API
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C)2024Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/dma-mapping.h>

#include "fp_library.h"
#include "fp_types.h"
#include "fp_net_driver.h"
//#include "fp_hif_reg.h"
#include "fp_bd_api.h"

int fp_wb_bd_init_tx_ring(struct bd_tx_ring *ring);
int fp_wb_bd_init_rx_ring(struct bd_rx_ring *ring);
int fp_enque_rx_bd(struct fp_private *fp, struct bd_rx_ring *ring, struct bd *curr_bd);
/* holds the total ring size of the entire dma alloc */
u32  fp_total_ring_size;

int fp_bd_create_ring(struct fp_private *fp)
{
	u32 ch_id;
	u32 hif_id = 0;

	for (ch_id = HIF_CHAN_START; ch_id < NUM_HIF_CHANNELS; ch_id++) {

		fp->hif[hif_id].ch[ch_id].txring =
			(struct bd_tx_ring *)
			kzalloc(sizeof(struct bd_tx_ring), GFP_KERNEL);

		if (!fp->hif[hif_id].ch[ch_id].txring) 
			return -ENOMEM;

		fp->hif[hif_id].ch[ch_id].rxring =
			(struct bd_rx_ring *)
			kzalloc(sizeof(struct bd_rx_ring),	GFP_KERNEL);

		if (!fp->hif[hif_id].ch[ch_id].rxring)
			return -ENOMEM;

		fp->hif[hif_id].ch[ch_id].txring->ring_index = ch_id;
		fp->hif[hif_id].ch[ch_id].txring->ch_index = ch_id / 8;
		fp->hif[hif_id].ch[ch_id].rxring->ring_index = ch_id;
		fp->hif[hif_id].ch[ch_id].rxring->ch_index = ch_id / 8;
	}

	return 0;
}

int fp_bd_destroy_ring(struct fp_private *fp)
{
	u32 ch_id;
	u32 hif_id = 0;

	for (ch_id = HIF_CHAN_START; ch_id < NUM_HIF_CHANNELS; ch_id++) {

		if (fp->hif[hif_id].ch[ch_id].txring) {
			kfree(fp->hif[hif_id].ch[ch_id].txring);
			fp->hif[hif_id].ch[ch_id].txring = NULL;
		}

		if (fp->hif[hif_id].ch[ch_id].rxring) {
			kfree(fp->hif[hif_id].ch[ch_id].rxring);
			fp->hif[hif_id].ch[ch_id].rxring = NULL;
		}

	}

	return 0;
}


/***************************************************************************
 * Function Name  : fp_bd_setup_txring
 * Description    : allocate memory for tx write_bk bd's and tx bufferinfo
 * Inputs
 *   Parameters   : struct fp_private *, struct bd_tx_ring *txring
 * Outputs        :
 *   Returns      : int
 * Changes        :
 ***************************************************************************/
int fp_bd_setup_tx_ring(struct fp_private *fp, struct bd_tx_ring *txring)
{
	u32 size, regval, seq_num = 1;

	size = sizeof(struct bd_tx_buffer) * NUM_TX_DESCR;
	txring->buffer_info = vzalloc(size);
	if (!txring->buffer_info)
		return -ENOMEM;

	size = (NUM_TX_WB_DESCR * sizeof(struct wb_bd)) +
		(NUM_TX_DESCR * sizeof(struct bd));

	txring->size = size + (2 * ALIGN_64_BIT_MASK);

	/* dma memory allocation */
	txring->cpu_addr = dma_alloc_coherent(fp->device,
			txring->size, &txring->dev_addr,
			GFP_KERNEL);


	if (!txring->cpu_addr) {
		dev_err(fp->device, "ERROR at alloc ch_id %u\n", txring->ring_index);
		vfree(txring->buffer_info);
		return -ENOMEM;
	}


	/******************************************************
	 *  DMA Memory Mapping Order:                         *
	 *   TX WR BACK BD Desc                               *
	 *   TX BD DESC                                       *
	 *   TX PKT BUFF                                      *
	 *   RX PKT BUFF                                      *
	 ******************************************************/

	/* Mapping above DMA memory into
	 *	corresponding BD structure format
	 */

	/* Virtual bdp address initialization */
	txring->wb_bd_tbl_va =
		(struct wb_bd *)
		((((unsigned long)txring->cpu_addr) +
		  ALIGN_64_BIT_MASK - 1) &
		 ~(ALIGN_64_BIT_MASK - 1));


	txring->bd_tbl_va = (struct bd *)
		(((unsigned long)txring->wb_bd_tbl_va +
		  (NUM_TX_WB_DESCR * sizeof(struct wb_bd)) +
		  ALIGN_64_BIT_MASK - 1) &
		 ~(ALIGN_64_BIT_MASK - 1));

	memset(txring->wb_bd_tbl_va, 0x0,
			sizeof(struct wb_bd) * NUM_TX_WB_DESCR);
	memset(txring->bd_tbl_va, 0x0,
			sizeof(struct bd) * NUM_TX_DESCR);


	/* Physical bdp address initialization */
	txring->wb_bd_tbl_pa = (dma_addr_t)
		(((unsigned long)txring->dev_addr +
		  ALIGN_64_BIT_MASK - 1) &
		 ~(ALIGN_64_BIT_MASK - 1));


	txring->bd_tbl_pa    = (dma_addr_t)
		(((unsigned long)txring->wb_bd_tbl_pa +
		  (NUM_TX_WB_DESCR * sizeof(struct wb_bd)) +
		  ALIGN_64_BIT_MASK - 1) &
		 ~(ALIGN_64_BIT_MASK - 1));

	dev_info(fp->device, "txring index : %d, va cpu_addr: %px, wb_bd_va: %px, bd_va: %px, buffer_info: %px.\n",
			txring->ring_index,
			(void *)txring->cpu_addr,
			(void *)txring->wb_bd_tbl_va,
			(void *)txring->bd_tbl_va,
			(void *)txring->buffer_info);

	dev_info(fp->device, "txring index : %d, pa dev_addr: %px, wb_bd_pa: %px, bd_pa: %px.\n",
			txring->ring_index,
			(void *)txring->dev_addr,
			(void *)txring->wb_bd_tbl_pa,
			(void *)txring->bd_tbl_pa);

	regval = CSR_REG_READ(fp_baseAddr, (ULONG)&fp->hif[0].ch_r[HIF_CHAN_START]->hif_tx_status_1_ch);
	if (regval) {
		seq_num = (regval & 0xffff) + 1;
	}

	txring->head = txring->tail = txring->bd_tbl_va;
	txring->wb_read_ptr = txring->wb_bd_tbl_va;
	txring->ring_len = NUM_TX_DESCR;
	txring->wb_seq_num = seq_num;
	txring->seq_num = seq_num;
	txring->wb_ring_len = NUM_TX_WB_DESCR;
	txring->flag = TX_BD_INIT_RING;

	return 0;
}

void fp_bd_free_tx_ring(struct fp_private *fp, struct bd_tx_ring *tx_ring)
{

	vfree(tx_ring->buffer_info);
	tx_ring->buffer_info = NULL;

	dma_free_coherent(fp->device, tx_ring->size, tx_ring->cpu_addr,
			  tx_ring->dev_addr);

	tx_ring->cpu_addr = NULL;

	return;
}
#if 0
int fp_bd_setup_tx_ring(struct fp_private *fp, struct bd_tx_ring *txring)
{
	u32 size;

	size = (NUM_TX_WB_DESCR * sizeof(struct wb_bd)) +
		(NUM_TX_DESCR * sizeof(struct bd)) +
		(NUM_TX_DESCR * MAX_TX_BUFF_SIZE);


	txring->size = size + (3 * ALIGN_64_BIT_MASK);

	/* dma memory allocation */
	txring->cpu_addr = dma_alloc_coherent(fp->device,
			txring->size, &txring->dev_addr,
			GFP_KERNEL);


	if (!txring->cpu_addr) {
		dev_err(fp->device, "ERROR at alloc ch_id %u\n", txring->ring_index);
		return -ENOMEM;
	}


	/******************************************************
	 *  DMA Memory Mapping Order:                         *
	 *   TX WR BACK BD Desc                               *
	 *   TX BD DESC                                       *
	 *   TX PKT BUFF                                      *
	 ******************************************************/

	/* Mapping above DMA memory into
	 *	corresponding BD structure format
	 */

	/* Virtual bdp address initialization */
	txring->wb_bd_tbl_va =
		(struct wb_bd *)
		((((unsigned long)txring->cpu_addr) +
		  ALIGN_64_BIT_MASK - 1) &
		 ~(ALIGN_64_BIT_MASK - 1));


	txring->bd_tbl_va = (struct bd *)
		(((unsigned long)txring->wb_bd_tbl_va +
		  (NUM_TX_WB_DESCR * sizeof(struct wb_bd)) +
		  ALIGN_64_BIT_MASK - 1) &
		 ~(ALIGN_64_BIT_MASK - 1));

	txring->bd_buff_va   = (dma_addr_t *)
		(((unsigned long)txring->bd_tbl_va +
		  (NUM_TX_DESCR * sizeof(struct bd)) +
		  ALIGN_64_BIT_MASK - 1) &
		 ~(ALIGN_64_BIT_MASK - 1));

	memset(txring->wb_bd_tbl_va, 0x0,
			sizeof(struct wb_bd) * NUM_TX_WB_DESCR);
	memset(txring->bd_tbl_va, 0x0,
			sizeof(struct bd) * NUM_TX_DESCR);



	/* Physical bdp address initialization */
	txring->wb_bd_tbl_pa = (dma_addr_t)
		(((unsigned long)txring->dev_addr +
		  ALIGN_64_BIT_MASK - 1) &
		 ~(ALIGN_64_BIT_MASK - 1));


	txring->bd_tbl_pa = (dma_addr_t)
		(((unsigned long)txring->wb_bd_tbl_pa +
		  (NUM_RX_WB_DESCR * sizeof(struct wb_bd)) +
		  ALIGN_64_BIT_MASK - 1) &
		 ~(ALIGN_64_BIT_MASK - 1));

	txring->bd_buff_pa = (dma_addr_t)
		(((unsigned long)txring->bd_tbl_pa +
		  (NUM_TX_DESCR * sizeof(struct bd)) +
		  ALIGN_64_BIT_MASK - 1) &
		 ~(ALIGN_64_BIT_MASK - 1));

	dev_info(fp->device, "tx_ring index : %d, va cpu_addr: %px, wb_bd_va: %px, bd_va: %px, bd_buff_va: %px.\n",
			txring->ring_index,
			(void *)txring->cpu_addr,
			(void *)txring->wb_bd_tbl_va,
			(void *)txring->bd_tbl_va,
			(void *)txring->bd_buff_va);

	dev_info(fp->device, "tx_ring index  %d, pa dev_addr: %px, wb_bd_pa: %px, bd_pa: %px, bd_buff_pa: %px.\n",
			txring->ring_index,
			(void *)txring->dev_addr,
			(void *)txring->wb_bd_tbl_pa,
			(void *)txring->bd_tbl_pa,
			(void *)txring->bd_buff_pa);

	txring->head = txring->tail = txring->bd_tbl_va;
	txring->wb_read_ptr = txring->wb_bd_tbl_va;
	txring->ring_len = NUM_TX_DESCR;
	txring->wb_seq_num = 1;
	txring->seq_num = 1;
	txring->wb_ring_len = NUM_TX_WB_DESCR;
	txring->flag = TX_BD_INIT_RING;


	return 0;
}


void fp_bd_free_tx_ring(struct fp_private *fp, struct bd_tx_ring *tx_ring)
{
	dma_free_coherent(fp->device, tx_ring->size, tx_ring->cpu_addr,
			  tx_ring->dev_addr);

	tx_ring->cpu_addr = NULL;

	return;
}
#endif

void fp_bd_free_all_tx_rings(struct fp_private *fp)
{
	u32 ch_id;

	for (ch_id = HIF_CHAN_START; ch_id < NUM_HIF_CHANNELS; ch_id++) {
		fp_bd_free_tx_ring(fp, fp->hif[0].ch[ch_id].txring);
	}

	return;
}

int fp_bd_setup_all_tx_rings(struct fp_private *fp)
{
	u32 ch_id;
	u32 hif_id = 0;
	int32_t i;
	int err;

	for (ch_id = HIF_CHAN_START; ch_id < NUM_HIF_CHANNELS; ch_id++) { 
		err = fp_bd_setup_tx_ring(fp, fp->hif[hif_id].ch[ch_id].txring);
		if (err) {
			dev_err(fp->device, "Allocation for Tx ring %u failed\n", ch_id);
			for (i-- ; i >= 0; i--)
				fp_bd_free_tx_ring(fp, fp->hif[hif_id].ch[ch_id].txring);
			break;
		}
	}

	return err;
}

/***************************************************************************
 * Function Name  : fp_bd_setup_rxring
 * Description    : allocate memory for rx write_bk bd's and rx bufferinfo
 * Inputs
 *   Parameters   : struct fp_private *, struct bd_rx_ring *rxring
 * Outputs        :
 *   Returns      : int
 * Changes        :
 ***************************************************************************/
int fp_bd_setup_rx_ring(struct fp_private *fp, struct bd_rx_ring *rxring)
{
	u32 size, regval, seq_num = 1;

	size = sizeof(struct bd_rx_buffer) * NUM_RX_DESCR;
	rxring->buffer_info = vzalloc(size);
	if (!rxring->buffer_info)
		return -ENOMEM;

	size = (NUM_RX_WB_DESCR * sizeof(struct wb_bd)) +
		(NUM_RX_DESCR * sizeof(struct bd));

	rxring->size = size + (2 * ALIGN_64_BIT_MASK);

	/* dma memory allocation */
	rxring->cpu_addr = dma_alloc_coherent(fp->device,
			rxring->size, &rxring->dev_addr,
			GFP_KERNEL);


	if (!rxring->cpu_addr) {
		dev_err(fp->device, "ERROR at alloc ch_id %u\n", rxring->ring_index);
		vfree(rxring->buffer_info);
		return -ENOMEM;
	}


	/******************************************************
	 *  DMA Memory Mapping Order:                         *
	 *   TX WR BACK BD Desc                               *
	 *   TX BD DESC                                       *
	 *   TX PKT BUFF                                      *
	 *   RX PKT BUFF                                      *
	 ******************************************************/

	/* Mapping above DMA memory into
	 *	corresponding BD structure format
	 */

	/* Virtual bdp address initialization */
	rxring->wb_bd_tbl_va =
		(struct wb_bd *)
		((((unsigned long)rxring->cpu_addr) +
		  ALIGN_64_BIT_MASK - 1) &
		 ~(ALIGN_64_BIT_MASK - 1));


	rxring->bd_tbl_va = (struct bd *)
		(((unsigned long)rxring->wb_bd_tbl_va +
		  (NUM_RX_WB_DESCR * sizeof(struct wb_bd)) +
		  ALIGN_64_BIT_MASK - 1) &
		 ~(ALIGN_64_BIT_MASK - 1));

	memset(rxring->wb_bd_tbl_va, 0x0,
			sizeof(struct wb_bd) * NUM_RX_WB_DESCR);
	memset(rxring->bd_tbl_va, 0x0,
			sizeof(struct bd) * NUM_RX_DESCR);


	/* Physical bdp address initialization */
	rxring->wb_bd_tbl_pa = (dma_addr_t)
		(((unsigned long)rxring->dev_addr +
		  ALIGN_64_BIT_MASK - 1) &
		 ~(ALIGN_64_BIT_MASK - 1));


	rxring->bd_tbl_pa    = (dma_addr_t)
		(((unsigned long)rxring->wb_bd_tbl_pa +
		  (NUM_RX_WB_DESCR * sizeof(struct wb_bd)) +
		  ALIGN_64_BIT_MASK - 1) &
		 ~(ALIGN_64_BIT_MASK - 1));

	dev_info(fp->device, "rxring index : %d, va cpu_addr: %px, wb_bd_va: %px, bd_va: %px, buffer_info: %px.\n",
			rxring->ring_index,
			(void *)rxring->cpu_addr,
			(void *)rxring->wb_bd_tbl_va,
			(void *)rxring->bd_tbl_va,
			(void *)rxring->buffer_info);

	dev_info(fp->device, "rxring index : %d, pa dev_addr: %px, wb_bd_pa: %px, bd_pa: %px.\n",
			rxring->ring_index,
			(void *)rxring->dev_addr,
			(void *)rxring->wb_bd_tbl_pa,
			(void *)rxring->bd_tbl_pa);

	regval = CSR_REG_READ(fp_baseAddr, (ULONG)&fp->hif[0].ch_r[HIF_CHAN_START]->hif_rx_status_0_ch);
	if (regval) {
		seq_num = (regval & 0xffff) + 1;
	}

	rxring->head = rxring->tail = rxring->bd_tbl_va;
	rxring->wb_read_ptr = rxring->wb_bd_tbl_va;
	rxring->ring_len = NUM_RX_DESCR;
	rxring->seq_num = seq_num;
	rxring->wb_seq_num = seq_num;
	rxring->wb_ring_len = NUM_RX_WB_DESCR;
	rxring->flag = RX_BD_INIT_RING;

	return 0;
}

int fp_bd_clean_rx_ring(struct fp_private *fp, struct bd_rx_ring *rxring)
{
	return 0;
}

void fp_bd_free_rx_ring(struct fp_private *fp, struct bd_rx_ring *rx_ring)
{

//	fp_bd_clean_rx_ring(fp, rxring);
//
	vfree(rx_ring->buffer_info);
	rx_ring->buffer_info = NULL;

	dma_free_coherent(fp->device, rx_ring->size, rx_ring->cpu_addr,
			  rx_ring->dev_addr);

	rx_ring->cpu_addr = NULL;

	return;
}

void fp_bd_free_all_rx_rings(struct fp_private *fp)
{
	u32 ch_id;
	u32 hif_id = 0;

	for (ch_id = HIF_CHAN_START; ch_id < NUM_HIF_CHANNELS; ch_id++) {
		fp_bd_free_rx_ring(fp, fp->hif[hif_id].ch[ch_id].rxring);
	}

	return;
}

int fp_bd_setup_all_rx_rings(struct fp_private *fp)
{
	u32 ch_id;
	u32 hif_id = 0;
	int i = 0;
	int err;

	for (ch_id = HIF_CHAN_START; ch_id < NUM_HIF_CHANNELS; ch_id++) {
		err = fp_bd_setup_rx_ring(fp, fp->hif[hif_id].ch[ch_id].rxring);
		if (err) {
			dev_err(fp->device, "Allocation for rx ring %u failed\n", ch_id);
			for (i-- ; i >= 0; i--)
				fp_bd_free_rx_ring(fp, fp->hif[hif_id].ch[ch_id].rxring);
			break;
		}
	}

	return err;
}

int fp_bd_init_tx_ring(struct fp_private *fp, struct bd_tx_ring *ring)
{
	int i = 0;
	struct bd *bd_va, *bd_pa;

	if (ring == NULL) {
		pr_err("%s: null ring ptr\n", __func__);
		return -1;
	}

	bd_va = ring->bd_tbl_va;
	bd_pa = (struct bd *)(dma_addr_t)ring->bd_tbl_pa;

	for (i = 0; i < ring->ring_len; i++) {
		bd_va->bd_ctrl = 0;

		if (i == ring->ring_len - 1) {
			bd_va->bd_nextptr = (u32)(dma_addr_t)bd_pa;
			bd_va->msb8_bd_nextptr = (u8)((dma_addr_t)bd_pa >> 32);
		} else {
			bd_va->bd_nextptr =(u32)(dma_addr_t)(bd_pa + i + 1);
			bd_va->msb8_bd_nextptr =(u8)((dma_addr_t)(bd_pa + i + 1) >> 32);
		}

		pr_debug("va: %p pa %u ms8_pa %u buf_ptr_pa %u, \n",
				(void *)bd_va, bd_va->bd_nextptr, bd_va->msb8_bd_nextptr,
				bd_va->bd_bufaddr);
		bd_va = bd_va + 1;
	}

	return 0;
}


int fp_bd_init_all_tx_rings(struct fp_private *fp)
{
	u32 ch_id;
	u32 hif_id = 0;
	int err;

	for (ch_id = HIF_CHAN_START; ch_id < NUM_HIF_CHANNELS; ch_id++) {
		err = fp_bd_init_tx_ring(fp, fp->hif[hif_id].ch[ch_id].txring);
		if (err) {
			dev_err(fp->device, "Init for tx bd ring %u failed\n", ch_id);
		}
		err = fp_wb_bd_init_tx_ring(fp->hif[hif_id].ch[ch_id].txring);
		if (err) {
			dev_err(fp->device, "Init for tx wb bd ring %u failed\n", ch_id);
		}
	}

	return err;
}

void fp_bd_uninit_rx_ring(struct fp_private *fp, struct bd_rx_ring *ring, int start, int len)
{
	int i = 0;
	struct bd_rx_buffer *buffer_info;

	for (i = start; i < start + len; i++) {
		buffer_info = &ring->buffer_info[i];
		if (buffer_info->dma)
			dma_unmap_page(fp->device, buffer_info->dma,
					MAX_RX_BUFF_SIZE,
					DMA_FROM_DEVICE);
		if (buffer_info->rxbuf.page) {
			put_page(buffer_info->rxbuf.page);
			buffer_info->rxbuf.page = NULL;
		}
		buffer_info->dma = 0;
	}

	return;
}


int fp_bd_uninit_all_rx_rings(struct fp_private *fp)
{
	u32 ch_id;
	u32 hif_id = 0;
	struct bd_rx_ring *ring = NULL;

	for (ch_id = HIF_CHAN_START; ch_id < NUM_HIF_CHANNELS; ch_id++) {
		ring = fp->hif[hif_id].ch[ch_id].rxring;
		fp_bd_uninit_rx_ring(fp, ring, 0, ring->ring_len);
	}

	return 0;
}

int fp_bd_init_rx_ring(struct fp_private *fp, struct bd_rx_ring *ring, int start, int len, bool init)
{
	int i = 0;
	struct bd *bd_va;
	struct bd *bd_pa;
	struct bd_rx_buffer *buffer_info;

	if (ring == NULL) {
		dev_err(fp->device, "%s: null ring ptr\n", __func__);
		return -1;
	}


	bd_pa = (struct bd *)(unsigned long)ring->bd_tbl_pa;

	for (i = start; len; len--, i++) {
		
		i = i % ring->ring_len;

		buffer_info = &ring->buffer_info[i];
//		buffer_info = READ_ONCE(((struct bd_rx_buffer *)&ring->buffer_info[i]));
//		pr_err("len = %d,  i = %d.\n", len, i);

		if (!buffer_info->rxbuf.page) {
//			pr_err("enter page.\n");
//			buffer_info->rxbuf.page = alloc_page(GFP_KERNEL);
			buffer_info->rxbuf.page = alloc_page(GFP_ATOMIC);
			if (unlikely(!buffer_info->rxbuf.page)) {
				ring->stats.alloc_failed++;
//				dev_err(fp->device, "%s: alloc rx ring page failed.\n", __func__);
				pr_err("%s: alloc rx ring page failed.\n", __func__);
				break;
			}
		}

		
		if (!READ_ONCE(buffer_info->dma)) {
//			pr_err("enter dma.\n");
			buffer_info->dma = dma_map_page(fp->device,
					buffer_info->rxbuf.page, 0,
					MAX_RX_BUFF_SIZE,
					DMA_FROM_DEVICE);
			if (dma_mapping_error(fp->device, buffer_info->dma)) {
				put_page(buffer_info->rxbuf.page);
				buffer_info->rxbuf.page = NULL;
				buffer_info->dma = 0;
				ring->stats.alloc_failed++;
//				dev_err(fp->device, "%s: dam map rx page failed.\n", __func__);
				pr_err("%s: dam map rx page failed.\n", __func__);
				break;
			}
//			dma_sync_single_range_for_device(fp->device, buffer_info->dma,
//					0, MAX_RX_BUFF_SIZE, DMA_FROM_DEVICE);
		}

		bd_va = &ring->bd_tbl_va[i];

		bd_va->bd_ctrl = 0;

		bd_va->bd_ctrl |= BD_CTRL_CBD_INT_EN |
			BD_CTRL_PKT_INT_EN | BD_CTRL_DIR;
		bd_va->bd_buflen = MAX_RX_BUFF_SIZE;

		/*-----------------------------------------------------------------------------
		 *  todo :bd_bufaddr request 64 byte boundary, rxbuf page already at 64 byte
		 *  boundary.if buffer_info->dma addr is not at 64 byte boundary, maybe we
		 *  should add the offest at buffer_info->dma and rxbuf.page addr.
		 *-----------------------------------------------------------------------------*/
		bd_va->bd_bufaddr = (u32) (buffer_info->dma);
		bd_va->msb8_bd_bufaddr = (u8)(buffer_info->dma >> 32);
//		pr_err("rx ring index = %d, i = %d, buff->dma = %px, bd_va->bd_buffaddr = %px, bd_va->msb8_bd_bufaddr \n", 
//				ring->ring_index, i, buffer_info->dma, bd_va->bd_bufaddr, bd_va->msb8_bd_bufaddr);
		fp_enque_rx_bd(fp, ring, bd_va);


		if (init) {
			if (i == ring->ring_len - 1) {
				bd_va->bd_nextptr = (u32)(dma_addr_t)bd_pa;
				bd_va->msb8_bd_nextptr = (u8)((dma_addr_t)bd_pa >> 32);
				//bd_va->bd_ctrl |= (bd_va->bd_ctrl | BD_CTRL_LAST_BD);
			} else {
				bd_va->bd_nextptr =(u32)(dma_addr_t)(bd_pa + i + 1);
				bd_va->msb8_bd_nextptr =(u8)((dma_addr_t)(bd_pa + i + 1) >> 32);
			}

//			pr_debug("va: %p pa %px, msb_pa %px\n",(void *)bd_va, bd_va->bd_nextptr, bd_va->msb8_bd_nextptr);
		}
	}

	dma_wmb();

	return len;
}


int fp_bd_init_all_rx_rings(struct fp_private *fp)
{
	u32 ch_id;
	u32 hif_id = 0;
	struct bd_rx_ring *ring = NULL;
	int err;

	for (ch_id = HIF_CHAN_START; ch_id < NUM_HIF_CHANNELS; ch_id++) {
		ring = fp->hif[hif_id].ch[ch_id].rxring;
		err = fp_bd_init_rx_ring(fp, ring, 0, ring->ring_len, true);
		if (err) {
			dev_err(fp->device, "Init for rx bd ring %u failed\n", ch_id);
		}
		err = fp_wb_bd_init_rx_ring(ring);
		if (err) {
			dev_err(fp->device, "Init for rx wb bd ring %u failed\n", ch_id);
		}
	}

	return err;
}


inline int fp_rx_bd_used(struct bd_rx_ring *ring)
{
	if (READ_ONCE(ring->tail) >= READ_ONCE(ring->head))
		return ring->ring_len - (READ_ONCE(ring->tail) - READ_ONCE(ring->head)) - 1 ;

	return READ_ONCE(ring->head) - READ_ONCE(ring->tail) - 1 ;
}


#if 0
inline int fp_tx_bd_unused(struct bd_tx_ring *ring)
{
	if (ring->tail >= ring->head)
		return ring->ring_len - (ring->tail - ring->head);

	return ring->head - ring->tail;
}
#endif

inline int fp_tx_bd_unused(struct bd_tx_ring *ring)
{
	if (READ_ONCE(ring->tail) >= READ_ONCE(ring->head))
		return ring->ring_len - (READ_ONCE(ring->tail) - READ_ONCE(ring->head)) - 1;

	return (READ_ONCE(ring->head) - READ_ONCE(ring->tail)) - 1;
}

/***************************************************************************
 * Function Name  : fp_dump_wb_bd_info
 * Description    : dump write back bd
 * Inputs
 *   Parameters   : struct wb_bd *
 * Outputs        :
 *   Parameters   :
 *   Returns      : void
 * Changes        :
 ***************************************************************************/
void fp_dump_wb_bd_info(struct wb_bd *dump_bd)
{
#if 0
	pr_info("WBBD:\t%p size %u\n", dump_bd,
					(unsigned int)sizeof(struct wb_bd));
	pr_info("ctrl\t%x bd_buflen\t%x bd_seqnum\t%x\n",
				dump_bd->bd_ctrl, dump_bd->bd_buflen,
				dump_bd->bd_seqnum);
#endif
}

/***************************************************************************
 * Function Name  : fp_wb_bd_init_ring
 * Description    : initialize the tx/tx/wb rings
 * Inputs
 *   Parameters   : struct fp_private *, int, int
 * Outputs        :
 *   Parameters   : struct fp_private *
 *   Returns      : int
 * Changes        :
 ***************************************************************************/
int fp_wb_bd_init_tx_ring(struct bd_tx_ring *ring)
{
	int desc_id = 0;
	struct wb_bd *curr_bd;

	if (ring == NULL) {
		pr_info("%s: null ring ptr\n", __func__);
		return -1;
	}

	memset(ring->wb_bd_tbl_va, 0, sizeof(struct wb_bd));

	curr_bd = ring->wb_bd_tbl_va;
	for (desc_id = 0; desc_id < ring->wb_ring_len; desc_id++) {
		curr_bd->bd_seqnum = 0xff;
		curr_bd++;
	}

	return 0;
}

int fp_wb_bd_init_rx_ring(struct bd_rx_ring *ring)
{
	int desc_id = 0;
	struct wb_bd *curr_bd;

	if (ring == NULL) {
		pr_info("%s: null ring ptr\n", __func__);
		return -1;
	}

	memset(ring->wb_bd_tbl_va, 0, sizeof(struct wb_bd));

	curr_bd = ring->wb_bd_tbl_va;
	for (desc_id = 0; desc_id < ring->wb_ring_len; desc_id++) {
		curr_bd->bd_seqnum = 0xff;
		curr_bd++;
	}

	return 0;
}
/***************************************************************************
 * Function Name  : fp_get_next_free_bd
 * Description    : initialize the tx/tx/wb rings
 * Inputs
 *   Parameters   :
 * Outputs        :
 *   Parameters   : struct fp_private *
 *   Returns      : int
 * Changes        :
 ***************************************************************************/
struct bd *fp_get_next_free_rx_bd(struct bd_rx_ring *ring)
{
	struct bd *tail;

	if (ring == NULL) {
		pr_err("%s: get_next_free_bd: bd_ring is NULL\n", __func__);
		return NULL;
	}

	tail = ring->tail;

	pr_debug("Free bd %p\n", tail);

	if (tail->bd_ctrl & BD_CTRL_DESC_EN)
		return NULL;
	else
		return tail;
}

struct bd *fp_get_next_free_tx_bd(struct bd_tx_ring *ring)
{
	struct bd *tail;

	if (ring == NULL) {
		pr_err("%s: get_next_free_bd: bd_ring is NULL\n", __func__);
		return NULL;
	}

//	tail = ring->tail;
	tail = READ_ONCE(ring->tail);

//	pr_debug("Free bd %p\n", tail);

	if (READ_ONCE(tail->bd_ctrl) & BD_CTRL_DESC_EN)
//	if (tail->bd_ctrl & BD_CTRL_DESC_EN)
	{
//		 pr_err("Free bd %px\n", READ_ONCE(tail));
		return NULL;
	}
	else
		return tail;
}


u32 fp_get_curr_rx_bd_index(struct bd_rx_ring *ring, struct bd *bd)
{
	return (u32)(bd - ring->bd_tbl_va);
}
/**********************************************************************
 * Function Name  : fp_get_curr_bd_id
 * Description    : get the curr bd id from the ring
 * Inputs
 *   Parameters   : struct bd_ring *, struct bd *
 * Outputs        :
 *   Parameters   :
 *   Returns      : int
 * Changes        :
 *********************************************************************/
u32 fp_get_curr_tx_bd_index(struct bd_tx_ring *ring, struct bd *bd)
{
	return (u32)(bd - ring->bd_tbl_va);
}


inline int fp_enque_rx_bd(struct fp_private *fp, struct bd_rx_ring *ring, struct bd *curr_bd)
{
	if (READ_ONCE(curr_bd->bd_ctrl) & BD_CTRL_DESC_EN) {
		pr_err("enque_bd: DESC_EN is already set\n");
		return -1;
	}

#if 0
	/* Update the bd seq. num */
	curr_bd->bd_seqnum = ring->seq_num;

	ring->seq_num = ring->seq_num + 1;
#endif

	if ((fp_get_curr_rx_bd_index(ring, curr_bd) == 0) && test_bit(HIFVMAC_RX_FIFO_CLEAR, &fp->state)) {
		fp->rx_fifo_clear = true;
	}

	if (fp->rx_fifo_clear) {
		WRITE_ONCE(curr_bd->bd_seqnum, 0xff);
		if (fp_get_curr_rx_bd_index(ring, curr_bd) == (NUM_RX_DESCR - 2)) {
			fp->rx_fifo_clear = false;
			set_bit(HIFVMAC_RX_FIFO_CLEARED, &fp->state);
		}
	} else {
		WRITE_ONCE(curr_bd->bd_seqnum, READ_ONCE(ring->seq_num));
	}

	WRITE_ONCE(ring->seq_num, READ_ONCE(ring->seq_num) + 1);

	if (ring->bd_tbl_va + ring->ring_len - 1 == curr_bd)
		curr_bd->bd_ctrl |= BD_CTRL_LAST_BD;
	
//	curr_bd->bd_ctrl |= BD_CTRL_DESC_EN;
	if (fp->rx_fifo_clear) {
		WRITE_ONCE(curr_bd->bd_ctrl, curr_bd->bd_ctrl & (~(BD_CTRL_DESC_EN)));
	} else {
		WRITE_ONCE(curr_bd->bd_ctrl, curr_bd->bd_ctrl | BD_CTRL_DESC_EN);
	}

//	pr_debug("%s:ring->tail is %lx\n", __func__, (unsigned long)ring->tail);

	/* update tail to next free location */
//	ring->tail = ring->tail + 1;
	WRITE_ONCE(ring->tail, READ_ONCE(ring->tail) + 1);

	if (ring->tail >= (ring->bd_tbl_va + ring->ring_len))
//		ring->tail = ring->bd_tbl_va;
	WRITE_ONCE(ring->tail, ring->bd_tbl_va);


	return 0;
}

/**********************************************************************
 * Function Name  : fp_enque_tx_bd
 * Description    : enque bd into the ring
 *                  after enque, bd gets owned by hardware unti it dequeued
 * Inputs
 *   Parameters   : struct bd_ring *, struct bd *
 * Outputs        :
 *   Parameters   :
 *   Returns      : int
 * Changes        :
 *********************************************************************/
int fp_enque_tx_bd(struct fp_private *fp, struct bd_tx_ring *ring, struct bd *curr_bd)
{
	if (curr_bd->bd_ctrl & BD_CTRL_DESC_EN) {
		pr_err("enque_bd: DESC_EN is already set\n");
		return -1;
	}

	if (test_bit(HIFVMAC_TX_FIFO_CLEAR, &fp->state) && (fp_get_curr_tx_bd_index(ring, curr_bd) == 0)) {
		fp->tx_fifo_clear = true;
	}

	/* Update the bd seq. num */
	if (fp->tx_fifo_clear) {
		curr_bd->bd_seqnum = 0xff;
		set_bit(HIFVMAC_TX_FIFO_CLEARED, &fp->state);
	} else {
		curr_bd->bd_seqnum = ring->seq_num;
	}

	ring->seq_num = ring->seq_num + 1;

	/* update tail to next free location */
//	ring->tail = ring->tail + 1;
	WRITE_ONCE(ring->tail, READ_ONCE(ring->tail) + 1);

	if (ring->tail >= (ring->bd_tbl_va + ring->ring_len)) {
//		ring->tail = ring->bd_tbl_va;
		WRITE_ONCE(ring->tail, ring->bd_tbl_va);
	}

	if (ring->bd_tbl_va + ring->ring_len - 1 == curr_bd)
		curr_bd->bd_ctrl |= BD_CTRL_LAST_BD;
	
	if (fp->tx_fifo_clear) {
		curr_bd->bd_ctrl &= (~BD_CTRL_DESC_EN);
	} else {
		curr_bd->bd_ctrl |= BD_CTRL_DESC_EN;
	}

	pr_debug("%s:ring->tail is %lx\n", __func__, (unsigned long)ring->tail);

	return 0;
}

struct bd *fp_deque_rx_bd(struct bd_rx_ring *ring, u32 *wb_buflen, bool *lifm)
{
	struct bd *head;
	struct wb_bd *wrbk = READ_ONCE(ring->wb_read_ptr);
	struct wb_bd *wrbk_end = ring->wb_bd_tbl_va + ring->wb_ring_len;

	head = ring->head;

	fp_dump_wb_bd_info(wrbk);
//	pr_debug("deque_bd: head is %lx, wrbk is %lx\n",
//				(unsigned long)head, (unsigned long)wrbk);

	if ((head->bd_seqnum != wrbk->bd_seqnum) ||
				(!(head->bd_ctrl & BD_CTRL_DESC_EN))) {
//		pr_debug("head seq num and wb seq num are not equal %d %d\n",
//				head->bd_seqnum, wrbk->bd_seqnum);
		return NULL;
	}

//		pr_debug("seq_no %x, bd_ctrl %x, buf_len %x\n", wrbk->bd_seqnum,
//						wrbk->bd_ctrl, wrbk->bd_buflen);

	*wb_buflen = wrbk->bd_buflen;
	*lifm = wrbk->bd_ctrl & WB_BD_LIFM;

	/* Reset DESC_EN bit, */
	head->bd_ctrl &= (~(BD_CTRL_DESC_EN));

	head->bd_seqnum = 0;

//	ring->head = ring->head + 1;
	WRITE_ONCE(ring->head, READ_ONCE(ring->head) +1);

	if (ring->head >= (ring->bd_tbl_va + ring->ring_len))
//		ring->head = ring->bd_tbl_va;
	WRITE_ONCE(ring->head, ring->bd_tbl_va);

//	ring->wb_read_ptr = (((wrbk + 1) >= wrbk_end) ?
//				(ring->wb_bd_tbl_va) : (wrbk + 1));
	WRITE_ONCE(ring->wb_read_ptr, (((wrbk + 1) >= wrbk_end) ?
				(ring->wb_bd_tbl_va) : (wrbk + 1)));

//	pr_debug("read_ptr is %lx, wrbk_end  %lx\n",
//		(unsigned long)ring->wb_read_ptr, (unsigned long)wrbk_end);
//
//	pr_debug("ring->wb_read_ptr is %lx\n",
//				(unsigned long)ring->wb_read_ptr);

	return head;
}

/**********************************************************************
 * Function Name  : fp_deque_bd
 * Description    : deque bd from the ring
 *                  after deque, bd gets owned by software unti it enqueued
 * Inputs
 *   Parameters   : struct bd_ring *, unsigend int *
 * Outputs        :
 *   Parameters   :
 *   Returns      : struct bd *
 * Changes        :
 *********************************************************************/
struct bd *fp_deque_tx_bd(struct bd_tx_ring *ring, u32 *wb_buflen, bool *lifm)
{
	struct bd *head;
	struct wb_bd *wrbk = ring->wb_read_ptr;
	struct wb_bd *wrbk_end = ring->wb_bd_tbl_va + ring->wb_ring_len;

	head = ring->head;

//	fp_dump_wb_bd_info(wrbk);
//	pr_debug("deque_bd: head is %lx, wrbk is %lx\n",
//				(unsigned long)head, (unsigned long)wrbk);
//
	if ((head->bd_seqnum != wrbk->bd_seqnum) ||
				(!(head->bd_ctrl & BD_CTRL_DESC_EN))) {
		pr_debug("head seq num and wb seq num are not equal %d %d\n",
				head->bd_seqnum, wrbk->bd_seqnum);
		return NULL;
	}

//	pr_err("seq_no %x, bd_ctrl %x, buf_len %x\n", wrbk->bd_seqnum,
//			wrbk->bd_ctrl, wrbk->bd_buflen);

	*wb_buflen = wrbk->bd_buflen;
	*lifm = wrbk->bd_ctrl & WB_BD_LIFM;

	/* Reset DESC_EN bit, */
	head->bd_ctrl = 0;
//	pr_err("head->bd_ctrl = %px.\n", head->bd_ctrl);

	head->bd_seqnum = 0;

//	ring->head = ring->head + 1;
	WRITE_ONCE(ring->head, READ_ONCE(ring->head) +1);

	if (ring->head >= (ring->bd_tbl_va + ring->ring_len))
//		ring->head = ring->bd_tbl_va;
	WRITE_ONCE(ring->head, ring->bd_tbl_va);

	ring->wb_read_ptr = (((wrbk + 1) >= wrbk_end) ?
				(ring->wb_bd_tbl_va) : (wrbk + 1));

#if 0
	pr_debug("read_ptr is %lx, wrbk_end  %lx\n",
		(unsigned long)ring->wb_read_ptr, (unsigned long)wrbk_end);

	pr_debug("ring->wb_read_ptr is %lx\n",
				(unsigned long)ring->wb_read_ptr);
#endif

	return head;
}


#if 0
void fp_init_bdp_base_reg(struct fp_private *fp)
{
	UINT hif_index = 0, ch_index = 0;


	for (ch_index = 0; ch_index < NUM_HIF_CHANNELS; ch_index++) {

		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				regs->ch[ch_index].hif_ch_int_en, 0x3ff);
		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				regs->ch[ch_index].hif_ctrl_ch, 0x10000);

		/* Program the RX WRBK BD reg */
		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				regs->ch[ch_index].hif_rx_bdp_wr_low_addr_ch,
				fp->hif[hif_index].ch[ch_index].
				rxring->wb_bd_tbl_pa);

		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				regs->ch[ch_index].hif_rx_bdp_wr_high_addr_ch,
				0x0);
		/* Program the RXBD reg */
		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				regs->ch[ch_index].hif_rx_bdp_rd_low_addr_ch,
				fp->hif[hif_index].ch[ch_index].
				rxring->bd_tbl_pa);

		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				regs->ch[ch_index].hif_rx_bdp_rd_high_addr_ch,
				0x0);

		/* Program the TX WRBK BD reg */
		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				regs->ch[ch_index].hif_tx_bdp_wr_low_addr_ch,
				fp->hif[hif_index].ch[ch_index].
				txring->wb_bd_tbl_pa);

		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				regs->ch[ch_index].hif_tx_bdp_wr_high_addr_ch,
				0x0);

		/* Program the TXBD reg */
		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				regs->ch[ch_index].hif_tx_bdp_rd_low_addr_ch,
				fp->hif[hif_index].ch[ch_index].
				txring->bd_tbl_pa);

		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				regs->ch[ch_index].hif_tx_bdp_rd_high_addr_ch,
				0x0);

		/* Program the RX WRBK buff size */
		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				regs->ch[ch_index].
				hif_rx_wrbk_bd_ch_buffer_size,
				NUM_RX_WB_DESCR);

		/* Program the TX WRBK buff size */
		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				regs->ch[ch_index].
				hif_tx_wrbk_bd_ch_buffer_size,
				NUM_TX_WB_DESCR);
	}

	/* Program the sequence num check  start in MISC reg */
	CSR_REG_WRITE(fp_baseAddr,
			(ULONG)&fp->hif[hif_index].regs->hif_misc, 0x1003f);
	//(ULONG)&fp->hif[hif_index].regs->hif_misc, 0x1003f);
	//(ULONG)&fp->hif[hif_index].regs->hif_misc, 0x1001f);
}
#endif

void fp_init_bdp_base_reg(struct fp_private *fp)
{
	unsigned long  hif_index = 0, ch_index = 0;


	for_each_set_bit(ch_index, fp->_channel_mask, NUM_HIF_CHANNELS) {
//		pr_err("set index = %d reg.\n", ch_index);

		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				ch_r[ch_index]->hif_ch_int_en, 0x3ff);
		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				ch_r[ch_index]->hif_ctrl_ch, 0x10000);

		/* Program the RX WRBK BD reg */
		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				ch_r[ch_index]->hif_rx_bdp_wr_low_addr_ch,
				(u32)fp->hif[hif_index].ch[ch_index].
				rxring->wb_bd_tbl_pa);

		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				ch_r[ch_index]->hif_rx_bdp_wr_high_addr_ch,
				(u32)(fp->hif[hif_index].ch[ch_index].
				rxring->wb_bd_tbl_pa >> 32));
		/* Program the RXBD reg */
		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				ch_r[ch_index]->hif_rx_bdp_rd_low_addr_ch,
				(u32)fp->hif[hif_index].ch[ch_index].
				rxring->bd_tbl_pa);

		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				ch_r[ch_index]->hif_rx_bdp_rd_high_addr_ch,
				(u32)(fp->hif[hif_index].ch[ch_index].
				rxring->bd_tbl_pa >> 32));

		/* Program the TX WRBK BD reg */
		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				ch_r[ch_index]->hif_tx_bdp_wr_low_addr_ch,
				(u32)fp->hif[hif_index].ch[ch_index].
				txring->wb_bd_tbl_pa);

		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				ch_r[ch_index]->hif_tx_bdp_wr_high_addr_ch,
				(u32)(fp->hif[hif_index].ch[ch_index].
				txring->wb_bd_tbl_pa >>32));

		/* Program the TXBD reg */
		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				ch_r[ch_index]->hif_tx_bdp_rd_low_addr_ch,
				(u32)fp->hif[hif_index].ch[ch_index].
				txring->bd_tbl_pa);

		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				ch_r[ch_index]->hif_tx_bdp_rd_high_addr_ch,
				(u32)(fp->hif[hif_index].ch[ch_index].
				txring->bd_tbl_pa >>32));

		/* Program the RX WRBK buff size */
		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				ch_r[ch_index]->hif_rx_wrbk_bd_ch_buffer_size,
				NUM_RX_WB_DESCR);

		/* Program the TX WRBK buff size */
		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].
				ch_r[ch_index]->hif_tx_wrbk_bd_ch_buffer_size,
				NUM_TX_WB_DESCR);
	}

	/* Program the sequence num check  start in MISC reg */
	CSR_REG_WRITE(fp_baseAddr,
			(ULONG)&fp->hif[hif_index].regs->hif_misc, 0x1003f);
	//(ULONG)&fp->hif[hif_index].regs->hif_misc, 0x1003f);
	//(ULONG)&fp->hif[hif_index].regs->hif_misc, 0x1001f);
}

int fp_bd_reinit(struct fp_private *fp)
{
	if (!fp) {
		pr_err("Global private device not found\n");
		return -1;
	}

	fp->reset_bd = 1;
//	fp_bd_init(fp);
//	fp_init_bdp_base_reg(fp);
	fp_en_hif_rxdma_engine(fp);
	fp->reset_bd = 0;
	return 0;
}
