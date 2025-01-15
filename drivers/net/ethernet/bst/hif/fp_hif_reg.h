/*
 * fp_hif_reg.h
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C)2024Black Sesame Technologies. All Rights Reserved.
 */

#ifndef _FP_HIF_REG_H
#define _FP_HIF_REG_H

#include "fp_net_driver.h"
#include "fp_types.h"
#include "fp_library.h"
#define FP_ASSIGNED_HIF 0

#define HIF_CH_INT           (1 << 0)
#define HIF_CH_RXBD_INT      (1 << 1)
#define HIF_CH_RXPKT_INT     (1 << 2)
#define HIF_CH_TXBD_INT      (1 << 3)
#define HIF_CH_TXPKT_INT     (1 << 4)
#define HIF_CH_INT_EN        (1 << 0)
#define HIF_CH_RXBD_INT_EN   (1 << 1)
#define HIF_CH_RXPKT_INT_EN  (1 << 2)
#define HIF_CH_TXBD_INT_EN   (1 << 3)
#define HIF_CH_TXPKT_INT_EN  (1 << 4)

#define HIF_CH_TX_CTRL_DMA_EN     (1 << 0)
#define HIF_CH_TX_BDP_POLL_CNTR_EN (1 << 1)
#define HIF_CH_RX_CTRL_DMA_EN     (1 << 16)
#define HIF_CH_RX_BDP_POLL_CNTR_EN (1 << 17)

#define HIF_CH_TX_START      (1 << 0)
#define HIF_CH_RX_START      (1 << 0)

#define HIF_QUEUE_MAP_VALID_MASK   3

#define HIF_MAX_TX_QUEUES     64
#define HIF_MAX_RX_QUEUES     64


#define HIF_BASE_ADDR                       0x680000


typedef struct channel_reg
{
    volatile uint32 hif_ctrl_ch;
    volatile uint32 hif_rx_bdp_wr_low_addr_ch;
    volatile uint32 hif_rx_bdp_wr_high_addr_ch;
    volatile uint32 hif_rx_bdp_rd_low_addr_ch;

    volatile uint32 hif_rx_bdp_rd_high_addr_ch;
    volatile uint32 hif_tx_bdp_wr_low_addr_ch;
    volatile uint32 hif_tx_bdp_wr_high_addr_ch;
    volatile uint32 hif_tx_bdp_rd_low_addr_ch;

    volatile uint32 hif_tx_bdp_rd_high_addr_ch;
    volatile uint32 hif_rx_wrbk_bd_ch_buffer_size;
    volatile uint32 hif_rx_ch_start;
    volatile uint32 hif_tx_wrbk_bd_ch_buffer_size;

    volatile uint32 hif_tx_ch_start;
    volatile uint32 hif_doorbell_addr_lsb_ch;
    volatile uint32 hif_doorbell_addr_msb_ch;
    volatile uint32 hif_rx_packet_drop_cnt_ch;

    volatile uint32 reserved1[8];

    volatile uint32 hif_ch_int_src;
    volatile uint32 hif_ch_int_en;
    volatile uint32 reserved2[6];

    volatile uint32 hif_tx_rd_curr_bd_low_addr_ch;
    volatile uint32 hif_tx_rd_curr_bd_high_addr_ch;
    volatile uint32 hif_tx_wr_curr_bd_low_addr_ch;
    volatile uint32 hif_tx_wr_curr_bd_high_addr_ch;

    volatile uint32 hif_bdp_ch_tx_fifo_cnt;
    volatile uint32 hif_tx_dma_status_0_ch;
    volatile uint32 hif_tx_status_0_ch;
    volatile uint32 hif_tx_status_1_ch;

    volatile uint32 hif_tx_pkt_cnt0_ch;
    volatile uint32 hif_tx_pkt_cnt1_ch;
    volatile uint32 hif_tx_pkt_cnt2_ch;
    volatile uint32 reserved3[5];

    volatile uint32 hif_rx_rd_curr_bd_low_addr_ch;
    volatile uint32 hif_rx_rd_curr_bd_high_addr_ch;
    volatile uint32 hif_rx_wr_curr_bd_low_addr_ch;
    volatile uint32 hif_rx_wr_curr_bd_high_addr_ch;

    volatile uint32 hif_bdp_ch_rx_fifo_cnt;
    volatile uint32 hif_rx_dma_status_0_ch;
    volatile uint32 hif_rx_status_0_ch;
    volatile uint32 hif_rx_pkt_cnt0_ch0;

    volatile uint32 hif_rx_pkt_cnt1_ch0;
    volatile uint32 hif_ltc_max_pkt_ch;
    volatile uint32 hif_abs_int_timer_ch;
    volatile uint32 hif_abs_frame_count_ch;

    volatile uint32 hif_int_coal_en_ch;
    volatile uint32 reserved4[3];
} channel_reg_t;

#define HIF_REGS_RESERVED2_GAP 0xf00/4

struct hif_regs
{
    volatile uint32 hif_version;
    volatile uint32 hif_tx_poll_ctrl;
    volatile uint32 hif_rx_poll_ctrl;
    volatile uint32 hif_misc;

    volatile uint32 hif_timeout_reg;
    volatile uint32 hif_soft_reset;
    volatile uint32 hif_doorbell_enable_reg1;
    volatile uint32 hif_doorbell_enable_reg2;

    volatile uint32 hif_single_bit_ecc_err0_en;
    volatile uint32 hif_single_bit_ecc_err0_status;
    volatile uint32 hif_multi_bit_ecc_err0_en;
    volatile uint32 hif_multi_bit_ecc_err0_status;

    volatile uint32 hif_multi_or_addr_bit_ecc_err0_en;
    volatile uint32 hif_multi_or_addr_bit_ecc_err0_status;
    volatile uint32 hif_addr_bit_ecc_err0_en;
    volatile uint32 hif_addr_bit_ecc_err0_status;

    volatile uint32 hif_int_src;
    volatile uint32 hif_int_src_reg2;
    volatile uint32 hif_int_src_reg3;
    volatile uint32 hif_wdt_int_reg1;

    volatile uint32 hif_wdt_int_reg2;
    volatile uint32 hif_rsvd_1;
    volatile uint32 hif_rsvd_2;
    volatile uint32 hif_rx_pkt_drop_en;

    volatile uint32 hif_rx_packet_drop_cnt;
    volatile uint32 hif_rx_packet_drop_threshold;
    volatile uint32 hif_err_int_src;
    volatile uint32 hif_err_int_en;

    volatile uint32 hif_tx_fifo_err_int_src;
    volatile uint32 hif_tx_fifo_err_int_en;
    volatile uint32 hif_rx_fifo_err_int_src;
    volatile uint32 hif_rx_fifo_err_int_en;

    volatile uint32 hif_tx_state;
    volatile uint32 hif_tx_actv;
    volatile uint32 hif_tx_curr_ch_no;
    volatile uint32 hif_dxr_tx_fifo_cnt;

    volatile uint32 hif_tx_ctrl_word_fifo_cnt1;
    volatile uint32 hif_tx_ctrl_word_fifo_cnt2;
    volatile uint32 hif_tx_bvalid_fifo_cnt;
    volatile uint32 hif_tx_pkt_cnt1;

    volatile uint32 hif_tx_pkt_cnt2;
    volatile uint32 hif_rx_state;
    volatile uint32 hif_rx_actv;
    volatile uint32 hif_rx_curr_ch_no;

    volatile uint32 hif_dxr_rx_fifo_cnt;
    volatile uint32 hif_rx_ctrl_word_fifo_cnt;
    volatile uint32 hif_rx_bvalid_fifo_cnt;
    volatile uint32 hif_rx_pkt_cnt1;

    volatile uint32 hif_rx_pkt_cnt2;
    volatile uint32 hif_dma_base_addr;
    volatile uint32 hif_dma_burst_size_addr;
    volatile uint32 hif_rx_queue_map_ch_no_addr;

    volatile uint32 hif_ltc_pkt_ctrl;
    volatile uint32 hif_rx_queue_map_ch_no_reg2_addr;
    volatile uint32 hif_rx_queue_map_ch_no_reg3_addr;
    volatile uint32 hif_rx_queue_map_ch_no_reg4_addr;

    volatile uint32 hif_single_bit_ecc_err1_en;
    volatile uint32 hif_single_bit_ecc_err1_status;
    volatile uint32 hif_multi_bit_ecc_err1_en;
    volatile uint32 hif_multi_bit_ecc_err1_status;

    volatile uint32 hif_multi_or_addr_bit_ecc_err1_en;
    volatile uint32 hif_multi_or_addr_bit_ecc_err1_status;
    volatile uint32 hif_addr_bit_ecc_err1_en;
    volatile uint32 hif_addr_bit_ecc_err1_status;
//    volatile uint32 reserved2[HIF_REGS_RESERVED2_GAP];
//    volatile channel_reg_t ch[NUM_HIF_CHANNELS];
};

/**********************************************************************
 * Function Name  : fp_assign_hif_base_addr
 * Description    : assign hif base address to global fp_private structure
 * Inputs
 *   Parameters   : struct fp_private *
 * Outputs        :
 *   Parameters   : -
 *   Returns      : -
 * Changes        :
 *********************************************************************/
static inline void fp_assign_hif_base_addr(struct fp_private *fp)
{
    INT hif_index = 0;
	INT ch = 0;
    UINT addr;

    addr =  HIF_BASE_ADDR;

	fp->hif[hif_index].regs = (struct hif_regs *)((unsigned long)(addr));

	for(ch = HIF_CHAN_START; ch < NUM_HIF_CHANNELS; ch++) {
		fp->hif[hif_index].ch_r[ch] = (struct channel_reg *)((unsigned long)(addr+(ch+1)*0x1000));
	}

}

/**********************************************************************
 * Function Name  : fp_en_hif_rx_engine 
 * Description    : enable hif rx dma engine
 * Inputs
 *   Parameters   : struct fp_private *
 * Outputs        :
 *   Parameters   : -
 *   Returns      : -
 * Changes        :
 *********************************************************************/
static inline void fp_en_hif_rxdma_engine(struct fp_private *fp)
{   
    UINT rx_ctrl = 0;
    UINT hif_index = 0, ch_index = 0;

    for(hif_index = 0; hif_index < NUM_TOTAL_HIF; hif_index++)
    {        
        /* CLEAR - Poll Counter */
        //CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].regs->hif_tx_poll_ctrl, 0x04000400);
        //CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].regs->hif_rx_poll_ctrl, 0x04000400);

        for(ch_index = HIF_CHAN_START; ch_index < NUM_HIF_CHANNELS; ch_index++)
        {
            /* Enable the RX DMA Engine & BDP Poll Cntrl */
            rx_ctrl = HIF_CH_RX_CTRL_DMA_EN | HIF_CH_RX_BDP_POLL_CNTR_EN;
            CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ctrl_ch, rx_ctrl);

            /* Enable rx write strobe */
            rx_ctrl = HIF_CH_RX_START;
            CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_rx_ch_start, rx_ctrl);
            /* Enable tx write strobe */
            rx_ctrl = HIF_CH_TX_START;
            CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_tx_ch_start, rx_ctrl);
        }
    }
}

static inline void fp_disable_hif_dma_engine(struct fp_private *fp)
{   
    UINT hif_index = 0, ch_index = 0;

    for(hif_index = 0; hif_index < NUM_TOTAL_HIF; hif_index++)
    {        
        /* CLEAR - Poll Counter */
        //CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].regs->hif_tx_poll_ctrl, 0x0);
        //CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].regs->hif_rx_poll_ctrl, 0x0);

        for(ch_index = HIF_CHAN_START; ch_index < NUM_HIF_CHANNELS; ch_index++)
        {
            /* Disable the RX DMA Engine & BDP Poll Cntrl */
            CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ctrl_ch, 0);
            CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_rx_ch_start, 0);
            CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_tx_ch_start, 0);
        }
    }
}

static inline void fp_disable_hif_tx_dma_engine(struct fp_private *fp)
{
    UINT hif_index = 0, ch_index = 0;
    
    for(hif_index = 0; hif_index < NUM_TOTAL_HIF; hif_index++)
    {        
        /* CLEAR - Poll Counter */
        CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].regs->hif_tx_poll_ctrl, 0x0);

        for(ch_index = HIF_CHAN_START; ch_index < NUM_HIF_CHANNELS; ch_index++)
        {
            /* Disable the RX DMA Engine & BDP Poll Cntrl */
            CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ctrl_ch, 0x00030000);
            CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_tx_ch_start, 0); 
        }
    }
}   

static inline void fp_disable_hif_rx_dma_engine(struct fp_private *fp)
{
    UINT hif_index = 0, ch_index = 0;
    
    for(hif_index = 0; hif_index < NUM_TOTAL_HIF; hif_index++)
    {
        /* CLEAR - Poll Counter */
        CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].regs->hif_rx_poll_ctrl, 0x0);

        for(ch_index = HIF_CHAN_START; ch_index < NUM_HIF_CHANNELS; ch_index++)
        {
            /* Disable the RX DMA Engine & BDP Poll Cntrl */
            CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ctrl_ch, 0x00000000);
            CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_rx_ch_start, 0);
        }
    }
}



void fp_init_bdp_base_reg(struct fp_private *fp);

#endif
