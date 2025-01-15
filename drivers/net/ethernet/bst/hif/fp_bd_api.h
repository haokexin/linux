/*
 * fp_bd_api.h
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C)2024Black Sesame Technologies. All Rights Reserved.
 */

#ifndef _FP_BD_API_H
#define _FP_BD_API_H

#include "fp_pre_header.h"

/* buffer descr ctrl flags */
#define BD_CTRL_BD_BUFLEN_MASK   (0xffff)
#define BD_CTRL_PKT_INT_EN       (1 << 0)
#define BD_CTRL_CBD_INT_EN       (1 << 1)
#define BD_CTRL_LIFM             (1 << 2)
#define BD_CTRL_LAST_BD          (1 << 3)
#define BD_CTRL_DIR              (1 << 4)
#define BD_CTRL_PKT_XFER         (1 << 8)
#define BD_CTRL_DESC_EN          (1 << 15)
#define BD_CTRL_PARSE_DISABLE    (1 << 9)
#define BD_CTRL_BRFETCH_DISABLE  (1 << 10)
#define BD_CTRL_RTFETCH_DISABLE  (1 << 11)

#define WB_BD_CTRL_DESC_EN       (1 << 9)

#define NUM_BD_DESCR		 1000
#define NUM_TX_DESCR             NUM_BD_DESCR
#define NUM_RX_DESCR             NUM_BD_DESCR
#define NUM_TX_WB_DESCR          NUM_BD_DESCR
#define NUM_RX_WB_DESCR          NUM_BD_DESCR
#define MAX_RX_BUFF_SIZE            PAGE_SIZE
//#define MAX_RX_BUFF_SIZE            1024
#define MAX_TX_BUFF_SIZE            (2 * 1024)
#define MAX_TX_PKT_BUFF_SIZE        (MAX_TX_BUFF_SIZE - FP_TX_HDR_LEN)
#define TX_BD_INIT_RING          1
#define RX_BD_INIT_RING          2
#define ALIGN_64_BIT_MASK        64
#define NUM_TOTAL_DESCR          (NUM_TX_DESCR + NUM_RX_DESCR + NUM_WB_DESCR)
#define HIF_TX_THRESH	         (NUM_TX_DESCR / 4)
#define HIF_RX_THRESH	         (NUM_RX_DESCR / 4)

#define NUM_RX_BD_LIMIT        50


#define WB_BD_LIFM          BIT(6)

#define TX_BD_USE_COUNT(S) DIV_ROUND_UP((S), MAX_TX_PKT_BUFF_SIZE - 16)

struct bd {
    u16 bd_seqnum;
    u16 bd_ctrl;
    u16 bd_buflen;
	u8  msb8_bd_bufaddr;
	u8  msb8_bd_nextptr; 
    u32 bd_bufaddr;
    u32 bd_nextptr;
};

struct wb_bd {
    u32 bd_ctrl; /* rsvd-[31:11] ctrl[10:0] */
    u16 bd_buflen; /* seq-[31:16] buflen-[15:0] */
    u16 bd_seqnum;
};

struct bd_tx_buffer {
	struct sk_buff *skb;
	dma_addr_t dma;
	u32 len;
//	u32 tx_flags;
};

struct bd_rx_buffer {
	union {
		struct page *page; /* jumbo: alloc_page */
		u8 *data; /* else, netdev_alloc_frag */
	} rxbuf;
	dma_addr_t dma;
	//	__u16 pagecnt_bias;
};


struct bd_tx_ring_stats {
	u64 packets;
	u64 bytes;
	u64 restart_queue;
};

struct bd_rx_ring_stats {
	u64 packets;
	u64 bytes;
	u64 drops;
	u64 alloc_failed;
};

struct bd_tx_ring {
    spinlock_t lock;
    INT          ring_index;             
	INT          ch_index;
    INT          queue_index;
	struct bd_tx_ring_stats stats;
    struct bd    *bd_tbl_va;         /* bd buffer虚拟内存地址 */       /* 64-bit aligned */
    struct wb_bd *wb_bd_tbl_va;      /* wb bd buffer虚拟内存地址 */     /* 64 bit aligned */
    struct wb_bd *wb_read_ptr;       /* 指向硬件下一个更新的wb bd */
    dma_addr_t   bd_tbl_pa;          /* bd buffer物理地址 */
    dma_addr_t   wb_bd_tbl_pa;       /* wb bd buffer物理内存地址 */
//    dma_addr_t   bd_buff_pa;         /* packet buffer 物理内存地址 */
    dma_addr_t   dev_addr;           /* total buffer physical address = bd + wb bd + packet*/
    void         *cpu_addr;          /* total buffer virtual address = bd + wb bd + packet */
	struct bd_tx_buffer *buffer_info;
//    void         *bd_buff_va;        /* packet buffer 虚拟内存地址 */
    struct bd    *head;              /* 指向硬件下一个读取的bd */
    struct bd    *tail;              /* 指向最后一个bd */
    INT          index;              /* wb bd 虚拟内存地址 */
    INT          free_bd_cnt;        /* wb bd 虚拟内存地址 */
    INT          wb_seq_num;         /* wb bd seq num，下一个添加的wb bd使用此值 */
    INT          seq_num;            /* bd seq num，下一个添加的bd使用此值 */
    INT          ring_len;           /* bd num */
    INT          size;               /* total buffer size , include bd buffer size, wb bd buffer size*/
    INT          wb_ring_len;        /* wb bd num */
    INT          flag;               /* flag */
};

struct bd_rx_ring {
    spinlock_t lock;
    INT          ring_index; 
	INT          ch_index;
	struct bd_rx_ring_stats stats;
    struct bd    *bd_tbl_va;         /* bd buffer虚拟内存地址 */       /* 64-bit aligned */
    struct wb_bd *wb_bd_tbl_va;      /* wb bd buffer虚拟内存地址 */     /* 64 bit aligned */
    struct wb_bd *wb_read_ptr;       /* 指向硬件下一个更新的wb bd */
    dma_addr_t   bd_tbl_pa;          /* bd buffer物理地址 */
    dma_addr_t   wb_bd_tbl_pa;       /* wb bd buffer物理内存地址 */
//    dma_addr_t   bd_buff_pa;         /* packet buffer 物理内存地址 */
    dma_addr_t   dev_addr;           /* total buffer physical address = bd + wb bd + packet*/
    void         *cpu_addr;          /* total buffer virtual address = bd + wb bd + packet */
	struct bd_rx_buffer *buffer_info;
	struct sk_buff *rx_skb_top;
//    void         *bd_buff_va;        /* packet buffer 虚拟内存地址 */
    struct bd    *head;              /* 指向硬件下一个读取的bd */
    struct bd    *tail;              /* 指向最后一个bd */
	INT          clean_start;
    INT          index;              /* wb bd 虚拟内存地址 */
    INT          free_bd_cnt;        /* wb bd 虚拟内存地址 */
    INT          wb_seq_num;         /* wb bd seq num，下一个添加的wb bd使用此值 */
    INT          seq_num;            /* bd seq num，下一个添加的bd使用此值 */
    INT          ring_len;           /* bd num */
    INT          size;               /* total buffer size , include bd buffer size, wb bd buffer size, packet buffer size*/
    INT          wb_ring_len;        /* wb bd num */
    INT          flag;               /* flag */
};
extern UINT fp_total_ring_size;


int fp_bd_create_ring(struct fp_private *fp);
int fp_bd_destroy_ring(struct fp_private *fp);
int fp_bd_setup_all_tx_rings(struct fp_private *fp);
int fp_bd_setup_all_rx_rings(struct fp_private *fp);
int fp_bd_init_all_tx_rings(struct fp_private *fp);
int fp_bd_init_all_rx_rings(struct fp_private *fp);
int fp_bd_uninit_all_rx_rings(struct fp_private *fp);
void fp_bd_free_all_tx_rings(struct fp_private *fp);
void fp_bd_free_all_rx_rings(struct fp_private *fp);
struct bd *fp_get_next_free_tx_bd(struct bd_tx_ring *ring);
u32 fp_get_curr_rx_bd_index(struct bd_rx_ring *ring, struct bd *bd);
u32 fp_get_curr_tx_bd_index(struct bd_tx_ring *ring, struct bd *bd);
int fp_bd_init_rx_ring(struct fp_private *fp, struct bd_rx_ring *ring, int start, int len, bool init);
int fp_enque_rx_bd(struct fp_private *fp, struct bd_rx_ring *ring, struct bd *curr_bd);
int fp_enque_tx_bd(struct fp_private *fp, struct bd_tx_ring *ring, struct bd *curr_bd);
struct bd *fp_deque_rx_bd(struct bd_rx_ring *ring, u32 *wb_buflen, bool *lifm);
struct bd *fp_deque_tx_bd(struct bd_tx_ring *ring, u32 *wb_buflen, bool *lifm);
void fp_dump_bd_info(struct bd *dump_bd);
int fp_bd_init(struct fp_private *fp);
void handle_hif(struct hif_base *hif);
void handle_channel(struct hif_base *hif, UINT ch_index);
void fp_disable_hif_ch_interrupts(struct hif_base *hif);
void fp_enable_hif_ch_interrupts(struct hif_base *hif);
inline int fp_rx_bd_used(struct bd_rx_ring *ring);
inline int fp_tx_bd_unused(struct bd_tx_ring *ring);
int fp_bd_reinit(struct fp_private *fp);

#endif /* _FP_BD_TABLE_H */

