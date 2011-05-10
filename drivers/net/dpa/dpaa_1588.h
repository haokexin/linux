/*
 * drivers/net/dpa/dpaa_1588.h
 *
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */
#ifndef __DPAA_1588_H__
#define __DPAA_1588_H__

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/circ_buf.h>
#include <linux/fsl_qman.h>

/* The temporary major number used by PTP driver */
#define PTP_MAJOR			232

#define DEFAULT_RX_QUEUE_SIZE		2048
#define DEFAULT_TX_QUEUE_SIZE		16

#define PTP_MSG_SYNC			0x0
#define PTP_MSG_DEL_REQ			0x1
#define PTP_MSG_P_DEL_REQ		0x2
#define PTP_MSG_P_DEL_RESP		0x3
#define PTP_MSG_DEL_RESP		0x4
#define PTP_MSG_ALL_OTHER		0x5

/* IOCTL macro */
#define PTP_GET_TX_TIMESTAMP		0x1
#define PTP_GET_RX_TIMESTAMP		0x9
#define PTP_SET_RTC_TIME		0x3
#define PTP_SET_COMPENSATION		0x4
#define PTP_GET_CURRENT_TIME		0x5
#define PTP_FLUSH_TIMESTAMP		0x6
#define PTP_ADJ_ADDEND			0x7
#define PTP_GET_ORIG_COMP		0x8
#define PTP_GET_ADDEND			0xB
#define PTP_GET_RX_TIMESTAMP_PDELAY_REQ		0xC
#define PTP_GET_RX_TIMESTAMP_PDELAY_RESP	0xD

/* byte offset of data in the PTP V2 headers */
#define k_OFFS_MSG_TYPE			0
#define k_OFFS_VER_PTP			1
#define k_OFFS_MSG_LEN			2
#define k_OFFS_DOM_NMB			4
#define k_OFFS_FLAGS			6
#define k_OFFS_CORFIELD			8
#define k_OFFS_SRCPRTID			20
#define k_OFFS_SEQ_ID			30
#define k_OFFS_CTRL			32
#define k_OFFS_LOGMEAN			33

#define PTP_IP_OFFS			14
#define PTP_UDP_OFFS			34
#define PTP_HEADER_OFFS			42
#define PTP_MSG_TYPE_OFFS		(PTP_HEADER_OFFS + k_OFFS_MSG_TYPE)
#define PTP_SPORT_ID_OFFS		(PTP_HEADER_OFFS + k_OFFS_SRCPRTID)
#define PTP_SEQ_ID_OFFS			(PTP_HEADER_OFFS + k_OFFS_SEQ_ID)
#define PTP_CTRL_OFFS			(PTP_HEADER_OFFS + k_OFFS_CTRL)

#define DPA_PTP_TIMESTAMP_OFFSET	0x30
#define DPA_PTP_NOMINAL_FREQ_PERIOD	0xa /* 10ns -> 100M */
#define NANOSEC_PER_SECOND		1000000000

/* PTP standard time representation structure */
struct ptp_time{
	u64 high;
	u32 low;
};

/* Structure for PTP Time Stamp */
struct dpa_ptp_data_t {
	u8		spid[10];
	int		key;
	struct ptp_time	ts_time;
};

/* Interface for PTP driver command GET_TX_TIME */
struct ptp_ts_data {
	/* PTP version */
	u8 version;
	/* PTP source port ID */
	u8 spid[10];
	/* PTP sequence ID */
	u16 seq_id;
	/* PTP message type */
	u8 message_type;
	/* PTP timestamp */
	struct ptp_time ts;
} __attribute__((packed));

/* Interface for PTP driver command SET_RTC_TIME/GET_CURRENT_TIME */
struct ptp_rtc_time {
	struct ptp_time rtc_time;
};

/* Interface for PTP driver command SET_COMPENSATION */
struct ptp_set_comp {
	u32 drift;
	bool o_ops;
	u32 freq_compensation;
};

/* Interface for PTP driver command GET_ORIG_COMP */
struct ptp_get_comp {
	/* the initial compensation value */
	u32 dw_origcomp;
	/* the minimum compensation value */
	u32 dw_mincomp;
	/*the max compensation value*/
	u32 dw_maxcomp;
	/*the min drift applying min compensation value in ppm*/
	u32 dw_mindrift;
	/*the max drift applying max compensation value in ppm*/
	u32 dw_maxdrift;
};

/* PTP TSU control structure */
struct ptp_tsu {
	struct dpa_priv_s *dpa_priv;
	bool valid;
	struct circ_buf rx_time_sync;
	struct circ_buf rx_time_del_req;
	struct circ_buf rx_time_pdel_req;
	struct circ_buf rx_time_pdel_resp;
	struct circ_buf tx_time_sync;
	struct circ_buf tx_time_del_req;
	struct circ_buf tx_time_pdel_req;
	struct circ_buf tx_time_pdel_resp;
	spinlock_t ptp_lock;

	u32 rx_time_queue_size;
	u32 tx_time_queue_size;
};

extern int dpa_ptp_init(struct dpa_priv_s *priv);
extern void dpa_ptp_cleanup(struct dpa_priv_s *priv);
extern int dpa_ptp_do_txstamp(struct sk_buff *skb);
extern void dpa_ptp_store_txstamp(struct net_device *dev, struct sk_buff *skb,
				  const struct qm_fd *fd);
extern void dpa_ptp_store_rxstamp(struct net_device *dev, struct sk_buff *skb,
				  const struct qm_fd *fd);

#endif
