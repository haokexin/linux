/*
 * fp_pre_header.h
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C)2024Black Sesame Technologies. All Rights Reserved.
 */
#ifndef _FP_PRE_HEADER_H_
#define _FP_PRE_HEADER_H_

#define FP_TX_HDR_LEN 16
#define FP_RX_HDR_LEN 16
#define FP_EGR_TSR_LEN 16

#define FP_TX_PKT_INJECT_EN     (1 << 0)
#define FP_TX_LAUNCH_TIME_VALID (1 << 1)
#define FP_TX_PKT_PTP_EN        (1 << 2) 
#define FP_EGR_TSR_VALID        (1 << 4)
#define FP_PUNT_VALID           (1 << 5)
#define FP_RX_TS_VALID          (1 << 6)

#define FP_INJECT_TO_EMAC_1     (1 << 0)
#define FP_INJECT_TO_EMAC_2     (1 << 1)
#define FP_INJECT_TO_HIF_1      (1 << 2)
#define FP_INJECT_TO_HIF_2      (1 << 3)

/* Punt reason codes */
#define FP_PUNT_L2_SPL          (1 << 0)
#define FP_PUNT_SA_MISS         (1 << 1)
#define FP_PUNT_SA_RELEARN      (1 << 2)
#define FP_PUNT_SA_IS_ACTIVE    (1 << 3)
#define FP_PUNT_SNOOP_UPPER     (1 << 4)
#define FP_PUNT_REQ             (1 << 5)
#define FP_PUNT_MGMT            (1 << 6)
#define FP_PUNT_IGMP            (1 << 7)
#define FP_PUNT_FLOOD           (1 << 8)
#define FP_PUNT_PARSE           (1 << 9)

struct tx_header
{
    UINT queue: 4; 
    UINT txport_map: 20;
    UINT ctrl: 8;

	UINT seq_num:16;
	UINT rx_ch_no:6;
	UINT ipsec_pbc:1;
	UINT ipsec_mcast_bcast:1;
	UINT ipsec_seq_num_valid:1;
	UINT ipsec_seq_num:2;	
    UINT rsvd1:5;	
	
    UINT ipsec_sap;
	
    USHORT ipsec_iphy;
    USHORT ipsec_ophy;
};

struct rx_header
{
    USHORT punt_reason;
    UCHAR rxport_num;
    UCHAR ctrl;
    UINT rsvd;
    UINT rx_timestamp_nsec; 
    UINT rx_timestamp_sec; 
};

struct egress_report
{
    UCHAR rsvd[3];
    UCHAR ctrl;
    UINT rsvd1;
    UINT egress_timestamp_nsec;
    UINT egress_timestamp_sec;
    UCHAR rsvd2;
    UCHAR rxport_num;
    USHORT ref_num;
};

/* forward declaration to avoid warnings */
struct sk_buff;

int fp_fill_tx_ptp_header(struct tx_header *txhdr, struct sk_buff *skb);
int fp_process_rx_header(struct sk_buff *skb);

#define FLOOD_TO_EMAC_PORTS  1
#ifdef FLOOD_TO_EMAC_PORTS
#define CB_INJ_TX_FLAG  47
#define CB_INJ_TX_PORT  42
#define CB_TX_SA_MISS   0x40
void fp_tx_inject_packet(struct sk_buff *skb, int rx_port);
#endif
#endif  /*End of _FP_PRE_HEADER_H_*/

