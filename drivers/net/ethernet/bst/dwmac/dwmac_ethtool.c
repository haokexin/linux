// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2007-2009 STMicroelectronics Ltd
 */

#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/mii.h>
#include <linux/phylink.h>
#include <linux/net_tstamp.h>
#include <linux/io.h>

#include "bstgmac.h"
#include "dwmac_dma.h"

#define REG_SPACE_SIZE	0x1060
#define GMAC_ETHTOOL_NAME	"st_gmac"
#define XGMAC_ETHTOOL_NAME	"st_xgmac"

#define ETHTOOL_DMA_OFFSET	55

struct bstgmac_stats {
	char stat_string[ETH_GSTRING_LEN];
	int sizeof_stat;
	int stat_offset;
};

#define BSTGMAC_STAT(m)	\
	{ #m, sizeof_field(struct bstgmac_extra_stats, m),	\
	offsetof(struct bstgmac_priv, xstats.m)}

static const struct bstgmac_stats bstgmac_gstrings_stats[] = {
	/* Transmit errors */
	BSTGMAC_STAT(tx_underflow),
	BSTGMAC_STAT(tx_carrier),
	BSTGMAC_STAT(tx_losscarrier),
	BSTGMAC_STAT(vlan_tag),
	BSTGMAC_STAT(tx_deferred),
	BSTGMAC_STAT(tx_vlan),
	BSTGMAC_STAT(tx_jabber),
	BSTGMAC_STAT(tx_frame_flushed),
	BSTGMAC_STAT(tx_payload_error),
	BSTGMAC_STAT(tx_ip_header_error),
	/* Receive errors */
	BSTGMAC_STAT(rx_desc),
	BSTGMAC_STAT(sa_filter_fail),
	BSTGMAC_STAT(overflow_error),
	BSTGMAC_STAT(ipc_csum_error),
	BSTGMAC_STAT(rx_collision),
	BSTGMAC_STAT(rx_crc_errors),
	BSTGMAC_STAT(dribbling_bit),
	BSTGMAC_STAT(rx_length),
	BSTGMAC_STAT(rx_mii),
	BSTGMAC_STAT(rx_multicast),
	BSTGMAC_STAT(rx_gmac_overflow),
	BSTGMAC_STAT(rx_watchdog),
	BSTGMAC_STAT(da_rx_filter_fail),
	BSTGMAC_STAT(sa_rx_filter_fail),
	BSTGMAC_STAT(rx_missed_cntr),
	BSTGMAC_STAT(rx_overflow_cntr),
	BSTGMAC_STAT(rx_vlan),
	/* Tx/Rx IRQ error info */
	BSTGMAC_STAT(tx_underflow_irq),
	BSTGMAC_STAT(tx_buf_unav_irq),
	BSTGMAC_STAT(tx_process_stopped_irq),
	BSTGMAC_STAT(tx_jabber_irq),
	BSTGMAC_STAT(rx_overflow_irq),
	BSTGMAC_STAT(rx_buf_unav_irq),
	BSTGMAC_STAT(rx_process_stopped_irq),
	BSTGMAC_STAT(rx_watchdog_irq),
	BSTGMAC_STAT(tx_early_irq),
	BSTGMAC_STAT(fatal_bus_error_irq),
	/* Tx/Rx IRQ Events */
	BSTGMAC_STAT(rx_early_irq),
	BSTGMAC_STAT(threshold),
	BSTGMAC_STAT(tx_pkt_n),
	BSTGMAC_STAT(rx_pkt_n),
	BSTGMAC_STAT(normal_irq_n),
	BSTGMAC_STAT(rx_normal_irq_n),
	BSTGMAC_STAT(napi_poll),
	BSTGMAC_STAT(rnapi_poll),
	BSTGMAC_STAT(rx_memwork_poll),
	BSTGMAC_STAT(tnapi_poll),
	BSTGMAC_STAT(txwork_poll),
	BSTGMAC_STAT(tx_normal_irq_n),
	BSTGMAC_STAT(tx_clean),
	BSTGMAC_STAT(tx_set_ic_bit),
	BSTGMAC_STAT(irq_receive_pmt_irq_n),
	/* MMC info */
	BSTGMAC_STAT(mmc_tx_irq_n),
	BSTGMAC_STAT(mmc_rx_irq_n),
	BSTGMAC_STAT(mmc_rx_csum_offload_irq_n),
	/* EEE */
	BSTGMAC_STAT(irq_tx_path_in_lpi_mode_n),
	BSTGMAC_STAT(irq_tx_path_exit_lpi_mode_n),
	BSTGMAC_STAT(irq_rx_path_in_lpi_mode_n),
	BSTGMAC_STAT(irq_rx_path_exit_lpi_mode_n),
	BSTGMAC_STAT(phy_eee_wakeup_error_n),
	/* Extended RDES status */
	BSTGMAC_STAT(ip_hdr_err),
	BSTGMAC_STAT(ip_payload_err),
	BSTGMAC_STAT(ip_csum_bypassed),
	BSTGMAC_STAT(ipv4_pkt_rcvd),
	BSTGMAC_STAT(ipv6_pkt_rcvd),
	BSTGMAC_STAT(no_ptp_rx_msg_type_ext),
	BSTGMAC_STAT(ptp_rx_msg_type_sync),
	BSTGMAC_STAT(ptp_rx_msg_type_follow_up),
	BSTGMAC_STAT(ptp_rx_msg_type_delay_req),
	BSTGMAC_STAT(ptp_rx_msg_type_delay_resp),
	BSTGMAC_STAT(ptp_rx_msg_type_pdelay_req),
	BSTGMAC_STAT(ptp_rx_msg_type_pdelay_resp),
	BSTGMAC_STAT(ptp_rx_msg_type_pdelay_follow_up),
	BSTGMAC_STAT(ptp_rx_msg_type_announce),
	BSTGMAC_STAT(ptp_rx_msg_type_management),
	BSTGMAC_STAT(ptp_rx_msg_pkt_reserved_type),
	BSTGMAC_STAT(ptp_frame_type),
	BSTGMAC_STAT(ptp_ver),
	BSTGMAC_STAT(timestamp_dropped),
	BSTGMAC_STAT(av_pkt_rcvd),
	BSTGMAC_STAT(av_tagged_pkt_rcvd),
	BSTGMAC_STAT(vlan_tag_priority_val),
	BSTGMAC_STAT(l3_filter_match),
	BSTGMAC_STAT(l4_filter_match),
	BSTGMAC_STAT(l3_l4_filter_no_match),
	/* PCS */
	BSTGMAC_STAT(irq_pcs_ane_n),
	BSTGMAC_STAT(irq_pcs_link_n),
	BSTGMAC_STAT(irq_rgmii_n),
	/* DEBUG */
	BSTGMAC_STAT(mtl_tx_status_fifo_full),
	BSTGMAC_STAT(mtl_tx_fifo_not_empty),
	BSTGMAC_STAT(mmtl_fifo_ctrl),
	BSTGMAC_STAT(mtl_tx_fifo_read_ctrl_write),
	BSTGMAC_STAT(mtl_tx_fifo_read_ctrl_wait),
	BSTGMAC_STAT(mtl_tx_fifo_read_ctrl_read),
	BSTGMAC_STAT(mtl_tx_fifo_read_ctrl_idle),
	BSTGMAC_STAT(mac_tx_in_pause),
	BSTGMAC_STAT(mac_tx_frame_ctrl_xfer),
	BSTGMAC_STAT(mac_tx_frame_ctrl_idle),
	BSTGMAC_STAT(mac_tx_frame_ctrl_wait),
	BSTGMAC_STAT(mac_tx_frame_ctrl_pause),
	BSTGMAC_STAT(mac_gmii_tx_proto_engine),
	BSTGMAC_STAT(mtl_rx_fifo_fill_level_full),
	BSTGMAC_STAT(mtl_rx_fifo_fill_above_thresh),
	BSTGMAC_STAT(mtl_rx_fifo_fill_below_thresh),
	BSTGMAC_STAT(mtl_rx_fifo_fill_level_empty),
	BSTGMAC_STAT(mtl_rx_fifo_read_ctrl_flush),
	BSTGMAC_STAT(mtl_rx_fifo_read_ctrl_read_data),
	BSTGMAC_STAT(mtl_rx_fifo_read_ctrl_status),
	BSTGMAC_STAT(mtl_rx_fifo_read_ctrl_idle),
	BSTGMAC_STAT(mtl_rx_fifo_ctrl_active),
	BSTGMAC_STAT(mac_rx_frame_ctrl_fifo),
	BSTGMAC_STAT(mac_gmii_rx_proto_engine),
	/* TSO */
	BSTGMAC_STAT(tx_tso_frames),
	BSTGMAC_STAT(tx_tso_nfrags),
};

#define BSTGMAC_STATS_LEN ARRAY_SIZE(bstgmac_gstrings_stats)

/* HW MAC Management counters (if supported) */
#define BSTGMAC_MMC_STAT(m)	\
	{ #m, sizeof_field(struct bstgmac_counters, m),	\
	offsetof(struct bstgmac_priv, mmc.m)}

static const struct bstgmac_stats bstgmac_mmc[] = {
	BSTGMAC_MMC_STAT(mmc_tx_octetcount_gb),
	BSTGMAC_MMC_STAT(mmc_tx_framecount_gb),
	BSTGMAC_MMC_STAT(mmc_tx_broadcastframe_g),
	BSTGMAC_MMC_STAT(mmc_tx_multicastframe_g),
	BSTGMAC_MMC_STAT(mmc_tx_64_octets_gb),
	BSTGMAC_MMC_STAT(mmc_tx_65_to_127_octets_gb),
	BSTGMAC_MMC_STAT(mmc_tx_128_to_255_octets_gb),
	BSTGMAC_MMC_STAT(mmc_tx_256_to_511_octets_gb),
	BSTGMAC_MMC_STAT(mmc_tx_512_to_1023_octets_gb),
	BSTGMAC_MMC_STAT(mmc_tx_1024_to_max_octets_gb),
	BSTGMAC_MMC_STAT(mmc_tx_unicast_gb),
	BSTGMAC_MMC_STAT(mmc_tx_multicast_gb),
	BSTGMAC_MMC_STAT(mmc_tx_broadcast_gb),
	BSTGMAC_MMC_STAT(mmc_tx_underflow_error),
	BSTGMAC_MMC_STAT(mmc_tx_singlecol_g),
	BSTGMAC_MMC_STAT(mmc_tx_multicol_g),
	BSTGMAC_MMC_STAT(mmc_tx_deferred),
	BSTGMAC_MMC_STAT(mmc_tx_latecol),
	BSTGMAC_MMC_STAT(mmc_tx_exesscol),
	BSTGMAC_MMC_STAT(mmc_tx_carrier_error),
	BSTGMAC_MMC_STAT(mmc_tx_octetcount_g),
	BSTGMAC_MMC_STAT(mmc_tx_framecount_g),
	BSTGMAC_MMC_STAT(mmc_tx_excessdef),
	BSTGMAC_MMC_STAT(mmc_tx_pause_frame),
	BSTGMAC_MMC_STAT(mmc_tx_vlan_frame_g),
	BSTGMAC_MMC_STAT(mmc_rx_framecount_gb),
	BSTGMAC_MMC_STAT(mmc_rx_octetcount_gb),
	BSTGMAC_MMC_STAT(mmc_rx_octetcount_g),
	BSTGMAC_MMC_STAT(mmc_rx_broadcastframe_g),
	BSTGMAC_MMC_STAT(mmc_rx_multicastframe_g),
	BSTGMAC_MMC_STAT(mmc_rx_crc_error),
	BSTGMAC_MMC_STAT(mmc_rx_align_error),
	BSTGMAC_MMC_STAT(mmc_rx_run_error),
	BSTGMAC_MMC_STAT(mmc_rx_jabber_error),
	BSTGMAC_MMC_STAT(mmc_rx_undersize_g),
	BSTGMAC_MMC_STAT(mmc_rx_oversize_g),
	BSTGMAC_MMC_STAT(mmc_rx_64_octets_gb),
	BSTGMAC_MMC_STAT(mmc_rx_65_to_127_octets_gb),
	BSTGMAC_MMC_STAT(mmc_rx_128_to_255_octets_gb),
	BSTGMAC_MMC_STAT(mmc_rx_256_to_511_octets_gb),
	BSTGMAC_MMC_STAT(mmc_rx_512_to_1023_octets_gb),
	BSTGMAC_MMC_STAT(mmc_rx_1024_to_max_octets_gb),
	BSTGMAC_MMC_STAT(mmc_rx_unicast_g),
	BSTGMAC_MMC_STAT(mmc_rx_length_error),
	BSTGMAC_MMC_STAT(mmc_rx_autofrangetype),
	BSTGMAC_MMC_STAT(mmc_rx_pause_frames),
	BSTGMAC_MMC_STAT(mmc_rx_fifo_overflow),
	BSTGMAC_MMC_STAT(mmc_rx_vlan_frames_gb),
	BSTGMAC_MMC_STAT(mmc_rx_watchdog_error),
	BSTGMAC_MMC_STAT(mmc_rx_ipc_intr_mask),
	BSTGMAC_MMC_STAT(mmc_rx_ipc_intr),
	BSTGMAC_MMC_STAT(mmc_rx_ipv4_gd),
	BSTGMAC_MMC_STAT(mmc_rx_ipv4_hderr),
	BSTGMAC_MMC_STAT(mmc_rx_ipv4_nopay),
	BSTGMAC_MMC_STAT(mmc_rx_ipv4_frag),
	BSTGMAC_MMC_STAT(mmc_rx_ipv4_udsbl),
	BSTGMAC_MMC_STAT(mmc_rx_ipv4_gd_octets),
	BSTGMAC_MMC_STAT(mmc_rx_ipv4_hderr_octets),
	BSTGMAC_MMC_STAT(mmc_rx_ipv4_nopay_octets),
	BSTGMAC_MMC_STAT(mmc_rx_ipv4_frag_octets),
	BSTGMAC_MMC_STAT(mmc_rx_ipv4_udsbl_octets),
	BSTGMAC_MMC_STAT(mmc_rx_ipv6_gd_octets),
	BSTGMAC_MMC_STAT(mmc_rx_ipv6_hderr_octets),
	BSTGMAC_MMC_STAT(mmc_rx_ipv6_nopay_octets),
	BSTGMAC_MMC_STAT(mmc_rx_ipv6_gd),
	BSTGMAC_MMC_STAT(mmc_rx_ipv6_hderr),
	BSTGMAC_MMC_STAT(mmc_rx_ipv6_nopay),
	BSTGMAC_MMC_STAT(mmc_rx_udp_gd),
	BSTGMAC_MMC_STAT(mmc_rx_udp_err),
	BSTGMAC_MMC_STAT(mmc_rx_tcp_gd),
	BSTGMAC_MMC_STAT(mmc_rx_tcp_err),
	BSTGMAC_MMC_STAT(mmc_rx_icmp_gd),
	BSTGMAC_MMC_STAT(mmc_rx_icmp_err),
	BSTGMAC_MMC_STAT(mmc_rx_udp_gd_octets),
	BSTGMAC_MMC_STAT(mmc_rx_udp_err_octets),
	BSTGMAC_MMC_STAT(mmc_rx_tcp_gd_octets),
	BSTGMAC_MMC_STAT(mmc_rx_tcp_err_octets),
	BSTGMAC_MMC_STAT(mmc_rx_icmp_gd_octets),
	BSTGMAC_MMC_STAT(mmc_rx_icmp_err_octets),
	BSTGMAC_MMC_STAT(mmc_tx_fpe_fragment_cntr),
	BSTGMAC_MMC_STAT(mmc_tx_hold_req_cntr),
	BSTGMAC_MMC_STAT(mmc_rx_packet_assembly_err_cntr),
	BSTGMAC_MMC_STAT(mmc_rx_packet_smd_err_cntr),
	BSTGMAC_MMC_STAT(mmc_rx_packet_assembly_ok_cntr),
	BSTGMAC_MMC_STAT(mmc_rx_fpe_fragment_cntr),
};

#define BSTGMAC_MMC_STATS_LEN ARRAY_SIZE(bstgmac_mmc)

static void bstgmac_ethtool_getdrvinfo(struct net_device *dev,
				       struct ethtool_drvinfo *info)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	if (priv->plat->has_gmac || priv->plat->has_gmac4)
		strlcpy(info->driver, GMAC_ETHTOOL_NAME, sizeof(info->driver));
	else if (priv->plat->has_xgmac)
		strlcpy(info->driver, XGMAC_ETHTOOL_NAME, sizeof(info->driver));
	else
		strlcpy(info->driver, GMAC_ETHTOOL_NAME,
			sizeof(info->driver));

	strlcpy(info->version, DRV_MODULE_VERSION, sizeof(info->version));
}

static int bstgmac_ethtool_get_link_ksettings(struct net_device *dev,
					      struct ethtool_link_ksettings
					      *cmd)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	u32 supported, advertising, lp_advertising;

	if (priv->plat->bypass)
		return -ENODEV;

	if (!priv->phylink)
		return -EOPNOTSUPP;

	phylink_ethtool_ksettings_get(priv->phylink, cmd);
	
	ethtool_convert_link_mode_to_legacy_u32(&supported,
							cmd->link_modes.supported);
	ethtool_convert_link_mode_to_legacy_u32(&advertising,
						cmd->link_modes.advertising);
	ethtool_convert_link_mode_to_legacy_u32(&lp_advertising,
						cmd->link_modes.lp_advertising);

	cmd->base.autoneg = 0;
	supported &= (~SUPPORTED_Autoneg);
	advertising &= (~ADVERTISED_Autoneg);
	lp_advertising &= (~ADVERTISED_Autoneg);

	supported |= SUPPORTED_Pause | SUPPORTED_Asym_Pause;
	advertising |= ADVERTISED_Pause | ADVERTISED_Asym_Pause;
	lp_advertising |= ADVERTISED_Pause | ADVERTISED_Asym_Pause;

	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.supported,
							supported);
	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.advertising,
						advertising);
	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.lp_advertising,
						lp_advertising);

	return 0;
}

static int
bstgmac_ethtool_set_link_ksettings(struct net_device *dev,
				   const struct ethtool_link_ksettings *cmd)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	struct phy_device *phy = dev->phydev;
	int rc;

	if (!phy) {
		pr_err("%s: %s: No PHY\n", __func__, dev->name);
		return -ENODEV;
	}

	if (cmd->base.speed != SPEED_1000 ||
	    cmd->base.autoneg != AUTONEG_ENABLE) {
		netdev_info(dev, "Only support speed 1000 and anto enable\n");
		return -EINVAL;
	}

	if (priv->hw->pcs & BSTGMAC_PCS_SGMII) {
		u32 mask = ADVERTISED_Autoneg | ADVERTISED_Pause;

		/* Only support ANE */
		if (cmd->base.autoneg != AUTONEG_ENABLE)
			return -EINVAL;

		mask &= (ADVERTISED_1000baseT_Half |
			 ADVERTISED_1000baseT_Full |
			 ADVERTISED_100baseT_Half |
			 ADVERTISED_100baseT_Full |
			 ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full);

		mutex_lock(&priv->lock);
		bstgmac_pcs_ctrl_ane(priv, priv->ioaddr, 1, priv->hw->ps, 0);
		mutex_unlock(&priv->lock);

		return 0;
	}

	rc = phy_ethtool_ksettings_set(phy, cmd);

	return rc;
}

static u32 bstgmac_ethtool_getmsglevel(struct net_device *dev)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	return priv->msg_enable;
}

static void bstgmac_ethtool_setmsglevel(struct net_device *dev, u32 level)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	unsigned long msglvl = (unsigned long)level, value = 0;

	netdev_dbg(dev, "%s(), level = 0x%x\n", __func__, level);
	if ((msglvl >> 28) == BSTGMAC_PKT_CAPTURE_OUI) {
		value = (msglvl & 0xffff);

		if (test_bit(27, &msglvl))
			priv->pkt_capture.tx = true;
		else
			priv->pkt_capture.tx = false;

		if (test_bit(26, &msglvl))
			priv->pkt_capture.rx = true;
		else
			priv->pkt_capture.rx = false;

		if (test_bit(25, &msglvl)) {
			if (value >= 1 && value <= 4094) {
				priv->pkt_capture.tci = value;
				priv->pkt_capture.vlan = true;
				priv->pkt_capture.ether_type = 0;
			}
		} else {
			if ((value == ETH_P_IP || value == ETH_P_IPV6 || value == ETH_P_ARP ||
			     value == ETH_P_8021Q)) {
				priv->pkt_capture.ether_type = value;
				priv->pkt_capture.vlan = false;
				priv->pkt_capture.tci = 0;
			}
		}

		if (test_bit(24, &msglvl))
			priv->pkt_capture.icmp = true;
		else
			priv->pkt_capture.icmp = false;

		if (test_bit(23, &msglvl))
			priv->pkt_capture.all = true;
		else
			priv->pkt_capture.all = false;
	} else {
		memset(&priv->pkt_capture, 0, sizeof(struct bstgmac_pkt_capture));
	}

	priv->msg_enable = (u32)msglvl;
	return;
}

static int bstgmac_check_if_running(struct net_device *dev)
{
	if (!netif_running(dev))
		return -EBUSY;
	return 0;
}

static int bstgmac_ethtool_get_regs_len(struct net_device *dev)
{
	return REG_SPACE_SIZE;
}

static void bstgmac_ethtool_gregs(struct net_device *dev,
				  struct ethtool_regs *regs, void *space)
{
	u32 *reg_space = (u32 *)space;

	struct bstgmac_priv *priv = netdev_priv(dev);

	memset(reg_space, 0x0, REG_SPACE_SIZE);

	bstgmac_dump_mac_regs(priv, priv->hw, reg_space);
	bstgmac_dump_dma_regs(priv, priv->ioaddr, reg_space);
	/* Copy DMA registers to where ethtool expects them */
	memcpy(&reg_space[ETHTOOL_DMA_OFFSET], &reg_space[DMA_BUS_MODE / 4],
	       NUM_DWMAC1000_DMA_REGS * 4);
}

static void
bstgmac_get_pauseparam(struct net_device *netdev,
		       struct ethtool_pauseparam *pause)
{
	struct bstgmac_priv *priv = netdev_priv(netdev);

	pause->rx_pause = (priv->flow_ctrl & FLOW_RX);
	pause->tx_pause = (priv->flow_ctrl & FLOW_TX);
	pause->autoneg = 0;
	//phylink_ethtool_get_pauseparam(priv->phylink, pause);
}

static int
bstgmac_set_pauseparam(struct net_device *netdev,
		       struct ethtool_pauseparam *pause)
{
	struct bstgmac_priv *priv = netdev_priv(netdev);

	if (pause->tx_pause || pause->autoneg) {
		pr_err("ONLY SUPPORT RX PAUSE\n");
		return 0;
	}
	/* Flow Control operation */
	if (pause->rx_pause)
		priv->flow_ctrl = FLOW_RX;
	else
		priv->flow_ctrl &= (~FLOW_RX);

	//phylink_ethtool_set_pauseparam(priv->phylink, pause);

	return 0;
}

static void bstgmac_get_ethtool_stats(struct net_device *dev,
				      struct ethtool_stats *dummy, u64 *data)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	unsigned long count;
	int i, j = 0, ret;

	if (priv->dma_cap.asp) {
		for (i = 0; i < BSTGMAC_SAFETY_FEAT_SIZE; i++) {
			if (!bstgmac_safety_feat_dump(priv, &priv->sstats, i,
						      &count, NULL))
				data[j++] = count;
		}
	}

	/* Update the DMA HW counters for dwmac10/100 */
	ret =
	    bstgmac_dma_diagnostic_fr(priv, &dev->stats, (void *)&priv->xstats,
				      priv->ioaddr);

	if (ret) {
		/* If supported, for new GMAC chips expose the MMC counters */
		if (priv->dma_cap.rmon) {
			bstmac_mmc_read(priv, priv->mmcaddr, &priv->mmc);

			for (i = 0; i < BSTGMAC_MMC_STATS_LEN; i++) {
				char *p;

				p = (char *)priv + bstgmac_mmc[i].stat_offset;

				data[j++] = (bstgmac_mmc[i].sizeof_stat ==
					     sizeof(u64)) ? (*(u64 *)p) :
				    (*(u32 *)p);
			}
		}
		if (priv->eee_enabled) {
			int val = phy_get_eee_err(dev->phydev);

			if (val)
				priv->xstats.phy_eee_wakeup_error_n = val;
		}

		if (priv->synopsys_id >= DWMAC_CORE_3_50)
			bstgmac_mac_debug(priv, priv->ioaddr,
					  (void *)&priv->xstats,
					  rx_queues_count, tx_queues_count);
	}
	for (i = 0; i < BSTGMAC_STATS_LEN; i++) {
		char *p = (char *)priv + bstgmac_gstrings_stats[i].stat_offset;

		data[j++] = (bstgmac_gstrings_stats[i].sizeof_stat ==
			     sizeof(u64)) ? (*(u64 *)p) : (*(u32 *)p);
	}
}

static int bstgmac_get_sset_count(struct net_device *netdev, int sset)
{
	struct bstgmac_priv *priv = netdev_priv(netdev);
	int i, len, safety_len = 0;

	switch (sset) {
	case ETH_SS_STATS:
		len = BSTGMAC_STATS_LEN;

		if (priv->dma_cap.rmon)
			len += BSTGMAC_MMC_STATS_LEN;
		if (priv->dma_cap.asp) {
			for (i = 0; i < BSTGMAC_SAFETY_FEAT_SIZE; i++) {
				if (!bstgmac_safety_feat_dump(priv,
							      &priv->sstats, i,
							      NULL, NULL))
					safety_len++;
			}

			len += safety_len;
		}

		return len;
	case ETH_SS_TEST:
		return bstmac_selftest_get_count(priv);
	default:
		return -EOPNOTSUPP;
	}
}

static void bstgmac_get_strings(struct net_device *dev, u32 stringset,
				u8 *data)
{
	int i;
	u8 *p = data;
	struct bstgmac_priv *priv = netdev_priv(dev);

	switch (stringset) {
	case ETH_SS_STATS:
		if (priv->dma_cap.asp) {
			for (i = 0; i < BSTGMAC_SAFETY_FEAT_SIZE; i++) {
				const char *desc;

				if (!bstgmac_safety_feat_dump(priv,
							      &priv->sstats, i,
							      NULL, &desc)) {
					memcpy(p, desc, ETH_GSTRING_LEN);
					p += ETH_GSTRING_LEN;
				}
			}
		}
		if (priv->dma_cap.rmon)
			for (i = 0; i < BSTGMAC_MMC_STATS_LEN; i++) {
				memcpy(p, bstgmac_mmc[i].stat_string,
				       ETH_GSTRING_LEN);
				p += ETH_GSTRING_LEN;
			}
		for (i = 0; i < BSTGMAC_STATS_LEN; i++) {
			memcpy(p, bstgmac_gstrings_stats[i].stat_string,
			       ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	case ETH_SS_TEST:
		bstmac_selftest_get_strings(priv, p);
		break;
	default:
		WARN_ON(1);
		break;
	}
}

/* Currently only support WOL through Magic packet. */
static void bstgmac_get_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	mutex_lock(&priv->lock);
	if (device_can_wakeup(priv->device)) {
		wol->supported = WAKE_MAGIC | WAKE_UCAST;
		wol->wolopts = priv->wolopts;
	}
	mutex_unlock(&priv->lock);
}

static int bstgmac_set_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	u32 support = WAKE_MAGIC | WAKE_UCAST;

	/* By default almost all GMAC devices support the WoL via
	 * magic frame but we can disable it if the HW capability
	 * register shows no support for pmt_magic_frame.
	 */
	if (priv->hw_cap_support && !priv->dma_cap.pmt_magic_frame)
		wol->wolopts &= ~WAKE_MAGIC;

	if (!device_can_wakeup(priv->device))
		return -EINVAL;

	if (wol->wolopts & ~support)
		return -EINVAL;

	if (wol->wolopts) {
		pr_info("bstgmac: wakeup enable\n");
		device_set_wakeup_enable(priv->device, 1);
		enable_irq_wake(priv->wol_irq);
	} else {
		device_set_wakeup_enable(priv->device, 0);
		disable_irq_wake(priv->wol_irq);
	}

	mutex_lock(&priv->lock);
	priv->wolopts = wol->wolopts;
	mutex_unlock(&priv->lock);

	return 0;
}

static int bstgmac_ethtool_op_get_eee(struct net_device *dev,
				      struct ethtool_eee *edata)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	if (!priv->dma_cap.eee)
		return -EOPNOTSUPP;

	edata->eee_enabled = priv->eee_enabled;
	edata->eee_active = priv->eee_active;
	edata->tx_lpi_timer = priv->tx_lpi_timer;

	return phy_ethtool_get_eee(dev->phydev, edata);
}

static int bstgmac_ethtool_op_set_eee(struct net_device *dev,
				      struct ethtool_eee *edata)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	int ret;
	
	if (!priv->dma_cap.eee)
		return -EOPNOTSUPP;

	if (priv->tx_lpi_enabled != edata->tx_lpi_enabled)
		netdev_warn(priv->dev,
			    "Setting EEE tx-lpi is not supported\n");

	if (!edata->eee_enabled) {
		bstgmac_disable_eee_mode(priv);
	}

	ret = phy_ethtool_set_eee(dev->phydev, edata);
	if (ret)
		return ret;

	if (edata->eee_enabled &&
	    priv->tx_lpi_timer != edata->tx_lpi_timer) {
		priv->tx_lpi_timer = edata->tx_lpi_timer;
		bstgmac_eee_init(priv);
	}
	return 0;
}

static u32 bstgmac_usec2riwt(u32 usec, struct bstgmac_priv *priv)
{
	unsigned long clk = clk_get_rate(priv->plat->stmmac_clk);

	if (!clk) {
		clk = priv->plat->clk_ref_rate;
		if (!clk)
			return 0;
	}

	return (usec * (clk / 1000000)) / 256;
}

static u32 bstgmac_riwt2usec(u32 riwt, struct bstgmac_priv *priv)
{
	unsigned long clk = clk_get_rate(priv->plat->stmmac_clk);

	if (!clk) {
		clk = priv->plat->clk_ref_rate;
		if (!clk)
			return 0;
	}

	return (riwt * 256) / (clk / 1000000);
}

static int __bstgmac_get_coalesce(struct net_device *dev,
				 struct ethtool_coalesce *ec,
				 int queue)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	u32 max_cnt;
	u32 rx_cnt;
	u32 tx_cnt;

	rx_cnt = priv->plat->rx_queues_to_use;
	tx_cnt = priv->plat->tx_queues_to_use;
	max_cnt = max(rx_cnt, tx_cnt);

	if (queue < 0)
		queue = 0;
	else if (queue >= max_cnt)
		return -EINVAL;

	if (queue < tx_cnt) {
		ec->tx_coalesce_usecs = priv->tx_coal_timer[queue];
		ec->tx_max_coalesced_frames = priv->tx_coal_frames[queue];
	} else {
		ec->tx_coalesce_usecs = 0;
		ec->tx_max_coalesced_frames = 0;
	}

	if (priv->use_riwt && queue < rx_cnt) {
		ec->rx_max_coalesced_frames = priv->rx_coal_frames[queue];
		ec->rx_coalesce_usecs = bstgmac_riwt2usec(priv->rx_riwt[queue],
							 priv);
	} else {
		ec->rx_max_coalesced_frames = 0;
		ec->rx_coalesce_usecs = 0;
	}

	return 0;
}

static int bstgmac_get_coalesce(struct net_device *dev,
			       struct ethtool_coalesce *ec,
			       struct kernel_ethtool_coalesce *kernel_coal,
			       struct netlink_ext_ack *extack)
{
	return __bstgmac_get_coalesce(dev, ec, -1);
}

static int bstgmac_get_per_queue_coalesce(struct net_device *dev, u32 queue,
					 struct ethtool_coalesce *ec)
{
	return __bstgmac_get_coalesce(dev, ec, queue);
}

static int __bstgmac_set_coalesce(struct net_device *dev,
				 struct ethtool_coalesce *ec,
				 int queue)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	bool all_queues = false;
	unsigned int rx_riwt;
	u32 max_cnt;
	u32 rx_cnt;
	u32 tx_cnt;

	rx_cnt = priv->plat->rx_queues_to_use;
	tx_cnt = priv->plat->tx_queues_to_use;
	max_cnt = max(rx_cnt, tx_cnt);

	if (queue < 0)
		all_queues = true;
	else if (queue >= max_cnt)
		return -EINVAL;

	if (priv->use_riwt && (ec->rx_coalesce_usecs > 0)) {
		rx_riwt = bstgmac_usec2riwt(ec->rx_coalesce_usecs, priv);

		if ((rx_riwt > MAX_DMA_RIWT) || (rx_riwt < MIN_DMA_RIWT))
			return -EINVAL;

		if (all_queues) {
			int i;

			for (i = 0; i < rx_cnt; i++) {
				priv->rx_riwt[i] = rx_riwt;
				bstgmac_rx_watchdog(priv, priv->ioaddr,
						   rx_riwt, i);
				priv->rx_coal_frames[i] =
					ec->rx_max_coalesced_frames;
			}
		} else if (queue < rx_cnt) {
			priv->rx_riwt[queue] = rx_riwt;
			bstgmac_rx_watchdog(priv, priv->ioaddr,
					   rx_riwt, queue);
			priv->rx_coal_frames[queue] =
				ec->rx_max_coalesced_frames;
		}
	}

	if ((ec->tx_coalesce_usecs == 0) &&
	    (ec->tx_max_coalesced_frames == 0))
		return -EINVAL;

	if ((ec->tx_coalesce_usecs > BSTGMAC_MAX_COAL_TX_TICK) ||
	    (ec->tx_max_coalesced_frames > BSTGMAC_TX_MAX_FRAMES))
		return -EINVAL;

	if (all_queues) {
		int i;

		for (i = 0; i < tx_cnt; i++) {
			priv->tx_coal_frames[i] =
				ec->tx_max_coalesced_frames;
			priv->tx_coal_timer[i] =
				ec->tx_coalesce_usecs;
		}
	} else if (queue < tx_cnt) {
		priv->tx_coal_frames[queue] =
			ec->tx_max_coalesced_frames;
		priv->tx_coal_timer[queue] =
			ec->tx_coalesce_usecs;
	}

	return 0;
}

static int bstgmac_set_coalesce(struct net_device *dev,
			       struct ethtool_coalesce *ec,
			       struct kernel_ethtool_coalesce *kernel_coal,
			       struct netlink_ext_ack *extack)
{
	return __bstgmac_set_coalesce(dev, ec, -1);
}

static int bstgmac_set_per_queue_coalesce(struct net_device *dev, u32 queue,
					 struct ethtool_coalesce *ec)
{
	return __bstgmac_set_coalesce(dev, ec, queue);
}

static int bstgmac_get_rxnfc(struct net_device *dev,
			    struct ethtool_rxnfc *rxnfc, u32 *rule_locs)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	switch (rxnfc->cmd) {
	case ETHTOOL_GRXRINGS:
		rxnfc->data = priv->plat->rx_queues_to_use;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static u32 bstgmac_get_rxfh_key_size(struct net_device *dev)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	return sizeof(priv->rss.key);
}

static u32 bstgmac_get_rxfh_indir_size(struct net_device *dev)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	return ARRAY_SIZE(priv->rss.table);
}

static int bstgmac_get_rxfh(struct net_device *dev, u32 *indir, u8 *key,
			   u8 *hfunc)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	int i;

	if (indir) {
		for (i = 0; i < ARRAY_SIZE(priv->rss.table); i++)
			indir[i] = priv->rss.table[i];
	}

	if (key)
		memcpy(key, priv->rss.key, sizeof(priv->rss.key));
	if (hfunc)
		*hfunc = ETH_RSS_HASH_TOP;

	return 0;
}

static int bstgmac_set_rxfh(struct net_device *dev, const u32 *indir,
			   const u8 *key, const u8 hfunc)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	int i;

	if ((hfunc != ETH_RSS_HASH_NO_CHANGE) && (hfunc != ETH_RSS_HASH_TOP))
		return -EOPNOTSUPP;

	if (indir) {
		for (i = 0; i < ARRAY_SIZE(priv->rss.table); i++)
			priv->rss.table[i] = indir[i];
	}

	if (key)
		memcpy(priv->rss.key, key, sizeof(priv->rss.key));

	return bstgmac_rss_configure(priv, priv->hw, &priv->rss,
				    priv->plat->rx_queues_to_use);
}

static int bstgmac_get_ts_info(struct net_device *dev,
			       struct ethtool_ts_info *info)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	if (priv->dma_cap.time_stamp || priv->dma_cap.atime_stamp) {
		info->so_timestamping = SOF_TIMESTAMPING_TX_SOFTWARE |
		    SOF_TIMESTAMPING_TX_HARDWARE |
		    SOF_TIMESTAMPING_RX_SOFTWARE |
		    SOF_TIMESTAMPING_RX_HARDWARE |
		    SOF_TIMESTAMPING_SOFTWARE | SOF_TIMESTAMPING_RAW_HARDWARE;

		if (priv->ptp_clock)
			info->phc_index = ptp_clock_index(priv->ptp_clock);

		info->tx_types = (1 << HWTSTAMP_TX_OFF) | (1 << HWTSTAMP_TX_ON);

		info->rx_filters = ((1 << HWTSTAMP_FILTER_NONE) |
				    (1 << HWTSTAMP_FILTER_PTP_V1_L4_EVENT) |
				    (1 << HWTSTAMP_FILTER_PTP_V1_L4_SYNC) |
				    (1 << HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_L4_EVENT) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_L4_SYNC) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_EVENT) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_SYNC) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_DELAY_REQ) |
				    (1 << HWTSTAMP_FILTER_ALL));
		return 0;
	} else {
		return ethtool_op_get_ts_info(dev, info);
	}
}

static int bstgmac_get_tunable(struct net_device *dev,
			       const struct ethtool_tunable *tuna, void *data)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	int ret = 0;

	switch (tuna->id) {
	case ETHTOOL_RX_COPYBREAK:
		*(u32 *)data = priv->rx_copybreak;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int bstgmac_set_tunable(struct net_device *dev,
			       const struct ethtool_tunable *tuna,
			       const void *data)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	int ret = 0;

	switch (tuna->id) {
	case ETHTOOL_RX_COPYBREAK:
		priv->rx_copybreak = *(u32 *)data;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

int	bstgmac_get_fpe(struct net_device *dev,
				   struct ethtool_fpe *eth_fpe)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	
	bstgmac_fpe_get_config(priv, priv->ioaddr, eth_fpe);

	return 0;
}

int	bstgmac_set_fpe(struct net_device *dev,
				   const struct ethtool_fpe *eth_fpe)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	if (eth_fpe->fpe && !priv->dma_cap.fpesel)
		return -EOPNOTSUPP;

	if (eth_fpe->fpe) {
		if (eth_fpe->tx_queue_mask > ((1 << priv->plat->tx_queues_to_use) - 1))
			return -EINVAL;

		if (eth_fpe->min_frag_size > 0x3)
			return -EINVAL;

		priv->fpe_tx_queue_mask = eth_fpe->tx_queue_mask;
		priv->fpe_min_frag_size = eth_fpe->min_frag_size;

		priv->plat->fpe_cfg->enable = eth_fpe->fpe;

		if (priv->fpe_hs) {
			bstgmac_fpe_handshake(priv, true);
			netdev_info(dev, "start FPE handshake\n");
		} else {
			bstgmac_fpe_configure(priv, priv->ioaddr,
					     priv->fpe_tx_queue_mask,
					     priv->plat->rx_queues_to_use,
						 priv->fpe_min_frag_size,
					     true);
			netdev_info(priv->dev, "enable FPE\n");
		}
	} else {
		priv->plat->fpe_cfg->enable = false;
		bstgmac_fpe_configure(priv, priv->ioaddr,
			     priv->fpe_tx_queue_mask,
			     priv->plat->rx_queues_to_use,
				 priv->fpe_min_frag_size,
			     false);
		netdev_info(priv->dev, "disabled FPE\n");
		if (priv->fpe_hs)
			bstgmac_fpe_handshake(priv, false);
	}

	return 0;
}
static const struct ethtool_ops bstgmac_ethtool_ops = {
	.supported_coalesce_params = ETHTOOL_COALESCE_USECS |
	    ETHTOOL_COALESCE_MAX_FRAMES,
	.begin = bstgmac_check_if_running,
	.get_drvinfo = bstgmac_ethtool_getdrvinfo,
	.get_msglevel = bstgmac_ethtool_getmsglevel,
	.set_msglevel = bstgmac_ethtool_setmsglevel,
	.get_regs = bstgmac_ethtool_gregs,
	.get_regs_len = bstgmac_ethtool_get_regs_len,
	.get_link = ethtool_op_get_link,
	.nway_reset = phy_ethtool_nway_reset,
	.get_pauseparam = bstgmac_get_pauseparam,
	.set_pauseparam = bstgmac_set_pauseparam,
	.self_test = bstmac_selftest_run,
	.get_ethtool_stats = bstgmac_get_ethtool_stats,
	.get_strings = bstgmac_get_strings,
	.get_wol = bstgmac_get_wol,
	.set_wol = bstgmac_set_wol,
	.get_eee = bstgmac_ethtool_op_get_eee,
	.set_eee = bstgmac_ethtool_op_set_eee,
	.get_sset_count = bstgmac_get_sset_count,
	.get_rxnfc = bstgmac_get_rxnfc,
	.get_rxfh_key_size = bstgmac_get_rxfh_key_size,
	.get_rxfh_indir_size = bstgmac_get_rxfh_indir_size,
	.get_rxfh = bstgmac_get_rxfh,
	.set_rxfh = bstgmac_set_rxfh,
	.get_ts_info = bstgmac_get_ts_info,
	.get_coalesce = bstgmac_get_coalesce,
	.set_coalesce = bstgmac_set_coalesce,
	.get_per_queue_coalesce = bstgmac_get_per_queue_coalesce,
	.set_per_queue_coalesce = bstgmac_set_per_queue_coalesce,
	.get_tunable = bstgmac_get_tunable,
	.set_tunable = bstgmac_set_tunable,
	.get_link_ksettings = bstgmac_ethtool_get_link_ksettings,
	.set_link_ksettings = bstgmac_ethtool_set_link_ksettings,
	.get_fpe = bstgmac_get_fpe,
	.set_fpe = bstgmac_set_fpe,
};

void bstgmac_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &bstgmac_ethtool_ops;
};
