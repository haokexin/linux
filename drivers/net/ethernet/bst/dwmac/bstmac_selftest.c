// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2019 Synopsys, Inc. and/or its affiliates.
 */

#include <linux/bitrev.h>
#include <linux/completion.h>
#include <linux/crc32.h>
#include <linux/ethtool.h>
#include <linux/ip.h>
#include <linux/phy.h>
#include <linux/udp.h>
#include <net/pkt_cls.h>
#include <net/pkt_sched.h>
#include <net/tcp.h>
#include <net/udp.h>
#include <net/tc_act/tc_gact.h>
#include <linux/ipv6.h>
#include <linux/in6.h>
#include <net/ip6_checksum.h>
#include <net/addrconf.h>
#include "bstgmac.h"
#include "dwxgmac2.h"

struct bstmachdr {
	__be32 version;
	__be64 magic;
	u8 id;
} __packed;

#define bstmac_TEST_PKT_SIZE (sizeof(struct ethhdr) + sizeof(struct iphdr) + \
			      sizeof(struct bstmachdr))
#define bstmac_TEST_IPV6PKT_SIZE (sizeof(struct ethhdr) + sizeof(struct ipv6hdr) + \
			      sizeof(struct bstmachdr))
#define bstmac_TEST_PKT_MAGIC	0xdeadcafecafedeadULL
#define bstmac_LB_TIMEOUT	msecs_to_jiffies(200)

struct bstmac_packet_attrs {
	int vlan;
	int vlan_id_in;
	int vlan_id_out;
	unsigned char *src;
	const unsigned char *dst;
	u32 ip_src;
	u32 ip_dst;
	int tcp;
	int sport;
	int dport;
	u32 exp_hash;
	int dont_wait;
	int timeout;
	int size;
	int max_size;
	int remove_sa;
	u8 id;
	int sarc;
	u16 queue_mapping;
	u64 timestamp;
	u8 non_csum;
	bool ieee_1588;
};

static u8 bstmac_test_next_id;

static struct sk_buff *bstmac_test_get_ptp_skb(struct bstgmac_priv *priv,
					       struct bstmac_packet_attrs *attr)
{
	struct sk_buff *skb = NULL;
	struct net_device *dev;
	struct ethhdr *ethdr;
	int length;

	char dmac[6] = {0x01, 0x1B, 0x19, 0x00, 0x00, 0x00};

	//ptp sync
	char data[] = {
		0x00, 0x02, 0x00, 0x2c, 0x03, 0x00, 0x02, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0xd4, 0x9e, 0x6e, 0xff,
		0xfe, 0x00, 0x01, 0xe7, 0x00, 0x01, 0x4f, 0x14,
		0x00, 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	dev = priv->dev;
	length = sizeof(struct ethhdr) + sizeof(data) + 10;

	if (!(skb = dev_alloc_skb(length))) {
		pr_err("dev_alloc_skb malloc skb error\n");
		return NULL;
	}

	skb_reserve(skb,length);
	skb->len =  0;

	//fill payload
	skb_push(skb, sizeof(data));
	memcpy(skb->data, data, sizeof(data));
	skb->len = sizeof(data);

	//fille eth header
	skb_push(skb, sizeof(struct ethhdr));
	ethdr = (struct ethhdr *)skb->data;
	skb->len += sizeof(struct ethhdr);

	memcpy(ethdr->h_source, priv->dev->dev_addr, ETH_ALEN);
	memcpy(ethdr->h_dest, dmac, ETH_ALEN);

	ethdr->h_proto = htons(ETH_P_1588);
	skb->protocol = htons(ETH_P_1588);

	//fill skb
	skb->pkt_type =  PACKET_OTHERHOST;
	skb->dev = dev;

	if (attr->ieee_1588)
		skb_shinfo(skb)->tx_flags |= SKBTX_HW_TSTAMP;

	return skb;
}

static struct sk_buff *bstmac_test_get_udp_skb(struct bstgmac_priv *priv,
					       struct bstmac_packet_attrs *attr)
{
	struct sk_buff *skb = NULL;
	struct udphdr *uhdr = NULL;
	struct tcphdr *thdr = NULL;
	struct bstmachdr *shdr;
	struct ethhdr *ehdr;
	struct iphdr *ihdr;
	int iplen, size;

	size = attr->size + bstmac_TEST_PKT_SIZE;
	if (attr->vlan) {
		size += 4;
		if (attr->vlan > 1)
			size += 4;
	}

	if (attr->tcp)
		size += sizeof(struct tcphdr);
	else
		size += sizeof(struct udphdr);

	if (attr->max_size && (attr->max_size > size))
		size = attr->max_size;

	skb = netdev_alloc_skb(priv->dev, size);
	if (!skb)
		return NULL;

	prefetchw(skb->data);

	if (attr->vlan > 1)
		ehdr = skb_push(skb, ETH_HLEN + 8);
	else if (attr->vlan)
		ehdr = skb_push(skb, ETH_HLEN + 4);
	else if (attr->remove_sa)
		ehdr = skb_push(skb, ETH_HLEN - 6);
	else
		ehdr = skb_push(skb, ETH_HLEN);
	skb_reset_mac_header(skb);

	skb_set_network_header(skb, skb->len);
	ihdr = skb_put(skb, sizeof(*ihdr));

	skb_set_transport_header(skb, skb->len);
	if (attr->tcp)
		thdr = skb_put(skb, sizeof(*thdr));
	else
		uhdr = skb_put(skb, sizeof(*uhdr));

	if (!attr->remove_sa)
		eth_zero_addr(ehdr->h_source);
	eth_zero_addr(ehdr->h_dest);
	if (attr->src && !attr->remove_sa)
		ether_addr_copy(ehdr->h_source, attr->src);
	if (attr->dst)
		ether_addr_copy(ehdr->h_dest, attr->dst);

	if (!attr->remove_sa) {
		ehdr->h_proto = htons(ETH_P_IP);
	} else {
		__be16 *ptr = (__be16 *)ehdr;

		/* HACK */
		ptr[3] = htons(ETH_P_IP);
	}

	if (attr->vlan) {
		__be16 *tag, *proto;

		if (!attr->remove_sa) {
			tag = (void *)ehdr + ETH_HLEN;
			proto = (void *)ehdr + (2 * ETH_ALEN);
		} else {
			tag = (void *)ehdr + ETH_HLEN - 6;
			proto = (void *)ehdr + ETH_ALEN;
		}

		proto[0] = htons(ETH_P_8021Q);
		tag[0] = htons(attr->vlan_id_out);
		tag[1] = htons(ETH_P_IP);
		if (attr->vlan > 1) {
			proto[0] = htons(ETH_P_8021AD);
			tag[1] = htons(ETH_P_8021Q);
			tag[2] = htons(attr->vlan_id_in);
			tag[3] = htons(ETH_P_IP);
		}
	}

	if (attr->tcp) {
		thdr->source = htons(attr->sport);
		thdr->dest = htons(attr->dport);
		thdr->doff = sizeof(struct tcphdr) / 4;
		thdr->check = 0;
	} else {
		uhdr->source = htons(attr->sport);
		uhdr->dest = htons(attr->dport);
		uhdr->len = htons(sizeof(*shdr) + sizeof(*uhdr) + attr->size);
		if (attr->max_size)
			uhdr->len = htons(attr->max_size -
					  (sizeof(*ihdr) + sizeof(*ehdr)));
		uhdr->check = 0;
	}

	ihdr->ihl = 5;
	ihdr->ttl = 32;
	ihdr->version = 4;
	if (attr->tcp)
		ihdr->protocol = IPPROTO_TCP;
	else
		ihdr->protocol = IPPROTO_UDP;
	iplen = sizeof(*ihdr) + sizeof(*shdr) + attr->size;
	if (attr->tcp)
		iplen += sizeof(*thdr);
	else
		iplen += sizeof(*uhdr);

	if (attr->max_size)
		iplen = attr->max_size - sizeof(*ehdr);

	ihdr->tot_len = htons(iplen);
	ihdr->frag_off = 0;
	ihdr->saddr = htonl(attr->ip_src);
	ihdr->daddr = htonl(attr->ip_dst);
	ihdr->tos = 0;
	ihdr->id = 0;
	ip_send_check(ihdr);

	shdr = skb_put(skb, sizeof(*shdr));
	shdr->version = 0;
	shdr->magic = cpu_to_be64(bstmac_TEST_PKT_MAGIC);
	attr->id = bstmac_test_next_id;
	shdr->id = bstmac_test_next_id++;

	if (attr->size)
		skb_put(skb, attr->size);
	if (attr->max_size && (attr->max_size > skb->len))
		skb_put(skb, attr->max_size - skb->len);

	skb->csum = 0;
	if (attr->non_csum) {
		skb->ip_summed = CHECKSUM_NONE;
	} else {
		skb->ip_summed = CHECKSUM_PARTIAL;
	}
	if (attr->tcp) {
		thdr->check = ~tcp_v4_check(skb->len, ihdr->saddr, ihdr->daddr, 0);
		skb->csum_start = skb_transport_header(skb) - skb->head;
		skb->csum_offset = offsetof(struct tcphdr, check);
	} else {
		udp4_hwcsum(skb, ihdr->saddr, ihdr->daddr);
	}

	skb->protocol = htons(ETH_P_IP);
	skb->pkt_type = PACKET_HOST;
	skb->dev = priv->dev;

	if (attr->timestamp)
		skb->tstamp = ns_to_ktime(attr->timestamp);

	return skb;
}

struct bstmac_test_priv {
	struct bstmac_packet_attrs *packet;
	struct packet_type pt;
	struct completion comp;
	int double_vlan;
	int vlan_id;
	int ok;
};

static int bstmac_test_loopback_validate(struct sk_buff *skb,
					 struct net_device *ndev,
					 struct packet_type *pt,
					 struct net_device *orig_ndev)
{
	struct bstmac_test_priv *tpriv = pt->af_packet_priv;
	unsigned char *src = tpriv->packet->src;
	const unsigned char *dst = tpriv->packet->dst;
	struct bstmachdr *shdr;
	struct ethhdr *ehdr;
	struct udphdr *uhdr;
	struct tcphdr *thdr;
	struct iphdr *ihdr;

	skb = skb_unshare(skb, GFP_ATOMIC);
	if (!skb)
		goto out;

	if (skb_linearize(skb))
		goto out;
	if (skb_headlen(skb) < (bstmac_TEST_PKT_SIZE - ETH_HLEN))
		goto out;

	ehdr = (struct ethhdr *)skb_mac_header(skb);
	if (dst) {
		if (!ether_addr_equal_unaligned(ehdr->h_dest, dst))
			goto out;
	}
	if (tpriv->packet->sarc) {
		if (!ether_addr_equal_unaligned(ehdr->h_source, ehdr->h_dest))
			goto out;
	} else if (src) {
		if (!ether_addr_equal_unaligned(ehdr->h_source, src))
			goto out;
	}

	ihdr = ip_hdr(skb);
	if (tpriv->double_vlan)
		ihdr = (struct iphdr *)(skb_network_header(skb) + 4);

	if (tpriv->packet->tcp) {
		if (ihdr->protocol != IPPROTO_TCP)
			goto out;

		thdr = (struct tcphdr *)((u8 *)ihdr + 4 * ihdr->ihl);
		if (thdr->dest != htons(tpriv->packet->dport))
			goto out;

		shdr = (struct bstmachdr *)((u8 *)thdr + sizeof(*thdr));
	} else {
		if (ihdr->protocol != IPPROTO_UDP)
			goto out;

		uhdr = (struct udphdr *)((u8 *)ihdr + 4 * ihdr->ihl);
		if (uhdr->dest != htons(tpriv->packet->dport))
			goto out;

		shdr = (struct bstmachdr *)((u8 *)uhdr + sizeof(*uhdr));
	}

	if (shdr->magic != cpu_to_be64(bstmac_TEST_PKT_MAGIC))
		goto out;
	if (tpriv->packet->exp_hash && !skb->hash)
		goto out;
	if (tpriv->packet->id != shdr->id)
		goto out;

	tpriv->ok = true;
	complete(&tpriv->comp);
out:
	kfree_skb(skb);
	return 0;
}

static int __bstmac_test_loopback(struct bstgmac_priv *priv,
				  struct bstmac_packet_attrs *attr)
{
	struct bstmac_test_priv *tpriv;
	struct sk_buff *skb = NULL;
	int ret = 0;

	tpriv = kzalloc(sizeof(*tpriv), GFP_KERNEL);
	if (!tpriv)
		return -ENOMEM;

	tpriv->ok = false;
	init_completion(&tpriv->comp);

	tpriv->pt.type = htons(ETH_P_IP);
	tpriv->pt.func = bstmac_test_loopback_validate;
	tpriv->pt.dev = priv->dev;
	tpriv->pt.af_packet_priv = tpriv;
	tpriv->packet = attr;

	if (!attr->dont_wait)
		dev_add_pack(&tpriv->pt);

	if (!attr->ieee_1588)
		skb = bstmac_test_get_udp_skb(priv, attr);
	else
		skb = bstmac_test_get_ptp_skb(priv, attr);
	if (!skb) {
		ret = -ENOMEM;
		goto cleanup;
	}

	ret = dev_direct_xmit(skb, attr->queue_mapping);
	if (ret)
		goto cleanup;

	if (attr->dont_wait)
		goto cleanup;

	if (!attr->timeout)
		attr->timeout = bstmac_LB_TIMEOUT;

	wait_for_completion_timeout(&tpriv->comp, attr->timeout);
	ret = tpriv->ok ? 0 : -ETIMEDOUT;

	if (attr->ieee_1588) {
		ret = -ETIMEDOUT;
		if (!bstmac_get_ts_flag_slt()) {//tx
			ret = 0;
			bstmac_clr_ts_flag_slt();
		}
	}

cleanup:
	if (!attr->dont_wait)
		dev_remove_pack(&tpriv->pt);
	kfree(tpriv);
	return ret;
}
#if 0
static struct sk_buff *bstmac_test_get_arp_skb(struct bstgmac_priv *priv,
					       struct bstmac_packet_attrs *attr)
{
	__be32 ip_src = htonl(attr->ip_src);
	__be32 ip_dst = htonl(attr->ip_dst);
	struct sk_buff *skb = NULL;

	skb = arp_create(ARPOP_REQUEST, ETH_P_ARP, ip_dst, priv->dev, ip_src,
			 NULL, attr->src, attr->dst);
	if (!skb)
		return NULL;

	skb->pkt_type = PACKET_HOST;
	skb->dev = priv->dev;

	return skb;
}
#endif
static int bstmac_test_mac_loopback(struct bstgmac_priv *priv)
{
	struct bstmac_packet_attrs attr = { };
	int ret;

	attr.dst = priv->dev->dev_addr;
	
	attr.size = 9;
	ret = __bstmac_test_loopback(priv, &attr);

	attr.size = 73;
	ret |= __bstmac_test_loopback(priv, &attr);

	attr.size = 500;
	ret |= __bstmac_test_loopback(priv, &attr);

	attr.size = 1450;
	ret |= __bstmac_test_loopback(priv, &attr);

	return ret;
}

#if 0
static int bstmac_test_phy_loopback(struct bstgmac_priv *priv)
{
	struct bstmac_packet_attrs attr = { };
	int ret;

	if (!priv->dev->phydev)
		return -EOPNOTSUPP;

	ret = phy_loopback(priv->dev->phydev, true);
	if (ret)
		return ret;

	attr.dst = priv->dev->dev_addr;
	ret = __bstmac_test_loopback(priv, &attr);

	phy_loopback(priv->dev->phydev, false);
	return ret;
}

static int bstmac_test_mmc(struct bstgmac_priv *priv)
{
	struct bstmac_counters initial, final;
	int ret;

	memset(&initial, 0, sizeof(initial));
	memset(&final, 0, sizeof(final));

	if (!priv->dma_cap.rmon)
		return -EOPNOTSUPP;

	/* Save previous results into internal struct */
	bstmac_mmc_read(priv, priv->mmcaddr, &priv->mmc);

	ret = bstmac_test_mac_loopback(priv);
	if (ret)
		return ret;

	/* These will be loopback results so no need to save them */
	bstmac_mmc_read(priv, priv->mmcaddr, &final);

	/*
	 * The number of MMC counters available depends on HW configuration
	 * so we just use this one to validate the feature. I hope there is
	 * not a version without this counter.
	 */
	if (final.mmc_tx_framecount_g <= initial.mmc_tx_framecount_g)
		return -EINVAL;

	return 0;
}

static int bstmac_test_eee(struct bstgmac_priv *priv)
{
	struct bstmac_extra_stats *initial, *final;
	int retries = 10;
	int ret;

	if (!priv->dma_cap.eee || !priv->eee_active)
		return -EOPNOTSUPP;

	initial = kzalloc(sizeof(*initial), GFP_KERNEL);
	if (!initial)
		return -ENOMEM;

	final = kzalloc(sizeof(*final), GFP_KERNEL);
	if (!final) {
		ret = -ENOMEM;
		goto out_free_initial;
	}

	memcpy(initial, &priv->xstats, sizeof(*initial));

	ret = bstmac_test_mac_loopback(priv);
	if (ret)
		goto out_free_final;

	/* We have no traffic in the line so, sooner or later it will go LPI */
	while (--retries) {
		memcpy(final, &priv->xstats, sizeof(*final));

		if (final->irq_tx_path_in_lpi_mode_n >
		    initial->irq_tx_path_in_lpi_mode_n)
			break;
		msleep(100);
	}

	if (!retries) {
		ret = -ETIMEDOUT;
		goto out_free_final;
	}

	if (final->irq_tx_path_in_lpi_mode_n <=
	    initial->irq_tx_path_in_lpi_mode_n) {
		ret = -EINVAL;
		goto out_free_final;
	}

	if (final->irq_tx_path_exit_lpi_mode_n <=
	    initial->irq_tx_path_exit_lpi_mode_n) {
		ret = -EINVAL;
		goto out_free_final;
	}

out_free_final:
	kfree(final);
out_free_initial:
	kfree(initial);
	return ret;
}
#endif

static int bstmac_filter_check(struct bstgmac_priv *priv)
{
	if (!(priv->dev->flags & IFF_PROMISC))
		return 0;

	netdev_warn(priv->dev, "Test can't be run in promiscuous mode!\n");
	return -EOPNOTSUPP;
}

static bool bstmac_hash_check(struct bstgmac_priv *priv, unsigned char *addr)
{
	int mc_offset = 32 - priv->hw->mcast_bits_log2;
	struct netdev_hw_addr *ha;
	u32 hash, hash_nr;

	/* First compute the hash for desired addr */
	hash = bitrev32(~crc32_le(~0, addr, 6)) >> mc_offset;
	hash_nr = hash >> 5;
	hash = 1 << (hash & 0x1f);

	/* Now, check if it collides with any existing one */
	netdev_for_each_mc_addr(ha, priv->dev) {
		u32 nr = bitrev32(~crc32_le(~0, ha->addr, ETH_ALEN)) >> mc_offset;
		if (((nr >> 5) == hash_nr) && ((1 << (nr & 0x1f)) == hash))
			return false;
	}

	/* No collisions, address is good to go */
	return true;
}

static bool bstmac_perfect_check(struct bstgmac_priv *priv, unsigned char *addr)
{
	struct netdev_hw_addr *ha;

	/* Check if it collides with any existing one */
	netdev_for_each_uc_addr(ha, priv->dev) {
		if (!memcmp(ha->addr, addr, ETH_ALEN))
			return false;
	}

	/* No collisions, address is good to go */
	return true;
}

int bstmac_test_hfilt(struct bstgmac_priv *priv)
{
	unsigned char gd_addr[ETH_ALEN] = {0xf1, 0xee, 0xdd, 0xcc, 0xbb, 0xaa};
	unsigned char bd_addr[ETH_ALEN] = {0xf1, 0xff, 0xff, 0xff, 0xff, 0xff};
	struct bstmac_packet_attrs attr = { };
	int ret, tries = 256;

	ret = bstmac_filter_check(priv);
	if (ret)
		return ret;

	if (netdev_mc_count(priv->dev) >= priv->hw->multicast_filter_bins)
		return -EOPNOTSUPP;

	while (--tries) {
		/* We only need to check the bd_addr for collisions */
		bd_addr[ETH_ALEN - 1] = tries;
		if (bstmac_hash_check(priv, bd_addr))
			break;
	}

	if (!tries)
		return -EOPNOTSUPP;

	ret = dev_mc_add(priv->dev, gd_addr);
	if (ret)
		return ret;

	attr.dst = gd_addr;

	/* Shall receive packet */
	ret = __bstmac_test_loopback(priv, &attr);
	if (ret)
		goto cleanup;

	attr.dst = bd_addr;

	/* Shall NOT receive packet */
	ret = __bstmac_test_loopback(priv, &attr);
	ret = ret ? 0 : -EINVAL;

cleanup:
	dev_mc_del(priv->dev, gd_addr);
	return ret;
}

int bstmac_test_pfilt(struct bstgmac_priv *priv)
{
	unsigned char gd_addr[ETH_ALEN] = {0xf0, 0x01, 0x44, 0x55, 0x66, 0x77};
	unsigned char bd_addr[ETH_ALEN] = {0xf0, 0xff, 0xff, 0xff, 0xff, 0xff};
	struct bstmac_packet_attrs attr = { };
	int ret, tries = 256;

	if (bstmac_filter_check(priv))
		return -EOPNOTSUPP;
	if (netdev_uc_count(priv->dev) >= priv->hw->unicast_filter_entries)
		return -EOPNOTSUPP;

	while (--tries) {
		/* We only need to check the bd_addr for collisions */
		bd_addr[ETH_ALEN - 1] = tries;
		if (bstmac_perfect_check(priv, bd_addr))
			break;
	}

	if (!tries)
		return -EOPNOTSUPP;

	ret = dev_uc_add(priv->dev, gd_addr);
	if (ret)
		return ret;

	attr.dst = gd_addr;

	/* Shall receive packet */
	ret = __bstmac_test_loopback(priv, &attr);
	if (ret)
		goto cleanup;

	attr.dst = bd_addr;

	/* Shall NOT receive packet */
	ret = __bstmac_test_loopback(priv, &attr);
	ret = ret ? 0 : -EINVAL;

cleanup:
	dev_uc_del(priv->dev, gd_addr);
	return ret;
}

int bstmac_test_mcfilt(struct bstgmac_priv *priv)
{
	unsigned char uc_addr[ETH_ALEN] = {0xf0, 0xff, 0xff, 0xff, 0xff, 0xff};
	unsigned char mc_addr[ETH_ALEN] = {0xf1, 0xff, 0xff, 0xff, 0xff, 0xff};
	struct bstmac_packet_attrs attr = { };
	int ret, tries = 256;

	if (bstmac_filter_check(priv))
		return -EOPNOTSUPP;
	if (netdev_uc_count(priv->dev) >= priv->hw->unicast_filter_entries)
		return -EOPNOTSUPP;
	if (netdev_mc_count(priv->dev) >= priv->hw->multicast_filter_bins)
		return -EOPNOTSUPP;

	while (--tries) {
		/* We only need to check the mc_addr for collisions */
		mc_addr[ETH_ALEN - 1] = tries;
		if (bstmac_hash_check(priv, mc_addr))
			break;
	}

	if (!tries)
		return -EOPNOTSUPP;

	ret = dev_uc_add(priv->dev, uc_addr);
	if (ret)
		return ret;

	attr.dst = uc_addr;

	/* Shall receive packet */
	ret = __bstmac_test_loopback(priv, &attr);
	if (ret)
		goto cleanup;

	attr.dst = mc_addr;

	/* Shall NOT receive packet */
	ret = __bstmac_test_loopback(priv, &attr);
	ret = ret ? 0 : -EINVAL;

cleanup:
	dev_uc_del(priv->dev, uc_addr);
	return ret;
}

int bstmac_test_ucfilt(struct bstgmac_priv *priv)
{
	unsigned char uc_addr[ETH_ALEN] = {0xf0, 0xff, 0xff, 0xff, 0xff, 0xff};
	unsigned char mc_addr[ETH_ALEN] = {0xf1, 0xff, 0xff, 0xff, 0xff, 0xff};
	struct bstmac_packet_attrs attr = { };
	int ret, tries = 256;

	if (bstmac_filter_check(priv))
		return -EOPNOTSUPP;
	if (netdev_uc_count(priv->dev) >= priv->hw->unicast_filter_entries)
		return -EOPNOTSUPP;
	if (netdev_mc_count(priv->dev) >= priv->hw->multicast_filter_bins)
		return -EOPNOTSUPP;

	while (--tries) {
		/* We only need to check the uc_addr for collisions */
		uc_addr[ETH_ALEN - 1] = tries;
		if (bstmac_perfect_check(priv, uc_addr))
			break;
	}

	if (!tries)
		return -EOPNOTSUPP;

	ret = dev_mc_add(priv->dev, mc_addr);
	if (ret)
		return ret;

	attr.dst = mc_addr;

	/* Shall receive packet */
	ret = __bstmac_test_loopback(priv, &attr);
	if (ret)
		goto cleanup;

	attr.dst = uc_addr;

	/* Shall NOT receive packet */
	ret = __bstmac_test_loopback(priv, &attr);
	ret = ret ? 0 : -EINVAL;

cleanup:
	dev_mc_del(priv->dev, mc_addr);
	return ret;
}

static int bstmac_test_flowctrl_validate(struct sk_buff *skb,
					 struct net_device *ndev,
					 struct packet_type *pt,
					 struct net_device *orig_ndev)
{
	struct bstmac_test_priv *tpriv = pt->af_packet_priv;
	struct ethhdr *ehdr;

	ehdr = (struct ethhdr *)skb_mac_header(skb);
	if (!ether_addr_equal_unaligned(ehdr->h_source, orig_ndev->dev_addr))
		goto out;
	if (ehdr->h_proto != htons(ETH_P_PAUSE))
		goto out;

	tpriv->ok = true;
	complete(&tpriv->comp);
out:
	kfree_skb(skb);
	return 0;
}

int bstmac_test_flowctrl(struct bstgmac_priv *priv)
{
	unsigned char paddr[ETH_ALEN] = {0x01, 0x80, 0xC2, 0x00, 0x00, 0x01};
	// struct phy_device *phydev = priv->dev->phydev;
	u32 rx_cnt = priv->plat->rx_queues_to_use;
	struct bstmac_test_priv *tpriv;
	unsigned int pkt_count;
	int i, ret = 0;


	tpriv = kzalloc(sizeof(*tpriv), GFP_KERNEL);
	if (!tpriv)
		return -ENOMEM;

	tpriv->ok = false;
	init_completion(&tpriv->comp);
	tpriv->pt.type = htons(ETH_P_PAUSE);
	tpriv->pt.func = bstmac_test_flowctrl_validate;
	tpriv->pt.dev = priv->dev;
	tpriv->pt.af_packet_priv = tpriv;
	dev_add_pack(&tpriv->pt);

	/* Compute minimum number of packets to make FIFO full */
	pkt_count = priv->plat->rx_fifo_size;
	if (!pkt_count)
		pkt_count = priv->dma_cap.rx_fifo_size;
	pkt_count /= 1400;
	pkt_count *= 2;

	for (i = 0; i < rx_cnt; i++) {
		bstgmac_stop_rx(priv, priv->ioaddr, i);
		writel(XGMAC_RSE, priv->ioaddr + XGMAC_DMA_CH_STATUS(i));   //clear interupt
		bstgmac_disable_dma_irq_bits(priv, priv->ioaddr, i, XGMAC_RSE);
	}

	ret = dev_set_promiscuity(priv->dev, 1);
	if (ret)
		goto cleanup;

	ret = dev_mc_add(priv->dev, paddr);
	if (ret)
		goto cleanup;

	for (i = 0; i < pkt_count; i++) {
		struct bstmac_packet_attrs attr = { };

		attr.dst = priv->dev->dev_addr;
		attr.dont_wait = true;
		attr.size = 1400;

		ret = __bstmac_test_loopback(priv, &attr);
		if (ret)
			goto cleanup;
		if (tpriv->ok)
			break;
	}

	/* Wait for some time in case RX Watchdog is enabled */
	msleep(20);

	for (i = 0; i < rx_cnt; i++) {
		struct bstgmac_channel *ch = &priv->rx_channel[i];
		u32 tail;

		tail = priv->rx_queue[i].dma_rx_phy +
			(priv->dma_rx_size * sizeof(struct dma_desc));

		bstgmac_set_rx_tail_ptr(priv, priv->ioaddr, tail, i);
		bstgmac_start_rx(priv, priv->ioaddr, i);

		local_bh_disable();
		if (priv->plat->dma_cfg->dma_int_mode != DMA_INT_M_0) {
			napi_reschedule(&ch->rnapi);
		} else {
			napi_reschedule(&ch->napi);
		}
		local_bh_enable();

		bstgmac_enable_dma_irq_bits(priv, priv->ioaddr, i, XGMAC_RSE);
	}

	wait_for_completion_timeout(&tpriv->comp, bstmac_LB_TIMEOUT);
	ret = tpriv->ok ? 0 : -ETIMEDOUT;

cleanup:
	dev_mc_del(priv->dev, paddr);
	dev_set_promiscuity(priv->dev, -1);
	dev_remove_pack(&tpriv->pt);
	kfree(tpriv);
	return ret;
}

#if 0
static int bstmac_test_rss(struct bstgmac_priv *priv)
{
	struct bstmac_packet_attrs attr = { };

	if (!priv->dma_cap.rssen || !priv->rss.enable)
		return -EOPNOTSUPP;

	attr.dst = priv->dev->dev_addr;
	attr.exp_hash = true;
	attr.sport = 0x321;
	attr.dport = 0x123;

	return __bstmac_test_loopback(priv, &attr);
}
#endif

static int bstmac_test_vlan_validate(struct sk_buff *skb,
				     struct net_device *ndev,
				     struct packet_type *pt,
				     struct net_device *orig_ndev)
{
	struct bstmac_test_priv *tpriv = pt->af_packet_priv;
	struct bstmachdr *shdr;
	struct ethhdr *ehdr;
	struct udphdr *uhdr;
	struct iphdr *ihdr;
	u16 proto;

	proto = tpriv->double_vlan ? ETH_P_8021AD : ETH_P_8021Q;

	skb = skb_unshare(skb, GFP_ATOMIC);
	if (!skb)
		goto out;

	if (skb_linearize(skb))
		goto out;
	if (skb_headlen(skb) < (bstmac_TEST_PKT_SIZE - ETH_HLEN))
		goto out;
	if (tpriv->vlan_id) {
		if (skb->vlan_proto != htons(proto))
			goto out;
		if (skb->vlan_tci != tpriv->vlan_id) {
			/* Means filter did not work. */
			tpriv->ok = false;
			complete(&tpriv->comp);
			goto out;
		}
	}

	ehdr = (struct ethhdr *)skb_mac_header(skb);
	if (!ether_addr_equal_unaligned(ehdr->h_dest, tpriv->packet->dst))
		goto out;

	ihdr = ip_hdr(skb);
	if (tpriv->double_vlan)
		ihdr = (struct iphdr *)(skb_network_header(skb) + 4);
	if (ihdr->protocol != IPPROTO_UDP)
		goto out;

	uhdr = (struct udphdr *)((u8 *)ihdr + 4 * ihdr->ihl);
	if (uhdr->dest != htons(tpriv->packet->dport))
		goto out;

	shdr = (struct bstmachdr *)((u8 *)uhdr + sizeof(*uhdr));
	if (shdr->magic != cpu_to_be64(bstmac_TEST_PKT_MAGIC))
		goto out;

	tpriv->ok = true;
	complete(&tpriv->comp);

out:
	kfree_skb(skb);
	return 0;
}

static int __bstmac_test_vlanfilt(struct bstgmac_priv *priv)
{
	struct bstmac_packet_attrs attr = { };
	struct bstmac_test_priv *tpriv;
	struct sk_buff *skb = NULL;
	int ret = 0, i;

	tpriv = kzalloc(sizeof(*tpriv), GFP_KERNEL);
	if (!tpriv)
		return -ENOMEM;

	tpriv->ok = false;
	init_completion(&tpriv->comp);

	tpriv->pt.type = htons(ETH_P_IP);
	tpriv->pt.func = bstmac_test_vlan_validate;
	tpriv->pt.dev = priv->dev;
	tpriv->pt.af_packet_priv = tpriv;
	tpriv->packet = &attr;

	/*
	 * As we use HASH filtering, false positives may appear. This is a
	 * specially chosen ID so that adjacent IDs (+4) have different
	 * HASH values.
	 */
	tpriv->vlan_id = 0x123;
	dev_add_pack(&tpriv->pt);

	ret = vlan_vid_add(priv->dev, htons(ETH_P_8021Q), tpriv->vlan_id);
	if (ret)
		goto cleanup;

	for (i = 0; i < 4; i++) {
		attr.vlan = 1;
		attr.vlan_id_out = tpriv->vlan_id + i;
		attr.dst = priv->dev->dev_addr;
		attr.sport = 9;
		attr.dport = 9;

		skb = bstmac_test_get_udp_skb(priv, &attr);
		if (!skb) {
			ret = -ENOMEM;
			goto vlan_del;
		}

		ret = dev_direct_xmit(skb, 0);
		if (ret)
			goto vlan_del;

		wait_for_completion_timeout(&tpriv->comp, bstmac_LB_TIMEOUT);
		ret = tpriv->ok ? 0 : -ETIMEDOUT;
		if (ret && !i) {
			goto vlan_del;
		} else if (!ret && i) {
			ret = -EINVAL;
			goto vlan_del;
		} else {
			ret = 0;
		}

		tpriv->ok = false;
	}

vlan_del:
	vlan_vid_del(priv->dev, htons(ETH_P_8021Q), tpriv->vlan_id);
cleanup:
	dev_remove_pack(&tpriv->pt);
	kfree(tpriv);
	return ret;
}

static int bstmac_test_vlanfilt(struct bstgmac_priv *priv)
{
	if (!priv->dma_cap.vlhash)
		return -EOPNOTSUPP;

	return __bstmac_test_vlanfilt(priv);
}

static int bstmac_test_vlanfilt_perfect(struct bstgmac_priv *priv)
{
	int ret, prev_cap = priv->dma_cap.vlhash;

	if (!(priv->dev->features & NETIF_F_HW_VLAN_CTAG_FILTER))
		return -EOPNOTSUPP;

	priv->dma_cap.vlhash = 0;
	ret = __bstmac_test_vlanfilt(priv);
	priv->dma_cap.vlhash = prev_cap;

	return ret;
}

static int __bstmac_test_dvlanfilt(struct bstgmac_priv *priv)
{
	struct bstmac_packet_attrs attr = { };
	struct bstmac_test_priv *tpriv;
	struct sk_buff *skb = NULL;
	int ret = 0, i;

	tpriv = kzalloc(sizeof(*tpriv), GFP_KERNEL);
	if (!tpriv)
		return -ENOMEM;

	tpriv->ok = false;
	tpriv->double_vlan = true;
	init_completion(&tpriv->comp);

	tpriv->pt.type = htons(ETH_P_8021Q);
	tpriv->pt.func = bstmac_test_vlan_validate;
	tpriv->pt.dev = priv->dev;
	tpriv->pt.af_packet_priv = tpriv;
	tpriv->packet = &attr;

	/*
	 * As we use HASH filtering, false positives may appear. This is a
	 * specially chosen ID so that adjacent IDs (+4) have different
	 * HASH values.
	 */
	tpriv->vlan_id = 0x123;
	dev_add_pack(&tpriv->pt);

	ret = vlan_vid_add(priv->dev, htons(ETH_P_8021AD), tpriv->vlan_id);
	if (ret)
		goto cleanup;

	for (i = 0; i < 4; i++) {
		attr.vlan = 2;
		attr.vlan_id_out = tpriv->vlan_id + i;
		attr.dst = priv->dev->dev_addr;
		attr.sport = 9;
		attr.dport = 9;

		skb = bstmac_test_get_udp_skb(priv, &attr);
		if (!skb) {
			ret = -ENOMEM;
			goto vlan_del;
		}

		ret = dev_direct_xmit(skb, 0);
		if (ret)
			goto vlan_del;

		wait_for_completion_timeout(&tpriv->comp, bstmac_LB_TIMEOUT);
		ret = tpriv->ok ? 0 : -ETIMEDOUT;
		if (ret && !i) {
			goto vlan_del;
		} else if (!ret && i) {
			ret = -EINVAL;
			goto vlan_del;
		} else {
			ret = 0;
		}

		tpriv->ok = false;
	}

vlan_del:
	vlan_vid_del(priv->dev, htons(ETH_P_8021AD), tpriv->vlan_id);
cleanup:
	dev_remove_pack(&tpriv->pt);
	kfree(tpriv);
	return ret;
}

static int bstmac_test_dvlanfilt(struct bstgmac_priv *priv)
{
	if (!priv->dma_cap.vlhash)
		return -EOPNOTSUPP;

	return __bstmac_test_dvlanfilt(priv);
}

static int bstmac_test_dvlanfilt_perfect(struct bstgmac_priv *priv)
{
	int ret, prev_cap = priv->dma_cap.vlhash;

	if (!(priv->dev->features & NETIF_F_HW_VLAN_STAG_FILTER))
		return -EOPNOTSUPP;

	priv->dma_cap.vlhash = 0;
	ret = __bstmac_test_dvlanfilt(priv);
	priv->dma_cap.vlhash = prev_cap;

	return ret;
}

#if 0
#ifdef CONFIG_NET_CLS_ACT
static int bstmac_test_rxp(struct bstgmac_priv *priv)
{
	unsigned char addr[ETH_ALEN] = {0xde, 0xad, 0xbe, 0xef, 0x00, 0x00};
	struct tc_cls_u32_offload cls_u32 = { };
	struct bstmac_packet_attrs attr = { };
	struct tc_action **actions, *act;
	struct tc_u32_sel *sel;
	struct tcf_exts *exts;
	int ret, i, nk = 1;

	if (!tc_can_offload(priv->dev))
		return -EOPNOTSUPP;
	if (!priv->dma_cap.frpsel)
		return -EOPNOTSUPP;

	sel = kzalloc(struct_size(sel, keys, nk), GFP_KERNEL);
	if (!sel)
		return -ENOMEM;

	exts = kzalloc(sizeof(*exts), GFP_KERNEL);
	if (!exts) {
		ret = -ENOMEM;
		goto cleanup_sel;
	}

	actions = kzalloc(nk * sizeof(*actions), GFP_KERNEL);
	if (!actions) {
		ret = -ENOMEM;
		goto cleanup_exts;
	}

	act = kzalloc(nk * sizeof(*act), GFP_KERNEL);
	if (!act) {
		ret = -ENOMEM;
		goto cleanup_actions;
	}

	cls_u32.command = TC_CLSU32_NEW_KNODE;
	cls_u32.common.chain_index = 0;
	cls_u32.common.protocol = htons(ETH_P_ALL);
	cls_u32.knode.exts = exts;
	cls_u32.knode.sel = sel;
	cls_u32.knode.handle = 0x123;

	exts->nr_actions = nk;
	exts->actions = actions;
	for (i = 0; i < nk; i++) {
		struct tcf_gact *gact = to_gact(&act[i]);

		actions[i] = &act[i];
		gact->tcf_action = TC_ACT_SHOT;
	}

	sel->nkeys = nk;
	sel->offshift = 0;
	sel->keys[0].off = 6;
	sel->keys[0].val = htonl(0xdeadbeef);
	sel->keys[0].mask = ~0x0;

	ret = bstgmac_tc_setup_cls_u32(priv, priv, &cls_u32);
	if (ret)
		goto cleanup_act;

	attr.dst = priv->dev->dev_addr;
	attr.src = addr;

	ret = __bstmac_test_loopback(priv, &attr);
	ret = ret ? 0 : -EINVAL; /* Shall NOT receive packet */

	cls_u32.command = TC_CLSU32_DELETE_KNODE;
	bstgmac_tc_setup_cls_u32(priv, priv, &cls_u32);

cleanup_act:
	kfree(act);
cleanup_actions:
	kfree(actions);
cleanup_exts:
	kfree(exts);
cleanup_sel:
	kfree(sel);
	return ret;
}
#else
static int bstmac_test_rxp(struct bstgmac_priv *priv)
{
	return -EOPNOTSUPP;
}
#endif
#endif
static int bstmac_test_desc_sai(struct bstgmac_priv *priv)
{
	unsigned char src[ETH_ALEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	struct bstmac_packet_attrs attr = { };
	int ret;

	if (!priv->dma_cap.vlins)
		return -EOPNOTSUPP;

	attr.remove_sa = true;
	attr.sarc = true;
	attr.src = src;
	attr.dst = priv->dev->dev_addr;

	priv->sarc_type = 0x1;

	ret = __bstmac_test_loopback(priv, &attr);

	priv->sarc_type = 0x0;
	return ret;
}

static int bstmac_test_desc_sar(struct bstgmac_priv *priv)
{
	unsigned char src[ETH_ALEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	struct bstmac_packet_attrs attr = { };
	int ret;

	if (!priv->dma_cap.vlins)
		return -EOPNOTSUPP;

	attr.sarc = true;
	attr.src = src;
	attr.dst = priv->dev->dev_addr;

	priv->sarc_type = 0x2;

	ret = __bstmac_test_loopback(priv, &attr);

	priv->sarc_type = 0x0;
	return ret;
}

static int bstmac_test_reg_sai(struct bstgmac_priv *priv)
{
	unsigned char src[ETH_ALEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	struct bstmac_packet_attrs attr = { };
	int ret;

	if (!priv->dma_cap.vlins)
		return -EOPNOTSUPP;

	attr.remove_sa = true;
	attr.sarc = true;
	attr.src = src;
	attr.dst = priv->dev->dev_addr;

	if (bstmac_sarc_configure(priv, priv->ioaddr, 0x2))
		return -EOPNOTSUPP;

	ret = __bstmac_test_loopback(priv, &attr);

	bstmac_sarc_configure(priv, priv->ioaddr, 0x0);
	return ret;
}

static int bstmac_test_reg_sar(struct bstgmac_priv *priv)
{
	unsigned char src[ETH_ALEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	struct bstmac_packet_attrs attr = { };
	int ret;

	if (!priv->dma_cap.vlins)
		return -EOPNOTSUPP;

	attr.sarc = true;
	attr.src = src;
	attr.dst = priv->dev->dev_addr;

	if (bstmac_sarc_configure(priv, priv->ioaddr, 0x3))
		return -EOPNOTSUPP;

	ret = __bstmac_test_loopback(priv, &attr);

	bstmac_sarc_configure(priv, priv->ioaddr, 0x0);
	return ret;
}

static int bstmac_test_vlanoff_common(struct bstgmac_priv *priv, bool svlan)
{
	struct bstmac_packet_attrs attr = { };
	struct bstmac_test_priv *tpriv;
	struct sk_buff *skb = NULL;
	int ret = 0;
	u16 proto;

	if (!priv->dma_cap.vlins)
		return -EOPNOTSUPP;

	tpriv = kzalloc(sizeof(*tpriv), GFP_KERNEL);
	if (!tpriv)
		return -ENOMEM;

	proto = svlan ? ETH_P_8021AD : ETH_P_8021Q;

	tpriv->ok = false;
	tpriv->double_vlan = svlan;
	init_completion(&tpriv->comp);

	tpriv->pt.type = svlan ? htons(ETH_P_8021Q) : htons(ETH_P_IP);
	tpriv->pt.func = bstmac_test_vlan_validate;
	tpriv->pt.dev = priv->dev;
	tpriv->pt.af_packet_priv = tpriv;
	tpriv->packet = &attr;
	tpriv->vlan_id = 0x123;
	dev_add_pack(&tpriv->pt);

	ret = vlan_vid_add(priv->dev, htons(proto), tpriv->vlan_id);
	if (ret)
		goto cleanup;

	attr.dst = priv->dev->dev_addr;

	skb = bstmac_test_get_udp_skb(priv, &attr);
	if (!skb) {
		ret = -ENOMEM;
		goto vlan_del;
	}

	__vlan_hwaccel_put_tag(skb, htons(proto), tpriv->vlan_id);
	skb->protocol = htons(proto);

	ret = dev_direct_xmit(skb, 0);
	if (ret)
		goto vlan_del;

	wait_for_completion_timeout(&tpriv->comp, bstmac_LB_TIMEOUT);
	ret = tpriv->ok ? 0 : -ETIMEDOUT;

vlan_del:
	vlan_vid_del(priv->dev, htons(proto), tpriv->vlan_id);
cleanup:
	dev_remove_pack(&tpriv->pt);
	kfree(tpriv);
	return ret;
}

static int bstmac_test_vlanoff(struct bstgmac_priv *priv)
{
	return bstmac_test_vlanoff_common(priv, false);
}

static int bstmac_test_svlanoff(struct bstgmac_priv *priv)
{
	if (!priv->dma_cap.dvlan)
		return -EOPNOTSUPP;
	return bstmac_test_vlanoff_common(priv, true);
}

#ifdef CONFIG_NET_CLS_ACT
static int __bstmac_test_l3filt(struct bstgmac_priv *priv, u32 dst, u32 src,
				u32 dst_mask, u32 src_mask)
{
	struct flow_dissector_key_ipv4_addrs key, mask;
	unsigned long dummy_cookie = 0xdeadbeef;
	struct bstmac_packet_attrs attr = { };
	struct flow_dissector *dissector;
	struct flow_cls_offload *cls;
	int ret, old_enable = 0;
	struct flow_rule *rule;

	if (!tc_can_offload(priv->dev))
		return -EOPNOTSUPP;
	if (!priv->dma_cap.l3l4fnum)
		return -EOPNOTSUPP;
	if (priv->rss.enable) {
		old_enable = priv->rss.enable;
		priv->rss.enable = false;
		bstgmac_rss_configure(priv, priv->hw, NULL,
				     priv->plat->rx_queues_to_use);
	}

	dissector = kzalloc(sizeof(*dissector), GFP_KERNEL);
	if (!dissector) {
		ret = -ENOMEM;
		goto cleanup_rss;
	}

	dissector->used_keys |= (1 << FLOW_DISSECTOR_KEY_IPV4_ADDRS);
	dissector->offset[FLOW_DISSECTOR_KEY_IPV4_ADDRS] = 0;

	cls = kzalloc(sizeof(*cls), GFP_KERNEL);
	if (!cls) {
		ret = -ENOMEM;
		goto cleanup_dissector;
	}

	cls->common.chain_index = 0;
	cls->command = FLOW_CLS_REPLACE;
	cls->cookie = dummy_cookie;

	rule = kzalloc(struct_size(rule, action.entries, 1), GFP_KERNEL);
	if (!rule) {
		ret = -ENOMEM;
		goto cleanup_cls;
	}

	rule->match.dissector = dissector;
	rule->match.key = (void *)&key;
	rule->match.mask = (void *)&mask;

	key.src = htonl(src);
	key.dst = htonl(dst);
	mask.src = src_mask;
	mask.dst = dst_mask;

	cls->rule = rule;

	rule->action.entries[0].id = FLOW_ACTION_DROP;
	rule->action.entries[0].hw_stats = FLOW_ACTION_HW_STATS_ANY;
	rule->action.num_entries = 1;

	attr.dst = priv->dev->dev_addr;
	attr.ip_dst = dst;
	attr.ip_src = src;

	/* Shall receive packet */
	ret = __bstmac_test_loopback(priv, &attr);
	if (ret)
		goto cleanup_rule;

	ret = bstgmac_tc_setup_cls(priv, priv, cls);
	if (ret)
		goto cleanup_rule;

	/* Shall NOT receive packet */
	ret = __bstmac_test_loopback(priv, &attr);
	ret = ret ? 0 : -EINVAL;

	cls->command = FLOW_CLS_DESTROY;
	bstgmac_tc_setup_cls(priv, priv, cls);
cleanup_rule:
	kfree(rule);
cleanup_cls:
	kfree(cls);
cleanup_dissector:
	kfree(dissector);
cleanup_rss:
	if (old_enable) {
		priv->rss.enable = old_enable;
		bstgmac_rss_configure(priv, priv->hw, &priv->rss,
				     priv->plat->rx_queues_to_use);
	}

	return ret;
}
#else
static int __bstmac_test_l3filt(struct bstgmac_priv *priv, u32 dst, u32 src,
				u32 dst_mask, u32 src_mask)
{
	return -EOPNOTSUPP;
}
#endif

int bstmac_test_l3filt_da(struct bstgmac_priv *priv)
{
	u32 addr = 0x10203040;

	return __bstmac_test_l3filt(priv, addr, 0, ~0, 0);
}

int bstmac_test_l3filt_sa(struct bstgmac_priv *priv)
{
	u32 addr = 0x10203040;

	return __bstmac_test_l3filt(priv, 0, addr, 0, ~0);
}

#ifdef CONFIG_NET_CLS_ACT
static int __bstmac_test_l4filt(struct bstgmac_priv *priv, u32 dst, u32 src,
				u32 dst_mask, u32 src_mask, bool udp)
{
	struct {
		struct flow_dissector_key_basic bkey;
		struct flow_dissector_key_ports key;
	} __aligned(BITS_PER_LONG / 8) keys;
	struct {
		struct flow_dissector_key_basic bmask;
		struct flow_dissector_key_ports mask;
	} __aligned(BITS_PER_LONG / 8) masks;
	unsigned long dummy_cookie = 0xdeadbeef;
	struct bstmac_packet_attrs attr = { };
	struct flow_dissector *dissector;
	struct flow_cls_offload *cls;
	int ret, old_enable = 0;
	struct flow_rule *rule;

	if (!tc_can_offload(priv->dev))
		return -EOPNOTSUPP;
	if (!priv->dma_cap.l3l4fnum)
		return -EOPNOTSUPP;
	if (priv->rss.enable) {
		old_enable = priv->rss.enable;
		priv->rss.enable = false;
		bstgmac_rss_configure(priv, priv->hw, NULL,
				     priv->plat->rx_queues_to_use);
	}

	dissector = kzalloc(sizeof(*dissector), GFP_KERNEL);
	if (!dissector) {
		ret = -ENOMEM;
		goto cleanup_rss;
	}

	dissector->used_keys |= (1 << FLOW_DISSECTOR_KEY_BASIC);
	dissector->used_keys |= (1 << FLOW_DISSECTOR_KEY_PORTS);
	dissector->offset[FLOW_DISSECTOR_KEY_BASIC] = 0;
	dissector->offset[FLOW_DISSECTOR_KEY_PORTS] = offsetof(typeof(keys), key);

	cls = kzalloc(sizeof(*cls), GFP_KERNEL);
	if (!cls) {
		ret = -ENOMEM;
		goto cleanup_dissector;
	}

	cls->common.chain_index = 0;
	cls->command = FLOW_CLS_REPLACE;
	cls->cookie = dummy_cookie;

	rule = kzalloc(struct_size(rule, action.entries, 1), GFP_KERNEL);
	if (!rule) {
		ret = -ENOMEM;
		goto cleanup_cls;
	}

	rule->match.dissector = dissector;
	rule->match.key = (void *)&keys;
	rule->match.mask = (void *)&masks;

	keys.bkey.ip_proto = udp ? IPPROTO_UDP : IPPROTO_TCP;
	keys.key.src = htons(src);
	keys.key.dst = htons(dst);
	masks.mask.src = src_mask;
	masks.mask.dst = dst_mask;

	cls->rule = rule;

	rule->action.entries[0].id = FLOW_ACTION_DROP;
	rule->action.entries[0].hw_stats = FLOW_ACTION_HW_STATS_ANY;
	rule->action.num_entries = 1;

	attr.dst = priv->dev->dev_addr;
	attr.tcp = !udp;
	attr.sport = src;
	attr.dport = dst;
	attr.ip_dst = 0;

	/* Shall receive packet */
	ret = __bstmac_test_loopback(priv, &attr);
	if (ret)
		goto cleanup_rule;

	ret = bstgmac_tc_setup_cls(priv, priv, cls);
	if (ret)
		goto cleanup_rule;

	/* Shall NOT receive packet */
	ret = __bstmac_test_loopback(priv, &attr);
	ret = ret ? 0 : -EINVAL;

	cls->command = FLOW_CLS_DESTROY;
	bstgmac_tc_setup_cls(priv, priv, cls);
cleanup_rule:
	kfree(rule);
cleanup_cls:
	kfree(cls);
cleanup_dissector:
	kfree(dissector);
cleanup_rss:
	if (old_enable) {
		priv->rss.enable = old_enable;
		bstgmac_rss_configure(priv, priv->hw, &priv->rss,
				     priv->plat->rx_queues_to_use);
	}

	return ret;
}
#else
static int __bstmac_test_l4filt(struct bstgmac_priv *priv, u32 dst, u32 src,
				u32 dst_mask, u32 src_mask, bool udp)
{
	return -EOPNOTSUPP;
}
#endif

int bstmac_test_l4filt_da_tcp(struct bstgmac_priv *priv)
{
	u16 dummy_port = 0x123;

	return __bstmac_test_l4filt(priv, dummy_port, 0, ~0, 0, false);
}

int bstmac_test_l4filt_sa_tcp(struct bstgmac_priv *priv)
{
	u16 dummy_port = 0x123;

	return __bstmac_test_l4filt(priv, 0, dummy_port, 0, ~0, false);
}

int bstmac_test_l4filt_da_udp(struct bstgmac_priv *priv)
{
	u16 dummy_port = 0x123;

	return __bstmac_test_l4filt(priv, dummy_port, 0, ~0, 0, true);
}

int bstmac_test_l4filt_sa_udp(struct bstgmac_priv *priv)
{
	u16 dummy_port = 0x123;

	return __bstmac_test_l4filt(priv, 0, dummy_port, 0, ~0, true);
}
#if 0
static int bstmac_test_arp_validate(struct sk_buff *skb,
				    struct net_device *ndev,
				    struct packet_type *pt,
				    struct net_device *orig_ndev)
{
	struct bstmac_test_priv *tpriv = pt->af_packet_priv;
	struct ethhdr *ehdr;
	struct arphdr *ahdr;

	ehdr = (struct ethhdr *)skb_mac_header(skb);
	if (!ether_addr_equal_unaligned(ehdr->h_dest, tpriv->packet->src))
		goto out;

	ahdr = arp_hdr(skb);
	if (ahdr->ar_op != htons(ARPOP_REPLY))
		goto out;

	tpriv->ok = true;
	complete(&tpriv->comp);
out:
	kfree_skb(skb);
	return 0;
}

static int bstmac_test_arpoffload(struct bstgmac_priv *priv)
{
	unsigned char src[ETH_ALEN] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
	unsigned char dst[ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	struct bstmac_packet_attrs attr = { };
	struct bstmac_test_priv *tpriv;
	struct sk_buff *skb = NULL;
	u32 ip_addr = 0xdeadcafe;
	u32 ip_src = 0xdeadbeef;
	int ret;

	if (!priv->dma_cap.arpoffsel)
		return -EOPNOTSUPP;

	tpriv = kzalloc(sizeof(*tpriv), GFP_KERNEL);
	if (!tpriv)
		return -ENOMEM;

	tpriv->ok = false;
	init_completion(&tpriv->comp);

	tpriv->pt.type = htons(ETH_P_ARP);
	tpriv->pt.func = bstmac_test_arp_validate;
	tpriv->pt.dev = priv->dev;
	tpriv->pt.af_packet_priv = tpriv;
	tpriv->packet = &attr;
	dev_add_pack(&tpriv->pt);

	attr.src = src;
	attr.ip_src = ip_src;
	attr.dst = dst;
	attr.ip_dst = ip_addr;

	skb = bstmac_test_get_arp_skb(priv, &attr);
	if (!skb) {
		ret = -ENOMEM;
		goto cleanup;
	}

	ret = bstmac_set_arp_offload(priv, priv->hw, true, ip_addr);
	if (ret)
		goto cleanup;

	ret = dev_set_promiscuity(priv->dev, 1);
	if (ret)
		goto cleanup;

	ret = dev_direct_xmit(skb, 0);
	if (ret)
		goto cleanup_promisc;

	wait_for_completion_timeout(&tpriv->comp, bstmac_LB_TIMEOUT);
	ret = tpriv->ok ? 0 : -ETIMEDOUT;

cleanup_promisc:
	dev_set_promiscuity(priv->dev, -1);
cleanup:
	bstmac_set_arp_offload(priv, priv->hw, false, 0x0);
	dev_remove_pack(&tpriv->pt);
	kfree(tpriv);
	return ret;
}

static int __bstmac_test_jumbo(struct bstgmac_priv *priv, u16 queue)
{
	struct bstmac_packet_attrs attr = { };
	int size = priv->dma_buf_sz;

	attr.dst = priv->dev->dev_addr;
	attr.max_size = size - ETH_FCS_LEN;
	attr.queue_mapping = queue;

	return __bstmac_test_loopback(priv, &attr);
}

static int bstmac_test_jumbo(struct bstgmac_priv *priv)
{
	return __bstmac_test_jumbo(priv, 0);
}

static int bstmac_test_mjumbo(struct bstgmac_priv *priv)
{
	u32 chan, tx_cnt = priv->plat->tx_queues_to_use;
	int ret;

	if (tx_cnt <= 1)
		return -EOPNOTSUPP;

	for (chan = 0; chan < tx_cnt; chan++) {
		ret = __bstmac_test_jumbo(priv, chan);
		if (ret)
			return ret;
	}

	return 0;
}

static int bstmac_test_sph(struct bstgmac_priv *priv)
{
	unsigned long cnt_end, cnt_start = priv->xstats.rx_split_hdr_pkt_n;
	struct bstmac_packet_attrs attr = { };
	int ret;

	if (!priv->sph)
		return -EOPNOTSUPP;

	/* Check for UDP first */
	attr.dst = priv->dev->dev_addr;
	attr.tcp = false;

	ret = __bstmac_test_loopback(priv, &attr);
	if (ret)
		return ret;

	cnt_end = priv->xstats.rx_split_hdr_pkt_n;
	if (cnt_end <= cnt_start)
		return -EINVAL;

	/* Check for TCP now */
	cnt_start = cnt_end;

	attr.dst = priv->dev->dev_addr;
	attr.tcp = true;

	ret = __bstmac_test_loopback(priv, &attr);
	if (ret)
		return ret;

	cnt_end = priv->xstats.rx_split_hdr_pkt_n;
	if (cnt_end <= cnt_start)
		return -EINVAL;

	return 0;
}
#endif
static int bstmac_test_tbs(struct bstgmac_priv *priv)
{
#define BSTMAC_TBS_LT_OFFSET		(50 * 1000 * 1000) /* 500 500 * 1000 * 1000ms*/
	struct bstmac_packet_attrs attr = { };
	struct tc_etf_qopt_offload qopt;
	u64 start_time, curr_time = 0;
	unsigned long flags;
	int ret, i;
	struct timespec64 ts;

	if (!priv->hwts_tx_en)
		return -EOPNOTSUPP;

	/* Find first TBS enabled Queue, if any */
	for (i = 0; i < priv->plat->tx_queues_to_use; i++)
		if (priv->tx_queue[i].tbs & BSTMAC_TBS_AVAIL)
			break;

	if (i >= priv->plat->tx_queues_to_use)
		return -EOPNOTSUPP;

	qopt.enable = true;
	qopt.queue = i;

	ret = bstmac_tc_setup_etf(priv, priv, &qopt);
	if (ret)
		return ret;

	spin_lock_irqsave(&priv->ptp_lock, flags);
	bstgmac_get_systime(priv, priv->ptpaddr, &curr_time);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	if (!curr_time) {
		ret = -EOPNOTSUPP;
		goto fail_disable;
	}

	start_time = curr_time;
	curr_time += (BSTMAC_TBS_LT_OFFSET);
	ts = ns_to_timespec64(curr_time);

	attr.dst = priv->dev->dev_addr;
	attr.timestamp = curr_time;
	attr.timeout = nsecs_to_jiffies(BSTMAC_TBS_LT_OFFSET);
	attr.queue_mapping = i;
/* expiry 
 1.mdelay(500);
 mdelay(500);
 mdelay(500);
 *2.open ptp4l in master side, do not excute ptp4l in slaver side
 * xgmac need set 0x1048
 */
	ret = __bstmac_test_loopback(priv, &attr);
	if (ret) {
		/* Check if expected time has elapsed */
		spin_lock_irqsave(&priv->ptp_lock, flags);
		bstgmac_get_systime(priv, priv->ptpaddr, &curr_time);
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
		ts = ns_to_timespec64(curr_time);
		pr_err("%s line %d sec 0x%llx nsec 0x%lx\n", __func__, __LINE__, ts.tv_sec, ts.tv_nsec);
		goto fail_disable;
	}
	/* Check if expected time has elapsed */
	spin_lock_irqsave(&priv->ptp_lock, flags);
	bstgmac_get_systime(priv, priv->ptpaddr, &curr_time);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	if ((curr_time - start_time) < BSTMAC_TBS_LT_OFFSET)
		ret = -EINVAL;

	ts = ns_to_timespec64(curr_time);
pr_err("%s line %d sec 0x%llx nsec 0x%lx\n", __func__, __LINE__, ts.tv_sec, ts.tv_nsec);
fail_disable:
	qopt.enable = false;
	bstmac_tc_setup_etf(priv, priv, &qopt);
	return ret;
}

static int bstmac_test_ptp(struct bstgmac_priv *priv)
{
	unsigned char mc_addr[ETH_ALEN] = {0x01, 0x1B, 0x19, 0x00, 0x00, 0x00};
	struct bstmac_packet_attrs attr = { };
	int ret;

	if (bstmac_filter_check(priv))
		return -EOPNOTSUPP;
	if (netdev_mc_count(priv->dev) >= priv->hw->multicast_filter_bins)
		return -EOPNOTSUPP;

	ret = dev_mc_add(priv->dev, mc_addr);
	if (ret)
		return ret;

	attr.ieee_1588 = true;
	attr.dst = mc_addr;

	/* Shall receive packet */
	ret = __bstmac_test_loopback(priv, &attr);
	if (ret)
		goto cleanup;

cleanup:
	dev_mc_del(priv->dev, mc_addr);
	return ret;
}

static int bstmac_test_coe(struct bstgmac_priv *priv)
{
	unsigned char mc_addr[ETH_ALEN] = {0xf1, 0x01, 0x44, 0x55, 0x66, 0x77};
	struct bstmac_packet_attrs attr = { };
	int ret;

	ret = dev_mc_add(priv->dev, mc_addr);
	if (ret)
		return ret;

	attr.dst = mc_addr;

	/* Shall receive packet */
	ret = __bstmac_test_loopback(priv, &attr);
	if (ret)
		goto cleanup;

	/* Do not fill checksum */
	attr.non_csum = 1;

	/* Shall NOT receive packet */
	ret = __bstmac_test_loopback(priv, &attr);
	ret = ret ? 0 : -EINVAL;

cleanup:
	dev_mc_del(priv->dev, mc_addr);
	return ret;
}

// 函数：构造并发送IPv6数据包
static struct sk_buff* build_ipv6_packet(struct bstgmac_priv *priv,
								char *src_addr, char *dst_addr)
{
    struct sk_buff *skb;
    struct udphdr *udp;
    struct ipv6hdr *ip6;
    size_t udp_len;
    int total_len;
	struct bstmachdr *shdr;
	struct ethhdr *ehdr;

    // 1. 计算包长度
    udp_len = sizeof(struct udphdr) + sizeof(struct bstmachdr);
    total_len = sizeof(struct ipv6hdr) + udp_len;
    
    // 2. 分配SKB缓冲区
    skb = alloc_skb(ETH_HLEN + total_len, GFP_ATOMIC);
    if (!skb) {
        pr_err("Failed to allocate skb\n");
        return NULL;
    }
    // 设置预留空间（网络和传输层头部）
    skb_reserve(skb, ETH_HLEN);

    // 3. 设置IPv6头部
    ip6 = skb_put_zero(skb, sizeof(struct ipv6hdr));
    ip6->version = 6;
    ip6->priority = 0;
    memset(ip6->flow_lbl, 0, sizeof(ip6->flow_lbl));
    ip6->payload_len = htons(udp_len);
    ip6->nexthdr = IPPROTO_UDP;  // 下一头部为UDP
    ip6->hop_limit = 64;         // TTL
    
    // 设置源和目标地址
    memcpy(&ip6->saddr, src_addr, sizeof(struct in6_addr));
    memcpy(&ip6->daddr, dst_addr, sizeof(struct in6_addr));
    
    // 重置网络层头部位置
    skb_reset_network_header(skb);
    
    // 4. 设置UDP头部
    udp = skb_put_zero(skb, sizeof(struct udphdr)); 
    udp->dest = htons(0x5678);
    udp->len = htons(udp_len);
    
    // 5. 添加数据载荷
    shdr = skb_put(skb, sizeof(struct bstmachdr));
	shdr->version = 0;
	shdr->magic = cpu_to_be64(bstmac_TEST_PKT_MAGIC);
	shdr->id = bstmac_test_next_id++;

    /* 6. 设置传输层头部位置（UDP头部位置）*/
	skb_set_transport_header(skb, sizeof(struct ipv6hdr));

	/* 7. 计算UDP校验和（包括伪头部、UDP头部和负载）*/
	// 注意：计算校验和需要完整的UDP段（头+负载）
	udp->check = csum_ipv6_magic(&ip6->saddr, &ip6->daddr,
				     udp_len, IPPROTO_UDP,
				     csum_partial(udp, udp_len, 0));
	// 8.添加mac头
	ehdr = skb_push(skb, ETH_HLEN);
    memset(ehdr, 0, ETH_HLEN);
	ether_addr_copy(ehdr->h_dest, priv->dev->dev_addr);
	ehdr->h_proto = htons(ETH_P_IPV6);
	skb_reset_mac_header(skb);
	
	// 9. 设置协议类型
    skb->pkt_type = PACKET_HOST;
	skb->dev = priv->dev;
    skb->protocol = htons(ETH_P_IPV6);
	skb->priority = 0;
	// // 10. 验证数据包完整性
    // if (skb_headroom(skb) < ETH_HLEN) {
    //     pr_err("Insufficient headroom: %d < %d\n",
    //               skb_headroom(skb), ETH_HLEN);
    //     kfree_skb(skb);
    //     return NULL;
    // }

	return skb;
}

static int bstmac_test_ipv6_loopback_validate(struct sk_buff *skb,
					 struct net_device *ndev,
					 struct packet_type *pt,
					 struct net_device *orig_ndev)
{
	struct bstmac_test_priv *tpriv = pt->af_packet_priv;
	const unsigned char *dst = tpriv->packet->dst;
	struct bstmachdr *shdr;
	struct ethhdr *ehdr;
	struct udphdr *uhdr;
	struct ipv6hdr *ip6h;

	skb = skb_unshare(skb, GFP_ATOMIC);
	if (!skb) {
		pr_err("%s line %d\n", __func__, __LINE__);
		goto out;
	}
	if (skb_linearize(skb)) {
		pr_err("%s line %d\n", __func__, __LINE__);
		goto out;
	}
	if (skb_headlen(skb) < (bstmac_TEST_IPV6PKT_SIZE - ETH_HLEN)) {
		pr_err("%s line %d\n", __func__, __LINE__);
		goto out;
	}
	ehdr = (struct ethhdr *)skb_mac_header(skb);
	if (dst) {
		if (!ether_addr_equal_unaligned(ehdr->h_dest, dst)) {
			pr_err("%s line %d\n", __func__, __LINE__);
			goto out;
		}
	}

	ip6h = ipv6_hdr(skb);
	if (ip6h->nexthdr != IPPROTO_UDP) {
		pr_err("%s line %d\n", __func__, __LINE__);
		goto out;
	}
	uhdr = udp_hdr(skb);
	if (uhdr->dest != htons(tpriv->packet->dport)) {
		pr_err("%s line %d dest 0x%x dport 0x%x\n", __func__, __LINE__, uhdr->dest, htons(tpriv->packet->dport));
		goto out;
	}
	shdr = (struct bstmachdr *)((u8 *)uhdr + sizeof(*uhdr));
	
	if (shdr->magic != cpu_to_be64(bstmac_TEST_PKT_MAGIC)) {
		pr_err("%s line %d\n", __func__, __LINE__);
		goto out;
	}
	tpriv->ok = true;
	complete(&tpriv->comp);
out:
	kfree_skb(skb);
	return 0;
}

static int __bstmac_test_ipv6_loopback(struct bstgmac_priv *priv,
				  struct bstmac_packet_attrs *attr)
{	
	struct bstmac_test_priv *tpriv;
	struct sk_buff *skb = NULL;
	int ret = 0;
	char src_addr[16] = {0x20,0x01,0x20,0x01,0x20,0x01,0x20,0x01,0x20,0x01,0x00,0x00,0x00,
						0x00,0x01,0x58};
	char dst_addr[16] = {0x20,0x01,0x20,0x01,0x20,0x01,0x20,0x01,0x20,0x01,0x00,0x00,0x00,
						0x00,0x01,0x50};

	tpriv = kzalloc(sizeof(*tpriv), GFP_KERNEL);
	if (!tpriv)
		return -ENOMEM;

	tpriv->ok = false;
	init_completion(&tpriv->comp);

	tpriv->pt.type = htons(ETH_P_IPV6);
	tpriv->pt.func = bstmac_test_ipv6_loopback_validate;
	tpriv->pt.dev = priv->dev;
	tpriv->pt.af_packet_priv = tpriv;
	tpriv->packet = attr;
	dev_add_pack(&tpriv->pt);

	skb = build_ipv6_packet(priv, src_addr, dst_addr);
	if (!skb) {
		ret = -ENOMEM;
		goto cleanup;
	}

	ret = dev_direct_xmit(skb, 0);
	if (ret) {
		pr_err("%s line %d ret %d\n", __func__, __LINE__, ret);
		goto cleanup;
	}
	if (!attr->timeout)
		attr->timeout = bstmac_LB_TIMEOUT;

	wait_for_completion_timeout(&tpriv->comp, attr->timeout);
	ret = tpriv->ok ? 0 : -ETIMEDOUT;

cleanup:
	dev_remove_pack(&tpriv->pt);
	kfree(tpriv);
	return ret;

    ret = dev_direct_xmit(skb, 0);
    if (ret) {
        pr_err("Failed to send packet: %d\n", ret);
    }

	return 0;
}

int bstmac_test_ipv6_l3filt(struct bstgmac_priv *priv, char *dst, char *src,
				char *dst_mask, char *src_mask)
{
	struct flow_dissector_key_ipv6_addrs key, mask;
	unsigned long dummy_cookie = 0xdeadbeef;
	struct bstmac_packet_attrs attr = { };
	struct flow_dissector *dissector;
	struct flow_cls_offload *cls;
	int ret, old_enable = 0;
	struct flow_rule *rule;

	if (!tc_can_offload(priv->dev))
		return -EOPNOTSUPP;
	if (!priv->dma_cap.l3l4fnum)
		return -EOPNOTSUPP;
	if (priv->rss.enable) {
		old_enable = priv->rss.enable;
		priv->rss.enable = false;
		bstgmac_rss_configure(priv, priv->hw, NULL,
				     priv->plat->rx_queues_to_use);
	}

	dissector = kzalloc(sizeof(*dissector), GFP_KERNEL);
	if (!dissector) {
		ret = -ENOMEM;
		goto cleanup_rss;
	}

	dissector->used_keys |= (1 << FLOW_DISSECTOR_KEY_IPV6_ADDRS);
	dissector->offset[FLOW_DISSECTOR_KEY_IPV6_ADDRS] = 0;

	cls = kzalloc(sizeof(*cls), GFP_KERNEL);
	if (!cls) {
		ret = -ENOMEM;
		goto cleanup_dissector;
	}

	cls->common.chain_index = 0;
	cls->command = FLOW_CLS_REPLACE;
	cls->cookie = dummy_cookie;

	rule = kzalloc(struct_size(rule, action.entries, 1), GFP_KERNEL);
	if (!rule) {
		ret = -ENOMEM;
		goto cleanup_cls;
	}

	rule->match.dissector = dissector;
	rule->match.key = (void *)&key;
	rule->match.mask = (void *)&mask;

	memcpy(&key.src, src, 16);
	memcpy(&key.dst, dst, 16);
	memcpy(&mask.src, src_mask, 16);
	memcpy(&mask.dst, dst_mask, 16);

	cls->rule = rule;

	rule->action.entries[0].id = FLOW_ACTION_DROP;
	rule->action.entries[0].hw_stats = FLOW_ACTION_HW_STATS_ANY;
	rule->action.num_entries = 1;

	attr.dst = priv->dev->dev_addr;
	attr.dport = 0x5678;

	/* Shall receive packet */
	ret = __bstmac_test_ipv6_loopback(priv, &attr);
	if (ret) {
		pr_err("%s line %d ret %d\n", __func__, __LINE__, ret);
		goto cleanup_rule;
	}
	ret = bstgmac_tc_setup_cls(priv, priv, cls);
	if (ret) {
		pr_err("%s line %d ret %d\n", __func__, __LINE__, ret);
		goto cleanup_rule;
	}
	/* Shall NOT receive packet */
	ret = __bstmac_test_ipv6_loopback(priv, &attr);
	ret = ret ? 0 : -EINVAL;

	cls->command = FLOW_CLS_DESTROY;
	bstgmac_tc_setup_cls(priv, priv, cls);
cleanup_rule:
	kfree(rule);
cleanup_cls:
	kfree(cls);
cleanup_dissector:
	kfree(dissector);
cleanup_rss:
	if (old_enable) {
		priv->rss.enable = old_enable;
		bstgmac_rss_configure(priv, priv->hw, &priv->rss,
				     priv->plat->rx_queues_to_use);
	}

	return ret;
}

int bstmac_test_ipv6_l3filt_da(struct bstgmac_priv *priv)
{
	char dst_addr[16] = {0x20,0x01,0x20,0x01,0x20,0x01,0x20,0x01,0x20,0x01,0x00,0x00,0x00,
						0x00,0x01,0x50};
	char dst_msk[16] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff,	0xff, 0xff,0xff, 0xff, 0xff, 0xff};
	char src_addr[16] = {0};
	char src_msk[16] = {0};

	return bstmac_test_ipv6_l3filt(priv, dst_addr, src_addr, dst_msk, src_msk);
}

int bstmac_test_ipv6_l3filt_sa(struct bstgmac_priv *priv)
{
	char src_addr[16] = {0x20,0x01,0x20,0x01,0x20,0x01,0x20,0x01,0x20,0x01,0x00,0x00,0x00,
						0x00,0x01,0x58};
	char src_msk[16] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff,	0xff, 0xff,0xff, 0xff, 0xff, 0xff};
	char dst_addr[16] = {0};
	char dst_msk[16] = {0};

	return bstmac_test_ipv6_l3filt(priv, dst_addr, src_addr, dst_msk, src_msk);
}

static int __bstmac_test_mchpacket(struct bstgmac_priv *priv, u16 queue)
{
	struct bstmac_packet_attrs attr = { };

	attr.dst = priv->dev->dev_addr;
	attr.size = 64;
	attr.queue_mapping = queue;

	return __bstmac_test_loopback(priv, &attr);
}

int bstmac_test_multichannel(struct bstgmac_priv *priv)
{
	u32 chan, val, tx_cnt = priv->plat->tx_queues_to_use;
	int ret;

	if (tx_cnt <= 1)
		return -EOPNOTSUPP;

	val = readl(priv->ioaddr+0xa4);
	for (chan = 0; chan < tx_cnt; chan++) {
		priv->tx_coal_frames[chan] = 1;
		writel(chan * 0x11111, priv->ioaddr+0xa4);
		ret = __bstmac_test_mchpacket(priv, chan);
		if (ret)
			goto bak_val;

        switch (chan) {
            case 0:
				if (!priv->plat->bus_id) {
					if ((priv->rx_irq_num != priv->perch_rx_irq[chan]) || (priv->tx_irq_num != priv->perch_tx_irq[chan]))
						break;
				}
				if (priv->plat->bus_id) {
					if ((priv->rx_irq_num != priv->perch_rx_irq[chan]) || (priv->tx_irq_num != priv->perch_tx_irq[chan]))
						break;					
				}
                break;
            case 1:
				if (!priv->plat->bus_id){
					if ((priv->rx_irq_num != priv->perch_rx_irq[chan]) || (priv->tx_irq_num != priv->perch_tx_irq[chan]))
						break;
				}
				if (priv->plat->bus_id) {
					if ((priv->rx_irq_num != priv->perch_rx_irq[chan]) || (priv->tx_irq_num != priv->perch_tx_irq[chan]))
						break;					
				}
                break;
            case 2:
				if (!priv->plat->bus_id){
					if ((priv->rx_irq_num != priv->perch_rx_irq[chan]) || (priv->tx_irq_num != priv->perch_tx_irq[chan]))
						break;
				}
				if (priv->plat->bus_id) {
					if ((priv->rx_irq_num != priv->perch_rx_irq[chan]) || (priv->tx_irq_num != priv->perch_tx_irq[chan]))
						break;					
				}
                break;
            case 3:
				if (!priv->plat->bus_id){
					if ((priv->rx_irq_num != priv->perch_rx_irq[chan]) || (priv->tx_irq_num != priv->perch_tx_irq[chan]))
						break;
				}
				if (priv->plat->bus_id) {
					if ((priv->rx_irq_num != priv->perch_rx_irq[chan]) || (priv->tx_irq_num != priv->perch_tx_irq[chan]))
						break;					
				}
                break;
            default:
                break;
		}
bak_val:	
		priv->tx_coal_frames[chan] = BSTGMAC_TX_FRAMES;
	}
	
	writel(val, priv->ioaddr+0xa4);

	return 0;
}

#if 0
static struct task_struct	*mac_send_task = NULL;
extern unsigned int send_pkt_type;

static int bstmac_testpkt_validate(struct sk_buff *skb,
					 struct net_device *ndev,
					 struct packet_type *pt,
					 struct net_device *orig_ndev)
{
	struct bstmac_test_priv *tpriv = pt->af_packet_priv;
	unsigned char *src = tpriv->packet->src;
	unsigned char *dst = tpriv->packet->dst;
	struct bstmachdr *shdr;
	struct ethhdr *ehdr;
	struct udphdr *uhdr;
	struct tcphdr *thdr;
	struct iphdr *ihdr;

	skb = skb_unshare(skb, GFP_ATOMIC);
	if (!skb)
		goto out;

	if (skb_linearize(skb))
		goto out;
	if (skb_headlen(skb) < (bstmac_TEST_PKT_SIZE - ETH_HLEN))
		goto out;

	ehdr = (struct ethhdr *)skb_mac_header(skb);
	if (dst) {
		if (!ether_addr_equal_unaligned(ehdr->h_dest, dst))
			goto out;
	}
	if (tpriv->packet->sarc) {
		if (!ether_addr_equal_unaligned(ehdr->h_source, ehdr->h_dest))
			goto out;
	} else if (src) {
		if (!ether_addr_equal_unaligned(ehdr->h_source, src))
			goto out;
	}

	ihdr = ip_hdr(skb);
	if (tpriv->double_vlan)
		ihdr = (struct iphdr *)(skb_network_header(skb) + 4);

	if (tpriv->packet->tcp) {
		if (ihdr->protocol != IPPROTO_TCP)
			goto out;

		thdr = (struct tcphdr *)((u8 *)ihdr + 4 * ihdr->ihl);
		if (thdr->dest != htons(tpriv->packet->dport))
			goto out;

		shdr = (struct bstmachdr *)((u8 *)thdr + sizeof(*thdr));
	} else {
		if (ihdr->protocol != IPPROTO_UDP)
			goto out;

		uhdr = (struct udphdr *)((u8 *)ihdr + 4 * ihdr->ihl);
		if (uhdr->dest != htons(tpriv->packet->dport))
			goto out;

		shdr = (struct bstmachdr *)((u8 *)uhdr + sizeof(*uhdr));
	}

	if (shdr->magic != cpu_to_be64(bstmac_TEST_PKT_MAGIC))
		goto out;
	if (tpriv->packet->exp_hash && !skb->hash)
		goto out;
	if (tpriv->packet->id != shdr->id)
		goto out;

	tpriv->ok = true;
out:
	kfree_skb(skb);
	return 0;
}

static int bstmac_sendpkt_test(struct bstgmac_priv *priv,
				  struct bstmac_packet_attrs *attr)
{
	struct bstmac_test_priv *tpriv;
	struct sk_buff *skb = NULL;
	int ret = 0;

	tpriv = kzalloc(sizeof(*tpriv), GFP_KERNEL);
	if (!tpriv)
		return -ENOMEM;

	tpriv->pt.type = htons(ETH_P_IP);
	tpriv->pt.func = bstmac_testpkt_validate;
	tpriv->pt.dev = priv->dev;
	tpriv->pt.af_packet_priv = tpriv;
	tpriv->packet = attr;

	if (!attr->dont_wait)
		dev_add_pack(&tpriv->pt);

	skb = bstmac_test_get_udp_skb(priv, attr);
	if (!skb) {
		ret = -ENOMEM;
		goto cleanup;
	}

	ret = dev_direct_xmit(skb, attr->queue_mapping);
	if (ret)
		goto cleanup;

	if (attr->dont_wait)
		goto cleanup;
cleanup:
	if (!attr->dont_wait)
		dev_remove_pack(&tpriv->pt);
	kfree(tpriv);
	return ret;
}
		

static int send_pkt(void *data)
{
	struct bstgmac_priv *priv = (struct bstgmac_priv *)data;
	struct bstmac_packet_attrs attr = { };
	int ret, i;
	unsigned char addr[ETH_ALEN] = {0xde, 0xad, 0xbe, 0xef, 0x00, 0x00};

	attr.src = addr;
	attr.dst = priv->dev->dev_addr;
	attr.queue_mapping = 0;
	attr.tcp = 0;
	if (send_pkt_type)
		attr.tcp = 1;

	while (1) {
		ret = bstmac_sendpkt_test(priv, &attr);
		if (ret) {
			pr_err("%s line %d ret %d\n", __func__, __LINE__, ret);
			break;
		}
		i++;
		if (i % 256 == 0)
			msleep(20);
	}

	return 0;
}

static int bstmac_send_pkt(struct bstgmac_priv *priv)
{
	mac_send_task = kthread_create(send_pkt, priv, "%s", "bst-send");
	if (IS_ERR(mac_send_task)) {
		pr_err("%s line %d\n", __func__, __LINE__);
		return PTR_ERR(mac_send_task);
	}
	wake_up_process(mac_send_task);
	return 0;
}
#endif

#define BSTMAC_LOOPBACK_NONE	0
#define BSTMAC_LOOPBACK_MAC	1
#define BSTMAC_LOOPBACK_PHY	2

static const struct bstmac_test {
	char name[ETH_GSTRING_LEN];
	int lb;
	int (*fn)(struct bstgmac_priv *priv);
} bstmac_selftests[] = {

    {
		.name = "MAC Loopback               ",
		.lb = BSTMAC_LOOPBACK_MAC,
		.fn = bstmac_test_mac_loopback,
	},
#if 0
	{
		.name = "PHY Loopback               ",
		.lb = BSTMAC_LOOPBACK_NONE, /* Test will handle it */
		.fn = bstmac_test_phy_loopback,
	}, {
		.name = "MMC Counters               ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_mmc,
	}, {
		.name = "EEE                        ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_eee,
	},
#endif
	{
		.name = "Hash Filter MC             ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_hfilt,
	}, {
		.name = "Perfect Filter UC          ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_pfilt,
	}, {
		.name = "MC Filter                  ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_mcfilt,
	}, {
		.name = "UC Filter                  ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_ucfilt,
	},{
		.name = "Flow Control               ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_flowctrl,
	},
#if 0
	{
		.name = "RSS                        ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_rss,
	},
#endif
	{
		.name = "VLAN Filtering             ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_vlanfilt,
	}, {
		.name = "VLAN Filtering (perf)      ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_vlanfilt_perfect,
	}, {
		.name = "Double VLAN Filter         ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_dvlanfilt,
	}, {
		.name = "Double VLAN Filter (perf)  ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_dvlanfilt_perfect,
	},
#if 0
	{
		.name = "Flexible RX Parser         ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_rxp,
	}, 
#endif
	{
		.name = "SA Insertion (desc)        ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_desc_sai,
	}, {
		.name = "SA Replacement (desc)      ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_desc_sar,
	}, {
		.name = "SA Insertion (reg)         ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_reg_sai,
	}, {
		.name = "SA Replacement (reg)       ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_reg_sar,
	}, {
		.name = "VLAN TX Insertion          ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_vlanoff,
	}, {
		.name = "SVLAN TX Insertion         ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_svlanoff,
	}, {
		.name = "L3 DA Filtering            ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_l3filt_da,
	}, {
		.name = "L3 SA Filtering            ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_l3filt_sa,
	}, {
		.name = "L4 DA TCP Filtering        ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_l4filt_da_tcp,
	}, {
		.name = "L4 SA TCP Filtering        ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_l4filt_sa_tcp,
	}, {
		.name = "L4 DA UDP Filtering        ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_l4filt_da_udp,
	}, {
		.name = "L4 SA UDP Filtering        ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_l4filt_sa_udp,
	},
#if 0
	{
		.name = "ARP Offload                ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_arpoffload,
	}, {
		.name = "Jumbo Frame                ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_jumbo,
	}, {
		.name = "Multichannel Jumbo         ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_mjumbo,
	}, {
		.name = "Split Header               ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_sph,
	},{
		.name = "Send Pkt               ",
		.lb = BSTMAC_LOOPBACK_NONE,
		.fn = bstmac_send_pkt,
	},
#endif         
	{
		.name = "TBS (ETF Scheduler)        ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_tbs,
	},{
		.name = "IEEE 1588        ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_ptp,
	},{
		.name = "Checksum Offload Engine        ",
		.lb = BSTMAC_LOOPBACK_PHY,
		.fn = bstmac_test_coe,
	},
};

void bstmac_selftest_run(struct net_device *dev,
			 struct ethtool_test *etest, u64 *buf)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	int count = bstmac_selftest_get_count(priv);
	int i, ret;

	memset(buf, 0, sizeof(*buf) * count);
	bstmac_test_next_id = 0;

	if (etest->flags != ETH_TEST_FL_OFFLINE) {
		netdev_err(priv->dev, "Only offline tests are supported\n");
		etest->flags |= ETH_TEST_FL_FAILED;
		return;
	} else if (!netif_carrier_ok(dev)) {
		netdev_err(priv->dev, "You need valid Link to execute tests\n");
		etest->flags |= ETH_TEST_FL_FAILED;
		return;
	}

	/* Wait for queues drain */
	msleep(200);

	for (i = 0; i < count; i++) {
		ret = 0;

		switch (bstmac_selftests[i].lb) {
		case BSTMAC_LOOPBACK_PHY:
			ret = -EOPNOTSUPP;
			if (dev->phydev)
				ret = phy_loopback(dev->phydev, true);
			if (!ret)
				break;
			fallthrough;
		case BSTMAC_LOOPBACK_MAC:
			ret = bstmac_set_mac_loopback(priv, priv->ioaddr, true);
			break;
		case BSTMAC_LOOPBACK_NONE:
			break;
		default:
			ret = -EOPNOTSUPP;
			break;
		}

		/*
		 * First tests will always be MAC / PHY loobpack. If any of
		 * them is not supported we abort earlier.
		 */
		if (ret) {
			netdev_err(priv->dev, "Loopback is not supported\n");
			etest->flags |= ETH_TEST_FL_FAILED;
			break;
		}

		ret = bstmac_selftests[i].fn(priv);
		if (ret && (ret != -EOPNOTSUPP))
			etest->flags |= ETH_TEST_FL_FAILED;
		buf[i] = ret;

		switch (bstmac_selftests[i].lb) {
		case BSTMAC_LOOPBACK_PHY:
			ret = -EOPNOTSUPP;
			if (dev->phydev)
				ret = phy_loopback(dev->phydev, false);
			if (!ret)
				break;
			fallthrough;
		case BSTMAC_LOOPBACK_MAC:
			bstmac_set_mac_loopback(priv, priv->ioaddr, false);
			break;
		default:
			break;
		}
	}
}

void bstmac_selftest_get_strings(struct bstgmac_priv *priv, u8 *data)
{
	u8 *p = data;
	int i;

	for (i = 0; i < bstmac_selftest_get_count(priv); i++) {
		snprintf(p, ETH_GSTRING_LEN, "%2d. %s", i + 1,
			 bstmac_selftests[i].name);
		p += ETH_GSTRING_LEN;
	}
}

int bstmac_selftest_get_count(struct bstgmac_priv *priv)
{
	return ARRAY_SIZE(bstmac_selftests);
}
