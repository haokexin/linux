// SPDX-License-Identifier: GPL-2.0
/* Marvell RVU Ethernet driver
 *
 * Copyright (C) 2021 Marvell.
 *
 */
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/inetdevice.h>
#include <linux/bitfield.h>

#include "otx2_common.h"
#include "cn10k.h"
#include "qos.h"

#define OTX2_QOS_QID_INNER		0xFFFFU
#define OTX2_QOS_QID_NONE		0xFFFEU
#define OTX2_QOS_ROOT_CLASSID		0xFFFFFFFF
#define OTX2_QOS_CLASS_NONE		0
#define OTX2_QOS_DEFAULT_PRIO		0xF
#define OTX2_QOS_MAX_PRIO		7
#define OTX2_QOS_STATIC_QUANTUM		INT_MAX
#define OTX2_QOS_INVALID_SQ		0xFFFF

/* Egress rate limiting definitions */
#define MAX_BURST_EXPONENT		0x0FULL
#define MAX_BURST_MANTISSA		0xFFULL
#define MAX_BURST_SIZE			130816ULL
#define MAX_RATE_DIVIDER_EXPONENT	12ULL
#define MAX_RATE_EXPONENT		0x0FULL
#define MAX_RATE_MANTISSA		0xFFULL

/* Bitfields in NIX_TLX_PIR register */
#define TLX_RATE_MANTISSA		GENMASK_ULL(8, 1)
#define TLX_RATE_EXPONENT		GENMASK_ULL(12, 9)
#define TLX_RATE_DIVIDER_EXPONENT	GENMASK_ULL(16, 13)
#define TLX_BURST_MANTISSA		GENMASK_ULL(36, 29)
#define TLX_BURST_EXPONENT		GENMASK_ULL(40, 37)

static bool otx2_qos_is_quantum_static(u32 quantum)
{
	return quantum == OTX2_QOS_STATIC_QUANTUM ? 1 : 0;
}

static int otx2_qos_update_tx_netdev_queues(struct otx2_nic *pfvf)
{
	int tx_queues, err, qos_txqs = 0;
	struct otx2_hw *hw = &pfvf->hw;

	qos_txqs = bitmap_weight(pfvf->qos.qos_sq_bmap,
				 OTX2_QOS_MAX_LEAF_NODES);

	tx_queues = hw->tx_queues + qos_txqs;

	err = netif_set_real_num_tx_queues(pfvf->netdev, tx_queues);
	if (err) {
		netdev_err(pfvf->netdev,
			   "Failed to set no of Tx queues: %d\n", tx_queues);
		return err;
	}

	return 0;
}

static u64 otx2_qos_convert_rate(u64 rate)
{
	u64 converted_rate;

	/* convert bytes per second to Mbps */
	converted_rate = rate * 8;
	converted_rate = max_t(u64, converted_rate / 1000000, 1);

	return converted_rate;
}

static int otx2_qos_quantum_to_dwrr_weight(struct otx2_nic *pfvf, u32 quantum)
{
	u32 weight;

	weight = quantum / pfvf->hw.dwrr_mtu;
	if (quantum % pfvf->hw.dwrr_mtu)
		weight += 1;

	return weight;
}

static void __otx2_qos_txschq_cfg(struct otx2_nic *pfvf,
				  struct otx2_qos_node *node,
				  struct nix_txschq_config *cfg)
{
	struct otx2_hw *hw = &pfvf->hw;
	int num_regs = 0;
	u16 rr_weight;
	u32 quantum;
	u64 maxrate;
	u8 level;

	level = node->level;

	/* program txschq registers */
	if (level == NIX_TXSCH_LVL_SMQ) {
		cfg->reg[num_regs] = NIX_AF_SMQX_CFG(node->schq);
		cfg->regval[num_regs] = ((u64)pfvf->tx_max_pktlen << 8) |
					OTX2_MIN_MTU;
		cfg->regval[num_regs] |= (0x20ULL << 51) | (0x80ULL << 39) |
					 (0x2ULL << 36);
		num_regs++;

		/* configure parent txschq */
		cfg->reg[num_regs] = NIX_AF_MDQX_PARENT(node->schq);
		cfg->regval[num_regs] = node->parent->schq << 16;
		num_regs++;

		/* configure prio/quantum */
		if (node->qid == OTX2_QOS_QID_NONE) {
			cfg->reg[num_regs] = NIX_AF_MDQX_SCHEDULE(node->schq);
			cfg->regval[num_regs] =  node->prio << 24 |
						 mtu_to_dwrr_weight(pfvf,
								    pfvf->tx_max_pktlen);
			num_regs++;
			goto txschq_cfg_out;
		}

		/* configure prio/quantum */
		cfg->reg[num_regs] = NIX_AF_MDQX_SCHEDULE(node->schq);
		if (otx2_qos_is_quantum_static(node->quantum)) {
			cfg->regval[num_regs] = (node->schq -
						 node->parent->prio_anchor) << 24;
		} else {
			quantum = node->quantum ?
				  node->quantum : pfvf->tx_max_pktlen;
			rr_weight = otx2_qos_quantum_to_dwrr_weight(pfvf,
								    quantum);
			cfg->regval[num_regs] = node->parent->act_dwrr_prio << 24 |
						rr_weight;
		}
		num_regs++;

		/* configure PIR */
		maxrate = (node->rate > node->ceil) ? node->rate : node->ceil;

		cfg->reg[num_regs] = NIX_AF_MDQX_PIR(node->schq);
		cfg->regval[num_regs] =
			otx2_get_txschq_rate_regval(pfvf, maxrate, 65536);
		num_regs++;

		/* configure CIR */
		if (!test_bit(QOS_CIR_PIR_SUPPORT, &pfvf->hw.cap_flag)) {
			/* Don't configure CIR when both CIR+PIR not supported
			 * On 96xx, CIR + PIR + RED_ALGO=STALL causes deadlock
			 */
			goto txschq_cfg_out;
		}

		cfg->reg[num_regs] = NIX_AF_MDQX_CIR(node->schq);
		cfg->regval[num_regs] =
			otx2_get_txschq_rate_regval(pfvf, node->rate, 65536);
		num_regs++;
	} else if (level == NIX_TXSCH_LVL_TL4) {
		/* configure parent txschq */
		cfg->reg[num_regs] = NIX_AF_TL4X_PARENT(node->schq);
		cfg->regval[num_regs] = node->parent->schq << 16;
		num_regs++;

		/* return if not htb node */
		if (node->qid == OTX2_QOS_QID_NONE) {
			cfg->reg[num_regs] = NIX_AF_TL4X_SCHEDULE(node->schq);
			cfg->regval[num_regs] =  node->prio << 24 |
						 mtu_to_dwrr_weight(pfvf,
								    pfvf->tx_max_pktlen);
			num_regs++;
			goto txschq_cfg_out;
		}

		/* configure priority/quantum */
		cfg->reg[num_regs] = NIX_AF_TL4X_SCHEDULE(node->schq);
		if (otx2_qos_is_quantum_static(node->quantum)) {
			cfg->regval[num_regs] = (node->schq -
						 node->parent->prio_anchor) << 24;
		} else {
			quantum = node->quantum ?
				  node->quantum : pfvf->tx_max_pktlen;
			rr_weight = otx2_qos_quantum_to_dwrr_weight(pfvf,
								    quantum);
			cfg->regval[num_regs] = node->parent->act_dwrr_prio << 24 |
						rr_weight;
		}
		num_regs++;

		/* configure PIR */
		maxrate = (node->rate > node->ceil) ? node->rate : node->ceil;
		cfg->reg[num_regs] = NIX_AF_TL4X_PIR(node->schq);
		cfg->regval[num_regs] =
			otx2_get_txschq_rate_regval(pfvf, maxrate, 65536);
		num_regs++;

		/* configure CIR */
		if (!test_bit(QOS_CIR_PIR_SUPPORT, &pfvf->hw.cap_flag)) {
			/* Don't configure CIR when both CIR+PIR not supported
			 * On 96xx, CIR + PIR + RED_ALGO=STALL causes deadlock
			 */
			goto txschq_cfg_out;
		}

		cfg->reg[num_regs] = NIX_AF_TL4X_CIR(node->schq);
		cfg->regval[num_regs] =
			otx2_get_txschq_rate_regval(pfvf, node->rate, 65536);
		num_regs++;
	} else if (level == NIX_TXSCH_LVL_TL3) {
		/* configure parent txschq */
		cfg->reg[num_regs] = NIX_AF_TL3X_PARENT(node->schq);
		cfg->regval[num_regs] = node->parent->schq << 16;
		num_regs++;

		/* return if not htb node */
		if (node->qid == OTX2_QOS_QID_NONE) {
			cfg->reg[num_regs] = NIX_AF_TL3X_SCHEDULE(node->schq);
			cfg->regval[num_regs] =  node->prio << 24 |
						 mtu_to_dwrr_weight(pfvf,
								    pfvf->tx_max_pktlen);
			num_regs++;
			goto txschq_cfg_out;
		}

		/* configure priority/quantum */
		cfg->reg[num_regs] = NIX_AF_TL3X_SCHEDULE(node->schq);
		if (otx2_qos_is_quantum_static(node->quantum)) {
			cfg->regval[num_regs] = (node->schq -
						 node->parent->prio_anchor) << 24;
		} else {
			quantum = node->quantum ?
				  node->quantum : pfvf->tx_max_pktlen;
			rr_weight = otx2_qos_quantum_to_dwrr_weight(pfvf,
								    quantum);
			cfg->regval[num_regs] = node->parent->act_dwrr_prio << 24 |
						rr_weight;
		}
		num_regs++;

		/* configure link cfg */
		if (level == pfvf->qos.link_cfg_lvl) {
			cfg->reg[num_regs] = NIX_AF_TL3_TL2X_LINKX_CFG(node->schq, hw->tx_link);
			cfg->regval[num_regs] = BIT_ULL(13) | BIT_ULL(12);
			num_regs++;
		}

		/* configure PIR */
		maxrate = (node->rate > node->ceil) ? node->rate : node->ceil;
		cfg->reg[num_regs] = NIX_AF_TL3X_PIR(node->schq);
		cfg->regval[num_regs] =
			otx2_get_txschq_rate_regval(pfvf, maxrate, 65536);
		num_regs++;

		/* configure CIR */
		if (!test_bit(QOS_CIR_PIR_SUPPORT, &pfvf->hw.cap_flag)) {
			/* Don't configure CIR when both CIR+PIR not supported
			 * On 96xx, CIR + PIR + RED_ALGO=STALL causes deadlock
			 */
			goto txschq_cfg_out;
		}

		cfg->reg[num_regs] = NIX_AF_TL3X_CIR(node->schq);
		cfg->regval[num_regs] =
			otx2_get_txschq_rate_regval(pfvf, node->rate, 65536);
		num_regs++;
	} else if (level == NIX_TXSCH_LVL_TL2) {
		/* configure parent txschq */
		cfg->reg[num_regs] = NIX_AF_TL2X_PARENT(node->schq);
		cfg->regval[num_regs] = hw->tx_link << 16;
		num_regs++;

		/* configure priority/quantum */
		rr_weight = mtu_to_dwrr_weight(pfvf, pfvf->tx_max_pktlen);
		cfg->reg[num_regs] = NIX_AF_TL2X_SCHEDULE(node->schq);
		cfg->regval[num_regs] = TXSCH_TL1_DFLT_RR_PRIO << 24 |
					rr_weight;
		num_regs++;

		/* configure link cfg */
		if (level == pfvf->qos.link_cfg_lvl) {
			cfg->reg[num_regs] = NIX_AF_TL3_TL2X_LINKX_CFG(node->schq, hw->tx_link);
			cfg->regval[num_regs] = BIT_ULL(13) | BIT_ULL(12);
			num_regs++;
		}
	}

txschq_cfg_out:
	cfg->num_regs = num_regs;
}

static int otx2_qos_update_tl1_topology(struct otx2_nic *pfvf, u16 tl2_schq)
{
	struct mbox *mbox = &pfvf->mbox;
	struct otx2_hw *hw = &pfvf->hw;
	struct nix_txschq_config *cfg;
	u16 schq;
	int rc;

	/* vf TL2 is dwrr child */
	if (pfvf->pcifunc & RVU_PFVF_FUNC_MASK)
		return 0;

	/* If the DOWN flag is set TLX nodes are already freed */
	if (pfvf->flags & OTX2_FLAG_INTF_DOWN)
		return -EOPNOTSUPP;

	/* get the parent TL1x queue */
	schq =  hw->txschq_list[NIX_TXSCH_LVL_TL1][0];

	mutex_lock(&mbox->lock);
	cfg = otx2_mbox_alloc_msg_nix_txschq_cfg(&pfvf->mbox);
	if (!cfg)
		return -ENOMEM;

	cfg->lvl = NIX_TXSCH_LVL_TL1;

	/* configure priority anchor */
	cfg->reg[0] = NIX_AF_TL1X_TOPOLOGY(schq);
	cfg->regval[0] = (u64)tl2_schq << 32;
	cfg->regval_mask[0] = ~((u64)0xFF << 32);
	cfg->num_regs = 1;

	rc = otx2_sync_mbox_msg(&pfvf->mbox);
	mutex_unlock(&mbox->lock);

	return rc;
}

static int otx2_qos_txschq_set_parent_topology(struct otx2_nic *pfvf,
					       struct otx2_qos_node *parent)
{
	struct mbox *mbox = &pfvf->mbox;
	struct nix_txschq_config *cfg;
	int rc;

	if (parent->level == NIX_TXSCH_LVL_MDQ ||
	    parent->level == NIX_TXSCH_LVL_TL1)
		return 0;

	mutex_lock(&mbox->lock);

	cfg = otx2_mbox_alloc_msg_nix_txschq_cfg(&pfvf->mbox);
	if (!cfg)
		return -ENOMEM;

	cfg->lvl = parent->level;

	if (parent->level == NIX_TXSCH_LVL_TL4)
		cfg->reg[0] = NIX_AF_TL4X_TOPOLOGY(parent->schq);
	else if (parent->level == NIX_TXSCH_LVL_TL3)
		cfg->reg[0] = NIX_AF_TL3X_TOPOLOGY(parent->schq);
	else if (parent->level == NIX_TXSCH_LVL_TL2)
		cfg->reg[0] = NIX_AF_TL2X_TOPOLOGY(parent->schq);

	cfg->regval[0] = (u64)parent->prio_anchor << 32;

	cfg->regval[0] |= ((parent->act_dwrr_prio != OTX2_QOS_DEFAULT_PRIO) ?
			    parent->act_dwrr_prio : 0)  << 1;
	cfg->num_regs++;

	rc = otx2_sync_mbox_msg(&pfvf->mbox);

	mutex_unlock(&mbox->lock);

	return rc;
}

static void otx2_qos_free_hw_node_schq(struct otx2_nic *pfvf,
				       struct otx2_qos_node *parent)
{
	struct otx2_qos_node *node, *tmp;

	list_for_each_entry_safe(node, tmp, &parent->child_schq_list, list)
		otx2_txschq_free_one(pfvf, node->level, node->schq);
}

static void otx2_qos_free_hw_node_static(struct otx2_nic *pfvf,
					 struct otx2_qos_node *parent)
{
	struct otx2_qos_node *node, *tmp;

	list_for_each_entry_safe(node, tmp, &parent->child_list, list) {
		otx2_qos_free_hw_node_static(pfvf, node);
		otx2_txschq_free_one(pfvf, node->level, node->schq);
		otx2_qos_free_hw_node_schq(pfvf, node);
	}
}

static void otx2_qos_free_hw_node_dwrr(struct otx2_nic *pfvf,
				       struct otx2_qos_node *parent)
{
	struct otx2_qos_node *node, *tmp;

	list_for_each_entry_safe(node, tmp, &parent->child_dwrr_list, list) {
		otx2_qos_free_hw_node_dwrr(pfvf, node);
		otx2_txschq_free_one(pfvf, node->level, node->schq);
		otx2_qos_free_hw_node_schq(pfvf, node);
	}
}

static void otx2_qos_free_hw_cfg(struct otx2_nic *pfvf,
				 struct otx2_qos_node *node)
{
	mutex_lock(&pfvf->qos.qos_lock);

	/* free child node hw mappings */
	otx2_qos_free_hw_node_static(pfvf, node);
	otx2_qos_free_hw_node_dwrr(pfvf, node);
	otx2_qos_free_hw_node_schq(pfvf, node);

	/* free node hw mappings */
	otx2_txschq_free_one(pfvf, node->level, node->schq);

	mutex_unlock(&pfvf->qos.qos_lock);
}

static void otx2_qos_sw_node_delete(struct otx2_nic *pfvf,
				    struct otx2_qos_node *node)
{
	hash_del(&node->hlist);

	if (node->qid != OTX2_QOS_QID_INNER && node->qid != OTX2_QOS_QID_NONE) {
		__clear_bit(node->qid, pfvf->qos.qos_sq_bmap);
		otx2_qos_update_tx_netdev_queues(pfvf);
	}

	list_del(&node->list);
	kfree(node);
}

static void otx2_qos_free_sw_node_schq(struct otx2_nic *pfvf,
				       struct otx2_qos_node *parent)
{
	struct otx2_qos_node *node, *tmp;

	list_for_each_entry_safe(node, tmp, &parent->child_schq_list, list) {
		list_del(&node->list);
		kfree(node);
	}
}

static void otx2_qos_free_sw_node_static(struct otx2_nic *pfvf,
					 struct otx2_qos_node *parent)
{
	struct otx2_qos_node *node, *tmp;

	list_for_each_entry_safe(node, tmp, &parent->child_list, list) {
		otx2_qos_free_sw_node_static(pfvf, node);
		otx2_qos_free_sw_node_schq(pfvf, node);
		otx2_qos_sw_node_delete(pfvf, node);
	}
}

static void otx2_qos_free_sw_node_dwrr(struct otx2_nic *pfvf,
				       struct otx2_qos_node *parent)
{
	struct otx2_qos_node *node, *tmp;

	list_for_each_entry_safe(node, tmp, &parent->child_dwrr_list, list) {
		otx2_qos_free_sw_node_dwrr(pfvf, node);
		otx2_qos_free_sw_node_schq(pfvf, node);
		otx2_qos_sw_node_delete(pfvf, node);
	}
}

static void otx2_qos_free_sw_node(struct otx2_nic *pfvf,
				  struct otx2_qos_node *node)
{
	mutex_lock(&pfvf->qos.qos_lock);

	otx2_qos_free_sw_node_static(pfvf, node);
	otx2_qos_free_sw_node_dwrr(pfvf, node);
	otx2_qos_free_sw_node_schq(pfvf, node);
	otx2_qos_sw_node_delete(pfvf, node);

	mutex_unlock(&pfvf->qos.qos_lock);
}

static void otx2_qos_destroy_node(struct otx2_nic *pfvf,
				  struct otx2_qos_node *node)
{
	otx2_qos_free_hw_cfg(pfvf, node);
	otx2_qos_free_sw_node(pfvf, node);
}

static void otx2_qos_fill_cfg_schq(struct otx2_qos_node *parent,
				   struct otx2_qos_cfg *cfg)
{
	struct otx2_qos_node *node;

	list_for_each_entry(node, &parent->child_schq_list, list)
		cfg->schq[node->level]++;
}

static void otx2_qos_fill_cfg_static(struct otx2_qos_node *parent,
				     struct otx2_qos_cfg *cfg)
{
	struct otx2_qos_node *node;

	list_for_each_entry(node, &parent->child_list, list) {
		otx2_qos_fill_cfg_static(node, cfg);
		cfg->schq_contig[node->level]++;
		otx2_qos_fill_cfg_schq(node, cfg);
	}
}

static void otx2_qos_fill_cfg_dwrr(struct otx2_qos_node *parent,
				   struct otx2_qos_cfg *cfg)
{
	struct otx2_qos_node *node;

	list_for_each_entry(node, &parent->child_dwrr_list, list) {
		otx2_qos_fill_cfg_dwrr(node, cfg);
		cfg->schq[node->level]++;
		otx2_qos_fill_cfg_schq(node, cfg);
	}
}

static void otx2_qos_prepare_txschq_cfg(struct otx2_nic *pfvf,
					struct otx2_qos_node *parent,
					struct otx2_qos_cfg *cfg,
					bool static_cfg)
{
	mutex_lock(&pfvf->qos.qos_lock);

	if (static_cfg)
		otx2_qos_fill_cfg_static(parent, cfg);
	else
		otx2_qos_fill_cfg_dwrr(parent, cfg);

	mutex_unlock(&pfvf->qos.qos_lock);
}

static void otx2_qos_read_txschq_cfg_schq(struct otx2_qos_node *parent,
					  struct otx2_qos_cfg *cfg)
{
	struct otx2_qos_node *node;
	int cnt;

	list_for_each_entry(node, &parent->child_schq_list, list) {
		cnt = cfg->dwrr_node_pos[node->level];
		cfg->schq_list[node->level][cnt] = node->schq;
		cfg->schq[node->level]++;
		cfg->dwrr_node_pos[node->level]++;
	}
}

static void otx2_qos_read_txschq_cfg_static(struct otx2_qos_node *parent,
					    struct otx2_qos_cfg *cfg)
{
	struct otx2_qos_node *node;
	int cnt;

	list_for_each_entry(node, &parent->child_list, list) {
		otx2_qos_read_txschq_cfg_static(node, cfg);
		cnt = cfg->static_node_pos[node->level];
		cfg->schq_contig_list[node->level][cnt] = node->schq;
		cfg->schq_contig[node->level]++;
		cfg->static_node_pos[node->level]++;
		otx2_qos_read_txschq_cfg_schq(node, cfg);
	}
}

static void otx2_qos_read_txschq_cfg_dwrr(struct otx2_qos_node *parent,
					  struct otx2_qos_cfg *cfg)
{
	struct otx2_qos_node *node;
	int cnt;

	list_for_each_entry(node, &parent->child_dwrr_list, list) {
		otx2_qos_read_txschq_cfg_dwrr(node, cfg);
		cnt = cfg->dwrr_node_pos[node->level];
		cfg->schq_list[node->level][cnt] = node->schq;
		cfg->schq[node->level]++;
		cfg->dwrr_node_pos[node->level]++;
		otx2_qos_read_txschq_cfg_schq(node, cfg);
	}
}

static void otx2_qos_read_txschq_cfg(struct otx2_nic *pfvf,
				     struct otx2_qos_node *node,
				     struct otx2_qos_cfg *cfg,
				     bool static_cfg)
{
	mutex_lock(&pfvf->qos.qos_lock);

	if (static_cfg)
		otx2_qos_read_txschq_cfg_static(node, cfg);
	else
		otx2_qos_read_txschq_cfg_dwrr(node, cfg);

	mutex_unlock(&pfvf->qos.qos_lock);
}

static struct otx2_qos_node *
otx2_qos_alloc_root(struct otx2_nic *pfvf)
{
	struct otx2_qos_node *node;

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return ERR_PTR(-ENOMEM);

	node->parent = NULL;
	node->level = NIX_TXSCH_LVL_TL2;
	node->qid = OTX2_QOS_QID_INNER;
	node->classid = OTX2_QOS_ROOT_CLASSID;
	node->child_dwrr_prio = OTX2_QOS_DEFAULT_PRIO;
	node->act_dwrr_prio = OTX2_QOS_DEFAULT_PRIO;

	hash_add(pfvf->qos.qos_hlist, &node->hlist, node->classid);
	list_add_tail(&node->list, &pfvf->qos.qos_tree);
	INIT_LIST_HEAD(&node->child_list);
	INIT_LIST_HEAD(&node->child_dwrr_list);
	INIT_LIST_HEAD(&node->child_schq_list);

	return node;
}

static int otx2_qos_add_static_node(struct otx2_qos_node *parent,
				    struct otx2_qos_node *node)
{
	struct list_head *head = &parent->child_list;
	struct otx2_qos_node *tmp_node;
	struct list_head *tmp;

	for (tmp = head->next; tmp != head; tmp = tmp->next) {
		tmp_node = list_entry(tmp, struct otx2_qos_node, list);
		if (tmp_node->prio == node->prio &&
		    otx2_qos_is_quantum_static(tmp_node->quantum))
			return -EEXIST;
		if (tmp_node->prio > node->prio) {
			list_add_tail(&node->list, tmp);
			return 0;
		}
	}

	list_add_tail(&node->list, head);
	return 0;
}

static int otx2_qos_alloc_txschq_node(struct otx2_nic *pfvf,
				      struct otx2_qos_node *node)
{
	struct otx2_qos_node *txschq_node, *parent, *tmp;
	int lvl;

	parent = node;
	for (lvl = node->level - 1; lvl >= NIX_TXSCH_LVL_MDQ; lvl--) {
		txschq_node = kzalloc(sizeof(*txschq_node), GFP_KERNEL);
		if (!txschq_node)
			goto err_out;

		txschq_node->parent = parent;
		txschq_node->level = lvl;
		txschq_node->classid = OTX2_QOS_CLASS_NONE;
		txschq_node->qid = OTX2_QOS_QID_NONE;
		txschq_node->rate = 0;
		txschq_node->ceil = 0;
		txschq_node->prio = 0;
		txschq_node->quantum = 0;
		txschq_node->is_static = true;
		txschq_node->child_dwrr_prio = OTX2_QOS_DEFAULT_PRIO;
		txschq_node->act_dwrr_prio = OTX2_QOS_DEFAULT_PRIO;

		mutex_lock(&pfvf->qos.qos_lock);
		list_add_tail(&txschq_node->list, &node->child_schq_list);
		mutex_unlock(&pfvf->qos.qos_lock);

		INIT_LIST_HEAD(&txschq_node->child_list);
		INIT_LIST_HEAD(&txschq_node->child_dwrr_list);
		INIT_LIST_HEAD(&txschq_node->child_schq_list);
		parent = txschq_node;
	}

	return 0;

err_out:
	list_for_each_entry_safe(txschq_node, tmp, &node->child_schq_list,
				 list) {
		list_del(&txschq_node->list);
		kfree(txschq_node);
	}
	return -ENOMEM;
}

static struct otx2_qos_node *
otx2_qos_sw_create_leaf_node(struct otx2_nic *pfvf,
			     struct otx2_qos_node *parent,
			     u16 classid, u32 prio, u64 rate, u64 ceil,
			     u32 quantum, u16 qid)
{
	struct otx2_qos_node *node;
	int err;

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return ERR_PTR(-ENOMEM);

	node->parent = parent;
	node->level = parent->level - 1;
	node->classid = classid;
	node->qid = qid;
	node->rate = otx2_qos_convert_rate(rate);
	node->ceil = otx2_qos_convert_rate(ceil);
	node->prio = prio;
	node->quantum = quantum;
	node->is_static = true;
	node->child_dwrr_prio = OTX2_QOS_DEFAULT_PRIO;
	node->act_dwrr_prio = OTX2_QOS_DEFAULT_PRIO;

	__set_bit(qid, pfvf->qos.qos_sq_bmap);

	hash_add(pfvf->qos.qos_hlist, &node->hlist, classid);

	mutex_lock(&pfvf->qos.qos_lock);
	if (node->is_static) {
		err = otx2_qos_add_static_node(parent, node);
		if (err) {
			mutex_unlock(&pfvf->qos.qos_lock);
			return ERR_PTR(err);
		}
	} else {
		list_add_tail(&node->list, &parent->child_dwrr_list);
	}
	mutex_unlock(&pfvf->qos.qos_lock);

	INIT_LIST_HEAD(&node->child_list);
	INIT_LIST_HEAD(&node->child_dwrr_list);
	INIT_LIST_HEAD(&node->child_schq_list);

	err = otx2_qos_alloc_txschq_node(pfvf, node);
	if (err) {
		otx2_qos_sw_node_delete(pfvf, node);
		return ERR_PTR(-ENOMEM);
	}

	return node;
}

static struct otx2_qos_node *
otx2_sw_node_find(struct otx2_nic *pfvf, u32 classid)
{
	struct otx2_qos_node *node = NULL;

	hash_for_each_possible(pfvf->qos.qos_hlist, node, hlist, classid) {
		if (node->classid == classid)
			break;
	}

	return node;
}

int otx2_get_txq_by_classid(struct otx2_nic *pfvf, u16 classid)
{
	struct otx2_qos_node *node;
	u16 qid;
	int res;

	node = otx2_sw_node_find(pfvf, classid);
	if (!node) {
		res = -ENOENT;
		goto out;
	}
	qid = READ_ONCE(node->qid);
	if (qid == OTX2_QOS_QID_INNER) {
		res = -EINVAL;
		goto out;
	}
	res = pfvf->hw.tx_queues + qid;
out:
	return res;
}

static int
otx2_qos_txschq_config(struct otx2_nic *pfvf, struct otx2_qos_node *node)
{
	struct mbox *mbox = &pfvf->mbox;
	struct nix_txschq_config *req;
	int rc;

	mutex_lock(&mbox->lock);

	req = otx2_mbox_alloc_msg_nix_txschq_cfg(&pfvf->mbox);
	if (!req)
		return -ENOMEM;

	req->lvl = node->level;
	__otx2_qos_txschq_cfg(pfvf, node, req);

	rc = otx2_sync_mbox_msg(&pfvf->mbox);

	mutex_unlock(&mbox->lock);

	return rc;
}

static int otx2_qos_txschq_alloc(struct otx2_nic *pfvf,
				 struct otx2_qos_cfg *cfg)
{
	struct nix_txsch_alloc_req *req;
	struct nix_txsch_alloc_rsp *rsp;
	struct mbox *mbox = &pfvf->mbox;
	int lvl, rc, schq;

	mutex_lock(&mbox->lock);
	req = otx2_mbox_alloc_msg_nix_txsch_alloc(&pfvf->mbox);
	if (!req)
		return -ENOMEM;

	for (lvl = 0; lvl < NIX_TXSCH_LVL_CNT; lvl++) {
		req->schq[lvl] = cfg->schq[lvl];
		req->schq_contig[lvl] = cfg->schq_contig[lvl];
	}

	rc = otx2_sync_mbox_msg(&pfvf->mbox);
	if (rc)
		return rc;

	rsp = (struct nix_txsch_alloc_rsp *)
	      otx2_mbox_get_rsp(&pfvf->mbox.mbox, 0, &req->hdr);

	for (lvl = 0; lvl < NIX_TXSCH_LVL_CNT; lvl++) {
		for (schq = 0; schq < rsp->schq_contig[lvl]; schq++) {
			cfg->schq_contig_list[lvl][schq] =
				rsp->schq_contig_list[lvl][schq];
		}
	}

	for (lvl = 0; lvl < NIX_TXSCH_LVL_CNT; lvl++) {
		for (schq = 0; schq < rsp->schq[lvl]; schq++) {
			cfg->schq_list[lvl][schq] =
				rsp->schq_list[lvl][schq];
		}
	}

	pfvf->qos.link_cfg_lvl = rsp->link_cfg_lvl;

	mutex_unlock(&mbox->lock);

	return rc;
}

static void otx2_qos_txschq_fill_cfg_schq(struct otx2_nic *pfvf,
					  struct otx2_qos_node *node,
					  struct otx2_qos_cfg *cfg)
{
	struct otx2_qos_node *tmp;
	int cnt;

	list_for_each_entry(tmp, &node->child_schq_list, list) {
		cnt = cfg->dwrr_node_pos[tmp->level];
		tmp->schq = cfg->schq_list[tmp->level][cnt];
		cfg->dwrr_node_pos[tmp->level]++;
	}
}

static void otx2_qos_txschq_fill_cfg_static(struct otx2_nic *pfvf,
					    struct otx2_qos_node *node,
					    struct otx2_qos_cfg *cfg)
{
	struct otx2_qos_node *tmp;
	int cnt;

	node->act_dwrr_prio = OTX2_QOS_DEFAULT_PRIO;
	list_for_each_entry(tmp, &node->child_list, list) {
		otx2_qos_txschq_fill_cfg_static(pfvf, tmp, cfg);
		cnt = cfg->static_node_pos[tmp->level];
		tmp->schq = cfg->schq_contig_list[tmp->level][cnt];
		if (cnt == 0)
			node->prio_anchor = tmp->schq;
		if (tmp->classid != OTX2_QOS_CLASS_NONE &&
		    !otx2_qos_is_quantum_static(tmp->quantum) &&
		    node->act_dwrr_prio == OTX2_QOS_DEFAULT_PRIO)
			node->act_dwrr_prio = tmp->schq - node->prio_anchor;
		cfg->static_node_pos[tmp->level]++;
		otx2_qos_txschq_fill_cfg_schq(pfvf, tmp, cfg);
	}
}

static void otx2_qos_txschq_fill_cfg_dwrr(struct otx2_nic *pfvf,
					  struct otx2_qos_node *node,
					  struct otx2_qos_cfg *cfg)
{
	struct otx2_qos_node *tmp;
	int cnt;

	list_for_each_entry(tmp, &node->child_dwrr_list, list) {
		otx2_qos_txschq_fill_cfg_dwrr(pfvf, tmp, cfg);
		cnt = cfg->dwrr_node_pos[tmp->level];
		tmp->schq = cfg->schq_list[tmp->level][cnt];
		cfg->dwrr_node_pos[tmp->level]++;
		otx2_qos_txschq_fill_cfg_schq(pfvf, tmp, cfg);
	}
}

static void otx2_qos_txschq_fill_cfg(struct otx2_nic *pfvf,
				     struct otx2_qos_node *node,
				     struct otx2_qos_cfg *cfg,
				     bool static_cfg)
{
	mutex_lock(&pfvf->qos.qos_lock);
	if (static_cfg)
		otx2_qos_txschq_fill_cfg_static(pfvf, node, cfg);
	else
		otx2_qos_txschq_fill_cfg_dwrr(pfvf, node, cfg);

	otx2_qos_txschq_fill_cfg_schq(pfvf, node, cfg);
	mutex_unlock(&pfvf->qos.qos_lock);
}

static int otx2_qos_txschq_push_cfg_schq(struct otx2_nic *pfvf,
					 struct otx2_qos_node *node,
					 struct otx2_qos_cfg *cfg)
{
	struct otx2_qos_node *tmp;
	int ret = 0;

	list_for_each_entry(tmp, &node->child_schq_list, list) {
		ret = otx2_qos_txschq_config(pfvf, tmp);
		if (ret)
			return -EIO;
		ret = otx2_qos_txschq_set_parent_topology(pfvf, tmp->parent);
		if (ret)
			return -EIO;
	}

	return 0;
}

static int otx2_qos_txschq_push_cfg_static(struct otx2_nic *pfvf,
					   struct otx2_qos_node *node,
					   struct otx2_qos_cfg *cfg)
{
	struct otx2_qos_node *tmp;
	int ret = 0;

	list_for_each_entry(tmp, &node->child_list, list) {
		ret = otx2_qos_txschq_push_cfg_static(pfvf, tmp, cfg);
		if (ret)
			return -EIO;
		ret = otx2_qos_txschq_config(pfvf, tmp);
		if (ret)
			return -EIO;
		ret = otx2_qos_txschq_push_cfg_schq(pfvf, tmp, cfg);
		if (ret)
			return -EIO;
	}

	ret = otx2_qos_txschq_set_parent_topology(pfvf, node);
	if (ret)
		return -EIO;

	return 0;
}

static int otx2_qos_txschq_push_cfg_dwrr(struct otx2_nic *pfvf,
					 struct otx2_qos_node *node,
					 struct otx2_qos_cfg *cfg)
{
	struct otx2_qos_node *tmp;
	int ret = 0;

	list_for_each_entry(tmp, &node->child_dwrr_list, list) {
		ret = otx2_qos_txschq_push_cfg_dwrr(pfvf, tmp, cfg);
		if (ret)
			return -EIO;
		ret = otx2_qos_txschq_config(pfvf, tmp);
		if (ret)
			return -EIO;
		ret = otx2_qos_txschq_push_cfg_schq(pfvf, tmp, cfg);
		if (ret)
			return -EIO;
	}

	ret = otx2_qos_txschq_set_parent_topology(pfvf, node);
	if (ret)
		return -EIO;

	return 0;
}

static int otx2_qos_txschq_push_cfg(struct otx2_nic *pfvf,
				    struct otx2_qos_node *node,
				    struct otx2_qos_cfg *cfg,
				    bool static_cfg)
{
	int ret = 0;

	mutex_lock(&pfvf->qos.qos_lock);
	if (static_cfg) {
		ret = otx2_qos_txschq_push_cfg_static(pfvf, node, cfg);
		if (ret)
			goto out;
	} else {
		ret = otx2_qos_txschq_push_cfg_dwrr(pfvf, node, cfg);
		if (ret)
			goto out;
	}
	ret = otx2_qos_txschq_push_cfg_schq(pfvf, node, cfg);
out:
	mutex_unlock(&pfvf->qos.qos_lock);
	return ret;
}

static int otx2_qos_txschq_update_config(struct otx2_nic *pfvf,
					 struct otx2_qos_node *node,
					 struct otx2_qos_cfg *cfg,
					 bool static_cfg)
{
	int ret = 0;

	otx2_qos_txschq_fill_cfg(pfvf, node, cfg, static_cfg);
	ret = otx2_qos_txschq_push_cfg(pfvf, node, cfg, static_cfg);

	return ret;
}

static int otx2_qos_txschq_update_root_cfg(struct otx2_nic *pfvf,
					   struct otx2_qos_node *root,
					   struct otx2_qos_cfg *cfg)
{
	int ret = 0;

	root->schq = cfg->schq_list[root->level][0];
	ret = otx2_qos_txschq_config(pfvf, root);

	return ret;
}

static void otx2_qos_free_cfg(struct otx2_nic *pfvf, struct otx2_qos_cfg *cfg,
			      bool static_cfg)
{
	int lvl, idx, schq;

	if (static_cfg) {
		/* stop static txschq queue entries */
		for (lvl = 0; lvl < NIX_TXSCH_LVL_CNT; lvl++) {
			for (idx = 0; idx < cfg->schq_contig[lvl]; idx++) {
				schq = cfg->schq_contig_list[lvl][idx];
				otx2_txschq_free_one(pfvf, lvl, schq);
			}
		}
	} else {
		/* free dwrr txschq queue entries */
		for (lvl = 0; lvl < NIX_TXSCH_LVL_CNT; lvl++) {
			for (idx = 0; idx < cfg->schq[lvl]; idx++) {
				schq = cfg->schq_list[lvl][idx];
				otx2_txschq_free_one(pfvf, lvl, schq);
			}
		}
	}
}

static void otx2_qos_enadis_sq(struct otx2_nic *pfvf,
			       struct otx2_qos_node *node,
			       u16 qid)
{
	if (pfvf->qos.qid_to_sqmap[qid] != OTX2_QOS_INVALID_SQ)
		otx2_qos_disable_sq(pfvf, qid);

	pfvf->qos.qid_to_sqmap[qid] = node->schq;
	otx2_qos_enable_sq(pfvf, qid);
}

static void otx2_qos_update_smq_schq(struct otx2_nic *pfvf,
				     struct otx2_qos_node *node,
				     bool action)
{
	struct otx2_qos_node *tmp;

	if (node->qid == OTX2_QOS_QID_INNER)
		return;

	list_for_each_entry(tmp, &node->child_schq_list, list) {
		if (tmp->level == NIX_TXSCH_LVL_MDQ) {
			if (action == QOS_SMQ_FLUSH)
				otx2_smq_flush(pfvf, tmp->schq);
			else
				otx2_qos_enadis_sq(pfvf, tmp, node->qid);
		}

	}
}

static void otx2_qos_update_smq_static(struct otx2_nic *pfvf,
				       struct otx2_qos_node *node,
				       bool action)
{
	struct otx2_qos_node *tmp;

	list_for_each_entry(tmp, &node->child_list, list) {
		otx2_qos_update_smq_static(pfvf, tmp, action);
		if (tmp->qid == OTX2_QOS_QID_INNER)
			continue;
		if (tmp->level == NIX_TXSCH_LVL_MDQ) {
			if (action == QOS_SMQ_FLUSH)
				otx2_smq_flush(pfvf, tmp->schq);
			else
				otx2_qos_enadis_sq(pfvf, tmp, tmp->qid);
		} else {
			otx2_qos_update_smq_schq(pfvf, tmp, action);
		}
	}
}

static void otx2_qos_update_smq_dwrr(struct otx2_nic *pfvf,
				     struct otx2_qos_node *node,
				     bool action)
{
	struct otx2_qos_node *tmp;

	list_for_each_entry(tmp, &node->child_dwrr_list, list) {
		if (tmp->qid == OTX2_QOS_QID_INNER)
			continue;
		if (tmp->level == NIX_TXSCH_LVL_MDQ) {
			if (action == QOS_SMQ_FLUSH)
				otx2_smq_flush(pfvf, tmp->schq);
			else
				otx2_qos_enadis_sq(pfvf, tmp, tmp->qid);
		} else {
			otx2_qos_update_smq_schq(pfvf, tmp, action);
		}

	}
}

static int otx2_qos_update_smq(struct otx2_nic *pfvf,
			       struct otx2_qos_node *node,
			       bool static_cfg, bool action)
{
	mutex_lock(&pfvf->qos.qos_lock);
	if (static_cfg)
		otx2_qos_update_smq_static(pfvf, node, action);
	else
		otx2_qos_update_smq_dwrr(pfvf, node, action);

	otx2_qos_update_smq_schq(pfvf, node, action);
	mutex_unlock(&pfvf->qos.qos_lock);

	return 0;
}

static int otx2_qos_push_txschq_cfg(struct otx2_nic *pfvf,
				    struct otx2_qos_node *node,
				    struct otx2_qos_cfg *cfg, bool static_cfg)
{
	int ret = 0;

	ret = otx2_qos_txschq_alloc(pfvf, cfg);
	if (ret)
		return -ENOSPC;

	if (!(pfvf->netdev->flags & IFF_UP)) {
		otx2_qos_txschq_fill_cfg(pfvf, node, cfg, static_cfg);
		return 0;
	}

	ret = otx2_qos_txschq_update_config(pfvf, node, cfg, static_cfg);
	if (ret) {
		otx2_qos_free_cfg(pfvf, cfg, static_cfg);
		return -EIO;
	}

	ret = otx2_qos_update_smq(pfvf, node, static_cfg, QOS_CFG_SQ);
	if (ret) {
		otx2_qos_free_cfg(pfvf, cfg, static_cfg);
		return -EIO;
	}

	return 0;
}

static int otx2_qos_update_tree(struct otx2_nic *pfvf,
				struct otx2_qos_node *node,
				struct otx2_qos_cfg *cfg)
{
	int ret = 0;

	if (node->is_static) {
		otx2_qos_prepare_txschq_cfg(pfvf, node->parent, cfg, true);
		ret = otx2_qos_push_txschq_cfg(pfvf, node->parent, cfg, true);
	} else {
		otx2_qos_prepare_txschq_cfg(pfvf, node->parent, cfg, false);
		ret = otx2_qos_push_txschq_cfg(pfvf, node->parent, cfg, false);
	}

	return ret;
}

static int otx2_qos_root_add(struct otx2_nic *pfvf, u16 htb_maj_id, u16 htb_defcls,
			     struct netlink_ext_ack *extack)
{
	struct otx2_qos_cfg *new_cfg;
	struct otx2_qos_node *root;
	int err;

	netdev_dbg(pfvf->netdev,
		   "TC_HTB_CREATE: handle=0x%x defcls=0x%x\n",
		   htb_maj_id, htb_defcls);

	INIT_LIST_HEAD(&pfvf->qos.qos_tree);
	mutex_init(&pfvf->qos.qos_lock);

	root = otx2_qos_alloc_root(pfvf);
	if (IS_ERR(root)) {
		mutex_destroy(&pfvf->qos.qos_lock);
		err = PTR_ERR(root);
		return err;
	}

	/* allocate txschq queue */
	new_cfg = kzalloc(sizeof(*new_cfg), GFP_KERNEL);
	if (!new_cfg) {
		NL_SET_ERR_MSG_MOD(extack, "Memory allocation error");
		mutex_destroy(&pfvf->qos.qos_lock);
		return -ENOMEM;
	}
	/* allocate one TL2 node for htb root */
	new_cfg->schq[root->level] = 1;
	err = otx2_qos_txschq_alloc(pfvf, new_cfg);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack, "Error allocating txschq");
		goto free_root_node;
	}

	if (!(pfvf->netdev->flags & IFF_UP)) {
		root->schq = new_cfg->schq_list[root->level][0];
		WRITE_ONCE(pfvf->qos.defcls, htb_defcls);
		smp_store_release(&pfvf->qos.maj_id, htb_maj_id); /* barrier */
		return 0;
	}

	/* update the txschq configuration in hw */
	err = otx2_qos_txschq_update_root_cfg(pfvf, root, new_cfg);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Error updating txschq configuration");
		goto txschq_free;
	}

	err = otx2_qos_update_tl1_topology(pfvf, root->schq);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Error updating TL1 topology");
		goto txschq_free;
	}

	WRITE_ONCE(pfvf->qos.defcls, htb_defcls);
	smp_store_release(&pfvf->qos.maj_id, htb_maj_id); /* barrier */

	return 0;

txschq_free:
	otx2_qos_free_cfg(pfvf, new_cfg, false);
free_root_node:
	kfree(new_cfg);
	otx2_qos_sw_node_delete(pfvf, root);
	mutex_destroy(&pfvf->qos.qos_lock);
	return err;
}

static int otx2_qos_root_destroy(struct otx2_nic *pfvf)
{
	struct otx2_qos_node *root;

	netdev_dbg(pfvf->netdev, "TC_HTB_DESTROY\n");

	/* find root node */
	root = otx2_sw_node_find(pfvf, OTX2_QOS_ROOT_CLASSID);
	if (!root)
		return -ENOENT;

	/* reset TL1 topology priority anchor */
	otx2_qos_update_tl1_topology(pfvf, 0);

	/* free the hw mappings */
	otx2_qos_destroy_node(pfvf, root);
	mutex_destroy(&pfvf->qos.qos_lock);

	return 0;
}

static int otx2_qos_validate_dwrr_cfg(struct otx2_qos_node *parent,
				      struct netlink_ext_ack *extack,
				      u64 prio)
{
	if (parent->child_dwrr_prio == OTX2_QOS_DEFAULT_PRIO) {
		parent->child_dwrr_prio = prio;
	} else if (prio != parent->child_dwrr_prio) {
		NL_SET_ERR_MSG_MOD(extack, "Only one DWRR group is allowed");
		return -EOPNOTSUPP;
	}

	parent->child_dwrr_cnt++;
	return 0;
}

static int otx2_qos_validate_maxnodes_per_tl(struct otx2_qos_node *parent,
					     struct netlink_ext_ack *extack,
					     struct otx2_nic *pfvf,
					     u64 prio, u64 quantum)
{
	int static_cnt = 0, dwrr_cnt = 0, highest_prio = 0;
	struct otx2_qos_node *node;

	mutex_lock(&pfvf->qos.qos_lock);
	list_for_each_entry(node, &parent->child_list, list) {
		if (prio == node->prio &&
		    otx2_qos_is_quantum_static(node->quantum)) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Static priority child with same priority exists");
			mutex_unlock(&pfvf->qos.qos_lock);
			return -EEXIST;
		}
		if (prio == parent->child_dwrr_prio &&
		    otx2_qos_is_quantum_static(quantum)) {
			NL_SET_ERR_MSG_MOD(extack, "DWRR child group with same priority exists");
			mutex_unlock(&pfvf->qos.qos_lock);
			return -EEXIST;
		}

		if (node->prio > highest_prio)
			highest_prio = node->prio;

		if (otx2_qos_is_quantum_static(node->quantum))
			static_cnt++;
		else
			dwrr_cnt++;
	}
	mutex_unlock(&pfvf->qos.qos_lock);

	/* DWRR group has highest prio allow user to add nodes*/
	if (!otx2_qos_is_quantum_static(quantum) && prio == highest_prio)
		return 0;

	if ((static_cnt + dwrr_cnt) >= 8) {
		NL_SET_ERR_MSG_MOD(extack, "Invalid Qos configuration, exceeded node count\n");
		return  -EOPNOTSUPP;
	}

	return 0;
}

static int otx2_qos_leaf_alloc_queue(struct otx2_nic *pfvf, u16 classid,
				     u32 parent_classid, u64 rate, u64 ceil,
				     u64 prio, u64 quantum,
				     struct netlink_ext_ack *extack)
{
	struct otx2_qos_cfg *old_cfg, *new_cfg;
	struct otx2_qos_node *node, *parent;
	int qid, ret, err;
	bool static_cfg;

	netdev_dbg(pfvf->netdev,
		   "TC_HTB_LEAF_ALLOC_QUEUE: classid=0x%x parent_classid=0x%x rate=%lld ceil=%lld prio=%lld quantum=%lld\n",
		   classid, parent_classid, rate, ceil, prio, quantum);

	if (prio > OTX2_QOS_MAX_PRIO) {
		NL_SET_ERR_MSG_MOD(extack, "Valid priority range 0 to 7");
		ret = -EOPNOTSUPP;
		goto out;
	}

	if (!quantum || quantum > INT_MAX) {
		NL_SET_ERR_MSG_MOD(extack, "Invalid quantum, range 1 - 2147483647 bytes");
		ret = -EOPNOTSUPP;
		goto out;
	}

	/* get parent node */
	parent = otx2_sw_node_find(pfvf, parent_classid);
	if (!parent) {
		NL_SET_ERR_MSG_MOD(extack, "parent node not found");
		ret = -ENOENT;
		goto out;
	}
	if (parent->level == NIX_TXSCH_LVL_MDQ) {
		NL_SET_ERR_MSG_MOD(extack, "HTB qos max levels reached");
		ret = -EOPNOTSUPP;
		goto out;
	}

	ret = otx2_qos_validate_maxnodes_per_tl(parent, extack, pfvf, prio, quantum);
	if (ret)
		goto out;

	if (!otx2_qos_is_quantum_static(quantum)) {
		ret = otx2_qos_validate_dwrr_cfg(parent, extack, prio);
		if (ret)
			goto out;
	}

	/* read current txschq configuration */
	static_cfg = true;
	old_cfg = kzalloc(sizeof(*old_cfg), GFP_KERNEL);
	if (!old_cfg) {
		NL_SET_ERR_MSG_MOD(extack, "Memory allocation error");
		ret = -ENOMEM;
		goto out;
	}
	otx2_qos_read_txschq_cfg(pfvf, parent, old_cfg, static_cfg);

	/* allocate a new sq */
	qid = otx2_qos_get_qid(pfvf);
	if (qid < 0) {
		NL_SET_ERR_MSG_MOD(extack, "Reached max supported QOS SQ's");
		ret = -ENOMEM;
		goto free_old_cfg;
	}

	/* Actual SQ mapping will be updated after SMQ alloc */
	pfvf->qos.qid_to_sqmap[qid] = OTX2_QOS_INVALID_SQ;

	/* allocate and initialize a new child node */
	node = otx2_qos_sw_create_leaf_node(pfvf, parent, classid, prio, rate,
					    ceil, quantum, qid);
	if (IS_ERR(node)) {
		NL_SET_ERR_MSG_MOD(extack, "Unable to allocate leaf node");
		ret = PTR_ERR(node);
		goto free_old_cfg;
	}

	/* push new txschq config to hw */
	new_cfg = kzalloc(sizeof(*new_cfg), GFP_KERNEL);
	if (!new_cfg) {
		NL_SET_ERR_MSG_MOD(extack, "Memory allocation error");
		ret = -ENOMEM;
		goto free_node;
	}
	ret = otx2_qos_update_tree(pfvf, node, new_cfg);
	if (ret) {
		NL_SET_ERR_MSG_MOD(extack, "HTB HW configuration error");
		kfree(new_cfg);
		otx2_qos_sw_node_delete(pfvf, node);
		/* restore the old qos tree */
		err = otx2_qos_txschq_update_config(pfvf, parent, old_cfg,
						    parent->is_static ? true : false);
		if (err) {
			netdev_err(pfvf->netdev,
				   "Failed to restore txcshq configuration");
			goto free_old_cfg;
		}
		err = otx2_qos_update_smq(pfvf, parent,
					  parent->is_static ? true : false,
					  QOS_CFG_SQ);
		if (err)
			netdev_err(pfvf->netdev,
				   "Failed to restore smq configuration");
		goto free_old_cfg;
	}

	/* update tx_real_queues */
	otx2_qos_update_tx_netdev_queues(pfvf);

	/* free new txschq config */
	kfree(new_cfg);

	/* free old txschq config */
	otx2_qos_free_cfg(pfvf, old_cfg, static_cfg);
	kfree(old_cfg);

	return pfvf->hw.tx_queues + qid;

free_node:
	otx2_qos_sw_node_delete(pfvf, node);
free_old_cfg:
	kfree(old_cfg);
out:
	return ret;
}

static int otx2_qos_leaf_to_inner(struct otx2_nic *pfvf, u16 classid,
				  u16 child_classid, u64 rate, u64 ceil, u64 prio,
				  u64 quantum, struct netlink_ext_ack *extack)
{
	struct otx2_qos_cfg *old_cfg, *new_cfg;
	struct otx2_qos_node *node, *child;
	bool static_cfg;
	int ret, err;
	u16 qid;

	netdev_dbg(pfvf->netdev,
		   "TC_HTB_LEAF_TO_INNER classid %04x, child %04x, rate %llu, ceil %llu\n",
		   classid, child_classid, rate, ceil);

	if (prio > OTX2_QOS_MAX_PRIO) {
		NL_SET_ERR_MSG_MOD(extack, "Valid priority range 0 to 7");
		ret = -EOPNOTSUPP;
		goto out;
	}

	if (!quantum || quantum > INT_MAX) {
		NL_SET_ERR_MSG_MOD(extack, "Invalid quantum, range 1 - 2147483647 bytes");
		ret = -EOPNOTSUPP;
		goto out;
	}

	/* find node related to classid */
	node = otx2_sw_node_find(pfvf, classid);
	if (!node) {
		NL_SET_ERR_MSG_MOD(extack, "HTB node not found");
		ret = -ENOENT;
		goto out;
	}
	/* check max qos txschq level */
	if (node->level == NIX_TXSCH_LVL_MDQ) {
		NL_SET_ERR_MSG_MOD(extack, "HTB qos level not supported");
		ret = -EOPNOTSUPP;
		goto out;
	}

	if (!otx2_qos_is_quantum_static(quantum)) {
		ret = otx2_qos_validate_dwrr_cfg(node, extack, prio);
		if (ret)
			goto out;
	}

	/* store the qid to assign to leaf node */
	qid = node->qid;

	/* read current txschq configuration */
	static_cfg = true;
	old_cfg = kzalloc(sizeof(*old_cfg), GFP_KERNEL);
	if (!old_cfg) {
		NL_SET_ERR_MSG_MOD(extack, "Memory allocation error");
		ret = -ENOMEM;
		goto out;
	}
	otx2_qos_read_txschq_cfg(pfvf, node, old_cfg, static_cfg);

	/* delete the txschq nodes allocated for this node */
	otx2_qos_free_sw_node_schq(pfvf, node);

	/* mark this node as htb inner node */
	node->qid = OTX2_QOS_QID_INNER;

	/* allocate and initialize a new child node */
	child = otx2_qos_sw_create_leaf_node(pfvf, node, child_classid,
					     prio, rate, ceil, quantum,
					     qid);
	if (IS_ERR(child)) {
		NL_SET_ERR_MSG_MOD(extack, "Unable to allocate leaf node");
		ret = PTR_ERR(child);
		goto free_old_cfg;
	}

	/* push new txschq config to hw */
	new_cfg = kzalloc(sizeof(*new_cfg), GFP_KERNEL);
	if (!new_cfg) {
		NL_SET_ERR_MSG_MOD(extack, "Memory allocation error");
		ret = -ENOMEM;
		goto free_node;
	}
	ret = otx2_qos_update_tree(pfvf, child, new_cfg);
	if (ret) {
		NL_SET_ERR_MSG_MOD(extack, "HTB HW configuration error");
		kfree(new_cfg);
		otx2_qos_sw_node_delete(pfvf, child);
		/* restore the old qos tree */
		node->qid = qid;
		err = otx2_qos_alloc_txschq_node(pfvf, node);
		if (err) {
			netdev_err(pfvf->netdev,
				   "Failed to restore old leaf node");
			goto free_old_cfg;
		}
		err = otx2_qos_txschq_update_config(pfvf, node, old_cfg,
						    node->is_static ? true : false);
		if (err) {
			netdev_err(pfvf->netdev,
				   "Failed to restore txcshq configuration");
			goto free_old_cfg;
		}
		err = otx2_qos_update_smq(pfvf, node,
					  node->is_static ? true : false,
					  QOS_CFG_SQ);
		if (err)
			netdev_err(pfvf->netdev,
				   "Failed to restore smq configuration");
		goto free_old_cfg;
	}

	/* free new txschq config */
	kfree(new_cfg);

	/* free old txschq config */
	otx2_qos_free_cfg(pfvf, old_cfg, static_cfg);
	kfree(old_cfg);

	return 0;

free_node:
	otx2_qos_sw_node_delete(pfvf, child);
free_old_cfg:
	kfree(old_cfg);
out:
	return ret;
}

static int otx2_qos_leaf_del(struct otx2_nic *pfvf, u16 *classid,
			     struct netlink_ext_ack *extack)
{
	struct otx2_qos_node *node, *parent;
	int dwrr_del_node = false, qid;

	netdev_dbg(pfvf->netdev, "TC_HTB_LEAF_DEL classid %04x\n", *classid);

	/* find node related to classid */
	node = otx2_sw_node_find(pfvf, *classid);
	if (!node) {
		NL_SET_ERR_MSG_MOD(extack, "HTB node not found");
		return -ENOENT;
	}
	parent = node->parent;
	qid    = node->qid;

	if (!otx2_qos_is_quantum_static(node->quantum))
		dwrr_del_node = true;

	/* disable sq */
	otx2_qos_disable_sq(pfvf, node->qid);

	/* destroy the leaf node */
	otx2_qos_destroy_node(pfvf, node);
	pfvf->qos.qid_to_sqmap[qid] = OTX2_QOS_INVALID_SQ;


	if (dwrr_del_node)
		parent->child_dwrr_cnt--;

	/* Reset DWRR priority if all dwrr nodes are deleted */
	if (!parent->child_dwrr_cnt) {
		parent->child_dwrr_prio = OTX2_QOS_DEFAULT_PRIO;
		parent->act_dwrr_prio = OTX2_QOS_DEFAULT_PRIO;
	}

	return 0;
}

static int otx2_qos_leaf_del_last(struct otx2_nic *pfvf, u16 classid, bool force,
				  struct netlink_ext_ack *extack)
{
	struct otx2_qos_node *node, *parent;
	struct otx2_qos_cfg *new_cfg;
	int dwrr_del_node = false;
	int err;
	u16 qid;

	netdev_dbg(pfvf->netdev,
		   "TC_HTB_LEAF_DEL_LAST classid %04x\n", classid);

	/* find node related to classid */
	node = otx2_sw_node_find(pfvf, classid);
	if (!node) {
		NL_SET_ERR_MSG_MOD(extack, "HTB node not found");
		return -ENOENT;
	}

	/* save qid for use by parent */
	qid = node->qid;
	parent = otx2_sw_node_find(pfvf, node->parent->classid);
	if (!parent) {
		NL_SET_ERR_MSG_MOD(extack, "parent node not found");
		return -ENOENT;
	}

	if (!otx2_qos_is_quantum_static(node->quantum))
		dwrr_del_node = true;

	/* destroy the leaf node */
	otx2_qos_destroy_node(pfvf, node);

	if (dwrr_del_node)
		parent->child_dwrr_cnt--;

	/* Reset DWRR priority if all dwrr nodes are deleted */
	if (!parent->child_dwrr_cnt) {
		parent->child_dwrr_prio = OTX2_QOS_DEFAULT_PRIO;
		parent->act_dwrr_prio = OTX2_QOS_DEFAULT_PRIO;
	}

	/* create downstream txschq entries to parent */
	otx2_qos_alloc_txschq_node(pfvf, parent);
	parent->qid = qid;

	/* push new txschq config to hw */
	new_cfg = kzalloc(sizeof(*new_cfg), GFP_KERNEL);
	if (!new_cfg) {
		NL_SET_ERR_MSG_MOD(extack, "Memory allocation error");
		return -ENOMEM;
	}
	/* fill txschq cfg and push txschq cfg to hw */
	otx2_qos_fill_cfg_schq(parent, new_cfg);
	err = otx2_qos_push_txschq_cfg(pfvf, parent, new_cfg, false);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack, "HTB HW configuration error");
		kfree(new_cfg);
		return err;
	}
	kfree(new_cfg);

	/* update tx_real_queues */
	otx2_qos_update_tx_netdev_queues(pfvf);

	return 0;
}

int otx2_clean_qos_queues(struct otx2_nic *pfvf)
{
	struct otx2_qos_node *root;

	root = otx2_sw_node_find(pfvf, OTX2_QOS_ROOT_CLASSID);
	if (!root)
		return 0;

	return otx2_qos_update_smq(pfvf, root, true, QOS_SMQ_FLUSH);
}

void otx2_qos_config_txschq(struct otx2_nic *pfvf)
{
	struct otx2_qos_node *root;
	int err;

	root = otx2_sw_node_find(pfvf, OTX2_QOS_ROOT_CLASSID);
	if (!root)
		return;

	err = otx2_qos_txschq_config(pfvf, root);
	if (err) {
		netdev_err(pfvf->netdev, "Error update txschq configuration\n");
		goto root_destroy;
	}

	err = otx2_qos_update_tl1_topology(pfvf, root->schq);
	if (err) {
		netdev_err(pfvf->netdev, "Error updating TL1 topology\n");
		goto root_destroy;
	}

	err = otx2_qos_txschq_push_cfg_static(pfvf, root, NULL);
	if (err) {
		netdev_err(pfvf->netdev, "Error update txschq configuration\n");
		goto root_destroy;
	}

	err = otx2_qos_update_smq(pfvf, root, true, QOS_CFG_SQ);
	if (err) {
		netdev_err(pfvf->netdev, "Error update smq configuration\n");
		goto root_destroy;
	}

	return;

root_destroy:
	otx2_qos_root_destroy(pfvf);
}

int otx2_setup_tc_htb(struct net_device *ndev, struct tc_htb_qopt_offload *htb)
{
	struct otx2_nic *pfvf = netdev_priv(ndev);
	int res;

	switch (htb->command) {
	case TC_HTB_CREATE:
		return otx2_qos_root_add(pfvf, htb->parent_classid,
					 htb->classid, htb->extack);
	case TC_HTB_DESTROY:
		return otx2_qos_root_destroy(pfvf);
	case TC_HTB_LEAF_ALLOC_QUEUE:
		res = otx2_qos_leaf_alloc_queue(pfvf, htb->classid,
						htb->parent_classid,
						htb->rate, htb->ceil,
						htb->prio, htb->quantum,
						htb->extack);
		if (res < 0)
			return res;
		htb->qid = res;
		return 0;
	case TC_HTB_LEAF_TO_INNER:
		return otx2_qos_leaf_to_inner(pfvf, htb->parent_classid,
					      htb->classid, htb->rate,
					      htb->ceil, htb->prio,
					      htb->quantum, htb->extack);
	case TC_HTB_LEAF_DEL:
		return otx2_qos_leaf_del(pfvf, &htb->classid, htb->extack);
	case TC_HTB_LEAF_DEL_LAST:
	case TC_HTB_LEAF_DEL_LAST_FORCE:
		return otx2_qos_leaf_del_last(pfvf, htb->classid,
				htb->command == TC_HTB_LEAF_DEL_LAST_FORCE,
					      htb->extack);
	case TC_HTB_LEAF_QUERY_QUEUE:
		res = otx2_get_txq_by_classid(pfvf, htb->classid);
		htb->qid = res;
		return 0;
	case TC_HTB_NODE_MODIFY:	/* fall through */
	default:
		return -EOPNOTSUPP;
	}
}
