// SPDX-License-Identifier: GPL-2.0-only
/*
 * PTP 1588 clock using the BSTMAC.
 * Copyright (C) 2013  Vayavya Labs Pvt Ltd
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#include "bstgmac.h"
#include "dwmac_ptp.h"
#include "dwmac_main.h"
#ifdef CONFIG_BST_GTC
#include "bst_gtc_common.h"
#endif
u64 tx_ts = 0;
#ifdef CONFIG_BST_C1200_ADAS
extern int xgmac_mb_init;
#endif
//#define  DIVSEC 1000000000ULL

/**
 * bstgmac_adjust_freq
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ppb: desired period change in parts ber billion
 *
 * Description: this function will adjust the frequency of hardware clock.
 */
static int bstgmac_adjust_freq(struct ptp_clock_info *ptp, s32 ppb)
{
	struct bstgmac_priv *priv =
	    container_of(ptp, struct bstgmac_priv, ptp_clock_ops);
	unsigned long flags;
	u32 diff, addend;
	int neg_adj = 0;
	u64 adj;
	struct bstptp_ctl *ctl = priv->ptpctl;

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}

	addend = priv->default_addend;
	adj = addend;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);
	addend = neg_adj ? (addend - diff) : (addend + diff);

	if (ctl->multip) {
		spin_lock_irqsave(&priv->ptp_lock, flags);
		bstgmac_config_addend(priv, ctl->ptp0_reg, addend);
		bstgmac_config_addend(priv, ctl->ptp1_reg, addend);
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
	} else {
		spin_lock_irqsave(&priv->ptp_lock, flags);
		bstgmac_config_addend(priv, priv->ptpaddr, addend);
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
	}
	//priv->ptp_clock_ops.elapsed = ktime_get_boottime()/DIVSEC;

	return 0;
}

/**
 * bstgmac_adjust_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @delta: desired change in nanoseconds
 *
 * Description: this function will shift/adjust the hardware clock time.
 */
static int bstgmac_adjust_time(struct ptp_clock_info *ptp, s64 delta)
{
	struct bstgmac_priv *priv =
	    container_of(ptp, struct bstgmac_priv, ptp_clock_ops);
	unsigned long flags;
	u32 sec, nsec;
	u32 quotient, reminder;
	int neg_adj = 0;
	bool xmac, est_rst = false;
	int ret;
	struct bstptp_ctl *ctl = priv->ptpctl;

	xmac = priv->plat->has_gmac4 || priv->plat->has_xgmac;

	if (delta < 0) {
		neg_adj = 1;
		delta = -delta;
	}

	quotient = div_u64_rem(delta, 1000000000ULL, &reminder);
	sec = quotient;
	nsec = reminder;

	/* If EST is enabled, disabled it before adjust ptp time. */
	if (priv->plat->est && priv->plat->est->enable) {
		est_rst = true;
		mutex_lock(&priv->plat->est->lock);
		priv->plat->est->enable = false;
		bstgmac_est_configure(priv, priv->ioaddr, priv->plat->est,
				      priv->plat->clk_ptp_rate);
		mutex_unlock(&priv->plat->est->lock);
	}

	if (ctl->multip) {
		spin_lock_irqsave(&priv->ptp_lock, flags);
		bstgmac_adjust_systime(priv, ctl->ptp1_reg, sec, nsec, neg_adj,
				       xmac);
		bstgmac_adjust_systime(priv, ctl->ptp0_reg, sec, nsec, neg_adj,
				       xmac);
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
	} else {
		spin_lock_irqsave(&priv->ptp_lock, flags);
		bstgmac_adjust_systime(priv, priv->ptpaddr, sec, nsec, neg_adj,
				       xmac);
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
	}
	//priv->ptp_clock_ops.elapsed = ktime_get_boottime()/DIVSEC;

	/* Calculate new basetime and re-configured EST after PTP time adjust. */
	if (est_rst) {
		struct timespec64 current_time, time;
		ktime_t current_time_ns, basetime;
		u64 cycle_time;

		mutex_lock(&priv->plat->est->lock);
		priv->ptp_clock_ops.gettime64(&priv->ptp_clock_ops,
					      &current_time);
		current_time_ns = timespec64_to_ktime(current_time);
		time.tv_nsec = priv->plat->est->btr_reserve[0];
		time.tv_sec = priv->plat->est->btr_reserve[1];
		basetime = timespec64_to_ktime(time);
		cycle_time = priv->plat->est->ctr[1] * NSEC_PER_SEC +
		    priv->plat->est->ctr[0];
		time = bstgmac_calc_tas_basetime(basetime,
						 current_time_ns, cycle_time);

		priv->plat->est->btr[0] = (u32)time.tv_nsec;
		priv->plat->est->btr[1] = (u32)time.tv_sec;
		priv->plat->est->enable = true;
		ret = bstgmac_est_configure(priv, priv->ioaddr, priv->plat->est,
					    priv->plat->clk_ptp_rate);
		mutex_unlock(&priv->plat->est->lock);
		if (ret)
			netdev_err(priv->dev, "failed to configure EST\n");
	}

	return 0;
}

/**
 * bstgmac_get_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ts: pointer to hold time/result
 *
 * Description: this function will read the current time from the
 * hardware clock and store it in @ts.
 */
static int bstgmac_get_time(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct bstgmac_priv *priv =
	    container_of(ptp, struct bstgmac_priv, ptp_clock_ops);
	unsigned long flags;
	u64 ns = 0;

	if ((readl(priv->ptpaddr + PTP_TCR) & 1) == 0) {
		pr_err("ptp clock uninitialized\n");
		return -EINTR;
	}

	spin_lock_irqsave(&priv->ptp_lock, flags);
	bstgmac_get_systime(priv, priv->ptpaddr, &ns);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	*ts = ns_to_timespec64(ns);

	return 0;
}

int bst_get_ts_from_xgmac(struct timespec64 *ts, struct ptp_system_timestamp *sts)
{
	struct bstgmac_priv *priv = gmac_priv_g[0];
	unsigned long flags;
	u64 ns = 0;

	if ((readl(priv->ptpaddr + PTP_TCR) & 1) == 0) {
		return -EINTR;
	}
#ifdef CONFIG_BST_C1200_ADAS
	if (xgmac_mb_init == BSTMAC_MB_SUB_OK)
		return -ENOENT;

	if (xgmac_mb_init != BSTMAC_MB_SUB_DONE)
		return -EPERM;
#endif
	spin_lock_irqsave(&priv->ptp_lock, flags);
	bstgmac_get_systime(priv, priv->ptpaddr, &ns);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	if (ts) {
		*ts = ns_to_timespec64(ns);
	}

	if (sts) {
		sts->pre_ts = ns_to_timespec64(ns + priv->time_offset);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(bst_get_ts_from_xgmac);
/**
 * bstgmac_get_timex
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ts: holds the PHC timestamp
 * @sts: if not NULL, it holds a pair of timestamps from the system clock
 * Description: this function will read the current time from the
 * hardware clock and store it in @ts.
 */
static int bstgmac_get_timex(struct ptp_clock_info *ptp, struct timespec64 *ts,
			struct ptp_system_timestamp *sts)
{
	struct bstgmac_priv *priv =
	    container_of(ptp, struct bstgmac_priv, ptp_clock_ops);
	unsigned long flags;
	u64 ns = 0;

	if ((readl(priv->ptpaddr + PTP_TCR) & 1) == 0) {
		pr_err("ptp clock uninitialized\n");
		return -EINTR;
	}

	spin_lock_irqsave(&priv->ptp_lock, flags);
	bstgmac_get_systime(priv, priv->ptpaddr, &ns);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	if (ts) {
		*ts = ns_to_timespec64(ns);
	}

	if (sts) {
		sts->pre_ts = ns_to_timespec64(ns + priv->time_offset);
	}
#ifdef CONFIG_BST_C1200_ADAS
	if (xgmac_mb_init != BSTMAC_MB_SUB_DONE) {
		if (sts)
			sts->pre_ts.tv_sec = 0;
	}
#endif

	return 0;
}

/**
 * bstgmac_set_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ts: time value to set
 *
 * Description: this function will set the current time on the
 * hardware clock.
 */
static int bstgmac_set_time(struct ptp_clock_info *ptp,
			    const struct timespec64 *ts)
{
	unsigned long flags;
	struct bstgmac_priv *priv =
	    container_of(ptp, struct bstgmac_priv, ptp_clock_ops);

	struct bstptp_ctl *ctl = priv->ptpctl;

	if (ctl->ppsfix) {
		if (ctl->ppssta--) {
			/* Time setting will be done in interrupt */
			ctl->utc.tv_sec = ts->tv_sec;
			ctl->utc.tv_nsec = ts->tv_nsec;
		} else {
			pr_debug("Settime Failed, NO PPS\n");
			return -1;
		}
	} else {
		if (ctl->multip) {
			spin_lock_irqsave(&priv->ptp_lock, flags);
			bstgmac_init_systime(priv, ctl->ptp0_reg, ts->tv_sec,
					     ts->tv_nsec);
			bstgmac_init_systime(priv, ctl->ptp1_reg, ts->tv_sec,
					     ts->tv_nsec);
			spin_unlock_irqrestore(&priv->ptp_lock, flags);
		} else {
			spin_lock_irqsave(&priv->ptp_lock, flags);
			bstgmac_init_systime(priv, priv->ptpaddr, ts->tv_sec,
					     ts->tv_nsec);
			spin_unlock_irqrestore(&priv->ptp_lock, flags);
		}
	}
	//priv->ptp_clock_ops.elapsed = ktime_get_boottime()/DIVSEC;

	return 0;
}

static int bstgmac_get_snapshot(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	u64 ns;
	struct bstgmac_priv *priv =
	    container_of(ptp, struct bstgmac_priv, ptp_clock_ops);
	struct bstptp_ctl *ctl = priv->ptpctl;

	ns = ctl->aux_snap.snap_time.tv_nsec;
	ns += ctl->aux_snap.snap_time.tv_sec * 1000000000ULL;
	*ts = ns_to_timespec64(ns);

	ctl->aux_snap.snap_time.tv_nsec = 0;
	ctl->aux_snap.snap_time.tv_sec = 0;
	return 0;
}

int bstgmac_get_synctime(unsigned int gmac_idx, long long *sec, long *nsec)
{
	struct bstgmac_priv *priv;
	struct bstptp_ctl *ctl;

	if (gmac_idx >= BSTGMAC_CORE_NUM) {
		pr_debug("invalid gmac idx is %d\n", gmac_idx);
		return -EINVAL;
	}

	if (gmac_priv_g[gmac_idx]) {
		priv = gmac_priv_g[gmac_idx];
		if (!priv)
			return -EPERM;
		ctl = priv->ptpctl;
		if (test_bit(BSTGMAC_DOWN, &priv->state))
			return -EPERM;

		if (priv->ptpctl) {
			*sec = ctl->aux_snap.snap_time.tv_sec;
			*nsec = ctl->aux_snap.snap_time.tv_nsec;
			ctl->aux_snap.snap_time.tv_sec = 0;
			ctl->aux_snap.snap_time.tv_nsec = 0;
			pr_debug("%s: aux sec=%lld, nsec=%ld\n", __func__, *sec, *nsec);
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(bstgmac_get_synctime);

int bstptp_get_tstargt_shift(struct bstgmac_priv *priv)
{
	int ret = 0;
	struct bstptp_ctl *ctl = priv->ptpctl;

	switch (ctl->flex_pps.idx) {
	case 0:
		ret = BIT(1);
		break;
	case 1:
		ret = BIT(4);
		break;
	case 2:
		ret = BIT(6);
		break;
	case 3:
		ret = BIT(8);
		break;
	default:
		netdev_err(priv->dev, "Invalid flex pps idx : %d\n", ctl->flex_pps.idx);
		return ret;
	}

	return ret;
}

int bstptp_aux_config(struct bstgmac_priv *priv, unsigned int idx)
{
	int ret = 0;
	u32 reg_value = 0;
	void __iomem *ptp_reg = priv->ptpaddr;

	if (idx < PTP_MAX_TRIG_IN_NUM) {
		reg_value = readl(ptp_reg + PTP_ACR);
		reg_value &= ~PTP_ACR_MASK;
		reg_value |= BIT(4 + idx);
		writel(reg_value, ptp_reg + PTP_ACR);
		priv->ptpctl->aux_snap.idx = idx;
		priv->ptpctl->aux_snap.status = true;
		netdev_dbg(priv->dev, "%s() reg_value=0x%x, aux_reg=0x%x\n",
				__func__, reg_value, readl(ptp_reg + PTP_ACR));
	} else {
		ret = -EINVAL;
		netdev_err(priv->dev, "%s() index%d is invalid!!\n", __func__, idx);
	}

	return ret;
}

void bstptp_extts_interrupt(int irq, struct bstgmac_priv *priv)
{
	unsigned long flags;
	struct bstptp_ctl *ctl = priv->ptpctl;
	u32 reg_value = 0, aux_nan, sta_nan, nan, shift;
#ifdef CONFIG_BST_GTC
	u32 work_flag = 0;
#endif
	/* Timestamp Interrupt disabled */
	reg_value = readl(priv->ioaddr + PTP_MAC_INTR_EN);
	reg_value = (reg_value & (~MAC_INTR_EN_TSIE));
	writel(reg_value, priv->ioaddr + PTP_MAC_INTR_EN);

	if (priv->plat->has_xgmac) {
		/* Get Timestamp_Status */
		reg_value = readl(priv->ptpaddr + PTP_TSR);
		/* get flex pps interrupt */
		if (ctl->flex_pps.status) {
			shift = bstptp_get_tstargt_shift(priv);
			if (reg_value & shift) {	
#ifdef CONFIG_BST_GTC
				work_flag = 1;
#endif
				netdev_dbg(priv->dev, "Hit PPS target time interrupt!!\n");
			}
		}

		/* get auxiliary snapshot time */
		if ((ctl->aux_snap.status) && (reg_value & BIT(2))) {
			ctl->aux_snap.snap_time.tv_nsec = readl(priv->ptpaddr + PTP_ATNSR);
			ctl->aux_snap.snap_time.tv_sec = readl(priv->ptpaddr + PTP_ATSR);
			netdev_dbg(priv->dev, "Hit Aux snapshot interrupt, aux sec=%lld, nsec=%ld!!\n",
					   ctl->aux_snap.snap_time.tv_sec, ctl->aux_snap.snap_time.tv_nsec);
			/* Auxiliary Snapshot FIFO Clear */
			reg_value = readl(priv->ptpaddr + PTP_ACR);
			reg_value = reg_value | 1;
			writel(reg_value, priv->ptpaddr + PTP_ACR);
#ifdef CONFIG_BST_GTC
			work_flag = 1;
#endif
		}
		if ((reg_value & BIT(15))) { 
			bstgmac_get_mac_tx_timestamp(priv, priv->hw, &tx_ts);	
		}
	} else if (priv->plat->has_gmac4) {
		aux_nan = readl(priv->ptpaddr + 0x48);
		sta_nan = readl(priv->ptpaddr + 0x0c);

		if (sta_nan >= aux_nan)
			nan = (sta_nan - aux_nan);	// + 10800;
		else
			nan = (0x3B9AC9FF - aux_nan + sta_nan);	// + 10800;

		if (ctl->utc.tv_sec != 0) {
			ctl->utc.tv_sec = ctl->utc.tv_sec + 1;

			if (ctl->multip) {
				spin_lock_irqsave(&priv->ptp_lock, flags);
				bstgmac_init_systime(priv, ctl->ptp0_reg,
							ctl->utc.tv_sec, nan);
				bstgmac_init_systime(priv, ctl->ptp1_reg,
							ctl->utc.tv_sec, nan);
				spin_unlock_irqrestore(&priv->ptp_lock, flags);
			} else {
				spin_lock_irqsave(&priv->ptp_lock, flags);
				bstgmac_init_systime(priv, priv->ptpaddr,
							ctl->utc.tv_sec, nan);
				spin_unlock_irqrestore(&priv->ptp_lock, flags);
			}

			ctl->utc.tv_sec = 0;
			ctl->utc.tv_nsec = 0;
		}
		ctl->ppssta = 2;

		/* Get Timestamp_Status */
		reg_value = readl(priv->ptpaddr + PTP_TSR);

		/* Auxiliary Snapshot FIFO Clear */
		reg_value = readl(priv->ptpaddr + PTP_ACR);
		reg_value = reg_value | 1;
		writel(reg_value, priv->ptpaddr + PTP_ACR);
	}
#ifdef CONFIG_BST_GTC
	if (priv->plat->has_xgmac && work_flag)
		queue_work_on(GTC_WQ_DEF_CPU, gtc_wq, &gtc_work);
#endif
	/* Timestamp Interrupt enable */
	reg_value = readl(priv->ioaddr + PTP_MAC_INTR_EN);
	reg_value = (reg_value | MAC_INTR_EN_TSIE);
	writel(reg_value, priv->ioaddr + PTP_MAC_INTR_EN);
}

static int bstptp_extts_config(struct bstgmac_priv *priv,
			       struct ptp_extts_request extts)
{
	int ret = 0;
	bool xmac;
	u64 temp = 0;
	struct timespec64 now;
	u32 reg_value = 0, sec_inc = 0, ts_ctrl;
	void __iomem *ptp_reg = priv->ptpaddr;
	struct bstptp_ctl *ctl = priv->ptpctl;

	xmac = priv->plat->has_gmac4 || priv->plat->has_xgmac;

	/* Timestamp Interrupt disabled */
	reg_value = readl(priv->ioaddr + PTP_MAC_INTR_EN);
	reg_value = (reg_value & (~MAC_INTR_EN_TSIE));
	writel(reg_value, priv->ioaddr + PTP_MAC_INTR_EN);

	/* Init ptp cfg */
	ts_ctrl = readl(ptp_reg + PTP_TCR);
	ts_ctrl = ts_ctrl | 0x203;
	writel(ts_ctrl, ptp_reg + PTP_TCR);

	/* Enable INxx interrupt */
	if (extts.flags & PTP_ENABLE_FEATURE) {
		/* auxiliary snapshot config */
		if (priv->plat->has_xgmac) {
			ret = bstptp_aux_config(priv, extts.index);
			if (!ret) {
				/* Timestamp Interrupt enable */
				reg_value = readl(priv->ioaddr + PTP_MAC_INTR_EN);
				reg_value = (reg_value | MAC_INTR_EN_TSIE);
				writel(reg_value, priv->ioaddr + PTP_MAC_INTR_EN);
				ctl->extintr = 1;
				netdev_dbg(priv->dev, "bstptp config extts intr on\n");
			}
		} else {
			/* Auxiliary Snapshot 0/3 Enable */
			reg_value = BIT(4) | BIT(7);	//EVB-IN00 EC-IN10
			writel(reg_value, ptp_reg + PTP_ACR);

			/* Timestamp Interrupt enable */
			reg_value = readl(priv->ioaddr + PTP_MAC_INTR_EN);
			reg_value = (reg_value | MAC_INTR_EN_TSIE);
			writel(reg_value, priv->ioaddr + PTP_MAC_INTR_EN);
			ctl->extintr = 1;
			netdev_dbg(priv->dev, "bstptp config extts intr on\n");
		}
	} else {
		/* Auxiliary Snapshot FIFO clear */
		writel(1, priv->ptpaddr + PTP_ACR);
		/* Timestamp Interrupt disabled */
		reg_value = readl(priv->ioaddr + PTP_MAC_INTR_EN);
		reg_value = (reg_value & (~MAC_INTR_EN_TSIE));
		writel(reg_value, priv->ioaddr + PTP_MAC_INTR_EN);
		ctl->extintr = 0;
		netdev_dbg(priv->dev, "bstptp config extts intr off\n");
	}
	if (priv->systime_flags & PTP_TCR_TSENA) {
		pr_err("PTP has been initialized and will not be executing initialization this time\n");
		return ret;
	}

	/* MAC_Sub_Second_Increment 0xb04 */
	/* program Sub Second Increment reg */
	bstgmac_config_sub_second_increment(priv,
					    priv->ptpaddr,
					    priv->plat->clk_ptp_rate, xmac,
					    &sec_inc);
	if (sec_inc)
		temp = div_u64(1000000000ULL, sec_inc);

	/* Store sub second increment and flags for later use */
	priv->sub_second_inc = sec_inc;
	priv->systime_flags = ts_ctrl;

	/* calculate default added value:
	 * formula is :
	 * addend = (2^32)/freq_div_ratio;
	 * where, freq_div_ratio = 1e9ns/sec_inc
	 */
	temp = (u64)(temp << 32);
	priv->default_addend = div_u64(temp, priv->plat->clk_ptp_rate);
	bstgmac_config_addend(priv, priv->ptpaddr, priv->default_addend);

	/* initialize system time */
	ktime_get_real_ts64(&now);

	/* lower 32 bits of tv_sec are safe until y2106 */
	bstgmac_init_systime(priv, priv->ptpaddr,
			     (u32)now.tv_sec, now.tv_nsec);
	//priv->ptp_clock_ops.elapsed = ktime_get_boottime()/DIVSEC;
	//priv->ptp_clock_ops.status = PTP_STA_INITED;

	if (priv->plat->has_gmac4) {
		/* gps-pps fix */
		if (extts.rsv[0]) {
			ctl->ppsfix = 1;
			pr_err("bstptp config ppsfix on\n");
		} else {
			ctl->ppsfix = 0;
			pr_err("bstptp config ppsfix off\n");
		}

		/* multip ptp set */
		if (extts.rsv[1]) {
			ctl->multip = 1;
			pr_err("bstptp config multi on\n");
		} else {
			ctl->multip = 0;
			pr_err("bstptp config multi off\n");
		}
	}

	return ret;
}

static int bstptp_pps_config(struct bstgmac_priv *priv, int on)
{
	bool needs_xgmac = priv->plat->has_xgmac;
	void __iomem *ptp_reg = NULL;

	ptp_reg = priv->ioaddr +
		(needs_xgmac ? PTP_XGMAC_OFFSET : PTP_GMAC4_OFFSET);

	if (on) {
		/* MAC_PPS_Control */
		writel(1, ptp_reg + PTP_PPSCR);
		pr_debug("PPS OUT ON\n");
	} else {
		writel(0, ptp_reg + PTP_PPSCR);
		pr_debug("PPS OUT OFF\n");
	}

	return 0;
}

static int bstgmac_enable(struct ptp_clock_info *ptp,
			  struct ptp_clock_request *rq, int on)
{
	struct bstgmac_priv *priv =
	    container_of(ptp, struct bstgmac_priv, ptp_clock_ops);
	struct bstgmac_pps_cfg *cfg;
	int ret = -EOPNOTSUPP;
	unsigned long flags;

	switch (rq->type) {
	case PTP_CLK_REQ_PEROUT:
		//return -EOPNOTSUPP; //conflict with ifconfig eth up
		cfg = &priv->pps[rq->perout.index];

		cfg->start.tv_sec = rq->perout.start.sec;
		cfg->start.tv_nsec = rq->perout.start.nsec;
		cfg->period.tv_sec = rq->perout.period.sec;
		cfg->period.tv_nsec = rq->perout.period.nsec;

		spin_lock_irqsave(&priv->ptp_flex_lock, flags);
		ret = bstgmac_flex_pps_config(priv, priv->ioaddr,
					      rq->perout.index, cfg, on,
					      priv->sub_second_inc,
					      priv->systime_flags);
		spin_unlock_irqrestore(&priv->ptp_flex_lock, flags);
		if (!ret) {
			if (on) {
				priv->ptpctl->flex_pps.status = true;
				priv->ptpctl->flex_pps.idx = rq->perout.index;
				priv->ptpctl->flex_pps.start_time = cfg->start;
				priv->ptpctl->flex_pps.period = cfg->period;
			} else {
				priv->ptpctl->flex_pps.status = false;
				priv->ptpctl->flex_pps.idx = 0;
				priv->ptpctl->flex_pps.start_time.tv_sec = 0;
				priv->ptpctl->flex_pps.start_time.tv_nsec = 0;
				priv->ptpctl->flex_pps.period.tv_sec = 0;
				priv->ptpctl->flex_pps.period.tv_nsec = 0;
			}
			netdev_dbg(priv->dev, "%s() idx:%d, start time: sec=%lld, nsec=%ld, status=%d, on=%d\n",
				   __func__, priv->ptpctl->flex_pps.idx,
				   priv->ptpctl->flex_pps.start_time.tv_sec,
				   priv->ptpctl->flex_pps.start_time.tv_nsec,
				   priv->ptpctl->flex_pps.status, on);
		}
		break;
	case PTP_CLK_REQ_EXTTS:
		ret = bstptp_extts_config(priv, rq->extts);
		break;
	case PTP_CLK_REQ_PPS:
		ret = bstptp_pps_config(priv, on);
		break;
	default:
		break;
	}

	return ret;
}

/* structure describing a PTP hardware clock */
static struct ptp_clock_info bstgmac_ptp_clock_ops = {
	.owner = THIS_MODULE,
	.name = "bstmac_ptp_clock",
	.max_adj = 62500000,
	.n_alarm = 0,
	.n_ext_ts = 4,
	.n_per_out = 0,		/* will be overwritten in bstgmac_ptp_register */
	.n_pins = 0,
	.pps = 1,
	.adjfreq = bstgmac_adjust_freq,
	.adjtime = bstgmac_adjust_time,
	.gettime64 = bstgmac_get_time,
	.gettimex64 = bstgmac_get_timex,
	.settime64 = bstgmac_set_time,
	.enable = bstgmac_enable,
	.getsnapshot = bstgmac_get_snapshot,
};

/**
 * bstgmac_ptp_register
 * @priv: driver private structure
 * Description: this function will register the ptp clock driver
 * to kernel. It also does some house keeping work.
 */
void bstgmac_ptp_register(struct bstgmac_priv *priv)
{
	int i;
	bool needs_xgmac = priv->plat->has_xgmac;
	bool needs_gmac4 = priv->plat->has_gmac4;

	for (i = 0; i < priv->dma_cap.pps_out_num; i++) {
		if (i >= BSTGMAC_PPS_MAX)
			break;
		priv->pps[i].available = true;
	}

	bstgmac_ptp_clock_ops.n_per_out = priv->dma_cap.pps_out_num;

	priv->ptpctl = kmalloc(sizeof(struct bstptp_ctl), GFP_KERNEL);
	if (priv->ptpctl) {
		memset(priv->ptpctl, 0, sizeof(struct bstptp_ctl));
		if (needs_gmac4) {
			priv->ptpctl->ptp0_reg = ioremap(BST_GMAC_0_BASE_ADDR + PTP_GMAC4_OFFSET, 200);
			priv->ptpctl->ptp1_reg = ioremap(BST_GMAC_1_BASE_ADDR + PTP_GMAC4_OFFSET, 200);
		} else if (needs_xgmac) {
			priv->ptpctl->ptp0_reg = ioremap(BST_XGMAC_BASE_ADDR + PTP_XGMAC_OFFSET, 200);
			spin_lock_init(&priv->ptpctl->tx_ts_lock);
		}
	}

	spin_lock_init(&priv->ptp_lock);
	spin_lock_init(&priv->ptp_flex_lock);

	priv->ptp_clock_ops = bstgmac_ptp_clock_ops;

	priv->device->id = priv->plat->bus_id;

	priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_ops,
					     priv->device);
	if (IS_ERR(priv->ptp_clock)) {
		netdev_err(priv->dev, "ptp_clock_register failed\n");
		priv->ptp_clock = NULL;
	} else if (priv->ptp_clock) {
		netdev_info(priv->dev, "registered PTP clock\n");
	}
}

/**
 * bstgmac_ptp_unregister
 * @priv: driver private structure
 * Description: this function will remove/unregister the ptp clock driver
 * from the kernel.
 */
void bstgmac_ptp_unregister(struct bstgmac_priv *priv)
{
	priv->systime_flags = 0;

	if (priv->ptpctl) {
		if (priv->ptpctl->ptp0_reg)
			iounmap(priv->ptpctl->ptp0_reg);
		if (priv->ptpctl->ptp1_reg)
			iounmap(priv->ptpctl->ptp1_reg);
		kfree(priv->ptpctl);
	}

	if (priv->ptp_clock) {
		ptp_clock_unregister(priv->ptp_clock);
		priv->ptp_clock = NULL;
		pr_err("Removed PTP HW clock successfully on %s\n",
			 priv->dev->name);
	}
}
