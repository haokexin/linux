// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include "bst_gtc_common.h"
#include <linux/arm-smccc.h>
#include <linux/time64.h>

#define BST_GTC_INIT_DEFAULT 0

static int user_pid = 0;
struct workqueue_struct *gtc_wq;
struct work_struct gtc_work;
static struct gtc_freq gtc_freq_info;
extern struct time_sync_parm record;
extern int bstgtc_log;
struct bst_gtc *pbst_gtc;
static u32 gtc_freq_val[8] = {25000000, 1200000000, 312500000, 125000000, 500000000, 62500000, 125000000, 200000000};

int gtc_user_msg_handler(struct sk_buff *skb, struct genl_info *info);
int gtc_send_msg_to_user(struct time_sync_parm *msg, int len);

#if 1
/* group config */
static struct gtc_mux_pin_t gtc_mux_pin[] = {
    /* safety_sync_out[3]->[0] */
    {0x00, (BIT(4)|BIT(3)|BIT(2)|BIT(1))},
    {0x04, (BIT(4)|BIT(3)|BIT(2)|BIT(1))},
    {0x08, (BIT(4)|BIT(3)|BIT(2)|BIT(1))},
    {0x0c, (BIT(4)|BIT(3)|BIT(2)|BIT(1))},
    /* soc_sync_out[3]->[0]  */
    {0x10, (BIT(8)|BIT(7)|BIT(6)|BIT(5))},
    {0x14, (BIT(8)|BIT(7)|BIT(6)|BIT(5))},
    {0x18, (BIT(8)|BIT(7)|BIT(6)|BIT(5))},
    {0x1c, (BIT(8)|BIT(7)|BIT(6)|BIT(5))},
    /* sw_sync_out[3]->[0] */
    {0x20, (BIT(12)|BIT(11)|BIT(10)|BIT(9))},
    {0x24, (BIT(12)|BIT(11)|BIT(10)|BIT(9))},
    {0x28, (BIT(12)|BIT(11)|BIT(10)|BIT(9))},
    {0x2c, (BIT(12)|BIT(11)|BIT(10)|BIT(9))},
    /* realtime_sync_out[3]->[0] */
    {0x30, (BIT(16)|BIT(15)|BIT(14)|BIT(13))},
    {0x34, (BIT(16)|BIT(15)|BIT(14)|BIT(13))},
    {0x38, (BIT(16)|BIT(15)|BIT(14)|BIT(13))},
    {0x3c, (BIT(16)|BIT(15)|BIT(14)|BIT(13))},
    /* pcie_sync_out[3]->[0] */
    {0x40, (BIT(20)|BIT(19)|BIT(18)|BIT(17))},
    {0x44, (BIT(20)|BIT(19)|BIT(18)|BIT(17))},
    {0x48, (BIT(20)|BIT(19)|BIT(18)|BIT(17))},
    {0x4c, (BIT(20)|BIT(19)|BIT(18)|BIT(17))},
    /* isp_sync_out[7]->[0] */
    {0x50, (BIT(28)|BIT(27)|BIT(26)|BIT(25)|BIT(24)|BIT(23)|BIT(22)|BIT(21))},
    {0x54, (BIT(28)|BIT(27)|BIT(26)|BIT(25)|BIT(24)|BIT(23)|BIT(22)|BIT(21))},
    {0x58, (BIT(28)|BIT(27)|BIT(26)|BIT(25)|BIT(24)|BIT(23)|BIT(22)|BIT(21))},
    {0x5c, (BIT(28)|BIT(27)|BIT(26)|BIT(25)|BIT(24)|BIT(23)|BIT(22)|BIT(21))},
    {0x60, (BIT(28)|BIT(27)|BIT(26)|BIT(25)|BIT(24)|BIT(23)|BIT(22)|BIT(21))},
    {0x64, (BIT(28)|BIT(27)|BIT(26)|BIT(25)|BIT(24)|BIT(23)|BIT(22)|BIT(21))},
    {0x68, (BIT(28)|BIT(27)|BIT(26)|BIT(25)|BIT(24)|BIT(23)|BIT(22)|BIT(21))},
    {0x6c, (BIT(28)|BIT(27)|BIT(26)|BIT(25)|BIT(24)|BIT(23)|BIT(22)|BIT(21))},
};
#else
/* single config */
static struct gtc_mux_pin_t gtc_mux_pin[] = {
    /* safety_sync_out[3]->[0] */
    {0x00, BIT(4)},
    {0x04, BIT(3)},
    {0x08, BIT(2)},
    {0x0c, BIT(1)},
    /* soc_sync_out[3]->[0]  */
    {0x10, BIT(8)},
    {0x14, BIT(7)},
    {0x18, BIT(6)},
    {0x1c, BIT(5)},
    /* sw_sync_out[3]->[0] */
    {0x20, BIT(12)},
    {0x24, BIT(11)},
    {0x28, BIT(10)},
    {0x2c, BIT(9)},
    /* realtime_sync_out[3]->[0] */
    {0x30, BIT(16)},
    {0x34, BIT(15)},
    {0x38, BIT(14)},
    {0x3c, BIT(13)},
    /* pcie_sync_out[3]->[0] */
    {0x40, BIT(20)},
    {0x44, BIT(19)},
    {0x48, BIT(18)},
    {0x4c, BIT(17)},
    /* isp_sync_out[7]->[0] */
    {0x50, BIT(28)},
    {0x54, BIT(27)},
    {0x58, BIT(26)},
    {0x5c, BIT(25)},
    {0x60, BIT(24)},
    {0x64, BIT(23)},
    {0x68, BIT(22)},
    {0x6c, BIT(21)},
};
#endif

/*
 * gtc_latch_en_sel - config gtc latch idx
 * @ioaddr: gtc csr base address
 * @value:  latch idx
 */
void gtc_latch_en_sel(void __iomem *ioaddr, u32 value)
{
	writel(value, ioaddr + GTC_LATCH_EN_SEL);
}

/*
 * gtc_syncbits_sel - config gtc sync bits if select gtc sync single
 * @ioaddr: gtc csr base address
 * @value:  sync bits
 */
void gtc_syncbits_sel(void __iomem *ioaddr, u32 value)
{
	writel(value, ioaddr + GTC_SYNCBITS_SEL);
}

/*
 * gtc_latch_clear - clear gtc latch register
 * @ioaddr: gtc csr base address
 * @value:  clear bits
 */
void gtc_latch_clear(void __iomem *ioaddr, u32 value)
{
	writel(value, ioaddr + GTC_LATCH_CLR);
    wmb();
}

/*
 * gtc_intr_mask - gtc interrupt enable/disable
 * @ioaddr: gtc csr base address
 * @value:  mask bits
 */
void gtc_intr_mask(void __iomem *ioaddr, u32 value)
{
	u32 data;

	data = readl(ioaddr + GTC_INTR_MASK);
	if (value) {
		data &= (~0x1);
		writel(data, ioaddr + GTC_INTR_MASK);
	} else {
		data |= 0x1;
		writel(data, ioaddr + GTC_INTR_MASK);	
	}
}

/*
 * bstgtc_tsgen_config - config tsgen module
 * @enable:  enable or disable tsgen
 * @hicnt:   counter[63:32] value
 * @lwcnt:   counter[31:0] value
 */
void bstgtc_tsgen_config(bool enable, u32 hicnt, u32 lwcnt)
{
    /* wait atf ready then release the following code */
#if 0
	int clk[] = {25, 1200, 312.5, 125, 500, 62.5, 125, 200};
	unsigned int reg_data, mux_idx, div;

	/* get clock div */
	reg_data = bst_sip_special_address_rw(TOP_CRM_BASE_ADDR + GTC_REFCLK_DIV_PARA, 0, 0);
	div = (reg_data >> 24);
	printk(KERN_DEBUG "bstgtc_tsgen_config div = 0x%x\r\n", div);

	/* get clock source idx */
	reg_data = bst_sip_special_address_rw(TOP_CRM_BASE_ADDR + GTC_REFCLK_MUX_CTRL, 0, 0);
	mux_idx = ((reg_data >> 5) & 0xf);
	printk(KERN_DEBUG "bstgtc_tsgen_config mux_idx = 0x%x\r\n", mux_idx);

    /* disable tsgen cnt */
	bst_sip_special_address_rw(GTC_TSGEN_BASE_ADDR + GTC_TSGEN_CNTCR, 0x0, 1);

    if (!enable)
        return;

	/* set tsgen cnt value */
    printk(KERN_DEBUG "bstgtc_tsgen_config lwcnt = 0x%x\r\n", lwcnt);
	bst_sip_special_address_rw(GTC_TSGEN_BASE_ADDR + GTC_TSGEN_CNTCVL, lwcnt, 1);

    printk(KERN_DEBUG "bstgtc_tsgen_config hicnt = 0x%x\r\n", hicnt);
	bst_sip_special_address_rw(GTC_TSGEN_BASE_ADDR + GTC_TSGEN_CNTCVU, hicnt, 1);

	/* set base frequency ID */
	if ((mux_idx >=0) && (mux_idx <= 7) && (div != 0)) {
		reg_data = (clk[mux_idx] * 1000000 / div);
		bst_sip_special_address_rw(GTC_TSGEN_BASE_ADDR + GTC_TSGEN_FREQID, reg_data, 1);
        printk(KERN_DEBUG "bstgtc_tsgen_config freq = 0x%x\r\n", reg_data);

		/* set base frequency ID for ISP */
		//writel(reg_data, 0x520300e4);
	}

    /* enable tsgen cnt */
    bst_sip_special_address_rw(GTC_TSGEN_BASE_ADDR + GTC_TSGEN_CNTCR, 0x1, 1);
#endif
}

/*
 * gtc_mux_config - config gtc mux
 * @ioaddr: gtc csr base address
 * @value:  mux idx
 */
int gtc_mux_config(void __iomem *ioaddr, u32 value)
{
    u32 reg_value = value;
    //u32 default_value = 0x0;
    int i;

    if (reg_value > BST_MAX_SYNC) {
        printk(KERN_ERR "invalid value %d (error: value > 28)\r\n", reg_value);
        return -EINVAL;
    }

    /*	0: gtc_sync_out
    *	4:1: safety_sync_out[3:0]
    *	8:5: soc_sync_out[3:0]
    *	12:9: sw_sync_out[3:0]
    *	16:13: realtime_sync_out[3:0]
    *	20:17: pcie_sync_out[3:0]
    *	28:21: isp_sync_out[3:0]
    */
    for (i = 0; i < ARRAY_SIZE(gtc_mux_pin); i++) {
        /*if(BIT(reg_value) & gtc_mux_pin[i].dis_bits)
            writel(default_value, ioaddr + gtc_mux_pin[i].reg_off);
        else*/
            writel(reg_value, ioaddr + gtc_mux_pin[i].reg_off);
    }

    return 0;
}
static const struct genl_multicast_group gtc_genl_mcgrps[] = {
	[BST_GTC_MCGRP_USER_INFO] = { .name = BST_GTC_MCGRP_USER_NAME},
};

static struct genl_ops gtc_genl_ops[] = {
    {
        .cmd = GTC_CMD_USER_INFO,
        .doit = gtc_user_msg_handler,
    },
};

static struct genl_family gtc_genl_family = {
    .hdrsize = 0,
    .name = GTC_GENL_NAME,
    .version = GTC_GENL_VERSION,
    .maxattr = GTC_ATTR_MAX,
    .ops = gtc_genl_ops,
    .n_ops = ARRAY_SIZE(gtc_genl_ops),
    .resv_start_op	= GTC_CMD_USER_INFO + 1,
    .mcgrps		= gtc_genl_mcgrps,
	.n_mcgrps	= ARRAY_SIZE(gtc_genl_mcgrps),
};

/*
 * gtc_user_msg_handler - receive user msg
 * @skb: genl msg content
 * @info: genl info
 */
int gtc_user_msg_handler(struct sk_buff *skb, struct genl_info *info)
{
    struct nlattr *nlahdr;
    struct genlmsghdr *genlhdr;
    struct user_msg *msg = NULL;
    
    printk("enter %s\n", __func__);
    genlhdr = nlmsg_data(nlmsg_hdr(skb));
    nlahdr = genlmsg_data(genlhdr);
    msg = (struct user_msg *)nla_data(nlahdr);
    if (msg->flag) {
        printk(KERN_DEBUG "receive user pid = %d\n", msg->data);
        user_pid = msg->data;
    }

    return 0;
}

/*
 * gtc_send_msg_to_user - send msg to user
 * @msg: send msg content
 * @len: msg len
 */
int gtc_send_msg_to_user(struct time_sync_parm *msg, int len)
{
    static int seq_num = 0;
    void *genl_data;
    int nl_len, ret = 0;
    struct sk_buff *skb;

    /* total length of attribute including padding */
    nl_len = nla_total_size(len);

    /* allocate a new generic netlink message */
    skb = genlmsg_new(nl_len, GFP_KERNEL);
    if (!skb) {
        printk(KERN_DEBUG "%s skb alloc fail\n", __func__);
        ret = -ENOMEM;
        goto end;
    }

    /* determine msg seq num */
    if (seq_num > GTC_MSG_SEQ_MAX) {
        seq_num = 0;
    } else {
        seq_num++;
    }

    /* add generic netlink header to netlink message */
    genl_data = genlmsg_put(skb, 0, 0, &gtc_genl_family, seq_num, GTC_CMD_SYNC_INFO);
    if (!genl_data) {
        printk(KERN_DEBUG "%s add genl header fail\n", __func__);
        ret = -1;
        goto genl_fail;
    }

    /* add netlink attribute to skb */
    ret = nla_put(skb, GTC_ATTR_SYNC_INFO, len, (void *)msg);
    if (ret) {
        printk(KERN_DEBUG "%s add genl attr fail, ret = %d\n", __func__, ret);
        goto genl_fail;
    }

    genlmsg_end(skb, genl_data);
    
    /* send msg to user space */
    //return genlmsg_unicast(&init_net, skb, user_pid);
    return genlmsg_multicast(&gtc_genl_family, skb , 0, BST_GTC_MCGRP_USER_INFO, GFP_KERNEL);

genl_fail:
    kfree_skb(skb);
end:
    return ret;
}

/*
 * bst_gtc_work - gtc workqueue handle function
 * @gtc_work: gtc workqueue struct
 */
static void bst_gtc_work(struct work_struct *gtc_work)
{
    int ret;
#ifdef CONFIG_BST_DWMAC_ETH
    long utc_nsec;
    long long utc_sec;
#endif

#ifdef CONFIG_BST_DWMAC_ETH
    ret = bstgmac_get_synctime(BST_XGMAC_IDX, &utc_sec, &utc_nsec);
    if (!ret) {
        record.phc_utc_sec = utc_sec;
        record.phc_utc_nsec = utc_nsec;
        if (bstgtc_log)
            printk(KERN_DEBUG "gtc sync utc time : sec = %lld, nsec = %ld\r\n", record.phc_utc_sec, record.phc_utc_nsec);
    }
#endif

    ret = gtc_send_msg_to_user(&record, sizeof(record));
    if (ret) {
        if (bstgtc_log)
            printk(KERN_DEBUG "%s gtc send msg fail, ret = %d\n", __func__, ret);
    }

    return;
}

ktime_t bst_gtc_cnt_to_sys_mono(u64 gtc_cnt)
{
    u64 hicnt1, lwcnt1, hicnt2, lwcnt2;
    u64 cur_cnt, diff_ns, cur_ns;
    unsigned long flags;

    spin_lock_irqsave(&pbst_gtc->ctm_lock, flags);
    cur_ns = ktime_get_ns();
    lwcnt1 = readl(pbst_gtc->addr + GTC_CNTREG20_LO);
    hicnt1 = readl(pbst_gtc->addr + GTC_CNTREG20_HI);
    lwcnt2 = readl(pbst_gtc->addr + GTC_CNTREG20_LO);
    hicnt2 = readl(pbst_gtc->addr + GTC_CNTREG20_HI);
    spin_unlock_irqrestore(&pbst_gtc->ctm_lock, flags);

    if (hicnt1 == hicnt2) {
        cur_cnt = (hicnt2 << 32) | lwcnt2;
    } else {
        cur_cnt = (hicnt1 << 32) | lwcnt1;
    }

    if (cur_cnt < gtc_cnt) {
        diff_ns = (gtc_cnt - cur_cnt) * pbst_gtc->freq_ns; // 1/200m*(10^9)
        if (bstgtc_log) {
            printk(KERN_DEBUG "%s gtc_cnt %llu cur_cnt %llu freq %d diff_ns %llu\n",
                __func__, gtc_cnt, cur_cnt, pbst_gtc->freq_ns, diff_ns);
        }
        return ns_to_ktime(cur_ns + diff_ns);
    } else {
        diff_ns = (cur_cnt - gtc_cnt) * pbst_gtc->freq_ns;
        if (bstgtc_log) {
            printk(KERN_DEBUG "%s gtc_cnt %llu cur_cnt %llu freq %d diff_ns %llu\n",
                __func__, gtc_cnt, cur_cnt, pbst_gtc->freq_ns, diff_ns);
        }
        return ns_to_ktime(cur_ns - diff_ns);
    }    
}
EXPORT_SYMBOL(bst_gtc_cnt_to_sys_mono);

/*
 * bstgtc_soc_interrupt - read gtc latch counter, used to update system time/...
 * @irq:    gtc soc interrupt num  
 * @dev_id: the pointer to the gtc resources structure
 */
static irqreturn_t bstgtc_soc_interrupt(int irq, void *dev_id)
{
    int ret = IRQ_HANDLED;
    unsigned int status;
    struct bst_gtc *gtc_res = (struct bst_gtc *)dev_id;
    void *ioaddr = gtc_res->addr;
    unsigned long flags;

    spin_lock_irqsave(&gtc_res->mono_lock, flags);
    record.latch_gtc_lwcnt = readl(ioaddr + GTC_INTR_LATCH_DATA0);
    record.latch_gtc_hicnt = readl(ioaddr + GTC_INTR_LATCH_DATA1);
    record.gtc_lwcnt = readl(ioaddr + GTC_CNTREG2_LO);
    record.gtc_hicnt = readl(ioaddr + GTC_CNTREG2_HI);
    spin_unlock_irqrestore(&gtc_res->mono_lock, flags);

    status = readl(ioaddr + 0x218);
    if (bstgtc_log) {
        printk(KERN_DEBUG "Hit gtc_sync_int[%d]:current hicnt= %d lwcnt= %d hilatch= %d lwlatch= %d int 0x%x\n",
            irq, record.gtc_hicnt, record.gtc_lwcnt, record.latch_gtc_hicnt, record.latch_gtc_lwcnt, status);
    }
    //status &= 0x7;
    gtc_latch_clear(ioaddr, status);

    return ret;
}

/*
 * bstgtc_interrupt_init - register gtc interrupt handler
*  @gtc_res: the pointer to the gtc resources structure
 */
static int bstgtc_interrupt_init(struct bst_gtc *gtc_res)
{
    int ret = 0;

    if (gtc_res->gtc_irq) {
        //gtc_intr_mask(gtc_res->addr, 0x1f);
        ret = request_irq(gtc_res->gtc_irq, bstgtc_soc_interrupt, IRQF_SHARED, "gtc", gtc_res);
        if (unlikely(ret < 0))
            printk(KERN_ERR "%s: ERROR: allocating the SOC IRQ %d (error: %d)\n",
                    __func__, gtc_res->gtc_irq, ret);
    }

    return ret;
}
#if BST_GTC_INIT_DEFAULT
/*
 * bst_gtc_init - init gtc module
 * @gtc_res:  the pointer to the gtc resources structure
 */
static int bst_gtc_init(struct bst_gtc *gtc_res)
{
    int ret = 0;
    struct timespec64 now;

    /* get system time */
	ktime_get_real_ts64(&now);

    /* initialize tsgen */
    bstgtc_tsgen_config(true, (u32)now.tv_sec, (u32)now.tv_nsec);

    /* config gtc mux */
    printk(KERN_DEBUG "%s() %d addr=%p\n", __func__, __LINE__, gtc_res->addr);
    ret = gtc_mux_config(gtc_res->addr, gtc_res->mux_idx);

    /* config latch idx */
    //gtc_latch_en_sel(gtc_res->addr, gtc_res->mux_idx);

    return ret;
}
#endif
/*
 * bstgtc_get_platform_resources - get dts config parm
 * @pdev:    the pointer to the platform device structure
 * @gtc_res: the pointer to the gtc resources structure
 */
static int bstgtc_get_platform_resources(struct platform_device *pdev,
				   struct bst_gtc *gtc_res)
{
    int ret = 0, i = 0;
    struct resource *res;

    //for (i = 0; i < BST_GTC_IOMEM_NUM; i++) {
    res = platform_get_resource(pdev, IORESOURCE_MEM, i);
    if (res == NULL) {
        printk(KERN_DEBUG "%s: gtc iomem info not found!!\n", __func__);
        ret = -ENOENT;
        goto end;
    }
    gtc_res->addr = devm_ioremap_resource(&pdev->dev, res);
    printk(KERN_DEBUG "%s: paddr start = 0x%llx, end = 0x%llx, vaddr = 0x%llx\n",
                __func__, res->start, res->end, (u64)gtc_res->addr);
    //}
#if defined(CONFIG_BST_C1200_ADAS)
	gtc_res->gtc_irq = platform_get_irq_byname(pdev, "gtc_irq");
	if (gtc_res->gtc_irq < 0) {
        ret = -EPROBE_DEFER;
        printk(KERN_ERR "%s: gtc_irq get fail\n", __func__);
        goto end;
	}
    printk(KERN_DEBUG "%s: irq = %d\n", __func__, gtc_res->gtc_irq);
#endif
    of_property_read_u32(pdev->dev.of_node, "mux_idx", &gtc_res->mux_idx);
    printk(KERN_DEBUG "%s: mux_idx = %d\n", __func__, gtc_res->mux_idx);
    if ((gtc_res->mux_idx) > BST_MAX_SYNC)
	    gtc_res->mux_idx = BST_SOC_XGMAC_SYNC0;

    of_property_read_u32(pdev->dev.of_node, "gtc_syncbit_sel", &gtc_res->gtc_syncbit);
    printk(KERN_DEBUG "%s: gtc_syncbit = %d\n", __func__, gtc_res->gtc_syncbit);
    if ((gtc_res->gtc_syncbit) > BST_GTC_MAX_SYNCBIT)
	    gtc_res->gtc_syncbit = 0;

end:
    return ret;
}

int bst_gtc_get_freq(struct gtc_freq *freq_info)
{
    int ret;
    u32 val;

    ret = scmi_read(TOP_CRM_BASE_ADDR+GTC_REFCLK_DIV_PARA, &val);
    if (ret < 0)
        return -EINVAL;

    freq_info->clk_div = ((val >> 24) & 0xf); 

    ret = scmi_read(TOP_CRM_BASE_ADDR+GTC_REFCLK_MUX_CTRL, &val);
    if (ret < 0)
       return -EINVAL;
    freq_info->clk_freq = ((val >> 5) & 0xf);

    return 0;
}

/*
 * bst_gtc_probe - This is the probe callback function of bst_gtc driver.  
 * @pdev:  the pointer to the platform device structure
 */
static int bst_gtc_probe(struct platform_device *pdev)
{
    int ret = 0;

    printk(KERN_DEBUG "BST_GTC driver initializing ...\n");

    pbst_gtc = devm_kzalloc(&pdev->dev, sizeof(*pbst_gtc), GFP_KERNEL);
    if (pbst_gtc == NULL) {
        return -ENOMEM;
    }

    pbst_gtc->pdev = pdev;
    platform_set_drvdata(pdev, pbst_gtc);

    /* get gtc dts config parm */
	ret = bstgtc_get_platform_resources(pdev, pbst_gtc);
	if (ret < 0) {
        printk(KERN_ERR "%s: get dts resources fail!!\n", __func__);
        goto fail;
    }
#if BST_GTC_INIT_DEFAULT
    /* gtc init */
    ret = bst_gtc_init(pbst_gtc);
    if (ret < 0) {
        printk(KERN_ERR "%s: gtc init fail!!\n", __func__);
        goto fail; 
    }
#endif
    /* register gtc misc dev */
    ret = bst_gtc_miscdev_init(pbst_gtc);
    if (ret < 0) {
        printk(KERN_ERR "bst_gtc_miscdev_init failed, ret %d\n", ret);
        goto fail;
    }

    /* register gtc genl family */
    ret = genl_register_family(&gtc_genl_family);
    if (ret < 0) {
        printk(KERN_ERR "Failed to register genl family: %d\n", ret);
        goto fail;
    }

    /* create gtc workqueue */
    gtc_wq = create_workqueue("gtc_tx");
	if (!gtc_wq) {
		printk(KERN_ERR "failed to create gtc workqueue\n");
		ret = -ENOMEM;
        goto fail;
	}

    INIT_WORK(&gtc_work, bst_gtc_work);

    /* register gtc soc interrupt handler */
    ret = bstgtc_interrupt_init(pbst_gtc);
    if (ret < 0) {
        printk(KERN_ERR "%s: soc interrupt register fail!!\n", __func__);
        goto fail; 
    }

    spin_lock_init(&pbst_gtc->mono_lock);
    spin_lock_init(&pbst_gtc->ctm_lock);
    ret = bst_gtc_get_freq(&gtc_freq_info);
    if (!ret) {
        if ((gtc_freq_info.clk_freq < 8) && (gtc_freq_info.clk_div))
            pbst_gtc->freq_ns = (1000000000 / (gtc_freq_val[gtc_freq_info.clk_freq] / gtc_freq_info.clk_div));
    } else {
        gtc_freq_info.clk_freq = 1;
        gtc_freq_info.clk_div = 6;
        pbst_gtc->freq_ns = 5;
    }

    if (ret == 0) {
        printk(KERN_DEBUG "BST_GTC probe completed!\n");
        return 0;
    }

fail:
    devm_kfree(&pdev->dev, pbst_gtc);
    return ret;
}

/*
 * bst_gtc_remove - This is the remove callback function of bst_gtc driver.
 * @pdev:  the pointer to the platform device structure
 */
static int bst_gtc_remove(struct platform_device *pdev)
{
    int ret = 0;
    struct bst_gtc *pbst_gtc = platform_get_drvdata(pdev);

    printk(KERN_ERR "%s: BST_GTC remove start\n", __func__);

    /* unregister gtc dev */
    bst_gtc_miscdev_exit(pbst_gtc);
#if defined(CONFIG_BST_C1200_ADAS)
    /* free gtc irq */
    if (pbst_gtc->gtc_irq)
        free_irq(pbst_gtc->gtc_irq, pbst_gtc);
#endif
#if BST_GTC_INIT_DEFAULT
    /* disable tsgen */
    bstgtc_tsgen_config(false, 0, 0);

    /* reset gtc mux */
    ret = gtc_mux_config(pbst_gtc->addr, 0);
	if (ret < 0)
        dev_err(&pdev->dev, "%s: gtc mux reset fail!!\n", __func__);
#endif
    /* free gtc resources */
    devm_kfree(&pdev->dev, pbst_gtc);
    pbst_gtc = NULL;
    /* unregister gtc genl family */
    genl_unregister_family(&gtc_genl_family);

    /* destroy gtc workqueue */
    destroy_workqueue(gtc_wq);

    printk(KERN_ERR "%s: BST_GTC remove completed\n", __func__);

    return ret;
}
static void bst_gtc_shutdown(struct platform_device *pdev)
{
    bst_gtc_remove(pdev);
}

static const struct of_device_id bst_gtc_of_match[] = {
    {.compatible = "bst,bst-gtc",},
    {},
};

static struct platform_driver bst_gtc_driver = {
    .probe   = bst_gtc_probe,
    .remove  = bst_gtc_remove,
    .shutdown = bst_gtc_shutdown,
    .driver  = {
        .name = BST_GTC_DRIVER_NAME,
        .of_match_table = of_match_ptr(bst_gtc_of_match),
    },
};

static int __init bst_gtc_driver_init(void)
{
    return platform_driver_register(&bst_gtc_driver);
}
late_initcall(bst_gtc_driver_init);

static void __exit bst_gtc_driver_exit(void)
{
    platform_driver_unregister(&bst_gtc_driver);
    return;
}
module_exit(bst_gtc_driver_exit);

MODULE_DESCRIPTION("BST GTC Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
