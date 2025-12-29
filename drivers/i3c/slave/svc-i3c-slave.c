// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/i3c/master.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

/* slave Mode Registers */
#define SVC_I3C_SCONFIG      0x004
#define   SVC_I3C_SCONFIG_SLAVE_EN BIT(0)
#define   SVC_I3C_SCONFIG_MATCHSS(x) FIELD_PREP(BIT(2), (x))
#define   SVC_I3C_SCONFIG_S0IGNORE(x) FIELD_PREP(BIT(3), (x))
#define   SVC_I3C_SCONFIG_BTML(x) FIELD_PREP(GENMASK(6, 5), (x))
#define   SVC_I3C_SCONFIG_OFFLINE(x) FIELD_PREP(BIT(9), (x))
#define   SVC_I3C_SCONFIG_BAMATCH(x) FIELD_PREP(GENMASK(23, 16), (x))
#define   SVC_I3C_SCONFIG_SADDR(x) FIELD_PREP(GENMASK(31, 25), (x))

#define SVC_I3C_SCTRL        0x00C
#define   SVC_I3C_SCTRL_EVENT(x) FIELD_PREP(GENMASK(1, 0), (x))
#define   SVC_I3C_SCTRL_MAPIDX(x) FIELD_PREP(GENMASK(7, 4), (x))
#define   SVC_I3C_SCTRL_PENDINT(x) FIELD_PREP(GENMASK(19, 16), (x))
#define   SVC_I3C_SCTRL_ACTSTATE(x) FIELD_PREP(GENMASK(21, 20), (x))
#define   SVC_I3C_SCTRL_VENDORINFO(x) FIELD_PREP(GENMASK(31, 24), (x))

#define SVC_I3C_SSTATUS      0x008
#define   SVC_I3C_SSTATUS_START BIT(8)
#define   SVC_I3C_SSTATUS_MATCHED BIT(9)
#define   SVC_I3C_SSTATUS_STOP BIT(10)
#define   SVC_I3C_SSTATUS_RXPEND BIT(11)
#define   SVC_I3C_SSTATUS_TXNOTFULL BIT(12)
#define   SVC_I3C_SSTATUS_DACHG BIT(13)
#define   SVC_I3C_SSTATUS_CCC BIT(14)
#define   SVC_I3C_SSTATUS_ERRWARN BIT(15)
#define   SVC_I3C_SSTATUS_HDRMACHED BIT(16)
#define   SVC_I3C_SSTATUS_CHANDLED BIT(17)
#define   SVC_I3C_SSTATUS_EVENT BIT(18)

#define SVC_I3C_SINTSET      0x010
#define SVC_I3C_SINTSET_START BIT(8)
#define SVC_I3C_SINTSET_MATCHED BIT(9)
#define SVC_I3C_SINTSET_STOP BIT(10)
#define SVC_I3C_SINTSET_RXPEND BIT(11)
#define SVC_I3C_SINTSET_TXNOTFULL BIT(12)
#define SVC_I3C_SINTSET_DACHG BIT(13)
#define SVC_I3C_SINTSET_CCC BIT(14)
#define SVC_I3C_SINTSET_ERRWARN BIT(15)
#define SVC_I3C_SINTSET_HDRMATCHED BIT(16)
#define SVC_I3C_SINTSET_CHANDLED BIT(17)
#define SVC_I3C_SINTSET_EVENT BIT(18)
#define SVC_I3C_SINTSET_TGTRST BIT(19)

#define SVC_I3C_SINTCLR      0x014
#define SVC_I3C_SINTMASKED   0x018

#define SVC_I3C_SERRWARN     0x01C
#define   SVC_I3C_SERRWARN_ORUN BIT(0)
#define   SVC_I3C_SERRWARN_URUN BIT(1)
#define   SVC_I3C_SERRWARN_URUNNACK BIT(2)
#define   SVC_I3C_SERRWARN_TERM BIT(3)
#define   SVC_I3C_SERRWARN_INVSTART BIT(4)
#define   SVC_I3C_SERRWARN_SPAR BIT(8)
#define   SVC_I3C_SERRWARN_HPAR BIT(9)
#define   SVC_I3C_SERRWARN_HCRC BIT(10)
#define   SVC_I3C_SERRWARN_S0S1 BIT(11)
#define   SVC_I3C_SERRWARN_OREAD BIT(16)
#define   SVC_I3C_SERRWARN_OWRITE BIT(17)

#define SVC_I3C_SDMACTRL     0x020
#define SVC_I3C_SDATACTRL    0x02C
#define SVC_I3C_SDATACTRL_RXCOUNT(x) FIELD_GET(GENMASK(29, 24), (x))
#define SVC_I3C_SDATACTRL_TXCOUNT(x) FIELD_GET(GENMASK(21, 16), (x))
#define SVC_I3C_SDATACTRL_RXTRIG(x) FIELD_PREP(GENMASK(7, 6), (x))
#define SVC_I3C_SDATACTRL_TXTRIG(x) FIELD_PREP(GENMASK(5, 4), (x))
#define SVC_I3C_SDATACTRL_UNLOCK BIT(3)
#define SVC_I3C_SDATACTRL_FLUSHTB BIT(0)
#define SVC_I3C_SDATACTRL_FLUSHFB BIT(1)

#define SVC_I3C_SWDATAB    0x030
#define SVC_I3C_SWDATABE    0x034
#define SVC_I3C_SWDATAH    0x038
#define SVC_I3C_SWDATAHE    0x03C

#define SVC_I3C_SRDATAB		0x040
#define SVC_I3C_SRDATAH		0x048

#define SVC_I3C_SDYNADDR   0x064
#define SVC_I3C_SDYNADDR_DAVALID BIT(0)

#define SVC_I3C_SMAXLIMITS   0x068
#define SVC_I3C_SIDPARTNO    0x06C
#define SVC_I3C_SIDEXT       0x070
#define SVC_I3C_SIDEXT_BCR(x) FIELD_PREP(GENMASK(23, 16), (x))
#define SVC_I3C_SIDEXT_DCR(x) FIELD_PREP(GENMASK(15, 8), (x))

#define SVC_I3C_SVENDORID    0x074
#define SVC_I3C_STCCLOCK     0x078

#define SVC_I3C_SIBIEXT1     0x140
#define SVC_I3C_SIBIEXT2     0x144

#define SVC_I3C_SLV_VENDORID     0x11B
#define SVC_I3C_SLV_BCR     0x3
#define SVC_I3C_SLV_DCR     0x63
#define SVC_I3C_SLV_PARTNO     0x152A0090
#define SVC_I3C_SLV_STATIC_ADDR     0x48

struct svc_i3c_cmd {
	u8 addr;
	bool rnw;
	u8 *in;
	const void *out;
	unsigned int len;
	unsigned int read_len;
	bool continued;
};

struct svc_i3c_xfer {
	struct list_head node;
	struct completion comp;
	int ret;
	unsigned int type;
	unsigned int ncmds;
	struct svc_i3c_cmd cmds[];
};

// typedef enum
// {
//     I3C_IP_TRANSFER_BYTES      = 0x00U,    /**< Send/read in bytes */
//     I3C_IP_TRANSFER_HALF_WORDS = 0x01U,    /**< Send/read in half-words */
//     I3C_IP_TRANSFER_WORDS      = 0x02U     /**< Send/read in words */
// } I3c_Ip_TransferSizeType;

/**
 * struct svc_i3c_slave - Silvaco I3C slave structure
 * @dev: Corresponding device
 * @regs: Memory mapping
 * @free_slots: Bit array of available slots
 * @addrs: Array containing the dynamic addresses of each attached device
 * @descs: Array of descriptors, one per attached device
 * @hj_work: Hot-join work
 * @ibi_work: IBI work
 * @irq: Main interrupt
 * @pclk: System clock
 * @fclk: Fast clock (bus)
 * @sclk: Slow clock (other events)
 * @xferqueue: Transfer queue structure
 * @xferqueue.list: List member
 * @xferqueue.cur: Current ongoing transfer
 * @xferqueue.lock: Queue lock
 * @ibi: IBI structure
 * @ibi.num_slots: Number of slots available in @ibi.slots
 * @ibi.slots: Available IBI slots
 * @ibi.tbq_slot: To be queued IBI slot
 * @ibi.lock: IBI lock
 * @lock: Transfer lock, protect between IBI work thread and callbacks from slave
 */
struct svc_i3c_slave {
	struct device *dev;
	void __iomem *regs;
	struct i3c_dev_desc desc;
	struct workqueue_struct *slave_queue;
	struct work_struct dynaddr_work;
	int irq;

	// u32 BufferSize;
	// u32 BufferPointer;
	// u16 * DataBuffer;
	// I3c_Ip_TransferSizeType TransferSize;
};

u16 SlaveTxBuffer[64]  = {0x0};

static void svc_i3c_slave_clear_status(struct svc_i3c_slave *slave)
{
	u32 mask = 	 readl(slave->regs + SVC_I3C_SSTATUS);
	mask |=	SVC_I3C_SSTATUS_START      |
                    SVC_I3C_SSTATUS_MATCHED    |
                    SVC_I3C_SSTATUS_STOP       |
                    SVC_I3C_SSTATUS_DACHG      |
                    SVC_I3C_SSTATUS_CCC        |
                    SVC_I3C_SSTATUS_HDRMACHED  |
                    SVC_I3C_SSTATUS_CHANDLED   |
                    SVC_I3C_SSTATUS_EVENT;

	writel(mask, slave->regs + SVC_I3C_SSTATUS);
}

static void svc_i3c_slave_disable_interrupts(struct svc_i3c_slave *slave)
{
	u32 mask = readl(slave->regs + SVC_I3C_SINTSET);

	writel(mask, slave->regs + SVC_I3C_SINTCLR);
}

static void svc_i3c_slave_clear_serrwarn(struct svc_i3c_slave *slave)
{
	/* Clear pending warnings */
	writel(readl(slave->regs + SVC_I3C_SERRWARN),
	       slave->regs + SVC_I3C_SERRWARN);
}

static void svc_i3c_slave_flush_fifo(struct svc_i3c_slave *slave)
{
	/* Flush FIFOs */
	writel(SVC_I3C_SDATACTRL_FLUSHTB | SVC_I3C_SDATACTRL_FLUSHFB,
	       slave->regs + SVC_I3C_SDATACTRL);
}

static void svc_i3c_slave_enable_interrupts(struct svc_i3c_slave *slave)
{
	u32 reg = 0;
	reg |=  SVC_I3C_SINTSET_START |
			SVC_I3C_SINTSET_MATCHED |
			SVC_I3C_SINTSET_STOP |
			SVC_I3C_SINTSET_DACHG |
			SVC_I3C_SINTSET_CCC |
			SVC_I3C_SINTSET_ERRWARN |
			SVC_I3C_SINTSET_HDRMATCHED |
			SVC_I3C_SINTSET_CHANDLED |
			SVC_I3C_SINTSET_EVENT;
	writel(reg, slave->regs + SVC_I3C_SINTSET);
}

static void svc_i3c_slave_config_init(struct svc_i3c_slave *slave)
{
	u32 reg;
	reg = readl(slave->regs + SVC_I3C_SCONFIG);
	reg &= ~(SVC_I3C_SCONFIG_MATCHSS(1) |
			SVC_I3C_SCONFIG_S0IGNORE(1) |
			SVC_I3C_SCONFIG_OFFLINE(1) |
			SVC_I3C_SCONFIG_BAMATCH(0xFF) |
			SVC_I3C_SCONFIG_BTML(0x3));

	reg |= 	SVC_I3C_SCONFIG_MATCHSS(1) |
			SVC_I3C_SCONFIG_S0IGNORE(0) |
			SVC_I3C_SCONFIG_OFFLINE(0) |
			SVC_I3C_SCONFIG_BAMATCH(10) |
			SVC_I3C_SCONFIG_BTML(0) |
			BIT(4);
	writel(reg, slave->regs + SVC_I3C_SCONFIG);
}

static void svc_i3c_slave_ctrl_init(struct svc_i3c_slave *slave)
{
	u32 reg;
	reg = readl(slave->regs + SVC_I3C_SCTRL);
	reg &= ~(SVC_I3C_SCTRL_PENDINT(0xF) |
			SVC_I3C_SCTRL_ACTSTATE(0x3) |
			SVC_I3C_SCTRL_VENDORINFO(0xFF) |
			SVC_I3C_SCTRL_MAPIDX(0xF));
	reg |= SVC_I3C_SCTRL_VENDORINFO(0) |
			SVC_I3C_SCTRL_ACTSTATE(0) |
			SVC_I3C_SCTRL_PENDINT(0) |
			SVC_I3C_SCTRL_MAPIDX(0);
	writel(reg, slave->regs + SVC_I3C_SCTRL);
}

static void svc_i3c_slave_config_tgten(struct svc_i3c_slave *slave)
{
	u32 reg;
	reg = readl(slave->regs + SVC_I3C_SCONFIG);
	reg &= ~SVC_I3C_SCONFIG_SLAVE_EN;
	reg |= SVC_I3C_SCONFIG_SLAVE_EN;
	writel(reg, slave->regs + SVC_I3C_SCONFIG);
}

static void svc_i3c_slave_pid_init(struct svc_i3c_slave *slave)
{
	u32 reg;
	reg = readl(slave->regs + SVC_I3C_SVENDORID);
	reg &= ~0x7ff;
	reg |= SVC_I3C_SLV_VENDORID;
	writel(reg, slave->regs + SVC_I3C_SVENDORID);

	reg = readl(slave->regs + SVC_I3C_SIDEXT);
	reg &= ~0xFFFF00;
	reg |= SVC_I3C_SIDEXT_BCR(SVC_I3C_SLV_BCR) |
			SVC_I3C_SIDEXT_DCR(SVC_I3C_SLV_DCR);
	writel(reg, slave->regs + SVC_I3C_SIDEXT);

	reg = 0;
	reg |= SVC_I3C_SLV_PARTNO;
	writel(reg, slave->regs + SVC_I3C_SIDPARTNO);
}

static void svc_i3c_slave_config_staticaddr(struct svc_i3c_slave *slave)
{
	u32 reg;
	reg = readl(slave->regs + SVC_I3C_SCONFIG);
	reg &= ~SVC_I3C_SCONFIG_SADDR(0x7F);
	reg |= SVC_I3C_SCONFIG_SADDR(SVC_I3C_SLV_STATIC_ADDR);
	writel(reg, slave->regs + SVC_I3C_SCONFIG);
}

static void svc_i3c_slave_reset(struct svc_i3c_slave *slave)
{
	svc_i3c_slave_clear_status(slave);
	svc_i3c_slave_clear_serrwarn(slave);
	svc_i3c_slave_disable_interrupts(slave);
	svc_i3c_slave_flush_fifo(slave);

	writel(0, slave->regs + SVC_I3C_SCONFIG);
	writel(0, slave->regs + SVC_I3C_SCTRL);
	writel(0, slave->regs + SVC_I3C_SDMACTRL);
	writel(0x80000030, slave->regs + SVC_I3C_SDATACTRL);
	writel(0, slave->regs + SVC_I3C_SMAXLIMITS);
	writel(0, slave->regs + SVC_I3C_SIDPARTNO);
	writel(0x660000, slave->regs + SVC_I3C_SIDEXT);
	writel(0x11B, slave->regs + SVC_I3C_SVENDORID);
	writel(0x300A, slave->regs + SVC_I3C_STCCLOCK);
	writel(0x7F, slave->regs + 0x10C);
	writel(0xF1F, slave->regs + 0x110);
	writel(0, slave->regs + 0x120);
	writel(0x70, slave->regs + SVC_I3C_SIBIEXT1);
	writel(0, slave->regs + SVC_I3C_SIBIEXT2);

	svc_i3c_slave_config_init(slave);
	svc_i3c_slave_pid_init(slave);
	svc_i3c_slave_ctrl_init(slave);
	svc_i3c_slave_config_staticaddr(slave);
	svc_i3c_slave_enable_interrupts(slave);
	svc_i3c_slave_config_tgten(slave);
	// pr_err("%s %d +++ SCONFIG 0x%x", __func__, __LINE__, readl(slave->regs + SVC_I3C_SCONFIG));
	// pr_err("%s %d +++ SCTRL 0x%x", __func__, __LINE__, readl(slave->regs + SVC_I3C_SCTRL));
	// pr_err("%s %d +++ SDMACTRL 0x%x", __func__, __LINE__, readl(slave->regs + SVC_I3C_SDMACTRL));
	// pr_err("%s %d +++ SDATACTRL 0x%x", __func__, __LINE__, readl(slave->regs + SVC_I3C_SDATACTRL));
	// pr_err("%s %d +++ SMAXLIMITS 0x%x", __func__, __LINE__, readl(slave->regs + SVC_I3C_SMAXLIMITS));
	// pr_err("%s %d +++ SIDPARTNO 0x%x", __func__, __LINE__, readl(slave->regs + SVC_I3C_SIDPARTNO));
	// pr_err("%s %d +++ SIDEXT 0x%x", __func__, __LINE__, readl(slave->regs + SVC_I3C_SIDEXT));
	// pr_err("%s %d +++ SVENDORID 0x%x", __func__, __LINE__, readl(slave->regs + SVC_I3C_SVENDORID));
}

static void svc_i3c_slave_set_txtrig(struct svc_i3c_slave *slave)
{
	u32 reg = 0;

	reg = readl(slave->regs + SVC_I3C_SDATACTRL);
	reg &= ~SVC_I3C_SDATACTRL_TXTRIG(3);
	reg |= SVC_I3C_SDATACTRL_TXTRIG(3) |
		SVC_I3C_SDATACTRL_UNLOCK;
	writel(reg, slave->regs + SVC_I3C_SDATACTRL);
}

static void svc_i3c_slave_enable_tx(struct svc_i3c_slave *slave)
{
	u32 reg = 0;

	reg = readl(slave->regs + SVC_I3C_SINTSET);
	reg |= SVC_I3C_SINTSET_TXNOTFULL;
	writel(reg, slave->regs + SVC_I3C_SINTSET);
}

static void svc_i3c_slave_disable_tx(struct svc_i3c_slave *slave)
{
	u32 reg = 0;

	reg = SVC_I3C_SINTSET_TXNOTFULL;
	writel(reg, slave->regs + SVC_I3C_SINTCLR);
}

static void svc_i3c_slave_enable_rx(struct svc_i3c_slave *slave)
{
	u32 reg = 0;

	reg = readl(slave->regs + SVC_I3C_SINTSET);
	reg |= SVC_I3C_SINTSET_RXPEND;
	writel(reg, slave->regs + SVC_I3C_SINTSET);
}

static void svc_i3c_slave_flush_rxfifo(struct svc_i3c_slave *slave)
{
	/* Flush FIFOs */
	writel(SVC_I3C_SDATACTRL_FLUSHFB, slave->regs + SVC_I3C_SDATACTRL);
}

// static void svc_i3c_slave_flush_txfifo(struct svc_i3c_slave *slave)
// {
// 	/* Flush FIFOs */
// 	writel(SVC_I3C_SDATACTRL_FLUSHTB, slave->regs + SVC_I3C_SDATACTRL);
// }

static void svc_i3c_slave_rx_recv_handler(struct svc_i3c_slave *slave)
{
	u8 rxCount, rxTotal = 0;
	u8 rxBuf[64];
	int i = 0;

	rxCount = SVC_I3C_SDATACTRL_RXCOUNT(readl(slave->regs + SVC_I3C_SDATACTRL));
	while(rxCount){
		for(i = 0; i < rxCount; i++){
			rxBuf[rxTotal] = readl(slave->regs + SVC_I3C_SRDATAB);
			rxTotal++;
		}
		rxCount = SVC_I3C_SDATACTRL_RXCOUNT(readl(slave->regs + SVC_I3C_SDATACTRL));
	}

	if(rxTotal == 3)
		writel((u32)(rxBuf[2] << 8 | rxBuf[1]), slave->regs + SVC_I3C_SWDATAHE);
	else
		svc_i3c_slave_enable_tx(slave);

	svc_i3c_slave_flush_rxfifo(slave);
}

static void svc_i3c_slave_tx_send_handler(struct svc_i3c_slave *slave)
{
	svc_i3c_slave_disable_tx(slave);
}

static irqreturn_t svc_i3c_slave_irq_handler(int irq, void *dev_id)
{
	struct svc_i3c_slave *slave = (struct svc_i3c_slave *)dev_id;
	u32 active = readl(slave->regs + SVC_I3C_SINTMASKED);
	u32 err_status;

	if(active & SVC_I3C_SSTATUS_ERRWARN){
		err_status = readl(slave->regs + SVC_I3C_SERRWARN);
		writel(err_status, slave->regs + SVC_I3C_SERRWARN);
	}

	if(active & SVC_I3C_SSTATUS_MATCHED){
		writel(SVC_I3C_SSTATUS_MATCHED, slave->regs + SVC_I3C_SSTATUS);
	}

	if(active & SVC_I3C_SSTATUS_RXPEND){
		svc_i3c_slave_rx_recv_handler(slave);
		writel(SVC_I3C_SSTATUS_RXPEND, slave->regs + SVC_I3C_SSTATUS);
		return IRQ_HANDLED;
	}

	if(active & SVC_I3C_SSTATUS_START){
		writel(SVC_I3C_SSTATUS_START, slave->regs + SVC_I3C_SSTATUS);
	}

	if(active & SVC_I3C_SSTATUS_STOP){
		writel(SVC_I3C_SSTATUS_STOP, slave->regs + SVC_I3C_SSTATUS);
	}

	if(active & SVC_I3C_SSTATUS_TXNOTFULL){
		svc_i3c_slave_tx_send_handler(slave);
		writel(SVC_I3C_SSTATUS_TXNOTFULL, slave->regs + SVC_I3C_SSTATUS);
	}

	if(active & SVC_I3C_SSTATUS_DACHG){
		queue_work(slave->slave_queue, &slave->dynaddr_work);
		writel(SVC_I3C_SSTATUS_DACHG, slave->regs + SVC_I3C_SSTATUS);
	}

	writel(active & 0xf6700, slave->regs + SVC_I3C_SSTATUS);
	return IRQ_HANDLED;
}


static void svc_i3c_slave_dynaddr_work(struct work_struct *work)
{
	struct svc_i3c_slave *slave;
	u32 mdynaddr;
	int ret;

	slave = container_of(work, struct svc_i3c_slave, dynaddr_work);
	ret = readl_poll_timeout(slave->regs + SVC_I3C_SDYNADDR,
					 mdynaddr,
					 (mdynaddr & SVC_I3C_SDYNADDR_DAVALID),
					 0, 1000);
	if (ret){
		dev_err(slave->dev, "%s %d +++ mdynaddr 0x%x", __func__, __LINE__, mdynaddr);
		return;
	}
	
	mdelay(500);
	//dev_err(slave->dev, "%s %d +++ mdynaddr 0x%x", __func__, __LINE__, mdynaddr);
	svc_i3c_slave_enable_rx(slave);
	svc_i3c_slave_set_txtrig(slave);
}


static int svc_i3c_slave_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct svc_i3c_slave *slave;
	int ret;
	// int i = 0;

	//pr_err("%s %d +++", __func__, __LINE__);
	slave = devm_kzalloc(dev, sizeof(*slave), GFP_KERNEL);
	if (!slave)
		return -ENOMEM;

	slave->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(slave->regs))
		return PTR_ERR(slave->regs);

	// slave->pclk = devm_clk_get(dev, "pclk");
	// if (IS_ERR(slave->pclk))
	// 	return PTR_ERR(slave->pclk);

	// slave->fclk = devm_clk_get(dev, "fast_clk");
	// if (IS_ERR(slave->fclk))
	// 	return PTR_ERR(slave->fclk);

	// slave->sclk = devm_clk_get(dev, "slow_clk");
	// if (IS_ERR(slave->sclk))
	// 	return PTR_ERR(slave->sclk);

	slave->irq = platform_get_irq(pdev, 0);
	if (slave->irq <= 0)
		return -ENOENT;
	slave->dev = dev;

	// ret = svc_i3c_slave_prepare_clks(slave);
	// if (ret)
	// 	return ret;

	slave->slave_queue = create_workqueue("slave_dynaddr_queue");
	INIT_WORK(&slave->dynaddr_work, svc_i3c_slave_dynaddr_work);

	ret = devm_request_irq(dev, slave->irq, svc_i3c_slave_irq_handler,
			       IRQF_NO_SUSPEND, "svc-i3c-sirq", slave);

	platform_set_drvdata(pdev, slave);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 1000);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	svc_i3c_slave_reset(slave);

	pm_runtime_mark_last_busy(&pdev->dev);
	pm_runtime_put_autosuspend(&pdev->dev);
	return 0;
}

static int svc_i3c_slave_remove(struct platform_device *pdev)
{
	struct svc_i3c_slave *slave = platform_get_drvdata(pdev);
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	cancel_work_sync(&slave->dynaddr_work);
	return 0;
}


static const struct of_device_id svc_i3c_slave_of_match_tbl[] = {
	{ .compatible = "silvaco,i3c-slave" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, svc_i3c_slave_of_match_tbl);

static struct platform_driver svc_i3c_slave = {
	.probe = svc_i3c_slave_probe,
	.remove = svc_i3c_slave_remove,
	.driver = {
		.name = "silvaco-i3c-slave",
		.of_match_table = svc_i3c_slave_of_match_tbl,
		// .pm = &svc_i3c_pm_ops,
	},
};

module_platform_driver(svc_i3c_slave);

MODULE_DESCRIPTION("Silvaco dual-role I3C slave driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
