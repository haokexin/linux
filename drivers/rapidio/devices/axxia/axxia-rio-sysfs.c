// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation

/* #define DEBUG */
/* #define IO_OPERATIONS */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#include "axxia-rio.h"
#include "axxia-rio-irq.h"

static ssize_t stat_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	char *str = buf;
	u32 reg_val = 0;

	if (priv->devid  == AXXIA_DEVID_AXM55XX) {
		str += sprintf(str, "AXM 55xx sRIO Controller");
		switch (priv->devrev) {
		case AXXIA_DEVREV_AXM55XX_V1_0:
			str += sprintf(str, "Revision 0\n");
			break;
		case AXXIA_DEVREV_AXM55XX_V1_1:
			str += sprintf(str, "Revision 1\n");
			break;
		case AXXIA_DEVREV_AXM55XX_V1_2:
			str += sprintf(str, "Revision 2\n");
			break;
		default:
			str += sprintf(str, "Revision Unknown\n");
			break;
		}
	}

	axxia_rio_port_get_state(mport, 0);
	str += sprintf(str, "Master Port state:\n");
	alcr(priv, RIO_ESCSR(priv->port_ndx), &reg_val);
	str += sprintf(str, "ESCSR (0x158) : 0x%08x\n", reg_val);
	return str - buf;
}
static DEVICE_ATTR_RO(stat);

static ssize_t misc_stat_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	char *str = buf;

	str += sprintf(str, "RIO PIO Stat:\n");
	str += sprintf(str, "\t Successful Count: %d\n",
		       priv->rpio_compl_count);
	str += sprintf(str, "\t Failed Count    : %d\n",
		       priv->rpio_compl_count);

	str += sprintf(str, "AXI PIO Stat:\n");
	str += sprintf(str, "\t Successful Count: %d\n",
		       priv->apio_compl_count);
	str += sprintf(str, "\t Failed Count    : %d\n",
		       priv->apio_compl_count);

	str += sprintf(str, "Port Write Stat:\n");
	str += sprintf(str, "\t Interrupt Count : %d\n", priv->rio_pw_count);
	str += sprintf(str, "\t Message Count   : %d\n",
		       priv->rio_pw_msg_count);

	return str - buf;
}
static DEVICE_ATTR_RO(misc_stat);

static ssize_t ib_dme_stat_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	char *str = buf;
	int e, j;
	struct rio_rx_mbox *mb;
	struct rio_msg_dme *me;

	str += sprintf(str, "Inbound Mailbox (DME) counters:\n");
	for (e = 0; e < RIO_MAX_RX_MBOX; e++) {
		mb = priv->ib_mbox[e];
		if (mb) {
			for (j = 0; j < RIO_MSG_MAX_LETTER; j++) {
				me = mb->me[j];
				str += sprintf(str,
					       "Mbox %d Letter %d DME %d\n",
					       mb->mbox_no, j, me->dme_no);
				str += sprintf(str,
					       "\tNumber of Desc Done  : %d\n",
					       me->desc_done_count);
				str += sprintf(str,
					       "\tNumber of Desc Errors: %d\n",
					       me->desc_error_count);
				str += sprintf(str,
					       "\t\tRIO Error    : %d\n",
					       me->desc_rio_err_count);
				str += sprintf(str,
					       "\t\tAXI Error    : %d\n",
					       me->desc_axi_err_count);
				str += sprintf(str,
					       "\t\tTimeout Error: %d\n",
					       me->desc_tmo_err_count);
			}
		}
	}
	return str - buf;
}
static DEVICE_ATTR_RO(ib_dme_stat);

static ssize_t ob_dme_stat_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	char *str = buf;
	int e;
	struct rio_msg_dme *me;
	struct rio_tx_mbox *mb;

	str += sprintf(str, "Outbound Message Engine Counters:\n");
	for (e = 0; e < DME_MAX_OB_ENGINES; e++) {
		me = priv->ob_dme_shared[e].me;
		if (me) {
			str += sprintf(str, "DME %d Enabled\n", e);
			str += sprintf(str, "\tNumber of Desc Done  : %d\n",
				       me->desc_done_count);
			str += sprintf(str, "\tNumber of Desc Errors: %d\n",
				       me->desc_error_count);
			str += sprintf(str, "\t\tRIO Error    : %d\n",
				       me->desc_rio_err_count);
			str += sprintf(str, "\t\tAXI Error    : %d\n",
				       me->desc_axi_err_count);
			str += sprintf(str, "\t\tTimeout Error: %d\n",
				       me->desc_tmo_err_count);
		} else {
			str += sprintf(str, "DME %d Disabled\n", e);
		}
	}
	str += sprintf(str, "*********************************\n");
	str += sprintf(str, "Outbound Mbox stats\n");
	for (e = 0; e < RIO_MAX_TX_MBOX; e++) {
		mb = priv->ob_mbox[e];
		if (!mb)
			continue;
		if (mb->sent_msg_count || mb->compl_msg_count) {
			if (test_bit(RIO_DME_OPEN, &mb->state))
				str += sprintf(str, "Mailbox %d: DME %d\n",
					       e, mb->dme_no);
			else
				str += sprintf(str, "Mailbox %d : Closed\n",
					       e);
			str += sprintf(str, "\tMessages sent     : %d\n",
				       mb->sent_msg_count);
			str += sprintf(str, "\tMessages Completed: %d\n",
				       mb->compl_msg_count);
		}
	}

	return str - buf;
}
static DEVICE_ATTR_RO(ob_dme_stat);

static ssize_t irq_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	u32 stat;
	char *str = buf;

	str += sprintf(str, "Interrupt enable bits:\n");
	alcr(priv, RAB_INTR_ENAB_GNRL, &stat);
	str += sprintf(str, "General Interrupt Enable (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_GNRL, stat);
	alcr(priv, RAB_INTR_ENAB_ODME, &stat);
	str += sprintf(str, "Outbound Message Engine  (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_ODME, stat);
	alcr(priv, RAB_INTR_ENAB_IDME, &stat);
	str += sprintf(str, "Inbound Message Engine   (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_IDME, stat);
	alcr(priv, RAB_INTR_ENAB_MISC, &stat);
	str += sprintf(str, "Miscellaneous Events     (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_MISC, stat);
	alcr(priv, RAB_INTR_ENAB_APIO, &stat);
	str += sprintf(str, "Axxia Bus to RIO Events  (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_APIO, stat);
	alcr(priv, RAB_INTR_ENAB_RPIO, &stat);
	str += sprintf(str, "RIO to Axxia Bus Events  (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_RPIO, stat);

	str += sprintf(str, "OBDME : in Timer Mode, Period %9.9d nanosecond\n",
		       axxia_hrtimer_delay);
	str += sprintf(str, "IBDME : ");
	if (priv->dme_mode == AXXIA_IBDME_TIMER_MODE)
		str += sprintf(str, "in Timer Mode, Period %9.9d nanosecond\n",
			       axxia_hrtimer_delay);
	else
		str += sprintf(str, "in Interrupt Mode\n");
	return str - buf;
}
static DEVICE_ATTR_RO(irq);

static ssize_t tmo_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	u32 stat;
	char *str = buf;

	str += sprintf(str, "Port Link Timeout Control Registers:\n");
	alcr(priv, RIO_PLTOCCSR, &stat);
	str += sprintf(str, "PLTOCCSR (%p)\t%8.8x\n",
		       (void *)RIO_PLTOCCSR, stat);
	alcr(priv, RIO_PRTOCCSR, &stat);
	str += sprintf(str, "PRTOCCSR (%p)\t%8.8x\n",
		       (void *)RIO_PRTOCCSR, stat);
	alcr(priv, RAB_STAT, &stat);
	str += sprintf(str, "RAB_STAT (%p)\t%8.8x\n",
		       (void *)RAB_STAT, stat);
	alcr(priv, RAB_APIO_STAT, &stat);
	str += sprintf(str, "RAB_APIO_STAT (%p)\t%8.8x\n",
		       (void *)RAB_APIO_STAT, stat);
	alcr(priv, RIO_ESCSR(priv->port_ndx), &stat);
	str += sprintf(str, "PNESCSR (%d)\t%8.8x\n",
		       RIO_ESCSR(priv->port_ndx), stat);

	return str - buf;
}
static DEVICE_ATTR_RO(tmo);

static ssize_t dme_log_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	u32 stat, log;
	char *str = buf;

	alcr(priv, RAB_INTR_STAT_MISC, &stat);
	log = (stat & UNEXP_MSG_LOG) >> 24;
	str += sprintf(str, "mbox[1:0]   %x\n", (log & 0xc0) >> 6);
	str += sprintf(str, "letter[1:0] %x\n", (log & 0x30) >> 4);
	str += sprintf(str, "xmbox[3:0] %x\n", log & 0x0f);

	return str - buf;
}
static DEVICE_ATTR_RO(dme_log);

static struct attribute *rio_attributes[] = {
	&dev_attr_stat.attr,
	&dev_attr_irq.attr,
	&dev_attr_misc_stat.attr,
	&dev_attr_ob_dme_stat.attr,
	&dev_attr_ib_dme_stat.attr,
	&dev_attr_tmo.attr,
	&dev_attr_dme_log.attr,
	NULL
};

static struct attribute_group rio_attribute_group = {
	.name = NULL,
	.attrs = rio_attributes,
};

int axxia_rio_init_sysfs(struct platform_device *dev)
{
	return sysfs_create_group(&dev->dev.kobj, &rio_attribute_group);
}

void axxia_rio_release_sysfs(struct platform_device *dev)
{
	sysfs_remove_group(&dev->dev.kobj, &rio_attribute_group);
}
