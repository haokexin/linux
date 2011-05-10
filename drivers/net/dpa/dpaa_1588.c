/*
 * drivers/net/dpa/dpaa_1588.c
 *
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 * Copyright (C) 2009 IXXAT Automation, GmbH
 *
 * DPAA Ethernet Driver -- IEEE 1588 interface functionality
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
#include <linux/io.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <asm/div64.h>
#include "dpaa_eth.h"
#include "dpaa_1588.h"

static DECLARE_WAIT_QUEUE_HEAD(ptp_rx_ts_wait);
static DECLARE_WAIT_QUEUE_HEAD(ptp_tx_ts_wait);
#define PTP_GET_RX_TIMEOUT      (HZ/10)
#define PTP_GET_TX_TIMEOUT      (HZ/10)

static int dpa_ptp_init_circ(struct circ_buf *ptp_buf, int size)
{
	ptp_buf->buf = vmalloc(sizeof(struct dpa_ptp_data_t) * size);

	if (!ptp_buf->buf)
		return 1;
	ptp_buf->head = 0;
	ptp_buf->tail = 0;

	return 0;
}

static void dpa_ptp_reset_circ(struct circ_buf *ptp_buf)
{
	ptp_buf->head = 0;
	ptp_buf->tail = 0;
}

static int dpa_ptp_insert(struct circ_buf *ptp_buf,
			  struct dpa_ptp_data_t *data,
			  struct ptp_tsu *tsu,
			  int size)
{
	struct dpa_ptp_data_t *tmp;
	unsigned long flags;
	int head, tail;

	spin_lock_irqsave(&tsu->ptp_lock, flags);

	head = ptp_buf->head;
	tail = ptp_buf->tail;

	if (CIRC_SPACE(head, tail, size) <= 0) {
		spin_unlock_irqrestore(&tsu->ptp_lock, flags);
		return 1;
	}

	tmp = (struct dpa_ptp_data_t *)(ptp_buf->buf) + head;
	tmp->key = data->key;
	memcpy(tmp->spid, data->spid, 10);
	tmp->ts_time.high = data->ts_time.high;
	tmp->ts_time.low = data->ts_time.low;

	ptp_buf->head = (head + 1) & (size - 1);

	spin_unlock_irqrestore(&tsu->ptp_lock, flags);

	return 0;
}

static int dpa_ptp_find_and_remove(struct circ_buf *ptp_buf,
				   struct dpa_ptp_data_t *data,
				   struct ptp_tsu *tsu,
				   int size)
{
	int head, tail, idx;
	unsigned long flags;
	struct dpa_ptp_data_t *tmp;

	spin_lock_irqsave(&tsu->ptp_lock, flags);

	head = ptp_buf->head;
	tail = idx = ptp_buf->tail;

	if (CIRC_CNT_TO_END(head, tail, size) == 0) {
		spin_unlock_irqrestore(&tsu->ptp_lock, flags);
		return 1;
	}

	while (idx != head) {
		tmp = (struct dpa_ptp_data_t *)(ptp_buf->buf) + idx;
		if (tmp->key == data->key &&
				!memcmp(tmp->spid, data->spid, 10))
			break;
		idx = (idx + 1) & (size - 1);
	}

	if (idx == head) {
		ptp_buf->tail = head;
		spin_unlock_irqrestore(&tsu->ptp_lock, flags);
		return 1;
	}

	data->ts_time.high = tmp->ts_time.high;
	data->ts_time.low = tmp->ts_time.low;

	ptp_buf->tail = (idx + 1) & (size - 1);

	spin_unlock_irqrestore(&tsu->ptp_lock, flags);

	return 0;
}

int dpa_ptp_do_txstamp(struct sk_buff *skb)
{
	struct iphdr *iph;
	struct udphdr *udph;

	if (skb->len > 44) {
		iph = ip_hdr(skb);
		if (iph == NULL || iph->protocol != IPPROTO_UDP)
			return 0;

		udph = udp_hdr(skb);
		if (udph != NULL && ntohs(udph->dest) == 319)
			return 1;
	}

	return 0;
}

static int dpa_ptp_get_time(const struct qm_fd *fd, u32 *high, u32 *low)
{
	dma_addr_t addr = qm_fd_addr(fd);
	u8 *ts_addr = (u8 *)phys_to_virt(addr);
	u32 sec, nsec, mod;
	u64 tmp;

	ts_addr += DPA_PTP_TIMESTAMP_OFFSET;
	sec = *((u32 *)ts_addr);
	nsec = *(((u32 *)ts_addr) + 1);
	tmp = ((u64)sec << 32 | nsec) * DPA_PTP_NOMINAL_FREQ_PERIOD;

	mod = do_div(tmp, NANOSEC_PER_SECOND);
	*high = (u32)tmp;
	*low = mod;

	return 0;
}

void dpa_ptp_store_txstamp(struct net_device *dev, struct sk_buff *skb,
			   const struct qm_fd *fd)
{
	struct dpa_priv_s *priv = netdev_priv(dev);
	struct ptp_tsu *tsu = priv->tsu;
	int msg_type, seq_id, control;
	struct dpa_ptp_data_t tmp_tx_time;
	unsigned char *sp_id;
	unsigned short portnum;
	u32 high, low;

	seq_id = *((u16 *)(skb->data + PTP_SEQ_ID_OFFS));
	control = *((u8 *)(skb->data + PTP_CTRL_OFFS));
	sp_id = skb->data + PTP_SPORT_ID_OFFS;
	portnum = ntohs(*((unsigned short *)(sp_id + 8)));

	tmp_tx_time.key = ntohs(seq_id);
	memcpy(tmp_tx_time.spid, sp_id, 8);
	memcpy(tmp_tx_time.spid + 8, (unsigned char *)&portnum, 2);
	dpa_ptp_get_time(fd, &high, &low);
	tmp_tx_time.ts_time.high = (u64)high;
	tmp_tx_time.ts_time.low = low;

	switch (control) {
	case PTP_MSG_SYNC:
		dpa_ptp_insert(&(tsu->tx_time_sync), &tmp_tx_time, tsu,
				tsu->tx_time_queue_size);
		break;

	case PTP_MSG_DEL_REQ:
		dpa_ptp_insert(&(tsu->tx_time_del_req), &tmp_tx_time, tsu,
				tsu->tx_time_queue_size);
		break;

	case PTP_MSG_ALL_OTHER:
		msg_type = (*((u8 *)(skb->data + PTP_MSG_TYPE_OFFS))) & 0x0F;
		switch (msg_type) {
		case PTP_MSG_P_DEL_REQ:
			dpa_ptp_insert(&(tsu->tx_time_pdel_req), &tmp_tx_time,
					tsu, tsu->tx_time_queue_size);
			break;
		case PTP_MSG_P_DEL_RESP:
			dpa_ptp_insert(&(tsu->tx_time_pdel_resp), &tmp_tx_time,
					tsu, tsu->tx_time_queue_size);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	wake_up_interruptible(&ptp_tx_ts_wait);
}

void dpa_ptp_store_rxstamp(struct net_device *dev, struct sk_buff *skb,
			   const struct qm_fd *fd)
{
	struct dpa_priv_s *priv = netdev_priv(dev);
	struct ptp_tsu *tsu = priv->tsu;
	int msg_type, seq_id, control;
	struct dpa_ptp_data_t tmp_rx_time;
	struct iphdr *iph;
	struct udphdr *udph;
	unsigned char *sp_id;
	unsigned short portnum;
	u32 high, low;

	/* Check for UDP, and Check if port is 319 for PTP Event */
	iph = (struct iphdr *)(skb->data + PTP_IP_OFFS);
	if (iph->protocol != IPPROTO_UDP)
		return;

	udph = (struct udphdr *)(skb->data + PTP_UDP_OFFS);
	if (ntohs(udph->dest) != 319)
		return;

	seq_id = *((u16 *)(skb->data + PTP_SEQ_ID_OFFS));
	control = *((u8 *)(skb->data + PTP_CTRL_OFFS));
	sp_id = skb->data + PTP_SPORT_ID_OFFS;
	portnum = ntohs(*((unsigned short *)(sp_id + 8)));

	tmp_rx_time.key = ntohs(seq_id);
	memcpy(tmp_rx_time.spid, sp_id, 8);
	memcpy(tmp_rx_time.spid + 8, (unsigned char *)&portnum, 2);
	dpa_ptp_get_time(fd, &high, &low);
	tmp_rx_time.ts_time.high = (u64)high;
	tmp_rx_time.ts_time.low = low;

	switch (control) {

	case PTP_MSG_SYNC:
		dpa_ptp_insert(&(tsu->rx_time_sync), &tmp_rx_time, tsu,
				tsu->rx_time_queue_size);
		break;

	case PTP_MSG_DEL_REQ:
		dpa_ptp_insert(&(tsu->rx_time_del_req), &tmp_rx_time, tsu,
				tsu->rx_time_queue_size);
		break;

	case PTP_MSG_ALL_OTHER:
		msg_type = (*((u8 *)(skb->data + PTP_MSG_TYPE_OFFS))) & 0x0F;
		switch (msg_type) {
		case PTP_MSG_P_DEL_REQ:
			dpa_ptp_insert(&(tsu->rx_time_pdel_req), &tmp_rx_time,
					tsu, tsu->rx_time_queue_size);
			break;
		case PTP_MSG_P_DEL_RESP:
			dpa_ptp_insert(&(tsu->rx_time_pdel_resp), &tmp_rx_time,
					tsu, tsu->rx_time_queue_size);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	wake_up_interruptible(&ptp_rx_ts_wait);
}

static uint8_t dpa_get_tx_timestamp(struct ptp_tsu *ptp_tsu,
				    struct ptp_ts_data *pts,
				    struct ptp_time *tx_time)
{
	struct ptp_tsu *tsu = ptp_tsu;
	struct dpa_ptp_data_t tmp;
	int flag;
	u8 mode;

	tmp.key = pts->seq_id;
	memcpy(tmp.spid, pts->spid, 10);

	mode = pts->message_type;
	switch (mode) {
	case PTP_MSG_SYNC:
		flag = dpa_ptp_find_and_remove(&(tsu->tx_time_sync), &tmp,
				tsu, tsu->tx_time_queue_size);
		break;
	case PTP_MSG_DEL_REQ:
		flag = dpa_ptp_find_and_remove(&(tsu->tx_time_del_req), &tmp,
				tsu, tsu->tx_time_queue_size);
		break;

	case PTP_MSG_P_DEL_REQ:
		flag = dpa_ptp_find_and_remove(&(tsu->tx_time_pdel_req), &tmp,
				tsu, tsu->tx_time_queue_size);
		break;
	case PTP_MSG_P_DEL_RESP:
		flag = dpa_ptp_find_and_remove(&(tsu->tx_time_pdel_resp), &tmp,
				tsu, tsu->tx_time_queue_size);
		break;

	default:
		flag = 1;
		printk(KERN_ERR "ERROR\n");
		break;
	}

	if (!flag) {
		tx_time->high = tmp.ts_time.high;
		tx_time->low = tmp.ts_time.low;
		return 0;
	} else {
		wait_event_interruptible_timeout(ptp_tx_ts_wait, 0,
					PTP_GET_TX_TIMEOUT);

		switch (mode) {
		case PTP_MSG_SYNC:
			flag = dpa_ptp_find_and_remove(&(tsu->tx_time_sync),
					&tmp, tsu, tsu->tx_time_queue_size);
			break;
		case PTP_MSG_DEL_REQ:
			flag = dpa_ptp_find_and_remove(&(tsu->tx_time_del_req),
					&tmp, tsu, tsu->tx_time_queue_size);
			break;
		case PTP_MSG_P_DEL_REQ:
			flag = dpa_ptp_find_and_remove(
					&(tsu->tx_time_pdel_req), &tmp, tsu,
					tsu->tx_time_queue_size);
			break;
		case PTP_MSG_P_DEL_RESP:
			flag = dpa_ptp_find_and_remove(
					&(tsu->tx_time_pdel_resp), &tmp, tsu,
					tsu->tx_time_queue_size);
			break;
		}

		if (flag == 0) {
			tx_time->high = tmp.ts_time.high;
			tx_time->low = tmp.ts_time.low;
			return 0;
		}

		return -1;
	}
}

static uint8_t dpa_get_rx_timestamp(struct ptp_tsu *ptp_tsu,
				    struct ptp_ts_data *pts,
				    struct ptp_time *rx_time)
{
	struct ptp_tsu *tsu = ptp_tsu;
	struct dpa_ptp_data_t tmp;
	int flag;
	u8 mode;

	tmp.key = pts->seq_id;
	memcpy(tmp.spid, pts->spid, 10);

	mode = pts->message_type;
	switch (mode) {
	case PTP_MSG_SYNC:
		flag = dpa_ptp_find_and_remove(&(tsu->rx_time_sync), &tmp,
				tsu, tsu->rx_time_queue_size);
		break;
	case PTP_MSG_DEL_REQ:
		flag = dpa_ptp_find_and_remove(&(tsu->rx_time_del_req), &tmp,
				tsu, tsu->rx_time_queue_size);
		break;

	case PTP_MSG_P_DEL_REQ:
		flag = dpa_ptp_find_and_remove(&(tsu->rx_time_pdel_req), &tmp,
				tsu, tsu->rx_time_queue_size);
		break;
	case PTP_MSG_P_DEL_RESP:
		flag = dpa_ptp_find_and_remove(&(tsu->rx_time_pdel_resp), &tmp,
				tsu, tsu->rx_time_queue_size);
		break;

	default:
		flag = 1;
		printk(KERN_ERR "ERROR\n");
		break;
	}

	if (!flag) {
		rx_time->high = tmp.ts_time.high;
		rx_time->low = tmp.ts_time.low;
		return 0;
	} else {
		wait_event_interruptible_timeout(ptp_rx_ts_wait, 0,
					PTP_GET_RX_TIMEOUT);

		switch (mode) {
		case PTP_MSG_SYNC:
			flag = dpa_ptp_find_and_remove(&(tsu->rx_time_sync),
				&tmp, tsu, tsu->rx_time_queue_size);
			break;
		case PTP_MSG_DEL_REQ:
			flag = dpa_ptp_find_and_remove(
				&(tsu->rx_time_del_req), &tmp,
				tsu, tsu->rx_time_queue_size);
			break;
		case PTP_MSG_P_DEL_REQ:
			flag = dpa_ptp_find_and_remove(
				&(tsu->rx_time_pdel_req), &tmp,
				tsu, tsu->rx_time_queue_size);
			break;
		case PTP_MSG_P_DEL_RESP:
			flag = dpa_ptp_find_and_remove(
				&(tsu->rx_time_pdel_resp), &tmp,
				tsu, tsu->rx_time_queue_size);
			break;
		}

		if (flag == 0) {
			rx_time->high = tmp.ts_time.high;
			rx_time->low = tmp.ts_time.low;
			return 0;
		}

		return -1;
	}
}

static void dpa_flush_timestamp(struct ptp_tsu *tsu)
{
	dpa_ptp_reset_circ(&tsu->rx_time_sync);
	dpa_ptp_reset_circ(&tsu->tx_time_sync);

	dpa_ptp_reset_circ(&tsu->rx_time_del_req);
	dpa_ptp_reset_circ(&tsu->tx_time_del_req);

	dpa_ptp_reset_circ(&tsu->rx_time_pdel_req);
	dpa_ptp_reset_circ(&tsu->tx_time_pdel_req);

	dpa_ptp_reset_circ(&tsu->rx_time_pdel_resp);
	dpa_ptp_reset_circ(&tsu->tx_time_pdel_resp);
}

static void dpa_get_curr_cnt(struct ptp_tsu *tsu, struct ptp_time *curr_time)
{
	struct mac_device *mac_dev = tsu->dpa_priv->mac_dev;
	u64 tmp;
	u32 mod;

	if (mac_dev->fm_rtc_get_cnt)
		mac_dev->fm_rtc_get_cnt(tsu->dpa_priv->net_dev, &tmp);

	mod = do_div(tmp, NANOSEC_PER_SECOND);
	curr_time->high = (u32)tmp;
	curr_time->low = mod;
}

static void dpa_set_1588cnt(struct ptp_tsu *tsu, struct ptp_time *cnt_time)
{
	struct mac_device *mac_dev = tsu->dpa_priv->mac_dev;
	u64 tmp;

	tmp = (u64)cnt_time->high * NANOSEC_PER_SECOND + (u64)cnt_time->low;

	if (mac_dev->fm_rtc_set_cnt)
		mac_dev->fm_rtc_set_cnt(tsu->dpa_priv->net_dev, tmp);
}

static void dpa_get_drift(struct ptp_tsu *tsu, struct ptp_get_comp *comp)
{
	struct mac_device *mac_dev = tsu->dpa_priv->mac_dev;
	u32 drift;

	if (mac_dev->fm_rtc_get_drift)
		mac_dev->fm_rtc_get_drift(tsu->dpa_priv->net_dev, &drift);

	comp->dw_origcomp = drift;
}

static void dpa_set_drift(struct ptp_tsu *tsu, struct ptp_set_comp *comp)
{
	struct mac_device *mac_dev = tsu->dpa_priv->mac_dev;
	u32 drift = comp->freq_compensation;

	if (mac_dev->fm_rtc_set_drift)
		mac_dev->fm_rtc_set_drift(tsu->dpa_priv->net_dev, drift);
}

static int ptp_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		     unsigned long arg)
{
	struct net_device *dev = __dev_get_by_name(&init_net, "eth0");
	struct dpa_priv_s *priv = netdev_priv(dev);
	struct ptp_tsu *tsu = priv->tsu;
	struct ptp_ts_data *p_ts;
	struct ptp_time rx_time, tx_time, curr_time;
	struct ptp_time *cnt;
	struct ptp_set_comp *set_comp;
	struct ptp_get_comp *get_comp;
	int retval = 0;

	switch (cmd) {
	case PTP_GET_RX_TIMESTAMP:
		p_ts = (struct ptp_ts_data *)arg;
		retval = dpa_get_rx_timestamp(tsu, p_ts, &rx_time);
		if (retval == 0)
			copy_to_user((void __user *)(&(p_ts->ts)), &rx_time,
					sizeof(rx_time));
		break;
	case PTP_GET_TX_TIMESTAMP:
		p_ts = (struct ptp_ts_data *)arg;
		dpa_get_tx_timestamp(tsu, p_ts, &tx_time);
		copy_to_user((void __user *)(&(p_ts->ts)), &tx_time,
				sizeof(tx_time));
		break;
	case PTP_GET_CURRENT_TIME:
		dpa_get_curr_cnt(tsu, &curr_time);
		copy_to_user((void __user *)arg, &curr_time, sizeof(curr_time));
		break;
	case PTP_SET_RTC_TIME:
		cnt = (struct ptp_time *)arg;
		dpa_set_1588cnt(tsu, cnt);
		break;
	case PTP_FLUSH_TIMESTAMP:
		dpa_flush_timestamp(tsu);
		break;
	case PTP_SET_COMPENSATION:
		set_comp = (struct ptp_set_comp *)arg;
		dpa_set_drift(tsu, set_comp);
		break;
	case PTP_GET_ORIG_COMP:
		get_comp = (struct ptp_get_comp *)arg;
		dpa_get_drift(tsu, get_comp);
		break;
	default:
		return -EINVAL;
	}

	return retval;
}

static const struct file_operations ptp_fops = {
	.owner	= THIS_MODULE,
	.ioctl	= ptp_ioctl,
};

int dpa_ptp_init(struct dpa_priv_s *priv)
{
	struct ptp_tsu *tsu;

	if (register_chrdev(PTP_MAJOR, "ptp", &ptp_fops)) {
		printk(KERN_ERR "Unable to register PTP device as char\n");
		return 1;
	} else {
		printk(KERN_INFO "Register PTP device as char /dev/ptp\n");
	}

	/* Allocate memory for PTP structure */
	tsu = kzalloc(sizeof(struct ptp_tsu), GFP_KERNEL);
	if (!tsu)
		return -ENOMEM;

	memset(tsu, 0, sizeof(*tsu));
	tsu->valid = TRUE;
	tsu->dpa_priv = priv;
	tsu->rx_time_queue_size = DEFAULT_RX_QUEUE_SIZE;
	tsu->tx_time_queue_size = DEFAULT_TX_QUEUE_SIZE;

	dpa_ptp_init_circ(&(tsu->rx_time_sync), tsu->rx_time_queue_size);
	dpa_ptp_init_circ(&(tsu->rx_time_del_req), tsu->rx_time_queue_size);
	dpa_ptp_init_circ(&(tsu->rx_time_pdel_req), tsu->rx_time_queue_size);
	dpa_ptp_init_circ(&(tsu->rx_time_pdel_resp), tsu->rx_time_queue_size);
	dpa_ptp_init_circ(&(tsu->tx_time_sync), tsu->tx_time_queue_size);
	dpa_ptp_init_circ(&(tsu->tx_time_del_req), tsu->tx_time_queue_size);
	dpa_ptp_init_circ(&(tsu->tx_time_pdel_req), tsu->tx_time_queue_size);
	dpa_ptp_init_circ(&(tsu->tx_time_pdel_resp), tsu->tx_time_queue_size);

	spin_lock_init(&tsu->ptp_lock);

	priv->tsu = tsu;

	return 0;
}
EXPORT_SYMBOL(dpa_ptp_init);

void dpa_ptp_cleanup(struct dpa_priv_s *priv)
{
	struct ptp_tsu *tsu = priv->tsu;

	tsu->valid = FALSE;
	vfree(tsu->rx_time_sync.buf);
	vfree(tsu->rx_time_del_req.buf);
	vfree(tsu->rx_time_pdel_req.buf);
	vfree(tsu->rx_time_pdel_resp.buf);
	vfree(tsu->tx_time_sync.buf);
	vfree(tsu->tx_time_del_req.buf);
	vfree(tsu->tx_time_pdel_req.buf);
	vfree(tsu->tx_time_pdel_resp.buf);
	kfree(tsu);

	/* Unregister the PTP device */
	unregister_chrdev(PTP_MAJOR, "ptp");
}
EXPORT_SYMBOL(dpa_ptp_cleanup);

static int __init __cold dpa_ptp_load(void)
{
	return 0;
}
module_init(dpa_ptp_load);

static void __exit __cold dpa_ptp_unload(void)
{
}
module_exit(dpa_ptp_unload);
