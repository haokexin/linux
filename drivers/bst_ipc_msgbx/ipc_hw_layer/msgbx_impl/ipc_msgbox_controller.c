// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/cpuhotplug.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/mailbox_controller.h>
#include <linux/of_reserved_mem.h>
#include <linux/mailbox_client.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/timekeeping.h>
#include <bst/ipc_trans_layer.h>
#include <bst/ipc_hw_layer.h>
#include <bst/ipc_hw_impl.h>
#include "../../ipc_trans_layer/src/ipc_trans_runtime.h"
#include "ipc_msgbox_controller.h"
/********************* extern global variables *******************/
ST_MSGBX_END_PARA msg_end_para[MAX_END_NUM] = {0};
EXPORT_SYMBOL(msg_end_para);
struct ipc_msgbox *g_ipc_msgbx;
EXPORT_SYMBOL(g_ipc_msgbx);
libipc_hw_compat_ops_t ipc_hw_ops;
EXPORT_SYMBOL(ipc_hw_ops);
/********************* local variables ***************************/
static ST_MSG_INF st_msgbx_info[MAX_END_NUM];
#if defined(MSGBX_HW_TYPE_C1200)
static char *irq_desc[MAX_END_NUM][CHANNEL_COUNT];
#endif
static spinlock_t tx_msg_spinlock[MAX_END_NUM];

// define for smp call
static ST_MSG_HW_INIT para_hw_init[MAX_END_NUM];
static ST_MSG_HW_DEINIT para_hw_deinit[MAX_END_NUM];
static ST_MSG_HW_SET_FLT_CFG para_hw_set_flt_cfg[MAX_END_NUM];
static ST_MSG_HW_SET_FLT_CFG para_hw_get_flt_cfg[MAX_END_NUM];
static ST_MSG_HW_CLR_FLT_CFG para_hw_clr_flt_cfg[MAX_END_NUM];
static ST_MSG_HW_RECV_MSG para_hw_recv_msg[MAX_END_NUM];
static ST_MSG_HW_STS_MGT_ENABLE para_hw_sts_mgt_enable[MAX_END_NUM];
static ST_MSG_HW_DEINIT para_hw_sts_mgt_disable[MAX_END_NUM];
static ST_MSG_HW_FLT_MGT_ENABLE para_hw_flt_mgt_enable[MAX_END_NUM];
static ST_MSG_HW_CLR_FLT_CFG para_hw_flt_mgt_disable[MAX_END_NUM];
static ST_MSG_HW_ERR_HDL para_hw_err_hdl[MAX_END_NUM];
static struct workqueue_struct *msgbx_wq[MAX_END_NUM];
static spinlock_t send_msg_lock[NR_CPUS];
static spinlock_t vpid_send_lock[NR_CPUS];
/********************* global variables ***************************/
struct platform_device *g_ipc_msgbx_pdev = NULL;
/* tell ipc trans the first msgbox end pid */
static u8 g_start_pid;
static struct msgbox_statis g_stats;
#if 0
uint64_t msgbx_max_send_time[MAX_END_NUM][CHANNEL_COUNT * SESSION_COUNT] = {0};
uint64_t msgbx_get_max_send_time(uint8_t endid, uint8_t handle)
{
	return msgbx_max_send_time[endid][handle];
}
EXPORT_SYMBOL(msgbx_get_max_send_time);

void msgbx_clr_max_send_time(uint8_t endid, uint8_t handle)
{
	msgbx_max_send_time[endid][handle] = 0;
}
EXPORT_SYMBOL(msgbx_clr_max_send_time);
#endif

uint8_t get_cpuid_by_endid(uint8_t end_id)
{
	uint8_t i = 0;
	for (i = 0; i < MAX_END_NUM; i++) {
		if (st_msgbx_info[i].ablt_r.bit.endid == end_id)
			return i;
	}
	// pr_info(KERN_ERR "\n %s end_id %u\n", __func__, end_id);
	return CPUID_ERR;
}
EXPORT_SYMBOL(get_cpuid_by_endid);

uint64_t bstn_max_send_time = 0;
uint64_t msgbx_get_max_send_time(uint8_t endid, uint8_t handle)
{
	return bstn_max_send_time;
}
EXPORT_SYMBOL(msgbx_get_max_send_time);
EXPORT_SYMBOL(bstn_max_send_time);

u8 msgbx_get_start_pid(void)
{
	return g_start_pid;
}
EXPORT_SYMBOL(msgbx_get_start_pid);

/**********************local functions ***************************/
int msgbx_hw_send_msg_direct(rw_msg_t *msg);
static int32_t msgbx_ops_register(void);

struct msgbox_statis *msgbx_get_statis(void)
{
	return &g_stats;
}
EXPORT_SYMBOL(msgbx_get_statis);

#if defined(MSGBX_HW_TYPE_C1200)
#define MSG_SWITCH1_MAX_PID (0x50)
#define IPC_SEM_BASE (0x30100000)
#define IPC_SEM_SZIE (0x100000)
#define SW1_TO_SW2_LOCK_ID (0x0)
#define SW2_TO_SW1_LOCK_ID (0x1)
#define MST_ID_LOCK (1)
#define REG32(addr) (*((u32 *)((unsigned long)addr)))

static int get_lock(uint8_t sem_id)
{
	int data = 0;
	UN_REG_SEM sem_reg = {0};

	if (sem_id != SW1_TO_SW2_LOCK_ID && sem_id != SW2_TO_SW1_LOCK_ID)
		return -1;
	sem_reg.data = IPC_SEM_BASE;
	sem_reg.bit.sem_id = sem_id;
	sem_reg.bit.bank_id = 0;
	sem_reg.bit.mst_id = MST_ID_LOCK;

	do {
		data = readl_relaxed(g_ipc_msgbx->ipc_sem_base +
				     (sem_reg.data - IPC_SEM_BASE));
	} while (data != 0);

	return data;
}

static int get_msg_cross_sw_lock(uint8_t pid, uint8_t cid)
{
	uint8_t pid_high = 0, cid_high = 0, sem_id = 0;

	pid_high = pid & 0xf0;
	cid_high = cid & 0xf0;

	if ((pid_high <= MSG_SWITCH1_MAX_PID) && (cid_high > MSG_SWITCH1_MAX_PID))
		sem_id = SW1_TO_SW2_LOCK_ID;
	else if ((pid_high > MSG_SWITCH1_MAX_PID) && (cid_high <= MSG_SWITCH1_MAX_PID))
		sem_id = SW2_TO_SW1_LOCK_ID;
	else
		sem_id = 0xff; //internal messages sends in switch

	get_lock(sem_id);
	return sem_id;
}

static int release_cross_sw_lock(uint8_t sem_id)
{
	UN_REG_SEM sem_reg = {0};

	/* do not need */
	if (sem_id != SW1_TO_SW2_LOCK_ID && sem_id != SW2_TO_SW1_LOCK_ID)
		return -1;
	sem_reg.data = IPC_SEM_BASE;
	sem_reg.bit.sem_id = sem_id;
	sem_reg.bit.bank_id = 0;
	sem_reg.bit.mst_id = MST_ID_LOCK;
	writel_relaxed(1, g_ipc_msgbx->ipc_sem_base + (sem_reg.data - IPC_SEM_BASE));

	return 0;
}
#endif

static inline u32 msgbx_get_flt_count(void)
{
	return g_ipc_msgbx->filter_num >= MSGBOX_MAX_FILTER_NUM ? MSGBOX_MAX_FILTER_NUM : g_ipc_msgbx->filter_num;
}

static void msgbx_end_register_on_each_cpus(void *info)
{
	int ret;
	uint8_t is_online = *(uint8_t *)info;
	rw_msg_t notify_msg = { 0 };
	u32 cpuid = get_cpu();

	put_cpu();
	notify_msg.header.cid = CENTRAL_MONITOR_END_ID;
	notify_msg.header.pid = g_ipc_end_array[cpuid]->g_ipc_pid;
	notify_msg.header.len = 0;
	notify_msg.header.is_eof = 1;
	notify_msg.header.fid = 0;
	notify_msg.header.sid = 0;
	notify_msg.header.typ = 4;
	notify_msg.header.cmd = is_online;
	notify_msg.header.tok = 0;
	notify_msg.header.idx = 0;
	notify_msg.header.res = 0;
	notify_msg.header.ver = 2;

	ret = msgbx_hw_send_msg_direct(&notify_msg);
	WARN(ret < 0, "msgbox can't send end register cmd %u on cpu %u\n", is_online, cpuid);
}

static void get_msgbox_info(struct ipc_msgbox *ipc_msgbx, ST_MSG_INF *msg_info)
{
	u32 cpuid = get_cpu();
	put_cpu();
	g_ipc_end_array[cpuid]->g_ipc_cpuid = cpuid;
	msg_info->ablt_r.reg1 = readl_relaxed(ipc_msgbx->fcsr_base + DEF_FIL_ABLT_R);
	msg_info->ablt_r2.reg2 = readl_relaxed(ipc_msgbx->fcsr_base + DEF_FIL_ABLT2_R);
	msg_info->version = (u8)readl_relaxed(ipc_msgbx->fcsr_base + DEF_FLT_VERSION_R);

	pr_info("cpu%u filter_num:%d end_id:0x%x tx_dep:%d rx_dep:%d version:%d\n",
		cpuid,
		msg_info->ablt_r.bit.filter_num, msg_info->ablt_r.bit.endid,
		msg_info->ablt_r2.bit.tx_fifo_depth, msg_info->ablt_r2.bit.rx_fifo_depth,
		msg_info->version);
}

static int32_t get_msgbox_irq_info(struct platform_device *pdev)
{
#if defined(MSGBX_HW_TYPE_C1200)
	u32 fid, spi_irq_count;
#endif
	u32 index = 0, endid_idx = 0;
	int irq_count = 0, irq = 0;
	unsigned long hwirq = 0;
	struct irq_desc *ipc_irq_desc = NULL;
	struct ipc_msgbox *ipc_msgbx = g_ipc_msgbx;
	u32 msgend_count = ipc_msgbx->msgend_count;

	irq_count = platform_irq_count(pdev);
	if (irq_count < 0) {
		IPC_LOG_WARNING("NO irq is platform_irq_count irq_count:%d", irq_count);
		return irq_count;
	}

#if defined(MSGBX_HW_TYPE_C1200)
	/* DB has no spi irq, soc has 4 spi irq*/
	device_property_read_u32(&pdev->dev, "spi-irq-num", &spi_irq_count);
	ipc_msgbx->irq_count = spi_irq_count;
	IPC_LOG_DEBUG("c1200 hw spi irq num %d, irq_count:%u", spi_irq_count, irq_count);
#elif defined(MSGBX_HW_TYPE_A2000)
	ipc_msgbx->irq_count = irq_count;
	IPC_LOG_DEBUG("a2000 hw irq_count:%u", irq_count);
#endif

#if defined(MSGBX_HW_TYPE_C1200)
	for (index = 0; index < irq_count; index++) {
		irq = platform_get_irq(pdev, index);
		if (irq < 0) {
			IPC_LOG_ERR("NO irq is configured for filter %d", irq);
			return irq;
		}
		ipc_irq_desc = irq_to_desc(irq);
		hwirq = ipc_irq_desc->irq_data.hwirq;
		IPC_LOG_DEBUG("virq:%d hwirq:%lu", irq, hwirq);
		/* PPI interrupt set for each msgbox end */
		if (hwirq < 32) {
			for (endid_idx = 0; endid_idx < msgend_count; endid_idx++)
				msg_end_para[endid_idx].irq[index] = irq;
		} else {
			endid_idx = (index - CPU_MSGEND_PPI_NUM) / 4;
			fid = CPU_MSGEND_PPI_NUM + (index % spi_irq_count);
			if (endid_idx >= MAX_END_NUM || fid >= msgbx_get_flt_count()) {
				IPC_LOG_ERR("endid[%u] filterid[%u] ERROR!", endid_idx, fid);
				return -ERR_PARA;
			}
			msg_end_para[endid_idx].irq[fid] = irq;
			IPC_LOG_DEBUG("end%u filter:%d, virq %d hwirq:%lu", endid_idx, fid, irq, hwirq);
		}
	}
#elif defined(MSGBX_HW_TYPE_A2000)
	for (index = 0; index < irq_count; index++) {
		irq = platform_get_irq(pdev, index);
		if (irq < 0) {
			IPC_LOG_ERR("NO irq is configured for filter %d", irq);
			return irq;
		}
		ipc_irq_desc = irq_to_desc(irq);
		hwirq = ipc_irq_desc->irq_data.hwirq;
		IPC_LOG_DEBUG("virq:%d hwirq:%lu", irq, hwirq);
		/* PPI interrupt set for each msgbox end */
		for (endid_idx = 0; endid_idx < msgend_count; endid_idx++)
			msg_end_para[endid_idx].irq[index] = irq;
	}
#endif
	return 0;
}

static inline int32_t virq_to_hwirq(int32_t virq)
{
	return irq_to_desc(virq)->irq_data.hwirq;
}

static int32_t get_filter_id_by_virq(ST_MSGBX_END_PARA *end_para, u32 irq)
{
	u32 fid = 0, filter_num = end_para->ipc_msgbx->filter_num;

	for (fid = 0; fid < filter_num; fid++) {
		if (irq == end_para->irq[fid])
			return fid;
	}

	IPC_LOG_ERR("do not find filter id by virq!");
	return -1;
}

static inline u32 msgbox_get_msg_num(void *filter_csr, u32 fid)
{
	u32 msg_num = 0;

	if (fid == FILTER_DEFAULT)
		msg_num = readl_relaxed(filter_csr + DEF_FLT_RXFIFO_STATUS);
	else
		msg_num = readl_relaxed(filter_csr + End_filter_RxFIFO_StatusR);

	return msg_num;
}

static int32_t get_msg(int32_t fid, ST_MSGBX_END_PARA *end_para)
{
	void *filter_csr = end_para->ipc_msgbx->fcsr_base + (fid * FILTER_CSR_SIZE);
	void *rxfifo_base = end_para->ipc_msgbx->rxfifo_base + (fid * FILTER_CSR_SIZE);
	ST_MSG_MESSAGE64 *msg64;
	ST_MSG_MESSAGE64 s_msg64 = {0};
	u32 cpuid = get_cpu();
	u32 msg_num = 0, inter_status = 0, end_id = 0;
	u32 len = 0, i = 0;
#if defined(MSGBX_HW_TYPE_A2000)
	sts_endmap_t endmap = {0};
#endif

	put_cpu();

	if (cpuid >= MAX_END_NUM) {
		IPC_LOG_ERR("get cpu error, the cpu is %u, max cpu %u", cpuid, MAX_END_NUM);
		return -ERR_PARA;
	}
	msg64 = &s_msg64;
	/* defalut filter message receive */
	if (fid == FILTER_DEFAULT) {
		inter_status = readl_relaxed(filter_csr + DEF_FLT_INTER_R);
		if (inter_status & DEF_FILTER_RX_OVERFLOW_INTR) {
			g_stats.rx_overflow_stats[current->thread_info.cpu]++;
			ipc_hw_err_msg_notify(cpuid, fid, MSGBX_ERR_RX_OVERFLOW);
		}
		if (inter_status & DEF_FILTER_TX_OVERFLOW_INTR) {
			g_stats.tx_overflow_stats[current->thread_info.cpu]++;
			ipc_hw_err_msg_notify(cpuid, fid, MSGBX_ERR_TX_OVERFLOW);
		}
		if (inter_status & DEF_FILTER_RX_UNDERFLOW_INTR) {
			g_stats.rx_underflow_stats[current->thread_info.cpu]++;
			ipc_hw_err_msg_notify(cpuid, fid, MSGBX_ERR_RX_UNDERFLOW);
		}
#if defined(MSGBX_HW_TYPE_A2000)
		if (inter_status & DEF_FILTER_TX_ERR_INTR) {
			ipc_hw_err_msg_notify(cpuid, fid, MSGBX_ERR_TX_MSG_ERR);
		}
		if (inter_status & DEF_FILTER_END_ST_NTF_INTR) {
			endmap.hi_map = 0;
			endmap.lo_map = 0;
			ipc_hw_endmap_notify(cpuid, &endmap);
		}
#endif
		if (inter_status & DEF_FILTER_RX_THRS_INTR) {
			/* receive all msseages */
			msg_num = readl_relaxed(filter_csr + DEF_FLT_RXFIFO_STATUS);
			while (msg_num-- > 0) {
				msg64->head.data = readq_relaxed(rxfifo_base);
				len = msg64->head.bit.is_64_bit ? msg64->head.bit.len : (msg64->head.bit.len / 2);
				end_id = get_cpuid_by_endid(msg64->head.bit.cid);
				// 32bit system change the msg len for ipc app
				if (!msg64->head.bit.is_64_bit)
					msg64->head.bit.len = msg64->head.bit.len >> 1;
				// IPC_LOG_DEBUG("end%d msg_head:%llx", cpuid, msg64->head.data);
				if (len > MSG_64_MAX_LEN) {
					IPC_LOG_ERR("invalid len, end%d msg_head:%llx\n", cpuid, msg64->head.data);
				} else {
					for (i = 0; i < len; i++) {
						msg64->payload[i] = readq_relaxed(rxfifo_base);
						// IPC_LOG_DEBUG("end%u pay%d:%llx\n", cpuid, i, msg64->payload[i]);
					}
					ipc_hw_recv_msg_notify(cpuid, fid, (rw_msg_t *)msg64);

					if (end_id < MAX_END_NUM) {
						g_stats.rx_stats[current->thread_info.cpu][end_id]++;
						g_stats.rx_pidcid_stats[end_id][msg64->head.bit.pid]++;
					}
				}
			}
			g_stats.rx_thrs_stats[current->thread_info.cpu]++;
		}
		/* clean all interrupt */
		writel_relaxed(inter_status, filter_csr + DEF_FLT_INTER_CLR_R);
	} else {/* filter1 to n receive message */
		inter_status = readl_relaxed(filter_csr + FILTER1_INTER_CLR_R);

		if (inter_status & (FILTER1_UNDERFLOW_INTR | FILTER1_OVERFLOW_INTR)) {
			if (inter_status & DEF_FILTER_RX_OVERFLOW_INTR) {
				g_stats.rx_overflow_stats[current->thread_info.cpu]++;
				ipc_hw_err_msg_notify(cpuid, fid, MSGBX_ERR_RX_OVERFLOW);
			}
			else if (inter_status & DEF_FILTER_RX_UNDERFLOW_INTR) {
				g_stats.rx_underflow_stats[current->thread_info.cpu]++;
				ipc_hw_err_msg_notify(cpuid, fid, MSGBX_ERR_RX_UNDERFLOW);
			}
		}

		if (inter_status & FILTER1_THRS_INTR) {
			/* receive all msseages */
			msg_num = readl_relaxed(filter_csr + End_filter_RxFIFO_StatusR);
			while (msg_num-- > 0) {
				msg64->head.data = readq_relaxed(rxfifo_base);
				len = msg64->head.bit.is_64_bit ? msg64->head.bit.len : (msg64->head.bit.len / 2);
				end_id = get_cpuid_by_endid(msg64->head.bit.cid);
				// 32bit system change the msg len for ipc app
				if (!msg64->head.bit.is_64_bit)
					msg64->head.bit.len = msg64->head.bit.len >> 1;
				// IPC_LOG_DEBUG("end%d fid %u msg_head:%llx", cpuid, fid, msg64->head.data);
				if (len > MSG_64_MAX_LEN) {
					IPC_LOG_ERR("invalid len, end%d fid %u msg_head:%llx\n", cpuid, fid, msg64->head.data);
				} else {
					for (i = 0; i < len; i++)
						msg64->payload[i] = readq_relaxed(rxfifo_base);

					if (end_id < MAX_END_NUM) {
						g_stats.rx_stats[current->thread_info.cpu][end_id]++;
						g_stats.rx_pidcid_stats[end_id][msg64->head.bit.pid]++;
					}
					ipc_hw_recv_msg_notify(cpuid, fid, (rw_msg_t *)msg64);
				}
			}
			g_stats.rx_thrs_stats[current->thread_info.cpu]++;
		}
		/* clean all interrupt */
		writel_relaxed(inter_status, filter_csr + FILTER1_INTER_ST_R);
	}

	return 0;
}

// msgbox interrupt processing function
static irqreturn_t ipc_msg_rece_handler(int32_t irq, void *p)
{
	int32_t fid, ret;
	u32 cpu_id = get_cpu();
	ST_MSGBX_END_PARA *end_para = &msg_end_para[cpu_id];

	put_cpu();

	fid = get_filter_id_by_virq(end_para, irq);
	if (fid < 0 || fid >= FILTER_NUM_BUFF) {
		IPC_LOG_ERR("illegal cpuid %u filter id:%d!", cpu_id, fid);
		return IRQ_NONE;
	}

	ret = get_msg(fid, end_para);
	return IRQ_HANDLED;
}

#if defined(MSGBX_HW_TYPE_C1200)
static int32_t of_msg_recv_init_spi_irqs(struct platform_device *pdev,
			     ST_MSGBX_END_PARA *end_para, u16 cpuid)
{
	int32_t irq = 0, ret = -1;
	u32 fid = 0, filter_num = 0, spi_count = 0;
	struct cpumask mask = {0};

	filter_num = end_para->ipc_msgbx->filter_num;
	spi_count = end_para->ipc_msgbx->irq_count;
	/* DB do not has gic_spi */
	if (!spi_count)
		return 0;

	IPC_LOG_DEBUG("filter num %u", filter_num);
	for (fid = (filter_num - spi_count); fid < filter_num; fid++) {
		irq = end_para->irq[fid];
		if (irq == 0) {
			pr_err("cpuid %u do not have irq define in dts\n", cpuid);
			continue;
		}
		IPC_LOG_DEBUG("register filter%u SPI[%u]", fid, virq_to_hwirq(irq));
		ret = devm_request_irq(&pdev->dev, irq,
				       ipc_msg_rece_handler,
				       IRQF_ONESHOT | IRQF_SHARED, irq_desc[cpuid][fid],
				       end_para);
		if (ret) {
			IPC_LOG_ERR("failed to request irq %d, ret = %d", irq, ret);
			continue;
		}
		disable_irq(irq);
		cpumask_clear(&mask);
		cpumask_set_cpu(cpuid, &mask);
		ret = irq_set_affinity_hint(irq, &mask);
		if (unlikely(ret < 0)) {
			devm_free_irq(&pdev->dev, irq, end_para);
			IPC_LOG_ERR("irq_set_affinity_hint32_t failed: ret = %d", ret);
			return ret;
		}
		IPC_LOG_DEBUG("spi:%d set to cpu%u", irq, cpuid);
	}
	return ret;
}

#if 0
static void of_msg_recv_deinit_spi_irqs(struct platform_device *pdev,
			     ST_MSGBX_END_PARA *end_para, u16 cpuid)
{
	int32_t irq = 0;
	int32_t fid = 0;
	u32 filter_num = 0, spi_count = 0;

	filter_num = end_para->ipc_msgbx->filter_num;
	spi_count = end_para->ipc_msgbx->irq_count;
	if (!spi_count)
		return;

	IPC_LOG_DEBUG("filter num %u", filter_num);
	for (fid = (filter_num - spi_count); fid < filter_num; fid++) {
		irq = end_para->irq[fid];
		IPC_LOG_DEBUG("deinit filter%u SPI[%u]", fid, virq_to_hwirq(irq));
		irq_set_affinity_hint(irq, NULL);
		devm_free_irq(&pdev->dev, irq, end_para);
	}
}
#endif
#endif

static u32 check_ppi_trigger(int irq)
{
	u32 flags = irq_get_trigger_type(irq);

	if (flags != IRQF_TRIGGER_HIGH && flags != IRQF_TRIGGER_LOW) {
		IPC_LOG_WARNING("WARNING: Invalid trigger for IRQ %d, assuming level low", irq);
		IPC_LOG_WARNING("WARNING: Please fix your firmware");
		flags = IRQF_TRIGGER_LOW;
	}
	return flags;
}

static void of_msg_enable_ppi_irqs(struct platform_device *pdev,
			     ST_MSGBX_END_PARA *end_para)
{
	int32_t irq = 0;
	u32 filter_num = 0, spi_count = 0, ppi_num = 0;
	u32 flags = 0, cpu_id = 0, fid = 0;

	filter_num = end_para->ipc_msgbx->filter_num;
	spi_count = end_para->ipc_msgbx->irq_count;
	/* DB do not has gic_spi */
	ppi_num = (spi_count) ? (filter_num - spi_count) : (filter_num);
	IPC_LOG_DEBUG("ppi num %u", ppi_num);
	cpu_id = get_cpu();
	put_cpu();

	//percpu_end_data = alloc_percpu(ST_MSGBX_END_PARA);
	//memcpy(this_cpu_ptr(percpu_end_data), end_para, sizeof(ST_MSGBX_END_PARA));
	//IPC_LOG_ERR("end_para:0x%lx percpu_end_data:0x%lx filter_num:%d\n", (u64)end_para, (u64)this_cpu_ptr(percpu_end_data), percpu_end_data->ipc_msgbx->filter_num);
	/* init end data for interrupt uses */
	for (fid = 0; fid < ppi_num; fid++) {
		irq = end_para->irq[fid];
		IPC_LOG_DEBUG("enable cpu%u filter%u PPI[%u]", cpu_id, fid, virq_to_hwirq(irq));
		flags = check_ppi_trigger(irq);
		enable_percpu_irq(irq, flags);
	}
}

static void of_msg_disable_ppi_irqs(struct platform_device *pdev,
			     ST_MSGBX_END_PARA *end_para)
{
	int32_t irq = 0, fid = 0;
	u32 cpu_id = 0;
	u32 ppi_num = 0;
#if defined(MSGBX_HW_TYPE_C1200)
	u32 spi_count = 0, filter_num = 0;

	filter_num = end_para->ipc_msgbx->filter_num;
	spi_count = end_para->ipc_msgbx->irq_count;
	/* DB do not has gic_spi */
	ppi_num = (spi_count) ? (filter_num - spi_count) : (filter_num);
	IPC_LOG_DEBUG("ppi num %u", ppi_num);
#elif defined(MSGBX_HW_TYPE_A2000)
	ppi_num = end_para->ipc_msgbx->irq_count;
#endif

	cpu_id = get_cpu();
	put_cpu();

	/* de-init end data for interrupt uses */
	for (fid = 0; fid < ppi_num; fid++) {
		irq = end_para->irq[fid];
		disable_percpu_irq(irq);
	}
}

static int32_t msgbox_ppi_irqs_request(struct platform_device *pdev,
			     ST_MSGBX_END_PARA *end_para)
{
	int32_t irq = 0, ret = -1;
	u32 irq_count = 0, cpu_id = 0;
#if defined(MSGBX_HW_TYPE_C1200)
	u32 filter_num = 0, ppi_num, fid = 0;
#elif defined(MSGBX_HW_TYPE_A2000)
	u32 irq_id = 0;
#endif
	char *irq_desc = NULL;

	irq_count = end_para->ipc_msgbx->irq_count;
#if defined(MSGBX_HW_TYPE_C1200)
	filter_num = end_para->ipc_msgbx->filter_num;
	/* DB do not has gic_spi */
	ppi_num = (irq_count) ? (filter_num - irq_count) : (filter_num);
	IPC_LOG_DEBUG("ppi num %u", ppi_num);
#endif

	cpu_id = get_cpu();
	put_cpu();

#if defined(MSGBX_HW_TYPE_C1200)
	for (fid = 0; fid < ppi_num; fid++) {
		irq = end_para->irq[fid];
		IPC_LOG_DEBUG("cpu%u register filter%u PPI[%u]", cpu_id, fid, virq_to_hwirq(irq));
		irq_desc = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s[%d]",
					"bst_msgbox_fid", fid);
		if (!irq_desc) {
			IPC_LOG_ERR("devm_kasprintf no enough memory!\n");
			return -ENOMEM;
		}

		ret = request_percpu_irq(irq, ipc_msg_rece_handler,
			irq_desc, end_para);

		if (ret) {
			IPC_LOG_ERR("failed to request PPI irq %d, ret = %d", irq, ret);
		}
	}
#elif defined(MSGBX_HW_TYPE_A2000)
	for (irq_id = 0; irq_id < irq_count; irq_id++) {
		irq = end_para->irq[irq_id];
		IPC_LOG_DEBUG("cpu%u register ppi%u irq[%u]", cpu_id, irq_id, virq_to_hwirq(irq));
		irq_desc = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s[%d]",
					"bst_msgbox_ppi", irq_id);
		if (!irq_desc) {
			IPC_LOG_ERR("devm_kasprintf no enough memory!\n");
			return -ENOMEM;
		}

		ret = request_percpu_irq(irq, ipc_msg_rece_handler,
			irq_desc, end_para);

		if (ret) {
			IPC_LOG_ERR("failed to request PPI irq %d, ret = %d", irq, ret);
		}
	}
#endif
	return ret;
}

// static int msgbx_init_on_each_cpu(ST_MSGBX_END_PARA *end_para, u16 cpuid)
static void msgbx_init_on_each_cpu(void* info)
{
	uint32_t cpuid = get_cpu();
	ST_MSGBX_END_PARA *end_para = &msg_end_para[cpuid];
	put_cpu();

	// get msgbox hardware info run at each cpu
	get_msgbox_info(end_para->ipc_msgbx, &st_msgbx_info[cpuid]);
	of_msg_enable_ppi_irqs(g_ipc_msgbx_pdev, end_para);

	return;
}

#if 0
static void msgbx_deinit_on_each_cpu(ST_MSGBX_END_PARA *end_para, u16 cpuid)
{
	// get msgbox hardware info run at each cpu
	get_msgbox_info(end_para->ipc_msgbx, &st_msgbx_info[cpuid]);
	of_msg_disable_ppi_irqs(g_ipc_msgbx_pdev, end_para);

#if defined(MSGBX_HW_TYPE_C1200)
	/* spi irq deinit */
	of_msg_recv_deinit_spi_irqs(g_ipc_msgbx_pdev, end_para, cpuid);
#endif
}
#endif

static void msgbx_suspend_on_each_cpu(void *pdev_info)
{
	struct platform_device *pdev = (struct platform_device *)pdev_info;
	u32 cpuid = get_cpu();
	ST_MSGBX_END_PARA *end_para = &msg_end_para[cpuid];
	u32 fid = 0, filter_num = 0, spi_count = 0;
	int32_t irq = 0;

	put_cpu();
	of_msg_disable_ppi_irqs(pdev, &msg_end_para[cpuid]);
	filter_num = end_para->ipc_msgbx->filter_num;
	spi_count = end_para->ipc_msgbx->irq_count;
	for (fid = (filter_num - spi_count); fid < filter_num; fid++) {
		irq = end_para->irq[fid];
		disable_irq(irq);
	}
}

static void msgbx_resmue_on_each_cpu(void *pdev_info)
{
	struct platform_device *pdev = (struct platform_device *)pdev_info;
	u32 cpuid = get_cpu();
	ST_MSGBX_END_PARA *end_para = &msg_end_para[cpuid];
	u32 fid = 0, filter_num = 0, spi_count = 0;
	int32_t irq = 0;

	put_cpu();
	// get msgbox hardware info run at each cpu
	get_msgbox_info(end_para->ipc_msgbx, &st_msgbx_info[cpuid]);
	of_msg_enable_ppi_irqs(pdev, end_para);
	filter_num = end_para->ipc_msgbx->filter_num;
	spi_count = end_para->ipc_msgbx->irq_count;
	for (fid = (filter_num - spi_count); fid < filter_num; fid++) {
		irq = end_para->irq[fid];
		enable_irq(irq);
	}
}

static int per_msgbx_end_register(u32 cpu)
{
	// int32_t ret = -1;
	uint8_t is_online = 1;
	ST_MSGBX_END_PARA *end_para;
	end_para = &msg_end_para[cpu];

	// ret = msgbx_init_on_each_cpu(end_para, cpu);
	// if (ret < 0) {
	// 	pr_err("msgbox catn't init on cpu%u\n", cpu);
	// 	return ret;
	// }
	msgbx_resmue_on_each_cpu(g_ipc_msgbx_pdev);

	if (g_ipc_end_array[cpu]->g_ipc_pid != 0)
		msgbx_end_register_on_each_cpus(&is_online);
	return 0;
}

static int per_msgbx_end_unregister(unsigned int cpu)
{
	ST_MSGBX_END_PARA *end_para;
	uint8_t is_online = 0;
	end_para = &msg_end_para[cpu];

	if (g_ipc_end_array[cpu]->g_ipc_pid == st_msgbx_info[cpu].ablt_r.bit.endid)
		msgbx_end_register_on_each_cpus(&is_online);

	// msgbx_deinit_on_each_cpu(end_para, cpu);
	msgbx_suspend_on_each_cpu(g_ipc_msgbx_pdev);
	return 0;
}

#ifdef CONFIG_MSGBOX_DEBUG
static void dump_regs(u32 fid)
{
	void *flt_addr = NULL;
	u32 i = 0, dump_count = fid == 0 ? 16: 40;

	flt_addr = msgbx_get_filter_base(fid);
	pr_info("================start %s for fid%u===================\n", __func__, fid);
	for (i = 0; i <= dump_count; i++)
		pr_info("=====0x%x : 0x%08x\n", i * 8, readl_relaxed(flt_addr + i * 8));
	pr_info("================end %s===================\n", __func__);
}
#endif

static void ipc_msgbox_fifo_flt_close(void *flt_addr, u32 fid, u32 close_bits)
{
	if (fid) {
		close_bits |= readl_relaxed(flt_addr + End_filter1_RxFIFO_ADDRR);
		close_bits &= (~BIT(31));
		writel_relaxed(close_bits, flt_addr + End_filter1_RxFIFO_ADDRR); //clear fifo
//		writel_relaxed(close_bits, flt_addr + End_filter1_MsgH_PIDF_CFGR); //close filter
//		writel_relaxed(close_bits, flt_addr + End_filter1_MsgH_ResF_MaskHR); //clear flt
	} else {
//		writel_relaxed(close_bits, flt_addr + DEF_RXFIFO_ADDR); //clear fifo
		writel_relaxed(close_bits, flt_addr + DEF_FLT_MSG_PIDF_CFGR); //close filter
		udelay(10);
	}
}
 
static void ipc_hw_flt_enable(void *info)
{
	void *flt_addr = NULL;
	u32 filter_num = 0, fid = 0, irq_en_reg = DEF_FLT_INTER_EN;
	bool enable = *(bool *)info;
	u32 irq_en = enable == 0 ? 0: DEF_FILTER_RX_THRS_INTR | DEF_FILTER_RX_OVERFLOW_INTR |
				      DEF_FILTER_RX_UNDERFLOW_INTR;

	flt_addr = msgbx_get_filter_base(fid); 
	filter_num = REGS_FLT_NUM(readl_relaxed(flt_addr));
	for (fid = 0; fid < filter_num; fid++) {
		irq_en_reg = fid == 0 ? DEF_FLT_INTER_EN: FILTER1_EN_INTER;
		flt_addr = msgbx_get_filter_base(fid);
		writel_relaxed(irq_en, flt_addr + irq_en_reg);
		if (!enable)
			ipc_msgbox_fifo_flt_close(flt_addr, fid, 0);
	}
}

static int32_t ipc_msgbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ipc_msgbox *ipc_msgbx = NULL;
	struct resource *iomem = NULL;
	char msgbx_wq_name[MSGBX_WQ_LEN];
	u32 i, cpuid = 0, endid = 0;
	bool irq_enable = false;
	int32_t ret = 0;
#if defined(MSGBX_HW_TYPE_C1200)
	u32 fid = 0;
#endif

	g_ipc_msgbx_pdev = pdev;
	cpuid = get_cpu();
	put_cpu();

	IPC_LOG_DEBUG("CPU%u probe msgbox!", cpuid);
	ipc_msgbx = devm_kzalloc(dev, sizeof(struct ipc_msgbox), GFP_KERNEL);
	if (IS_ERR_OR_NULL(ipc_msgbx)) {
		ret = PTR_ERR_OR_ZERO(ipc_msgbx);
		IPC_LOG_ERR("devm_kzalloc ipc_msgbx return %d", ret);
		return ret;
	}

	g_ipc_msgbx = ipc_msgbx;
	ipc_msgbx->dev = &pdev->dev;

	// filter csr register
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, RES_ID_FILTER_CSR);
	if (IS_ERR_OR_NULL(iomem)) {
		ret = PTR_ERR_OR_ZERO(iomem);
		IPC_LOG_ERR("platform_get_resource IORESOURCE_MEM 0 return %d", ret);
		goto ERR_SYS_INIT;
	}
	IPC_LOG_DEBUG("msgbox end addr start: 0x%llx, end: 0x%llx", iomem->start, iomem->end);
	ipc_msgbx->fcsr_base = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR_OR_NULL(ipc_msgbx->fcsr_base)) {
		ret = PTR_ERR_OR_ZERO(ipc_msgbx->fcsr_base);
		IPC_LOG_ERR("Failed to remap msgbox regs: %d\n", ret);
		goto ERR_SYS_INIT;
	}

#if defined(MSGBX_HW_TYPE_C1200)
	ipc_msgbx->ipc_sem_base = ioremap(IPC_SEM_BASE, IPC_SEM_SZIE);
	if (IS_ERR_OR_NULL(ipc_msgbx->ipc_sem_base)) {
		ret = PTR_ERR_OR_ZERO(ipc_msgbx->ipc_sem_base);
		IPC_LOG_ERR("Failed to remap ipc_sem_base regs: %d\n", ret);
		goto ERR_SYS_INIT;
	}
#endif

	// msgbox rxfifo register base
	ipc_msgbx->rxfifo_base = ipc_msgbx->fcsr_base + MSGBOX_RXFIFO_OFFSET;
	IPC_LOG_DEBUG("msgbox base addr va:0x%llx rxfifo va_start: 0x%llx", (u64)ipc_msgbx->fcsr_base, (u64)ipc_msgbx->rxfifo_base);

	// msgbox txfifo register base
	ipc_msgbx->txfifo_base = ipc_msgbx->fcsr_base + MSGBOX_TXFIFO_OFFSET;
	IPC_LOG_DEBUG("msgbox txfifo va_start: 0x%llx", (u64)ipc_msgbx->txfifo_base);

	ipc_msgbx->msgend_count = num_online_cpus();
	if (ipc_msgbx->msgend_count == 0 || ipc_msgbx->msgend_count > MAX_END_NUM) {
		IPC_LOG_ERR("Failed to get end number: %u ret:%d\n", ipc_msgbx->msgend_count, ret);
		goto ERR_SYS_INIT;
	}
	ret = device_property_read_u32(&pdev->dev, "filter-num", &ipc_msgbx->filter_num);
	IPC_LOG_DEBUG("msgbox msgend_count: %u filter_num:%d", ipc_msgbx->msgend_count, ipc_msgbx->filter_num);

	for (i = 0; i < ipc_msgbx->msgend_count; i++) {
		msg_end_para[i].ipc_msgbx = ipc_msgbx;
		// alloc buffer for ipc trans layer
		g_ipc_end_array[i] = devm_kzalloc(dev, sizeof(msgbx_end_device_t), GFP_KERNEL);
		if (IS_ERR_OR_NULL(g_ipc_end_array[i])) {
			ret = PTR_ERR_OR_ZERO(g_ipc_end_array[i]);
			IPC_LOG_ERR("g_ipc_end_array[%u] devm_kzalloc error return %d", i, ret);
			goto ERR_SYS_INIT;
		}
		spin_lock_init(&tx_msg_spinlock[i]);
	}

	msgbx_ops_register();
	get_msgbox_irq_info(pdev);

#if (MSGBOX_DEBUG == 1)
	// for (i = 0; i < ipc_msgbx->msgend_count; i++) {
	// 	for (j = 0; j < ARRAY_SIZE(msg_end_para[0].irq); j++)
	// 		IPC_LOG_ERR("end[%u]filter[%u]:irq %d", i, j, msg_end_para[i].irq[j]);
	// }
#endif

	// disable all filter
	i = 0;
	for_each_online_cpu(i) {
		smp_call_function_single(i, ipc_hw_flt_enable, &irq_enable, 1);
	}
	// request ppi irqs
	msgbox_ppi_irqs_request(g_ipc_msgbx_pdev, &msg_end_para[cpuid]);

#if defined(MSGBX_HW_TYPE_C1200)
	/* init each msgbox end spi except self core */
	for_each_online_cpu(i) {
		for (fid = (ipc_msgbx->filter_num - ipc_msgbx->irq_count);
				fid < ipc_msgbx->filter_num; fid++) {
			irq_desc[i][fid] = devm_kasprintf(&pdev->dev, GFP_KERNEL,
							  "bst_msgbox_end%u_fid%d", i, fid);
			if (unlikely(!irq_desc[i][fid])) {
				IPC_LOG_ERR("devm_kasprintf no enough memory!\n");
				return -ENOMEM;
			}
		}
	}
#endif
	// request spi irqs and enable irqs
	i = 0;
	for_each_online_cpu(i) {
		smp_call_function_single(i, msgbx_init_on_each_cpu, &irq_enable, 1);
#if defined(MSGBX_HW_TYPE_C1200)
	/* spi irq init */
	ret = of_msg_recv_init_spi_irqs(g_ipc_msgbx_pdev, &msg_end_para[i], i);
	if (ret < 0)
		pr_err("spi interrupt init fail ret:%d", ret);
#endif
	}
	ret = cpuhp_setup_state(CPUHP_AP_BST_STARTING, "bst/msgbox:starting",
				per_msgbx_end_register, per_msgbx_end_unregister);
	if (ret)
		goto ERR_SYS_INIT;

#ifndef CONFIG_MSGBOX_MISCDEV_MOD
	ipc_msgbx_miscdev_init();
#endif

	g_start_pid = st_msgbx_info[0].ablt_r.bit.endid;
	platform_set_drvdata(pdev, ipc_msgbx);

	cpuid = 0;
	for_each_online_cpu(cpuid) {
		memset(msgbx_wq_name, 0, MSGBX_WQ_LEN);
		sprintf(msgbx_wq_name, "msgbx_wq%d", cpuid);
		msgbx_wq[cpuid] = alloc_workqueue(msgbx_wq_name, WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
		spin_lock_init(&send_msg_lock[cpuid]);
		spin_lock_init(&vpid_send_lock[cpuid]);
		if (unlikely(!msgbx_wq[cpuid])) {
			IPC_LOG_ERR("%s can't alloc msgbox wq\n", __func__);
			ret = -ENOMEM;
			goto msgbx_wq_err;
		}
	}

	cpuid = 0;
	for_each_online_cpu(cpuid) {
		// endid = cpuid + g_start_pid;
		endid = st_msgbx_info[cpuid].ablt_r.bit.endid;
		ret = ipc_trans_layer_start(endid, 0);
		if (ret < 0) {
			IPC_LOG_WARNING("layer start endid(0x%x) fail!", endid);
			return ret;
		}
	}
#ifdef CONFIG_MSGBOX_DEBUG_FS
	msgbx_dbg_sysfs_init(dev);
#endif

	IPC_LOG_ERR("ipc msgbox driver is ready! g_start_pid:%x", g_start_pid);
	return ret;

msgbx_wq_err:
	for_each_online_cpu(cpuid) {
		if (msgbx_wq[cpuid])
			destroy_workqueue(msgbx_wq[cpuid]);
	}

ERR_SYS_INIT:
	if (g_ipc_msgbx)
		devm_kfree(&pdev->dev, g_ipc_msgbx);

	for (i = 0; i < ipc_msgbx->msgend_count; i++) {
		if (g_ipc_end_array[i]) {
			devm_kfree(&pdev->dev, g_ipc_end_array[i]);
		}
	}

	IPC_LOG_ERR("probe error exit");
	return ret;
}

static int32_t ipc_msgbox_remove(struct platform_device *pdev)
{
	u32 i, cur_cpuid = get_cpu(), cpuid = 0;
#if defined(MSGBX_HW_TYPE_C1200)
	u32 fid = 0, flt_num = g_ipc_msgbx->filter_num;
	u32 spi_cnt = g_ipc_msgbx->irq_count;
#endif
	bool irq_enable = false;

	put_cpu();
	for_each_online_cpu(cpuid) {
		if (cpuid != cur_cpuid)
			smp_call_function_single(cpuid, ipc_hw_flt_enable, &irq_enable, 1);
		else
			ipc_hw_flt_enable(&irq_enable);
	}

#if defined(MSGBX_HW_TYPE_C1200)
	for_each_online_cpu(cpuid) {
		for (fid = (flt_num - spi_cnt);	fid < flt_num; fid++)
			kfree(irq_desc[cpuid][fid]);
	}
#endif

	if (g_ipc_msgbx)
		devm_kfree(&pdev->dev, g_ipc_msgbx);

	for_each_online_cpu(cpuid) {
		if (msgbx_wq[cpuid])
			destroy_workqueue(msgbx_wq[cpuid]);
	}

	for (i = 0; i < num_online_cpus(); i++) {
		if (g_ipc_end_array[i]) {
			devm_kfree(&pdev->dev, g_ipc_end_array[i]);
		}
	}

#ifndef CONFIG_MSGBOX_MISCDEV_MOD
	ipc_msgbx_miscdev_exit();
#endif
#ifdef CONFIG_MSGBOX_DEBUG_FS
	msgbx_dbg_sysfs_exit(&pdev->dev);
#endif

	return 0;
}
 
static void ipc_msgbox_shutdown(struct platform_device *pdev)
{
	u32 cpuid = 0, cur_cpuid = get_cpu();
	bool irq_enable = false;
	uint8_t is_online = 0;

	put_cpu();

#if defined(MSGBX_HW_TYPE_C1200)
#if defined(CONFIG_C1200_SLT) || defined(CONFIG_C1200_MASS)
	if (loopback)
		kthread_stop(loopback);
#endif
#endif

	for_each_online_cpu(cpuid) {
		if (msgbx_wq[cpuid])
			destroy_workqueue(msgbx_wq[cpuid]);
	}

	for_each_online_cpu(cpuid) {
		if (cpuid != cur_cpuid){
			smp_call_function_single(cpuid, msgbx_end_register_on_each_cpus, &is_online, 1);
			smp_call_function_single(cpuid, ipc_hw_flt_enable, &irq_enable, 1);
		}
		else{
			msgbx_end_register_on_each_cpus(&is_online);
			ipc_hw_flt_enable(&irq_enable);
		}
	}
}

/*
 * ipc_msgbox_suspend - msgbox suspend
 * @pdev : platform device data
 *
 * Disable ppi and msgbox intr_en.
 */
static int ipc_msgbox_suspend(struct platform_device *pdev, pm_message_t state)
{
	u32 cpuid, cur_cpuid = get_cpu();
	bool irq_enable = false;
	uint8_t is_online = 0;

	put_cpu();
	for_each_online_cpu(cpuid) {
		if (cpuid != cur_cpuid) {
			smp_call_function_single(cpuid, msgbx_end_register_on_each_cpus, &is_online, 1);
			smp_call_function_single(cpuid, ipc_hw_flt_enable, &irq_enable, 1);
			smp_call_function_single(cpuid, msgbx_suspend_on_each_cpu, pdev, 1);
		} else {
			msgbx_end_register_on_each_cpus(&is_online);
			msgbx_suspend_on_each_cpu((void *)pdev);
			ipc_hw_flt_enable(&irq_enable);
		}
	}

	return 0;
}

/*
 * ipc_msgbox_resume - msgbox resume
 * @pdev : platform device data
 *
 * If there is not the resume callback, no ppi irq would be recvd from gic due to closing the
 * bits of ppi for msgbox.
 */
static int ipc_msgbox_resume(struct platform_device *pdev)
{
	int ret = 0;
	u32 cpuid = 0, cur_cpuid = get_cpu();
	bool irq_enable = true;

	put_cpu();
	for_each_online_cpu(cpuid) {
		ret = ipc_trans_reinit(g_ipc_end_array[cpuid]);
		WARN(ret < 0, "msgbox catn't reonline on cpu%u\n", cpuid);
		if (cpuid != cur_cpuid) {
			smp_call_function_single(cpuid, msgbx_resmue_on_each_cpu, pdev, 1);
			smp_call_function_single(cpuid, ipc_hw_flt_enable, &irq_enable, 1);
		} else {
			msgbx_resmue_on_each_cpu((void *)pdev);
			ipc_hw_flt_enable(&irq_enable);
		}
	}

	return ret;
}

static const struct of_device_id ipc_msgbox_of_match[] = {
	{
		.compatible = "bst,bst-msgbox",
	},
	{},
};
MODULE_DEVICE_TABLE(of, ipc_msgbox_of_match);
static struct platform_driver ipc_msgbox_driver = {
	.driver = {
		.name = "bst-msgbox",
		.of_match_table = ipc_msgbox_of_match,
	},
	.probe		= ipc_msgbox_probe,
	.remove		= ipc_msgbox_remove,
	.shutdown	= ipc_msgbox_shutdown,
	.suspend	= ipc_msgbox_suspend,
	.resume		= ipc_msgbox_resume,
};

static int32_t __init ipc_msgbox_init(void)
{
	return platform_driver_register(&ipc_msgbox_driver);
}

#ifndef CONFIG_MSGBOX_MISCDEV_MOD
module_platform_driver(ipc_msgbox_driver);
#else
subsys_initcall(ipc_msgbox_init);
#endif

void *msgbx_get_filter_base(u32 fid)
{
	if (!g_ipc_msgbx->fcsr_base || fid >= msgbx_get_flt_count()) {
		IPC_LOG_ERR("FILTER_BASE or fid:%u error", fid);
		return NULL;
	}
	return g_ipc_msgbx->fcsr_base + (fid * FILTER_CSR_SIZE);
}
EXPORT_SYMBOL(msgbx_get_filter_base);

static inline void *msgbx_get_filter_tx_fifo_base(void)
{
	return g_ipc_msgbx->txfifo_base;
}

static inline void *msgbx_get_filter_rx_fifo_base(u32 fid)
{
	if (!g_ipc_msgbx->fcsr_base || fid >= msgbx_get_flt_count())
		IPC_LOG_ERR("FILTER_BASE or fid:%u error", fid);
	return g_ipc_msgbx->rxfifo_base + (fid * FILTER_CSR_SIZE);
}

#if defined(MSGBX_HW_TYPE_A2000)
static u64 msgbox_get_fil_ppi_sel_reg(u32 cpu)
{
	u64 filter_ppi_sel_reg = MSGBOX_IRQ_REG;
	u32 cpuid = cpu >= 16 ? cpu - 16 : cpu;

	switch (cpuid) {
	case 0:
		filter_ppi_sel_reg += CORE0_FILTER_PPI_SEL_REG;
		break;
	case 1:
		filter_ppi_sel_reg += CORE1_FILTER_PPI_SEL_REG;
		break;
	case 2:
		filter_ppi_sel_reg += CORE2_FILTER_PPI_SEL_REG;
		break;
	case 3:
		filter_ppi_sel_reg += CORE3_FILTER_PPI_SEL_REG;
		break;
	default:
		filter_ppi_sel_reg += INVALID_FILTER_PPI_SEL_REG;
		break;
	}

	return filter_ppi_sel_reg;
}

static void msgbox_clear_mask_reg(void *flt_addr, u32 filter_num)
{
	void *filter_mask_reg = flt_addr;

	filter_mask_reg += (filter_num ? 0x140 + filter_num * FILTER_CSR_SIZE : 0x80);
	writel_relaxed(0, filter_mask_reg);
}

static int msgbox_sel_ppi_rx_irq(u32 filter_num, u32 ppix, u32 cpu, bool set_mask)
{
	void *flt_addr = msgbx_get_filter_base(filter_num);
	void *filter_ppi_sel = flt_addr;
	u32 filter_ppi_sel_reg_val = (ppix & 0x3) << (filter_num * 2);
	u64 filter_ppi_sel_reg = msgbox_get_fil_ppi_sel_reg(cpu);
#ifdef CONFIG_MSGBOX_DEBUG
	void *ppi_status_reg = NULL;
	u32 ppi_status_reg_val = 0;
#endif

	ppix &= 0xf;
	if (set_mask)
		msgbox_clear_mask_reg(flt_addr, filter_num);

	if (filter_ppi_sel_reg == (MSGBOX_IRQ_REG + INVALID_FILTER_PPI_SEL_REG))
		return -EINVAL;

	filter_ppi_sel = flt_addr + filter_ppi_sel_reg;
	writel_relaxed(filter_ppi_sel_reg_val, filter_ppi_sel);

#ifdef CONFIG_MSGBOX_DEBUG
	ppi_status_reg = flt_addr + filter_ppi_sel_reg + 4 * (ppix + 1);
	IPC_LOG_DEBUG("ppi status(0x%px), filter sel(0x%llx), 4 * (%u + 1) = %u",
			ppi_status_reg, filter_ppi_sel_reg,
			ppix, 4 * (ppix + 1));

	filter_ppi_sel_reg_val = readl_relaxed(filter_ppi_sel);
	IPC_LOG_DEBUG("msgbox (0x%llx)filter%u selects ppi%u: 0x%x",
			filter_ppi_sel_reg, filter_num, ppix & 0xf,
			filter_ppi_sel_reg_val);
	ppi_status_reg_val = readl_relaxed(ppi_status_reg);
	IPC_LOG_DEBUG("CORE%u_PPI%u_STATUS_REG(0x%px) : 0x%08x",
			cpu, ppix, ppi_status_reg, ppi_status_reg_val);
#endif
	return 0;
}
#endif

static void msgbox_set_filter_addr(u32 filter, u32 fifo_st, u32 fifo_end)
{
	void *flt_addr = NULL;
	u32 value = 0;

	/** base addr */
	flt_addr = msgbx_get_filter_base(filter);
	if (filter == FILTER_DEFAULT) {
		//fifo depth config
		value = ((fifo_st & END_RXFIFO_ADDR_MASK)  << END_RXFIFO_ADDR_RX_FIFO_ST_ADDR_SHIFT_U32) |
			((fifo_end & END_RXFIFO_ADDR_MASK) << END_RXFIFO_ADDR_RX_FIFO_END_ADDR_SHIFT_U32);
		writel_relaxed(value, flt_addr + DEF_RXFIFO_ADDR);
	} else {
		//fifo depth config
		value = (1UL << END_RXFIFO_ADDR_FILTER_EN_SHIFT_U32) |
			((fifo_st & END_RXFIFO_ADDR_MASK)  << END_RXFIFO_ADDR_RX_FIFO_ST_ADDR_SHIFT_U32) |
			((fifo_end & END_RXFIFO_ADDR_MASK) << END_RXFIFO_ADDR_RX_FIFO_END_ADDR_SHIFT_U32);
		writel_relaxed(value, flt_addr + FILTER1_RXFIFO_ADDRR);
	}
}

static void msgbox_set_filter_pid(u32 filter, u32 pid_st, u32 pid_end, u32 invert)
{
	void *flt_addr = NULL;
	u32 value = 0;

	/** base addr */
	flt_addr = msgbx_get_filter_base(filter);
	if (filter == FILTER_DEFAULT) {
		//filter0 receive pid select
		value = ((pid_st & END_PIDF_CFGR_MASK)  << END_PIDF_CFGR_RX_PID_ST_SHIFT_U32)|
			((pid_end & END_PIDF_CFGR_MASK) << END_PIDF_CFGR_RX_PID_END_SHIFT_U32)|
			(1ull << END_PIDF_CFGR_RX_FILTER_EN_SHIFT_U32) |
			((invert & 0x1) << END_PIDF_CFGR_RX_FILTER_INVERT_SHIFT_U32);
		writel_relaxed(value, flt_addr + DEF_FLT_MSG_PIDF_CFGR);
	} else {
		//filter1 receive pid select
		value = (1UL << END_RXFIFO_ADDR_FILTER_EN_SHIFT_U32) |
			((invert & 0x1) << END_PIDF_CFGR_RX_FILTER_INVERT_SHIFT_U32) |
			((pid_st & END_PIDF_CFGR_MASK)  << END_PIDF_CFGR_RX_PID_ST_SHIFT_U32) |
			((pid_end & END_PIDF_CFGR_MASK) << END_PIDF_CFGR_RX_PID_END_SHIFT_U32);
		writel_relaxed(value, flt_addr + FILTER1_MSG_PIDF_CFGR);
	}
}

static int32_t msgbox_set_filter_len(u32 filter, u32 len_st, u32 len_end, u32 invert)
{
	void *flt_addr = NULL;
	FLT1_LEN_CFGR reg = {0};
	int32_t ret = 0;

	/** base addr */
	flt_addr = msgbx_get_filter_base(filter);
	if (filter == FILTER_DEFAULT) {
		IPC_LOG_ERR(" default filter can not set msg len filter!");
		ret = -1;
	} else {
		//filter1 receive pid select
		reg.bit.rx_len_filter_en = 1;
		reg.bit.rx_len_filter_invert = invert & 0x1;
		reg.bit.rx_len_st = len_st & 0xf;
		reg.bit.rx_len_end = len_end & 0xf;
		writel_relaxed(reg.data, flt_addr + End_filter1_MsgH_LenF_CFGR);
	}

	return ret;
}

/*
 * set filter rx fifo threshold
 */
static void msgbox_set_filter_thres(u32 filter, u32 thres)
{
	void *flt_addr = NULL;
	u32 value = thres & 0x3ff;

	/** base addr */
	flt_addr = msgbx_get_filter_base(filter);
	if (filter == FILTER_DEFAULT)
		writel_relaxed(value, flt_addr + DEF_DEFAULT_RXTHRS_CFGR);
	else
		writel_relaxed(value, flt_addr + End_filter_Thrs_CFGR);
}

static void msgbox_set_filter_inter(u32 filter, u32 inter_en)
{
	void *flt_addr = NULL;

	/** base addr */
	flt_addr = msgbx_get_filter_base(filter);
	if (unlikely(!flt_addr))
		return;
	if (filter == FILTER_DEFAULT)
		writel_relaxed(inter_en, flt_addr + DEF_FLT_INTER_EN);
	else
		writel_relaxed(inter_en, flt_addr + FILTER1_EN_INTER);
}

static u32 msgbox_get_filter_inter(u32 filter)
{
	void *flt_addr = NULL;
	u32 inter_enable = 0;
	/** base addr */
	flt_addr = msgbx_get_filter_base(filter);
	if (filter == FILTER_DEFAULT)
		inter_enable = readl_relaxed(flt_addr + DEF_FLT_INTER_EN);
	else
		inter_enable = readl_relaxed(flt_addr + FILTER1_EN_INTER);
	return inter_enable;
}

void msgbx_hw_init_real(void *info)
{
	uint32_t reg_data = 0;
	ST_MSG_HW_INIT *msg_hw_init = info;
#ifdef IPC_STATE_MGT_ENABLE
	u32 cpuid = get_cpu();
	ipc_init_params_t *msgbx_param = msg_hw_init->msgbx_param;
#endif
	 void *flt_addr = msgbx_get_filter_base(0);

	put_cpu();

	// close default filter
    writel_relaxed(0, flt_addr + DEF_FLT_MSG_PIDF_CFGR);

    // close filter 1
	flt_addr = msgbx_get_filter_base(1);
	writel_relaxed(0, flt_addr + End_filter1_RxFIFO_ADDRR);
	writel_relaxed(0, flt_addr + End_filter1_RxFIFO_CFGR);
	writel_relaxed(0, flt_addr + FILTER1_EN_INTER);

	msgbox_set_filter_addr(FILTER_DEFAULT, DEF_FILTER_ST_ADDR, DEF_FILTER_END_ADDR);
	msgbox_set_filter_thres(FILTER_DEFAULT, DEF_FILTER_THRES);
	msgbox_set_filter_pid(FILTER_DEFAULT, DEF_FILTER_PID_ST,
			DEF_FILTER_PID_END, DEF_FILTER_PID_INVERT);

#ifdef IPC_STATE_MGT_ENABLE
	IPC_LOG_DEBUG("cpu%u msgbx_end_mgt_flag:%x", cpuid, msgbx_param->msgbx_end_mgt_flag);
	if (msgbx_param != NULL) {
		if (msgbx_param->msgbx_end_mgt_flag) {
			reg_data |= (msgbx_param->msgbx_end_mgt_flag & (DEF_FILTER_RX_UNDERFLOW_INTR |
						DEF_FILTER_RX_OVERFLOW_INTR | DEF_FILTER_TX_OVERFLOW_INTR));
		}
	} else {
		msg_hw_init->ret = -ERR_PARA;
		IPC_LOG_ERR(" parameter error!");
		return;
	}
#endif

#if defined(MSGBX_HW_TYPE_A2000)
	msg_hw_init->ret = msgbox_sel_ppi_rx_irq(FILTER_DEFAULT, PPI_DEFAULT, cpuid, true);
	if (unlikely(msg_hw_init->ret))
		return;
#endif
	/* enable rx fifo receive interrupt */
	reg_data |= DEF_FILTER_RX_THRS_INTR;
	msgbox_set_filter_inter(FILTER_DEFAULT, reg_data);
	msg_hw_init->ret = 0;
}

/**
 * @brief msgbx hw init function
 *
 * @param msgbx_param
 * @return int32_t
 */
int32_t msgbx_hw_init(uint8_t cpu_id, const ipc_init_params_t *msgbx_param)
{
	u32 cpuid = get_cpu();
	s32 ret = 0;

	put_cpu();
	CHECK_CPUID(cpu_id);
	if (ret < 0)
		return ret;

	IPC_LOG_DEBUG("[%s] cpu%u end%u", __func__, cpuid, cpu_id);
	para_hw_init[cpu_id].msgbx_param = (ipc_init_params_t *)msgbx_param;
	FUNC_SMP_CALL(msgbx_hw_init_real, para_hw_init);
	return para_hw_init[cpu_id].ret;
}

/**
 * @brief msgbx hw deinit
 *
 * @return int32_t
 */
void msgbx_hw_deinit_real(void *info)
{
	ST_MSG_HW_DEINIT *msg_hw_deinit = info;
	msg_hw_deinit->ret = 0;
}

int32_t msgbx_hw_deinit(uint8_t cpu_id)
{
	s32 ret = 0;

	CHECK_CPUID(cpu_id);
	if (ret < 0)
		return ret;

	FUNC_SMP_CALL(msgbx_hw_deinit_real, para_hw_deinit);
	return para_hw_deinit[cpu_id].ret;
}

/**
 * @brief get msgbx info
 *
 * @param hw_info
 * @return int32_t
 */
int32_t msgbx_hw_get_info(uint8_t cpu_id, msgbx_hw_info_t *hw_info)
{
	ST_MSG_INF *msg_info;
	s32 ret = 0;

	CHECK_CPUID(cpu_id);
	if (ret < 0)
		return ret;

	if (cpu_id >= MAX_END_NUM) {
		IPC_LOG_ERR("parameter error!");
		return -ERR_PARA;
	}

	msg_info = &st_msgbx_info[cpu_id];
	hw_info->mbx_flt_cnt = msg_info->ablt_r.bit.filter_num;
	hw_info->is_64_bit = msg_info->ablt_r.bit.is_64_bit;
	hw_info->mbx_end_id = msg_info->ablt_r.bit.endid;
	hw_info->mbx_rxfifo_depth = msg_info->ablt_r2.bit.rx_fifo_depth;
	hw_info->mbx_txfifo_depth = msg_info->ablt_r2.bit.tx_fifo_depth;
	hw_info->mbx_version = msg_info->version;

	return 0;
}

/**
 * @brief msgbx hw init function
 *
 * @param msgbx_param
 * @return int32_t
 */
int32_t msgbx_hw_set_rule_pid(const uint8_t flt_id, const msgbx_flt_rule_pid_t *rule)
{
	int32_t ret = -1;

	if (flt_id < msgbx_get_flt_count()) {
//		msgbox_set_filter_addr(flt_id, (flt_id * END_FILTER_RX_FIFO_DEPTH),
//					(flt_id * END_FILTER_RX_FIFO_DEPTH) +
//					(END_FILTER_RX_FIFO_DEPTH - 1));
		msgbox_set_filter_thres(flt_id, DEF_FILTER_THRES);
		/* enalbe filter1 or default filter receive message interrupt */
		msgbox_set_filter_inter(flt_id, (msgbox_get_filter_inter(flt_id)) |
					FILTER1_THRS_INTR);
		msgbox_set_filter_pid(flt_id, rule->mbx_rx_pid_st, rule->mbx_rx_pid_end,
				      rule->mbx_pid_flt_invert);
		ret = 0;
	} else {
		IPC_LOG_ERR("filter id error:%u", flt_id);
	}

	return ret;
}

int32_t msgbx_hw_set_rule_len(const uint8_t flt_id, const msgbx_flt_rule_len_t *rule)
{
	return msgbox_set_filter_len(flt_id, rule->mbx_rx_len_st,
		rule->mbx_rx_len_end, rule->mbx_len_flt_invert);
}

static void msgbox_set_filter1_res_rule(void *base_addr, const msgbx_flt_rule_user_t *rule)
{
	FLT1_RX_FIFO_CFGR reg = {0};

	reg.data = readl_relaxed(base_addr + End_filter1_RxFIFO_CFGR);

	/* set low 12 bit for head resv */
	writel_relaxed(rule->rx_res_mask,
		       base_addr + End_filter1_MsgH_ResF_MaskR);
	writel_relaxed(rule->rx_res_min & FILTER1_RES_LOW_BIT_MASK,
		       base_addr + End_filter1_MsgH_ResF_MinR);
	writel_relaxed(rule->rx_res_max & FILTER1_RES_LOW_BIT_MASK,
		       base_addr + End_filter1_MsgH_ResF_MaxR);

	/* set high 32 bit for head resv */
	writel_relaxed(rule->rx_res_maskh,
		       base_addr + End_filter1_MsgH_ResF_MaskHR);
	writel_relaxed(rule->rx_res_minh, base_addr + End_filter1_MsgH_ResF_MinHR);
	writel_relaxed(rule->rx_res_maxh, base_addr + End_filter1_MsgH_ResF_MaxHR);

	reg.bit.msgh_flilter_en = 1;
	reg.bit.msg_flilter_invert = rule->msg_flilter_invert & 0x1;
	reg.bit.msg_combi_lh_comp = rule->msg_combi_lh_comp & 0x1;
	writel_relaxed(reg.data, base_addr + End_filter1_RxFIFO_CFGR);
}

static void msgbox_set_filter1_payload_rule(void *base_addr, const msgbx_flt_rule_user_t *rule,
					    u32 pay_rule_id)
{
	FLT1_RX_FIFO_CFGR reg = {0};
	u32 offset = (End_filter1_MsgP2_MaskR - End_filter1_MsgP1_MaskR) * pay_rule_id;
	void *reg_base = base_addr + offset;

	reg.data = readl_relaxed(base_addr + End_filter1_RxFIFO_CFGR);

	/* set low 32 bit for filter payload */
	writel_relaxed(rule->rx_res_mask,
		       reg_base + End_filter1_MsgP1_MaskR);
	writel_relaxed(rule->rx_res_min & FILTER1_PAYLOAD_LOW_BIT_MASK,
		       reg_base + End_filter1_MsgP1_MinR);
	writel_relaxed(rule->rx_res_max & FILTER1_PAYLOAD_LOW_BIT_MASK,
		       reg_base + End_filter1_MsgP1_MaxR);

	/* set high 32 bit for filter payload */
	writel_relaxed(rule->rx_res_maskh,
		       reg_base + End_filter1_MsgP1_MaskHR);
	writel_relaxed(rule->rx_res_minh, reg_base + End_filter1_MsgP1_MinHR);
	writel_relaxed(rule->rx_res_maxh, reg_base + End_filter1_MsgP1_MaxHR);

	switch (pay_rule_id) {
	case MSG_FILTER1_RULE_PAYLOAD1:
		reg.bit.msgp1_filter_en = 1;
		reg.bit.msgp1_filter_invert = rule->msg_flilter_invert & 0x1;
		reg.bit.msgp1_combi_lh_comp = rule->msg_combi_lh_comp & 0x1;
		break;
	case MSG_FILTER1_RULE_PAYLOAD2:
		reg.bit.msgp2_filter_en = 1;
		reg.bit.msgp2_filter_invert = rule->msg_flilter_invert & 0x1;
		reg.bit.msgp2_combi_lh_comp = rule->msg_combi_lh_comp & 0x1;
		break;
	case MSG_FILTER1_RULE_PAYLOAD3:
		reg.bit.msgp3_filter_en = 1;
		reg.bit.msgp3_filter_invert = rule->msg_flilter_invert & 0x1;
		reg.bit.msgp3_combi_lh_comp = rule->msg_combi_lh_comp & 0x1;
		break;
	case MSG_FILTER1_RULE_PAYLOAD4:
		reg.bit.msgp4_filter_en = 1;
		reg.bit.msgp4_filter_invert = rule->msg_flilter_invert & 0x1;
		reg.bit.msgp4_combi_lh_comp = rule->msg_combi_lh_comp & 0x1;
		break;
	default:
		break;
	}

	writel_relaxed(reg.data, reg_base + End_filter1_RxFIFO_CFGR);
}

static void msgbox_set_filter_combi_mode(void *reg_base, u32 combi_mode)
{
	FLT1_RX_FIFO_CFGR reg = {0};

	reg.data = readl_relaxed(reg_base + End_filter1_RxFIFO_CFGR);
	reg.bit.filter_combi_mode = (combi_mode & 0x3);
	writel_relaxed(reg.data, reg_base + End_filter1_RxFIFO_CFGR);
}

int32_t msgbx_hw_set_rule_user(const uint8_t flt_id, const msgbx_flt_rule_user_t *rule, uint8_t loc)
{

	uint32_t ret = -1;
	uint32_t offset = 0;

	void *flt_addr = msgbx_get_filter_base(flt_id);

	if (flt_id > 0 && flt_id < msgbx_get_flt_count()) {
		/* combi mode set logic and */
		msgbox_set_filter_combi_mode(flt_addr, COMBI_MODE_AND);
		switch (loc) {
		case MSGBX_FLT_RULE_HEADER:
			msgbox_set_filter1_res_rule(flt_addr, rule);
			ret = 0;
			break;
		case MSGBX_FLT_RULE_PAY1:
		case MSGBX_FLT_RULE_PAY2:
		case MSGBX_FLT_RULE_PAY3:
		case MSGBX_FLT_RULE_PAY4:
			offset = (loc-MSGBX_FLT_RULE_PAY1);
			msgbox_set_filter1_payload_rule(flt_addr, rule, offset);
			ret = 0;
			break;
		default:
			IPC_LOG_ERR("rule->cfg_loc error:%u", loc);
			break;
		}
	} else {
		IPC_LOG_ERR("filter id error:%u", flt_id);
	}

	return ret;
}

/**
 * @brief msgbx hw config filter
 *
 * @param flt_id
 * @param rule
 * @return int32_t
 */
void msgbx_hw_set_flt_cfg_real(void *info)
{
	int32_t ret = -1;
	ST_MSG_HW_SET_FLT_CFG *msg_hw_set_flt_cfg = (ST_MSG_HW_SET_FLT_CFG *)info;
	uint8_t flt_id = msg_hw_set_flt_cfg->flt_id;
	msgbx_flt_rule_cfg_t *rule = msg_hw_set_flt_cfg->rule;

	switch (rule->cfg_type) {
	case MSGBX_FLT_RULE_PID:
		ret = msgbx_hw_set_rule_pid(flt_id, &rule->rule_pid);
		break;
	case MSGBX_FLT_RULE_LEN:
		ret = msgbx_hw_set_rule_len(flt_id, &rule->rule_len);
		break;
	case MSGBX_FLT_RULE_USER:
		ret = msgbx_hw_set_rule_user(flt_id, &rule->rule_user, rule->cfg_loc);
		break;
	default:
		break;
	}

	msg_hw_set_flt_cfg->ret = ret;
}

/**
 * @brief msgbx hw config filter
 *
 * @param flt_id
 * @param rule
 * @return int32_t
 */
int32_t msgbx_hw_set_flt_cfg(uint8_t cpu_id, const uint8_t flt_id, const msgbx_flt_rule_cfg_t *rule)
{
	s32 ret = 0;

	CHECK_CPUID(cpu_id);
	if (ret < 0)
		return ret;

	para_hw_set_flt_cfg[cpu_id].rule = (msgbx_flt_rule_cfg_t *)rule;
	para_hw_set_flt_cfg[cpu_id].flt_id = flt_id;

	FUNC_SMP_CALL(msgbx_hw_set_flt_cfg_real, para_hw_set_flt_cfg);
	return para_hw_set_flt_cfg[cpu_id].ret;
}

/**
 * @brief get msgbx hw config filter
 *
 * @param flt_id
 * @param rule
 * @return int32_t
 */
void msgbx_hw_get_flt_info_real(void *info)
{
	ST_MSG_HW_SET_FLT_CFG *msg_hw_get_flt_info = info;
	int32_t ret = -1;

	msg_hw_get_flt_info->ret = ret;
}

/**
 * @brief get msgbx hw config filter
 *
 * @param flt_id
 * @param rule
 * @return int32_t
 */
int32_t msgbx_hw_get_flt_info(uint8_t cpu_id, const uint8_t flt_id, msgbx_flt_rule_cfg_t *info)
{
	s32 ret = 0;

	CHECK_CPUID(cpu_id);
	if (ret < 0)
		return ret;

	para_hw_get_flt_cfg[cpu_id].rule = (msgbx_flt_rule_cfg_t *)info;
	para_hw_get_flt_cfg[cpu_id].flt_id = flt_id;

	FUNC_SMP_CALL(msgbx_hw_get_flt_info_real, para_hw_get_flt_cfg);
	return para_hw_get_flt_cfg[cpu_id].ret;
}

/**
 * @brief msgbx hw clear filter config
 *
 * @param flt_id
 * @return int32_t
 */
void msgbx_hw_clr_flt_cfg_real(void *info)
{
	u32 value = 0;
	int32_t ret = 0;
	ST_MSG_HW_CLR_FLT_CFG *msg_hw_clr_flt_cfg = (ST_MSG_HW_CLR_FLT_CFG *)info;
	uint8_t flt_id = msg_hw_clr_flt_cfg->flt_id;
	void *flt_addr = msgbx_get_filter_base(flt_id);

	if (!flt_id) {
		value = readl_relaxed(flt_addr + DEF_FLT_MSG_PIDF_CFGR);
		value &= ~(1ull << END_PIDF_CFGR_RX_FILTER_EN_SHIFT_U32);
		writel_relaxed(value, flt_addr + DEF_FLT_MSG_PIDF_CFGR);
	} else if (flt_id < msgbx_get_flt_count()) {
		value = readl_relaxed(flt_addr + End_filter1_RxFIFO_ADDRR);
		value &= ~(1ull << END_RXFIFO_ADDR_FILTER_EN_SHIFT_U32);
		writel_relaxed(value, flt_addr + End_filter1_RxFIFO_ADDRR);
	} else {
		ret = -ERR_PARA;
	}

	msg_hw_clr_flt_cfg->ret = ret;
}

/**
 * @brief msgbx hw clear filter config
 *
 * @param flt_id
 * @return int32_t
 */
static int32_t msgbx_hw_clr_flt_cfg(uint8_t cpu_id, const uint8_t flt_id)
{
	s32 ret = 0;

	CHECK_CPUID(cpu_id);
	if (ret < 0)
		return ret;

	para_hw_clr_flt_cfg[cpu_id].flt_id = flt_id;
	FUNC_SMP_CALL(msgbx_hw_clr_flt_cfg_real, para_hw_clr_flt_cfg);
	return para_hw_clr_flt_cfg[cpu_id].ret;
}

/**
 * @brief
 *
 * @param base_reg msgbx base reg aadr
 * @param msg64 message
 * @return int32_t 0:valid, -1: invalid
 */

static int32_t msgbx_header_is_valid(void *base_reg, ST_MSG_MESSAGE64 *msg64)
{
	int32_t ret = -1;
	uint8_t pid = (readl_relaxed(base_reg + DEF_FIL_ABLT_R) & 0xFF);

#if defined(MSGBX_HW_TYPE_C1200)
	if ((msg64->head.bit.cid != pid) && (msg64->head.bit.pid == pid) &&
	   (msg64->head.bit.len <= MSG_64_MAX_LEN)) {
		/* check CID, if CID is illegal, then the msgbox system will crash */
		switch (msg64->head.bit.cid) {
		case PID_CMN_MSG_END0:
		case PID_CMN_MSG_END1:
		case PID_CMN_MSG_END2:
		case PID_CMN_MSG_END3:
		case PID_CMN_MSG_END4:
		case PID_CMN_MSG_END5:
		case PID_CMN_MSG_END6:
		case PID_CMN_MSG_END7:
		case PID_DB_MSG_END0:
		case PID_DB_MSG_END1:
		case PID_ISPCV_MSG_END0:
		case PID_ISPCV_MSG_END1:
		case PID_ISPCV_MSG_END2:
		case PID_ISPCV_MSG_END3:
		case PID_RISCV_MSG_END0:
		case PID_NET_MSG_END0:
		case PID_DMA_MSG_END0:
		case PID_DMA_MSG_END1:
		case PID_R5_SW_MSG_END0:
		case PID_R5_SW_MSG_END1:
		case PID_R5_SW_MSG_END2:
		case PID_R5_SW_MSG_END3:
		case PID_R5_SW_MSG_END4:
		case PID_R5_SW_MSG_END5:
		case PID_R5_SECURE_END0:
		case PID_R5_SECURE_END1:
		case PID_R5_SAFETY_END0:
		case PID_R5_SAFETY_END1:
		case PID_R5_REALTIME_END0:
		case PID_R5_REALTIME_END1:
		case PID_R5_REALTIME_END2:
		case PID_R5_REALTIME_END3:
		case PID_R5_REALTIME_END4:
		case PID_R5_REALTIME_END5:
		case PID_MEDIA_MSG_END0:
			ret = 0;
			break;
		default:
			IPC_LOG_ERR("send message error, illegal cid:%x, pid %d\n",
				     msg64->head.bit.cid, pid);
			ret = -2;
			break;
		}
	} else {
		IPC_LOG_ERR("send message head:0x%llx error, real pid:%x\n",
			     msg64->head.data, pid);
		ret = -3;
	}
#elif defined(MSGBX_HW_TYPE_A2000)
if ((msg64->head.bit.cid != pid) && (msg64->head.bit.pid == pid) &&
	   (msg64->head.bit.len <= MSG_64_MAX_LEN)) {
		/* check CID, if CID is illegal, then the msgbox system will crash */
		switch (msg64->head.bit.cid) {
		case ID_A78_MSG_NODE0_END0:
		case ID_A78_MSG_NODE0_END1:
		case ID_A78_MSG_NODE0_END2:
		case ID_A78_MSG_NODE0_END3:
		case ID_A78_MSG_NODE1_END0:
		case ID_A78_MSG_NODE1_END1:
		case ID_A78_MSG_NODE1_END2:
		case ID_A78_MSG_NODE1_END3:
		case ID_A78_MSG_NODE2_END0:
		case ID_A78_MSG_NODE2_END1:
		case ID_A78_MSG_NODE2_END2:
		case ID_A78_MSG_NODE2_END3:
		case ID_A78_MSG_NODE3_END0:
		case ID_A78_MSG_NODE3_END1:
		case ID_A78_MSG_NODE3_END2:
		case ID_A78_MSG_NODE3_END3:
		case ID_R52_DOWN_PADSYS_MSG_NODE4_END0:
		case ID_R52_DOWN_PADSYS_MSG_NODE4_END1:
		case ID_R52_DOWN_PADSYS_MSG_NODE4_END2:
		case ID_R52_DOWN_PADSYS_MSG_NODE4_END3:
		case ID_R52_DOWN_PADSYS_MSG_NODE4_END4:
		case ID_R52_DOWN_PADSYS_MSG_NODE4_END5:
		case ID_ISP_CV_MSG_NODE5_END0:
		case ID_ISP_CV_MSG_NODE5_END1:
		case ID_ISP_CV_MSG_NODE5_END2:
		case ID_ISP_CV_MSG_NODE5_END3:
		case ID_SEC_MSG_NODE6_END0:
		case ID_SAFE_MSG_NODE7_END0:
		case ID_NET_MSG_NODE8_END0:
		case ID_NET_MSG_NODE8_END1:
		case ID_R52_UP_PADSYS_MSG_NODE9_END0:
		case ID_R52_UP_PADSYS_MSG_NODE9_END1:
			ret = 0;
			break;
		default:
			IPC_LOG_ERR("send message error, illegal cid:%x",
				     msg64->head.bit.cid);
			ret = -1;
			break;
		}
	} else {
		IPC_LOG_ERR("send message head:0x%llx error, real pid:%x",
			     msg64->head.data, pid);
	}
#endif

	return ret;
}

int msgbx_hw_send_msg_direct(rw_msg_t *msg)
{
	s32 ret = 0;
	uint32_t reg_data = 0, time_out_cnt = 0, i = 0, cpuid = 0, real_cpuid = 0;
	uint8_t sem_id = 0xff;
	void *tx_fifo_addr;
	void *reg_base = msgbx_get_filter_base(FILTER_DEFAULT);
	ST_MSG_MESSAGE64 *msg_ptr = NULL;
	unsigned long flags = 0;
	uint8_t is_in_same_sys = 0;

	if(!msg)
		return -1;
	msg_ptr = (ST_MSG_MESSAGE64 *)msg;
	
	cpuid = get_cpuid_by_endid(msg_ptr->head.bit.pid);

	// real_cpuid = get_cpu();
	// put_cpu();

	tx_fifo_addr = msgbx_get_filter_tx_fifo_base();
	/*
	 * bit20 of msg header is 'is_64bit'
	 * bit21 of msg header is 'is_Nonsec'
	 * if these bits are not set, this would have error when testing ecc inject
	 */
	set_bit(MSGHEADER_IS_64BIT, (unsigned long *)&msg_ptr->head.data);
	// clear_bit(MSGHEADER_IS_NONSEC, (unsigned long *)&msg_ptr->head.data);
	//bugfix : this bit is non secure world, A core must set is 1 . 
	set_bit(MSGHEADER_IS_NONSEC, (unsigned long *)&msg_ptr->head.data);

	real_cpuid = get_cpu();
	put_cpu();

	is_in_same_sys = get_cpuid_by_endid(msg_ptr->head.bit.cid);
	if (is_in_same_sys == real_cpuid) {
		spin_trylock_irqsave(&vpid_send_lock[real_cpuid],flags);
		ret = ipc_hw_recv_msg_notify(real_cpuid, 0, (rw_msg_t *)msg);
		spin_unlock_irqrestore(&vpid_send_lock[real_cpuid],flags);
		return ret;
	}

	// if (msg_ptr->head.bit.cid < g_start_pid + NR_CPUS && msg_ptr->head.bit.cid >= g_start_pid) {
	// 	if ((msg_ptr->head.bit.cid - g_start_pid) == real_cpuid) {
	// 		spin_trylock_irqsave(&send_msg_lock[real_cpuid],flags);
	// 		ret = ipc_hw_recv_msg_notify(real_cpuid, 0, (rw_msg_t *)msg);
	// 		spin_unlock_irqrestore(&send_msg_lock[real_cpuid],flags);
	// 		return ret;
	// 	}
	// }

	// ret = msgbx_header_is_valid(reg_base, msg_ptr);
	// spin_trylock_irqsave(&send_msg_lock[real_cpuid],flags);
	if (!ret) {
		/*wait tx fifo available*/
		do {
			reg_data = readl_relaxed(reg_base + DEF_Tx_FIFO_Available);
			time_out_cnt++;
			if (time_out_cnt > MSGBX_WAIT_TIMEOUT_CNT) {
				// add statistic due to unavailable tx fifo
				g_stats.tx_fifo_unavail_stats[cpuid]++;
				// spin_unlock_irqrestore(&send_msg_lock[real_cpuid],flags);
				return -3;
			}
		} while (reg_data == 0);

		spin_trylock_irqsave(&send_msg_lock[real_cpuid],flags);
		writeq_relaxed(msg_ptr->head.data, tx_fifo_addr);
		for (i = 0; i < msg_ptr->head.bit.len; i++) {
			/* last payload check wheterh to cross switch */
			if ((i + 1) == msg_ptr->head.bit.len)
				sem_id = get_msg_cross_sw_lock(msg_ptr->head.bit.pid,
					 msg_ptr->head.bit.cid);

			writeq_relaxed(msg_ptr->payload[i], tx_fifo_addr);

			if (((i + 1) == msg_ptr->head.bit.len) && (sem_id <= SW2_TO_SW1_LOCK_ID)) {
				udelay(1);
				release_cross_sw_lock(sem_id);
			}
		}
		g_stats.tx_pidcid_stats[cpuid][msg_ptr->head.bit.cid]++;
		spin_unlock_irqrestore(&send_msg_lock[real_cpuid],flags);
	}
	// spin_unlock_irqrestore(&send_msg_lock[real_cpuid],flags);
	return 0;
}

/**
 * @brief msgbx hw send msg data
 *
 * @work work_struct pointer for msgbox queue
 * @return int32_t 0: is send ok, other: send failed
 */
void msgbx_hw_send_msg_real(struct work_struct *work)
{
	s32 ret = -1;
	uint32_t reg_data = 0, time_out_cnt = 0, i = 0, cpuid = 0, real_cpuid = 0;
#if defined(MSGBX_HW_TYPE_C1200)
	uint8_t sem_id = 0xff;
#endif
	void *tx_fifo_addr = g_ipc_msgbx->txfifo_base;
	void *reg_base = g_ipc_msgbx->fcsr_base;
	ST_MSG_HW_SEND_MSG *msg_hw_send_msg = NULL;
	ST_MSG_MESSAGE64 *msg_ptr = NULL;
	unsigned long flags = 0;
	uint64_t cur_time = 0, cost_time = 0;
#if 0
	uint32_t handle = 0;
	uint8_t max_cpu_id = 0;
#endif

	if (unlikely(!work))
		return;

	msg_hw_send_msg = container_of(work, ST_MSG_HW_SEND_MSG, wk);
	msg_ptr = (ST_MSG_MESSAGE64 *)msg_hw_send_msg->msg;
	if (unlikely(!msg_ptr))
		goto free_msg;
	cpuid = get_cpuid_by_endid(msg_ptr->head.bit.pid);

	real_cpuid = get_cpu();
	put_cpu();

#if 0
	max_cpu_id = ENDID_TO_CPUID(msg_ptr->head.bit.pid);
	handle = ((msg_ptr->head.bit.sid << CHANNEL_COUNT_BITS) | msg_ptr->head.bit.fid);
	if (msgbx_max_send_time[max_cpu_id][handle] < cost_time) {
		msgbx_max_send_time[max_cpu_id][handle] = cost_time;
	}
#endif
	if (msg_ptr->head.bit.cid == NET_0) {
		cur_time = ktime_get_raw();
		cost_time = cur_time -  msg_hw_send_msg->send_time;
		if (bstn_max_send_time < cost_time)
			bstn_max_send_time = cost_time;
	}

	// add workaround for isp fw recv msg out of order.
	if (msg_ptr->head.bit.cid == ISPCV_4)
		udelay(10);

	/*
	 * bit20 of msg header is 'is_64bit'
	 * bit21 of msg header is 'is_Nonsec'
	 * if these bits are not set, this would have error when testing ecc inject
	 */
	set_bit(MSGHEADER_IS_64BIT, (unsigned long *)&msg_ptr->head.data);
	// clear_bit(MSGHEADER_IS_NONSEC, (unsigned long *)&msg_ptr->head.data);
	//bugfix : this bit is non secure world, A core must set is 1 . 
	set_bit(MSGHEADER_IS_NONSEC, (unsigned long *)&msg_ptr->head.data);

	IPC_LOG_DEBUG("msgbx end%d send head=0x%llx data[0]=0x%llx\n",
		    real_cpuid, msg_ptr->head.data, msg_ptr->payload[0]);
	ret = msgbx_header_is_valid(reg_base, msg_ptr);
	if (!ret) {
		/*wait tx fifo available*/
		do {
			reg_data = readl_relaxed(reg_base + DEF_Tx_FIFO_Available);
			time_out_cnt++;
			if (time_out_cnt > MSGBX_WAIT_TIMEOUT_CNT) {
				// add statistic due to unavailable tx fifo
				g_stats.tx_fifo_unavail_stats[cpuid]++;
				goto free_all;
			}
		} while (reg_data == 0);

		spin_trylock_irqsave(&send_msg_lock[real_cpuid],flags);
		/*write msg head*/
		writeq_relaxed(msg_ptr->head.data, tx_fifo_addr);
		for (i = 0; i < msg_ptr->head.bit.len; i++) {
#if defined(MSGBX_HW_TYPE_C1200)
			/* last payload check wheterh to cross switch */
			if ((i + 1) == msg_ptr->head.bit.len)
				sem_id = get_msg_cross_sw_lock(msg_ptr->head.bit.pid,
					 msg_ptr->head.bit.cid);
#endif
			writeq_relaxed(msg_ptr->payload[i], tx_fifo_addr);
#if defined(MSGBX_HW_TYPE_C1200)
			if (((i + 1) == msg_ptr->head.bit.len) && (sem_id <= SW2_TO_SW1_LOCK_ID)) {
				udelay(1);
				release_cross_sw_lock(sem_id);
			}
#endif
		}
		g_stats.tx_pidcid_stats[cpuid][msg_ptr->head.bit.cid]++;
		spin_unlock_irqrestore(&send_msg_lock[real_cpuid],flags);
	}

free_all:
	kfree(msg_hw_send_msg->msg);
free_msg:
	kfree(msg_hw_send_msg);
}

/**
 * @brief check msg
 * @msg
 * @return 0: is ok, other: failed
 */
// static int msgbx_msg_check(ST_MSG_HW_SEND_MSG *msg_hw_send_msg, u32 cpuid)
// {
// 	int32_t ret = 0;
//	u32 pid = st_msgbx_info[cpuid].ablt_r.bit.endid;
// 	ST_MSG_MESSAGE64 *msg_ptr = (ST_MSG_MESSAGE64 *)msg_hw_send_msg->msg;

// 	if ((msg_ptr->head.bit.pid != pid) || (msg_ptr->head.bit.cid == pid))
// 		ret = 1;

// 	msg_hw_send_msg->ret = ret;
// 	return ret;
// }

int32_t msgbx_hw_send_msg(uint8_t cpu_id, const rw_msg_t *msg)
{
	// ST_MSG_HW_SEND_MSG *hw_msg = NULL;
	s32 ret = 0;
	// unsigned long flags = 0;
	uint32_t real_cpuid = 0;
	real_cpuid = get_cpu();
	put_cpu();

	BUG_ON(in_interrupt());

	CHECK_CPUID(cpu_id);
	if (ret < 0)
		return ret;

	// if (real_cpuid == msg->header.cid) {
	// 	ret = ipc_hw_recv_msg_notify(real_cpuid, 0, (rw_msg_t *)msg);
	// 	return ret;
	// } else {
	// 	spin_trylock_irqsave(&send_msg_lock[real_cpuid],flags);
	// 	ret = msgbx_hw_send_msg_direct((rw_msg_t *)msg);
	// 	spin_unlock_irqrestore(&send_msg_lock[real_cpuid],flags);
	// 	return ret;
	// }
	// spin_trylock_irqsave(&vpid_send_lock[real_cpuid],flags);
	ret = msgbx_hw_send_msg_direct((rw_msg_t *)msg);
	// spin_unlock_irqrestore(&vpid_send_lock[real_cpuid],flags);

	// hw_msg = kzalloc(sizeof(ST_MSG_HW_SEND_MSG), GFP_KERNEL);
	// if (unlikely(!hw_msg)) {
	// 	IPC_LOG_ERR("can't alloc for hw_msg\n");
	// 	return -ENOMEM;
	// }

	// hw_msg->msg = kzalloc(sizeof(rw_msg_t), GFP_KERNEL);
	// if (unlikely(!hw_msg->msg)) {
	// 	IPC_LOG_ERR("can't alloc for msg\n");
	// 	kfree(hw_msg);
	// 	return -ENOMEM;
	// }
	// hw_msg->send_time = ktime_get_raw();
	// memcpy(hw_msg->msg, (rw_msg_t *)msg, sizeof(rw_msg_t));
	// g_stats.tx_stats[current->thread_info.cpu][cpu_id]++;

	// FUNC_WQ_CALL(msgbx_hw_send_msg_real, msgbx_msg_check,
	// 	     hw_msg, msgbx_wq[cpu_id]);
	// if (ret) {
	// 	kfree(hw_msg->msg);
	// 	kfree(hw_msg);
	// 	return -ENOMEM;
	// }
	return ret;
}
/**
 * @brief msgbx get a msg from recv fifo
 * @info msg infomation
 */
void msgbx_hw_recv_msg_real(void *info)
{
	int32_t ret = -1;
	u32 i = 0, read_len = 0;
	ST_MSG_HW_RECV_MSG *msg_hw_recv_msg = (ST_MSG_HW_RECV_MSG *)info;
	uint8_t fid = msg_hw_recv_msg->flt_id;
	void *read_addr = msgbx_get_filter_rx_fifo_base(fid);
	rw_msg_t *msg = msg_hw_recv_msg->msg;
	ST_MSG_MESSAGE64 *msg_ptr;

	if (fid < msgbx_get_flt_count()) {
		if (msgbox_get_msg_num(msgbx_get_filter_base(fid), fid)) {
			msg_ptr = (ST_MSG_MESSAGE64 *)msg;
			msg_ptr->head.data = readq_relaxed(read_addr);
			read_len = msg_ptr->head.bit.is_64_bit ? msg_ptr->head.bit.len :
				   (msg_ptr->head.bit.len / 2);
			if (read_len <= MSG_64_MAX_LEN) {
				for (i = 0; i < read_len; i++)
					msg_ptr->payload[i] = readq_relaxed(read_addr);
			}
			ret = 0;
		} else {
			ret = -1;
		}
	} else {
		IPC_LOG_ERR("filter id:0x%x error", fid);
	}

	msg_hw_recv_msg->ret = ret;
}

int32_t msgbx_hw_recv_msg(const uint8_t endid, const uint8_t fid, rw_msg_t *msg)
{
	s32 ret = 0;
	u8 cpu_id = endid;

	CHECK_CPUID(cpu_id);
	if (ret < 0)
		return ret;

	para_hw_recv_msg[cpu_id].msg = msg;
	para_hw_recv_msg[cpu_id].flt_id = (uint8_t)fid;
	FUNC_SMP_CALL(msgbx_hw_recv_msg_real, para_hw_recv_msg);
	return para_hw_recv_msg[cpu_id].ret;
}

int32_t msgbx_hw_get_time(uint64_t *timestamp)
{
	*timestamp = ktime_get_raw();
	return 0;
}

/**
 * @brief enable msgbx statues management
 *
 * @param cfg
 * @return int32_t a
 */
void msgbx_hw_sts_mgt_enable_real(void *info)
{
	void *flt_csr = msgbx_get_filter_base(FILTER_DEFAULT);
	ST_MSG_HW_STS_MGT_ENABLE *msg_hw_sts_mgt_enable = (ST_MSG_HW_STS_MGT_ENABLE *)info;
	writel_relaxed(msg_hw_sts_mgt_enable->flag, flt_csr + END_FMU_SAFETY_INTR_EN);
	msg_hw_sts_mgt_enable->ret = 0;
}

int32_t msgbx_hw_sts_mgt_enable(uint8_t cpu_id, const uint32_t flag)
{
	s32 ret = 0;

	CHECK_CPUID(cpu_id);
	if (ret < 0)
		return ret;

	para_hw_sts_mgt_enable[cpu_id].flag = (uint32_t)flag;
	FUNC_SMP_CALL(msgbx_hw_sts_mgt_enable_real, para_hw_sts_mgt_enable);
	return para_hw_sts_mgt_enable[cpu_id].ret;
}

/**
 * @brief disable msgbx status management
 *
 * @return int32_t
 */
void msgbx_hw_sts_mgt_disable_real(void *info)
{
	void *flt_csr = msgbx_get_filter_base(FILTER_DEFAULT);
	ST_MSG_HW_DEINIT *msg_hw_sts_mgt_disable = info;
	writel_relaxed(0, flt_csr + END_FMU_SAFETY_INTR_EN);
	msg_hw_sts_mgt_disable->ret = 0;
}

int32_t msgbx_hw_sts_mgt_disable(uint8_t cpu_id)
{
	s32 ret = 0;

	CHECK_CPUID(cpu_id);
	if (ret < 0)
		return ret;

	FUNC_SMP_CALL(msgbx_hw_sts_mgt_disable_real, para_hw_sts_mgt_disable);
	return para_hw_sts_mgt_disable[cpu_id].ret;
}

/**
 * @brief msgbx filter state management config
 *
 * @param flt_id filter id
 * @param flag
 * @param flt_thrs
 * @return int32_t
 */
void msgbx_hw_flt_mgt_enble_real(void *info)
{
	int32_t ret = -1;
	ST_MSG_HW_FLT_MGT_ENABLE *msg_hw_flt_mgt_enable = info;
	uint8_t flt_id = msg_hw_flt_mgt_enable->flt_id;
	uint8_t flag = msg_hw_flt_mgt_enable->flag;

	if (flt_id < msgbx_get_flt_count()) {
		if (!flt_id) {
			msgbox_set_filter_inter((u32)flt_id, (flag & (DEF_FILTER_RX_UNDERFLOW_INTR |
				DEF_FILTER_RX_OVERFLOW_INTR | DEF_FILTER_TX_OVERFLOW_INTR)));
		} else {
			msgbox_set_filter_inter((u32)flt_id, (flag & (FILTER1_UNDERFLOW_INTR |
							FILTER1_OVERFLOW_INTR)));
		}
		ret = 0;
	}
	msg_hw_flt_mgt_enable->ret = ret;
}

int32_t msgbx_hw_flt_mgt_enble(uint8_t cpu_id, const uint8_t flt_id, const uint8_t flag)
{
	s32 ret = 0;

	CHECK_CPUID(cpu_id);
	if (ret < 0)
		return ret;

	para_hw_flt_mgt_enable[cpu_id].flt_id = (uint8_t)flt_id;
	para_hw_flt_mgt_enable[cpu_id].flag = (uint8_t)flag;
	FUNC_SMP_CALL(msgbx_hw_flt_mgt_enble_real, para_hw_flt_mgt_enable);
	return para_hw_flt_mgt_enable[cpu_id].ret;
}
/**
 * @brief disable msgbx filter management
 *
 * @param flt_id
 * @return int32_t
 */
void msgbx_hw_flt_mgt_disable_real(void *info)
{
	int32_t ret = -1;
	ST_MSG_HW_CLR_FLT_CFG *msg_hw_flt_mgt_disable = info;
	uint8_t flt_id = msg_hw_flt_mgt_disable->flt_id;
	void *flt_addr = msgbx_get_filter_base((u32)flt_id);
	u32 temp = 0;

	if (flt_id < msgbx_get_flt_count()) {
		if (!flt_id) {
			temp = readl_relaxed(flt_addr + DEF_FLT_INTER_EN);
			temp &= ~(DEF_FILTER_RX_UNDERFLOW_INTR |
				  DEF_FILTER_RX_OVERFLOW_INTR |
				  DEF_FILTER_TX_OVERFLOW_INTR);
			msgbox_set_filter_inter((u32)flt_id, temp);
		} else {
			temp = readl_relaxed(flt_addr + FILTER1_EN_INTER);
			temp &= ~(FILTER1_UNDERFLOW_INTR | FILTER1_OVERFLOW_INTR);
			msgbox_set_filter_inter((u32)flt_id, temp);
		}
		ret = 0;
	}
	msg_hw_flt_mgt_disable->ret = ret;
}

int32_t msgbx_hw_flt_mgt_disable(uint8_t cpu_id, const uint8_t flt_id)
{
	s32 ret = 0;

	CHECK_CPUID(cpu_id);
	if (ret < 0)
		return ret;

	para_hw_flt_mgt_disable[cpu_id].flt_id = (uint8_t)flt_id;
	FUNC_SMP_CALL(msgbx_hw_flt_mgt_disable_real, para_hw_flt_mgt_disable);
	return para_hw_flt_mgt_disable[cpu_id].ret;
}

 /**
  * @brief  msgbx fault handle
  *
  * @param type
  * @param id
  * @param hdl
  * @return int32_t
  */
void msgbx_hw_err_hdl_real(void *info)
{
	int32_t ret = -1;
	ST_MSG_HW_ERR_HDL *msg_hw_err_hdl = info;
	msg_hw_err_hdl->ret = ret;
}

int32_t msgbx_hw_err_hdl(const uint8_t endid, const uint8_t id,const uint32_t hdl)
{
	s32 ret = 0;
	u8 cpu_id = endid;

	CHECK_CPUID(cpu_id);
	if (ret < 0)
		return ret;

	para_hw_err_hdl[cpu_id].id = (uint8_t)id;
	para_hw_err_hdl[cpu_id].hdl = hdl;
	FUNC_SMP_CALL(msgbx_hw_err_hdl_real, para_hw_err_hdl);
	return para_hw_err_hdl[cpu_id].ret;
}

int32_t msgbx_hw_set_endmap(const uint8_t endid, const uint8_t idx, const uint8_t status)
{
#if defined(MSGBX_HW_TYPE_A2000)
#endif
	return 0;
}

int32_t msgbx_hw_get_endmap(const uint8_t endid, sts_endmap_t *map)
{
#if defined(MSGBX_HW_TYPE_A2000)
#endif
	return 0;
}

int32_t msgbx_hw_update_endmap(const uint8_t endid, const uint8_t updated_chipid)
{
	return 0;
}

int32_t msgbx_hw_get_hw_counter(const uint8_t endid, const uint8_t fid, msgbox_hw_counter_t *hw_cnt)
{
#if defined(MSGBX_HW_TYPE_A2000)
#endif
	return 0;
}

int32_t msgbx_hw_clr_hw_counter(const uint8_t endid, const uint8_t fid, const uint32_t flag)
{
#if defined(MSGBX_HW_TYPE_A2000)
#endif
	return 0;
}

static int32_t msgbx_ops_register()
{
	ipc_hw_ops.ipc_hw_init = msgbx_hw_init;
	ipc_hw_ops.ipc_hw_deinit = msgbx_hw_deinit;
	ipc_hw_ops.ipc_hw_get_info = msgbx_hw_get_info;
	ipc_hw_ops.ipc_hw_set_flt_cfg = msgbx_hw_set_flt_cfg;
	ipc_hw_ops.ipc_hw_get_flt_info = msgbx_hw_get_flt_info;
	ipc_hw_ops.ipc_hw_clr_flt_cfg = msgbx_hw_clr_flt_cfg;
	ipc_hw_ops.ipc_hw_send_msg = msgbx_hw_send_msg;
	ipc_hw_ops.ipc_hw_get_msg = msgbx_hw_recv_msg;
	ipc_hw_ops.ipc_hw_get_time = msgbx_hw_get_time;
	ipc_hw_ops.ipc_hw_fmu_mgt_enble = msgbx_hw_sts_mgt_enable;
	ipc_hw_ops.ipc_hw_flt_mgt_enble = msgbx_hw_flt_mgt_enble;
	ipc_hw_ops.ipc_hw_err_hdl = msgbx_hw_err_hdl;
	ipc_hw_ops.ipc_hw_set_endmap = msgbx_hw_set_endmap;
	ipc_hw_ops.ipc_hw_get_endmap = msgbx_hw_get_endmap;
	ipc_hw_ops.ipc_hw_update_endmap = msgbx_hw_update_endmap;
	ipc_hw_ops.ipc_hw_get_hw_counter = msgbx_hw_get_hw_counter;
	ipc_hw_ops.ipc_hw_clr_hw_counter = msgbx_hw_clr_hw_counter;
	return 0;
}

