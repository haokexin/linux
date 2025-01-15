/*
 * IPC: Linux device driver for Blck Sesame Technologies inter-processor
 * communication
 *
 */
#include <linux/device.h>
#include <linux/platform_device.h>
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

#include "ipc_hw_layer.h"
#include "ipc_hw_impl.h"

#include "ipc_msgbox_controller.h"

/********************* macros *******************/
#define IPC_DRIVER_NAME	 "ipc_msgbox"
#define CPU_NR		 8
#define SEND_RETRY_TIMES 10000
#define MAX_END_NUM (8)

#define MSGBOX_DEBUG 0
#if (MSGBOX_DEBUG == 0)
#define PRINT_DEBUG IPC_LOG_DEBUG
#else
#define PRINT_DEBUG IPC_LOG_ERR
#endif

#define ROLES_CLIENT 1
#define ROLES_SERVICE 2
#define ROLES_NULL 0
#define A78_ROLES ROLES_NULL

/********************* extern global variables *******************/
//extern enum ipc_core_e ipc_channel[IPC_CORE_MAX];

/********************* local variables ***************************/

// init status variable
static bool ipc_init_status;
static ST_MSG_INF st_msgbx_info;
static ST_MSGBX_END_PARA msg_end_para[MAX_END_NUM] = {0};
static ST_MSG_MESSAGE64 st_msg_64[MAX_END_NUM] = {0};
static struct ipc_msgbox *g_ipc_msgbx;
static struct task_struct *g_test_tid;
static ST_MSGBX_END_PARA __percpu *percpu_end_data;
static ST_MSGBX_END_PARA *g_end_data[MAX_END_NUM];

/********************* global variables ***************************/
struct platform_device *g_ipc_msgbx_pdev;
// addr variable

/**********************extern functions ***************************/
extern int32_t ipc_hw_recv_msg_notify(const uint8_t fid, rw_msg *msg);
extern int32_t ipc_hw_err_msg_notify(const uint8_t fid);

static inline u32 msgbx_get_flt_count(void)
{
	return g_ipc_msgbx->filter_num >= MSGBOX_MAX_FILTER_NUM ? MSGBOX_MAX_FILTER_NUM : g_ipc_msgbx->filter_num;
}

static void get_msgbox_info(struct ipc_msgbox *ipc_msgbx, ST_MSG_INF *msg_info)
{
	msg_info->ablt_r.reg1 = readl_relaxed(ipc_msgbx->fcsr_base + DEF_FIL_ABLT_R);

	msg_info->ablt_r2.reg2 = readl_relaxed(ipc_msgbx->fcsr_base + DEF_FIL_ABLT2_R);

	msg_info->version = (u8)readl_relaxed(ipc_msgbx->fcsr_base + DEF_FLT_VERSION_R);

	PRINT_DEBUG("\n filter_num:%d end_id:%d tx_dep:%d rx_dep:%d version:%d\n",
		msg_info->ablt_r.bit.filter_num, msg_info->ablt_r.bit.endid,
		msg_info->ablt_r2.bit.tx_fifo_depth, msg_info->ablt_r2.bit.rx_fifo_depth,
		msg_info->version);
}

static int32_t get_msgbox_irq_info(struct platform_device *pdev, ST_MSGBX_END_PARA *end_para_array)
{
	u32 index, fid, end_id, spi_irq_count;
	int32_t irq_count, irq;
	unsigned long hwirq;
	struct irq_desc *ipc_irq_desc;
	struct ipc_msgbox *ipc_msgbx = end_para_array[0].ipc_msgbx;
	u32 msgend_count = ipc_msgbx->msgend_count;

	irq_count = platform_irq_count(pdev);
	if (irq_count < 0) {
		IPC_LOG_WARNING("NO irq is platform_irq_count irq_count:%d",
							irq_count);
		return irq_count;	
	}

	/* DB has no spi irq, soc has 4 spi irq*/
	device_property_read_u32(&pdev->dev, "spi-irq-num", &spi_irq_count);
	ipc_msgbx->spi_count = spi_irq_count;
	PRINT_DEBUG("spi irq num %d, irq_count:%u", spi_irq_count, irq_count); 

	for (index = 0; index < irq_count; index++) {
		irq = platform_get_irq(pdev, index);
		if (irq < 0) {
			IPC_LOG_ERR("NO irq is configured for filter %d",
					irq);
			return irq;
		}
		ipc_irq_desc = irq_to_desc(irq);
		hwirq = ipc_irq_desc->irq_data.hwirq;
		IPC_LOG_INFO("virq:%d hwirq:%lu", irq, hwirq);
		/* PPI interrupt set for each msgbox end */
		if (hwirq < 32) {
			for (end_id = 0; end_id < msgend_count; end_id++) {
				end_para_array[end_id].irq[index] = irq;
			}
		} else {
			end_id = (index - CPU_MSGEND_PPI_NUM) / 4;
			fid = CPU_MSGEND_PPI_NUM + (index % spi_irq_count);
			if (end_id >= MAX_END_NUM || fid >= msgbx_get_flt_count()) {
				IPC_LOG_ERR("endid[%u] filterid[%u] ERROR!",
					end_id, fid);
			}
			end_para_array[end_id].irq[fid] = irq;
			PRINT_DEBUG("end%u filter:%d, virq %d hwirq:%lu", end_id, fid, irq, hwirq);
		}
	}

	return 0;
}

static inline int32_t virq_to_hwirq(int32_t virq)
{
	return irq_to_desc(virq)->irq_data.hwirq;
}

static int32_t get_filter_id_by_virq(ST_MSGBX_END_PARA *end_para, u32 irq)
{
	u32 fid, filter_num = end_para->ipc_msgbx->filter_num;
	for (fid = 0; fid < filter_num; fid++) {
		if (irq == end_para->irq[fid]) {
			return fid;
		}
	}

	IPC_LOG_ERR("do not find filter id by virq!");
	return -1;
}

static inline u32 msgbox_get_msg_num(void *filter_csr, u32 fid)
{
	u32 msg_num = 0;
	if (fid == FILTER_DEFAULT) {
		msg_num = readl_relaxed(filter_csr + DEF_FLT_RXFIFO_STATUS);
	} else {
		msg_num = readl_relaxed(filter_csr + End_filter_RxFIFO_StatusR);
	}

	return msg_num;
}

static int32_t get_msg(int32_t fid, ST_MSGBX_END_PARA *end_para)
{
	void *filter_csr = end_para->ipc_msgbx->fcsr_base + (fid * FILTER_CSR_SIZE);
	void *rxfifo_base = end_para->ipc_msgbx->rxfifo_base + (fid * FILTER_CSR_SIZE);
	ST_MSG_MESSAGE64 *msg64;
	u32 cpuid = smp_processor_id();
	u32 msg_num = 0, inter_status;
	u32 len = 0, i;

	if (cpuid >= MAX_END_NUM) {
		IPC_LOG_ERR("get msg end id error!");
		return -ERR_PARA;
	}
	msg64 = (ST_MSG_MESSAGE64 *)&st_msg_64[cpuid];
	/* defalut filter message receive */
	if (fid == FILTER_DEFAULT) {
		inter_status = readl_relaxed(filter_csr + DEF_FLT_INTER_R);
		if (inter_status & DEF_FILTER_RX_THRS_INTR) {
			/* receive all msseages */
			msg_num = readl_relaxed(filter_csr + DEF_FLT_RXFIFO_STATUS);
			while (msg_num-- > 0) {
				msg64->head.data = readq_relaxed(rxfifo_base);
				len = msg64->head.bit.is_64_bit ? msg64->head.bit.len : (msg64->head.bit.len / 2);
				// 32bit system change the msg len for ipc app
				if (!msg64->head.bit.is_64_bit) {
					msg64->head.bit.len = msg64->head.bit.len >> 1;
				}
				PRINT_DEBUG("\n msg_head:%llx", msg64->head.data); 
				if (len > MSG_64_MAX_LEN) {
					IPC_LOG_ERR("get msg end id error!");
				} else {
					for (i = 0; i < len; i++) {
						msg64->payload[i] = readq_relaxed(rxfifo_base);
						PRINT_DEBUG("\n pay%d:%llx", i, msg64->payload[i]);
					}
					ipc_hw_recv_msg_notify(fid, (rw_msg *)msg64);
				}
			}
		}

		if (inter_status & (DEF_FILTER_RX_UNDERFLOW_INTR | DEF_FILTER_TX_OVERFLOW_INTR |
			DEF_FILTER_RX_UNDERFLOW_INTR)) {
			ipc_hw_err_msg_notify(fid);
		}
		/* clean all interrupt */
		writel_relaxed(inter_status, filter_csr + DEF_FLT_STATUS_R);
	} else {/* filter1 to n receive message */
		inter_status = readl_relaxed(filter_csr + FILTER1_INTER_R);
		if (inter_status & FILTER1_THRS_INTR) {
			/* receive all msseages */
			msg_num = readl_relaxed(filter_csr + End_filter_RxFIFO_StatusR);
			while (msg_num-- > 0) {
				msg64->head.data = readq_relaxed(rxfifo_base);
				len = msg64->head.bit.is_64_bit ? msg64->head.bit.len : (msg64->head.bit.len / 2);
				if (len > MSG_64_MAX_LEN) {
					IPC_LOG_ERR("get msg len error!");
				} else {
					for (i = 0; i < len; i++) {
						msg64->payload[i] = readq_relaxed(rxfifo_base);
					}
					ipc_hw_recv_msg_notify(fid, (rw_msg *)msg64);
				}
			}
		}

		if (inter_status & (FILTER1_UNDERFLOW_INTR | FILTER1_OVERFLOW_INTR)) {
			ipc_hw_err_msg_notify(fid);
		}
		/* clean all interrupt */
		writel_relaxed(inter_status, filter_csr + FILTER1_STATUS_R);		
	}
	
	return 0;
}

// ipc interrupt processing function
static irqreturn_t ipc_msg_rece_handler(int32_t irq, void *p)
{
	int32_t fid, ret;
	u32	cpu_id = smp_processor_id();
	ST_MSGBX_END_PARA *end_para = g_end_data[cpu_id];

	fid = get_filter_id_by_virq(end_para, irq);
	if (fid < 0 || fid >= FILTER_NUM_BUFF) {
		IPC_LOG_ERR("illegal filter id:%d!", fid);
		return IRQ_NONE;
	}

	ret = get_msg(fid, end_para);

	return IRQ_HANDLED;
}

static int32_t of_msg_recv_init_spi_irqs(struct platform_device *pdev,
			     ST_MSGBX_END_PARA *end_para, u16 cpuid)
{
	int32_t irq = 0, ret;
	int32_t fid = 0;
	u32 filter_num = 0, spi_count;
	struct cpumask mask;
	char *irq_desc;

	filter_num = end_para->ipc_msgbx->filter_num;
	spi_count = end_para->ipc_msgbx->spi_count;
	/* DB do not has gic_spi */
	if (!spi_count) {
		return 0;
	} 

	PRINT_DEBUG("filter num %u", filter_num); 
	for (fid = (filter_num - spi_count); fid < filter_num; fid++) {
		irq = end_para->irq[fid];

		PRINT_DEBUG("register filter%u SPI[%u]", fid, virq_to_hwirq(irq));

		irq_desc = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s[%u][%d]",
					  "bst_ipc", cpuid, fid);
		if (!irq_desc) {
			IPC_LOG_ERR("devm_kasprintf no enough memory!\n");
			return -ENOMEM;
		}

		ret = devm_request_irq(&pdev->dev, irq,
				       ipc_msg_rece_handler,
				       IRQF_ONESHOT | IRQF_SHARED, irq_desc,
				       end_para);
		if (ret) {
			IPC_LOG_ERR(
				    "failed to request irq %d, ret = %d",
				    irq, ret);
			continue;
		}

		cpumask_clear(&mask);
		cpumask_set_cpu(cpuid, &mask);
		ret = irq_set_affinity_hint(irq, &mask);
		if (unlikely(ret < 0)) {
			devm_free_irq(&pdev->dev, irq, end_para);
			IPC_LOG_ERR(
					"irq_set_affinity_hint32_t failed: ret = %d",
					ret);
			return ret;
		}

		PRINT_DEBUG("spi:%d set to cpu%u", irq, cpuid);
	}
	return ret;
}

static u32 check_ppi_trigger(int irq)
{
	u32 flags = irq_get_trigger_type(irq);

	if (flags != IRQF_TRIGGER_HIGH && flags != IRQF_TRIGGER_LOW) {
		pr_warn("WARNING: Invalid trigger for IRQ%d, assuming level low\n", irq);
		pr_warn("WARNING: Please fix your firmware\n");
		flags = IRQF_TRIGGER_LOW;
	}

	return flags;
}

static int32_t of_msg_init_ppi_irqs(struct platform_device *pdev,
			     ST_MSGBX_END_PARA *end_para)
{
	int32_t irq = 0, ret;
	int32_t fid = 0;
	u32 filter_num = 0, spi_count, ppi_num;
	u32 flags, cpu_id;
	char *irq_desc;

	filter_num = end_para->ipc_msgbx->filter_num;
	spi_count = end_para->ipc_msgbx->spi_count;
	/* DB do not has gic_spi */
	ppi_num = (spi_count) ? (filter_num - spi_count) : (filter_num);

	PRINT_DEBUG("ppi num %u", ppi_num); 

	cpu_id = smp_processor_id();

	//percpu_end_data = alloc_percpu(ST_MSGBX_END_PARA);
	//memcpy(this_cpu_ptr(percpu_end_data), end_para, sizeof(ST_MSGBX_END_PARA));
	//IPC_LOG_ERR("end_para:0x%lx percpu_end_data:0x%lx filter_num:%d\n", (u64)end_para, (u64)this_cpu_ptr(percpu_end_data), percpu_end_data->ipc_msgbx->filter_num); 
	g_end_data[cpu_id] = end_para;
	PRINT_DEBUG("end_para:0x%lx filter_num:%d\n", (u64)g_end_data[cpu_id], g_end_data[cpu_id]->ipc_msgbx->filter_num); 	
	for (fid = 0; fid < ppi_num; fid++) {
		irq = end_para->irq[fid];

		PRINT_DEBUG("register cpu%u filter%u PPI[%u]", cpu_id, fid, virq_to_hwirq(irq));

		if (cpu_id == CPU_ID0) {
			irq_desc = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s[%d]",
						"bst_msgbox_fid", fid);
			if (!irq_desc) {
				IPC_LOG_ERR("devm_kasprintf no enough memory!\n");
				return -ENOMEM;
			}

			ret = request_percpu_irq(irq, ipc_msg_rece_handler,
				irq_desc, end_para);
			u32 test_fid = 0;
			
			if (ret) {
				IPC_LOG_ERR(
						"failed to request PPI irq %d, ret = %d",
						irq, ret);
				continue;
			}
			flags = check_ppi_trigger(irq);
			enable_percpu_irq(irq, flags);	
		} else {
			flags = check_ppi_trigger(irq);
			enable_percpu_irq(irq, flags);	
		}

	}
	return ret;
}

static void msgbx_init_on_each_cpu(ST_MSGBX_END_PARA *end_para, u16 cpuid)
{
	int32_t ret;

	of_msg_init_ppi_irqs(g_ipc_msgbx_pdev, end_para);
	/* spi irq init */
	ret = of_msg_recv_init_spi_irqs(g_ipc_msgbx_pdev, end_para, cpuid);
	if (ret < 0) {
		IPC_LOG_ERR("spi interrupt init fail ret:%d", ret);
	}	
}

static void per_msgbx_end_register(void *per_data)
{
	ST_MSGBX_END_PARA *end_para_array = (ST_MSGBX_END_PARA *)per_data;
	int32_t cpu = smp_processor_id();
	ST_MSGBX_END_PARA *end_para;

	if (cpu > MAX_END_NUM) {
		IPC_LOG_ERR("get cpu%d id error!", cpu);
	}
	end_para = &end_para_array[cpu];

	msgbx_init_on_each_cpu(end_para, cpu);

}

static int32_t ipc_msgbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ipc_msgbox *ipc_msgbx;
	struct resource *iomem;
	u32 i;
	int32_t ret = 0;

	g_ipc_msgbx_pdev = pdev;

	ipc_msgbx = devm_kzalloc(dev, sizeof(struct ipc_msgbox), GFP_KERNEL);
	if (IS_ERR_OR_NULL(ipc_msgbx)) {
		ret = PTR_ERR_OR_ZERO(ipc_msgbx);
		IPC_LOG_ERR("devm_kzalloc return %d", ret);
		return ret;
	}

	g_ipc_msgbx = ipc_msgbx;
	ipc_msgbx->dev = &pdev->dev;

	// filter csr register
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, RES_ID_FILTER_CSR);
	if (IS_ERR_OR_NULL(iomem)) {
		ret = PTR_ERR_OR_ZERO(iomem);
		IPC_LOG_ERR(
			    "platform_get_resource IORESOURCE_MEM 0 return %d",
			    ret);
		return ret;
	}
	PRINT_DEBUG("ipc_msgox filter start: 0x%llx, end: 0x%llx", iomem->start,
		     iomem->end);
	ipc_msgbx->fcsr_base = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR_OR_NULL(ipc_msgbx->fcsr_base)) {
		ret = PTR_ERR_OR_ZERO(ipc_msgbx->fcsr_base);
		IPC_LOG_ERR("Failed to remap ipc_msgbox regs: %d\n",
			    ret);
		return ret;
	}

#if 1 //for test
	writel_relaxed(0x1110, ipc_msgbx->fcsr_base);
	writel_relaxed(0x2040, ipc_msgbx->fcsr_base + 8);	
#endif

	// msgbox rxfifo register base
	ipc_msgbx->rxfifo_base = ipc_msgbx->fcsr_base + MSGBOX_RXFIFO_OFFSET;
	PRINT_DEBUG("ipc_msgox filter_csr va:0x%llx rxfifo va_start: 0x%llx", (u64)ipc_msgbx->fcsr_base, (u64)ipc_msgbx->rxfifo_base);

	// msgbox txfifo register base
	ipc_msgbx->txfifo_base = ipc_msgbx->fcsr_base + MSGBOX_TXFIFO_OFFSET;
	PRINT_DEBUG("ipc_msgox txfifo va_start: 0x%llx", (u64)ipc_msgbx->txfifo_base);	

	ret = device_property_read_u32(&pdev->dev, "msgend-num", &ipc_msgbx->msgend_count);

	ret = device_property_read_u32(&pdev->dev, "filter-num", &ipc_msgbx->filter_num);

	PRINT_DEBUG("ipc_msgox msgend_count: %u filter_num:%d", ipc_msgbx->msgend_count, ipc_msgbx->filter_num);

	for (i = 0; i < ipc_msgbx->msgend_count; i++) {
		msg_end_para[i].ipc_msgbx = ipc_msgbx;
	}

	get_msgbox_irq_info(pdev, msg_end_para);

#if (MSGBOX_DEBUG == 1)
	u32 j;
	for (i = 0; i < ipc_msgbx->msgend_count; i++) {
		for (j = 0; j < ARRAY_SIZE(msg_end_para[0].irq); j++) {
			IPC_LOG_ERR("end[%u]filter[%u]:irq %d", i, j, msg_end_para[i].irq[j]);
		}
	}
#endif

	// get msgbox hardware info run at each cpu
	get_msgbox_info(ipc_msgbx, &st_msgbx_info);

	per_msgbx_end_register(msg_end_para);

	/* init each msgbox end */
	//smp_call_function_single(1, per_msgbx_end_register, msg_end_para, 1);

	platform_set_drvdata(pdev, ipc_msgbx);

	IPC_LOG_ERR("ipc msgbox driver is ready!");

#if (A78_ROLES == ROLES_CLIENT)
	extern struct task_struct *start_client_test(void);
	g_test_tid = start_client_test();
	wake_up_process(g_test_tid);	
#elif (A78_ROLES == ROLES_SERVICE)
	extern struct task_struct *start_server_test(void);
	g_test_tid = start_server_test();
	wake_up_process(g_test_tid);
#endif
	return ret;
}

static int32_t ipc_msgbox_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id ipc_msgbox_of_match[] = {
	{
		.compatible = "bst,bst-msgbox",
	}, // this name need change
	{},
};
MODULE_DEVICE_TABLE(of, ipc_msgbox_of_match);
static struct platform_driver ipc_msgbox_driver = {
	.driver = {
		.name = "bst-msgbox",    //this name need change
		.of_match_table = ipc_msgbox_of_match,
	},
	.probe		= ipc_msgbox_probe,
	.remove		= ipc_msgbox_remove,
};

static int32_t __init ipc_msgbox_init(void)
{
	return platform_driver_register(&ipc_msgbox_driver);
}

module_platform_driver(ipc_msgbox_driver);

#define DEF_FILTER_ST_ADDR (0)
#define DEF_FILTER_END_ADDR (7)

static inline void* msgbx_get_filter_base(u32 fid)
{
	if (!g_ipc_msgbx->fcsr_base || fid >= msgbx_get_flt_count()) {
		IPC_LOG_ERR(" FILTER_BASE or fid:%u error", fid);
	}
	return g_ipc_msgbx->fcsr_base + (fid * FILTER_CSR_SIZE);
}

static inline void* msgbx_get_filter_tx_fifo_base(void)
{
	return g_ipc_msgbx->txfifo_base;
}

static inline void* msgbx_get_filter_rx_fifo_base(u32 fid)
{
	if (!g_ipc_msgbx->fcsr_base || fid >= msgbx_get_flt_count()) {
		IPC_LOG_ERR(" FILTER_BASE or fid:%u error", fid);
	}
	return g_ipc_msgbx->rxfifo_base + (fid * FILTER_CSR_SIZE);
}

static void msgbox_set_filter_addr(u32 filter, u32 fifo_st, u32 fifo_end) 
{
    void *flt_addr;
    u32 value = 0;

    /** base addr */
    flt_addr = msgbx_get_filter_base(filter);
    if(filter == FILTER_DEFAULT) {
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
    void *flt_addr;
    u32 value = 0;

    /** base addr */
    flt_addr = msgbx_get_filter_base(filter);
    if(filter == FILTER_DEFAULT) {
        //filter0 receive pid select
        value = ((pid_st & END_PIDF_CFGR_MASK)  << END_PIDF_CFGR_RX_PID_ST_SHIFT_U32) | 
			((pid_end & END_PIDF_CFGR_MASK) << END_PIDF_CFGR_RX_PID_END_SHIFT_U32) |
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
    void *flt_addr;
    FLT1_LEN_CFGR reg = {0};
	int32_t ret = 0;

    /** base addr */
    flt_addr = msgbx_get_filter_base(filter);
    if(filter == FILTER_DEFAULT) {
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
    void *flt_addr;
	u32 value = thres & 0x3ff;
    /** base addr */
    flt_addr = msgbx_get_filter_base(filter);
    if(filter == FILTER_DEFAULT) {
        writel_relaxed(value, flt_addr + DEF_DEFAULT_RXTHRS_CFGR);
    } else {
        writel_relaxed(value, flt_addr + End_filter_Thrs_CFGR);
    }
}

static void msgbox_set_filter_inter(u32 filter, u32 inter_en)
{
    void *flt_addr;
    /** base addr */
    flt_addr = msgbx_get_filter_base(filter);
    if(filter == FILTER_DEFAULT) {
        writel_relaxed(inter_en, flt_addr + DEF_FLT_INTER_EN);
    } else {
        writel_relaxed(inter_en, flt_addr + FILTER1_EN_INTER);
    }	
}

static u32 msgbox_get_filter_inter(u32 filter)
{
    void *flt_addr;
	u32 inter_enable = 0;
    /** base addr */
    flt_addr = msgbx_get_filter_base(filter);
    if(filter == FILTER_DEFAULT) {
        inter_enable = readl_relaxed(flt_addr + DEF_FLT_INTER_EN);
    } else {
        inter_enable = readl_relaxed(flt_addr + FILTER1_EN_INTER);
    }	
	return inter_enable;
}

/**
 * @brief msgbx hw init function
 * 
 * @param msgbx_param 
 * @return int32_t 
 */
int32_t msgbx_hw_init(const struct ipc_init_params *msgbx_param)
{
    uint32_t reg_data = 0;


    msgbox_set_filter_addr(FILTER_DEFAULT, DEF_FILTER_ST_ADDR, 
        DEF_FILTER_END_ADDR);
    msgbox_set_filter_pid(FILTER_DEFAULT, DEF_FILTER_PID_ST, 
        DEF_FILTER_PID_END, DEF_FILTER_PID_INVERT);
	msgbox_set_filter_thres(FILTER_DEFAULT, DEF_FILTER_THRES);

    if(msgbx_param != NULL)
    {
        if(msgbx_param->msgbx_end_mgt_flag)
        {
            reg_data |= (msgbx_param->msgbx_end_mgt_flag & (DEF_FILTER_RX_UNDERFLOW_INTR | 
                        DEF_FILTER_RX_OVERFLOW_INTR | DEF_FILTER_TX_OVERFLOW_INTR));
        }
    } else {
		return -ERR_PARA;
		IPC_LOG_ERR(" parameter error!");
	}
	/* enable rx fifo receive interrupt */
    reg_data |= DEF_FILTER_RX_THRS_INTR;
	msgbox_set_filter_inter(FILTER_DEFAULT, reg_data);
	return 0;
}

/**
 * @brief msgbx hw deinit
 * 
 * @return int32_t 
 */
int32_t msgbx_hw_deinit(void)
{
	return 0;
}

/**
 * @brief get msgbx info
 * 
 * @param hw_info 
 * @return int32_t 
 */
int32_t msgbx_hw_get_info(struct msgbx_hw_info *hw_info)
{
    ST_MSG_INF *msg_info = &st_msgbx_info;

    hw_info->mbx_flt_cnt = msg_info->ablt_r.bit.filter_num;
    hw_info->is_64_bit = msg_info->ablt_r.bit.is_64_bit;
    hw_info->mbx_end_id = msg_info->ablt_r.bit.endid;

    hw_info->mbx_rxfifo_depth = msg_info->ablt_r2.bit.rx_fifo_depth;
    hw_info->mbx_txfifo_depth = msg_info->ablt_r2.bit.tx_fifo_depth;

    hw_info->mbx_version = msg_info->version;

    return (0);
}

int32_t msgbx_hw_set_rule_pid(const uint8_t flt_id, const struct msgbx_flt_rule_pid *rule)
{
    int32_t ret = -1;

	if (flt_id < msgbx_get_flt_count()) {
		msgbox_set_filter_addr(flt_id, (flt_id * END_FILTER_RX_FIFO_DEPTH), 
			(flt_id * END_FILTER_RX_FIFO_DEPTH) + (END_FILTER_RX_FIFO_DEPTH - 1));
		msgbox_set_filter_thres(flt_id, DEF_FILTER_THRES);
		/* enalbe filter1 or default filter receive message interrupt */
		msgbox_set_filter_inter(flt_id, (msgbox_get_filter_inter(flt_id)) | FILTER1_THRS_INTR);
		msgbox_set_filter_pid(flt_id, rule->mbx_rx_pid_st, rule->mbx_rx_pid_end, rule->mbx_pid_flt_invert);
		ret = 0;
	} else {
		IPC_LOG_ERR("filter id error:%u", flt_id);
	}

    return ret;
}

int32_t msgbx_hw_set_rule_len(const uint8_t flt_id, const struct msgbx_flt_rule_len *rule)
{
    return msgbox_set_filter_len(flt_id, rule->mbx_rx_len_st, 
        rule->mbx_rx_len_end, rule->mbx_len_flt_invert);
}

static void msgbox_set_filter1_res_rule(void *base_addr, const struct msgbx_flt_rule_user *rule)
{
	FLT1_RX_FIFO_CFGR reg;
	reg.data = readl_relaxed(base_addr + End_filter1_RxFIFO_CFGR);

	/* set low 12 bit for head resv */
	writel_relaxed(rule->tx_reserved_filter_mask & FILTER1_RES_LOW_BIT_MASK, base_addr + End_filter1_MsgH_ResF_MaskR);
	writel_relaxed(rule->rx_res_min & FILTER1_RES_LOW_BIT_MASK, base_addr + End_filter1_MsgH_ResF_MinR);
	writel_relaxed(rule->rx_res_max & FILTER1_RES_LOW_BIT_MASK, base_addr + End_filter1_MsgH_ResF_MaxR);

	/* set high 32 bit for head resv */
	writel_relaxed(rule->tx_reserved_filter_mask >> 32, base_addr + End_filter1_MsgH_ResF_MaskHR);
	writel_relaxed(rule->rx_res_min >> 32, base_addr + End_filter1_MsgH_ResF_MinHR);
	writel_relaxed(rule->rx_res_max >> 32, base_addr + End_filter1_MsgH_ResF_MaxHR);	

	reg.bit.msgh_flilter_en = 1;
	reg.bit.msgh_flilter_invert = rule->msgh_flilter_invert & 0x1;
	reg.bit.msgh_combi_lh_comp = rule->msgh_combi_lh_comp & 0x1;

	writel_relaxed(reg.data, base_addr + End_filter1_RxFIFO_CFGR);
}

static void msgbox_set_filter1_payload_rule(void *base_addr, const struct msgbx_flt_rule_user *rule, u32 pay_rule_id)
{
	FLT1_RX_FIFO_CFGR reg;
	u32 offset = (End_filter1_MsgP2_MaskR - End_filter1_MsgP1_MaskR) * pay_rule_id;
	void * reg_base = base_addr + offset;

	reg.data = readl_relaxed(base_addr + End_filter1_RxFIFO_CFGR);

	/* set low 32 bit for filter payload */
	writel_relaxed(rule->tx_reserved_filter_mask & FILTER1_PAYLOAD_LOW_BIT_MASK, reg_base + End_filter1_MsgP1_MaskR);
	writel_relaxed(rule->rx_res_min & FILTER1_PAYLOAD_LOW_BIT_MASK, reg_base + End_filter1_MsgP1_MinR);
	writel_relaxed(rule->rx_res_max & FILTER1_PAYLOAD_LOW_BIT_MASK, reg_base + End_filter1_MsgP1_MaxR);

	/* set high 32 bit for filter payload */
	writel_relaxed(rule->tx_reserved_filter_mask >> 32, reg_base + End_filter1_MsgP1_MaskHR);
	writel_relaxed(rule->rx_res_min >> 32, reg_base + End_filter1_MsgP1_MinHR);
	writel_relaxed(rule->rx_res_max >> 32, reg_base + End_filter1_MsgP1_MaxHR);	

	switch (pay_rule_id) {
	case MSG_FILTER1_RULE_PAYLOAD1:
		reg.bit.msgp1_filter_en = 1;
		reg.bit.msgp1_filter_invert = rule->msgh_flilter_invert & 0x1;
		reg.bit.msgp1_combi_lh_comp = rule->msgh_combi_lh_comp & 0x1;
		break;
	case MSG_FILTER1_RULE_PAYLOAD2:
		reg.bit.msgp2_filter_en = 1;
		reg.bit.msgp2_filter_invert = rule->msgh_flilter_invert & 0x1;
		reg.bit.msgp2_combi_lh_comp = rule->msgh_combi_lh_comp & 0x1;
		break;	
	case MSG_FILTER1_RULE_PAYLOAD3:
		reg.bit.msgp3_filter_en = 1;
		reg.bit.msgp3_filter_invert = rule->msgh_flilter_invert & 0x1;
		reg.bit.msgp3_combi_lh_comp = rule->msgh_combi_lh_comp & 0x1;
		break;	
	case MSG_FILTER1_RULE_PAYLOAD4:
		reg.bit.msgp4_filter_en = 1;
		reg.bit.msgp4_filter_invert = rule->msgh_flilter_invert & 0x1;
		reg.bit.msgp4_combi_lh_comp = rule->msgh_combi_lh_comp & 0x1;
		break;	
	default:
		break;
	}

	writel_relaxed(reg.data, reg_base + End_filter1_RxFIFO_CFGR);
}

static void msgbox_set_filter_combi_mode(void *reg_base, u32 combi_mode)
{
	FLT1_RX_FIFO_CFGR reg;
	reg.data = readl_relaxed(reg_base + End_filter1_RxFIFO_CFGR);

	reg.bit.filter_combi_mode = (combi_mode & 0x3);

	writel_relaxed(reg.data, reg_base + End_filter1_RxFIFO_CFGR);
}

int32_t msgbx_hw_set_rule_user(const uint8_t flt_id, const struct msgbx_flt_rule_user *rule)
{

    uint32_t ret = -1;
    uint32_t offset = 0;

	void *flt_addr = msgbx_get_filter_base(flt_id);

    if(0 < flt_id && flt_id < msgbx_get_flt_count())
    {
		/* combi mode set logic and */
		msgbox_set_filter_combi_mode(flt_addr, COMBI_MODE_AND);
        switch (rule->cfg_loc)
        {
        case IPC_FLT_RULE_HEADER:
			msgbox_set_filter1_res_rule(flt_addr, rule);
            ret = 0;
            break;
        case IPC_FLT_RULE_PAY1:
        case IPC_FLT_RULE_PAY2:
        case IPC_FLT_RULE_PAY3:
        case IPC_FLT_RULE_PAY4:
            offset = (rule->cfg_loc-IPC_FLT_RULE_PAY1);
			msgbox_set_filter1_payload_rule(flt_addr, rule, offset);
            ret = 0;
            break;
        default:
			IPC_LOG_ERR("rule->cfg_loc error:%u", rule->cfg_loc);
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
int32_t msgbx_hw_set_flt_cfg(const uint8_t flt_id, const struct msgbx_flt_rule_cfg *rule)
{
    int32_t ret = -1;

    switch (rule->cfg_type)
    {
    case IPC_FLT_RULE_PID:
        ret = msgbx_hw_set_rule_pid(flt_id, &rule->rule_pid);
        break;
    case IPC_FLT_RULE_LEN:
        ret = msgbx_hw_set_rule_len(flt_id, &rule->rule_len);
        break;
    case IPC_FLT_RULE_USER:
        ret = msgbx_hw_set_rule_user(flt_id, &rule->rule_user);
        break;
    default:
        break;
    }

    return ret;
}

/**
 * @brief get msgbx hw config filter
 * 
 * @param flt_id 
 * @param rule 
 * @return int32_t 
 */
int32_t msgbx_hw_get_flt_info(const uint8_t flt_id, struct msgbx_flt_rule_cfg *info)
{
    int32_t ret = -1;

    return ret; 
}

/**
 * @brief msgbx hw clear filter config
 * 
 * @param flt_id 
 * @return int32_t 
 */
int32_t msgbx_hw_clr_flt_cfg(const uint8_t flt_id)
{
	u32 value;
	int32_t ret = 0;
	void *flt_addr = msgbx_get_filter_base(flt_id);
    if(flt_id == 0) {
        value = readl_relaxed(flt_addr + DEF_FLT_MSG_PIDF_CFGR);
		value &= ~(1ull << END_PIDF_CFGR_RX_FILTER_EN_SHIFT_U32);
        writel_relaxed(value, flt_addr + DEF_FLT_MSG_PIDF_CFGR);
    } else if(flt_id < msgbx_get_flt_count()) {
        value = readl_relaxed(flt_addr + End_filter1_RxFIFO_ADDRR);
		value &= ~(1ull << END_RXFIFO_ADDR_FILTER_EN_SHIFT_U32);
        writel_relaxed(value, flt_addr + End_filter1_RxFIFO_ADDRR);		
    } else {
		ret = -ERR_PARA;
	}

	return ret;
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

    if((msg64->head.bit.cid != pid) && 
        (msg64->head.bit.pid == pid) && 
        (msg64->head.bit.len <= MSG_64_MAX_LEN))
    {
		/* check CID, if CID is illegal, then the msgbox system will crash */
        switch (msg64->head.bit.cid)
        {
		#define MSG_END_MACRO(id, num) case id:
			PID_MSG_END
		#undef MSG_END_MACRO
			ret = 0;
            break;
        default:
                ret = -1;
            break;
        }
    } else {
		IPC_LOG_ERR("send message head:0x%llx error", msg64->head.data);
	}

    return ret;
}

/**
 * @brief msgbx hw send msg data
 * 
 * @param msg 
 * @return int32_t 0: is send ok, other: send failed
 */
int32_t msgbx_hw_send_msg(const rw_msg *msg)
{
    int32_t ret = -1;
    uint32_t reg_data, time_out_cnt = 0, i;
    void *tx_fifo_addr;
	void *reg_base = msgbx_get_filter_base(FILTER_DEFAULT);
    ST_MSG_MESSAGE64 *msg_ptr = (ST_MSG_MESSAGE64 *)msg;

	tx_fifo_addr = msgbx_get_filter_tx_fifo_base();

    PRINT_DEBUG("msgbx send head=0x%llx data[0]=0x%llx\n", msg_ptr->head.data, msg_ptr->payload[0]);
    if(0 == msgbx_header_is_valid(reg_base, msg_ptr))
    {
        /*wait tx fifo available*/
        do{
            reg_data = readl_relaxed(reg_base + DEF_Tx_FIFO_Available);
            time_out_cnt++;
            if(time_out_cnt > MSGBX_WAIT_TIMEOUT_CNT) {
                ret = 2;
                return ret;
            }
        } while (reg_data == 0);

        /*write msg head*/
        // printf("addr[0x%x]=0x%x\r\n",write_addr, msg_ptr->header.data);
		writeq_relaxed(msg_ptr->head.data, tx_fifo_addr);
        for (i = 0; i < msg_ptr->head.bit.len; i++)
        {
            /*write msg payload*/
            // printf("addr[0x%x]=0x%x\n",write_addr, msg_ptr->payload[i]);
            writeq_relaxed(msg_ptr->payload[i], tx_fifo_addr);
        }
        ret = 0;
    } else {
        ret = 1;
    }

    return ret;
}

/**
 * @brief msgbx get a msg from recv fifo
 * 
 * @param msg msg buffer
 * @param fid filter id
 * @return int32_t 
 */
int32_t msgbx_hw_recv_msg(rw_msg *msg, const int8_t fid)
{
    int32_t ret = -1;
    uint32_t i;
    void *read_addr;
    uint32_t read_len = 0;
	u32 flterid = (u32)fid;
    ST_MSG_MESSAGE64 *msg_ptr;

    if(fid < msgbx_get_flt_count()) {
        if(msgbox_get_msg_num(msgbx_get_filter_base(flterid), flterid)) {
            read_addr = msgbx_get_filter_rx_fifo_base(fid);
            msg_ptr = (ST_MSG_MESSAGE64 *)msg;
            msg_ptr->head.data = readq_relaxed(read_addr);
            read_len = msg_ptr->head.bit.is_64_bit ? msg_ptr->head.bit.len : (msg_ptr->head.bit.len / 2);
            // myprintf("header=0x%0x,read_len=%d\n",msg_ptr->header.data,read_len);
            if(read_len <= MSG_64_MAX_LEN) {
                for (i = 0; i < read_len; i++) {
                    msg_ptr->payload[i] = readq_relaxed(read_addr);
                }
            }
            ret = 0;
        } else {
            ret = -1;
        }
    } else {
		IPC_LOG_ERR("filter id:0x%llx error", fid);		
	}

    return ret;
}

/**
 * @brief enable msgbx statues management
 * 
 * @param cfg 
 * @return int32_t a
 */
int32_t msgbx_hw_sts_mgt_enable(const uint32_t flag)
{
	void *flt_csr = msgbx_get_filter_base(FILTER_DEFAULT);

	writel_relaxed(flag, flt_csr + END_FMU_SAFETY_INTR_EN);
    return 0;
}

/**
 * @brief disable msgbx status management
 * 
 * @return int32_t 
 */
int32_t msgbx_hw_sts_mgt_disable(void)
{
	void *flt_csr = msgbx_get_filter_base(FILTER_DEFAULT);
	writel_relaxed(0, flt_csr + END_FMU_SAFETY_INTR_EN);
    return 0;
}

/**
 * @brief msgbx filter state management config
 * 
 * @param flt_id filter id
 * @param flag 
 * @param flt_thrs 
 * @return int32_t 
 */
int32_t msgbx_hw_flt_mgt_enble(const uint8_t flt_id, const uint8_t flag)
{
    int32_t ret = -1;

    if(flt_id < msgbx_get_flt_count())
    {
        if(0 == flt_id) {
			msgbox_set_filter_inter((u32)flt_id, (flag & (DEF_FILTER_RX_UNDERFLOW_INTR | 
                        DEF_FILTER_RX_OVERFLOW_INTR | DEF_FILTER_TX_OVERFLOW_INTR)));
        } else {
			msgbox_set_filter_inter((u32)flt_id, (flag & (FILTER1_UNDERFLOW_INTR | 
			FILTER1_OVERFLOW_INTR)));
        }
        ret = 0;
    }

    return ret;
}

/**
 * @brief disable msgbx filter management
 * 
 * @param flt_id 
 * @return int32_t 
 */
int32_t msgbx_hw_flt_mgt_disable(const uint8_t flt_id)
{
    int32_t ret = -1;
	void *flt_addr = msgbx_get_filter_base((u32)flt_id);
	u32 temp;

    if(flt_id < msgbx_get_flt_count())
    {
        if(0 == flt_id) {
			temp = readl_relaxed(flt_addr + DEF_FLT_INTER_EN);
			temp &= ~(DEF_FILTER_RX_UNDERFLOW_INTR | 
                        DEF_FILTER_RX_OVERFLOW_INTR | DEF_FILTER_TX_OVERFLOW_INTR);
			msgbox_set_filter_inter((u32)flt_id, temp);
        } else {
			temp = readl_relaxed(flt_addr + FILTER1_EN_INTER);
			temp &= ~(FILTER1_UNDERFLOW_INTR | FILTER1_OVERFLOW_INTR);
			msgbox_set_filter_inter((u32)flt_id, temp);
        }
        ret = 0;
    }

    return ret;
}

/**
 * @brief msgbx get error msg
 * 
 * @param err_msg 
 * @return int32_t 
 */
int32_t msgbx_hw_get_err_msg(const uint8_t fid, struct msgbx_err_msg *err_msg)
{
    uint32_t reg_data = 0;
    void *flt_addr = msgbx_get_filter_base((u32)fid);
	uint32_t ret = -1;

    if(fid < msgbx_get_flt_count())
    {
        if(0 == fid) {
            reg_data = readl_relaxed(flt_addr + DEF_FLT_INTER_R);
            err_msg->msg = reg_data & (DEF_FILTER_RX_UNDERFLOW_INTR | 
                        DEF_FILTER_RX_OVERFLOW_INTR | DEF_FILTER_TX_OVERFLOW_INTR);
        } else {
            reg_data = readl_relaxed(flt_addr + FILTER1_INTER_R);
            err_msg->msg = reg_data & (FILTER1_UNDERFLOW_INTR | FILTER1_OVERFLOW_INTR);
        }
        err_msg->type = 2;
        err_msg->id = 0;
		ret = 0;
    }

	return ret;
}

 /**
  * @brief  msgbx fault handle
  * 
  * @param type 
  * @param id 
  * @param hdl 
  * @return int32_t 
  */
int32_t msgbx_hw_err_hdl(uint8_t type, uint8_t id, uint32_t hdl)
{
	int32_t ret = -1;

	return ret;
}

struct libipc_hw_compat_ops ipc_hw_shm_ops = {
    .ipc_hw_init = msgbx_hw_init,
    .ipc_hw_deinit = msgbx_hw_deinit,
    .ipc_hw_get_info = msgbx_hw_get_info,
    .ipc_hw_set_flt_cfg = msgbx_hw_set_flt_cfg,

    .ipc_hw_get_flt_info = msgbx_hw_get_flt_info,
    .ipc_hw_clr_flt_cfg = msgbx_hw_clr_flt_cfg,
    .ipc_hw_send_msg = msgbx_hw_send_msg,
    .ipc_hw_get_msg = msgbx_hw_recv_msg,
    .ipc_hw_sts_mgt_enble = msgbx_hw_sts_mgt_enable,
    .ipc_hw_sts_mgt_disable = msgbx_hw_sts_mgt_disable,
    .ipc_hw_flt_mgt_enble = msgbx_hw_flt_mgt_enble,
    .ipc_hw_flt_mgt_disable = msgbx_hw_flt_mgt_disable,
    .ipc_hw_get_err_msg = msgbx_hw_get_err_msg,
    .ipc_hw_err_hdl = msgbx_hw_err_hdl,
};
