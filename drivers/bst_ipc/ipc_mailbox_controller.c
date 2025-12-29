// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

/*
 * IPC: Linux device driver for Blck Sesame Technologies inter-processor
 * communication
 *
 */
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/mailbox_controller.h>
#include <linux/of_reserved_mem.h>
#include <linux/mailbox_client.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/dma-mapping.h>

#include "ipc_mempool.h"
#include "ipc_communication_manager.h"

#include "ipc_mailbox_controller.h"
#include "ipc_regs.h"
#include "ipc_session.h"

/********************* macros *******************/
#define IPC_DRIVER_NAME	 "ipc_controller"
#define CPU_NR		 8
#define SEND_RETRY_TIMES 20000

/********************* local variables ***************************/
static struct ipc_client_info *a55_client_info[CPU_NR];
static int8_t s_core_of_irq[400];
// init status variable
static bool ipc_init_status;

/********************* global variables ***************************/
struct platform_device *g_ipc_platform_dev;
// addr variable
static uint64_t g_ipc_all_cores_register_addr_phy;
struct ipc_memblock *g_ipc_memblock;
struct ipc_all_cores_register_addr *g_ipc_all_cores_register_addr;
struct ipc_all_cores_register_addr *g_ipc_all_cores_register_addr_uaddr;
// spinlock send

static unsigned int max_recv_time = 0;
static unsigned int max_send_time = 0;
static unsigned int last_send_method_token = 0;
static unsigned int last_recv_reply_token = 0;
static unsigned int last_recv_signal_token = 0;
static unsigned int last_recv_method_token = 0;

module_param(max_recv_time, uint, 0644);
MODULE_PARM_DESC(max_recv_time, "max recv time");


module_param(max_send_time, uint, 0644);
MODULE_PARM_DESC(max_send_time, "max send time");

module_param(last_send_method_token, uint, 0644);
MODULE_PARM_DESC(last_send_method_token, "max send time");


module_param(last_recv_reply_token, uint, 0644);
MODULE_PARM_DESC(last_recv_reply_token, "max send time");


module_param(last_recv_method_token, uint, 0644);
MODULE_PARM_DESC(last_recv_method_token, "max send time");

module_param(last_recv_signal_token, uint, 0644);
MODULE_PARM_DESC(last_recv_signal_token, "max send time");





#ifdef CONFIG_BST_IPC_STRESS
extern void ipc_stress_init(void);
#endif



/********************* functions declaration ***************************/
bool get_ipc_init_status(void)
{
	return ipc_init_status;
};

unsigned int get_last_token(void ){
	return last_send_method_token;
}
EXPORT_SYMBOL(get_last_token);

void * translate_address_by_system(void * addr)
{
	uint64_t  base = 0;
	uint64_t  sys_offset = 0;

	#ifdef CONFIG_BST_C1200_ADAS
	sys_offset = 0;
	#endif

	#ifdef CONFIG_BST_C1200_IVI
	sys_offset = IPC_BASE_OFFSET;
	#endif

	#ifdef CONFIG_BST_C1200_DB
	sys_offset = IPC_BASE_OFFSET*2;
	#endif
	
	base = (uint64_t)((uint64_t)addr + sys_offset);
		

	return (void *)base;
}
 

 void * translate_address_by_src(enum ipc_core_e cpu_id,void * addr)
{
	uint64_t  base = 0;
	uint64_t  sys_offset = 0;

	switch(cpu_id)
	{
		case IPC_CORE_ARM0:
		case IPC_CORE_ARM1:
		{
			sys_offset = IPC_BASE_OFFSET;
			break;
		}
		case IPC_CORE_ARM2:
		case IPC_CORE_ARM3:
		{
			sys_offset = 0;
			break;
		}
		case IPC_CORE_DB0:
		case IPC_CORE_DB1:
		{
			sys_offset = IPC_BASE_OFFSET*2;
			break;
		}
		default:
		{
			#ifdef CONFIG_BST_C1200_ADAS
			sys_offset = 0;
			#endif

			#ifdef CONFIG_BST_C1200_IVI
			sys_offset = IPC_BASE_OFFSET;
			#endif

			#ifdef CONFIG_BST_C1200_DB
			sys_offset = IPC_BASE_OFFSET*2;
			#endif
			break;
		}
	}
	
	
	base = (uint64_t)((uint64_t)addr + sys_offset);
		

	return (void *)base;
}




// ipc message send function,
int32_t ipc_send_data(struct ipc_client_info *client_info, enum ipc_core_e src,
		      void *data)
{
	void *read_reg = NULL;
	struct ipc_mbox *ipc_mbox = NULL;
	// write dst source register to enable IPC hw to trigger interrupt
	uint32_t source_reg_value = 0;
	void *write_reg = client_info->tx_reg;
	uint32_t dst = client_info->core_id;
	uint32_t max_read_cnt = SEND_RETRY_TIMES;
	uint32_t send_count = SEND_RETRY_TIMES;
	uint32_t trylock_count = SEND_RETRY_TIMES;
	int flag = 0;
	unsigned long lock = 0;
	struct ipc_fill_register_msg * intr_msg = NULL;
	struct ipc_all_cores_register_addr *g_ipc_all_cores_register_addr_ptr = NULL;
	ktime_t cur_time =0 ,last_time=0;
	




	IPC_LOG_INFO("send data from %d to %d", src, client_info->core_id);

	ipc_mbox = platform_get_drvdata(g_ipc_platform_dev);
	read_reg = IRQ_CHECK_ADDR(ipc_mbox->event_base, dst);
	client_info->channel_status = CHANNEL_SENDING;
	source_reg_value |= (1ull << (src));
	IPC_LOG_INFO("write to addr : 0x%llx, value is : 0x%x, src = %d",
		     __virt_to_phys(write_reg), source_reg_value, src);

	//spin_lock_irqsave(&ipc_mbox->lock[src][dst],flags);

	while(trylock_count--){
		lock = spin_trylock(&ipc_mbox->lock[src][dst]);
		if(lock){
			flag = 1;
			break;
		}
		udelay(1);
	}

	if(flag == 0){
		IPC_LOG_ERR("try to request spinlock failed src:%d dst:%d\n", src,dst);
		return -1;
	}


	cur_time =  ktime_get();

	flag = 0;

	while (max_read_cnt--) {
		// note: only check trigger reg's value
		if ((readl_relaxed(read_reg) & source_reg_value) == 0) {
			flag = 1;
			break;
		}

		udelay(1);
	}

	if (!flag) {
		spin_unlock(&ipc_mbox->lock[src][dst]);
		IPC_LOG_ERR("The dst keeps interrupt! dst = %d", dst);
		return -1;
	}


	
	

	g_ipc_all_cores_register_addr_ptr = (struct ipc_all_cores_register_addr *)translate_address_by_system(g_ipc_all_cores_register_addr);
	if (!g_ipc_all_cores_register_addr_ptr) {
		IPC_LOG_ERR("core %d not support ipc", src);
		spin_unlock(&ipc_mbox->lock[src][dst]);
		return -1;
	}

	memcpy(&(g_ipc_all_cores_register_addr_ptr->addr[src].msg), data,sizeof(struct ipc_fill_register_msg));

	intr_msg =(struct ipc_fill_register_msg *)data;
	if((IPC_MSG_TYPE_METHOD == intr_msg->type) && (dst ==  IPC_CORE_SAFE)) {
			last_send_method_token = intr_msg->short_param;
	}

	ipc_dsb(); // force to write msg to ddr
	writel_relaxed(source_reg_value, write_reg);

	// check irq status
	flag = 0;

	while (send_count--) {
		
		if ((readl_relaxed(read_reg) & source_reg_value) == 0) {
			client_info->channel_status = CHANNEL_SEND_SUCCESS;
			flag = 1;
			break;
		}

		udelay(1);
	}

	// clear sending box
	memset(&g_ipc_all_cores_register_addr_ptr->addr[src].msg, 0,
	       sizeof(struct ipc_fill_register_msg));

	if (!flag) {
		ipc_dsb(); // force to write msg to ddr
		// clear trigger reg
		writel_relaxed(source_reg_value, read_reg);
		client_info->channel_status = CHANNEL_SEND_FAIL;
		IPC_LOG_ERR("send to %d fail", dst);
		
		spin_unlock(&ipc_mbox->lock[src][dst]);
		return -1;
	}


	spin_unlock(&ipc_mbox->lock[src][dst]);

	last_time =  ktime_get();

//	if(last_time - cur_time > max_send_time){
//		IPC_LOG_ERR("smax:%lld", last_time - cur_time);
//	}

	max_send_time = last_time - cur_time > max_send_time ?last_time - cur_time:max_send_time;


	return 0;
}



// ipc interrupt processing function
static irqreturn_t ipc_event_interrupt_handler(int32_t irq, void *p)
{
	struct ipc_mbox *ipc_mbox = p;
	int32_t cpu_id = s_core_of_irq[irq];
	int32_t intr_src = 0;
	ktime_t cur_time =0 ,last_time=0;
	struct ipc_fill_register_msg intr_msg;
	struct ipc_all_cores_register_addr *g_ipc_all_cores_register_addr_ptr = NULL;

	void *this_cpu_src_reg_addr =
		READ_SRC_AND_CLEAR_IRQ_ADDR(ipc_mbox->event_base, cpu_id);

	uint32_t value = readl_relaxed(this_cpu_src_reg_addr);
	if (unlikely(value == 0)) {
		IPC_LOG_ERR("src reg value is wrong 0x%x", value);
		return IRQ_NONE;
	}

	cur_time =  ktime_get();


	intr_src = get_msb_bit1_index(value);

	if (unlikely(intr_src < 0) || unlikely(intr_src >= IPC_CORE_MAX)) {
		#ifndef CONFIG_SECOND_KERNEL
		IPC_LOG_ERR("intr_src is wrong %d", intr_src);
		#endif
		return IRQ_NONE;
	}


	if (unlikely(g_ipc_all_cores_register_addr == NULL)) {
		value = (1ull << intr_src); // set bit to 0
		writel_relaxed(value, this_cpu_src_reg_addr); // W1C
		#ifndef CONFIG_SECOND_KERNEL
		IPC_LOG_ERR("msg addr is not ready %d", intr_src);
		#endif
		return IRQ_NONE;
	}

	// msg process
	ipc_dsb(); // force to write msg to ddr


	g_ipc_all_cores_register_addr_ptr = (struct ipc_all_cores_register_addr *)translate_address_by_src(intr_src,g_ipc_all_cores_register_addr);
	if (!g_ipc_all_cores_register_addr_ptr) {
		value = (1ull << intr_src); // set bit to 0
		writel_relaxed(value, this_cpu_src_reg_addr); // W1C
		#ifndef CONFIG_SECOND_KERNEL
		IPC_LOG_WARNING("core %d not support ipc", cpu_id);
		#endif
		return IRQ_NONE;
	}

	memcpy(&intr_msg, &(g_ipc_all_cores_register_addr_ptr->addr[intr_src].msg),sizeof(struct ipc_fill_register_msg));


	if (likely(intr_msg.type > 0)) {

		
		
		
		if((IPC_MSG_TYPE_REPLY == intr_msg.type) && (intr_src ==  IPC_CORE_SAFE)) {
			last_recv_reply_token = intr_msg.short_param;
		}

		if((IPC_MSG_TYPE_SIGNAL == intr_msg.type) && (intr_src ==  IPC_CORE_SAFE)) {
			last_recv_signal_token++;
		}
		
		
		if((IPC_MSG_TYPE_METHOD == intr_msg.type) && (intr_src ==  IPC_CORE_SAFE)) {
			last_recv_method_token++;
		}
		
		
		
		
		#ifndef CONFIG_SECOND_KERNEL
		ipc_drv_recv(intr_src, cpu_id, (void *)&intr_msg,
			     sizeof(intr_msg));
		#endif
		
	} else {
		#ifndef CONFIG_SECOND_KERNEL
		IPC_LOG_ERR(
			"intr_msg type wrong, intr_msg.type = %d, src = %d, cmd = %d",
			intr_msg.type, intr_src, intr_msg.cmd);
		#endif
	}
       
	value = (1ull << intr_src);
	writel_relaxed(value, this_cpu_src_reg_addr); // W1C

	last_time =  ktime_get();

	max_recv_time = last_time - cur_time > max_recv_time?last_time - cur_time:max_recv_time;

	return IRQ_HANDLED;
}

// interrupt init
static int32_t of_event_irqs(struct platform_device *pdev,
			     struct ipc_mbox *ipc_mbox)
{
	int32_t ret = 0;
	int32_t cpu_id = 0;
	int32_t irq_num;
	int count;
	struct cpumask mask;
	char *irq_desc;
	int32_t ipc_cpus[4];

	count = of_property_count_u32_elems(pdev->dev.of_node,"ipc-cpus");
	if(count < 0 || count > 4) {
		IPC_LOG_ERR("ipc-cpus dts none!!!!\n");
		return -1;
	}

	ret = of_property_read_u32_array(pdev->dev.of_node, "ipc-cpus",ipc_cpus, count);
	if (ret) {
		IPC_LOG_ERR("ipc-cpus dts none!!!!\n");
		return -1;
	}


	for (cpu_id = 0; cpu_id < count; cpu_id++) {

		irq_num = platform_get_irq(pdev, ipc_cpus[cpu_id]);
		if (irq_num < 0) {
			IPC_LOG_WARNING("NO irq is platform_get_irq for cpu %d",
					ipc_cpus[cpu_id]);
			// return irq_num;
		}
		irq_num = irq_of_parse_and_map(pdev->dev.of_node, ipc_cpus[cpu_id]);
		if (irq_num < 0) {
			IPC_LOG_WARNING("NO irq is configured for cpu %d",
					ipc_cpus[cpu_id]);
			return irq_num;
		}
		s_core_of_irq[irq_num] = ipc_cpus[cpu_id];
		IPC_LOG_INFO("cpu %d, irq %d", ipc_cpus[cpu_id], irq_num);

		a55_client_info[ipc_cpus[cpu_id]] = devm_kzalloc(
			&pdev->dev, sizeof(*a55_client_info[ipc_cpus[cpu_id]]),
			GFP_KERNEL);
		if (!a55_client_info[ipc_cpus[cpu_id]]) {
			IPC_LOG_ERR("no enough memory!\n");
			return -ENOMEM;
		}

		a55_client_info[ipc_cpus[cpu_id]]->dev = &pdev->dev;

		irq_desc = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s[%d]",
					  "bst_ipc", ipc_cpus[cpu_id]);
		if (!irq_desc) {
			IPC_LOG_ERR("devm_kasprintf no enough memory!\n");
			return -ENOMEM;
		}
		ret = devm_request_irq(&pdev->dev, irq_num,
				       ipc_event_interrupt_handler,
				       IRQF_ONESHOT | IRQF_SHARED, irq_desc,
				       ipc_mbox);
		if (ret) {
			IPC_LOG_ERR(
				    "failed to request irq %d, ret = %d",
				    irq_num, ret);
			continue;
			// return ret;
		}

		cpumask_clear(&mask);
		cpumask_set_cpu(cpu_id, &mask);
		ret = irq_set_affinity_hint(irq_num, &mask);
		if (unlikely(ret < 0)) {
			devm_free_irq(&pdev->dev, irq_num, ipc_mbox);
			IPC_LOG_ERR(
				    "irq_set_affinity_hint32_t failed: ret = %d",
				    ret);
			return ret;
		}
	}
	return ret;
}





// ipc message interrupt init
static int32_t set_en_irq_mask(struct ipc_mbox *ipc_mbox)
{
	int32_t cpu_id;
	int32_t i;
	void *cpu_en_reg_addr;
	uint32_t en_mask;

	for (cpu_id = 0; cpu_id <= IPC_CORE_DB1; cpu_id++) {
		en_mask = 0;
		// set enable irq mask
		for (i = 0; i <= IPC_CORE_DB1; i++) {
#ifndef ENABLE_SRC_DST_SAME
			if (cpu_id != i)
#endif
			{
				en_mask |= (1 << i);
			}
		}
		en_mask |= (1 << EVENT_BIT_ISP);
		en_mask |= (1 << EVENT_BIT_DSP_0);
		en_mask |= (1 << EVENT_BIT_DSP_1);
		en_mask |= (1 << EVENT_BIT_DSP_2);
		en_mask |= (1 << EVENT_BIT_DSP_3);
		en_mask |= (1 << EVENT_BIT_NET);
		en_mask |= (1 << EVENT_BIT_SEC);
		en_mask |= (1 << EVENT_BIT_SAFE);
		
		en_mask |= (1 << 30);
		IPC_LOG_INFO("cpu %d en_mask = 0x%x", cpu_id, en_mask);

		cpu_en_reg_addr = CPU_EN_REG_ADDR(ipc_mbox->event_base, cpu_id);
		writel_relaxed(en_mask, cpu_en_reg_addr);
		en_mask = readl_relaxed(cpu_en_reg_addr);
		IPC_LOG_INFO("cpu %d after write enable reg, en_mask = 0x%x",
			     cpu_id, en_mask);
	}

	return 0;
}

// ipc message share buffer init
int32_t alloc_mem(struct ipc_buffer *ipc_buffer)
{
	uint64_t uaddr = 0;
	int32_t ret = 0;
	struct ipc_memblock *memblock = NULL;
	struct ipc_mbox *ipc_mbox = platform_get_drvdata(g_ipc_platform_dev);

	ret = ipc_mbox->pool->ops->alloc(ipc_mbox->pool, ipc_buffer->size,
					 ipc_buffer->align, &memblock);
	if (ret) {
		IPC_LOG_ERR("alloc fail!");
		return ret;
	}

	memblock->u_addr = (void *)uaddr;
	ipc_buffer->uaddr = uaddr;
	ipc_buffer->handle = (uint64_t)memblock;
	ipc_buffer->phy_addr.ptr_64 = memblock->phy_addr;

	IPC_LOG_INFO("uaddr: 0x%llx, handle: 0x%llx, phy_addr: 0x%llx",
		     ipc_buffer->uaddr, ipc_buffer->handle,
		     ipc_buffer->phy_addr.ptr_64);

	return ret;
}

int32_t alloc_ipc_msg_share_buffer(bool user)
{
	// this function only can be called once.
	static atomic_t init_done = ATOMIC_INIT(0);
	struct ipc_buffer ipc_buffer;

	
	if (atomic_cmpxchg(&init_done, 0, 1) == 1)
		return 0;

	memset(&ipc_buffer,0,sizeof(struct ipc_buffer));

	// message share buffer size
	ipc_buffer.size = 0x300000;
	ipc_buffer.align = 64;

	// struct ipc_memblock *memblock = NULL;
	if (alloc_mem(&ipc_buffer) < 0) {
		IPC_LOG_ERR("ipc message share buffer create fail ");
		return -1;
	}

	// TODO:this memblock do not free, even if there is no case need to free
	// it
	// TODO:this shared buffer addr can not be leaked out, this addr need to
	// manage in ipc_mbox
	g_ipc_memblock = (struct ipc_memblock *)ipc_buffer.handle;
	g_ipc_all_cores_register_addr = g_ipc_memblock->k_addr;
	g_ipc_all_cores_register_addr_phy = ipc_buffer.phy_addr.low;
	g_ipc_all_cores_register_addr_uaddr = g_ipc_memblock->u_addr;

	return 0;
}

static int32_t ipc_mempool_init(struct ipc_mbox *ipc_mbox)
{
	int32_t ret = 0;

	ret = of_reserved_mem_device_init(ipc_mbox->dev);
	if (ret < 0) {
		IPC_LOG_ERR("of_reserved_mem_device_init fail, ret: %d\n", ret);
		return -ENODEV;
	}
	return ipc_init_cma_mempool(&ipc_mbox->pool, ipc_mbox->dev);
}




static int32_t ipc_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ipc_mbox *ipc_mbox;
	struct resource *iomem;
	int32_t ret = 0;
	int32_t dst = 0,src =0;

	g_ipc_platform_dev = pdev;

	send_msg_init();
	ipc_mbox = devm_kzalloc(dev, sizeof(*ipc_mbox), GFP_KERNEL);
	if (!ipc_mbox) {
		ret = PTR_ERR_OR_ZERO(ipc_mbox);
		IPC_LOG_ERR("devm_kzalloc return %d", ret);
		return ret;
	}

	ipc_mbox->dev = &pdev->dev;




	// init share mempool
	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(36));
	if (ret < 0) {
		IPC_LOG_ERR("dma_set_coherent_mask fail, ret %d\n",
			    ret);
	}
	ret = ipc_mempool_init(ipc_mbox);
	IPC_LOG_INFO("ipc mempool init result: %d\n", ret);
	if (ret < 0) {
		IPC_LOG_ERR("ipc_mempool_init fail, ret %d\n", ret);
		return ret;
	}

	// event register
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(iomem)) {
		ret = PTR_ERR_OR_ZERO(iomem);
		IPC_LOG_ERR(
			    "platform_get_resource IORESOURCE_MEM 0 return %d",
			    ret);
		return ret;
	}
	IPC_LOG_INFO("ipc_mbox event start: 0x%llx, end: 0x%llx", iomem->start,
		     iomem->end);
	ipc_mbox->event_base = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR_OR_NULL(ipc_mbox->event_base)) {
		ret = PTR_ERR_OR_ZERO(ipc_mbox->event_base);
		IPC_LOG_ERR("Failed to remap ipc_mbox regs: %d\n",
			    ret);
		return ret;
	}

	// semaphore
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (IS_ERR_OR_NULL(iomem)) {
		ret = PTR_ERR_OR_ZERO(iomem);
		IPC_LOG_ERR(
			    "platform_get_resource IORESOURCE_MEM 1 return %d",
			    ret);
		return ret;
	}
	IPC_LOG_INFO("ipc_mbox semaphore start: 0x%llx, end: 0x%llx",
		     iomem->start, iomem->end);
	ipc_mbox->sem_base = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR_OR_NULL(ipc_mbox->sem_base)) {
		ret = PTR_ERR_OR_ZERO(ipc_mbox->sem_base);
		IPC_LOG_ERR("Failed to remap ipc_mbox regs: %d\n",
			    ret);
		return ret;
	}

	ret = of_event_irqs(pdev, ipc_mbox);


	set_en_irq_mask(ipc_mbox);



	platform_set_drvdata(pdev, ipc_mbox);

	for(src =IPC_CORE_ARM0;src<=IPC_CORE_DB1;src++){

		for(dst=EVENT_BIT_CPU0;dst<EVENT_BIT_HIFI;dst++){

			spin_lock_init(&ipc_mbox->lock[src][dst]);
		}
	}
	// ipc communication layer create
	ret = ipc_communication_create();
	if (ret < 0) {
		IPC_LOG_ERR(
			    "ipc_communication_create fail, ret %d\n", ret);
		return ret;
	}
	// ipc message share buffer create
	ret = alloc_ipc_msg_share_buffer(false);
	if (ret < 0) {
		IPC_LOG_ERR(
			    "ipc_communication_create fail, ret %d\n", ret);
		return ret;
	}

	#if CONFIG_BST_IPC_STRESS
	ipc_stress_init();
	#endif

	ipc_init_status = true;
	IPC_LOG_INFO("ipc driver is ready");
	return ret;
}

static int32_t ipc_mbox_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id ipc_mbox_of_match[] = {
	{
		.compatible = "bst,bst-mbox",
	}, // this name need change
	{},
};



#ifdef CONFIG_PM_SLEEP
static int dw_ipc_plat_suspend(struct device *dev){

	return 0;
}


static int  dw_ipc_plat_resume(struct device *dev)
{
	struct ipc_mbox *ipc_mbox = NULL;
	
	ipc_mbox = platform_get_drvdata(g_ipc_platform_dev);
	if(!ipc_mbox){
		IPC_LOG_ERR(
			    "dw_ipc_plat_resume resume failed\n");
		return -1;
	}

	set_en_irq_mask(ipc_mbox);

	IPC_LOG_ERR("ipc  resume success\n");
	return 0;
}

static const struct dev_pm_ops dw_ipc_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(dw_ipc_plat_suspend, dw_ipc_plat_resume)
};

#endif


MODULE_DEVICE_TABLE(of, ipc_mbox_of_match);
static struct platform_driver ipc_mbox_driver = {
	.driver = {
		.name = "bst-mbox",    //this name need change
		.of_match_table = ipc_mbox_of_match,
		#ifdef CONFIG_PM_SLEEP
			.pm	= &dw_ipc_pm_ops,
		#endif
	},
	.probe		= ipc_mbox_probe,
	.remove		= ipc_mbox_remove,
};

static int32_t __init ipc_mbox_init(void)
{
	return platform_driver_register(&ipc_mbox_driver);
}

subsys_initcall(ipc_mbox_init);
