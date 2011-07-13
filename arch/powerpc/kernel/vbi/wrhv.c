/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2, or (at your option) any
 *  later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  Copyright (C) 2009 Wind River Systems, Inc.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/profile.h>
#include <linux/wrhv.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <vbi/interface.h>
#include <vbi/interrupt.h>
#include <vbi/errors.h>

#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/time.h>

#include <linux/threads.h>
#include <linux/kernel_stat.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/timex.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/seq_file.h>
#include <linux/cpumask.h>
#include <linux/bitops.h>
#include <linux/list.h>
#include <linux/radix-tree.h>
#include <linux/mutex.h>
#include <linux/bootmem.h>
#include <linux/pci.h>
#include <linux/debugfs.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/cache.h>
#include <asm/prom.h>
#include <asm/machdep.h>
#include <asm/udbg.h>
#include <asm/firmware.h>

#include <asm/pgalloc.h>
#include <asm/mmu_context.h>
#include <asm/mmu.h>
#include <asm/smp.h>
#include <asm/btext.h>
#include <asm/tlb.h>
#include <asm/sections.h>
#include <asm/pgtable.h>

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/stddef.h>
#include <linux/highmem.h>
#include <linux/initrd.h>
#include <linux/pagemap.h>

#include <linux/kprobes.h>
#include <linux/kexec.h>
#include <linux/backlight.h>
#include <linux/bug.h>
#include <linux/kdebug.h>
#include <linux/kallsyms.h>

#include <mm/mmu_decl.h>
#include <linux/lmb.h>

#include <linux/major.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>
#include <linux/spi/spi.h>
#include <linux/fsl_devices.h>
#include <linux/fs_enet_pd.h>
#include <linux/fs_uart_pd.h>

#include <asm/irq.h>
#include <sysdev/fsl_soc.h>
#include <asm/cpm2.h>

#include <asm/current.h>
#include <asm/processor.h>

#include <asm/paravirt.h>

#include <linux/perf_event.h>
#include <asm/trace.h>
#include <trace/irq.h>
#include <asm/ptrace.h>

/* powerpc clocksource/clockevent code */

#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/kgdb.h>

/* Context switching code */
#include <asm/mmu_context.h>
#include <asm/tlbflush.h>

#include <asm/cputhreads.h>
#include <linux/irq.h>
#include <asm/tlb.h>
#include <asm/arch_vbi.h>

#include <vbi/vbi.h>
#include <vbi/interface.h>

/*
 * We will set the default interrupt handler address into exec_table before
 * early_init, then adjust the interrupt handler in early_init according to
 * the cpu type. Since the early_init will zero all the bss section, we can't
 * put the exec_table in bss section.
 */
VBI_EXC_OFFSETS_TABLE  exec_table __attribute__((__section__(".data")));
struct vb_config *wr_config;		/* TODO kernel relocation friendly ? */
struct vb_control *wr_control;
struct vb_status *wr_status;
EXPORT_SYMBOL(wr_config);
EXPORT_SYMBOL(wr_status);

extern int is_wrhv_duart_inited;

void wrhv_mapping(void);
void mpc85xx_power_down(void);

extern int map_page(unsigned long, phys_addr_t, int);

extern int vb_context_mmu_on(int pid,  /* context id */
			void *pgtable,    /* level 1 page table */
			int pagesize, int asid, int vmmu_handle, int debug);

/* declared in linux/arch/powerpc/kernel/time.c */

#define p_mapped_by_bats(x)     (0UL)
#define p_mapped_by_tlbcam(x)   (0UL)

unsigned long wrhv_cpu_freq = 0;

int wrhv_earlycon = -1;
char wrhv_macaddr[6];

char wrhv_net_name[15]; /* eth0, eth1, eth2... */
int wrhv_nic_num = 0; /* start with eth0 as default */
int wrhv_nic_start = 0;

unsigned int first_context, last_context;
unsigned int next_context, nr_free_contexts;
unsigned long *context_map;
unsigned long *stale_map[NR_CPUS];
struct mm_struct **context_mm;

EXPORT_SYMBOL(context_mm);
EXPORT_SYMBOL(stale_map);

DEFINE_RAW_SPINLOCK(wrhv_context_lock);

#define CTX_MAP_SIZE	\
	(sizeof(unsigned long) * (last_context / BITS_PER_LONG + 1))

extern unsigned int steal_context_up(unsigned int id);
static void context_check_map(void) { }
#define NO_CONTEXT	((unsigned long) -1)
#define ASID_LAST_CONTEXT	CONFIG_WRHV_NUM_ASID
#define ASID_FIRST_CONTEXT	2 /* ASID of 1 is reserved in the HV
					for kernel context, 2 and > are
					assumed to be userspace */
#undef CONFIG_WRHV_DEBUG

#ifdef CONFIG_WRHV_ASID_OPTIMIZATION
#define KERNEL_BASE_ASID 1
#else
#define KERNEL_BASE_ASID 0
#endif

#define WRHV_EARLYCON_SIZE  14  /* sizeof("wrhv_earlycon=") */
int __init wrhv_earlycon_setup(void)
{
	char *p = NULL;

	if ((p = strstr(cmd_line, "wrhv_earlycon=")) != NULL) {
	/* Since the maximal number supported of serial port is 8
	 * in legacy serial, so here we just use this convention
	 */
		wrhv_earlycon =  p[WRHV_EARLYCON_SIZE] - 0x30;
		printk(KERN_INFO "WRHV: early serial output port is at %d\n",
			wrhv_earlycon);
		return 1;
	}

	return 0;
}

#define WRHV_IF_NAME 7 /* sizeof(ifname) */
#define ETH_LENGTH   4 /* length of eth */
static int __init wrhv_net_name_setup(void)
{
	char *p = NULL;

	if ((p = strstr(cmd_line, "ifname=")) != NULL) {
		printk(KERN_INFO "WRHV: replacing MAC address for %s\n", p);
		/* First occurance of ifname= in the bootline. Only copy 1
		"ethX" for now */
		strncpy(wrhv_net_name, p + WRHV_IF_NAME, ETH_LENGTH);
		wrhv_nic_start = p[WRHV_IF_NAME + ETH_LENGTH-1] - 0x30;
		return 1;
	}
	return 0;
}
__setup("ifname=", wrhv_net_name_setup);

/* How many nics are we setting up? */
#define WRHV_IFNIC_SIZE 7
static int __init wrhv_net_num_setup(void)
{
	char *p = NULL;

	if ((p = strstr(cmd_line, "ifnics=")) != NULL) {
		wrhv_nic_num = p[WRHV_IFNIC_SIZE] - 0x30;
		return 1;
	}
	return 0;
}
__setup("ifnics=", wrhv_net_num_setup);


static int __init wrhv_macaddr_setup(char *str)
{
	int i;

	for (i = 0; i < 6; i++) {
		int ret, oct;

		ret = get_option(&str, &oct);
		if (!ret)
			break;

		wrhv_macaddr[i] = oct;
		if (ret != 2)
			break;
	}
	return 1;
}
__setup("ifmacaddr=", wrhv_macaddr_setup);

uint32_t service_handle;
void get_hv_bsp_server_handle(void)
{
	int32_t rc;

	rc = vbi_ns_lookup ("bspServer", 0, &service_handle,
						VBI_NS_NO_TIMEOUT, VBI_NS_OPTION_NONE);
	if (rc)
		printk ("bspServer lookup returned error code: %d\n", rc);
}

int bsp_service_handle(VBI_BSP_MSG *ask_msg, VBI_BSP_MSG_REPLY *reply_msg)
{
	int32_t rc = -1;

	if (!service_handle) {
		printk(KERN_ERR "Can't get bsp service handle!\n");
		return rc;
	}

	rc = vbi_send (service_handle, ask_msg, sizeof(VBI_BSP_MSG),
		reply_msg, sizeof(VBI_BSP_MSG_REPLY), NULL, NULL);

	if (rc)
		printk("vbi_send to the bspServer returned error code: %d\n", rc);

        return rc;
}

uint32_t get_bsp_clock_freq(void)
{
	VBI_BSP_MSG clk_msg;
	VBI_BSP_MSG_REPLY clk_reply;
	uint16_t rc = -1;

	clk_msg.request = VBI_BSP_CORE_FREQ; /* request CPU freq */

	rc = bsp_service_handle(&clk_msg, &clk_reply);
	if (rc)
		return rc;
	return (clk_reply.dataVal);
}

int wrhv_pci_law = 0;
int __init wrhv_enable_pci_law(void)
{
	const char *opt;
	opt = strstr(cmd_line, "wrhv_pci_law=");
	if (opt) {
		opt += 13;
		while (*opt && *opt == ' ')
			opt++;
		if (!strncmp(opt, "on", 2))
			wrhv_pci_law = 1;
		else
			wrhv_pci_law = 0;
	}
	
	return 0;
}

int wrhv_dir_irq = 0;
static int __init wrhv_enable_dir_irq(void)
{
	const char *opt;
	opt = strstr(cmd_line, "wrhv_dir_irq=");
	if (opt) {
		opt += 13;
		while (*opt && *opt == ' ')
			opt++;
		if (!strncmp(opt, "on", 2))
			wrhv_dir_irq = 1;
		else
			wrhv_dir_irq = 0;
	}

	return 0;
}

static void wrhv_map_pages(unsigned long addr, size_t size, int flags)
{
	unsigned long va, i;
	int r;

	va = addr & PAGE_MASK;
	size = PAGE_ALIGN(addr + size) - va;

	for (i = 0; i < size; i += PAGE_SIZE) {
		r = map_page(va + i, va + i, flags);
		WARN_ON_ONCE(r);
	}
}

void wrhv_mapping(void)
{
	struct config_page_map *pConfigPageMap = &wr_config->configPageMap[0];
	uint32_t index = wr_config->configPageNum, i = 0;
	for (i = 0;i < index;i++) {
#ifdef WRHV_CONFIG_DEBUG
		printk("Region No.: %d Address 0x%p Access Privilege 0x%x"
			" Mapping Size 0x%x\n",i,
			pConfigPageMap[i].address,
			pConfigPageMap[i].accessPriv,
			pConfigPageMap[i].size);
#endif
		wrhv_map_pages((unsigned long)pConfigPageMap[i].address,
			pConfigPageMap[i].size,
#ifdef CONFIG_PPC85xx_VT_MODE
			pConfigPageMap[i].accessPriv ? PAGE_KERNEL : PAGE_KERNEL_RO);
#else
			pConfigPageMap[i].accessPriv ? PAGE_KERNEL_X : PAGE_KERNEL_ROX);
#endif
	}

	return;
}

unsigned long __init wrhv_find_end_of_memory(void)
{
	return wr_config->phys_mem_size;
}

int __init wrhv_early_init_dt_scan_memory_ppc(unsigned long node,
			const char *uname, int depth, void *data)
{
	/* instead of using the memory size from
	 * device tree, we use RamSize from linux.xml
	 */
	u64 base, size;
	/*
	 * add the first memory region which is
	 * from 0x00000000 to end of virtual board memory.
	 */
	base = 0x00000000ul;
	size = wrhv_find_end_of_memory();
	lmb_add(base, size);
	memstart_addr = min((u64)memstart_addr, base);

	return 0;
}

void wrhv_power_save(void)
{
	local_irq_enable();

	/*
	 * wrhv_power_save() is called by cpu_idle. When running in direct irq or
	 * duart mode, we need core 0 to handle interrupt from hypervisor. In this
	 * case we do not allow core 0 to go into idle.
	 */
#ifdef CONFIG_WRHV_DUART
	if ((wrhv_dir_irq) || (is_wrhv_duart_inited)) {
#else
	if (wrhv_dir_irq) {
#endif
		/* running in direct irq or using duart */
		if (smp_processor_id())
			vbi_idle(1);
	}
	else {
		/* not using direct irq and not using duart */
		vbi_idle(1);
	}

}

static void wrhv_do_restart(void *data)
{
	int ret;
	int cpu = smp_processor_id();

	if (!cpu) {
		printk(KERN_INFO "WRHV: rebooting \n");

		ret = vbi_vb_reset(VBI_BOARD_ID_GET(), VBI_VB_CORES_ALL,
				VBI_VBMGMT_RESET_AND_START_CORE0 |
				VBI_VBMGMT_RESET_DOWNLOAD
				);

		if (ret)
			printk(KERN_ERR "WRHV: reboot failed. ret = %d\n", ret);
	}
}

void wrhv_restart(char *cmd)
{
	int cpu = smp_processor_id();

	if (!cpu)
		wrhv_do_restart(NULL);
	else
		smp_call_function(wrhv_do_restart, NULL, 1);

	while (1);
}

void __devinit wrhv_calibrate_decr(void)
{
	ppc_tb_freq = VBI_TIMESTAMP_FREQ_GET();
	ppc_proc_freq = wrhv_cpu_freq;
	printk(KERN_DEBUG "WRHV-TIME: wrhv_cpu_freq=%lu  ppc_tb_freq =%lu\n",
			wrhv_cpu_freq, ppc_tb_freq);

	if (VBI_TICK_TIMER_FREQ_GET() != CONFIG_HZ) {
		printk(KERN_ERR "Mismatch between CONFIG_HZ and TickTimerFreq detected!");
		printk(KERN_ERR "  Your decrementer has not been setup correctly\n");
		BUG();
	}
}

void __init wrhv_time_init(void)
{
	return;
}

#define LAW_TARGET_ID (0xff << 20)
#define LAW_EN (0x1 << 31)
#define LAW_SIZE_MASK (0x1f)

unsigned int high_base = -1;
unsigned int low_base = -1;
unsigned int law_attrib = -1;
unsigned int law_offset = 0x10;
unsigned char *law_base;
int law_num;
int law_prepare_done;
int ppc_prepare_law_setup(void)
{
	struct device_node *dev;
	const int *prop;

	if (law_prepare_done == 0) {
		dev = of_find_compatible_node(NULL, NULL, "fsl,corenet-law");
		if (!dev) {
			printk(KERN_ERR
				"%s: No corenet law device node.\n", __func__);
			return -1;
		}

		law_base = of_iomap(dev, 0);
		if (!law_base) {
			printk(KERN_ERR
				"%s: Can't iomap corenet law.\n", __func__);
			return -1;
		}

		/* Get law numbers property */
		prop = of_get_property(dev, "fsl,num-laws", NULL);
		if (prop)
			law_num = *prop;
		else
			return -1;

		/* Get the high base offset for setting law address */
		prop = of_get_property(dev, "high-base", NULL);
		if (prop)
			high_base = *prop;
		else
			return -1;

		/* Get the low base offset for setting the law address
		no need to bail if this property is missing since the
		8572 does not have this property */
		prop = of_get_property(dev, "low-base", NULL);
		if (prop)
			low_base = *prop;

		/* Get the law attribute used for setting law attributes */
		prop = of_get_property(dev, "law-attrib", NULL);
		if (prop)
			law_attrib = *prop;
		else
			return -1;

		/* Get the law register offset for setting law address */
		prop = of_get_property(dev, "offset", NULL);
		if (prop)
			law_offset = *prop;

		law_prepare_done = 1;
	}

	return 0;
}

int ppc_search_free_law(int target_id, unsigned long long addr)
{
	int i;
	static int index = -1;

	if ((!ppc_md.set_law_attr) || (!ppc_md.set_law_base) || (!ppc_md.get_law_attr))
		return -1;

	for (i = index + 1; i < law_num; i++) {
		u32 base = ppc_md.get_law_base(i);
		u32 attr = ppc_md.get_law_attr(i);
		u32 len = 1 << ((attr & LAW_SIZE_MASK) + 1) - 1;

		/*
		 * clean LAW settings in cases:
		 * #1. target bus has valid LAW setting
		 * #2. target address space was mapped yet
		 */
		if (((attr & LAW_TARGET_ID) == (target_id << 20))
				|| (base == addr))
			ppc_md.set_law_attr(i, 0);

		/*
		 * we'd better to check if existing LAW setting
		 * overlapped with us, some *strange* thing might
		 * happen
		 */
		if (addr > base && addr < base + len)
			printk(KERN_WARNING "\n\t--- Warning!!! ---\n\t"
					"Existing LAW setting partly overlapped with "
					"new setting:\n\t"
					"LAW[%d] already mapped 0x%08x to 0x%08x\n",
					i, base, base + len);

		/* Skip these used LAW item */
		if (attr & LAW_EN)
			continue;

		index = i;
		break;
	}

	return  index;
}

void ppc_setup_law(unsigned int target_id, unsigned long long addr, unsigned int attr)
{
	int index;
	ppc_prepare_law_setup();

	index = ppc_search_free_law(target_id, addr);
	if (index >= 0) {
		printk(KERN_INFO "WRHV-setup-law: index: "
				"%#x, addr: %#llx, attr: %#x\n",
				index, addr, attr);
		ppc_md.set_law_base(index, addr);
		ppc_md.set_law_attr(index, attr);
	} else {
		printk(KERN_ERR "WRHV-setup-law: fail setup LAW[%d] "
				"with addr:[%#llx], attr:[%#x]\n",
				target_id, addr, attr);
	}
}

int ppc_setup_pci_law( struct device_node *dev)
{
	unsigned long long pci_addr, cpu_addr, pci_next, cpu_next, size;
	const u32 *ranges;
	int rlen;
	int pna = 0;
	int np = 0;
	u32 pci_space, attr;
	int pcie_index = 0;
	const int *prop;
	char *space_type;

	if (!dev)
		return -ENODEV;

	if (wrhv_pci_law == 1) {
		/* Get PCIE target id property */
		prop = of_get_property(dev, "target-id", NULL);
		if (prop)
			pcie_index = *prop;
		else
			return -1;

		pna = of_n_addr_cells(dev);
		np = pna + 5;
		/* Get ranges property */
		ranges = of_get_property(dev, "ranges", &rlen);
		if (ranges == NULL)
			return -1;
		/* Parse it */
		while ((rlen -= np * 4) >= 0) {
			/* Read next ranges element */
			pci_space = ranges[0];
			pci_addr = of_read_number(ranges + 1, 2);
			cpu_addr = of_translate_address(dev, ranges + 3);
			size = of_read_number(ranges + pna + 3, 2);
			ranges += np;
			if (cpu_addr == OF_BAD_ADDR || size == 0)
				continue;

			/* Now consume following elements while they are contiguous */
			for (; rlen >= np * sizeof(u32);
				     ranges += np, rlen -= np * 4) {
				if (ranges[0] != pci_space)
					break;
				pci_next = of_read_number(ranges + 1, 2);
				cpu_next = of_translate_address(dev, ranges + 3);
				if (pci_next != pci_addr + size ||
						    cpu_next != cpu_addr + size)
					break;
				size += of_read_number(ranges + pna + 3, 2);
			}

			if (((pci_space >> 24 ) & 0x3) == 0x1)
				space_type = "I/O";

			if (((pci_space >> 24 ) & 0x3) == 0x2)
				space_type = "MEM";

			printk(KERN_INFO "WRHV: fixup LAW for PCIE[%d] %s Space "
					"[ 0x%016llx..0x%016llx ]\n", pcie_index,
					space_type, cpu_addr, cpu_addr + size - 1);

			attr = LAW_EN | (pcie_index << 20) | (__ilog2(size) - 1) ;
			ppc_setup_law(pcie_index, cpu_addr, attr);
		}
	}

	return 0;
}

int wrhv_set_law_base(int index, unsigned long long addr)
{
	/* Set High base address */
	/* The secret here is that the P4080 has both a high-base and low-base
	   where as the 8572 only has a high-base attribute */
	if (low_base == -1)
		out_be32((unsigned int *)(law_base + high_base + index * law_offset), (unsigned int)(addr >> 12));
	else
	{
		out_be32((unsigned int *)(law_base + high_base + index * law_offset), (addr >> 32) & 0xf);
		/* Set Low base address */
		out_be32((unsigned int *)(law_base + low_base + index * law_offset), (unsigned int)addr);
	}
	return 0;
}

unsigned long long wrhv_get_law_base(int index)
{
	unsigned long long volatile val = 0;
	if (low_base == -1)
		val = in_be32((unsigned int *)(law_base + high_base + index * law_offset));

	return (val << 12);
}

int wrhv_set_law_attr(int index, unsigned int attr)
{
	/* Set Attributes */
	out_be32((unsigned int *)(law_base + law_attrib + index * law_offset), attr);
	return 0;
}
 
int wrhv_get_law_attr(int index)
{
	unsigned int volatile attr = -1;

	/* Get Attributes */
	attr = in_be32((unsigned int *)(law_base + law_attrib + index * law_offset));
	return attr;
}

#ifdef CONFIG_SPARSE_IRQ
#define WRHV_NR_IRQS	NR_IRQS_LEGACY
#else
#define WRHV_NR_IRQS	NR_IRQS
#endif
void __init wrhv_init_irq(void)
{
	int i;
	struct irq_desc *desc;

#ifdef CONFIG_SMP
	/* Be default all the irqs will be routed to core0 */
	cpumask_copy(irq_default_affinity, cpumask_of(0));
#endif

	wrhv_irq_chip.typename = "WRHV-PIC";
	for (i = 0; i < WRHV_NR_IRQS; i++) {
		desc = irq_to_desc_alloc_node(i, 0);
		desc->status = IRQ_DISABLED | IRQ_LEVEL;
		desc->action = NULL;
		desc->depth = 1;
		set_irq_chip_and_handler(i, &wrhv_irq_chip, handle_fasteoi_irq);
	}
}

#ifdef CONFIG_DEBUG_VIRTUAL_IRQS
static irqreturn_t wrhv_vbint(int irq, void * dev_id)
{
	printk("[DEBUG VIRTUAL IRQS] Handling the DEBUG IRQ %d\n", irq);
	return IRQ_HANDLED;
}

static int __init wrhv_late_init_irq(void)
{
	int dev_id = 1;
	int i;

	/* IRQ 0 is unknown IRQ number for Hypervisor */
	for (i = 1; i < 32; i++) {
		if(request_irq(i, wrhv_vbint, IRQF_SHARED, "vbint_single", &dev_id))
			printk("Unable request IRQ for IRQ %d\n", i);
	}

	return 0;
}
subsys_initcall(wrhv_late_init_irq);
#endif

unsigned int wrhv_vioapic_get_irq(void)
{
	unsigned int irq;

	irq = wr_control->irq_pend;

#ifdef CONFIG_DEBUG_VIRTUAL_IRQS
	/* Maybe this is useless for real external interrupt */
	wr_status->irq_pend = 0;
#endif

	if (irq == 0xffff)
		irq = NO_IRQ_IGNORE;
	else
		wr_control->irq_pend = 0xffff;

	return irq;
}

/* We get irw by reading EPR for handling direct interrupt. 
 * Note the irq should be asked automatically by the hardware 
 * once read EPR. 
 */
unsigned int wrhv_get_direct_irq(void)
{
	unsigned int irq = mfspr(SPRN_EPR);
	/* When reboot some devices may generate spurious irq so here
	 * eoi that and skip this unnecessary interrupt process.
	 */
	if (!irq_to_desc(irq)) {
		printk(KERN_INFO "wrhv: spurious interrupt %d acknowledged.\n",
			irq);
		vbi_di_eoi();
		irq = NO_IRQ;
	}
	return irq;
}

/* refer to native implementation in arch/powerpc/kernel/irq.c */
extern inline void check_stack_overflow(void);
extern inline void handle_one_irq(unsigned int irq);

/* Refer to wrhv_do_IRQ */
void wrhv_do_direct_IRQ(struct pt_regs *regs)
{
	struct pt_regs *old_regs = set_irq_regs(regs);
	unsigned int irq;

	trace_irq_entry(0, regs, NULL);

	irq_enter();

	check_stack_overflow();

	irq = ppc_md.get_direct_irq();

	if (irq != NO_IRQ && irq != NO_IRQ_IGNORE) {
		handle_one_irq(irq);
	} else if (irq != NO_IRQ_IGNORE)
		__get_cpu_var(irq_stat).spurious_irqs++;

	irq_exit();
	set_irq_regs(old_regs);

#ifdef CONFIG_PPC_ISERIES
	if (firmware_has_feature(FW_FEATURE_ISERIES) &&
			get_lppaca()->int_dword.fields.decr_int) {
		get_lppaca()->int_dword.fields.decr_int = 0;
		/* Signal a fake decrementer interrupt */
		timer_interrupt(regs);
	}
#endif

	trace_irq_exit(IRQ_HANDLED);

}

void wrhv_do_IRQ(struct pt_regs *regs)
{
	struct pt_regs *old_regs = set_irq_regs(regs);
	unsigned int irq;

	trace_irq_entry(0, regs, NULL);

	irq_enter();

	check_stack_overflow();

check_again:
	irq = ppc_md.get_irq();

	if (irq != NO_IRQ && irq != NO_IRQ_IGNORE) {
		handle_one_irq(irq);
		goto check_again;	
	} else if (irq != NO_IRQ_IGNORE)
		__get_cpu_var(irq_stat).spurious_irqs++;

	irq_exit();
	set_irq_regs(old_regs);

#ifdef CONFIG_PPC_ISERIES
	if (firmware_has_feature(FW_FEATURE_ISERIES) &&
			get_lppaca()->int_dword.fields.decr_int) {
		get_lppaca()->int_dword.fields.decr_int = 0;
		/* Signal a fake decrementer interrupt */
		timer_interrupt(regs);
	}
#endif

	trace_irq_exit(IRQ_HANDLED);
}

struct irq_desc * __ref wrhv_irq_to_desc(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);
	if (desc)
		return desc;

#ifdef CONFIG_SPARSE_IRQ
	desc = irq_to_desc_alloc_node(irq, 0);
	if (unlikely(!desc)) {
		pr_err("can't get irq_desc for irq%d\n", irq);
		goto out;
	}

	desc->status = IRQ_DISABLED | IRQ_LEVEL;
	desc->action = NULL;
	desc->depth = 1;
	set_irq_chip_and_handler(irq, &wrhv_irq_chip, handle_fasteoi_irq);
out:
#endif
	return desc;
}
EXPORT_SYMBOL(wrhv_irq_to_desc);

unsigned int wrhv_map_irq_of_desc(char *irq_name, int32_t irq_dir)
{
	int irq;
	struct irq_desc *desc;

	irq = vbi_find_irq(irq_name, irq_dir);
	if (irq == VBI_INVALID_IRQ) {
		printk(KERN_WARNING "WRHV: no defined INT for [ %s ].\n",
				irq_name);
		return NO_IRQ;
	}

	desc = wrhv_irq_to_desc(irq);
	if (unlikely(!desc))
		return NO_IRQ;

	return irq;
}

unsigned int wrhv_irq_of_parse_and_map(struct device_node *dev, int index)
{
	char *p, tmp[120];

	/* Currently we only support 0 ~ 9 index */
	if (index > 9 || index < 0)
		return NO_IRQ;

	if (!index)
		p = dev->full_name;
	else {
		int i;

		p = tmp;
		strncpy(p, dev->full_name, 120 - 2);
		i = strlen(p);
		p[i++] = '#';
		p[i++] = index + '0';
		p[i]   = '\0';
	}

	return wrhv_map_irq_of_desc(p, VB_INPUT_INT);
}

unsigned int wrhv_get_pvr(void)
{
	return wr_status->vb_status_regs.pvr;
}

unsigned int wrhv_get_svr(void)
{
	return wr_status->vb_status_regs.svr;
}

static void wrhv_set_mode(enum clock_event_mode mode,
				 struct clock_event_device *dev)
{
	return;
}

static int wrhv_set_next_event(unsigned long evt,
				      struct clock_event_device *dev)
{
	return 0;
}

/* See arch/x86/kernel/vbi/wrhv.c */
static struct clock_event_device wrhv_clockevent = {
	.name		= "wrhv",
	.features	= CLOCK_EVT_FEAT_PERIODIC,
	.set_mode	= wrhv_set_mode,
	.set_next_event = wrhv_set_next_event,
	.max_delta_ns	= 0xffffffff,
	.min_delta_ns	= 10000,
	.shift		= 32,   /* nanoseconds to cycles divisor 2^ */
	.mult		= 1,     /* To be filled in */
	.irq		= 0,
	.rating		= 1,
};

/* Refer to arch/powerpc/kernel/time.c. */
#undef	test_perf_event_pending
#undef	clear_perf_event_pending

#ifdef CONFIG_PERF_EVENTS
extern DEFINE_PER_CPU(u8, perf_event_pending);
#define test_perf_event_pending()	__get_cpu_var(perf_event_pending)
#define clear_perf_event_pending()	__get_cpu_var(perf_event_pending) = 0
#else
#define test_perf_event_pending()	0
#define clear_perf_event_pending()
#endif

void wrhv_hw_timer_interrupt(struct pt_regs * regs)
{
	struct pt_regs *old_regs;

	trace_timer_interrupt_entry(regs);

	__get_cpu_var(irq_stat).timer_irqs++;

#ifdef CONFIG_PPC32
	if (atomic_read(&ppc_n_lost_interrupts) != 0)
		do_IRQ(regs);
#endif

	old_regs = set_irq_regs(regs);
	irq_enter();

	calculate_steal_time();

	if (test_perf_event_pending()) {
		clear_perf_event_pending();
		perf_event_do_pending();
	}

	wrhv_timer_interrupt(0, NULL);

	irq_exit();
	set_irq_regs(old_regs);

	trace_timer_interrupt_exit(regs);
}

void __init wrhv_time_init_cont(void)
{
	wrhv_clockevent.cpumask = get_cpu_mask(0);
	clockevents_register_device(&wrhv_clockevent);
}


/* arch/powerpc/mm/fault.c */
void wrhv_vmmu_restore(void)
{
	/*
	 * called by the end of page fault handling to reinstall the vmmu
	 */
	wr_control->vmmu0 = wr_status->vmmu0;
	wr_control->vmmu1 = wr_status->vmmu1;
#ifdef CONFIG_WRHV_ASID_OPTIMIZATION
	wr_control->vb_control_regs.vmmu_handle =
		wr_status->vb_status_regs.vmmu_handle;
	wr_control->vb_control_regs.asid =
		wr_status->vb_status_regs.asid;
#endif
	return;
}

/* arch/powerpc/mm/fsl_booke_mmu.c */
void __init wrhv_MMU_init_hw(void)
{
	return;
}

unsigned long __init wrhv_mmu_mapin_ram(unsigned long top)
{
       return 0;
}

/* arch/powerpc/mm/init_32.c */
void wrhv_MMU_setup(void)
{
	__map_without_bats = 1;

#ifdef CONFIG_DEBUG_PAGEALLOC
	__map_without_bats = 1;
	__map_without_ltlbs = 1;
#endif
}

void __init wrhv_MMU_init(void)
{
#ifdef DEBUG
	int i;
#endif

	if (ppc_md.progress)
		ppc_md.progress("MMU:enter", 0x111);

	/* parse args from command line */
	wrhv_MMU_setup();

	if (lmb.memory.cnt > 1) {
		lmb.memory.cnt = 1;
		lmb_analyze();
		printk(KERN_WARNING "Only using first contiguous memory region");
	}

	total_lowmem = total_memory = lmb_end_of_DRAM() - memstart_addr;
	lowmem_end_addr = memstart_addr + total_lowmem;

	if (total_lowmem > __max_low_memory) {
		total_lowmem = __max_low_memory;
		lowmem_end_addr = memstart_addr + total_lowmem;
#ifndef CONFIG_HIGHMEM
		total_memory = total_lowmem;
		lmb_enforce_memory_limit(lowmem_end_addr);
		lmb_analyze();
#endif /* CONFIG_HIGHMEM */
	}

	/* Initialize the MMU hardware */
	if (ppc_md.progress)
		ppc_md.progress("MMU:hw init", 0x300);
	MMU_init_hw();

	/* Map in all of RAM starting at KERNELBASE */
	if (ppc_md.progress)
		ppc_md.progress("MMU:mapin", 0x301);
	mapin_ram();

	/* Initialize early top-down ioremap allocator */
	ioremap_bot = IOREMAP_TOP;

	/* Map in I/O resources */
	if (ppc_md.progress)
		ppc_md.progress("MMU:setio", 0x302);

	if (ppc_md.progress)
		ppc_md.progress("MMU:exit", 0x211);

	/* From now on, btext is no longer BAT mapped if it was at all */
#ifdef CONFIG_BOOTX_TEXT
	btext_unmap();
#endif

#ifndef CONFIG_PPC85xx_VT_MODE
	/*
	 * we enable the mmu here without having to do this from the caller
	 * (which is in assembly world)
	 */
	vb_context_mmu_on(0, swapper_pg_dir, PAGE_SIZE, KERNEL_BASE_ASID,
		KERNEL_BASE_ASID, 0);
#endif

	/* Check if enable direct interrupt mode. */
	wrhv_enable_dir_irq();
	
	vbi_set_exc_offset(&exec_table);

#ifdef DEBUG
	printk("****DUMP EXEC OFFSET AFTER SET***\n");
	vbi_get_exc_offset(&exec_table);
	for(i=0;i<VBI_ARCH_MAX_EXC_OFFSETS;i++)
		printk("execoffset:%d	----	0x%08x\n",i,exec_table.excOffset[i]);
#endif
}

/* arch/powerpc/mm/mem.c */
extern void __flush_dcache_icache_phys(unsigned long physaddr);
void wrhv_flush_dcache_page(struct page *page)
{
	if (cpu_has_feature(CPU_FTR_COHERENT_ICACHE))
		return;
	/* avoid an atomic op if possible */
	if (test_bit(PG_arch_1, &page->flags))
		clear_bit(PG_arch_1, &page->flags);

	vbi_flush_dcache((void *)(page_to_pfn(page) << PAGE_SHIFT),
					 PAGE_SIZE);
}

void set_context(unsigned long contextid, pgd_t *pgd) 
	__attribute__((weak, alias("wrhv_set_context")));

/* arch/powerpc/mm/mmu_context_32.c */
void wrhv_set_context(unsigned long contextId, pgd_t * pgd)
{

	pgd_t * kpdStart, *kpdEnd, *updStart;
	/* we attach (copy) kernel page mapping to the user page table
	 * Note, we only copy the L1 entrys to user L1 pageTable,
	 * then letting L1 share the same L2 page table
	 */

	kpdStart = pgd_offset_k(KERNELBASE);
	kpdEnd =   pgd_offset_k(0xffffffff);

	updStart = pgd + pgd_index(KERNELBASE);

	memcpy(updStart, kpdStart, (kpdEnd - kpdStart + 1) * sizeof (pgd_t));

	/* in linux context, page table entry is not set up yet */
#ifdef CONFIG_WRHV_ASID_OPTIMIZATION
	vb_context_mmu_on(contextId, pgd, PAGE_SIZE,
		wr_control->vb_control_regs.asid,
		wr_control->vb_control_regs.vmmu_handle, 0);
#else
	vb_context_mmu_on(contextId, pgd, PAGE_SIZE, KERNEL_BASE_ASID,
		KERNEL_BASE_ASID, 0);
#endif
}

/* arch/powerpc/mm/pgtable_32.c */
int wrhv_map_page(unsigned long va, phys_addr_t pa, int flags)
{
	pmd_t *pd;
	pte_t *pg;
	int err = -ENOMEM;

	/* Use upper 10 bits of VA to index the first level map */
	pd = pmd_offset(pud_offset(pgd_offset_k(va), va), va);
	/* Use middle 10 bits of VA to index the second-level map */
	pg = pte_alloc_kernel(pd, va);
	if (pg != 0) {
		err = 0;
		/* The PTE should never be already set nor present in the
		 * hash table
		 */
		BUG_ON(pte_val(*pg) & (_PAGE_PRESENT | _PAGE_HASHPTE));
		set_pte_at(&init_mm, va, pg, pfn_pte(pa >> PAGE_SHIFT,
						     __pgprot(flags)));
#if defined(CONFIG_WRHV_E500) && !defined(CONFIG_PHYS_64BIT) \
			      && !defined(CONFIG_PPC85xx_VT_MODE)
		if (current->active_mm != &init_mm)
			*pgd_offset(current->active_mm, va) =
						*pgd_offset_k(va);
#endif
	}

	return err;
}

void __iomem *
wrhv___ioremap(phys_addr_t addr, unsigned long size, unsigned long flags)
{
	unsigned long v, i;
	phys_addr_t p;
	int err;

	/* writeable implies dirty for kernel addresses */
	if (flags & _PAGE_RW)
		flags |= _PAGE_DIRTY | _PAGE_HWWRITE;

	/* we don't want to let _PAGE_USER and _PAGE_EXEC leak out */
	flags &= ~(_PAGE_USER | _PAGE_EXEC);

	/* Make sure we have the base flags */
	if ((flags & _PAGE_PRESENT) == 0)
		flags |= PAGE_KERNEL;

	/* Non-cacheable page cannot be coherent */
	if (flags & _PAGE_NO_CACHE)
		flags &= ~_PAGE_COHERENT;

	/*
	 * Choose an address to map it to.
	 * Once the vmalloc system is running, we use it.
	 * Before then, we use space going down from ioremap_base
	 * (ioremap_bot records where we're up to).
	 */
	p = addr & PAGE_MASK;
	size = PAGE_ALIGN(addr + size) - p;

	/*
	 * If the address lies within the first 16 MB, assume it's in ISA
	 * memory space
	 */
	if (p < 16*1024*1024)
		p += _ISA_MEM_BASE;

#ifndef CONFIG_CRASH_DUMP
	/*
	 * Don't allow anybody to remap normal RAM that we're using.
	 * mem_init() sets high_memory so only do the check after that.
	 */
	if (mem_init_done && (p < virt_to_phys(high_memory))) {
		printk("__ioremap(): phys addr 0x%llx is RAM lr %p\n",
		       (unsigned long long)p, __builtin_return_address(0));
		return NULL;
	}
#endif

	if (size == 0)
		return NULL;

	/*
	 * Is it already mapped?  Perhaps overlapped by a previous
	 * BAT mapping.  If the whole area is mapped then we're done,
	 * otherwise remap it since we want to keep the virt addrs for
	 * each request contiguous.
	 *
	 * We make the assumption here that if the bottom and top
	 * of the range we want are mapped then it's mapped to the
	 * same virt address (and this is contiguous).
	 *  -- Cort
	 */
	if ((v = p_mapped_by_bats(p)) /*&& p_mapped_by_bats(p+size-1)*/ )
		goto out;

	if ((v = p_mapped_by_tlbcam(p)))
		goto out;

	if (mem_init_done) {
		struct vm_struct *area;
		area = get_vm_area(size, VM_IOREMAP);
		if (area == 0)
			return NULL;
		v = (unsigned long) area->addr;
	} else {
		v = (ioremap_bot -= size);
	}

	/*
	 * Should check if it is a candidate for a BAT mapping
	 */

	err = 0;
	for (i = 0; i < size && err == 0; i += PAGE_SIZE)
		err = map_page(v+i, p+i, flags);
	if (err) {
		if (mem_init_done)
			vunmap((void *)v);
		return NULL;
	}

/* Just E500 Guest OS need copy kernel PTEs in ioremap.
 * And, don't support 36 bit physical address now.
 */
#if !defined(CONFIG_PPC85xx_VT_MODE) && !defined(CONFIG_PHYS_64BIT)
	{
		pgd_t *kpd_start, *kpd_end, *upd_start, *pgd;
		if (mem_init_done && (current->mm != NULL) && (current->mm != &init_mm)) {
			pgd = current->mm->pgd;

			/* we attach (copy) kernel page mapping to the user page table
			 * Note, we only copy the L1 entrys to user L1 pageTable,
			 * then letting L1 share the same L2 page table.
			 */
			kpd_start = pgd_offset_k(KERNELBASE);
			kpd_end =   pgd_offset_k(0xffffffff);

			upd_start = pgd + pgd_index(KERNELBASE);
			memcpy(upd_start, kpd_start, (kpd_end - kpd_start + 1) * sizeof (pgd_t));
		}
	}
#endif
out:
	return (void __iomem *) (v + ((unsigned long)addr & ~PAGE_MASK));
}

#if defined(CONFIG_WRHV_GUEST_PROTECTION)
/* In native Linux the kernel page table addresses are marked as supervisor
 * RWX only.  So that the Linux guest can protect kernel memory space from
 *  nefarious users the same range of pages need to be set. */
#define KERNEL_MEMORY_START	(CONFIG_KERNEL_START)
#define KERNEL_MEMORY_END	(CONFIG_KERNEL_START+0x0FFFFFFF)
#endif

/* From arch/powerpc/include/asm/pgtable.h */
static inline void wrhv__set_pte_at(struct mm_struct *mm, unsigned long addr,
				pte_t *ptep, pte_t pte, int percpu)
{
#if defined(CONFIG_PPC_STD_MMU_32) && defined(CONFIG_SMP) && !defined(CONFIG_PTE_64BIT)
	/* First case is 32-bit Hash MMU in SMP mode with 32-bit PTEs. We use the
	 * helper pte_update() which does an atomic update. We need to do that
	 * because a concurrent invalidation can clear _PAGE_HASHPTE. If it's a
	 * per-CPU PTE such as a kmap_atomic, we do a simple update preserving
	 * the hash bits instead (ie, same as the non-SMP case)
	 */
	if (percpu)
		*ptep = __pte((pte_val(*ptep) & _PAGE_HASHPTE)
			      | (pte_val(pte) & ~_PAGE_HASHPTE));
	else
		pte_update(ptep, ~_PAGE_HASHPTE, pte_val(pte));

#elif defined(CONFIG_PPC32) && defined(CONFIG_PTE_64BIT)
	/* Second case is 32-bit with 64-bit PTE.  In this case, we
	 * can just store as long as we do the two halves in the right order
	 * with a barrier in between. This is possible because we take care,
	 * in the hash code, to pre-invalidate if the PTE was already hashed,
	 * which synchronizes us with any concurrent invalidation.
	 * In the percpu case, we also fallback to the simple update preserving
	 * the hash bits
	 */
	if (percpu) {
		*ptep = __pte((pte_val(*ptep) & _PAGE_HASHPTE)
			      | (pte_val(pte) & ~_PAGE_HASHPTE));
		return;
	}
#if _PAGE_HASHPTE != 0
	if (pte_val(*ptep) & _PAGE_HASHPTE)
		flush_hash_entry(mm, ptep, addr);
#endif
	__asm__ __volatile__("\
		stw%U0%X0 %2,%0\n\
		eieio\n\
		stw%U0%X0 %L2,%1"
	: "=m" (*ptep), "=m" (*((unsigned char *)ptep+4))
	: "r" (pte) : "memory");

#elif defined(CONFIG_PPC_STD_MMU_32)
	/* Third case is 32-bit hash table in UP mode, we need to preserve
	 * the _PAGE_HASHPTE bit since we may not have invalidated the previous
	 * translation in the hash yet (done in a subsequent flush_tlb_xxx())
	 * and see we need to keep track that this PTE needs invalidating
	 */
	*ptep = __pte((pte_val(*ptep) & _PAGE_HASHPTE)
		      | (pte_val(pte) & ~_PAGE_HASHPTE));

#else
	/* Anything else just stores the PTE normally. That covers all 64-bit
	 * cases, and 32-bit non-hash with 32-bit PTEs.
	 */
	*ptep = pte;

#if defined(CONFIG_WRHV) && !defined(CONFIG_PPC85xx_VT_MODE)
	/* linux does not use valid bit, hypervisor does, in word0 */
	*(u_int *)ptep |= (u_int) VMMU_PTE_VALID_MASK;

#if defined(CONFIG_WRHV_GUEST_PROTECTION)
	/* The P4080 does not suffer from lack of protection like that
	of the 8548 and 8572. */
	if ((addr >= KERNEL_MEMORY_START) && (addr <= KERNEL_MEMORY_END))
		*(u_int *)ptep |= (u_int) VMMU_PTE_SUPER_MASK;
#endif

#endif /* CONFIG_WRHV */

#endif

}

static void wrhv_handle_debug(struct pt_regs *regs, unsigned long debug_status)
{
	int changed = 0;
	/*
	 * Determine the cause of the debug event, clear the
	 * event flags and send a trap to the handler. Torez
	 */
	if (debug_status & (DBSR_DAC1R | DBSR_DAC1W)) {
		dbcr_dac(current) &= ~(DBCR_DAC1R | DBCR_DAC1W);
#ifdef CONFIG_PPC_ADV_DEBUG_DAC_RANGE
		current->thread.dbcr2 &= ~DBCR2_DAC12MODE;
#endif
		do_send_trap(regs, mfspr(SPRN_DAC1), debug_status, TRAP_HWBKPT,
			     5);
		changed |= 0x01;
	}  else if (debug_status & (DBSR_DAC2R | DBSR_DAC2W)) {
		dbcr_dac(current) &= ~(DBCR_DAC2R | DBCR_DAC2W);
		do_send_trap(regs, mfspr(SPRN_DAC2), debug_status, TRAP_HWBKPT,
			     6);
		changed |= 0x01;
	}  else if (debug_status & DBSR_IAC1) {
		current->thread.dbcr0 &= ~DBCR0_IAC1;
		dbcr_iac_range(current) &= ~DBCR_IAC12MODE;
		do_send_trap(regs, mfspr(SPRN_IAC1), debug_status, TRAP_HWBKPT,
			     1);
		changed |= 0x01;
	}  else if (debug_status & DBSR_IAC2) {
		current->thread.dbcr0 &= ~DBCR0_IAC2;
		do_send_trap(regs, mfspr(SPRN_IAC2), debug_status, TRAP_HWBKPT,
			     2);
		changed |= 0x01;
	}  else if (debug_status & DBSR_IAC3) {
		current->thread.dbcr0 &= ~DBCR0_IAC3;
		dbcr_iac_range(current) &= ~DBCR_IAC34MODE;
		do_send_trap(regs, mfspr(SPRN_IAC3), debug_status, TRAP_HWBKPT,
			     3);
		changed |= 0x01;
	}  else if (debug_status & DBSR_IAC4) {
		current->thread.dbcr0 &= ~DBCR0_IAC4;
		do_send_trap(regs, mfspr(SPRN_IAC4), debug_status, TRAP_HWBKPT,
			     4);
		changed |= 0x01;
	}
	/*
	 * At the point this routine was called, the MSR(DE) was turned off.
	 * Check all other debug flags and see if that bit needs to be turned
	 * back on or not.
	 */
	if (DBCR_ACTIVE_EVENTS(current->thread.dbcr0, current->thread.dbcr1))
		regs->msr |= MSR_DE;
	else
		/* Make sure the IDM flag is off */
		current->thread.dbcr0 &= ~DBCR0_IDM;

	if (changed & 0x01)
		mtspr(SPRN_DBCR0, current->thread.dbcr0);
}

/* arch/powerpc/include/asm/reg.h */
#ifdef CONFIG_PPC85xx_VT_MODE
void wrhv_mtspr(unsigned int sprn, unsigned int value)
{

	switch(sprn){
		case SPRN_DBCR0:
			__asm__ __volatile__(
			"lis    3,%0@h\n"
			"ori    3,3,%0@l\n"
			"mr	4,%1\n"
			"mtspr	0x134,4\n"
			::"i" (SPRN_DBCR0_W), "r" (value)
			);
			break;

		case SPRN_DBSR:
			__asm__ __volatile__(
			"lis    3,%0@h\n"
			"ori    3,3,%0@l\n"
			"mr	4,%1\n"
			"mtspr	0x130,4\n"
			::"i" (SPRN_DBSR_W), "r" (value)
			);
			break;

		case SPRN_IAC1:
			__asm__ __volatile__(
			"lis    3,%0@h\n"
			"ori    3,3,%0@l\n"
			"mr	4,%1\n"
			"mtspr	0x138,4\n"
			::"i" (SPRN_IAC1_W), "r" (value)
			);
			break;

		case SPRN_IAC2:
			__asm__ __volatile__(
			"lis    3,%0@h\n"
			"ori    3,3,%0@l\n"
			"mr	4,%1\n"
			"mtspr	0x139,4\n"
			::"i" (SPRN_IAC2_W), "r" (value)
			);
			break;

		case SPRN_DAC1:
			__asm__ __volatile__(
			"lis    3,%0@h\n"
			"ori    3,3,%0@l\n"
			"mr	4,%1\n"
			"mtspr	0x13C,4\n"
			::"i" (SPRN_DAC1_W), "r" (value)
			);
			break;

		case SPRN_DAC2:
			__asm__ __volatile__(
			"lis    3,%0@h\n"
			"ori    3,3,%0@l\n"
			"mr	4,%1\n"
			"mtspr	0x13D,4\n"
			::"i" (SPRN_DAC2_W), "r" (value)
			);
			break;

		case SPRN_DBCR1:
			__asm__ __volatile__(
			"lis    3,%0@h\n"
			"ori    3,3,%0@l\n"
			"mr	4,%1\n"
			"mtspr	0x135,4\n"
			::"i" (SPRN_DBCR1_W), "r" (value)
			);
			break;

		case SPRN_DBCR2:
			__asm__ __volatile__(
			"lis    3,%0@h\n"
			"ori    3,3,%0@l\n"
			"mr	4,%1\n"
			"mtspr	0x136,4\n"
			::"i" (SPRN_DBCR2_W), "r" (value)
			);
			break;
	}
}
#else
void wrhv_mtspr(unsigned int sprn, unsigned int value)
{
	switch(sprn){
		case SPRN_DBCR0:
			wr_control->vb_control_regs.dbcr0 = value;
			break;

		case SPRN_DBSR:
			wr_control->vb_control_regs.dbsr = value;
			break;
	}
}
#endif

/* arch/powerpc/kernel/process.c */ 
extern void wrhv_mtspr(unsigned int, unsigned int);
void wrhv_prime_debug_regs(struct thread_struct *thread)
{
	wrhv_mtspr(SPRN_IAC1, thread->iac1);
	wrhv_mtspr(SPRN_IAC2, thread->iac2);
#if CONFIG_PPC_ADV_DEBUG_IACS > 2
	wrhv_mtspr(SPRN_IAC3, thread->iac3);
	wrhv_mtspr(SPRN_IAC4, thread->iac4);
#endif
	wrhv_mtspr(SPRN_DAC1, thread->dac1);
	wrhv_mtspr(SPRN_DAC2, thread->dac2);
#if CONFIG_PPC_ADV_DEBUG_DVCS > 0
	wrhv_mtspr(SPRN_DVC1, thread->dvc1);
	wrhv_mtspr(SPRN_DVC2, thread->dvc2);
#endif
	wrhv_mtspr(SPRN_DBCR0, thread->dbcr0);
	wrhv_mtspr(SPRN_DBCR1, thread->dbcr1);
#ifdef CONFIG_BOOKE
	wrhv_mtspr(SPRN_DBCR2, thread->dbcr2);
#endif
}

#ifdef CONFIG_PPC85xx_VT_MODE
unsigned int wrhv_mfspr(unsigned int sprn)
{
	unsigned int value = 0;
	switch(sprn){
		case SPRN_DBCR0:
			__asm__ __volatile__(
			"lis    3,%1@h\n"
			"ori    3,3,%1@l\n"
			"mfspr	4,0x134\n"
			"mr	%0,4\n"
			:"=r" (value)
			:"i" (SPRN_DBCR0_R)
			);
			break;

		case SPRN_DBSR:
			__asm__ __volatile__(
			"lis    3,%1@h\n"
			"ori    3,3,%1@l\n"
			"mfspr	4,0x130\n"
			"mr	%0,4\n"
			:"=r" (value)
			:"i" (SPRN_DBSR_R)
			);
			break;

		case SPRN_IAC1:
			__asm__ __volatile__(
			"lis    3,%1@h\n"
			"ori    3,3,%1@l\n"
			"mfspr	4,0x138\n"
			"mr	%0,4\n"
			:"=r" (value)
			:"i" (SPRN_IAC1_R)
			);
			break;

		case SPRN_IAC2:
			__asm__ __volatile__(
			"lis    3,%1@h\n"
			"ori    3,3,%1@l\n"
			"mfspr	4,0x139\n"
			"mr	%0,4\n"
			:"=r" (value)
			:"i" (SPRN_IAC2_R)
			);
			break;

		case SPRN_DAC1:
			__asm__ __volatile__(
			"lis    3,%1@h\n"
			"ori    3,3,%1@l\n"
			"mfspr	4,0x13C\n"
			"mr	%0,4\n"
			:"=r" (value)
			:"i" (SPRN_DAC1_R)
			);
			break;

		case SPRN_DAC2:
			__asm__ __volatile__(
			"lis    3,%1@h\n"
			"ori    3,3,%1@l\n"
			"mfspr	4,0x13D\n"
			"mr	%0,4\n"
			:"=r" (value)
			:"i" (SPRN_DAC2_R)
			);
			break;

		case SPRN_DBCR1:
			__asm__ __volatile__(
			"lis    3,%1@h\n"
			"ori    3,3,%1@l\n"
			"mfspr	4,0x135\n"
			"mr	%0,4\n"
			:"=r" (value)
			:"i" (SPRN_DBCR1_R)
			);
			break;

		case SPRN_DBCR2:
			__asm__ __volatile__(
			"lis    3,%1@h\n"
			"ori    3,3,%1@l\n"
			"mfspr	4,0x136\n"
			"mr	%0,4\n"
			:"=r" (value)
			:"i" (SPRN_DBCR2_R)
			);
			break;
	}

	return value;
}
#else
unsigned int wrhv_mfspr(unsigned int sprn)
{
	unsigned int value = 0;
	switch(sprn){
		case SPRN_DBCR0:
			value = wr_control->vb_control_regs.dbcr0;
			break;

		case SPRN_DBSR:
			value = wr_control->vb_control_regs.dbsr;
			break;
	}

	return value;
}
#endif

/* arch/powerpc/kernel/traps.c */
void __kprobes wrhv_DebugException(struct pt_regs *regs, unsigned long debug_status)
{
	debug_status = wr_control->vb_control_regs.dbsr;
	wr_control->vb_control_regs.emsr &= ~MSR_DE;

	current->thread.dbsr = debug_status;

	/* Hack alert: On BookE, Branch Taken stops on the branch itself, while
	 * on server, it stops on the target of the branch. In order to simulate
	 * the server behaviour, we thus restart right away with a single step
	 * instead of stopping here when hitting a BT
	 */
	if (debug_status & DBSR_BT) {
		regs->msr &= ~MSR_DE;

#ifdef CONFIG_PPC85xx_VT_MODE
		/* Disable BT */
		wrhv_mtspr(SPRN_DBCR0, wrhv_mfspr(SPRN_DBCR0) & ~DBCR0_BT);
		/* Clear the BT event */
		wrhv_mtspr(SPRN_DBSR, DBSR_BT);
#endif

		/* Do the single step trick only when coming from userspace */
		if (user_mode(regs)) {
			current->thread.dbcr0 &= ~DBCR0_BT;
			current->thread.dbcr0 |= DBCR0_IDM | DBCR0_IC;
			regs->msr |= MSR_DE;
			return;
		}

		if (notify_die(DIE_SSTEP, "block_step", regs, 5,
			       5, SIGTRAP) == NOTIFY_STOP) {
			return;
		}
		if (debugger_sstep(regs))
			return;
	} else if (debug_status & DBSR_IC) { 	/* Instruction complete */
		regs->msr &= ~MSR_DE;

#ifdef CONFIG_PPC85xx_VT_MODE
		/* Disable instruction completion */
		wrhv_mtspr(SPRN_DBCR0, wrhv_mfspr(SPRN_DBCR0) & ~DBCR0_IC);
		/* Clear the instruction completion event */
		wrhv_mtspr(SPRN_DBSR, DBSR_IC);
#endif

		if (notify_die(DIE_SSTEP, "single_step", regs, 5,
			       5, SIGTRAP) == NOTIFY_STOP) {
			return;
		}

		if (debugger_sstep(regs))
			return;

		if (user_mode(regs)) {
			current->thread.dbcr0 &= ~DBCR0_IC;
#ifdef CONFIG_PPC_ADV_DEBUG_REGS
			if (DBCR_ACTIVE_EVENTS(current->thread.dbcr0,
					       current->thread.dbcr1))
				regs->msr |= MSR_DE;
			else
				/* Make sure the IDM bit is off */
				current->thread.dbcr0 &= ~DBCR0_IDM;
#endif
		}

		_exception(SIGTRAP, regs, TRAP_TRACE, regs->nip);
	} else
		wrhv_handle_debug(regs, debug_status);
}

/* arch/powerpc/kernel/kgdb.c */
int wrhv_kgdb_arch_handle_exception(int vector, int signo, int err_code,
			       char *remcom_in_buffer, char *remcom_out_buffer,
			       struct pt_regs *linux_regs)
{
	char *ptr = &remcom_in_buffer[1];
	unsigned long addr;

	switch (remcom_in_buffer[0]) {
		/*
		 * sAA..AA   Step one instruction from AA..AA
		 * This will return an error to gdb ..
		 */
	case 's':
	case 'c':
		/* handle the optional parameter */
		if (kgdb_hex2long(&ptr, &addr))
			linux_regs->nip = addr;

		atomic_set(&kgdb_cpu_doing_single_step, -1);
		/* set the trace bit if we're stepping */
		if (remcom_in_buffer[0] == 's') {
#ifdef CONFIG_PPC_ADV_DEBUG_REGS
#ifdef CONFIG_PPC85xx_VT_MODE
			wrhv_mtspr(SPRN_DBCR0,
			      wrhv_mfspr(SPRN_DBCR0) | DBCR0_IC | DBCR0_IDM);
#else
			wr_control->vb_control_regs.dbcr0 |= (DBCR0_IC | DBCR0_IDM);
#endif
			wr_control->vb_control_regs.emsr |= MSR_DE;
			linux_regs->msr |= MSR_DE;
#else
			linux_regs->msr |= MSR_SE;
#endif
			kgdb_single_step = 1;
			atomic_set(&kgdb_cpu_doing_single_step,
				   raw_smp_processor_id());
		}
		return 0;
	}
	return -1;
}

int wrhv_ppc_cpu_freq(void)
{
	return wrhv_cpu_freq;
}

/* Use bus 0 by default. On p4080 bus is encoded as famn0:dtsec0. */
uint32_t wrhv_mdio_bus = 0;
int wrhv_mdio_write(struct mii_bus *bus, int mii_id, int devad, int regnum,
			u16 value)
{
	VBI_BSP_MSG		mdio_msg;
	VBI_BSP_MSG_REPLY	mdio_reply;
	int			rc = -1;
	u32			bid = 0;

	if (ppc_md.get_mdio_bus)
		bid = ppc_md.get_mdio_bus(bus, mii_id);

	mdio_msg.request = VBI_MDIO_WRITE;
	mdio_msg.arg.mdioWrite.bus = bid;
	mdio_msg.arg.mdioWrite.phyAddr = (uint32_t) mii_id;
	mdio_msg.arg.mdioWrite.regNum = (uint32_t) regnum;
	mdio_msg.arg.mdioWrite.page = 0;
	mdio_msg.arg.mdioWrite.dataVal = value;

	rc = bsp_service_handle(&mdio_msg, &mdio_reply);

	if (rc || mdio_reply.status)
		rc = -1;

	return rc;

}

int wrhv_mdio_read(struct mii_bus *bus, int mii_id, int devad, int regnum)
{
	VBI_BSP_MSG		mdio_msg;
	VBI_BSP_MSG_REPLY	mdio_reply;
	int			rc = -1;
	u32			bid = 0;

	if (ppc_md.get_mdio_bus)
		bid = ppc_md.get_mdio_bus(bus, mii_id);

	mdio_msg.request = VBI_MDIO_READ;
	mdio_msg.arg.mdioRead.bus = bid;
	mdio_msg.arg.mdioRead.phyAddr = (uint32_t) mii_id;
	mdio_msg.arg.mdioRead.regNum = (uint32_t) regnum;
	mdio_msg.arg.mdioRead.page = 0;

	rc = bsp_service_handle(&mdio_msg, &mdio_reply);

	if (rc || mdio_reply.status || (mdio_reply.dataVal >> 16 != 0))
		return -1;

	return mdio_reply.dataVal;
}

#ifdef CONFIG_WRHV_ASID_OPTIMIZATION
/* Clone of: arch/powerpc/mm/mmu_context_nohash.c */

unsigned int wrhv_steal_context_up(unsigned int id)
{
	struct mm_struct *mm;
	int cpu = smp_processor_id();

	static VMMU_CONFIG vmmu_cfg;

	/* Pick up the victim mm */
	mm = context_mm[id];
#ifdef CONFIG_WRHV_DEBUG
	printk(" | steal %d from 0x%p MM->ctxID %d \n", id, mm, mm->context.id);
#endif

	/* Flush the TLB for that context */
	local_flush_tlb_mm(mm);

	/* Mark this mm has having no context anymore */
	mm->context.id = MMU_NO_CONTEXT;

	/* XXX This clear should ultimately be part of local_flush_tlb_mm */
	__clear_bit(id, stale_map[cpu]);

	return id;
}



int wrhv_init_new_context(struct task_struct *t, struct mm_struct *mm)
{
	unsigned long ctx = next_context;
	static VMMU_CONFIG vmmu_cfg;
	unsigned int ret_code;
	pgd_t *kpdStart, *kpdEnd, *updStart;


	pgd_t *pgd = mm->pgd;

	kpdStart = pgd_offset_k(KERNELBASE);
	kpdEnd =   pgd_offset_k(0xffffffff);

	updStart = pgd + pgd_index(KERNELBASE);

	memcpy(updStart, kpdStart, (kpdEnd - kpdStart + 1) * sizeof (pgd_t));

	vmmu_cfg.addr = (VMMU_LEVEL_1_DESC *) pgd;
	vmmu_cfg.flush_type = VMMU_TLB_FLUSH_ASID;
	vmmu_cfg.asid = ctx;
	vmmu_cfg.vmmu_handle = ctx;

	ret_code = vbi_create_vmmu(&vmmu_cfg);
	if (ret_code) {
		printk(" Error creating VMMU handles \n");
	}
	mm->context.vmmu_handle = vmmu_cfg.vmmu_handle;

	mm->context.id = NO_CONTEXT;
	mm->context.active = 0;

	return 0;
}


void wrhv_destroy_context(struct mm_struct *mm)
{
	unsigned long flags;
	unsigned int id;
	static VMMU_CONFIG vmmu_cfg;

	if (mm->context.id == NO_CONTEXT)
		return;

	vmmu_cfg.addr = -1;
	vmmu_cfg.flush_type = VMMU_TLB_FLUSH_ASID;
	vmmu_cfg.asid = mm->context.id;
	vmmu_cfg.vmmu_handle = mm->context.vmmu_handle;

	WARN_ON(mm->context.active != 0);

	raw_spin_lock_irqsave(&wrhv_context_lock, flags);
	id = mm->context.id;
	if (id != NO_CONTEXT) {
		__clear_bit(id, context_map);
		mm->context.id = NO_CONTEXT;
#ifdef DEBUG_MAP_CONSISTENCY
		mm->context.active = 0;
#endif
		context_mm[id] = NULL;
		nr_free_contexts++;
	}
	vbi_delete_vmmu(&vmmu_cfg);
	raw_spin_unlock_irqrestore(&wrhv_context_lock, flags);
}

void wrhv_switch_mmu_context(struct mm_struct *prev, struct mm_struct *next)
{
	unsigned int i, id, cpu = smp_processor_id();
	unsigned long *map;
	static VMMU_CONFIG vmmu_cfg;

	/* No lockless fast path .. yet */
	raw_spin_lock(&wrhv_context_lock);
#ifdef DEBUG_MAP_CONSISTENCY
	printk("[%d] activating context for mm @%p, active=%d, id=%d",
		cpu, next, next->context.active, next->context.id);
#endif
#ifdef CONFIG_SMP
	/* Mark us active and the previous one not anymore */
	next->context.active++;
	if (prev) {
		printk(" (old=0x%p a=%d)", prev, prev->context.active);
		WARN_ON(prev->context.active < 1);
		prev->context.active--;
	}

 again:
#endif /* CONFIG_SMP */

	/* If we already have a valid assigned context, skip all that */
	id = next->context.id;
	if (likely(id != NO_CONTEXT)) {
#ifdef DEBUG_MAP_CONSISTENCY
		if (context_mm[id] != next)
			printk("MMU: mm 0x%p has id %d but context_mm[%d] says 0x%p\n",
				next, id, id, context_mm[id]);
#endif
		goto ctxt_ok;
	}

	/* We really don't have a context, let's try to acquire one */
	id = next_context;
	if (id > last_context)
		id = first_context;
	map = context_map;

	/* No more free contexts, let's try to steal one */
	if (nr_free_contexts == 0) {
#ifdef CONFIG_SMP
		if (num_online_cpus() > 1) {
			id = steal_context_smp(id);
			if (id == NO_CONTEXT)
				goto again;
			goto stolen;
		}
#endif /* CONFIG_SMP */
		id = wrhv_steal_context_up(id);
		goto stolen;
	}
	nr_free_contexts--;

	/* We know there's at least one free context, try to find it */
	while (__test_and_set_bit(id, map)) {
		id = find_next_zero_bit(map, last_context+1, id);
		if (id > last_context)
			id = first_context;
	}
 stolen:
	if (id <= first_context)
		id = first_context;
	next_context = id + 1;
	context_mm[id] = next;
	next->context.id = id;
#ifdef CONFIG_WRHV_DEBUG
	printk(" | new id=%d,nrf=%d", id, nr_free_contexts);
#endif

	context_check_map();
 ctxt_ok:

	/* If that context got marked stale on this CPU, then flush the
	 * local TLB for it and unmark it before we use it
	 */
	if (test_bit(id, stale_map[cpu])) {
#ifdef CONFIG_WRHV_DEBUG
		printk(" | stale flush %d [%d..%d]",
				id, cpu_first_thread_in_core(cpu),
				cpu_last_thread_in_core(cpu));
#endif

		local_flush_tlb_mm(next);

		/* XXX This clear should ultimately be part of local_flush_tlb_mm */
		for (i = cpu_first_thread_in_core(cpu);
			i <= cpu_last_thread_in_core(cpu); i++) {
			__clear_bit(id, stale_map[i]);
		}
	}

	/* Flick the MMU and release lock */
#ifdef CONFIG_WRHV_ASID_OPTIMIZATION
#ifdef CONFIG_WRHV_DEBUG
	printk(" -> %d\n", id);
#endif
	vb_context_mmu_on(id, next->pgd, PAGE_SIZE, next->context.id,
		next->context.vmmu_handle, 0);
#else
	set_context(id, next->pgd);
#endif
	raw_spin_unlock(&wrhv_context_lock);
}


/*
 * Initialize the context management stuff.
 */
void __init wrhv_mmu_context_init(void)
{
	/* Mark init_mm as being active on all possible CPUs since
	 * we'll get called with prev == init_mm the first time
	 * we schedule on a given CPU
	 */
	init_mm.context.active = NR_CPUS;

	/*
	 *   The MPC8xx has only 16 contexts.  We rotate through them on each
	 * task switch.  A better way would be to keep track of tasks that
	 * own contexts, and implement an LRU usage.  That way very active
	 * tasks don't always have to pay the TLB reload overhead.  The
	 * kernel pages are mapped shared, so the kernel can run on behalf
	 * of any task that makes a kernel entry.  Shared does not mean they
	 * are not protected, just that the ASID comparison is not performed.
	 *      -- Dan
	 *
	 * The IBM4xx has 256 contexts, so we can just rotate through these
	 * as a way of "switching" contexts.  If the TID of the TLB is zero,
	 * the PID/TID comparison is disabled, so we can use a TID of zero
	 * to represent all kernel pages as shared among all contexts.
	 * 	-- Dan
	 */
	first_context = ASID_FIRST_CONTEXT;
	last_context = ASID_LAST_CONTEXT;

	/*
	 * Allocate the maps used by context management
	 */
	context_map = alloc_bootmem(CTX_MAP_SIZE);
	context_mm = alloc_bootmem(sizeof(void *) * (last_context + 1));
	stale_map[0] = alloc_bootmem(CTX_MAP_SIZE);

#ifdef CONFIG_SMP
	register_cpu_notifier(&mmu_context_cpu_nb);
#endif

	printk(KERN_INFO
		"MMU: Allocated %zu bytes of context maps for %d contexts\n",
		2 * CTX_MAP_SIZE + (sizeof(void *) * (last_context + 1)),
		last_context - first_context + 1);

	/*
	 * Some processors have too few contexts to reserve one for
	 * init_mm, and require using context 0 for a normal task.
	 * Other processors reserve the use of context zero for the kernel.
	 * This code assumes first_context < 32.
	 */
	context_map[0] = (1 << first_context) - 1;
	next_context = first_context;
	nr_free_contexts = last_context - first_context + 1;
}
#endif

void wrhv_duart_init(void);
void wrhv_udbg_init_uart(void __iomem *comport, unsigned int speed,
		unsigned int clock)
{
#ifdef CONFIG_WRHV_DUART
	/* initialize hypervisor duart serial device driver */
	wrhv_duart_init();
#endif
}

void wrhv_init(void)
{
	/* initialize wr_config so that we can access
	 * vbi configuration. The vbi configuration space
	 * is defined in Hypervisor linux.xml
	 */
	wr_config = (struct vb_config *)0xF0000000;
	wr_control = wr_config->vb_control;
	wr_status = wr_config->vb_status;

	pv_info.name = "wrhv";
	pv_info.paravirt_enabled = 1;

	pv_time_ops.time_init_cont = wrhv_time_init_cont;
	pv_time_ops.timer_interrupt = wrhv_hw_timer_interrupt;
	pv_time_ops.clocksource_init = native_clocksource_init;

	pv_irq_ops.do_IRQ = wrhv_do_IRQ;
	pv_irq_ops.irq_of_parse_and_map =
			wrhv_irq_of_parse_and_map;

#ifndef CONFIG_PPC85xx_VT_MODE
	pv_cpu_ops.get_pvr = wrhv_get_pvr;
	pv_cpu_ops.get_svr = wrhv_get_svr;
#endif

	pv_cpu_ops.DebugException = wrhv_DebugException;
	pv_cpu_ops.prime_debug_regs = wrhv_prime_debug_regs;
	pv_cpu_ops.kgdb_arch_handle_exception =
		wrhv_kgdb_arch_handle_exception;
	pv_cpu_ops.ppc_proc_freq =
		wrhv_ppc_cpu_freq;

#ifndef CONFIG_PPC85xx_VT_MODE
	pv_mmu_ops.vmmu_restore = wrhv_vmmu_restore;
#ifdef CONFIG_WRHV_ASID_OPTIMIZATION
	pv_context_ops.init_new_context = wrhv_init_new_context;
	pv_context_ops.destroy_context = wrhv_destroy_context;
	pv_context_ops.switch_mmu_context = wrhv_switch_mmu_context;
	pv_context_ops.mmu_context_init = wrhv_mmu_context_init;
#endif
#endif
	pv_mmu_ops.MMU_init_hw = wrhv_MMU_init_hw;
	pv_mmu_ops.mmu_mapin_ram = wrhv_mmu_mapin_ram;
	pv_mmu_ops.MMU_setup = wrhv_MMU_setup;
	pv_mmu_ops.MMU_init = wrhv_MMU_init;
	pv_mmu_ops.flush_dcache_page = wrhv_flush_dcache_page;
	pv_mmu_ops.map_page = wrhv_map_page;
	pv_mmu_ops.early_init_dt_scan_memory_ppc =
		wrhv_early_init_dt_scan_memory_ppc;
	pv_mmu_ops.__ioremap = wrhv___ioremap;
	pv_mmu_ops.__set_pte_at = wrhv__set_pte_at;
	pv_mdio_ops.fsl_pq_mdio_write = wrhv_mdio_write;
	pv_mdio_ops.fsl_pq_mdio_read = wrhv_mdio_read;

#ifdef CONFIG_PCI
	ppc_pci_set_flags(PPC_PCI_REASSIGN_ALL_RSRC);
#endif

#ifdef CONFIG_WRHV_DUART
	pv_serial_ops.udbg_init_uart = wrhv_udbg_init_uart;
#endif
}

__weak void wrhv_setup_msr_for_ap(VBI_HREG_SET_CMPLX_QUALIFIED *regs)
{
	return;
}
EXPORT_SYMBOL(wrhv_setup_msr_for_ap);

#ifdef CONFIG_SMP
VBI_HREG_SET_CMPLX_QUALIFIED bootREG;
#define IPI_IRQ_BASE_NAME "ipi0"
int irq_base = 0xFFFF; /*init as invalid IRQ number*/
#define WRHV_IPI_NUM      4

DEFINE_PER_CPU(long long, tb_diff);

long long wrhv_gettb_diff()
{
	return __raw_get_cpu_var(tb_diff);
}

void wrhv_settb_diff(long long diff)
{
	 __get_cpu_var(tb_diff) = diff;
}

static irqreturn_t wrhv_ipi_action(int irq, void *data)
{
	long ipi = (long)data;

	smp_message_recv(ipi);

	return IRQ_HANDLED;
}

void __devinit smp_wrhv_setup_cpu(int cpu_nr)
{
	return;
}

static void wrhv_mask_IPIs_for_vcore(void)
{
	int i;

	for (i = 0; i < WRHV_IPI_NUM; i++)
		vbi_mask_vioapic_irq(irq_base + i);
}

void wrhv_umask_IPIs_for_vcore(void)
{
	int i;

	for (i = 0; i < WRHV_IPI_NUM; i++)
		vbi_unmask_vioapic_irq(irq_base + i);
}

void wrhv_request_ipis(void)
{
	static char *ipi_names[] = {
		"IPI0 (call function)",
		"IPI1 (reschedule)",
		"IPI2 (call function single)",
		"IPI3 (debugger break)",
	};
	int i, err;

	printk(KERN_INFO "WRHV requesting IPIs ... \n");

	irq_base = wrhv_map_irq_of_desc(IPI_IRQ_BASE_NAME, VB_INPUT_INT);
	if (irq_base == VBI_INVALID_IRQ)
		panic("WRHV reslove irq for IPI failed.\n");

	for (i = 0; i < WRHV_IPI_NUM; i++) {
		err = request_irq(irq_base + i, wrhv_ipi_action,
				IRQF_DISABLED | IRQF_PERCPU | IRQF_NOBALANCING,
				ipi_names[i], (void *)i);
		if (err) {
			printk(KERN_ERR "WRHV Request of irq %d for %s failed\n",
					irq_base + i, ipi_names[i]);
			/*
			 * There is no really clean work here. The best choice
			 * here might panic() immediately
			 */
			panic("WRHV: request_irq for IPI faild.\n");
		}

		set_irq_chip_and_handler_name(irq_base + i,
				&wrhv_ipi_irq_chip, handle_percpu_irq, "per_cpu");
	}
}

int __init smp_wrhv_probe(void)
{
	int nr_cpus;

	pr_debug("smp_mpic_probe()...\n");

	nr_cpus = cpus_weight(cpu_possible_map);

	pr_debug("nr_cpus: %d\n", nr_cpus);

	if (nr_cpus > 1) {
		wrhv_request_ipis();
		wrhv_umask_IPIs_for_vcore();
	}

	return nr_cpus;
}

static inline void wrhv_send_IPI_mask(int irq, cpumask_t mask)
{
	unsigned long coreset = cpus_addr(mask)[0];
	unsigned long flags;

	local_irq_save(flags);
	WARN_ON(coreset & ~cpus_addr(cpu_online_map)[0]);
	vbi_send_vcore_vioapic_irq(irq, coreset, 0);
	local_irq_restore(flags);
}

void smp_wrhv_message_pass(int target, int msg)
{
	cpumask_t mask, dst;
	int self = smp_processor_id();

	/* make sure we're sending something that translates to an IPI */
	if (msg > 3) {
		printk(KERN_INFO "SMP %d: smp_message_pass: unknown msg %d\n",
				smp_processor_id(), msg);
		return;
	}

	switch (target) {
		case MSG_ALL:
			cpus_setall(dst);
			cpus_and(mask, dst, cpu_online_map);
			wrhv_send_IPI_mask(msg + irq_base,mask);
			break;
		case MSG_ALL_BUT_SELF:
			cpus_setall(dst);
			cpu_clear(self, dst);
			cpus_and(mask, dst, cpu_online_map);
			wrhv_send_IPI_mask(msg + irq_base, mask);
			break;
		default:
			cpus_clear(dst);
			cpu_set(target, dst);
			cpus_and(mask, dst, cpu_online_map);
			wrhv_send_IPI_mask(msg + irq_base, mask);
			break;
	}
}

static void __devinit smp_wrhv_kick_cpu(int nr)
{
	unsigned long flags;
	int32_t ret;
	
	int n = 0;
	WARN_ON (nr < 0 || nr >= NR_CPUS);

	local_irq_save(flags);

	bootREG.vbiRegType = VBI_REG_SET_32BIT;

	ret = vbi_vb_read_reg(&bootREG, VBI_BOARD_ID_GET(), nr);
	if (ret)
		printk(KERN_ERR "WRHV read REG failed: %d\n", ret);

	bootREG.vbiRegType = VBI_REG_SET_32BIT;
	bootREG.vbiRegSet.hreg32.pc = 0xc0000000;
	wrhv_setup_msr_for_ap(&bootREG);

	ret = vbi_vb_write_reg(&bootREG, VBI_BOARD_ID_GET(), nr);
	if (ret)
		printk(KERN_ERR "WRHV write REG failed: %d\n", ret);

	ret = vbi_vb_resume(VBI_BOARD_ID_GET(), nr);
	if (ret)
		printk(KERN_ERR "WRHV resume CPU failed: %d\n", ret);

	/* Wait a bit for the CPU to ack. */
	while ((__secondary_hold_acknowledge != nr) && (++n < 1000))
		mdelay(1);

	local_irq_restore(flags);
}

unsigned long mpc85xx_smp_message[NR_CPUS]; /*fix doorbell_exception link error */

#ifdef CONFIG_HOTPLUG_CPU
DECLARE_PER_CPU(int, cpu_state);
void cpu_die(void)
{
	unsigned int cpu;
	int ret;

	idle_task_exit();

	local_irq_disable();
	cpu = smp_processor_id();
	printk(KERN_DEBUG "CPU%d offline\n", cpu);
	__get_cpu_var(cpu_state) = CPU_DEAD;
	smp_wmb();

	preempt_enable();
	ret = vbi_vb_suspend(VBI_BOARD_ID_GET(), cpu);
	if (ret)
		printk(KERN_ERR "%s: suspend result: %d\n", __func__, ret);

	while (1);
}

static int wrhv_cpu_disable(void)
{
	int cpu = smp_processor_id();
	printk(KERN_INFO "%s: cpu = %d\n", __func__, cpu);

	if (cpu == boot_cpuid)
		return -EBUSY;

	wrhv_fixup_irqs(cpu);

	wrhv_mask_IPIs_for_vcore();

	set_cpu_online(cpu, false);

	return 0;
}

static int wrhv_cpu_enable(unsigned int cpu)
{
	printk(KERN_INFO "%s: cpu = %d\n", __func__, cpu);

	if (system_state != SYSTEM_RUNNING)
		return -ENOSYS;

	return 1;
}

#endif

struct smp_ops_t smp_wrhv_ops = {
	.kick_cpu = smp_wrhv_kick_cpu,
#if defined(CONFIG_HOTPLUG_CPU) && defined(CONFIG_PPC32)
	.cpu_enable  = wrhv_cpu_enable,
	.cpu_disable = wrhv_cpu_disable,
	.cpu_die = generic_cpu_die,
#endif
	.probe = smp_wrhv_probe,
	.message_pass = smp_wrhv_message_pass,
	.setup_cpu = smp_wrhv_setup_cpu,
	.take_timebase = smp_generic_take_timebase,
	.give_timebase = smp_generic_give_timebase,
};

void __init wrhv_smp_init(void)
{
	smp_ops = &smp_wrhv_ops;
}

extern struct smp_ops_t *smp_ops;
extern volatile unsigned int cpu_callin_map[NR_CPUS];

static void __devinit smp_store_cpu_info(int id)
{
	per_cpu(cpu_pvr, id) = get_pvr();
}

/* Must be called when no change can occur to cpu_present_map,
 * i.e. during cpu online or offline.
 */
static struct device_node *cpu_to_l2cache(int cpu)
{
	struct device_node *np;
	struct device_node *cache;

	if (!cpu_present(cpu))
		return NULL;

	np = of_get_cpu_node(cpu, NULL);
	if (np == NULL)
		return NULL;

	cache = of_find_next_cache_node(np);

	of_node_put(np);

	return cache;
}

/* Activate a secondary processor. */
int __devinit wrhv_start_secondary(void *unused)
{
	unsigned int cpu = smp_processor_id();
	struct device_node *l2_cache;
	int i, base;

	local_irq_disable();
#ifndef CONFIG_PPC85xx_VT_MODE
	vb_context_mmu_on(0, swapper_pg_dir, PAGE_SIZE, KERNEL_BASE_ASID,
			KERNEL_BASE_ASID, 0);
#endif

	wrhv_umask_IPIs_for_vcore();
	vbi_set_exc_offset(&exec_table);

	atomic_inc(&init_mm.mm_count);
	current->active_mm = &init_mm;

	smp_store_cpu_info(cpu);
	preempt_disable();
	cpu_callin_map[cpu] = 1;

	if (smp_ops->setup_cpu)
		smp_ops->setup_cpu(cpu);
	if (smp_ops->take_timebase)
		smp_ops->take_timebase();

	if (system_state > SYSTEM_BOOTING)
		snapshot_timebase();

	ipi_call_lock();
	notify_cpu_starting(cpu);
	set_cpu_online(cpu, true);
	/* Update sibling maps */
	base = cpu_first_thread_in_core(cpu);
	for (i = 0; i < threads_per_core; i++) {
		if (cpu_is_offline(base + i))
			continue;
		cpu_set(cpu, per_cpu(cpu_sibling_map, base + i));
		cpu_set(base + i, per_cpu(cpu_sibling_map, cpu));

		/* cpu_core_map should be a superset of
		 * cpu_sibling_map even if we don't have cache
		 * information, so update the former here, too.
		 */
		cpu_set(cpu, per_cpu(cpu_core_map, base +i));
		cpu_set(base + i, per_cpu(cpu_core_map, cpu));
	}
	l2_cache = cpu_to_l2cache(cpu);
	for_each_online_cpu(i) {
		struct device_node *np = cpu_to_l2cache(i);
		if (!np)
			continue;
		if (np == l2_cache) {
			cpu_set(cpu, per_cpu(cpu_core_map, i));
			cpu_set(i, per_cpu(cpu_core_map, cpu));
		}
		of_node_put(np);
	}
	of_node_put(l2_cache);
	ipi_call_unlock();

	local_irq_enable();

	cpu_idle();
	return 0;
}
#else
long long wrhv_gettb_diff()
{
	return 0;
}
#endif

#ifdef CONFIG_PCI

/* On PPC we have to disable MSI firstly to adopt the legacy interrupt 
 * since the BootROM with supporting PCIE always use MSI way. 
 */
void pci_msi_disable(struct pci_dev *dev)
{
	u16 control;
	int pos = pci_find_capability(dev, PCI_CAP_ID_MSI);

	if (pos) {
		pci_read_config_word(dev, pos + PCI_MSI_FLAGS, &control);
		if (control & PCI_MSI_FLAGS_ENABLE) {
			control &= ~PCI_MSI_FLAGS_ENABLE;
			pci_write_config_word(dev, pos + PCI_MSI_FLAGS, control);
		}
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, pci_msi_disable);
#endif

static int __init parse_memmap_opt(char *p)
{
	char *oldp;
	u64 start, size, offset;

	if (!p)
		return -EINVAL;

	oldp = p;
	size = memparse(p, &p);
	if (p == oldp)
		return -EINVAL;

	if (*p == '$')
		start = memparse(p + 1, &p);
	else
		return -EINVAL;

	/*
	 * The guest physical address of a VB is start from 0. So we can get
	 * the real RAM offset of a VB by using following method.
	 */
	vbi_guest_phys_to_phys(0, &offset);

	start = start - offset;	/* Convert to guest physical address */
	lmb_reserve(start, size);
	return 0;
}
early_param("memmap", parse_memmap_opt);

