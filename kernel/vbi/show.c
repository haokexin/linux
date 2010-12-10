/*
 * show.c - virtual board data show functions
 *
 * Copyright (c) 2007-2010 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 */

/*
DESCRIPTION

The Virtual Board show functions display virtual board status,
control and configuration data structures.

*/

#include <linux/types.h>
#include <linux/kernel.h>
#include <vbi/vbi.h>

static const char * device_class_name[]={"Serial", "Network", "Block",
						"Video", "Audio",
						"Controller",
						"Other", "Software"};

static const char * device_type_name[]={"None", "ADD", "Emulated",
					"Passthrough", "Other"};

/*
 * vbi_show_shmem - print information about the shared memory
 * configuration
 *
 * This routine traverse the array of shared memories descriptor for a
 * given board and displays the information on hypervisor console.
 *
 */

void vbi_show_shmem(void)
{
	struct vb_config *config = VBI_CONFIG_ADDR_GET();
	struct vb_sm_info *p = config->sharedMemoryRegionsConfigAddress;
	int num_sm = config->num_sm;
	void *pState = config->sharedMemoryRegionsStateAddress;
	int i;

	/* standard certifiable hypervisor does not support this function */
	if (cert_hyp_version == CERT_HYP_VER_STD) {
		VBISTAT_VERBOSE(vbi_show_shmem);
		return;
	}

	printk("%d Shared memory regions, Config at 0x%p:\n", num_sm, p);

	/* dump shared memory info if we have any attached */
	for (i=0; i < num_sm; i++,p++) {
		printk("%8s: 0x%p -> 0x%p  (attr: 0x%x)\n",
		p->name, p->addr, p->addr + p->length - 1, p->attr);
	}

	printk("Shared memory State at 0x%p:\n", pState);
}

/*
 * vbi_show_mem - print information about a board's  memory
 * configuration
 *
 * This routine traverse the array of memory regions descriptor for a
 * given board and displays the information on hypervisor console.
 *
 */
void vbi_show_mem(void)

{
	struct vb_config *config = VBI_CONFIG_ADDR_GET();
	struct vb_mem_info *p = config->memoryRegionsConfigAddress;
	int num_sm = config->num_mem;
	int i;

	/* standard certifiable hypervisor does not support this function */
	if (cert_hyp_version == CERT_HYP_VER_STD) {
		VBISTAT_VERBOSE(vbi_show_mem);
		return;
	}

	printk("%d memory regions, Config at 0x%p:\n", num_sm, p);

	/* dump shared memory info if we have any attached */
	for (i=0; i < num_sm; i++,p++) {
		printk("%8s: 0x%p -> 0x%p  (attr: 0x%x) (type: %s)\n",
			p->name, p->addr, p->addr + p->length - 1,
			p->attr, p->type);
	}
}

/*
 * vbi_show_irq - print information about a board's
 * interrupts configuration
 *
 * This routine traverse the interrupts descriptors table for a given
 * board and displays the information on hypervisor console.
 *
 */
static void vbi_show_irq(void)
{
	struct vb_config *config = VBI_CONFIG_ADDR_GET();
	struct vb_int_info *p = config->interruptConfiguration;
	int num_int = config->num_ints;
	int i;

	printk("%d Interrupt Configurations, config at 0x%p\n", num_int, p);

	for (i = 0; i < num_int; i++, p++) {
		if (p->irq_dir == VB_INPUT_INT)
			printk("%8s: Dir: In, Vector #: %d, Core: %d\n",
		            p->irq_name, p->irq_num, p->irq_core);
		else
			printk("%8s: Dir: Out, Vector #: %d\n",
		            p->irq_name, p->irq_num);
	}
}

/*
 * vbi_show_device - print information about a board's device configuration
 *
 * This routine traverse the device information for a given board and
 * displays the information on hypervisor console.
 *
 */

static void vbi_show_device(void)
{
	struct vb_config *config = VBI_CONFIG_ADDR_GET();
	struct vb_dev_info *p = config->deviceConfiguration;
	struct vb_dev_int_info *pi = NULL;
	struct vb_dev_regset_info *pr = NULL;
	struct vb_dev_device_tree_source_info *pd = NULL;
	unsigned int num_devices = config->numDevices;
	unsigned int i, j, k;

	printk("%d device configurations, Config at %p\n", num_devices,
			p);

	for (i = 0; i < num_devices; i++, p++) {

		/* print the device info */

		printk("\n%-8s: %-10s(%d): %-10s: %-10s(%d) "
			"numInts %d numRegs %d numDTS %d\n",
			p->deviceName, device_class_name[p->deviceClass],
			p->deviceClass, p->deviceTemplate,
			device_type_name[p->deviceType], p->deviceType,
			p->numInterrupts, p->numRegSets,
			p->numDeviceTreeSources);

		/* print interrupts for the device */

		if (p->numInterrupts) {
			pi = (struct vb_dev_int_info *)((char *)p +
				p->intInfoOffset);
			for (j = 0; j < p->numInterrupts; j++, pi++) {
				printk("Interrupt of device:%d intName %s "
					"intType %s intNum %d\n",
					pi->indexDevice, pi->intName,
					pi->intType, pi->intNum);
			}
		}

		/* print register set for the device */

		if (p->numRegSets) {
			pr = (struct vb_dev_regset_info *)((char *)p +
				p->regSetInfoOffset);
			for (k = 0; k < p->numRegSets; k++, pr++) {
				printk("Register Set of device:%d regSetName "
					"%s Address %llx Length %llx "
					"Type:%lld\n",
					pr->indexDevice, pr->regSetName,
					pr->regSetAddress, pr->regSetLength,
					pr->regSetType);
			}
		}

		/* print device tree source for the device */

		if (p->numDeviceTreeSources) {
			pd = (struct vb_dev_device_tree_source_info *)
				((char *)p + p->deviceTreeSourceInfoOffset);
			for (k = 0; k < p->numDeviceTreeSources; k++, pd++) {
				printk("Device Tree Source of device:%d "
					"deviceTreeSourceName %s %p\n",
					pd->indexDevice,
					pd->deviceTreeSourceName,
					&pd->deviceTreeSourceName);
			}
		}
	}
}


/*
 * vbi_show_stat - print information about a board's status structures
 *
 * This routine displays information in the status structure for a given board.
 *
 */
void vbi_show_stat(void)
{
	struct vb_status *p = VBI_STATUS_ADDR_GET();

	/* standard certifiable hypervisor does not support this function */
	if (cert_hyp_version == CERT_HYP_VER_STD) {
		VBISTAT_VERBOSE(vbi_show_stat);
		return;
	}
	printk("VB status: 0x%p\n", p);

	printk("  Pending interrupts: 0x%08x\n", p->irq_pend_type);
	printk("  timestamp:          %lld\n", p->timeStamp);
	printk("  old int disable:    0x%08x\n", p->prev_irq_disable);
#ifdef _WRHV_ARCH_HAS_STATUS_REGS
	vbi_disp_status_regs();
#endif
}

/*
 * vbi_show_ctrl - print information about a board's control structures
 *
 */
void vbi_show_ctrl(void)
{
	struct vb_control *p = VBI_CNTRL_ADDR_GET();
	int i;

	/* standard certifiable hypervisor does not support this function */
	if (cert_hyp_version == CERT_HYP_VER_STD) {
		VBISTAT_VERBOSE(vbi_show_ctrl);
		return;
	}

	printk("VB control data: 0x%p\n", p);

	printk("  Disable interrupts:  0x%08x\n", p->irq_disable);

#ifdef _WRHV_ARCH_HAS_STATUS_REGS
	vbi_disp_ctrl_regs();
#endif

	printk("  Disabled interrupts:");
	for (i=0; i < VB_MAX_INTERRUPTS; i++) {
		if ((p->level_irq_disable & (1 << (32 - i))) != 0)
			printk(" %d", i);
	}
	printk("\n");
}

#if defined(CONFIG_WRHV_COREVBI_ONLY)
void vbi_show_config_page_map()
{
	/* certifiable hypervisor does not have page map field in the
	 * vb_config structure
	 */
	VBISTAT_VERBOSE(vbi_show_config_page_map);
	return;
}
#else
void vbi_show_config_page_map()
{
	uint32_t i;
	struct vb_config *config = VBI_CONFIG_ADDR_GET();
	struct config_page_map *pConfigPageMap = &config->configPageMap[0];
	uint32_t index=config->configPageNum;

	printk("vbi_show_config_page_map\n\n");
	for(i=0;i<index;i++)
		printk("Region No.: %d Address 0x%p Access Privilege 0x%x"
			" Mapping Size 0x%x\n",i,
			pConfigPageMap[i].address,
			pConfigPageMap[i].accessPriv,
			(unsigned int)pConfigPageMap[i].size);
}
#endif

/*
 * vbi_show_cfg - print information about a board's config structure
 *
 */

void vbi_show_cfg(void)
{
	struct vb_config *p = VBI_CONFIG_ADDR_GET();

	/* standard certifiable hypervisor does not support this function */
	if (cert_hyp_version == CERT_HYP_VER_STD) {
		VBISTAT_VERBOSE(vbi_show_cfg);
		return;
	}

	printk("VB config data: 0x%p\n", p);

	printk("pid:                %d\n", p->pid);
	printk("Board ID:           %d\n", p->boardID);

	printk("Status Address:     0x%p\n", p->vb_status);
	printk("Control Address:    0x%p\n", p->vb_control);
#if defined (_WRHV_ARCH_HAS_VB_SYSTBL)
	printk("Syscall Table Address:    0x%lx\n",
			p->vbSyscallTable);
#endif
	printk("SupervisoryMode:    %d\n", p->supervisoryMode);

	printk("Board Name:         %s\n", p->board_name);
	printk("Board Type:         %d\n", p->board_type);
	printk("Physical Memory:    %d\n", p->phys_mem_size);
	printk("Memory Alias Addr:  0x%08x\n", p->mem_alias_addr);
	printk("Memory Alias Size:  %u\n", p->mem_alias_size);
	printk("Reset PC:           0x%p\n", p->reset_pc);

	printk("Timer Frequency:     %d/sec\n", p->tick_freq);
	printk("TimeStamp Frequency: %dULL/sec\n", p->stamp_freq);

	printk("num_ints:             %d\n", p->num_ints);

	vbi_show_mem();
	vbi_show_shmem();
	vbi_show_irq();
	vbi_show_device();
	vbi_show_config_page_map();
}
