/*
 * arch/arm/mach-spear13xx/dw_pcie.c
 *
 * PCIe functions for Synopsys DW controllers
 *
 * Copyright (C) 2010-2011 ST Microelectronics
 * Pratyush Anand <pratyush.anand@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/msi.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/slab.h>
#include <mach/dw_pcie.h>
static struct list_head	pcie_port_list;
static struct hw_pci pci;

#define PCI_FIND_CAP_TTL	48

static int pci_find_own_next_cap_ttl(struct pcie_port *pp,
		u32 pos, int cap, int *ttl)
{
	u32 id;

	while ((*ttl)--) {
		pp->ops.rd_own(pp, pos, 1, &pos);
		if (pos < 0x40)
			break;
		pos &= ~3;
		pp->ops.rd_own(pp, pos + PCI_CAP_LIST_ID, 1, &id);
		if (id == 0xff)
			break;
		if (id == cap)
			return pos;
		pos += PCI_CAP_LIST_NEXT;
	}
	return 0;
}

static int pci_find_own_next_cap(struct pcie_port *pp, u32 pos, int cap)
{
	int ttl = PCI_FIND_CAP_TTL;

	return pci_find_own_next_cap_ttl(pp, pos, cap, &ttl);
}

static int pci_find_own_cap_start(struct pcie_port *pp, u8 hdr_type)
{
	u32 status;

	pp->ops.rd_own(pp, PCI_STATUS, 2, &status);
	if (!(status & PCI_STATUS_CAP_LIST))
		return 0;

	switch (hdr_type) {
	case PCI_HEADER_TYPE_NORMAL:
	case PCI_HEADER_TYPE_BRIDGE:
		return PCI_CAPABILITY_LIST;
	case PCI_HEADER_TYPE_CARDBUS:
		return PCI_CB_CAPABILITY_LIST;
	default:
		return 0;
	}

	return 0;
}

/*
 * Tell if a device supports a given PCI capability.
 * Returns the address of the requested capability structure within the
 * device's PCI configuration space or 0 in case the device does not
 * support it. Possible values for @cap:
 *
 * %PCI_CAP_ID_PM	Power Management
 * %PCI_CAP_ID_AGP	Accelerated Graphics Port
 * %PCI_CAP_ID_VPD	Vital Product Data
 * %PCI_CAP_ID_SLOTID	Slot Identification
 * %PCI_CAP_ID_MSI	Message Signalled Interrupts
 * %PCI_CAP_ID_CHSWP	CompactPCI HotSwap
 * %PCI_CAP_ID_PCIX	PCI-X
 * %PCI_CAP_ID_EXP	PCI Express
 */
int pci_find_own_capability(struct pcie_port *pp, int cap)
{
	u32 pos, hdr_type;

	pp->ops.rd_own(pp, PCI_HEADER_TYPE, 1, &hdr_type);

	pos = pci_find_own_cap_start(pp, hdr_type);
	if (pos)
		pos = pci_find_own_next_cap(pp, pos, cap);

	return pos;
}

struct pcie_port *portno_to_port(int port)
{
	struct pcie_port *pp;

	list_for_each_entry(pp, &pcie_port_list, next) {
		if (pp->port == port)
			return pp;
	}
	return NULL;
}

static struct pcie_port *controller_to_port(int controller)
{
	struct pcie_port *pp;

	if (controller >= pci.nr_controllers)
		return NULL;

	list_for_each_entry(pp, &pcie_port_list, next) {
		if (pp->controller == controller)
			return pp;
	}
	return NULL;
}

static struct pcie_port *bus_to_port(int bus)
{
	int i;
	int rbus;
	struct pcie_port *pp;

	for (i = pci.nr_controllers - 1 ; i >= 0; i--) {
		pp = controller_to_port(i);
		rbus = pp->root_bus_nr;
		if (rbus != -1 && rbus <= bus)
			break;
	}

	return i >= 0 ? pp : NULL;
}

#ifdef CONFIG_PCI_MSI
static DECLARE_BITMAP(msi_irq_in_use[MAX_PCIE_PORT_SUPPORTED],
		NUM_MSI_IRQS);
unsigned int msi_data[MAX_PCIE_PORT_SUPPORTED];

/* MSI int handler */
void handle_msi(struct pcie_port *pp)
{
	unsigned long val;
	int i, pos;

	for (i = 0; i < 8; i++) {
		pp->ops.rd_own(pp, PCIE_MSI_INTR0_STATUS + i * 12, 4,
				(u32 *)&val);
		if (val) {
			pos = 0;
			while ((pos = find_next_bit(&val, 32, pos)) != 32) {
				generic_handle_irq(MSI0_INT_BASE
					+ pp->port * NUM_MSI_IRQS
					+ (i * 32) + pos);
				pos++;
			}
		}
		pp->ops.wr_own(pp, PCIE_MSI_INTR0_STATUS + i * 12, 4, val);
	}
}

static int find_valid_pos0(int port, int nvec, int pos, int *pos0)
{
	int flag = 1;
	do {
		pos = find_next_zero_bit(msi_irq_in_use[port],
				NUM_MSI_IRQS, pos);
		/*if you have reached to the end then get out from here.*/
		if (pos == NUM_MSI_IRQS)
			return -ENOSPC;
		/*
		 * Check if this position is at correct offset.nvec is always a
		 * power of two. pos0 must be nvec bit alligned.
		 */
		if (pos % nvec)
			pos += nvec - (pos % nvec);
		else
			flag = 0;
	} while (flag);

	*pos0 = pos;
	return 0;
}

static void msi_nop(struct irq_data *data)
{
	return;
}

static struct irq_chip msi_chip = {
	.name = "PCI-MSI",
	.irq_ack = msi_nop,
	.irq_enable = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask = mask_msi_irq,
	.irq_unmask = unmask_msi_irq,
};

/* Dynamic irq allocate and deallocation */
static int get_irq(int nvec, struct msi_desc *desc, int *pos)
{
	int res, bit, irq, pos0, pos1, i;
	u32 val;
	struct pcie_port *pp = bus_to_port(desc->dev->bus->number);

	if (!pp) {
		BUG();
		return -EINVAL;
	}

	pos0 = find_first_zero_bit(msi_irq_in_use[pp->port],
			NUM_MSI_IRQS);
	if (pos0 % nvec) {
		if (find_valid_pos0(pp->port, nvec, pos0, &pos0))
			goto no_valid_irq;
	}
	if (nvec > 1) {
		pos1 = find_next_bit(msi_irq_in_use[pp->port],
				NUM_MSI_IRQS, pos0);
		/* there must be nvec number of consecutive free bits */
		while ((pos1 - pos0) < nvec) {
			if (find_valid_pos0(pp->port, nvec, pos1, &pos0))
				goto no_valid_irq;
			pos1 = find_next_bit(msi_irq_in_use[pp->port],
					NUM_MSI_IRQS, pos0);
		}
	}

	irq = (MSI0_INT_BASE + (pp->port * NUM_MSI_IRQS)) + pos0;

	if ((irq + nvec) > (MSI0_INT_END
				+ (pp->port * NUM_MSI_IRQS)))
		goto no_valid_irq;

	i = 0;
	while (i < nvec) {
		set_bit(pos0 + i, msi_irq_in_use[pp->port]);
		dynamic_irq_init(irq + i);
		irq_set_msi_desc(irq + i, desc);
		irq_set_chip_and_handler(irq + i, &msi_chip,
				handle_simple_irq);
		set_irq_flags(irq + i, IRQF_VALID);

		/*
		 * Enable corresponding interrupt on MSI interrupt
		 * controller.
		 */
		res = ((pos0 + i) / 32) * 12;
		bit = (pos0 + i) % 32;
		pp->ops.rd_own(pp, PCIE_MSI_INTR0_ENABLE + res, 4, &val);
		val |= 1 << bit;
		pp->ops.wr_own(pp, PCIE_MSI_INTR0_ENABLE + res, 4, val);
		i++;
	}

	*pos = pos0;
	return irq;
no_valid_irq:
	*pos = pos0;
	return -ENOSPC;
}

static void clean_irq(unsigned int irq)
{
	int res, bit, val, pos;
	struct irq_desc *desc = irq_to_desc(irq);
	struct msi_desc *msi_desc = irq_desc_get_msi_desc(desc);
	struct pcie_port *pp = bus_to_port(msi_desc->dev->bus->number);

	if (!pp) {
		BUG();
		return;
	}

	pos = irq - (MSI0_INT_BASE + (pp->port * NUM_MSI_IRQS));

	dynamic_irq_cleanup(irq);

	clear_bit(pos, msi_irq_in_use[pp->port]);

	/*
	 * Disable corresponding interrupt on MSI interrupt
	 * controller.
	 */
	res = (pos / 32) * 12;
	bit = pos % 32;
	pp->ops.rd_own(pp, PCIE_MSI_INTR0_ENABLE + res, 4, &val);
	val &= ~(1 << bit);
	pp->ops.wr_own(pp, PCIE_MSI_INTR0_ENABLE + res, 4, val);
}

int arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	int cvec, rvec, irq, pos;
	struct msi_msg msg;
	uint16_t control;
	struct pcie_port *pp = bus_to_port(pdev->bus->number);

	if (!pp) {
		BUG();
		return -EINVAL;
	}

	/*
	 * Read the MSI config to figure out how many IRQs this device
	 * wants.Most devices only want 1, which will give
	 * configured_private_bits and request_private_bits equal 0.
	 */
	pci_read_config_word(pdev, desc->msi_attrib.pos + PCI_MSI_FLAGS,
			&control);

	/*
	 * If the number of private bits has been configured then use
	 * that value instead of the requested number. This gives the
	 * driver the chance to override the number of interrupts
	 * before calling pci_enable_msi().
	 */

	cvec = (control & PCI_MSI_FLAGS_QSIZE) >> 4;

	if (cvec == 0) {
		/* Nothing is configured, so use the hardware requested size */
		rvec = (control & PCI_MSI_FLAGS_QMASK) >> 1;
	} else {
		/*
		 * Use the number of configured bits, assuming the
		 * driver wanted to override the hardware request
		 * value.
		 */
		rvec = cvec;
	}

	/*
	 * The PCI 2.3 spec mandates that there are at most 32
	 * interrupts. If this device asks for more, only give it one.
	 */
	if (rvec > 5)
		rvec = 0;

	irq = get_irq((1 << rvec), desc, &pos);

	 if (irq < 0)
		return irq;

	 /* Update the number of IRQs the device has available to it */
	 control &= ~PCI_MSI_FLAGS_QSIZE;
	 control |= rvec << 4;
	 pci_write_config_word(pdev, desc->msi_attrib.pos + PCI_MSI_FLAGS,
			 control);
	 desc->msi_attrib.multiple = rvec;

	/*
	 * An EP will modify lower 8 bits(max) of msi data while
	 * sending any msi interrupt
	 */
	msg.address_hi = 0x0;
	msg.address_lo = __virt_to_phys((u32)(&msi_data[pp->port]));
	msg.data = pos;
	write_msi_msg(irq, &msg);

	return 0;
}

void arch_teardown_msi_irq(unsigned int irq)
{
	clean_irq(irq);
}

#endif

static void __init pcie_preinit(void)
{
	int i;
	struct pcie_port *pp;

	for (i = 0; i < pci.nr_controllers; i++) {
		pp = controller_to_port(i);
		if (!pp) {
			BUG();
			return ;
		}
		snprintf(pp->mem_space_name, sizeof(pp->mem_space_name),
			"PCIe %d MEM", pp->port);
		pp->mem_space_name[sizeof(pp->mem_space_name) - 1] = 0;
		pp->res[0].name = pp->mem_space_name;
		pp->res[0].start = (resource_size_t)pp->mem_base;
		pp->res[0].end = (resource_size_t)pp->mem_base
			+ (pp->config.mem_size - 1);
		pp->res[0].flags = IORESOURCE_MEM;

		snprintf(pp->io_space_name, sizeof(pp->io_space_name),
			"PCIe %d I/O", pp->port);
		pp->io_space_name[sizeof(pp->io_space_name) - 1] = 0;
		pp->res[1].name = pp->io_space_name;
		pp->res[1].start =
			__phys_to_pfn((u32)pp->io_base);
		pp->res[1].end = pp->res[1].start + (pp->config.io_size - 1);
		pp->res[1].flags = IORESOURCE_IO;

		if (request_resource(&iomem_resource, &pp->res[0]))
			panic("can't allocate PCIe Mem space");
		if (request_resource(&ioport_resource, &pp->res[1]))
			panic("can't allocate PCIe IO space");
	}
}

static int pcie_get_payload(struct pci_dev *dev)
{
	int ret, cap;
	u16 ctl;

	cap = pci_find_capability(dev, PCI_CAP_ID_EXP);
	if (!cap)
		return -EINVAL;

	ret = pci_read_config_word(dev, cap + PCI_EXP_DEVCTL, &ctl);
	if (!ret)
		ret = 128 << ((ctl & PCI_EXP_DEVCTL_PAYLOAD) >> 5);

	return ret;
}

static int pcie_set_payload(struct pci_dev *dev, int rq)
{
	int cap, err = -EINVAL;
	u16 ctl, v;

	if (rq < 128 || rq > 4096 || !is_power_of_2(rq))
		goto out;

	v = (ffs(rq) - 8) << 5;

	cap = pci_find_capability(dev, PCI_CAP_ID_EXP);
	if (!cap)
		goto out;

	err = pci_read_config_word(dev, cap + PCI_EXP_DEVCTL, &ctl);
	if (err)
		goto out;

	if ((ctl & PCI_EXP_DEVCTL_PAYLOAD) != v) {
		ctl &= ~PCI_EXP_DEVCTL_PAYLOAD;
		ctl |= v;
		err = pci_write_config_dword(dev, cap + PCI_EXP_DEVCTL, ctl);
	}

out:
	return err;
}

static void set_readrq(struct pci_bus *bus, int rq)
{
	struct pci_dev *dev;

	list_for_each_entry(dev, &bus->devices, bus_list) {
		if (rq < pcie_get_readrq(dev))
			pcie_set_readrq(dev, rq);
		if (dev->subordinate)
			set_readrq(dev->subordinate, rq);
	}
}

static int get_max_payload(struct pci_bus *bus, int rq)
{
	struct pci_dev *dev;
	int payload;
	int max_payload = rq;

	list_for_each_entry(dev, &bus->devices, bus_list) {
		payload = pcie_get_payload(dev);
		if (payload < max_payload)
			max_payload = payload;
		if (dev->subordinate)
			max_payload = get_max_payload(dev->subordinate,
					max_payload);
	}
	return max_payload;
}

static void set_payload(struct pci_bus *bus, int rq)
{
	struct pci_dev *dev;

	list_for_each_entry(dev, &bus->devices, bus_list) {
		pcie_set_payload(dev, rq);
		if (dev->subordinate)
			set_payload(dev->subordinate, rq);
	}
}

static void __init pcie_postinit(void)
{
	struct hw_pci *hw = &pci;
	struct pci_sys_data *sys;
	struct pci_bus *bus;
	int cap, ctl, payload, readrq;
	int max_payload = 4096;
	struct pcie_port *pp;

	/*
	 * allign Max_Payload_Size for all devices to the minimum
	 * Max_Payload_Size of any of the device in tree.
	 * Max_Read_Request_Size of any of the DS device should be less
	 * than or equal to that of RC's Max_Read_Request_Size.
	 */

	list_for_each_entry(sys, &hw->buses, node) {
		bus = sys->bus;
		pp = bus_to_port(bus->number);
		if (!pp) {
			BUG();
			return;
		}
		cap = pci_find_own_capability(pp, PCI_CAP_ID_EXP);
		pp->ops.rd_own(pp, cap + PCI_EXP_DEVCTL, 2, &ctl);
		payload = 128 << ((ctl & PCI_EXP_DEVCTL_PAYLOAD) >> 5);
		if (payload < max_payload)
			max_payload = payload;
		readrq = 128 << ((ctl & PCI_EXP_DEVCTL_READRQ) >> 12);
		max_payload = get_max_payload(bus, max_payload);
		set_payload(bus, max_payload);
		set_readrq(bus, readrq);
	}
}

static int __init pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct pcie_port *pp;

	pp = controller_to_port(nr);

	if (!pp)
		return 0;

	pp->root_bus_nr = sys->busnr;

	pci_add_resource(&sys->resources, &pp->res[0]);
	pci_add_resource(&sys->resources, &pp->res[1]);

	return 1;
}

static int pcie_valid_config(struct pcie_port *pp, struct pci_bus *bus, int dev)
{
	/*If there is no link, then there is no device*/
	if (bus->number != pp->root_bus_nr) {
		if (!pp->ops.link_up(pp->va_app_base))
			return 0;
	}
	/*
	 * Don't go out when trying to access nonexisting devices
	 * on the local bus.
	 * we have only one slot on each root port.
	 */
	if (bus->number == pp->root_bus_nr && dev > 0)
		return 0;

	/*
	 * do not read more than one device on the bus directly attached
	 * to RC's (Virtual Bridge's) DS side.
	 */
	if (bus->primary == pp->root_bus_nr && dev > 0)
		return 0;

	return 1;
}

static int pcie_rd_conf(struct pci_bus *bus, u32 devfn, int where,
			int size, u32 *val)
{
	struct pcie_port *pp = bus_to_port(bus->number);
	unsigned long flags;
	int ret;

	if (!pp) {
		BUG();
		return -EINVAL;
	}

	if (pcie_valid_config(pp, bus, PCI_SLOT(devfn)) == 0) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	spin_lock_irqsave(&pp->conf_lock, flags);
	if (bus->number != pp->root_bus_nr)
		ret = pp->ops.rd_other(pp, bus, devfn, where, size, val);
	else {
		pp->ops.rd_own(pp, where, size, val);
		ret = 0;
	}
	spin_unlock_irqrestore(&pp->conf_lock, flags);

	return ret;
}

static int pcie_wr_conf(struct pci_bus *bus, u32 devfn,
			int where, int size, u32 val)
{
	struct pcie_port *pp = bus_to_port(bus->number);
	unsigned long flags;
	int ret;

	if (!pp) {
		BUG();
		return -EINVAL;
	}

	if (pcie_valid_config(pp, bus, PCI_SLOT(devfn)) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	spin_lock_irqsave(&pp->conf_lock, flags);
	if (bus->number != pp->root_bus_nr)
		ret = pp->ops.wr_other(pp, bus, devfn, where, size, val);
	else {
		pp->ops.wr_own(pp, where, size, val);
		ret = 0;
	}
	spin_unlock_irqrestore(&pp->conf_lock, flags);

	return ret;
}

static struct pci_ops pcie_ops = {
	.read = pcie_rd_conf,
	.write = pcie_wr_conf,
};

static struct pci_bus __init *
pcie_scan_bus(int nr, struct pci_sys_data *sys)
{
	struct pci_bus *bus;
	struct pcie_port *pp = controller_to_port(nr);

	if (pp) {
		bus = pci_scan_root_bus(NULL, sys->busnr, &pcie_ops, sys,
					&sys->resources);
	} else {
		bus = NULL;
		BUG();
	}

	return bus;
}

static int __init pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	struct pcie_port *pp = bus_to_port(dev->bus->number);
	int irq;

	if (!pp) {
		BUG();
		return -EINVAL;
	}

	irq = (INTX0_BASE + pp->port * NUM_INTX_IRQS + pin - 1);

	return irq;
}

static struct hw_pci pci = {
	.preinit	= pcie_preinit,
	.postinit	= pcie_postinit,
	.swizzle	= pci_std_swizzle,
	.setup		= pcie_setup,
	.scan		= pcie_scan_bus,
	.map_irq	= pcie_map_irq,
};

#ifdef CONFIG_PM
int spear_pcie_suspend(void)
{
	struct pcie_port *pp;
	int i;

	for (i = 0; i < pci.nr_controllers; i++) {
		pp = controller_to_port(i);
		if (pp->ops.link_up(pp->va_app_base)) {
			pp->ops.host_exit(pp);
			pp->ops.clk_exit(pp);
			pp->susp_state = 1;
		}
	}

	return 0;
}

int spear_pcie_resume(void)
{
	struct pcie_port *pp;
	int i;

	for (i = 0; i < pci.nr_controllers; i++) {
		pp = controller_to_port(i);
		if (pp->susp_state) {
			pp->ops.clk_init(pp);
			pp->susp_state = 0;
			if (!pp->ops.link_up(pp->va_app_base))
				pp->ops.host_init(pp);
		}
	}

	return 0;
}
#endif

static int __init pcie_probe(struct platform_device *pdev)
{
	int err;
	struct clk *clk;
	struct pcie_port *pp;

	if (!pdev->dev.platform_data)
		return -EINVAL;

	if (!((struct pcie_port_info *)pdev->dev.platform_data)->is_host)
		return -EINVAL;

	pp = kzalloc(sizeof(*pp), GFP_KERNEL);
	if (!pp) {
		dev_err(&pdev->dev, "no memory for pcie port\n");
		return -ENOMEM;
	}

	memcpy(&pp->config, pdev->dev.platform_data, (sizeof(pp->config)));

	switch (pp->config.vendor) {
#ifdef CONFIG_SPEAR_PCIE_REV341
	case 341:
		spear_pcie_341_add_ops(pp);
		break;
#endif
#ifdef CONFIG_SPEAR_PCIE_REV370
	case 370:
		spear_pcie_370_add_ops(pp);
		break;
#endif
	default:
		dev_err(&pdev->dev, "ops not defined for this vendor\n");
		err = -EINVAL;
		goto free_mem;
	}

	if (pp->ops.clk_init(pp)) {
		err = -EINVAL;
		goto free_mem;
	}

	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "couldn't get clk for pcie\n");
		err = PTR_ERR(clk);;
		goto free_mem;
	}

	if (clk_enable(clk)) {
		dev_err(&pdev->dev, "couldn't enable clk for pcie\n");
		err = -EINVAL;
		goto clk_put;
	}

	if (!pp->ops.add_port(pp, pdev)) {
		pp->controller = pci.nr_controllers;
		pci.nr_controllers++;
		list_add_tail(&pp->next, &pcie_port_list);
		return 0;
	}
clk_put:
	clk_put(clk);
free_mem:
	kfree(pp);
	return err;
}

static struct platform_driver pcie_driver = {
	.probe = pcie_probe,
	.driver = {
		.name	= "dw_pcie",
	},
};

static int __init pcie_init(void)
{
	INIT_LIST_HEAD(&pcie_port_list);
	platform_driver_register(&pcie_driver);
	if (pci.nr_controllers) {
		pci_common_init(&pci);
		pci_assign_unassigned_resources();
		pr_info("pcie init successful\n");
	}

	return 0;
}
subsys_initcall(pcie_init);

static void __exit pcie_exit(void)
{
	platform_driver_unregister(&pcie_driver);
}
module_exit(pcie_exit);

