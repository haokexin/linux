 /*
  * drivers/usb/host/ehci-ci13612.c
  *
  * USB Host Controller Driver for LSI's ACP
  *
  * Copyright (C) 2010 LSI Inc.
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
  * You should have received a copy of the GNU General Public License
  * along with this program; if not, write to the Free Software
  * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
  *
  */

#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/of_platform.h>
#include "ehci-ci13612.h"


/* Patch the code to fix the bugs in Bugzilla */
static void ci13612_usb_setup(struct usb_hcd *hcd)
{
	int USB_TXFIFOTHRES, VUSB_HS_TX_BURST;

	/* fix Bugzilla #31874 */
	/* fix Bugzilla #32212 */
	VUSB_HS_TX_BURST = inl(USB_HWTXBUF) & 0x0f;
	USB_TXFIFOTHRES = (inl(USB_TXFILLTUNING) & 0x3f0000) >> 16;

	printk(KERN_INFO "ehci-ci13612 (ci13612_usb_setup): "
			 "VUSB_HS_TX_BURST = 0x%x, USB_TXFIFOTHRES = 0x%x\n",
			 VUSB_HS_TX_BURST, USB_TXFIFOTHRES);

	return;
}

/* called after powerup, by probe or system-pm "wakeup" */
static int ehci_ci13612_reinit(struct ehci_hcd *ehci)
{
	ci13612_usb_setup(ehci_to_hcd(ehci));
	ehci_port_power(ehci, 0);

	return 0;
}


static int ci13612_ehci_init(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval = 0;
	int len;


	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + 0x100;
	ehci->regs = hcd->regs + 0x100
		+ HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase));
	len = HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase));

	/* configure other settings */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);
	hcd->has_tt = 1;

	ehci->sbrn = 0x20;

	/* reset and halt controller */
	ehci_reset(ehci);

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;
	hcd->self.sg_tablesize = 0;

	retval = ehci_ci13612_reinit(ehci);

	return retval;
}

static int ehci_run_fix(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	u32 port_status;
	unsigned burst_size;
	int retval;

	/* fix Bugzilla 33669 */
	port_status = ehci_readl(ehci, &ehci->regs->port_status[0]);
	printk(KERN_INFO "ehci_run: port_status = 0x%x\n", port_status);
	if (port_status & 0x100) {
		printk(KERN_ERR "USB port is in reset status, not able to "
				"change host controller status to run\n");
		return -EFAULT;
	}

	retval = ehci_run(hcd);
	if (retval)
		return retval;

	burst_size = ehci_readl(ehci, &ehci->regs->reserved[1]);
	burst_size = (burst_size & 0xffff00ff) | 0x4000;	/* TXPBURST */
	ehci_writel(ehci, burst_size, &ehci->regs->reserved[1]);

	return 0;
}

static const struct hc_driver ci13612_hc_driver = {
	.description		= "ci13612_hcd",
	.product_desc		= "CI13612A EHCI USB Host Controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),
	.irq			= ehci_irq,
	.flags			= HCD_MEMORY | HCD_USB2 | HCD_LOCAL_MEM,
	.reset			= ci13612_ehci_init,
	.start			= ehci_run_fix,
	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.get_frame_number	= ehci_get_frame,
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
#if defined(CONFIG_PM)
	.bus_suspend		= ehci_bus_suspend,
	.bus_resume		= ehci_bus_resume,
#endif
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,
};

static int ci13612_ehci_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	const struct hc_driver *driver = &ci13612_hc_driver;
	void __iomem *USB_base = (void __iomem *) 0xF00A0000;
	void __iomem *gpreg_base = (void __iomem *) 0xF000C000;
	int irq;
	int retval;
	struct device_node *np = pdev->dev.of_node;
	const int *enabled;


	enabled = of_get_property(np, "enabled", NULL);
	if (!enabled || !*enabled)
		return -ENODEV;

	if (usb_disabled())
		return -ENODEV;

	/* Map the irq in the PPC476 to get the irq number */
	irq = irq_create_mapping(NULL, 31);

	if (NO_IRQ == irq) {
		dev_dbg(&pdev->dev, "error mapping irq number\n");
		retval = -EBUSY;
		goto fail_create_hcd;
	}

	if (0 != irq_set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH)) {
		dev_dbg(&pdev->dev, "set_irq_type() failed\n");
		retval = -EBUSY;
		goto fail_create_hcd;
	}

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto fail_create_hcd;
	}

	hcd->rsrc_start = ci13612_PHY_ADDR;
	hcd->rsrc_len = 0x20000;

	hcd->regs = USB_base;

	/* Setup GPREG for USB to enable the 6-bit address line */
	writel(0x0, gpreg_base + 0x8);

	retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (retval == 0) {
		platform_set_drvdata(pdev, hcd);
		return retval;
	}

	usb_put_hcd(hcd);
fail_create_hcd:
	dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), retval);
	return retval;
}

static int ci13612_ehci_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

MODULE_ALIAS("platform:ci13612-ehci");

static struct of_device_id ci13612_match[] = {
	{
		.type	= "usb",
		.compatible = "acp-usb",
	},
	{},
};

static struct platform_driver ci13612_ehci_driver = {
	.probe = ci13612_ehci_probe,
	.remove = ci13612_ehci_remove,
	.driver = {
		.name = "ci13612-ehci",
		.of_match_table = ci13612_match,
	},
};
