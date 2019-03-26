// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation
/*
 * USB Host Controller Driver for INTEL Axxia's AXM
 */

#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include "ehci-ci13612.h"

static int ci13612_ehci_halt(struct ehci_hcd *ehci);

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

	/* Reset is only allowed on a stopped controller */
	ci13612_ehci_halt(ehci);

	/* reset controller */
	ehci_reset(ehci);

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;
	hcd->self.sg_tablesize = 0;

	return 0;
}

#define ci13612_fixup_usbcmd_rs(_ehci) (0)
#define ci13612_fixup_txpburst(ehci) { (void)ehci; }

static int ci13612_ehci_run(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;
	u32 tmp;

	retval = ci13612_fixup_usbcmd_rs(ehci);
	if (retval)
		return retval;

	/* Setup AMBA interface to force INCR16 busts when possible */
	writel(3, USB_SBUSCFG);

	retval = ehci_run(hcd);
	if (retval)
		return retval;

	ci13612_fixup_txpburst(ehci);

	/* Set ITC (bits [23:16]) to zero for interrupt on every micro-frame */
	tmp = ehci_readl(ehci, &ehci->regs->command);
	tmp &= 0xFFFF;
	ehci_writel(ehci, tmp & 0xFFFF, &ehci->regs->command);

	return retval;
}

static const struct hc_driver ci13612_hc_driver = {
	.description		= "ci13612_hcd",
	.product_desc		= "CI13612A EHCI USB Host Controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),
	.irq			= ehci_irq,
	.flags			= HCD_MEMORY | HCD_USB2 | HCD_BH,
	.reset			= ci13612_ehci_init,
	.start			= ci13612_ehci_run,
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
	struct device_node *np = pdev->dev.of_node;
	struct usb_hcd *hcd;
	void __iomem *gpreg_base;
	int irq;
	int retval;
	struct resource *res;

	if (usb_disabled())
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_dbg(&pdev->dev, "error getting irq number\n");
		retval = irq;
		goto fail_create_hcd;
	}

	if (irq_set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH) != 0) {
		dev_dbg(&pdev->dev, "set_irq_type() failed\n");
		retval = -EBUSY;
		goto fail_create_hcd;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Error: resource addr %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}

	/* Device using 32-bit addressing */
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	hcd = usb_create_hcd(&ci13612_hc_driver,
			     &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto fail_create_hcd;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	hcd->regs = of_iomap(np, 0);
	if (!hcd->regs) {
		dev_err(&pdev->dev, "of_iomap error\n");
		retval = -ENOMEM;
		goto fail_put_hcd;
	}

	gpreg_base = of_iomap(np, 1);
	if (!gpreg_base) {
		dev_warn(&pdev->dev, "of_iomap error can't map region 1\n");
		retval = -ENOMEM;
		goto fail_put_hcd;
	} else {
		/* Set address bits [39:32] to zero */
		writel(0x0, gpreg_base + 0x8);
		/* hprot cachable and bufferable */
		writel(0xc, gpreg_base + 0x74);
		iounmap(gpreg_base);
	}

	retval = usb_add_hcd(hcd, irq, 0);
	if (retval == 0) {
		platform_set_drvdata(pdev, hcd);
		return retval;
	}

fail_put_hcd:
	usb_put_hcd(hcd);
fail_create_hcd:
	dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), retval);
	return retval;
}

static int ci13612_ehci_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	usb_put_hcd(hcd);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int ci13612_ehci_halt(struct ehci_hcd *ehci)
{
	u32     temp;

	temp = ehci_readl(ehci, &ehci->regs->command);
	temp &= ~CMD_RUN;
	ehci_writel(ehci, temp, &ehci->regs->command);

	return ehci_handshake(ehci, &ehci->regs->status,
			      STS_HALT, STS_HALT, 16 * 125);
}

MODULE_ALIAS("platform:ci13612-ehci");

static const struct of_device_id ci13612_match[] = {
	{
		.type	= "usb",
		.compatible = "axxia,acp-usb",
	},
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
