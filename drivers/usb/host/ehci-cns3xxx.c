
#include <linux/platform_device.h>
#include <mach/cns3xxx.h>

static int cns3xxx_ehci_init(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval = 0;

	printk(KERN_INFO "%s: !!WARNING!!"
		" to verify the following ehci->caps ehci->regs\n",
		__func__);
	cns3xxx_pwr_power_up(1<<PM_PLL_HM_PD_CTRL_REG_OFFSET_PLL_USB);
	cns3xxx_pwr_clk_en(1<<PM_CLK_GATE_REG_OFFSET_USB_HOST);
	cns3xxx_pwr_soft_rst(1<<PM_SOFT_RST_REG_OFFST_USB_HOST);

	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs
		+ HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	hcd->has_tt = 0;
	ehci_reset(ehci);

	retval = ehci_init(hcd);
	if (retval)
		return retval;

	ehci_writel(ehci, 0x00600060, hcd->regs + 0x94);
	printk(KERN_INFO "%s,***Threshold OUT=0x60,IN=0x60 ***\n", __func__);
	writel((readl((CNS3XXX_MISC_BASE_VIRT+0x04)) | (0X2<<24)),
		(CNS3XXX_MISC_BASE_VIRT+0x04));

	ehci_port_power(ehci, 0);

	return retval;
}

static const struct hc_driver cns3xxx_ehci_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "CNS3XXX EHCI Host Controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),
	.irq			= ehci_irq,
	.flags			= HCD_MEMORY | HCD_USB2,
	.reset			= cns3xxx_ehci_init,
	.start			= ehci_run,
	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset		= ehci_endpoint_reset,
	.get_frame_number	= ehci_get_frame,
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
#if defined(CONFIG_PM)
	.bus_suspend		= ehci_bus_suspend,
	.bus_resume		= ehci_bus_resume,
#endif
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,

	.clear_tt_buffer_complete	= ehci_clear_tt_buffer_complete,
};

static int cns3xxx_ehci_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	const struct hc_driver *driver = &cns3xxx_ehci_hc_driver;
	struct resource *res;
	int irq;
	int retval;

	if (usb_disabled())
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}
	irq = res->start;

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto fail_create_hcd;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			dev_name(&pdev->dev));
		retval = -ENODEV;
		goto fail_request_resource;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = res->end - res->start + 1;

	hcd->regs = (void __iomem *) CNS3XXX_USB_BASE_VIRT;

	retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (retval)
		goto fail_add_hcd;

	return retval;

fail_request_resource:
fail_add_hcd:
	usb_put_hcd(hcd);
fail_create_hcd:
	dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), retval);
	return retval;
}

static int cns3xxx_ehci_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

	return 0;
}

MODULE_ALIAS("platform:cns3xxx-ehci");

static struct platform_driver cns3xxx_ehci_driver = {
	.probe = cns3xxx_ehci_probe,
	.remove = cns3xxx_ehci_remove,
	.driver = {
		.name = "cns3xxx-ehci",
	},
};
