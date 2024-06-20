#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/usb/ulpi.h>

static int ulpi_generic_probe(struct platform_device *pdev)
{
	struct usb_phy *phy;
	int flags = 0;
	int ret;

	ret = of_property_read_u32(pdev->dev.of_node, "usb-ulpi-flags", &flags);
	if (ret) {
		dev_warn(&pdev->dev, "No usb-ulpi-flags property!\n");
	}

	phy = devm_otg_ulpi_create(&pdev->dev, &ulpi_viewport_access_ops, flags);
	if (!phy) {
		dev_err(&pdev->dev, "Failed to create ULPI OTG phy.\n");
		return -ENOMEM;
	}

	phy->dev = &pdev->dev;

	ret = usb_add_phy_dev(phy);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add ULPI OTG phy.\n");
		return ret;
	}

	platform_set_drvdata(pdev, phy);

	return 0;
}

static int ulpi_generic_remove(struct platform_device *pdev)
{
	struct usb_phy *phy = platform_get_drvdata(pdev);

	usb_remove_phy(phy);

	return 0;
}

static const struct of_device_id generic_ulpi_dt_ids[] = {
	{ .compatible = "usb-phy-ulpi-generic" },
	{ }
};

MODULE_DEVICE_TABLE(of, generic_ulpi_dt_ids);

static struct platform_driver ulpi_generic_driver = {
	.probe		= ulpi_generic_probe,
	.remove		= ulpi_generic_remove,
	.driver		= {
		.name	= "usb_phy_ulpi_generic",
		.of_match_table = generic_ulpi_dt_ids,
	},
};

static int __init ulpi_generic_init(void)
{
	return platform_driver_register(&ulpi_generic_driver);
}
subsys_initcall(ulpi_generic_init);

static void __exit ulpi_generic_exit(void)
{
	platform_driver_unregister(&ulpi_generic_driver);
}
module_exit(ulpi_generic_exit);

MODULE_ALIAS("platform:usb_phy_ulpi_generic");
MODULE_AUTHOR("Wind River Systems Inc");
MODULE_DESCRIPTION("Generic ULPI PHY Transceiver driver");
MODULE_LICENSE("GPL");
