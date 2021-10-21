// SPDX-License-Identifier: GPL-2.0+
#include "spi-cadence-xspi.h"
#include <linux/pci.h>
#include <linux/of.h>

#define DRV_NAME       "spi-cadence-octeon"
#define STIG_OFFSET    (0x10000000)
#define AUX_OFFSET     (0x2000)
#define MAX_CS_COUNT   (4)
#define PCI_CDNS_XSPI             0xA09B

#define CDNS_XSPI_CLOCK_IO_HZ			  800000000
#define CDNS_XSPI_CLOCK_DIVIDED(div)      ((CDNS_XSPI_CLOCK_IO_HZ) / (div))
#define CDNS_XSPI_CLK_CTRL_REG		      0x4020
#define CDNS_XSPI_CLK_ENABLE              BIT(0)
#define CDNS_XSPI_CLK_DIV                 GENMASK(4, 1)

#define ASIM_PLATFORM_NAME "ASIM_PLATFORM"

const int cdns_xspi_clk_div_list[] = {
	4,	//0x0 = Divide by 4.   SPI clock is 200 MHz.
	6,	//0x1 = Divide by 6.   SPI clock is 133.33 MHz.
	8,	//0x2 = Divide by 8.   SPI clock is 100 MHz.
	10,	//0x3 = Divide by 10.  SPI clock is 80 MHz.
	12,	//0x4 = Divide by 12.  SPI clock is 66.666 MHz.
	16,	//0x5 = Divide by 16.  SPI clock is 50 MHz.
	18,	//0x6 = Divide by 18.  SPI clock is 44.44 MHz.
	20,	//0x7 = Divide by 20.  SPI clock is 40 MHz.
	24,	//0x8 = Divide by 24.  SPI clock is 33.33 MHz.
	32,	//0x9 = Divide by 32.  SPI clock is 25 MHz.
	40,	//0xA = Divide by 40.  SPI clock is 20 MHz.
	50,	//0xB = Divide by 50.  SPI clock is 16 MHz.
	64,	//0xC = Divide by 64.  SPI clock is 12.5 MHz.
	128,	//0xD = Divide by 128. SPI clock is 6.25 MHz.
	-1	//End of list
};

static bool mrvl_platform_is_asim(void)
{
	/* Check runmode dtb entry */
	struct device_node *np = of_find_node_by_name(NULL, "soc");
	const char *runplatform;
	int ret;

	ret = of_property_read_string(np, "runplatform", &runplatform);

	if (!ret) {
		if (!strncmp(runplatform, ASIM_PLATFORM_NAME, strlen(ASIM_PLATFORM_NAME))) {
			pr_debug("asim platform detected\n");
			return true;
		}
	}

	return false;
}

static int mrvl_setup_clock(struct cdns_xspi_dev *cdns_xspi, int req_clk)
{
	int i = 0;
	int clk_val;
	u32 clk_reg;
	bool update_clk;

	while (cdns_xspi_clk_div_list[i] > 0) {
		clk_val = CDNS_XSPI_CLOCK_DIVIDED(cdns_xspi_clk_div_list[i]);
		if (clk_val <= req_clk)
			break;
		i++;
	}

	if (cdns_xspi_clk_div_list[i] == -1) {
		pr_err("Unable to find clock divider for CLK: %d - setting 6.25MHz\n",
		       req_clk);
		i = 0x0D;
	} else {
		pr_debug("Found clk div: %d, clk val: %d\n", cdns_xspi_clk_div_list[i],
			  CDNS_XSPI_CLOCK_DIVIDED(cdns_xspi_clk_div_list[i]));
	}

	clk_reg = readl(cdns_xspi->iobase + CDNS_XSPI_CLK_CTRL_REG);

	if (FIELD_GET(CDNS_XSPI_CLK_DIV, clk_reg) != i) {
		clk_reg = FIELD_PREP(CDNS_XSPI_CLK_DIV, i);
		clk_reg |= CDNS_XSPI_CLK_ENABLE;
		update_clk = true;
	}

	if (update_clk)
		writel(clk_reg, cdns_xspi->iobase + CDNS_XSPI_CLK_CTRL_REG);

	return update_clk;
}

static int cadence_octeon_spi_probe(struct pci_dev *pdev,
			       const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	void __iomem *register_base;
	int ret;

	struct spi_master *master = NULL;
	struct cdns_xspi_dev *cdns_xspi = NULL;
	struct cdns_xspi_platform_data *plat_data = NULL;

	master = cdns_xspi_prepare_master(dev);
	if (!master) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to allocate memory for spi_master\n");
		goto err_no_mem;
	}

	ret = pcim_enable_device(pdev);
	if (ret)
		return -ENODEV;

	ret = pci_request_regions(pdev, DRV_NAME);
	if (ret)
		goto error_disable;

	register_base = pcim_iomap(pdev, 0, pci_resource_len(pdev, 0));
	if (!register_base) {
		ret = -EINVAL;
		goto error_disable;
	}

	plat_data = devm_kzalloc(dev, sizeof(*plat_data), GFP_KERNEL);
	if (!plat_data) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to allocate memory for platform_data\n");
		goto error_disable;
	}
	cdns_xspi = spi_master_get_devdata(master);

	dev->platform_data = plat_data;

	cdns_xspi->plat_data = plat_data;
	cdns_xspi->dev = dev;
	cdns_xspi->iobase   = register_base;
	cdns_xspi->auxbase  = register_base + AUX_OFFSET;
	cdns_xspi->sdmabase = register_base + STIG_OFFSET;
	cdns_xspi->irq = 0;
	cdns_xspi->do_phy_init = mrvl_platform_is_asim();
	cdns_xspi->prepare_clock = mrvl_setup_clock;

	init_completion(&cdns_xspi->cmd_complete);
	init_completion(&cdns_xspi->sdma_complete);

	ret = cdns_xspi_of_get_plat_data(dev);
	if (ret)
		dev_err(dev, "Failed to parse xSPI params");

	ret = cdns_xspi_configure(cdns_xspi);
	if (ret) {
		dev_err(dev, "Failed to prepare xSPI");
		goto error_disable;
	}

	master->num_chipselect = 1 << cdns_xspi->hw_num_banks;

	ret = devm_spi_register_master(dev, master);
	if (ret)
		dev_err(dev, "Failed to register SPI master\n");

	return 0;

error_disable:
	spi_master_put(master);

err_no_mem:
	return ret;
}

static void cadence_octeon_spi_remove(struct pci_dev *pdev)
{
	pci_disable_device(pdev);
}

static const struct pci_device_id cadence_octeon_spi_pci_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM,
		     PCI_CDNS_XSPI) },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, cadence_octeon_spi_pci_id_table);

static struct pci_driver cadence_octeon_spi_driver = {
	.name		= DRV_NAME,
	.id_table	= cadence_octeon_spi_pci_id_table,
	.probe		= cadence_octeon_spi_probe,
	.remove		= cadence_octeon_spi_remove,
};

module_pci_driver(cadence_octeon_spi_driver);

MODULE_DESCRIPTION("xSPI PCI bus driver");
MODULE_AUTHOR("Marvell Inc.");
MODULE_LICENSE("GPL");
