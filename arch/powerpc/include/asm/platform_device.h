#ifndef __ASM_PLATFORM_DEVICE_H_
#define __ASM_PLATFORM_DEVICE_H_

#include <linux/platform_device.h>
#include <asm/dma-mapping.h>

#define ARCH_HAS_PDEV_ARCHDATA_SETUP

static inline void arch_setup_pdev_archdata(struct platform_device *pdev)
{
	pdev->dev.dma_mask = &pdev->archdata.dma_mask;
	set_dma_ops(&pdev->dev, &dma_direct_ops);

	return;
}

#endif /* __ASM_GENERIC_PLATFORM_DEVICE_H_ */
