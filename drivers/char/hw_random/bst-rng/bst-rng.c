// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Black Sesame Technologies, Inc.
 *
 * Author: Xuran Yang <xuran.yang@bst.ai>
 */
#define pr_fmt(fmt) "BST-RNG " fmt
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/hw_random.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_device.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <bst/bstipc_cfg.h>
#include "security_client.h"

struct bst_rng_data {
	u64 shared_mem_mask;
	u64 shared_mem_offset;
};

struct bst_rng {
	// enum ipc_msg_pid_e pid;
	uint32_t pid;
	struct _security_client_t *client;
	struct _security_client_data_t *ins;
	dma_addr_t mem_handle;
	void *mem_virt_addr;
	struct hwrng rng;
};

#define to_bst_rng(p)	container_of(p, struct bst_rng, rng)

static int bst_rng_init(struct hwrng *rng)
{
	struct bst_rng *dev_rng = to_bst_rng(rng);
	int ret;

	dev_rng->ins = kzalloc(sizeof(*dev_rng->ins), GFP_KERNEL);
	dev_rng->ins->com_data.pid = dev_rng->pid;
	dev_rng->client = security_client_init(dev_rng->ins);
	ret = dev_rng->client->start();
	if (ret) {
		pr_err("Failed to start security client\n");
		return ret;
	}

	return 0;
}

static int bst_rng_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	struct bst_rng *dev_rng = to_bst_rng(rng);
	service_ErrorEnum_t err = SERVICE_NO_ERROR;
	int num = (int)(max > 4096 ? 4096 : max);

	dev_rng->client->service_client.trng_sync(max, dev_rng->mem_handle, &err, 100, NULL);
	if (err != SERVICE_NO_ERROR) {
		pr_err("Failed to read RNG data\n");
		return 0;
	}
	memcpy(buf, dev_rng->mem_virt_addr, max);
	return num;
}

static int bst_rng_probe(struct platform_device *pdev)
{
	struct bst_rng *dev_rng;
	struct device *dev = &pdev->dev;
	int ret;
	struct bst_rng_data *data;

	data = (struct bst_rng_data *)of_device_get_match_data(dev);
	if (!data)
		return -EINVAL;

	dev_rng = devm_kzalloc(dev, sizeof(*dev_rng), GFP_KERNEL);
	if (!dev_rng)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev_rng);

	ret = of_property_read_u32(dev->of_node, "msgbx-pid", &dev_rng->pid);
	if (ret) {
		pr_err("Msgbox PID must be specified in DT\n");
		return -EINVAL;
	}

	ret = of_reserved_mem_device_init_by_idx(dev, dev->of_node, 0);
	if (ret) {
		pr_err("Failed to reserve memory\n");
		return -ENOMEM;
	}

	ret = dma_set_mask_and_coherent(dev, data->shared_mem_mask);
	if (ret) {
		pr_err("Failed to set DMA mask\n");
		return -EINVAL;
	}

	dev_rng->mem_virt_addr = dma_alloc_coherent(dev, PAGE_SIZE, &dev_rng->mem_handle, GFP_KERNEL | GFP_DMA);
	if (!dev_rng->mem_virt_addr)
		return -ENOMEM;

	dev_rng->mem_handle -= data->shared_mem_offset;
	dev_rng->rng.name = pdev->name;
	dev_rng->rng.init = bst_rng_init;
	dev_rng->rng.read = bst_rng_read;
	dev_rng->rng.quality = 900;

	return devm_hwrng_register(&pdev->dev, &dev_rng->rng);
}

static struct bst_rng_data bst_c1200_rng_data = {
	.shared_mem_mask = 0x8FFFFFFFFULL,
	.shared_mem_offset = 0x800000000ULL - 0x80000000ULL,
};

static const struct of_device_id bst_rng_ids[]  = {
	{ .compatible = "bst,c1200-rng",
	  .data = &bst_c1200_rng_data },
	{ }
};
MODULE_DEVICE_TABLE(of, bst_rng_ids);

static struct platform_driver bst_rng_driver = {
	.driver = {
		.name		= "bst,bst_rng",
		.of_match_table = of_match_ptr(bst_rng_ids),
	},
	.probe		= bst_rng_probe,
};
module_platform_driver(bst_rng_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xuran Yang <xuran.yang@bst.ai>");
MODULE_DESCRIPTION("BST SOC random number generator driver");
