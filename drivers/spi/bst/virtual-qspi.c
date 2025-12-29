// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 Black Sesame Technologies
 *
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/mtdram.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-direct.h>
#include "SocClient.h"
#include "virtual-qspi.h"

static unsigned long total_size = CONFIG_VIRTUAL_TOTAL_SIZE;
static unsigned long erase_size = CONFIG_VIRTUAL_ERASE_SIZE;
static unsigned long writebuf_size = 64;
#define MTDRAM_TOTAL_SIZE (total_size * 1024)
#define MTDRAM_ERASE_SIZE (erase_size * 1024)

module_param(total_size, ulong, 0);
MODULE_PARM_DESC(total_size, "Total device size in KiB");
module_param(erase_size, ulong, 0);
MODULE_PARM_DESC(erase_size, "Device erase block size in KiB");
module_param(writebuf_size, ulong, 0);
MODULE_PARM_DESC(writebuf_size, "Device write buf size in Bytes (Default: 64)");

int qspi_client_method(SocClient_t *client, struct nor_ipcmsg_ *msg_data)
{
	int32_t ret = 0;
	st_public_qspi_cmd_head_t fw_cmd_msg = { 0 };
	st_public_ErrorEnum_t st_err;

	if (!client) {
		pr_err("%s line:%d fls is not initial!\n", __func__, __LINE__);
		return -1;
	}

	if (!msg_data) {
		pr_err("%s line:%d msg_data is null !\n", __func__, __LINE__);
		return -1;
	}

	fw_cmd_msg.addr = msg_data->addr;
	fw_cmd_msg.maxsize = msg_data->maxsize;
	fw_cmd_msg.offset = msg_data->offset;
	fw_cmd_msg.size = msg_data->size;
	fw_cmd_msg.bus_num = msg_data->bus_num;
	fw_cmd_msg.flag = msg_data->flag;

	ret = client->st_public_client.qspi_method_sync(&fw_cmd_msg, &st_err,
							1000, NULL);
	if (ret < 0) {
		pr_err("%s line:%d send msg async method failed. ret is %d\n",
		       __func__, __LINE__, ret);
		return -1;
	}

	return 0;
}

// We could store these in the mtd structure, but we only support 1 device..
static int check_offs_len(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	int ret = 0;

	/* Start address must align on block boundary */
	if (mtd_mod_by_eb(ofs, mtd)) {
		pr_debug("%s: unaligned address\n", __func__);
		ret = -EINVAL;
	}

	/* Length must align on block boundary */
	if (mtd_mod_by_eb(len, mtd)) {
		pr_debug("%s: length not block aligned\n", __func__);
		ret = -EINVAL;
	}

	return ret;
}

static int ram_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct virtual_qspi_ *v_qspi_info = (struct virtual_qspi_ *)mtd->priv;
	struct nor_ipcmsg_ msg_data;
	int i, ret;

	if (check_offs_len(mtd, instr->addr, instr->len)) {
		pr_err("%s %d erase must align 4K\n", __func__, __LINE__);
		return -EINVAL;
	}

	for (i = 0; i < instr->len / ERASE_BLOCK_SIZE; i++) {
		msg_data.addr = v_qspi_info->safety_addr;
		msg_data.maxsize = 0xffffff;
		msg_data.offset = instr->addr + ERASE_BLOCK_SIZE * i;
		msg_data.size = ERASE_BLOCK_SIZE;
		msg_data.bus_num = 0;
		msg_data.flag = 2;

		ret = qspi_client_method(v_qspi_info->client, &msg_data);
		if (ret < 0) {
			pr_err("%s %d erase error!", __func__, __LINE__);
			return -EINVAL;
		}
	}

	return 0;
}

static int ram_point(struct mtd_info *mtd, loff_t from, size_t len,
		     size_t *retlen, void **virt, resource_size_t *phys)
{
	return 0;
}

static int ram_unpoint(struct mtd_info *mtd, loff_t from, size_t len)
{
	return 0;
}

static int ram_read(struct mtd_info *mtd, loff_t from, size_t len,
		    size_t *retlen, u_char *buf)
{
	struct virtual_qspi_ *v_qspi_info = (struct virtual_qspi_ *)mtd->priv;
	struct nor_ipcmsg_ msg_data;
	int i, size, ret;

	for (i = 0; i < len / MAX_BLOCK_SIZE + 1; i++) {
		size = MAX_BLOCK_SIZE;
		msg_data.addr = v_qspi_info->safety_addr;
		msg_data.maxsize = 0xffffff;
		msg_data.offset = from + i * MAX_BLOCK_SIZE;
		msg_data.size = size;
		msg_data.bus_num = 0;
		msg_data.flag = 0;

		ret = qspi_client_method(v_qspi_info->client, &msg_data);
		if (ret < 0)
			break;

		udelay(60);
		if (i == len / MAX_BLOCK_SIZE)
			memcpy(&buf[i * MAX_BLOCK_SIZE], v_qspi_info->map_addr,
			       len % MAX_BLOCK_SIZE);
		else
			memcpy(&buf[i * MAX_BLOCK_SIZE], v_qspi_info->map_addr,
			       MAX_BLOCK_SIZE);
	}

	*retlen = len;
	return 0;
}

static int ram_write(struct mtd_info *mtd, loff_t to, size_t len,
		     size_t *retlen, const u_char *buf)
{
	struct virtual_qspi_ *v_qspi_info = (struct virtual_qspi_ *)mtd->priv;
	struct nor_ipcmsg_ msg_data;
	int i, size, ret;
	u_char *sbuf;

	sbuf = kmalloc(MAX_BLOCK_SIZE, GFP_KERNEL);
	if (sbuf == NULL)
		return -ENOMEM;

	for (i = 0; i < len / MAX_BLOCK_SIZE + 1; i++) {
		if (i == len / MAX_BLOCK_SIZE) {
			memset(sbuf, 0xff, MAX_BLOCK_SIZE);
			memcpy(sbuf, &buf[i * MAX_BLOCK_SIZE],
			       len % MAX_BLOCK_SIZE);

		} else {
			memcpy(sbuf, &buf[i * MAX_BLOCK_SIZE], MAX_BLOCK_SIZE);
		}

		size = MAX_BLOCK_SIZE;

		memcpy(v_qspi_info->map_addr, sbuf, size);

		msg_data.addr = v_qspi_info->safety_addr;
		msg_data.maxsize = 0xffffff;
		msg_data.offset = to + i * MAX_BLOCK_SIZE;
		msg_data.size = size;
		msg_data.bus_num = 0;
		msg_data.flag = 1;

		ret = qspi_client_method(v_qspi_info->client, &msg_data);
		if (ret < 0)
			break;
	}

	kfree(sbuf);

	*retlen = len;
	return 0;
}

int mtdram_init_device(struct mtd_info *mtd, void *info, unsigned long size,
		       const char *name)
{
	memset(mtd, 0, sizeof(*mtd));

	/* Setup the MTD structure */
	mtd->name = name;
	mtd->type = MTD_NORFLASH;
	mtd->flags = MTD_CAP_NORFLASH;
	mtd->size = size;
	mtd->writesize = 1;
	mtd->writebufsize = writebuf_size;
	mtd->erasesize = MTDRAM_ERASE_SIZE;
	mtd->priv = info;

	mtd->owner = THIS_MODULE;
	mtd->_erase = ram_erase;
	mtd->_point = ram_point;
	mtd->_unpoint = ram_unpoint;
	mtd->_read = ram_read;
	mtd->_write = ram_write;

	if (mtd_device_register(mtd, NULL, 0))
		return -EIO;

	return 0;
}

static int virtual_qspi_driver_probe(struct platform_device *pdev)
{
	int err, ret;
	struct virtual_qspi_ *v_qspi_info;
	static SocClient_data_t soc_data;

	if (!total_size)
		return -EINVAL;

	v_qspi_info = kmalloc(sizeof(struct virtual_qspi_), GFP_KERNEL);
	if (v_qspi_info == NULL)
		return -EINVAL;

	/* Allocate some memory */
	v_qspi_info->mtd_info = kmalloc(sizeof(struct mtd_info), GFP_KERNEL);
	if (!v_qspi_info->mtd_info) {
		kfree(v_qspi_info);
		return -ENOMEM;
	}

	ret = of_reserved_mem_device_init_by_idx(&pdev->dev, pdev->dev.of_node,
						 0);
	if (ret) {
		kfree(v_qspi_info->mtd_info);
		kfree(v_qspi_info);
		return -1;
	}

	v_qspi_info->map_addr = dma_alloc_coherent(
		&pdev->dev, 0x10000, &v_qspi_info->partition_dma_addr,
		GFP_KERNEL);

	if (v_qspi_info->partition_dma_addr == 0x800130000) {
		soc_data.com_data.pid = CPU_4;
		soc_data.com_data.fid = F1;
		soc_data.com_data.sid = 1U;

		v_qspi_info->safety_addr = 0x80130000;

	} else if (v_qspi_info->partition_dma_addr == 0x800120000) {
		soc_data.com_data.pid = CPUMP2_0;
		soc_data.com_data.fid = F1;
		soc_data.com_data.sid = 1U;

		v_qspi_info->safety_addr = 0x80120000;

	} else {
		soc_data.com_data.pid = CPU_0;
		soc_data.com_data.fid = F1;
		soc_data.com_data.sid = 1U;

		v_qspi_info->safety_addr = 0x80140000;
	}

	err = mtdram_init_device(v_qspi_info->mtd_info, v_qspi_info,
				 MTDRAM_TOTAL_SIZE, "virtual device");
	if (err) {
		kfree(v_qspi_info->map_addr);
		kfree(v_qspi_info->mtd_info);
		kfree(v_qspi_info);

		v_qspi_info = NULL;
		return err;
	}

	v_qspi_info->client = SocClient_init(&soc_data);
	if (v_qspi_info->client == NULL) {
		kfree(v_qspi_info->map_addr);
		kfree(v_qspi_info->mtd_info);
		kfree(v_qspi_info);

		v_qspi_info = NULL;
		return err;
	}

	v_qspi_info->client->start();

	v_qspi_info->client->st_public_client.version();

	return err;
}

static const struct of_device_id virtual_qpsi_of_match[] = {
	{
		.compatible = "bst,virtual-qspi",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, virtual_qpsi_of_match);

static struct platform_driver virtual_qspi_driver = {
	.driver = {
		.name = "virtual_qspi",
		.of_match_table = of_match_ptr(virtual_qpsi_of_match),
	},
	.probe = virtual_qspi_driver_probe,
};

static int __init virtual_qpsi_init(void)
{
	platform_driver_register(&virtual_qspi_driver);
	return 0;
}

module_init(virtual_qpsi_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("VIRTUAL QSPI FROM MSGBOX");
