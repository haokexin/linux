// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *
 * Copyright bst Corporation 2023
 *
 * Author: yongfeiliu <yongfei.liu@bst.ai>
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/atomic.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/unaligned.h>
#include <linux/input/touchscreen.h>
#include <linux/input/mt.h>
#include <linux/blkdev.h>
#include <linux/hdreg.h>
#include <linux/blk-mq.h>
#include <linux/bitops.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-direct.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include "bst/ipc_interface.h"

int safety_session_id;
struct dentry *debugfs;
static unsigned char *vbuf;

#define SET_SAFETY_TIMEOUT 1000
#define SAFETY_CMD_TO_R5    3
#define COMMAND_SIZE  256


//ipc_shell:pmic pd name


#ifdef CONFIG_BST_C1200_IVI
#define COMMAND_ADDR  0x80255000 
#endif

#ifdef CONFIG_BST_C1200_ADAS
#define COMMAND_ADDR  0x80254000 
#endif

#ifdef CONFIG_BST_C1200_DB
#define COMMAND_ADDR 0x80253000
#endif


ipc_msg safety_cmd = {
	.type = IPC_MSG_TYPE_METHOD,
	.cmd = SAFETY_CMD_TO_R5<<4,
	.data = COMMAND_ADDR,
};


struct cmd_msg_ {
	int len;
	char content[COMMAND_SIZE-4];
};

static ssize_t dw_send_command(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
	char command[COMMAND_SIZE];
	int ret;
	struct cmd_msg_ msg;

	

	if (count > sizeof(command))
		return -EINVAL;
	memset(command, 0, COMMAND_SIZE);

	ret = copy_from_user(command, user_buf, count);
	if (ret)
		return -EFAULT;
	msg.len = strlen(command) + strlen("ipc_shell ");
	snprintf(msg.content, COMMAND_SIZE-4, "ipc_shell %s", command);
	if (vbuf != NULL) {
		
		memcpy(vbuf, &msg, COMMAND_SIZE);
		
		ret = ipc_send(safety_session_id, &safety_cmd, SET_SAFETY_TIMEOUT);
		if (ret < 0) {
			ret = -EFAULT;
			
			if (vbuf != NULL) {
				snprintf(msg.content,COMMAND_SIZE-4,"%s","failed");
				msg.len = strlen("failed");
			}
			
			return ret;
		}
		
		udelay(30);
	}
	

	return count;
}


static ssize_t dw_recv_command(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	struct cmd_msg_ msg;

	// printk("%s %d\n",__func__,__LINE__);
	
	if (vbuf != NULL) {
		memcpy(&msg, vbuf, COMMAND_SIZE);
	}
	
	// printk("%s %d %s\n",__func__,__LINE__, msg.content);
	ret = simple_read_from_buffer(user_buf, count, ppos, msg.content, msg.len);
	
	
	udelay(30);
	
	memset(vbuf, 0, COMMAND_SIZE);
	
	return ret;
}


static const struct file_operations dw_command_ops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.read		= dw_recv_command,
	.write      = dw_send_command,
};


static int safety_command_debugfs_init(void)
{
	char name[32];

	snprintf(name, 32, "%s", "safety");
	debugfs = debugfs_create_dir(name, NULL);
	if (!debugfs)
		return -ENOMEM;

	debugfs_create_file("command", S_IFREG | 0444, debugfs, NULL, &dw_command_ops);
	//debugfs_create_file("recv_command", S_IFREG | 0444,debugfs, NULL, &dw_command_ops);
	return 0;
}

static int safety_driver_remove(struct platform_device *args)
{
	debugfs_remove_recursive(debugfs);
	ipc_close(safety_session_id);
	return 0;
}

static int safety_driver_probe(struct platform_device *pdev)
{
	int ret = 0;
	//dma_addr_t partition_dma_addr;

	resource_size_t size;
	struct resource res;
	struct device_node *np;



	#ifdef CONFIG_BST_C1200_IVI
	uint32_t cpu_id = IPC_CORE_ARM0;
	#endif

	#ifdef CONFIG_BST_C1200_ADAS
	uint32_t cpu_id = IPC_CORE_ARM2;
	#endif

	#ifdef CONFIG_BST_C1200_DB
	uint32_t cpu_id = IPC_CORE_DB0;
	#endif


	printk("%s %d\n",__func__,__LINE__);
	// ret = of_reserved_mem_device_init_by_idx(&pdev->dev, pdev->dev.of_node, 0);
	// if (ret) {
	// 	dev_err(&pdev->dev, "safety of_reserved_mem_device_init error\n");
	// 	ret = -EINVAL;
	// 	return ret;
	// }

	// vbuf  = dma_alloc_coherent(&pdev->dev, COMMAND_SIZE, &partition_dma_addr, GFP_KERNEL);
	// if ((unsigned long)partition_dma_addr != (unsigned long)COMMAND_ADDR) {
	// 	pr_err("safety phyaddr %llx %llx = 0x87000000\n", partition_dma_addr, dma_to_phys(&pdev->dev, partition_dma_addr));
	// 	dma_free_coherent(&pdev->dev, COMMAND_SIZE, vbuf, partition_dma_addr);
	// 	ret = -EINVAL;
	// 	return ret;
	// }



	np = of_parse_phandle(pdev->dev.of_node, "shmem", 0);
	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret) {
		dev_err(&pdev->dev, "failed to get shared memory\n");
		return -EADDRNOTAVAIL;
	}

	size = resource_size(&res);
	vbuf = devm_ioremap_wc(&pdev->dev, res.start, size);
	if (!vbuf) {
		dev_err(&pdev->dev, "failed to ioremap shared memory\n");
		return -EADDRNOTAVAIL;
	}


	// printk("%s %d\n",__func__,__LINE__);

	safety_session_id =  ipc_init(IPC_CORE_SAFE,cpu_id,NULL);



	//safety_session_id = ipc_init(IPC_CORE_R5_0, IPC_CORE_ARM5, NULL);
	safety_command_debugfs_init();
	return 0;
}


static const struct of_device_id safety_of_match[] = {
	{ .compatible = "bst,safety_command", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, safety_of_match);



static struct platform_driver safety_driver = {
	.driver = {
		.name = "safety",
		.of_match_table = of_match_ptr(safety_of_match),
	},
	.probe = safety_driver_probe,
	.remove = safety_driver_remove,
};

static int __init safety_command_init(void)
{
	platform_driver_register(&safety_driver);
	return 0;
}

static void __exit safety_command_exit(void)
{
	platform_driver_unregister(&safety_driver);
}

module_init(safety_command_init);
module_exit(safety_command_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("soc to safety command");