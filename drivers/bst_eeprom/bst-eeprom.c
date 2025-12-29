/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include "eeprom_common_op.h"

static struct i2c_client *eeprom_client;
#define MAX_SIZE 1024 * 4 // eeprom size

//eeprom address[] = {0x50, 0x51, 0x52, 0x53};
#define MASS_EEPROM_ADDR 0x52
#define EVM_EEPROM_ADDR 0x50
#define SLT_EEPROM_ADDR 0x50

uint8_t eeprom_address = EVM_EEPROM_ADDR;

static int bst_eeprom_open(struct inode *inode, struct file *file)
{
	printk("bst_eeprom_open-->ok\n");
	return 0;
}

static ssize_t bst_eeprom_read(struct file *file, char __user *buf, size_t size, loff_t *seek)
{

	uint16_t offset = 0;
	size_t read_ok_cnt = 0;
	int ret;
	int err;
	static uint8_t output_buffer[MAX_SIZE];

#ifdef CONFIG_C1200_SLT
	eeprom_address = SLT_EEPROM_ADDR;//evm or slt
#endif

#ifdef CONFIG_C1200_MASS
	eeprom_address = MASS_EEPROM_ADDR;//cdcu mass
#endif

	//printk(" bst_eeprom_read ");
	for (offset = 0; offset < size; offset++)
	{
		ret = i2c_read_byte_data_word_reg(eeprom_client->adapter, eeprom_address, offset, &output_buffer[offset]);
		if (ret < 0)
		{

			printk("LCT  bst_eeprom_read err");
			return -1;
		}
		//printk("LCT  bst_eeprom_read: eeprom_address:0x%x offset: 0x%x, 0x%x, size:%ld\n", eeprom_address, offset, output_buffer[offset], size);

		read_ok_cnt += 1;
		*seek += 1;
		if (read_ok_cnt == size)
		{
			printk("LCT  bst_eeprom_read: read_ok_cnt == size");
			break;
		}
	}
	msleep(10);

	err = copy_to_user(buf, output_buffer, size /*sizeof(output_buffer)*/);

	if (err != 0)
	{

		return -1;
	}
	printk("LCT  bst_eeprom_read OK\n");

	return size;
}

static ssize_t bst_eeprom_write(struct file *file, const char __user *buf, size_t size, loff_t *seek)
{
	uint16_t offset = 0;
	size_t write_ok_cnt = 0;
	int ret;
	static uint8_t input_buffer[MAX_SIZE];

	unsigned long err;

#ifdef CONFIG_C1200_SLT
	eeprom_address = SLT_EEPROM_ADDR;//evm or slt
#endif

#ifdef CONFIG_C1200_MASS
	eeprom_address = MASS_EEPROM_ADDR;//cdcu mass
#endif

	err = copy_from_user(input_buffer, buf, size);

	for (offset = 0; offset < size; offset++)
	{
		uint16_t offset_in_eeprom = offset;

		//printk("LCT  bst_eeprom_write:eeprom_address:0x%x,offset: 0x%x, offset_in_eeprom: 0x%x,0x%x\n", eeprom_address, offset, offset_in_eeprom, input_buffer[offset]);

		ret = i2c_write_byte_data_word_reg(eeprom_client->adapter, eeprom_address, offset_in_eeprom, input_buffer[offset]);
        
		msleep(10);

		if (ret < 0)
		{
			printk("LCT  bst_eeprom_write err");
			return ret;
		}

		write_ok_cnt++;

		if (write_ok_cnt == size)
		{

            printk("bst_eeprom_write OK");
			break;
		}
	}
	printk("bst_eeprom_write OK");

	return write_ok_cnt;
}


static loff_t bst_eeprom_llseek(struct file *filp, loff_t offset, int whence)
{
	loff_t newpos = 0;
	switch (whence)
	{
	case SEEK_SET:
		newpos = offset;
		break;
	case SEEK_CUR:
		newpos = filp->f_pos + offset;
		break;
	case SEEK_END:
		if (MAX_SIZE + offset >= MAX_SIZE)
		{
			newpos = MAX_SIZE;
		}
		else
		{
			newpos = MAX_SIZE + offset;
		}
		break;
	default:
		return -EINVAL; 
	}
	filp->f_pos = newpos;
	return newpos;
}

static int bst_eeprom_release(struct inode *inode, struct file *file)
{
	//printk("bst_eeprom_release-->ok\n");
	return 0;
}

static struct file_operations fops =
	{
		.open = bst_eeprom_open,
		.read = bst_eeprom_read,
		.write = bst_eeprom_write,
		.release = bst_eeprom_release,
		.llseek = bst_eeprom_llseek
	};


static struct miscdevice misc =
	{
		.minor = MISC_DYNAMIC_MINOR, 
		.name = "bst_eeprom",		 
		.fops = &fops,				 
};

static int bst_eeprom_probe(struct i2c_client *client, const struct i2c_device_id *device_id)
{
	printk("bst_eeprom_probe :%#X\n", client->addr);
	eeprom_client = client;

	misc_register(&misc);

	return 0;
}

static void bst_eeprom_remove(struct i2c_client *client)
{

	misc_deregister(&misc);
	printk("bst_eeprom_remove.\n");
	// return ;
}

static struct i2c_device_id id_table[] =
	{
		{"bst_eeprom", 0},
		{}};

static struct i2c_driver drv =
	{
		.probe = bst_eeprom_probe,
		.remove = bst_eeprom_remove,
		.driver =
			{
				.name = "eeprom_iic"},
		.id_table = id_table};

static int __init bst_eeprom_drv_init(void)
{
	i2c_add_driver(&drv);
	printk("bst_eeprom_drv_init sucess!\n");
	return 0;
}

static void __exit bst_eeprom_drv_cleanup(void)
{
	i2c_del_driver(&drv);
	printk("bst_eeprom_drv_cleanup sucess!\n");
}

module_init(bst_eeprom_drv_init);	
module_exit(bst_eeprom_drv_cleanup); 

MODULE_AUTHOR("chaotu.liao@bst.ai");
MODULE_DESCRIPTION("I2C-EEPROM /dev entries driver");
MODULE_LICENSE("GPL v2"); 