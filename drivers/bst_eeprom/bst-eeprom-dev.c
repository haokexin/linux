/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>

#define RT_I2C2_NR 22 //RT_I2C2_NR is I2C id

static struct i2c_client *i2c_dev=NULL;
static struct i2c_adapter *adap=NULL;
static struct i2c_board_info info=
{
	.type="bst_eeprom",
	.addr=0x50, /*device addr 0x50*/
};

static int __init bst_eeprom_drv_init(void)
{	

	adap=i2c_get_adapter(RT_I2C2_NR);
    if(!adap){
          printk("LCT failed to get I2C adater\n");
          return -ENODEV;
    }

	i2c_dev=i2c_new_client_device(adap,&info);

    if(!i2c_dev){
          printk("LCT failed to new i2c_dev\n");
          return -ENODEV;
    }
    printk("bst_eeprom_drv_init sucess\n");
    return 0;
}

static void __exit bst_eeprom_drv_cleanup(void)
{

	i2c_unregister_device(i2c_dev);
	i2c_put_adapter(adap);
    printk("bst_eeprom_drv_cleanup sucess\n");
}

module_init(bst_eeprom_drv_init);    
module_exit(bst_eeprom_drv_cleanup); 

MODULE_AUTHOR("chaotu.liao@bst.ai");
MODULE_DESCRIPTION("I2C-EEPROM /dev entries driver");
MODULE_LICENSE("GPL");  