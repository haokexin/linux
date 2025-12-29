// SPDX-License-Identifier: GPL-2.0
/*
 * BST I2C bus semaphore implementation
 * Copyright (c) 2025, BST Corporation.
 */
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/bst_samphore.h>

#include "i2c-designware-core.h"


#define I2C_SEMAPHORE_TIMEOUT 5



int hw_lock_semaphore(struct dw_i2c_dev *dev)
{
	int sem_id;

	if(dev->i2c_hw_lock == NULL) {
		return -1;
	}

	sem_id = get_sem_lock_with_timeout(dev->i2c_hw_lock,I2C_SEMAPHORE_TIMEOUT);
	if(sem_id != 0 && dev->i2c_hw_lock->mst_id != sem_id) {
		pr_warn("i2c can't get semaphore : %x !!!!!\n",sem_id);
		return sem_id;
	}

	dev->hw_lock_cnt++;

	return 0;
}

void hw_unlock_semaphore(struct dw_i2c_dev *dev)
{

	if(dev->i2c_hw_lock == NULL) {
		return;
	}

	if(dev->hw_lock_cnt > 1)
	{
		dev->hw_lock_cnt--;
		return;
	}

	dev->hw_lock_cnt = 0;


	
	release_bst_sem_lock(dev->i2c_hw_lock);
	return;
}



int i2c_dw_bst_probe_lock_support(struct dw_i2c_dev *dev)
{
    u32 sem_id[3]={0};
	int ret;
	int mst_id, bank_id, msg_id;

	ret = device_property_read_u32_array(dev->dev,"ipc-sem", sem_id, 3);
	if (ret) {
		dev_warn(dev->dev, "i2c dts ipc-sem node loss,dw use default\n");
		return -ENODEV;
	}

	mst_id =  sem_id[0];
	bank_id = sem_id[1];
	msg_id =  sem_id[2];

	dev_info(dev->dev, "ipc-sem id: <%d %d %d>", mst_id, bank_id, msg_id);

	dev->i2c_hw_lock = bst_semaphore_init(mst_id, bank_id, msg_id);

	dev->acquire_hw_lock = hw_lock_semaphore;
	dev->release_hw_lock = hw_unlock_semaphore;

	return 0;
}