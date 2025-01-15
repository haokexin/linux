// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>

#include "st_public1_client.h"
#include "thermalclient.h"

#define PVTREG_SPI_CONTROL    0x0
#define PVTREG_SPI_SLAVE      0x8
#define PVTREG_SPI_WBUFF      0x10
#define PVTREG_SENSOR0	      0x20
#define PVTREG_SENSOR1	      0x24
#define PVTREG_SENSOR2	      0x28
#define PVTREG_SENSOR3	      0x2c
#define PVTREG_SENSOR4	      0x30
#define PVTREG_TEMP_THRESHOLD 0x50

#define PVT_NUM	      5
#define BST_CDEV_SIZE 1

/* jianeng.chen 2023.02.02 */
static int bst_cur_state;
static int pwm_duty = 200;
// static struct gpio_desc *simpwm_gpio;
// static struct kthread_worker *pwm_kworker;
// static struct kthread_work pwmfan;
static bool mod_run;

struct bst_cooling_dev_info {
	const char *name;
	struct thermal_cooling_device *cdev;
};

struct bst_thermal {
	struct thermal_zone_device *tz;
	struct bst_cooling_dev_info bst_cdev_info[BST_CDEV_SIZE];
	void __iomem *pvtreg;
	thermalclient_t *client;
};

enum PVT_TEMP
{
	TEMP_CPU_MP4_C0 = 0,
	TEMP_CPU_MP4_C1 = 1,
	TEMP_GPU_MP8    = 2,
	TEMP_ISP_CV     = 3,
	TEMP_NET_CV     = 4,
	TEMP_DDR        = 5,
	TEMP_SEIP       = 8,

};

static thermalclient_data_t data= {{0}};

// static void gpio_simulate_pwm(struct kthread_work *work)
// {
// 	int output = 0;

// 	while (mod_run) {
// 		output = 1;
// 		gpiod_set_value(simpwm_gpio, output);
// 		usleep_range(pwm_duty, pwm_duty + 10);

// 		output = 0;
// 		gpiod_set_value(simpwm_gpio, output);
// 		usleep_range(1000 - pwm_duty, 1010 - pwm_duty);
// 	}

// }

static int bst_get_cdev_max_state(struct thermal_cooling_device *cdev,
				  unsigned long *max_state)
{
	*max_state = 3;
	return 0;
}

static int bst_get_cdev_cur_state(struct thermal_cooling_device *cdev,
				  unsigned long *cur_state)
{
	*cur_state = bst_cur_state;
	return 0;
}

static int bst_set_cdev_cur_state(struct thermal_cooling_device *cdev,
				  unsigned long cur_state)
{
	if (cur_state == 0) {
		pwm_duty = 200;
	} else if (cur_state == 1) {
		pwm_duty = 500;
	} else if (cur_state == 2) {
		pwm_duty = 800;
	} else if (cur_state == 3) {
		pwm_duty = 1000;
	}

	bst_cur_state = cur_state;
	return 0;
}

static const struct thermal_cooling_device_ops bst_cooling_ops = {
	.get_max_state = bst_get_cdev_max_state,
	.get_cur_state = bst_get_cdev_cur_state,
	.set_cur_state = bst_set_cdev_cur_state,
};


static int bst_thermal_get_temp(struct thermal_zone_device *tz, int *temp)
{
	// u32 reg;
	// u32 val;
	st_public1_ErrorEnum_t err = 0;
	uint32_t result;
	struct bst_thermal *bt = tz->devdata;

	// reg = readl(bt->pvtreg + PVTREG_SENSOR0);
	// val = reg >> 3;
	// *temp = transfer(val) * 1000;

	bt->client->st_public1_client.gettemp_method_sync(TEMP_CPU_MP4_C0, &result, &err, 1000, NULL);
	// printk("gettemp reply : %d err: %d.\n", result, err);
	*temp = result;
	return 0;
}

static const struct thermal_zone_device_ops bst_thermal_ops = {
	.get_temp = bst_thermal_get_temp,
};

static ssize_t cpuc0_temp_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bst_thermal *bt = platform_get_drvdata(pdev);
	int temp;
	st_public1_ErrorEnum_t err = 0;
	uint32_t result;

	bt->client->st_public1_client.gettemp_method_sync(TEMP_CPU_MP4_C0, &result, &err, 1000, NULL);
	// printk("gettemp reply : %d err: %d.\n", result, err);
	temp = result;
	return sprintf(buf, "%d\n", temp);
}

static ssize_t cpuc1_temp_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bst_thermal *bt = platform_get_drvdata(pdev);
	int temp;
	st_public1_ErrorEnum_t err = 0;
	uint32_t result;

	bt->client->st_public1_client.gettemp_method_sync(TEMP_CPU_MP4_C1, &result, &err, 1000, NULL);
	// printk("gettemp reply : %d err: %d.\n", result, err);
	temp = result;
	return sprintf(buf, "%d\n", temp);
}

static ssize_t gpu_temp_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bst_thermal *bt = platform_get_drvdata(pdev);
	int temp;
	st_public1_ErrorEnum_t err = 0;
	uint32_t result;

	bt->client->st_public1_client.gettemp_method_sync(TEMP_GPU_MP8, &result, &err, 1000, NULL);
	// printk("gettemp reply : %d err: %d.\n", result, err);
	temp = result;
	return sprintf(buf, "%d\n", temp);
}

static ssize_t net_temp_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bst_thermal *bt = platform_get_drvdata(pdev);
	int temp;
	st_public1_ErrorEnum_t err = 0;
	uint32_t result;

	bt->client->st_public1_client.gettemp_method_sync(TEMP_NET_CV, &result, &err, 1000, NULL);
	// printk("gettemp reply : %d err: %d.\n", result, err);
	temp = result;

	return sprintf(buf, "%d\n", temp);
}

static ssize_t isp_temp_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bst_thermal *bt = platform_get_drvdata(pdev);
	int temp;
	st_public1_ErrorEnum_t err = 0;
	uint32_t result;

	bt->client->st_public1_client.gettemp_method_sync(TEMP_ISP_CV, &result, &err, 1000, NULL);
	// printk("gettemp reply : %d err: %d.\n", result, err);
	temp = result;

	return sprintf(buf, "%d\n", temp);
}

static ssize_t ddr_temp_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bst_thermal *bt = platform_get_drvdata(pdev);
	int temp;
	st_public1_ErrorEnum_t err = 0;
	uint32_t result;

	bt->client->st_public1_client.gettemp_method_sync(TEMP_DDR, &result, &err, 1000, NULL);
	// printk("gettemp reply : %d err: %d.\n", result, err);
	temp = result;
	return sprintf(buf, "%d\n", temp);
}

static ssize_t seip_temp_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bst_thermal *bt = platform_get_drvdata(pdev);
	int temp;
	st_public1_ErrorEnum_t err = 0;
	uint32_t result;

	bt->client->st_public1_client.gettemp_method_sync(TEMP_SEIP, &result, &err, 1000, NULL);
	// printk("gettemp reply : %d err: %d.\n", result, err);
	temp = result;

	return sprintf(buf, "%d\n", temp);
}



static DEVICE_ATTR_RO(cpuc0_temp);
static DEVICE_ATTR_RO(cpuc1_temp);
static DEVICE_ATTR_RO(gpu_temp);
static DEVICE_ATTR_RO(net_temp);
static DEVICE_ATTR_RO(isp_temp);
static DEVICE_ATTR_RO(ddr_temp);
static DEVICE_ATTR_RO(seip_temp);
static struct attribute *temp_attrs[] = { &dev_attr_cpuc0_temp.attr,
					  &dev_attr_cpuc1_temp.attr,
					  &dev_attr_gpu_temp.attr,
					  &dev_attr_net_temp.attr,
					  &dev_attr_isp_temp.attr,
					  &dev_attr_ddr_temp.attr,
					  &dev_attr_seip_temp.attr, NULL };

static const struct attribute_group temp_attribute_group = { .attrs =
								     temp_attrs,
							     .name = "temps" };

static void init_cooling_device(struct platform_device *pdev)
{
	struct device_node *np, *child;
	struct bst_thermal *bt = platform_get_drvdata(pdev);
	int cdev_index;

	cdev_index = 0;
	np = of_find_node_by_name(NULL, "cooling_dev");
	if (!np) {
		dev_info(&pdev->dev, "can't find bst cooling_dev\n");
		return;
	}

	for_each_available_child_of_node(np, child) {
		struct thermal_cooling_device *tcd;

		if (cdev_index + 1 > BST_CDEV_SIZE) {
			of_node_put(child);
			of_node_put(np);
			return;
		}
		dev_info(&pdev->dev, "cooling_dev, name=%s", child->name);
		bt->bst_cdev_info[cdev_index].name = child->name;

		tcd = thermal_of_cooling_device_register(
			child, (char *)child->name, bt, &bst_cooling_ops);
		if (IS_ERR_OR_NULL(tcd)) {
			dev_err(&pdev->dev,
				"bst cooling_dev: %s: failed to register cooling device\n",
				child->name);
			continue;
		}
		bt->bst_cdev_info[cdev_index].cdev = tcd;
		cdev_index++;
	}
	of_node_put(np);

	// simpwm_gpio = devm_gpiod_get(&pdev->dev, "simpwm", GPIOD_OUT_HIGH);
	// if (IS_ERR_OR_NULL(simpwm_gpio)) {
	// 	dev_err(&pdev->dev, "unable to get simpwm gpio %d\n", PTR_ERR(simpwm_gpio));
	// } else {
	// 	//printk("thermal sim pwm gpiod get ok\n");
	// 	gpiod_direction_output(simpwm_gpio, 0);
	// 	// kthread_queue_work(pwm_kworker, &pwmfan);
	// }
}

static uint32_t s_index = 0;
static int init_pvt_msgbox(struct bst_thermal *bt)
{
	//thermalclient_data_t data= {{0}};
	ipc_inf_version_t version;
	int32_t ret, total, index, result;
	st_public1_ErrorEnum_t err;

    bt->client = thermalclient_init(&data);
    if (!bt->client)
    {
        printk("init client fail.\n");
        return -1;
    }

    // get version
    version = bt->client->st_public1_client.version();
    printk("Interface version: major %d, minor %d.\n", version.major, version.minor);

    // start test_client
    ret = bt->client->start();
    // int32_t ret = test_client_start(client);
    if (ret < 0)
    {
        printk("start test client failed!\n");
        return -2;
    }


    total = 1u;
	index = 0;
	err = 0;

    while (s_index < total)
    {
        bt->client->st_public1_client.gettemp_method_sync(index, &result, &err, 1000, NULL);
        printk("gettemp reply : %d err: %d.\n", result, err);

        ++s_index;
    }

    return 0;
}

static int bst_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bst_thermal *bt;
	int ret;

	bt = devm_kzalloc(dev, sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return -ENOMEM;


	ret = init_pvt_msgbox(bt);
	if (ret) {
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, bt);
	init_cooling_device(pdev);


	bt->tz = devm_thermal_of_zone_register(dev, 0, bt,
						      &bst_thermal_ops);
	if (IS_ERR(bt->tz)) {
		return PTR_ERR(bt->tz);
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &temp_attribute_group);
	if (ret)
		return ret;

	
	return 0;
}

static int bst_thermal_remove(struct platform_device *pdev)
{
	int i, ret;
	struct bst_thermal *bt = platform_get_drvdata(pdev);

	for (i = 0; i < BST_CDEV_SIZE; i++)
		thermal_cooling_device_unregister(bt->bst_cdev_info[i].cdev);

	// iounmap(bt->pvtreg);
	sysfs_remove_group(&pdev->dev.kobj, &temp_attribute_group);

	mod_run = false;
	// kthread_cancel_work_sync(&pwmfan);
	// kthread_destroy_worker(pwm_kworker);
	ret = bt->client->stop();
	ret = thermalclient_destroy();

	return 0;
}

static const struct of_device_id bst_thermal_of_match[] = {
	{
		.compatible = "bst,bst-thermal",
	},
	{},
};
MODULE_DEVICE_TABLE(of, bst_thermal_of_match);

static struct platform_driver bst_thermal_driver = {
	.probe = bst_thermal_probe,
	.remove = bst_thermal_remove,
	.driver = {
	.name = "bst-thermal",
	.of_match_table = bst_thermal_of_match,
	},
};
module_platform_driver(bst_thermal_driver);

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("BST thermal driver");
MODULE_LICENSE("GPL v2");
