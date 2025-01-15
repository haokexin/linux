// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

static char mybuf[20] = "";
struct pinctrl *p;
static ssize_t pinmux_switch_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t len)
{
    
    struct pinctrl_state *state = NULL;
    int size;
    int max;

    if (len > sizeof(mybuf)) {
        max = sizeof(mybuf);
    } else {
        max = len;
    }
        
    size = snprintf(mybuf, max, buf);
    
    state = pinctrl_lookup_state(p, mybuf);
    if (IS_ERR(state)) {
        dev_err(dev, "error find pinmux %s\n", mybuf);
        state = NULL;
        return size;
    }
    pinctrl_select_state(p, state);

    return size;
}
//static DEVICE_ATTR(pinmux_switch, S_IWUSR|S_IRUSR, NULL, pin_switch_store);
static DEVICE_ATTR_WO(pinmux_switch);

static int pinmux_switch_remove(struct platform_device *pdev)
{
    sysfs_remove_file(&pdev->dev.kobj, &dev_attr_pinmux_switch.attr);

    return 0;
}

static int pinmux_switch_probe(struct platform_device *pdev)
{
    int ret;

    // dev_err(&pdev->dev, "%s\n", __func__);

    p = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(p)) {
        dev_err(&pdev->dev, "error get pinmux\n");
        return 0;
    }

    ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_pinmux_switch.attr);
	if (ret)
		return ret;

    return 0;
}


static const struct of_device_id pinmux_of_match[] = {
    { .compatible = "bst,pinmux_switch", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pinmux_of_match);

static struct platform_driver pinmux_switch_driver = {
    .driver = {
            .name = "pinmux-switch",
            .of_match_table = of_match_ptr(pinmux_of_match),
    },
    .probe = pinmux_switch_probe,
    .remove = pinmux_switch_remove,

};

module_platform_driver(pinmux_switch_driver);
MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("BST pin switch driver");
MODULE_LICENSE("GPL v2");