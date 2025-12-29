// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/kthread.h>
#include "virtual-backlight/virt_backlight.h"

struct virt_backlight_data {
    struct platform_device *pdev;
    struct task_struct *init_task;

    /* backlight device */
	struct backlight_device *bd;
    struct virt_bl_resource *res;
	int	curr_brightness;

    /* user config */
    const char *dev_name;
    unsigned int client_id;
	unsigned int screen_id;
    int default_brightness;
};


static int virt_backlight_update_status(struct backlight_device *bl)
{
    struct virt_backlight_data *data = bl_get_data(bl);
    unsigned int new_brightness = backlight_get_brightness(bl);
    int ret;

    data->curr_brightness = new_brightness;
    ret = bst_bl_set_brightness(data->res, new_brightness);
    if (ret) {
        dev_err(&data->pdev->dev, "update screen:%d brightness:%d failed:%d.",
                data->screen_id, new_brightness, ret);
        return ret;
    }
    dev_dbg(&data->pdev->dev, "set brightness %d\n", data->curr_brightness);
    return ret;
}

static int virt_backlight_get_brightness(struct backlight_device *bl)
{
	struct virt_backlight_data *data = bl_get_data(bl);
    unsigned int brightness;
    int ret;

    ret = bst_bl_get_brightness(data->res, &brightness);
    if (ret) {
        dev_err(&data->pdev->dev, "get screen:%d brightness failed:%d\n",
                data->screen_id, ret);
        return -EINVAL;
    }
    data->curr_brightness = brightness;
    dev_dbg(&data->pdev->dev, "get brightness %d\n", data->curr_brightness);
	return data->curr_brightness;
}

static const struct backlight_ops virt_backlight_ops = {
	.options	= BL_CORE_SUSPENDRESUME,
	.update_status	= virt_backlight_update_status,
	.get_brightness	= virt_backlight_get_brightness
};

static void virt_brightness_change_event(void *priv,
                                            uint32_t brightness)
{
    struct virt_backlight_data *bl_data = priv;

    bl_data->curr_brightness = brightness;
    if (!bl_data->bd) {
        return;
    }

	backlight_force_update(bl_data->bd, BACKLIGHT_UPDATE_HOTKEY);
}

static int virt_backlight_request_screen(struct virt_backlight_data *bl_data)
{
    struct platform_device *pdev = bl_data->pdev;
    struct virt_bl_resource *res;
    int ret;

    /* load hardware info */
    res = bst_bl_request_resource(bl_data->client_id,
                                    bl_data->screen_id,
                                    virt_brightness_change_event,
                                    bl_data);
    if (!res) {
        dev_err(&pdev->dev, "request client:0x%x screen:0x%x failed!",
                bl_data->client_id, bl_data->screen_id);
        return -EINVAL;
    }

    bl_data->res = res;
    dev_info(&pdev->dev, "screen id:%d name:%s\n", bl_data->screen_id, res->hw_name);

    /* get default brightness */
    if (bl_data->default_brightness == -1) {
        bl_data->default_brightness = res->max_brightness / 2 + (res->max_brightness % 2);
        dev_info(&pdev->dev,
                "No default-brightness-level specified in DT, "
                "using %u (max_brightness/2) as default\n",
                bl_data->default_brightness);
    }

    if (bl_data->default_brightness >= 0) {
        if (bl_data->default_brightness > res->max_brightness) {
            dev_warn(&pdev->dev,
                    "invalid default brightness level: %u, using %u\n",
                    bl_data->default_brightness, res->max_brightness);
            bl_data->default_brightness = res->max_brightness;
        }
    }

    ret = bst_bl_get_brightness(bl_data->res,
            &bl_data->curr_brightness);
    if (ret) {
        dev_err(&pdev->dev, "get screen:%d brightness failed:%d\n",
                bl_data->screen_id, ret);
        ret = -EINVAL;
        goto out;
    }

    return 0;
out:
    if (res) {
        bst_bl_release_resource(res);
    }
    bl_data->res = NULL;
    return ret;
}

static void virt_backlight_release_screen(struct virt_backlight_data *bl_data)
{
    (void)bst_bl_release_resource(bl_data->res);
}

static int virt_backlight_create_dev(struct virt_backlight_data *bl_data)
{
    struct platform_device *pdev = bl_data->pdev;
    struct backlight_device *bl;
    struct backlight_properties props;
    char name[40];

	memset(name, 0, sizeof(name));
    if (bl_data->dev_name) {
        strncpy(name, bl_data->dev_name, sizeof(name)-1);
    } else if (bl_data->res->hw_name[0] != '\0') {
        snprintf(name, sizeof(name), "bst_%s", bl_data->res->hw_name);
    } else {
        snprintf(name, sizeof(name), "bst_virt_%x_%d", bl_data->client_id, bl_data->screen_id);
    }

    memset(&props, 0, sizeof(struct backlight_properties));
    props.type = BACKLIGHT_RAW;
    props.max_brightness = bl_data->res->max_brightness;

	bl = devm_backlight_device_register(&bl_data->pdev->dev, name,
					    &bl_data->pdev->dev, bl_data, &virt_backlight_ops,
					    &props);
	if (IS_ERR(bl)) {
        dev_err(&pdev->dev, "backlight dev:%s created failed.\n", name);
		return PTR_ERR(bl);
    }

    if (bl_data->curr_brightness == 0) {
        bl->props.brightness = bl_data->default_brightness;
        bl->props.power = FB_BLANK_POWERDOWN;
    } else {
        bl->props.brightness = bl_data->curr_brightness;
        bl->props.power = FB_BLANK_UNBLANK;
    }
    bl_data->bd = bl;

    dev_info(&pdev->dev, "backlight dev:%s created.\n", name);
    return 0;
}

static void virt_backlight_dev_deinit(struct virt_backlight_data *bl_data)
{
    struct platform_device *pdev = bl_data->pdev;

    if (bl_data->bd) {
        devm_backlight_device_unregister(&pdev->dev, bl_data->bd);
        bl_data->bd = NULL;
    }
    virt_backlight_release_screen(bl_data);
}

static int virt_backlight_dev_init(void *data)
{
    struct virt_backlight_data *bl_data = (struct virt_backlight_data*)data;
	struct platform_device *pdev;
	int ret;

	if(!bl_data || !bl_data->pdev)
		return -EINVAL;

	pdev = bl_data->pdev;

    dev_info(&pdev->dev, "start create devices!\n");
	ret = bst_backlight_init();
	if (ret) {
		dev_err(&pdev->dev, "failed to initial msgbox client!\n");
		return -EINVAL;
	}

    ret = virt_backlight_request_screen(bl_data);
    if (ret) {
        dev_err(&pdev->dev, "load screen:%d config failed:%d\n", bl_data->screen_id, ret);
        goto deinit;
    }
    
    ret = virt_backlight_create_dev(bl_data);
    if (ret < 0) {
        dev_err(&pdev->dev, "create screen:%d dev failed:%d\n", bl_data->screen_id, ret);
        virt_backlight_dev_deinit(bl_data);
        goto deinit;
    }
    dev_err(&pdev->dev, "create screen:%d dev success\n", bl_data->screen_id);
    return 0;

deinit:
    devm_kfree(&pdev->dev, bl_data);
    return ret;
}

static int virt_backlight_parse_dt(struct virt_backlight_data *bl_data)
{
    struct platform_device *pdev = bl_data->pdev;
	struct device_node *node = dev_of_node(&pdev->dev);

	if(!node)
		return -EINVAL;

	if (of_property_read_u32(node, "client-id", &bl_data->client_id)) {
		dev_err(&pdev->dev, "failed to read client-id property.\n");
		return -EINVAL;
	}

	dev_dbg(&pdev->dev, "client-id: 0x%x\n", bl_data->client_id);

    if (of_property_read_u32(node, "screen-id", &bl_data->screen_id)) {
        dev_warn(&pdev->dev, "no screen-id found in:%s, skip!", node->name);
        return -EINVAL;
    }

    if (of_property_read_string(node, "device-name", &bl_data->dev_name)) {
        bl_data->dev_name = NULL;
    }

    if (of_property_read_u32(node, "default-brightness-level",
                    &bl_data->default_brightness)) {
        bl_data->default_brightness = -1;
    }

	return 0;
}

static int bst_virt_bl_drv_probe(struct platform_device *pdev)
{
	struct virt_backlight_data *bl_data;
    int ret;

	bl_data = devm_kzalloc(&pdev->dev, sizeof(*bl_data), GFP_KERNEL);
	if (!bl_data)
		return -ENOMEM;

	bl_data->pdev = pdev;
	platform_set_drvdata(pdev, bl_data);

    ret = virt_backlight_parse_dt(bl_data);
	if (ret) {
		dev_err(&pdev->dev, "failed to parse dts!\n");
		goto err_init;
	}

    bl_data->init_task = kthread_run(virt_backlight_dev_init, (void *)bl_data, "virt_bl_init");
	if (!bl_data->init_task) {
		dev_err(&pdev->dev, "failed to start create devices thread!\n");
		goto err_init;
	}

	dev_info(&pdev->dev, "probe success.");
    return 0;

err_init:
	devm_kfree(&pdev->dev, bl_data);
	return ret;
}

static int bst_virt_bl_drv_remove(struct platform_device *pdev)
{
	struct virt_backlight_data *bl_data = platform_get_drvdata(pdev);

    if (bl_data->init_task) {
        kthread_stop(bl_data->init_task);
    }
	virt_backlight_dev_deinit(bl_data);
	devm_kfree(&pdev->dev, bl_data);
    bst_backlight_deinit();
    dev_info(&pdev->dev, "driver removed.");
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bst_virt_bl_suspend(struct device *dev)
{
    return 0;
}

static int bst_virt_bl_resume(struct device *dev)
{
    struct virt_backlight_data *data = dev_get_drvdata(dev);
    int ret;

    if (!data || !data->res) {
        return 0;
    }

    ret = bst_bl_declare_resource(data->res);
    if (ret) {
        dev_err(&data->pdev->dev, "resume screen:%d backlight failed:%d.",
                data->screen_id, ret);
    }

    return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(bst_virt_bl_pm_ops, bst_virt_bl_suspend,
			bst_virt_bl_resume);

static const struct of_device_id bst_virt_bl_of_match[] = {
    {.compatible = "bst,virt-backlight"},
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bst_virt_bl_of_match);

static struct platform_driver bst_virt_backlight_driver = {
    .driver = {
        .name = "bst-virt-backlight",
        .of_match_table = of_match_ptr(bst_virt_bl_of_match),
        .pm = &bst_virt_bl_pm_ops,
    },
    .probe = bst_virt_bl_drv_probe,
    .remove = bst_virt_bl_drv_remove,
};

module_platform_driver(bst_virt_backlight_driver);

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("BST Virtual Backlight Driver");
MODULE_LICENSE("GPL v2");
