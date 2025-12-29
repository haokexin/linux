/*
 * (C) COPYRIGHT ARM Limited. All rights reserved.
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained
 * from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 */
#define pr_fmt(fmt)		KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <linux/platform_device.h>
#include <linux/poll.h>

#include "mve_blk1_driver.h"
#include "mve_blk1_ioctl.h"
#include "mve_blk1_command.h"
#include "mve_blk1_fw.h"
#include "mve_blk1_session.h"

#include "mve_blk1_rsrc_log.h"
#include "mve_blk1_rsrc_register.h"

#define MVE_DRIVER_NAME "mv501"

/**
 * @brief Track information about the driver device.
 */
struct mve_device
{
    struct cdev cdev;        /**< Character-device information for the kernel.*/
    struct class *mve_class; /**< Class-information for the kernel. */
    struct device *dev;      /**< Pointer to the device struct */
    dev_t device;
};

struct device *mve_blk1_device;

static long mve_blk1_driver_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    switch (cmd)
    {
        case MVE_BASE_COMMAND:
        {
            struct mve_base_command_header header;
            void *data = NULL;
            struct mve_response *result;
            struct mve_base_response_header tmp;
            struct mve_base_response_header *dst;

            /* Get command header */
            if (0 != copy_from_user(&header, (void __user *)arg, sizeof(struct mve_base_command_header)))
            {
                MVE_LOG_PRINT(&mve_blk1_rsrc_log, MVE_LOG_ERROR, "copy_from_user() failed to get the MVE_COMMAND header.");
                return -EFAULT;
            }

            if (0 < header.size)
            {
                /* Fetch command data */
                data = MVE_RSRC_MEM_CACHE_ALLOC(header.size, GFP_KERNEL);
                if (NULL == data)
                {
                    MVE_LOG_PRINT(&mve_blk1_rsrc_log, MVE_LOG_ERROR, "Failed to allocate memory.");
                    return -ENOMEM;
                }

                if (0 != copy_from_user(data,
                                        (void __user *)(arg + sizeof(struct mve_base_command_header)),
                                        header.size))
                {
                    MVE_LOG_PRINT(&mve_blk1_rsrc_log, MVE_LOG_ERROR, "copy_from_user() failed to get MVE_COMMAND data.");
                    MVE_RSRC_MEM_CACHE_FREE(data, header.size);
                    return -EFAULT;
                }
            }

            /* Process command */
            result = mve_blk1_command_execute(&header, data, file);

            if (NULL != data)
            {
                MVE_RSRC_MEM_CACHE_FREE(data, header.size);
                data = NULL;
            }

            if (NULL != result)
            {
                tmp.error = result->error;
                tmp.firmware_error = result->firmware_error;
                tmp.size  = result->size;
            }
            else
            {
                return -ENOMEM;
            }

            /* Copy response header to userspace */
            dst = (struct mve_base_response_header *)arg;
            if (copy_to_user((void __user *)dst, &tmp, sizeof(tmp)))
            {
                MVE_LOG_PRINT(&mve_blk1_rsrc_log, MVE_LOG_ERROR, "copy_to_user() failed to copy MVE_COMMAND header.");
                //--------------------begin-----------------------------------//
                //Coverity check£º 'result' will resource leak here. so invoke free for 'result'
                if (result->size > 0 && NULL != result->data)
                    MVE_RSRC_MEM_CACHE_FREE(result->data, result->size);
                result->data = NULL;

                MVE_RSRC_MEM_CACHE_FREE(result, sizeof(struct mve_response));
                result = NULL;
                //----------------------end-----------------------------------//
                return -EFAULT;
            }

            if (0 < result->size)
            {
                //WARN_ON(NULL == result->data);
                if (NULL == result->data)
                {
                    MVE_LOG_PRINT(&mve_blk1_rsrc_log, MVE_LOG_ERROR, "warning result->data is NULL.");
                }
                else
                {
                    /* Copy response data to userspace */
                    if (copy_to_user((void __user *)&dst->data, result->data, result->size))
                    {
                        MVE_LOG_PRINT(&mve_blk1_rsrc_log, MVE_LOG_ERROR, "copy_to_user() failed to copy MVE_COMMAND data.");

                        MVE_RSRC_MEM_CACHE_FREE(result->data, result->size);
                        result->data = NULL;
                        MVE_RSRC_MEM_CACHE_FREE(result, sizeof(struct mve_response));
                        result = NULL;
                        return -EFAULT;
                    }

                    MVE_RSRC_MEM_CACHE_FREE(result->data, result->size);
                    result->data = NULL;
                }
            }

            MVE_RSRC_MEM_CACHE_FREE(result, sizeof(struct mve_response));
            result = NULL;

            break;
        }
        default:
            MVE_LOG_PRINT(&mve_blk1_rsrc_log, MVE_LOG_ERROR, "Unknown ioctl. cmd=%u.", cmd);
            break;
    }
    return 0;
}

static int mve_blk1_driver_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int mve_blk1_driver_release(struct inode *inode, struct file *file)
{
    /* Cleanup all sessions that have been created using this fd but not yet been destroyed */
    mve_blk1_session_cleanup_client(file);

    return 0;
}

static unsigned int mve_blk1_driver_poll(struct file *filp, struct poll_table_struct *poll_table)
{
    return mve_blk1_session_poll(filp, poll_table);
}

static struct file_operations fops =
{
    .owner = THIS_MODULE,
    .open = mve_blk1_driver_open,
    .release = mve_blk1_driver_release,
    .unlocked_ioctl = mve_blk1_driver_ioctl,
#ifdef CONFIG_64BIT
    .compat_ioctl = mve_blk1_driver_ioctl,
#endif
    .poll = mve_blk1_driver_poll,
};

static int mve_blk1_driver_probe(struct platform_device *pdev)
{
    struct mve_device *mdev;
    int err;

    /* Save pointer to platform device. */
    mve_blk1_device = &pdev->dev;

    /* Allocate MVE device. */
    mdev = MVE_RSRC_MEM_ZALLOC(sizeof(*mve_blk1_device), GFP_KERNEL);
    memset(mdev, 0, sizeof(*mdev));

    /* Register a range of char device numbers. */
    err = alloc_chrdev_region(&mdev->device, 0, 1, MVE_DRIVER_NAME);

    /* Initialize our char dev data. */
    cdev_init(&mdev->cdev, &fops);
    mdev->cdev.owner = THIS_MODULE;

    /* Register char dev with the kernel */
    err = cdev_add(&mdev->cdev, mdev->device, 1);

    /* Create class for device driver. */
    mdev->mve_class = class_create(THIS_MODULE, MVE_DRIVER_NAME);

    /* Create a device node. */
    mdev->dev = device_create(mdev->mve_class, NULL, mdev->device, NULL, MVE_DRIVER_NAME);

    platform_set_drvdata(pdev, mdev);

    /* Initialize session module. */
    mve_blk1_session_init(&pdev->dev);

    /* Initialize firmware module. */
    mve_blk1_fw_init();

    dev_info(&pdev->dev, "0801 MVE_blk1 base driver loaded successfully\n");

    return 0;
}

static int mve_blk1_driver_remove(struct platform_device *pdev)
{
    struct mve_device *mdev = platform_get_drvdata(pdev);
    dev_t dev = MKDEV(MAJOR(mdev->device), 0);

    device_destroy(mdev->mve_class, dev);
    class_destroy(mdev->mve_class);

    /* Unregister char device. */
    cdev_del(&mdev->cdev);

    /* Free major. */
    unregister_chrdev_region(dev, 1);

    /* Deinitialize session. */
    mve_blk1_session_deinit(&pdev->dev);

    /* Free device structure. */
    MVE_RSRC_MEM_FREE(mdev);

    dev_info(&pdev->dev, "MVE_blk1 base driver unloaded successfully12\n");

    return 0;
}

static struct platform_driver mv500_driver =
{
    .probe = mve_blk1_driver_probe,
    .remove = mve_blk1_driver_remove,

    .driver = {
        .name = MVE_DRIVER_NAME,
        .owner = THIS_MODULE,
    },
};

static struct resource mve_resources[] =
{};

static void mve_blk1_device_release(struct device *dev)
{}

static struct platform_device mve_platform_device =
{
    .name = "mv501",
    .id = 0,
    .num_resources = ARRAY_SIZE(mve_resources),
    .resource = mve_resources,
    .dev = {
        .platform_data = NULL,
        .release = mve_blk1_device_release,
    },
};

extern int register_driver_mve_rsrc_blk1_notifier(struct notifier_block *nb);
extern int unregister_driver_mve_rsrc_blk1_notifier(struct notifier_block *nb);

static int mve_notify_event_handler(struct notifier_block *nb, unsigned long event, void *data)
{
    pr_info("%s.%d: Received event prepare to register mve base driver\n",  mve_platform_device.name, mve_platform_device.id);
    platform_device_register(&mve_platform_device);

    return NOTIFY_OK;
}

static struct notifier_block mve_notifier = {
    .notifier_call = mve_notify_event_handler,
};

static int __init mve_driver_init(void)
{
    int ret;

    platform_driver_register(&mv500_driver);
    ret = register_driver_mve_rsrc_blk1_notifier(&mve_notifier);
    if (ret) {
        return platform_device_register(&mve_platform_device);
    }

    return 0;
}

static void __exit mve_driver_exit(void)
{
    unregister_driver_mve_rsrc_blk1_notifier(&mve_notifier);
    platform_driver_unregister(&mv500_driver);
    platform_device_unregister(&mve_platform_device);

    MVE_LOG_PRINT(&mve_blk1_rsrc_log, MVE_LOG_INFO, "MVE_blk1 base driver unregistered");
}

MODULE_LICENSE("GPL");

#if (1 == DMA_BUF_NAMESPACE)
MODULE_IMPORT_NS(DMA_BUF);
#endif /*(1 == DMA_BUF_NAMESPACE)*/

MODULE_DESCRIPTION("Mali-V500 video engine driver");

module_init(mve_driver_init);
module_exit(mve_driver_exit);
MODULE_IMPORT_NS(DMA_BUF);
