// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include "bst_gtc_common.h"
#include <bst/bst_gtc.h>
struct time_sync_parm record;
int bstgtc_log;

/*
 * bst_gtc_ioctl - This function is a wrapper bst_gtc ioctl interface
 * @filp:  file pointer to the gtc device
 * @cmd:   ioctl command
 * @arg:   pointer to the argument of the ioctl command
 */
static long bst_gtc_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
    int ret = 0, parm;
    u64 cnt;
    struct bst_gtc *pbst_gtc = NULL;
    struct gtc_freq freq_info;

    if ((filp == NULL) || (filp->private_data == NULL)) {
        return -EFAULT;
    }

	if (_IOC_TYPE(cmd) != GTC_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_NR(cmd) > GTC_IOC_MAXNR)
		return -ENOTTY;

    if (!try_module_get(THIS_MODULE))
        return -EFAULT;

    pbst_gtc = container_of(filp->private_data, struct bst_gtc, miscdev);
    if (!pbst_gtc) {
         printk(KERN_ERR "%s gtc uninitialized\n", __func__);
         return -EFAULT;
    }

    printk(KERN_DEBUG "%s cmd = 0x%x\n", __func__, cmd);

    switch(cmd) {
        case GTC_IOC_LATCH_CFG:
            ret = copy_from_user(&parm, (const void *)args, _IOC_SIZE(cmd));
            if (ret) {
                return -EFAULT;
            }
            switch (parm) {
            case 0:
                gtc_latch_clear(pbst_gtc->addr, 0x7);
                break;
            default:
                gtc_latch_en_sel(pbst_gtc->addr, parm);
                break;
            }
            printk(KERN_DEBUG "GTC_IOC_LATCH_CFG: %d\n", parm);
            break;
        case GTC_IOC_INTR_CFG:
            ret = copy_from_user(&parm, (const void *)args, _IOC_SIZE(cmd));
            if (ret) {
                return -EFAULT;
            }
            switch (parm) {
            case 0:
                gtc_intr_mask(pbst_gtc->addr, 0);
                break;
            case 1:
                gtc_intr_mask(pbst_gtc->addr, 1);
                break;
            }
            printk(KERN_DEBUG "GTC_IOC_INTR_CFG: %d\n", parm);
            break;
        case GTC_IOC_MUX_CFG:
            ret = copy_from_user(&parm, (const void *)args, _IOC_SIZE(cmd));
            if (ret) {
                return -EFAULT;
            }
            if (parm == 0xff) {
                gtc_mux_config(pbst_gtc->addr, 0x0);
                gtc_syncbits_sel(pbst_gtc->addr, 0x0);
            } else if ((parm >=0 ) && (parm <= 28)) {
                gtc_mux_config(pbst_gtc->addr, parm);
            } else
                ret = -EINVAL;
            printk(KERN_DEBUG "GTC_IOC_MUX_CFG: %d\n", parm);
            break;
        case GTC_IOC_GET_PARM:
            ret = copy_to_user((void __user *)args, &record, _IOC_SIZE(cmd));
            if (ret) {
                return -EFAULT;
            }
            printk(KERN_DEBUG "GTC_IOC_GET_PARM\n");
            break;
        case GTC_IOC_GET_FREQ:
            ret = bst_gtc_get_freq(&freq_info);
            if (ret)
                break;

            ret = copy_to_user((void __user *)args, &freq_info, _IOC_SIZE(cmd));
            if (ret) {
                return -EFAULT;
            }
            printk(KERN_DEBUG "GTC_IOC_GET_FREQ\n");
            break;
        case GTC_IOC_LOG:
            ret = copy_from_user(&parm, (const void *)args, _IOC_SIZE(cmd));
            if (ret) {
                return -EFAULT;
            }
            bstgtc_log = parm;
            printk(KERN_DEBUG "GTC_IOC_LOG %d\n", bstgtc_log);
            break;
        case GTC_IOC_TEST_KTIME:
            ret = copy_from_user(&cnt, (const void *)args, _IOC_SIZE(cmd));
            if (ret) {
                return -EFAULT;
            }
            printk(KERN_DEBUG "GTC_IOC_TEST_KTIME %llu\n", cnt);
            bst_gtc_cnt_to_sys_mono(cnt);         
            break;
        default:
            ret = -EINVAL;
    }

    module_put(THIS_MODULE);
    return ret;
}

static struct file_operations bst_gtc_fops = {
    .owner  = THIS_MODULE,
    .unlocked_ioctl = bst_gtc_ioctl,
};


/*
 * bst_gtc_miscdev_init - gtc dev register
 * @pbst_gtc:  file pointer to the gtc parm structure
 */
int bst_gtc_miscdev_init(struct bst_gtc *pbst_gtc)
{
    int ret;
    char bst_gtc_dev_name[sizeof(BST_GTC_DEV_NAME) + BST_GTC_DEV_ID_LEN];

    snprintf(bst_gtc_dev_name, sizeof(BST_GTC_DEV_NAME) + BST_GTC_DEV_ID_LEN,
        "%s", BST_GTC_DEV_NAME);
    printk(KERN_ERR "gtc device name: %s\n", bst_gtc_dev_name);

    // init & register bst_cv miscdev
    pbst_gtc->miscdev.minor = MISC_DYNAMIC_MINOR;
    pbst_gtc->miscdev.fops = &bst_gtc_fops;
    pbst_gtc->miscdev.name = devm_kstrdup(&pbst_gtc->pdev->dev, bst_gtc_dev_name, GFP_KERNEL);
    pbst_gtc->miscdev.nodename = devm_kstrdup(&pbst_gtc->pdev->dev, bst_gtc_dev_name, GFP_KERNEL);

    ret = misc_register(&pbst_gtc->miscdev);
    if (ret < 0)
        printk(KERN_ERR "%s: gtc miscdev register fail\n", __func__);

    return ret;
}

/*
 * bst_gtc_miscdev_exit - gtc dev unregister
 * @pbst_gtc:  file pointer to the gtc parm structure
 */
void bst_gtc_miscdev_exit(struct bst_gtc *pbst_gtc)
{
    misc_deregister(&pbst_gtc->miscdev);
    printk(KERN_ERR "gtc miscdev deregister success\n");
    return;
}
