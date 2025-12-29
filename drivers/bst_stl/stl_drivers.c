#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/io.h>
#define DEVICE_NAME "stl"
#define CLASS_NAME  "smc"
#define BUFFER_SIZE 256
#define ATF_EL3_SMC_FUNC_ID 0xc2000006
#define OPTEE_EL1_SMC_FUNC_ID    0xF2000006
typedef int (*stl_a78ae_test_func)(unsigned long *arg_list, int argc);
stl_a78ae_test_func stl_a78ae_test=NULL;
static int major;
static struct class *smc_class = NULL;
static struct device *smc_device_dev = NULL;




struct smc_device {
    struct cdev cdev;
    char buffer[BUFFER_SIZE];
    int buffer_len;
};
struct smc_device *smc_devicep;

static inline uint64_t smc_call(uint64_t function_id) {
    register uint64_t x0 asm("x0") = function_id;

       asm volatile (
        "smc #0"
        : "+r"(x0)
        :
        : "memory"
    );

    return x0;
}

static int device_open(struct inode *inode, struct file *file) {

    file->private_data = smc_devicep;
    return 0;
}

static ssize_t device_read(struct file *filp, char *user_buffer, size_t length, loff_t *offset) {
    struct smc_device *smc_devicep = filp->private_data;
    int ret;

    if (*offset >= smc_devicep->buffer_len)
        return 0;

    if (length > smc_devicep->buffer_len - *offset)
        length = smc_devicep->buffer_len - *offset;

    ret = copy_to_user(user_buffer, smc_devicep->buffer + *offset, length);
    if (ret) {
        return -EFAULT;
    }

    *offset += length;
    return length;
}



static ssize_t device_write(struct file *filp, const char *user_buffer, size_t length, loff_t *offset) {
    
    struct smc_device *smc_devicep = filp->private_data;
    uint64_t smc_result_el1 = 0;
    uint64_t smc_result_el2 = 0;
    uint64_t smc_result_el3 = 0;
    int ret;
    unsigned long arg_list2[] = {2, 2, 2};
    int argc = 3;
    
    ret = copy_from_user(smc_devicep->buffer, user_buffer, length);
    if (ret) {
        return -EFAULT;
    }
    smc_devicep->buffer[length] = '\0'; 
    smc_devicep->buffer_len = length;

    smc_result_el1 = smc_call(OPTEE_EL1_SMC_FUNC_ID);
    smc_result_el2 = stl_a78ae_test(arg_list2, argc);
    smc_result_el3 = smc_call(ATF_EL3_SMC_FUNC_ID);

    
    if (smc_result_el1 == 1 && smc_result_el2 == 1 && smc_result_el3 == 1) {
        smc_devicep->buffer[0] = '1';
    } else {
        smc_devicep->buffer[0] = '0'; 
    }
    smc_devicep->buffer[1] = '\n'; 
    smc_devicep->buffer[2] = '\0';
    smc_devicep->buffer_len = 2;   

    printk("smc_result_el1 is %lld\n",smc_result_el1);
    printk("smc_result_el2 is %lld\n",smc_result_el2);
    printk("smc_result_el3 is %lld\n",smc_result_el3);
    return length;
}

static int device_release(struct inode *inode, struct file *file) {
    return 0;
}


static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = device_open,
    .read = device_read,
    .write = device_write,
    .release = device_release,
};

static int __init smc_device_init(void) {
    dev_t dev;
    int ret;

    ret = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("alloc_chrdev_region failed\n");
        return ret;
    }
    major = MAJOR(dev);

    smc_devicep = kzalloc(sizeof(struct smc_device), GFP_KERNEL);
    if (!smc_devicep) {
        pr_err("kzalloc failed\n");
        unregister_chrdev_region(dev, 1);
        return -ENOMEM;
    }

    cdev_init(&smc_devicep->cdev, &fops);
    smc_devicep->cdev.owner = THIS_MODULE;
    ret = cdev_add(&smc_devicep->cdev, dev, 1);
    if (ret) {
        pr_err("cdev_add failed\n");
        kfree(smc_devicep);
        unregister_chrdev_region(dev, 1);
        return ret;
    }

    smc_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(smc_class)) {
        pr_err("class_create failed\n");
        cdev_del(&smc_devicep->cdev);
        kfree(smc_devicep);
        unregister_chrdev_region(dev, 1);
        return PTR_ERR(smc_class);
    }

    smc_device_dev = device_create(smc_class, NULL, dev, NULL, DEVICE_NAME);
    if (IS_ERR(smc_device_dev)) {
        pr_err("device_create failed\n");
        class_destroy(smc_class);
        cdev_del(&smc_devicep->cdev);
        kfree(smc_devicep);
        unregister_chrdev_region(dev, 1);
        return PTR_ERR(smc_device_dev);
    }
    return 0;
}

static void __exit smc_device_exit(void) {
    dev_t dev = MKDEV(major, 0);
    device_destroy(smc_class, dev);
    cdev_del(&smc_devicep->cdev);
    class_unregister(smc_class);
    kfree(smc_devicep);
    unregister_chrdev_region(dev, 1);
    pr_info("smc_device unregistered\n");
}

module_init(smc_device_init);
module_exit(smc_device_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("A character device driver for executing SMC calls and returning the result");

