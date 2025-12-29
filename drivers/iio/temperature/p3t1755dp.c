// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i3c/device.h>
#include <linux/i3c/master.h>
#include <linux/slab.h>
#include <linux/regmap.h>


#define P3T1755DP_ID                        0x39
#define P3T1755DP_TEMP                      0x0
#define P3T1755DP_CONF                      0x1
#define P3T1755DP_TLOW                      0x2
#define P3T1755DP_THIGH                     0x3


struct p3t1755dp_dev {
    struct i3c_device *i3cdev;
    struct regmap *regmap;
    struct mutex lock;
};

static const struct i3c_device_id nxp_p3t1755dp_i3c_ids[] = {
        I3C_DEVICE(0x011b, 0x152a, (void *)P3T1755DP_ID),
        { /* sentinel */ },
};
MODULE_DEVICE_TABLE(i3c, nxp_p3t1755dp_i3c_ids);

static int p3t1755dp_temp_read_raw(struct iio_dev *indio_dev,
                                struct iio_chan_spec const *chan,
                                int *val, int *val2, long mask)
{
        struct p3t1755dp_dev *p3t1755_dt = iio_priv(indio_dev);
        int ret;
        u16 val_buf, temp;
        
        switch (mask) {
        case IIO_CHAN_INFO_RAW:
                mutex_lock(&p3t1755_dt->lock);
                ret = regmap_raw_read(p3t1755_dt->regmap, P3T1755DP_TEMP,
                                      &val_buf, sizeof(val_buf));
                if (ret)
                        return ret;

                temp = (val_buf >> 8) | (val_buf << 8);
                if(temp & 0x8000)
                        *val = 0 - ((~ temp) >> 4)*5/80;
                else
                        *val = (temp>>4)*5/80;
                mutex_unlock(&p3t1755_dt->lock);
                return IIO_VAL_INT;

        default:
                return -EINVAL;
        }
}
static const struct iio_info p3t1755dp_temp_info = {
        .read_raw = &p3t1755dp_temp_read_raw,
};

static const struct iio_chan_spec p3t1755dp_temp_channels[] = {
        {
                .type = IIO_TEMP,
                .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) ,
        },
};

#if 1
static struct iio_dev *p_iiodev[2];
static unsigned int p3t1755dp_reg;
static unsigned int p3t1755dp_bus = 0;
static struct kobject *p3t1755dp_kobj;
static int call_num = 0;

ssize_t p3t1755dp_val_show(struct kobject *object,
			     struct kobj_attribute *attr, char *buf)
{
	int ret;
        u16 val_buf;
        struct p3t1755dp_dev *p3t1755_dt = iio_priv(p_iiodev[p3t1755dp_bus]);
        mutex_lock(&p3t1755_dt->lock);
        ret = regmap_raw_read(p3t1755_dt->regmap, p3t1755dp_reg,
                                      &val_buf, sizeof(val_buf));

	ret = sprintf(buf,"0x%x\n", val_buf);
	ret = strlen(buf);
        mutex_unlock(&p3t1755_dt->lock);        
        // pr_err("%s %d bus: %d reg: %x  val: %x", __func__, __LINE__, p3t1755dp_bus, p3t1755dp_reg, val_buf);
	return ret > PAGE_SIZE ? PAGE_SIZE : ret;
}

ssize_t p3t1755dp_val_store(struct kobject *object,
			      struct kobj_attribute *attr, const char *buf,
			      size_t count)
{
	unsigned long value;
	int ret;
        u16 val_buf;
        struct p3t1755dp_dev *p3t1755_dt = iio_priv(p_iiodev[p3t1755dp_bus]);
        mutex_lock(&p3t1755_dt->lock);
	if (kstrtoul(buf, 10, &value))
		return -EINVAL;
        val_buf = value;
        // pr_err("%s %d bus: %d reg: %x  val: %x", __func__, __LINE__, p3t1755dp_bus, p3t1755dp_reg, val_buf);

        ret = regmap_raw_write(p3t1755_dt->regmap, p3t1755dp_reg,
                                      &val_buf, sizeof(val_buf));
        mutex_unlock(&p3t1755_dt->lock);

	return count;
}

ssize_t p3t1755dp_reg_show(struct kobject *object,
			     struct kobj_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf,"current reg: 0x%x\n", p3t1755dp_reg);
	ret = strlen(buf);

	return ret > PAGE_SIZE ? PAGE_SIZE : ret;
}

ssize_t p3t1755dp_reg_store(struct kobject *object,
			      struct kobj_attribute *attr, const char *buf,
			      size_t count)
{
	unsigned long value;

	if (kstrtoul(buf, 10, &value))
		return -EINVAL;

	p3t1755dp_reg = value;

	return count;
}

ssize_t p3t1755dp_bus_show(struct kobject *object,
			     struct kobj_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf,"current bus: %d\n", p3t1755dp_bus);
	ret = strlen(buf);

	return ret > PAGE_SIZE ? PAGE_SIZE : ret;
}

ssize_t p3t1755dp_bus_store(struct kobject *object,
			      struct kobj_attribute *attr, const char *buf,
			      size_t count)
{
	unsigned long value;

	if (kstrtoul(buf, 10, &value))
		return -EINVAL;

	p3t1755dp_bus = value;

	return count;
}

static struct kobj_attribute s_p3t1755dp_val_attribute = __ATTR(
	p3t1755dp_val, 0664, p3t1755dp_val_show, p3t1755dp_val_store);
static struct kobj_attribute s_p3t1755dp_reg_attribute = __ATTR(
	p3t1755dp_reg, 0664, p3t1755dp_reg_show, p3t1755dp_reg_store);
static struct kobj_attribute s_p3t1755dp_bus_attribute = __ATTR(
	p3t1755dp_bus, 0664, p3t1755dp_bus_show, p3t1755dp_bus_store);

static struct attribute *attrs[] = {
	&s_p3t1755dp_val_attribute.attr, 
	&s_p3t1755dp_reg_attribute.attr, 
	&s_p3t1755dp_bus_attribute.attr, 
	NULL, /* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

int p3t1755dp_temp_register_sysfs(struct iio_dev *indio_dev)
{
	int ret = -1;

	if(!p3t1755dp_kobj){
                p3t1755dp_kobj = kobject_create_and_add("p3t1755dp", NULL);
                if (!p3t1755dp_kobj) {
                        pr_err("kobject_create_and_add failed");
                        return -ENOMEM;
                }

                /* Create the files associated with this kobject */
                ret = sysfs_create_group(p3t1755dp_kobj, &attr_group);
                if (ret) {
                        pr_err("sysfs_create_group failed, error code = %d", ret);
                        kobject_put(p3t1755dp_kobj);
                }
        }
        p_iiodev[call_num] = indio_dev;
	call_num++;
        return ret;
}
#endif


static int p3t1755dp_temp_probe(struct i3c_device *i3cdev)
{
        struct p3t1755dp_dev *p3t1755_dt;
        struct iio_dev *indio_dev;

        struct regmap_config p3t1755dp_i3c_regmap_config = {
                .reg_bits = 8,
                .val_bits = 16,
        };
        struct regmap *regmap;
        
        dev_err(&i3cdev->dev, "%s %d  +++\n", __func__, __LINE__);
        regmap = devm_regmap_init_i3c(i3cdev, &p3t1755dp_i3c_regmap_config);
        if (IS_ERR(regmap)) {
                dev_err(&i3cdev->dev, "Failed to register i3c regmap %ld\n", PTR_ERR(regmap));
                return PTR_ERR(regmap);
        }

        indio_dev = devm_iio_device_alloc(&i3cdev->dev, sizeof(*p3t1755_dt));
        if (!indio_dev)
                return -ENOMEM;

        p3t1755_dt = iio_priv(indio_dev);
        p3t1755_dt->regmap = regmap;
        p3t1755_dt->i3cdev = i3cdev;
        mutex_init(&p3t1755_dt->lock);

        indio_dev->modes = INDIO_DIRECT_MODE;
        indio_dev->channels = p3t1755dp_temp_channels;
        indio_dev->num_channels = ARRAY_SIZE(p3t1755dp_temp_channels);
        indio_dev->name = "nxp_p3t1755dp_i3c";
        indio_dev->info = &p3t1755dp_temp_info;

        p3t1755dp_temp_register_sysfs(indio_dev);
        return devm_iio_device_register(&i3cdev->dev, indio_dev);
}

static void p3t1755dp_temp_remove(struct i3c_device *i3cdev)
{
	call_num--;
        if(call_num == 0)
		if(p3t1755dp_kobj){
		       	kobject_put(p3t1755dp_kobj);
			p3t1755dp_kobj = 0;
		}
        return; 
}

static const struct of_device_id nxp_p3t1755dp_of_match[] = {
        { .compatible = "nxp,p3t1755dp", },
        { },
};
MODULE_DEVICE_TABLE(of, nxp_p3t1755dp_of_match);

static struct i3c_driver nxp_p3t1755dp_driver = {
        .driver = {
                .name = "nxp_p3t1755dp_i3c",
                .of_match_table = nxp_p3t1755dp_of_match,
        },
        .probe = p3t1755dp_temp_probe,
        .remove = p3t1755dp_temp_remove,
        .id_table = nxp_p3t1755dp_i3c_ids,
};
module_i3c_driver(nxp_p3t1755dp_driver);

MODULE_DESCRIPTION("NXP p3t1755dp temperature sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
