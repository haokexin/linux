/*
 * arch/arm/plat-spear/pwm.c
 *
 * ST Microelectronics SPEAr Pulse Width Modulator driver
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <mach/hardware.h>

/* PWM registers and bits definitions */
#define PWMCR			0x00
#define PWMDCR			0x04
#define PWMPCR			0x08
#define SPEAR13XX_PWMMCR	0x3C

#define PWM_EN_MASK		0x1
#define MIN_PRESCALE		0x00
#define MAX_PRESCALE		0x3FFF
#define PRESCALE_SHIFT		2
#define MIN_DUTY		0x0001
#define MAX_DUTY		0xFFFF
#define MAX_PERIOD		0xFFFF
#define MIN_PERIOD		0x0001
#define SPEAR13XX_MIN_PERIOD	0x0000

#define PWM_DEVICE_PER_IP	4
#define PWM_DEVICE_OFFSET	0x10

/* lock for pwm_list */
static DEFINE_SPINLOCK(list_lock);
/* list of all pwm ips available in system */
static LIST_HEAD(pwm_list);

/**
 * struct pwm_device: struct representing pwm device/channel
 *
 * pwmd_id: id of pwm device
 * pwm: pointer to parent pwm ip
 * label: used for storing label passed in pwm_request
 * offset: base address offset from parent pwm mmio_base
 * busy: represents usage status of a pwm device
 * lock: lock specific to a pwm device
 * node: node for adding device to parent pwm's devices list
 *
 * Each pwm IP contains four independent pwm device/channels. Some or all of
 * which may be present in our configuration.
 */
struct pwm_device {
	unsigned pwmd_id;
	struct pwm *pwm;
	const char *label;
	unsigned offset;
	unsigned busy;
	spinlock_t lock;
	struct list_head node;
	u32 csave_reg_PWMCR;
	u32 csave_reg_PWMDCR;
	u32 csave_reg_PWMPCR;
};

/**
 * struct pwm: struct representing pwm ip
 *
 * id: id of pwm ip
 * mmio_base: base address of pwm
 * clk: pointer to clk structure of pwm ip
 * clk_enabled: clock enable status
 * pwmd_enabled: pwm devices enabled
 * pdev: pointer to pdev structure of pwm
 * lock: lock specific to current pwm ip
 * devices: list of devices/childrens of pwm ip
 * node: node for adding pwm to global list of all pwm ips
 */
struct pwm {
	unsigned id;
	void __iomem *mmio_base;
	struct clk *clk;
	int clk_enabled;
	int pwmd_enabled;
	struct platform_device *pdev;
	spinlock_t lock;
	struct list_head devices;
	struct list_head node;
};

/*
 * For SPEAr3xx:
 * period_ns = 10^9 * (PRESCALE + 1) * PV / PWM_CLK_RATE
 * PV = (PWM_CLK_RATE * period_ns)/ (10^9 * (PRESCALE + 1))
 * For SPEAr13xx:
 * period_ns = 10^9 * (PRESCALE + 1) * (PV + 1) / PWM_CLK_RATE
 * PV = ((PWM_CLK_RATE * period_ns)/ (10^9 * (PRESCALE + 1))) - 1
 *
 * duty_ns = 10^9 * (PRESCALE + 1) * DC / PWM_CLK_RATE
 * DC = (PWM_CLK_RATE * duty_ns)/ (10^9 * (PRESCALE + 1))
 */
int pwm_config(struct pwm_device *pwmd, int duty_ns, int period_ns)
{
	u64 val, div, clk_rate;
	unsigned long prescale = MIN_PRESCALE, pv, dc, min_period;
	int ret = -EINVAL;

	if (!pwmd) {
		pr_err("pwm: config - NULL pwm device pointer\n");
		return -EFAULT;
	}

	if (period_ns == 0 || duty_ns > period_ns)
		goto err;

	min_period = cpu_is_spear1340() ? SPEAR13XX_MIN_PERIOD : MIN_PERIOD;

	/* TODO: Need to optimize this loop */
	while (1) {
		div = 1000000000;
		div *= 1 + prescale;
		clk_rate = clk_get_rate(pwmd->pwm->clk);
		val = clk_rate * period_ns;
		pv = div64_u64(val, div);
		if (!pv)
			goto err;

		if (cpu_is_spear1340())
			pv -= 1;

		val = clk_rate * duty_ns;
		dc = div64_u64(val, div);
		if (!dc || dc < MIN_DUTY || pv < min_period)
			goto err;

		if ((pv > MAX_PERIOD) || (dc > MAX_DUTY)) {
			prescale++;
			if (prescale > MAX_PRESCALE)
				goto err;
			continue;
		}
		break;
	}

	/*
	 * NOTE: the clock to PWM has to be enabled first
	 * before writing to the registers
	 */
	spin_lock(&pwmd->pwm->lock);
	ret = clk_enable(pwmd->pwm->clk);
	if (ret) {
		spin_unlock(&pwmd->pwm->lock);
		goto err;
	}

	spin_lock(&pwmd->lock);
	writel(prescale << PRESCALE_SHIFT, pwmd->pwm->mmio_base +
			pwmd->offset + PWMCR);
	writel(dc, pwmd->pwm->mmio_base + pwmd->offset + PWMDCR);
	writel(pv, pwmd->pwm->mmio_base + pwmd->offset + PWMPCR);
	spin_unlock(&pwmd->lock);
	clk_disable(pwmd->pwm->clk);
	spin_unlock(&pwmd->pwm->lock);

	return 0;
err:
	dev_err(&pwmd->pwm->pdev->dev, "pwm config fail\n");
	return ret;
}
EXPORT_SYMBOL(pwm_config);

int pwm_enable(struct pwm_device *pwmd)
{
	int ret = 0;
	u32 val = 0;

	if (!pwmd) {
		pr_err("pwm: enable - NULL pwm device pointer\n");
		return -EFAULT;
	}

	spin_lock(&pwmd->pwm->lock);
	ret = clk_enable(pwmd->pwm->clk);
	if (!ret)
		pwmd->pwm->clk_enabled++;
	else {
		spin_unlock(&pwmd->pwm->lock);
		goto err;
	}

	if (cpu_is_spear1340()) {
		if (!pwmd->pwm->pwmd_enabled)
			writel(1, pwmd->pwm->mmio_base + SPEAR13XX_PWMMCR);
		pwmd->pwm->pwmd_enabled++;
	}

	spin_lock(&pwmd->lock);
	val = readl(pwmd->pwm->mmio_base + pwmd->offset + PWMCR);
	writel(val | PWM_EN_MASK, pwmd->pwm->mmio_base + pwmd->offset + PWMCR);
	spin_unlock(&pwmd->lock);
	spin_unlock(&pwmd->pwm->lock);
	return 0;
err:
	dev_err(&pwmd->pwm->pdev->dev, "pwm enable fail\n");
	return ret;
}
EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwmd)
{
	if (!pwmd) {
		pr_err("pwm: disable - NULL pwm device pointer\n");
		return;
	}

	spin_lock(&pwmd->pwm->lock);
	spin_lock(&pwmd->lock);
	writel(0, pwmd->pwm->mmio_base + pwmd->offset + PWMCR);
	spin_unlock(&pwmd->lock);

	if (cpu_is_spear1340()) {
		pwmd->pwm->pwmd_enabled--;
		if (!pwmd->pwm->pwmd_enabled)
			writel(0, pwmd->pwm->mmio_base + SPEAR13XX_PWMMCR);
	}

	if (pwmd->pwm->clk_enabled) {
		clk_disable(pwmd->pwm->clk);
		pwmd->pwm->clk_enabled--;
	}
	spin_unlock(&pwmd->pwm->lock);
}
EXPORT_SYMBOL(pwm_disable);

struct pwm_device *pwm_request(int pwmd_id, const char *label)
{
	int found = 0;
	struct pwm *pwm;
	struct pwm_device *pwmd = NULL;

	spin_lock(&list_lock);
	list_for_each_entry(pwm, &pwm_list, node) {
		spin_lock(&pwm->lock);
		list_for_each_entry(pwmd, &pwm->devices, node) {
			if (pwmd->pwmd_id == pwmd_id) {
				found = 1;
				break;
			}
		}
		spin_unlock(&pwm->lock);
		if (found)
			break;
	}
	spin_unlock(&list_lock);

	if (found) {
		spin_lock(&pwmd->lock);
		if (pwmd->busy == 0) {
			pwmd->busy++;
			pwmd->label = label;
		} else
			pwmd = ERR_PTR(-EBUSY);
		spin_unlock(&pwmd->lock);
	} else
		pwmd = ERR_PTR(-ENOENT);

	if (IS_ERR(pwmd))
		pr_err("pwm: request fail\n");

	return pwmd;
}
EXPORT_SYMBOL(pwm_request);

void pwm_free(struct pwm_device *pwmd)
{
	if (!pwmd) {
		pr_err("pwm: disable - NULL pwm device pointer\n");
		return;
	}

	spin_lock(&pwmd->lock);
	if (pwmd->busy) {
		pwmd->busy--;
		pwmd->label = NULL;
	} else {
		spin_unlock(&pwmd->lock);
		dev_warn(&pwmd->pwm->pdev->dev, "pwm device already freed\n");
		return;
	}

	spin_unlock(&pwmd->lock);
}
EXPORT_SYMBOL(pwm_free);

/* creates and add pwmd device to parent pwm's devices list */
static int add_pwm_device(unsigned int pwmd_id, struct pwm *pwm)
{
	struct pwm_device *pwmd;

	pwmd = kzalloc(sizeof(*pwmd), GFP_KERNEL);
	if (!pwmd)
		return -ENOMEM;

	pwmd->pwm = pwm;
	pwmd->busy = 0;
	pwmd->pwmd_id = pwmd_id + pwm->id * PWM_DEVICE_PER_IP;
	pwmd->offset = pwmd_id * PWM_DEVICE_OFFSET;
	spin_lock_init(&pwmd->lock);

	spin_lock(&pwm->lock);
	list_add_tail(&pwmd->node, &pwm->devices);
	spin_unlock(&pwm->lock);

	return 0;
}

/* removes all pwmd devices from parent pwm's devices list */
static void remove_pwm_devices(struct pwm *pwm)
{
	struct pwm_device *pwmd;

	spin_lock(&pwm->lock);
	list_for_each_entry(pwmd, &pwm->devices, node) {
		list_del(&pwmd->node);
		kfree(pwmd);
	}
	spin_unlock(&pwm->lock);
}

static int __devinit spear_pwm_probe(struct platform_device *pdev)
{
	struct pwm *pwm = NULL;
	struct resource *res;
	int ret = 0, pwmd_id = 0;

	pwm = kzalloc(sizeof(*pwm), GFP_KERNEL);
	if (!pwm) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "failed to allocate memory\n");
		goto err;
	}

	pwm->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(pwm->clk)) {
		ret = PTR_ERR(pwm->clk);
		dev_dbg(&pdev->dev, "Error getting clock\n");
		goto err_free;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		dev_dbg(&pdev->dev, "no memory resource defined\n");
		goto err_free_clk;
	}

	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
		ret = -EBUSY;
		dev_dbg(&pdev->dev, "failed to request memory resource\n");
		goto err_free_clk;
	}

	pwm->mmio_base = ioremap(res->start, resource_size(res));
	if (!pwm->mmio_base) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "failed to ioremap\n");
		goto err_free_mem;
	}

	/* initialize pwm structure */
	pwm->clk_enabled = 0;
	pwm->pdev = pdev;
	/* if pdev->id is -1, only one pwm ip is present */
	if (pdev->id == -1)
		pwm->id = 0;
	else
		pwm->id = pdev->id;

	spin_lock_init(&pwm->lock);
	INIT_LIST_HEAD(&pwm->devices);
	platform_set_drvdata(pdev, pwm);

	/* add pwm to pwm list */
	spin_lock(&list_lock);
	list_add_tail(&pwm->node, &pwm_list);
	spin_unlock(&list_lock);

	/* add pwm devices */
	for (pwmd_id = 0; pwmd_id < PWM_DEVICE_PER_IP; pwmd_id++) {
		ret = add_pwm_device(pwmd_id, pwm);
		if (!ret)
			continue;
		dev_err(&pdev->dev, "Add device fail for pwm device id: %d\n",
				pwmd_id);
	}

	if (list_empty(&pwm->node))
		goto err_remove_pwm;

	dev_info(&pdev->dev, "Initialization successful\n");
	return 0;

err_remove_pwm:
	spin_lock(&list_lock);
	list_del(&pwm->node);
	spin_unlock(&list_lock);

	platform_set_drvdata(pdev, NULL);
	iounmap(pwm->mmio_base);
err_free_mem:
	release_mem_region(res->start, resource_size(res));
err_free_clk:
	clk_put(pwm->clk);
err_free:
	kfree(pwm);
err:
	dev_err(&pdev->dev, "Initialization Fail. Error: %d\n", ret);

	return ret;
}

static int __devexit spear_pwm_remove(struct platform_device *pdev)
{
	struct pwm *pwm;
	struct resource *res;
	int ret = 0;

	pwm = platform_get_drvdata(pdev);
	if (pwm == NULL) {
		ret = -ENODEV;
		dev_dbg(&pdev->dev, "Remove: get_drvdata fail\n");
		goto err;
	}
	platform_set_drvdata(pdev, NULL);

	/* remove pwm devices */
	remove_pwm_devices(pwm);

	/* remove pwm from pwm_list */
	spin_lock(&list_lock);
	list_del(&pwm->node);
	spin_unlock(&list_lock);

	iounmap(pwm->mmio_base);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		dev_dbg(&pdev->dev, "Remove: get_resource fail\n");
		goto err;
	}
	release_mem_region(res->start, resource_size(res));

	if (pwm->clk_enabled)
		clk_disable(pwm->clk);
	clk_put(pwm->clk);

	kfree(pwm);
	return 0;

err:
	dev_err(&pdev->dev, "Remove: Fail - %d\n", ret);
	return ret;
}

#ifdef CONFIG_PM
static int pwm_suspend(struct device *dev, bool csave)
{
	struct pwm *pwm = dev_get_drvdata(dev);
	struct pwm_device *pwmd;

	if (cpu_is_spear1340()) {
		if (pwm->pwmd_enabled)
			writel(0, pwm->mmio_base + SPEAR13XX_PWMMCR);
	}

	if (!csave)
		goto sus_clk_dis;

	list_for_each_entry(pwmd, &pwm->devices, node) {
		if (pwmd->busy) {
			pwmd->csave_reg_PWMCR =
				readl(pwm->mmio_base + pwmd->offset + PWMCR);
			pwmd->csave_reg_PWMDCR =
				readl(pwm->mmio_base + pwmd->offset + PWMDCR);
			pwmd->csave_reg_PWMPCR =
				readl(pwm->mmio_base + pwmd->offset + PWMPCR);
		}
	}

sus_clk_dis:
	/* disable clock */
	if (pwm->clk_enabled)
		clk_disable(pwm->clk);

	return 0;
}

static int spear_pwm_suspend(struct device *dev)
{
	return pwm_suspend(dev, true);
}

static int spear_pwm_poweroff(struct device *dev)
{
	return pwm_suspend(dev, false);
}

static int pwm_resume(struct device *dev, bool csave)
{
	struct pwm *pwm = dev_get_drvdata(dev);
	struct pwm_device *pwmd;
	int ret = 0;

	if (pwm->clk_enabled) {
		ret = clk_enable(pwm->clk);
		if (ret) {
			dev_err(dev, "clk enable failed%d\n", ret);
			return ret;
		}
	}

	if (!csave)
		goto res_pwm_enable;

	list_for_each_entry(pwmd, &pwm->devices, node) {
		if (pwmd->busy) {
			writel(pwmd->csave_reg_PWMDCR,
					pwm->mmio_base + pwmd->offset + PWMDCR);
			writel(pwmd->csave_reg_PWMPCR,
					pwm->mmio_base + pwmd->offset + PWMPCR);
			writel(pwmd->csave_reg_PWMCR,
					pwm->mmio_base + pwmd->offset + PWMCR);
		}
	}

res_pwm_enable:
	if (cpu_is_spear1340()) {
		if (pwm->pwmd_enabled)
			writel(1, pwm->mmio_base + SPEAR13XX_PWMMCR);
	}

	return ret;
}

static int spear_pwm_resume(struct device *dev)
{
	return pwm_resume(dev, true);
}

static int spear_pwm_thaw(struct device *dev)
{
	return pwm_resume(dev, false);
}

static const struct dev_pm_ops spear_pwm_dev_pm_ops = {
	.suspend = spear_pwm_suspend,
	.resume = spear_pwm_resume,
	.freeze = spear_pwm_suspend,
	.thaw = spear_pwm_thaw,
	.poweroff = spear_pwm_poweroff,
	.restore = spear_pwm_resume,
};

#define SPEAR_PWM_DEV_PM_OPS (&spear_pwm_dev_pm_ops)
#else
#define SPEAR_PWM_DEV_PM_OPS NULL
#endif /* CONFIG_PM */

static struct platform_driver spear_pwm_driver = {
	.driver = {
		.name = "pwm",
		.bus = &platform_bus_type,
		.owner = THIS_MODULE,
		.pm = SPEAR_PWM_DEV_PM_OPS,
	},
	.probe = spear_pwm_probe,
	.remove = __devexit_p(spear_pwm_remove)
};

static int __init spear_pwm_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&spear_pwm_driver);
	if (ret)
		pr_err("failed to register spear_pwm_driver\n");

	return ret;
}
module_init(spear_pwm_init);

static void __exit spear_pwm_exit(void)
{
	platform_driver_unregister(&spear_pwm_driver);
}
module_exit(spear_pwm_exit);

MODULE_AUTHOR("Viresh Kumar");
MODULE_DESCRIPTION("SPEAr PWM Driver");
MODULE_LICENSE("GPL");
