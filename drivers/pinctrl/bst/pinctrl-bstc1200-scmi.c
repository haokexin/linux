// SPDX-License-Identifier: GPL-2.0
/*
 * System Control and Power Interface (SCMI) Protocol based clock driver
 *
 * Copyright (C) 2021 EPAM.
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/scmi_protocol.h>
#include <linux/slab.h>

static const struct scmi_pinctrl_ops *pinctrl_ops;

#define DRV_NAME "scmi-pinctrl"
#define DT_PROPERTY_NAME_BUF_MAX 32

struct scmi_pinctrl_funcs {
	unsigned num_groups;
	const char **groups;
};

struct scmi_pinctrl {
	struct device *dev;
	struct scmi_protocol_handle *ph;
	struct pinctrl_dev *pctldev;
	struct pinctrl_desc pctl_desc;
	struct scmi_pinctrl_funcs *functions;
	unsigned int nr_functions;
	char **groups;
	unsigned int nr_groups;
	struct pinctrl_pin_desc *pins;
	unsigned nr_pins;
};

typedef struct pin_name_ {
	char name[16];
}pin_name_s;

static struct scmi_pinctrl *pmx;

static int pinctrl_scmi_get_groups_count(struct pinctrl_dev *pctldev)
{
	const struct scmi_protocol_handle *ph = pmx->ph;

	return pinctrl_ops->get_groups_count(ph);
}

static const char *pinctrl_scmi_get_group_name(struct pinctrl_dev *pctldev,
					 unsigned selector)
{
	int ret;
	const char *name;
	const struct scmi_protocol_handle *ph = pmx->ph;

	ret = pinctrl_ops->get_group_name(ph, selector, &name);
	if (ret) {
		dev_err(pmx->dev, "get name failed with err %d", ret);
		return "";
	}

	return name;
}

static int pinctrl_scmi_get_group_pins(struct pinctrl_dev *pctldev,
				unsigned selector, const unsigned **pins, unsigned *num_pins)
{
	const struct scmi_protocol_handle *ph = pmx->ph;

	return pinctrl_ops->get_group_pins(ph, selector,
											   pins, num_pins);
}

static void pinctrl_scmi_pin_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
				unsigned offset)
{
	seq_puts(s, DRV_NAME);
}

const char *int_to_str_alloc(unsigned int param)
{
	char buf[DT_PROPERTY_NAME_BUF_MAX];
	char *res;
	int size;

	size = snprintf(buf, DT_PROPERTY_NAME_BUF_MAX, "%u", param);
	if (!size)
		return NULL;

	res = kmemdup(buf, size + 1, GFP_KERNEL);
	return res;
}

static void str_from_int_free(const char *addr)
{
	if (likely(addr))
		kfree(addr);
}

static const struct pinctrl_ops pinctrl_scmi_pinctrl_ops = {
	.get_groups_count	= pinctrl_scmi_get_groups_count,
	.get_group_name		= pinctrl_scmi_get_group_name,
	.get_group_pins		= pinctrl_scmi_get_group_pins,
	.pin_dbg_show		= pinctrl_scmi_pin_dbg_show,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_group,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static int pinctrl_scmi_get_functions_count(struct pinctrl_dev *pctldev)
{
	const struct scmi_protocol_handle *ph = pmx->ph;

	return pinctrl_ops->get_functions_count(ph);
}

static const char *pinctrl_scmi_get_function_name(struct pinctrl_dev *pctldev,
					    unsigned selector)
{
	int ret;
	const char *name;
	const struct scmi_protocol_handle *ph = pmx->ph;

	ret = pinctrl_ops->get_function_name(ph, selector, &name);
	if (ret) {
		dev_err(pmx->dev, "get name failed with err %d", ret);
		return "";
	}

	return name;
}

static int pinctrl_scmi_get_function_groups(struct pinctrl_dev *pctldev,
				      unsigned selector,
				      const char * const **groups,
				      unsigned * const num_groups)
{
	const void *group_ids;
	const struct scmi_protocol_handle *ph = pmx->ph;
	int ret, i;
	pin_name_s *p;
	
	if ((selector < pmx->nr_functions)
		&& (pmx->functions[selector].num_groups)) {
		dev_dbg(pmx->dev, "1");
		*groups = (const char * const *)pmx->functions[selector].groups;
		*num_groups = pmx->functions[selector].num_groups;
		return 0;
	}

	ret = pinctrl_ops->get_function_groups(ph, selector,
					&pmx->functions[selector].num_groups, &group_ids);
	if (ret) {
		dev_err(pmx->dev, "Unable to get function groups, err %d", ret);
		return ret;
	}

	p = (pin_name_s *)group_ids;

	*num_groups = pmx->functions[selector].num_groups;

	pmx->functions[selector].groups = devm_kzalloc(pmx->dev,
			sizeof(*pmx->functions[selector].groups) * *num_groups,
			GFP_KERNEL);
	if (unlikely(!pmx->functions[selector].groups))
		return -ENOMEM;

	for (i = 0; i < *num_groups; i++) {
		pmx->functions[selector].groups[i] = p[i].name;

		if (unlikely(!pmx->functions[selector].groups[i])) {
			ret = -ENOMEM;
			goto error;
		}
	}

	*groups = (const char * const *)pmx->functions[selector].groups;
	dev_dbg(pmx->dev, "got groups %d", *num_groups);

	return 0;

error:
	if (pmx->functions[selector].num_groups) {
		for (i = 0; i < pmx->functions[selector].num_groups; i++) {
			if (pmx->functions[selector].groups[i])
				str_from_int_free(pmx->functions[selector].groups[i]);
		}

		kfree(pmx->functions[selector].groups);
	}

	return ret;
}

static int pinctrl_scmi_func_set_mux(struct pinctrl_dev *pctldev,
				      unsigned selector, unsigned group)
{
	const struct scmi_protocol_handle *ph = pmx->ph;

	return pinctrl_ops->set_mux(ph, selector, group);
}

static const struct pinmux_ops pinctrl_scmi_pinmux_ops = {
	.get_functions_count	= pinctrl_scmi_get_functions_count,
	.get_function_name	= pinctrl_scmi_get_function_name,
	.get_function_groups	= pinctrl_scmi_get_function_groups,
	.set_mux		= pinctrl_scmi_func_set_mux,
};

static int pinctrl_scmi_pinconf_get(struct pinctrl_dev *pctldev, unsigned _pin,
			      unsigned long *config)
{
	const struct scmi_protocol_handle *ph = pmx->ph;

	return pinctrl_ops->get_config(ph, _pin, (u32 *)config);
}

static int pinctrl_scmi_pinconf_set(struct pinctrl_dev *pctldev, unsigned _pin,
			      unsigned long *configs, unsigned num_configs)
{
	const struct scmi_protocol_handle *ph = pmx->ph;
	int i, ret;

	dev_dbg(pmx->dev, "Enter pin = %d, num_configs = %d\n", _pin, num_configs);

	for (i=0; i<num_configs; i++) {
		ret = pinctrl_ops->set_config(ph, _pin, configs[i]);
		if (ret) {
			dev_err(pmx->dev, "Error parsing config %ld\n", configs[i]);
			break;
		}
	}

	return ret;
}

static int pinctrl_scmi_pinconf_group_set(struct pinctrl_dev *pctldev,
				    unsigned group,
				    unsigned long *configs,
				    unsigned num_configs)
{
	const struct scmi_protocol_handle *ph = pmx->ph;
	int i, ret;

	for (i=0; i<num_configs; i++) {
		ret = pinctrl_ops->set_config_group(ph, group, configs[i]);
		if (ret) {
			dev_err(pmx->dev, "Error parsing config = %ld", configs[i]);
			break;
		}
	}

	return ret;
};

static int pinctrl_scmi_pinconf_group_get(struct pinctrl_dev *pctldev,
				    unsigned group,
				    unsigned long *configs)
{
	const struct scmi_protocol_handle *ph = pmx->ph;

	return pinctrl_ops->get_config_group(ph, group, (u32 *)configs);
};

static const struct pinconf_ops pinctrl_scmi_pinconf_ops = {
	.is_generic			= true,
	.pin_config_get			= pinctrl_scmi_pinconf_get,
	.pin_config_set			= pinctrl_scmi_pinconf_set,
	.pin_config_group_set		= pinctrl_scmi_pinconf_group_set,
	.pin_config_group_get       = pinctrl_scmi_pinconf_group_get,
//	.pin_config_config_dbg_show	= pinconf_generic_dump_config,
};


static int pinctrl_scmi_get_pins(struct scmi_protocol_handle *ph,
								 unsigned *nr_pins,
								 const struct pinctrl_pin_desc **pins)
{
	const void *pin_ids;
	pin_name_s *p;
	int ret, i;
	
	if (pmx->nr_pins) {
		*pins = pmx->pins;
		*nr_pins = pmx->nr_pins;
		return 0;
	}

	ret = pinctrl_ops->get_pins(ph, nr_pins, &pin_ids);
	if (ret) {
		dev_err(pmx->dev, "get pins failed with err %d", ret);
		return ret;
	}

	p = (pin_name_s *)pin_ids;

	pmx->nr_pins = *nr_pins;
	pmx->pins = devm_kzalloc(pmx->dev, sizeof(*pmx->pins) * *nr_pins,
							 GFP_KERNEL);
	if (unlikely(!pmx->pins))
		return -ENOMEM;

	for (i = 0; i < *nr_pins; i++) {
		pmx->pins[i].number = i;
		pmx->pins[i].name = (p[i].name);

	}

	*pins = pmx->pins;
	dev_dbg(pmx->dev, "got pins %d", *nr_pins);


	return 0;
}

static const struct scmi_device_id scmi_id_table[] = {
	{ SCMI_PROTOCOL_PINCTRL, "pinctrl" },
	{ },
};
MODULE_DEVICE_TABLE(scmi, scmi_id_table);

static int scmi_pinctrl_probe(struct scmi_device *sdev)
{
	int ret;
	const struct scmi_handle *handle = sdev->handle;
	struct scmi_protocol_handle *ph;

	pinctrl_ops = handle->devm_protocol_get(sdev, SCMI_PROTOCOL_PINCTRL, &ph);
	if (IS_ERR(pinctrl_ops))
		return PTR_ERR(pinctrl_ops);

	pmx = devm_kzalloc(&sdev->dev, sizeof(*pmx), GFP_KERNEL);
	if (unlikely(!pmx))
		return -ENOMEM;

	pmx->ph = ph;
	if (unlikely(!pmx->ph)) {
		ret = -ENOMEM;
		goto clean;
	}

	pmx->dev = &sdev->dev;
	pmx->pctl_desc.name = DRV_NAME;
	pmx->pctl_desc.owner = THIS_MODULE;
	pmx->pctl_desc.pctlops = &pinctrl_scmi_pinctrl_ops;
	pmx->pctl_desc.pmxops = &pinctrl_scmi_pinmux_ops;
	pmx->pctl_desc.confops = &pinctrl_scmi_pinconf_ops;

	ret = pinctrl_scmi_get_pins(pmx->ph, &pmx->pctl_desc.npins,
								&pmx->pctl_desc.pins);
	if (ret)
		goto clean;

	ret = devm_pinctrl_register_and_init(&sdev->dev, &pmx->pctl_desc, pmx,
					     &pmx->pctldev);
	if (ret) {
		dev_err(&sdev->dev, "could not register: %i\n", ret);
		goto clean;
	}

	pmx->nr_functions = pinctrl_scmi_get_functions_count(pmx->pctldev);
	pmx->nr_groups = pinctrl_scmi_get_groups_count(pmx->pctldev);

	if (pmx->nr_functions) {
		pmx->functions = devm_kzalloc(&sdev->dev, sizeof(*pmx->functions) *
								  pmx->nr_functions, GFP_KERNEL);
		if (unlikely(!pmx->functions)) {
			ret = -ENOMEM;
			goto clean;
		}
	}

	if (pmx->nr_groups) {
		pmx->groups = devm_kzalloc(&sdev->dev, sizeof(*pmx->groups) *
							   pmx->nr_groups, GFP_KERNEL);
		if (unlikely(!pmx->groups)) {
			ret = -ENOMEM;
			goto clean;
		}
	}

	return pinctrl_enable(pmx->pctldev);

clean:
	if (pmx) {
		if (pmx->functions)
			kfree(pmx->functions);

		if (pmx->groups)
			kfree(pmx->groups);

		kfree(pmx);
	}

	return 0;
}

static struct scmi_driver scmi_pinctrl_driver = {
	.name = DRV_NAME,
	.probe = scmi_pinctrl_probe,
	.id_table = scmi_id_table,
};
module_scmi_driver(scmi_pinctrl_driver);

MODULE_AUTHOR("Oleksii Moisieiev <oleksii_moisieiev@epam.com>");
MODULE_DESCRIPTION("ARM SCMI pin controller driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
