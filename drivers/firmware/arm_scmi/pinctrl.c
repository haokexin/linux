// SPDX-License-Identifier: GPL-2.0
/*
 * System Control and Management Interface (SCMI) Pinctrl Protocol
 *
 * Copyright (C) 2021 EPAM.
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#define pr_fmt(fmt) "SCMI Notifications PINCTRL - " fmt
#include <linux/io.h>
#include <linux/scmi_protocol.h>
#include <linux/delay.h>
#include "common.h"
#include "notify.h"
#define SCMI_PINCTRL_MAX_PINS_CNT 512
#define SCMI_PINCTRL_MAX_GROUPS_CNT 512
enum scmi_pinctrl_protocol_cmd {
	GET_PINS = 0xc,
	PIN_LIST = 0x4,
	SET_MUX = 0x7,
	PINCTRL_ATTR = 0x3,
	GET_CONFIG = 0x9,
	SET_CONFIG = 0xa,
	GET_CONFIG_GROUP = 0x5,
	SET_CONFIG_GROUP = 0x6,
	GET_GROUP_PINS = 0xb,
	GET_FUNC_GROUP = 0xd,
};

struct scmi_group_info {
	bool has_name;
	char name[SCMI_MAX_STR_SIZE];
	unsigned group_pins[SCMI_PINCTRL_MAX_PINS_CNT];
	unsigned nr_pins;
};

typedef struct pin_name_ {
	char name[16];
}pin_name_s;

struct scmi_function_info {
	bool has_name;
	char name[SCMI_MAX_STR_SIZE];
	pin_name_s groups[SCMI_PINCTRL_MAX_GROUPS_CNT];
	u8 nr_groups;
};

struct scmi_pinctrl_info {
	u32 version;
	u16 nr_groups;
	u16 nr_functions;
	u16 nr_pins;
	struct scmi_group_info *groups;
	struct scmi_function_info *functions;
	pin_name_s pin[SCMI_PINCTRL_MAX_PINS_CNT];
};

static int scmi_pinctrl_attributes_get(const struct scmi_protocol_handle *ph,
				     struct scmi_pinctrl_info *pi)
{
	int ret;
	struct scmi_xfer *t;
	char * p = NULL;
	struct scmi_msg_pinctrl_protocol_attributes {
		__le16 nr_pins;
		__le16 nr_groups;		
		__le16 nr_functions;
		__le16 reserved;
	} *attr;

	ret = ph->xops->xfer_get_init(ph, PROTOCOL_ATTRIBUTES,
				      0,sizeof(struct scmi_msg_pinctrl_protocol_attributes), &t);
	if (ret)
		return ret;

	attr = t->rx.buf;
	p = t->rx.buf;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		pi->nr_functions = le16_to_cpu(attr->nr_functions);
		pi->nr_groups = le16_to_cpu(attr->nr_groups);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_pinctrl_get_groups_count(const struct scmi_protocol_handle *ph)
{
	struct scmi_pinctrl_info *pi = ph->get_priv(ph);

	return pi->nr_groups;
}

static int scmi_pinctrl_get_group_name(const struct scmi_protocol_handle *ph,
								u32 selector, const char **name)
{
	struct scmi_pinctrl_info *pi = ph->get_priv(ph);

	if (selector >= SCMI_PINCTRL_MAX_GROUPS_CNT)
		return -EINVAL;

	if (!pi->groups[selector].has_name) {
		snprintf(pi->groups[selector].name, SCMI_MAX_STR_SIZE, "%d", selector);
		pi->groups[selector].has_name = true;
	}

//	*name = pi->groups[selector].name;
	*name = pi->pin[selector].name;

	return 0;
}

static int scmi_pinctrl_get_group_pins(const struct scmi_protocol_handle *ph,
									   u32 selector, const unsigned **pins,
									   unsigned *nr_pins)
{
	struct scmi_pinctrl_info *pi = ph->get_priv(ph);
	u16 *list;
	int loop, ret = 0;
	struct scmi_xfer *t;
	__le32 *num_ret;
	u32 tot_num_ret = 0, loop_num_ret;
	struct scmi_group_pins_tx {
		__le32 identifier;
		__le32 flags;
		__le32 index;
	} *tx;
	

	if (selector > SCMI_PINCTRL_MAX_GROUPS_CNT)
		return -EINVAL;

	if (pi->groups[selector].nr_pins) {
		*nr_pins = pi->groups[selector].nr_pins;
		*pins = pi->groups[selector].group_pins;
		return 0;
	}

	ret = ph->xops->xfer_get_init(ph, PIN_LIST, sizeof(*tx), 0, &t);
	if (ret)
		return ret;

	tx = t->tx.buf;
	num_ret = t->rx.buf;
	list = t->rx.buf + sizeof(*num_ret);

	tx->index = cpu_to_le32(tot_num_ret);
	tx->identifier = cpu_to_le32(selector);
	tx->flags = 1;

	ret = ph->xops->do_xfer(ph, t);
	if (ret)
		goto put;

	loop_num_ret = le32_to_cpu(*num_ret);

	for (loop = 0; loop < loop_num_ret; loop++) {
		pi->groups[selector].group_pins[loop] =
			le16_to_cpu(list[loop]);
	}

put:
	ph->xops->xfer_put(ph, t);
	pi->groups[selector].nr_pins = le32_to_cpu(*num_ret);
	*pins = pi->groups[selector].group_pins;
	*nr_pins = pi->groups[selector].nr_pins;

	return ret;
}

static int scmi_pinctrl_get_functions_count(const struct scmi_protocol_handle *ph)
{
	struct scmi_pinctrl_info *pi = ph->get_priv(ph);

	return pi->nr_functions;
}

static int scmi_pinctrl_get_function_name(const struct scmi_protocol_handle *ph,
								   u32 selector, const char **name)
{
	struct scmi_pinctrl_info *pi = ph->get_priv(ph);
	int ret = 0;
	struct pin_attr_a2p {
		__le32 identifier;
		__le32 flags;
	} *tx;
	struct pin_attr_p2a {
		__le32 attributes;
		u8 name[16];
	} *rx;
	struct scmi_xfer *t;
	int num_ret = 0;
	int loop;
	u8 *list;

	if (selector >= pi->nr_functions)
		return -EINVAL;
	

	if (pi->functions[selector].has_name) {
		*name = pi->functions[selector].name;
		return 0;
	}

	ret = ph->xops->xfer_get_init(ph, PINCTRL_ATTR, sizeof(*tx), 0, &t);
	if (ret)
		return ret;

	tx = t->tx.buf;
	rx = t->rx.buf;

	tx->identifier = cpu_to_le32(selector);
	tx->flags = 2;

	ret = ph->xops->do_xfer(ph, t);

	num_ret = rx->attributes;
	list = t->rx.buf + sizeof(rx->attributes);

	for (loop = 0; loop < num_ret; loop++) {
		if (!pi->functions[loop].has_name) {
			memcpy(pi->functions[loop].name, list, 16);
			pi->functions[loop].has_name = true;
			list+=16;
		}
	}

/*
	if (!pi->functions[selector].has_name) {
		snprintf(pi->functions[selector].name, SCMI_MAX_STR_SIZE,
				 "%s", rx->name);
		pi->functions[selector].has_name = true;
	}
*/
	ph->xops->xfer_put(ph, t);

	*name = pi->functions[selector].name;
	return 0;
}

static int scmi_pinctrl_get_function_groups(const struct scmi_protocol_handle *ph,
									 u32 selector, u32 *nr_groups,
									 const void **groups)
{
	struct scmi_pinctrl_info *pi = ph->get_priv(ph);
	u8 *list;
	int loop, ret = 0;
	struct scmi_xfer *t;
	struct scmi_func_groups {
		__le32 selector;
		__le32 skip;
	} *tx;
	__le32 *num_ret;
	u32 tot_num_ret = 0, loop_num_ret;

	if (selector >= pi->nr_functions)
		return -EINVAL;

	if (pi->functions[selector].nr_groups) {
		*nr_groups = pi->functions[selector].nr_groups;
		*groups = pi->functions[selector].groups;
		return 0;
	}

	ret = ph->xops->xfer_get_init(ph, GET_FUNC_GROUP, sizeof(*tx), 0, &t);
	if (ret)
		return ret;

	tx = t->tx.buf;
	num_ret = t->rx.buf;

	do {
		/* Set the number of pins to be skipped/already read */
		tx->skip = cpu_to_le32(tot_num_ret);
		tx->selector = cpu_to_le32(selector);
		list = t->rx.buf + sizeof(*num_ret);

		ret = ph->xops->do_xfer(ph, t);
		if (ret)
			break;

		loop_num_ret = le32_to_cpu(*num_ret);

		if (tot_num_ret + loop_num_ret > SCMI_PINCTRL_MAX_GROUPS_CNT) {
			dev_err(ph->dev, "No. of PINS > SCMI_PINCTRL_MAX_GROUPS_CNT");
			break;
		}
		
		for (loop = 0; loop < loop_num_ret; loop++) {
			memcpy(pi->functions[selector].groups[loop+tot_num_ret].name, list, 16);
			list+=16;
		}

		tot_num_ret += loop_num_ret;

//		scmi_reset_rx_to_maxsz(handle, t);
	} while (loop_num_ret);

	ph->xops->xfer_put(ph, t);
	if (!ret) {
	pi->functions[selector].nr_groups = tot_num_ret;
	*groups = pi->functions[selector].groups;
	*nr_groups = pi->functions[selector].nr_groups;
	}
	return ret;
}

static int scmi_pinctrl_set_mux(const struct scmi_protocol_handle *ph, u32 selector,
						u32 group)
{
	//struct scmi_pinctrl_info *pi = ph->get_priv(ph);
	struct scmi_xfer *t;
	struct scmi_mux_tx {
		__le32 identifier;
		__le32 function_id;
		__le32 flags;
	} *tx;
	int ret;

	ret = ph->xops->xfer_get_init(ph, SET_MUX, sizeof(*tx), 0, &t);
	if (ret)
		return ret;

	tx = t->tx.buf;
	tx->function_id = cpu_to_le16(selector);
	tx->identifier = cpu_to_le16(group);
	tx->flags = 0;

	ret = ph->xops->do_xfer(ph, t);
	if (ret) {
		printk("setmux err fun:%s\n", __func__);
	}

	ph->xops->xfer_put(ph, t);
	return ret;
}

static int scmi_pinctrl_get_pins(const struct scmi_protocol_handle *ph, u32 *nr_pins,
						  const void **pins)
{
	struct scmi_pinctrl_info *pi = ph->get_priv(ph);
	u8 *list;
	int loop, ret = 0;
	struct scmi_xfer *t;
	__le32 *num_skip, *num_ret;
	u32 tot_num_ret = 0, loop_num_ret;

	if (pi->nr_pins) {
		*nr_pins = pi->nr_pins;
		*pins = pi->pin;
		return 0;
	}

	ret = ph->xops->xfer_get_init(ph, GET_PINS, sizeof(*num_skip), 0, &t);
	if (ret)
		return ret;

	num_skip = t->tx.buf;
	num_ret = t->rx.buf;

	do {
		list = t->rx.buf + sizeof(*num_ret);
		/* Set the number of pins to be skipped/already read */
		*num_skip = cpu_to_le32(tot_num_ret);
		
		ret = ph->xops->do_xfer(ph, t);
		if (ret)
			break;

		loop_num_ret = le32_to_cpu(*num_ret);

		if (tot_num_ret + loop_num_ret > SCMI_PINCTRL_MAX_PINS_CNT) {
			dev_err(ph->dev, "No. of PINS > SCMI_PINCTRL_MAX_PINS_CNT");
			break;
		}

		for (loop = 0; loop < loop_num_ret; loop++) {
			memcpy(pi->pin[loop+tot_num_ret].name, list, 16);
			list+=16;
		}

		tot_num_ret += loop_num_ret;

//		scmi_reset_rx_to_maxsz(handle, t);

	} while (loop_num_ret);

	ph->xops->xfer_put(ph, t);

	pi->nr_pins = tot_num_ret;
	*pins = (void*)pi->pin;
	*nr_pins = pi->nr_pins;

	return ret;
}

static int scmi_pinctrl_get_config(const struct scmi_protocol_handle *ph, u32 pin,
				  u32 *config)
{
	struct scmi_xfer *t;
	struct scmi_conf_tx {
		__le32 pin;
		__le32 config;
	} *tx;
	__le32 *packed_config;
	int ret;

	ret = ph->xops->xfer_get_init(ph, GET_CONFIG, sizeof(*tx), sizeof(*packed_config), &t);
	if (ret)
		return ret;

	tx = t->tx.buf;
	packed_config = t->rx.buf;
	tx->pin = cpu_to_le32(pin);
	tx->config = cpu_to_le32(*config);

	ret = ph->xops->do_xfer(ph, t);
	if (!ret)
		*config = le32_to_cpu(*packed_config);

	ph->xops->xfer_put(ph, t);
	return ret;
}

static int scmi_pinctrl_set_config(const struct scmi_protocol_handle *ph, u32 pin,
				  u32 config)
{
	struct scmi_xfer *t;
	struct scmi_conf_tx {
		__le32 pin;
		__le32 config;
	} *tx;
	int ret;

	ret = ph->xops->xfer_get_init(ph, SET_CONFIG, sizeof(*tx), 0, &t);
	if (ret)
		return ret;

	tx = t->tx.buf;
	tx->pin = cpu_to_le32(pin);
	tx->config = cpu_to_le32(config);

	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);
	return ret;
}

static int scmi_pinctrl_get_config_group(const struct scmi_protocol_handle *ph,
										 u32 group, u32 *config)
{
	struct scmi_xfer *t;
	struct scmi_conf_tx {
		__le32 group;
		__le32 config;
	} *tx;
	__le32 *packed_config;
	int ret;

	ret = ph->xops->xfer_get_init(ph, GET_CONFIG_GROUP, sizeof(*tx), sizeof(*packed_config), &t);
	if (ret)
		return ret;

	tx = t->tx.buf;
	packed_config = t->rx.buf;
	tx->group = cpu_to_le32(group);
	tx->config = cpu_to_le32(*config);

	ret = ph->xops->do_xfer(ph, t);
	if (!ret)
		*config = le32_to_cpu(*packed_config);

	ph->xops->xfer_put(ph, t);
	return ret;
}

static int scmi_pinctrl_set_config_group(const struct scmi_protocol_handle *ph,
										 u32 group, u32 config)
{
	struct scmi_xfer *t;
	struct scmi_conf_tx {
		__le32 group;
		__le32 config;
	} *tx;
	int ret;

	ret = ph->xops->xfer_get_init(ph, SET_CONFIG_GROUP, sizeof(*tx), 0, &t);
	if (ret)
		return ret;

	tx = t->tx.buf;
	tx->group = cpu_to_le32(group);
	tx->config = cpu_to_le32(config);

	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);
	return ret;
}

static const struct scmi_pinctrl_ops pinctrl_ops = {
	.get_groups_count = scmi_pinctrl_get_groups_count,
	.get_group_name = scmi_pinctrl_get_group_name,
	.get_group_pins = scmi_pinctrl_get_group_pins,
	.get_functions_count = scmi_pinctrl_get_functions_count,
	.get_function_name = scmi_pinctrl_get_function_name,
	.get_function_groups = scmi_pinctrl_get_function_groups,
	.set_mux = scmi_pinctrl_set_mux,
	.get_pins = scmi_pinctrl_get_pins,
	.get_config = scmi_pinctrl_get_config,
	.set_config = scmi_pinctrl_set_config,
	.get_config_group = scmi_pinctrl_get_config_group,
	.set_config_group = scmi_pinctrl_set_config_group,
};

static int scmi_pinctrl_protocol_init(const struct scmi_protocol_handle *ph)
{
	u32 version;
	struct scmi_pinctrl_info *pinfo;
	int ret;
	//void __iomem *addr;
	//unsigned int val;	

	ph->xops->version_get(ph, &version);

	dev_dbg(ph->dev, "Pinctrl Version %d.%d\n",
		PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));

	pinfo = devm_kzalloc(ph->dev, sizeof(*pinfo), GFP_KERNEL);
	if (!pinfo)
		return -ENOMEM;

	ret = scmi_pinctrl_attributes_get(ph, pinfo);
	if (ret)
		goto free;

	pinfo->groups = devm_kcalloc(ph->dev, pinfo->nr_groups,
								 sizeof(*pinfo->groups), GFP_KERNEL);
	if (!pinfo->groups) {
		ret = -ENOMEM;
		goto free;
	}

	pinfo->functions = devm_kcalloc(ph->dev, pinfo->nr_functions,
								 sizeof(*pinfo->functions), GFP_KERNEL);
	if (!pinfo->functions) {
		ret = -ENOMEM;
		goto free;
	}

	pinfo->version = version;

	ph->set_priv(ph, pinfo);

	return 0;
free:
	if (pinfo) {
		if (pinfo->functions)
			devm_kfree(ph->dev,pinfo->functions);

		if (pinfo->groups)
			devm_kfree(ph->dev,pinfo->groups);

		devm_kfree(ph->dev,pinfo);
	}

	return ret;
}

static const struct scmi_protocol scmi_pinctrl = {
	.id = SCMI_PROTOCOL_PINCTRL,
	.owner = THIS_MODULE,
	.instance_init = &scmi_pinctrl_protocol_init,
	.ops = &pinctrl_ops,
};

DEFINE_SCMI_PROTOCOL_REGISTER_UNREGISTER(pinctrl, scmi_pinctrl)
MODULE_AUTHOR("BST Ltd.");
