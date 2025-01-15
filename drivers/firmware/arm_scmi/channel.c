// SPDX-License-Identifier: GPL-2.0
/*
 * System Control and Management Interface (SCMI) Reset Protocol
 * 
 * Copyright (C) 2019-2020 ARM Ltd.
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#define pr_fmt(fmt) "SCMI Notifications RESET - " fmt

#include <linux/module.h>
#include <linux/scmi_protocol.h>

#include "common.h"
#include "notify.h"

enum scmi_channel_protocol_cmd {
	CHANNEL_READ_CHANNEL = 0x0,
	CHANNEL_WRITE_CHANNEL = 0x1,
};


struct scmi_channel_info {
	u32 version;
};

struct scmi_msg_read_domain_channel {
	__le32 reg;
};


struct scmi_msg_write_domain_channel {
	__le32 reg;
	__le32 value;
};


int channel_read(const struct scmi_protocol_handle *ph, u32 reg,u32 *val){

    int ret;
	struct scmi_xfer *t;
    struct scmi_msg_read_domain_channel * dom;

    ret = ph->xops->xfer_get_init(ph, CHANNEL_READ_CHANNEL, sizeof(*dom), 0, &t);
	if (ret)
		return ret;

	dom = t->tx.buf;
	dom->reg = cpu_to_le32(reg);


	ret = ph->xops->do_xfer(ph, t);
    if (!ret){
        *val = get_unaligned_le32(t->rx.buf);
    }

	ph->xops->xfer_put(ph, t);


	return ret;
}

int channel_write(const struct scmi_protocol_handle *ph, u32 reg,u32 val){

    int ret = -1;
	struct scmi_xfer *t;
    struct scmi_msg_write_domain_channel * dom;


    ret = ph->xops->xfer_get_init(ph, CHANNEL_WRITE_CHANNEL, sizeof(*dom), 0, &t);
	if (ret)
		return ret;

	dom = t->tx.buf;
	dom->reg = cpu_to_le32(reg);
    dom->value = cpu_to_le32(val);

	ret = ph->xops->do_xfer(ph, t);
    if (!ret){
        
    }

	ph->xops->xfer_put(ph, t);

	return ret;
}

static const struct scmi_channel_proto_ops channel_proto_ops = {
	.read = channel_read,
	.write = channel_write,
};



static int scmi_channel_protocol_init(const struct scmi_protocol_handle *ph)
{

	u32 version;
	struct scmi_channel_info *pinfo;


	ph->xops->version_get(ph, &version);

	pinfo = devm_kzalloc(ph->dev, sizeof(*pinfo), GFP_KERNEL);
	if (!pinfo)
		return -ENOMEM;

	pinfo->version = version;

	return ph->set_priv(ph, pinfo);
}



static const struct scmi_protocol scmi_channel = {
	.id = SCMI_PROTOCOL_CHANNEL,
	.owner = THIS_MODULE,
	.instance_init = &scmi_channel_protocol_init,
	.ops = &channel_proto_ops,
};

DEFINE_SCMI_PROTOCOL_REGISTER_UNREGISTER(channel, scmi_channel)
MODULE_AUTHOR("BST Ltd.");