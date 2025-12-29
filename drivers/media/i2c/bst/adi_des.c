// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#include <dt-bindings/media/bst-isp.h>
#include <dt-bindings/media/bst-mdev.h>

#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>

#include <media/v4l2-device.h>

#include <bst/media-dev.h>

#include "adi_des.h"

#include "utils.h"

/* NOTE: suitable for: Max9295, Max96717[F|R] */
static const u32 ser_mfp_ctrl_regs[] = {
	0x02BE, 0x02C1, 0x02C4, 0x02C7, 0x02CA, 0x02CD,
	0x02D0, 0x02D3, 0x02D6, 0x02D9, 0x02DC,
};

static const u32 ser_mfp_rx_id_regs[] = {
	0x02C0, 0x02C3, 0x02C6, 0x02C9, 0x02CC, 0x02CF,
	0x02D2, 0x02D5, 0x02D8, 0x02DB, 0x02DE,
};
/* clang-format on */

static int parse_rx_ports(struct adi_des *des)
{
	struct device *dev;
	struct device_node *parent, *port, *remote;
	int i;

	dev = des->dev;
	parent = of_get_child_by_name(dev->of_node, "rx-ports");
	if (!parent) {
		dev_err(dev, "Failed to parse rx-ports\n");
		return -EINVAL;
	}
	for (i = 0; i < des->param->num_gmsl; ++i) {
		struct rx_port *rxp;

		port = of_graph_get_port_by_id(parent, i);
		if (!port) {
			dev_warn(dev, "Unused rx port %u\n", i);
			continue;
		}
		remote = of_graph_get_remote_node(parent, i, 0);
		if (!remote) {
			dev_warn(dev, "Unconnected rx port %u\n", i);
			of_node_put(port);
			continue;
		}

		rxp = &des->rx_ports[i];
		rxp->id = i;
		(void)of_property_read_u32(port, "gmsl-ver", &rxp->gmsl_ver);
		(void)of_property_read_u32(port, "rx-rate", &rxp->rx_rate);
		rxp->him = of_property_read_bool(port, "him");
		rxp->cfg_with_delay =
			of_property_read_bool(port, "cfg-with-delay");
		rxp->enable = true;
		rxp->des = des;
		rxp->node = port;
		rxp->remote_fwnode = of_fwnode_handle(remote);
		of_node_put(remote);
		of_node_put(port);
	}
	of_node_put(parent);

	return 0;
}

static void parse_pipes(struct adi_des *des)
{
	struct device *dev;
	struct device_node *parent, *pipe;
	struct pipe *p;
	int i;

	dev = des->dev;
	for (i = 0; i < des->param->num_pipe; ++i) {
		p = &des->pipes[i];
		p->id = i;
		p->from_port = i;
		p->from_sid = PIPEZ;
		p->from_vc = 0;
		p->to_vc = i;
		p->to_csi = CSI1;
		p->enable = true;
	}

	parent = of_get_child_by_name(dev->of_node, "pipes");
	if (!parent) {
		dev_warn(dev, "Unable to parse pipes, use defaults\n");
		return;
	}

	for (i = 0; i < des->param->num_pipe; ++i) {
		pipe = of_graph_get_port_by_id(parent, i);
		if (!pipe) {
			dev_warn(dev, "Undefined pipe %u\n", i);
			continue;
		}
		p = &des->pipes[i];
		(void)of_property_read_u32(pipe, "from-port", &p->from_port);
		(void)of_property_read_u32(pipe, "from-sid", &p->from_sid);
		(void)of_property_read_u32(pipe, "from-vc", &p->from_vc);
		(void)of_property_read_u32(pipe, "to-vc", &p->to_vc);
		(void)of_property_read_u32(pipe, "to-csi", &p->to_csi);
		p->enable = !of_property_read_bool(pipe, "disabled");
		of_node_put(pipe);
	}
	of_node_put(parent);
}

static int parse_tx_ports(struct adi_des *des)
{
	struct device *dev;
	struct device_node *node;
	int phy_cp_group;

	dev = des->dev;
	phy_cp_group = 0;
	for_each_child_of_node(dev->of_node, node) {
		u32 id = 0;
		u32 copy_to;
		struct csi_tx_dev *txp;

		if (!node->name || of_node_cmp(node->name, "csi"))
			goto next;

		if (of_property_read_u32(node, "id", &id)) {
			dev_err(dev, "No id defined for %s\n", node->full_name);
			goto next;
		}

		if (id >= des->param->num_csi) {
			dev_err(dev, "ID %u is out of range %u\n", id,
				des->param->num_csi);
			goto next;
		}

		txp = &des->tx_ports[id];
		txp->enable = true;
		txp->drv_data = des;
		(void)of_property_read_u32(node, "phy-if", &txp->phy_if);
		(void)of_property_read_u32(node, "lane-num", &txp->lane_num);
		(void)of_property_read_u32(node, "lane-speed",
					   &txp->lane_speed);
		if (!of_property_read_u32(node, "copy-to", &copy_to)) {
			if (copy_to > des->param->csi_up ||
			    copy_to < des->param->csi_lo || copy_to == id ||
			    phy_cp_group >= des->param->num_csi / 2) {
				dev_err(dev,
					"Invalid phy copy cfg: destination: %u, group: %d\n",
					copy_to, phy_cp_group);
			} else {
				des->phy_cps[phy_cp_group].src = id;
				des->phy_cps[phy_cp_group].dst = copy_to;
				memcpy(&des->tx_ports[copy_to], txp,
				       sizeof(*txp));
				++phy_cp_group;
			}
		}
		txp->subdev.fwnode = of_fwnode_handle(node);
		dev_info(
			dev,
			"Find CSI %u, fwnode: 0x%016llX, phy-if: %u, lane: %ux%u\n",
			id, (u64)txp->subdev.fwnode, txp->phy_if, txp->lane_num,
			txp->lane_speed);
		/* NOTE: the last will be referenced by des */
		des->subdev = &txp->subdev;
next:
		of_node_put(node);
	}

	return 0;
}

int adi_des_parse_dt(struct adi_des *des)
{
	int rv;
	struct device *dev;
	struct device_node *node;

	dev = des->dev;
	node = dev->of_node;
	if (!node)
		return -EINVAL;

	des->role = ROLE_AUTO;
	(void)of_property_read_u32(node, "role", &des->role);
	des->pdb_gpio = of_get_named_gpio(node, "pdb-gpio", 0);
	des->csi_mode = CSI_MODE_2X4;
	(void)of_property_read_u32(node, "csi-mode", &des->csi_mode);
	(void)of_property_read_u32(node, "i2c-port", &des->i2c_port);
	des->cfg_with_delay = of_property_read_bool(node, "cfg-with-delay");

	rv = parse_rx_ports(des);
	if (rv)
		return rv;

	parse_pipes(des);

	rv = parse_tx_ports(des);
	if (rv)
		return rv;

	/* FSYNC */
	(void)of_property_read_u32(node, "fsync-mode", &des->fsync_mode);
	(void)of_property_read_u32(node, "fsync-fps", &des->fsync_fps);
	(void)of_property_read_u32(node, "fsync-rx-pin", &des->fsync_rx_pin);
	(void)of_property_read_u32(node, "fsync-tx-pin", &des->fsync_tx_pin);

	/* Hotplug */
	des->lock_gpio = of_get_named_gpio(node, "lock-gpio", 0);
	des->check_lock_period = CHECK_LOCK_PERIOD;
	(void)of_property_read_u32(node, "check-lock-period",
				   &des->check_lock_period);
	des->check_lock_period *= US_PER_MS;

	/* Safety */
	des->err_gpio = of_get_named_gpio(node, "err-gpio", 0);
	des->check_err_period = CHECK_ERR_PERIOD;
	(void)of_property_read_u32(node, "check-err-period",
				   &des->check_err_period);
	des->check_err_period *= US_PER_MS;

	return 0;
}

int adi_des_verify_cfg(struct adi_des *des)
{
	int i;
	struct device *dev;

	dev = des->dev;
	if (des->role > ROLE_AUTO) {
		dev_err(dev, "Invalid role: %u\n", des->role);
		return -EINVAL;
	}

	if (des->csi_mode > CSI_MODE_1X4B_2X2) {
		dev_err(dev, "Invalid csi-mode: %u\n", des->csi_mode);
		return -EINVAL;
	}

	if (des->i2c_port > des->param->num_i2c) {
		dev_err(dev, "Invalid i2c-port: %u\n", des->i2c_port);
		return -EINVAL;
	}

	if (des->fsync_mode > FSYNC_OUTER) {
		dev_err(dev, "Invalid fsync-mode: %u\n", des->fsync_mode);
		return -EINVAL;
	}

	if (des->fsync_rx_pin > des->param->num_mfp) {
		dev_err(dev, "Invalid fsync-rx-pin: %u\n", des->fsync_rx_pin);
		return -EINVAL;
	}

	if (des->fsync_tx_pin > des->param->num_mfp) {
		dev_err(dev, "Invalid fsync-tx-pin: %u\n", des->fsync_tx_pin);
		return -EINVAL;
	}

	for (i = 0; i < des->param->num_gmsl; ++i) {
		if (!des->rx_ports[i].enable)
			continue;

		if (des->rx_ports[i].gmsl_ver < des->param->gmsl_ver_lo ||
		    des->rx_ports[i].gmsl_ver > des->param->gmsl_ver_up) {
			dev_err(dev, "Invalid gmsl-ver %u for rx ports %u\n",
				des->rx_ports[i].gmsl_ver, i);
			return -EINVAL;
		}
	}

	return 0;
}

int adi_des_power_up(struct adi_des *des)
{
	int rv;
	struct device *dev;

	dev = des->dev;
	/* Some deserialize does not have pdb pin */
	if (!gpio_is_valid(des->pdb_gpio))
		return 0;

	rv = gpio_request(des->pdb_gpio, dev_name(dev));
	if (rv) {
		dev_err(dev, "Failed to request pdb gpio %d, rv: %d\n",
			des->pdb_gpio, rv);
		return rv;
	}

	rv = gpio_get_value(des->pdb_gpio);
	gpio_direction_output(des->pdb_gpio, 1);
	dev_info(dev, "pdb output %d\n", rv);
	gpio_free(des->pdb_gpio);

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 sub-device operations
 */
static int s_power(struct v4l2_subdev *sd, int on)
{
	struct csi_tx_dev *tx_dev;
	struct adi_des *des;
	int rv;

	tx_dev = (struct csi_tx_dev *)container_of(sd, struct csi_tx_dev,
						   subdev);
	des = tx_dev->drv_data;
	dev_info(des->dev, "%s: role: %s\n", __func__, str_role(des->role));
	if (des->role != ROLE_MASTER)
		return 0;

	if (!on) {
		des->on = 0;
		adi_des_exit_lock_handler(des);

#ifndef CONFIG_VIDEO_BST_ISP_MULTI_OS
		des->ops->csi_pre_streamon(des);
#endif
		return 0;
	}

	if (des->on)
		return 0;

	if (des->ops->is_des_setuped && des->ops->is_des_setuped(des)) {
		/* NOTE: If des has been setuped, we consider links have beed setuped */
		des->link_init_map = des->link_en_map;
		rv = 0;
	} else {
		rv = des->ops->des_setup(des);
	}

	if (rv)
		return rv;

	des->on = 1;
	/* NOTE: We initialized deserializer, delay to wait links locked */
	ursleep(des->param->t_lock);
	return adi_des_init_lock_handler(des);
}

static const struct v4l2_subdev_core_ops v4l2_core_ops = {
	.s_power = s_power,
};

static int s_stream(struct v4l2_subdev *sd, int enable)
{
	int rv;
	struct csi_tx_dev *tx_dev;
	struct adi_des *des;

	tx_dev = (struct csi_tx_dev *)container_of(sd, struct csi_tx_dev,
						   subdev);
	des = tx_dev->drv_data;
	if (des->role != ROLE_MASTER)
		return 0;

	/* NOTE: Sync with link setup */
	mutex_lock(&des->lock);
	rv = des->ops->csi_stream(des, enable);
	mutex_unlock(&des->lock);

	return rv;
}

static int pre_streamon(struct v4l2_subdev *sd, u32 flags)
{
	struct csi_tx_dev *tx_dev;
	struct adi_des *des;

	tx_dev = (struct csi_tx_dev *)container_of(sd, struct csi_tx_dev,
						   subdev);
	des = (struct adi_des *)tx_dev->drv_data;
	return des->ops->csi_pre_streamon(des);
}

static const struct v4l2_subdev_video_ops v4l2_video_ops = {
	.s_stream = s_stream,
	.pre_streamon = pre_streamon,
};

static const struct v4l2_subdev_pad_ops v4l2_pad_ops = {
	.get_mbus_config = csi_tx_get_mbus_config,
};

static const struct v4l2_subdev_ops v4l2_ops = {
	.core = &v4l2_core_ops,
	.video = &v4l2_video_ops,
	.pad = &v4l2_pad_ops,
};

static int des_notify_bound(struct v4l2_async_notifier *notifier,
			    struct v4l2_subdev *sd,
			    struct v4l2_async_subdev *asd)
{
	int i;
	struct camera_dev *cam;
	struct rx_port *rxp;
	struct adi_des *des;

	cam = subdev_to_camera_dev(sd);
	rxp = container_of(asd, struct rx_port, asd);
	des = rxp->des;

	dev_info(des->dev, "RX %u bound\n", rxp->id);
	rxp->cam = cam;
	for (i = 0; i < des->param->num_pipe; ++i) {
		struct pipe *pipe;

		pipe = &des->pipes[i];
		if (pipe->from_port != rxp->id)
			continue;
		des->link_en_map |= BIT(rxp->id);
		des->tx_ports[pipe->to_csi].cameras[pipe->to_vc] = cam;
		dev_info(des->dev, "CSI %u bound vc %u\n", pipe->to_csi,
			 pipe->to_vc);
	}

	return 0;
}

static void des_notify_unbind(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *subdev,
			      struct v4l2_async_subdev *asd)
{
}

static const struct v4l2_async_notifier_operations des_async_ops = {
	.bound = des_notify_bound,
	.unbind = des_notify_unbind,
};

int adi_des_init_v4l2_dev(struct adi_des *des)
{
	int rv;
	int i;
	struct device *dev;
	struct v4l2_subdev *sd;

	dev = des->dev;
	sd = des->subdev;
	for (i = 0; i < des->param->num_csi; ++i) {
		if (!des->tx_ports[i].enable)
			continue;

		sd = &des->tx_ports[i].subdev;
		v4l2_subdev_init(sd, &v4l2_ops);
		sd->dev = dev;
		snprintf(sd->name, sizeof(sd->name), "%s-csi%u",
			 dev_name(des->dev), i);
		v4l2_set_subdevdata(sd, des);
		rv = v4l2_async_register_subdev(sd);
		if (rv) {
			dev_err(dev, "Failed to register subdev for CSI %u\n",
				i);
			goto err_register_subdev;
		}
	}

	v4l2_async_nf_init(&des->notifier);
	des->notifier.ops = &des_async_ops;
	for (i = 0; i < des->param->num_gmsl; ++i) {
		struct rx_port *rxp;

		rxp = &des->rx_ports[i];
		if (!rxp->enable)
			continue;

		rxp->des = des;
		rxp->asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
		rxp->asd.match.fwnode = rxp->remote_fwnode;
		rv = __v4l2_async_nf_add_subdev(&des->notifier, &(rxp->asd));
		if (rv < 0) {
			dev_err(dev,
				"Failed to add async dev for port %d to notifier\n",
				i);
			goto err_add_async_dev;
		}
	}
	rv = v4l2_async_subdev_nf_register(des->subdev, &des->notifier);
	if (rv < 0) {
		dev_err(dev, "Failed to register notifier\n");
		goto err_register_nf;
	}

	return 0;

err_register_nf:
err_add_async_dev:
	v4l2_async_nf_cleanup(&des->notifier);
	v4l2_async_unregister_subdev(sd);
err_register_subdev:
	return rv;
}

void adi_des_setup_links(struct adi_des *des)
{
	int rv;
	int i;
	struct device *dev = des->dev;
	u32 last_map;
	struct rx_port *rxp;
	struct camera_dev *cam;
	bool locked;

	/* NOTE: Sync with stream operations */
	mutex_lock(&des->lock);
	last_map = des->video_lock_map;
	for (i = 0; i < des->param->num_gmsl; ++i) {
		rxp = &des->rx_ports[i];
		cam = rxp->cam;
		if (cam == NULL)
			continue;

		locked = des->ops->is_video_locked(des, i);
		if (locked) {
			if (des->link_init_map & BIT(i))
				continue;

			if (dt_is_raw(cam->data_type))
				dev_alert(
					dev,
					"Port %d video locked from not initialized\n",
					i);
		}

		/* Video locked -> unlocked */
		if (des->video_lock_map & BIT(i) && !des->resume) {
			cam->power_on = false;
			des->video_lock_map &= ~(BIT(i));
			v4l2_subdev_notify(des->subdev,
					   ISP_EVENT_CAMERA_DISCONNECT, cam);
		}

		locked = des->ops->is_link_locked(des, i);
		dev_dbg(dev, "port: %d, link lock: %d\n", i, locked);
		if (!locked)
			continue;

		rv = des->ops->gmsl_setup(des, i);
		if (rv)
			dev_err_ratelimited(
				dev,
				"Failed to setup link for port %d, rv: %d\n", i,
				rv);
		des->link_init_map |= BIT(i);
	}

	ursleep(des->param->t_lock);
	for (i = 0; i < des->param->num_gmsl; ++i) {
		rxp = &des->rx_ports[i];
		cam = rxp->cam;
		if (cam == NULL)
			continue;

		locked = des->ops->is_video_locked(des, i);
		dev_dbg(dev, "port: %d, video lock: %d\n", i, locked);
		/* Video unlocked -> locked */
		if (locked && !(last_map & BIT(i)) &&
		    (des->link_init_map & BIT(i))) {
			v4l2_subdev_notify(des->subdev,
					   ISP_EVENT_CAMERA_CONNECT, cam);
			des->video_lock_map |= BIT(i);
		}
	}
	mutex_unlock(&des->lock);
}

static int link_setup_fn(void *data)
{
	struct adi_des *des = (struct adi_des *)data;
	struct device *dev = des->dev;

	dev_info(dev, "Link setup: init on CPU %u\n", smp_processor_id());
	while (!kthread_should_stop()) {
		do {
			if (!des->lock_disable)
				adi_des_setup_links(des);
			dev_dbg(dev, "en: 0x%08X, lock: 0x%08X\n",
				des->link_en_map, des->video_lock_map);
			if ((des->video_lock_map == des->link_en_map) &&
			    des->lock_irq_enable)
				break;

			if (kthread_should_stop())
				break;

			ursleep(des->check_lock_period);
		} while (true);

		if (des->lock_irq_enable) {
			enable_irq(des->lock_irq);
			wait_for_completion(&des->check_lock_comp);
		}
	}
	dev_info(dev, "Link setup: exit\n");

	return 0;
}

static irqreturn_t lock_irq_handler(int irq, void *p)
{
	struct adi_des *des;

	des = (struct adi_des *)p;
	disable_irq_nosync(irq);
	complete(&des->check_lock_comp);

	return IRQ_HANDLED;
}

int adi_des_init_lock_handler(struct adi_des *des)
{
	int rv;
	struct device *dev;
	struct sched_param sched_param;
	cpumask_t cpu_mask;
	unsigned int cpu_num;
	int preferred_cpu;

	if (!IS_ERR_OR_NULL(des->check_lock_task))
		return 0;

	dev = des->dev;
	if (gpio_is_valid(des->lock_gpio)) {
		rv = gpio_request(des->lock_gpio, dev_name(dev));
		if (rv) {
			dev_err(dev, "Failed to request lock gpio %d, rv: %d\n",
				des->lock_gpio, rv);
		} else {
			gpio_direction_input(des->lock_gpio);
			des->lock_irq = gpio_to_irq(des->lock_gpio);
			dev_info(dev, "lock irq is %d\n", des->lock_irq);
			rv = devm_request_threaded_irq(
				dev, des->lock_irq, NULL, lock_irq_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT |
					IRQF_NO_AUTOEN,
				dev_name(dev), des);
			if (rv) {
				dev_err(dev,
					"Failed to request lock irq %d, rv: %d\n",
					des->lock_irq, rv);
			} else {
				des->lock_irq_enable = true;
				init_completion(&des->check_lock_comp);
			}
		}
	}

	if (!des->lock_irq_enable)
		dev_warn(dev, "lock irq is invalid, use poll mode\n");

	des->check_lock_task = kthread_create(link_setup_fn, des,
					      "%s-lock-task", dev_name(dev));
	if (IS_ERR(des->check_lock_task)) {
		rv = PTR_ERR(des->check_lock_task);
		dev_err(dev, "Failed to create link setup task, rv: %d\n", rv);
		if (des->lock_irq_enable) {
			devm_free_irq(dev, des->lock_irq, des);
			des->lock_irq_enable = false;
		}
		gpio_free(des->lock_gpio);
		return rv;
	}

	sched_param.sched_priority = CHECK_LOCK_RT_PRIO;
	sched_setscheduler(des->check_lock_task, SCHED_FIFO, &sched_param);

	cpumask_clear(&cpu_mask);
	cpu_num = num_online_cpus();
	preferred_cpu = des->i2c_adap->nr % cpu_num;
	if (preferred_cpu == 0)
		preferred_cpu = cpu_num - 1;
	cpumask_set_cpu(preferred_cpu, &cpu_mask);
	set_cpus_allowed_ptr(des->check_lock_task, &cpu_mask);

	wake_up_process(des->check_lock_task);

	return 0;
}

void adi_des_exit_lock_handler(struct adi_des *des)
{
	if (des->lock_irq_enable) {
		devm_free_irq(des->dev, des->lock_irq, des);
		des->lock_irq_enable = false;
		complete(&des->check_lock_comp);
	}
	if (!IS_ERR_OR_NULL(des->check_lock_task)) {
		kthread_stop(des->check_lock_task);
		des->check_lock_task = NULL;
	}
	gpio_free(des->lock_gpio);
}

int adi_des_set_pre_ser(struct adi_des *des, struct device_node *node,
			i2cset *i2cset, bool cfg_with_delay)
{
	int cfg_num;

	cfg_num = of_property_count_u32_elems(node, "pre-ser");
	if (cfg_num > 0)
		return i2csetc_from_dt(des->i2c_client, i2cset, node, "pre-ser",
				       cfg_with_delay);

	return 0;
}

int adi_des_set_post_ser(struct adi_des *des, struct device_node *node,
			 i2cset *i2cset, bool cfg_with_delay)
{
	int cfg_num;

	cfg_num = of_property_count_u32_elems(node, "post-ser");
	if (cfg_num > 0)
		return i2csetc_from_dt(des->i2c_client, i2cset, node,
				       "post-ser", cfg_with_delay);

	return 0;
}

int adi_des_set_pre_gmsl(struct adi_des *des, i2cset *i2cset)
{
	int cfg_num;

	cfg_num = of_property_count_u32_elems(des->dev->of_node, "pre-gmsl");
	if (cfg_num > 0)
		return i2csetc_from_dt(des->i2c_client, i2cset,
				       des->dev->of_node, "pre-gmsl",
				       des->cfg_with_delay);
	else
		return i2csetc_in_bulk(des->i2c_client, i2cset,
				       &des->param->pre_gmsl);
}

int adi_des_set_post_gmsl(struct adi_des *des, i2cset *i2cset)
{
	int cfg_num;

	cfg_num = of_property_count_u32_elems(des->dev->of_node, "post-gmsl");
	if (cfg_num > 0)
		return i2csetc_from_dt(des->i2c_client, i2cset,
				       des->dev->of_node, "post-gmsl",
				       des->cfg_with_delay);
	else
		return i2csetc_in_bulk(des->i2c_client, i2cset,
				       &des->param->post_gmsl);
}

int adi_des_set_pre_csi(struct adi_des *des, i2cset *i2cset)
{
	int cfg_num;

	cfg_num = of_property_count_u32_elems(des->dev->of_node, "pre-csi");
	if (cfg_num > 0)
		return i2csetc_from_dt(des->i2c_client, i2cset,
				       des->dev->of_node, "pre-csi",
				       des->cfg_with_delay);
	else
		return i2csetc_in_bulk(des->i2c_client, i2cset,
				       &des->param->pre_csi);
}

int adi_des_set_post_csi(struct adi_des *des, i2cset *i2cset)
{
	int cfg_num;

	cfg_num = of_property_count_u32_elems(des->dev->of_node, "post-csi");
	if (cfg_num > 0)
		return i2csetc_from_dt(des->i2c_client, i2cset,
				       des->dev->of_node, "post-csi",
				       des->cfg_with_delay);
	else
		return i2csetc_in_bulk(des->i2c_client, i2cset,
				       &des->param->post_csi);
}

static ssize_t info_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct adi_des *des;
	int len;

	des = dev_get_drvdata(dev);
	len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "%-24s: %s\n", "Role",
			str_role(des->role));
	len += snprintf(buf + len, PAGE_SIZE - len, "%-24s: %u\n", "CSI mode",
			des->csi_mode);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-24s: %u\n", "I2C port",
			des->i2c_port);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-24s: %u\n", "On",
			des->on);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-24s: %u/%u/%u/%u\n",
			"FSync mode/fps/RX/TX", des->fsync_mode, des->fsync_fps,
			des->fsync_rx_pin, des->fsync_tx_pin);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-24s: %ums\n",
			"Check lock period",
			des->check_lock_period / US_PER_MS);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-24s: %u\n",
			"Lock irq enable", des->lock_irq_enable);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-24s: 0x%X\n",
			"Link enable map", des->link_en_map);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-24s: 0x%X\n",
			"Video lock map", des->video_lock_map);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-24s: %u\n",
			"Lock handler disable", des->lock_disable);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-24s: %ums\n",
			"Check err period", des->check_err_period / US_PER_MS);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-24s: %u\n",
			"Err irq enable", des->err_irq_enable);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-24s: %u\n",
			"Err handler disable", des->err_disable);

	return len;
}

static struct device_attribute info_attr = __ATTR(info, 0644, info_show, NULL);

static ssize_t lock_disable_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct adi_des *des;
	int rv;
	long disable;

	des = dev_get_drvdata(dev);
	rv = kstrtol(buf, 0, &disable);
	if (rv)
		return -EIO;

	des->lock_disable = !!disable;
	if (des->lock_irq_enable) {
		if (des->lock_disable)
			disable_irq_nosync(des->lock_irq);
		else
			enable_irq(des->lock_irq);
	}

	return count;
}

static struct device_attribute lock_disable_attr =
	__ATTR(lock_disable, 0644, NULL, lock_disable_store);

static ssize_t err_disable_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct adi_des *des;
	int rv;
	long disable;

	des = dev_get_drvdata(dev);
	rv = kstrtol(buf, 0, &disable);
	if (rv)
		return -EIO;

	des->err_disable = !!disable;

	return count;
}

static struct device_attribute err_disable_attr =
	__ATTR(err_disable, 0644, NULL, err_disable_store);

/* clang-format off */
static struct attribute *attrs[] = {
	&info_attr.attr,
	&lock_disable_attr.attr,
	&err_disable_attr.attr,
	NULL,
};
/* clang-format on */

static const struct attribute_group attr_group = {
	.attrs = attrs,
};

int adi_des_sysfs_init(struct adi_des *des)
{
	int rv;
	struct device *dev;

	dev = des->dev;

	rv = sysfs_create_group(&dev->kobj, &attr_group);
	if (rv) {
		dev_err(dev, "Failed to create sysfs group, rv: %d\n", rv);
		return rv;
	}

	return 0;
}

void adi_des_sysfs_exit(struct adi_des *des)
{
	sysfs_remove_group(&des->dev->kobj, &attr_group);
}

int adi_des_dt_to_bpp(int dt)
{
	if (dt >= 0x31 && dt <= 0x37)
		return 8;

	switch (dt) {
	case DT_BLANKING:
	case DT_EMBEDDED_8B:
	case DT_RAW8:
		return 8;
	case DT_RAW10:
		return 10;
	case DT_RAW12:
		return 12;
	case DT_RAW14:
		return 14;
	case DT_YUV422_8B:
	case DT_RGB565:
	case DT_RAW16:
		return 16;
	case DT_RGB666:
		return 18;
	case DT_YUV422_10B:
	case DT_RAW20:
		return 20;
	case DT_RGB888:
	case DT_YUV422_12B:
		return 24;
	default:
		pr_err("%s: Unsupported dt: 0x%02X\n", __func__, dt);
		return 0;
	}
}

int adi_ser_set_alias(struct adi_des *des, int port)
{
	int rv;
	int tries;
	u32 val;
	struct camera_dev *cam;

	cam = des->rx_ports[port].cam;
	tries = 0;
	do {
		cam->ser_i2cset(des->i2c_adap, cam->ser_addr, 0x0000,
				MK_I2C_MAP(cam->ser_alias));
		ursleep(I2C_ADDR_VALID_TIME);
		rv = cam->ser_i2cget(des->i2c_adap, cam->ser_alias, 0x0000,
				     &val);
		if (!rv && (val & 0xFE) == MK_I2C_MAP(cam->ser_alias))
			return 0;
		usleep_range(I2C_OP_DELAY, I2C_OP_DELAY * 2);
	} while (++tries < I2C_OP_TRIES);

	return -EIO;
}

static void ser_set_i2c_map1(struct adi_des *des, int port)
{
	struct camera_dev *cam;

	cam = des->rx_ports[port].cam;

	cam->ser_i2cset(des->i2c_adap, cam->ser_alias, 0x0009,
			MK_I2C_MAP(cam->sensor_alias));
	cam->ser_i2cset(des->i2c_adap, cam->ser_alias, 0x000A,
			MK_I2C_MAP(cam->sensor_addr));
	cam->ser_i2cset(des->i2c_adap, cam->ser_alias, 0x000B,
			MK_I2C_MAP(cam->ext_alias));
	cam->ser_i2cset(des->i2c_adap, cam->ser_alias, 0x000C,
			MK_I2C_MAP(cam->ext_addr));
}

static void ser_set_i2c_map2(struct adi_des *des, int port)
{
	struct camera_dev *cam;

	cam = des->rx_ports[port].cam;

	cam->ser_i2cset(des->i2c_adap, cam->ser_alias, 0x0042,
			MK_I2C_MAP(cam->sensor_alias));
	cam->ser_i2cset(des->i2c_adap, cam->ser_alias, 0x0043,
			MK_I2C_MAP(cam->sensor_addr));
	cam->ser_i2cset(des->i2c_adap, cam->ser_alias, 0x0044,
			MK_I2C_MAP(cam->ext_alias));
	cam->ser_i2cset(des->i2c_adap, cam->ser_alias, 0x0045,
			MK_I2C_MAP(cam->ext_addr));
}

int adi_ser_set_i2c_map(struct adi_des *des, int port)
{
	struct camera_dev *cam;
	u32 gmsl_ver;

	cam = des->rx_ports[port].cam;
	switch (cam->ser_type) {
	case SER_MAX9295:
	case SER_MAX9295A:
	case SER_MAX9295E:
	case SER_MAX96717:
	case SER_MAX96717F:
	case SER_MAX96717R:
		ser_set_i2c_map2(des, port);
		break;
	case SER_MAX96701:
	case SER_MAX96705:
		ser_set_i2c_map1(des, port);
		break;
	default:
		dev_dbg(cam->dev,
			"Unsupported serializer type: %u, fallback to GMSL ver\n",
			cam->ser_type);
		gmsl_ver = des->rx_ports[port].gmsl_ver;
		switch (gmsl_ver) {
		case GMSL1:
			ser_set_i2c_map1(des, port);
			break;
		case GMSL2:
			ser_set_i2c_map2(des, port);
			break;
		default:
			dev_warn(cam->dev, "Unsupported gmsl ver: %u\n",
				 gmsl_ver);
		}
	}

	ursleep(des->param->t_lock);
	return 0;
}

/* TODO: compatible with GMSL 1. */
int adi_ser_set_fsync(struct adi_des *des, int port)
{
	struct camera_dev *cam;

	cam = des->rx_ports[port].cam;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
	switch (des->fsync_mode) {
	case FSYNC_OUTER:
		cam->ser_i2cset(des->i2c_adap, cam->ser_alias,
				ser_mfp_rx_id_regs[cam->ser_fsync_tx_pin],
				des->fsync_rx_pin);
	case FSYNC_INNER:
		cam->ser_i2cset(des->i2c_adap, cam->ser_alias,
				ser_mfp_ctrl_regs[cam->ser_fsync_tx_pin],
				FSYNC_OUTER_SER_TX_CFG);
		break;
	default:
		return 0;
	}
#pragma GCC diagnostic pop

	return 0;
}

int adi_ser_reset(struct adi_des *des, int port)
{
	struct camera_dev *cam;

	cam = des->rx_ports[port].cam;
	switch (cam->ser_type) {
	case SER_MAX9295:
	case SER_MAX9295A:
	case SER_MAX9295E:
	case SER_MAX96717:
	case SER_MAX96717F:
	case SER_MAX96717R:
		cam->ser_i2cset(des->i2c_adap, cam->ser_alias, 0x0010, 0x80);
		break;
	default:
		dev_err(cam->dev, "Unsupported serializer: %u\n",
			cam->ser_type);
		return -EINVAL;
	}

	ursleep(des->param->t_lock);
	return 0;
}
