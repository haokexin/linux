/* Copyright 2008-2011 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/device.h>
#include <linux/io.h>		/* devm_ioremap() */

#include "dpaa_eth-common.h"
#include "port.h"

static struct port_device * __devinit __cold
alloc_portdev(struct device *dev, size_t sizeof_priv,
		void (*setup)(struct port_device *port_dev))
{
	struct port_device *port_dev;

	port_dev = devm_kzalloc(dev, sizeof(*port_dev) + sizeof_priv,
			GFP_KERNEL);
	if (port_dev == NULL)
		return ERR_PTR(-ENOMEM);

	port_dev->dev = dev;
	dev_set_drvdata(dev, port_dev);
	setup(port_dev);

	return port_dev;
}

static int __devexit __cold free_portdev(struct port_device *port_dev)
{
	dev_set_drvdata(port_dev->dev, NULL);

	return likely(port_dev->uninit) ? port_dev->uninit(port_dev) : 0;
}

static const struct of_device_id port_match[] __devinitconst = {
	[RX] = {
		.compatible	= "fsl,fman-port-rx"
	},
	[TX] = {
		.compatible	= "fsl,fman-port-tx"
	},
	{}
};
MODULE_DEVICE_TABLE(of, port_match);

static const char port_type[][8] __devinitconst = {
	[RX] = "port-rx",
	[TX] = "port-tx"
};

static int __devinit __cold port_probe(struct of_device *_of_dev, const struct of_device_id *match)
{
	int			 _errno, i, lenp;
	struct device		*dev;
	struct device_node	*port_node, *fm_node;
	struct port_device	*port_dev;
	struct of_device	*of_dev;
	struct resource		 res;
	const uint32_t		*uint32_prop;

	dev = &_of_dev->dev;

	port_node = _of_dev->node;

	for (i = 0; i < ARRAY_SIZE(port_match) - 1 && match != port_match + i; i++);
	BUG_ON(i >= ARRAY_SIZE(port_match) - 1);

	port_dev = alloc_portdev(dev, port_sizeof_priv[i], port_setup[i]);
	if (IS_ERR(port_dev)) {
		dpaa_eth_err(dev, "alloc_portdev() = %d\n", _errno);
		_errno = PTR_ERR(port_dev);
		goto _return;
	}

	/* Get the FM node */
	fm_node = of_get_parent(port_node);
	if (unlikely(fm_node == NULL)) {
		dpaa_eth_err(dev, "of_get_parent(%s) failed\n",
				port_node->full_name);
		_errno = -EINVAL;
		goto _return_dev_set_drvdata;
	}

	of_dev = of_find_device_by_node(fm_node);
	if (unlikely(of_dev == NULL)) {
		dpaa_eth_err(dev, "of_find_device_by_node(%s) failed\n",
				fm_node->full_name);
		of_node_put(fm_node);
		_errno = -EINVAL;
		goto _return_dev_set_drvdata;
	}
	of_node_put(fm_node);

	port_dev->fm_dev = dev_get_drvdata(&of_dev->dev);
	BUG_ON(port_dev->fm_dev == NULL);

	/* Get the address of the memory mapped registers */
	_errno = of_address_to_resource(port_node, 0, &res);
	if (unlikely(_errno < 0)) {
		dpaa_eth_err(dev, "of_address_to_resource(%s) = %d\n",
				port_node->full_name, _errno);
		goto _return_dev_set_drvdata;
	}

	port_dev->res = __devm_request_region(dev, port_dev->fm_dev->res,
					      res.start, res.end + 1 - res.start, port_type[i]);
	if (unlikely(port_dev->res == NULL)) {
		dpaa_eth_err(dev, "__devm_request_mem_region(port) failed\n");
		_errno = -EBUSY;
		goto _return_dev_set_drvdata;
	}

	port_dev->vaddr = devm_ioremap(dev, port_dev->res->start,
				       port_dev->res->end + 1 - port_dev->res->start);
	if (unlikely(port_dev->vaddr == NULL)) {
		dpaa_eth_err(dev, "devm_ioremap() failed\n");
		_errno = -EIO;
		goto _return_dev_set_drvdata;
	}

	uint32_prop = of_get_property(port_node, "cell-index", &lenp);
	if (unlikely(uint32_prop == NULL)) {
		dpaa_eth_err(dev, "of_get_property(%s, cell-index) failed\n",
				port_node->full_name);
		_errno = -EINVAL;
		goto _return_dev_set_drvdata;
	}
	BUG_ON(lenp != sizeof(uint32_t));
	port_dev->cell_index = *uint32_prop;

	if (match != port_match + RX) {
		uint32_prop = of_get_property(port_node,
				"fsl,qman-channel-id", &lenp);
		if (unlikely(uint32_prop == NULL)) {
			dpaa_eth_err(dev,
				"of_get_property(%s, fsl,qman-channel-id)"
				" failed\n", port_node->full_name);
			_errno = -EINVAL;
			goto _return_dev_set_drvdata;
		}
		BUG_ON(lenp != sizeof(uint32_t));
		port_dev->channel = *uint32_prop;
	}

	_errno = port_dev->init(port_dev);
	if (unlikely(_errno < 0)) {
		dpaa_eth_err(dev, "port_dev->init() = %d\n", _errno);
		goto _return_dev_set_drvdata;
	}

	goto _return;

_return_dev_set_drvdata:
	dev_set_drvdata(dev, NULL);
_return:
	return _errno;
}

static int __devexit __cold port_remove(struct of_device *of_dev)
{
	int		 _errno;
	struct device	*dev;

	dev = &of_dev->dev;

	_errno = free_portdev((struct port_device *)dev_get_drvdata(dev));

	return _errno;
}

static struct of_platform_driver port_driver = {
	.name		= KBUILD_MODNAME,
	.match_table	= port_match,
	.owner		= THIS_MODULE,
	.probe		= port_probe,
	.remove		= __devexit_p(port_remove)
};

static int __init __cold port_load(void)
{
	int	 _errno;

	cpu_pr_debug(KBUILD_MODNAME ": -> %s:%s()\n", __file__, __func__);

	cpu_pr_info(KBUILD_MODNAME ": %s (" VERSION ")\n", port_driver_description);

	_errno = of_register_platform_driver(&port_driver);
	if (unlikely(_errno < 0)) {
		cpu_pr_err(KBUILD_MODNAME ": %s:%hu:%s(): of_register_platform_driver() = %d\n",
			   __file__, __LINE__, __func__, _errno);
		goto _return;
	}

	goto _return;

_return:
	cpu_pr_debug(KBUILD_MODNAME ": %s:%s() ->\n", __file__, __func__);

	return _errno;
}
module_init(port_load);

static void __exit __cold port_unload(void)
{
	cpu_pr_debug(KBUILD_MODNAME ": -> %s:%s()\n", __file__, __func__);

	of_unregister_platform_driver(&port_driver);

	cpu_pr_debug(KBUILD_MODNAME ": %s:%s() ->\n", __file__, __func__);
}
module_exit(port_unload);
