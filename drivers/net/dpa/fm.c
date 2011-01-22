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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>		/* devm_ioremap() */

#include "dpaa_eth-common.h"
#include "fm.h"

static struct fm_device * __devinit __cold
alloc_fmdev(struct device *dev, size_t sizeof_priv, void (*setup)(struct fm_device *fm_dev))
{
	struct fm_device	*fm_dev;

	fm_dev = devm_kzalloc(dev, sizeof(*fm_dev) + sizeof_priv, GFP_KERNEL);
	if (likely(fm_dev == NULL))
		fm_dev = ERR_PTR(-ENOMEM);
	else {
		fm_dev->dev = dev;
		dev_set_drvdata(dev, fm_dev);
		setup(fm_dev);
	}

	return fm_dev;
}

static int __devexit __cold free_fmdev(struct fm_device *fm_dev)
{
	dev_set_drvdata(fm_dev->dev, NULL);

	return likely(fm_dev->uninit) ? fm_dev->uninit(fm_dev) : 0;
}

struct resource * __cold fm_mem_region(struct of_device *of_dev)
{
	return ((struct fm_device *)dev_get_drvdata(&of_dev->dev))->res;
}
EXPORT_SYMBOL(fm_mem_region);

const uint16_t irqf[] __devinitconst = {0};

static int __devinit __cold fm_probe(struct of_device *of_dev, const struct of_device_id *match)
{
	int			 _errno, interrupt, i, lenp;
	struct device		*dev;
	struct device_node	*fm_node, *dev_node;
	struct fm_device	*fm_dev;
	struct resource		 res;
	const uint32_t		*uint32_prop;

	dev = &of_dev->dev;

	fm_dev = alloc_fmdev(dev, fm_sizeof_priv, fm_setup);
	if (IS_ERR(fm_dev)) {
		_errno = PTR_ERR(fm_dev);
		dpaa_eth_err(dev, "alloc_fmdev() = %d\n", _errno);
		goto _return;
	}

	fm_node = of_dev->node;

	/* Get the FM interrupts */
	for (i = 0; i < ARRAY_SIZE(irqf); i++) {
		interrupt = of_irq_to_resource(fm_node, i, NULL);
		if (unlikely(interrupt == NO_IRQ)) {
			dpaa_eth_err(dev,
				"of_irq_to_resource(%d) = NO_IRQ\n", i);
			_errno = -EINVAL;
			goto _return_dev_set_drvdata;
		}

		if (unlikely(!can_request_irq(interrupt, irqf[i])))
			dpaa_eth_warn(dev, "can_request_irq(%d) failed\n", i);
		_errno = devm_request_irq(dev, interrupt, fm_dev->isr, irqf[i], "fman", fm_dev);
		if (unlikely(_errno < 0)) {
			dpaa_eth_err(dev, "devm_request_irq(%d) = %d\n",
					i, _errno);
			goto _return_dev_set_drvdata;
		}
	}

	/* Get the port/device address */
	_errno = of_address_to_resource(fm_node, 0, &res);
	if (unlikely(_errno < 0)) {
		dpaa_eth_err(dev, "of_address_to_resource() = %d\n", _errno);
		goto _return_dev_set_drvdata;
	}

	fm_dev->res = devm_request_mem_region(dev, res.start, res.end + 1 - res.start, "fman");
	if (unlikely(fm_dev->res == NULL)) {
		dpaa_eth_err(dev, "devm_request_mem_region(fman) failed\n");
		_errno = -EBUSY;
		goto _return_dev_set_drvdata;
	}

	fm_dev->vaddr = devm_ioremap(dev,
				     fm_dev->res->start, fm_dev->res->end + 1 - fm_dev->res->start);
	if (unlikely(fm_dev->vaddr == 0)) {
		dpaa_eth_err(dev, "devm_ioremap() failed\n");
		_errno = -EIO;
		goto _return_dev_set_drvdata;
	}

	uint32_prop = of_get_property(fm_node, "cell-index", &lenp);
	if (unlikely(uint32_prop == NULL)) {
		dpaa_eth_err(dev, "of_get_property(%s, cell-index) failed\n",
				fm_node->full_name);
		_errno = -EINVAL;
		goto _return_dev_set_drvdata;
	}
	BUG_ON(lenp != sizeof(uint32_t));
	fm_dev->cell_index = *uint32_prop;

	dev_node = of_find_compatible_node(of_node_get(fm_node), NULL, "fsl,fman-muram");
	if (unlikely(dev_node == NULL)) {
		dpaa_eth_err(dev,
			"of_find_compatible_node(fsl,fman-muram) failed\n");
		_errno = -EINVAL;
		goto _return_dev_set_drvdata;
	}

	_errno = of_address_to_resource(dev_node, 0, &res);
	if (unlikely(_errno < 0)) {
		dpaa_eth_err(dev, "of_address_to_resource(%s) = %d\n",
				dev_node->full_name, _errno);
		goto _return_of_node_put;
	}
	of_node_put(dev_node);

	fm_dev->muram_res = __devm_request_region(dev, fm_dev->res,
						  res.start, res.end + 1 - res.start, "muram");
	if (unlikely(fm_dev->res == NULL)) {
		dpaa_eth_err(dev, "__devm_request_region(muram) failed\n");
		_errno = -EBUSY;
		goto _return_dev_set_drvdata;
	}

	fm_dev->muram_vaddr = devm_ioremap(dev, fm_dev->muram_res->start,
					   fm_dev->muram_res->end + 1 - fm_dev->muram_res->start);
	if (unlikely(fm_dev->muram_vaddr == 0)) {
		dpaa_eth_err(dev, "devm_ioremap() failed\n");
		_errno = -EIO;
		goto _return_dev_set_drvdata;
	}

	dev_node = of_find_compatible_node(of_node_get(fm_node), NULL, "fsl,fman-parser");
	if (unlikely(dev_node == NULL))
		dpaa_eth_err(dev,
			"of_find_compatible_node(fsl,fman-parser) failed. "
			"Won't be using the parser\n");
	else {
		_errno = of_address_to_resource(dev_node, 0, &res);
		if (unlikely(_errno < 0)) {
			dpaa_eth_err(dev, "of_address_to_resource(%s) = %d\n",
					dev_node->full_name, _errno);
			goto _return_of_node_put;
		}
		of_node_put(dev_node);

		fm_dev->parser_res = __devm_request_region(dev, fm_dev->res,
							   res.start, res.end + 1 - res.start,
							   "parser");
		if (unlikely(fm_dev->res == NULL)) {
			dpaa_eth_err(dev,
				"__devm_request_region(parser) failed\n");
			_errno = -EBUSY;
			goto _return_dev_set_drvdata;
		}

		fm_dev->parser_vaddr = devm_ioremap(
			dev,
			fm_dev->parser_res->start,
			fm_dev->parser_res->end + 1 - fm_dev->parser_res->start);
		if (unlikely(fm_dev->parser_vaddr == 0)) {
			dpaa_eth_err(dev, "devm_ioremap() failed\n");
			_errno = -EIO;
			goto _return_dev_set_drvdata;
		}
	}

	dev_node = of_find_compatible_node(of_node_get(fm_node), NULL,
						"fsl,fman-keygen");
	if (unlikely(dev_node == NULL)) {
		dpaa_eth_err(dev,
			"of_find_compatible_node(fsl,fman-keygen) failed. "
			"Won't be using the KeyGen\n");
	} else {
		_errno = of_address_to_resource(dev_node, 0, &res);
		if (unlikely(_errno < 0)) {
			dpaa_eth_err(dev, "of_address_to_resource(%s) = %d\n",
					dev_node->full_name, _errno);
			goto _return_of_node_put;
		}
		of_node_put(dev_node);

		fm_dev->keygen_res = __devm_request_region(dev, fm_dev->res,
							   res.start, res.end + 1 - res.start,
							   "keygen");
		if (unlikely(fm_dev->res == NULL)) {
			dpaa_eth_err(dev,
				"__devm_request_region(keygen) failed\n");
			_errno = -EBUSY;
			goto _return_dev_set_drvdata;
		}

		fm_dev->keygen_vaddr = devm_ioremap(
			dev,
			fm_dev->keygen_res->start,
			fm_dev->keygen_res->end + 1 - fm_dev->keygen_res->start);
		if (unlikely(fm_dev->keygen_vaddr == 0)) {
			dpaa_eth_err(dev, "devm_ioremap() failed\n");
			_errno = -EIO;
			goto _return_dev_set_drvdata;
		}
	}

	dev_node = of_find_compatible_node(of_node_get(fm_node), NULL, "fsl,fman-policer");
	if (unlikely(dev_node == NULL))
		dpaa_eth_err(dev,
			"of_find_compatible_node(fsl,fman-policer) failed. "
			"Won't be using the policer\n");
	else {
		_errno = of_address_to_resource(dev_node, 0, &res);
		if (unlikely(_errno < 0)) {
			dpaa_eth_err(dev, "of_address_to_resource(%s) = %d\n",
					dev_node->full_name, _errno);
			goto _return_of_node_put;
		}
		of_node_put(dev_node);

		fm_dev->policer_res = __devm_request_region(dev, fm_dev->res,
							    res.start, res.end + 1 - res.start,
							    "policer");
		if (unlikely(fm_dev->res == NULL)) {
			dpaa_eth_err(dev,
				"__devm_request_region(policer) failed\n");
			_errno = -EBUSY;
			goto _return_dev_set_drvdata;
		}

		fm_dev->policer_vaddr = devm_ioremap(
			dev,
			fm_dev->policer_res->start,
			fm_dev->policer_res->end + 1 - fm_dev->policer_res->start);
		if (unlikely(fm_dev->policer_vaddr == 0)) {
			dpaa_eth_err(dev, "devm_ioremap() failed\n");
			_errno = -EIO;
			goto _return_dev_set_drvdata;
		}
	}

	_errno = fm_dev->init(fm_dev);
	if (unlikely(_errno < 0)) {
		dpaa_eth_err(dev, "port_dev->init() = %d\n", _errno);
		goto _return_dev_set_drvdata;
	}

	goto _return;

_return_of_node_put:
	of_node_put(dev_node);
_return_dev_set_drvdata:
	dev_set_drvdata(dev, NULL);
_return:
	return _errno;
}

static int __devexit __cold fm_remove(struct of_device *of_dev)
{
	int		 _errno;
	struct device	*dev;

	dev = &of_dev->dev;

	_errno = free_fmdev((struct fm_device *)dev_get_drvdata(dev));

	return _errno;
}

static const struct of_device_id fm_match[] __devinitconst = {
	{
		.compatible	= "fsl,fman"
	},
	{}
};
MODULE_DEVICE_TABLE(of, fm_match);

static struct of_platform_driver fm_driver = {
	.name		= KBUILD_MODNAME,
	.match_table	= fm_match,
	.owner		= THIS_MODULE,
	.probe		= fm_probe,
	.remove		= __devexit_p(fm_remove)
};

static int __init __cold fm_load(void)
{
	int	 _errno;

	cpu_pr_debug(KBUILD_MODNAME ": -> %s:%s()\n", __file__, __func__);

	cpu_pr_info(KBUILD_MODNAME ": %s (" VERSION ")\n", fm_driver_description);

	_errno = of_register_platform_driver(&fm_driver);
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
module_init(fm_load);

static void __exit __cold fm_unload(void)
{
	cpu_pr_debug(KBUILD_MODNAME ": -> %s:%s()\n", __file__, __func__);

	of_unregister_platform_driver(&fm_driver);

	cpu_pr_debug(KBUILD_MODNAME ": %s:%s() ->\n", __file__, __func__);
}
module_exit(fm_unload);
