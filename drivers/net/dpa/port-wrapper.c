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
#include <linux/io.h>		/* in_be32() */
#include <linux/of_platform.h>

#include "dpaa_eth-common.h"
#include "fm.h"			/* fmdev_priv() */
#include "fm-wrapper.h"		/* struct fm_priv_s */
#include "port.h"

#include "fm_common.h"		/* BASE_RX/TX_PORTID */
#include "lnxwrp_fm.h"		/* t_LnxWrpFmPortDev */
#include "lnxwrp_fm_ext.h"

#define PORT_DESCRIPTION "FSL FMan port wrapper based driver"

MODULE_LICENSE("Dual BSD/GPL");

MODULE_AUTHOR("Emil Medve <Emilian.Medve@Freescale.com>");

MODULE_DESCRIPTION(PORT_DESCRIPTION);

struct port_priv_s {
	t_LnxWrpFmPortDev		*port;
	t_SysObjectAdvConfigEntry	 config[FM_MAX_NUM_OF_ADV_SETTINGS];
};

struct rx_priv_s {
	struct port_priv_s	port;
	fm_port_rx_params_t	param;
};

struct tx_priv_s {
	struct port_priv_s	port;
	fm_port_non_rx_params_t	param;
};

const char	*port_driver_description __initconst = PORT_DESCRIPTION;
const size_t	 port_sizeof_priv[] __devinitconst = {
	[RX] = sizeof(struct rx_priv_s),
	[TX] = sizeof(struct tx_priv_s)
};

/* RX */

static int __devinit __cold __attribute__((nonnull))
init(const struct port_device *port_dev, uint8_t logical_index)
{
	struct port_priv_s	*priv;

	priv = portdev_priv(port_dev);

	priv->port->id		= logical_index;
	priv->port->baseAddr	= (typeof(priv->port->baseAddr))port_dev->vaddr;
	priv->port->memSize	= port_dev->res->end + 1 - port_dev->res->start;

	priv->port->settings.param.fmId		= port_dev->fm_dev->cell_index;
	priv->port->settings.param.portId	= logical_index;
	priv->port->settings.param.h_App	= priv->port;

	priv->port->settings.advConfig		= priv->config;

	priv->port->h_LnxWrpFmDev	= &((struct fm_priv_s *)fmdev_priv(port_dev->fm_dev))->fm;

	return 0;
}

static int __devinit __cold init_rx(struct port_device *port_dev)
{
	struct rx_priv_s	*priv;
	uint8_t			 logical_index;

	priv = portdev_priv(port_dev);

	logical_index = port_dev->cell_index - BASE_RX_PORTID;
	priv->port.port =
		((struct fm_priv_s *)fmdev_priv(port_dev->fm_dev))->fm.rxPorts + logical_index;

	priv->port.port->settings.param.portType	= e_FM_PORT_TYPE_RX;

	/* Ugly hack! */
	priv->port.port->settings.param.specificParams.rxParams.rxPartitionId	=
		in_be32(port_dev->fm_dev->vaddr + 0x80000 + 0x300 +
			sizeof(uint32_t) * port_dev->cell_index);

	priv->param.priv_data_size	= 16;
	priv->param.parse_results	= true;

	priv->port.port->defPcd	= e_FM_PCD_PARSE_RESULTS;

	return init(port_dev, logical_index);
}

static int __cold __attribute__((nonnull)) _init_rx(const struct port_device *port_dev)
{
	const struct rx_priv_s	*priv;

	priv = portdev_priv(port_dev);

	priv->port.port->active	= true;

	fm_set_rx_port_params(priv->port.port, &priv->param);

	return 0;
}

static int __devinit __cold set_default_fq_rx(struct port_device *port_dev, uint32_t fq)
{
	((struct rx_priv_s *)portdev_priv(port_dev))->param.defq = fq;
	return 0;
}

static int __devinit __cold set_error_fq_rx(struct port_device *port_dev, uint32_t fq)
{
	((struct rx_priv_s *)portdev_priv(port_dev))->param.errq = fq;
	return 0;
}

static int __devinit __cold
set_bp_info_rx(struct port_device *port_dev, const struct port_bp_info *bp_info, uint8_t count)
{
	int			 i;
	struct rx_priv_s	*priv;

	priv = portdev_priv(port_dev);

	priv->param.num_pools = min(count,
			(uint8_t)ARRAY_SIZE(priv->param.pool_param));
	for (i = 0; i < priv->param.num_pools; i++) {
		priv->param.pool_param[i].id	= bp_info[i].bpid;
		priv->param.pool_param[i].size	= bp_info[i].size;
	}

	return 0;
}

static int __cold start_rx(struct port_device *port_dev)
{
	int			 _errno;
	const struct rx_priv_s	*priv;

	priv = portdev_priv(port_dev);

	if (unlikely(!priv->port.port->active)) {
		_errno = _init_rx(port_dev);
		if (unlikely(_errno < 0))
			goto _return;
	}

	fm_port_enable(priv->port.port);
	_errno = 0;

_return:
	return _errno;
}

static int __cold stop(struct port_device *port_dev)
{
	fm_port_disable(((struct port_priv_s *)portdev_priv(port_dev))->port);
	return 0;
}

#ifdef CONFIG_BUG
static uint8_t get_lpid(const struct port_device *port_dev)
{
	return port_dev->fm_dev->cell_index << 6 | port_dev->cell_index;
}
#endif

/* TX */

static int __devinit __cold init_tx(struct port_device *port_dev)
{
	struct tx_priv_s	*priv;
	uint8_t			 logical_index;

	priv = portdev_priv(port_dev);

	logical_index = port_dev->cell_index - BASE_TX_PORTID;
	priv->port.port =
		((struct fm_priv_s *)fmdev_priv(port_dev->fm_dev))->fm.txPorts + logical_index;

	priv->port.port->settings.param.portType = e_FM_PORT_TYPE_TX;

	priv->port.port->settings.param.specificParams.nonRxParams.deqSubPortal	=
		port_dev->channel & 0xf;

	priv->port.port->defPcd	= e_NO_PCD;

	return init(port_dev, logical_index);
}

static int __cold __attribute__((nonnull)) _init_tx(const struct port_device *port_dev)
{
	const struct tx_priv_s	*priv;

	priv = portdev_priv(port_dev);

	priv->port.port->active	= true;

	fm_set_tx_port_params(priv->port.port, &priv->param);

	return 0;
}

static int __devinit __cold set_default_fq_tx(struct port_device *port_dev, uint32_t fq)
{
	((struct tx_priv_s *)portdev_priv(port_dev))->param.defq = fq;
	return 0;
}

static int __devinit __cold set_error_fq_tx(struct port_device *port_dev, uint32_t fq)
{
	((struct tx_priv_s *)portdev_priv(port_dev))->param.errq = fq;
	return 0;
}

static int __cold start_tx(struct port_device *port_dev)
{
	int			 _errno;
	const struct tx_priv_s	*priv;

	priv = portdev_priv(port_dev);

	if (unlikely(!priv->port.port->active)) {
		_errno = _init_tx(port_dev);
		if (unlikely(_errno < 0))
			goto _return;
	}

	fm_port_enable(priv->port.port);
	_errno = 0;

_return:
	return _errno;
}

static void __devinit __cold setup_rx(struct port_device *port_dev)
{
	port_dev->init			= init_rx;
	port_dev->start			= start_rx;
	port_dev->stop			= stop;
	port_dev->set_default_fq	= set_default_fq_rx;
	port_dev->set_error_fq		= set_error_fq_rx;
	port_dev->set_bp_info		= set_bp_info_rx;
#ifdef CONFIG_BUG
	port_dev->get_lpid		= get_lpid;
#endif
	port_dev->uninit		= NULL;
}

static void __devinit __cold setup_tx(struct port_device *port_dev)
{
	port_dev->init			= init_tx;
	port_dev->start			= start_tx;
	port_dev->stop			= stop;
	port_dev->set_default_fq	= set_default_fq_tx;
	port_dev->set_error_fq		= set_error_fq_tx;
	port_dev->set_bp_info		= NULL;
#ifdef CONFIG_BUG
	port_dev->get_lpid		= NULL;
#endif
	port_dev->uninit		= NULL;
}

void (*const port_setup[])(struct port_device *port_dev) __devinitconst = {
	[RX] = setup_rx,
	[TX] = setup_tx
};
