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
#include <sysdev/fsl_soc.h>

#include "dpaa_eth-common.h"
#include "fm.h"
#include "fm-wrapper.h"

#include "lnxwrp_fm_ext.h"	/* fm_init() */

#define FM_DESCRIPTION "FSL FMan wrapper based driver"

MODULE_LICENSE("Dual BSD/GPL");

MODULE_AUTHOR("Emil Medve <Emilian.Medve@Freescale.com>");

MODULE_DESCRIPTION(FM_DESCRIPTION);

const char	*fm_driver_description __initconst = FM_DESCRIPTION;
const size_t	 fm_sizeof_priv __devinitconst = sizeof(struct fm_priv_s);

static void fm_exception(t_Handle _fm_dev, e_FmExceptions exception)
{
	struct fm_device	*fm_dev;

	fm_dev = (struct fm_device *)_fm_dev;

	cpu_dev_dbg(fm_dev->dev, "-> %s:%s()\n", __file__, __func__);

	cpu_dev_dbg(fm_dev->dev, "%s:%s() ->\n", __file__, __func__);
}

static void fm_bus_error(t_Handle	_fm_dev,
			 e_FmPortType	port_type,
			 uint8_t	cell_index,
			 uint64_t	addr,
			 uint8_t	tnum,
			 uint8_t	partition)
{
	struct fm_device	*fm_dev;

	fm_dev = (struct fm_device *)_fm_dev;

	cpu_dev_dbg(fm_dev->dev, "-> %s:%s()\n", __file__, __func__);

	cpu_dev_dbg(fm_dev->dev, "%s:%s() ->\n", __file__, __func__);
}

static int __devinit __cold init(struct fm_device *fm_dev)
{
	struct fm_priv_s	*priv;
	int			 i;

	priv = fmdev_priv(fm_dev);

	priv->fm.id			= fm_dev->cell_index;
	priv->fm.prsActive		= fm_dev->parser_res != NULL;
	priv->fm.kgActive		= fm_dev->keygen_res != NULL;
	priv->fm.plcrActive		= fm_dev->policer_res != NULL;
	priv->fm.pcdActive		= priv->fm.prsActive	||
					  priv->fm.kgActive	||
					  priv->fm.plcrActive;
	priv->fm.fmBaseAddr		= (typeof(priv->fm.fmBaseAddr))fm_dev->vaddr;
	priv->fm.fmMemSize		= fm_dev->res->end + 1 - fm_dev->res->start;
	priv->fm.fmMuramBaseAddr	= (typeof(priv->fm.fmMuramBaseAddr))fm_dev->muram_vaddr;
	priv->fm.fmMuramMemSize		= fm_dev->muram_res->end + 1 - fm_dev->muram_res->start;

	priv->fm.fmDevSettings.param.fmId		= fm_dev->cell_index;
	priv->fm.fmDevSettings.param.h_App		= &priv->fm;
	priv->fm.fmDevSettings.param.fmClkFreq		= fsl_get_sys_freq();

	/* Ugly hack! */
	for (i = 0; i < ARRAY_SIZE(priv->fm.fmDevSettings.param.liodnPerPartition); i++)
		priv->fm.fmDevSettings.param.liodnPerPartition[i] =
			in_be32(fm_dev->vaddr + 0xc2000 + 0x60	+ sizeof(uint32_t) * (i / 2)) >>
			16 * (1 - (i & 1));

	priv->fm.fmDevSettings.param.f_Exceptions	= fm_exception;
	priv->fm.fmDevSettings.param.f_BusError		= fm_bus_error;

	priv->fm.fmDevSettings.advConfig	= priv->fm_config;

	priv->fm.fmPcdDevSettings.advConfig	= priv->pcd_config;

	priv->fm.active	= true;

	fm_init(&priv->fm);

	return 0;
}

static irqreturn_t isr(int irq, void *_fm_dev)
{
	irqreturn_t		 _errno;
	struct fm_device	*fm_dev;
	struct fm_priv_s	*priv;

	fm_dev = _fm_dev;

	priv = fmdev_priv(fm_dev);

	if (likely(priv->fm.h_Dev != NULL)) {
		FM_Isr(priv->fm.h_Dev);
		_errno = IRQ_HANDLED;
	} else
		_errno = IRQ_NONE;

	return _errno;
}

static void __devinit __cold setup(struct fm_device *fm_dev)
{
	fm_dev->init	= init;
	fm_dev->isr	= isr;
	fm_dev->uninit	= NULL;
}

void (*const fm_setup)(struct fm_device *fm_dev) __devinitconst = setup;
