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

#ifndef __DPA_COMMON_H
#define __DPA_COMMON_H

#include <linux/kernel.h>	/* pr_*() */
#include <linux/device.h>	/* dev_*() */
#include <linux/smp.h>		/* hard_smp_processor_id() */
#ifndef CONFIG_SMP
#include <asm/smp.h>		/* hard_smp_processor_id() */
#endif

#define __file__ KBUILD_BASENAME".c"	/* The basename of the source file is being compiled */

#define __hot

#define cpu_pr_emerg(format, arg...)	\
	pr_emerg("cpu%d/%d: " format, hard_smp_processor_id(), smp_processor_id(), ##arg)
#define cpu_pr_alert(format, arg...)	\
	pr_alert("cpu%d/%d: " format, hard_smp_processor_id(), smp_processor_id(), ##arg)
#define cpu_pr_crit(format, arg...)	\
	pr_crit("cpu%d/%d: " format, hard_smp_processor_id(), smp_processor_id(), ##arg)
#define cpu_pr_err(format, arg...)	\
	pr_err("cpu%d/%d: " format, hard_smp_processor_id(), smp_processor_id(), ##arg)
#define cpu_pr_warning(format, arg...)	\
	pr_warning("cpu%d/%d: " format, hard_smp_processor_id(), smp_processor_id(), ##arg)
#define cpu_pr_notice(format, arg...)	\
	pr_notice("cpu%d/%d: " format, hard_smp_processor_id(), smp_processor_id(), ##arg)
#define cpu_pr_info(format, arg...)	\
	pr_info("cpu%d/%d: " format, hard_smp_processor_id(), smp_processor_id(), ##arg)
#define cpu_pr_debug(format, arg...)	\
	pr_debug("cpu%d/%d: " format, hard_smp_processor_id(), smp_processor_id(), ##arg)

/* Keep these in sync with the dev_*() definitions from linux/device.h */
#define cpu_dev_emerg(dev, format, arg...)	\
	cpu_pr_emerg("%s: %s: " format, dev_driver_string(dev), dev_name(dev) , ##arg)
#define cpu_dev_alert(dev, format, arg...)	\
	cpu_pr_alert("%s: %s: " format, dev_driver_string(dev), dev_name(dev) , ##arg)
#define cpu_dev_crit(dev, format, arg...)	\
	cpu_pr_crit("%s: %s: " format, dev_driver_string(dev), dev_name(dev) , ##arg)
#define cpu_dev_err(dev, format, arg...)	\
	cpu_pr_err("%s: %s: " format, dev_driver_string(dev), dev_name(dev) , ##arg)
#define cpu_dev_warn(dev, format, arg...)	\
	cpu_pr_warning("%s: %s: " format, dev_driver_string(dev), dev_name(dev) , ##arg)
#define cpu_dev_notice(dev, format, arg...)	\
	cpu_pr_notice("%s: %s: " format, dev_driver_string(dev), dev_name(dev) , ##arg)
#define cpu_dev_info(dev, format, arg...)	\
	cpu_pr_info("%s: %s: " format, dev_driver_string(dev), dev_name(dev) , ##arg)
#define cpu_dev_dbg(dev, format, arg...)	\
	cpu_pr_debug("%s: %s: " format, dev_driver_string(dev), dev_name(dev) , ##arg)

#define cpu_netdev_emerg(net_dev, format, arg...)	\
	cpu_dev_emerg((net_dev)->dev.parent, "%s: " format, (net_dev)->name , ##arg)
#define cpu_netdev_alert(net_dev, format, arg...)	\
	cpu_dev_alert((net_dev)->dev.parent, "%s: " format, (net_dev)->name , ##arg)
#define cpu_netdev_crit(net_dev, format, arg...)	\
	cpu_dev_crit((net_dev)->dev.parent, "%s: " format, (net_dev)->name , ##arg)
#define cpu_netdev_err(net_dev, format, arg...)		\
	cpu_dev_err((net_dev)->dev.parent, "%s: " format, (net_dev)->name , ##arg)
#define cpu_netdev_warn(net_dev, format, arg...)	\
	cpu_dev_warn((net_dev)->dev.parent, "%s: " format, (net_dev)->name , ##arg)
#define cpu_netdev_notice(net_dev, format, arg...)	\
	cpu_dev_notice((net_dev)->dev.parent, "%s: " format, (net_dev)->name , ##arg)
#define cpu_netdev_info(net_dev, format, arg...)	\
	cpu_dev_info((net_dev)->dev.parent, "%s: " format, (net_dev)->name , ##arg)
#define cpu_netdev_dbg(net_dev, format, arg...)		\
	cpu_dev_dbg((net_dev)->dev.parent, "%s: " format, (net_dev)->name , ##arg)

enum {RX, TX};

#endif	/* __DPA_COMMON_H */
