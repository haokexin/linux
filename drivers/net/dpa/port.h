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

#ifndef __PORT_H
#define __PORT_H

#include <linux/device.h>	/* struct device */

#include "fm.h"			/* struct fm_device */

struct port_bp_info {
	uint8_t	bpid;
	size_t	size;
};

struct port_device {
	struct device		*dev;
	void			*priv;
	uint8_t			 cell_index;
	struct resource		*res;
	void			*vaddr;
	uint16_t		 channel;

	struct fm_device	*fm_dev;

	int (*init)(struct port_device *port_dev);
	int (*set_default_fq)(struct port_device *port_dev, uint32_t fq);
	int (*set_error_fq)(struct port_device *port_dev, uint32_t fq);
	int (*set_bp_info)(struct port_device *port_dev, const struct port_bp_info *bp_info, uint8_t count);
	int (*start)(struct port_device *port_dev);
	int (*stop)(struct port_device *port_dev);
#ifdef CONFIG_BUG
	uint8_t (*get_lpid)(const struct port_device *port_dev);
#endif
	int (*uninit)(struct port_device *port_dev);
};

static inline void * __attribute((nonnull)) portdev_priv(const struct port_device *port_dev)
{
	return (void *)port_dev + sizeof(*port_dev);
}

extern const char	*port_driver_description;
extern const size_t	 port_sizeof_priv[];
extern void (*const port_setup[])(struct port_device *port_dev);

#endif	/* __PORT_H */
