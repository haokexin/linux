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

#ifndef __FM_H
#define __FM_H

#include <linux/ioport.h>	/* struct resource */
#include <linux/of_device.h>	/* struct of_device */
#include <linux/irqreturn.h>	/* irqreturn_t */

#define FM_FD_STAT_DME	       0x01000000      /* DMA Error - QMI */
#define FM_FD_STAT_FHE	       0x00080000      /* Physical Error - BMI */
#define FM_FD_STAT_FSE	       0x00040000      /* Frame Size Error - BMI */
#define FM_FD_STAT_DIS	       0x00020000      /* Discarded frame - BMI */
#define FM_FD_STAT_EOF	       0x00008000      /* Extract Out of Frame - KEYGEN */
#define FM_FD_STAT_NSS	       0x00004000      /* No Scheme Selected - KEYGEN */
#define FM_FD_STAT_FCL	       0x00000c00      /* Frame Color - Policer */
#define FM_FD_STAT_IPP	       0x00000200      /* Illegal Policer Profile - Policer */
#define FM_FD_STAT_PTE	       0x00000080      /* Parser Timeout Exceeded - Parser */
#define FM_FD_STAT_ISP	       0x00000040      /* Invalid Soft Parser Instruction - Parser */
#define FM_FD_STAT_PHE	       0x00000020      /* Parsing Header Error - Parser */
#define FM_FD_STAT_ERRORS      (FM_FD_STAT_DME | FM_FD_STAT_FHE | FM_FD_STAT_FSE |     \
				FM_FD_STAT_DIS | FM_FD_STAT_EOF | FM_FD_STAT_NSS |     \
				FM_FD_STAT_IPP | FM_FD_STAT_PTE | FM_FD_STAT_ISP |     \
				FM_FD_STAT_PHE)

#define FM_FD_CMD_FCO  0x80000000      /* Frame queue Context Override */
#define FM_FD_CMD_RPD  0x40000000      /* Read Prepended Data */
#define FM_FD_CMD_UDP  0x20000000      /* Update Prepended Data */
#define FM_FD_CMD_BMF  0x10000000      /* Buffer Must not be Freed */
#define FM_FD_CMD_DTC  0x08000000      /* Do TCP Checksum */
#define FM_FD_CMD_DME  0x01000000      /* DMA Error */
#define FM_FD_CMD_CFQ  0x00ffffff      /* Confirmation Frame Queue */

/* Parse results memory layout */
struct fman_parse_results {
	uint8_t		lpid;
	uint8_t		shimr;
	uint16_t	l2r;
	uint16_t	l3r;
	uint8_t		l4r;
	uint8_t		cplan;
	uint16_t	nxthdr;
	uint16_t	cksum;
	uint32_t	lcv;
	uint8_t		shim_off[3];
	uint8_t		eth_off;
	uint8_t		llc_snap_off;
	uint8_t		vlan_off;
	uint8_t		etype_off;
	uint8_t		pppoe_off;
	uint8_t		mpls_off;
	uint8_t		ip_off;
	uint8_t		gre_off;
	uint8_t		l4_off;
	uint8_t		nxthdr_off;
} __packed;

struct fm_device {
	struct device	*dev;
	void		*priv;
	uint8_t		 cell_index;
	struct resource	*res;
	void		*vaddr;

	struct resource	*muram_res;
	void		*muram_vaddr;
	struct resource	*parser_res;
	void		*parser_vaddr;
	struct resource	*keygen_res;
	void		*keygen_vaddr;
	struct resource	*policer_res;
	void		*policer_vaddr;

	int (*init)(struct fm_device *fm_dev);
	irqreturn_t (*isr)(int irq, void *_fm_dev);
	int (*uninit)(struct fm_device *fm_dev);
};

static inline void * __attribute((nonnull)) fmdev_priv(const struct fm_device *fm_dev)
{
	return (void *)fm_dev + sizeof(*fm_dev);
}

extern const char	*fm_driver_description;
extern const size_t	 fm_sizeof_priv;
extern void (* const fm_setup)(struct fm_device *fm_dev);

#endif	/* __FM_H */
