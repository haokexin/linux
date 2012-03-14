/* Copyright 2011 Freescale Semiconductor, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef FSL_USDPAA_H
#define FSL_USDPAA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/uaccess.h>
#include <linux/ioctl.h>

#ifdef CONFIG_FSL_USDPAA

/* Character-device interface. NB: these definitions need to be duplicated in
 * user-space. It is all temporary until being replaced by HugeTLB. */
#define USDPAA_IOCTL_MAGIC 'u'
struct usdpaa_ioctl_get_region {
	uint64_t phys_start;
	uint64_t phys_len;
};
#define USDPAA_IOCTL_GET_PHYS_BASE \
	_IOR(USDPAA_IOCTL_MAGIC, 0x01, struct usdpaa_ioctl_get_region)

#ifdef __KERNEL__

/* Physical address range */
extern u64 usdpaa_phys_start;
extern u64 usdpaa_phys_size;

/* PFN versions */
extern unsigned long usdpaa_pfn_start;
extern unsigned long usdpaa_pfn_len;

/* TLB1 index */
extern unsigned int usdpaa_tlbcam_index;

/* Early-boot hook */
void __init fsl_usdpaa_init_early(void);

#endif /* __KERNEL__ */

#endif /* CONFIG_FSL_USDPAA */

#ifdef __cplusplus
}
#endif

#endif /* FSL_USDPAA_H */
