/* device.c - Device Configuration VBI */

/*
 * Copyright (c) 2010 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 */


/*
 * DESCRIPTION
 *
 * VBI calls used by Guest OSs to discover Guest Devices
 *
 */

#include <linux/types.h>
#include <vbi/vbi.h>
#include <vbi/syscall.h>
#include <vbi/pdc.h>
#include <vbi/syscalls.h>
#include <vbi/device.h>
#include <linux/kernel.h>

#undef DEBUG

#ifdef DEBUG
#define DEBUG_PRINTF(fmt, args...) do { printk("%s: %d: %s: " fmt, \
					VBI_BOARD_NAME_GET(), 	     \
					VBI_BOARD_ID_GET(), __FUNCTION__, \
					## args); } while (0)
#define PRINT_IF_CALLED() DEBUG_PRINTF("called\n")
#else
#define DEBUG_PRINTF(fmt, args...)
#define PRINT_IF_CALLED()
#endif



/*
 *
 * vbi_dev_count - get the number of devices
 *
 */

uint32_t vbi_dev_count(void)
{
	struct vb_config *config = VBI_CONFIG_ADDR_GET();

	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_dev_count);
		return 0;
	}
	return config->numDevices;
}

/*
 *
 * vbi_get_dev - get device details
 *
 */

uint32_t vbi_get_dev(uint32_t deviceIndex, struct vb_dev_info **vbiDevInfo)
{
	struct vb_config *config = VBI_CONFIG_ADDR_GET();
	struct vb_dev_info *pDevInfo = config->deviceConfiguration;
	uint32_t retVal = -1;

	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_get_dev);
		return retVal;
	}

	/* validate the input device index */

	if (deviceIndex < config->numDevices) {
		/* make sure the device configuration is valid */

		if (pDevInfo) {
			/* yes, return the pointer to device info */

			*vbiDevInfo = &pDevInfo[deviceIndex];
			retVal = 0;
		}
	}

	return retVal;
}

/*
 *
 * vbi_get_dev_interrupt - get the details of an Interrupt for a device
 *
 */

uint32_t vbi_get_dev_interrupt(uint32_t deviceIndex, uint32_t intIndex,
					struct vb_dev_int_info **vbiIntDetails)
{
	struct vb_config *config = VBI_CONFIG_ADDR_GET();
	struct vb_dev_info *pDevInfo;
	struct vb_dev_int_info *pDevIntInfo;
	uint32_t retVal = -1;

	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_get_dev_interrupt);
		return retVal;
	}
	/* Is the input device index valid */

	if (deviceIndex < config->numDevices) {
		/* yes, valid device index, get device info */

		pDevInfo = &config->deviceConfiguration[deviceIndex];

		/* is the device info valid */

		if (pDevInfo) {
			/* yes, device info is valid. get interrupt info */

			pDevIntInfo = (struct vb_dev_int_info *)
				((char *)pDevInfo + pDevInfo->intInfoOffset);
			/* is the interrupt info valid */

			if (pDevIntInfo) {
				/*
				 * yes, interrupt info is valid, check the
				 * interrupt index
				 */

				if (intIndex < pDevInfo->numInterrupts) {
					/*
					 * valid interrupt index, return
					 * pointer to interrupt info
					 */

					*vbiIntDetails = &pDevIntInfo[intIndex];
					retVal = 0;
				}
			}
		}
	}

	return retVal;
}


/*
 *
 * vbi_get_dev_registers - get the details of a register set for a device
 *
 */

uint32_t vbi_get_dev_registers(uint32_t deviceIndex, uint32_t regSetIndex,
				struct vb_dev_regset_info **vbiRegSetDetails)
{
	struct vb_config *config = VBI_CONFIG_ADDR_GET();
	struct vb_dev_info *pDevInfo;
	struct vb_dev_regset_info *pDevRegInfo;
	uint32_t retVal = -1;

	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_get_dev_registers);
		return retVal;
	}

	/* Is the input device index valid */

	if (deviceIndex < config->numDevices) {

		/* yes, valid device index, get device info */

		pDevInfo = &config->deviceConfiguration[deviceIndex];

		/* is the device info valid */

		if (pDevInfo) {

			/* yes, device info is valid. get register set info */

			pDevRegInfo = (struct vb_dev_regset_info *)
				((char *)pDevInfo + pDevInfo->regSetInfoOffset);
			/* is the register set info valid */

			if (pDevRegInfo) {
				/*
				 * yes, register set info is valid,
				 * check the regset index
				 */

				if (regSetIndex < pDevInfo->numRegSets) {
					/*
					 * valid regset index, return
					 * pointer to regset info
					 */

					*vbiRegSetDetails =
						&pDevRegInfo[regSetIndex];
					retVal = 0;
				}
			}
		}
	}

	return retVal;
}
/*
 *
 * vbi_get_dev_device_tree_source - get the details of a device tree source for 
 *                                  a device
 *
 */
 
uint32_t vbi_get_dev_device_tree_source(uint32_t deviceIndex, 
					uint32_t deviceTreeSourceIndex, 
					struct vb_dev_device_tree_source_info
						**vbiDeviceTreeSourceDetails)
{
	struct vb_config *config = VBI_CONFIG_ADDR_GET();
	struct vb_dev_info *pDevInfo;
	struct vb_dev_device_tree_source_info *pDevDTSInfo;
	uint32_t retVal = -1;

	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_get_dev_device_tree_source);
		return retVal;
	}

	/* Is the input device index valid */

	if (deviceIndex < config->numDevices) {
		/* yes, valid device index, get device info */

		pDevInfo = &config->deviceConfiguration[deviceIndex];

		/* is the device info valid */

		if (pDevInfo) {

			/* device info is valid. get device-tree-source info */

			pDevDTSInfo = (struct vb_dev_device_tree_source_info *)
					((char *)pDevInfo + 
					 pDevInfo->deviceTreeSourceInfoOffset);    
			/* is the device-tree-source info valid */

			if (pDevDTSInfo) {

				/*
				 * device-tree-source info is valid,
				 * check the devicetreesource index
				 */

				if (deviceTreeSourceIndex <
					pDevInfo->numDeviceTreeSources) {

					/*
					 * valid devicetreesource index, return
					 * pointer to devicetreesource info
					 */

					*vbiDeviceTreeSourceDetails =
					   &pDevDTSInfo[deviceTreeSourceIndex];
					retVal = 0;
				}
			}
		}
	}
    
	return retVal;
}
