/*
 * lib.c - virtual Board Interface Library
 *
 * Copyright (c) 2009 Wind River Systems, Inc.
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

DESCRIPTION
The vbi library (Virtual board interface) provides support functionality for
software developers who are writing Virtual board applications, or as a guide
to developers porting an operating system to the virtual board environment.
This file contains the generic API that architecture independent. Separate
documents are provided as supplement for APIs tied to a particular hardware
architecture platform.

A given system may be composed of multiple VB where each VB may contain multiple
cores. During system bring-up stage hypervisor VB manager allocates resources
specified in the XML configuration file for the each VB and initializes the
configuration structure. Then the cores are launched starting at the entry point
specified in the guest payload image. Hypervisor passes a pointer to the
configuration page and a 32bit flag that holds the boot options of the core.
These parameters are passed via stack or registers based on the underlying
architecture platform.


figure 1.1: A simplified overview diagram of a hypervisor system.

                           ________Hypercalls___||_______
       Virtual board 0    |                     ||       |
     _____________________V___                  ||       |
    |    _____         _____  |\                ||       |
    |   |Core |       |Core | | \     shared    ||    ___V____
    |   | 0   |       | n   | |  \    pages     ||   |        |
    |   |_____|       |_____| |   \   ______    ||   |        |
    |_________________________|    \ |______|   ||   |        |
                         ^           |______|--------|  H     |
                         |           |______|   ||   |  Y     |
                .        | interrupts           ||   |  P     |
                .         ---------------------------|  E     |
                .        |                      ||   |  R     |
       Virtual board N   |           ______     ||   |  V     |
     ____________________V____      |______|    ||   |  I     |
    |    _____         _____  |    /|______|----||---|  S     |
    |   |Core |       |Core | |   / |______|    ||   |  O     |
    |   | 0   |       | n   | |  /              ||   |  R     |
    |   |_____|       |_____| | /               ||   |________|
    |_________________________|/                ||        ^
                          ^                     ||        |
                          |                     ||        |
                          |_____________________||________|
                                                ||

                                                ^
                                                |
                                             guest/hypervisor
                                              speration line

Once the guest starts running it must call vbi_init() with the save parameters
passed to it's entry routine which is the base address of the configuration.
page. This routine retrieves the control and status page page pointers and
saves them in the corresponding global variables.

If the VMMU is turned on care must be taken to ensure that the memory regions
below are mapped via the VMMU. It is suggested to treat these regions as
devices regions where the mapping is flat and the cache attribute is guarded.

A virtual board core is presented by hypervisor with 3 memory areas:

1) Virtual Board Configuration Area

This write protected memory area provides fixed configuration information to
the virtual board.  The address of this is kept in the global variable
wr_config.  The address of this area is passed to the virtual board as the
first parameter when the virtual board is started running at it's boot location.

2) Virtual Board Status Area

This write protected memory area provides data to the virtual board that
changes due to run-time activity, such as registers saved due to an
interrupt, timestamps, virtual interrupts etc.  The address of this is
kept in the global variable wr_status.

3) Virtual Board Control Area

This writable memory area is used by the virtual board to communicate
information to the hypervisor, where they cannot be passed in a faster manner.
The address of this area is kept in the global variable wr_control.

*/

#include <linux/types.h>
#include <linux/module.h>
#include <vbi/vbi.h>
#include <vbi/private.h>

struct vb_config *wr_vb_config;	/* The address of the core's Config area
				 * This value is passed to the virtual board
				 * as the first parameter upon startup */
EXPORT_SYMBOL(wr_vb_config);

struct vb_control *wr_vb_control;/* The address of the core's Control area */

struct vb_status *wr_vb_status;	/* The address of the core's Status area */

EXPORT_SYMBOL(wr_vb_control);
EXPORT_SYMBOL(wr_vb_status);

/*
 * vbi_init - Initialize support for vbi library functions
 *
 * This routine initializes the vbi library
 *
 * The routine should be called before accessing the virtual board configuration
 * data or making any hypercall. The parameters passed to this function should
 * be same as the boot parameters passed to the first executing program in the
 * running core by hypervisor which is a pointer to the core configuration
 * address. This routine retrieves the status and control page from the
 * configuration page and initializes the following global variables for future
 * reference.
 *
 * Configuration page base address is stored in wr_vb_config
 * Status page base address is stored in wr_vb_status
 * Control page base address is stored in wr_vb_control
 *
 * The user must ensure that this memory regions are mapped if the MMU is
 * turn-on.
 * A good practice would be to map these regions as devices meaning 1-1
 * translation.
 *
 * The evolution of the vbi library is tracked by the versioning information.
 * The different versions of the vbi library are guaranteed to be at least
 * source level compatible. If additional features that compromise the backward
 * compatibility are required then solution that provides separate libraries
 * should be considered. In case the source level compatibility must be violated
 * in the future then vbi_init() must updated to trap an erroneous configuration
 * at runtime.
 *
 * VBI versioning mechanism is composed of three fragments.
 *
 * - Major version
 * - Minor version
 * - Maintenance version
 *
 * Each of these version fragments serves the purpose of tracking a specific
 * modification of the vbi library.
 *
 * The Major version is updated (incremented by one) when the vbi runtime code
 * modification has the effect of altering the binary compatibility with it's
 * predecessor. Which means that the VBI lib used in the guest must match the
 * version that hypervisor kernel was built against in order to be binary
 * compatible. In otherwords the major version fragments must be identical
 * between a guest OS and hypervisor. These type of mods may update the C
 * structures but conserve the vbi functions signatures. Additional APIs may be
 * added.
 *
 * The Minor version fragment is dedicated for tracking VBI updates that are
 * carefully done to prevent breaking Guest OS using a older version of the vbi
 * library. This implies that a Minor version updates must maintain the binary
 * compatibility between VBI versions. It is important to note that this is
 * applicable to only VBI 2.0 or later versions.
 * This type of updates should not alter the API or structures definitions.
 * The vbi library may be extended with new API's that are orthogonal with the
 * existing ones (the current version at any point in time). The existing
 * structures may be appended with new fields in a fashion not to impact the
 * current offsets.
 *
 * Finally the maintenance version is used for the purpose of fixing bugs on a
 * specific version of the vbi library. The purpose is to facility the customer
 * support effort.
 *
 * The source compatibility is assured for any version of the VBI lib. Binary
 * compatibility is guaranteed for the same minor and maint. versions if the
 * major version is fixed.
 *
 */
void vbi_init(struct vb_config *config)
{
	/*
	 * validate the vbi version; minor and maintenance should be backward
	 * compatible.
	 */
	if ((config->major != vbi_version_major) ||
					 (config->minor < vbi_version_minor))
		vbi_panic("Invalid vbi version");

	wr_vb_config = config;
	wr_vb_control = wr_vb_config->vb_control;
	wr_vb_status = wr_vb_config->vb_status;

}

/*
 *
 * vbi_vb_find_board_config - return board config of VB
 *
 * This function gets the board config guest address of the VB's configuration
 *
 */
int32_t vbi_vb_find_board_config(uint32_t board_id, int32_t core_id,
					void *paddr)
{
	/* Certifiable hypervisor does not support this function */
	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_vb_find_board_config);
		return -1;
	}
	return vbi_vb_remote(VBI_VBREMOTE_BOARDCONFIG, board_id, core_id,
				paddr);
}

