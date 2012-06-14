/*
 *	Fusion Kernel Module
 *
 *	(c) Copyright 2002-2003  Convergence GmbH
 *
 *      Written by Denis Oliver Kropp <dok@directfb.org>
 *
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */
 
#ifndef __FUSION__SHMPOOL_H__
#define __FUSION__SHMPOOL_H__

#include "fusiondev.h"
#include "types.h"

#define FUSION_SHM_BASE    0x20010000     /* virtual base address */
#define FUSION_SHM_SIZE    0x1FFEF000     /* size of virtual address space */

/* module init/cleanup */

int  fusion_shmpool_init   (FusionDev *dev);
void fusion_shmpool_deinit (FusionDev *dev);


/* public API */

int fusion_shmpool_new      (FusionDev             *dev,
                             FusionSHMPoolNew      *pool);

int fusion_shmpool_attach   (FusionDev             *dev,
                             FusionSHMPoolAttach   *attach,
                             FusionID               fusion_id);

int fusion_shmpool_detach   (FusionDev             *dev,
                             int                    id,
                             FusionID               fusion_id);

int fusion_shmpool_dispatch (FusionDev             *dev,
                             FusionSHMPoolDispatch *dispatch,
                             Fusionee              *fusionee );

int fusion_shmpool_destroy  (FusionDev             *dev,
                             int                    id);


/* internal functions */

void fusion_shmpool_detach_all (FusionDev *dev,
                                FusionID   fusion_id);

int  fusion_shmpool_fork_all   (FusionDev *dev,
                                FusionID   fusion_id,
                                FusionID   from_id);

#endif

