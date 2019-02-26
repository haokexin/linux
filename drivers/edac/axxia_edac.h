/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (C) 2021 INTEL Corporation
 */

#ifndef DRIVERS_EDAC_AXXIA_EDAC_H_
#define DRIVERS_EDAC_AXXIA_EDAC_H_

void edac_device_handle_multi_ce(struct edac_device_ctl_info *edac_dev,
				 int inst_nr, int block_nr, int events,
				 const char *msg);

void edac_device_handle_multi_ue(struct edac_device_ctl_info *edac_dev,
				 int inst_nr, int block_nr, int events,
				 const char *msg);

#endif /* DRIVERS_EDAC_AXXIA_EDAC_H_ */
