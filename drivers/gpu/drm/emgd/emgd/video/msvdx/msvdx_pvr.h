/*
 *-----------------------------------------------------------------------------
 * Filename: msvdx_pvr.h
 * $Revision: 1.6 $
 *-----------------------------------------------------------------------------
 * Copyright (c) 2002-2010, Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 *-----------------------------------------------------------------------------
 * Description:
 *
 *-----------------------------------------------------------------------------
 */

#ifndef MSVDX_POWER_H_
#define MSVDX_POWER_H_

#include "services_headers.h"
#include "sysconfig.h"

extern struct drm_device *gpDrmDevice;

/* function define */
PVRSRV_ERROR MSVDXRegisterDevice(PVRSRV_DEVICE_NODE *psDeviceNode);
PVRSRV_ERROR MSVDXDevInitCompatCheck(PVRSRV_DEVICE_NODE *psDeviceNode);

/* power function define */
PVRSRV_ERROR MSVDXPrePowerState(IMG_HANDLE	hDevHandle,
			PVRSRV_DEV_POWER_STATE	eNewPowerState,
			PVRSRV_DEV_POWER_STATE	eCurrentPowerState);
PVRSRV_ERROR MSVDXPostPowerState(IMG_HANDLE	hDevHandle,
			 PVRSRV_DEV_POWER_STATE	eNewPowerState,
			 PVRSRV_DEV_POWER_STATE	eCurrentPowerState);
PVRSRV_ERROR MSVDXPreClockSpeedChange(IMG_HANDLE	hDevHandle,
			      IMG_BOOL			bIdleDevice,
			      PVRSRV_DEV_POWER_STATE	eCurrentPowerState);
PVRSRV_ERROR MSVDXPostClockSpeedChange(IMG_HANDLE	hDevHandle,
			       IMG_BOOL			bIdleDevice,
			       PVRSRV_DEV_POWER_STATE	eCurrentPowerState);
PVRSRV_ERROR MSVDXInitOSPM(PVRSRV_DEVICE_NODE *psDeviceNode);

#endif /* !MSVDX_POWER_H_ */
