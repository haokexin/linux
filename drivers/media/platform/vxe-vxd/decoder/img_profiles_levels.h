/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD DEC SYSDEV and UI Interface header
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 */

#ifndef __IMG_PROFILES_LEVELS_H
#define __IMG_PROFILES_LEVELS_H

#include "vdecdd_utils.h"

/* Minimum level value for h.264 */
#define H264_LEVEL_MIN              (9)
/* Maximum level value for h.264 */
#define H264_LEVEL_MAX             (52)
/* Number of major levels for h.264 (5 + 1 for special levels) */
#define H264_LEVEL_MAJOR_NUM        (6)
/* Number of minor levels for h.264 */
#define H264_LEVEL_MINOR_NUM            (4)
/* h.264 Baseline/Constrained Baseline profile id.  */
#define H264_PROFILE_BASELINE      (66)
/* h.264 Main profile id.                           */
#define H264_PROFILE_MAIN          (77)
/* h.264 Extended profile id.                       */
#define H264_PROFILE_EXTENDED      (88)
/* h.264 High profile id.                           */
#define H264_PROFILE_HIGH         (100)
/* h.264 High 4:4:4 profile id.                     */
#define H264_PROFILE_HIGH444      (244)
/* h.264 High 4:2:2 profile id.                     */
#define H264_PROFILE_HIGH422      (122)
/* h.264 High 10 profile id.                        */
#define H264_PROFILE_HIGH10       (110)
/* h.264 CAVLC 4:4:4 Intra profile id.              */
#define H264_PROFILE_CAVLC444   (44)
/* h.264 Multiview High profile id.                 */
#define H264_PROFILE_MVC_HIGH     (118)
/* h.264 Stereo High profile id.                    */
#define H264_PROFILE_STEREO_HIGH  (128)

/* HEVC related definitions */

/* Minimum level value for HEVC */
#define HEVC_LEVEL_MIN             (30)
/* Maximum level value for HEVC */
#define HEVC_LEVEL_MAX            (186)
/* Number of major levels for HEVC */
#define HEVC_LEVEL_MAJOR_NUM        (6)
/* Number of minor levels for HEVC */
#define HEVC_LEVEL_MINOR_NUM        (3)
/* HEVC Main general_profile_idc value */
#define HEVC_PROFILE_MAIN           (1)
/* HEVC Main 10 general_profile_idc value */
#define HEVC_PROFILE_MAIN10         (2)
/* HEVC Main Still Picture general_profile_idc value */
#define HEVC_PROFILE_MAINSP         (3)

#endif /*__IMG_PROFILES_LEVELS_H */
