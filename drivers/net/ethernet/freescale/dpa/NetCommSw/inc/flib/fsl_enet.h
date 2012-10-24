/*
 * Copyright 2008-2012 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
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

#ifndef __FSL_ENET_H
#define __FSL_ENET_H

/**
 @Description  Ethernet MAC-PHY Interface
*/

enum enet_interface {
    enet_if_mii    = 0x00010000, /**< MII interface */
    enet_if_rmii   = 0x00020000, /**< RMII interface */
    enet_if_smii   = 0x00030000, /**< SMII interface */
    enet_if_gmii   = 0x00040000, /**< GMII interface */
    enet_if_rgmii  = 0x00050000, /**< RGMII interface */
    enet_if_tbi    = 0x00060000, /**< TBI interface */
    enet_if_rtbi   = 0x00070000, /**< RTBI interface */
    enet_if_sgmii  = 0x00080000, /**< SGMII interface */
    enet_if_xgmii  = 0x00090000, /**< XGMII interface */
    enet_if_qsgmii = 0x000a0000, /**< QSGMII interface */
    enet_if_xfi    = 0x000b0000  /**< XFI interface */
};

/**
 @Description  Ethernet Speed (nominal data rate)
*/
enum enet_speed {
    enet_speed_10     = 10,      /**< 10 Mbps */
    enet_speed_100    = 100,     /**< 100 Mbps */
    enet_speed_1000   = 1000,    /**< 1000 Mbps = 1 Gbps */
    enet_speed_10000  = 10000    /**< 10000 Mbps = 10 Gbps */
};

enum mac_stat_level {
    /* No statistics */
    mac_stat_none = 0,
    /* Only RMON MIB group 1 (ether stats). Optimized for performance */
    mac_stat_mib_grp1,
    /* Only error counters are available. Optimized for performance */
    mac_stat_partial,
    /* All counters available. Not optimized for performance */
    mac_stat_full
};


#endif /* __ENET_EXT_H */

