/*!
 * @file pch_gbe_param.c
 * @brief Linux PCH Gigabit Ethernet Driver parameter check source file
 *
 * @version 1.00
 *
 * @section
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307, USA.
 */

/*
 * History:
 * Copyright (C) 2010 OKI SEMICONDUCTOR CO., LTD.
 *
 * created:
 *   OKI SEMICONDUCTOR 04/13/2010
 * modified:
 *
 */

#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>

#include "pch_debug.h"
#include "pch_gbe_osdep.h"
#include "pch_gbe_defines.h"
#include "pch_gbe_hw.h"
#include "pch_gbe.h"

/* This is the only thing that needs to be changed to adjust the
 * maximum number of ports that the driver can manage.
 */

#define PCH_GBE_MAX_NIC 1

#define OPTION_UNSET   -1
#define OPTION_DISABLED 0
#define OPTION_ENABLED  1

/* All parameters are treated the same, as an integer array of values.
 * This macro just reduces the need to repeat the same declaration code
 * over and over (plus this helps to avoid typo bugs).
 */

#define PCH_GBE_PARAM_INIT { [0 ... PCH_GBE_MAX_NIC] = OPTION_UNSET }
#ifdef PCH_GBE_QAC
#define PCH_GBE_PARAM(X, desc)
#else
#define PCH_GBE_PARAM(X, desc) \
 static int X[PCH_GBE_MAX_NIC+1] = PCH_GBE_PARAM_INIT; \
 static int num_##X; \
 module_param_array_named(X, X, int, &num_##X, 0); \
 MODULE_PARM_DESC(X, desc);
#endif

/*
 * Transmit Descriptor Count
 *    Valid Range:   PCH_GBE_MIN_TXD - PCH_GBE_MAX_TXD
 *    Default Value: PCH_GBE_DEFAULT_TXD
 */
PCH_GBE_PARAM(TxDescriptors, "Number of transmit descriptors");

/*
 * Receive Descriptor Count
 *    Valid Range:   PCH_GBE_MIN_RXD - PCH_GBE_MAX_RXD
 *    Default Value: PCH_GBE_DEFAULT_RXD
 */
PCH_GBE_PARAM(RxDescriptors, "Number of receive descriptors");

/* User Specified Speed Override
 *
 * Valid Range: 0, 10, 100, 1000
 *  - 0    - auto-negotiate at all supported speeds
 *  - 10   - only link at 10 Mbps
 *  - 100  - only link at 100 Mbps
 *  - 1000 - only link at 1000 Mbps
 *
 * Default Value: 0
 */
PCH_GBE_PARAM(Speed, "Speed setting");

/* User Specified Duplex Override
 *
 * Valid Range: 0-2
 *  - 0 - auto-negotiate for duplex
 *  - 1 - only link at half duplex
 *  - 2 - only link at full duplex
 *
 * Default Value: 0
 */
PCH_GBE_PARAM(Duplex, "Duplex setting");

/*
 * Auto-negotiation Advertisement Override
 *    Valid Range: 0x01-0x0F, 0x20-0x2F
 *
 *       The AutoNeg value is a bit mask describing which speed and duplex
 *       combinations should be advertised during auto-negotiation.
 *       The supported speed and duplex modes are listed below
 *
 *       Bit           7     6     5      4      3     2     1      0
 *       Speed (Mbps)  N/A   N/A   1000   N/A    100   100   10     10
 *       Duplex                    Full          Full  Half  Full   Half
 *
 *    Default Value: 0x2F (copper)
 */
PCH_GBE_PARAM(AutoNeg, "Advertised auto-negotiation setting");
#define AUTONEG_ADV_DEFAULT  0x2F

/*
 * User Specified Flow Control Override
 *    Valid Range: 0-3
 *     - 0 - No Flow Control
 *     - 1 - Rx only, respond to PAUSE frames but do not generate them
 *     - 2 - Tx only, generate PAUSE frames but ignore them on receive
 *     - 3 - Full Flow Control Support
 *    Default Value: Read flow control settings from the EEPROM
 */
PCH_GBE_PARAM(FlowControl, "Flow Control setting");

/*
 * XsumRX - Receive Checksum Offload Enable/Disable
 *    Valid Range: 0, 1
 *     - 0 - disables all checksum offload
 *     - 1 - enables receive IP/TCP/UDP checksum offload
 *    Default Value: PCH_GBE_DEFAULT_RX_CSUM
 */
PCH_GBE_PARAM(XsumRX, "Disable or enable Receive Checksum offload");

/*
 * XsumTX - Transmit Checksum Offload Enable/Disable
 *    Valid Range: 0, 1
 *     - 0 - disables all checksum offload
 *     - 1 - enables transmit IP/TCP/UDP checksum offload
 *    Default Value: PCH_GBE_DEFAULT_TX_CSUM
 */
PCH_GBE_PARAM(XsumTX, "Disable or enable Transmit Checksum offload");

struct pch_gbe_option {
 enum { enable_option, range_option, list_option } type;
 signed char *name;
 signed char *err;
 int  def;
 union {
  struct { /* range_option info */
   int min;
   int max;
  } r;
  struct { /* list_option info */
   int nr;
   struct pch_gbe_opt_list { int i; signed char *str; } *p;
  } l;
 } arg;
};

/* ----------------------------------------------------------------------------
 Function prototype
---------------------------------------------------------------------------- */
static void pch_gbe_check_copper_options(struct pch_gbe_adapter *adapter);
static int pch_gbe_validate_option(int *value,
     struct pch_gbe_option *opt,
     struct pch_gbe_adapter *adapter);

/* ----------------------------------------------------------------------------
 Function
---------------------------------------------------------------------------- */

/*!
 * @ingroup Linux driver internal function
 * @fn      static int pch_gbe_validate_option(int *value,
 *                                             struct pch_gbe_option *opt,
 *                                             struct pch_gbe_adapter *adapter)
 * @brief   Validate option
 * @param   value   [IN] value
 * @param   opt     [IN] option
 * @param   adapter [IN] Board private structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
static int
pch_gbe_validate_option(int *value, struct pch_gbe_option *opt,
  struct pch_gbe_adapter *adapter)
{
 if (*value == OPTION_UNSET) {
  *value = opt->def;
  return 0;
 }

 switch (opt->type) {
 case enable_option:
  switch (*value) {
  case OPTION_ENABLED:
   DPRINTK(PROBE, INFO, "%s Enabled\n", opt->name);
   return 0;
  case OPTION_DISABLED:
   DPRINTK(PROBE, INFO, "%s Disabled\n", opt->name);
   return 0;
  }
  break;
 case range_option:
  if (*value >= opt->arg.r.min && *value <= opt->arg.r.max) {
   DPRINTK(PROBE, INFO,
     "%s set to %i\n", opt->name, *value);
   return 0;
  }
  break;
 case list_option: {
  int i;
  struct pch_gbe_opt_list *ent;

  for (i = 0; i < opt->arg.l.nr; i++) {
   ent = &opt->arg.l.p[i];
   if (*value == ent->i) {
    if (ent->str[0] != '\0')
     DPRINTK(PROBE, INFO, "%s\n", ent->str);
    return 0;
   }
  }
 }
  break;
 default:
  BUG();
 }

 DPRINTK(PROBE, INFO, "Invalid %s value specified (%i) %s\n",
   opt->name, *value, opt->err);
 *value = opt->def;
 return -1;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      void pch_gbe_check_options(struct pch_gbe_adapter *adapter)
 * @brief   Range Checking for Command Line Parameters
 * @param   adapter  [IN] Board private structure
 * @return  None
 * @remarks
 *  This routine checks all command line parameters for valid user
 *  input.  If an invalid value is given, or if no user specified
 *  value exists, a default value is used.  The final value is stored
 *  in a variable in the adapter structure.
 */
void
pch_gbe_check_options(struct pch_gbe_adapter *adapter)
{
 struct pch_gbe_hw *hw = &adapter->hw;
 int bd = adapter->bd_number;

 PCH_DEBUG("pch_gbe_check_options\n");

 if (bd >= PCH_GBE_MAX_NIC) {
  DPRINTK(PROBE, NOTICE,
	"Warning: no configuration for board #%i\n", bd);
  DPRINTK(PROBE, NOTICE, "Using defaults for all values\n");
 }

 { /* Transmit Descriptor Count */
  struct pch_gbe_option opt = {
   .type = range_option,
   .name = "Transmit Descriptors",
   .err  = "using default of "
    __MODULE_STRING(PCH_GBE_DEFAULT_TXD),
   .def  = PCH_GBE_DEFAULT_TXD,
   .arg  = { .r = { .min = PCH_GBE_MIN_TXD } },
   .arg  = { .r = { .max = PCH_GBE_MAX_TXD } }
  };
  struct pch_gbe_tx_ring *tx_ring = adapter->tx_ring;
  if (num_TxDescriptors > bd) {
   tx_ring->count = TxDescriptors[bd];
   pch_gbe_validate_option(&tx_ring->count, &opt, adapter);
   PCH_GBE_ROUNDUP(tx_ring->count,
     PCH_GBE_TX_DESC_MULTIPLE);
  } else {
   tx_ring->count = opt.def;
  }
 }
 { /* Receive Descriptor Count */
  struct pch_gbe_option opt = {
   .type = range_option,
   .name = "Receive Descriptors",
   .err  = "using default of "
    __MODULE_STRING(PCH_GBE_DEFAULT_RXD),
   .def  = PCH_GBE_DEFAULT_RXD,
   .arg  = { .r = { .min = PCH_GBE_MIN_RXD } },
   .arg  = { .r = { .max = PCH_GBE_MAX_RXD } }
  };
  struct pch_gbe_rx_ring *rx_ring = adapter->rx_ring;
  if (num_RxDescriptors > bd) {
   rx_ring->count = RxDescriptors[bd];
   pch_gbe_validate_option(&rx_ring->count, &opt, adapter);
   PCH_GBE_ROUNDUP(rx_ring->count,
     PCH_GBE_RX_DESC_MULTIPLE);
  } else {
   rx_ring->count = opt.def;
  }
 }
 { /* Checksum Offload Enable/Disable */
  struct pch_gbe_option opt = {
   .type = enable_option,
   .name = "Checksum Offload",
   .err  = "defaulting to Enabled",
   .def  = PCH_GBE_DEFAULT_RX_CSUM
  };

  if (num_XsumRX > bd) {
   adapter->rx_csum = XsumRX[bd];
   pch_gbe_validate_option((int *)(&adapter->rx_csum),
      &opt, adapter);
  } else {
   adapter->rx_csum = opt.def;
  }
 }
 { /* Checksum Offload Enable/Disable */
  struct pch_gbe_option opt = {
   .type = enable_option,
   .name = "Checksum Offload",
   .err  = "defaulting to Enabled",
   .def  = PCH_GBE_DEFAULT_TX_CSUM
  };

  if (num_XsumTX > bd) {
   adapter->tx_csum = XsumTX[bd];
   pch_gbe_validate_option((int *)(&adapter->tx_csum),
      &opt, adapter);
  } else {
   adapter->tx_csum = opt.def;
  }
 }
 { /* Flow Control */

  struct pch_gbe_opt_list fc_list[] = {
   {pch_gbe_fc_none, "Flow Control Disabled"},
   {pch_gbe_fc_rx_pause, "Flow Control Receive Only"},
   {pch_gbe_fc_tx_pause, "Flow Control Transmit Only"},
   {pch_gbe_fc_full, "Flow Control Enabled"} };

  struct pch_gbe_option opt = {
   .type = list_option,
   .name = "Flow Control",
   .err  = "reading default settings from EEPROM",
   .def  = PCH_GBE_FC_DEFAULT,
   .arg  = { .l = { .nr = (int)ARRAY_SIZE(fc_list),
      .p = fc_list } }
  };

  if (num_FlowControl > bd) {
   hw->mac.fc = FlowControl[bd];
   pch_gbe_validate_option((int *)(&hw->mac.fc),
       &opt, adapter);
  } else {
   hw->mac.fc = opt.def;
  }
 }

 pch_gbe_check_copper_options(adapter);
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_check_copper_options(
 *                      struct pch_gbe_adapter *adapter)
 * @brief   Range Checking for Link Options, Copper Version
 * @param   adapter  [IN] Board private structure
 * @return  None
 * @remarks
 *  Handles speed and duplex options on copper adapters
 */
static void
pch_gbe_check_copper_options(struct pch_gbe_adapter *adapter)
{
 struct pch_gbe_hw *hw = &adapter->hw;
 int speed, dplx;
 int bd = adapter->bd_number;

 { /* Speed */
  struct pch_gbe_opt_list speed_list[] = {
    {0, "" },
    {SPEED_10, ""},
    {SPEED_100, ""},
    {SPEED_1000, ""} };

  struct pch_gbe_option opt = {
   .type = list_option,
   .name = "Speed",
   .err  = "parameter ignored",
   .def  = 0,
   .arg  = { .l = { .nr = (int)ARRAY_SIZE(speed_list),
      .p = speed_list } }
  };

  if (num_Speed > bd) {
   speed = Speed[bd];
   pch_gbe_validate_option(&speed, &opt, adapter);
  } else {
   speed = opt.def;
  }
 }
 { /* Duplex */
  struct pch_gbe_opt_list dplx_list[] = {
    {0, ""},
    {PHY_HALF_DUPLEX, ""},
    {PHY_FULL_DUPLEX, ""} };

  struct pch_gbe_option opt = {
   .type = list_option,
   .name = "Duplex",
   .err  = "parameter ignored",
   .def  = 0,
   .arg  = { .l = { .nr = (int)ARRAY_SIZE(dplx_list),
      .p = dplx_list } }
  };

  if (num_Duplex > bd) {
   dplx = Duplex[bd];
   pch_gbe_validate_option(&dplx, &opt, adapter);
  } else {
   dplx = opt.def;
  }
 }

 { /* Autoneg */
  struct pch_gbe_opt_list an_list[] =
   #define AA "AutoNeg advertising "
   {{ 0x01, AA "10/HD" },
    { 0x02, AA "10/FD" },
    { 0x03, AA "10/FD, 10/HD" },
    { 0x04, AA "100/HD" },
    { 0x05, AA "100/HD, 10/HD" },
    { 0x06, AA "100/HD, 10/FD" },
    { 0x07, AA "100/HD, 10/FD, 10/HD" },
    { 0x08, AA "100/FD" },
    { 0x09, AA "100/FD, 10/HD" },
    { 0x0a, AA "100/FD, 10/FD" },
    { 0x0b, AA "100/FD, 10/FD, 10/HD" },
    { 0x0c, AA "100/FD, 100/HD" },
    { 0x0d, AA "100/FD, 100/HD, 10/HD" },
    { 0x0e, AA "100/FD, 100/HD, 10/FD" },
    { 0x0f, AA "100/FD, 100/HD, 10/FD, 10/HD" },
    { 0x20, AA "1000/FD" },
    { 0x21, AA "1000/FD, 10/HD" },
    { 0x22, AA "1000/FD, 10/FD" },
    { 0x23, AA "1000/FD, 10/FD, 10/HD" },
    { 0x24, AA "1000/FD, 100/HD" },
    { 0x25, AA "1000/FD, 100/HD, 10/HD" },
    { 0x26, AA "1000/FD, 100/HD, 10/FD" },
    { 0x27, AA "1000/FD, 100/HD, 10/FD, 10/HD" },
    { 0x28, AA "1000/FD, 100/FD" },
    { 0x29, AA "1000/FD, 100/FD, 10/HD" },
    { 0x2a, AA "1000/FD, 100/FD, 10/FD" },
    { 0x2b, AA "1000/FD, 100/FD, 10/FD, 10/HD" },
    { 0x2c, AA "1000/FD, 100/FD, 100/HD" },
    { 0x2d, AA "1000/FD, 100/FD, 100/HD, 10/HD" },
    { 0x2e, AA "1000/FD, 100/FD, 100/HD, 10/FD" },
    { 0x2f, AA "1000/FD, 100/FD, 100/HD, 10/FD, 10/HD" } };

  struct pch_gbe_option opt = {
   .type = list_option,
   .name = "AutoNeg",
   .err  = "parameter ignored",
   .def  = AUTONEG_ADV_DEFAULT,
   .arg  = { .l = { .nr = (int)ARRAY_SIZE(an_list),
      .p = an_list} }
  };

  if (num_AutoNeg > bd) {
   if (speed != 0 || dplx != 0) {
    DPRINTK(PROBE, INFO,
    "AutoNeg specified along with Speed or Duplex, "
    "parameter ignored\n");
    hw->phy.autoneg_advertised = opt.def;
   } else {
    hw->phy.autoneg_advertised = AutoNeg[bd];
    pch_gbe_validate_option(
     (int *)(&hw->phy.autoneg_advertised),
     &opt, adapter);
   }
  } else {
   hw->phy.autoneg_advertised = opt.def;
  }
 }

 switch (speed + dplx) {
 case 0:
  hw->mac.autoneg = hw->mac.fc_autoneg = 1;
  if ((num_Speed > bd) && (speed != 0 || dplx != 0))
   DPRINTK(PROBE, INFO,
    "Speed and duplex autonegotiation enabled\n");
  hw->mac.link_speed = SPEED_10;
  hw->mac.link_duplex = DUPLEX_HALF;
  break;
 case PHY_HALF_DUPLEX:
  DPRINTK(PROBE, INFO, "Half Duplex specified without Speed\n");
  DPRINTK(PROBE, INFO, "Using Autonegotiation at "
   "Half Duplex only\n");
  hw->mac.autoneg = hw->mac.fc_autoneg = 1;
  hw->phy.autoneg_advertised = PHY_ADVERTISE_10_HALF |
      PHY_ADVERTISE_100_HALF;
  hw->mac.link_speed = SPEED_10;
  hw->mac.link_duplex = DUPLEX_HALF;
  break;
 case PHY_FULL_DUPLEX:
  DPRINTK(PROBE, INFO, "Full Duplex specified without Speed\n");
  DPRINTK(PROBE, INFO, "Using Autonegotiation at "
   "Full Duplex only\n");
  hw->mac.autoneg = hw->mac.fc_autoneg = 1;
  hw->phy.autoneg_advertised = PHY_ADVERTISE_10_FULL |
      PHY_ADVERTISE_100_FULL |
      PHY_ADVERTISE_1000_FULL;
  hw->mac.link_speed = SPEED_10;
  hw->mac.link_duplex = DUPLEX_FULL;
  break;
 case PHY_SPEED_10:
  DPRINTK(PROBE, INFO, "10 Mbps Speed specified "
   "without Duplex\n");
  DPRINTK(PROBE, INFO, "Using Autonegotiation at 10 Mbps only\n");
  hw->mac.autoneg = hw->mac.fc_autoneg = 1;
  hw->phy.autoneg_advertised = PHY_ADVERTISE_10_HALF |
      PHY_ADVERTISE_10_FULL;
  hw->mac.link_speed = SPEED_10;
  hw->mac.link_duplex = DUPLEX_HALF;
  break;
 case PHY_SPEED_10 + PHY_HALF_DUPLEX:
  DPRINTK(PROBE, INFO, "Forcing to 10 Mbps Half Duplex\n");
  hw->mac.autoneg = hw->mac.fc_autoneg = 0;
  hw->phy.autoneg_advertised = 0;
  hw->mac.link_speed = SPEED_10;
  hw->mac.link_duplex = DUPLEX_HALF;
  break;
 case PHY_SPEED_10 + PHY_FULL_DUPLEX:
  DPRINTK(PROBE, INFO, "Forcing to 10 Mbps Full Duplex\n");
  hw->mac.autoneg = hw->mac.fc_autoneg = 0;
  hw->phy.autoneg_advertised = 0;
  hw->mac.link_speed = SPEED_10;
  hw->mac.link_duplex = DUPLEX_FULL;
  break;
 case PHY_SPEED_100:
  DPRINTK(PROBE, INFO, "100 Mbps Speed specified "
   "without Duplex\n");
  DPRINTK(PROBE, INFO, "Using Autonegotiation at "
   "100 Mbps only\n");
  hw->mac.autoneg = hw->mac.fc_autoneg = 1;
  hw->phy.autoneg_advertised = PHY_ADVERTISE_100_HALF |
      PHY_ADVERTISE_100_FULL;
  hw->mac.link_speed = SPEED_100;
  hw->mac.link_duplex = DUPLEX_HALF;
  break;
 case PHY_SPEED_100 + PHY_HALF_DUPLEX:
  DPRINTK(PROBE, INFO, "Forcing to 100 Mbps Half Duplex\n");
  hw->mac.autoneg = hw->mac.fc_autoneg = 0;
  hw->phy.autoneg_advertised = 0;
  hw->mac.link_speed = SPEED_100;
  hw->mac.link_duplex = DUPLEX_HALF;
  break;
 case PHY_SPEED_100 + PHY_FULL_DUPLEX:
  DPRINTK(PROBE, INFO, "Forcing to 100 Mbps Full Duplex\n");
  hw->mac.autoneg = hw->mac.fc_autoneg = 0;
  hw->phy.autoneg_advertised = 0;
  hw->mac.link_speed = SPEED_100;
  hw->mac.link_duplex = DUPLEX_FULL;
  break;
 case PHY_SPEED_1000:
  DPRINTK(PROBE, INFO, "1000 Mbps Speed specified without "
   "Duplex\n");
  goto full_duplex_only;
 case PHY_SPEED_1000 + PHY_HALF_DUPLEX:
  DPRINTK(PROBE, INFO,
   "Half Duplex is not supported at 1000 Mbps\n");
  /* fall through */
 case PHY_SPEED_1000 + PHY_FULL_DUPLEX:
full_duplex_only:
  DPRINTK(PROBE, INFO,
	"Using Autonegotiation at 1000 Mbps Full Duplex only\n");
  hw->mac.autoneg = hw->mac.fc_autoneg = 1;
  hw->phy.autoneg_advertised = PHY_ADVERTISE_1000_FULL;
  hw->mac.link_speed = SPEED_1000;
  hw->mac.link_duplex = DUPLEX_FULL;
  break;
 default:
  BUG();
 }
}

