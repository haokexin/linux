/*!
 * @file pch_phub_hal.c
 * @brief Provides all the implementation of the interfaces pertaining to the
 *           HAL.
 * @version 1.0.0.0
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
 * Copyright (C) 2010 OKI SEMICONDUCTOR Co., LTD.
 *
 * created:
 * OKI SEMICONDUCTOR 04/14/2010
 * modified:
 *
 */

/*includes*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/delay.h>
#include "pch_common.h"
#include "pch_debug.h"
#include "pch_phub.h"
#include "pch_phub_hal.h"

/* Status Register offset */
#define PHUB_STATUS (0x00)

/* Control Register offset */
#define PHUB_CONTROL (0x04)

/* Time out value for Status Register */
#define PHUB_TIMEOUT (0x05)

/* Enabling for writing ROM */
#define PCH_PHUB_ROM_WRITE_ENABLE (0x01)

/* Disabling for writing ROM */
#define PCH_PHUB_ROM_WRITE_DISABLE (0x00)

/* ROM data area start address offset */
#define PCH_PHUB_ROM_START_ADDR (0x14)

/* MAX number of INT_REDUCE_CONTROL registers */
#define MAX_NUM_INT_REDUCE_CONTROL_REG (128)

/* global variables */
struct pch_phub_reg {
 u32 phub_id_reg; /* PHUB_ID register val */
 u32 q_pri_val_reg; /* QUEUE_PRI_VAL register val */
 u32 rc_q_maxsize_reg; /* RC_QUEUE_MAXSIZE register val */
 u32 bri_q_maxsize_reg; /* BRI_QUEUE_MAXSIZE register val */
 u32 comp_resp_timeout_reg; /* COMP_RESP_TIMEOUT register val */
 u32 bus_slave_control_reg; /* BUS_SLAVE_CONTROL_REG register val */
 u32 deadlock_avoid_type_reg; /* DEADLOCK_AVOID_TYPE register val */
 u32 intpin_reg_wpermit_reg0; /* INTPIN_REG_WPERMIT register 0 val */
 u32 intpin_reg_wpermit_reg1; /* INTPIN_REG_WPERMIT register 1 val */
 u32 intpin_reg_wpermit_reg2; /* INTPIN_REG_WPERMIT register 2 val */
 u32 intpin_reg_wpermit_reg3; /* INTPIN_REG_WPERMIT register 3 val */
 /* INT_REDUCE_CONTROL registers val */
 u32 int_reduce_control_reg[MAX_NUM_INT_REDUCE_CONTROL_REG];
#ifdef PCH_CAN_PCLK_50MHZ
 u32 clkcfg_reg;  /* CLK CFG register val */
#endif
} g_pch_phub_reg;

/*functions implementations*/
/*! @ingroup PHUB_HALLayerAPI
  @fn  void pch_phub_read_reg(unsigned long reg_addr_offset,
	unsigned long *data)
  @remarks  Implements the functionality of reading register.
  @param  reg_addr_offset [@ref IN] Contains the register offset address value
  @param  *data           [@ref INOUT] Contains the register value
  @retval NONE
  @see
  */
void pch_phub_read_reg(unsigned long reg_addr_offset, unsigned long *data)
{
 unsigned long reg_addr = pch_phub_base_address + reg_addr_offset;
 *data = PCH_READ32(reg_addr);

 return;
}

/*! @ingroup PHUB_HALLayerAPI
  @fn  void pch_phub_write_reg(unsigned long reg_addr_offset,
	unsigned long data)
  @remarks  Implements the functionality of writing register.
  @param  reg_addr_offset [@ref IN] Contains the register offset address value
  @param  data            [@ref IN] Contains the writing value
  @retval NONE
  @see
  */
void pch_phub_write_reg(unsigned long reg_addr_offset, unsigned long data)
{
 unsigned long reg_addr = pch_phub_base_address + reg_addr_offset;
 PCH_WRITE32(data, reg_addr);

 return;
}

/*! @ingroup PHUB_HALLayerAPI
  @fn  void pch_phub_read_modify_write_reg(unsigned long reg_addr_offset,
      unsigned long data, unsigned long mask)
  @remarks  Implements the functionality of reading, modifying and writing
	register.
  @param  reg_addr_offset [@ref IN] Contains the register offset address value
  @param  data            [@ref IN] Contains the writing value
  @param  mask            [@ref IN] Contains the mask value
  @retval NONE
  @see
  */
void pch_phub_read_modify_write_reg(unsigned long reg_addr_offset,
	unsigned long data, unsigned long mask)
{
 unsigned long reg_addr = pch_phub_base_address + reg_addr_offset;
 PCH_WRITE32(((PCH_READ32(reg_addr) & ~mask)) | data, reg_addr);

 return;
}

/*! @ingroup PHUB_HALLayerAPI
  @fn int pch_phub_read_gbe_mac_addr(unsigned long offset_address,
	unsigned char *data)
  @param  unsigned long offset_address
 [@ref IN] Contains the Gigabit Ethernet MAC address offset value
  @param  *data [@ref INOUT] Contains the Gigabit Ethernet MAC address value
  @retval return value [@ref OUT] contains the result
    for the reading Gigabit Ethernet MAC address attempt
  @see
  */
int pch_phub_read_gbe_mac_addr(unsigned long offset_address,
	unsigned char *data)
{
 int retval = PCH_PHUB_SUCCESS;

 retval = pch_phub_read_serial_rom_val(offset_address, data);

 return retval;
}
EXPORT_SYMBOL(pch_phub_read_gbe_mac_addr);

/*! @ingroup PHUB_HALLayerAPI
  @fn int pch_phub_write_gbe_mac_addr(unsigned long offset_address,
	unsigned char data)
  @param  unsigned long offset_address
 [@ref IN] Contains the Gigabit Ethernet MAC address offset value
  @param  data [@ref IN] Contains the Gigabit Ethernet MAC address value
  @retval return value [@ref OUT] contains the result for the
     writing Gigabit Ethernet MAC address attempt
  @see
  */
int pch_phub_write_gbe_mac_addr(unsigned long offset_address,
	unsigned char data)
{
 int retval = PCH_PHUB_SUCCESS;

 retval = pch_phub_gbe_serial_rom_conf();
 retval |= pch_phub_write_serial_rom_val(offset_address, data);

 return retval;
}
EXPORT_SYMBOL(pch_phub_write_gbe_mac_addr);

/*! @ingroup PHUB_HALLayerAPI
  @fn  void pch_phub_save_reg_conf(void)
  @remarks  saves register configuration
  @param NONE
  @retval  NONE
  @see
  pch_phub_suspend
  */
void pch_phub_save_reg_conf(void)
{
 u32 base_addr = pch_phub_base_address;
 u32 i = 0;

 PCH_DEBUG("pch_phub_save_reg_conf ENTRY\n");
 /* to store contents of PHUB_ID register */
 g_pch_phub_reg.phub_id_reg =
     PCH_READ32(base_addr + PCH_PHUB_PHUB_ID_REG);
 /* to store contents of QUEUE_PRI_VAL register */
 g_pch_phub_reg.q_pri_val_reg =
     PCH_READ32(base_addr + PCH_PHUB_QUEUE_PRI_VAL_REG);
 /* to store contents of RC_QUEUE_MAXSIZE register */
 g_pch_phub_reg.rc_q_maxsize_reg =
     PCH_READ32(base_addr + PCH_PHUB_RC_QUEUE_MAXSIZE_REG);
 /* to store contents of BRI_QUEUE_MAXSIZE register */
 g_pch_phub_reg.bri_q_maxsize_reg =
     PCH_READ32(base_addr + PCH_PHUB_BRI_QUEUE_MAXSIZE_REG);
 /* to store contents of COMP_RESP_TIMEOUT register */
 g_pch_phub_reg.comp_resp_timeout_reg =
     PCH_READ32(base_addr + PCH_PHUB_COMP_RESP_TIMEOUT_REG);
 /* to store contents of BUS_SLAVE_CONTROL_REG register */
 g_pch_phub_reg.bus_slave_control_reg =
     PCH_READ32(base_addr + PCH_PHUB_BUS_SLAVE_CONTROL_REG);
 /* to store contents of DEADLOCK_AVOID_TYPE register */
 g_pch_phub_reg.deadlock_avoid_type_reg =
     PCH_READ32(base_addr + PCH_PHUB_DEADLOCK_AVOID_TYPE_REG);
 /* to store contents of INTPIN_REG_WPERMIT register 0 */
 g_pch_phub_reg.intpin_reg_wpermit_reg0 =
     PCH_READ32(base_addr + PCH_PHUB_INTPIN_REG_WPERMIT_REG0);
 /* to store contents of INTPIN_REG_WPERMIT register 1 */
 g_pch_phub_reg.intpin_reg_wpermit_reg1 =
     PCH_READ32(base_addr + PCH_PHUB_INTPIN_REG_WPERMIT_REG1);
 /* to store contents of INTPIN_REG_WPERMIT register 2 */
 g_pch_phub_reg.intpin_reg_wpermit_reg2 =
     PCH_READ32(base_addr + PCH_PHUB_INTPIN_REG_WPERMIT_REG2);
 /* to store contents of INTPIN_REG_WPERMIT register 3 */
 g_pch_phub_reg.intpin_reg_wpermit_reg3 =
     PCH_READ32(base_addr + PCH_PHUB_INTPIN_REG_WPERMIT_REG3);
 PCH_DEBUG("pch_phub_save_reg_conf : "
  "g_pch_phub_reg.phub_id_reg=%x, "
  "g_pch_phub_reg.q_pri_val_reg=%x, "
  "g_pch_phub_reg.rc_q_maxsize_reg=%x, "
  "g_pch_phub_reg.bri_q_maxsize_reg=%x, "
  "g_pch_phub_reg.comp_resp_timeout_reg=%x, "
  "g_pch_phub_reg.bus_slave_control_reg=%x, "
  "g_pch_phub_reg.deadlock_avoid_type_reg=%x, "
  "g_pch_phub_reg.intpin_reg_wpermit_reg0=%x, "
  "g_pch_phub_reg.intpin_reg_wpermit_reg1=%x, "
  "g_pch_phub_reg.intpin_reg_wpermit_reg2=%x, "
  "g_pch_phub_reg.intpin_reg_wpermit_reg3=%x\n",
  g_pch_phub_reg.phub_id_reg,
  g_pch_phub_reg.q_pri_val_reg,
  g_pch_phub_reg.rc_q_maxsize_reg,
  g_pch_phub_reg.bri_q_maxsize_reg,
  g_pch_phub_reg.comp_resp_timeout_reg,
  g_pch_phub_reg.bus_slave_control_reg,
  g_pch_phub_reg.deadlock_avoid_type_reg,
  g_pch_phub_reg.intpin_reg_wpermit_reg0,
  g_pch_phub_reg.intpin_reg_wpermit_reg1,
  g_pch_phub_reg.intpin_reg_wpermit_reg2,
  g_pch_phub_reg.intpin_reg_wpermit_reg3);
 /* to store contents of INT_REDUCE_CONTROL registers */
 for (i = 0; i < MAX_NUM_INT_REDUCE_CONTROL_REG; i++) {
  g_pch_phub_reg.int_reduce_control_reg[i] =
	PCH_READ32(base_addr +
	PCH_PHUB_INT_REDUCE_CONTROL_REG_BASE + 4 * i);
  PCH_DEBUG("pch_phub_save_reg_conf : "
   "g_pch_phub_reg.int_reduce_control_reg[%d]=%x\n",
   i, g_pch_phub_reg.int_reduce_control_reg[i]);
 }
#ifdef PCH_CAN_PCLK_50MHZ
 /* save clk cfg register */
 g_pch_phub_reg.clkcfg_reg =
     PCH_READ32(base_addr + CLKCFG_REG_OFFSET);
#endif
 return;
}

/*! @ingroup PHUB_HALLayerAPI
  @fn  void pch_phub_restore_reg_conf(void)
  @remarks  restore register configuration
  @param NONE
  @retval  NONE
  @see
  pch_phub_resume
  */
void pch_phub_restore_reg_conf(void)
{
 u32 base_addr = pch_phub_base_address;
 u32 i = 0;

 PCH_DEBUG("pch_phub_restore_reg_conf ENTRY\n");
 /* to store contents of PHUB_ID register */
 PCH_WRITE32(g_pch_phub_reg.phub_id_reg,
      base_addr + PCH_PHUB_PHUB_ID_REG);
 /* to store contents of QUEUE_PRI_VAL register */
 PCH_WRITE32(g_pch_phub_reg.q_pri_val_reg,
      base_addr + PCH_PHUB_QUEUE_PRI_VAL_REG);
 /* to store contents of RC_QUEUE_MAXSIZE register */
 PCH_WRITE32(g_pch_phub_reg.rc_q_maxsize_reg,
      base_addr + PCH_PHUB_RC_QUEUE_MAXSIZE_REG);
 /* to store contents of BRI_QUEUE_MAXSIZE register */
 PCH_WRITE32(g_pch_phub_reg.bri_q_maxsize_reg,
      base_addr + PCH_PHUB_BRI_QUEUE_MAXSIZE_REG);
 /* to store contents of COMP_RESP_TIMEOUT register */
 PCH_WRITE32(g_pch_phub_reg.comp_resp_timeout_reg,
      base_addr + PCH_PHUB_COMP_RESP_TIMEOUT_REG);
 /* to store contents of BUS_SLAVE_CONTROL_REG register */
 PCH_WRITE32(g_pch_phub_reg.bus_slave_control_reg,
      base_addr + PCH_PHUB_BUS_SLAVE_CONTROL_REG);
 /* to store contents of DEADLOCK_AVOID_TYPE register */
 PCH_WRITE32(g_pch_phub_reg.deadlock_avoid_type_reg,
      base_addr + PCH_PHUB_DEADLOCK_AVOID_TYPE_REG);
 /* to store contents of INTPIN_REG_WPERMIT register 0 */
 PCH_WRITE32(g_pch_phub_reg.intpin_reg_wpermit_reg0,
      base_addr + PCH_PHUB_INTPIN_REG_WPERMIT_REG0);
 /* to store contents of INTPIN_REG_WPERMIT register 1 */
 PCH_WRITE32(g_pch_phub_reg.intpin_reg_wpermit_reg1,
      base_addr + PCH_PHUB_INTPIN_REG_WPERMIT_REG1);
 /* to store contents of INTPIN_REG_WPERMIT register 2 */
 PCH_WRITE32(g_pch_phub_reg.intpin_reg_wpermit_reg2,
      base_addr + PCH_PHUB_INTPIN_REG_WPERMIT_REG2);
 /* to store contents of INTPIN_REG_WPERMIT register 3 */
 PCH_WRITE32(g_pch_phub_reg.intpin_reg_wpermit_reg3,
      base_addr + PCH_PHUB_INTPIN_REG_WPERMIT_REG3);
 PCH_DEBUG("pch_phub_save_reg_conf : "
  "g_pch_phub_reg.phub_id_reg=%x, "
  "g_pch_phub_reg.q_pri_val_reg=%x, "
  "g_pch_phub_reg.rc_q_maxsize_reg=%x, "
  "g_pch_phub_reg.bri_q_maxsize_reg=%x, "
  "g_pch_phub_reg.comp_resp_timeout_reg=%x, "
  "g_pch_phub_reg.bus_slave_control_reg=%x, "
  "g_pch_phub_reg.deadlock_avoid_type_reg=%x, "
  "g_pch_phub_reg.intpin_reg_wpermit_reg0=%x, "
  "g_pch_phub_reg.intpin_reg_wpermit_reg1=%x, "
  "g_pch_phub_reg.intpin_reg_wpermit_reg2=%x, "
  "g_pch_phub_reg.intpin_reg_wpermit_reg3=%x\n",
  g_pch_phub_reg.phub_id_reg,
  g_pch_phub_reg.q_pri_val_reg,
  g_pch_phub_reg.rc_q_maxsize_reg,
  g_pch_phub_reg.bri_q_maxsize_reg,
  g_pch_phub_reg.comp_resp_timeout_reg,
  g_pch_phub_reg.bus_slave_control_reg,
  g_pch_phub_reg.deadlock_avoid_type_reg,
  g_pch_phub_reg.intpin_reg_wpermit_reg0,
  g_pch_phub_reg.intpin_reg_wpermit_reg1,
  g_pch_phub_reg.intpin_reg_wpermit_reg2,
  g_pch_phub_reg.intpin_reg_wpermit_reg3);
 /* to store contents of INT_REDUCE_CONTROL register */
 for (i = 0; i < MAX_NUM_INT_REDUCE_CONTROL_REG; i++) {
  PCH_WRITE32(g_pch_phub_reg.int_reduce_control_reg[i],
	base_addr +
	PCH_PHUB_INT_REDUCE_CONTROL_REG_BASE + 4 * i);
  PCH_DEBUG("pch_phub_save_reg_conf : "
   "g_pch_phub_reg.int_reduce_control_reg[%d]=%x\n",
   i, g_pch_phub_reg.int_reduce_control_reg[i]);
 }

#ifdef PCH_CAN_PCLK_50MHZ
 /*restore the clock config reg */
 PCH_WRITE32(g_pch_phub_reg.clkcfg_reg,
      base_addr + CLKCFG_REG_OFFSET);
#endif

 return;
}

/*! @ingroup PHUB_HALLayerAPI
  @fn  void pch_phub_read_serial_rom
  (unsigned long offset_address, unsigned char *data)
  @remarks  Implements the functionality of reading Serial ROM.
  @param  unsigned long offset_address
  [@ref IN] Contains the Serial ROM address offset value
  @param  *data [@ref INOUT] Contains the Serial ROM value
  @retval returnvalue
 [@ref OUT] contains the result for the reading Serial ROM attempt
  @see
  */
int pch_phub_read_serial_rom(unsigned long offset_address,
    unsigned char *data)
{
 unsigned long mem_addr =
     pch_phub_extrom_base_address + offset_address;

 PCH_DEBUG("pch_phub_read_serial_rom:mem_addr=0x%08x\n", mem_addr);
 *data = PCH_READ8(mem_addr);

 return PCH_PHUB_SUCCESS;
}

/*! @ingroup PHUB_HALLayerAPI
  @fn  void pch_phub_write_serial_rom(unsigned long offset_address,
	unsigned char data)
  @remarks  Implements the functionality of writing Serial ROM.
  @param  unsigned long offset_address
 [@ref IN] Contains the Serial ROM address offset value
  @param  data  [@ref IN] Contains the Serial ROM value
  @retval returnvalue
 [@ref OUT] contains the result for the writing Serial ROM attempt
  @see
  */
int pch_phub_write_serial_rom(unsigned long offset_address,
     unsigned char data)
{
 int retval = PCH_PHUB_SUCCESS;
 unsigned long mem_addr =
     pch_phub_extrom_base_address + offset_address;
 int i = 0;
 unsigned long word_data = 0;

 PCH_DEBUG("pch_phub_write_serial_rom:mem_addr=0x%08x\n", mem_addr);
 PCH_WRITE32(PCH_PHUB_ROM_WRITE_ENABLE,
      pch_phub_extrom_base_address + PHUB_CONTROL);

 word_data = PCH_READ32((mem_addr & 0xFFFFFFFC));
 PCH_DEBUG("word_data=0x%08x\n", word_data);
 PCH_DEBUG("data=0x%02x\n", data);
 switch (mem_addr % 4) {
 case 0:
  {
   word_data &= 0xFFFFFF00;
   PCH_WRITE32((word_data | (unsigned long)data),
	(mem_addr & 0xFFFFFFFC));
  } break;
 case 1:
  {
   word_data &= 0xFFFF00FF;
   PCH_WRITE32((word_data | ((unsigned long)data << 8)),
	(mem_addr & 0xFFFFFFFC));
  } break;
 case 2:
  {
   word_data &= 0xFF00FFFF;
   PCH_WRITE32((word_data | ((unsigned long)data << 16)),
	(mem_addr & 0xFFFFFFFC));
  } break;
 case 3:
  {
   word_data &= 0x00FFFFFF;
   PCH_WRITE32((word_data | ((unsigned long)data << 24)),
	(mem_addr & 0xFFFFFFFC));
  } break;
 }
 while (0x00 !=
	PCH_READ8(pch_phub_extrom_base_address + PHUB_STATUS)) {
  msleep(1);
  if (PHUB_TIMEOUT == i) {
   retval = PCH_PHUB_FAIL;
   break;
  }
  i++;
 }

 PCH_WRITE32(PCH_PHUB_ROM_WRITE_DISABLE,
      pch_phub_extrom_base_address + PHUB_CONTROL);

 return retval;
}

/*! @ingroup PHUB_HALLayerAPI
  @fn  void pch_phub_read_serial_rom_val(unsigned long offset_address,
	unsigned char *data)
  @remarks  Implements the functionality of reading Serial ROM value.
  @param  unsigned long offset_address
 [@ref IN] Contains the Serial ROM address offset value
  @param  *data [@ref INOUT] Contains the Serial ROM value
  @retval returnvalue
 [@ref OUT] contains the result for the reading Serial ROM attempt
  @see
  */
int pch_phub_read_serial_rom_val(unsigned long offset_address,
	unsigned char *data)
{
 int retval = PCH_PHUB_SUCCESS;
 unsigned long mem_addr;

 mem_addr =
     (offset_address / 4 * 8) + 3 - (offset_address % 4) +
     PCH_PHUB_ROM_START_ADDR;
 retval = pch_phub_read_serial_rom(mem_addr, data);

 return retval;
}

/*! @ingroup PHUB_HALLayerAPI
  @fn  void pch_phub_write_serial_rom_val(unsigned long offset_address,
	unsigned char data)
  @remarks  Implements the functionality of writing Serial ROM value.
  @param  unsigned long offset_address
 [@ref IN] Contains the Serial ROM address offset value
  @param  data [@ref IN] Contains the Serial ROM value
  @retval returnvalue
 [@ref OUT] contains the result for the writing Serial ROM attempt
  @see
  */
int pch_phub_write_serial_rom_val(unsigned long offset_address,
	unsigned char data)
{
 int retval = PCH_PHUB_SUCCESS;
 unsigned long mem_addr;

 mem_addr =
     (offset_address / 4 * 8) + 3 - (offset_address % 4) +
     PCH_PHUB_ROM_START_ADDR;
 retval = pch_phub_write_serial_rom(mem_addr, data);

 return retval;
}

/*! @ingroup PHUB_HALLayerAPI
  @fn int pch_phub_gbe_serial_rom_conf(void)
  @remarks makes Serial ROM header format configuration
  for Gigabit Ethernet MAC address
  @param NONE
  @retval returnvalue
 [@ref OUT] contains the result for the writing Serial ROM attempt
  @see
  */
int pch_phub_gbe_serial_rom_conf(void)
{
 int retval = PCH_PHUB_SUCCESS;

 retval |= pch_phub_write_serial_rom(0x0b, 0xbc);
 retval |= pch_phub_write_serial_rom(0x0a, 0x10);
 retval |= pch_phub_write_serial_rom(0x09, 0x01);
 retval |= pch_phub_write_serial_rom(0x08, 0x02);

 retval |= pch_phub_write_serial_rom(0x0f, 0x00);
 retval |= pch_phub_write_serial_rom(0x0e, 0x00);
 retval |= pch_phub_write_serial_rom(0x0d, 0x00);
 retval |= pch_phub_write_serial_rom(0x0c, 0x80);

 retval |= pch_phub_write_serial_rom(0x13, 0xbc);
 retval |= pch_phub_write_serial_rom(0x12, 0x10);
 retval |= pch_phub_write_serial_rom(0x11, 0x01);
 retval |= pch_phub_write_serial_rom(0x10, 0x18);

 retval |= pch_phub_write_serial_rom(0x1b, 0xbc);
 retval |= pch_phub_write_serial_rom(0x1a, 0x10);
 retval |= pch_phub_write_serial_rom(0x19, 0x01);
 retval |= pch_phub_write_serial_rom(0x18, 0x19);

 retval |= pch_phub_write_serial_rom(0x23, 0xbc);
 retval |= pch_phub_write_serial_rom(0x22, 0x10);
 retval |= pch_phub_write_serial_rom(0x21, 0x01);
 retval |= pch_phub_write_serial_rom(0x20, 0x3a);

 retval |= pch_phub_write_serial_rom(0x27, 0x01);
 retval |= pch_phub_write_serial_rom(0x26, 0x00);
 retval |= pch_phub_write_serial_rom(0x25, 0x00);
 retval |= pch_phub_write_serial_rom(0x24, 0x00);

 return retval;
}

