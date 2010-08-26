#ifndef __PCH_PHUB_H__
#define __PCH_PHUB_H__
/*!
 * @file pch_phub.h
 * @brief Provides all the interfaces pertaining to the Packet Hub module.
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

/*! @defgroup PHUB */
/*! @defgroup PHUB_Global      Global
  @ingroup PHUB */
/*! @defgroup PHUB_GlobalGeneral   General
  @ingroup PHUB_Global */
/*! @defgroup PHUB_GlobalResultCodes   StatusCodes
  @ingroup PHUB_Global */
/*! @defgroup PHUB_InterfaceLayer   InterfaceLayer
  @ingroup PHUB */
/*! @defgroup PHUB_InterfaceLayerAPI    Providers
  @ingroup PHUB_InterfaceLayer
  */
/*! @defgroup PHUB_InterfaceLayerNotifyRoutines   Notifiers
  @ingroup PHUB_InterfaceLayer
  */
/*! @defgroup PHUB_PCILayer    PCILayer
  @ingroup PHUB */
/*! @defgroup PHUB_PCILayerAPI     Providers
  @ingroup PHUB_PCILayer
  */
/*! @defgroup PHUB_PCILayerFacilitators    Facilitators
  @ingroup PHUB_PCILayer
  */
/*! @defgroup PHUB_HALLayer    HALLayer
  @ingroup PHUB */
/*! @defgroup PHUB_HALLayerAPI     Providers
  @ingroup PHUB_HALLayer
  */
/*! @defgroup PHUB_HALLayerFacilitators    Facilitators
  @ingroup PHUB_HALLayer
  */
/*! @defgroup PHUB_Utilities    Utilities
  @ingroup PHUB */
/*! @defgroup PHUB_UtilitiesAPI     Providers
  @ingroup PHUB_Utilities
  */

/*! @ingroup PHUB_InterfaceLayer
  @def PHUB_IOCTL_MAGIC
  @brief Outlines the ioctl magic.
  */
#define PHUB_IOCTL_MAGIC   (0xf7)

/*! @ingroup PHUB_InterfaceLayer
  @def IOCTL_PHUB_READ_REG
  @brief Outlines the read register function signature.
  */
#define IOCTL_PHUB_READ_REG (_IOW(PHUB_IOCTL_MAGIC, 1, unsigned long))

/*! @ingroup PHUB_InterfaceLayer
  @def IOCTL_PHUB_WRITE_REG
  @brief Outlines the write register function signature.
  */
#define IOCTL_PHUB_WRITE_REG (_IOW(PHUB_IOCTL_MAGIC, 2, unsigned long))

/*! @ingroup PHUB_InterfaceLayer
  @def IOCTL_PHUB_READ_MODIFY_WRITE_REG
  @brief Outlines the read, modify and write register function signature.
  */
#define IOCTL_PHUB_READ_MODIFY_WRITE_REG (_IOW(PHUB_IOCTL_MAGIC, 3,\
	unsigned long))

/*! @ingroup PHUB_InterfaceLayer
  @def IOCTL_PHUB_READ_OROM
  @brief Outlines the read option rom function signature.
  */
#define IOCTL_PHUB_READ_OROM (_IOW(PHUB_IOCTL_MAGIC, 4, unsigned long))

/*! @ingroup PHUB_InterfaceLayer
  @def IOCTL_PHUB_WRITE_OROM
  @brief Outlines the write option rom function signature.
  */
#define IOCTL_PHUB_WRITE_OROM (_IOW(PHUB_IOCTL_MAGIC, 5, unsigned long))

/*! @ingroup PHUB_InterfaceLayer
  @def IOCTL_PHUB_READ_MAC_ADDR
  @brief Outlines the read mac address function signature.
  */
#define IOCTL_PHUB_READ_MAC_ADDR (_IOW(PHUB_IOCTL_MAGIC, 6,\
	unsigned long))

/*! @ingroup PHUB_InterfaceLayer
  @def IOCTL_PHUB_WRITE_MAC_ADDR
  @brief Outlines the write mac address function signature.
  */
#define IOCTL_PHUB_WRITE_MAC_ADDR (_IOW(PHUB_IOCTL_MAGIC, 7,\
	unsigned long))

/*! @ingroup PHUB_InterfaceLayer
  @def PHUB STATUS CODE
  @brief Outlines PHUB SUCCESS STATUS CODE
  */
#define PCH_PHUB_SUCCESS  (0)

/*! @ingroup PHUB_InterfaceLayer
  @def PHUB STATUS CODE
  @brief Outlines PHUB ERROR STATUS CODE
  */
#define PCH_PHUB_FAIL   (-1)

/* Registers address offset */
#define PCH_PHUB_PHUB_ID_REG   (0x0000)
#define PCH_PHUB_QUEUE_PRI_VAL_REG  (0x0004)
#define PCH_PHUB_RC_QUEUE_MAXSIZE_REG (0x0008)
#define PCH_PHUB_BRI_QUEUE_MAXSIZE_REG (0x000C)
#define PCH_PHUB_COMP_RESP_TIMEOUT_REG (0x0010)
#define PCH_PHUB_BUS_SLAVE_CONTROL_REG (0x0014)
#define PCH_PHUB_DEADLOCK_AVOID_TYPE_REG (0x0018)
#define PCH_PHUB_INTPIN_REG_WPERMIT_REG0 (0x0020)
#define PCH_PHUB_INTPIN_REG_WPERMIT_REG1 (0x0024)
#define PCH_PHUB_INTPIN_REG_WPERMIT_REG2 (0x0028)
#define PCH_PHUB_INTPIN_REG_WPERMIT_REG3 (0x002C)
#define PCH_PHUB_INT_REDUCE_CONTROL_REG_BASE (0x0040)
#define CLKCFG_REG_OFFSET             (0x500)

/*structures*/
/*! @ingroup PHUB_InterfaceLayer
  @struct pch_phub_reqt
  @brief It is a structure used for perserving information related to the
  Packet Hub request.
  @note
  The concerned details should be provided during the read register,
   write register and read / modify / write register.
  @see
  pch_phub_ioctl
  */
struct pch_phub_reqt {
 unsigned long addr_offset; /*specifies the register address offset */
 unsigned long data; /*specifies the data */
 unsigned long mask; /*specifies the mask */
};

/* exported function prototypes */
/*! @ingroup PHUB_InterfaceLayerAPI
  @fn nt pch_phub_open( struct inode *inode,struct file *file )
  @brief  Provides the functionality of initialization of the module
  */
int pch_phub_open(struct inode *inode, struct file *file);

/*! @ingroup PHUB_InterfaceLayerAPI
  @fn int pch_phub_release(struct inode *inode,struct file *file)
  @brief  Provides the functionality of releasing the module
  */
int pch_phub_release(struct inode *inode, struct file *file);

/*! @ingroup PHUB_InterfaceLayerAPI
  @fn int pch_phub_ioctl(struct inode * inode,struct file * file,
	unsigned int cmd, unsigned long arg)
  @brief  Provides the functionality of invoking various functionalities of
	the Packet Hub.
  */
int pch_phub_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	unsigned long arg);

/**global variables*/
extern u32 pch_phub_base_address; /* base address */
extern s32 pch_phub_suspended; /* suspend status */

extern s32 pch_phub_opencount;
extern spinlock_t pch_phub_lock;
extern const struct file_operations pch_phub_fops;
#endif
