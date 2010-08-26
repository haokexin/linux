/*!
 * @file pch_phub.c
 * @brief Provides all the implementation of the interfaces pertaining to
 *  the Packet Hub module.
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

/* includes */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/string.h>

#include "pch_common.h"
#include "pch_debug.h"
#include "pch_phub.h"
#include "pch_phub_hal.h"

#define MODULE_NAME "pch_phub"

/* global variables */
s32 pch_phub_opencount; /* check whether opened or not */

DEFINE_SPINLOCK(pch_phub_lock); /* for spin lock */

/**
 * file_operations structure initialization
 */
const struct file_operations pch_phub_fops = {
 .owner = THIS_MODULE,
 .open = pch_phub_open,
 .release = pch_phub_release,
 .ioctl = pch_phub_ioctl,
};

/*function implementations*/

/*! @ingroup PHUB_InterfaceLayerAPI
  @fn  int pch_phub_open( struct inode *inode,struct file *file)
  @remarks  Implements the Initializing and opening of the Packet Hub module.
  @param  inode  [@ref INOUT] Contains the reference of the inode structure
  @param  file  [@ref INOUT] Contains the reference of the file structure
  @retval returnvalue [@ref OUT] contains the result for the concerned attempt.
  The result would generally comprise of success code or failure code.
  The failure code will indicate reason for failure.
  @see
  EBUSY
  */
int pch_phub_open(struct inode *inode, struct file *file)
{
 int ret;

 spin_lock(&pch_phub_lock);
 PCH_DEBUG("pch_phub_open : open count value = %d",
	pch_phub_opencount);
 if (pch_phub_opencount) {
	PCH_LOG(KERN_ERR, "pch_phub_open :  device already opened\n");
  ret = -EBUSY;
 } else {
  pch_phub_opencount++;
  ret = PCH_PHUB_SUCCESS;
 }
 spin_unlock(&pch_phub_lock);

 PCH_DEBUG("pch_phub_open returns=%d\n", ret);
 return ret;
}

/*! @ingroup PHUB_InterfaceLayerAPI
  @fn  int pch_phub_release(struct inode *inode,struct file *file)
  @remarks  Implements the release functionality of the Packet Hub module.
  @param  inode  [@ref INOUT] Contains the reference of the inode structure
  @param  file   [@ref INOUT] Contains the reference of the file structure
  @retval returnvalue  [@ref OUT] contains the result for the concerned attempt.
  The result would generally comprise of success code
  or failure code. The failure code will indicate reason for
  failure.
  @see
  SUCCESS
  */
int pch_phub_release(struct inode *inode, struct file *file)
{
 spin_lock(&pch_phub_lock);

 if (pch_phub_opencount > 0)
	pch_phub_opencount--;
 spin_unlock(&pch_phub_lock);

 PCH_DEBUG("pch_phub_release : pch_phub_opencount =%d\n",
   pch_phub_opencount);

 PCH_DEBUG("pch_phub_release returning=%d\n", PCH_PHUB_SUCCESS);
 return PCH_PHUB_SUCCESS;
}

/*! @ingroup PHUB_InterfaceLayerAPI
  @fn  int pch_phub_ioctl(struct inode * inode,struct file * file,
      unsigned int cmd,unsigned long arg)
  @remarks  Implements the various ioctl functionalities of
  the Packet Hub module.
  @param  inode [@ref INOUT] Contains the reference of the inode structure
  @param  file  [@ref INOUT] Contains the reference of the file structure
  @param  cmd  [@ref IN] Contains the command value
  @param  arg  [@ref IN] Contains the command argument value
  @retval returnvalue  [@ref OUT] contains the result for the concerned attempt.
  The result would generally comprise of success code
  or failure code. The failure code will indicate reason for
  failure.
  @see
  EINVAL
  EFAULT
  */
int pch_phub_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{

 int ret_value = PCH_PHUB_SUCCESS;
 struct pch_phub_reqt *p_pch_phub_reqt;
 unsigned long addr_offset;
 unsigned long data;
 unsigned long mask;

 do {
	if (pch_phub_suspended == true) {
	PCH_LOG(KERN_ERR, "pch_phub_ioctl : "
	"suspend initiated returning =%d\n",
	PCH_PHUB_FAIL);
   ret_value = PCH_PHUB_FAIL;
   break;
  }

  p_pch_phub_reqt = (struct pch_phub_reqt *)arg;
  ret_value =
	copy_from_user((void *)&addr_offset,
	(void *)&p_pch_phub_reqt->addr_offset,
	sizeof(addr_offset));
  if (ret_value) {
   PCH_LOG(KERN_ERR, "pch_phub_ioctl : "
	"copy_from_user fail returning =%d\n",
	-EFAULT);
   ret_value = -EFAULT;
   break;
  }
  PCH_DEBUG("pch_phub_ioctl  : copy_from_user returns =%d\n",
    ret_value);

  switch (cmd) {
  case IOCTL_PHUB_READ_REG:
   {

    pch_phub_read_reg(addr_offset, &data);
    PCH_DEBUG("pch_phub_ioctl  : Invoked "
     "pch_phub_read_reg successfully\n");

    ret_value =
	copy_to_user((void *)&p_pch_phub_reqt->data,
	 (void *)&data, sizeof(data));
    if (ret_value) {
	PCH_LOG(KERN_ERR, "pch_phub_ioctl : "
	"copy_to_user fail returning =%d\n",
	-EFAULT);
     ret_value = -EFAULT;
     break;
    }
    break;
   }

  case IOCTL_PHUB_WRITE_REG:
   {

    ret_value =
	copy_from_user((void *)&data,
	(void *)&p_pch_phub_reqt->data, sizeof(data));
    if (ret_value) {
	PCH_LOG(KERN_ERR, "pch_phub_ioctl : "
	"copy_from_user fail returning =%d\n", -EFAULT);
     ret_value = -EFAULT;
     break;
    }
    pch_phub_write_reg(addr_offset, data);
    PCH_DEBUG("pch_phub_ioctl  : Invoked "
	"pch_phub_write_reg successfully\n");
    break;
   }

  case IOCTL_PHUB_READ_MODIFY_WRITE_REG:
   {

    ret_value =
	copy_from_user((void *)&data,
	(void *)&p_pch_phub_reqt->data, sizeof(data));
    if (ret_value) {
     PCH_LOG(KERN_ERR, "pch_phub_ioctl : "
	"copy_from_user fail "
	"returning =%d\n", -EFAULT);
     ret_value = -EFAULT;
     break;
    }
    ret_value =
	copy_from_user((void *)&mask,
	(void *)&p_pch_phub_reqt->mask, sizeof(mask));
    if (ret_value) {
     PCH_LOG(KERN_ERR, "pch_phub_ioctl : "
      "copy_from_user fail "
      "returning =%d\n", -EFAULT);
     ret_value = -EFAULT;
     break;
    }
    pch_phub_read_modify_write_reg(addr_offset,
	data, mask);
    PCH_DEBUG("pch_phub_ioctl  : Invoked "
     "pch_phub_read_modify_write_reg "
     "successfully\n");
    break;
   }

  case IOCTL_PHUB_READ_OROM:
   {

    ret_value =
	pch_phub_read_serial_rom(addr_offset,
       (unsigned char *)&data);
    if (ret_value) {
     PCH_LOG(KERN_ERR,
      "pch_phub_ioctl : Invoked "
      "pch_phub_read_serial_rom "
      "=%d\n", -EFAULT);
     ret_value = -EFAULT;
     break;
    } else {
     PCH_DEBUG("pch_phub_ioctl : Invoked "
      "pch_phub_read_serial_rom "
      "successfully\n");
    }

    ret_value =
	copy_to_user((void *)&p_pch_phub_reqt->data,
		(void *)&data, sizeof(data));
    if (ret_value) {
     PCH_LOG(KERN_ERR, "pch_phub_ioctl : "
	"copy_to_user fail returning "
	"=%d\n", -EFAULT);
     ret_value = -EFAULT;
     break;
    }
    break;
   }

  case IOCTL_PHUB_WRITE_OROM:
   {

    ret_value =
	copy_from_user((void *)&data,
	(void *)&p_pch_phub_reqt->data, sizeof(data));
    if (ret_value) {
     PCH_LOG(KERN_ERR, "pch_phub_ioctl : "
	"copy_from_user fail returning "
	"=%d\n", -EFAULT);
     ret_value = -EFAULT;
     break;
    }
    ret_value =
	pch_phub_write_serial_rom(addr_offset, data);
    if (ret_value) {
     PCH_LOG(KERN_ERR,
	"pch_phub_ioctl : Invoked "
	"pch_phub_write_serial_rom "
	"=%d\n", -EFAULT);
     ret_value = -EFAULT;
     break;
    } else {
     PCH_DEBUG("pch_phub_ioctl : Invoked "
	"pch_phub_write_serial_rom "
	"successfully\n");
    }
    break;
   }

  case IOCTL_PHUB_READ_MAC_ADDR:
   {

    pch_phub_read_gbe_mac_addr(addr_offset,
	(unsigned char *)&data);
    PCH_DEBUG("pch_phub_ioctl : Invoked "
	"pch_phub_read_gbe_mac_addr "
	"successfully\n");

    ret_value =
	copy_to_user((void *)&p_pch_phub_reqt->data,
	(void *)&data, sizeof(data));
    if (ret_value) {
     PCH_LOG(KERN_ERR, "pch_phub_ioctl : "
	"copy_to_user fail "
	"returning =%d\n", -EFAULT);
     ret_value = -EFAULT;
     break;
    }
    break;
   }

  case IOCTL_PHUB_WRITE_MAC_ADDR:
   {

    ret_value =
	copy_from_user((void *)&data,
	(void *)&p_pch_phub_reqt->data, sizeof(data));
    if (ret_value) {
     PCH_LOG(KERN_ERR, "pch_phub_ioctl : "
	"copy_from_user fail "
	"returning =%d\n", -EFAULT);
     ret_value = -EFAULT;
     break;
    }
    pch_phub_write_gbe_mac_addr(addr_offset, data);
    PCH_DEBUG("pch_phub_ioctl : Invoked "
     "pch_phub_write_gbe_mac_addr "
     "successfully\n");
    break;
   }

  default:
   {
    PCH_LOG(KERN_ERR, "pch_write_ioctl invalid "
     "command returning=%d\n", -EINVAL);
    ret_value = -EINVAL;
    break;
   }
  }
  break;

 } while (0);
 PCH_LOG(KERN_ERR, "pch_write_ioctl returns=%d\n", ret_value);
 return ret_value;
}
