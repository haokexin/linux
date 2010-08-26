/*!
 * @file pch_phub_pci.c
 * @brief Provides all the implementation of the interfaces pertaining to the
 *        pci and gpic registrations.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/string.h>

#include "pch_common.h"
#include "pch_debug.h"
#include "pch_phub.h"
#include "pch_phub_hal.h"

/*macros*/

/*! @ingroup PHUB_PCILayer
  @def PCI_DEVICE_ID_PCH1_PHUB
  @brief Outlines the PCI Device ID.
  */
#define PCI_DEVICE_ID_PCH1_PHUB (0x8801)

/*! @ingroup PHUB_PCILayer
  @def PCH_MINOR_NOS
  @brief Outlines the Packet Hub minor numbers limit.
  */
#define PCH_MINOR_NOS (1)

/*values for configuring CLKCFG reg
 * for CAN clock of 50Mhz*/

/*! @ingroup PHUB_PCILayer
  @def CLKCFG_CAN_50MHZ
  @brief CLKCFG register setting for CAN clock of 50Mhz.
  */
#define CLKCFG_CAN_50MHZ (0x12000000)

/*! @ingroup PHUB_PCILayer
  @def CLKCFG_CANCLK_MASK
  @brief Bit mask for bit fields in CLKCFG register
      to set CAN clock to 50Mhz.
  */
#define CLKCFG_CANCLK_MASK (0xFF000000)

/**global variables*/
u32 pch_phub_base_address;
u32 pch_phub_extrom_base_address;
s32 pch_phub_suspended;

/* ToDo: major number allocation via module parameter */
static dev_t pch_phub_dev_no;
static int pch_phub_major_no;

static struct cdev pch_phub_dev;

/*! @ingroup PHUB_PCILayerAPI
  @fn static int __devinit pch_phub_probe(struct pci_dev* pch_pci_dev,
      const struct pci_device_id* pci_id)
  @brief  Provides the functionality of probing the module
  */
static int __devinit pch_phub_probe(struct pci_dev *pdev, const
	struct pci_device_id *id);

/*! @ingroup PHUB_PCILayerAPI
  @fn static void __devexit pch_phub_remove(struct pci_dev * pch_pci_dev)
  @brief  Provides the functionality of removing the module
  */
static void __devexit pch_phub_remove(struct pci_dev *pdev);

/*! @ingroup PHUB_PCILayerAPI
  @fn static int pch_phub_suspend(struct pci_dev* pDev,pm_message_t state)
  @brief  Provides the functionality of suspending the module
  */
static int pch_phub_suspend(struct pci_dev *pdev, pm_message_t state);

/*! @ingroup PHUB_PCILayerAPI
  @fn static int pch_phub_resume(struct pci_dev* pDev)
  @brief  Provides the functionality of resuming the module
  */
static int pch_phub_resume(struct pci_dev *pdev);

/*structures*/
/*! @ingroup PHUB_PCILayerFacilitators
  @static struct pci_device_id
  @brief It is a structure used for perserving information related to the
  device id.
  @note
  The concerned details should be provided as a reference in the pci driver
  structure.
  */
static struct pci_device_id pch_phub_pcidev_id[] = {

 {PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_PCH1_PHUB)},
 {0,}
};

/*! @ingroup PHUB_PCILayerFacilitators
  @static struct pch_phub_driver
  @brief It is a structure used for perserving information related to
  the Packet Hub device and preserves function signatures to
  manipulate the device.
  @note  The structure contains the various interfaces aspects
  provided to the pci layer.
  @see
  pch_phub_probe
  pch_phub_suspend
  pch_phub_resume
  pch_phub_remove
  */
static struct pci_driver pch_phub_driver = {
 .name = "pch_phub",
 .id_table = pch_phub_pcidev_id,
 .probe = pch_phub_probe,
 .remove = __devexit_p(pch_phub_remove),
#ifdef CONFIG_PM
 .suspend = pch_phub_suspend,
 .resume = pch_phub_resume
#endif
};

/*! @ingroup PHUB_PCILayerAPI
 * @fn static int __init pch_phub_pci_init(void)
 * @brief  Provides the functionality of initializing the module
 * */
static int __init pch_phub_pci_init(void);
/*! @ingroup PHUB_PCILayerAPI
 * @fn static void __exit pch_phub_pci_exit(void)
 * @brief  Provides the functionality of exiting the module
 * */
static void __exit pch_phub_pci_exit(void);

MODULE_DESCRIPTION("PCH PACKET HUB PCI Driver");
MODULE_LICENSE("GPL");
module_init(pch_phub_pci_init);
module_exit(pch_phub_pci_exit);
module_param(pch_phub_major_no, int, S_IRUSR | S_IWUSR);

/*function implementations*/

/*! @ingroup PHUB_PCILayerAPI
  @fn  static int __init pch_phub_pci_init(void)
  @remarks  Implements the initialization functionality of the module.
  @param  NONE
  @retval returnvalue [@ref OUT] contains the result for the concerned attempt.
   The result would generally comprise of success code
   or failure code. The failure code will indicate reason for
   failure.
  @see
   pch_phub_pci_exit
  */
static int __init pch_phub_pci_init(void)
{
 s32 ret;
 ret = pci_register_driver(&pch_phub_driver);
 PCH_DEBUG("pch_phub_pci_init : "
   "Invoked pci_register_driver successfully\n");
 PCH_DEBUG("pch_phub_pci_init returns %d\n", ret);
 return ret;
}

/*! @ingroup PHUB_PCILayerAPI
  @fn  static void __exit pch_phub_pci_exit(void)
  @remarks  Implements the exit functionality of the module.
  @param  NONE
  @retval returnvalue  [@ref OUT] contains the result for the concerned attempt.
   The result would generally comprise of success code
   or failure code. The failure code will indicate reason for
   failure.
  @see
   pch_phub_pci_init
  */
static void __exit pch_phub_pci_exit(void)
{
 pci_unregister_driver(&pch_phub_driver);
 PCH_DEBUG("pch_phub_pci_exit : "
   "Invoked pci_unregister_driver successfully\n");
}

/*! @ingroup PHUB_PCILayerAPI
  @fn  static int __devinit pch_phub_probe(struct pci_dev* pdev,
       const struct pci_device_id* id)
  @remarks  Implements the probe functionality of the module.
  @param  pdev [@ref INOUT] Contains the reference of the pci_dev structure
  @param  id [@ref INOUT] Contains the reference of the pci_device_id structure
  @retval returnvalue [@ref OUT] contains the result for the concerned attempt.
   The result would generally comprise of success code
   or failure code. The failure code will indicate reason for
   failure.
  @see
   pch_phub_pci_init
  */
static int __devinit pch_phub_probe(struct pci_dev *pdev,
	const struct pci_device_id *id)
{

 char *DRIVER_NAME = "pch_phub";
 int ret;
 unsigned int rom_size;

 pch_phub_major_no = (pch_phub_major_no < 0
    || pch_phub_major_no >
    254) ? 0 : pch_phub_major_no;

 do {
  ret = pci_enable_device(pdev);
  if (ret) {
   PCH_LOG(KERN_ERR, "\npch_phub_probe : "
    "pci_enable_device FAILED");
   break;
  }
  PCH_DEBUG("pch_phub_probe : "
    "pci_enable_device returns %d\n", ret);

  ret = pci_request_regions(pdev, DRIVER_NAME);
  if (ret) {
   PCH_LOG(KERN_ERR, "pch_phub_probe : "
	"pci_request_regions FAILED");
	pci_disable_device(pdev);
   break;
  }
  PCH_DEBUG("pch_phub_probe : "
	"pci_request_regions returns %d\n", ret);

  pch_phub_base_address = (unsigned long)pci_iomap(pdev, 1, 0);

  if (pch_phub_base_address == 0) {
   PCH_LOG(KERN_ERR,
    "pch_phub_probe : pci_iomap FAILED");
   pci_release_regions(pdev);
   pci_disable_device(pdev);
   ret = -ENOMEM;
   break;
  }
  PCH_DEBUG("pch_phub_probe : "
   "pci_iomap SUCCESS and value "
   "in pch_phub_base_address variable is 0x%08x\n",
   pch_phub_base_address);

  pch_phub_extrom_base_address =
      (unsigned long)pci_map_rom(pdev, &rom_size);
  if (pch_phub_extrom_base_address == 0) {
   PCH_LOG(KERN_ERR,
    "pch_phub_probe : pci_map_rom FAILED");
   pci_iounmap(pdev, (void *)pch_phub_base_address);
   pci_release_regions(pdev);
   pci_disable_device(pdev);
   ret = -ENOMEM;
   break;
  }
  PCH_DEBUG("pch_phub_probe : "
   "pci_map_rom SUCCESS and value in "
   "pch_phub_extrom_base_address variable is 0x%08x\n",
   pch_phub_extrom_base_address);

  if (pch_phub_major_no) {
   pch_phub_dev_no = MKDEV(pch_phub_major_no, 0);
   ret =
	register_chrdev_region(pch_phub_dev_no,
	PCH_MINOR_NOS, DRIVER_NAME);
   if (ret) {
    PCH_LOG(KERN_ERR, "pch_phub_probe : "
	"register_chrdev_region FAILED");
    pci_unmap_rom(pdev,
	(void *)pch_phub_extrom_base_address);
    pci_iounmap(pdev,
	(void *)pch_phub_base_address);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    break;
   }
   PCH_DEBUG("pch_phub_probe : "
	"register_chrdev_region returns %d\n", ret);
  } else {
   ret =
	alloc_chrdev_region(&pch_phub_dev_no, 0,
	PCH_MINOR_NOS, DRIVER_NAME);
   if (ret) {
    PCH_LOG(KERN_ERR, "pch_phub_probe : "
	"alloc_chrdev_region FAILED");
    pci_unmap_rom(pdev,
	(void *)pch_phub_extrom_base_address);
    pci_iounmap(pdev,
	(void *)pch_phub_base_address);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    break;
   }
   PCH_DEBUG("pch_phub_probe : "
	"alloc_chrdev_region returns %d\n", ret);
  }

  cdev_init(&pch_phub_dev, &pch_phub_fops);
  PCH_DEBUG
	("pch_phub_probe :  cdev_init invoked successfully\n");

  pch_phub_dev.owner = THIS_MODULE;
  pch_phub_dev.ops = &pch_phub_fops;

  ret =
	cdev_add(&pch_phub_dev, pch_phub_dev_no,
	PCH_MINOR_NOS);
  if (ret) {
   PCH_LOG(KERN_ERR,
	"pch_phub_probe :  cdev_add FAILED");
   unregister_chrdev_region(pch_phub_dev_no,
	PCH_MINOR_NOS);
   pci_unmap_rom(pdev,
	(void *)pch_phub_extrom_base_address);
   pci_iounmap(pdev, (void *)pch_phub_base_address);
   pci_release_regions(pdev);
   pci_disable_device(pdev);
   break;
  }
  PCH_DEBUG("pch_phub_probe :  cdev_add returns %d\n", ret);

#ifdef PCH_CAN_PCLK_50MHZ
  /*set the clock config reg if CAN clock is 50Mhz */
  PCH_DEBUG("pch_phub_probe : invoking "
	"pch_phub_read_modify_write_reg "
	"to set CLKCFG reg for CAN clk 50Mhz\n");
  pch_phub_read_modify_write_reg(CLKCFG_REG_OFFSET,
	CLKCFG_CAN_50MHZ, CLKCFG_CANCLK_MASK);
#endif
  /* set the prefech value */
  pch_phub_write_reg(0x14, 0x000ffffa);
  /* set the interrupt delay value */
  pch_phub_write_reg(0x44, 0x25);
  return PCH_PHUB_SUCCESS;
 } while (0);
 PCH_DEBUG("pch_phub_probe returns %d\n", ret);
 return ret;
}

/*! @ingroup PHUB_PCILayerAPI
  @fn  static void __devexit pch_phub_remove(struct pci_dev * pdev)
  @remarks  Implements the remove functionality of the module.
  @param  pdev [@ref INOUT] Contains the reference of the pci_dev structure
  @retval returnvalue [@ref OUT] contains the result for the concerned attempt.
   The result would generally comprise of success code
   or failure code. The failure code will indicate reason for failure.
  @see
   pch_phub_pci_init
  */
static void __devexit pch_phub_remove(struct pci_dev *pdev)
{

 cdev_del(&pch_phub_dev);
 PCH_DEBUG("pch_phub_remove - cdev_del Invoked successfully\n");

 unregister_chrdev_region(pch_phub_dev_no, PCH_MINOR_NOS);
 PCH_DEBUG("pch_phub_remove - "
  "unregister_chrdev_region Invoked successfully\n");

 pci_unmap_rom(pdev, (void *)pch_phub_extrom_base_address);

 pci_iounmap(pdev, (void *)pch_phub_base_address);

 PCH_DEBUG("pch_phub_remove - pci_iounmap Invoked successfully\n");

 pci_release_regions(pdev);
 PCH_DEBUG
     ("pch_phub_remove - pci_release_regions Invoked successfully\n");

 pci_disable_device(pdev);
 PCH_DEBUG
     ("pch_phub_remove - pci_disable_device Invoked successfully\n");

}

#ifdef CONFIG_PM

/*! @ingroup PHUB_PCILayerAPI
  @fn static int pch_phub_suspend(struct pci_dev* pdev,pm_message_t state)
  @remarks  Implements the suspend functionality of the module.
  @param  pdev [@ref INOUT] Contains the reference of the pci_dev structure
  @param  state [@ref INOUT] Contains the reference of the pm_message_t
	structure
  @retval returnvalue [@ref OUT] contains the result for the concerned attempt.
   The result would generally comprise of success code
   or failure code. The failure code will indicate reason for failure.
  @see
   pch_phub_pci_init
   pch_phub_resume
  */
static int pch_phub_suspend(struct pci_dev *pdev, pm_message_t state)
{
 int ret;

 pch_phub_suspended = true; /* For blocking further IOCTLs */

 pch_phub_save_reg_conf();
 PCH_DEBUG("pch_phub_suspend - "
	"pch_phub_save_reg_conf Invoked successfully\n");

 ret = pci_save_state(pdev);
 if (ret) {
  PCH_LOG(KERN_ERR, " pch_phub_suspend -pci_save_state returns-%d\n",
	ret);
  return ret;
 }

 pci_enable_wake(pdev, PCI_D3hot, 0);
 PCH_DEBUG("pch_phub_suspend - "
   "pci_enable_wake Invoked successfully\n");
 PCH_DEBUG("pch_phub_suspend - pci_save_state returns %d\n", ret);

 pci_disable_device(pdev);
 PCH_DEBUG("pch_phub_suspend - "
	"pci_disable_device Invoked successfully\n");

 pci_set_power_state(pdev, pci_choose_state(pdev, state));
 PCH_DEBUG("pch_phub_suspend - "
	"pci_set_power_state Invoked successfully\n");
 PCH_DEBUG("pch_phub_suspend - return %d\n", PCH_PHUB_SUCCESS);

 return PCH_PHUB_SUCCESS;
}

/*! @ingroup PHUB_PCILayerAPI
  @fn static int pch_phub_resume(struct pci_dev* pdev)
  @remarks  Implements the resume functionality of the module.
  @param  pdev [@ref INOUT] Contains the reference of the pci_dev structure
  @retval returnvalue [@ref OUT] contains the result for the concerned attempt.
   The result would generally comprise of success code
   or failure code. The failure code will indicate reason for failure.
  @see
   pch_phub_pci_init
   pch_phub_suspend
  */
static int pch_phub_resume(struct pci_dev *pdev)
{

 int ret;

 pci_set_power_state(pdev, PCI_D0);
 PCH_DEBUG("pch_phub_resume - "
	"pci_set_power_state Invoked successfully\n");

 pci_restore_state(pdev);
 PCH_DEBUG("pch_phub_resume - "
	"pci_restore_state Invoked successfully\n");

 ret = pci_enable_device(pdev);
 if (ret) {
  PCH_LOG(KERN_ERR,
	"pch_phub_resume-pci_enable_device failed ");
  return ret;
 }

 PCH_DEBUG("pch_phub_resume - pci_enable_device returns -%d\n", ret);

 pci_enable_wake(pdev, PCI_D3hot, 0);
 PCH_DEBUG("pch_phub_resume - "
	"pci_enable_wake Invoked successfully\n");

 pch_phub_restore_reg_conf();
 PCH_DEBUG("pch_phub_resume - "
	"pch_phub_restore_reg_conf Invoked successfully\n");

 pch_phub_suspended = false;

 PCH_DEBUG("pch_phub_resume  returns- %d\n", PCH_PHUB_SUCCESS);
 return PCH_PHUB_SUCCESS;
}

#endif
