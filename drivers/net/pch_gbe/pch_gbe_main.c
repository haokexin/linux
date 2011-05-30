/*!
 * @file pch_gbe_main.c
 * @brief Linux PCH Gigabit Ethernet Driver main source file
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307,USA.
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

#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>

#include <linux/in.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <net/ip.h>

#include "pch_debug.h"
#include "pch_gbe_osdep.h"
#include "pch_gbe_regs.h"
#include "pch_gbe_defines.h"
#include "pch_gbe_hw.h"
#include "pch_gbe_api.h"
#include "pch_gbe.h"

/* ----------------------------------------------------------------------------
 Function prototype
---------------------------------------------------------------------------- */
static int
pch_gbe_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id);
static void pch_gbe_remove(struct pci_dev *pdev);
static int pch_gbe_suspend(struct pci_dev *pdev, pm_message_t state);
static int pch_gbe_resume(struct pci_dev *pdev);
static void pch_gbe_shutdown(struct pci_dev *pdev);
#ifdef CONFIG_NET_POLL_CONTROLLER
static void pch_gbe_netpoll(struct net_device *netdev);
#endif
static pci_ers_result_t
pch_gbe_io_error_detected(struct pci_dev *pdev, pci_channel_state_t state);
static pci_ers_result_t pch_gbe_io_slot_reset(struct pci_dev *pdev);
static void pch_gbe_io_resume(struct pci_dev *pdev);

static int pch_gbe_init_module(void);
static void pch_gbe_exit_module(void);
static int pch_gbe_open(struct net_device *netdev);
static int pch_gbe_stop(struct net_device *netdev);
static int pch_gbe_xmit_frame(struct sk_buff *skb, struct net_device *netdev);
static struct net_device_stats *pch_gbe_get_stats(struct net_device *netdev);
static void pch_gbe_set_multi(struct net_device *netdev);
static int pch_gbe_set_mac(struct net_device *netdev, void *p);
static int pch_gbe_change_mtu(struct net_device *netdev, int new_mtu);
static int pch_gbe_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd);
static void pch_gbe_tx_timeout(struct net_device *dev);

static int pch_gbe_sw_init(struct pch_gbe_adapter *adapter);
static int pch_gbe_alloc_queues(struct pch_gbe_adapter *adapter);
static void pch_gbe_init_stats(struct pch_gbe_adapter *adapter);
static int pch_gbe_init_nvm(struct pch_gbe_adapter *adapter);
static int pch_gbe_init_phy(struct pch_gbe_adapter *adapter);
static void pch_gbe_reset_task(struct work_struct *work);
static int pch_gbe_request_irq(struct pch_gbe_adapter *adapter);
static void pch_gbe_free_irq(struct pch_gbe_adapter *adapter);
static void pch_gbe_irq_disable(struct pch_gbe_adapter *adapter);
static void pch_gbe_irq_enable(struct pch_gbe_adapter *adapter);
static void pch_gbe_clean_tx_ring(struct pch_gbe_adapter *adapter,
    struct pch_gbe_tx_ring *tx_ring);
static void pch_gbe_clean_rx_ring(struct pch_gbe_adapter *adapter,
    struct pch_gbe_rx_ring *rx_ring);
static void
pch_gbe_unmap_and_free_tx_resource(struct pch_gbe_adapter *adapter,
    struct pch_gbe_buffer *buffer_info);
static void
pch_gbe_unmap_and_free_rx_resource(struct pch_gbe_adapter *adapter,
    struct pch_gbe_buffer *buffer_info);

static void pch_gbe_setup_tctl(struct pch_gbe_adapter *adapter);
static void pch_gbe_configure_tx(struct pch_gbe_adapter *adapter);
static void pch_gbe_setup_rctl(struct pch_gbe_adapter *adapter);
static void pch_gbe_configure_rx(struct pch_gbe_adapter *adapter);
static void pch_gbe_set_rgmii_ctrl(struct pch_gbe_adapter *adapter,
     u16 speed, u16 duplex);
static void pch_gbe_set_mode(struct pch_gbe_adapter *adapter,
     u16 speed, u16 duplex);
static void pch_gbe_watchdog(unsigned long data);
static irqreturn_t pch_gbe_intr(int irq, void *data);
static unsigned char pch_gbe_clean_tx(struct pch_gbe_adapter *adapter,
    struct pch_gbe_tx_ring *tx_ring);
static int pch_gbe_napi_poll(struct napi_struct *napi, int budget);
static unsigned char pch_gbe_clean_rx(struct pch_gbe_adapter *adapter,
    struct pch_gbe_rx_ring *rx_ring,
    int *work_done, int work_to_do);
static void pch_gbe_alloc_rx_buffers(struct pch_gbe_adapter *adapter,
    struct pch_gbe_rx_ring *rx_ring,
    int cleaned_count);
static void pch_gbe_alloc_tx_buffers(struct pch_gbe_adapter *adapter,
    struct pch_gbe_tx_ring *tx_ring);
static void pch_gbe_tx_queue(struct pch_gbe_adapter *adapter,
   struct pch_gbe_tx_ring *tx_ring,
   struct sk_buff *skb);

/* ----------------------------------------------------------------------------
 Data
---------------------------------------------------------------------------- */
/*!
 * @ingroup PCI driver Layer
 * @struct  pch_gbe_pcidev_id
 * @brief   PCI Device ID Table
 * @remarks
 *  This is an instance of pci_device_id structure defined in linux/pci.h,
 *  and holds information of the PCI devices that are supported by this driver.
 */
static const struct pci_device_id pch_gbe_pcidev_id[3] = {
 {.vendor = PCI_VENDOR_ID_INTEL,
  .device = PCI_DEVICE_ID_INTEL_IOH1_GBE,
  .subvendor = PCI_ANY_ID,
  .subdevice = PCI_ANY_ID,
  .class = (PCI_CLASS_NETWORK_ETHERNET << 8),
  .class_mask = (0xFFFF00)
  },
 /* required last entry */
 {0}
};

/*!
 * @ingroup PCI driver Layer
 * @struct  pch_gbe_err_handler
 * @brief   Error handler Table
 * @remarks
 *  This is an instance of pci_error_handlers structure defined in linux/pci.h,
 *  and holds information of the PCI devices that are supported by this driver.
 */
static struct pci_error_handlers pch_gbe_err_handler = {
 .error_detected = pch_gbe_io_error_detected,
 .slot_reset = pch_gbe_io_slot_reset,
 .resume = pch_gbe_io_resume
};

/*!
 * @ingroup PCI driver Layer
 * @struct  pch_gbe_pcidev
 * @brief   Store the pointers of pci driver interfaces to kernel
 */
static struct pci_driver pch_gbe_pcidev = {
 .name = DRV_NAME,
 .id_table = pch_gbe_pcidev_id,
 .probe = pch_gbe_probe,
 .remove = pch_gbe_remove,
 /* Power Managment Hooks */
#ifdef CONFIG_PM
 .suspend = pch_gbe_suspend,
 .resume = pch_gbe_resume,
#endif
 .shutdown = pch_gbe_shutdown,
 .err_handler = &pch_gbe_err_handler
};

static int debug = PCH_GBE_NETIF_MSG_DEFAULT;
static unsigned int copybreak __read_mostly = PCH_GBE_COPYBREAK_DEFAULT;

/* ----------------------------------------------------------------------------
 module
---------------------------------------------------------------------------- */

MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_AUTHOR(DRV_COPYRIGHT);
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, pch_gbe_pcidev_id);

module_param(copybreak, uint, 0644);
MODULE_PARM_DESC(copybreak,
 "Maximum size of packet that is copied to a new buffer on receive");
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debug level (0=none,...,16=all)");

module_init(pch_gbe_init_module);
module_exit(pch_gbe_exit_module);

/* ----------------------------------------------------------------------------
 Macro Function
---------------------------------------------------------------------------- */
#define PCH_GBE_GET_DESC(R, i, type)    (&(((struct type *)((R).desc))[i]))
#define PCH_GBE_RX_DESC(R, i)           PCH_GBE_GET_DESC(R, i, pch_gbe_rx_desc)
#define PCH_GBE_TX_DESC(R, i)           PCH_GBE_GET_DESC(R, i, pch_gbe_tx_desc)
#define PCH_GBE_DESC_UNUSED(R) \
 ((((R)->next_to_clean > (R)->next_to_use) ? 0 : (R)->count) + \
 (R)->next_to_clean - (R)->next_to_use - 1)

/* ----------------------------------------------------------------------------
 Function
---------------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------
 PCI driver methods
---------------------------------------------------------------------------- */

static const struct net_device_ops pch_gbe_netdev_ops = {
 .ndo_open = pch_gbe_open,
 .ndo_stop = pch_gbe_stop,
 .ndo_start_xmit = pch_gbe_xmit_frame,
 .ndo_get_stats = pch_gbe_get_stats,
 .ndo_set_mac_address = pch_gbe_set_mac,
 .ndo_tx_timeout = pch_gbe_tx_timeout,
 .ndo_change_mtu = pch_gbe_change_mtu,
 .ndo_do_ioctl = pch_gbe_ioctl,
 .ndo_set_multicast_list = &pch_gbe_set_multi,
#ifdef CONFIG_NET_POLL_CONTROLLER
 .ndo_poll_controller = pch_gbe_netpoll,
#endif
};

/*!
 * @ingroup PCI driver method
 * @fn      static int pch_gbe_probe(struct pci_dev *pdev,
 *                                   const struct pci_device_id *pci_id)
 * @brief   Device Initialization Routine
 * @param   pdev    [INOUT] PCI device information struct
 * @param   pci_id  [INOUT] Entry in pch_gbe_pcidev_id
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 * @remarks
 *  This function initializes an adapter identified by a pci_dev structure.
 *  The OS initialization, configuring of the adapter private structure,
 *  and a hardware reset occur.
 */
static int
pch_gbe_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
 struct net_device *netdev;
 struct pch_gbe_adapter *adapter;
 unsigned long mmio_start;
 unsigned long mmio_len;
 static int cards_found;
 int i, ret;

 PCH_DEBUG("pch_gbe_probe\n");

 cards_found = 0;
 ret = pci_enable_device(pdev);
 if (ret != 0)
  return ret;
 ret = pci_set_dma_mask(pdev, DMA_32BIT_MASK);
 if (ret != 0) {
  ret = pci_set_consistent_dma_mask(pdev, DMA_32BIT_MASK);
  if (ret != 0) {
   PCH_LOG(KERN_ERR,
       "ERR: No usable DMA configuration, aborting\n");
   goto err_disable_device;
  }
 }
 ret = pci_request_regions(pdev, DRV_NAME);
 if (ret != 0) {
  PCH_LOG(KERN_ERR,
      "ERR: Can't reserve PCI I/O and memory resources\n");
  goto err_disable_device;
 }
 pci_set_master(pdev);

 netdev = alloc_etherdev((int)sizeof(struct pch_gbe_adapter));
 if (!netdev) {
  ret = -ENOMEM;
  PCH_LOG(KERN_ERR,
      "ERR: Can't allocates and sets up an Ethernet device\n");
  goto err_release_pci;
 }
 SET_NETDEV_DEV(netdev, &pdev->dev);

 pci_set_drvdata(pdev, netdev);
 adapter = netdev_priv(netdev);
 adapter->netdev = netdev;
 adapter->pdev = pdev;
 adapter->msg_enable = (1 << debug) - 1;
 adapter->bd_number = cards_found;
 adapter->hw.back = adapter;
 mmio_start = pci_resource_start(pdev, PCH_GBE_PCI_BAR);
 mmio_len = pci_resource_len(pdev, PCH_GBE_PCI_BAR);
 adapter->hw.hw_addr = ioremap(mmio_start, mmio_len);
 if (!adapter->hw.hw_addr) {
  ret = -EIO;
  DPRINTK(PROBE, ERR, "Can't ioremap\n");
  goto err_free_netdev;
 }

 netdev->netdev_ops = &pch_gbe_netdev_ops;

 netdev->watchdog_timeo = PCH_GBE_WATCHDOG_PERIOD;
 netif_napi_add(netdev, &adapter->napi,
	pch_gbe_napi_poll, PCH_GBE_RX_WEIGHT);
 strncpy(netdev->name, pci_name(pdev), (int)sizeof(netdev->name) - 1);
 netdev->mem_start = mmio_start;
 netdev->mem_end = mmio_start + mmio_len;
 netdev->features = NETIF_F_HW_CSUM;
 pch_gbe_set_ethtool_ops(netdev);

 /* setup the private structure */
 ret = pch_gbe_sw_init(adapter);
 if (ret != 0)
  goto err_iounmap;

 pch_gbe_hal_reset_hw(&adapter->hw);
 /* Initialize PHY */
 ret = pch_gbe_init_phy(adapter);
 if (ret != 0) {
  DPRINTK(PROBE, ERR, "PHY initialize error\n");
  goto err_free_adapter;
 }

 pch_gbe_hal_get_bus_info(&adapter->hw);

 /* Initialize NVM */
 ret = pch_gbe_init_nvm(adapter);
 if (ret != 0) {
  DPRINTK(PROBE, ERR, "NVM initialize error\n");
  goto err_free_adapter;
 }

#ifdef CONFIG_PCH_PHUB
 /* Read the MAC address. and store to the private data */
 ret = pch_gbe_hal_read_mac_addr(&adapter->hw);
 if (ret != 0) {
  DPRINTK(PROBE, ERR, "MAC address Read Error\n");
  goto err_free_adapter;
 }
#endif
 memcpy(netdev->dev_addr, adapter->hw.mac.addr, netdev->addr_len);
 if (!is_valid_ether_addr(netdev->dev_addr)) {
  DPRINTK(PROBE, ERR, "Invalid MAC Address\n");
  ret = -EIO;
  goto err_free_adapter;
 }

 init_timer(&adapter->watchdog_timer);
 adapter->watchdog_timer.function = &pch_gbe_watchdog;
 adapter->watchdog_timer.data = (unsigned long)adapter;

 INIT_WORK(&adapter->reset_task, pch_gbe_reset_task);

 pch_gbe_check_options(adapter);

 if (adapter->tx_csum != 0)
  netdev->features |= NETIF_F_HW_CSUM;
 else
  netdev->features &= ~NETIF_F_HW_CSUM;

 /* initialize the wol settings based on the eeprom settings */
 adapter->wake_up_evt = PCH_GBE_WL_INIT_SETTING;

 /* print bus type/speed/width info */
 {
  struct pch_gbe_hw *hw = &adapter->hw;
  DPRINTK(PROBE, INFO, "(PCI%s:%s:%s) ",
  ((hw->bus.type == pch_gbe_bus_type_pcix) ? "-X" :
    (hw->bus.type == pch_gbe_bus_type_pci_express) ? " Express" :
  ""),
  ((hw->bus.speed == pch_gbe_bus_speed_2500) ? "2.5Gb/s" :
    (hw->bus.speed == pch_gbe_bus_speed_133) ? "133MHz" :
    (hw->bus.speed == pch_gbe_bus_speed_120) ? "120MHz" :
    (hw->bus.speed == pch_gbe_bus_speed_100) ? "100MHz" :
    (hw->bus.speed == pch_gbe_bus_speed_66) ? "66MHz" :
    (hw->bus.speed == pch_gbe_bus_speed_33) ? "33MHz" :
  ""),
  ((hw->bus.width == pch_gbe_bus_width_64) ? "64-bit" :
    (hw->bus.width == pch_gbe_bus_width_32) ? "32-bit" :
    (hw->bus.width == pch_gbe_bus_width_pcie_x4) ? "Width x4" :
    (hw->bus.width == pch_gbe_bus_width_pcie_x2) ? "Width x2" :
    (hw->bus.width == pch_gbe_bus_width_pcie_x1) ? "Width x1" :
  ""));
 }
 for (i = 0; i < 6; i++)
  printk(KERN_INFO "%2.2x%c",
   netdev->dev_addr[i], i == 5 ? '\n' : ':');

 /* reset the hardware with the new settings */
 pch_gbe_reset(adapter);

 strcpy(netdev->name, "eth%d");
 ret = register_netdev(netdev);
 if (ret != 0)
  goto err_free_adapter;
 /* tell the stack to leave us alone until pch_gbe_open() is called */
 netif_carrier_off(netdev);
 netif_stop_queue(netdev);

 DPRINTK(PROBE, INFO, "OKIsemi(R) PCH Network Connection\n");

 cards_found++;
 device_set_wakeup_enable(&pdev->dev, 1);
 return PCH_GBE_SUCCESS;

err_free_adapter:
 pch_gbe_hal_phy_hw_reset(&adapter->hw);
 dev_put(adapter->polling_netdev);
 kfree(adapter->tx_ring);
 kfree(adapter->rx_ring);
 kfree(adapter->polling_netdev);
err_iounmap:
 iounmap(adapter->hw.hw_addr);
err_free_netdev:
 free_netdev(netdev);
err_release_pci:
 pci_release_regions(pdev);
err_disable_device:
 pci_disable_device(pdev);
 return ret;
}

/*!
 * @ingroup PCI driver method
 * @fn      static void pch_gbe_remove(struct pci_dev *pdev)
 * @brief   Device Removal Routine
 * @param   pdev     [INOUT] PCI device information struct
 * @return  None
 * @remarks
 *  This function is called by the PCI subsystem to alert the driver
 *  that it should release a PCI device.  The could be caused by a
 *  Hot-Plug event, or because the driver is going to be removed from
 *  memory.
 */
static void pch_gbe_remove(struct pci_dev *pdev)
{
 struct net_device *netdev = pci_get_drvdata(pdev);
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 DPRINTK(PROBE, DEBUG, "\n");

 flush_scheduled_work();
 unregister_netdev(netdev);
 dev_put(adapter->polling_netdev);

 pch_gbe_hal_phy_hw_reset(&adapter->hw);

 kfree(adapter->tx_ring);
 kfree(adapter->rx_ring);
 kfree(adapter->polling_netdev);

 iounmap(adapter->hw.hw_addr);
 pci_release_regions(pdev);
 free_netdev(netdev);
 pci_disable_device(pdev);
}

/*!
 * @ingroup PCI driver method
 * @fn      static int pch_gbe_suspend(struct pci_dev *pdev, pm_message_t state)
 * @brief   Device Suspend Routine
 * @param   pdev     [INOUT] PCI device information struct
 * @param   state    [IN] Power status
 * @return  None
 */
static int pch_gbe_suspend(struct pci_dev *pdev, pm_message_t state)
{
 struct net_device *netdev = pci_get_drvdata(pdev);
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_hw *hw = &adapter->hw;
 u32 wufc = adapter->wake_up_evt;
 int retval = PCH_GBE_SUCCESS;

 DPRINTK(PROBE, DEBUG, "\n");

 netif_device_detach(netdev);

 if (netif_running(netdev) != 0)
  pch_gbe_down(adapter);
#ifdef CONFIG_PM
 /* Implement our own version of pci_save_state(pdev) because pci-
  * express adapters have 256-byte config spaces. */
 retval = pci_save_state(pdev);
 if (retval != 0) {
  DPRINTK(PROBE, DEBUG, "pci_save_state failed\n");
  return retval;
 }
#endif

 if (wufc != 0) {
  pch_gbe_set_multi(netdev);
  pch_gbe_setup_rctl(adapter);
  pch_gbe_configure_rx(adapter);
  pch_gbe_set_rgmii_ctrl(adapter, hw->mac.link_speed,
     hw->mac.link_duplex);
  pch_gbe_set_mode(adapter, hw->mac.link_speed,
     hw->mac.link_duplex);
  pch_gbe_hal_set_wol_event(hw, wufc);
  pci_disable_device(pdev);
  retval = pci_set_power_state(pdev, PCI_D0);
  if (retval)
   DPRINTK(PROBE, DEBUG, "pci_set_power_state failed\n");
  retval = pci_enable_wake(pdev, PCI_D0, 1);
  if (retval)
   DPRINTK(PROBE, DEBUG, "pci_enable_wake failed\n");
 } else {
  pch_gbe_hal_power_down_phy(hw);
  pch_gbe_hal_set_wol_event(hw, wufc);
  pci_disable_device(pdev);
  pci_enable_wake(pdev, PCI_D0, 0);
  pci_set_power_state(pdev, pci_choose_state(pdev, state));
 }
 return retval;
}

#ifdef CONFIG_PM
/*!
 * @ingroup PCI driver method
 * @fn      static int pch_gbe_resume(struct pci_dev *pdev)
 * @brief   Device Resume Routine
 * @param   pdev     [INOUT] PCI device information struct
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  negative:  Failed
 */
static int pch_gbe_resume(struct pci_dev *pdev)
{
 struct net_device *netdev = pci_get_drvdata(pdev);
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_hw *hw = &adapter->hw;
 u32 err;

 DPRINTK(PROBE, DEBUG, "\n");

 pci_enable_wake(pdev, PCI_D0, 0);
 pci_set_power_state(pdev, PCI_D0);
 pci_restore_state(pdev);
 err = pci_enable_device(pdev);
 if (err != 0) {
  DPRINTK(PROBE, ERR, "Cannot enable PCI device from suspend\n");
  return err;
 }
 pci_set_master(pdev);
 pch_gbe_hal_power_up_phy(hw);
 pch_gbe_reset(adapter);
 /* Clear wake on lan control and status */
 pch_gbe_hal_set_wol_event(hw, 0);

 if (netif_running(netdev) != 0)
  pch_gbe_up(adapter);
 netif_device_attach(netdev);

 return PCH_GBE_SUCCESS;
}
#endif /* CONFIG_PM */

/*!
 * @ingroup PCI driver method
 * @fn      static void pch_gbe_shutdown(struct pci_dev *pdev)
 * @brief   Device shutdown Routine
 * @param   pdev    [INOUT] PCI device information struct
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 */
static void pch_gbe_shutdown(struct pci_dev *pdev)
{
 struct net_device *netdev = pci_get_drvdata(pdev);
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 DPRINTK(PROBE, DEBUG, "\n");
 pch_gbe_suspend(pdev, PMSG_SUSPEND);
}

/*!
 * @ingroup PCI driver method
 * @fn      static pci_ers_result_t pch_gbe_io_error_detected(
 *                                  struct pci_dev *pdev,
 *                                  pci_channel_state_t state)
 * @brief   Called when PCI error is detected
 * @param   pdev     [INOUT] PCI device information struct
 * @param   state    [IN] The current pci connection state
 * @return  PCI_ERS_RESULT_NEED_RESET
 * @remarks
 *   This function is called after a PCI bus error affecting
 *   this device has been detected.
 */
static pci_ers_result_t
pch_gbe_io_error_detected(struct pci_dev *pdev, pci_channel_state_t state)
{
 struct net_device *netdev = pci_get_drvdata(pdev);
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 DPRINTK(PROBE, DEBUG, "\n");

 netif_device_detach(netdev);

 if (netif_running(netdev) != 0)
  pch_gbe_down(adapter);
 pci_disable_device(pdev);

 /* Request a slot slot reset. */
 return PCI_ERS_RESULT_NEED_RESET;
}

/*!
 * @ingroup PCI driver method
 * @fn      static pci_ers_result_t pch_gbe_io_slot_reset(struct pci_dev *pdev)
 * @brief   Called after the pci bus has been reset.
 * @param   pdev    [INOUT] PCI device information struct
 * @return  PCI_ERS_RESULT_DISCONNECT
 * @return  PCI_ERS_RESULT_RECOVERED
 * @remarks
 *   Restart the card from scratch, as if from a cold-boot. Implementation
 *   resembles the first-half of the pch_gbe_resume routine.
 */
static pci_ers_result_t pch_gbe_io_slot_reset(struct pci_dev *pdev)
{
 struct net_device *netdev = pci_get_drvdata(pdev);
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_hw *hw = &adapter->hw;

 DPRINTK(PROBE, DEBUG, "\n");

 if (pci_enable_device(pdev) != 0) {
  DPRINTK(PROBE, ERR,
   "Cannot re-enable PCI device after reset.\n");
  return PCI_ERS_RESULT_DISCONNECT;
 }
 pci_set_master(pdev);
 pci_enable_wake(pdev, PCI_D0, 0);
 pch_gbe_hal_power_up_phy(hw);
 pch_gbe_reset(adapter);
 /* Clear wake up status */
 pch_gbe_hal_set_wol_event(hw, 0);

 return PCI_ERS_RESULT_RECOVERED;
}

/*!
 * @ingroup PCI driver method
 * @fn      static void pch_gbe_io_resume(struct pci_dev *pdev)
 * @brief   Called when traffic can start flowing again.
 * @param   pdev    [INOUT] PCI device information struct
 * @return  PCI_ERS_RESULT_DISCONNECT
 * @return  PCI_ERS_RESULT_RECOVERED
 * @remarks
 *   This callback is called when the error recovery driver tells us that
 *   its OK to resume normal operation. Implementation resembles the
 *   second-half of the pch_gbe_resume routine.
 */
static void pch_gbe_io_resume(struct pci_dev *pdev)
{
 struct net_device *netdev = pci_get_drvdata(pdev);
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 DPRINTK(PROBE, DEBUG, "\n");

 if (netif_running(netdev) != 0) {
  if (pch_gbe_up(adapter) != 0) {
   DPRINTK(PROBE, DEBUG,
    "can't bring device back up after reset\n");
   return;
  }
 }
 netif_device_attach(netdev);
}

/* ----------------------------------------------------------------------------
 Gigabit Ethernet driver methods
---------------------------------------------------------------------------- */
/*!
 * @ingroup Gigabit Ethernet driver methods
 * @fn      static int __init pch_gbe_init_module(void)
 * @brief   Driver Registration Routine
 * @return  PCH_GBE_SUCCESS  Successfully
 * @return  negative value  Failed
 * @remarks
 *   pch_gbe_init_module is the first routine called when the driver is
 *   loaded. All it does is register with the PCI subsystem.
 */
static int __init pch_gbe_init_module(void)
{
 int ret;
 PCH_DEBUG("pch_gbe_init_module\n");
 printk(KERN_INFO "%s - version %s\n", DRV_STRING, DRV_VERSION);

 ret = pci_register_driver(&pch_gbe_pcidev);
 if (copybreak != PCH_GBE_COPYBREAK_DEFAULT) {
  if (copybreak == 0) {
   printk(KERN_INFO "pch_gbe: copybreak disabled\n");
  } else {
   printk(KERN_INFO "pch_gbe: copybreak enabled for "
    "packets <= %u bytes\n", copybreak);
  }
 }
 return ret;
}

/*!
 * @ingroup Gigabit Ethernet driver methods
 * @fn      static void __exit pch_gbe_exit_module(void)
 * @brief   Driver Exit Cleanup Routine
 * @return  None
 * @remarks
 *   pch_gbe_exit_module is called just before the driver is removed
 *   from memory.
 */
static void __exit pch_gbe_exit_module(void)
{
 PCH_DEBUG("pch_gbe_exit_module\n");
 pci_unregister_driver(&pch_gbe_pcidev);

 printk(KERN_INFO "%s - unregister\n", DRV_STRING);
}

/*!
 * @ingroup Gigabit Ethernet driver methods
 * @fn      static int pch_gbe_open(struct net_device *netdev)
 * @brief   Called when a network interface is made active
 * @param   netdev  [INOUT] network interface device structure
 * @return  PCH_GBE_SUCCESS - Successfully
 * @return  negative value - Failed
 * @remarks
 *   The open entry point is called when a network interface is made
 *   active by the system (IFF_UP).  At this point all resources needed
 *   for transmit and receive operations are allocated, the interrupt
 *   handler is registered with the OS, the watchdog timer is started,
 *   and the stack is notified that the interface is ready.
 */
static int pch_gbe_open(struct net_device *netdev)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_hw *hw = &adapter->hw;
 int err;

 DPRINTK(IFUP, DEBUG, "\n");

 /* disallow open during test */
 if ((test_bit(__PCH_GBE_TESTING, &adapter->flags)) != 0)
  return -EBUSY;
 /* allocate transmit descriptors */
 err = pch_gbe_setup_tx_resources(adapter, adapter->tx_ring);
 if (err != 0)
  goto err_setup_tx;
 /* allocate receive descriptors */
 err = pch_gbe_setup_rx_resources(adapter, adapter->rx_ring);
 if (err != 0)
  goto err_setup_rx;
 pch_gbe_hal_power_up_phy(hw);
 err = pch_gbe_up(adapter);
 if (err != 0)
  goto err_up;
 DPRINTK(IFUP, DEBUG, "Success End\n");
 return PCH_GBE_SUCCESS;

err_up:
 if (!adapter->wake_up_evt)
  pch_gbe_hal_power_down_phy(hw);
 pch_gbe_free_rx_resources(adapter, adapter->rx_ring);
err_setup_rx:
 pch_gbe_free_tx_resources(adapter, adapter->tx_ring);
err_setup_tx:
 pch_gbe_reset(adapter);
 DPRINTK(IFUP, ERR, "Error End\n");
 return err;
}

/*!
 * @ingroup Gigabit Ethernet driver methods
 * @fn      static int pch_gbe_stop(struct net_device *netdev)
 * @brief   Disables a network interface
 * @param   netdev  [INOUT] network interface device structure
 * @return  PCH_GBE_SUCCESS - Successfully (This is not allowed to fail)
 * @remarks
 *   The close entry point is called when an interface is de-activated
 *   by the OS.  The hardware is still under the drivers control, but
 *   needs to be disabled.  A global MAC reset is issued to stop the
 *   hardware, and all transmit and receive resources are freed.
 */
static int pch_gbe_stop(struct net_device *netdev)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_hw *hw = &adapter->hw;

 DPRINTK(IFDOWN, DEBUG, "\n");

 pch_gbe_down(adapter);
 if (!adapter->wake_up_evt)
  pch_gbe_hal_power_down_phy(hw);
 pch_gbe_free_tx_resources(adapter, adapter->tx_ring);
 pch_gbe_free_rx_resources(adapter, adapter->rx_ring);

 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup Gigabit Ethernet driver methods
 * @fn      static int pch_gbe_xmit_frame(struct sk_buff *skb,
 *                                        struct net_device *netdev)
 * @brief   Packet transmitting start
 * @param   skb    [INOUT] socket buffer structure
 * @param   netdev [INOUT] network interface device structure
 * @return  NETDEV_TX_OK: Normal end
 * @return  NETDEV_TX_BUSY: Error end
 */
static int pch_gbe_xmit_frame(struct sk_buff *skb, struct net_device *netdev)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_tx_ring *tx_ring = adapter->tx_ring;
 unsigned long flags;

 DPRINTK(PROBE, DEBUG, "\n");

 if (unlikely(skb->len <= 0)) {
  dev_kfree_skb_any(skb);
  PCH_DEBUG("Return : OK  skb len : %d\n", skb->len);
  return NETDEV_TX_OK;
 }
 if (unlikely(skb->len > (adapter->hw.mac.max_frame_size - 4))) {
  DPRINTK(PROBE, ERR, "Transfer length Error: %d over %d\n",
   skb->len, adapter->hw.mac.max_frame_size);
  dev_kfree_skb_any(skb);
  adapter->stats.tx_length_errors++;
  return NETDEV_TX_BUSY;
 }
 if (!spin_trylock_irqsave(&tx_ring->tx_lock, flags)) {
  /* Collision - tell upper layer to requeue */
  return NETDEV_TX_LOCKED;
 }
 if (unlikely(!PCH_GBE_DESC_UNUSED(tx_ring))) {
  netif_stop_queue(netdev);
  spin_unlock_irqrestore(&tx_ring->tx_lock, flags);
  PCH_DEBUG
  ("Return : BUSY  next_to use : 0x%08x  "
   "next_to clean : 0x%08x\n",
   tx_ring->next_to_use, tx_ring->next_to_clean);
  return NETDEV_TX_BUSY;
 }
 spin_unlock_irqrestore(&tx_ring->tx_lock, flags);

 /* CRC,ITAG no support */
 pch_gbe_tx_queue(adapter, tx_ring, skb);
 netdev->trans_start = jiffies;
 return NETDEV_TX_OK;
}

/*!
 * @ingroup Gigabit Ethernet driver methods
 * @fn      static struct net_device_stats *pch_gbe_get_stats(
 *                                          struct net_device *netdev)
 * @brief   Get System Network Statistics
 * @param   netdev [INOUT] network interface device structure
 * @return  The current stats
 * @remarks
 *   Returns the address of the device statistics structure.
 *   The statistics are actually updated from the timer callback.
 */
static struct net_device_stats *pch_gbe_get_stats(struct net_device *netdev)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 DPRINTK(PROBE, DEBUG, "\n");

 /* only return the current stats */
 return &adapter->net_stats;
}

/*!
 * @ingroup Gigabit Ethernet driver methods
 * @fn      static void pch_gbe_set_multi(struct net_device *netdev)
 * @brief   Multicast and Promiscuous mode set
 * @param   netdev [INOUT] network interface device structure
 * @return  None
 * @remarks
 *   The set_multi entry point is called whenever the multicast address
 *   list or the network interface flags are updated.  This routine is
 *   responsible for configuring the hardware for proper multicast,
 *   promiscuous mode, and all-multi behavior.
 */
static void pch_gbe_set_multi(struct net_device *netdev)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_hw *hw = &adapter->hw;
 struct pch_gbe_mac_info *mac = &hw->mac;
 struct dev_mc_list *mc_ptr;
 u8 *mta_list;
 u32 rctl;
 int i;

 DPRINTK(PROBE, DEBUG, "\n");
 PCH_DEBUG("netdev->flags : 0x%08x\n", netdev->flags);

 /* Check for Promiscuous and All Multicast modes */
 rctl = PCH_GBE_READ_REG(hw, RX_MODE);

 if ((netdev->flags & IFF_PROMISC) != 0) {
  rctl &= ~PCH_GBE_ADD_FIL_EN;
  rctl &= ~PCH_GBE_MLT_FIL_EN;
 } else if ((netdev->flags & IFF_ALLMULTI) != 0) {
  /* all the multicasting receive permissions */
  rctl |= PCH_GBE_ADD_FIL_EN;
  rctl &= ~PCH_GBE_MLT_FIL_EN;
 } else {
  if (netdev->mc_count >= PCH_GBE_MAR_ENTRIES) {
   /* all the multicasting receive permissions */
   rctl |= PCH_GBE_ADD_FIL_EN;
   rctl &= ~PCH_GBE_MLT_FIL_EN;
  } else {
   rctl |= (PCH_GBE_ADD_FIL_EN | PCH_GBE_MLT_FIL_EN);
  }
 }
 PCH_GBE_WRITE_REG(hw, RX_MODE, rctl);

 if (netdev->mc_count >= PCH_GBE_MAR_ENTRIES)
  return;
 mta_list = kmalloc(netdev->mc_count * ETH_ALEN, GFP_ATOMIC);
 if (!mta_list)
  return;
 /* The shared function expects a packed array of only addresses. */
 mc_ptr = netdev->mc_list;

 for (i = 0; i < netdev->mc_count; i++) {
  if (!mc_ptr)
   break;
  memcpy(mta_list + (i * ETH_ALEN), mc_ptr->dmi_addr, ETH_ALEN);
  mc_ptr = mc_ptr->next;
 }

 pch_gbe_hal_mc_addr_list_update(hw, mta_list, i, 1,
     mac->mar_entry_count);
 kfree(mta_list);

 PCH_DEBUG
 ("RX_MODE reg(check bit31,30 ADD,MLT) : 0x%08x  "
  "netdev->mc_count : 0x%08x\n",
  PCH_GBE_READ_REG(hw, RX_MODE), netdev->mc_count);
}

/*!
 * @ingroup Gigabit Ethernet driver methods
 * @fn      static int pch_gbe_set_mac(struct net_device *netdev, void *addr)
 * @brief   Change the Ethernet Address of the NIC
 * @param   netdev [INOUT] Network interface device structure
 * @param   addr   [IN] Pointer to an address structure
 * @return  PCH_GBE_SUCCESS: Successfully
 * @return  -EADDRNOTAVAIL:  Failed
 */
static int pch_gbe_set_mac(struct net_device *netdev, void *addr)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct sockaddr *skaddr = addr;
 int ret_val;

 DPRINTK(PROBE, DEBUG, "\n");

 if (!is_valid_ether_addr(skaddr->sa_data)) {
  ret_val = -EADDRNOTAVAIL;
 } else {
  memcpy(netdev->dev_addr, skaddr->sa_data, netdev->addr_len);
  memcpy(adapter->hw.mac.addr, skaddr->sa_data, netdev->addr_len);
  pch_gbe_hal_mar_set(&adapter->hw, adapter->hw.mac.addr, 0);
  ret_val = PCH_GBE_SUCCESS;
 }
#ifdef DEBUG_TEST
 PCH_DEBUG("ret_val : 0x%08x\n", ret_val);
 PCH_DEBUG("dev_addr : %02x:%02x:%02x:%02x:%02x:%02x\n",
   netdev->dev_addr[0], netdev->dev_addr[1],
   netdev->dev_addr[2], netdev->dev_addr[3],
   netdev->dev_addr[4], netdev->dev_addr[5]);
 PCH_DEBUG("mac_addr : %02x:%02x:%02x:%02x:%02x:%02x\n",
   adapter->hw.mac.addr[0], adapter->hw.mac.addr[1],
   adapter->hw.mac.addr[2], adapter->hw.mac.addr[3],
   adapter->hw.mac.addr[4], adapter->hw.mac.addr[5]);
 PCH_DEBUG("MAC_ADR1AB reg : 0x%08x 0x%08x\n",
   PCH_GBE_READ_REG(&adapter->hw, MAC_ADR1A),
   PCH_GBE_READ_REG(&adapter->hw, MAC_ADR1B));
#endif
 return ret_val;
}

/*!
 * @ingroup Gigabit Ethernet driver methods
 * @fn      static int pch_gbe_change_mtu(struct net_device *netdev,
 *                                        int new_mtu)
 * @brief   Change the Maximum Transfer Unit
 * @param   netdev  [INOUT] Network interface device structure
 * @param   new_mtu [IN] New value for maximum frame size
 * @return  PCH_GBE_SUCCESS: Successfully
 * @return  -EINVAL:  Failed
 */
static int pch_gbe_change_mtu(struct net_device *netdev, int new_mtu)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 int max_frame;

 DPRINTK(PROBE, DEBUG, "\n");

 max_frame = new_mtu + ETH_HLEN + ETH_FCS_LEN;
 if ((max_frame < ETH_ZLEN + ETH_FCS_LEN) ||
  (max_frame > PCH_GBE_MAX_JUMBO_FRAME_SIZE)) {
  DPRINTK(PROBE, ERR, "Invalid MTU setting\n");
  return -EINVAL;
 }
 if (max_frame <= PCH_GBE_FRAME_SIZE_2048)
  adapter->rx_buffer_len = PCH_GBE_FRAME_SIZE_2048;
 else if (max_frame <= PCH_GBE_FRAME_SIZE_4096)
  adapter->rx_buffer_len = PCH_GBE_FRAME_SIZE_4096;
 else if (max_frame <= PCH_GBE_FRAME_SIZE_8192)
  adapter->rx_buffer_len = PCH_GBE_FRAME_SIZE_8192;
 else
  adapter->rx_buffer_len = PCH_GBE_MAX_JUMBO_FRAME_SIZE;
 netdev->mtu = new_mtu;
 adapter->hw.mac.max_frame_size = max_frame;

 if (netif_running(netdev) != 0)
  pch_gbe_reinit_locked(adapter);
 else
  pch_gbe_reset(adapter);

 PCH_DEBUG
 ("max_frame : %d  rx_buffer_len : %d  mtu : %d  max_frame_size : %d\n",
  max_frame, (u32) adapter->rx_buffer_len, netdev->mtu,
  adapter->hw.mac.max_frame_size);

 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup Gigabit Ethernet driver methods
 * @fn      static int pch_gbe_ioctl(struct net_device *netdev,
 *                                   struct ifreq *ifr, int cmd)
 * @brief   Controls register through a MII interface
 * @param   netdev   [INOUT] Network interface device structure
 * @param   ifr      [IN] Pointer to ifr structure
 * @param   cmd      [IN] Control command
 * @return  PCH_GBE_SUCCESS: Successfully
 * @return  Negative value:  Failed
 */
static int pch_gbe_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 DPRINTK(PROBE, DEBUG, "\n");
 PCH_DEBUG("cmd : 0x%04x\n", cmd);

 return generic_mii_ioctl(&adapter->mii, if_mii(ifr), cmd, NULL);
}

/*!
 * @ingroup Gigabit Ethernet driver methods
 * @fn      static void pch_gbe_tx_timeout(struct net_device *netdev)
 * @brief   Respond to a Tx Hang
 * @param   netdev   [INOUT] Network interface device structure
 * @return  None
 */
static void pch_gbe_tx_timeout(struct net_device *netdev)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 DPRINTK(TX_ERR, DEBUG, "\n");

 /* Do the reset outside of interrupt context */
 adapter->stats.tx_timeout_count++;
 schedule_work(&adapter->reset_task);
}

/*!
 * @ingroup Gigabit Ethernet driver methods
 * @fn      static int pch_gbe_napi_poll(struct napi_struct *napi, int budget)
 * @brief   NAPI receive and transfer polling callback
 * @param   napi    [INOUT] Pointer of polling device struct
 * @param   budget  [IN] The maximum number of a packet
 * @return  0 : Exit the polling mode
 * @return  1 : Continue the polling mode
 */
static int pch_gbe_napi_poll(struct napi_struct *napi, int budget)
{
 struct pch_gbe_adapter *adapter =
     container_of(napi, struct pch_gbe_adapter, napi);
 struct net_device *netdev = adapter->netdev;
 int work_done = 0;
 int poll_end_flag = 0;
 int cleaned = 0;

 DPRINTK(PROBE, DEBUG, "\n");
 PCH_DEBUG("budget : %d\n", budget);

 /* Keep link state information with original netdev */
 if (!netif_carrier_ok(netdev)) {
  poll_end_flag = 1;
 } else {
  cleaned = pch_gbe_clean_tx(adapter, adapter->tx_ring);
  pch_gbe_clean_rx(adapter, adapter->rx_ring, &work_done, budget);

  if (cleaned)
   work_done = budget;
  /* If no Tx and not enough Rx work done,
   * exit the polling mode
   */
  if ((work_done < budget) || !netif_running(netdev))
   poll_end_flag = 1;
 }

 if (poll_end_flag == 1) {
  napi_complete(napi);
  pch_gbe_irq_enable(adapter);
 }

 PCH_DEBUG("poll_end_flag : %d  work_done : %d  budget : %d\n",
   poll_end_flag, work_done, budget);
 return work_done;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/*!
 * @ingroup Gigabit Ethernet driver methods
 * @fn      static void pch_gbe_netpoll(struct net_device *netdev)
 * @brief   Used by things like netconsole to send skbs
 * @param   netdev  [INOUT] Network interface device structure
 * @return  None
 * @remarks
 *   used by things like netconsole to send skbs
 *   without having to re-enable interrupts.
 *   It's not called while the interrupt routine is executing.
 */
static void pch_gbe_netpoll(struct net_device *netdev)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 DPRINTK(PROBE, DEBUG, "\n");

 disable_irq(adapter->pdev->irq);
 pch_gbe_intr(adapter->pdev->irq, netdev);
 enable_irq(adapter->pdev->irq);
}
#endif

/* ----------------------------------------------------------------------------
 Linux driver internal function
---------------------------------------------------------------------------- */
/*!
 * @ingroup Linux driver internal function
 * @fn      static int pch_gbe_sw_init(struct pch_gbe_adapter *adapter)
 * @brief   Initialize general software structures (struct pch_gbe_adapter)
 * @param   adapter  [INOUT] Board private structure to initialize
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *   pch_gbe_sw_init initializes the Adapter private data structure.
 *   Fields are initialized based on PCI device information and
 *   OS network device settings (MTU size).
 */
static int pch_gbe_sw_init(struct pch_gbe_adapter *adapter)
{
 struct pch_gbe_hw *hw = &adapter->hw;
 struct net_device *netdev = adapter->netdev;
 struct pci_dev *pdev = adapter->pdev;

 DPRINTK(DRV, DEBUG, "\n");

 /* PCI config space info */
 hw->vendor_id = pdev->vendor;
 hw->device_id = pdev->device;
 hw->subsystem_vendor_id = pdev->subsystem_vendor;
 hw->subsystem_device_id = pdev->subsystem_device;

 pci_read_config_byte(pdev, PCI_REVISION_ID, &hw->revision_id);

 adapter->rx_buffer_len = PCH_GBE_FRAME_SIZE_2048;
 hw->mac.max_frame_size = netdev->mtu + ETH_HLEN + ETH_FCS_LEN;
 hw->mac.min_frame_size = ETH_ZLEN + ETH_FCS_LEN;

 /* Initialize the hardware-specific values */
 if (pch_gbe_hal_setup_init_funcs(hw) != 0) {
  DPRINTK(DRV, ERR, "Hardware Initialization Failure\n");
  return -EIO;
 }
 if (pch_gbe_alloc_queues(adapter) != 0) {
  DPRINTK(DRV, ERR, "Unable to allocate memory for queues\n");
  return -ENOMEM;
 }
 dev_hold(adapter->polling_netdev);
 set_bit(__LINK_STATE_START, &adapter->polling_netdev->state);

 spin_lock_init(&adapter->hw.miim_lock);
 spin_lock_init(&adapter->stats_lock);
 atomic_set(&adapter->irq_sem, 0);
 pch_gbe_irq_disable(adapter);

 pch_gbe_init_stats(adapter);

#ifdef DEBUG_TEST
 PCH_DEBUG("hw->vendor_id : 0x%08x\n", hw->vendor_id);
 PCH_DEBUG("hw->device_id : 0x%08x\n", hw->device_id);
 PCH_DEBUG("hw->subsystem_vendor_id :0x%08x\n",
   hw->subsystem_vendor_id);
 PCH_DEBUG("hw->subsystem_device_id :0x%08x\n",
   hw->subsystem_device_id);
 PCH_DEBUG("hw->revision_id :0x%08x\n", hw->revision_id);
 PCH_DEBUG("adapter->rx_buffer_len : %d\n",
   (u32) adapter->rx_buffer_len);
 PCH_DEBUG("hw->mac.max_frame_size : %d\n",
   hw->mac.max_frame_size);
 PCH_DEBUG("hw->mac.min_frame_size : %d\n",
   hw->mac.min_frame_size);
#endif
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static int pch_gbe_alloc_queues(struct pch_gbe_adapter *adapter)
 * @brief   Allocate memory for all rings
 * @param   adapter  [INOUT] Board private structure to initialize
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *   We allocate one ring per queue at run-time since we don't know the
 *   number of queues at compile-time.  The polling_netdev array is
 *   intended for Multiqueue, but should work fine with a single queue.
 */
static int pch_gbe_alloc_queues(struct pch_gbe_adapter *adapter)
{
 int size;

 DPRINTK(DRV, DEBUG, "\n");

 size = (int)sizeof(struct pch_gbe_tx_ring);
 adapter->tx_ring = kmalloc(size, GFP_KERNEL);
 if (!adapter->tx_ring)
  return -ENOMEM;
 memset(adapter->tx_ring, 0, size);

 size = (int)sizeof(struct pch_gbe_rx_ring);
 adapter->rx_ring = kmalloc(size, GFP_KERNEL);
 if (!adapter->rx_ring) {
  kfree(adapter->tx_ring);
  return -ENOMEM;
 }
 memset(adapter->rx_ring, 0, size);

 size = (int)sizeof(struct net_device);
 adapter->polling_netdev = kmalloc(size, GFP_KERNEL);
 if (!adapter->polling_netdev) {
  kfree(adapter->tx_ring);
  kfree(adapter->rx_ring);
  return -ENOMEM;
 }
 memset(adapter->polling_netdev, 0, size);

#ifdef DEBUG_TEST
 {
  u32 *st_area, *sp_area;
  st_area = (u32 *) adapter->tx_ring;
  sp_area = (u32 *) (adapter->tx_ring +
   (int)sizeof(struct pch_gbe_tx_ring) - 1);
  PCH_DEBUG("tx_ring : 0x%08x - 0x%08x\n", *st_area,
    *sp_area);
  st_area = (u32 *) adapter->rx_ring;
  sp_area = (u32 *) (adapter->rx_ring +
   (int)sizeof(struct pch_gbe_rx_ring) - 1);
  PCH_DEBUG("rx_ring : 0x%08x - 0x%08x\n", *st_area,
    *sp_area);
  st_area = (u32 *) adapter->polling_netdev;
  sp_area = (u32 *) (adapter->polling_netdev +
   (int)sizeof(struct net_device) - 1);
  PCH_DEBUG("polling_netdev : 0x%08x - 0x%08x\n",
    *st_area, *sp_area);
 }
#endif
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_init_stats(struct pch_gbe_adapter *adapter)
 * @brief   Initialize status
 * @param   adapter  [INOUT] Board private structure to initialize
 * @return  None
 */
static void pch_gbe_init_stats(struct pch_gbe_adapter *adapter)
{
 struct pch_gbe_hw_stats *stats = &adapter->stats;

 DPRINTK(DRV, DEBUG, "\n");

 stats->rx_packets = 0;
 stats->tx_packets = 0;
 stats->rx_bytes = 0;
 stats->tx_bytes = 0;
 stats->rx_errors = 0;
 stats->tx_errors = 0;
 stats->rx_dropped = 0;
 stats->tx_dropped = 0;
 stats->multicast = 0;
 stats->collisions = 0;
 stats->rx_crc_errors = 0;
 stats->rx_frame_errors = 0;
 stats->rx_alloc_buff_failed = 0;
 stats->tx_length_errors = 0;
 stats->tx_aborted_errors = 0;
 stats->tx_carrier_errors = 0;
 stats->tx_timeout_count = 0;
 stats->tx_restart_count = 0;
 stats->intr_rx_dsc_empty_count = 0;
 stats->intr_rx_frame_err_count = 0;
 stats->intr_rx_fifo_err_count = 0;
 stats->intr_rx_dma_err_count = 0;
 stats->intr_tx_fifo_err_count = 0;
 stats->intr_tx_dma_err_count = 0;
 stats->intr_tcpip_err_count = 0;
 return;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static int pch_gbe_init_nvm(struct pch_gbe_adapter *adapter)
 * @brief   Initialize NVM
 * @param   adapter  [INOUT] Board private structure to initialize
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 */
static int pch_gbe_init_nvm(struct pch_gbe_adapter *adapter)
{
 DPRINTK(DRV, DEBUG, "\n");

#ifdef CONFIG_PCH_PHUB
 /* make sure the NVM is good */
 if ((pch_gbe_hal_validate_nvm_checksum(&adapter->hw) < 0)) {
  DPRINTK(DRV, ERR, "The NVM Checksum Is Not Valid\n");
  return -EIO;
 }
#endif
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static int pch_gbe_init_phy(struct pch_gbe_adapter *adapter)
 * @brief   Initialize PHY
 * @param   adapter  [INOUT] Board private structure to initialize
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 */
static int pch_gbe_init_phy(struct pch_gbe_adapter *adapter)
{
 struct net_device *netdev = adapter->netdev;
 u32 addr;
 u16 bmcr, stat;

 DPRINTK(DRV, DEBUG, "\n");

 /* Discover phy addr by searching addrs in order {1,0,2,..., 31} */
 for (addr = 0; addr <= PHY_MAX_REG_ADDRESS; addr++) {
  adapter->mii.phy_id = (addr == 0) ? 1 : (addr == 1) ? 0 : addr;
  bmcr = pch_gbe_mdio_read(netdev, adapter->mii.phy_id, MII_BMCR);
  stat = pch_gbe_mdio_read(netdev, adapter->mii.phy_id, MII_BMSR);
  stat = pch_gbe_mdio_read(netdev, adapter->mii.phy_id, MII_BMSR);
  if (!((bmcr == 0xFFFF) || ((stat == 0) && (bmcr == 0))))
   break;
 }
 adapter->hw.phy.addr = adapter->mii.phy_id;
 DPRINTK(DRV, DEBUG, "phy_addr = %d\n", adapter->mii.phy_id);
 if (addr == 32)
  return -EAGAIN;
 /* Selected the phy and isolate the rest */
 for (addr = 0; addr <= PHY_MAX_REG_ADDRESS; addr++) {
  if (addr != adapter->mii.phy_id) {
   pch_gbe_mdio_write(netdev, addr, MII_BMCR,
      BMCR_ISOLATE);
  } else {
   bmcr = pch_gbe_mdio_read(netdev, addr, MII_BMCR);
   pch_gbe_mdio_write(netdev, addr, MII_BMCR,
      bmcr & ~BMCR_ISOLATE);
  }
 }

 /* MII setup */
 adapter->mii.phy_id_mask = 0x1F;
 adapter->mii.reg_num_mask = 0x1F;
 adapter->mii.dev = adapter->netdev;
 adapter->mii.mdio_read = pch_gbe_mdio_read;
 adapter->mii.mdio_write = pch_gbe_mdio_write;
 adapter->mii.supports_gmii = mii_check_gmii_support(&adapter->mii);
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      int pch_gbe_mdio_read(struct net_device *netdev, int addr, int reg)
 * @brief   The read function for mii
 * @param   netdev [INOUT] Network interface device structure
 * @param   addr   [IN] Phy ID
 * @param   reg    [IN] Access location
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
int pch_gbe_mdio_read(struct net_device *netdev, int addr, int reg)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_hw *hw = &adapter->hw;

#ifdef DEBUG_TEST
 PCH_DEBUG("pch_gbe_mdio_read\n");
#endif
 return pch_gbe_hal_ctrl_miim(hw, addr, PCH_GBE_HAL_MIIM_READ,
     reg, (u16) 0);
}

/*!
 * @ingroup Linux driver internal function
 * @fn      void pch_gbe_mdio_write(struct net_device *netdev,
 *                                  int addr, int reg, int data)
 * @brief   TThe write function for mii
 * @param   netdev [INOUT] Network interface device structure
 * @param   addr   [IN] Phy ID (not used)
 * @param   reg    [IN]Access location
 * @param   data   [IN] Write data
 * @return  None
 */
void pch_gbe_mdio_write(struct net_device *netdev, int addr, int reg, int data)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_hw *hw = &adapter->hw;

#ifdef DEBUG_TEST
 PCH_DEBUG("pch_gbe_mdio_write\n");
#endif
 pch_gbe_hal_ctrl_miim(hw, addr, PCH_GBE_HAL_MIIM_WRITE, reg, data);
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_reset_task(struct work_struct *work)
 * @brief   Reset processing at the time of transmission timeout
 * @param   work  [INOUT] Pointer of board private structure
 * @return  None
 */
static void pch_gbe_reset_task(struct work_struct *work)
{
 struct pch_gbe_adapter *adapter;
 adapter = container_of(work, struct pch_gbe_adapter, reset_task);

 DPRINTK(DRV, DEBUG, "\n");

 pch_gbe_reinit_locked(adapter);
}

/*!
 * @ingroup Linux driver internal function
 * @fn      void pch_gbe_reinit_locked(struct pch_gbe_adapter *adapter)
 * @brief   Re-initialization
 * @param   adapter  [INOUT] Board private structure
 * @return  None
 */
void pch_gbe_reinit_locked(struct pch_gbe_adapter *adapter)
{
 DPRINTK(DRV, DEBUG, "\n");

 while ((test_and_set_bit(__PCH_GBE_RESETTING, &adapter->flags)) != 0)
  msleep(1);
 pch_gbe_down(adapter);
 pch_gbe_up(adapter);
 clear_bit(__PCH_GBE_RESETTING, &adapter->flags);
}

/*!
 * @ingroup Linux driver internal function
 * @fn      void pch_gbe_reset(struct pch_gbe_adapter *adapter)
 * @brief   Reset GbE
 * @param   adapter  [INOUT] Board private structure
 * @return  None
 */
void pch_gbe_reset(struct pch_gbe_adapter *adapter)
{
 DPRINTK(DRV, DEBUG, "\n");

 pch_gbe_hal_reset_hw(&adapter->hw);

 if (pch_gbe_hal_init_hw(&adapter->hw) != 0)
  DPRINTK(DRV, ERR, "Hardware Error\n");
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static int pch_gbe_request_irq(struct pch_gbe_adapter *adapter)
 * @brief   Allocate an interrupt line
 * @param   adapter  [INOUT] Board private structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 */
static int pch_gbe_request_irq(struct pch_gbe_adapter *adapter)
{
 struct net_device *netdev = adapter->netdev;
 int err;
 int flags;

 DPRINTK(DRV, DEBUG, "\n");

 flags = IRQF_SHARED;
 adapter->have_msi = FALSE;
 err = pci_enable_msi(adapter->pdev);
 PCH_DEBUG("call pci_enable_msi\n");
 if (err != 0) {
  DPRINTK(DRV, ERR,
   "Unable to allocate MSI interrupt Error: %d\n", err);
 } else {
  flags = 0;
  adapter->have_msi = TRUE;
 }
 err = request_irq(adapter->pdev->irq, &pch_gbe_intr,
     flags, netdev->name, netdev);
 if (err != 0) {
  DPRINTK(DRV, ERR, "Unable to allocate interrupt Error: %d\n",
   err);
 }

 PCH_DEBUG
 ("adapter->have_msi : %d  flags : 0x%04x  return : 0x%04x\n",
 adapter->have_msi, flags, err);
 return err;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_free_irq(struct pch_gbe_adapter *adapter)
 * @brief   Free an interrupt
 * @param   adapter  [INOUT] Board private structure
 * @return  None
 */
static void pch_gbe_free_irq(struct pch_gbe_adapter *adapter)
{
 struct net_device *netdev = adapter->netdev;

 DPRINTK(DRV, DEBUG, "\n");

 free_irq(adapter->pdev->irq, netdev);
 if (adapter->have_msi != 0) {
  pci_disable_msi(adapter->pdev);
  PCH_DEBUG("call pci_disable_msi");
 }
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_irq_disable(struct pch_gbe_adapter *adapter)
 * @brief   Mask off interrupt generation on the NIC
 * @param   adapter  [INOUT] Board private structure
 * @return  None
 */
static void pch_gbe_irq_disable(struct pch_gbe_adapter *adapter)
{
 struct pch_gbe_hw *hw = &adapter->hw;

 DPRINTK(DRV, DEBUG, "\n");

 atomic_inc(&adapter->irq_sem);
 PCH_GBE_WRITE_REG(hw, INT_EN, 0);
 synchronize_irq(adapter->pdev->irq);

 PCH_DEBUG("INT_EN reg : 0x%08x\n", PCH_GBE_READ_REG(hw, INT_EN));
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_irq_enable(struct pch_gbe_adapter *adapter)
 * @brief   Enable default interrupt generation settings
 * @param   adapter  [INOUT] Board private structure
 * @return  None
 */
static void pch_gbe_irq_enable(struct pch_gbe_adapter *adapter)
{
 struct pch_gbe_hw *hw = &adapter->hw;

 DPRINTK(DRV, DEBUG, "\n");

 if (likely(atomic_dec_and_test(&adapter->irq_sem)))
  PCH_GBE_WRITE_REG(hw, INT_EN, PCH_GBE_INT_ENABLE_MASK);
 PCH_DEBUG("INT_EN reg : 0x%08x\n", PCH_GBE_READ_REG(hw, INT_EN));
}

/*!
 * @ingroup Linux driver internal function
 * @fn      int pch_gbe_up(struct pch_gbe_adapter *adapter)
 * @brief   Up GbE network device
 * @param   adapter  [INOUT] Board private structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 */
int pch_gbe_up(struct pch_gbe_adapter *adapter)
{
 struct net_device *netdev = adapter->netdev;
 struct pch_gbe_tx_ring *tx_ring = adapter->tx_ring;
 struct pch_gbe_rx_ring *rx_ring = adapter->rx_ring;
 int err;

 DPRINTK(IFUP, DEBUG, "\n");

 /* hardware has been reset, we need to reload some things */
 pch_gbe_set_multi(netdev);

 pch_gbe_setup_tctl(adapter);
 pch_gbe_configure_tx(adapter);
 pch_gbe_setup_rctl(adapter);
 pch_gbe_configure_rx(adapter);

 err = pch_gbe_request_irq(adapter);
 if (err != 0) {
  PCH_LOG(KERN_ERR, "Error: can't bring device up\n");
  return err;
 }
 pch_gbe_alloc_tx_buffers(adapter, tx_ring);
 pch_gbe_alloc_rx_buffers(adapter, rx_ring, rx_ring->count);
 adapter->tx_queue_len = netdev->tx_queue_len;

 mod_timer(&adapter->watchdog_timer, jiffies);

 napi_enable(&adapter->napi);
 pch_gbe_irq_enable(adapter);
 netif_start_queue(adapter->netdev);

 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      void pch_gbe_down(struct pch_gbe_adapter *adapter)
 * @brief   Down GbE network device
 * @param   adapter  [INOUT] Board private structure
 * @return  None
 */
void pch_gbe_down(struct pch_gbe_adapter *adapter)
{
 struct net_device *netdev = adapter->netdev;

 DPRINTK(IFDOWN, DEBUG, "\n");

 /* signal that we're down so the interrupt handler does not
  * reschedule our watchdog timer */
 napi_disable(&adapter->napi);
 atomic_set(&adapter->irq_sem, 0);

 pch_gbe_irq_disable(adapter);
 pch_gbe_free_irq(adapter);

 del_timer_sync(&adapter->watchdog_timer);

 netdev->tx_queue_len = adapter->tx_queue_len;
 netif_carrier_off(netdev);
 netif_stop_queue(netdev);

 pch_gbe_reset(adapter);
 pch_gbe_clean_tx_ring(adapter, adapter->tx_ring);
 pch_gbe_clean_rx_ring(adapter, adapter->rx_ring);
}

/*!
 * @ingroup Linux driver internal function
 * @fn      int pch_gbe_setup_tx_resources(struct pch_gbe_adapter *adapter,
 *                                         struct pch_gbe_tx_ring *tx_ring)
 * @brief   Allocate Tx resources (Descriptors)
 * @param   adapter  [INOUT] Board private structure
 * @param   tx_ring  [OUT] Tx descriptor ring (for a specific queue) to setup
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 */
int
pch_gbe_setup_tx_resources(struct pch_gbe_adapter *adapter,
      struct pch_gbe_tx_ring *tx_ring)
{
 struct pci_dev *pdev = adapter->pdev;
 struct pch_gbe_tx_desc *tx_desc;
 int size;
 int desNo;

 DPRINTK(DRV, DEBUG, "\n");

 size = (int)sizeof(struct pch_gbe_buffer) * tx_ring->count;
 tx_ring->buffer_info = vmalloc(size);
 if (!tx_ring->buffer_info) {
  DPRINTK(DRV, ERR,
   "Unable to allocate memory "
   "for the buffer infomation\n");
  return -ENOMEM;
 }
 memset(tx_ring->buffer_info, 0, size);

 tx_ring->size = tx_ring->count * (int)sizeof(struct pch_gbe_tx_desc);

 tx_ring->desc =
  pci_alloc_consistent(pdev, tx_ring->size, &tx_ring->dma);
 if (!tx_ring->desc) {
  vfree(tx_ring->buffer_info);
  DPRINTK(DRV, ERR,
   "Unable to allocate memory "
   "for the transmit descriptor ring\n");
  return -ENOMEM;
 }
 memset(tx_ring->desc, 0, tx_ring->size);

 tx_ring->next_to_use = 0;
 tx_ring->next_to_clean = 0;
 spin_lock_init(&tx_ring->tx_lock);

 for (desNo = 0; desNo < tx_ring->count; desNo++) {
  tx_desc = PCH_GBE_TX_DESC(*tx_ring, desNo);
  tx_desc->gbec_status = DSC_INIT16;
 }

#ifdef DEBUG_TEST
 PCH_DEBUG("tx_ring->desc = 0x%08x  tx_ring->dma = 0x%08x\n",
   (u32) tx_ring->desc, tx_ring->dma);
 PCH_DEBUG("next_to_clean = 0x%08x  next_to_use = 0x%08x\n",
   tx_ring->next_to_clean, tx_ring->next_to_use);
#endif
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      int pch_gbe_setup_rx_resources(struct pch_gbe_adapter *adapter,
 *                                         struct pch_gbe_rx_ring *rx_ring)
 * @brief   Allocate Rx resources (Descriptors)
 * @param   adapter  [INOUT] Board private structure
 * @param   rx_ring  [OUT] Rx descriptor ring (for a specific queue) to setup
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 */
int
pch_gbe_setup_rx_resources(struct pch_gbe_adapter *adapter,
    struct pch_gbe_rx_ring *rx_ring)
{
 struct pci_dev *pdev = adapter->pdev;
 struct pch_gbe_rx_desc *rx_desc;
 int size;
 int desNo;

 DPRINTK(DRV, DEBUG, "\n");

 size = (int)sizeof(struct pch_gbe_buffer) * rx_ring->count;
 rx_ring->buffer_info = vmalloc(size);
 if (!rx_ring->buffer_info) {
  DPRINTK(DRV, ERR,
   "Unable to allocate memory "
   "for the receive descriptor ring\n");
  return -ENOMEM;
 }
 memset(rx_ring->buffer_info, 0, size);

 rx_ring->size = rx_ring->count * (int)sizeof(struct pch_gbe_rx_desc);

 rx_ring->desc =
  pci_alloc_consistent(pdev, rx_ring->size, &rx_ring->dma);

 if (!rx_ring->desc) {
  DPRINTK(DRV, ERR,
   "Unable to allocate memory "
   "for the receive descriptor ring\n");
  vfree(rx_ring->buffer_info);
  return -ENOMEM;
 }

 memset(rx_ring->desc, 0, rx_ring->size);

 rx_ring->next_to_clean = 0;
 rx_ring->next_to_use = 0;

 for (desNo = 0; desNo < rx_ring->count; desNo++) {
  rx_desc = PCH_GBE_RX_DESC(*rx_ring, desNo);
  rx_desc->gbec_status = DSC_INIT16;
 }

#ifdef DEBUG_TEST
 PCH_DEBUG("rx_ring->desc = 0x%08x  rx_ring->dma = 0x%08x\n",
   (u32) rx_ring->desc, rx_ring->dma);
 PCH_DEBUG("next_to_clean = 0x%08x  next_to_use = 0x%08x\n",
   rx_ring->next_to_clean, rx_ring->next_to_use);
#endif
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      void pch_gbe_free_tx_resources(struct pch_gbe_adapter *adapter,
 *                                         struct pch_gbe_tx_ring *tx_ring)
 * @brief   Free Tx Resources
 * @param   adapter  [INOUT] Board private structure
 * @param   tx_ring  [OUT] Tx descriptor ring for a specific queue
 * @return  None
 * @remarks
 *    Free all transmit software resources
 */
void
pch_gbe_free_tx_resources(struct pch_gbe_adapter *adapter,
     struct pch_gbe_tx_ring *tx_ring)
{
 struct pci_dev *pdev = adapter->pdev;

 DPRINTK(DRV, DEBUG, "\n");

 pch_gbe_clean_tx_ring(adapter, tx_ring);
 vfree(tx_ring->buffer_info);
 tx_ring->buffer_info = NULL;
 pci_free_consistent(pdev, tx_ring->size, tx_ring->desc, tx_ring->dma);
 tx_ring->desc = NULL;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      void pch_gbe_free_rx_resources(struct pch_gbe_adapter *adapter,
 *                                         struct pch_gbe_rx_ring *rx_ring)
 * @brief   Free Rx Resources
 * @param   adapter  [INOUT] Board private structure
 * @param   rx_ring  [OUT] ring to clean the resources from
 * @return  None
 * @remarks
 *   Free all receive software resources
 */
void
pch_gbe_free_rx_resources(struct pch_gbe_adapter *adapter,
     struct pch_gbe_rx_ring *rx_ring)
{
 struct pci_dev *pdev = adapter->pdev;

 DPRINTK(DRV, DEBUG, "\n");

 pch_gbe_clean_rx_ring(adapter, rx_ring);
 vfree(rx_ring->buffer_info);
 rx_ring->buffer_info = NULL;
 pci_free_consistent(pdev, rx_ring->size, rx_ring->desc, rx_ring->dma);
 rx_ring->desc = NULL;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_setup_tctl(struct pch_gbe_adapter *adapter)
 * @brief   configure the Transmit control registers
 * @param   adapter  [INOUT] Board private structure
 * @return  None
 */
static void pch_gbe_setup_tctl(struct pch_gbe_adapter *adapter)
{
 struct pch_gbe_hw *hw = &adapter->hw;
 u32 tx_mode, tcpip;

 DPRINTK(IFUP, DEBUG, "\n");

 tx_mode = PCH_GBE_TM_LONG_PKT |
  PCH_GBE_TM_ST_AND_FD |
  PCH_GBE_TM_SHORT_PKT |
  PCH_GBE_TM_TH_TX_STRT_8 |
  PCH_GBE_TM_TH_ALM_EMP_4 | PCH_GBE_TM_TH_ALM_FULL_8;

 PCH_GBE_WRITE_REG(hw, TX_MODE, tx_mode);

 tcpip = PCH_GBE_READ_REG(hw, TCPIP_ACC);
 tcpip |= PCH_GBE_TX_TCPIPACC_EN;
 PCH_GBE_WRITE_REG(hw, TCPIP_ACC, tcpip);

#ifdef DEBUG_TEST
 PCH_DEBUG("TX_MODE reg = 0x%08x  TCPIP_ACC reg = 0x%08x\n",
   PCH_GBE_READ_REG(hw, TX_MODE),
   PCH_GBE_READ_REG(hw, TCPIP_ACC));
#endif
 return;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_configure_tx(struct pch_gbe_adapter *adapter)
 * @brief   Configure Transmit Unit after Reset
 * @param   adapter  [INOUT] Board private structure
 * @return  None
 * @remarks
 *   Configure the Tx unit of the MAC after a reset.
 */
static void pch_gbe_configure_tx(struct pch_gbe_adapter *adapter)
{
 struct pch_gbe_hw *hw = &adapter->hw;
 u32 tdba, tdlen, dctrl;

 DPRINTK(IFUP, DEBUG, "\n");
 PCH_DEBUG("dma adr = 0x%08x  size = 0x%08x\n",
   adapter->tx_ring->dma, adapter->tx_ring->size);

 /* Setup the HW Tx Head and Tail descriptor pointers */
 tdba = adapter->tx_ring->dma;
 tdlen = adapter->tx_ring->size - 0x10;
 PCH_GBE_WRITE_REG(hw, TX_DSC_BASE, tdba);
 PCH_GBE_WRITE_REG(hw, TX_DSC_SIZE, tdlen);
 PCH_GBE_WRITE_REG(hw, TX_DSC_SW_P, tdba);

 /* Enables Transmission DMA */
 dctrl = PCH_GBE_READ_REG(hw, DMA_CTRL);
 dctrl |= PCH_GBE_TX_DMA_EN;
 PCH_GBE_WRITE_REG(hw, DMA_CTRL, dctrl);

#ifdef DEBUG_TEST
 PCH_DEBUG
 ("BASE = 0x%08x  HW_P = 0x%08x  SIZE = 0x%08x  SW_P = 0x%08x\n",
 PCH_GBE_READ_REG(hw, TX_DSC_BASE),
 PCH_GBE_READ_REG(hw, TX_DSC_HW_P),
 PCH_GBE_READ_REG(hw, TX_DSC_SIZE),
 PCH_GBE_READ_REG(hw, TX_DSC_SW_P));
 PCH_DEBUG("DMA_CTRL reg[bit0] = 0x%08x\n",
   PCH_GBE_READ_REG(hw, DMA_CTRL));
#endif
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_setup_rctl(struct pch_gbe_adapter *adapter)
 * @brief   Configure the receive control registers
 * @param   adapter  [INOUT] Board private structure
 * @return  None
 * @remarks
 *   Configure the Tx unit of the MAC after a reset.
 */
static void pch_gbe_setup_rctl(struct pch_gbe_adapter *adapter)
{
 struct pch_gbe_hw *hw = &adapter->hw;
 u32 rx_mode, tcpip;

 DPRINTK(IFUP, DEBUG, "\n");

 rx_mode = PCH_GBE_ADD_FIL_EN | PCH_GBE_MLT_FIL_EN |
 PCH_GBE_RH_ALM_EMP_4 | PCH_GBE_RH_ALM_FULL_4 | PCH_GBE_RH_RD_TRG_8;

 PCH_GBE_WRITE_REG(hw, RX_MODE, rx_mode);

 tcpip = PCH_GBE_READ_REG(hw, TCPIP_ACC);

 if (adapter->rx_csum == TRUE) {
  tcpip &= ~PCH_GBE_RX_TCPIPACC_OFF;
  tcpip |= PCH_GBE_RX_TCPIPACC_EN;
 } else {
  tcpip |= PCH_GBE_RX_TCPIPACC_OFF;
  tcpip &= ~PCH_GBE_RX_TCPIPACC_EN;
 }
 PCH_GBE_WRITE_REG(hw, TCPIP_ACC, tcpip);

#ifdef DEBUG_TEST
 PCH_DEBUG("RX_MODE reg = 0x%08x  TCPIP_ACC reg = 0x%08x\n",
   PCH_GBE_READ_REG(hw, RX_MODE),
   PCH_GBE_READ_REG(hw, TCPIP_ACC));
#endif
 return;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_configure_rx(struct pch_gbe_adapter *adapter)
 * @brief   Configure Receive Unit after Reset
 * @param   adapter  [INOUT] Board private structure
 * @return  None
 * @remarks
 *   Configure the Rx unit of the MAC after a reset.
 */
static void pch_gbe_configure_rx(struct pch_gbe_adapter *adapter)
{
 struct pch_gbe_hw *hw = &adapter->hw;
 u32 rdba, rdlen, rctl, rxdma;

 DPRINTK(IFUP, DEBUG, "\n");
 PCH_DEBUG("dma adr = 0x%08x  size = 0x%08x\n",
   adapter->rx_ring->dma, adapter->rx_ring->size);

 pch_gbe_hal_force_mac_fc(hw);

 /* Disables Receive MAC */
 rctl = PCH_GBE_READ_REG(hw, MAC_RX_EN);
 PCH_GBE_WRITE_REG(hw, MAC_RX_EN, (rctl & ~PCH_GBE_MRE_MAC_RX_EN));

 /* Disables Receive DMA */
 rxdma = PCH_GBE_READ_REG(hw, DMA_CTRL);
 rxdma &= ~PCH_GBE_RX_DMA_EN;
 PCH_GBE_WRITE_REG(hw, DMA_CTRL, rxdma);

 PCH_DEBUG("MAC_RX_EN reg = 0x%08x  DMA_CTRL reg = 0x%08x\n",
   PCH_GBE_READ_REG(hw, MAC_RX_EN),
   PCH_GBE_READ_REG(hw, DMA_CTRL));

 /* Setup the HW Rx Head and Tail Descriptor Pointers and
  * the Base and Length of the Rx Descriptor Ring */
 rdba = adapter->rx_ring->dma;
 rdlen = adapter->rx_ring->size - 0x10;
 PCH_GBE_WRITE_REG(hw, RX_DSC_BASE, rdba);
 PCH_GBE_WRITE_REG(hw, RX_DSC_SIZE, rdlen);
 PCH_GBE_WRITE_REG(hw, RX_DSC_SW_P, rdba + rdlen);

 /* Enables Receive DMA */
 rxdma = PCH_GBE_READ_REG(hw, DMA_CTRL);
 rxdma |= PCH_GBE_RX_DMA_EN;
 PCH_GBE_WRITE_REG(hw, DMA_CTRL, rxdma);
 /* Enables Receive */
 PCH_GBE_WRITE_REG(hw, MAC_RX_EN, PCH_GBE_MRE_MAC_RX_EN);

#ifdef DEBUG_TEST
 PCH_DEBUG
 ("BASE = 0x%08x  HW_P = 0x%08x  SIZE = 0x%08x  SW_P = 0x%08x\n",
 PCH_GBE_READ_REG(hw, RX_DSC_BASE),
 PCH_GBE_READ_REG(hw, RX_DSC_HW_P),
 PCH_GBE_READ_REG(hw, RX_DSC_SIZE),
 PCH_GBE_READ_REG(hw, RX_DSC_SW_P));
 PCH_DEBUG("MAC_RX_EN reg = 0x%08x  DMA_CTRL reg = 0x%08x\n",
   PCH_GBE_READ_REG(hw, MAC_RX_EN),
   PCH_GBE_READ_REG(hw, DMA_CTRL));
#endif
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_unmap_and_free_tx_resource(
 *                                          struct pch_gbe_adapter *adapter,
 *                                          struct pch_gbe_buffer *buffer_info)
 * @brief   Unmap and free tx socket buffer
 * @param   adapter     [INOUT] Board private structure
 * @param   buffer_info [OUT] Buffer information structure
 * @return  None
 */
static void
pch_gbe_unmap_and_free_tx_resource(struct pch_gbe_adapter *adapter,
     struct pch_gbe_buffer *buffer_info)
{
#ifdef DEBUG_TEST
 PCH_DEBUG("pch_gbe_unmap_and_free_tx_resource\n");
#endif
 if (buffer_info->dma != 0) {
  pci_unmap_page(adapter->pdev, buffer_info->dma,
    buffer_info->length, PCI_DMA_TODEVICE);
  buffer_info->dma = 0;
 }
 if (buffer_info->skb != 0) {
  dev_kfree_skb_any(buffer_info->skb);
  buffer_info->skb = NULL;
 }
 if (buffer_info->kernel_skb != 0) {
  dev_kfree_skb_any(buffer_info->kernel_skb);
  buffer_info->kernel_skb = NULL;
 }
#ifdef DEBUG_TEST
 PCH_DEBUG
  ("buffer_info->dma : 0x%08x  buffer_info->skb : 0x%08x\n",
  buffer_info->dma, buffer_info->skb);
#endif
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_unmap_and_free_rx_resource(
 *                                          struct pch_gbe_adapter *adapter,
 *                                          struct pch_gbe_buffer *buffer_info)
 * @brief   Unmap and free rx socket buffer
 * @param   adapter      [INOUT] Board private structure
 * @param   buffer_info  [OUT] Buffer information structure
 * @return  None
 */
static void
pch_gbe_unmap_and_free_rx_resource(struct pch_gbe_adapter *adapter,
       struct pch_gbe_buffer *buffer_info)
{
#ifdef DEBUG_TEST
 PCH_DEBUG("pch_gbe_unmap_and_free_rx_resource\n");
#endif
 if (buffer_info->dma != 0) {
  pci_unmap_single(adapter->pdev, buffer_info->dma,
     buffer_info->length, PCI_DMA_FROMDEVICE);
  buffer_info->dma = 0;
 }
 if (buffer_info->skb != 0) {
  dev_kfree_skb_any(buffer_info->skb);
  buffer_info->skb = NULL;
 }
#ifdef DEBUG_TEST
 PCH_DEBUG
  ("buffer_info->dma : 0x%08x  buffer_info->skb : 0x%08x\n",
  buffer_info->dma, buffer_info->skb);
#endif
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_clean_tx_ring(struct pch_gbe_adapter *adapter,
 *                                            struct pch_gbe_tx_ring *tx_ring)
 * @brief   Free Tx Buffers
 * @param   adapter  [INOUT] Board private structure
 * @param   tx_ring  [OUT] Ring to be cleaned
 * @return  None
 */
static void
pch_gbe_clean_tx_ring(struct pch_gbe_adapter *adapter,
   struct pch_gbe_tx_ring *tx_ring)
{
 struct pch_gbe_hw *hw = &adapter->hw;
 struct pch_gbe_buffer *buffer_info;
 unsigned long size;
 unsigned int i;

 DPRINTK(DRV, DEBUG, "\n");

 /* Free all the Tx ring sk_buffs */
 for (i = 0; i < tx_ring->count; i++) {
  buffer_info = &tx_ring->buffer_info[i];
  pch_gbe_unmap_and_free_tx_resource(adapter, buffer_info);
 }
 PCH_DEBUG("call pch_gbe_unmap_and_free_tx_resource() %d count\n",
   i);

 size = (unsigned long)sizeof(struct pch_gbe_buffer) * tx_ring->count;
 memset(tx_ring->buffer_info, 0, size);

 /* Zero out the descriptor ring */
 memset(tx_ring->desc, 0, tx_ring->size);

 tx_ring->next_to_use = 0;
 tx_ring->next_to_clean = 0;

 PCH_GBE_WRITE_REG(hw, TX_DSC_HW_P, tx_ring->dma);
 PCH_GBE_WRITE_REG(hw, TX_DSC_SIZE, (tx_ring->size - 0x10));

#ifdef DEBUG_TEST
 PCH_DEBUG("next_to_use : %d  next_to_clean : %d\n",
   tx_ring->next_to_use, tx_ring->next_to_clean);
 PCH_DEBUG("TX_DSC_HW_P reg : 0x%08x  TX_DSC_SIZE reg : 0x%08x\n",
   PCH_GBE_READ_REG(hw, TX_DSC_HW_P),
   PCH_GBE_READ_REG(hw, TX_DSC_SIZE));
#endif
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_clean_rx_ring(struct pch_gbe_adapter *adapter,
 *                                            struct pch_gbe_rx_ring *rx_ring)
 * @brief   Free Rx Buffers
 * @param   adapter  [INOUT] Board private structure
 * @param   rx_ring  [OUT] Ring to free buffers from
 * @return  None
 */
static void
pch_gbe_clean_rx_ring(struct pch_gbe_adapter *adapter,
	struct pch_gbe_rx_ring *rx_ring)
{
 struct pch_gbe_hw *hw = &adapter->hw;
 struct pch_gbe_buffer *buffer_info;
 unsigned long size;
 unsigned int i;

 DPRINTK(DRV, DEBUG, "\n");

 /* Free all the Rx ring sk_buffs */
 for (i = 0; i < rx_ring->count; i++) {
  buffer_info = &rx_ring->buffer_info[i];
  pch_gbe_unmap_and_free_rx_resource(adapter, buffer_info);
 }
 PCH_DEBUG("call pch_gbe_unmap_and_free_rx_resource() %d count\n",
   i);

 size = (unsigned long)sizeof(struct pch_gbe_buffer) * rx_ring->count;
 memset(rx_ring->buffer_info, 0, size);

 /* Zero out the descriptor ring */
 memset(rx_ring->desc, 0, rx_ring->size);

 rx_ring->next_to_clean = 0;
 rx_ring->next_to_use = 0;

 PCH_GBE_WRITE_REG(hw, RX_DSC_HW_P, rx_ring->dma);
 PCH_GBE_WRITE_REG(hw, RX_DSC_SIZE, (rx_ring->size - 0x10));
#ifdef DEBUG_TEST
 PCH_DEBUG("next_to_use : %d  next_to_clean : %d\n",
   rx_ring->next_to_use, rx_ring->next_to_clean);
 PCH_DEBUG("RX_DSC_HW_P reg : 0x%08x  RX_DSC_SIZE reg : 0x%08x\n",
   PCH_GBE_READ_REG(hw, RX_DSC_HW_P),
   PCH_GBE_READ_REG(hw, RX_DSC_SIZE));
#endif
}

static void
pch_gbe_set_rgmii_ctrl(struct pch_gbe_adapter *adapter, u16 speed, u16 duplex)
{
 struct pch_gbe_hw *hw = &adapter->hw;
 unsigned long rgmii = 0;

 DPRINTK(DRV, DEBUG, "\n");
 /* Set the RGMII control. */
#ifdef PCH_GBE_MAC_IFOP_RGMII
 switch (speed) {
 case SPEED_10:
  rgmii = (PCH_GBE_RGMII_RATE_2_5M |
    PCH_GBE_MAC_RGMII_CTRL_SETTING);
  break;
 case SPEED_100:
  rgmii = (PCH_GBE_RGMII_RATE_25M |
    PCH_GBE_MAC_RGMII_CTRL_SETTING);
  break;
 case SPEED_1000:
  rgmii = (PCH_GBE_RGMII_RATE_125M |
    PCH_GBE_MAC_RGMII_CTRL_SETTING);
  break;
 }
 PCH_GBE_WRITE_REG(hw, RGMII_CTRL, rgmii);
#else /* GMII */
 rgmii = 0;
 PCH_GBE_WRITE_REG(hw, RGMII_CTRL, rgmii);
#endif
}
static void
pch_gbe_set_mode(struct pch_gbe_adapter *adapter, u16 speed, u16 duplex)
{
 struct net_device *netdev = adapter->netdev;
 struct pch_gbe_hw *hw = &adapter->hw;
 unsigned long mode = 0;

 DPRINTK(DRV, DEBUG, "\n");
 /* Set the communication mode */
 switch (speed) {
 case SPEED_10:
  mode = PCH_GBE_MODE_MII_ETHER;
  netdev->tx_queue_len = 10;
  break;
 case SPEED_100:
  mode = PCH_GBE_MODE_MII_ETHER;
  netdev->tx_queue_len = 100;
  break;
 case SPEED_1000:
  mode = PCH_GBE_MODE_GMII_ETHER;
  break;
 }
 if (duplex == DUPLEX_FULL)
  mode |= PCH_GBE_MODE_FULL_DUPLEX;
 else
  mode |= PCH_GBE_MODE_HALF_DUPLEX;
 PCH_GBE_WRITE_REG(hw, MODE, mode);
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_watchdog(unsigned long data)
 * @brief   Watchdog process
 * @param   data  [INOUT] Board private structure
 * @return  None
 */
static void pch_gbe_watchdog(unsigned long data)
{
 struct pch_gbe_adapter *adapter = (struct pch_gbe_adapter *)data;
 struct net_device *netdev = adapter->netdev;
 struct pch_gbe_hw *hw = &adapter->hw;
 struct ethtool_cmd cmd;

 DPRINTK(INTR, DEBUG, "right now = %ld\n", jiffies);

 pch_gbe_update_stats(adapter);
 if ((mii_link_ok(&adapter->mii)) && (!netif_carrier_ok(netdev))) {
  netdev->tx_queue_len = adapter->tx_queue_len;
  /* mii library handles link maintenance tasks */
  if (mii_ethtool_gset(&adapter->mii, &cmd) != 0) {
   DPRINTK(INTR, ERR, "ethtool get setting Error\n");
   mod_timer(&adapter->watchdog_timer,
      round_jiffies(jiffies +
      PCH_GBE_WATCHDOG_PERIOD));
   return;
  }
  hw->mac.link_speed = cmd.speed;
  hw->mac.link_duplex = cmd.duplex;
  /* Set the RGMII control. */
  pch_gbe_set_rgmii_ctrl(adapter, hw->mac.link_speed,
      hw->mac.link_duplex);
  /* Set the communication mode */
  pch_gbe_set_mode(adapter, hw->mac.link_speed,
      hw->mac.link_duplex);
  DPRINTK(INTR, INFO, "NIC Link is Up %d Mbps %s-Duplex\n",
   cmd.speed, cmd.duplex == DUPLEX_FULL ? "Full" : "Half");
  netif_carrier_on(netdev);
  netif_wake_queue(netdev);
 } else if ((!mii_link_ok(&adapter->mii)) &&
     (netif_carrier_ok(netdev))) {
  DPRINTK(INTR, INFO, "NIC Link is Down\n");
  hw->mac.link_speed = SPEED_10;
  hw->mac.link_duplex = DUPLEX_HALF;
  netif_carrier_off(netdev);
  netif_stop_queue(netdev);
 }
 mod_timer(&adapter->watchdog_timer,
    round_jiffies(jiffies + PCH_GBE_WATCHDOG_PERIOD));
#ifdef DEBUG_TEST
 PCH_DEBUG
  ("RGMII_CTRL reg : 0x%08x  RGMII_ST reg : 0x%08x "
  " MODE reg : 0x%08x\n",
  PCH_GBE_READ_REG(hw, RGMII_CTRL),
  PCH_GBE_READ_REG(hw, RGMII_ST),
  PCH_GBE_READ_REG(hw, MODE));
 PCH_DEBUG
  ("link_speed : %d  link_duplex : %d  tx_queue_len : %d\n",
  hw->mac.link_speed, hw->mac.link_duplex,
  (u32) netdev->tx_queue_len);
#endif
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_tx_queue(struct pch_gbe_adapter *adapter,
 *                    struct pch_gbe_tx_ring *tx_ring, struct sk_buff *skb)
 * @brief   Carry out queuing of the transmission data
 * @param   adapter  [INOUT] Board private structure
 * @param   tx_ring  [OUT] Tx descriptor ring structure
 * @param   skb      [IN] Sockt buffer structure
 * @return  None
 */
static void
pch_gbe_tx_queue(struct pch_gbe_adapter *adapter,
   struct pch_gbe_tx_ring *tx_ring, struct sk_buff *skb)
{
 struct pch_gbe_hw *hw = &adapter->hw;
 struct pch_gbe_tx_desc *tx_desc;
 struct pch_gbe_buffer *buffer_info;
 struct sk_buff *tmp_skb;
 unsigned int frame_ctrl;
 unsigned int ring_num;
 unsigned long flags;

 DPRINTK(DRV, DEBUG, "\n");

 /*-- Set frame control --*/
 frame_ctrl = 0;
 if (unlikely(skb->len < PCH_GBE_SHORT_PKT))
  frame_ctrl |= PCH_GBE_TXD_CTRL_APAD;
 if (unlikely(adapter->tx_csum == FALSE))
  frame_ctrl |= PCH_GBE_TXD_CTRL_TCPIP_ACC_OFF;

 /* Performs checksum processing */
 /*
  * It is because the hardware accelerator does not support a checksum,
  * when the received data size is less than 64 bytes.
  */
 if ((skb->len < PCH_GBE_SHORT_PKT) && (adapter->tx_csum == TRUE)) {
  frame_ctrl |=
   PCH_GBE_TXD_CTRL_APAD | PCH_GBE_TXD_CTRL_TCPIP_ACC_OFF;
  if (skb->protocol == htons(ETH_P_IP) && !(ip_hdr(skb)->frag_off & htons(IP_MF | IP_OFFSET))) {
   struct iphdr *iph = ip_hdr(skb);
   unsigned int offset;
   iph->check = 0;
   iph->check = ip_fast_csum((u8 *) iph, iph->ihl);
   offset = skb_transport_offset(skb);
   if (iph->protocol == IPPROTO_TCP) {
    skb->csum = 0;
    tcp_hdr(skb)->check = 0;
    skb->csum =
     skb_checksum(skb, offset,
      skb->len - offset, 0);
    tcp_hdr(skb)->check =
     csum_tcpudp_magic(iph->saddr,
       iph->daddr,
       skb->len - offset,
       IPPROTO_TCP, skb->csum);
   } else if (iph->protocol == IPPROTO_UDP && !(ip_hdr(skb)->frag_off & htons(IP_MF | IP_OFFSET))) {
    skb->csum = 0;
    udp_hdr(skb)->check = 0;
    skb->csum =
     skb_checksum(skb, offset,
      skb->len - offset, 0);
    udp_hdr(skb)->check =
     csum_tcpudp_magic(iph->saddr,
       iph->daddr,
       skb->len - offset,
       IPPROTO_UDP, skb->csum);
   }
  }
 }

 spin_lock_irqsave(&tx_ring->tx_lock, flags);
 ring_num = tx_ring->next_to_use;
 if (unlikely((ring_num + 1) == tx_ring->count))
  tx_ring->next_to_use = 0;
 else
  tx_ring->next_to_use = ring_num + 1;

 spin_unlock_irqrestore(&tx_ring->tx_lock, flags);
 buffer_info = &tx_ring->buffer_info[ring_num];
 tmp_skb = buffer_info->skb;

 /* [Header:14][payload] ---> [Header:14][paddong:2][payload]    */
 memcpy(tmp_skb->data, skb->data, ETH_HLEN);
 tmp_skb->data[ETH_HLEN] = 0x00;
 tmp_skb->data[ETH_HLEN + 1] = 0x00;
 tmp_skb->len = skb->len;
 memcpy(&tmp_skb->data[ETH_HLEN + 2], &skb->data[ETH_HLEN],
	(skb->len - ETH_HLEN));
 buffer_info->kernel_skb = skb;
 skb = tmp_skb;

 /*-- Set Buffer infomation --*/
 buffer_info->length = skb->len;
 buffer_info->dma =
     pci_map_single(adapter->pdev, skb->data, buffer_info->length,
      PCI_DMA_TODEVICE);
 buffer_info->time_stamp = jiffies;

 /*-- Set Tx descriptor --*/
 tx_desc = PCH_GBE_TX_DESC(*tx_ring, ring_num);
 tx_desc->buffer_addr = (buffer_info->dma);
 tx_desc->length = (skb->len);
 tx_desc->tx_words_eob = ((skb->len + 3));
 tx_desc->tx_frame_ctrl = (frame_ctrl);
 tx_desc->gbec_status = (DSC_INIT16);

 if (unlikely(++ring_num == tx_ring->count))
  ring_num = 0;

#ifdef DEBUG_TEST
 {
  unsigned char *rd_data;

  rd_data = (unsigned char *)tx_desc;
  PCH_DEBUG
      ("buffer_info->dma : 0x%08x  skb->len : 0x%08x  "
       "frame_ctrl : 0x%08x\n",
       buffer_info->dma, skb->len, frame_ctrl);
  PCH_DEBUG
      ("tx_desc: \n 0x%02x 0x%02x 0x%02x 0x%02x\n 0x%02x "
       "0x%02x 0x%02x 0x%02x\n 0x%02x 0x%02x 0x%02x 0x%02x\n "
       "0x%02x 0x%02x 0x%02x 0x%02x\n",
       rd_data[0], rd_data[1], rd_data[2], rd_data[3], rd_data[4],
       rd_data[5], rd_data[6], rd_data[7], rd_data[8], rd_data[9],
       rd_data[10], rd_data[11], rd_data[12], rd_data[13],
       rd_data[14], rd_data[15]);
 }
#endif

 /* Update software pointer of TX descriptor */
 PCH_GBE_WRITE_REG(hw, TX_DSC_SW_P,
     tx_ring->dma +
     (int)sizeof(struct pch_gbe_tx_desc) * ring_num);
}

/*!
 * @ingroup Linux driver internal function
 * @fn      void pch_gbe_update_stats(struct pch_gbe_adapter *adapter)
 * @brief   Update the board statistics counters
 * @param   adapter  [INOUT] Board private structure
 * @return  None
 */
void pch_gbe_update_stats(struct pch_gbe_adapter *adapter)
{
 struct pci_dev *pdev = adapter->pdev;
 struct pch_gbe_hw_stats *stats = &adapter->stats;
 struct net_device_stats *net_stats = &adapter->net_stats;
 unsigned long flags;

 DPRINTK(DRV, DEBUG, "\n");
 /*
  * Prevent stats update while adapter is being reset, or if the pci
  * connection is down.
  */
 if ((pdev->error_state) && (pdev->error_state != pci_channel_io_normal))
  return;

 spin_lock_irqsave(&adapter->stats_lock, flags);

 /* Update device status "adapter->stats" */
 stats->rx_errors = stats->rx_crc_errors + stats->rx_frame_errors;
 stats->tx_errors = stats->tx_length_errors +
     stats->tx_aborted_errors +
     stats->tx_carrier_errors + stats->tx_timeout_count;

 /* Update network device status "adapter->net_stats" */
 net_stats->rx_packets = stats->rx_packets;
 net_stats->tx_packets = stats->tx_packets;
 net_stats->rx_bytes = stats->rx_bytes;
 net_stats->tx_bytes = stats->tx_bytes;
 net_stats->rx_errors = stats->rx_errors;
 net_stats->tx_errors = stats->tx_errors;
 net_stats->rx_dropped = stats->rx_dropped;
 net_stats->tx_dropped = stats->tx_dropped;
 net_stats->multicast = stats->multicast;
 net_stats->collisions = stats->collisions;
 net_stats->rx_crc_errors = stats->rx_crc_errors;
 net_stats->rx_frame_errors = stats->rx_frame_errors;
 net_stats->tx_aborted_errors = stats->tx_aborted_errors;
 net_stats->tx_carrier_errors = stats->tx_carrier_errors;

 spin_unlock_irqrestore(&adapter->stats_lock, flags);
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static irqreturn_t pch_gbe_intr(int irq, void *data)
 * @brief   Interrupt Handler
 * @param   irq   [IN] Interrupt number
 * @param   data  [INOUT] Pointer to a network interface device structure
 * @return  None
 */
static irqreturn_t pch_gbe_intr(int irq, void *data)
{
 struct net_device *netdev = data;
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_hw *hw = &adapter->hw;
 u32 int_st;
 u32 int_en;

 DPRINTK(INTR, DEBUG, "\n");

 /* Check request status */
 int_st = PCH_GBE_READ_REG(hw, INT_ST);
 PCH_DEBUG("int_st = 0x%08x\n", int_st);

 int_st = int_st & PCH_GBE_READ_REG(hw, INT_EN);
 /* When request status is no interruption factor */
 if (unlikely(!int_st)) {
  /* End processing. */
  PCH_DEBUG("return = 0x%08x\n", IRQ_NONE);
  return IRQ_NONE; /* Not our interrupt */
 }
 if (int_st & PCH_GBE_INT_RX_FRAME_ERR)
  adapter->stats.intr_rx_frame_err_count++;
 if (int_st & PCH_GBE_INT_RX_FIFO_ERR)
  adapter->stats.intr_rx_fifo_err_count++;
 if (int_st & PCH_GBE_INT_RX_DMA_ERR)
  adapter->stats.intr_rx_dma_err_count++;
 if (int_st & PCH_GBE_INT_TX_FIFO_ERR)
  adapter->stats.intr_tx_fifo_err_count++;
 if (int_st & PCH_GBE_INT_TX_DMA_ERR)
  adapter->stats.intr_tx_dma_err_count++;
 if (int_st & PCH_GBE_INT_TCPIP_ERR)
  adapter->stats.intr_tcpip_err_count++;
 /* When Rx descriptor is empty  */
 if ((int_st & PCH_GBE_INT_RX_DSC_EMP) != 0) {
  adapter->stats.intr_rx_dsc_empty_count++;
  DPRINTK(INTR, ERR, "Rx descriptor is empty\n");
  int_en = PCH_GBE_READ_REG(hw, INT_EN);
  PCH_GBE_WRITE_REG(hw, INT_EN,
      (int_en & ~PCH_GBE_INT_RX_DSC_EMP));
  if (hw->mac.tx_fc_enable == TRUE) {
   /* Set Pause packet */
   pch_gbe_hal_set_pause_packet(hw);
  }
  if ((int_en & (PCH_GBE_INT_RX_DMA_CMPLT | PCH_GBE_INT_TX_CMPLT))
      == 0) {
   return IRQ_HANDLED;
  }
 }

 /* When request status is Receive interruption */
 if ((int_st & (PCH_GBE_INT_RX_DMA_CMPLT | PCH_GBE_INT_TX_CMPLT)) != 0) {
  if (likely(napi_schedule_prep(&adapter->napi))) {
   /* Enable only Rx Descriptor empty */
   atomic_inc(&adapter->irq_sem);
   int_en = PCH_GBE_READ_REG(hw, INT_EN);
   int_en &=
       ~(PCH_GBE_INT_RX_DMA_CMPLT | PCH_GBE_INT_TX_CMPLT);
   PCH_GBE_WRITE_REG(hw, INT_EN, int_en);
   /* Start polling for NAPI */
   __napi_schedule(&adapter->napi);
  }
 }
 PCH_DEBUG("return = 0x%08x  INT_EN reg = 0x%08x\n",
   IRQ_HANDLED, PCH_GBE_READ_REG(hw, INT_EN));
 return IRQ_HANDLED;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static unsigned char pch_gbe_clean_tx(struct pch_gbe_adapter *adapter,
 *                                            struct pch_gbe_tx_ring *tx_ring)
 * @brief   Reclaim resources after transmit completes
 * @param   adapter   [INOUT] Board private structure
 * @param   tx_ring   [OUT] Tx descriptor ring
 * @return  TRUE:  Cleaned the descriptor
 * @return  FALSE: Not cleaned the descriptor
 */
static unsigned char
pch_gbe_clean_tx(struct pch_gbe_adapter *adapter,
   struct pch_gbe_tx_ring *tx_ring)
{
 struct pch_gbe_tx_desc *tx_desc;
 struct pch_gbe_buffer *buffer_info;
 struct sk_buff *skb;
 unsigned int i;
 unsigned int cleaned_count = 0;
 unsigned char cleaned = FALSE;

 DPRINTK(DRV, DEBUG, "\n");
 PCH_DEBUG("next_to_clean : %d\n", tx_ring->next_to_clean);

 i = tx_ring->next_to_clean;
 tx_desc = PCH_GBE_TX_DESC(*tx_ring, i);
 PCH_DEBUG("gbec_status:0x%04x  dma_status:0x%04x\n",
   tx_desc->gbec_status, tx_desc->dma_status);

 while ((tx_desc->gbec_status & DSC_INIT16) == 0x0000) {
  PCH_DEBUG("gbec_status:0x%04x\n", tx_desc->gbec_status);
  cleaned = TRUE;
  buffer_info = &tx_ring->buffer_info[i];
  skb = buffer_info->skb;

  if ((tx_desc->gbec_status & PCH_GBE_TXD_GMAC_STAT_ABT) != 0) {
   adapter->stats.tx_aborted_errors++;
   DPRINTK(DRV, ERR, "Transfer Aboat Error\n");
  } else if ((tx_desc->gbec_status & PCH_GBE_TXD_GMAC_STAT_CRSER)
      != 0) {
   adapter->stats.tx_carrier_errors++;
   DPRINTK(DRV, ERR, "Transfer Carrier Sense Error\n");
  } else if ((tx_desc->gbec_status & PCH_GBE_TXD_GMAC_STAT_EXCOL)
      != 0) {
   adapter->stats.tx_aborted_errors++;
   DPRINTK(DRV, ERR, "Transfer Collision Abort Error\n");
  } else if ((tx_desc->gbec_status &
	(PCH_GBE_TXD_GMAC_STAT_SNGCOL |
	PCH_GBE_TXD_GMAC_STAT_MLTCOL)) != 0) {
   adapter->stats.collisions++;
   adapter->stats.tx_packets++;
   adapter->stats.tx_bytes += skb->len;
   DPRINTK(DRV, DEBUG, "Transfer Collision\n");
  } else if ((tx_desc->gbec_status & PCH_GBE_TXD_GMAC_STAT_CMPLT)
      != 0) {
   adapter->stats.tx_packets++;
   adapter->stats.tx_bytes += skb->len;
  }
  if (buffer_info->dma != 0) {
   PCH_DEBUG("unmap buffer_info->dma : %d\n", i);
   pci_unmap_page(adapter->pdev, buffer_info->dma,
	buffer_info->length, PCI_DMA_TODEVICE);
   buffer_info->dma = 0;
  }
  if (buffer_info->skb != 0) {
   PCH_DEBUG("trim buffer_info->skb : %d\n", i);
   skb_trim(buffer_info->skb, 0);
  }
  if (buffer_info->kernel_skb != 0) {
   PCH_DEBUG
	("free buffer_info->kernel_skb adr: 0x%x\n",
	(u32)(buffer_info->kernel_skb));
   dev_kfree_skb(buffer_info->kernel_skb);
   buffer_info->kernel_skb = NULL;
  }
  tx_desc->gbec_status = DSC_INIT16;
  if (unlikely(++i == tx_ring->count))
   i = 0;
  tx_desc = PCH_GBE_TX_DESC(*tx_ring, i);

  /* weight of a sort for tx, to avoid endless transmit cleanup */
  if (cleaned_count++ == PCH_GBE_TX_WEIGHT)
   break;
 }
 PCH_DEBUG("called pch_gbe_unmap_and_free_tx_resource() %dcount\n",
   cleaned_count);
 /* Recover from running out of Tx resources in xmit_frame */
 if (unlikely(cleaned && (netif_queue_stopped(adapter->netdev)))) {
  netif_wake_queue(adapter->netdev);
  adapter->stats.tx_restart_count++;
  DPRINTK(DRV, DEBUG, "Tx wake queue\n");
 }
 spin_lock(&adapter->tx_queue_lock);
 tx_ring->next_to_clean = i;
 spin_unlock(&adapter->tx_queue_lock);
 PCH_DEBUG("next_to_clean : %d\n", tx_ring->next_to_clean);
 return cleaned;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static unsigned char pch_gbe_clean_rx(struct pch_gbe_adapter *adapter,
 *                                            struct pch_gbe_rx_ring *rx_ring,
 *                                            int *work_done, int work_to_do)
 * @brief   Send received data up the network stack; legacy
 * @param   adapter     [INOUT] Board private structure
 * @param   rx_ring     [OUT] Rx descriptor ring
 * @param   work_done   [OUT] Completed count
 * @param   work_to_do  [IN] Request count
 * @return  TRUE:  Cleaned the descriptor
 * @return  FALSE: Not cleaned the descriptor
 */
static unsigned char
pch_gbe_clean_rx(struct pch_gbe_adapter *adapter,
   struct pch_gbe_rx_ring *rx_ring,
   int *work_done, int work_to_do)
{
 struct net_device *netdev = adapter->netdev;
 struct pci_dev *pdev = adapter->pdev;
 struct pch_gbe_buffer *buffer_info;
 struct pch_gbe_rx_desc *rx_desc;
 u32 length;
 unsigned char tmp_packet[ETH_HLEN];
 unsigned int i;
 unsigned int cleaned_count = 0;
 unsigned char cleaned = FALSE;
 struct sk_buff *skb;
 u8 dma_status;
 u16 gbec_status;
 u32 tcp_ip_status;
 u8 skb_copy_flag = 0;
 u8 skb_padding_flag = 0;

 DPRINTK(DRV, DEBUG, "\n");

 i = rx_ring->next_to_clean;

 while (*work_done < work_to_do) {
  /* Check Rx descriptor status */
  rx_desc = PCH_GBE_RX_DESC(*rx_ring, i);
  if (rx_desc->gbec_status == DSC_INIT16)
   break;
  cleaned = TRUE;
  cleaned_count++;

  dma_status = rx_desc->dma_status;
  gbec_status = rx_desc->gbec_status;
  tcp_ip_status = rx_desc->tcp_ip_status;
  rx_desc->gbec_status = DSC_INIT16;
  buffer_info = &rx_ring->buffer_info[i];
  skb = buffer_info->skb;

  /* unmap dma */
  pci_unmap_single(pdev, buffer_info->dma, buffer_info->length,
     PCI_DMA_FROMDEVICE);
  buffer_info->dma = 0;
  /* Prefetch the packet */
  prefetch(skb->data);

  PCH_DEBUG
   ("RxDecNo = 0x%04x  Status[DMA:0x%02x GBE:0x%04x "
    "TCP:0x%08x]  BufInf = 0x%08x\n",
    i, dma_status, gbec_status, tcp_ip_status,
    (u32) (buffer_info));
  /* Error check */
  if (unlikely(gbec_status & PCH_GBE_RXD_GMAC_STAT_NOTOCTAL)) {
   adapter->stats.rx_frame_errors++;
   DPRINTK(DRV, ERR, "Receive Not Octal Error\n");
  } else if (unlikely(gbec_status &
    PCH_GBE_RXD_GMAC_STAT_NBLERR)) {
   adapter->stats.rx_frame_errors++;
   DPRINTK(DRV, ERR, "Receive Nibble Error\n");
  } else if (unlikely(gbec_status &
    PCH_GBE_RXD_GMAC_STAT_CRCERR)) {
   adapter->stats.rx_crc_errors++;
   DPRINTK(DRV, ERR, "Receive CRC Error\n");
  } else {
   /* get receive length */
   /* length convert[-3], padding[-2] */
   length = (rx_desc->rx_words_eob) - 3 - 2;

   /* Decide the data conversion method */
   if (adapter->rx_csum != TRUE) {
    /* [Header:14][payload] */
    skb_padding_flag = 0;
    skb_copy_flag = 1;
   } else {
    /* [Header:14][padding:2][payload] */
    skb_padding_flag = 1;
    if (length < copybreak)
     skb_copy_flag = 1;
    else
     skb_copy_flag = 0;
   }

   /* Data conversion */
   if (skb_copy_flag != 0) { /* recycle  skb */
    struct sk_buff *new_skb;
    new_skb =
	netdev_alloc_skb(netdev,
	length + NET_IP_ALIGN);
    if (new_skb != 0) {
     if (!skb_padding_flag) {
      skb_reserve(new_skb,
	NET_IP_ALIGN);
     }
     memcpy(new_skb->data, skb->data,
      length);
     /* save the skb
      * in buffer_info as good */
     skb = new_skb;
    } else if (!skb_padding_flag) {
     /* dorrop error */
     DPRINTK(DRV, ERR,
      "New skb allocation Error\n");
     goto dorrop;
    }
   } else {
    buffer_info->skb = NULL;
   }
   if (skb_padding_flag != 0) {
    memcpy(&tmp_packet[0], &skb->data[0], ETH_HLEN);
    memcpy(&skb->data[NET_IP_ALIGN], &tmp_packet[0],
     ETH_HLEN);
    skb_reserve(skb, NET_IP_ALIGN);

   }

   /* update status of driver */
   adapter->stats.rx_bytes += length;
   adapter->stats.rx_packets++;
   if ((gbec_status & PCH_GBE_RXD_GMAC_STAT_MARMLT) != 0)
    adapter->stats.multicast++;
   /* Write meta date of skb */
   skb_put(skb, length);
   skb->protocol = eth_type_trans(skb, netdev);
   if ((tcp_ip_status & PCH_GBE_RXD_ACC_STAT_TCPIPOK) ==
       PCH_GBE_RXD_ACC_STAT_TCPIPOK) {
    skb->ip_summed = CHECKSUM_UNNECESSARY;
   } else {
    skb->ip_summed = CHECKSUM_NONE;
   }

   if (netif_receive_skb(skb) == NET_RX_DROP) {
    adapter->stats.rx_dropped++;
    DPRINTK(DRV, ERR,
     "Receive Netif Receive Dropped Error\n");
   }
   (*work_done)++;
   netdev->last_rx = jiffies;
   PCH_DEBUG
       ("Receive skb->ip_summed: %d length: %d\n",
	skb->ip_summed, length);
  }
dorrop:
  /* return some buffers to hardware, one at a time is too slow */
  if (unlikely(cleaned_count >= PCH_GBE_RX_BUFFER_WRITE)) {
   pch_gbe_alloc_rx_buffers(adapter, rx_ring,
       cleaned_count);
   cleaned_count = 0;
  }
  if (++i == rx_ring->count)
   i = 0;
 }
 rx_ring->next_to_clean = i;
 if (cleaned_count != 0)
  pch_gbe_alloc_rx_buffers(adapter, rx_ring, cleaned_count);
#ifdef DEBUG_TEST
 {
  u32 tmp1, tmp2, tmp3, tmp4;
  struct pch_gbe_hw *hw = &adapter->hw;

  PCH_DEBUG
   ("cleaned_count = %d  next_to_clean = %d  "
    "next_to_use = %d\n",
    cleaned_count, rx_ring->next_to_clean,
    rx_ring->next_to_use);
  tmp1 = PCH_GBE_READ_REG(hw, RX_DSC_BASE);
  tmp2 = PCH_GBE_READ_REG(hw, RX_DSC_HW_P);
  tmp3 = PCH_GBE_READ_REG(hw, RX_DSC_SIZE);
  tmp4 = PCH_GBE_READ_REG(hw, RX_DSC_SW_P);
  PCH_DEBUG
   ("BASE = 0x%08x  HW_P = 0x%08x  "
    "SIZE = 0x%08x  SW_P = 0x%08x\n",
    tmp1, tmp2, tmp3, tmp4);
  tmp1 = PCH_GBE_READ_REG(hw, DMA_CTRL);
  tmp2 = PCH_GBE_READ_REG(hw, MAC_RX_EN);
  PCH_DEBUG("DMA_CTRL = 0x%08x  MAC_RX_EN = 0x%08x\n", tmp1,
    tmp2);
  tmp1 = PCH_GBE_READ_REG(hw, RX_MODE);
  tmp2 = PCH_GBE_READ_REG(hw, ADDR_MASK);
  tmp3 = PCH_GBE_READ_REG(hw, MAC_ADR1A);
  tmp4 = PCH_GBE_READ_REG(hw, MAC_ADR1B);
  PCH_DEBUG
   ("RX_MODE = 0x%08x  ADDR_MASK = 0x%08x  "
    "MAC_ADR1A = 0x%08x  MAC_ADR1B = 0x%08x\n",
    tmp1, tmp2, tmp3, tmp4);
 }
#endif
 return cleaned;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_alloc_rx_buffers(
 *                      struct pch_gbe_adapter *adapter,
 *                      struct pch_gbe_rx_ring *rx_ring, int cleaned_count)
 * @brief   Replace used receive buffers; legacy & extended
 * @param   adapter       [INOUT] Board private structure
 * @param   rx_ring       [OUT] Rx descriptor ring
 * @param   cleaned_count [IN] Cleaned count
 * @return  None
 */
static void
pch_gbe_alloc_rx_buffers(struct pch_gbe_adapter *adapter,
    struct pch_gbe_rx_ring *rx_ring, int cleaned_count)
{
 struct net_device *netdev = adapter->netdev;
 struct pci_dev *pdev = adapter->pdev;
 struct pch_gbe_hw *hw = &adapter->hw;
 struct pch_gbe_rx_desc *rx_desc;
 struct pch_gbe_buffer *buffer_info;
 struct sk_buff *skb;
 unsigned int i;
 unsigned int bufsz;

 DPRINTK(DRV, DEBUG, "\n");

 bufsz = adapter->rx_buffer_len + PCH_GBE_DMA_ALIGN;
 i = rx_ring->next_to_use;

 while ((cleaned_count--) != 0) {
  buffer_info = &rx_ring->buffer_info[i];
  skb = buffer_info->skb;
  if (skb != 0) {
   skb_trim(skb, 0);
  } else {
   skb = netdev_alloc_skb(netdev, bufsz);
   if (unlikely(!skb)) {
    /* Better luck next round */
    adapter->stats.rx_alloc_buff_failed++;
    break;
   }
   /* 64byte align */
   skb_reserve(skb, PCH_GBE_DMA_ALIGN);

   buffer_info->skb = skb;
   buffer_info->length = adapter->rx_buffer_len;
  }

  buffer_info->dma = pci_map_single(pdev,
	skb->data,
	buffer_info->length,
	PCI_DMA_FROMDEVICE);

  rx_desc = PCH_GBE_RX_DESC(*rx_ring, i);
  rx_desc->buffer_addr = (buffer_info->dma);
  rx_desc->gbec_status = DSC_INIT16;

  PCH_DEBUG("i = %d  buffer_info->dma = 0x%x  "
    "buffer_info->length = 0x%x\n",
    i, buffer_info->dma, buffer_info->length);

  if (unlikely(++i == rx_ring->count))
   i = 0;
 }
 if (likely(rx_ring->next_to_use != i)) {
  rx_ring->next_to_use = i;
  if (unlikely(i-- == 0))
   i = (rx_ring->count - 1);
  wmb();
  PCH_GBE_WRITE_REG(hw, RX_DSC_SW_P,
      rx_ring->dma +
      (int)sizeof(struct pch_gbe_rx_desc) * i);
 }
 return;
}

/*!
 * @ingroup Linux driver internal function
 * @fn      static void pch_gbe_alloc_tx_buffers(
 *                                   struct pch_gbe_adapter *adapter,
 *                                   struct pch_gbe_tx_ring *tx_ring)
 * @brief   Allocate transmit buffers
 * @param   adapter   [INOUT] Board private structure
 * @param   tx_ring   [OUT] Tx descriptor ring
 * @return  None
 */
static void
pch_gbe_alloc_tx_buffers(struct pch_gbe_adapter *adapter,
    struct pch_gbe_tx_ring *tx_ring)
{
 struct pch_gbe_buffer *buffer_info;
 struct sk_buff *skb;
 unsigned int i;
 unsigned int bufsz;
 struct pch_gbe_tx_desc *tx_desc;

 DPRINTK(DRV, DEBUG, "\n");

 bufsz =
     adapter->hw.mac.max_frame_size + PCH_GBE_DMA_ALIGN + NET_IP_ALIGN;

 for (i = 0; i < tx_ring->count; i++) {
  buffer_info = &tx_ring->buffer_info[i];
  skb = netdev_alloc_skb(adapter->netdev, bufsz);
  skb_reserve(skb, PCH_GBE_DMA_ALIGN);
  buffer_info->skb = skb;
  tx_desc = PCH_GBE_TX_DESC(*tx_ring, i);
  tx_desc->gbec_status = (DSC_INIT16);
 }

 return;
}
/* pch_gbe_main.c */
