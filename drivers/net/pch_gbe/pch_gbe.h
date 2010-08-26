/*!
 * @file pch_gbe.h
 * @brief Linux PCH Gigabit Ethernet Driver main header file
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

#ifndef _PCH_GBE_H_
#define _PCH_GBE_H_

struct pch_gbe_adapter;

#define PFX "pch_gbe: "
#define DPRINTK(nlevel, klevel, fmt, args...) \
 do { \
  if (NETIF_MSG_##nlevel & adapter->msg_enable) \
   printk(KERN_##klevel PFX "%s: %s: " fmt, \
    adapter->netdev->name, __func__ , ## args); \
 } while (0)

/* only works for sizes that are powers of 2 */
#define PCH_GBE_ROUNDUP(i, size) ((i) = (((i) + (size) - 1) & ~((size) - 1)))

/*!
 * @ingroup Gigabit Ether driver Layer
 * @struct  pch_gbe_buffer
 * @brief   Buffer infomation
 * @remarks
 *  wrapper around a pointer to a socket buffer,
 *  so a DMA handle can be stored along with the buffer
 */
struct pch_gbe_buffer {
 struct sk_buff *skb;     /**< pointer to a socket buffer  */
 struct sk_buff *kernel_skb;
  /**< pointer to a socket buffer received from the kernel */
 dma_addr_t dma;      /**< DMA address  */
 unsigned long time_stamp;   /**< time stamp  */
 u16 length;      /**< data size  */
};

/*!
 * @ingroup Gigabit Ether driver Layer
 * @struct  pch_gbe_tx_ring
 * @brief   tx ring infomation
 */
struct pch_gbe_tx_ring {
 void *desc;  /**< pointer to the descriptor ring memory  */
 dma_addr_t dma;  /**< physical address of the descriptor ring  */
 unsigned int size; /**< length of descriptor ring in bytes  */
 unsigned int count; /**< number of descriptors in the ring  */
 unsigned int next_to_use;
   /**< next descriptor to associate a buffer with  */
 unsigned int next_to_clean;
   /**< next descriptor to check for DD status bit  */
 struct pch_gbe_buffer *buffer_info;
   /**< array of buffer information structs  */
 spinlock_t tx_lock; /**< spinlock structs  */
};

/*!
 * @ingroup Gigabit Ether driver Layer
 * @struct  pch_gbe_rx_ring
 * @brief   rx ring infomation
 */
struct pch_gbe_rx_ring {
 void *desc;  /**< pointer to the descriptor ring memory  */
 dma_addr_t dma;  /**< physical address of the descriptor ring  */
 unsigned int size; /**< length of descriptor ring in bytes  */
 unsigned int count; /**< number of descriptors in the ring  */
 unsigned int next_to_use;
   /**< next descriptor to associate a buffer with  */
 unsigned int next_to_clean;
   /**< next descriptor to check for DD status bit  */
 struct pch_gbe_buffer *buffer_info;
   /**< array of buffer information structs  */
};

/*!
 * @ingroup Gigabit Ether driver Layer
 * @struct  pch_gbe_hw_stats
 * @brief   Statistics counters collected by the MAC
 */
struct pch_gbe_hw_stats {
 u64 rx_packets;      /**< total packets received  */
 u64 tx_packets;      /**< total packets transmitted  */
 u64 rx_bytes;      /**< total bytes received  */
 u64 tx_bytes;      /**< total bytes transmitted  */
 u64 rx_errors;      /**< bad packets received  */
 u64 tx_errors;      /**< packet transmit problems  */
 u64 rx_dropped;      /**< no space in Linux buffers  */
 u64 tx_dropped;      /**< no space available in Linux  */
 u64 multicast;      /**< multicast packets received  */
 u64 collisions;      /**< collisions */
 u64 rx_crc_errors;     /**< received packet with crc error  */
 u64 rx_frame_errors;     /**< received frame alignment error  */
 u64 rx_alloc_buff_failed;   /**< allocate failure of a receive buffer */
 u64 tx_length_errors;     /**< transmit length error  */
 u64 tx_aborted_errors;     /**< transmit aborted error  */
 u64 tx_carrier_errors;     /**< transmit carrier error  */
 u64 tx_timeout_count;     /**< Number of transmit timeout  */
 u64 tx_restart_count;     /**< Number of transmit restert  */
 u64 intr_rx_dsc_empty_count;
   /**< Interrupt count of receive descriptor empty  */
 u64 intr_rx_frame_err_count;
   /**< Interrupt count of receive frame error  */
 u64 intr_rx_fifo_err_count;
   /**< Interrupt count of receive FIFO error  */
 u64 intr_rx_dma_err_count;
   /**< Interrupt count of receive DMA error  */
 u64 intr_tx_fifo_err_count;
   /**< Interrupt count of transmit FIFO error  */
 u64 intr_tx_dma_err_count;
   /**< Interrupt count of transmit DMA error  */
 u64 intr_tcpip_err_count;
   /**< Interrupt count of TCP/IP Accelerator  */
};

/*!
 * @ingroup Gigabit Ether driver Layer
 * @struct  pch_gbe_adapter
 * @brief   board specific private data structure
 */
struct pch_gbe_adapter {
 /* OS defined structs */
 struct net_device *netdev;  /**<  Pointer of network device structure */
 struct pci_dev *pdev;     /**<  Pointer of pci device structure */
 struct net_device_stats net_stats; /**<  Network status */
 struct net_device *polling_netdev;
   /**<  Pointer of polling network device structure */
 struct napi_struct napi; /**<  NAPI structure */

 /* structs defined in pch_gbe_hw.h */
 struct pch_gbe_hw hw;  /**<  Pointer of hardware structure */
 struct pch_gbe_hw_stats stats; /**<  Hardware status */
 struct work_struct reset_task; /**<  Reset task */
 struct mii_if_info mii;  /**<  MII information structure */
 struct timer_list watchdog_timer; /**<  Watchdog timer list */

 u32 bd_number;  /**<  The number of the found NIC cards */
 u32 wake_up_evt; /**<  Wake up event */
 u32 *config_space; /**<  Configuration space */
 int msg_enable;  /**<  Driver message level */

 spinlock_t stats_lock;     /**<  Spinlock structure for status */
 spinlock_t tx_queue_lock;   /**<  Spinlock structure for transmit */
 spinlock_t int_en_lock;     /**<  Spinlock structure for IRQ enable */
 atomic_t irq_sem;     /**<  Semaphore for interrupt */

 struct timer_list blink_timer; /**<  LED blink timer list */
 unsigned long led_status; /**<  LED status */

 /* TX,RX */
 struct pch_gbe_tx_ring *tx_ring;
  /**<  Pointer of Tx descriptor ring structure  */
 struct pch_gbe_rx_ring *rx_ring;
  /**<  Pointer of Rx descriptor ring structure */
 unsigned long rx_buffer_len; /**<  Receive buffer length */
 unsigned long tx_queue_len; /**<  Transmit queue length */

 unsigned char rx_csum;
  /**<  Receive TCP/IP checksum enable/disable */
 unsigned char tx_csum;
  /**<  Transmit TCP/IP checksum enable/disable */

 unsigned char have_msi;  /**<  PCI MSI mode flag */

 /* to not mess up cache alignment, always add to the bottom */
 unsigned long flags;  /**<  Driver status flag */
};

/*!
 * @ingroup Gigabit Ether driver Layer
 * @def  pch_gbe_state_t
 * @brief   Driver Status
 */
enum pch_gbe_state_t {
 __PCH_GBE_TESTING,    /**<  Testing  */
 __PCH_GBE_RESETTING,    /**<  Reseting */
};

/* pch_gbe_main.c */
int pch_gbe_up(struct pch_gbe_adapter *adapter);
void pch_gbe_down(struct pch_gbe_adapter *adapter);
void pch_gbe_reinit_locked(struct pch_gbe_adapter *adapter);
void pch_gbe_reset(struct pch_gbe_adapter *adapter);
int pch_gbe_setup_tx_resources(struct pch_gbe_adapter *adapter,
    struct pch_gbe_tx_ring *txdr);
int pch_gbe_setup_rx_resources(struct pch_gbe_adapter *adapter,
    struct pch_gbe_rx_ring *rxdr);
void pch_gbe_free_tx_resources(struct pch_gbe_adapter *adapter,
    struct pch_gbe_tx_ring *tx_ring);
void pch_gbe_free_rx_resources(struct pch_gbe_adapter *adapter,
    struct pch_gbe_rx_ring *rx_ring);
void pch_gbe_update_stats(struct pch_gbe_adapter *adapter);
int pch_gbe_mdio_read(struct net_device *netdev, int addr, int reg);
void pch_gbe_mdio_write(struct net_device *netdev, int addr, int reg, int data);
/* pch_gbe_param.c */
void pch_gbe_check_options(struct pch_gbe_adapter *adapter);

/* pch_gbe_ethtool.c */
void pch_gbe_set_ethtool_ops(struct net_device *netdev);


#endif /* _PCH_GBE_H_ */
