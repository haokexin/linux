/* DWC OTG (On The Go) defines */

#ifndef __LINUX_DWC_OTG_H
#define __LINUX_DWC_OTG_H

/*
 * The following parameters may be specified when starting the module. These
 * parameters define how the DWC_otg controller should be configured.  Parameter
 * values are passed to the CIL initialization function dwc_otg_cil_init.
 */
struct core_params {
	/*
	 * Specifies the OTG capabilities. The driver will automatically
	 * detect the value for this parameter if none is specified.
	 * 0 - HNP and SRP capable (default)
	 * 1 - SRP Only capable
	 * 2 - No HNP/SRP capable
	 */
	int otg_cap;
#define DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE		0
#define DWC_OTG_CAP_PARAM_SRP_ONLY_CAPABLE		1
#define DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE		2

#define dwc_param_otg_cap_default	DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE

	/*
	 * Specifies whether to use slave or DMA mode for accessing the data
	 * FIFOs. The driver will automatically detect the value for this
	 * parameter if none is specified.
	 * 0 - Slave
	 * 1 - DMA (default, if available)
	 */
	int dma_enable;
#ifdef CONFIG_DWC_SLAVE
#define dwc_param_dma_enable_default			0
#else
#define dwc_param_dma_enable_default			1
#endif

	/*
	 * The DMA Burst size (applicable only for External DMA Mode).
	 * 1, 4, 8 16, 32, 64, 128, 256 (default 32)
	 */
	int dma_burst_size;	/* Translate this to GAHBCFG values */
#define dwc_param_dma_burst_size_default		32

	/*
	 * Specifies the maximum speed of operation in host and device mode.
	 * The actual speed depends on the speed of the attached device and
	 * the value of phy_type. The actual speed depends on the speed of the
	 * attached device.
	 *      0 - High Speed (default)
	 *      1 - Full Speed
	 */
	int speed;
#define dwc_param_speed_default				0
#define DWC_SPEED_PARAM_HIGH				0
#define DWC_SPEED_PARAM_FULL				1

	/*
	 * Specifies whether low power mode is supported when attached to a Full
	 * Speed or Low Speed device in host mode.
	 *      0 - Don't support low power mode (default)
	 *      1 - Support low power mode
	 */
	int host_support_fs_ls_low_power;
#define dwc_param_host_support_fs_ls_low_power_default	0

	/*
	 * Specifies the PHY clock rate in low power mode when connected to a
	 * Low Speed device in host mode. This parameter is applicable only if
	 * HOST_SUPPORT_FS_LS_LOW_POWER is enabled. If PHY_TYPE is set to FS
	 * then defaults to 6 MHZ otherwise 48 MHZ.
	 *
	 *      0 - 48 MHz
	 *      1 - 6 MHz
	 */
	int host_ls_low_power_phy_clk;
#define dwc_param_host_ls_low_power_phy_clk_default	0
#define DWC_HOST_LS_LOW_POWER_PHY_CLK_PARAM_48MHZ	0
#define DWC_HOST_LS_LOW_POWER_PHY_CLK_PARAM_6MHZ	1

	/*
	 * 0 - Use cC FIFO size parameters
	 * 1 - Allow dynamic FIFO sizing (default)
	 */
	int enable_dynamic_fifo;
#define dwc_param_enable_dynamic_fifo_default		1

	/*
	 * Number of 4-byte words in the Rx FIFO in device mode when dynamic
	 * FIFO sizing is enabled.  16 to 32768 (default 1064)
	 */
	int dev_rx_fifo_size;
#define dwc_param_dev_rx_fifo_size_default		1064

	/*
	 * Number of 4-byte words in the non-periodic Tx FIFO in device mode
	 * when dynamic FIFO sizing is enabled.  16 to 32768 (default 1024)
	 */
	int dev_nperio_tx_fifo_size;
#define dwc_param_dev_nperio_tx_fifo_size_default	1024

	/*
	 * Number of 4-byte words in each of the periodic Tx FIFOs in device
	 * mode when dynamic FIFO sizing is enabled.  4 to 768 (default 256)
	 */
#define MAX_TX_FIFOS			15	/* Max periodic FIFOs */
	u32 dev_perio_tx_fifo_size[MAX_TX_FIFOS];
#define dwc_param_dev_perio_tx_fifo_size_default	256

	/*
	 * Number of 4-byte words in the Rx FIFO in host mode when dynamic
	 * FIFO sizing is enabled.  16 to 32768 (default 1024)
	 */
	int host_rx_fifo_size;
#define dwc_param_host_rx_fifo_size_default		1024

	/*
	 * Number of 4-byte words in the non-periodic Tx FIFO in host mode
	 * when Dynamic FIFO sizing is enabled in the core.  16 to 32768
	 * (default 1024)
	 */
	int host_nperio_tx_fifo_size;
#define dwc_param_host_nperio_tx_fifo_size_default	1024

	/*
	   Number of 4-byte words in the host periodic Tx FIFO when dynamic
	   * FIFO sizing is enabled.  16 to 32768 (default 1024)
	 */
	int host_perio_tx_fifo_size;
#define dwc_param_host_perio_tx_fifo_size_default	1024

	/*
	 * The maximum transfer size supported in bytes. 2047 to 65,535
	 * (default 65,535)
	 */
	int max_transfer_size;
#define dwc_param_max_transfer_size_default		65535

	/*
	 * The maximum number of packets in a transfer. 15 to 511  (default 511)
	 */
	int max_packet_count;
#define dwc_param_max_packet_count_default		511

	/*
	 * The number of host channel registers to use.
	 * 1 to 16 (default 12)
	 * Note: The FPGA configuration supports a maximum of 12 host channels.
	 */
	int host_channels;
#define dwc_param_host_channels_default			12

	/*
	 * The number of endpoints in addition to EP0 available for device
	 * mode operations.
	 * 1 to 15 (default 6 IN and OUT)
	 * Note: The FPGA configuration supports a maximum of 6 IN and OUT
	 * endpoints in addition to EP0.
	 */
	int dev_endpoints;
#define dwc_param_dev_endpoints_default			6

	/*
	 * Specifies the type of PHY interface to use. By default, the driver
	 * will automatically detect the phy_type.
	 *
	 *      0 - Full Speed PHY
	 *      1 - UTMI+ (default)
	 *      2 - ULPI
	 */
	int phy_type;
#define DWC_PHY_TYPE_PARAM_FS			0
#define DWC_PHY_TYPE_PARAM_UTMI			1
#define DWC_PHY_TYPE_PARAM_ULPI			2
#define dwc_param_phy_type_default		DWC_PHY_TYPE_PARAM_UTMI

	/*
	 * Specifies the UTMI+ Data Width.  This parameter is applicable for a
	 * PHY_TYPE of UTMI+ or ULPI. (For a ULPI PHY_TYPE, this parameter
	 * indicates the data width between the MAC and the ULPI Wrapper.) Also,
	 * this parameter is applicable only if the OTG_HSPHY_WIDTH cC parameter
	 * was set to "8 and 16 bits", meaning that the core has been configured
	 * to work at either data path width.
	 *
	 * 8 or 16 bits (default 16)
	 */
	int phy_utmi_width;
#define dwc_param_phy_utmi_width_default	16

	/*
	 * Specifies whether the ULPI operates at double or single
	 * data rate. This parameter is only applicable if PHY_TYPE is
	 * ULPI.
	 *
	 *      0 - single data rate ULPI interface with 8 bit wide data
	 *              bus (default)
	 *      1 - double data rate ULPI interface with 4 bit wide data
	 *              bus
	 */
	int phy_ulpi_ddr;
#define dwc_param_phy_ulpi_ddr_default		0

	/*
	 * Specifies whether to use the internal or external supply to
	 * drive the vbus with a ULPI phy.
	 */
	int phy_ulpi_ext_vbus;
#define DWC_PHY_ULPI_INTERNAL_VBUS		0
#define DWC_PHY_ULPI_EXTERNAL_VBUS		1
#define dwc_param_phy_ulpi_ext_vbus_default	DWC_PHY_ULPI_INTERNAL_VBUS

	/*
	 * Specifies whether to use the I2Cinterface for full speed PHY. This
	 * parameter is only applicable if PHY_TYPE is FS.
	 *      0 - No (default)
	 *      1 - Yes
	 */
	int i2c_enable;
#define dwc_param_i2c_enable_default		0

	int ulpi_fs_ls;
#define dwc_param_ulpi_fs_ls_default		0

	int ts_dline;
#define dwc_param_ts_dline_default		0

	/*
	 * Specifies whether dedicated transmit FIFOs are enabled for non
	 * periodic IN endpoints in device mode
	 *      0 - No
	 *      1 - Yes
	 */
	int en_multiple_tx_fifo;
#define dwc_param_en_multiple_tx_fifo_default	1

	/*
	 * Number of 4-byte words in each of the Tx FIFOs in device
	 * mode when dynamic FIFO sizing is enabled. 4 to 768 (default 256)
	 */
	u32 dev_tx_fifo_size[MAX_TX_FIFOS];
	u32 fifo_number;
#define dwc_param_dev_tx_fifo_size_default	256

	/*
	 * Thresholding enable flag
	 *      bit 0 - enable non-ISO Tx thresholding
	 *      bit 1 - enable ISO Tx thresholding
	 *      bit 2 - enable Rx thresholding
	 */
	u32 thr_ctl;
#define dwc_param_thr_ctl_default		0

	/* Thresholding length for Tx FIFOs in 32 bit DWORDs */
	u32 tx_thr_length;
#define dwc_param_tx_thr_length_default		64

	/* Thresholding length for Rx FIFOs in 32 bit DWORDs */
	u32 rx_thr_length;
#define dwc_param_rx_thr_length_default		64

};


/**
 * struct dwc_otg_plat_data - device.platform_data for dwc otg driver.
 */
struct dwc_otg_plat_data {
	int (*phy_init)(void);
	int (*param_init)(struct core_params *);
};

#endif /* __LINUX_DWC_OTG_H */
