#
# OTG infrastructure and transceiver drivers
#

# infrastructure
obj-$(CONFIG_USB_OTG_UTILS)	+= otg.o

# transceiver drivers
obj-$(CONFIG_USB_GPIO_VBUS)	+= gpio_vbus.o
obj-$(CONFIG_ISP1301_OMAP)	+= isp1301_omap.o
obj-$(CONFIG_TWL4030_USB)	+= twl4030-usb.o
obj-$(CONFIG_NOP_USB_XCEIV)	+= nop-usb-xceiv.o
obj-$(CONFIG_USB_ULPI)		+= ulpi.o
fsl_usb2_otg-objs		:= fsl_otg.o otg_fsm.o
obj-$(CONFIG_FSL_USB2_OTG)	+= fsl_usb2_otg.o

ccflags-$(CONFIG_USB_DEBUG)	+= -DDEBUG
ccflags-$(CONFIG_USB_GADGET_DEBUG) += -DDEBUG

ifneq ($(CONFIG_USB_CNS3XXX_OTG_HCD_ONLY),)
EXTRA_CFLAGS    += -DDWC_HOST_ONLY
endif

ifneq ($(CONFIG_USB_CNS3XXX_OTG_PCD_ONLY),)
EXTRA_CFLAGS    += -DDWC_DEVICE_ONLY
endif

#obj-$(CONFIG_USB_CNS3XXX_OTG)	+= dwc_otg.o

obj-$(CONFIG_USB_CNS3XXX_OTG)	+= dwc_otg_driver.o
obj-$(CONFIG_USB_CNS3XXX_OTG)	+= dwc_otg_cil.o dwc_otg_cil_intr.o
obj-$(CONFIG_USB_CNS3XXX_OTG)	+= dwc_otg_pcd.o dwc_otg_pcd_intr.o
obj-$(CONFIG_USB_CNS3XXX_OTG)	+= dwc_otg_hcd.o dwc_otg_hcd_intr.o
obj-$(CONFIG_USB_CNS3XXX_OTG)	+= dwc_otg_hcd_queue.o
