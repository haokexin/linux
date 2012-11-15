#
# Makefile for the Freescale Ethernet controllers
#
EXTRA_CFLAGS           += -DVERSION=\"\"
#
#Include netcomm SW specific definitions
include $(srctree)/drivers/net/ethernet/freescale/dpa/NetCommSw/ncsw_config.mk

EXTRA_CFLAGS += -I$(NET_DPA)

#Netcomm SW tree
obj-$(CONFIG_FSL_FMAN) += NetCommSw/
obj-$(CONFIG_FSL_DPA_1588) += dpaa_1588.o
obj-$(CONFIG_DPAA_ETH_SG_SUPPORT) += fsl-dpa-sg.o
obj-$(CONFIG_DPA) += fsl-mac.o fsl-dpa.o
obj-$(CONFIG_DPA_OFFLINE_PORTS) += fsl-oh.o

fsl-dpa-objs	:= dpa-ethtool.o dpaa_eth.o xgmac_mdio.o memac_mdio.o
fsl-dpa-sg-objs	:= dpaa_eth_sg.o
fsl-mac-objs	:= mac.o mac-api.o
fsl-oh-objs	:= offline_port.o
