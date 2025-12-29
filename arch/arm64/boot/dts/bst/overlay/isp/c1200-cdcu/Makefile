dtbos		:= $(patsubst %.dts,%.dtbo,$(notdir $(wildcard $(srctree)/$(obj)/*.dts)))

dtb-$(CONFIG_ARCH_BSTC1200) += $(dtbos)

DTC_FLAGS	+= -@ -Wno-graph_endpoint -Wno-graph_port -Wno-unit_address_vs_reg
DTC_INCLUDE	+= $(objtree)/include
