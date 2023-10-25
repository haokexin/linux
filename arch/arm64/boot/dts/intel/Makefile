# SPDX-License-Identifier: GPL-2.0-only
dtb-$(CONFIG_ARCH_INTEL_SOCFPGA) += socfpga_agilex_n6000.dtb \
				socfpga_agilex_socdk.dtb \
				socfpga_agilex_socdk_nand.dtb \
				socfpga_n5x_socdk.dtb
dtb-$(CONFIG_ARCH_KEEMBAY) += keembay-evm.dtb
dtb-$(CONFIG_ARCH_AXXIA) += \
	axm5604-sim.dtb axm5608-sim.dtb axm5616-sim.dtb \
	axm5606-emu.dtb \
	axm5616-victoria.dtb \
	axc6704-sim.dtb axc6716-sim.dtb axc6732-sim.dtb \
	axc6704-emu.dtb axc6712-emu.dtb \
	axc6704-waco.dtb axc6732-waco.dtb

always-y	:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
