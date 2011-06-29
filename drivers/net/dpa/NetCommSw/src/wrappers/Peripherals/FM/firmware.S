	.section .firmware,"a"
	.globl p4080_fman_firmware
p4080_fman_firmware:
	.incbin "../linux/drivers/net/dpa/NetCommSw/src/wrappers/Peripherals/FM/fsl_fman_ucode_p4080_91_1.bin"
