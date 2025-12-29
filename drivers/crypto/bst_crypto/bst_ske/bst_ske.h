/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2024 Black Sesame Technologies. Inc.
 */

#ifndef __BST_SKE_H__
#define __BST_SKE_H__

#define SKE_HP_REVERSE_BYTE_ORDER_IN_WORD_OFFSET (24)
#define SKE_HP_MODE_OFFSET (28)
#define SKE_HP_CRYPTO_OFFSET (11)
// #define SKE_HP_SECURE_PORT_OFFSET                    (17)
#define SKE_HP_UP_CFG_OFFSET (12)
#define SKE_HP_DMA_OFFSET (16)
#define SKE_HP_DMA_LL_OFFSET (17)
#define SKE_HP_LAST_DATA_OFFSET (16)

#define SKE_CTRL 0x00	   // 2 SKE 控制寄存器 W1S 0x0
#define SKE_CFG 0x04	   // 18 SKE 配置寄存器 RW 0x0
#define SKE_SR 0x08		   // 3 SKE 状态寄存器 RO 0x0001_0001
#define SKE_RISR 0x0C	   // 3 SKE 中断源寄存器 W0C 0x0
#define SKE_IMCR 0x10	   // 3 SKE 中断使能寄存器 RW 0x0
#define SKE_MISR 0x14	   // 3 SKE 中断输出寄存器 RO 0x0
#define SKE_SP 0x1C		   // 1 SKE 安全端口配置寄存器 RW 0x0
#define SKE_KEY1 0x20	   // – 0x3C 32 SKE 密钥 1 寄存器 RW 0x0
#define SKE_KEY2 0x40	   // – 0x5C 32 SKE 密钥 2 寄存器 RW 0x0
#define SKE_A_LEN_L 0x60   // – 0x64 32 SKE AAD 长度寄存器 RW 0x0
#define SKE_A_LEN_H 0x64   // – 0x64 32 SKE AAD 长度寄存器 RW 0x0
#define SKE_C_LEN_L 0x68   // – 0x6C 32 SKE 密文长度寄存器 RW 0x0
#define SKE_C_LEN_H 0x6c   // – 0x6C 32 SKE 密文长度寄存器 RW 0x0
#define SKE_IV 0x70		   // – 0x7C 32 SKE 初始向量寄存器 RW 0x0
#define SKE_DIN_CR 0x80	   // 9 SKE 数据标志寄存器 RW 0x0
#define SKE_DIN 0x90	   // – 0x9C 32 SKE 数据输入寄存器 RW 0x0
#define SKE_DOUT 0xB0	   // – 0xBC 32 SKE 数据输出寄存器 RO 0x0
#define SKE_VERSION 0xFC   // 24 SKE 版本寄存器 RO 0xXX7E_0020
#define SKE_SEED 0x100	   // – 0x18C 32 SKE 随机数初始种子 RW 0x0
#define SKE_ALARM 0x190	   // 1 SKE 警报寄存器 RO 0x0
#define SKE_DMA_CR 0x300   // 5 DMA 控制寄存器 RW 0x0
#define SKE_DMA_SR 0x304   // 1 DMA 状态寄存器 W0C 0x0
#define SKE_DMA_TO 0x308   // 16 DMA 超时阈值寄存器 RW 0x0
#define SKE_DMA_L_SADDR 0x310   // 32 DMA 源地址Low寄存器 RW 0x0
#define SKE_DMA_H_SADDR 0x314   // 32 DMA 源地址High寄存器 RW 0x0
#define SKE_DMA_L_DADDR 0x320   // 32 DMA 目的地址Low寄存器 RW 0x0
#define SKE_DMA_H_DADDR 0x324   // 32 DMA 目的地址High寄存器 RW 0x0
#define SKE_DMA_RLEN 0x330 // 32 DMA 读数据长度寄存器 RW 0x0
#define SKE_DMA_WLEN 0x334 // 32 DMA 写数据长度寄存器 RW 0x0
#define SKE_DMA_AWCC 0x340 // 19 DMA 写地址通道控制寄存器 RW 0x0
#define SKE_DMA_ARCC 0x344 // 19 DMA 读地址通道控制寄存器 RW 0x0
#define SKE_DMA_LLP 0x348  // – 0x34C 32 DMA 链表地址寄存器 RW 0x0
#define SKE_DMA_OST 0x350  // 32 DMA 读写最大outstanding 寄存器RW

enum ske_mode {
	SKE_MODE_BYPASS = 0, // BYPASS Mode
	SKE_MODE_ECB = 1,	 // ECB Mode
	SKE_MODE_XTS = 2,	 // XTS Mode
	SKE_MODE_CBC = 3,	 // CBC Mode
	SKE_MODE_CFB = 4,	 // CFB Mode
	SKE_MODE_OFB = 5,	 // OFB Mode
	SKE_MODE_CTR = 6,	 // CTR Mode
	SKE_MODE_CMAC = 7,	 // CMAC Mode
	SKE_MODE_CBC_MAC = 8,
	SKE_MODE_GCM = 9,	// GCM Mode
	SKE_MODE_CCM = 10,	// CCM Mode
	SKE_MODE_GMAC = 18, // GMAC Mode
};
// SKE Crypto Action
enum ske_crypto {
	SKE_CRYPTO_ENCRYPT = 0, // encrypt
	SKE_CRYPTO_DECRYPT,		// decrypt
};
//SKE Mac Action
enum ske_mac {
	SKE_GENERATE_MAC = SKE_CRYPTO_ENCRYPT, // generate
	SKE_VERIFY_MAC = SKE_CRYPTO_DECRYPT,   // verify
};

enum ske_alg {
	SKE_ALG_DES = 0,		  // DES
	SKE_ALG_TDES,
	SKE_ALG_AES,
	SKE_ALG_TDES_128,	  // TDES 128 bits key
	SKE_ALG_TDES_192,	  // TDES 192 bits key
	SKE_ALG_TDES_EEE_128, // TDES_EEE 128 bits key
	SKE_ALG_TDES_EEE_192, // TDES_EEE 192 bits key
	SKE_ALG_AES_128,	  // AES 128 bits key
	SKE_ALG_AES_192,	  // AES 192 bits key
	SKE_ALG_AES_256,	  // AES 256 bits key
	SKE_ALG_SM4,		  // SM4
};

//hash callback function type
typedef void (*SKE_CALLBACK)(void);

enum ske_hp_mode {
	SKE_HP_CPU_MODE = 0,
	SKE_HP_DMA_MODE = 1,
};

// SKE return code
enum SKE_RET_CODE {
	SKE_SUCCESS = 0,
	SKE_BUFFER_NULL,
	SKE_CONFIG_INVALID,
	SKE_INPUT_INVALID,
	SKE_ATTACK_ALARM,
	SKE_ERROR,
};

// SKE padding scheme
enum ske_padding {
	SKE_NO_PADDING,
	SKE_ZERO_PADDING,
};

// SKE calc wait mode
enum ske_wait_mode {
	WAIT_TILL_EXPAND_KEY_DONE = 0, // wait till ske_hp expanding key is done
	WAIT_TILL_COULD_INPUT,		   // wait till ske_hp is waiting to input
	WAIT_TILL_OUTPUT_READY,		   // wait till ske_hp output is ready
	WAIT_TILL_CALC_DONE			   // wait till ske_hp calculating is done
} ;

struct dma_alloc_addr {
	uint8_t *virt_in;
	uint8_t *virt_out;
	dma_addr_t phys_in;
	dma_addr_t phys_out;
	uint32_t alloc_size;
};

// SKE alg infos
const char *ske_alg_info[] = {
	"des",		 
	"tdes_128",	 
	"tdes_192",	 
	"tdes_eee_128",
	"tdes_eee_192",
	"aes_128",	 
	"aes_192",	 
	"aes_256",	 
	"sm4",		 
};
#endif //__BST_SKE_H__
