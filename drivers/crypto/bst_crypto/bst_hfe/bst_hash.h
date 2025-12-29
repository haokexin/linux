/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2024 Black Sesame Technologies. Inc.
 */

#ifndef __BST_HASH_H__
#define __BST_HASH_H__

#define HASH_DMA_FUNCTION

#define HFE_CTRL 0x00		   // 1 哈希控制寄存器 RW 0x0
#define HFE_CFG 0x04		   // 9 哈希配置寄存器 RW 0x0
#define HFE_RISR 0x10		   // 2 哈希中断源寄存器 W0C 0x0
#define HFE_IMCR 0x14		   // 2 哈希中断使能寄存器 RW 0x0
#define HFE_MISR 0x18		   // 2 哈希中断输出寄存器 RO 0x0
#define HFE_MSG_LEN 0x30	   // – 0x3C 32[3] 消息总长度寄存器 RW 0x0
#define HFE_MSG_CNT 0x40	   // – 0x4C 32[3] 已处理消息长度计数器RW 0x0
#define HFE_KEY_LEN 0x60	   // 32 HMAC 密钥长度寄存器RW 0x0
#define HFE_KEY_CNT 0x70	   // 32 已处理 HMAC 密钥长度计数器RW 0x0
#define HFE_MDIN_CR 0xB0	   // 2 数据标志寄存器 RW 0x0
#define HFE_MDIN 0xC0		   // 32 消息输入寄存器 WO 0x0
#define HFE_VERSION 0xFC	   // 24 版本寄存器 RO 0xXXE4_0010
#define HFE_IN 0x100		   // – 0x1C4 32[3] 哈希值输入寄存器 WO 0x0
#define HFE_OUT 0x200		   // – 0x2C4 32[3] 哈希值输出寄存器 RO 0x0DMA 寄存器
#define HFE_DMA_L_SADDR 0x490  // – 0x490 32 DMA 源地址Low寄存器 RW 0x0
#define HFE_DMA_H_SADDR 0x494  // – 0x494 32 DMA 源地址High寄存器 RW 0x0
#define HFE_DMA_L_DADDR 0x498  // – 0x498 32 DMA 目的地址Low寄存器 RW 0x0
#define HFE_DMA_H_DADDR 0x49C  // – 0x49C 32 DMA 目的地址High寄存器 RW 0x0
#define HFE_DMA_RLEN 0x4A0	   // 32 DMA 读数据长度寄存 RW 0x0
#define HFE_DMA_WLEN 0x4A4	   // 32 DMA 写数据长度寄存器RW 0x0
#define HFE_DMA_AWCC 0x4A8	   // 32 DMA 写通道控制信息寄存器RW 0x0
#define HFE_DMA_ARCC 0x4AC	   // 32 DMA 读通道控制信息寄存器RW 0x0

// some register offset
#define HASH_HMAC_OFFSET (4)
#define HASH_HMAC_SECURE_PORT_OFFSET (5)
#define HASH_REVERSE_BYTE_ORDER_IN_WORD_OFFSET (8)
#define HASH_UPDATE_CONFIG_OFFSET (12)
#define HASH_DMA_OFFSET (16)
#define HASH_LAST_BLOCK_OFFSET (16)

// HASH max length
#define HASH_DIGEST_MAX_WORD_LEN (16)
#define HASH_BLOCK_MAX_WORD_LEN (32)
#define HASH_BLOCK_MAX_BYTE_LEN (HASH_BLOCK_MAX_WORD_LEN << 2)
#define HASH_ITERATOR_MAX_WORD_LEN (50)
#define HASH_TOTAL_LEN_MAX_WORD_LEN (4)

// HASH algorithm definition
enum BST_HASH_ALG {
	HASH_SM3 = 0,
	HASH_MD5 = 1,
	HASH_SHA256 = 2,
	HASH_SHA384 = 3,
	HASH_SHA512 = 4,
	HASH_SHA1 = 5,
	HASH_SHA224 = 6,
	HASH_SHA512_224 = 7,
	HASH_SHA512_256 = 8,
	// HASH_SHA3_224                 = 9,
	// HASH_SHA3_256                 = 10,
	// HASH_SHA3_384                 = 11,
	// HASH_SHA3_512                 = 12,
};

// HASH return code
enum HASH_RET_CODE {
	HASH_SUCCESS = 0,
	HASH_BUFFER_NULL,
	HASH_CONFIG_INVALID,
	HASH_INPUT_INVALID,
	HASH_LEN_OVERFLOW,
	HASH_ERROR,
};

//hash callback function type
typedef void (*HASH_CALLBACK)(void);

// to calculate hash or hmac
enum HFE_MODE {
	HASH_MODE,
	HMAC_MODE
};
// to calculate hash or hmac
enum HFE_STATE {
	HEF_SET_KEY_DONE,
	HEF_UPDATE_DONE,
	HEF_FINAL_DONE,
};

// HASH status
struct hash_status {
	uint32_t busy : 1; // calculate busy flag
};

struct bst_hash_ctx {
	uint8_t hash_buffer[HASH_BLOCK_MAX_BYTE_LEN]; // block buffer
	uint32_t total[HASH_TOTAL_LEN_MAX_WORD_LEN];  // total byte length of the whole message
	enum BST_HASH_ALG hash_alg;					  // current hash algorithm
	enum HFE_MODE hfe_mode;						  // the input message is for hash algorithm or for hmac algorithm
	uint32_t block_byte_len;
	uint8_t iterator_word_len;
	uint32_t digest_byte_len;
	struct hash_status status; // hash update status, .busy=1 means doing閿涳拷=0 means idle
	uint8_t first_update_flag; // whether first time to update message(1:yes, 0:no)
	uint8_t finish_flag;	   // whether the whole message has been inputted(1:yes, 0:no)
	void __iomem *base;
	uint8_t inited; // whether inited 0: not inited 1: inited
	/* for hmac*/
	struct mutex key_lock;
	uint8_t *keySrc;
	uint32_t keySrc_len;
	uint32_t key[HASH_BLOCK_MAX_WORD_LEN];
	uint32_t key_len;
	uint32_t key_len_flag;
};

#ifdef HASH_DMA_FUNCTION
// HASH DMA context
struct dma_alloc_addr {
	uint8_t *virt_in;
	uint8_t *virt_out;
	dma_addr_t phys_in;
	dma_addr_t phys_out;
	uint32_t alloc_size[2];
};

struct bst_hash_dma_ctx {
	uint8_t hash_buffer[HASH_BLOCK_MAX_BYTE_LEN]; // block buffer
	uint32_t total[HASH_TOTAL_LEN_MAX_WORD_LEN]; // total byte length of the whole message
	enum BST_HASH_ALG hash_alg;					  // current hash algorithm
	enum HFE_MODE hfe_mode;						  // the input message is for hash algorithm or for hmac algorithm
	uint32_t block_byte_len;
	uint8_t block_word_len;
	uint8_t iterator_word_len;
	uint32_t digest_byte_len;
	struct hash_status status;
	uint8_t first_update_flag; // whether first time to update message(1:yes, 0:no)
	uint8_t finish_flag;	   // whether the whole message has been inputted(1:yes, 0:no)
	uint32_t msg_bytes;
	HASH_CALLBACK callback;
	uint32_t *remainder_msg;
	uint32_t remainder_bytes;
	uint32_t block_words;
	struct dma_alloc_addr dma_addr;
	void __iomem *base;
	uint8_t inited; // whether inited 0: not inited 1: inited
	/* for hmac*/
	struct mutex key_lock;
	uint8_t *keySrc;
	uint32_t keySrc_len;
	uint32_t key[HASH_BLOCK_MAX_WORD_LEN];
	uint32_t key_len;
	uint32_t key_len_flag;
};

extern void uint32_clear(uint32_t *a, uint32_t word_len);

#endif

#endif //__BST_HASH_H__
