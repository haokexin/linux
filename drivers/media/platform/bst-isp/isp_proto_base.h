/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

/******************************************************************************
 * @brief        This is a brief description.
 *               +----------------------+
 *               |  FBUF partition      |
 *               +----------------------+
 *
 *               +----------------------+
 *               |  SLAB partition      |
 *               +----------------------+
 *               |  CMDP partition      |
 *               +----------------------+
 *               |  INIT partition      |
 *        0x0000 +----------------------+
 *
 *      INIT partition:
 *               +----------------------+
 *               |  RODA1               |
 *               +----------------------+
 *               |  RODA0               |
 *               +----------------------+
 *               |  DATA                |
 *               +----------------------+
 *               |  EP                  |
 *               +----------------------+
 *               |  FBUF info           |
 *               +----------------------+
 *               |  SLAB info           |
 *               +----------------------+
 *               |  Control             |
 *        0x0000 +----------------------+
 */

#ifndef __BST_COREIP_PROTO_H__
#define __BST_COREIP_PROTO_H__

#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"

#define API_VERSION    0x0037
#define DOC_VERSION    0x0037
#define SW_VERSION     'A'

#define SONE_MIN(X, Y) ((X) > (Y) ? (Y) : (X))
#define SONE_MAX(X, Y) ((X) > (Y) ? (X) : (Y))

enum {
	DRV_CH_INDEX   = 0,
	FW_CH_INDEX    = 1,
	SONE_MAX_CHNUM = 2
};

// list table used by CMDP payload(in/out), SLAB, FBUF
/* ******************************************************
 * circular memory buffer control algorithm:
 *   name[4]  == 0x00000000: can be replace
 *   *header, *tail, *cur as circular element point
 */

struct generic_link {
	uint32_t length;
	uint32_t addr;
};

#define SONE_MAX_FBUF_TABNUM 8

//-----------INIT--------------------------
struct parti_init_control {
	char	 magic_sone[4]; //"SONE"
	uint16_t api_version;
	uint16_t doc_version;
	uint32_t base_addr;
	uint32_t reserve0[5];

	char	 magic_init[4]; //"INIT"
	unsigned init_roda0_offset_div32 : 8;
	unsigned rodata_targ_addr_div4	 : 16;
	unsigned fbuf_num		 : 3;
	unsigned base_addr_high		 : 3; // bus address >32bits, here is bits [33:31]
	unsigned load_mediainfo_disable	 : 1; // disable mediainfo loading in INIT
	unsigned trace_print_mode	 : 1; // 0: print by F/W(image-tool), 1: print by APU
	uint32_t reserve1[3];
	uint32_t slab_base;
	uint32_t cmdp_base[SONE_MAX_CHNUM];
	uint32_t fbuf_table[SONE_MAX_FBUF_TABNUM];
};

#define SONE_MAX_ENTITY_NUM 32
#define SONE_MEDIAINFO_SIZE 4096
#define SONE_MEDIA_RESV	    (SONE_MEDIAINFO_SIZE - 4 - SONE_MAX_ENTITY_NUM * sizeof(struct sone_media_entity))

struct media_v_pad {
	unsigned width	    : 16;
	unsigned height	    : 16;
	unsigned format	    : 8;
	unsigned frame_rate : 8;
	unsigned resv	    : 16;
	uint32_t frmbuf_addr;
};

struct sone_media_entity {
	char		    name[4];
	unsigned	    srcpad_num	: 4; // max: 16 pad
	unsigned	    sinkpad_num : 4; // max: 16 pad
	struct media_v_pad  srcpad0;
	struct media_v_pad  sinkpad0;

	struct generic_link entity_property_addr;  // entity extension reserve: SLAB addr
	struct generic_link srcpad_property_addr;  // souce pad extension reserve: SLAB addr
	struct generic_link sinkpad_property_addr; // sink pad extension reserve: SLAB addr
	struct generic_link resv_info_addr;	   // SLAB addr
};

struct parti_init_mediainfo {
	unsigned		 media_entity_num : 8;
	unsigned		 resv		  : 24;
	struct sone_media_entity entity[SONE_MAX_ENTITY_NUM];
	uint8_t			 resv2[SONE_MEDIA_RESV];
};

#define PARTI_INIT_SYSCFG_LEN 512
#define CORE_VENDOR_DESC_LEN  64
#define CORE_GLOBAL_CFG_LEN   64

struct core_vendor_desc {
	char	 corp_name[16];
	char	 IP_name[16];
	char	 short_name;
	uint32_t resv[(CORE_VENDOR_DESC_LEN - 16 - 16 - 4) / 4];
};

struct core_global_cfg {
	char	 platform[4];
	char	 host_env[4];
	uint32_t trace_mask;
	uint32_t ipc_reg_base;
	uint32_t vsp_reg_base;
	uint32_t isp_reg_base;
	uint32_t resv[(CORE_GLOBAL_CFG_LEN - 4 * 6) / 4];
};

struct parti_init_syscfg {
	struct core_vendor_desc vendor;
	struct core_global_cfg	corecfg;
	uint32_t		resv[(PARTI_INIT_SYSCFG_LEN - 128) / 4];
};

#define SONE_ENTITY_MAX_NUM 32
#define SONE_RODA0_MAXLEN   1024

typedef struct sPROTOCOL_INIT {
	struct parti_init_control   ctrl;
	struct parti_init_mediainfo mediainfo[SONE_ENTITY_MAX_NUM];

	struct parti_init_syscfg    cfg;
	uint32_t		    rodata_resv[SONE_RODA0_MAXLEN / 4];
} tSoneInit;

//-----------CMDP-------------------------
// payload memory size: 1MB
#define SONE_MEDIA_PAYLOAD_MEMSIZE 0x100000
#define SONE_MEMORY_EMPTYSIZE	   0x400
#define SONE_MEDIA_QSIZE	   256

struct ep_info {
	char	 name[4];
	unsigned port_id : 8; // for RISCV, it's sync_counter ID; Driver, it's port
	unsigned resvbit : 24;
	uint32_t resv[2];
};

#define SONE_EP_MAX_NUM	 32
#define SONE_EPINFO_SIZE 1024
#define SONE_EP_RESV	 (SONE_EPINFO_SIZE - 4 - (SONE_EP_MAX_NUM * sizeof(struct ep_info)))

struct parti_cmdp_ep {
	char	       ch_name;
	uint8_t	       ch_id;
	uint8_t	       ep_num;
	uint8_t	       resv;
	struct ep_info EP[SONE_EP_MAX_NUM];
	uint32_t       ep_resv[SONE_EP_RESV / 4];
};

struct media_oneQ_ctrl {
	unsigned total_num_minus1 : 9;
	unsigned curpos_in	  : 9;
	unsigned cur_in_num	  : 9;
	unsigned resvbit	  : 5;
	uint32_t resv;
};

typedef struct media_queue_ctrl {
	struct media_oneQ_ctrl p0;
	uint32_t	       reserve[2];
} tQCtrl;

struct cmdmsg_attachment {
	uint32_t timestamp_send;
	uint32_t timestamp_ack;
	uint32_t resv[6];
};

struct media_hdr_info {
	uint32_t cmd_type_main	 : 6;
	uint32_t cmd_type_minor	 : 10;
	uint32_t usr_flag0	 : 1;
	uint32_t usr_flag1	 : 1;
	uint32_t usr_flag2	 : 1;
	uint32_t sync_mode	 : 1; // 0: sync mode, 1: async mode
	uint32_t multicast_en	 : 1;
	uint32_t attachment_en	 : 1;
	uint32_t follow_pack_num : 4;
	uint32_t reserved	 : 6;
};

struct media_magic_data {
	union {
		char		magic[2];
		uint16_t	exp_lines;
	};
	uint16_t sequ_cnt   : 4;
	uint16_t checksum   : 3;
	uint16_t priority   : 1;
	uint16_t cmd_status : 1; // 0: done, 1: unfinished
	uint16_t uid	    : 7; // Unique ID for matching between cmd and msg
};

struct media_cmdmsg_hdr {
	struct media_magic_data magic;
	struct media_hdr_info	hdr_info;

	union {
		uint64_t gtc;

		struct {
			uint32_t gtc_low;
			uint32_t gtc_high;
		};
	};
};

struct media_command {
	struct media_cmdmsg_hdr cmd_hdr;
	uint32_t		user_cmd_data[4];
};

struct media_cmd_queue {
	char			 magic[4]; //"CMD0"
	uint32_t		 resv0[7];
	tQCtrl			 ctrl_cmd[SONE_MAX_CHNUM]; // for every target core
	struct media_command	 c0[SONE_MEDIA_QSIZE];
	uint8_t			 payload_cmd[SONE_MEDIA_PAYLOAD_MEMSIZE + SONE_MEMORY_EMPTYSIZE];
	struct cmdmsg_attachment ca0[SONE_MEDIA_QSIZE];
};

struct media_cmdmsg_queue {
	struct parti_cmdp_ep   ep;
	struct media_cmd_queue cqueue;
	uint32_t	       empty0[1024 / 4];
};

struct channel_header {
	char	 magic[4]; //"CMDP"
	unsigned chnum	 : 4;
	unsigned resvbit : 28;
	uint32_t resv[32 / 4 - 2];
};

typedef volatile struct media_cmdmsg_channels {
	struct channel_header	  hdr;
	struct media_cmdmsg_queue ch[SONE_MAX_CHNUM];
	// Driver: ch_id=0, "D"
	// FW:  ch_id=1, "F"
} tSoneCmdp;

// init & cmdp partition 4KB Bytes (0x1000) align
#define SONE_PART_ALIGN_SIZE	 0x1000
#define SONE_PART_ALIGN_MASK	 0xFFF
// slab and internal frame buf 1MB (0x100000) align
#define SONE_BUF_ADDR_ALIGN_SIZE 0x100000
#define SONE_BUF_ADDR_ALIGN_MASK 0xFFFFF
#define SONE_MEDIA_SLAB_BUFSIZE	 0x100000
#define SONE_MAGIC_SONE		 "SONE"
#define SONE_MAGIC_INIT		 "INIT"
#define SONE_MAGIC_CMDP		 "CMDP"
#define SONE_MAGIC_CMDP_C0	 "CMD0"
#define SONE_MAGIC_CMDP_M0	 "MSG0"
#define SONE_CORP_NAME		 "Black Sesame Inc"

// cmd_type_minor definition
enum {
	MINOR_INVALID = 0, // rsv

	MINOR_BOOT_DONE = 0x01,
	MINOR_ABNORMAL,
	MINOR_RW_ADDR,

	MINOR_ISP_BOOTLD_RECONF = 0x30, // boot up, use payload, reply
	MINOR_ISP_BOOTLD_ALGO_BIN,	// boot up, use payload, reply
	MINOR_ISP_BOOTLD_IQ_BIN,	// boot up, use payload, reply
	MINOR_ISP_START,		// boot up, reply

	MINOR_ISP_CAM_OPEN = 0x40,
	MINOR_ISP_CAM_CLOSE,
	MINOR_ISP_CAM_PLUGOUT,
	MINOR_ISP_CAM_PLUGIN,

	MINOR_ISP_NEW_VIEW_BUF = 0x50,
	MINOR_ISP_VIEW_FRAME_DONE,

	MINOR_ISP_RAW_OPEN = 0x60,
	MINOR_ISP_RAW_CLOSE,
	MINOR_ISP_NEW_RAW_BUF,
	MINOR_ISP_RAW_FRAME_DONE,
};

/* Setup INIT partition for communication between driver and firmware.
 * @pInit: The start virtual address of INIT partition from CPU side
 * @dma_base_low: The low memory address from firmware side
 * @dma_base_high: The high memory address from firmware side
 * @platform: Platform name for debug, should be less than 4 bytes
 * @env: Environment name for debug, should be less than 4 bytes
 * @trace_mask: Trace mask for debug
 * @core_name: IP core name for debug, should be less than 16 bytes
 * @short_name: One character's abbreviation for IP core name
 */
void setup_init_parti(tSoneInit *pInit, uint32_t dma_base_low, uint32_t dma_base_high,
		      char *platform, char *env,
		      uint32_t trace_mask, char *core_name, char short_name);

/* Setup CMDP partition for communication between driver and firmware.
 * @pInit: The start virtual address of INIT partition from CPU side
 * @pCmdp: The start virtual address of CMDP partition from CPU side
 * @cmdp_dma_base: The CMDP partition's base memory address from firmware side
 */
void setup_cmdp_parti(tSoneInit *pInit, tSoneCmdp *pCmdp, uint32_t cmdp_dma_base);

/* Setup SLAB partition for communication between driver and firmware.
 * @pInit: The start virtual address of INIT partition from CPU side
 * @pSlab: The start virtual address of SLAB partition from CPU side
 * @slab_dma_base: The SLAB partition's base memory address from firmware side
 */
void setup_slab_parti(tSoneInit *pInit, void *pSlab, uint32_t slab_dma_base);

#endif /* __BST_COREIP_PROTO_H__ */
