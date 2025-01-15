#ifndef IPC_MSGBOX_CONTROLLER_H
#define IPC_MSGBOX_CONTROLLER_H

#include "ipc_common.h"

#if 0
typedef enum rx_mode {
	IRQ_MODE = 0,
	POLL_MODE
} RX_MODE;

struct ipc_mbox {
	struct device *dev;
	void __iomem *event_base;
	void __iomem *sem_base;
	struct ipc_mempool *pool;
#ifdef ON_FPGA
	void __iomem *fpga_reset;
	void __iomem *fpga_status;
#endif
};

// share buffer format definition
struct ipc_aligned_msg {
	struct ipc_fill_register_msg msg;
#ifdef MSG_SIZE_EXTENSION
	uint64_t payload[6];
#else
	uint64_t payload[7];
#endif
};

// ipc shared buffer
struct ipc_all_cores_register_addr // place in one page : 4096Byte
{
	struct ipc_aligned_msg addr[IPC_CORE_MAX]; // 64B*18 = 1152B
	uint64_t data_saved[IPC_CORE_MAX][4096]; // 8B * 18 * 4096 = 576 KB ????
};

int32_t ipc_send_data(struct ipc_client_info *client_info, enum ipc_core_e src,
		      void *data);
#endif

#define MSGBOX_MAX_FILTER_NUM (8)
#define CPU_MSGEND_PPI_NUM (4)
#define MSG_64_MAX_LEN (4)
#define MSGBX_WAIT_TIMEOUT_CNT 0xFF

struct ipc_msgbox {
	struct device *dev;
	void __iomem *fcsr_base;
	void __iomem *rxfifo_base;
	void __iomem *txfifo_base;	
	u32 spi_count;
	u32 msgend_count;
	u32 filter_num;
};

typedef struct {
	u16 irq[MSGBOX_MAX_FILTER_NUM];
	struct ipc_msgbox *ipc_msgbx;
} ST_MSGBX_END_PARA;

typedef struct
{
    union
    {
        struct
        {
            u64 pid:8;
            u64 cid:8;
            u64 len:4;
			u64 is_64_bit:1;
			u64 nonsec:1;
            u64 res:42;
        } bit;
        u64 data;
    }head;
    u64 payload[MSG_64_MAX_LEN];
} ST_MSG_MESSAGE64;

enum {
	RES_ID_FILTER_CSR = 0,
	RES_ID_RXFIFO,
	RES_ID_TXFIFI,
};

enum {
	FILTER_DEFAULT = 0,
	FILTER_NUM1,
	FILTER_NUM2,
	FILTER_NUM3,
	FILTER_NUM4,	
	FILTER_NUM5,	
	FILTER_NUM6,
	FILTER_NUM7,
	FILTER_NUM_BUFF,					
};

enum {
	CPU_ID0 = 0,
	CPU_ID1,
	CPU_ID2,
	CPU_ID3,		
};

typedef struct {
    union
    {
        struct
        {
            u32 endid:8;
            u32 is_64_bit:1;
            u32 filter_num:4;
            u32 res:19;
        } bit;
        u32 reg1;
    } ablt_r;

    union
    {
        struct
        {
            u32 rx_fifo_depth:10;
            u32 tx_fifo_depth:10;
            u32 res:12;
        } bit;
        u32 reg2;
    } ablt_r2;
	u8 version;
} ST_MSG_INF;

enum {
	FILTER1_THRS_INTR = (1 << 0),
	FILTER1_UNDERFLOW_INTR = (1 << 1),
	FILTER1_OVERFLOW_INTR = (1 << 2),
	FILTER1_BUFF_INTR,
};

enum {
	DEF_FILTER_RX_THRS_INTR = (1 << 0),
	DEF_FILTER_RX_UNDERFLOW_INTR = (1 << 1),
	DEF_FILTER_RX_OVERFLOW_INTR = (1 << 2),
	DEF_FILTER_TX_THRS_INTR = (1 << 3),
	DEF_FILTER_TX_OVERFLOW_INTR = (1 << 4),
	DEF_FILTER_MSGBX_END_POOL_STATUS_INTR = (1 << 5),
	DEF_FILTER_BUFF_INTR = (1 << 6),
};

enum {
	COMBI_MODE_AND = 0,
	COMBI_MODE_OR,
	COMBI_MODE_RES,
};

#define FILTER1_RES_LOW_BIT_MASK (0xfff00000ull)
#define FILTER1_PAYLOAD_LOW_BIT_MASK (0xffffffffull)

#define MSGBOX_RXFIFO_OFFSET (0x10000)
#define MSGBOX_TXFIFO_OFFSET (0x11000)
#define FILTER_MAX 7
#define FILTER_CSR_SIZE (0x200)
/* default filter csr offset */
#define DEF_FIL_ABLT_R (0x0)
#define DEF_FIL_ABLT2_R (0x8)
#define DEF_RXFIFO_ADDR (0x10)
#define DEF_FLT_MSG_PIDF_CFGR (0x18)
#define DEF_TX_FIFO_THRES (0x20)
#define DEF_Tx_FIFO_Available (0x28)
#define DEF_DEFAULT_RXTHRS_CFGR (0x30)
#define DEF_FLT_RXFIFO_STATUS (0x38)
#define DEF_FLT_INTER_EN (0x60)
#define DEF_FLT_STATUS_R (0x58)
#define DEF_FLT_INTER_R (0x68)
#define DEF_FLT_VERSION_R (0x78)

#define END_RXFIFO_ADDR_RX_FIFO_ST_ADDR_SHIFT_U32 (0)
#define END_RXFIFO_ADDR_RX_FIFO_END_ADDR_SHIFT_U32 (10)
#define END_RXFIFO_ADDR_FILTER_EN_SHIFT_U32 (31)
#define END_RXFIFO_ADDR_MASK (0x3fful)

#define END_PIDF_CFGR_RX_PID_ST_SHIFT_U32 (0)
#define END_PIDF_CFGR_RX_PID_END_SHIFT_U32 (8)
#define END_PIDF_CFGR_RX_FILTER_INVERT_SHIFT_U32 (30)
#define END_PIDF_CFGR_RX_FILTER_EN_SHIFT_U32 (31)
#define END_PIDF_CFGR_MASK (0xfful)

/* filterN csr offset */
#define FILTER1_RXFIFO_ADDRR (0x0)
#define FILTER1_MSG_PIDF_CFGR (0x8)
#define	End_filter1_RxFIFO_ADDRR	0x00	
#define	End_filter1_MsgH_PIDF_CFGR	0x08	
#define	End_filter1_MsgH_LenF_CFGR	0x10	
#define	End_filter1_MsgH_ResF_MaskR	0x18	
#define	End_filter1_MsgH_ResF_MaskHR	0x20	
#define	End_filter1_MsgH_ResF_MinR	0x28	
#define	End_filter1_MsgH_ResF_MinHR	0x30	
#define	End_filter1_MsgH_ResF_MaxR	0x38	
#define	End_filter1_MsgH_ResF_MaxHR	0x40	
#define	End_filter1_MsgP1_MaskR	0x48	
#define	End_filter1_MsgP1_MaskHR	0x50	
#define	End_filter1_MsgP1_MinR	0x58	
#define	End_filter1_MsgP1_MinHR	0x60	
#define	End_filter1_MsgP1_MaxR	0x68		
#define	End_filter1_MsgP1_MaxHR	0x70	
#define	End_filter1_MsgP2_MaskR	0x78	
#define	End_filter1_MsgP2_MaskHR	0x80	
#define	End_filter1_MsgP2_MinR	0x88	
#define	End_filter1_MsgP2_MinHR	0x90
#define	End_filter1_MsgP2_MaxR	0x98	
#define	End_filter1_MsgP2_MaxHR	0xA0	
#define	End_filter1_MsgP3_MaskR	0xA8	
#define	End_filter1_MsgP3_MaskHR	0xB0	
#define	End_filter1_MsgP3_MinR	0xB8	
#define	End_filter1_MsgP3_MinHR	0xC0	
#define	End_filter1_MsgP3_MaxR	0xD0	
#define	End_filter1_MsgP3_MaxHR	0xD8	
#define	End_filter1_MsgP4_MaskR	0xE0	
#define	End_filter1_MsgP4_MaskHR	0xE8	
#define	End_filter1_MsgP4_MinR	0xF0	
#define	End_filter1_MsgP4_MinHR	0xF8	
#define	End_filter1_MsgP4_MaxR	0x100	
#define	End_filter1_MsgP4_MaxHR	0x108		
#define	End_filter1_RxFIFO_CFGR	0x110		
#define	End_filter_Thrs_CFGR	0x118	
#define	End_filter_RxFIFO_StatusR	0x120

#define FILTER1_STATUS_R (0x128)
#define FILTER1_EN_INTER (0x130)
#define FILTER1_INTER_R (0x138)
#define FILTER_INTER_R_OFFSET (FILTER1_INTER_R - FILTER1_STATUS_R)

/* end fmu register */
#define END_FMU_SAFETY_INTR (0x8000)
#define END_FMU_SAFETY_INTR_EN (0x8008)

//default pid config
#define DEF_FILTER_PID_ST       0x0
#define DEF_FILTER_PID_END      0xFF
#define DEF_FILTER_PID_INVERT   0x0
/* receive one message triggle an interrupt */
#define DEF_FILTER_THRES (1)
/* rxfifo depth */
#define END_FILTER_RX_FIFO_DEPTH (8)

/* parameter is error */
#define  ERR_PARA 1

/* define for register */
typedef union {
	struct
	{
		u32 rx_len_st:4;
		u32 rx_len_end:4;
		u32 res:22;
		u32 rx_len_filter_invert:1;
		u32 rx_len_filter_en:1;
	} bit;
	u32 data;
} FLT1_LEN_CFGR;

typedef union {
	struct
	{
		u32 msgh_flilter_en:1;
		u32 msgp1_filter_en:1;
		u32 msgp2_filter_en:1;	
		u32 msgp3_filter_en:1;
		u32 msgp4_filter_en:1;	//BIT4		
		u32 res:3;
		u32 msgh_flilter_invert:1;	//BIT8
		u32 msgp1_filter_invert:1;
		u32 msgp2_filter_invert:1;	//BIT10
		u32 msgp3_filter_invert:1;
		u32 msgp4_filter_invert:1;	//BIT12	
		u32 msgh_combi_lh_comp:1;	//BIT13
		u32 msgp1_combi_lh_comp:1;		
		u32 msgp2_combi_lh_comp:1;
		u32 msgp3_combi_lh_comp:1;		
		u32 msgp4_combi_lh_comp:1;	//BIT17
		u32 res1:12;			
		u32 filter_combi_mode:2;
	} bit;
	u32 data;
} FLT1_RX_FIFO_CFGR;

/* defined for rxfifo config register */
enum {
	RXFIFO_MSGH_FLILTER_EN = 0,
	RXFIFO_MSGP1_FILTER_EN,
	RXFIFO_MSGP2_FILTER_EN,
	RXFIFO_MSGP3_FILTER_EN,
	RXFIFO_MSGP4_FILTER_EN,
	RXFIFO_MSGH_FLILTER_INVERT = 8,
	RXFIFO_MSGP1_FILTER_INVERT,
	RXFIFO_MSGP2_FILTER_INVERT,
	RXFIFO_MSGP3_FILTER_INVERT,
	RXFIFO_MSGP4_FILTER_INVERT,
	RXFIFO_MSGH_COMBI_LH_COMP = 13,
	RXFIFO_MSGP1_COMBI_LH_COMP,
	RXFIFO_MSGP2_COMBI_LH_COMP,
	RXFIFO_MSGP3_COMBI_LH_COMP,
	RXFIFO_MSGP4_COMBI_LH_COMP,
	RXFIFO_FILTER_COMBI_MODE = 30,
};

enum {
	MSG_FILTER1_RULE_PAYLOAD1 = 0,
	MSG_FILTER1_RULE_PAYLOAD2,	
	MSG_FILTER1_RULE_PAYLOAD3,
	MSG_FILTER1_RULE_PAYLOAD4,		
};

#define PID_MSG_END \
        MSG_END_MACRO(PID_CMN_MSG_END0,        0x10) \
        MSG_END_MACRO(PID_CMN_MSG_END1,        0x11) \
        MSG_END_MACRO(PID_CMN_MSG_END2,        0x12) \
        MSG_END_MACRO(PID_CMN_MSG_END3,        0x13) \
        MSG_END_MACRO(PID_CMN_MSG_END4,        0x14) \
        MSG_END_MACRO(PID_CMN_MSG_END5,        0x15) \
        MSG_END_MACRO(PID_CMN_MSG_END6,        0x16) \
        MSG_END_MACRO(PID_CMN_MSG_END7,        0x17) \
        MSG_END_MACRO(PID_DB_MSG_END0,         0x20) \
        MSG_END_MACRO(PID_DB_MSG_END1,         0x21) \
        MSG_END_MACRO(PID_ISPCV_MSG_END0,      0x30) \
        MSG_END_MACRO(PID_NET_MSG_END0,        0x40) \
        MSG_END_MACRO(PID_DMA_MSG_END0,        0x50) \
        MSG_END_MACRO(PID_DMA_MSG_END1,        0x51) \
        MSG_END_MACRO(PID_R5_SW_MSG_END0,      0x60) \
        MSG_END_MACRO(PID_R5_SW_MSG_END1,      0x61) \
        MSG_END_MACRO(PID_R5_SW_MSG_END2,      0x62) \
        MSG_END_MACRO(PID_R5_SW_MSG_END3,      0x63) \
        MSG_END_MACRO(PID_R5_SW_MSG_END4,      0x64) \
        MSG_END_MACRO(PID_R5_SW_MSG_END5,      0x65) \
        MSG_END_MACRO(PID_R5_SECURE_EDN0,      0x70) \
        MSG_END_MACRO(PID_R5_SECURE_EDN1,      0x71) \
        MSG_END_MACRO(PID_R5_SAFETY_END0,      0x80) \
        MSG_END_MACRO(PID_R5_SAFETY_END1,      0x81) \
        MSG_END_MACRO(PID_R5_REALTIME_END0,    0x90) \
        MSG_END_MACRO(PID_R5_REALTIME_END1,    0x91) \
        MSG_END_MACRO(PID_R5_REALTIME_END2,    0x92) \
        MSG_END_MACRO(PID_R5_REALTIME_END3,    0x93) \
        MSG_END_MACRO(PID_R5_REALTIME_END4,    0x94) \
        MSG_END_MACRO(PID_R5_REALTIME_END5,    0x95) \
        MSG_END_MACRO(PID_MEDIA_MSG_END0,      0xA0)

typedef enum{
    #define MSG_END_MACRO(id, num) id = num,
        PID_MSG_END
    #undef MSG_END_MACRO
} EN_MSGBOX_PID;

#endif
