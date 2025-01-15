#ifndef _IPC_TRANS_CONFIG_H
#define _IPC_TRANS_CONFIG_H

#include "../../ipc_hw_layer/include/ipc_hw_common.h"

// developer config value ---------------------------------------------------------------------
// module configuration
// 1 means single mode, 2 means multi mode
// this definition will impact session related processing
#define IPC_TRANS_LAYER_SES_MODE 1

// static transferring layer configuration
// note: channel_count * session_count = support max thread
// YOU MUST FILL CHANNEL COUNT BY YOUR HARDWARE SPEC
#define CHANNEL_COUNT 4 // rw 
#define SESSION_COUNT 2 // rw

// state management function enable flag
// note: you could select which error interrupt message will be notified to trans_layer
// note: below is full function definition
#define MSG_END_MGT_CONFIG      MSGBX_ECC_RX_MULTIP_EN_BIT | MSGBX_ECC_RX_DETECT_EN_BIT | MSGBX_PARITY_HWDATA_EN_BIT | MSGBX_PARITY_HADDR_EN_BIT
#define MSG_DEF_FLT_MGT_CONFIG  MSGBX_TX_OVERFLOW_EN_BIT | MSGBX_RX_OVERFLOW_EN_BIT | MSGBX_RX_UNDERFLOW_EN_BIT
#define MSG_FLT_MGT_CONFIG      MSGBX_RX_OVERFLOW_EN_BIT | MSGBX_RX_UNDERFLOW_EN_BIT

// buffer size for each session
#define EACH_SESSION_RECV_MSG_FIFO_SIZE 16 // rw

#endif