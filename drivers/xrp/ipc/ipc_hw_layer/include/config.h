#ifndef _IPC_HW_CONFIG_H
#define _IPC_HW_CONFIG_H

// enum definition
#define IPC_HW_DEVICE IPC_HW_MSGBX
#define IPC_ROLE_DEFAULT IPC_ROLE_SERVER_ONLY

#define IPC_HW_LAYER_VERSION 1
//#define IPC_HW_IS_32BIT 0

// reg addr definition
#define MSGBX_BASE_ADDR 0x32011000
#define MSGBX_ADDR_WIDTH 12
#define MSGBX_DATA_WIDTH 32

// irq & addr int, hw impl need set at first
#define RX_DEF_RXFIFO_END_ADDR 0x0
#define RX_DEF_RXFIFO_ST_ADDR 0x0

// spec config bit
/*this bit is used for indicating if this end is shared by multi-environment*/
#define MSGBX_IS_SHARED_END 0

#endif