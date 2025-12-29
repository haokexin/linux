#include <linux/kthread.h>
#include <linux/delay.h>
#include <bst/bstipc_cfg.h>
#include <bst/ipc_app_client_utils.h>
#include <bst/ipc_app_common.h>
#include <bst/ipc_trans_layer.h>
#include "ipc_trans_common.h"

#ifndef __MSGBOX_SEND_H__
#define __MSGBOX_SEND_H__

// macro definitions
#define CID SAFETY_0
#define FID DEF
#define SID 6U

#define CMD_METHOD_SCMI 5U

    typedef struct
    {
        /**
         * Get the version of the interface.
         * 
         * @return The version struct, containing major and minor.
         */
        ipc_inf_version_t (*version)(void);
        
        /**
         * Fire and forget call to the no_reply_method. 
         * This is one way method call. The server will NOT return. 
         * 
         * @param interval The input argument of method heartbeat.
         * @return 0 if success, negative if fail.
         * @note This is unreliable transmission, be used ONLY if message losing is accepted. 
         */
        int32_t (*send)(const uint8_t interval);

        /**
         * Start the message router.
         * 
         * @return 0 if success, negative if fail.
         */
        int32_t (*start)(void);

        /**
         * Stop the message router.
         * 
         * @return 0 if success, negative if fail.
         */
        int32_t (*stop)(void);
    }msgbox_send_client_t;

/**
 * struct scmi_msgbox - Structure representing a SCMI msgbox transport
 *
 * @cinfo: SCMI channel info
 * @shmem: Transmit/Receive shared memory area
 * @shmem_lock: Lock to protect access to Tx/Rx shared memory area
 * @tx_complete: represent transmit is compelted
 */

struct scmi_msgbox {
    uint64_t  addr; //addr[0~31]: shmem addr, addr[31~63]: do_xfer status addr.
	struct scmi_chan_info *cinfo;
	struct scmi_shared_mem __iomem *shmem;
	spinlock_t shmem_lock;
	struct completion *tx_complete;
};

// internal data structure
typedef struct
{
    msgbox_send_client_t client;
    ipc_inf_version_t version;
    uint8_t handle;
    des_buf_t des_buf;
    callback_registration_t method_registry[IPC_TOKEN_NUM];
    
    serdes_t serializer;
    serdes_t deserializer;
    struct task_struct *route_task;
    _Atomic volatile bool bRunning;
    struct mutex send_mtx;
    ext_info_t info;
}msgbox_send_t;

int32_t send_by_msgbox(const uint64_t addr);
msgbox_send_client_t *msgbox_client_init(void);
int32_t msgbox_client_destroy(void);



#endif