# ipc_hw_layer Programmer Guide

## Code directory description

├── CMakeLists.txt      #CMakeList.txt(reserved)
├── ipc_hw_layer.c      #ipc hardware layer source code
├── include            #common header directory, all apis in this directorty are used for upper layer
 └── config.h           #ipc_hw_layer only configuration
 └── ipc_hw_common.h    #ipc_hw_layer data structure definition
 └── ipc_hw_layer.h     #ipc_hw_layer api definition, which is used by ipc_trans_layer
├── msgbx_impl          #msgbx-based ipc hardware plugin, implementated by driver developers
 └── ipc_hw_impl.h      #msgbx header, general api defined by bstipc
 └── test.cpp           #sub-directory source codes
├── irq_shm_impl        #irq+shared-buffer-based ipc hardware plugin, implementated by driver developers
 └── test.h             #header, general api defined by bstipc
 └── test.cpp           #sub-directory source codes
├── semaphore           #semaphore-based ipc hardware plugin, implementated by driver developers
 └── test.h             #header, general api defined by bstipc
 └── test.cpp           #sub-directory source codes

## hw_impl overview

### Prerequisites

* hardware register access
* interrupt initiation and handle
* AXI protocol (MsgBx required)

### Support environment

* Linux Kernel
* QNX
* Baremetal or FreeRTOS

### Files you need to care

* **./msgbx_impl/ipc_hw_impl.h** : this header refers to what APIs you must support to port libipc to other subsystem.
* **./ipc_hw_layer.c** : this file indicates logic in ipc_hw_layer, which is the interface for upper layer. You may modify this file, if necessary.

and below are something you should implemente in different os or baremetal.

* ./msgbx_impl/xxx_impl_linux.c
* ./msgbx_impl/xxx_impl_linux.h

### How to use ipc_hw_layer

#### Start environment, add and open the devices

##### initialize libipc environment

1. register your options

    ```
    int32_t ipc_hw_register_ops(libipc_hw_compat_ops_t *ops)
    ```

2. get information from hardware

   ```
    msgbx_hw_info_t hw_info;
    g_hw_ctl_ops.ipc_hw_get_info(hw_info);
   ```

3. set configuration information to hardware

   ```  
    ipc_init_params_t init_params = {
        .mbx_device = IPC_HW_MSGBX,
        .mbx_role = IPC_ROLE_CLIENT_ONLY,
    };
   g_hw_ctl_ops.ipc_hw_init(ipc_param);
   ```

4. enable MsgEnd state management

   ```
    if (ipc_param->msgbx_end_mgt_flag != 0) {
        struct msgbx_hw_sts_mgt hw_mgt_info = {
            .mbx_rx_thr = ipc_param->msgbx_def_rx_thr,
            .mbx_end_sts_mgt_flag = ipc_param->msgbx_end_mgt_flag,
        };
        g_hw_ctl_ops.ipc_hw_sts_mgt_enble(&hw_mgt_info);
    }
   ```

**IMPORTANT:**
As for hardware driver developers
in *ipc_hw_init()*, you should prepare basic running environment, which contains:
a. MsgEnd Default RXFIFO related info
    Please refers to **ipc_default_reg_list.xlsx** register definition.
b. MsgEnd Default Filter interrupt and irq_handle

> For Linux kernel, devices are described in a device tree.
> for Baremetal and FreeRTOS, because there is no device tree abstraction, devices must be defined statically before attempting to open them.

#### Send message

below is message format definition,

```
#if (IPC_HW_LAYER_VERSION == 1)
typedef struct ipc_hw_raw_msg_header
{
    uint32_t pid : 8;
    uint32_t cid : 8;
    uint32_t len : 4;
    uint32_t is_sec : 1;
    uint32_t is_32b : 1;
    uint32_t resh: 2;
    uint32_t sid : 4;
    uint32_t fid : 4;
    uint32_t cmd : 8;
    uint32_t typ : 4;
    uint32_t tok : 4;
    uint32_t idx : 4;
    uint32_t res : 8;
    uint32_t ver : 4;
} rw_msg_header;
#endif

typedef struct ipc_hw_raw_msg
{
    rw_msg_header header; 
    uint64_t payload[5];
} rw_msg_t;
```

**IMPORTANT:**
As for hardware driver developers
in *ipc_hw_send_msg()*, you should ensure message content push in MsgBx TXFIFO, which contains：
a. when you send message to MsgEnd via AXI protocol, you should make sure finish whole message producing process in some time.
b. return send message to MsgEnd result.

#### Configure MsgBx Filter

below is MsgBx filter related config definition,

```
struct msgbx_flt_info {
    uint32_t mbx_rxfifo_end_addr : 10;      // RW, this info is got from hw impl
    uint32_t mbx_rxfifo_st_addr : 10;       // RW
    uint16_t mbx_rx_pid_end : 8;            // RW
    uint16_t mbx_rx_pid_st : 8;             // RW   
    uint8_t mbx_pid_flt_invert : 1;         // RW
};

// msgbx filter config
msgbx_flt_cfg_t {
    uint8_t flt_id;
    enum ipc_flt_rule_location_e cfg_loc;
    // msgbx filter header filtering rule
    struct msgbx_flt_cmp_cfg rule;
    uint32_t msgbx_flt_thr : 10;

#ifdef IPC_FLT_MGT_ENABLE
    // msgbx filter general config, use bit offset refer to different config
    uint8_t mbx_flt_mgt_flag;
#endif
};

struct msgbx_flt_device {
    struct msgbx_flt_info info;
    msgbx_flt_cfg_t cfg;
};
```

Basically, MsgBx default Filter config is static in each MsgBx End, so the filter we talk about here refers to filter allocated by hardware to each end. These filter can be set in runtime according to the upper software layer.
Below are the steps to configure MsgBx filter:

1. set rule to filter

```
    msgbx_flt_cfg_t flt_cfg = {
        .flt_id = 1,            // fid start with 1, fid 0 means default fliter
        .cfg_loc = IPC_FLT_RULE_HEADER,
        .rule.tx_reserved_filter_mask = 8,
        .rule.rx_res_min = 256,   // fid = 1, sid = 0
        .rule.rx_res_max = 496,  // fid = 1, sid( 4bits ) = 15
        .mbx_flt_mgt_flag = 0,
        .msgbx_flt_thr = 15,    // default thr is 15
    };
    g_hw_ctl_ops.ipc_hw_set_flt_cfg(cfg->flt_id, &cfg->rule);
```

2. get hardware information about filter

```
    g_hw_ctl_ops.ipc_hw_get_flt_info(cfg->flt_id, info);
```

3. enable filter state management

```
#ifdef IPC_FLT_MGT_ENABLE
    // g_hw_ctl_ops.ipc_hw_flt_mgt_enble(cfg->flt_id, cfg->mbx_flt_mgt_flag, cfg->msgbx_flt_thr);
#endif
```

**IMPORTANT:**
As for hardware driver developers
in *ipc_hw_set_flt_cfg()*, you should enable filter related process, which contains:
a. set filtering registers
b. enable filter
c. enable irq

#### Receive message

below is the sequence diagram of receiving message.
![](../doc/assets/recv_seq.jpg)
As for message receiving function, we use notify-getting strategy. It means that hardware layer would send a notification, when they receive interrupt from MsgBx default filter or dedicated config filter. IPC_trans_layer receive this notification, then call read_message from related filter fifo until it goes empty.
a. notifying

```
extern int32_t ipc_hw_recv_msg_notify(const uint8_t fid);
```

you may use this api in your irq_handle function and then clear the interrupt. *fid* means which filter interrupt has been triggered.

b. read message from MsgBx filter FIFO

```
    // fid means from which filter FIFO we read message.
    g_hw_ctl_ops.ipc_hw_get_msg(msg, fid);
```

**IMPORTANT:**
As for hardware driver developers
in *ipc_hw_get_msg()*, you may consider the issue about data structure format, such as big-endian and little-endian compatibility. you could find message format definition from previous sector. Please make sure the data format match header definition, if necessary.

#### MsgBx state management

MsgBx state management contains End's state management, which is configured in initialization process, and Filter's state management, which is configured in runtime.
But whole MsgBx management function can be enable or disable staticly in global config file when we compile this module.

##### MsgBx End management

we have illustrated the steps in *initialize libipc environment* section.
below is the description in detail.

```
    ipc_init_params_t init_params = {
        .mbx_device = IPC_HW_MSGBX,
        .mbx_role = IPC_ROLE_SERVER_ONLY,
        .msgbx_end_mgt_flag = MSGBX_END_POOL_STS_EN_BIT | MSGBX_TX_OVERFLOW_EN_BIT,
    }; 

    struct msgbx_hw_sts_mgt hw_mgt_info = {
        .mbx_rx_thr = ipc_param->msgbx_def_rx_thr,
        .mbx_end_sts_mgt_flag = ipc_param->msgbx_end_mgt_flag,
    };
    g_hw_ctl_ops.ipc_hw_sts_mgt_enble(&hw_mgt_info);
```

MsgBx End management contains follow content:

```
struct msgbx_hw_sts_mgt {
    // uint32_t mbx_rx_thr : 10;
    // uint32_t mbx_rxfifo_sts : 10;
    // target_end_id * target_end_sts
    // uint32_t mbx_end_sts : 32;
    uint8_t mbx_end_sts_mgt_flag;
};

// state management enable flag bit
#define MSGBX_END_POOL_STS_EN_BIT       0x20
#define MSGBX_TX_OVERFLOW_EN_BIT        0x10
#define MSGBX_TX_THRS_EN_BIT            0x08
#define MSGBX_DEF_RX_OVERFLOW_EN_BIT    0x04
#define MSGBX_DEF_RX_UNDERFLOW_EN_BIT   0x02
#define MSGBX_DEF_RX_THRS_EN            0x01
```

**IMPORTANT:**
As for hardware driver developers
in *ipc_hw_sts_mgt_enble()*, you should enable management process, which contains:
a. set related enable register
b. enable irq
c. wait for interrupt

when you receive related interrupt, you should do:
a. enter irq_handler
b. read related status register
c. notify upper layer, we would introduce this function in *MsgBx fault notifying* sections.
d. irq ack

##### MsgBx Filter management

the steps are same as *MsgBx End management* section.
Below is MsgBx filter config flag

```
    msgbx_flt_cfg_t flt_cfg = {
        ...

        .mbx_flt_mgt_flag = MSGBX_FLT_THRS_EN_BIT | MSGBX_FLT_UNDERFLOW_EN_BIT | MSGBX_FLT_OVERFLOW_EN_BIT,
        .msgbx_flt_thr = 15,    // default thr is 15
    };

    g_hw_ctl_ops.ipc_hw_flt_mgt_enble(cfg->flt_id, cfg->mbx_flt_mgt_flag, cfg->msgbx_flt_thr);
```

MsgBx Filter management contains follow content:

```
// filter management enable flag bit
#define MSGBX_FLT_OVERFLOW_EN_BIT       0x04
#define MSGBX_FLT_UNDERFLOW_EN_BIT      0x02
#define MSGBX_FLT_THRS_EN_BIT           0x01
```

##### MsgBx get fault message

prerequisite:
As said before, when hardware layer receive fault interrupt from MsgBx, it would read related status register in irq_handle and notify to the upper layer.
below is some demo code in implementation layer which should be wrote by hardware driver developers

```
    xxx irq_handle() {
        ...
        ipc_hw_err_msg_notify();
        ...
        irq_ack();
    }
```

1. read error message

```
    msgbx_err_msg_t {
    uint8_t type : 4;       // msgbx. msgend, msgflt
    uint8_t id : 4;         
    uint8_t msg : 8;        // overflow, underflow, threshold
    uint64_t res : 16;
    };

    msgbx_err_msg_t err_msg;
    g_hw_ctl_ops.ipc_hw_get_err_msg(&err_msg);
```

**IMPORTANT:**
As for hardware driver developers
in *ipc_hw_get_err_msg()*, you should do follow steps:
a. package fault information into date structure *msgbx_err_msg*.

##### MsgBx fault handle

TBD
support feature list:
for MsgBx End:

* clear MsgBx RXFIFO
* clear MsgBx TXFIFO

for MsgBx Filter:

* clear single/multi MsgBx Filter FIFO