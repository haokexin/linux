# Ipc_trans_layer User Guide

## Terminology

**session**: session management. it refers to each thread or process has one session id. The maximum session id range is from 0 to 15. Of course, driver developers could set their **SESSION_COUNT_MAX** to restrict session number in their own environment.

**channel**: Basically, it refers to message transferring channel. In ipc transferring layer, we define one MsgBx Filter of the MsgBx End as a channel, but it would only be available when it comes about receiving messages. What's more, what the relationship between session and channel? According to definition, session is a software conception while channel is a hardware conception. Different sessions can be combined and be dispatched in one message channel, and their binding mapping will be defined in end-userd interface - *.fdepl files. So each channel would have its own completed and consecutive session id(from 0 to 15, default).

**handle**: session management tag. which is only available in ipc_trans_layer and upper layer. This "id" is a 'uint32_t' type integer value which contains channel index and session index.
> note: if your environment is a single session runtime, session id value only contains session index information.

**IMPORTANT:**
we provide two demos over baremetal and posix. Posix environment means your environment support task or thread related functions, while baremetal is just as its literal meaning. In additon, if you implemente it on baremetal, you should care about the **whole calling sequence** issue to make sure this module can work well.

## Code Directory Description

├── CMakeLists.txt      #本目录下的CMakeList.txt
├── baremetal
 └── ipc_trans_impl.c   #ipc传输层裸驱版本兼容层源文件，驱动开发者需对照ipc_trans_runtime.h实现兼容
├── posix
 └── ipc_trans_impl.c   #ipc传输层posix版本兼容层源文件，驱动开发者需对照ipc_trans_runtime.h实现兼容
├── include            #头文件，该层级提供给更上层结构代码API及属性定义
 └── config.h           #ipc_trans_layer配置文件
 └── ipc_trans_common.h #ipc_trans_layer通用定义头文件
 └── ipc_trans_layer.h  #ipc_trans_layer头文件，供ipc应用层使用
├── src                 #ipc传输层各组件源文件
 └── ipc_trans_runtime.h #ipc传输层核心逻辑接口
 └── others

## Trans_layer Configuration

feature list:

- IPC_TRANS_LAYER_SES_MODE
- MSG_FLT_THR
- EACH_SESSION_RECV_MSG_FIFO_SIZE
- MSG_END_MGT_DEFAUT
- MSG_FLT_MGT_DEFAUT

If you define **IPC_TRANS_LAYER_SES_MODE** into multi session mode, you need define below two configurations to restrict your count of channel and session, if necessary.

- CHANNEL_COUNT
- SESSION_COUNT

### Trans_layer Mode Definition

You could define **IPC_TRANS_LAYER_SES_MODE** to change trans_layer mode.

#### Single Session Mode

In this mode, you could only create one session, which could be client or server. In this case, that means you could only act as a server to wait remote calling and process related options and reply results or broadcast signals to different clients. Or you could only act as a client to call remote end or wait for signals.
Significantly, this configuration is a **MUST** define in your implementated ipc driver.
impact range:

1. support session number
2. receive message dispatching rules
3. support api
4. compile code size

#### Trans_layer Dispatching Rules

 1. trans_layer message

- push in status management module to process

 2. method message

- single server: enter in the only one session
- multi server: according to method registerring map to find sid & fid

 3. reply & broadcast message

- single client: enter in the only one session
- multi client: get sid from receive message header

#### Multi Session Mode
>
> note: current version only support single session mode, that means you could only create one session to transfer message to remote msgbx end.

### Trans_layer Communication Definition

#### MsgBx Filter Threashold
>
> TBD

#### Software Session Receive Message Fifo Size

We prepare two received message quques in each session. which can store messages as a buffer for receivers.
Driver developers could change **EACH_SESSION_RECV_MSG_FIFO_SIZE** to modify these queues size, if necessary.

### Trans_layer Status Management Definition

Prerequisite:
These two defines **IPC_STATE_MGT_ENABLE** and **IPC_FLT_MGT_ENABLE** are enabled.

#### End Management

You could define **MSG_END_MGT_DEFAUT** to enable MsgBx End default enable state management features.
below is support options

```
// state management enable flag bit
#define MSGBX_END_POOL_STS_EN_BIT       0x20
#define MSGBX_TX_OVERFLOW_EN_BIT        0x10
#define MSGBX_TX_THRS_EN_BIT            0x08
#define MSGBX_DEF_RX_OVERFLOW_EN_BIT    0x04
#define MSGBX_DEF_RX_UNDERFLOW_EN_BIT   0x02
#define MSGBX_DEF_RX_THRS_EN            0x01

// define your own management enable error status monitor features
#define MSG_END_MGT_DEFAUT MSGBX_END_POOL_STS_EN_BIT | MSGBX_TX_OVERFLOW_EN_BIT
```

#### Filter Management

You could define **MSG_FLT_MGT_DEFAUT** to enable MsgBx End filter default enable state management features.
below is support options

```
// filter management enable flag bit
#define MSGBX_FLT_OVERFLOW_EN_BIT       0x04
#define MSGBX_FLT_UNDERFLOW_EN_BIT      0x02
#define MSGBX_FLT_THRS_EN_BIT           0x01

// define your own management enable error status monitor features
#define MSG_FLT_MGT_DEFAUT MSGBX_FLT_OVERFLOW_EN_BIT | MSGBX_FLT_UNDERFLOW_EN_BIT
```

## Ipc_trans Layer Introduction

### Files you need to care

* **./include/ipc_trans_layer.h** : this header refers to what APIs you must support to port libipc to other subsystem.
- **./src/ipc_trans_runtime.h** : this is the ipc trans_layer core logic APIs, you could use these APIs to implement your ipc driver in your own environment.

and below are something you should implemente in different os or baremetal.
- ./baremetal/xxx_ipc_trans_impl.c

### Component Diagram

Below is trans_layer component diagram. There are eight modules in ipc_trans layer, which include:
- **Runtime** : ipc_trans layer core module, which provide most APIs for developers.
- **Configuration** : ipc_trans layer config management module, which provide support of static config parsing, MsgBx filter config rules support.
- **State Management** : ipc_trans layer status management module, which provide support of MsgBx end and filters management enable, remote MsgBx status monitor and ipc_trans layer message processing.
- **Session Management** : ipc_trans layer session management module, which provide support of trans_layer session create and destroy, session storing message and session status management.
- **Routing** : ipc_trans layer message dispatching module, which provide support of message dispatching.
- **Utilities** : ipc_trans layer utilities. Currently, it only support a internal ring-buffer.

![](../doc/assets/component.png)

### Main Feature Sequence Diagram

#### Ipc_trans Layer Initiate

![](../doc/assets/trans_layer_init.bmp)

#### Session Create

![](../doc/assets/trans_layer_session_create.bmp)

## Implement in Baremetal Version

Please refer to *./baremetal/ipc_trans_impl.c*

### Prerequisites

### APIs for Implemente

**./include/ipc_trans_layer.h** : this header refers to what APIs you must support to port libipc to other subsystem.

#### Initialization

```
// only for api calling, if you use server-mode, you don't need to call these two api to start the whole ipc driver
int32_t ipc_trans_layer_start(uint8_t role);
```

**IMPORTANT:**
As for trans_layer developers
In **ipc_trans_layer_start()**, you should call **ipc_trans_init()** to pass your receiving message function callback pointer and error handle callback pointer. Specifically, *role* at here is only a demo required value, which means in release version, we will delete this input parameter.
What you need do in both callback function, we will introduce in later fault handle and receiving message sections.

#### Create handle

```
// server api only
int32_t ipc_trans_layer_stub_create_handle(const uint8_t fid, const uint8_t sid, uint32_t *handle);

// client api only
int32_t ipc_trans_layer_proxy_create_handle(const uint8_t fid, const uint8_t sid, uint32_t *handle);
```

**IMPORTANT:**
As for trans_layer developers
Both APIs above have same process logic except the second step.

1. input parameter check
2. call **ipc_trans_create_session()** to create a session.

> Note: you should pass correct role enum in this API

3. return handle id

#### Send Message

```
// server api only
int32_t ipc_trans_layer_stub_send_reply_msg(const uint32_t handle, rw_msg_t msg);
int32_t ipc_trans_layer_stub_send_broadcast(const uint32_t handle, rw_msg_t msg);

// client api only
int32_t ipc_trans_layer_proxy_send_method(const uint32_t handle, rw_msg_t msg);
```

**IMPORTANT:**
As for trans_layer developers

1. call **ipc_trans_send_msg()** to send message. what you should do is to pass correct message type in input parameter.

#### Query Available Receive Message

```
// this API is only for baremetal version
int32_t ipc_trans_layer_get_msg(const uint32_t handle);

```

**IMPORTANT:**
As for trans_layer developers

1. call **status_msg_process()** to process trans_layer protocol message
2. call **ipc_trans_get_avail_info** to query if there is any received message in session message queues.

#### Get Message

```
// server api only
int32_t ipc_trans_layer_stub_get_method_msg(const uint32_t handle, rw_msg_t *msg);

// client api only
int32_t ipc_trans_layer_proxy_get_reply_msg(const uint32_t handle, rw_msg_t *msg);
int32_t ipc_trans_layer_proxy_get_broadcast_msg(const uint32_t handle, rw_msg_t *msg);
```

**IMPORTANT:**
As for trans_layer developers

1. call **ipc_trans_get_msg()** to send message. what you should do is to pass correct message type in input parameter.

### Trans_layer Self Process Logic

#### Receive Message

##### Sequence Diagram

![](../doc/assets/trans_layer_msg_dispatch_baremetal.bmp)

As you can see above, in this sequence diagram illustrate the whole receiving and dispatching message steps.
In hardware layer, it can receive interrupt from MsgBx and unmask interrupt, which means there is some message available in MsgBx filter. Trans_layer's callback will be triggered in interrupt handle function and call **ipc_trans_read_msg()** to read message from filter and dispatch it into session. Finally, when finish whol callback function, hardware layer will mask interrupt and finish receiving this time.

If upper layer will get message, it would call **ipc_trans_get_msg()** to get message from session message queues directly.

#### Fault Handle

In hardware layer, it can receive error interrupt from MsgBx and get error status from related register, then it will notify trans_layer via callback function. So in this function, developers will get passed *err_msg* and decide to how to handle this error, maybe you could call **ipc_trans_err_hdl()**  to clean or do other options, maybe you could change your own status or disable some function according to your environment or requirement. These information and process only in trans_layer and do not need to pass to upper layer.

## Implement in Posix Version

Please refer to *./posix/ipc_trans_impl.c*

### Prerequisites

* thread or task
- OS related IPC(inter-process communication), such as MsgReceive on QNX, system call on Linux userspace or ipc on freeRTOS.
- semaphore or completion

### APIs for Implemente

**./include/ipc_trans_layer.h** : this header refers to what APIs you must support to port libipc to other subsystem.

Main API implementation introduction please refer to baremetal corresponding section.

#### Initialization

```
int32_t ipc_trans_layer_start(uint8_t role);
```

**IMPORTANT:**
As for trans_layer developers
In **ipc_trans_layer_start()**, you should call **ipc_trans_init()** to pass your receiving message function callback pointer and error handle callback pointer. Specifically, *role* at here is only a demo required value, which means in release version, we will delete this input parameter.
What you need do in both callback function, we will introduce in later fault handle and receiving message sections.
Then you should start a task or thread to wait for receiving message.

#### Create handle
Please refer to same section in baremetal version implement.

#### Send Message
Please refer to same section in baremetal version implement.

### Trans_layer Self Process Logic
#### Receive Message
##### Sequence Diagram
![](../doc/assets/trans_layer_msg_dispatch_posix.bmp)

As you can see above, in this sequence diagram illustrate the whole receiving and dispatching message steps.
We suggest you to start a specific thread or task to receive message for performance consideration. In this task or thread you could use software semaphore or completion to wait fro receive message. Once receive callback function will be triggered, you could post or release related semaphore or completion. In receiving thread, when we pass waiting step, you could call **ipc_trans_read_msg()**, which reads messages from fifo and dispatches them into different sessions. Then you would return to waiting step again to wait another messages. 

If upper layer will get message, it would call **ipc_trans_get_msg()** to get message from session message queues directly.

#### Fault Handle
This function is same as receiving message. We suggest you to start other specific thread or task to process error messages.