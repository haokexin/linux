# IPC Application Layer User Guide
- [IPC Application Layer User Guide](#ipc-application-layer-user-guide)
  - [BareMetal Sample](#baremetal-sample)
    - [fidl/fdepl](#fidlfdepl)
    - [Server interface](#server-interface)
    - [Client interface](#client-interface)

The application layer code are all generated from fidl and fdepl files.
Based on your RTE setting in fdepl, BareMetal or POSIX code are generated.

The fidl and fdepl files are the core of the application layer.
The language grammar and deploy specifications are commented in sample files.
For each project, it always has a specified fidl file, describing the interface, including
version, constants, data structures, methods, and broadcasts.
The deployment for the interface is defined in xxx-inf.fdepl, which will be imported in both
server and client deployments.
The xxx-server.fdepl defines the server properties, which may lead code generation, and passed
to source code.
The xxx-client.fdepl defines the client properties.
All possible properties are defined in the ipc_spec.fdepl, which will be unique across projects.
It will be contained in the directory of code generator, users shall NOT re-define it.

The code generator is bst_idl_code_gen, which takes the xxx-server.fdepl or xxx-client.fdepl as
input, and verifies and generates code for it. A simple script is given as gen.sh to shown the
usage.

After code generation, users should pay attention to the interface files, e.g. xxx-server.h and
xxx-client.h in BareMetal code.
These files defines the funtions for server and client.

Now, let's take a sample, and go through it.

## BareMetal Sample

### fidl/fdepl

In sample.fidl, we define a simple interface, com.bst.ipc.sample.test.
The package name is "com.bst.ipc.sample", while the interface name is "test".
In the "test", method "hello" and broadcast "heartbeat" are defined.

Now, let's see the sample-inf.fdepl. It defines "MaxMethodID" and "MaxBroadcastID" for interface 
"test", as well as the "MethodID" for method "hello", and "BroadcastID" "SubscribeMethodID"
"UnsubscribeMethodID" for broadcast "heartbeat".

The sample-server.fdepl imports sample-inf.fdepl, and defines properties for server.
Note, "ProviderType" indicates the role for "Server" or "Client", "RTE" defines which code template
is used for code generation. Users could refer to the ipc_spec.fdepl to get the meaning of rest 
properties.

The sample-client.fdepl is defined as sample-server.fdepl.

### Server interface

After code generation, the server interfaces are defined in sample-server.h and sample-server.c.
Serialization and deserialization, packing and unpacking for large data, are already supported.
User should NOT modify these files manually.

The server interface is defined as struct "test_server", while "test" is the interface name defined
in sample.fidl. It contains several function pointers, including "version", "register_hello",
"heartbeat", and two message processing functions, "receive_message" and "dispatch_message".

User should first call "test_server_init" to get the interface pointer, then register hello method,
then call "receive_message" and "dispatch_message" in the main loop, and finally, when exit, call
"test_server_destroy".

When a message received and dispatched, the registered method is called automatically. Here is the
hello method, which is defined as `void (*)(const char *name, char *message)`. The "name" is input
argument, and "message" is output. User only need read from "name", and write to "message". The
code generated in test_server.c will fetch the output data, make message, and send it via transport
layer.

a sample app is shown in `test/baremetal/server.c`. User can found more detail in it.

### Client interface

The generated client code is sample-client.h and sample-client.c.

Like server code, client interface also defines a structure called "test_client".

```C
typedef struct
{
    // get version
    ipc_inf_version (*version)();
    // call hello method
    int32_t (*hello)(const char *name, test_hello_callback_t cb, void *ext);
    // subscribe heartbeat broadcast
    int32_t (*heartbeat_sub)(test_heartbeat_callback_t cb, void *ext, test_heartbeat_sub_callback_t cb2,
                                void *ext2);
    // unsubscribe heartbeat broadcast
    int32_t (*heartbeat_unsub)(test_heartbeat_unsub_callback_t cb, void *ext);

    // message router, call in main loop.
    // receive messages
    int32_t (*receive_message)();
    // dispatch message
    int32_t (*dispatch_message)();
} test_client;
```

The client sample app is shown in `test/baremetal/client.c`.
