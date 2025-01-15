// sample-client.h
#ifndef SAMPLE_SERVER_H
#define SAMPLE_SERVER_H

#include "ipc_app_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // type definition

    // constant definition

    // Method types
    typedef void (*test_hello_callback_t)(const char *message, void *ext);

    // Broadcast types
    typedef void (*test_heartbeat_callback_t)(uint8_t status, void *ext);
    typedef void (*test_heartbeat_sub_callback_t)(uint32_t err, void *ext);
    typedef void (*test_heartbeat_unsub_callback_t)(uint32_t err, void *ext);

    // Interface client
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

    // init client
    test_client *test_client_init();
    // destroy client
    int32_t test_client_destroy();

#ifdef __cplusplus
}
#endif

#endif