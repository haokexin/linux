// sample-server.h
#ifndef SAMPLE_SERVER_H
#define SAMPLE_SERVER_H

#include "ipc_app_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // constant definition

    // method type
    typedef void (*test_hello_t)(const char *name, char *message);

    // Interface server
    typedef struct
    {
        // get version
        ipc_inf_version (*version)();
        // register hello method
        int32_t (*register_hello)(test_hello_t func);
        // trigger heartbeat broadcast
        int32_t (*heartbeat)(uint8_t status);

        // message router, call in main loop.
        // receive messages
        int32_t (*receive_message)();
        // dispatch message
        int32_t (*dispatch_message)();
    } test_server;

    // init server
    test_server *test_server_init();
    // destroy client
    int32_t test_server_destroy();

#ifdef __cplusplus
}
#endif

#endif