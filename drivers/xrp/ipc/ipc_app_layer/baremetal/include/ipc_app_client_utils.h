#ifndef IPC_APP_LAYER_CLIENT_UTILITIES_H
#define IPC_APP_LAYER_CLIENT_UTILITIES_H

typedef struct
{
    void *cb;
    void *ext;
} callback_registration_t;

static inline void add_registry(callback_registration_t *reg, void *cb, void *ext)
{
    reg->cb = cb;
    reg->ext = ext;
}

#endif