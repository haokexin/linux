#include "ipc_trans_utl.h"

const uint8_t end_id_idx[MSGBX_END_COUNT_MAX] = {
#define X_MACRO(a, b) b,
    MACROS_TABLE
#undef X_MACRO
};

int32_t end_is_valid(const uint8_t end_id)
{
    int32_t ret = 0;

    switch (end_id)
    {
#define X_MACRO(a, b) case a:
        MACROS_TABLE
#undef X_MACRO
        ret = 1;
        break;
    default:
        ret = 0;
    }
    return ret;
}

int32_t query_end_id_idx(uint8_t end_id)
{
    uint8_t cnt;

    for (cnt = 0; cnt < MSGBX_END_COUNT_MAX; cnt++)
    {
        if (end_id_idx[cnt] == end_id)
        {
            return cnt;
        }
    }
    return -1;
}
