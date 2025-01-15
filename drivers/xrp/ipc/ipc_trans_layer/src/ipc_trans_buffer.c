#include "ipc_trans_buffer.h"
// #include <stdio.h>

// extern ipc_bst_sys //sys_mb;
#define buffer_min(x, y)                                                                                               \
    ({                                                                                                                 \
        typeof(x) _min1 = (x);                                                                                         \
        typeof(y) _min2 = (y);                                                                                         \
        (void)(&_min1 == &_min2);                                                                                      \
        _min1 < _min2 ? _min1 : _min2;                                                                                 \
    })

/* Function definitions ------------------------------------------------------*/
void *bst_memcpy(void *dest, const void *src, uint32_t len)
{
    if (dest == NULL || src == NULL)
    {
        return NULL;
    }
    void *res = dest;
    int wordnum = len / 4;
    if (dest <= src || (uint32_t *)dest >= (uint32_t *)src + len)
    {
        while (wordnum--)
        {
            *(uint32_t *)dest = *(uint32_t *)src;
            dest = (uint32_t *)dest + 1;
            src = (uint32_t *)src + 1;
        }
    }
    else
    {
        src = (uint32_t *)src + len - 1;
        dest = (uint32_t *)dest + len - 1;
        while (wordnum--)
        {
            *(uint32_t *)dest = *(uint32_t *)src;
            dest = (uint32_t *)dest - 1;
            src = (uint32_t *)src - 1;
        }
    }
    return res;
}
void *bst_memset(void *dst, int val, uint32_t count)
{
    if (dst == NULL)
    {
        return NULL;
    }
    char *ret = (char *)dst;
    while (count--)
    {
        *ret++ = (char)val;
    }
    return dst;
}

/**
 * @brief  Allocates a new FIFO and its internal buffer.
 * @return buffer pointer.
 */

void buffer_Malloc(struct buffer *fifo)
{
    fifo->size = BUFFER_SIZE * BUFFER_MAX_COUNT;
    fifo->in = 0;
    fifo->out = 0;
    bst_memset(fifo->buffer, 0, fifo->size);
}

/**
 * @brief  Frees the FIFO.
 * @param  [in] fifo: The fifo to be freed.
 * @return None.
 */
void buffer_Free(struct buffer *fifo)
{
    bst_memset(fifo->buffer, 0, fifo->size);
    fifo->in = 0;
    fifo->out = 0;
    fifo->size = 0;
}

/**
 * @brief  Puts some data into the FIFO.
 * @param  [in] fifo: The fifo to be used.
 * @param  [in] in:   The data to be added.
 * @param  [in] len:  The length of the data to be added.
 * @return The number of bytes copied.
 * @note   This function copies at most @len bytes from the @in into
 *         the FIFO depending on the free space, and returns the number
 *         of bytes copied.
 */
uint32_t buffer_In(struct buffer *fifo, void *in, uint32_t len)
{
    uint32_t buf_len = len;

    buf_len = buffer_min(buf_len, buffer_Avail(fifo));
    /* First put the data starting from fifo->in to buffer end. */
    uint32_t l = buffer_min(buf_len, fifo->size - (fifo->in & (fifo->size - 1)));
    bst_memcpy(fifo->buffer + (fifo->in & (fifo->size - 1)), in, l);
    /* Then put the rest (if any) at the beginning of the buffer. */
    bst_memcpy(fifo->buffer, (uint8_t *)in + l, buf_len - l);
    // g_libipc_compat_ops.mem_barrier();
    fifo->in += BUFFER_SIZE;
    return buf_len;
}

/**
 * @brief  Gets some data from the FIFO.
 * @param  [in] fifo: The fifo to be used.
 * @param  [in] out:  Where the data must be copied.
 * @param  [in] len:  The size of the destination buffer.
 * @return The number of copied bytes.
 * @note   This function copies at most @len bytes from the FIFO into
 *         the @out and returns the number of copied bytes.
 */
uint32_t buffer_Out(struct buffer *fifo, void *out, uint32_t len)
{
    uint32_t buf_len = len;
    buf_len = buffer_min(buf_len, buffer_Len(fifo));
    /* First get the data from fifo->out until the end of the buffer. */
    uint32_t l = buffer_min(buf_len, fifo->size - (fifo->out & (fifo->size - 1)));
    bst_memcpy(out, fifo->buffer + (fifo->out & (fifo->size - 1)), l);

    /* Then get the rest (if any) from the beginning of the buffer. */
    bst_memcpy((uint8_t *)out + l, fifo->buffer, buf_len - l);
    // g_libipc_compat_ops.mem_barrier();
    fifo->out += BUFFER_SIZE;
    return buf_len;
}
