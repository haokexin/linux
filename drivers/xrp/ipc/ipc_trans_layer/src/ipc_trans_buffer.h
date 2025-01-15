#ifndef IPC_BUFFER_H
#define IPC_BUFFER_H

/* Header include -----------------------------------------------------------*/
#include "bstipc_cfg.h"

#define BUFFER_SIZE 64
#define BUFFER_MAX_COUNT 128

/* Type definitions ----------------------------------------------------------*/
struct buffer
{
    uint8_t buffer[BUFFER_SIZE * BUFFER_MAX_COUNT];
    uint32_t size;
    volatile uint32_t in;
    volatile uint32_t out;
};

/* Variable declarations -----------------------------------------------------*/
/* Variable definitions ------------------------------------------------------*/
/* Function declarations -----------------------------------------------------*/
void buffer_Malloc(struct buffer *fifo);
void buffer_Free(struct buffer *fifo);
uint32_t buffer_In(struct buffer *fifo, void *in, uint32_t len);
uint32_t buffer_Out(struct buffer *fifo, void *out, uint32_t len);

/* Function definitions ------------------------------------------------------*/

/**
 * @brief  Removes the entire FIFO contents.
 * @param  [in] fifo: The fifo to be emptied.
 * @return None.
 */
static inline void buffer_Reset(struct buffer *fifo)
{
    fifo->in = fifo->out = 0;
}

/**
 * @brief  Returns the size of the FIFO in bytes.
 * @param  [in] fifo: The fifo to be used.
 * @return The size of the FIFO.
 */
static inline uint32_t buffer_Size(struct buffer *fifo)
{
    return fifo->size;
}

/**
 * @brief  Returns the number of used bytes in the FIFO.
 * @param  [in] fifo: The fifo to be used.
 * @return The number of used bytes.
 */
static inline uint32_t buffer_Len(struct buffer *fifo)
{
    return fifo->in - fifo->out;
}

/**
 * @brief  Returns the number of bytes available in the FIFO.
 * @param  [in] fifo: The fifo to be used.
 * @return The number of bytes available.
 */
static inline uint32_t buffer_Avail(struct buffer *fifo)
{
    return buffer_Size(fifo) - buffer_Len(fifo);
}

/**
 * @brief  Is the FIFO empty?
 * @param  [in] fifo: The fifo to be used.
 * @retval bst_true:      Yes.
 * @retval bst_false:     No.
 */
static inline int32_t buffer_IsEmpty(struct buffer *fifo)
{
    if (buffer_Len(fifo) == 0)
        return 1;
    else
        return 0;
}

/**
 * @brief  Is the FIFO full?
 * @param  [in] fifo: The fifo to be used.
 * @retval bst_true:      Yes.
 * @retval bst_false:     No.
 */
static inline int32_t buffer_IsFull(struct buffer *fifo)
{
    if (buffer_Avail(fifo) == 0)
        return 1;
    else
        return 0;
}

// #ifdef __cplusplus
// }
// #endif

#endif