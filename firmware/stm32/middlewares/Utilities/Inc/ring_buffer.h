/**
 * @file    ring_buffer.h
 * @brief   Lock-free-for-single-producer/single-consumer byte ring buffer.
 *
 * Used between DMA interrupt producers (UART RX) and task consumers. The
 * head/tail indices are updated atomically, so no critical section is
 * needed for the SPSC case; guard with a critical section when mixing
 * producers.
 */
#ifndef THUNDERS_RING_BUFFER_H
#define THUNDERS_RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RING_BUFFER_MAX_SIZE 512U

typedef struct
{
    uint8_t  buffer[RING_BUFFER_MAX_SIZE];
    volatile uint16_t head;   /* write index */
    volatile uint16_t tail;   /* read index  */
    uint16_t capacity;        /* actual usable size (<= RING_BUFFER_MAX_SIZE) */
    uint32_t overrun_count;   /* dropped bytes when full */
} RingBuffer;

void     RingBuffer_Init(RingBuffer *rb, uint16_t capacity);
bool     RingBuffer_Push(RingBuffer *rb, uint8_t byte);
bool     RingBuffer_Pop(RingBuffer *rb, uint8_t *byte);
bool     RingBuffer_Peek(const RingBuffer *rb, uint16_t offset, uint8_t *byte);
size_t   RingBuffer_Read(RingBuffer *rb, uint8_t *dest, size_t maxLen);
size_t   RingBuffer_Write(RingBuffer *rb, const uint8_t *src, size_t len);
size_t   RingBuffer_Count(const RingBuffer *rb);
size_t   RingBuffer_Free(const RingBuffer *rb);
void     RingBuffer_Flush(RingBuffer *rb);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_RING_BUFFER_H */
