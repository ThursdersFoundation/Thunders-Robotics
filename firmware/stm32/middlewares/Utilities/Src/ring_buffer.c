/**
 * @file    ring_buffer.c
 * @brief   Byte ring buffer implementation (one byte reserved to disambiguate
 *          full vs. empty).
 */
#include "ring_buffer.h"

void RingBuffer_Init(RingBuffer *rb, uint16_t capacity)
{
    if (rb == NULL) { return; }
    if (capacity == 0U || capacity > RING_BUFFER_MAX_SIZE - 1U)
    {
        capacity = RING_BUFFER_MAX_SIZE - 1U;
    }
    rb->head = 0U;
    rb->tail = 0U;
    rb->capacity = capacity;
    rb->overrun_count = 0U;
}

/** Number of stored bytes (uint16 indices wrap modulo 65536). */
size_t RingBuffer_Count(const RingBuffer *rb)
{
    if (rb == NULL) { return 0U; }
    uint16_t h = rb->head, t = rb->tail;
    if (h >= t) { return (size_t)(h - t); }
    return (size_t)(0x10000UL - (t - h));
}

size_t RingBuffer_Free(const RingBuffer *rb)
{
    if (rb == NULL) { return 0U; }
    return (size_t)rb->capacity - RingBuffer_Count(rb);
}

bool RingBuffer_Push(RingBuffer *rb, uint8_t byte)
{
    if (rb == NULL || RingBuffer_Count(rb) >= rb->capacity)
    {
        if (rb != NULL) { rb->overrun_count++; }
        return false;
    }
    rb->buffer[rb->head % rb->capacity] = byte;
    rb->head++;
    return true;
}

bool RingBuffer_Pop(RingBuffer *rb, uint8_t *byte)
{
    if (rb == NULL || byte == NULL || RingBuffer_Count(rb) == 0U)
    {
        return false;
    }
    *byte = rb->buffer[rb->tail % rb->capacity];
    rb->tail++;
    return true;
}

bool RingBuffer_Peek(const RingBuffer *rb, uint16_t offset, uint8_t *byte)
{
    if (rb == NULL || byte == NULL || offset >= RingBuffer_Count(rb))
    {
        return false;
    }
    *byte = rb->buffer[(rb->tail + offset) % rb->capacity];
    return true;
}

size_t RingBuffer_Read(RingBuffer *rb, uint8_t *dest, size_t maxLen)
{
    size_t n = 0U;
    while (n < maxLen && RingBuffer_Pop(rb, &dest[n]))
    {
        n++;
    }
    return n;
}

size_t RingBuffer_Write(RingBuffer *rb, const uint8_t *src, size_t len)
{
    size_t written = 0U;
    while (written < len && RingBuffer_Push(rb, src[written]))
    {
        written++;
    }
    return written;
}

void RingBuffer_Flush(RingBuffer *rb)
{
    if (rb == NULL) { return; }
    rb->tail = rb->head;
}
