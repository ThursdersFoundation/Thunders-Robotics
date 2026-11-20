/**
 * @file    moving_average.h
 * @brief   Integer moving-average filter for sensor smoothing.
 */
#ifndef THUNDERS_MOVING_AVERAGE_H
#define THUNDERS_MOVING_AVERAGE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOVING_AVERAGE_MAX_WINDOW 16U

typedef struct
{
    int32_t  window[MOVING_AVERAGE_MAX_WINDOW];
    uint8_t  size;        /* configured window size (1..16) */
    uint8_t  index;       /* next write slot                */
    uint8_t  filled;      /* samples seen so far (for warmup) */
    int64_t  sum;
} MovingAverage;

void    MovingAverage_Init(MovingAverage *ma, uint8_t windowSize);
int32_t MovingAverage_Update(MovingAverage *ma, int32_t sample);
int32_t MovingAverage_Last(const MovingAverage *ma);
void    MovingAverage_Reset(MovingAverage *ma);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_MOVING_AVERAGE_H */
