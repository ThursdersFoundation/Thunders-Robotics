/**
 * @file    moving_average.c
 * @brief   O(1) moving average over an int32 ring of samples.
 */
#include "moving_average.h"

void MovingAverage_Init(MovingAverage *ma, uint8_t windowSize)
{
    if (ma == NULL) { return; }
    if (windowSize == 0U || windowSize > MOVING_AVERAGE_MAX_WINDOW)
    {
        windowSize = MOVING_AVERAGE_MAX_WINDOW;
    }
    for (uint8_t i = 0U; i < MOVING_AVERAGE_MAX_WINDOW; i++)
    {
        ma->window[i] = 0;
    }
    ma->size   = windowSize;
    ma->index  = 0U;
    ma->filled = 0U;
    ma->sum    = 0;
}

int32_t MovingAverage_Update(MovingAverage *ma, int32_t sample)
{
    if (ma == NULL || ma->size == 0U) { return sample; }

    if (ma->filled < ma->size)
    {
        /* Warm-up: accumulate without evicting */
        ma->window[ma->index] = sample;
        ma->sum += sample;
        ma->filled++;
    }
    else
    {
        ma->sum -= ma->window[ma->index];
        ma->window[ma->index] = sample;
        ma->sum += sample;
    }

    ma->index = (uint8_t)((ma->index + 1U) % ma->size);
    return (int32_t)(ma->sum / ma->filled);
}

int32_t MovingAverage_Last(const MovingAverage *ma)
{
    if (ma == NULL || ma->filled == 0U) { return 0; }
    uint8_t last = (uint8_t)((ma->index + ma->size - 1U) % ma->size);
    return ma->window[last];
}

void MovingAverage_Reset(MovingAverage *ma)
{
    if (ma == NULL) { return; }
    for (uint8_t i = 0U; i < MOVING_AVERAGE_MAX_WINDOW; i++)
    {
        ma->window[i] = 0;
    }
    ma->index  = 0U;
    ma->filled = 0U;
    ma->sum    = 0;
}
