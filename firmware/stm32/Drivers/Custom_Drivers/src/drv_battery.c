/**
 * @file drv_battery.c
 * @brief Battery monitor driver.  Samples the ADC channel wired to the pack
 *        voltage through an external 11:1 divider and converts to volts and
 *        a 0..100 state-of-charge percentage using a piecewise-linear map.
 */
#include "drv_battery.h"

#include <string.h>

static ADC_HandleTypeDef *s_hadc    = NULL;
static uint32_t           s_channel = 0;
static uint8_t            s_inited  = 0;

/* Voltage -> SoC curve breakpoints for a 3S Li-Ion pack at light load. */
typedef struct { float v; uint8_t pct; } VoltPct_t;
static const VoltPct_t s_curve[] = {
    {  9.00f, 0   }, { 10.00f, 5  }, { 10.50f, 10 },
    { 11.00f, 20 }, { 11.40f, 40 }, { 11.70f, 60 },
    { 12.00f, 80 }, { 12.30f, 95 }, { 12.60f, 100 }
};
#define CURVE_LEN (sizeof(s_curve) / sizeof(s_curve[0]))

HAL_StatusTypeDef DrvBattery_Init(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    if (hadc == NULL) {
        return HAL_ERROR;
    }
    s_hadc    = hadc;
    s_channel = channel;
    s_inited = 1;
    return HAL_OK;
}

static HAL_StatusTypeDef sample_average(uint16_t *raw_out)
{
    if (s_hadc == NULL || raw_out == NULL) {
        return HAL_ERROR;
    }
    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = s_channel;
    ch.Rank         = 1;
    ch.SamplingTime = ADC_SAMPLETIME_56CYCLES;
    if (HAL_ADC_ConfigChannel(s_hadc, &ch) != HAL_OK) {
        return HAL_ERROR;
    }
    uint32_t sum = 0;
    for (uint8_t i = 0; i < DRV_BATTERY_SAMPLES; i++) {
        if (HAL_ADC_Start(s_hadc) != HAL_OK) {
            return HAL_ERROR;
        }
        if (HAL_ADC_PollForConversion(s_hadc, 50U) != HAL_OK) {
            return HAL_ERROR;
        }
        sum += HAL_ADC_GetValue(s_hadc);
        HAL_ADC_Stop(s_hadc);
    }
    *raw_out = (uint16_t)(sum / DRV_BATTERY_SAMPLES);
    return HAL_OK;
}

float DrvBattery_RawToVolts(uint16_t raw)
{
    /* Vref = 3.3 V, 12-bit ADC, then scale by the external divider ratio. */
    const float lsb_v = 3.3f / (float)((1UL << 12) - 1UL);
    return (float)raw * lsb_v * DRV_BATTERY_DIVIDER_RATIO;
}

HAL_StatusTypeDef DrvBattery_Read(float *volts)
{
    if (volts == NULL || !s_inited) {
        return HAL_ERROR;
    }
    uint16_t raw;
    if (sample_average(&raw) != HAL_OK) {
        return HAL_ERROR;
    }
    *volts = DrvBattery_RawToVolts(raw);
    return HAL_OK;
}

HAL_StatusTypeDef DrvBattery_ReadPercent(uint8_t *pct)
{
    if (pct == NULL) {
        return HAL_ERROR;
    }
    float v;
    if (DrvBattery_Read(&v) != HAL_OK) {
        return HAL_ERROR;
    }
    /* Clamp at the curve endpoints. */
    if (v <= s_curve[0].v) {
        *pct = 0;
        return HAL_OK;
    }
    if (v >= s_curve[CURVE_LEN - 1].v) {
        *pct = 100;
        return HAL_OK;
    }
    /* Find the bracketing points and interpolate linearly. */
    for (size_t i = 1; i < CURVE_LEN; i++) {
        if (v < s_curve[i].v) {
            float frac = (v - s_curve[i - 1].v) / (s_curve[i].v - s_curve[i - 1].v);
            float p = (float)s_curve[i - 1].pct +
                       frac * (float)(s_curve[i].pct - s_curve[i - 1].pct);
            *pct = (uint8_t)(p + 0.5f);
            return HAL_OK;
        }
    }
    *pct = 100;
    return HAL_OK;
}
