/**
 * @file drv_battery.h
 * @brief Battery monitor driver.  Reads the ADC channel wired to the pack
 *        voltage via an external 11:1 divider and converts to volts and a
 *        0..100 state-of-charge percentage using a piecewise-linear map.
 */
#ifndef DRV_BATTERY_H
#define DRV_BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* -------------------------------------------------------------------------
 * 3S Li-Ion pack parameters.  These bounds are checked by the safety
 * manager; the ADC channel carries the divided-down voltage.
 * ------------------------------------------------------------------------- */
#define DRV_BATTERY_FULL_V        12.6f   /* 4.2 V/cell * 3        */
#define DRV_BATTERY_EMPTY_V        9.0f    /* 3.0 V/cell cut-off    */
#define DRV_BATTERY_NOMINAL_V     11.1f   /* 3.7 V/cell nominal    */
#define DRV_BATTERY_DIVIDER_RATIO 11.0f
#define DRV_BATTERY_SAMPLES        16U

/**
 * @brief Initialise the battery driver.  Stores the ADC handle and channel.
 * @param hadc     Handle to ADC1.
 * @param channel  ADC channel used for the battery monitor.
 * @return HAL_OK on success, HAL_ERROR on null pointer.
 */
HAL_StatusTypeDef DrvBattery_Init(ADC_HandleTypeDef *hadc, uint32_t channel);

/**
 * @brief Read the pack voltage in volts.  Samples the ADC
 *        DRV_BATTERY_SAMPLES times and returns the average, then scales by
 *        the external divider ratio.
 * @param volts  Output pointer.
 * @return HAL_OK on success, HAL_ERROR on ADC failure.
 */
HAL_StatusTypeDef DrvBattery_Read(float *volts);

/**
 * @brief Read the state-of-charge as a 0..100 percentage.  Uses a
 *        piecewise-linear map between DRV_BATTERY_EMPTY_V and
 *        DRV_BATTERY_FULL_V, clamped at the endpoints.
 * @param pct  Output pointer (0..100).
 * @return HAL_OK on success, HAL_ERROR on ADC failure.
 */
HAL_StatusTypeDef DrvBattery_ReadPercent(uint8_t *pct);

/**
 * @brief Convenience: convert a raw 12-bit ADC code to pack volts using the
 *        external divider ratio.  Used by tests to avoid hitting the ADC.
 * @param raw  12-bit ADC code (0..4095).
 * @return Pack voltage in volts.
 */
float DrvBattery_RawToVolts(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* DRV_BATTERY_H */
