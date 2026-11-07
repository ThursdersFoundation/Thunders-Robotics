/**
 * @file drv_encoder.h
 * @brief Low-level quadrature encoder driver.  Wraps TIM2 (left wheel) and
 *        TIM5 (right wheel) in encoder mode TI1+TI2 and provides cumulative
 *        32-bit counter reads plus a counter reset.
 */
#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/**
 * @brief Initialise the encoder driver.  Stores the per-wheel timer handles.
 *        Both timers are assumed to have been configured in encoder mode
 *        TI1+TI2 by MX_TIM2_Init / MX_TIM5_Init already.
 * @param htim_left   Handle to TIM2 (left wheel encoder).
 * @param htim_right  Handle to TIM5 (right wheel encoder).
 * @return HAL_OK on success, HAL_ERROR on null pointer.
 */
HAL_StatusTypeDef DrvEncoder_Init(TIM_HandleTypeDef *htim_left,
                                   TIM_HandleTypeDef *htim_right);

/**
 * @brief Read the latest raw 32-bit CNT register values for both wheels.
 *        The values are NOT adjusted for counter wrapping; the higher-level
 *        Encoder module in Core/Src/encoder.c handles 32-bit overflow.
 * @param left   Output left raw counter.
 * @param right  Output right raw counter.
 * @return HAL_OK on success, HAL_ERROR on null pointer / not initialised.
 */
HAL_StatusTypeDef DrvEncoder_GetCounts(int32_t *left, int32_t *right);

/**
 * @brief Compute per-wheel RPM from the latest raw counter delta.
 * @param dt_ms    Sample interval in milliseconds since the last call.
 * @param rpm_l    Output left wheel RPM (positive = forward).
 * @param rpm_r    Output right wheel RPM.
 * @return HAL_OK on success, HAL_ERROR on null pointer.
 */
HAL_StatusTypeDef DrvEncoder_GetRPM(uint32_t dt_ms, float *rpm_l, float *rpm_r);

/**
 * @brief Reset both wheel counters to zero.  Used after a homing event or a
 *        manual zero-odometry command.
 */
void DrvEncoder_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* DRV_ENCODER_H */
