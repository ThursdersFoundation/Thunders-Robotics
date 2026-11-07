/**
 * @file drv_ultrasonic.h
 * @brief HC-SR04-style ultrasonic distance sensor driver.  Generates a
 *        10 us trigger pulse on a TIM channel and measures the echo width
 *        via input capture (TIM3 CH3 on PB0).  Sound speed is compensated
 *        for an assumed 22 deg C ambient temperature.
 */
#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* -------------------------------------------------------------------------
 * HC-SR04 timing and physical constants.
 * ------------------------------------------------------------------------- */
#define DRV_US_TRIG_PULSE_US     10U     /* 10 us pulse width              */
#define DRV_US_TIMEOUT_MS         35U    /* ~6 m max echo at 343 m/s         */
#define DRV_US_SPEED_SOUND_MPS   343.0f  /* sound speed at 22 deg C         */

/**
 * @brief Initialise the ultrasonic driver.  Stores the trigger timer and
 *        channel, and the echo pin/port.  Assumes the trigger timer is in
 *        PWM output mode and the echo pin is configured as alternate-function
 *        input-capture (caller's responsibility via MX_TIM3_Init / MX_GPIO_Init).
 * @param trig_tim       Handle to the trigger timer (TIM3).
 * @param trig_channel   TIM channel used for the trigger pulse.
 * @param echo_pin       GPIO pin used for the echo signal.
 * @param echo_port      GPIO port used for the echo signal.
 * @return HAL_OK on success, HAL_ERROR on null pointer.
 */
HAL_StatusTypeDef DrvUltrasonic_Init(TIM_HandleTypeDef *trig_tim,
                                       uint32_t trig_channel,
                                       uint16_t echo_pin,
                                       GPIO_TypeDef *echo_port);

/**
 * @brief Fire a single 10 us trigger pulse and start the echo timer.
 */
void DrvUltrasonic_Trigger(void);

/**
 * @brief Compute the most recent echo distance in metres.  Uses the
 *        sound-speed constant and the echo pulse width captured by input
 *        capture (1 LSB = 1 us at the assumed timer prescaler).
 * @param dist_m  Output pointer.
 * @return HAL_OK on success, HAL_ERROR on no echo received (timeout).
 */
HAL_StatusTypeDef DrvUltrasonic_GetDistance(float *dist_m);

/**
 * @brief Median-filter N consecutive samples to suppress single-sample
 *        outliers (e.g. from a fly-by trigger reflection).
 * @param dist_m  Output median distance in metres.
 * @param n       Number of samples (1..8).
 * @return HAL_OK on success, HAL_ERROR on null pointer or n out of range.
 */
HAL_StatusTypeDef DrvUltrasonic_GetMedian(float *dist_m, uint8_t n);

#ifdef __cplusplus
}
#endif

#endif /* DRV_ULTRASONIC_H */
