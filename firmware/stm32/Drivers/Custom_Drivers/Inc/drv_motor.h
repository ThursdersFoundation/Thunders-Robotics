/**
 * @file drv_motor.h
 * @brief Low-level motor driver: PWM on TIM1, sign-magnitude direction control
 *        via GPIO, and a deadband to protect against shoot-through at low PWM.
 */
#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* -------------------------------------------------------------------------
 * PWM scaling constants.  The TIM1 ARR is 8399 -> PWM range -8399..+8399.
 * The motor controller uses the convention -1000..+1000 internally, and
 * DrvMotor scales that to the actual TIM1 CCR by multiplying by ARR/1000.
 * ------------------------------------------------------------------------- */
#define DRV_MOTOR_PWM_MAX       1000        /* logical PWM range            */
#define DRV_MOTOR_PWM_ARR       8399        /* TIM1 auto-reload             */
#define DRV_MOTOR_PWM_DEADBAND  35          /* ~3.5% of full scale           */

/**
 * @brief Initialise the motor driver.  Stores the TIM handle, channel
 *        indices and GPIO port/pin for each motor's direction line, then
 *        forces the motor disabled (ENABLE lines low, PWM = 0).
 * @param htim            Handle to TIM1 (motor PWM base timer).
 * @param channel_left    TIM1 channel for the left motor (TIM_CHANNEL_x).
 * @param channel_right   TIM1 channel for the right motor.
 * @param dir_port_left   GPIO port for the left motor direction line.
 * @param dir_pin_left    GPIO pin for the left motor direction line.
 * @param dir_port_right  GPIO port for the right motor direction line.
 * @param dir_pin_right   GPIO pin for the right motor direction line.
 * @return HAL_OK on success, HAL_ERROR on null pointer.
 */
HAL_StatusTypeDef DrvMotor_Init(TIM_HandleTypeDef *htim,
                                 uint32_t channel_left, uint32_t channel_right,
                                 GPIO_TypeDef *dir_port_left,  uint16_t dir_pin_left,
                                 GPIO_TypeDef *dir_port_right, uint16_t dir_pin_right);

/**
 * @brief Apply a per-wheel PWM command.  Negative values reverse direction.
 *        Both arguments are clamped to [-DRV_MOTOR_PWM_MAX, +DRV_MOTOR_PWM_MAX]
 *        and a small deadband is applied near zero.
 * @param left   Left PWM in [-1000, +1000].
 * @param right  Right PWM in [-1000, +1000].
 * @return HAL_OK on success, HAL_ERROR on driver not initialised.
 */
HAL_StatusTypeDef DrvMotor_SetPWM(int16_t left, int16_t right);

/**
 * @brief Apply active braking - both motor drivers short the motor windings
 *        to GND and PWM is forced to 0.
 */
void DrvMotor_Brake(void);

/**
 * @brief Disable the motor drivers (ENABLE lines low).  PWM is also zeroed.
 */
void DrvMotor_Disable(void);

/**
 * @brief Return the most recently-applied left and right PWM.
 * @param left   Output last left PWM.
 * @param right  Output last right PWM.
 */
void DrvMotor_GetLastPWM(int16_t *left, int16_t *right);

#ifdef __cplusplus
}
#endif

#endif /* DRV_MOTOR_H */
