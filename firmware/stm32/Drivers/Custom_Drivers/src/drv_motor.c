/**
 * @file drv_motor.c
 * @brief Low-level motor driver implementation.  Applies sign-magnitude PWM
 *        drive to TIM1, with a small deadband near zero and hard clamps at
 *        +/- DRV_MOTOR_PWM_MAX.  The ENABLE lines (PC2/PC3) are kept high
 *        while active and low while disabled or braked.
 */
#include "drv_motor.h"

#include <string.h>

/* -------------------------------------------------------------------------
 * Private state.
 * ------------------------------------------------------------------------- */
static TIM_HandleTypeDef *s_htim       = NULL;
static uint32_t           s_chan_left  = 0;
static uint32_t           s_chan_right = 0;
static GPIO_TypeDef      *s_dir_port_left  = NULL;
static GPIO_TypeDef      *s_dir_port_right = NULL;
static uint16_t           s_dir_pin_left   = 0;
static uint16_t           s_dir_pin_right  = 0;
static int16_t            s_last_left  = 0;
static int16_t            s_last_right = 0;
static uint8_t            s_enabled = 0;

/* PC2 = left motor enable, PC3 = right motor enable. */
#define DRV_MOTOR_EN_L_PORT  GPIOC
#define DRV_MOTOR_EN_L_PIN   GPIO_PIN_2
#define DRV_MOTOR_EN_R_PORT  GPIOC
#define DRV_MOTOR_EN_R_PIN   GPIO_PIN_3

static int16_t clamp_pwm(int16_t v)
{
    if (v >  (int16_t)DRV_MOTOR_PWM_MAX) return  (int16_t)DRV_MOTOR_PWM_MAX;
    if (v < -(int16_t)DRV_MOTOR_PWM_MAX) return -(int16_t)DRV_MOTOR_PWM_MAX;
    return v;
}

static uint32_t scale_to_arr(int16_t v)
{
    /* Map -1000..+1000 to 0..ARR.  Center is ARR/2 (50% duty). */
    int32_t scaled = (int32_t)v * (int32_t)DRV_MOTOR_PWM_ARR / DRV_MOTOR_PWM_MAX;
    if (scaled < 0) scaled = -scaled;
    return (uint32_t)scaled;
}

HAL_StatusTypeDef DrvMotor_Init(TIM_HandleTypeDef *htim,
                                 uint32_t channel_left, uint32_t channel_right,
                                 GPIO_TypeDef *dir_port_left,  uint16_t dir_pin_left,
                                 GPIO_TypeDef *dir_port_right, uint16_t dir_pin_right)
{
    if (htim == NULL || dir_port_left == NULL || dir_port_right == NULL) {
        return HAL_ERROR;
    }
    s_htim          = htim;
    s_chan_left     = channel_left;
    s_chan_right    = channel_right;
    s_dir_port_left  = dir_port_left;
    s_dir_pin_left   = dir_pin_left;
    s_dir_port_right = dir_port_right;
    s_dir_pin_right  = dir_pin_right;
    s_last_left  = 0;
    s_last_right = 0;
    s_enabled = 0;

    /* Disable both motor drivers and zero the PWM. */
    HAL_GPIO_WritePin(DRV_MOTOR_EN_L_PORT, DRV_MOTOR_EN_L_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV_MOTOR_EN_R_PORT, DRV_MOTOR_EN_R_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(s_htim, s_chan_left,  0);
    __HAL_TIM_SET_COMPARE(s_htim, s_chan_right, 0);
    return HAL_OK;
}

HAL_StatusTypeDef DrvMotor_SetPWM(int16_t left, int16_t right)
{
    if (s_htim == NULL) {
        return HAL_ERROR;
    }
    left  = clamp_pwm(left);
    right = clamp_pwm(right);

    /* Apply deadband: if |v| < deadband, force PWM to 0. */
    if (left  > -DRV_MOTOR_PWM_DEADBAND && left  < DRV_MOTOR_PWM_DEADBAND) left  = 0;
    if (right > -DRV_MOTOR_PWM_DEADBAND && right < DRV_MOTOR_PWM_DEADBAND) right = 0;

    /* Direction: 1 = forward (DIR high), 0 = reverse (DIR low). */
    HAL_GPIO_WritePin(s_dir_port_left,  s_dir_pin_left,
                      (left  >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(s_dir_port_right, s_dir_pin_right,
                      (right >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(s_htim, s_chan_left,  scale_to_arr(left));
    __HAL_TIM_SET_COMPARE(s_htim, s_chan_right, scale_to_arr(right));

    /* Enable drivers if we have a non-zero command. */
    HAL_GPIO_WritePin(DRV_MOTOR_EN_L_PORT, DRV_MOTOR_EN_L_PIN,
                      (left  != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV_MOTOR_EN_R_PORT, DRV_MOTOR_EN_R_PIN,
                      (right != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    s_last_left  = left;
    s_last_right = right;
    s_enabled = (left != 0 || right != 0) ? 1u : 0u;
    return HAL_OK;
}

void DrvMotor_Brake(void)
{
    if (s_htim == NULL) {
        return;
    }
    /* Active brake: ENABLE high (drivers on) but PWM = 0 -> both low-side
     * FETs on, shorting the motor to GND. */
    HAL_GPIO_WritePin(DRV_MOTOR_EN_L_PORT, DRV_MOTOR_EN_L_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DRV_MOTOR_EN_R_PORT, DRV_MOTOR_EN_R_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(s_htim, s_chan_left,  0);
    __HAL_TIM_SET_COMPARE(s_htim, s_chan_right, 0);
    s_last_left  = 0;
    s_last_right = 0;
    s_enabled = 0;
}

void DrvMotor_Disable(void)
{
    if (s_htim == NULL) {
        return;
    }
    HAL_GPIO_WritePin(DRV_MOTOR_EN_L_PORT, DRV_MOTOR_EN_L_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV_MOTOR_EN_R_PORT, DRV_MOTOR_EN_R_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(s_htim, s_chan_left,  0);
    __HAL_TIM_SET_COMPARE(s_htim, s_chan_right, 0);
    s_last_left  = 0;
    s_last_right = 0;
    s_enabled = 0;
}

void DrvMotor_GetLastPWM(int16_t *left, int16_t *right)
{
    if (left)  *left  = s_last_left;
    if (right) *right = s_last_right;
}
