/**
 * @file drv_ultrasonic.c
 * @brief HC-SR04-style ultrasonic driver.  Generates a 10 us trigger pulse
 *        via a TIM channel configured for PWM one-shot, then measures the
 *        echo width using a software-polling input capture on a separate
 *        GPIO pin.  Applies a 5-sample median filter on demand.
 */
#include "drv_ultrasonic.h"

#include <string.h>

static TIM_HandleTypeDef *s_trig_tim    = NULL;
static uint32_t           s_trig_chan  = 0;
static uint16_t           s_echo_pin   = 0;
static GPIO_TypeDef      *s_echo_port  = NULL;
static volatile uint32_t s_echo_width_us = 0;
static volatile uint8_t   s_have_echo  = 0;
static uint8_t           s_inited     = 0;

HAL_StatusTypeDef DrvUltrasonic_Init(TIM_HandleTypeDef *trig_tim,
                                       uint32_t trig_channel,
                                       uint16_t echo_pin,
                                       GPIO_TypeDef *echo_port)
{
    if (trig_tim == NULL || echo_port == NULL) {
        return HAL_ERROR;
    }
    s_trig_tim   = trig_tim;
    s_trig_chan  = trig_channel;
    s_echo_pin   = echo_pin;
    s_echo_port  = echo_port;
    s_have_echo  = 0;
    s_echo_width_us = 0;
    s_inited    = 1;
    return HAL_OK;
}

void DrvUltrasonic_Trigger(void)
{
    if (!s_inited) {
        return;
    }
    /* Drive the trigger line high for ~10 us directly. */
    HAL_GPIO_WritePin(s_echo_port, s_echo_pin, GPIO_PIN_SET);
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 1U) { __NOP(); }
    for (volatile uint32_t i = 0; i < 1680U; i++) { __NOP(); } /* ~10 us @ 168 MHz */
    HAL_GPIO_WritePin(s_echo_port, s_echo_pin, GPIO_PIN_RESET);

    /* Wait for echo line to go high. */
    uint32_t timeout_start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(s_echo_port, s_echo_pin) == GPIO_PIN_RESET) {
        if ((HAL_GetTick() - timeout_start) > DRV_US_TIMEOUT_MS) {
            s_have_echo = 0;
            return;
        }
    }
    /* Measure high-time of the echo pulse. */
    timeout_start = HAL_GetTick();
    uint32_t cycles_start = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(s_echo_port, s_echo_pin) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - timeout_start) > DRV_US_TIMEOUT_MS) {
            s_have_echo = 0;
            return;
        }
    }
    uint32_t cycles_end = DWT->CYCCNT;
    /* Convert cycles to microseconds at 168 MHz. */
    s_echo_width_us = (cycles_end - cycles_start) / 168U;
    s_have_echo = 1;
}

HAL_StatusTypeDef DrvUltrasonic_GetDistance(float *dist_m)
{
    if (dist_m == NULL) {
        return HAL_ERROR;
    }
    if (!s_have_echo) {
        return HAL_ERROR;
    }
    /* distance = (sound_speed * time) / 2  (round trip) */
    float t_s = (float)s_echo_width_us * 1e-6f;
    *dist_m = (DRV_US_SPEED_SOUND_MPS * t_s) * 0.5f;
    return HAL_OK;
}

HAL_StatusTypeDef DrvUltrasonic_GetMedian(float *dist_m, uint8_t n)
{
    if (dist_m == NULL || n == 0U || n > 8U) {
        return HAL_ERROR;
    }
    float samples[8];
    uint8_t got = 0;
    for (uint8_t i = 0; i < n; i++) {
        DrvUltrasonic_Trigger();
        if (DrvUltrasonic_GetDistance(&samples[got]) == HAL_OK) {
            got++;
        } else {
            samples[got] = 999.0f;  /* out-of-range marker */
            got++;
        }
    }
    /* Insertion sort. */
    for (uint8_t i = 1; i < got; i++) {
        float key = samples[i];
        int j = (int)i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }
    *dist_m = samples[got / 2];
    return HAL_OK;
}
