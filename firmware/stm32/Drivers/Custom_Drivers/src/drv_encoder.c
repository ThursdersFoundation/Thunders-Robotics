/**
 * @file drv_encoder.c
 * @brief Low-level quadrature encoder driver.  Wraps TIM2 (left wheel) and
 *        TIM5 (right wheel) in encoder mode TI1+TI2 and reads the 32-bit CNT
 *        registers.  Overflow handling and RPM conversion live in the
 *        higher-level Core/Src/encoder.c module.
 */
#include "drv_encoder.h"

#include <string.h>

static TIM_HandleTypeDef *s_htim_left  = NULL;
static TIM_HandleTypeDef *s_htim_right = NULL;
static int32_t            s_last_left  = 0;
static int32_t            s_last_right = 0;
static uint8_t            s_inited    = 0;

HAL_StatusTypeDef DrvEncoder_Init(TIM_HandleTypeDef *htim_left,
                                   TIM_HandleTypeDef *htim_right)
{
    if (htim_left == NULL || htim_right == NULL) {
        return HAL_ERROR;
    }
    s_htim_left  = htim_left;
    s_htim_right = htim_right;
    s_last_left  = 0;
    s_last_right = 0;
    s_inited    = 1;
    DrvEncoder_Reset();
    return HAL_OK;
}

HAL_StatusTypeDef DrvEncoder_GetCounts(int32_t *left, int32_t *right)
{
    if (!s_inited || (left == NULL && right == NULL)) {
        return HAL_ERROR;
    }
    if (left) {
        *left = (int32_t)__HAL_TIM_GET_COUNTER(s_htim_left);
        s_last_left = *left;
    }
    if (right) {
        *right = (int32_t)__HAL_TIM_GET_COUNTER(s_htim_right);
        s_last_right = *right;
    }
    return HAL_OK;
}

HAL_StatusTypeDef DrvEncoder_GetRPM(uint32_t dt_ms, float *rpm_l, float *rpm_r)
{
    if (rpm_l == NULL && rpm_r == NULL) {
        return HAL_ERROR;
    }
    if (dt_ms == 0U) {
        if (rpm_l) *rpm_l = 0.0f;
        if (rpm_r) *rpm_r = 0.0f;
        return HAL_OK;
    }
    int32_t left_now, right_now;
    if (DrvEncoder_GetCounts(&left_now, &right_now) != HAL_OK) {
        return HAL_ERROR;
    }
    int32_t d_left  = left_now  - s_last_left;
    int32_t d_right = right_now - s_last_right;
    s_last_left  = left_now;
    s_last_right = right_now;

    /* rpm = (counts / counts_per_rev) / dt_s * 60. */
    const float counts_per_rev = 1024.0f;   /* matches ENC_COUNTS_PER_REV */
    float dt_s = (float)dt_ms * 0.001f;
    if (rpm_l) *rpm_l = ((float)d_left  / counts_per_rev) / dt_s * 60.0f;
    if (rpm_r) *rpm_r = ((float)d_right / counts_per_rev) / dt_s * 60.0f;
    return HAL_OK;
}

void DrvEncoder_Reset(void)
{
    if (!s_inited) {
        return;
    }
    __HAL_TIM_SET_COUNTER(s_htim_left,  0);
    __HAL_TIM_SET_COUNTER(s_htim_right, 0);
    s_last_left  = 0;
    s_last_right = 0;
}
