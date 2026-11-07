/**
 * @file drv_lidar.c
 * @brief Slamtec RPLIDAR A1 driver.  Reads the 5-byte response descriptor and
 *        the 360 sample-packet stream over UART4 at 460800 baud.  Each sample
 *        is encoded in 5 bytes: a quality byte, a starting-flag + angle MSB,
 *        an angle LSB, a distance check bit + distance LSB, and a distance
 *        MSB.  The 360 samples per descriptor are aggregated into a
 *        LidarFrame_t distance table indexed by bearing degree.
 */
#include "drv_lidar.h"

#include <string.h>

static UART_HandleTypeDef *s_huart = NULL;
static LidarFrame_t        s_frame;
static uint8_t             s_rxbuf[64];
static uint8_t             s_inited = 0;

/* RPLIDAR commands. */
static const uint8_t CMD_STOP[] = { 0xA5, 0x25 };
static const uint8_t CMD_SCAN[] = { 0xA5, 0x20 };
static const uint8_t CMD_RESET[] = { 0xA5, 0x40 };

HAL_StatusTypeDef DrvLidar_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return HAL_ERROR;
    }
    s_huart = huart;
    memset(&s_frame, 0, sizeof(s_frame));

    HAL_UART_Transmit(s_huart, (uint8_t *)CMD_STOP,  sizeof(CMD_STOP),  50);
    HAL_Delay(1);
    HAL_UART_Transmit(s_huart, (uint8_t *)CMD_RESET, sizeof(CMD_RESET), 50);
    HAL_Delay(50);
    HAL_UART_Transmit(s_huart, (uint8_t *)CMD_SCAN,  sizeof(CMD_SCAN),  50);
    HAL_Delay(10);

    s_inited = 1;
    return HAL_OK;
}

static uint8_t descriptor_checksum(const uint8_t *d)
{
    /* 5-byte descriptor checksum: XOR all bytes except the checksum byte
     * with the bits in byte 0 Q6. */
    uint8_t cs = 0U;
    for (uint8_t i = 0; i < 4; i++) cs ^= d[i];
    cs ^= (uint8_t)((d[0] & 0xC0U) >> 6);  /* not standard; included for parity */
    return cs;
}

static uint8_t sample_checksum(const uint8_t *s)
{
    /* RPLIDAR A1 sample checksum is computed over the 4 data bytes. */
    uint32_t cs = ((uint32_t)s[0] >> 2) +
                  ((uint32_t)s[1] << 4) +
                  ((uint32_t)s[2] << 4) +
                  ((uint32_t)s[3] << 4);
    cs ^= ((uint32_t)s[0] & 0x03U);
    return (uint8_t)(cs & 0xFFU);
}

HAL_StatusTypeDef DrvLidar_ReadFrame(LidarFrame_t *out)
{
    if (!s_inited || out == NULL) {
        return HAL_ERROR;
    }

    /* Wait for the SYNC descriptor (Q6 = 0xA1). */
    uint8_t desc[5];
    uint8_t sync = 0;
    uint32_t start = HAL_GetTick();
    while (sync != 0xA5U) {
        if (HAL_UART_Receive(s_huart, &sync, 1, 5) != HAL_OK) {
            if ((HAL_GetTick() - start) > 200U) {
                return HAL_TIMEOUT;
            }
            continue;
        }
    }
    if (HAL_UART_Receive(s_huart, &desc[1], 4, 50) != HAL_OK) {
        return HAL_ERROR;
    }
    desc[0] = 0xA5;
    if (descriptor_checksum(desc) != desc[4]) {
        return HAL_ERROR;
    }

    /* The RPLIDAR A1 descriptor Q6 nibble is 0xA1. */
    if ((desc[0] & 0xC0U) != 0xA0U) {
        return HAL_ERROR;
    }
    uint16_t samples = (uint16_t)(desc[1] | ((uint16_t)(desc[2] & 0x3FU) << 8));
    if (samples == 0U || samples > 360U) {
        samples = 360U;  /* clamp */
    }
    s_frame.start_angle_raw = (uint32_t)((desc[3] | ((uint32_t)(desc[4] & 0x0FU) << 8)) << 1);
    s_frame.end_angle_raw   = 0U;

    for (uint16_t i = 0; i < samples; i++) {
        if (HAL_UART_Receive(s_huart, s_rxbuf, 5, 50) != HAL_OK) {
            return HAL_ERROR;
        }
        if (sample_checksum(s_rxbuf) != s_rxbuf[4]) {
            continue;
        }
        uint16_t angle_q2 = (uint16_t)(((s_rxbuf[1] >> 1) | ((uint32_t)s_rxbuf[2] << 7)) & 0x7FFFU);
        uint16_t dist_q2  = (uint16_t)(((s_rxbuf[3] >> 1) | ((uint32_t)s_rxbuf[4] << 7)) & 0xFFFFU);
        float angle_deg = (float)angle_q2 * 90.0f / 16384.0f;
        float dist_m    = (float)dist_q2  / 4.0f / 1000.0f;
        if (dist_m > DRV_LIDAR_MAX_RANGE_M) {
            dist_m = 0.0f;
        }
        uint16_t idx = (uint16_t)(angle_deg) % DRV_LIDAR_ANGLE_BUCKETS;
        s_frame.distance_m[idx] = dist_m;
        s_frame.quality[idx]    = s_rxbuf[0];
        s_frame.end_angle_raw    = angle_q2;
    }
    s_frame.complete = 1;
    s_frame.frame_count++;
    *out = s_frame;
    return HAL_OK;
}

HAL_StatusTypeDef DrvLidar_GetDistance(uint16_t angle_deg, float *dist_m)
{
    if (dist_m == NULL) {
        return HAL_ERROR;
    }
    if (!s_frame.complete) {
        return HAL_ERROR;
    }
    uint16_t idx = angle_deg % DRV_LIDAR_ANGLE_BUCKETS;
    if (s_frame.distance_m[idx] == 0.0f) {
        return HAL_ERROR;
    }
    *dist_m = s_frame.distance_m[idx];
    return HAL_OK;
}

HAL_StatusTypeDef DrvLidar_Stop(void)
{
    if (!s_inited) {
        return HAL_ERROR;
    }
    return HAL_UART_Transmit(s_huart, (uint8_t *)CMD_STOP, sizeof(CMD_STOP), 50);
}
