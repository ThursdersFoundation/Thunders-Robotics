/**
 * @file drv_lidar.h
 * @brief Slamtec RPLIDAR A1 driver.  Receives scan frames over UART4 at
 *        460800 baud, validates the descriptor / data byte checksums, and
 *        publishes a per-angle distance table.  This file also defines the
 *        canonical `LidarFrame_t` struct used by the sensor manager.
 */
#ifndef DRV_LIDAR_H
#define DRV_LIDAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* -------------------------------------------------------------------------
 * RPLIDAR A1 protocol constants.
 * ------------------------------------------------------------------------- */
#define DRV_LIDAR_ANGLE_BUCKETS  360U    /* 1 bucket per degree of bearing  */
#define DRV_LIDAR_SYNC_Q6        0xA1U   /* start byte for the descriptor     */
#define DRV_LIDAR_QUALITY_MAX    255U
#define DRV_LIDAR_MAX_RANGE_M     6.0f

/** @brief A single 360-degree scan with per-degree distance and quality. */
typedef struct
{
    float    distance_m[DRV_LIDAR_ANGLE_BUCKETS]; /* 0 if no return for that angle */
    uint8_t  quality[DRV_LIDAR_ANGLE_BUCKETS];     /* 0 = no return                */
    uint32_t start_angle_raw;                       /* Q6 descriptor start angle  */
    uint32_t end_angle_raw;                         /* Q6 descriptor end angle    */
    uint32_t frame_count;                           /* monotonic frame index        */
    bool     complete;                              /* set when one full scan done */
} LidarFrame_t;

/**
 * @brief Initialise the RPLIDAR driver.  Issues a STOP + RESET command
 *        and clears the per-angle distance table.
 * @param huart  Handle to UART4 (PC10/PC11, 460800 baud).
 * @return HAL_OK on success, HAL_ERROR on null pointer.
 */
HAL_StatusTypeDef DrvLidar_Init(UART_HandleTypeDef *huart);

/**
 * @brief Parse one complete frame from the UART4 RX ring buffer into the
 *        given LidarFrame_t.  Returns when the next SYNC descriptor is
 *        encountered, marking the start of the next frame.
 * @param out  Output pointer.
 * @return HAL_OK on success, HAL_ERROR on UART/parse failure.
 */
HAL_StatusTypeDef DrvLidar_ReadFrame(LidarFrame_t *out);

/**
 * @brief Look up the distance at the given bearing.
 * @param angle_deg  Bearing in degrees (0..359).
 * @param dist_m    Output pointer.
 * @return HAL_OK on success, HAL_ERROR if no return at that angle.
 */
HAL_StatusTypeDef DrvLidar_GetDistance(uint16_t angle_deg, float *dist_m);

/**
 * @brief Send an RPLIDAR STOP command (0xA5 0x25) and wait for any
 *        in-flight bytes to flush.
 * @return HAL_OK on success.
 */
HAL_StatusTypeDef DrvLidar_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* DRV_LIDAR_H */
