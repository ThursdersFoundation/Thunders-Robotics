/**
 * @file drv_imu.h
 * @brief Bosch BNO055 IMU driver over I2C.  Provides fused Euler-angle and
 *        accelerometer / gyroscope readings, plus an in-place calibration
 *        routine.  This file also defines the canonical `ImuData_t` struct
 *        used throughout the firmware.
 */
#ifndef DRV_IMU_H
#define DRV_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* -------------------------------------------------------------------------
 * BNO055 I2C register addresses.
 * ------------------------------------------------------------------------- */
#define BNO055_PAGE_ID          0x07U
#define BNO055_SYS_TRIGGER      0x3FU
#define BNO055_PWR_MODE         0x3EU
#define BNO055_OPR_MODE         0x3DU
#define BNO055_UNIT_SEL         0x3BU
#define BNO055_SYS_STATUS       0x39U
#define BNO055_SYS_ERROR        0x3AU
#define BNO055_CALIB_STAT       0x35U
#define BNO055_EUL_HEADING_L    0x1AU  /* Euler angles, LSB first */
#define BNO055_LIA_DATA_X_LSB   0x28U  /* Linear acceleration */
#define BNO055_GYR_DATA_X_LSB   0x14U  /* Gyroscope */
#define BNO055_OPR_MODE_NDOF    0x0CU  /* 9-axis fusion output */
#define BNO055_OPR_MODE_CONFIG  0x00U

/** @brief Fused IMU reading in m/s^2, deg, dps.  Origin of ImuData_t. */
typedef struct
{
    float    yaw;          /* Euler yaw   in degrees, 0..360            */
    float    pitch;        /* Euler pitch in degrees, -180..+180        */
    float    roll;         /* Euler roll  in degrees, -90..+90           */
    float    ax;           /* Linear accel X in m/s^2                   */
    float    ay;           /* Linear accel Y in m/s^2                   */
    float    az;           /* Linear accel Z in m/s^2                   */
    float    gx;           /* Gyroscope X in degrees per second         */
    float    gy;           /* Gyroscope Y in degrees per second         */
    float    gz;           /* Gyroscope Z in degrees per second         */
    bool     calibrated;   /* 1 if calibration status is 0xFF            */
    uint32_t tick_ms;     /* HAL_GetTick() at the last read             */
} ImuData_t;

/**
 * @brief Initialise the BNO055.  Issues a system reset, configures the unit
 *        selection (m/s^2, degrees, degrees/s), and switches to NDOF mode.
 * @param hi2c  I2C bus handle (I2C1).
 * @return HAL_OK on success, HAL_ERROR on comms failure.
 */
HAL_StatusTypeDef DrvIMU_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Read the latest fused IMU data and raw gyro from the BNO055.
 * @param out  Pointer to an ImuData_t filled in by the call.
 * @return HAL_OK on success, HAL_ERROR on comms failure.
 */
HAL_StatusTypeDef DrvIMU_Read(ImuData_t *out);

/**
 * @brief Block until the BNO055 reports all four calibration statuses fully
 *        calibrated, or until the timeout expires.
 * @param timeout_ms  Maximum wait time in milliseconds.
 * @return HAL_OK if fully calibrated, HAL_TIMEOUT if the timeout expired.
 */
HAL_StatusTypeDef DrvIMU_Calibrate(void);

/**
 * @brief Read the raw 8-bit calibration status byte from the BNO055.
 * @param out  Output pointer; bit layout: SYS(7:6) GYR(5:4) ACC(3:2) MAG(1:0).
 * @return HAL_OK on success.
 */
HAL_StatusTypeDef DrvIMU_ReadCalibration(uint8_t *out);

#ifdef __cplusplus
}
#endif

#endif /* DRV_IMU_H */
