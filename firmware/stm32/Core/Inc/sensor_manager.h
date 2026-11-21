/**
 * @file    sensor_manager.h
 * @brief   Sensor fusion task: IMU, ultrasonic, battery, lidar scan and
 *          odometry. Publishes an atomic snapshot to consumers.
 */
#ifndef THUNDERS_SENSOR_MANAGER_H
#define THUNDERS_SENSOR_MANAGER_H

#include "drv_imu.h"
#include "drv_ultrasonic.h"
#include "drv_battery.h"
#include "drv_lidar.h"
#include "encoder.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_SCAN_SECTORS 16U

typedef struct
{
    ImuState           imu;
    uint16_t           distanceMm[DRV_ULTRASONIC_COUNT];
    uint16_t           minFrontMm;
    BatteryState       battery;
    Odometry           odom;
    uint8_t            scanSectorCm[SENSOR_SCAN_SECTORS];  /* lidar summary */
    bool               scanValid;
    uint32_t           timestampMs;
    uint32_t           snapshotSeq;
} SensorSnapshot;

void SensorManager_Init(void);
/** Free-running task body (called by TaskSensors at SENSOR_TASK_HZ). */
void SensorManager_Task(void);
/** Thread-safe copy of the latest snapshot. */
bool SensorManager_GetSnapshot(SensorSnapshot *out);
/** Raw 360-bin scan for the communication task (lidar polar map). */
bool SensorManager_TakeScan(uint16_t *distanceMm, uint8_t *quality);

/* Safety hooks consumed by SafetyManager */
float SensorManager_GetTiltDeg(void);
uint16_t SensorManager_GetMinFrontMm(void);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_SENSOR_MANAGER_H */
