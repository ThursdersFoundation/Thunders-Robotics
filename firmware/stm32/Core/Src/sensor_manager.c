/**
 * @file    sensor_manager.c
 * @brief   Sensor task implementation at 200 Hz with staggered subtasks:
 *   every cycle : IMU read + odometry integration
 *   every 5th   : ultrasonic trigger (40 Hz round-robin)
 *   every 2nd   : battery update (100 Hz)
 *   every 20th  : lidar scan harvest + sector summary (10 Hz)
 */
#include "sensor_manager.h"
#include "safety_manager.h"
#include "FreeRTOS.h"
#include "adc.h"
#include "drv_imu.h"
#include "drv_ultrasonic.h"
#include "drv_battery.h"
#include "drv_lidar.h"
#include "drv_encoder.h"
#include "main.h"
#include <string.h>
#include <math.h>

static SensorSnapshot s_snapshot;
static volatile uint32_t s_seq = 0U;

static void PublishSnapshot(void)
{
    taskENTER_CRITICAL();
    s_snapshot.snapshotSeq++;
    taskEXIT_CRITICAL();
}

void SensorManager_Init(void)
{
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    for (uint16_t i = 0U; i < LIDAR_SCAN_BINS; i++)
    {
        /* placeholder init handled inside the lidar driver */
        (void)i;
    }

    DrvUltrasonic_Init();
    DrvBattery_Init(adc_raw);
    DrvLidar_Init();

    if (DrvImu_Init())
    {
        SafetyManager_ClearFault(FAULT_IMU_TIMEOUT);
    }
    else
    {
        SafetyManager_RaiseFault(FAULT_IMU_TIMEOUT, 0U);
    }

    DrvLidar_StartMotor();
    DrvLidar_StartScan();
    DrvLidar_RequestDeviceInfo();
    DrvLidar_RequestHealth();
}

void SensorManager_Task(void)
{
    static uint32_t cycle = 0U;
    cycle++;

    /* 1) IMU + odometry at full rate */
    DrvImu_Update();
    const ImuState *imu = DrvImu_GetState();
    SafetyManager_Heartbeat(HEARTBEAT_ID_SENSORS);

    float wheelSpeed[4];
    for (uint8_t i = 0U; i < 4U; i++)
    {
        const EncoderState *enc = DrvEncoder_GetState(i);
        wheelSpeed[i] = enc ? enc->radPerSec * ROBOT_WHEEL_RADIUS_M : 0.0f;
    }
    Odometry_Update(wheelSpeed, imu->yawDeg * 0.01745329f, 0.005f);

    /* 2) Battery at 100 Hz */
    if ((cycle & 1U) == 0U)
    {
        DrvBattery_Update();
    }

    /* 3) Ultrasonic trigger at 40 Hz (one sensor per call) */
    if ((cycle % 5U) == 0U)
    {
        DrvUltrasonic_TriggerNext();
    }

    /* 4) Lidar harvest at 10 Hz */
    if ((cycle % 20U) == 0U)
    {
        uint16_t dist[LIDAR_SCAN_BINS];
        uint8_t qual[LIDAR_SCAN_BINS];
        if (DrvLidar_TakeScan(dist, qual))
        {
            /* 16-sector minimum distance summary (sector 0 = +x forward) */
            for (uint8_t s = 0U; s < SENSOR_SCAN_SECTORS; s++)
            {
                float a0 = (float)s * (360.0f / SENSOR_SCAN_SECTORS);
                uint16_t minMm = DrvLidar_MinDistanceInSector(a0, a0 + 22.5f);
                s_snapshot.scanSectorCm[s] = (minMm == LIDAR_INVALID_MM)
                    ? 255U : (uint8_t)(minMm / 10U);
            }
            s_snapshot.scanValid = true;
        }

        const LidarStateData *lidar = DrvLidar_GetState();
        if (lidar->health == LIDAR_HEALTH_ERROR)
        {
            SafetyManager_RaiseFault(FAULT_LIDAR_ERROR, lidar->healthErrorCode);
        }
    }

    /* 5) Refresh the published snapshot */
    taskENTER_CRITICAL();
    s_snapshot.imu = *imu;
    for (uint8_t i = 0U; i < DRV_ULTRASONIC_COUNT; i++)
    {
        const UltrasonicState *us = DrvUltrasonic_GetState((UltrasonicId)i);
        s_snapshot.distanceMm[i] = us->valid ? us->distanceMm : 0xFFFFU;
    }
    s_snapshot.minFrontMm = DrvUltrasonic_MinFrontDistance();
    s_snapshot.battery = *DrvBattery_GetState();
    s_snapshot.odom = *Odometry_Get();
    s_snapshot.timestampMs = HAL_GetTick();
    taskEXIT_CRITICAL();
    PublishSnapshot();

    /* 6) Forward safety-relevant signals */
    SafetyManager_ReportBattery(s_snapshot.battery.voltageMv, s_snapshot.battery.currentMa);
    SafetyManager_ReportTilt(imu->rollDeg, imu->pitchDeg);
    SafetyManager_ReportObstacle(s_snapshot.minFrontMm);
}

bool SensorManager_GetSnapshot(SensorSnapshot *out)
{
    if (out == NULL) { return false; }
    taskENTER_CRITICAL();
    *out = s_snapshot;
    taskEXIT_CRITICAL();
    return true;
}

bool SensorManager_TakeScan(uint16_t *distanceMm, uint8_t *quality)
{
    return DrvLidar_TakeScan(distanceMm, quality);
}

float SensorManager_GetTiltDeg(void)
{
    float roll = fabsf(s_snapshot.imu.rollDeg);
    float pitch = fabsf(s_snapshot.imu.pitchDeg);
    return (roll > pitch) ? roll : pitch;
}

uint16_t SensorManager_GetMinFrontMm(void)
{
    return s_snapshot.minFrontMm;
}
