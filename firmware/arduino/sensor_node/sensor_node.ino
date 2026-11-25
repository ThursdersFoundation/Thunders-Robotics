/**
 * @file    sensor_node.ino
 * @brief   Thunders Robotic sensor node (CAN node 2).
 *
 * Publishes: distances (2x HC-SR04), IMU (MPU6050 on Wire), power estimate.
 * Sensor cadence: ultrasonic 20 ms round-robin, IMU 10 ms, heartbeat 100 ms.
 */
#include <Wire.h>
#include "RobotCommon.h"
#include "src/usonic_pair.h"
#include "src/imu_mpu6050.h"
#include "src/can_interface.h"

static UsonicPair s_usonic;
static ImuMpu6050 s_imu;
static CanInterface s_can;

static uint32_t s_lastUsMs = 0;
static uint32_t s_lastImuMs = 0;
static uint32_t s_lastBeatMs = 0;
static uint32_t s_bootMs = 0;
static uint8_t  s_state = 1;    /* 1 = running */

void setup()
{
    Serial.begin(115200UL);
    Wire.begin();
    Wire.setClock(400000UL);

    s_usonic.begin(22, 23, 24, 25);   /* trig1, echo1, trig2, echo2 */
    s_imu.begin();
    s_can.begin(CAN_NODE_SENSOR);
    s_bootMs = millis();
}

void loop()
{
    uint32_t now = millis();

    if ((now - s_lastImuMs) >= 10U)
    {
        s_lastImuMs = now;
        s_imu.update(0.010f);
        float dps[3];
        s_imu.gyroDps(dps);
        uint8_t buf[8];
        (void)Can_EncodeImu(buf, s_imu.yawDeg(), dps);
        s_can.send(CAN_ID_SENSOR_IMU, buf, 8U);
    }

    if ((now - s_lastUsMs) >= 20U)
    {
        s_lastUsMs = now;
        s_usonic.triggerNext();
        uint16_t mm[4] = { s_usonic.distanceMm(0), s_usonic.distanceMm(1),
                           0xFFFFU, 0xFFFFU };
        uint8_t buf[8];
        (void)Can_EncodeDistances(buf, mm);
        s_can.send(CAN_ID_SENSOR_DISTANCES, buf, 8U);
    }

    if ((now - s_lastBeatMs) >= 100U)
    {
        s_lastBeatMs = now;
        s_can.sendHeartbeat(s_state, (uint16_t)((now - s_bootMs) / 1000U), 0U);
        digitalWrite(LED_BUILTIN, (now / 500U) & 1U);
    }
}
