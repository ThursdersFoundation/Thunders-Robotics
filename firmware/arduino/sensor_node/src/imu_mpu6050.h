/**
 * @file    imu_mpu6050.h
 * @brief   Minimal MPU6050 driver on Wire (yaw integration + gyro bias).
 */
#ifndef SENSOR_NODE_IMU_H
#define SENSOR_NODE_IMU_H
#include <stdint.h>
#include <Wire.h>

class ImuMpu6050
{
public:
    bool begin();
    void update(float dtSec);
    float yawDeg() const { return m_yaw; }
    void gyroDps(float out[3]) const;
private:
    bool writeReg(uint8_t reg, uint8_t val);
    bool readBurst(uint8_t reg, uint8_t *buf, uint8_t len);
    float m_bias[3];
    float m_gyro[3];
    float m_yaw = 0.0f;
    bool  m_ok = false;
};
#endif /* SENSOR_NODE_IMU_H */
