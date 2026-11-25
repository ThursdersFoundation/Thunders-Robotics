/** @file imu_mpu6050.cpp */
#include <Arduino.h>
#include "imu_mpu6050.h"

static const uint8_t kAddr = 0x68U << 1;

bool ImuMpu6050::writeReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(kAddr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0U;
}

bool ImuMpu6050::readBurst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(kAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0U) { return false; }
    if (Wire.requestFrom(kAddr, len) != len) { return false; }
    for (uint8_t i = 0U; i < len; i++) { buf[i] = Wire.read(); }
    return true;
}

bool ImuMpu6050::begin()
{
    uint8_t who = 0U;
    if (!readBurst(0x75U, &who, 1U) || who != 0x68U) { return false; }

    bool ok = true;
    ok &= writeReg(0x6BU, 0x00U);   /* PWR_MGMT_1: wake */
    ok &= writeReg(0x1AU, 0x03U);   /* DLPF 42 Hz */
    ok &= writeReg(0x1BU, 0x18U);   /* gyro +-2000 dps */
    if (!ok) { return false; }

    /* bias: 64 samples while stationary */
    int32_t acc[3] = { 0, 0, 0 };
    uint8_t raw[14];
    for (uint8_t n = 0U; n < 64U; n++)
    {
        if (readBurst(0x3BU, raw, 14U))
        {
            for (uint8_t a = 0U; a < 3U; a++)
            {
                acc[a] += (int16_t)(((uint16_t)raw[8 + 2U * a] << 8) | raw[9 + 2U * a]);
            }
        }
        delay(2);
    }
    for (uint8_t a = 0U; a < 3U; a++)
    {
        m_bias[a] = ((float)acc[a] / 64.0f) / 16.4f;
    }
    m_ok = true;
    return true;
}

void ImuMpu6050::update(float dtSec)
{
    if (!m_ok || dtSec <= 0.0f) { return; }
    uint8_t raw[14];
    if (!readBurst(0x3BU, raw, 14U)) { return; }
    for (uint8_t a = 0U; a < 3U; a++)
    {
        m_gyro[a] = (float)(int16_t)(((uint16_t)raw[8 + 2U * a] << 8) | raw[9 + 2U * a]) / 16.4f - m_bias[a];
    }
    m_yaw += m_gyro[2] * dtSec;
    if (m_yaw > 180.0f)       { m_yaw -= 360.0f; }
    else if (m_yaw < -180.0f) { m_yaw += 360.0f; }
}

void ImuMpu6050::gyroDps(float out[3]) const
{
    out[0] = m_gyro[0]; out[1] = m_gyro[1]; out[2] = m_gyro[2];
}
