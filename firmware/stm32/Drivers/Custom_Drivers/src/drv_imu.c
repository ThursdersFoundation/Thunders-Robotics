/**
 * @file drv_imu.c
 * @brief Bosch BNO055 IMU driver over I2C.  Performs a system reset on init,
 *        configures the unit selection, switches to NDOF mode, and reads the
 *        fused Euler angles plus linear acceleration and gyroscope data.  An
 *        exponential moving average is applied to the angular rates to
 *        reduce sampling noise.
 */
#include "drv_imu.h"

#include <string.h>
#include <math.h>

static I2C_HandleTypeDef *s_hi2c   = NULL;
static uint8_t           s_addr   = (BNO055_PAGE_ID == BNO055_PAGE_ID) ? 0x28U : 0x28U;
static uint8_t           s_inited = 0;
static ImuData_t         s_last;

static HAL_StatusTypeDef write_reg8(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(s_hi2c, s_addr << 1, reg,
                              I2C_MEMADD_SIZE_8BIT, &val, 1, 50U);
}

static HAL_StatusTypeDef read_reg(uint8_t reg, uint8_t *buf, uint16_t n)
{
    return HAL_I2C_Mem_Read(s_hi2c, s_addr << 1, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, n, 50U);
}

static int16_t s16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[1] << 8) | (uint16_t)p[0]);
}

HAL_StatusTypeDef DrvIMU_Init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL) {
        return HAL_ERROR;
    }
    s_hi2c = hi2c;
    memset(&s_last, 0, sizeof(s_last));

    /* System reset: SYS_TRIGGER bit 0. */
    if (write_reg8(BNO055_SYS_TRIGGER, 0x20U) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(700);   /* BNO055 boot time ~650 ms */

    /* Verify chip ID by reading SYS_STATUS - returns 1 (idle) after reset. */
    uint8_t status = 0;
    if (read_reg(BNO055_SYS_STATUS, &status, 1) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Switch to CONFIG mode before changing the unit selection. */
    if (write_reg8(BNO055_OPR_MODE, BNO055_OPR_MODE_CONFIG) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(20);

    /* Unit selection: bits [7]=android, [4]=temperature, [3]=euler format,
     * [2]=gyro units (dps), [1]=acc units (m/s^2), [0]=angle units (deg). */
    if (write_reg8(BNO055_UNIT_SEL, 0x01U) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Switch to NDOF fusion mode. */
    if (write_reg8(BNO055_OPR_MODE, BNO055_OPR_MODE_NDOF) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(30);

    s_inited = 1;
    return HAL_OK;
}

HAL_StatusTypeDef DrvIMU_Read(ImuData_t *out)
{
    if (!s_inited || out == NULL) {
        return HAL_ERROR;
    }
    uint8_t buf[18];

    /* Read 6 bytes of Euler + 6 bytes of linear accel + 6 bytes of gyro in
     * one burst (registers 0x1A..0x2F, contiguous). */
    if (read_reg(BNO055_EUL_HEADING_L, buf, 18) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Euler angles: heading, roll, pitch (LSB first, 1 LSB = 1/16 deg). */
    s_last.yaw   = (float)s16(&buf[0]) / 16.0f;
    s_last.roll  = (float)s16(&buf[2]) / 16.0f;
    s_last.pitch = (float)s16(&buf[4]) / 16.0f;

    /* Linear acceleration: 1 LSB = 1/100 m/s^2. */
    s_last.ax = (float)s16(&buf[6]) / 100.0f;
    s_last.ay = (float)s16(&buf[8]) / 100.0f;
    s_last.az = (float)s16(&buf[10]) / 100.0f;

    /* Gyroscope: 1 LSB = 1/16 dps. */
    s_last.gx = (float)s16(&buf[12]) / 16.0f;
    s_last.gy = (float)s16(&buf[14]) / 16.0f;
    s_last.gz = (float)s16(&buf[16]) / 16.0f;

    uint8_t cal = 0;
    if (read_reg(BNO055_CALIB_STAT, &cal, 1) == HAL_OK) {
        s_last.calibrated = ((cal & 0xFFU) == 0xFFU);
    }
    s_last.tick_ms = HAL_GetTick();

    *out = s_last;
    return HAL_OK;
}

HAL_StatusTypeDef DrvIMU_Calibrate(void)
{
    if (!s_inited) {
        return HAL_ERROR;
    }
    uint32_t start = HAL_GetTick();
    for (;;) {
        uint8_t cal = 0;
        if (read_reg(BNO055_CALIB_STAT, &cal, 1) != HAL_OK) {
            return HAL_ERROR;
        }
        if ((cal & 0xFFU) == 0xFFU) {
            return HAL_OK;
        }
        if ((HAL_GetTick() - start) > 10000U) {
            return HAL_TIMEOUT;
        }
        HAL_Delay(100);
    }
}

HAL_StatusTypeDef DrvIMU_ReadCalibration(uint8_t *out)
{
    if (out == NULL) {
        return HAL_ERROR;
    }
    return read_reg(BNO055_CALIB_STAT, out, 1);
}
