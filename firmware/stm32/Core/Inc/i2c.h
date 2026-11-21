/**
 * @file    i2c.h
 * @brief   I2C1 master @ 400 kHz: MPU6050 IMU (PB8/PB9).
 */
#ifndef THUNDERS_I2C_H
#define THUNDERS_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void MX_I2C1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_I2C_H */
