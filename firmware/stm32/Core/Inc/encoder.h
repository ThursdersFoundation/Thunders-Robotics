/**
 * @file    encoder.h
 * @brief   Odometry application layer on top of the encoder driver:
 *          wheel-based pose integration fused with the IMU yaw.
 */
#ifndef THUNDERS_ENCODER_H
#define THUNDERS_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    double  x;            /* m, robot frame origin at start      */
    double  y;
    double  theta;        /* rad, IMU yaw when available         */
    float   vx;           /* m/s body frame                      */
    float   vy;
    float   wz;
    uint32_t updateCount;
} Odometry;

void Odometry_Init(void);
/**
 * Integrate one kinematic step (call at the sensor task rate, 200 Hz).
 * @param wheelSpeed  wheel surface speeds (m/s), kinematic order FL FR RL RR
 * @param yawRad      absolute yaw from the IMU (or gyro-integrated)
 */
void Odometry_Update(const float wheelSpeed[4], float yawRad, float dtSec);
const Odometry *Odometry_Get(void);
void Odometry_Reset(void);

/* Pure inverse kinematics for odometry: wheel speeds -> chassis twist */
void Odometry_ComputeTwist(const float wheelSpeed[4], float *vx, float *vy, float *wz);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_ENCODER_H */
