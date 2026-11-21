/**
 * @file    encoder.c
 * @brief   Odometry integration (pure math + driver access).
 *
 * Forward kinematics (inverse of the X-drive mapping):
 *   vx = ( wFL + wFR + wRL + wRR ) / 2 ... folded by the 1/sqrt(2) factor:
 *   vx = ( wFL + wFR + wRL + wRR ) * s / 2?  -- see derivation below.
 *
 * From the inverse map w = K * u with u = [vx, vy, w*L]:
 *   FL = s( vx + vy + wL ), FR = s( vx - vy - wL ),
 *   RL = s( vx - vy + wL ), RR = s( vx + vy - wL ),  s = 1/sqrt(2)
 * => vx = (FL + FR + RL + RR) / (4s)? No:
 *   FL + FR + RL + RR = s*4vx + s*(vy-vy-vy+vy) + s*(wL -wL +wL -wL) = 4 s vx
 *   FL - FR - RL + RR = 4 s vy
 *   FL - FR + RL - RR = 4 s wL
 * So vx = sum / (4s) = sum * sqrt(2)/2 / ... i.e. sum * s/ (4 s^2) = sum * s / 2.
 */
#include "encoder.h"
#include "motor_control.h"
#include "main.h"   /* KINE_L, wheel order */
#include <math.h>
#include <string.h>

#define ODOM_DT 0.005f

static Odometry s_odom;

void Odometry_ComputeTwist(const float wheelSpeed[4], float *vx, float *vy, float *wz)
{
    const float s = 0.70710678f;   /* 1/sqrt(2) */
    float sum   = wheelSpeed[0] + wheelSpeed[1] + wheelSpeed[2] + wheelSpeed[3];
    float diffY = wheelSpeed[0] - wheelSpeed[1] - wheelSpeed[2] + wheelSpeed[3];
    /* rotation lives in the rear pair vs front pair (X-drive convention) */
    float diffW = (wheelSpeed[2] + wheelSpeed[3]) - (wheelSpeed[0] + wheelSpeed[1]);

    if (vx) { *vx = sum * s / 2.0f; }
    if (vy) { *vy = diffY * s / 2.0f; }
    if (wz) { *wz = diffW * s / 2.0f / (ROBOT_WHEELBASE_X_M + ROBOT_WHEELBASE_Y_M); }
}

void Odometry_Init(void)
{
    memset(&s_odom, 0, sizeof(s_odom));
}

void Odometry_Update(const float wheelSpeed[4], float yawRad, float dtSec)
{
    float vxBody, vyBody, wzBody;
    Odometry_ComputeTwist(wheelSpeed, &vxBody, &vyBody, &wzBody);

    s_odom.vx = vxBody;
    s_odom.vy = vyBody;
    s_odom.wz = wzBody;

    /* Heading: IMU yaw preferred (absolute, no slip error) */
    s_odom.theta = (double)yawRad;

    /* Body -> world velocity using the fused yaw */
    float cosT = cosf((float)s_odom.theta);
    float sinT = sinf((float)s_odom.theta);
    s_odom.x += (double)(vxBody * cosT - vyBody * sinT) * dtSec;
    s_odom.y += (double)(vxBody * sinT + vyBody * cosT) * dtSec;
    s_odom.updateCount++;
}

const Odometry *Odometry_Get(void)
{
    return &s_odom;
}

void Odometry_Reset(void)
{
    double x = s_odom.x, y = s_odom.y;
    memset(&s_odom, 0, sizeof(s_odom));
    s_odom.x = x;
    s_odom.y = y;
}
