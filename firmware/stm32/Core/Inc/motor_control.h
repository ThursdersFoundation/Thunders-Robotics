/**
 * @file    motor_control.h
 * @brief   Omni-drive (4-wheel X configuration) velocity controller.
 *
 * Architecture:
 *   SetVelocity(vx, vy, wz)          m/s, m/s, rad/s (robot frame)
 *     -> kinematics -> per-wheel target speeds (m/s)
 *     -> per-wheel PI speed loop @ 1 kHz (rad/s wheel shaft)
 *     -> DrvMotor_SetDuty with slew limiting
 *
 * Wheel arrangement (top view, +x forward, +y left, CCW positive):
 *
 *        y
 *        ^    FL (+45)   FR (-45)
 *        |      \         /
 *        |       \       /
 *        |  RL (135)    RR (225/-135)
 *        +--------------> x
 *
 * Inverse kinematics (L = Lx + Ly, wheel rolling directions along the
 * diagonals, s = 1/sqrt(2)):
 *   wFL = ( vx + vy + wz*L ) * s
 *   wFR = ( vx - vy - wz*L ) * s
 *   wRL = ( vx - vy + wz*L ) * s
 *   wRR = ( vx + vy - wz*L ) * s
 * (per-motor mounting signs are folded into MOTOR_SIGN[].)
 */
#ifndef THUNDERS_MOTOR_CONTROL_H
#define THUNDERS_MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_INDEX_COUNT 4U

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integMax;      /* anti-windup clamp on the integrator (duty units) */
} PidGains;

typedef struct
{
    PidGains gains;
    float integ;
    float prevMeasurement;
    float outMin;
    float outMax;
    bool  firstCycle;
} PidController;

/** Pure kinematics: chassis twist -> wheel speeds (m/s, wheel surface). */
void MotorControl_ComputeWheelTargets(float vx, float vy, float wz, float wheelSpeed[4]);

/** Pure PID step (derivative on measurement, clamped integrator). */
float Pid_Update(PidController *pid, float measurement, float setpoint, float dtSec);

void MotorControl_Init(void);
/** Enable output (safe state must be clear). */
bool MotorControl_Enable(void);
/** Disable: brake all motors, keep PID state. */
void MotorControl_Disable(void);
/** Command a chassis twist (only applied while enabled). */
void MotorControl_SetVelocity(float vx, float vy, float wz);
/** Stop command source: zero the twist. */
void MotorControl_ZeroVelocity(void);
/** Emergency: brake immediately and disarm (requires SafetyManager clear). */
void MotorControl_EmergencyStop(void);
/** Re-apply current command (used when leaving safe stop). */
bool MotorControl_Recover(void);
bool MotorControl_IsEnabled(void);
void MotorControl_SetPid(uint8_t wheel, const PidGains *gains);
const PidGains *MotorControl_GetDefaultPid(void);

/** 1 kHz control tick: sample encoders, run PIDs, update motor outputs. */
void MotorControl_ControlTick(void);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_MOTOR_CONTROL_H */
