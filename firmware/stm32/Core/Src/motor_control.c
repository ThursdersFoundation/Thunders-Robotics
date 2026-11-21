/**
 * @file    motor_control.c
 * @brief   Omni-drive velocity control implementation.
 *
 * The kinematics and the PID are pure functions (unit tested on the host);
 * only ControlTick touches the hardware abstraction.
 */
#include "motor_control.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "main.h"
#include <math.h>
#include <string.h>

/* Geometry from main.h; s normalizes the 45-degree projection */
#define KINE_S            0.70710678f   /* 1/sqrt(2) */
#define KINE_L            (ROBOT_WHEELBASE_X_M + ROBOT_WHEELBASE_Y_M)
#define WHEEL_RADIUS      ROBOT_WHEEL_RADIUS_M
#define CONTROL_DT        0.001f

/* Mounting orientation of each motor relative to the kinematic convention */
static const float MOTOR_SIGN[4] = { 1.0f, -1.0f, -1.0f, 1.0f };

/* Default velocity PI gains for the 30:1 gearbox + 11 PPR encoders.
 * kp converts rad/s error into duty fraction; ki removes static friction
 * droop; kd left at 0 (noisy at 1 kHz with 1320 counts/rev). */
static const PidGains kDefaultGains = {
    .kp = 0.28f, .ki = 3.2f, .kd = 0.0f, .integMax = 0.6f
};

static PidController s_pid[4];
static PidGains      s_gains[4];
static float s_command[3] = { 0.0f, 0.0f, 0.0f };   /* vx, vy, wz */
static float s_wheelTarget[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
static bool  s_enabled = false;
static bool  s_estopLatched = false;

/* ------------------------------------------------------------------ */
/* Pure control math                                                   */
/* ------------------------------------------------------------------ */
void MotorControl_ComputeWheelTargets(float vx, float vy, float wz, float wheelSpeed[4])
{
    float l = KINE_L * wz;
    wheelSpeed[MOTOR_FRONT_LEFT]  = ( vx + vy - l ) * KINE_S;
    wheelSpeed[MOTOR_FRONT_RIGHT] = ( vx - vy - l ) * KINE_S;
    wheelSpeed[MOTOR_REAR_LEFT]   = ( vx - vy + l ) * KINE_S;
    wheelSpeed[MOTOR_REAR_RIGHT]  = ( vx + vy + l ) * KINE_S;
}

float Pid_Update(PidController *pid, float measurement, float setpoint, float dtSec)
{
    if (pid == NULL || dtSec <= 0.0f)
    {
        return 0.0f;
    }

    float error = setpoint - measurement;

    /* Derivative on measurement avoids setpoint kick */
    float deriv = pid->firstCycle ? 0.0f : -(measurement - pid->prevMeasurement) / dtSec;
    pid->firstCycle = false;
    pid->prevMeasurement = measurement;

    pid->integ += error * dtSec;
    if (pid->integ > pid->gains.integMax)  { pid->integ = pid->gains.integMax; }
    if (pid->integ < -pid->gains.integMax) { pid->integ = -pid->gains.integMax; }

    float out = pid->gains.kp * error + pid->gains.ki * pid->integ + pid->gains.kd * deriv;

    if (out > pid->outMax) { out = pid->outMax; }
    if (out < pid->outMin) { out = pid->outMin; }
    return out;
}

/* ------------------------------------------------------------------ */
/* Hardware edge                                                       */
/* ------------------------------------------------------------------ */
void MotorControl_Init(void)
{
    DrvMotor_Init();
    DrvEncoder_Init();

    for (uint8_t i = 0U; i < MOTOR_INDEX_COUNT; i++)
    {
        s_gains[i] = kDefaultGains;
        memset(&s_pid[i], 0, sizeof(PidController));
        s_pid[i].gains = kDefaultGains;
        s_pid[i].outMin = -1.0f;
        s_pid[i].outMax =  1.0f;
        s_pid[i].firstCycle = true;
    }
    memset(s_command, 0, sizeof(s_command));
    memset(s_wheelTarget, 0, sizeof(s_wheelTarget));
    s_enabled = false;
    s_estopLatched = false;
}

bool MotorControl_Enable(void)
{
    if (s_estopLatched)
    {
        return false;
    }
    DrvMotor_EnableAll();
    s_enabled = true;
    return true;
}

void MotorControl_Disable(void)
{
    DrvMotor_DisableAll();
    s_enabled = false;
}

void MotorControl_SetVelocity(float vx, float vy, float wz)
{
    if (!s_enabled)
    {
        return;
    }
    s_command[0] = vx;
    s_command[1] = vy;
    s_command[2] = wz;
}

void MotorControl_ZeroVelocity(void)
{
    s_command[0] = 0.0f;
    s_command[1] = 0.0f;
    s_command[2] = 0.0f;
}

void MotorControl_EmergencyStop(void)
{
    DrvMotor_BrakeAll();
    DrvMotor_DisableAll();
    s_enabled = false;
    s_estopLatched = true;
    for (uint8_t i = 0U; i < MOTOR_INDEX_COUNT; i++)
    {
        s_pid[i].integ = 0.0f;
        s_pid[i].firstCycle = true;
    }
    memset(s_command, 0, sizeof(s_command));
}

bool MotorControl_Recover(void)
{
    s_estopLatched = false;
    return MotorControl_Enable();
}

bool MotorControl_IsEnabled(void)
{
    return s_enabled;
}

void MotorControl_SetPid(uint8_t wheel, const PidGains *gains)
{
    if (wheel >= MOTOR_INDEX_COUNT || gains == NULL)
    {
        return;
    }
    s_gains[wheel] = *gains;
    s_pid[wheel].gains = *gains;
    s_pid[wheel].integ = 0.0f;
}

const PidGains *MotorControl_GetDefaultPid(void)
{
    return &kDefaultGains;
}

/**
 * @brief 1 kHz control tick (TaskControl).
 *
 * Order matters: sample encoders first so the PID uses fresh speed data,
 * then compute targets from the current command, then step the PID and
 * push duty (with slew limiting) into the motor driver.
 */
void MotorControl_ControlTick(void)
{
    DrvEncoder_Update();
    DrvMotor_Tick();

    if (!s_enabled)
    {
        return;
    }

    MotorControl_ComputeWheelTargets(s_command[0], s_command[1], s_command[2], s_wheelTarget);

    int32_t countsPerRev = DrvEncoder_CountsPerRev();
    for (uint8_t i = 0U; i < MOTOR_INDEX_COUNT; i++)
    {
        /* wheel surface m/s -> wheel shaft rad/s */
        float setpointRad = (s_wheelTarget[i] / WHEEL_RADIUS);
        const EncoderState *enc = DrvEncoder_GetState(i);
        float measuredRad = enc ? enc->radPerSec : 0.0f;
        float sign = MOTOR_SIGN[i];

        float duty = Pid_Update(&s_pid[i], measuredRad, sign * setpointRad, CONTROL_DT);
        DrvMotor_SetDuty((MotorId)i, duty * sign);
    }
    (void)countsPerRev;
}
