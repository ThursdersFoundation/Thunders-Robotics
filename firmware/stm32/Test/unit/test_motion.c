/**
 * @file    test_motion.c
 * @brief   Unit tests: omni kinematics, wheel PID, slew limiter, encoder
 *          math, odometry twist. Drivers are mocked (recording stubs).
 */
#include "motor_control.h"
#include "encoder.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "stm32f4xx_hal_tim.h"
#include "drv_ultrasonic.h"
#include "../mocks/mock_hal.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

static int s_fail = 0;
static int s_total = 0;
#define CHECK(cond) do { s_total++; if (!(cond)) { printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); s_fail++; } } while (0)
#define CHECK_F(a, b, tol) do { s_total++; float _a=(float)(a), _b=(float)(b); \
    if ((_a-_b)>(tol)||(_b-_a)>(tol)) { printf("  FAIL %s:%d  %s=%f != %s=%f\n", __FILE__, __LINE__, #a,_a,#b,_b); s_fail++; } } while (0)
#define CHECK_EQ(a, b) do { s_total++; long _a=(long)(a), _b=(long)(b); \
    if (_a!=_b) { printf("  FAIL %s:%d  %s=%ld != %s=%ld\n", __FILE__, __LINE__, #a,_a,#b,_b); s_fail++; } } while (0)

static void MockResetMotion(void);

/* Real drv_motor.c / drv_encoder.c / drv_ultrasonic.c are linked; the
 * fake timer registers from mock_hal.c let the register-level code run. */
static TIM_HandleTypeDef *s_encTimers[4];

static void MockDriveInit(void)
{
    s_encTimers[0] = &htim3; s_encTimers[1] = &htim4;
    s_encTimers[2] = &htim8; s_encTimers[3] = &htim1;
    for (int i = 0; i < 4; i++) { Mock_SetTimerCount(s_encTimers[i], 0); }
}

/** Advance the fake quadrature counters with a first-order motor model. */
static void MockDriveStep(float dutyPerWheel[4])
{
    static float speed[4];
    for (int i = 0; i < 4; i++)
    {
        speed[i] += (dutyPerWheel[i] * 12.0f - speed[i]) * 0.01f;
        uint16_t prev = Mock_GetTimerCount(s_encTimers[i]);
        uint16_t add = (uint16_t)(speed[i] * 1320.0f / 6.28318f * 0.001f);
        Mock_SetTimerCount(s_encTimers[i], (uint16_t)(prev + add));
    }
}

static int16_t DutyOf(int wheel)
{
    const MotorState *m = DrvMotor_GetState((MotorId)wheel);
    return m ? m->pwm : 0;
}

static float DutyFloat(int wheel)
{
    return (float)DutyOf(wheel) / (float)DRV_MOTOR_PWM_PERIOD;
}

static void TestKinematicsPureTranslation(void)
{
    float w[4];
    /* +x forward: all wheels equal */
    MotorControl_ComputeWheelTargets(1.0f, 0.0f, 0.0f, w);
    CHECK_F(w[0], 0.7071, 1e-3);
    CHECK_F(w[1], 0.7071, 1e-3);
    CHECK_F(w[2], 0.7071, 1e-3);
    CHECK_F(w[3], 0.7071, 1e-3);
}

static void TestKinematicsPureRotation(void)
{
    float w[4];
    /* CCW rotation: every wheel contributes CCW torque:
     * FL/FR backward, RL/RR forward, equal magnitudes */
    MotorControl_ComputeWheelTargets(0.0f, 0.0f, 2.0f, w);
    CHECK_F(w[0], -2.0f * 0.3f * 0.7071, 1e-3);   /* L = 0.3, wz = 2 */
    CHECK_F(w[0], w[1], 1e-6);
    CHECK_F(w[2], w[3], 1e-6);
    CHECK_F(w[2], -w[0], 1e-6);
}

static void TestKinematicsLateral(void)
{
    float w[4];
    /* +y left: FL/RR forward, FR/RL backward */
    MotorControl_ComputeWheelTargets(0.0f, 1.0f, 0.0f, w);
    CHECK_F(w[0], 0.7071, 1e-3);
    CHECK_F(w[1], -0.7071, 1e-3);
    CHECK_F(w[2], -0.7071, 1e-3);
    CHECK_F(w[3], 0.7071, 1e-3);
}

static void TestKinematicsOdometryConsistency(void)
{
    /* Inverse pair: encode twist to wheels, decode back */
    float w[4], vx, vy, wz;
    MotorControl_ComputeWheelTargets(0.7f, -0.2f, 1.5f, w);
    Odometry_ComputeTwist(w, &vx, &vy, &wz);
    CHECK_F(vx, 0.7, 1e-4);
    CHECK_F(vy, -0.2, 1e-4);
    CHECK_F(wz, 1.5, 1e-4);
}

static void TestPidConverges(void)
{
    PidController pid;
    memset(&pid, 0, sizeof(pid));
    pid.gains = *MotorControl_GetDefaultPid();
    pid.outMin = -1.0f; pid.outMax = 1.0f; pid.firstCycle = true;

    /* First-order wheel model: speed follows duty with 0.5 s time constant */
    float speed = 0.0f, setpoint = 8.0f;   /* rad/s */
    for (int i = 0; i < 4000; i++)
    {
        float duty = Pid_Update(&pid, speed, setpoint, 0.001f);
        speed += (duty * 12.0f - speed) * 0.002f;   /* simple motor model */
    }
    CHECK_F(speed, setpoint, 0.1);   /* settled within 1.25 % */
}

static void TestPidAntiWindup(void)
{
    PidController pid;
    memset(&pid, 0, sizeof(pid));
    pid.gains = *MotorControl_GetDefaultPid();
    pid.outMin = -1.0f; pid.outMax = 1.0f; pid.firstCycle = true;

    /* Demand far beyond what the output can deliver: integrator must clamp */
    for (int i = 0; i < 2000; i++)
    {
        (void)Pid_Update(&pid, 0.0f, 500.0f, 0.001f);
    }
    CHECK(pid.integ <= pid.gains.integMax + 1e-6);
    CHECK(pid.integ >= -pid.gains.integMax - 1e-6);
}

static void TestMotorControlSetVelocityNeedsEnable(void)
{
    MockDriveInit();
    MotorControl_Init();
    MotorControl_SetVelocity(0.5f, 0.0f, 0.0f);
    (void)MotorControl_ControlTick();
    /* disabled: no output was ever applied */
    CHECK_EQ(DutyOf(0), 0);
    CHECK_EQ(DutyOf(3), 0);

    CHECK(MotorControl_Enable());
    MotorControl_SetVelocity(0.5f, 0.0f, 0.0f);
    /* run a few ticks: the slew limiter moves pwm toward the target from
     * the tick AFTER SetDuty, so output is nonzero only from tick 2 on */
    for (int i = 0; i < 3; i++) { MotorControl_ControlTick(); }
    CHECK(DutyOf(0) != 0);
    CHECK(DutyOf(3) != 0);
}

static void TestSlewLimiter(void)
{
    MockDriveInit();
    MotorControl_Init();
    CHECK(MotorControl_Enable());
    /* step from 0 to full duty: each tick must move at most 0.04 */
    MotorControl_SetVelocity(32.0f, 0.0f, 0.0f);   /* saturates the PID */
    float prev = 0.0f;
    for (int i = 0; i < 40; i++)
    {
        MotorControl_ControlTick();
        float now = DutyFloat(0);
        float step = now - prev;
        if (step < 0) { step = -step; }
        CHECK(step <= 0.0401f);
        prev = now;
    }
}

static void TestEStopLatches(void)
{
    MockDriveInit();
    MotorControl_Init();
    CHECK(MotorControl_Enable());
    MotorControl_SetVelocity(1.0f, 0.0f, 0.0f);
    (void)MotorControl_ControlTick();
    MotorControl_EmergencyStop();
    CHECK(!MotorControl_IsEnabled());
    MotorControl_SetVelocity(1.0f, 0.0f, 0.0f);
    (void)MotorControl_ControlTick();
    CHECK_EQ(DutyOf(0), 0);   /* stays silent while latched */

    CHECK(MotorControl_Recover());   /* explicit recovery re-enables */
    CHECK(MotorControl_IsEnabled());
}

static void TestEncoderExtendDelta(void)
{
    /* forward wrap: 65534 -> 2 means +4 counts */
    CHECK_EQ(DrvEncoder_ExtendDelta(65534, 2), 4);
    /* backward wrap: 2 -> 65534 means -4 counts */
    CHECK_EQ(DrvEncoder_ExtendDelta(2, 65534), -4);
    /* normal: no wrap */
    CHECK_EQ(DrvEncoder_ExtendDelta(1000, 1100), 100);
    CHECK_EQ(DrvEncoder_ExtendDelta(1100, 1000), -100);
}

static void TestEncoderSpeedConversion(void)
{
    /* 66 counts in 25 ms = 2 rev/s = 12.566 rad/s (1320 counts/rev) */
    float rad = DrvEncoder_CountsToRadPerSec(66, 0.025f, 1320);
    CHECK_F(rad, 12.5664, 1e-3);
    CHECK_F(DrvEncoder_CountsToRadPerSec(66, 0.0f, 1320), 0.0f, 1e-9);
}

static void TestUltrasonicConversion(void)
{
    /* 343 m/s round trip: mm = us * 343 / 2000 (integer math) */
    CHECK_EQ(DrvUltrasonic_PulseToDistanceMm(580), 99);    /* 99.47 -> 99 */
    CHECK_EQ(DrvUltrasonic_PulseToDistanceMm(1164), 199);  /* ~200 mm     */
    CHECK_EQ(DrvUltrasonic_PulseToDistanceMm(0), 0);
    CHECK_EQ(DrvUltrasonic_PulseToDistanceMm(50000), 4000);  /* clamped */
}

int main(void)
{
    printf("== test_motion ==\n");
    TestKinematicsPureTranslation();
    TestKinematicsPureRotation();
    TestKinematicsLateral();
    TestKinematicsOdometryConsistency();
    TestPidConverges();
    TestPidAntiWindup();
    TestMotorControlSetVelocityNeedsEnable();
    TestEStopLatches();
    TestSlewLimiter();
    TestEncoderExtendDelta();
    TestEncoderSpeedConversion();
    TestUltrasonicConversion();

    printf("%d/%d passed, %d failed\n", s_total - s_fail, s_total, s_fail);
    return (s_fail == 0) ? 0 : 1;
}

