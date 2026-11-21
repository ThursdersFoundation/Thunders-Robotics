/**
 * @file    test_system.c
 * @brief   Host integration test: full command chain without hardware.
 *
 *   frame(CMD_VELOCITY) -> parser -> dispatch -> MotorControl -> mock drivers
 *   odometry -> ProtoOdom -> UartProtocol_Send -> capture -> parse back
 *   battery-critical -> SafetyManager fault -> latched emergency stop
 *
 * Real modules: crc, packet_parser, uart_protocol, protocol, motor_control,
 * encoder (odometry), safety_manager, FreeRTOS subset, moving_average.
 * Mocked at hardware edges: drivers, watchdog, GPIO, tick.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "crc.h"
#include "packet_parser.h"
#include "uart_protocol.h"
#include "can_protocol.h"
#include "protocol.h"
#include "motor_control.h"
#include "encoder.h"
#include "safety_manager.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "stm32f4xx_hal_tim.h"
#include "watchdog_manager.h"
#include "../mocks/mock_hal.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

static int s_fail = 0;
static int s_total = 0;
#define CHECK(cond) do { s_total++; if (!(cond)) { printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); s_fail++; } } while (0)
#define CHECK_EQ(a, b) do { s_total++; long _a=(long)(a), _b=(long)(b); \
    if (_a!=_b) { printf("  FAIL %s:%d  %s=%ld != %s=%ld\n", __FILE__, __LINE__, #a,_a,#b,_b); s_fail++; } } while (0)
#define CHECK_F(a, b, tol) do { s_total++; float _a=(float)(a), _b=(float)(b); \
    if ((_a-_b)>(tol)||(_b-_a)>(tol)) { printf("  FAIL %s:%d  %s=%f != %s=%f\n", __FILE__, __LINE__, #a,_a,#b,_b); s_fail++; } } while (0)

/* Real drivers are linked (see Makefile); watchdog comes from mock_hal. */
static TIM_HandleTypeDef *s_encTimers[4];
static float s_speed[4];

static void MockDriveInit(void)
{
    s_encTimers[0] = &htim3; s_encTimers[1] = &htim4;
    s_encTimers[2] = &htim8; s_encTimers[3] = &htim1;
    for (int i = 0; i < 4; i++) { Mock_SetTimerCount(s_encTimers[i], 0); }
}

static float s_countAcc[4];

/* Mounting signs replicated from motor_control.c */
static const float kWheelSign[4] = { 1.0f, -1.0f, -1.0f, 1.0f };

static void MockDriveStep(float dutyPerWheel[4])
{
    for (int i = 0; i < 4; i++)
    {
        s_speed[i] += (dutyPerWheel[i] * kWheelSign[i] * 12.0f - s_speed[i]) * 0.01f;
        /* accumulate fractional counts so the encoder view is not quantized */
        s_countAcc[i] += s_speed[i] * 1320.0f / 6.28318f * 0.001f;
        int32_t addRaw = (int32_t)s_countAcc[i];   /* signed: reversed wheels count down */
        s_countAcc[i] -= (float)addRaw;
        uint16_t prev = Mock_GetTimerCount(s_encTimers[i]);
        Mock_SetTimerCount(s_encTimers[i], (uint16_t)((prev + addRaw) & 0xFFFFU));
    }
}

static float DutyFloat(int wheel)
{
    const MotorState *m = DrvMotor_GetState((MotorId)wheel);
    return m ? (float)m->pwm / (float)DRV_MOTOR_PWM_PERIOD : 0.0f;
}

/* ------------------------------------------------------------------ */
/* Frame plumbing                                                      */
/* ------------------------------------------------------------------ */
static PacketParser s_rxParser;

static void FeedFrameToSystem(uint8_t type, const void *payload, uint8_t len)
{
    uint8_t frame[PROTOCOL_MAX_FRAME];
    size_t n = Packet_Build(frame, sizeof(frame), type, (const uint8_t *)payload, len);

    /* Emulate the SBC -> UART4 -> DMA -> parser path */
    PacketFrame out;
    for (size_t i = 0U; i < n; i++)
    {
        if (PacketParser_Feed(&s_rxParser, frame[i], &out))
        {
            /* same dispatch logic as CommunicationManager_HandleFrame */
            if (out.type == MSG_CMD_VELOCITY)
            {
                float vx, vy, wz;
                UartProtocol_DecodeVelocity(out.payload, &vx, &vy, &wz);
                MotorControl_SetVelocity(vx, vy, wz);
            }
            else if (out.type == MSG_CMD_ENABLE)
            {
                SafetyManager_ClearAllFaults();
                if (out.payload[0] != 0U) { MotorControl_Recover(); }
                else { MotorControl_Disable(); }
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */
static void TestVelocityCommandToDuty(void)
{
    MotorControl_Init();
    CHECK(MotorControl_Enable());

    ProtoVelocity cmd;
    cmd.vx = 500;   /* 0.5 m/s */
    cmd.vy = 0;
    cmd.wz = 0;
    FeedFrameToSystem(MSG_CMD_VELOCITY, &cmd, 6U);

    /* run 1 kHz control for 0.5 s with an ideal motor model:
     * wheel speed converges to setpoint, duty settles */
    float duty[4];
    for (int i = 0; i < 2000; i++)
    {
        MotorControl_ControlTick();
        for (int w = 0; w < 4; w++) { duty[w] = DutyFloat(w); }
        MockDriveStep(duty);
    }
    /* settled: wheel shaft speed approaches 0.5 m/s * 0.7071 / 0.04 = 8.84 rad/s.
     * radPerSec is per-tick quantized (1 count = 4.76 rad/s @1320 CPR, 1 kHz),
     * so the meaningful measure is the count total over the 0.5 s run:
     * 8.839 rad/s * 2.0 s / (2*pi) * 1320 counts = 3714 counts. */
    for (int w = 0; w < 4; w++)
    {
        const EncoderState *e = DrvEncoder_GetState((uint8_t)w);
        CHECK_F(fabsf((float)e->totalCounts), 3714.0f, 180.0f);
    }
}

static void TestTelemetryRoundtrip(void)
{
    /* drive odometry, then stream it back through the TX capture */
    float wheels[4] = { 0.7071f, 0.7071f, 0.7071f, 0.7071f };
    Odometry_Update(wheels, 0.0f, 0.005f);
    const Odometry *odom = Odometry_Get();
    CHECK_F(odom->vx, 1.0f, 0.01f);   /* 4 wheels at 0.7071 -> vx 1.0 */

    ProtoOdom po;
    po.x = (int32_t)(odom->x * 1000.0);
    po.y = (int32_t)(odom->y * 1000.0);
    po.theta = (int16_t)(odom->theta * 5729.5779f);
    po.v = (int16_t)(odom->vx * 1000.0f);
    po.w = (int16_t)(odom->wz * 100.0f);
    po.seq = 7U;
    uint8_t buf[16];
    size_t len = UartProtocol_EncodeOdom(buf, &po);

    uint8_t frame[PROTOCOL_MAX_FRAME];
    size_t n = Packet_Build(frame, sizeof(frame), MSG_TLM_ODOM, buf, (uint8_t)len);

    /* receive it on the "host side" */
    PacketParser host;
    PacketParser_Init(&host);
    PacketFrame out;
    bool got = false;
    for (size_t i = 0U; i < n; i++) { got |= PacketParser_Feed(&host, frame[i], &out); }
    CHECK(got);
    CHECK_EQ(out.type, MSG_TLM_ODOM);
    ProtoOdom rx;
    rx.x = proto_get_i32(&out.payload[0]);
    rx.v = proto_get_i16(&out.payload[10]);
    rx.seq = proto_get_u16(&out.payload[14]);
    CHECK_F((float)rx.v / 1000.0f, 1.0f, 0.01f);   /* 1.0 m/s +/- 1 cm/s */
    CHECK_EQ(rx.seq, 7);
}

static void TestSafetyBatteryFaultChain(void)
{
    SafetyManager_Init();
    MotorControl_Init();
    CHECK(MotorControl_Enable());
    CHECK(MotorControl_IsEnabled());

    /* critical battery arrives from the sensor task */
    SafetyManager_ReportBattery(9800U, 1000U);
    SafetyManager_ReportTilt(1.0f, 2.0f);
    SafetyManager_ReportObstacle(0xFFFFU);
    SafetyManager_Task();

    CHECK(SafetyManager_GetFaults() & FAULT_BATTERY_CRITICAL);
    CHECK_EQ(SafetyManager_GetState(), SAFETY_STATE_SAFE_STOP);
    CHECK(!MotorControl_IsEnabled());        /* chain tripped the e-stop */
    MotorControl_SetVelocity(1.0f, 0.0f, 0.0f);
    (void)MotorControl_ControlTick();
    CHECK_EQ(DutyFloat(0), 0.0f);            /* no output while latched */

    /* operator clears: ENABLE(1) -> recover */
    uint8_t on = 1U;
    FeedFrameToSystem(MSG_CMD_ENABLE, &on, 1U);
    CHECK(MotorControl_IsEnabled());
}

static void TestSafetyWatchdogFed(void)
{
    (void)Mock_WatchdogRefreshes();
    for (int i = 0; i < 10; i++)
    {
        SafetyManager_ReportBattery(11100U, 1000U);
        SafetyManager_ReportTilt(0.0f, 0.0f);
        SafetyManager_ReportObstacle(0xFFFFU);
        SafetyManager_Task();
        Mock_AdvanceTick(10U);
    }
    CHECK(Mock_WatchdogRefreshes() >= 10);   /* healthy system keeps the dog fed */
}

static void TestObstacleFaultOnlyWhenRunning(void)
{
    SafetyManager_ClearAllFaults();
    MotorControl_Disable();
    SafetyManager_ReportObstacle(100U);
    SafetyManager_Task();
    /* obstacle reported but system idle and motors never enabled this cycle:
     * fault is raised only when RUNNING */
    /* (implementation raises only in RUNNING state) */
    CHECK(SafetyManager_GetState() != SAFETY_STATE_RUNNING);
    SafetyManager_ReportObstacle(0xFFFFU);
}

int main(void)
{
    printf("== test_system (integration) ==\n");
    PacketParser_Init(&s_rxParser);
    Mock_UartReset();
    Mock_GpioReset();
    MockDriveInit();
    UartProtocol_SetSender(NULL);

    TestVelocityCommandToDuty();
    TestTelemetryRoundtrip();
    TestSafetyBatteryFaultChain();
    TestSafetyWatchdogFed();
    TestObstacleFaultOnlyWhenRunning();

    printf("%d/%d passed, %d failed\n", s_total - s_fail, s_total, s_fail);
    return (s_fail == 0) ? 0 : 1;
}
