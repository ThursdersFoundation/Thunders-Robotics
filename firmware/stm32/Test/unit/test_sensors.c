/**
 * @file    test_sensors.c
 * @brief   Unit tests: battery conversions, ultrasonic capture, lidar
 *          protocol parsing, moving average, IMU filter, ring buffer.
 */
#include "drv_battery.h"
#include "drv_ultrasonic.h"
#include "drv_lidar.h"
#include "moving_average.h"
#include "ring_buffer.h"
#include "drv_imu.h"
#include "mock_hal.h"
#include "adc.h"
#include <stdio.h>
#include <math.h>

static int s_fail = 0;
static int s_total = 0;
#define CHECK(cond) do { s_total++; if (!(cond)) { printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); s_fail++; } } while (0)
#define CHECK_EQ(a, b) do { s_total++; long _a=(long)(a), _b=(long)(b); \
    if (_a!=_b) { printf("  FAIL %s:%d  %s=%ld != %s=%ld\n", __FILE__, __LINE__, #a,_a,#b,_b); s_fail++; } } while (0)
#define CHECK_F(a, b, tol) do { s_total++; float _a=(float)(a), _b=(float)(b); \
    if ((_a-_b)>(tol)||(_b-_a)>(tol)) { printf("  FAIL %s:%d  %s=%f != %s=%f\n", __FILE__, __LINE__, #a,_a,#b,_b); s_fail++; } } while (0)

/* ---- battery ---- */
static void TestBatteryVoltage(void)
{
    /* 3.3 V ref, 12-bit, divider x4.3: raw=1260 -> 1.0146 V * 4.3 = 4363 mV?  */
    uint16_t mv = DrvBattery_AdcToMillivolt(4095U);
    CHECK_EQ(mv, 14190U);   /* 3.3 * 4.3 = 14.19 V full-scale */

    /* raw for 11.1 V: 11100 / (3300*4.3/4095) = 11100/3.465 = 3203.5 */
    CHECK_EQ(DrvBattery_AdcToMillivolt(3204U), 11102U);
}

static void TestBatteryPercent(void)
{
    CHECK_EQ(DrvBattery_VoltageToPercent(12600), 100);
    CHECK_EQ(DrvBattery_VoltageToPercent(9900), 0);
    CHECK_EQ(DrvBattery_VoltageToPercent(11250), 50);   /* midpoint */
}

static void TestBatteryUpdateFlow(void)
{
    static volatile uint16_t adc[3];
    /* ~11.1 V, ~2 A, ~35 C: pick raws for each stage */
    adc[0] = 3204U;                       /* 11.1 V   */
    /* current: sensor mV = raw/4095*3300; 2 A -> 1650+200 = 1850 mV -> raw 2295 */
    adc[1] = 2295U;
    /* NTC ~ Rntc such that temp ~ 35 C: R = 10k * exp(beta*(1/T-1/T0)) =
     * 10k * exp(3950*(1/308.15 - 1/298.15)) = 10k*exp(-0.4317) = 6494 ohm
     * raw = 4095 * R/(R+10k) = 4095*0.3937 = 1612 */
    adc[2] = 1612U;

    DrvBattery_Init(adc);
    for (int i = 0; i < 20; i++) { DrvBattery_Update(); }
    const BatteryState *b = DrvBattery_GetState();
    CHECK_F(b->voltageMv, 11100, 30);
    CHECK_F(b->currentMa, 2000, 50);
    CHECK_F(b->temperatureCx10, 350, 30);   /* +/-3 C */
    CHECK(b->percent > 40U && b->percent < 60U);
    CHECK(!b->warn);
    CHECK(!b->critical);
    CHECK(!b->overcurrent);
}

/* ---- ultrasonic capture state machine ---- */
static void TestUltrasonicCapture(void)
{
    DrvUltrasonic_Init();
    Mock_SetTick(1000U);

    /* rising edge: measuring=false -> records rise, sets measuring */
    DrvUltrasonic_OnCapture(0, 5000U);
    UltrasonicState *us = (UltrasonicState *)(void *)DrvUltrasonic_GetState(USONIC_FRONT);
    CHECK(us->measuring);

    /* falling edge at +1000 us: width 1000 us -> 171 mm */
    DrvUltrasonic_OnCapture(0, 6000U);
    CHECK(!us->measuring);
    CHECK(us->valid);
    CHECK_EQ(us->distanceMm, 171U);

    /* timer wrap case: rise 65500, fall 300 -> width (65536+300)-65500 = 336 us -> 57 mm */
    us->measuring = false;
    DrvUltrasonic_OnCapture(1, 65500U);
    DrvUltrasonic_OnCapture(1, 300U);
    UltrasonicState *us2 = (UltrasonicState *)(void *)DrvUltrasonic_GetState(USONIC_LEFT);
    CHECK(us2->valid);
    CHECK_EQ(us2->distanceMm, 57U);

    /* out-of-range pulse (26 ms) is dropped as invalid */
    us2->measuring = false;
    us2->valid = false;
    DrvUltrasonic_OnCapture(1, 100U);
    DrvUltrasonic_OnCapture(1, 26100U);
    CHECK(!us2->valid);
}

/* ---- lidar byte-stream parser ---- */
static void TestLidarDeviceInfoResponse(void)
{
    DrvLidar_Init();
    /* descriptor: A5 5A 14 00 00 00 82 (20 bytes, device info) */
    const uint8_t desc[7] = { 0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x82 };
    uint8_t data[20] = { 0 };
    data[0] = 0x18;           /* model */
    data[1] = 0x20; data[2] = 0x01;   /* fw 1.32 */
    data[3] = 0x03;           /* hardware */
    memset(&data[4], 0xAB, 16);       /* serial */

    for (size_t i = 0; i < sizeof(desc); i++) { DrvLidar_OnByte(desc[i]); }
    for (size_t i = 0; i < sizeof(data); i++) { DrvLidar_OnByte(data[i]); }

    const LidarStateData *st = DrvLidar_GetState();
    CHECK(st->infoValid);
    CHECK_EQ(st->info.firmwareMajor, 1);
    CHECK_EQ(st->info.firmwareMinor, 32);
    CHECK_EQ(st->info.serialNumber[0], (char)0xAB);
}

/* Fold-aware test node sender: emulates the q6 counter with 2-degree folds */
static int s_testFold = 0;
static void LidarTest_SendNode(float angleDeg, uint16_t distMm, uint8_t quality, uint8_t startFlag)
{
    int folds = (int)(angleDeg / 2.0f);
    uint8_t q6 = (uint8_t)((angleDeg - (float)folds * 2.0f) * 64.0f + 0.5f);
    if (q6 == 128U) { q6 = 0U; folds++; }

    /* emit wrap pairs (127 -> 0) until the driver fold counter catches up */
    while (s_testFold < folds)
    {
        uint8_t wrap[5] = { 0, 0, 0, 0, 0 };   /* invalid distance */
        wrap[1] = (uint8_t)(127U << 1);
        for (int i = 0; i < 5; i++) { DrvLidar_OnByte(wrap[i]); }
        wrap[1] = 0U;
        for (int i = 0; i < 5; i++) { DrvLidar_OnByte(wrap[i]); }
        s_testFold++;
    }

    uint16_t dq = (uint16_t)((distMm << 2) | 1U);
    uint8_t node[5];
    node[0] = (uint8_t)((quality << 2) | (startFlag ? 0x01U : 0x00U));
    node[1] = (uint8_t)(q6 << 1);
    node[2] = (uint8_t)(dq & 0xFFU);
    node[3] = (uint8_t)(dq >> 8);
    node[4] = 0U;
    for (int i = 0; i < 5; i++) { DrvLidar_OnByte(node[i]); }
}

static void TestLidarScanNodes(void)
{
    DrvLidar_Init();
    s_testFold = 0;
    const uint8_t desc[7] = { 0xA5, 0x5A, 0x05, 0x00, 0x00, 0x00, 0x81 };
    for (size_t i = 0; i < sizeof(desc); i++) { DrvLidar_OnByte(desc[i]); }

    /* Revolution start at 0 deg, quality 10, 1000 mm */
    LidarTest_SendNode(0.0f, 1000U, 10U, 1U);
    const LidarStateData *st = DrvLidar_GetState();
    CHECK_EQ(st->sampleCount, 1U);
    CHECK_EQ(st->distanceMm[0], 1000U);
    CHECK_EQ(st->quality[0], 10U);

    /* Second sample at 1.0 deg (q6=64), 2000 mm */
    LidarTest_SendNode(1.0f, 2000U, 7U, 0U);
    CHECK_EQ(st->distanceMm[1], 2000U);
    CHECK_EQ(st->quality[1], 7U);

    /* Next revolution starts: S flag again -> scanComplete */
    LidarTest_SendNode(0.0f, 1000U, 10U, 1U);
    CHECK(st->scanComplete);
    CHECK_EQ(st->revolutionCount, 1U);

    uint16_t scan[360];
    CHECK(DrvLidar_TakeScan(scan, NULL));
    CHECK(!st->scanComplete);   /* consumed */
    CHECK_EQ(scan[1], 2000U);
}

static void TestLidarHelpers(void)
{
    CHECK_EQ(Lidar_QualityFromBytes(0xFF), 0x3FU);   /* top 6 bits */
    CHECK_EQ(Lidar_DistanceFromBytes((const uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00}), 0U);
    /* invalid marker: bit0 of distQ2 = 0 -> 0 mm */
    CHECK_EQ(Lidar_DistanceFromBytes((const uint8_t[]){0, 0, 0x10, 0x02, 0}), 0U);
    CHECK_EQ(Lidar_SectorBin(359.6f), 0U);   /* rounds into bin 360 -> wraps */
    CHECK_EQ(Lidar_SectorBin(-5.0f), 355U);
}

static void TestLidarSectorMin(void)
{
    DrvLidar_Init();
    s_testFold = 0;
    const uint8_t desc[7] = { 0xA5, 0x5A, 0x05, 0x00, 0x00, 0x00, 0x81 };
    for (size_t i = 0; i < sizeof(desc); i++) { DrvLidar_OnByte(desc[i]); }
    /* sector 0 spans 0..22.5 deg; all three samples land inside it */
    LidarTest_SendNode(0.0f, 3000U, 10U, 1U);
    LidarTest_SendNode(5.0f, 1200U, 10U, 0U);
    LidarTest_SendNode(12.0f, 2500U, 10U, 0U);
    LidarTest_SendNode(19.0f, 1800U, 10U, 0U);
    CHECK_EQ(DrvLidar_MinDistanceInSector(0.0f, 22.5f), 1200U);
    /* empty sector reports "invalid" */
    CHECK_EQ(DrvLidar_MinDistanceInSector(180.0f, 202.5f), LIDAR_INVALID_MM);
}

/* ---- moving average ---- */
static void TestMovingAverage(void)
{
    MovingAverage ma;
    MovingAverage_Init(&ma, 4);
    CHECK_EQ(MovingAverage_Update(&ma, 100), 100);
    CHECK_EQ(MovingAverage_Update(&ma, 200), 150);
    CHECK_EQ(MovingAverage_Update(&ma, 300), 200);
    CHECK_EQ(MovingAverage_Update(&ma, 400), 250);
    CHECK_EQ(MovingAverage_Update(&ma, 800), 425);   /* evicts the 100 */
    MovingAverage_Reset(&ma);
    CHECK_EQ(MovingAverage_Last(&ma), 0);
}

/* ---- ring buffer ---- */
static void TestRingBuffer(void)
{
    RingBuffer rb;
    RingBuffer_Init(&rb, 8U);
    CHECK_EQ(RingBuffer_Count(&rb), 0U);

    uint8_t out[4];
    CHECK(RingBuffer_Push(&rb, 1));
    CHECK(RingBuffer_Push(&rb, 2));
    CHECK_EQ(RingBuffer_Pop(&rb, &out[0]), true);
    CHECK_EQ(out[0], 1);

    /* fill to capacity, next push must fail and count overrun */
    for (int i = 0; i < 7; i++) { RingBuffer_Push(&rb, (uint8_t)(10 + i)); }
    CHECK_EQ(RingBuffer_Count(&rb), 8U);
    CHECK(!RingBuffer_Push(&rb, 0xFF));
    CHECK_EQ(rb.overrun_count, 1U);

    /* bulk read drains */
    uint8_t big[16];
    CHECK_EQ(RingBuffer_Read(&rb, big, sizeof(big)), 8U);
    CHECK_EQ(RingBuffer_Count(&rb), 0U);

    /* wraparound index space */
    for (int i = 0; i < 40; i++) { CHECK(RingBuffer_Push(&rb, (uint8_t)i)); CHECK(RingBuffer_Pop(&rb, &out[0])); }
    CHECK_EQ(out[0], 39);
}

/* ---- IMU complementary filter ---- */
static void TestImuFilter(void)
{
    float angle = 0.0f;
    /* Static: accel says 10 deg, gyro 0 -> angle converges to 10 */
    for (int i = 0; i < 500; i++)
    {
        DrvImu_ComplementaryStep(0.0f, 10.0f, 0.005f, 0.98f, &angle);
    }
    CHECK_F(angle, 10.0f, 0.2);

    /* Rotating at 50 deg/s with accel lagging: gyro should dominate short term */
    angle = 0.0f;
    for (int i = 0; i < 20; i++)
    {
        DrvImu_ComplementaryStep(50.0f, 0.0f, 0.005f, 0.98f, &angle);
    }
    CHECK_F(angle, 4.07f, 0.2);  /* gyro drive minus accel pull-over 20 ticks */
}

static void TestImuInitAndRead(void)
{
    Mock_I2cSetWhoAmI(0x68U);
    /* accel x=0.5g, gyro z=1000 raw */
    uint8_t raw[14] = { 0 };
    uint16_t ax = (uint16_t)(0.5f * 16384.0f);
    raw[0] = (uint8_t)(ax >> 8); raw[1] = (uint8_t)ax;
    uint16_t gz = 1000;
    raw[12] = (uint8_t)(gz >> 8); raw[13] = (uint8_t)gz;
    Mock_I2cSetBurst14(raw);
    Mock_SetTick(0U);

    CHECK(DrvImu_Init());
    for (int i = 0; i < 200; i++) { DrvImu_Update(); Mock_AdvanceTick(5U); }
    const ImuState *imu = DrvImu_GetState();
    CHECK(imu->initialized);
    CHECK_F(imu->accelG[0], 0.5f, 0.01f);
    /* gyro z bias captured while "stationary": first 64 reads all 1000 raw
     * -> bias 60.98 dps; updates subtract it, leaving ~0 */
    CHECK_F(imu->gyroDps[2], 0.0f, 1.0f);
}

int main(void)
{
    printf("== test_sensors ==\n");
    TestImuInitAndRead();
    TestBatteryVoltage();
    TestBatteryPercent();
    TestBatteryUpdateFlow();
    TestUltrasonicCapture();
    TestLidarDeviceInfoResponse();
    TestLidarScanNodes();
    TestLidarHelpers();
    TestLidarSectorMin();
    TestMovingAverage();
    TestRingBuffer();
    TestImuFilter();

    printf("%d/%d passed, %d failed\n", s_total - s_fail, s_total, s_fail);
    return (s_fail == 0) ? 0 : 1;
}
