/**
 * @file    test_protocol.c
 * @brief   Unit tests: CRC-16, frame parser, payload (de)serialization.
 *
 * Minimal assert-style harness (no external framework needed).
 */
#include "crc.h"
#include "packet_parser.h"
#include "uart_protocol.h"
#include "can_protocol.h"
#include "protocol.h"
#include <stdio.h>
#include <string.h>

static int s_fail = 0;
static int s_total = 0;

#define CHECK(cond) do { \
    s_total++; \
    if (!(cond)) { printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); s_fail++; } \
} while (0)

#define CHECK_EQ(a, b) do { \
    s_total++; \
    long _a = (long)(a), _b = (long)(b); \
    if (_a != _b) { printf("  FAIL %s:%d  %s=%ld != %s=%ld\n", __FILE__, __LINE__, #a, _a, #b, _b); s_fail++; } \
} while (0)

#define CHECK_F(a, b, tol) do { \
    s_total++; \
    float _a = (float)(a), _b = (float)(b); \
    if ((_a - _b) > (tol) || (_b - _a) > (tol)) { printf("  FAIL %s:%d  %s=%f != %s=%f\n", __FILE__, __LINE__, #a, _a, #b, _b); s_fail++; } \
} while (0)

static void TestCrc16Vector(void)
{
    /* CCITT-FALSE check value: CRC("123456789") = 0x29B1 */
    CHECK_EQ(crc16((const uint8_t *)"123456789", 9), 0x29B1);
    CHECK_EQ(crc16(NULL, 0), CRC16_CCITT_INIT);
}

static void TestParserRoundtrip(void)
{
    PacketParser p;
    PacketParser_Init(&p);

    uint8_t payload[6] = { 0x10, 0x27, 0x00, 0x00, 0xE8, 0x03 };
    uint8_t frame[32];
    size_t n = Packet_Build(frame, sizeof(frame), MSG_CMD_VELOCITY, payload, 6U);
    CHECK_EQ(n, PROTOCOL_HEADER_SIZE + 6U + PROTOCOL_CRC_SIZE);

    PacketFrame out;
    bool got = false;
    for (size_t i = 0U; i < n; i++)
    {
        got |= PacketParser_Feed(&p, frame[i], &out);
    }
    CHECK(got);
    CHECK_EQ(out.type, MSG_CMD_VELOCITY);
    CHECK_EQ(out.length, 6U);
    CHECK_EQ(out.payload[0], 0x10);
    CHECK_EQ(p.frames_ok, 1U);
    CHECK_EQ(p.frames_bad_crc, 0U);
}

static void TestParserZeroLength(void)
{
    PacketParser p;
    PacketParser_Init(&p);
    uint8_t frame[16];
    size_t n = Packet_Build(frame, sizeof(frame), MSG_PING, NULL, 0U);
    CHECK_EQ(n, 6U);

    PacketFrame out;
    bool got = false;
    for (size_t i = 0U; i < n; i++) { got |= PacketParser_Feed(&p, frame[i], &out); }
    CHECK(got);
    CHECK_EQ(out.type, MSG_PING);
    CHECK_EQ(out.length, 0U);
}

static void TestParserBadCrc(void)
{
    PacketParser p;
    PacketParser_Init(&p);
    uint8_t frame[16];
    (void)Packet_Build(frame, sizeof(frame), MSG_PING, NULL, 0U);
    frame[5] ^= 0xFFU;   /* corrupt CRC high byte */

    PacketFrame out;
    bool got = false;
    for (size_t i = 0U; i < 6U; i++) { got |= PacketParser_Feed(&p, frame[i], &out); }
    CHECK(!got);
    CHECK_EQ(p.frames_bad_crc, 1U);
}

static void TestParserGarbageAndResync(void)
{
    PacketParser p;
    PacketParser_Init(&p);
    uint8_t frame[16];
    (void)Packet_Build(frame, sizeof(frame), MSG_PING, NULL, 0U);

    PacketFrame out;
    uint8_t noise[40];
    memset(noise, 0x55, sizeof(noise));

    bool got = false;
    for (size_t i = 0U; i < sizeof(noise); i++) { PacketParser_Feed(&p, noise[i], &out); }
    for (size_t i = 0U; i < 6U; i++) { got |= PacketParser_Feed(&p, frame[i], &out); }
    CHECK(got);   /* parser resynced after garbage */
    CHECK(p.frames_ok >= 1U);
}

static void TestParserBackToBackFrames(void)
{
    PacketParser p;
    PacketParser_Init(&p);
    uint8_t frame[16];
    (void)Packet_Build(frame, sizeof(frame), MSG_PING, NULL, 0U);

    PacketFrame out;
    int frames = 0;
    for (int r = 0; r < 3; r++)
    {
        for (size_t i = 0U; i < 6U; i++)
        {
            if (PacketParser_Feed(&p, frame[i], &out)) { frames++; }
        }
    }
    CHECK_EQ(frames, 3);
    CHECK_EQ(p.frames_ok, 3U);
}

static void TestParserOversizedLength(void)
{
    PacketParser p;
    PacketParser_Init(&p);
    PacketFrame out;
    uint8_t bytes[] = { 0xA5, 0x5A, MSG_EVT_TEXT, 0xFFU };
    for (size_t i = 0U; i < sizeof(bytes); i++) { PacketParser_Feed(&p, bytes[i], &out); }
    CHECK_EQ(p.state, PARSER_SYNC0);   /* rejected, back to sync hunting */
}

static void TestVelocityEncoding(void)
{
    uint8_t payload[6];
    proto_put_i16(&payload[0], 500);    /*  0.5 m/s  */
    proto_put_i16(&payload[2], -250);   /* -0.25 m/s */
    proto_put_i16(&payload[4], 1000);   /*  1.0 rad/s */

    float vx, vy, wz;
    UartProtocol_DecodeVelocity(payload, &vx, &vy, &wz);
    CHECK_F(vx, 0.5, 1e-5);
    CHECK_F(vy, -0.25, 1e-5);
    CHECK_F(wz, 1.0, 1e-5);
}

static void TestOdomPayload(void)
{
    ProtoOdom odom;
    odom.x = 12345; odom.y = -6789;
    odom.theta = 9000;   /* 90.00 deg */
    odom.v = 1500; odom.w = 100;
    odom.seq = 42;
    uint8_t buf[16];
    CHECK_EQ(UartProtocol_EncodeOdom(buf, &odom), 16U);
    CHECK_EQ(proto_get_i32(&buf[0]), 12345);
    CHECK_EQ(proto_get_i32(&buf[4]), -6789);
    CHECK_EQ(proto_get_i16(&buf[8]), 9000);
    CHECK_EQ(proto_get_u16(&buf[14]), 42);
}

static void TestCanVelocity(void)
{
    float speeds[4] = { 1.25f, -0.5f, 2.0f, -3.2f };
    uint8_t buf[8];
    CHECK_EQ(Can_EncodeVelocity(buf, speeds), 8U);

    float out[4];
    CHECK(Can_DecodeVelocity(buf, 8U, out));
    CHECK_F(out[0], 1.25, 1e-4);
    CHECK_F(out[1], -0.5, 1e-4);
    CHECK_F(out[2], 2.0, 1e-4);
    CHECK_F(out[3], -3.2, 1e-4);
    CHECK(!Can_DecodeVelocity(buf, 2U, out));   /* short DLC rejected */
}

static void TestCanServoStepper(void)
{
    uint8_t buf[8];
    CHECK_EQ(Can_EncodeServo(buf, 2U, 87.5f), 3U);
    uint8_t id; float ang;
    CHECK(Can_DecodeServo(buf, 3U, &id, &ang));
    CHECK_EQ(id, 2U);
    CHECK_F(ang, 87.5, 1e-4);

    CHECK_EQ(Can_EncodeStepper(buf, 1U, -123456, 500), 7U);
    uint8_t axis; int32_t pos; int16_t vel;
    CHECK(Can_DecodeStepper(buf, 7U, &axis, &pos, &vel));
    CHECK_EQ(axis, 1U);
    CHECK_EQ(pos, -123456);
    CHECK_EQ(vel, 500);
}

static void TestCanPower(void)
{
    uint8_t buf[8];
    CHECK_EQ(Can_EncodePower(buf, 12300U, 4200U, 35.7f), 6U);
    uint16_t mv, ma; float tc;
    CHECK(Can_DecodePower(buf, 6U, &mv, &ma, &tc));
    CHECK_EQ(mv, 12300);
    CHECK_EQ(ma, 4200);
    CHECK_F(tc, 35.7, 0.05);
}

static void TestMessageNameAndValidation(void)
{
    CHECK(strcmp(Protocol_MessageName(MSG_CMD_VELOCITY), "CMD_VELOCITY") == 0);
    CHECK(strcmp(Protocol_MessageName(0xEE), "UNKNOWN") == 0);
    CHECK(Protocol_PayloadSizeValid(MSG_CMD_VELOCITY, 6U));
    CHECK(!Protocol_PayloadSizeValid(MSG_CMD_VELOCITY, 5U));
    CHECK(Protocol_PayloadSizeValid(MSG_EVT_TEXT, 10U));  /* variable length */
}

int main(void)
{
    printf("== test_protocol ==\n");
    TestCrc16Vector();
    TestParserRoundtrip();
    TestParserZeroLength();
    TestParserBadCrc();
    TestParserGarbageAndResync();
    TestParserBackToBackFrames();
    TestParserOversizedLength();
    TestVelocityEncoding();
    TestOdomPayload();
    TestCanVelocity();
    TestCanServoStepper();
    TestCanPower();
    TestMessageNameAndValidation();

    printf("%d/%d passed, %d failed\n", s_total - s_fail, s_total, s_fail);
    return (s_fail == 0) ? 0 : 1;
}
