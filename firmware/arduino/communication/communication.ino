/**
 * @file    communication.ino
 * @brief   Thunders Robotic communication node (CAN node 4): USB-CAN bridge.
 *
 * PC console link on Serial1 (pins 18/19) at 115200..921600:
 *   PC -> robot : MSG_CMD_VELOCITY / CMD_ENABLE / PING ... (frame protocol)
 *   robot -> PC : TLM_* frames built from CAN traffic + heartbeats
 * Every CAN frame is mirrored to the PC as MSG_CAN_BRIDGE_RX for sniffing.
 */
#include "RobotCommon.h"
#include "src/can_interface.h"
#include "src/usb_bridge.h"

static CanInterface s_can;
static UsbBridge    s_bridge;

void setup()
{
    Serial.begin(115200UL);      /* USB debug */
    Serial1.begin(921600UL);     /* console bridge */
    s_can.begin(CAN_NODE_COMM);
    s_bridge.begin(&s_can);
    s_can.setBridge(&s_bridge);
}

void loop()
{
    uint32_t now = millis();

    /* PC frames in -> CAN out */
    while (Serial1.available() > 0)
    {
        s_bridge.onByte((uint8_t)Serial1.read());
    }

    /* CAN in -> PC out (drains RX) */
    s_can.poll(now);

    /* periodic master heartbeat over the bridge */
    s_bridge.tick(now);
}
