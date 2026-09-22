// RemoteUartBus protocol of the hoverboard firmware (HoverBoardMindMotion/Src/remoteUartBus.c).
// Packed little-endian frames, CRC-16/XMODEM over all bytes except the CRC (sent low byte first).
//   ESP32 -> board  SerialServer2Hover:  '/' 0 slave speed:int16 wState:uint8 crc:uint16
//   board -> ESP32  SerialHover2Server:  0xABCD slave speed:int16 volt:uint16 amp:int16 odom:int32 crc:uint16
// Answer units: speed = rpm * 10, volt = V * 100, amp = FOC torque current iq * 100 (A).
#pragma once
#include <Arduino.h>

struct HoverAnswer {
    uint8_t  slave;
    float    rpm;
    float    volt;
    float    iq;
    int32_t  odom;
    uint32_t receivedMs;   // millis() when received, 0 = never
};

uint16_t hoverCrc(const uint8_t *data, size_t len);

// Writes the 8-byte speed frame into out, returns its length.
size_t hoverSpeedFrame(uint8_t *out, uint8_t slave, int16_t speed);

// Byte-wise parser for board answers.
class HoverAnswerParser {
public:
    // Returns true when a complete answer with a valid CRC was received (stored in *answer).
    bool feed(uint8_t c, HoverAnswer *answer);
    uint32_t badCrc = 0;

private:
    static const size_t kSize = 15;
    uint8_t buf_[kSize];
    size_t  pos_ = 0;
};
