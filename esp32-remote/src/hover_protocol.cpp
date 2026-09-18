#include "hover_protocol.h"

uint16_t hoverCrc(const uint8_t *data, size_t len)
{
    // Same as CalcCRC() in the firmware (CRC-16/XMODEM).
    uint16_t crc = 0;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

size_t hoverSpeedFrame(uint8_t *out, uint8_t slave, int16_t speed)
{
    out[0] = '/';
    out[1] = 0;              // iDataType 0 = SerialServer2Hover
    out[2] = slave;
    out[3] = (uint8_t)(speed & 0xFF);
    out[4] = (uint8_t)((uint16_t)speed >> 8);
    out[5] = 0;              // wState (LEDs), unused
    uint16_t crc = hoverCrc(out, 6);
    out[6] = (uint8_t)(crc & 0xFF);
    out[7] = (uint8_t)(crc >> 8);
    return 8;
}

bool HoverAnswerParser::feed(uint8_t c, HoverAnswer *answer)
{
    // Start frame 0xABCD, low byte first.
    if (pos_ == 0 && c != 0xCD) {
        return false;
    }
    if (pos_ == 1 && c != 0xAB) {
        pos_ = (c == 0xCD) ? 1 : 0;
        return false;
    }
    buf_[pos_++] = c;
    if (pos_ < kSize) {
        return false;
    }
    pos_ = 0;
    uint16_t crc = (uint16_t)buf_[13] | ((uint16_t)buf_[14] << 8);
    if (crc != hoverCrc(buf_, kSize - 2)) {
        badCrc++;
        return false;
    }
    int16_t  speed = (int16_t)(buf_[3] | (buf_[4] << 8));
    uint16_t volt  = (uint16_t)(buf_[5] | (buf_[6] << 8));
    int16_t  amp   = (int16_t)(buf_[7] | (buf_[8] << 8));
    int32_t  odom  = (int32_t)((uint32_t)buf_[9] | ((uint32_t)buf_[10] << 8) |
                               ((uint32_t)buf_[11] << 16) | ((uint32_t)buf_[12] << 24));
    answer->slave      = buf_[2];
    answer->rpm        = speed / 10.0f;
    answer->volt       = volt / 100.0f;
    answer->iq         = amp / 100.0f;
    answer->odom       = odom;
    answer->receivedMs = millis();
    return true;
}
