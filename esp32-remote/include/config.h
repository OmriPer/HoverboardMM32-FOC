// Settings for the ESP32 hoverboard remote. Change these, then rebuild.
#pragma once

// Wi-Fi hotspot the phone connects to. The page is at http://192.168.4.1
#define WIFI_SSID           "Hoverboard"
#define WIFI_PASSWORD       "hoverboard123"   // at least 8 characters; change it

// UART to the master board's PC header (UART1: master TX1 -> ESP32 RX, master RX1 <- ESP32 TX),
// 3.3 V logic on both sides. Connect GND as well.
#define HOVER_RX_PIN        16
#define HOVER_TX_PIN        17
#define HOVER_BAUD          19200

// Boards (SLAVE_ID in the firmware). The master relays frames for the slave.
#define LEFT_ID             1
#define RIGHT_ID            2
// Wheel direction: the two hub motors are mounted mirrored, so the same speed command turns them
// opposite ways. Set so that "forward" on the phone drives both wheels forward.
#define LEFT_DIR            1
#define RIGHT_DIR           -1

// Speed commands. The boards run FOC speed mode (DRIVEMODE 5), so commands are in rpm.
#define SPEED_LIMIT_RPM     300    // hard cap, whatever the page sends
#define DEFAULT_MAX_RPM     100    // page slider start value
#define RAMP_RPM_PER_S      300    // how fast a wheel command may change

// Safety: both wheels get 0 when the phone sends nothing for this long (and on disconnect).
// The boards themselves stop after 1 s without frames (SERIAL_TIMEOUT).
#define PHONE_TIMEOUT_MS    300

// Frames per second to each board. Both boards share the 19200 baud line.
#define SEND_RATE_HZ        20
