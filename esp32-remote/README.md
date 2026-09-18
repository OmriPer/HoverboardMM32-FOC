# ESP32 Wi-Fi remote for the hoverboard

Drive both wheels from a phone browser. The ESP32 runs a Wi-Fi hotspot with a control page and
sends speed commands to the master board over the RemoteUartBus protocol; the master relays the
slave's frames (see `HoverBoardMindMotion/Inc/board_config.h`).

```
phone --Wi-Fi--> ESP32 --UART 19200--> master (id 1, left) --cable--> slave (id 2, right)
```

## Wiring (ESP32 DevKit)

| ESP32           | Master board PC header (UART1) |
|-----------------|--------------------------------|
| GPIO16 (RX2)    | TX1 (PD0)                      |
| GPIO17 (TX2)    | RX1 (PD1)                      |
| GND             | GND                            |

Both sides are 3.3 V logic. Disconnect the USB-UART adapter from the header while the ESP32 is
connected: two transmitters on the master's RX line corrupt each other. Power the ESP32 from USB
(power bank) or a separate 5 V supply.

## Build and flash

1. VS Code with the PlatformIO extension, open this folder (`esp32-remote`).
2. Connect the ESP32 over USB, click **Upload** (or `pio run -t upload`).
3. Optional: **Monitor** (115200 baud) shows the Wi-Fi name and address at startup.

Settings are in `include/config.h`: Wi-Fi name/password, UART pins, which board is left/right,
wheel directions, speed limits, ramp rate and the phone timeout.

## Use

1. Power the hoverboard as usual (bypass switch OFF), wheels lifted for the first test.
2. Phone: join Wi-Fi **Hoverboard** (password in `config.h`), open **http://192.168.4.1**.
3. The dot top left turns green when connected; each wheel card turns green when that board
   answers.
4. **Joystick**: up/down = speed, left/right = turn. **Tank**: one slider per wheel. Both spring
   back to 0 when released. **Max** sets the speed for full stick (rpm, boards in FOC speed mode).

First test: small stick movements, check that "forward" turns both wheels forward. If a wheel
turns backwards, flip its `LEFT_DIR`/`RIGHT_DIR` in `config.h`; if left and right are swapped,
swap `LEFT_ID`/`RIGHT_ID`.

## Safety

- The page sends its input every 100 ms. If the ESP32 hears nothing for 300 ms (phone locked,
  browser in background, Wi-Fi lost), both wheels ramp to 0.
- **STOP** sets both commands to 0 at once.
- Commands are ramped (`RAMP_RPM_PER_S`) and capped (`SPEED_LIMIT_RPM`).
- If the ESP32 itself stops, the boards stop after 1 s without frames (firmware `SERIAL_TIMEOUT`).
- The boards' own current limit (`FOC_I_MOT_MAX`, 5 A) still applies.
