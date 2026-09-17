#!/usr/bin/env python3
"""PC-side RemoteUartBus tool for the hoverboard firmware (Src/remoteUartBus.c).

Sends speed commands and decodes the board's answers over a USB-UART adapter,
for bench bring-up. Standard library only (no pyserial).

Protocol (packed little-endian structs, CRC-16/XMODEM over all bytes except the CRC):
  PC -> board, type 0  SerialServer2Hover:       '/' 0 slave speed:int16 wState:uint8 crc:uint16
  PC -> board, type 2  SerialServer2HoverConfig:  '/' 2 slave battFull:float battEmpty:float
                                                  driveMode:uint8 slaveNew:int8 crc:uint16
  board -> PC          SerialHover2Server:        0xABCD slave speed:int16 volt:uint16 amp:int16
                                                  odom:int32 crc:uint16
The board answers after every valid command addressed to its SLAVE_ID, and sets the
speed command to 0 when no valid command arrives for SERIAL_TIMEOUT (1000 ms).

Units in the answer: volt = V*100, amp = A*100 (always 0 on this board: no DC current
sensing), speed = realspeed*10. With the EFeru controller (FOC_EFERU) realspeed is the
motor speed in rpm, so the tool shows speed/10 as rpm.

Examples:
  remote_uart_bus.py monitor                       # speed 0, print telemetry
  remote_uart_bus.py run --speed 100 --seconds 5   # constant command, then stop
  remote_uart_bus.py interactive                   # +/- keys change the command
  remote_uart_bus.py config --drive-mode 2         # select DRIVEMODE (not kept over reboot
                                                   #  with EEPROMEN 0; writes board flash)
"""
import argparse
import os
import select
import struct
import sys
import termios
import time
import tty

START_ANSWER = 0xABCD
ANSWER_FMT = "<HBhHhiH"          # cStart, iSlave, iSpeed, iVolt, iAmp, iOdom, checksum
ANSWER_SIZE = struct.calcsize(ANSWER_FMT)
DRIVE_MODES = {0: "COM_VOLT", 1: "COM_SPEED", 2: "SINE_VOLT", 3: "SINE_SPEED",
               4: "FOC_VOLT", 5: "FOC_SPEED", 6: "FOC_TORQUE"}


def crc16_xmodem(data: bytes) -> int:
    """Same algorithm as CalcCRC() in Src/remoteUartBus.c."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def frame_speed(slave: int, speed: int, w_state: int = 0) -> bytes:
    body = struct.pack("<BBBhB", ord("/"), 0, slave, max(-1000, min(1000, speed)), w_state)
    return body + struct.pack("<H", crc16_xmodem(body))


def frame_config(slave: int, batt_full: float, batt_empty: float, drive_mode: int) -> bytes:
    # Values the firmware ignores: battery 0 (it only accepts 0 < V < 60), slaveNew -1.
    body = struct.pack("<BBBffBb", ord("/"), 2, slave, batt_full, batt_empty, drive_mode, -1)
    return body + struct.pack("<H", crc16_xmodem(body))


class AnswerParser:
    """Finds SerialHover2Server frames in the byte stream and checks their CRC."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(struct.pack("<H", START_ANSWER))
            if i < 0:
                del self.buf[:-1]
                return out
            if len(self.buf) - i < ANSWER_SIZE:
                del self.buf[:i]
                return out
            raw = bytes(self.buf[i:i + ANSWER_SIZE])
            _, slave, speed, volt, amp, odom, crc = struct.unpack(ANSWER_FMT, raw)
            if crc == crc16_xmodem(raw[:-2]):
                out.append({"slave": slave, "speed": speed, "volt": volt / 100.0,
                            "amp": amp / 100.0, "odom": odom})
                del self.buf[:i + ANSWER_SIZE]
            else:
                self.bad_crc += 1
                del self.buf[:i + 1]


def open_port(dev: str, baud: int) -> int:
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, "B%d" % baud)
    attrs[0] = 0                                             # iflag
    attrs[1] = 0                                             # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag: 8N1, no flow control
    attrs[3] = 0                                             # lflag: raw
    attrs[4] = attrs[5] = speed
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def read_available(fd: int, timeout: float) -> bytes:
    r, _, _ = select.select([fd], [], [], timeout)
    if not r:
        return b""
    try:
        return os.read(fd, 256)
    except BlockingIOError:
        return b""


def show(answer: dict, command: int):
    print(f"cmd {command:+5d} | speed {answer['speed'] / 10:+8.1f} rpm | "
          f"{answer['volt']:6.2f} V | {answer['amp']:+6.2f} A | odom {answer['odom']:+8d} | "
          f"slave {answer['slave']}", flush=True)


def stop_motor(fd: int, slave: int):
    for _ in range(5):
        os.write(fd, frame_speed(slave, 0))
        time.sleep(0.05)


def drive(fd, args, get_command, stop_after=None, on_tick=None):
    parser = AnswerParser()
    period = 1.0 / args.rate
    t_start = time.monotonic()
    next_send = t_start
    last_print = 0.0
    answers = 0
    try:
        while stop_after is None or time.monotonic() - t_start < stop_after:
            now = time.monotonic()
            if now >= next_send:
                command = get_command()
                if command is None:
                    break
                os.write(fd, frame_speed(args.slave, command))
                next_send = now + period
            for a in parser.feed(read_available(fd, max(0.0, next_send - time.monotonic()))):
                answers += 1
                if time.monotonic() - last_print >= args.print_interval:
                    show(a, get_command.last)
                    last_print = time.monotonic()
            if on_tick:
                on_tick()
            if answers == 0 and time.monotonic() - t_start > 2.0 and not getattr(drive, "warned", False):
                print("no answers yet: check wiring (TX1/RX1), baud, SLAVE_ID, and that the board runs",
                      file=sys.stderr)
                drive.warned = True
    finally:
        stop_motor(fd, args.slave)
        print(f"stopped. answers {answers}, CRC errors {parser.bad_crc}")


class Command:
    def __init__(self, value=0):
        self.last = value

    def __call__(self):
        return self.last


def cmd_monitor(fd, args):
    drive(fd, args, Command(0))


def cmd_run(fd, args):
    if abs(args.speed) > args.max_speed:
        sys.exit(f"--speed {args.speed} exceeds --max-speed {args.max_speed}")
    drive(fd, args, Command(args.speed), stop_after=args.seconds)


def cmd_interactive(fd, args):
    command = Command(0)
    print(f"keys: + / - change command by {args.step} (limit ±{args.max_speed}), "
          "space or 0 = stop, q = quit")
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    def keys():
        while select.select([sys.stdin], [], [], 0)[0]:
            k = sys.stdin.read(1)
            if k in "+=":
                command.last = min(args.max_speed, command.last + args.step)
            elif k in "-_":
                command.last = max(-args.max_speed, command.last - args.step)
            elif k in " 0":
                command.last = 0
            elif k in "qQ":
                raise KeyboardInterrupt

    try:
        drive(fd, args, command, on_tick=keys)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def cmd_config(fd, args):
    if args.drive_mode not in DRIVE_MODES:
        sys.exit(f"--drive-mode must be one of {DRIVE_MODES}")
    # The firmware drops the first frame after line noise, so retry, but only while there is no
    # answer: every accepted config frame writes the board's flash (EEPROM_Write).
    parser = AnswerParser()
    for attempt in range(1, 4):
        os.write(fd, frame_config(args.slave, args.batt_full, args.batt_empty, args.drive_mode))
        t_end = time.monotonic() + 0.5
        while time.monotonic() < t_end:
            for a in parser.feed(read_available(fd, 0.1)):
                print(f"config accepted by slave {a['slave']} (attempt {attempt}): "
                      f"DRIVEMODE {args.drive_mode} ({DRIVE_MODES[args.drive_mode]})")
                return
    print("no answer to the config frame (wrong SLAVE_ID, wiring, or board not running)")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=19200)
    p.add_argument("--slave", type=int, default=1, help="SLAVE_ID of the board (default 1)")
    p.add_argument("--rate", type=float, default=20.0, help="commands per second (default 20)")
    p.add_argument("--print-interval", type=float, default=0.25)
    p.add_argument("--max-speed", type=int, default=300,
                   help="refuse commands above this magnitude (default 300 of 1000, for bench safety)")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("monitor", help="send speed 0 and print telemetry")
    r = sub.add_parser("run", help="send a constant speed command, then stop")
    r.add_argument("--speed", type=int, required=True)
    r.add_argument("--seconds", type=float, default=5.0)
    i = sub.add_parser("interactive", help="change the speed command with + and - keys")
    i.add_argument("--step", type=int, default=20)
    c = sub.add_parser("config", help="send a config frame (select DRIVEMODE)")
    c.add_argument("--drive-mode", type=int, required=True)
    c.add_argument("--batt-full", type=float, default=0.0, help="volts; 0 = leave unchanged")
    c.add_argument("--batt-empty", type=float, default=0.0, help="volts; 0 = leave unchanged")
    args = p.parse_args()

    fd = open_port(args.port, args.baud)
    try:
        {"monitor": cmd_monitor, "run": cmd_run, "interactive": cmd_interactive,
         "config": cmd_config}[args.cmd](fd, args)
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
