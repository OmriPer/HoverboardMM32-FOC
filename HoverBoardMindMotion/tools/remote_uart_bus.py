#!/usr/bin/env python3
"""PC-side RemoteUartBus tool for the hoverboard firmware (Src/remoteUartBus.c).

Sends speed commands and decodes the board's answers over a USB-UART adapter,
for bench bring-up. Standard library only (no pyserial).

Protocol (packed little-endian structs, CRC-16/XMODEM over all bytes except the CRC):
  PC -> board, type 0  SerialServer2Hover:       '/' 0 slave speed:int16 wState:uint8 crc:uint16
  PC -> board, type 2  SerialServer2HoverConfig:  '/' 2 slave battFull:float battEmpty:float
                                                  driveMode:uint8 slaveNew:int8 crc:uint16
  PC -> board, type 3  SerialServer2HoverFocLimits: '/' 3 slave currentMax:uint16 (0.1 A)
                                                  speedMax:uint16 (rpm) crc:uint16   (RAM only)
  board -> PC          SerialHover2Server:        0xABCD slave speed:int16 volt:uint16 amp:int16
                                                  odom:int32 crc:uint16
The board answers after every valid command addressed to its SLAVE_ID, and sets the
speed command to 0 when no valid command arrives for SERIAL_TIMEOUT (1000 ms).

Units in the answer: volt = V*100, amp = A*100, speed = realspeed*10. With the EFeru
controller (FOC_EFERU) realspeed is the motor speed in rpm, so the tool shows speed/10 as
rpm, and amp is the filtered FOC torque current iq (this board has no DC current sensing;
0 in the COM and SINE modes).

Examples:
  remote_uart_bus.py monitor                       # speed 0, print telemetry
  remote_uart_bus.py run --speed 100 --seconds 5   # constant command, then stop
  remote_uart_bus.py interactive                   # +/- keys change the command
  remote_uart_bus.py --slaves 1,2 interactive      # both boards: 1/2 select one, a = both
  remote_uart_bus.py config --drive-mode 2         # select DRIVEMODE (not kept over reboot
                                                   #  with EEPROMEN 0; writes board flash)
  remote_uart_bus.py limits --current 2 --speed 150  # FOC current [A] / speed [rpm] limits,
                                                   #  RAM only, back to foc_config.h on reboot
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


def frame_limits(slave: int, current_a: float, speed_rpm: int) -> bytes:
    body = struct.pack("<BBBHH", ord("/"), 3, slave, int(round(current_a * 10)), speed_rpm)
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
          f"{answer['volt']:6.2f} V | iq {answer['amp']:+6.2f} A | odom {answer['odom']:+8d} | "
          f"slave {answer['slave']}", flush=True)


def stop_motors(fd: int, slaves):
    for _ in range(5):
        for slave in slaves:
            os.write(fd, frame_speed(slave, 0))
            time.sleep(0.02)


class Commands:
    """Speed command per board. `selected` is the board the interactive keys change (None = all)."""

    def __init__(self, slaves, value=0):
        self.values = {s: value for s in slaves}
        self.selected = None

    def targets(self):
        return list(self.values) if self.selected is None else [self.selected]


def drive(fd, args, commands, stop_after=None, on_tick=None):
    """Send each board's command at args.rate (frames alternate between the boards) and print the
    answers. With several boards the master relays the frames for the others (see the firmware's
    Inc/board_config.h)."""
    slaves = list(commands.values)
    parser = AnswerParser()
    period = 1.0 / (args.rate * len(slaves))
    t_start = time.monotonic()
    next_send = t_start
    last_print = {s: 0.0 for s in slaves}
    answers = {s: 0 for s in slaves}
    turn = 0
    try:
        while stop_after is None or time.monotonic() - t_start < stop_after:
            now = time.monotonic()
            if now >= next_send:
                slave = slaves[turn % len(slaves)]
                turn += 1
                os.write(fd, frame_speed(slave, commands.values[slave]))
                next_send = now + period
            for a in parser.feed(read_available(fd, max(0.0, next_send - time.monotonic()))):
                s = a["slave"]
                answers[s] = answers.get(s, 0) + 1
                if time.monotonic() - last_print.get(s, 0.0) >= args.print_interval:
                    show(a, commands.values.get(s, 0))
                    last_print[s] = time.monotonic()
            if on_tick:
                on_tick()
            silent = [s for s in slaves if answers.get(s, 0) == 0]
            if silent and time.monotonic() - t_start > 2.0 and not getattr(drive, "warned", False):
                print(f"no answers yet from slave {silent}: check wiring (TX1/RX1), baud, SLAVE_ID, "
                      "the master-slave cable, and that the boards run", file=sys.stderr)
                drive.warned = True
    finally:
        stop_motors(fd, slaves)
        counts = ", ".join(f"slave {s}: {answers.get(s, 0)}" for s in slaves)
        print(f"stopped. answers {counts}, CRC errors {parser.bad_crc}")


def cmd_monitor(fd, args):
    drive(fd, args, Commands(args.slaves))


def cmd_run(fd, args):
    if abs(args.speed) > args.max_speed:
        sys.exit(f"--speed {args.speed} exceeds --max-speed {args.max_speed}")
    drive(fd, args, Commands(args.slaves, args.speed), stop_after=args.seconds)


def cmd_interactive(fd, args):
    commands = Commands(args.slaves)
    select_keys = "".join(str(s) for s in args.slaves if 0 <= s <= 9)
    help_select = (f", {'/'.join(select_keys)} = control that board only, a = all boards"
                   if len(args.slaves) > 1 else "")
    print(f"keys: + / - change command by {args.step} (limit \u00b1{args.max_speed}){help_select}, "
          "space or 0 = stop all, q = quit")
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    def change(delta):
        for s in commands.targets():
            commands.values[s] = max(-args.max_speed, min(args.max_speed, commands.values[s] + delta))

    def keys():
        while select.select([sys.stdin], [], [], 0)[0]:
            k = sys.stdin.read(1)
            if k in "+=":
                change(args.step)
            elif k in "-_":
                change(-args.step)
            elif k == " " or (k == "0" and "0" not in select_keys):
                for s in commands.values:
                    commands.values[s] = 0
            elif len(args.slaves) > 1 and k in select_keys:
                commands.selected = int(k)
                print(f"-> controlling slave {k}", flush=True)
            elif len(args.slaves) > 1 and k in "aA":
                commands.selected = None
                print("-> controlling all boards", flush=True)
            elif k in "qQ":
                raise KeyboardInterrupt

    try:
        drive(fd, args, commands, on_tick=keys)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def cmd_config(fd, args):
    if args.drive_mode not in DRIVE_MODES:
        sys.exit(f"--drive-mode must be one of {DRIVE_MODES}")
    for slave in args.slaves:
        config_one(fd, args, slave)


def config_one(fd, args, slave):
    # The firmware drops the first frame after line noise, so retry, but only while there is no
    # answer: every accepted config frame writes the board's flash (EEPROM_Write).
    parser = AnswerParser()
    for attempt in range(1, 4):
        os.write(fd, frame_config(slave, args.batt_full, args.batt_empty, args.drive_mode))
        t_end = time.monotonic() + 0.5
        while time.monotonic() < t_end:
            for a in parser.feed(read_available(fd, 0.1)):
                print(f"config accepted by slave {a['slave']} (attempt {attempt}): "
                      f"DRIVEMODE {args.drive_mode} ({DRIVE_MODES[args.drive_mode]})")
                return
    print(f"no answer from slave {slave} to the config frame (wrong SLAVE_ID, wiring, or board not running)")


def cmd_limits(fd, args):
    if args.current is None and args.speed is None:
        sys.exit("give --current and/or --speed")
    current = args.current or 0.0
    speed = args.speed or 0
    if not 0 <= current <= 15 or not 0 <= speed <= 1000:
        sys.exit("--current must be 0..15 A, --speed 0..1000 rpm (0 = unchanged)")
    for slave in args.slaves:
        limits_one(fd, args, slave, current, speed)


def limits_one(fd, args, slave, current, speed):
    parser = AnswerParser()
    for attempt in range(1, 4):
        os.write(fd, frame_limits(slave, current, speed))
        t_end = time.monotonic() + 0.5
        while time.monotonic() < t_end:
            for a in parser.feed(read_available(fd, 0.1)):
                print(f"limits accepted by slave {a['slave']} (attempt {attempt}): "
                      f"current {current or 'unchanged'} A, speed {speed or 'unchanged'} rpm")
                return
    print(f"no answer from slave {slave} to the limits frame (wrong SLAVE_ID, wiring, or board not running)")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=19200)
    p.add_argument("--slave", type=int, default=1, help="SLAVE_ID of the board (default 1)")
    p.add_argument("--slaves", type=lambda v: [int(x) for x in v.split(",")],
                   help="several boards, comma separated, e.g. --slaves 1,2 (the master relays to the "
                        "others); overrides --slave")
    p.add_argument("--rate", type=float, default=20.0, help="commands per second per board (default 20)")
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
    lim = sub.add_parser("limits", help="set FOC current/speed limits (RAM only)")
    lim.add_argument("--current", type=float, help="current limit in A (0.1 A steps)")
    lim.add_argument("--speed", type=int, help="speed limit in rpm")
    args = p.parse_args()
    args.slaves = args.slaves or [args.slave]

    fd = open_port(args.port, args.baud)
    try:
        {"monitor": cmd_monitor, "run": cmd_run, "interactive": cmd_interactive,
         "config": cmd_config, "limits": cmd_limits}[args.cmd](fd, args)
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
