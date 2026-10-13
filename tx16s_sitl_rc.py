#!/usr/bin/env python3
"""RadioMaster TX16S MK3 (EdgeTX 3.x) to Betaflight SITL RC bridge.

This profile expects an EdgeTX model with these mixer outputs:

    CH1  Roll       CH5  SA (3-pos)   CH9  SE (3-pos)   CH13 S1
    CH2  Pitch      CH6  SB (3-pos)   CH10 SF (2-pos)   CH14 S2
    CH3  Throttle   CH7  SC (3-pos)   CH11 SG (3-pos)   CH15 LS
    CH4  Yaw        CH8  SD (3-pos)   CH12 SH (moment.) CH16 RS

It also expects EdgeTX Model Settings > USB Joystick > Extended mode with
the axis/button assignments documented in print_edgetx_setup(). Classic mode
cannot preserve the middle position of channels above CH8.
"""

import argparse
import os
import socket
import struct
import sys
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import pygame


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 9004
SEND_PERIOD = 0.02

# Pygame axis index -> zero-based SITL channel index.
# The order matches the EdgeTX axis assignments printed by --setup.
AXIS_CHANNELS = (0, 1, 2, 3, 12, 13, 14, 15)
REVERSE_AXES = set()

# EdgeTX Extended mode encodes a multi-position switch as a one-hot group.
# channel: (control name, first USB button, number of positions)
SWITCHES = {
    4: ("SA", 0, 3),
    5: ("SB", 3, 3),
    6: ("SC", 6, 3),
    7: ("SD", 9, 3),
    8: ("SE", 12, 3),
    9: ("SF", 15, 2),
    10: ("SG", 17, 3),
    11: ("SH", 20, 2),
}

EXPECTED_AXES = 8
EXPECTED_BUTTONS = 22


# [新增] 功能：打印 TX16S Extended USB Joystick 的通道配置要求。
def print_edgetx_setup():
    print(
        """EdgeTX model mixer outputs
--------------------------
CH1=Ail  CH2=Ele  CH3=Thr  CH4=Rud
CH5=SA   CH6=SB   CH7=SC   CH8=SD
CH9=SE   CH10=SF  CH11=SG  CH12=SH
CH13=S1  CH14=S2  CH15=LS  CH16=RS

EdgeTX Model Settings > USB Joystick
------------------------------------
Extended mode: ON
Interface mode: Joystick

Channel  Mode    Assignment                    Invert
CH1      Axis    X                             No
CH2      Axis    Y                             No
CH3      Axis    Z                             No
CH4      Axis    Rx                            No
CH5      Button  Normal, 3 positions, button 0 No
CH6      Button  Normal, 3 positions, button 3 No
CH7      Button  Normal, 3 positions, button 6 No
CH8      Button  Normal, 3 positions, button 9 No
CH9      Button  Normal, 3 positions, button 12 No
CH10     Button  Normal, 2 positions, button 15 No
CH11     Button  Normal, 3 positions, button 17 No
CH12     Button  Normal, 2 positions, button 20 No
CH13     Axis    Ry                            No
CH14     Axis    Rz                            No
CH15     Axis    Slider                        No
CH16     Axis    Dial                          No

Press Apply changes, disconnect USB, reconnect it, and select USB Joystick.
Windows joy.cpl should then show 8 axes and 22 buttons.
"""
    )


# [新增] 功能：将 USB 摇杆轴值转换为 1000 到 2000 的 PWM 值。
def axis_to_pwm(value, reverse=False):
    value = max(-1.0, min(1.0, value))
    if reverse:
        value = -value
    return max(1000, min(2000, round(1500 + value * 500)))


# [新增] 功能：将一组 USB 按钮解码为二段或三段开关 PWM 值。
def switch_to_pwm(joystick, first_button, positions):
    active = [
        position
        for position in range(positions)
        if first_button + position < joystick.get_numbuttons()
        and joystick.get_button(first_button + position)
    ]
    if len(active) != 1:
        return 1000, active
    if positions == 2:
        return (1000, 2000)[active[0]], active
    return (1000, 1500, 2000)[active[0]], active


# [新增] 功能：读取摇杆全部轴和开关并生成 16 路 RC 通道。
def read_channels(joystick):
    channels = [1000] * 16
    invalid_switches = []

    for axis, channel in enumerate(AXIS_CHANNELS):
        channels[channel] = axis_to_pwm(
            joystick.get_axis(axis), axis in REVERSE_AXES
        )

    for channel, (name, first_button, positions) in SWITCHES.items():
        channels[channel], active = switch_to_pwm(
            joystick, first_button, positions
        )
        if len(active) != 1:
            invalid_switches.append((name, active))

    return channels, invalid_switches


# [新增] 功能：按索引查找并初始化 TX16S USB 摇杆。
def find_joystick(index):
    count = pygame.joystick.get_count()
    if count == 0:
        raise SystemExit(
            "No USB joystick found. Reconnect the TX16S and select USB Joystick."
        )
    if not 0 <= index < count:
        raise SystemExit(f"Joystick index {index} does not exist; found {count} device(s).")

    joystick = pygame.joystick.Joystick(index)
    joystick.init()
    return joystick


# [新增] 功能：检查摇杆轴/按钮布局是否符合 EdgeTX Extended 配置。
def validate_layout(joystick):
    axes = joystick.get_numaxes()
    buttons = joystick.get_numbuttons()

    if axes == 8 and buttons == 24:
        raise SystemExit(
            "The TX16S is in Classic USB Joystick mode (8 axes, 24 buttons).\n"
            "Classic mode discards the middle position above CH8. Run this script\n"
            "with --setup and configure EdgeTX Extended mode first."
        )
    if axes < EXPECTED_AXES or buttons < EXPECTED_BUTTONS:
        raise SystemExit(
            f"Unexpected HID layout: {axes} axes, {buttons} buttons; expected at "
            f"least {EXPECTED_AXES} axes and {EXPECTED_BUTTONS} buttons.\n"
            "Run with --setup and apply the EdgeTX USB Joystick assignments."
        )
    if axes != EXPECTED_AXES or buttons != EXPECTED_BUTTONS:
        print(
            f"Warning: HID reports {axes} axes and {buttons} buttons; "
            f"this profile expects {EXPECTED_AXES}/{EXPECTED_BUTTONS}.",
            file=sys.stderr,
        )


# [新增] 功能：将 RC 通道数组格式化为便于监视的文本。
def format_channels(channels):
    return " ".join(f"CH{i + 1}={value:4d}" for i, value in enumerate(channels))


# [新增] 功能：持续显示原始 HID 输入及其解码后的 RC 通道。
def diagnose(joystick):
    print("Move one control at a time; Ctrl+C stops.")
    previous = None
    try:
        while True:
            pygame.event.pump()
            axes = tuple(
                round(joystick.get_axis(i), 3)
                for i in range(joystick.get_numaxes())
            )
            buttons = tuple(
                i
                for i in range(joystick.get_numbuttons())
                if joystick.get_button(i)
            )
            channels, invalid = read_channels(joystick)
            state = (axes, buttons, tuple(channels))
            if state != previous:
                print(f"axes={axes}")
                print(f"active_buttons={buttons}")
                print(format_channels(channels))
                if invalid:
                    print(f"invalid_switch_groups={invalid}")
                print()
                previous = state
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass


# [新增] 功能：解析主机、端口、摇杆和诊断模式等命令行参数。
def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST, help="SITL IPv4 address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="SITL RC UDP port")
    parser.add_argument("--device", type=int, default=0, help="Pygame joystick index")
    parser.add_argument("--diagnose", action="store_true", help="show raw HID and decoded RC values")
    parser.add_argument("--monitor", action="store_true", help="print decoded RC values while sending")
    parser.add_argument("--setup", action="store_true", help="print the required EdgeTX setup and exit")
    return parser.parse_args()


# [新增] 功能：读取 TX16S USB 摇杆并以 50 Hz 向 SITL 发送 16 路 RC 数据。
def main():
    args = parse_args()
    if args.setup:
        print_edgetx_setup()
        return 0

    pygame.init()
    pygame.joystick.init()
    joystick = find_joystick(args.device)
    print(f"Joystick: {joystick.get_name()}")
    print(
        f"HID layout: {joystick.get_numaxes()} axes, "
        f"{joystick.get_numbuttons()} buttons"
    )
    validate_layout(joystick)

    try:
        if args.diagnose:
            diagnose(joystick)
            return 0

        destination = (args.host, args.port)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"Sending 16 RC channels to {args.host}:{args.port} at 50 Hz")
        print("Press Ctrl+C to stop")

        last_monitor = 0.0
        last_invalid = None
        while True:
            pygame.event.pump()
            channels, invalid = read_channels(joystick)
            sock.sendto(struct.pack("<d16H", time.time(), *channels), destination)

            now = time.monotonic()
            if args.monitor and now - last_monitor >= 0.25:
                print("\r" + format_channels(channels), end="", flush=True)
                last_monitor = now
            invalid_tuple = tuple((name, tuple(active)) for name, active in invalid)
            if invalid_tuple and invalid_tuple != last_invalid:
                print(
                    f"\nInvalid switch encoding: {invalid}. "
                    "Check EdgeTX Extended USB settings.",
                    file=sys.stderr,
                )
            last_invalid = invalid_tuple
            time.sleep(SEND_PERIOD)
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        pygame.quit()
        if "sock" in locals():
            sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
