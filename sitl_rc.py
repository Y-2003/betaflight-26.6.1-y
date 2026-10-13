import socket
import struct
import threading
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 9004

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

channels = [1500] * 16

channels[0] = 1500  # CH1 Roll
channels[1] = 1500  # CH2 Pitch
channels[2] = 1000  # CH3 Throttle
channels[3] = 1500  # CH4 Yaw
channels[4] = 1000
channels[5] = 1000
channels[6] = 1000
channels[7] = 1000
channels[8] = 1000
channels[9] = 1000
channels[10] = 1000
channels[11] = 1000
channels[12] = 1000
channels[13] = 1000
channels[14] = 1000
channels[15] = 1000

running = True
lock = threading.Lock()


# [新增] 功能：打印当前 16 个 SITL RC 通道值。
def print_status():
    with lock:
        print("\nCurrent RC:")
        for i in range(16):
            label = ("  # Roll", "  # Pitch", "  # Throttle", "  # Yaw")[i] if i < 4 else ""
            print(f"  CH{i + 1}: {channels[i]}{label}")
        print()


# [新增] 功能：在后台读取命令行并更新 RC 通道或触发开关脉冲。
def command_thread():
    global running

    print("""
Commands:
  ch1 <value>            # Roll
  ch2 <value>            # Pitch
  ch3 <value>            # Throttle
  ch4 <value>            # Yaw
  ch5 <value>
  ch6 <value>
  ch7 <value>
  ch8 <value>
  ch9 <value>
  ch10 <value>
  ch11 <value>
  ch12 <value>
  ch13 <value>
  ch14 <value>
  ch15 <value>
  ch16 <value>

  ch <number> <value>    e.g. ch 5 2000

  ud<number>             e.g. ud5 (also accepts ud+5)

  status
  help
  quit
""")

    while running:
        try:
            line = input("> ").strip().lower()
        except EOFError:
            running = False
            break

        if not line:
            continue

        parts = line.split()

        try:
            if len(parts) == 1 and parts[0].startswith("ud"):
                channel_text = parts[0][2:].lstrip("+")
                ch = int(channel_text)

                if not 1 <= ch <= 16:
                    print("Channel must be 1..16")
                    continue

                with lock:
                    channels[ch - 1] = 2000
                print(f"UD CH{ch} = 2000")

                time.sleep(0.5)

                with lock:
                    channels[ch - 1] = 1000
                print(f"UD CH{ch} = 1000")

            elif len(parts) == 2 and parts[0].startswith("ch"):
                ch = int(parts[0][2:])
                value = int(parts[1])

                if not 1 <= ch <= 16:
                    print("Channel must be 1..16")
                    continue

                if not 800 <= value <= 2200:
                    print("Value should normally be 800..2200")
                    continue

                with lock:
                    channels[ch - 1] = value

                print(f"CH{ch} = {value}")

            elif parts[0] == "ch" and len(parts) == 3:
                ch = int(parts[1])
                value = int(parts[2])

                if not 1 <= ch <= 16:
                    print("Channel must be 1..16")
                    continue

                if not 800 <= value <= 2200:
                    print("Value should normally be 800..2200")
                    continue

                with lock:
                    channels[ch - 1] = value

                print(f"CH{ch} = {value}")

            elif parts == ["status"]:
                print_status()

            elif parts == ["help"]:
                print("""
Commands:
  ch1..ch16 <value>
  ch <number> <value>
  ud<number> (or ud+<number>)
  status
  quit
""")

            elif parts == ["quit"]:
                running = False

            else:
                print("Unknown command. Type 'help'.")

        except ValueError:
            print("Invalid number.")


thread = threading.Thread(target=command_thread, daemon=True)
thread.start()

print(f"Sending RC to {UDP_IP}:{UDP_PORT} at 50 Hz")

try:
    while running:
        packet = struct.pack("<d", time.time())

        with lock:
            current_channels = channels.copy()

        for ch in current_channels:
            packet += struct.pack("<H", ch)

        sock.sendto(packet, (UDP_IP, UDP_PORT))

        time.sleep(0.02)

except KeyboardInterrupt:
    running = False

finally:
    sock.close()
    print("\nStopped.")
