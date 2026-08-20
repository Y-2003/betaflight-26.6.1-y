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
channels[4] = 1000  # CH5 ARM
channels[5] = 1000  # CH6 AUTOPILOT
channels[6] = 1000  # CH7 autonomous TEST trigger
channels[7] = 1000  # CH8 BOXFAILSAFE

running = True
lock = threading.Lock()


def print_status():
    with lock:
        print("\nCurrent RC:")
        for i in range(8):
            print(f"  CH{i + 1}: {channels[i]}")

        print()
        print(f"  ARM         : {'ON' if channels[4] > 1800 else 'OFF'}")
        print(f"  AUTOPILOT   : {'ON' if channels[5] > 1800 else 'OFF'}")
        print(f"  AUTO TEST   : {'ON' if channels[6] > 1800 else 'OFF'}")
        print(f"  BOXFAILSAFE : {'ON' if channels[7] > 1800 else 'OFF'}")
        print()


def command_thread():
    global running

    print("""
Commands:
  ch <number> <value>   e.g. ch 5 2000

  arm on
  arm off

  autopilot on
  autopilot off

  auto on
  auto off

  boxfailsafe on
  boxfailsafe off

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
            if parts[0] == "ch" and len(parts) == 3:
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

            elif parts == ["arm", "on"]:
                with lock:
                    channels[4] = 2000
                print("ARM ON (CH5=2000)")

            elif parts == ["arm", "off"]:
                with lock:
                    channels[4] = 1000
                print("ARM OFF (CH5=1000)")

            elif parts == ["autopilot", "on"]:
                with lock:
                    channels[5] = 2000
                print("AUTOPILOT ON (CH6=2000)")

            elif parts == ["autopilot", "off"]:
                with lock:
                    channels[5] = 1000
                print("AUTOPILOT OFF (CH6=1000)")

            elif parts == ["auto", "on"]:
                with lock:
                    channels[6] = 2000
                print("AUTO TEST ON (CH7=2000)")

            elif parts == ["auto", "off"]:
                with lock:
                    channels[6] = 1000
                print("AUTO TEST OFF (CH7=1000)")

            elif parts == ["boxfailsafe", "on"]:
                with lock:
                    channels[7] = 2000
                print("BOXFAILSAFE ON (CH8=2000)")

            elif parts == ["boxfailsafe", "off"]:
                with lock:
                    channels[7] = 1000
                print("BOXFAILSAFE OFF (CH8=1000)")

            elif parts == ["status"]:
                print_status()

            elif parts == ["help"]:
                print("""
Commands:
  ch <number> <value>
  arm on/off
  autopilot on/off
  auto on/off
  boxfailsafe on/off
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