#!/usr/bin/env python3

import math
import subprocess


WORLD_NAME = "betaloop_demo"
MODEL_NAME = "iris"
SERVICE_TIMEOUT_MS = 1000

position = {
    "x": 0.0,
    "y": 0.0,
    "z": 50.0,
}

attitude_deg = {
    "roll": 60.0,
    "pitch": 0.0,
    "yaw": 0.0,
}


def quaternion_from_euler(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return {
        "w": cr * cp * cy + sr * sp * sy,
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
    }


def current_quaternion():
    return quaternion_from_euler(
        attitude_deg["roll"],
        attitude_deg["pitch"],
        attitude_deg["yaw"],
    )


def print_status():
    quaternion = current_quaternion()

    print("\nCurrent Gazebo pose:")
    print(f"  World : {WORLD_NAME}")
    print(f"  Model : {MODEL_NAME}")
    print(f"  Position (m) : x={position['x']:.3f} "
          f"y={position['y']:.3f} z={position['z']:.3f}")
    print(f"  Attitude (deg): roll={attitude_deg['roll']:.3f} "
          f"pitch={attitude_deg['pitch']:.3f} "
          f"yaw={attitude_deg['yaw']:.3f}")
    print(f"  Quaternion   : w={quaternion['w']:.7f} "
          f"x={quaternion['x']:.7f} "
          f"y={quaternion['y']:.7f} "
          f"z={quaternion['z']:.7f}\n")


def apply_pose():
    quaternion = current_quaternion()
    request = (
        f'name: "{MODEL_NAME}", '
        f"position: {{x: {position['x']}, y: {position['y']}, "
        f"z: {position['z']}}}, "
        f"orientation: {{w: {quaternion['w']}, x: {quaternion['x']}, "
        f"y: {quaternion['y']}, z: {quaternion['z']}}}"
    )

    command = [
        "gz",
        "service",
        "-s",
        f"/world/{WORLD_NAME}/set_pose",
        "--reqtype",
        "gz.msgs.Pose",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        str(SERVICE_TIMEOUT_MS),
        "--req",
        request,
    ]

    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        print("gz command not found.")
        return

    output = result.stdout.strip()
    error = result.stderr.strip()

    if result.returncode == 0 and "true" in output.lower():
        print("Pose applied.")
        return

    print("Failed to apply pose. Is Gazebo running?")
    if output:
        print(output)
    if error:
        print(error)


def print_help():
    print("""
Commands:
  x <meters>
  y <meters>
  z <meters>

  roll <degrees>
  pitch <degrees>
  yaw <degrees>

  pose <x> <y> <z> <roll> <pitch> <yaw>
  level                     set roll/pitch/yaw to zero
  apply                     move the model to the configured pose
  status
  help
  quit
""")


def main():
    print_help()
    print_status()

    while True:
        try:
            parts = input("> ").strip().lower().split()
        except (EOFError, KeyboardInterrupt):
            print("\nStopped.")
            break

        if not parts:
            continue

        try:
            if parts[0] in position and len(parts) == 2:
                position[parts[0]] = float(parts[1])
                print(f"{parts[0]} = {position[parts[0]]} m")

            elif parts[0] in attitude_deg and len(parts) == 2:
                attitude_deg[parts[0]] = float(parts[1])
                print(f"{parts[0]} = {attitude_deg[parts[0]]} deg")

            elif parts[0] == "pose" and len(parts) == 7:
                values = [float(value) for value in parts[1:]]
                position["x"], position["y"], position["z"] = values[:3]
                attitude_deg["roll"], attitude_deg["pitch"], attitude_deg["yaw"] = values[3:]
                print_status()

            elif parts == ["level"]:
                attitude_deg["roll"] = 0.0
                attitude_deg["pitch"] = 0.0
                attitude_deg["yaw"] = 0.0
                print("Attitude set to level.")

            elif parts == ["apply"]:
                apply_pose()

            elif parts == ["status"]:
                print_status()

            elif parts == ["help"]:
                print_help()

            elif parts in (["quit"], ["exit"]):
                print("Stopped.")
                break

            else:
                print("Unknown command. Type 'help'.")

        except ValueError:
            print("Invalid number.")


if __name__ == "__main__":
    main()
