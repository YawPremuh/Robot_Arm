import os
import re
import time
from typing import Dict, List, Optional

try:
    from xarm.wrapper import XArmAPI
except ImportError:
    from xarm import XArmAPI

ARM_IP = "192.168.1.206"
GRIP_CLOSE = 200
GRIP_OPEN = 250
MOVE_SPEED = 100
MOVE_ACCEL = 250
ORIENTATION_RPY = [0.0, 180.0, 0.0]
POSITIONS_FILE = os.path.join(os.path.dirname(__file__), "positions.txt")


def parse_vector(line: str) -> Optional[List[float]]:
    if "[" not in line or "]" not in line:
        return None
    numbers = re.findall(r"-?\d+(?:\.\d+)?", line)
    if len(numbers) < 3:
        return None
    return [float(numbers[0]), float(numbers[1]), float(numbers[2])]


def load_positions(file_path: str) -> Dict[str, object]:
    positions: Dict[str, object] = {}
    with open(file_path, "r", encoding="utf-8") as fp:
        for raw_line in fp:
            if ":" not in raw_line:
                continue
            key, value = raw_line.split(":", 1)
            key = key.strip().lower()
            value = value.strip()
            if not key:
                continue
            vector = parse_vector(value)
            if vector is not None:
                positions[key] = vector
                continue
            scalar_match = re.search(r"-?\d+(?:\.\d+)?", value)
            if scalar_match:
                positions[key] = float(scalar_match.group())
    missing = [name for name in ("initial_position", "weigh_boat_position", "weigh_boat_destination") if name not in positions]
    if missing:
        raise ValueError(f"Missing required positions in {file_path}: {missing}")
    return positions


def build_pose(point: List[float], z_override: Optional[float] = None) -> List[float]:
    x, y, z = point
    if z_override is not None:
        z = z_override
    return [x, y, z, ORIENTATION_RPY[0], ORIENTATION_RPY[1], ORIENTATION_RPY[2]]


def send_position(
    arm: XArmAPI,
    pose: List[float],
    speed: float = MOVE_SPEED,
    mvacc: float = MOVE_ACCEL
) -> None:

    print(f"Sending pose: {pose}")

    result = arm.set_position(
        x=pose[0],
        y=pose[1],
        z=pose[2],
        roll=pose[3],
        pitch=pose[4],
        yaw=pose[5],
        speed=speed,
        mvacc=mvacc,
        wait=True
    )

    if isinstance(result, (list, tuple)):
        if result[0] != 0:
            raise RuntimeError(f"xArm move failed: {result}")
    elif result != 0:
        raise RuntimeError(f"xArm move failed: {result}")


def move_gripper(arm: XArmAPI, position: int) -> None:
    result = arm.set_gripper_position(position, wait=True)
    if isinstance(result, (list, tuple)):
        if result[0] != 0:
            raise RuntimeError(f"Gripper move failed: {result}")
    elif result != 0:
        raise RuntimeError(f"Gripper move failed: {result}")


def set_joint_angles(arm: XArmAPI, angles: List[float], speed: float = 50) -> None:
    """Set joint angles (degrees). Tries common SDK signatures."""
    try:
        result = arm.set_servo_angle(angle=angles, speed=speed, is_radian=False, wait=True)
    except TypeError:
        try:
            result = arm.set_servo_angle(angles, wait=True)
        except Exception as exc:  # fallback
            raise RuntimeError(f"Failed to set servo angles: {exc}")
    if isinstance(result, (list, tuple)):
        if result[0] != 0:
            raise RuntimeError(f"Set joint angles failed: {result}")
    elif result != 0:
        raise RuntimeError(f"Set joint angles failed: {result}")


def main() -> None:
    positions = load_positions(POSITIONS_FILE)
    initial = positions["initial_position"]  # type: ignore[assignment]
    weigh_boat = positions["weigh_boat_position"]  # type: ignore[assignment]
    destination = positions["weigh_boat_destination"]  # type: ignore[assignment]
    gripper_pick = int(positions.get("gripper_pick", GRIP_CLOSE))
    gripper_release = int(positions.get("gripper_release", GRIP_OPEN))

    table_clearance = 150.0
    safe_z = max(initial[2], weigh_boat[2] + table_clearance, destination[2] + table_clearance, 220.0)
    pickup_z = weigh_boat[2] + 35.0
    place_z = destination[2] + 35.0

    arm = XArmAPI(ARM_IP)
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(1.0)

    print("Opening gripper and moving to initial pose...")
    move_gripper(arm, GRIP_OPEN)
    send_position(arm, build_pose(initial))

    print("Moving to safe above weigh boat...")
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], safe_z]))
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], weigh_boat[2]]))

    print("Closing gripper on weigh boat...")
    move_gripper(arm, gripper_pick)
    time.sleep(0.5)

    print("Lifting weigh boat to safe transit height...")
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], safe_z]))

    print("Transiting to safe above destination...")
    send_position(arm, build_pose([destination[0], destination[1], safe_z]))
    send_position(arm, build_pose([destination[0], destination[1], destination[2]]))

    print("Releasing weigh boat at destination...")
    move_gripper(arm, gripper_release)
    time.sleep(0.5)

    print("Retracting to safe height and returning to initial pose...")
    send_position(arm, build_pose([destination[0], destination[1], safe_z]))
    send_position(arm, build_pose(initial))

    # brief pause at start pose before re-picking for pour
    print("Pausing at start pose...")
    time.sleep(2.0)

    # Prepare for pick-up in pour orientation (use provided joint angles)
    pour_joints = [-76.5, 29.0, -32.0, 283.7, 93.9, 175.0]
    reactor = [384.9, 175.4, 731.8]

    print("Setting pour orientation and moving to scale to re-pick weigh boat...")
    set_joint_angles(arm, pour_joints)
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], safe_z]))
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], pickup_z]))

    print("Closing gripper to secure weigh boat for pour...")
    move_gripper(arm, gripper_pick)
    time.sleep(0.5)
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], safe_z]))

    # Move to reactor and perform a tilt to pour
    reactor_safe_z = reactor[2] + 50.0
    print("Moving to reactor and performing pour motion...")
    send_position(arm, build_pose([reactor[0], reactor[1], reactor_safe_z]))

    # Tilt wrist (joint 4) to pour and perform a small oscillation
    tilted = pour_joints.copy()
    tilted[3] = tilted[3] + 40.0
    set_joint_angles(arm, tilted)
    time.sleep(0.6)
    for delta in (8.0, -8.0, 8.0):
        tilted[3] += delta
        set_joint_angles(arm, tilted)
        time.sleep(0.25)

    # Return orientation and bring weigh boat back to start
    set_joint_angles(arm, pour_joints)
    time.sleep(0.4)

    print("Returning weigh boat to start position and releasing...")
    send_position(arm, build_pose(initial))
    move_gripper(arm, gripper_release)
    time.sleep(0.5)

    arm.disconnect()
    print("Completed full pick, pour, return sequence.")


if __name__ == "__main__":
    main()

