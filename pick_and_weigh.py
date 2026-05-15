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
POUR_ORIENTATION_RPY = [0.0, 120.0, 0.0]
DEFAULT_SCALE_PICKUP = [-201.5, -277.0, 88.3]
DEFAULT_REACTOR_POSITION = [384.9, 175.4, 731.8]
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


def build_pose(point: List[float], orientation: Optional[List[float]] = None, z_override: Optional[float] = None) -> List[float]:
    x, y, z = point
    if z_override is not None:
        z = z_override
    orientation = orientation or ORIENTATION_RPY
    return [x, y, z, orientation[0], orientation[1], orientation[2]]


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


def main() -> None:
    positions = load_positions(POSITIONS_FILE)
    initial = positions["initial_position"]  # type: ignore[assignment]
    weigh_boat = positions["weigh_boat_position"]  # type: ignore[assignment]
    destination = positions["weigh_boat_destination"]  # type: ignore[assignment]
    scale_pickup = positions.get("scale_pickup_position", DEFAULT_SCALE_PICKUP)  # type: ignore[assignment]
    reactor = positions.get("reactor_position", DEFAULT_REACTOR_POSITION)  # type: ignore[assignment]
    gripper_pick = int(positions.get("gripper_pick", GRIP_CLOSE))
    gripper_release = int(positions.get("gripper_release", GRIP_OPEN))

    table_clearance = 150.0
    safe_z = max(initial[2], weigh_boat[2] + table_clearance, destination[2] + table_clearance, reactor[2] + table_clearance, 220.0)
    pickup_z = weigh_boat[2] + 35.0
    place_z = destination[2] + 35.0
    scale_pickup_approach = scale_pickup[2] + 40.0
    reactor_safe_z = reactor[2] + 150.0
    reactor_pour_z = reactor[2] + 60.0

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

    print("Waiting briefly before the pour sequence...")
    time.sleep(1.0)

    print("Picking weigh boat back up from the scale...")
    send_position(arm, build_pose([scale_pickup[0], scale_pickup[1], safe_z]))
    send_position(arm, build_pose([scale_pickup[0], scale_pickup[1], scale_pickup_approach]))
    send_position(arm, build_pose(scale_pickup))
    move_gripper(arm, gripper_pick)
    time.sleep(0.5)

    print("Lifting weigh boat after scale pickup...")
    send_position(arm, build_pose([scale_pickup[0], scale_pickup[1], safe_z]))

    print("Moving to reactor safe position...")
    send_position(arm, build_pose([reactor[0], reactor[1], reactor_safe_z]))
    send_position(arm, build_pose([reactor[0], reactor[1], reactor_pour_z], POUR_ORIENTATION_RPY))

    print("Pouring powder into reactor...")
    time.sleep(1.5)

    print("Retracting from reactor...")
    send_position(arm, build_pose([reactor[0], reactor[1], reactor_safe_z], POUR_ORIENTATION_RPY))

    print("Returning weigh boat to start position...")
    send_position(arm, build_pose(initial))

    arm.disconnect()
    print("Completed the weigh boat pour sequence.")


if __name__ == "__main__":
    main()

