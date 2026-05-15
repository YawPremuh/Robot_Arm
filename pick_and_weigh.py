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
POUR_ORIENTATION_DEFAULT = [0.0, 150.0, 45.0]
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
    pose_orientation = orientation if orientation is not None else ORIENTATION_RPY
    return [x, y, z, pose_orientation[0], pose_orientation[1], pose_orientation[2]]


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
    scale_pick = positions.get("scale_pick_position", destination)  # type: ignore[assignment]
    reactor = positions.get("reactor_position")  # type: ignore[assignment]
    pour_orientation = positions.get("pour_orientation", POUR_ORIENTATION_DEFAULT)  # type: ignore[assignment]
    gripper_pick = int(positions.get("gripper_pick", GRIP_CLOSE))
    gripper_release = int(positions.get("gripper_release", GRIP_OPEN))

    if reactor is None:
        raise ValueError("Missing required reactor_position in positions.txt for pouring.")

    table_clearance = 150.0
    safe_z = max(initial[2], weigh_boat[2] + table_clearance, destination[2] + table_clearance, 220.0)
    pickup_z = weigh_boat[2] + 35.0
    place_z = destination[2] + 35.0
    scale_pick_safe_z = max(scale_pick[2] + 120.0, safe_z)
    scale_pick_down_z = max(destination[2] + 20.0, 10.0)
    reactor_safe_z = max(reactor[2] + 120.0, safe_z)
    reactor_pour_z = max(reactor[2] + 30.0, 10.0)

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
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], pickup_z]))

    print("Closing gripper on weigh boat...")
    move_gripper(arm, gripper_pick)
    time.sleep(0.5)

    print("Lifting weigh boat to safe transit height...")
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], safe_z]))

    print("Transiting to safe above destination...")
    send_position(arm, build_pose([destination[0], destination[1], safe_z]))
    send_position(arm, build_pose([destination[0], destination[1], place_z]))

    print("Releasing weigh boat at destination...")
    move_gripper(arm, gripper_release)
    time.sleep(0.5)

    print("Retracting from scale and returning to initial position...")
    send_position(arm, build_pose([destination[0], destination[1], safe_z]))
    send_position(arm, build_pose(initial))
    print("Holding at initial pose before redesign pick-up...")
    time.sleep(2.0)

    print("Repositioning for scale pickup with pour orientation...")
    send_position(arm, build_pose([scale_pick[0], scale_pick[1], scale_pick_safe_z], orientation=pour_orientation))
    send_position(arm, build_pose([destination[0], destination[1], scale_pick_down_z], orientation=pour_orientation))

    print("Picking weigh boat from the scale...")
    move_gripper(arm, gripper_pick)
    time.sleep(0.5)
    send_position(arm, build_pose([destination[0], destination[1], scale_pick_safe_z], orientation=pour_orientation))

    print("Moving to reactor pour point...")
    send_position(arm, build_pose([reactor[0], reactor[1], reactor_safe_z], orientation=pour_orientation))
    send_position(arm, build_pose([reactor[0], reactor[1], reactor_pour_z], orientation=pour_orientation))

    print("Pouring powder into reactor...")
    time.sleep(2.0)
    send_position(arm, build_pose([reactor[0], reactor[1], reactor_safe_z], orientation=pour_orientation))

    print("Returning weigh boat to original pickup location...")
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], safe_z], orientation=pour_orientation))
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], pickup_z], orientation=pour_orientation))
    move_gripper(arm, gripper_release)
    time.sleep(0.5)
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], safe_z], orientation=pour_orientation))

    print("Returning to initial arm pose...")
    send_position(arm, build_pose(initial))

    arm.disconnect()
    print("Completed full weigh boat transfer, pour, and return sequence.")


if __name__ == "__main__":
    main()
