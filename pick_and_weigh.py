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

    # --- New: wait at initial pose briefly, then regrasp for pouring ---
    print("Returning to initial pose briefly before regrasp...")
    send_position(arm, build_pose([destination[0], destination[1], safe_z]))
    send_position(arm, build_pose(initial))
    time.sleep(1.0)

    # Helper: move joints if available (useful to change orientation safely)
    def move_joints_safe(arm_obj: XArmAPI, joints: List[float], wait: bool = True) -> None:
        try:
            # SDK: set_servo_angle expects a list of 6 angles in degrees
            arm_obj.set_servo_angle(joints, is_radian=False, wait=wait)
        except Exception:
            # If set_servo_angle isn't available, ignore and continue with cartesian moves
            print("Joint move not supported on this SDK; skipping joint orientation step.")

    # Skipping joint reorientation per user request; use current orientation.
    # Move to a safe height before regrasping without changing orientation.
    print("Moving up for safe reorientation (no orientation change)...")
    send_position(arm, build_pose([initial[0], initial[1], safe_z]))
    time.sleep(0.2)

    # Approach the weigh boat from the regrasp orientation but stay offset in X to avoid the scale
    offset = 80.0
    # approach_offset: start offset from the weigh boat along +X direction
    # Use an incremental, waypoint-based transit to avoid kinematic singularities
    def safe_cartesian_transit(target_xyz, fast_speed=60, slow_speed=40, fast_mvacc=120, slow_mvacc=80):
        tx, ty, tz = target_xyz
        # midpoint between current (initial) and target at safe_z
        mid = [(initial[0] + tx) / 2.0, (initial[1] + ty) / 2.0, safe_z]
        try:
            send_position(arm, build_pose(mid), speed=fast_speed, mvacc=fast_mvacc)
            send_position(arm, build_pose([tx, ty, safe_z]), speed=fast_speed, mvacc=fast_mvacc)
        except Exception as e:
            print(f"Primary transit failed: {e}. Falling back to finer steps.")
            # try finer-grained interpolation
            for t in (0.25, 0.5, 0.75, 1.0):
                ix = initial[0] + (tx - initial[0]) * t
                iy = initial[1] + (ty - initial[1]) * t
                send_position(arm, build_pose([ix, iy, safe_z]), speed=slow_speed, mvacc=slow_mvacc)
        # finally lower to requested tz
        send_position(arm, build_pose([tx, ty, tz]), speed=slow_speed, mvacc=slow_mvacc)

    approach_target = [weigh_boat[0] + offset, weigh_boat[1], pickup_z]
    print("Moving to approach offset away from weigh boat with safe transit...")
    safe_cartesian_transit(approach_target)

    # Now translate along X to the exact weigh boat XY while at pickup_z
    print("Translating along X to weigh boat exact XY at pickup height...")
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], pickup_z]), speed=40, mvacc=80)

    print("Grasping weigh boat for transfer to reactor...")
    move_gripper(arm, gripper_pick)
    time.sleep(0.6)

    # Lift and transit to reactor approach
    print("Lifting weigh boat to safe transit height...")
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], safe_z]))

    reactor_pos = [384.9, 175.4, 731.8]
    # Approach reactor above by first moving to same XY at safe_z
    print("Transiting to reactor approach position...")
    send_position(arm, build_pose([reactor_pos[0], reactor_pos[1], safe_z]))
    # Move down to reactor height (keep a small overhead)
    reactor_approach_z = reactor_pos[2]
    send_position(arm, build_pose([reactor_pos[0], reactor_pos[1], reactor_approach_z]))

    # Perform a simple pour motion by adjusting wrist yaw/pitch slightly
    print("Performing pour motion into reactor...")
    try:
        # Tilt by changing yaw (or use joint move to tilt wrist)
        pour_pose = [reactor_pos[0], reactor_pos[1], reactor_approach_z, ORIENTATION_RPY[0], ORIENTATION_RPY[1], ORIENTATION_RPY[2] + 70.0]
        send_position(arm, pour_pose)
        time.sleep(0.8)
        # Return from pour pose
        send_position(arm, build_pose([reactor_pos[0], reactor_pos[1], reactor_approach_z]))
    except Exception:
        print("Pour motion failed or not supported; continuing.")

    # After pouring, return the weigh boat to its original position on the scale
    print("Returning weigh boat to original scale position...")
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], safe_z]))
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], pickup_z]))
    move_gripper(arm, gripper_release)
    time.sleep(0.6)

    # Retract and return to initial pose
    print("Retracting and returning to initial pose...")
    send_position(arm, build_pose([weigh_boat[0], weigh_boat[1], safe_z]))
    send_position(arm, build_pose(initial))

    arm.disconnect()
    print("Completed pick, pour, and return sequence.")


if __name__ == "__main__":
    main()



