import time

try:
    from xarm.wrapper import XArmAPI
except ImportError:
    from xarm import XArmAPI


# =========================================================
# CONFIG
# =========================================================

ARM_IP = "192.168.1.206"

MOVE_SPEED = 80
MOVE_ACCEL = 200

SAFE_Z = 250

DEFAULT_RPY = [0.0, 180.0, 0.0]
POUR_RPY = [0.0, 150.0, 45.0]

# =========================================================
# POSITIONS
# =========================================================

INITIAL_POS = [-64.8, -245.5, 301.5]

WEIGH_BOAT_POS = [279.7, -555.5, 10.5]
WEIGH_BOAT_DEST = [-195.5, -269.4, 84.8]

SCALE_PICK_POS = [-201.5, -277.0, 88.3]

REACTOR_POS = [384.9, 175.4, 731.8]

POWDER_POS = [190.8, -322.4, 93.3]

SCOOP_INITIAL = [-42.0, -246.5, 94.9]

# =========================================================
# GRIPPER VALUES
# =========================================================

GRIPPER_PICK_WEIGHBOAT = 200
GRIPPER_RELEASE_WEIGHBOAT = 250

GRIPPER_PICK_SCOOP = 725
GRIPPER_OPEN_SCOOP = 500
GRIPPER_RELEASE_SCOOP = 850

# =========================================================
# ARM SETUP
# =========================================================

arm = XArmAPI(ARM_IP)

arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)

time.sleep(1)


# =========================================================
# HELPER FUNCTIONS
# =========================================================

def move_pose(x, y, z, rpy=DEFAULT_RPY,
              speed=MOVE_SPEED,
              accel=MOVE_ACCEL):

    print(f"Moving to: {[x, y, z]}")

    code = arm.set_position(
        x=x,
        y=y,
        z=z,
        roll=rpy[0],
        pitch=rpy[1],
        yaw=rpy[2],
        speed=speed,
        mvacc=accel,
        wait=True
    )

    if isinstance(code, (list, tuple)):
        code = code[0]

    if code != 0:
        raise RuntimeError(f"Move failed: {code}")


def move_above(position, offset=SAFE_Z):
    move_pose(position[0], position[1], offset)


def descend(position, rpy=DEFAULT_RPY):
    move_pose(position[0], position[1], position[2], rpy=rpy)


def retract(position):
    move_pose(position[0], position[1], SAFE_Z)


def gripper(position):
    print(f"Gripper -> {position}")

    code = arm.set_gripper_position(position, wait=True)

    if isinstance(code, (list, tuple)):
        code = code[0]

    if code != 0:
        raise RuntimeError(f"Gripper failed: {code}")

    time.sleep(0.5)


# =========================================================
# SCOOP FUNCTIONS
# =========================================================

def pick_scoop():

    print("\n=== PICKING SCOOP ===")

    gripper(GRIPPER_OPEN_SCOOP)

    move_above(SCOOP_INITIAL)

    descend(SCOOP_INITIAL)

    gripper(GRIPPER_PICK_SCOOP)

    retract(SCOOP_INITIAL)


def scoop_powder():

    print("\n=== SCOOPING POWDER ===")

    move_above(POWDER_POS)

    # descend into powder
    descend(POWDER_POS)

    # simulated scoop motion
    move_pose(
        POWDER_POS[0] + 40,
        POWDER_POS[1],
        POWDER_POS[2] - 10
    )

    move_pose(
        POWDER_POS[0] + 70,
        POWDER_POS[1],
        POWDER_POS[2]
    )

    retract(POWDER_POS)


def pour_into_weighboat():

    print("\n=== POURING INTO WEIGH BOAT ===")

    above_boat = [
        WEIGH_BOAT_POS[0],
        WEIGH_BOAT_POS[1],
        120
    ]

    move_pose(
        above_boat[0],
        above_boat[1],
        above_boat[2]
    )

    # tilt scoop to pour
    move_pose(
        above_boat[0],
        above_boat[1],
        above_boat[2],
        rpy=POUR_RPY
    )

    time.sleep(2)

    # return orientation
    move_pose(
        above_boat[0],
        above_boat[1],
        above_boat[2]
    )


def return_scoop():

    print("\n=== RETURNING SCOOP ===")

    move_above(SCOOP_INITIAL)

    descend(SCOOP_INITIAL)

    gripper(GRIPPER_RELEASE_SCOOP)

    retract(SCOOP_INITIAL)


# =========================================================
# WEIGH BOAT FUNCTIONS
# =========================================================

def pick_weighboat():

    print("\n=== PICKING WEIGH BOAT ===")

    gripper(GRIPPER_RELEASE_WEIGHBOAT)

    approach = [
        WEIGH_BOAT_POS[0],
        WEIGH_BOAT_POS[1],
        120
    ]

    move_pose(*approach)

    descend(WEIGH_BOAT_POS)

    gripper(GRIPPER_PICK_WEIGHBOAT)

    retract(WEIGH_BOAT_POS)


def move_weighboat_to_scale():

    print("\n=== MOVING WEIGH BOAT TO SCALE ===")

    move_above(SCALE_PICK_POS)

    descend(SCALE_PICK_POS)

    gripper(GRIPPER_RELEASE_WEIGHBOAT)

    print("\n=== WAITING FOR SCALE READING ===")
    time.sleep(5)

    gripper(GRIPPER_PICK_WEIGHBOAT)

    retract(SCALE_PICK_POS)


def return_weighboat():

    print("\n=== RETURNING WEIGH BOAT ===")

    move_above(WEIGH_BOAT_POS)

    descend(WEIGH_BOAT_POS)

    gripper(GRIPPER_RELEASE_WEIGHBOAT)

    retract(WEIGH_BOAT_POS)


# =========================================================
# MAIN PROGRAM
# =========================================================

def main():

    print("\n=== STARTING PROGRAM ===")

    # Home
    move_pose(*INITIAL_POS)

    # -----------------------------------------------------
    # SCOOP PROCESS
    # -----------------------------------------------------

    pick_scoop()

    scoop_powder()

    pour_into_weighboat()

    return_scoop()

    # -----------------------------------------------------
    # WEIGH BOAT PROCESS
    # -----------------------------------------------------

    pick_weighboat()

    move_weighboat_to_scale()

    return_weighboat()

    # -----------------------------------------------------
    # RETURN HOME
    # -----------------------------------------------------

    print("\n=== RETURNING HOME ===")

    move_pose(*INITIAL_POS)

    arm.disconnect()

    print("\n=== PROCESS COMPLETE ===")


if __name__ == "__main__":
    main()
