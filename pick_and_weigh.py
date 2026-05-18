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

SAFE_Z = 260

DEFAULT_RPY = [0.0, 180.0, 0.0]

# =========================================================
# POSITIONS
# =========================================================

INITIAL_POS = [-64.8, -245.5, 301.5]

WEIGH_BOAT_POS = [279.7, -555.5, 10.5]

SCALE_POS = [-201.5, -277.0, 88.3]

POWDER_POS = [190.8, -322.4, 93.3]

SCOOP_POS = [-42.0, -246.5, 94.9]

# =========================================================
# SCOOP JOINT ORIENTATION
# =========================================================

SCOOP_JOINTS = [
    -99,
    -36.7,
    -11.7,
    -1.2,
    50.1,
    171.2
]

# =========================================================
# GRIPPER VALUES
# =========================================================

# Weigh boat
GRIPPER_PICK_WEIGHBOAT = 200
GRIPPER_RELEASE_WEIGHBOAT = 250

# Scoop
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

def move_pose(
    x,
    y,
    z,
    rpy=DEFAULT_RPY,
    speed=MOVE_SPEED,
    accel=MOVE_ACCEL
):

    print(f"Moving to {[x, y, z]}")

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


def move_joints(joints):

    print(f"Moving joints -> {joints}")

    code = arm.set_servo_angle(
        angle=joints,
        is_radian=False,
        speed=30,
        wait=True
    )

    if isinstance(code, (list, tuple)):
        code = code[0]

    if code != 0:
        raise RuntimeError(f"Joint move failed: {code}")


def gripper(position):

    print(f"Gripper -> {position}")

    code = arm.set_gripper_position(
        position,
        wait=True
    )

    if isinstance(code, (list, tuple)):
        code = code[0]

    if code != 0:
        raise RuntimeError(f"Gripper failed: {code}")

    time.sleep(0.7)


def move_above(position):

    move_pose(
        position[0],
        position[1],
        SAFE_Z
    )


def descend(position):

    move_pose(
        position[0],
        position[1],
        position[2]
    )


def retract(position):

    move_pose(
        position[0],
        position[1],
        SAFE_Z
    )


# =========================================================
# SCOOP FUNCTIONS
# =========================================================

def pick_scoop():

    print("\n=== PICKING SCOOP ===")

    # move to scoop joint orientation
    move_joints(SCOOP_JOINTS)

    # open robot gripper
    gripper(GRIPPER_RELEASE_SCOOP)

    move_above(SCOOP_POS)

    descend(SCOOP_POS)

    # grab scoop tool
    gripper(GRIPPER_PICK_SCOOP)

    retract(SCOOP_POS)


def scoop_powder():

    print("\n=== SCOOPING POWDER ===")

    move_above(POWDER_POS)

    # OPEN scoop before entering powder
    gripper(GRIPPER_OPEN_SCOOP)

    descend(POWDER_POS)

    # CLOSE scoop to capture powder
    gripper(GRIPPER_PICK_SCOOP)

    time.sleep(1)

    retract(POWDER_POS)


def release_powder_into_weighboat():

    print("\n=== RELEASING POWDER INTO WEIGH BOAT ===")

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

    # RELEASE powder by opening scoop
    gripper(GRIPPER_OPEN_SCOOP)

    time.sleep(1)

    # re-close scoop
    gripper(GRIPPER_PICK_SCOOP)


def return_scoop():

    print("\n=== RETURNING SCOOP ===")

    move_above(SCOOP_POS)

    descend(SCOOP_POS)

    # release scoop tool
    gripper(GRIPPER_RELEASE_SCOOP)

    retract(SCOOP_POS)


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

    approach = [
        SCALE_POS[0],
        SCALE_POS[1],
        150
    ]

    move_pose(*approach)

    descend(SCALE_POS)

    # place on scale
    gripper(GRIPPER_RELEASE_WEIGHBOAT)

    print("\n=== WAITING FOR SCALE READING ===")

    time.sleep(5)

    # pick it back up
    gripper(GRIPPER_PICK_WEIGHBOAT)

    retract(SCALE_POS)


def return_weighboat():

    print("\n=== RETURNING WEIGH BOAT ===")

    approach = [
        WEIGH_BOAT_POS[0],
        WEIGH_BOAT_POS[1],
        120
    ]

    move_pose(*approach)

    descend(WEIGH_BOAT_POS)

    # release weigh boat back
    gripper(GRIPPER_RELEASE_WEIGHBOAT)

    time.sleep(1)

    retract(WEIGH_BOAT_POS)


# =========================================================
# MAIN
# =========================================================

def main():

    print("\n=== STARTING PROGRAM ===")

    move_pose(*INITIAL_POS)

    # -----------------------------------------------------
    # SCOOP PROCESS
    # -----------------------------------------------------

    pick_scoop()

    scoop_powder()

    release_powder_into_weighboat()

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

    print("\n=== COMPLETE ===")


if __name__ == "__main__":
    main()
