import time

try:
    from xarm.wrapper import XArmAPI
except ImportError:
    from xarm import XArmAPI


# =========================================================
# CONNECTION
# =========================================================

ARM_IP = "192.168.1.206"

arm = XArmAPI(ARM_IP)

arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)

time.sleep(1)

# =========================================================
# SPEEDS
# =========================================================

FAST_SPEED = 80
SLOW_SPEED = 30

FAST_ACCEL = 200
SLOW_ACCEL = 80

SAFE_Z = 260

# =========================================================
# POSITIONS
# =========================================================

INITIAL_POS = [-64.8, -245.5, 301.5]

WEIGH_BOAT_POS = [279.7, -555.5, 11]

SCALE_POS = [-201.5, -277.0, 88.3]

POWDER_POS = [190.8, -322.4, 93.3]

SCOOP_POS = [-42.1, -246.5, 94.9]

POWDER_RELEASE_POS = [177.7, -555.2, 123.1]

# =========================================================
# ORIENTATIONS
# =========================================================

DEFAULT_RPY = [0.0, 180.0, 0.0]

SCOOP_RPY = [-178.5, -2.0, 91.4]

RELEASE_RPY = [180.0, 0.0, -87.9]

# =========================================================
# JOINT CONFIGURATIONS
# =========================================================

SCOOP_JOINTS = [
    -99,
    -36.7,
    -11.7,
    -1.2,
    50.1,
    171.2
]

RELEASE_JOINTS = [
    -72.3,
    26.6,
    -84.9,
    0.6,
    58.4,
    15.9
]

# =========================================================
# GRIPPER VALUES
# =========================================================

# weigh boat
GRIPPER_PICK = 200
GRIPPER_RELEASE = 250

# scoop
GRIPPER_PICK_SCOOP = 715
GRIPPER_OPEN_SCOOP = 550
GRIPPER_RELEASE_SCOOP = 850

# =========================================================
# HELPER FUNCTIONS
# =========================================================

def move_cartesian(
    pos,
    rpy=DEFAULT_RPY,
    speed=FAST_SPEED,
    accel=FAST_ACCEL
):

    print(f"Moving to {pos}")

    code = arm.set_position(
        x=pos[0],
        y=pos[1],
        z=pos[2],
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

    print(f"Moving joints: {joints}")

    code = arm.set_servo_angle(
        angle=joints,
        is_radian=False,
        speed=20,
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

    time.sleep(0.8)


def safe_above(pos, rpy=DEFAULT_RPY):

    move_cartesian(
        [pos[0], pos[1], SAFE_Z],
        rpy=rpy
    )


def descend(
    pos,
    rpy=DEFAULT_RPY,
    speed=SLOW_SPEED,
    accel=SLOW_ACCEL
):

    move_cartesian(
        pos,
        rpy=rpy,
        speed=speed,
        accel=accel
    )


def retract(pos, rpy=DEFAULT_RPY):

    move_cartesian(
        [pos[0], pos[1], SAFE_Z],
        rpy=rpy
    )


# =========================================================
# SCOOP PICKUP
# =========================================================

def pickup_scoop():

    print("\n=== PICKING SCOOP ===")

    gripper(GRIPPER_RELEASE_SCOOP)

    move_joints(SCOOP_JOINTS)

    safe_above(SCOOP_POS, SCOOP_RPY)

    descend(
        SCOOP_POS,
        rpy=SCOOP_RPY
    )

    gripper(GRIPPER_PICK_SCOOP)

    retract(
        SCOOP_POS,
        rpy=SCOOP_RPY
    )


# =========================================================
# SCOOP POWDER
# =========================================================

def scoop_powder():

    print("\n=== SCOOPING POWDER ===")

    safe_above(POWDER_POS, SCOOP_RPY)

    # open scoop before entering powder
    gripper(GRIPPER_OPEN_SCOOP)

    descend(
        POWDER_POS,
        rpy=SCOOP_RPY
    )

    time.sleep(1)

    # close scoop to trap powder
    gripper(GRIPPER_PICK_SCOOP)

    time.sleep(1)

    retract(
        POWDER_POS,
        rpy=SCOOP_RPY
    )


# =========================================================
# RELEASE POWDER
# =========================================================

def release_powder():

    print("\n=== RELEASING POWDER ===")

    move_joints(RELEASE_JOINTS)

    safe_above(
        POWDER_RELEASE_POS,
        RELEASE_RPY
    )

    descend(
        POWDER_RELEASE_POS,
        rpy=RELEASE_RPY
    )

    # open scoop to release powder
    gripper(GRIPPER_OPEN_SCOOP)

    time.sleep(2)

    # close scoop again
    gripper(GRIPPER_PICK_SCOOP)

    retract(
        POWDER_RELEASE_POS,
        rpy=RELEASE_RPY
    )


# =========================================================
# RETURN SCOOP
# =========================================================

def return_scoop():

    print("\n=== RETURNING SCOOP ===")

    move_joints(SCOOP_JOINTS)

    safe_above(
        SCOOP_POS,
        SCOOP_RPY
    )

    descend(
        SCOOP_POS,
        rpy=SCOOP_RPY
    )

    gripper(GRIPPER_RELEASE_SCOOP)

    time.sleep(1)

    retract(
        SCOOP_POS,
        rpy=SCOOP_RPY
    )


# =========================================================
# PICK WEIGH BOAT
# =========================================================

def pickup_weighboat():

    print("\n=== PICKING WEIGH BOAT ===")

    gripper(GRIPPER_RELEASE)

    safe_above(WEIGH_BOAT_POS)

    descend(WEIGH_BOAT_POS)

    gripper(GRIPPER_PICK)

    retract(WEIGH_BOAT_POS)


# =========================================================
# MOVE TO SCALE
# =========================================================

def move_to_scale():

    print("\n=== MOVING TO SCALE ===")

    safe_above(SCALE_POS)

    descend(SCALE_POS)

    gripper(GRIPPER_RELEASE)

    print("\n=== WAITING FOR SCALE READING ===")

    time.sleep(5)

    gripper(GRIPPER_PICK)

    retract(SCALE_POS)


# =========================================================
# RETURN WEIGH BOAT
# =========================================================

def return_weighboat():

    print("\n=== RETURNING WEIGH BOAT ===")

    safe_above(WEIGH_BOAT_POS)

    descend(WEIGH_BOAT_POS)

    gripper(GRIPPER_RELEASE)

    time.sleep(1)

    retract(WEIGH_BOAT_POS)


# =========================================================
# MAIN
# =========================================================

def main():

    print("\n=== STARTING PROGRAM ===")

    # home
    move_cartesian(INITIAL_POS)

    # -----------------------------------------------------
    # SCOOP WORKFLOW
    # -----------------------------------------------------

    pickup_scoop()

    scoop_powder()

    release_powder()

    return_scoop()

    # -----------------------------------------------------
    # WEIGH BOAT WORKFLOW
    # -----------------------------------------------------

    pickup_weighboat()

    move_to_scale()

    return_weighboat()

    # -----------------------------------------------------
    # RETURN HOME
    # -----------------------------------------------------

    print("\n=== RETURNING HOME ===")

    move_cartesian(INITIAL_POS)

    arm.disconnect()

    print("\n=== PROCESS COMPLETE ===")


if __name__ == "__main__":
    main()
