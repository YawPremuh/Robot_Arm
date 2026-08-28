import time

try:
    from xarm.wrapper import XArmAPI
except ImportError:
    from xarm import XArmAPI

# =========================================================
# CONNECTION
# =========================================================

ARM_IP = "192.168.1.206"

# =========================================================
# SPEEDS
# =========================================================

FAST_SPEED = 80
SLOW_SPEED = 25

FAST_ACCEL = 200
SLOW_ACCEL = 80

SAFE_Z = 260

# =========================================================
# POSITIONS
# =========================================================

INITIAL_POS = [-64.8, -245.5, 301.5]

WEIGH_BOAT_POS = [279.7, -555.5, 11]

SCALE_POS = [-285, -183, 86]

POWDER_POS = [192, -322.4, 130]

SCOOP_POS = [-42.1, -244, 94.9]

POWDER_RELEASE_POS = [-286.1, -277.0, 200]

POWDER_POUR_POS = [220.6, -326.6, 287]

REACTOR_POS = [403.6, 85.6, 704.5]

REACTOR_APPROACH = [ 403.6, 85.6, 780.0]

# =========================================================
# SCOOP SAFETY
# =========================================================

SCOOP_HEIGHT = 122

# high approach height to avoid knocking scoop over
SCOOP_APPROACH_Z = SCOOP_POS[2] + SCOOP_HEIGHT + 120

# =========================================================
# ORIENTATIONS
# =========================================================

DEFAULT_RPY = [180.0, 0.0, 90.0]

SCOOP_RPY = [-178.5, -2.0, 91.4]

RELEASE_RPY = [180.0, 0.0, -87.9]

POWDER_RELEASE_RPY = [-178.5, -2, 2.1]

POUR_BACK_RPY = [180.0, -35.0, -87.9]

REACTOR_APPROACH_RPY = [106.4, 89.6, 161.9]

REACTOR_FUNNEL_RPY = [106.4, 89.6, 161.9]

POUR_RPY = [60.0, 89.6, 161.9]

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

REACTOR_JOINTS = [
    25.1,
    -31.6,
    -70.3,
    89.6,
    80.3,
    103.3
]

# =========================================================
# GRIPPER VALUES
# =========================================================

# weigh boat
GRIPPER_PICK = 200
GRIPPER_RELEASE = 850

# scoop
GRIPPER_PICK_SCOOP = 715
GRIPPER_OPEN_SCOOP = 450
GRIPPER_RELEASE_SCOOP = 850

# =========================================================
# INITIALIZE ARM
# =========================================================

arm = XArmAPI(ARM_IP)

time.sleep(2)

print("Cleaning warnings/errors...")

arm.clean_warn()
arm.clean_error()

time.sleep(2)

print("Enabling motion...")

arm.motion_enable(enable=True)

time.sleep(2)

print("Setting mode...")

arm.set_mode(0)

time.sleep(1)

print("Setting state...")

arm.set_state(state=0)

time.sleep(3)

# =========================================================
# CHECK ROBOT STATUS
# =========================================================

state = arm.state
error = arm.error_code

print(f"Robot state: {state}")
print(f"Robot error: {error}")

if error != 0:

    print("Robot has actual error")

    arm.disconnect()

    exit()

if state in [0, 2]:

    print("xArm ready")

else:

    print(f"Unexpected state: {state}")

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

def main():

    print("\n=== RECOVERING ROBOT ===")

    arm.clean_warn()
    arm.clean_error()

    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)

    time.sleep(2)

    try:
        move_cartesian(INITIAL_POS)
        print("Robot returned home.")

    except Exception as e:
        print(f"Recovery failed: {e}")


if __name__ == "__main__":
    main()
