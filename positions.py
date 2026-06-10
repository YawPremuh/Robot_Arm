import time
import serial
import re

try:
    from xarm.wrapper import XArmAPI
except ImportError:
    from xarm import XArmAPI

# =========================================================
# CONNECTION
# =========================================================

ARM_IP = "192.168.1.206"

SCALE_PORT = "COM19"

scale = serial.Serial(
    port=SCALE_PORT,
    baudrate=9600,
    timeout=1
)

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

POWDER_POS = [193.4, -322.4, 130]

SCOOP_POS = [-42.1, -244, 94.9]

POWDER_RELEASE_POS = [-286.1, -277.0, 200]

POWDER_POUR_POS = [220.6, -326.6, 287]

REACTOR_FUNNEL_POS = [403.6, 85.6, 704.5]

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

DEFAULT_RPY = [0.0, 180.0, 0.0]

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
    
def get_scale_weight():

    """
    Returns current weight in grams
    """

    while True:

        line = scale.readline().decode(
            errors="ignore"
        ).strip()

        match = re.search(
            r'(-?\d+\.\d+)g',
            line
        )

        if match:

            return float(
                match.group(1)
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
# PLACE WEIGH BOAT ON SCALE
# =========================================================

def place_weighboat_on_scale():

    print("\n=== PLACING WEIGH BOAT ON SCALE ===")

    safe_above(
        SCALE_POS,
        RELEASE_RPY
        )

    descend(    
        SCALE_POS,
        rpy=RELEASE_RPY
        )

    time.sleep(1)

    gripper(GRIPPER_RELEASE)

    time.sleep(1)

    # ascend vertically first
    retract(SCALE_POS)

    print("\n=== WEIGH BOAT PLACED SAFELY ===")

# =========================================================
# REGRASP WEIGH BOAT
# =========================================================

def regrasp_weighboat_from_scale():

    print("\n=== REGRASPING WEIGH BOAT ===")

    # first move high above scale
    move_cartesian([
        SCALE_POS[0],
        SCALE_POS[1],
        SAFE_Z
    ])

 

    time.sleep(1)

    # descend vertically with the exact orientation
    move_cartesian(
        SCALE_POS,
        rpy=RELEASE_RPY,
        speed=70,
        accel=40
    )

    time.sleep(1)

    gripper(GRIPPER_PICK)

    time.sleep(1)

    # lift straight up
    move_cartesian(
        [
            SCALE_POS[0],
            SCALE_POS[1],
            SAFE_Z
        ],
        rpy=RELEASE_RPY
    )

    print("\n=== WEIGH BOAT REGRASPED ===")
# =========================================================
# PICKUP SCOOP
# =========================================================

def pickup_scoop():

    print("\n=== PICKING SCOOP ===")

    gripper(GRIPPER_RELEASE_SCOOP)

    # move high above scoop BEFORE rotating
    move_cartesian(
        [SCOOP_POS[0], SCOOP_POS[1], SCOOP_APPROACH_Z]
    )

    # rotate safely above scoop
    move_joints(SCOOP_JOINTS)

    time.sleep(1)

    

    # grab scoop
    gripper(GRIPPER_PICK_SCOOP)

    time.sleep(1)

    # retract vertically
    move_cartesian(
        [SCOOP_POS[0], SCOOP_POS[1], SCOOP_APPROACH_Z],
        rpy=SCOOP_RPY
    )

# =========================================================
# SCOOP POWDER
# =========================================================

def scoop_powder():

    print("\n=== SCOOPING POWDER ===")

    safe_above(POWDER_POS, SCOOP_RPY)

    # open scoop
    gripper(GRIPPER_OPEN_SCOOP)

    move_cartesian(
        POWDER_POS,
        rpy=SCOOP_RPY,
        speed=8,
        accel=30
    )

    time.sleep(1)

    # close scoop
    gripper(GRIPPER_PICK_SCOOP)

    time.sleep(1)

    retract(
        POWDER_POS,
        rpy=SCOOP_RPY
    )

# =========================================================
# RELEASE POWDER INTO WEIGH BOAT
# =========================================================

def release_powder():

    print("\n=== RELEASING POWDER INTO WEIGH BOAT ===")

    

    safe_above(
        POWDER_RELEASE_POS,
        POWDER_RELEASE_RPY
    )

    descend(
        POWDER_RELEASE_POS,
        rpy=POWDER_RELEASE_RPY
    )

    # release powder
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
# MOVE WEIGH BOAT TO REACTOR
# =========================================================

def move_to_reactor():

    print("\n=== MOVING TO REACTOR ===")

    # move to known safe reactor configuration
    move_joints(REACTOR_JOINTS)

    time.sleep(1)

    # move above funnel
    move_cartesian(
        REACTOR_APPROACH,
        rpy=REACTOR_APPROACH_RPY
    )

    # descend
    move_cartesian(
        REACTOR_FUNNEL_POS,
        rpy=REACTOR_FUNNEL_RPY,
        speed=15,
        accel=40
    )

    print("\n=== AT REACTOR ===")
    
def pour_into_reactor():

    print("\n=== POURING INTO REACTOR ===")

    intermediate_1 = [108.5, 75.0, 163.5]
    intermediate_2 = [110.5, 55.0, 165.5]

    move_cartesian(
        REACTOR_FUNNEL_POS,
        rpy=intermediate_1,
        speed=8,
        accel=15
    )

    move_cartesian(
        REACTOR_FUNNEL_POS,
        rpy=intermediate_2,
        speed=8,
        accel=15
    )

    move_cartesian(
        REACTOR_FUNNEL_POS,
        rpy=POUR_RPY,
        speed=8,
        accel=15
    )

    time.sleep(3)

    move_cartesian(
        REACTOR_FUNNEL_POS,
        rpy=REACTOR_FUNNEL_RPY,
        speed=8,
        accel=15
    )

    print("\n=== POUR COMPLETE ===")

# =========================================================
# ADD POWDER LOOP
# =========================================================
def add_powder_until_target():

    empty_weight = get_scale_weight()

    print(
        f"\nEmpty weight: "
        f"{empty_weight:.2f} g"
    )

    desired_powder = float(
        input(
            "\nDesired powder amount (g): "
        )
    )

    target_weight = (
        empty_weight
        + desired_powder
    )

    print(
        f"\nTarget total weight:"
        f" {target_weight:.2f} g"
    )

    while True:

        current_weight = (
            get_scale_weight()
        )

        print(
            f"\nCurrent:"
            f" {current_weight:.2f} g"
        )

        if current_weight >= target_weight:

            print(
                "\nTarget reached."
            )

            break

        print(
            "\nScooping powder..."
        )

        pickup_scoop()

        scoop_powder()

        release_powder()


        time.sleep(3)
        
    return_scoop
    
def add_powder():

    while True:

        user_input = input(
            "\nEnter 'add' or 'no add': "
        ).strip().lower()

        # -------------------------------------------------
        # ADD MORE POWDER
        # -------------------------------------------------

        if user_input == "add":

            print("\n=== ADDING MORE POWDER ===")

            pickup_scoop()

            scoop_powder()

            release_powder()

            return_scoop()

            print("\n=== WAITING FOR SCALE READING ===")

            time.sleep(5)


        # -------------------------------------------------
        # MOVE TO REACTOR
        # -------------------------------------------------

        elif user_input == "no add":

            print("\n=== MOVING TO REACTOR ===")
            regrasp_weighboat_from_scale()
            move_to_reactor()
            pour_into_reactor()

            break

        else:

            print("\nInvalid input.")
            print("Please enter:")
            print("'add'")
            print("or")
            print("'no add'")

# =========================================================
# MAIN
# =========================================================

def main():

    try:

        print("\n=== STARTING PROGRAM ===")

        time.sleep(2)
        # home
        move_cartesian(INITIAL_POS)

        # place weigh boat on scale
        pickup_weighboat()

        place_weighboat_on_scale()

        # scoop workflow

        print("\n=== WAITING FOR SCALE READING ===")

        time.sleep(1)

        # ask user if more powder should be added
        add_powder_until_target()

        # return home
        print("\n=== RETURNING HOME ===")

        move_cartesian(INITIAL_POS)

        print("\n=== PROCESS COMPLETE ===")

    except Exception as e:

        print(f"\nERROR: {e}")

        arm.clean_error()

    finally:

        arm.disconnect()

        print("\nRobot disconnected")

if __name__ == "__main__":
    main()
