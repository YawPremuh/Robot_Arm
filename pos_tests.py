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

WEIGH_BOAT_POS = [-63.2, -555.5, 11]

SCALE_POS = [-600.9, -183, 87]

POWDER_POS = [-160.2, -322.4, 130]

SCOOP_POS = [-395.9, -242, 101.3]

POWDER_RELEASE_POS = [-600.9, -270.4, 219.4]

POWDER_POUR_POS = [220.6, -326.6, 287]

REGRASP_POS = [-590.9, -200.8, 85]

REACTOR_FUNNEL_POS = [120, 345.9, 725]

REACTOR_APPROACH = [68.7, 280.7, 684.1]

TECAN_APPROACH_POS = [532.5, -732.6, 271.3]

TEST_TUBE_1 = [587.7, -674.7, 204]

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

SCOOP_RPY = [-179.9, -0.3, 96.8]

RELEASE_RPY = [180.0, 0.0, -87.9]

POWDER_RELEASE_RPY = [178.2, 2.1, 93]

POUR_BACK_RPY = [180.0, -35.0, -87.9]

REGRASP_RPY = [-24.6, 88.1, -114.6]

REACTOR_APPROACH_RPY = [-89.9, 79, -2.2]

REACTOR_FUNNEL_RPY = [89.8, 88.7, 170.5]

POUR_RPY = [89.9, -18.6, 170.6]

TECAN_APPROACH_RPY = [170.7, 75, 115.6]

# =========================================================
# JOINT CONFIGURATIONS
# =========================================================

SCOOP_JOINTS = [
    -148.8,
    6.4,
    -52.7,
    0,
    46.1,
    115
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
   -302.7,
   -71.2,
   -44.6,
   54.6,
   38.5,
   123.3
 ]

REGRASP_JOINTS = [
    -185.9,
    61.2,
    -100.4,
    87.3,
    95,
    39.1
]

# =========================================================
# GRIPPER VALUES
# =========================================================

# weigh boat
GRIPPER_PICK = 170
GRIPPER_RELEASE = 850

# scoop
GRIPPER_PICK_SCOOP = 717
GRIPPER_OPEN_SCOOP = 500
GRIPPER_RELEASE_SCOOP = 850

#test tube
GRIPPER_PICK_TEST_TUBE = 172
GRIPPER_RELEASE_TEST_TUBE = 850

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

    time.sleep(0.8)


def safe_above(pos, rpy=DEFAULT_RPY):

    move_cartesian(
        [pos[0], pos[1], SAFE_Z],
        rpy=rpy
    )


def descend(
    pos,
    rpy=DEFAULT_RPY,
    speed=FAST_SPEED,
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

    scale.reset_input_buffer()

    while True:

        line = (
            scale.readline()
            .decode(errors="ignore")
            .strip()
        )

        match = re.search(
            r'(-?\d+\.\d+)g',
            line
        )

        if match:

            return float(
                match.group(1)
            )
            
def get_stable_weight(samples=5):

    readings = []

    while len(readings) < samples:

        try:

            weight = get_scale_weight()

            readings.append(weight)

        except Exception:

            pass

        time.sleep(0.3)

    return sum(readings) / len(readings)
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
        REGRASP_POS[0],
        REGRASP_POS[1],
        SAFE_Z
    ])

    move_joints(REGRASP_JOINTS)

    time.sleep(1)

    # descend vertically with the exact orientation
    move_cartesian(
        REGRASP_POS,
        rpy=REGRASP_RPY,
        speed=70,
        accel=40
    )

    time.sleep(1)

    gripper(GRIPPER_PICK)

    time.sleep(1)

    # lift straight up
    move_cartesian(
        [
            REGRASP_POS[0],
            REGRASP_POS[1],
            SAFE_Z
        ],
        rpy=REGRASP_RPY
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
        [SCOOP_POS[0], SCOOP_POS[1], SCOOP_APPROACH_Z],
        rpy=SCOOP_RPY
    )

    # rotate safely above scoop
    move_joints(SCOOP_JOINTS)

    time.sleep(3)

    
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
        speed=40,
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
# MOVE WEIGH BOAT TO REACTOR
# =========================================================

def move_to_reactor():

    print("\n=== MOVING TO REACTOR ===")

    # move to known safe reactor configuration
    move_cartesian([-600.7, -200.8, 494.5], rpy=[-25.3, 88.1, -115.2])

    time.sleep(1)

    move_cartesian([-315, 134.2, 653.6], rpy=[-89.6, 79.3, 110.2])

    time.sleep(1)

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
        speed=20,
        accel=25
    )

    print("\n=== AT REACTOR ===")
    
def pour_into_reactor():

    print("\n=== POURING INTO REACTOR ===")

    move_cartesian(
        REACTOR_FUNNEL_POS,
        rpy=POUR_RPY,
        speed=20,
        accel=25
    )

    time.sleep(1)

    move_cartesian(
        REACTOR_FUNNEL_POS,
        rpy=REACTOR_FUNNEL_RPY,
        speed=40,
        accel=30
    )

    time.sleep(3)

    move_cartesian(REACTOR_APPROACH, REACTOR_APPROACH_RPY)

    time.sleep(1)

    move_cartesian([-130.3, 259.6, 683.6], rpy = [-90.4, 79.1, 37.9])

    time.sleep(1)
    
    move_cartesian([SCALE_POS[0],SCALE_POS[1], SAFE_Z], rpy=REGRASP_RPY)
    
    time.sleep(1)

    print("\n=== POUR COMPLETE ===")

# =========================================================
# ADD POWDER LOOP
# =========================================================
def add_powder_until_target():

    print("\nWaiting for scale stabilization...")

    time.sleep(5)

    empty_weight = get_stable_weight()

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
        empty_weight +
        desired_powder
    )

    print(
        f"\nTarget total weight: "
        f"{target_weight:.2f} g"
    )

    # pick scoop only once
    pickup_scoop()

    while True:

        current_weight = get_stable_weight()

        print(
            f"\nCurrent: "
            f"{current_weight:.2f} g"
        )

        if current_weight >= target_weight:

            print("\nTarget reached.")

            break

        print("\nScooping more powder...")

        POWDER_POS[2] -= 2
        
        scoop_powder()

        release_powder()

        time.sleep(2)

    return_scoop()


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
# RETURN WEIGH BOAT HOME
# =========================================================
def return_weighboat_home():

    print("\n=== RETURNING WEIGH BOAT HOME ===")

    descend(
        SCALE_POS, rpy=REGRASP_RPY
    )

    gripper(GRIPPER_RELEASE)

    safe_above(SCALE_POS)

    move_cartesian(SCALE_POS, RELEASE_RPY)

    time.sleep(1)

    gripper(GRIPPER_PICK)

    time.sleep(1)

    safe_above(
        WEIGH_BOAT_POS)

    move_cartesian(
        WEIGH_BOAT_POS,
        rpy=DEFAULT_RPY
    )

    print("\n=== WEIGH BOAT STORED ===")

    # =========================================================
# GRAB TEST TUBES FROM TECAN EVO
# =========================================================


ORIGIN_X, ORIGIN_Y, ORIGIN_Z = TEST_TUBE_1

SPACING_X = 38.1
SPACING_Y = 25.4
NUM_COLUMNS = 8


def get_tube_position(row: int, tube: int):
    column_idx = tube - 1
    row_idx = row - 1

    x = ORIGIN_X + column_idx * SPACING_X
    y = ORIGIN_Y + row_idx * SPACING_Y
    z = ORIGIN_Z

    return (x, y, z)


def grab_test_tubes():

    print("\n=== GRABBING TEST TUBES FROM TECAN EVO ===")

    time.sleep(1)

    row_input = input("What row do you want to grab? (1 or 2): ")
    tube_input = input("What test tube do you want to grab? (1 - 8): ")

    TARGET_XYZ = get_tube_position(int(row_input), int(tube_input))

    move_cartesian(
        TECAN_APPROACH_POS,
        rpy=TECAN_APPROACH_RPY
    )

    move_cartesian(
        TARGET_XYZ,
        rpy=[176.5, 55, 132.1]
    )

    move_cartesian(
        [607.1, -673.8, 321.7],
        rpy=[-177.4, 80.2, 134.3]
    )

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

        time.sleep(2)

        # ask user if more powder should be added
        add_powder_until_target()
        
        regrasp_weighboat_from_scale()
        
        move_to_reactor()
        
        pour_into_reactor()
        
        return_weighboat_home()

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
