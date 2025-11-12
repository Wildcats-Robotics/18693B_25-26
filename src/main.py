# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       charlieiwanski                                               #
# 	Created:      8/17/2025, 8:47:21 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

brain = Brain()
controller = Controller()

motorL1 = Motor(Ports.PORT13, True)
motorL2 = Motor(Ports.PORT11, True)
motorL3 = Motor(Ports.PORT12, True)
motorR1 = Motor(Ports.PORT18, False)
motorR2 = Motor(Ports.PORT19, False)
motorR3 = Motor(Ports.PORT20, False)

left_dt = MotorGroup(motorL1, motorL2, motorL3)
right_dt = MotorGroup(motorR1, motorR2, motorR3)

vision__RED_BALL = Signature(1, 10333, 15491, 12912,-1401, -947, -1174,3.9, 0)
vision__BLUE_BALL = Signature(2, -4779, -4069, -4424,6677, 7949, 7313,7, 0)
vision = Vision(Ports.PORT20, 50, vision__RED_BALL, vision__BLUE_BALL)

filterMotor = Motor(Ports.PORT2, True)
filterMotor.set_velocity(100, PERCENT)

conveyorMotor1 = Motor(Ports.PORT1, True)
conveyorMotor2 = Motor(Ports.PORT3, True)

conveyorGroup = MotorGroup(conveyorMotor1, conveyorMotor2)
conveyorGroup.set_velocity(80, PERCENT)

doorPiston = DigitalOut(brain.three_wire_port.a)
scraperPiston = DigitalOut(brain.three_wire_port.b)
hookPiston = DigitalOut(brain.three_wire_port.c)

inertial = Inertial(Ports.PORT10)
drivetrain = SmartDrive(
    left_dt,
    right_dt,
    inertial,
    220,
    307.975,
    257.175,
    DistanceUnits.MM,
    4/3
)

motorList = [
    (motorL1, "Left DT Motor 1"),
    (motorL2, "Left DT Motor 2"),
    (motorR1, "Right DT Motor 1"),
    (motorR2, "Right DT Motor 2"),
    (filterMotor, "Filter Motor"),
    (conveyorMotor1, "Conveyor Motor 1"),
    (conveyorMotor2, "Conveyor Motor 2")
]

overheatingMotors = set()

MODE_RED = 0
MODE_BLUE = 1

"""
    ----- DRIVER CONTROL -----
"""

def dampPercent(percent):
    if -5 <= percent and percent <= 5: return 0
    if percent >= 95: return 100
    if percent <= -95: return -100

    #return (percent * percent / 100) * (percent / abs(percent))
    return percent

def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))

"""
def leftStickChanged():
    vel = dampPercent(controller.axis3.position())

    left_dt.set_velocity(vel, PERCENT)
    left_dt.spin(FORWARD)

    #print("Left stick percent: " + str(controller.axis3.position()) + "%")
    #print("Left torque: " + str(motorL1.torque()) + " NM")

def rightStickChanged():
    vel = dampPercent(controller.axis2.position())
    
    right_dt.set_velocity(vel, PERCENT)
    right_dt.spin(FORWARD)

    #print("Right stick percent: " + str(controller.axis2.position()) + "%")
    #print("Right torque: " + str(motorR1.torque()) + " NM")
"""

lat_kP = 0.6 # Reduce if oscillating
lat_kI = 0.0 # Make non-zero if there is steady-state error
lat_kD = 0.06 # Increase if overshoots too much

turn_kP = 0.8
turn_kI = 0.0
turn_kD = 0.06

vel_prev_error = 0
vel_error_sum = 0

turn_prev_error = 0
turn_error_sum = 0

# For heading_velocity()
prev_time = None
prev_heading = None
prev_heading_deriv = 0.0
h_alpha = 0.2

def heading_velocity():
    global prev_time, prev_heading, prev_heading_deriv

    now_s = brain.timer.time(SECONDS)

    # read heading in [-180, 180)
    heading = (inertial.heading() + 180) % 360 - 180

    if prev_time is None or prev_heading is None:
        # First call: seed values
        prev_time = now_s
        prev_heading = heading
        prev_heading_deriv = 0.0
        return 0.0

    dt = now_s - prev_time
    if dt <= 0:
        return prev_heading_deriv

    # compute wrapped delta heading in [-180, 180]
    delta = heading - prev_heading
    if delta > 180:
        delta -= 360
    elif delta < -180:
        delta += 360

    deriv = delta / dt

    # low-pass filter on derivative
    prev_heading_deriv = h_alpha * prev_heading_deriv + (1 - h_alpha) * deriv

    # save for next call
    prev_time = now_s
    prev_heading = heading

    return prev_heading_deriv

def updateDT(dt=0.015):
    global vel_prev_error, vel_error_sum
    global turn_prev_error, turn_error_sum

    base = dampPercent(controller.axis3.position())
    turn = dampPercent(controller.axis2.position())

    base_vel = 0
    turn_vel = 0

    if (base == 0):
        vel_error_sum = 0
    else:
        average_vel = (left_dt.velocity(PERCENT) + right_dt.velocity(PERCENT)) / 2

        vel_error = base - average_vel
        vel_error_deriv = (vel_error - vel_prev_error) / dt
        vel_error_sum += vel_error * dt

        base_vel = lat_kP * vel_error + lat_kI * vel_error_sum + lat_kD * vel_error_deriv
        vel_prev_error = vel_error

    if (turn == 0):
        turn_error_sum = 0

    turn_error = turn - heading_velocity()
    turn_error_deriv = (turn_error - turn_prev_error) / dt
    turn_error_sum += turn_error * dt

    turn_vel = turn_kP * turn_error + turn_kI * turn_error_sum + turn_kD * turn_error_deriv
    turn_prev_error = turn_error

    left_vel = clamp(base_vel + turn_vel, -100, 100)
    right_vel = clamp(base_vel - turn_vel, -100, 100)

    print("Base vel: " + str(base_vel))
    print("Turn vel: " + str(turn_vel))

    left_dt.spin(FORWARD, left_vel, PERCENT)
    right_dt.spin(FORWARD, right_vel, PERCENT)

conveyorState = 0

def conveyorForward():
    global conveyorState

    if conveyorState != 1:
        conveyorGroup.set_velocity(100, PERCENT)
        conveyorGroup.spin(FORWARD)
        conveyorState = 1
    else:
        conveyorGroup.stop()
        conveyorState = 0
    
def conveyorBackward():
    global conveyorState

    if conveyorState != 2:
        conveyorGroup.set_velocity(100, PERCENT)
        conveyorGroup.spin(REVERSE)
        conveyorState = 2
    else:
        conveyorGroup.stop()
        conveyorState = 0

filterState = 0

def filterForward():
    global filterState

    if filterState != 1:
        filterMotor.set_velocity(100, PERCENT)
        filterMotor.spin(FORWARD)
        filterState = 1
    else:
        filterMotor.stop()
        filterState = 0

def filterBackward():
    global filterState

    if filterState != 2:
        filterMotor.set_velocity(100, PERCENT)
        filterMotor.spin(REVERSE)
        filterState = 2
    else:
        filterMotor.stop()
        filterState = 0
    
def toggleDoor():
    doorPiston.set(1 - doorPiston.value())

def toggleScraper():
    scraperPiston.set(1 - scraperPiston.value())

def toggleHook():
    hookPiston.set(1 - hookPiston.value())

"""
    ----- PROBLEM DETECTION -----
"""

def checkOverheating():
    for motor, name in motorList:
        if motor not in overheatingMotors and motor.temperature() > 55:
            controller.screen.print(name + " is overheating")
            controller.rumble("..")
            overheatingMotors.add(motor)
        elif motor in overheatingMotors and motor.temperature() < 52:
            overheatingMotors.remove(motor)

"""
    ----- AUTONOMOUS CODE -----
"""

WHEEL_DIAMETER = 8.66142
GEAR_RATIO = 4/3

# Returns the motor rotation, in degrees, required for a motor to travel [distance] inches
def rotation_required(distance):
    return distance / WHEEL_DIAMETER / GEAR_RATIO * 360

driving = False

# distance is in INCHES; negative distance makes the bot go backwards
# turn is in DEGREES; positive makes the bot turn right, negative makes the bot turn left
def pid_drive(distance, turn):
    global driving

    # Race conditions 😍
    # Probably not needed since this is also handled in drive and turn
    while driving: wait(15, MSEC)
    driving = True

    # Critical section (PID)

    LAT_TOLERANCE = 2 # 2 degrees of motor rotation
    ROT_TOLERANCE = 2 # 2 degrees of heading

    dt = 0.015

    inertial.set_heading(0)
    left_dt.set_position(0)
    right_dt.set_position(0)

    target_dist = rotation_required(distance)
    target_heading = turn

    # Proportionality constants
    lat_kP = 0.6
    lat_kI = 0.001
    lat_kD = 0.03

    rot_kP = 1.8
    rot_kI = 0.0
    rot_kD = 0.08

    lat_error = 0
    lat_prev_error = 0
    lat_deriv = 0
    lat_err_sum = 0

    rot_error = 0
    rot_prev_error = 0
    rot_deriv = 0
    rot_err_sum = 0

    average_pos = 0
    heading = 0

    # Loops while it is unstable or not at the target
    while (
        not (target_dist - LAT_TOLERANCE <= average_pos <= target_dist + LAT_TOLERANCE) or
        not (target_heading - ROT_TOLERANCE <= heading <= target_heading + ROT_TOLERANCE) or
        abs(lat_deriv) > 5 or abs(rot_deriv) > 5
    ):
        """
        Lateral movement PID
        """

        left_pos = left_dt.position()
        right_pos = right_dt.position()

        average_pos = (left_pos + right_pos) / 2

        lat_error = target_dist - average_pos
        lat_err_sum += lat_error * dt
        lat_deriv = (lat_error - lat_prev_error) / dt
        
        lat_motor_power = lat_kP * lat_error + lat_kI * lat_err_sum + lat_kD * lat_deriv

        """
        Rotational movement PID
        """

        heading = inertial.heading()

        rot_error = target_heading - heading
        rot_err_sum += rot_error * dt
        rot_deriv = (rot_error - rot_prev_error) / dt

        rot_motor_power = rot_kP * rot_error + rot_kI * rot_err_sum + rot_kD * rot_deriv

        left_dt.spin(FORWARD, lat_motor_power + rot_motor_power, PERCENT)
        right_dt.spin(FORWARD, lat_motor_power - rot_motor_power, PERCENT)

        lat_prev_error = lat_error
        rot_prev_error = rot_error
        wait(dt * 1000, MSEC) # 15 milliseconds

    driving = False

# distance is in INCHES; negative distance makes the bot go backwards
# yield_thread determines whether or not the function should stop the current thread
def drive(distance, yield_thread):
    """
        Makes the bot drive forward
    """
    global driving
    while (driving): wait(15, MSEC)

    if (yield_thread):
        pid_drive(distance, 0)
    else:
        Thread(pid_drive, (distance, 0))

# angle is in DEGREES; positive angle turns right and negative turns left
# yield_thread determines whether or not the function should stop the current thread
def turn(angle, yield_thread):
    """
        Makes the bot turn
    """
    global driving
    while (driving): wait(15, MSEC)

    if (yield_thread):
        pid_drive(0, angle)
    else:
        Thread(pid_drive, (0, angle))

def left_auton():
    brain.screen.print("Left-side autonomous code")
    
    drivetrain.drive_for(FORWARD, 4.5, INCHES)
    wait(0.5, SECONDS)
    drivetrain.turn_for(RIGHT, 90, DEGREES)

    # Take those 3 balls
    conveyorForward()
    drivetrain.drive_for(FORWARD, 20.5, INCHES)
    conveyorForward()

    drivetrain.turn_for(LEFT, 135, DEGREES)
    drivetrain.drive_for(REVERSE, 12, INCHES)

    # Drop into middle goal
    conveyorForward()
    filterBackward()
    wait(5, SECONDS)
    conveyorForward()
    filterBackward()

def right_auton():
    brain.screen.print("Right-side autonomous code")
    
    drivetrain.drive_for(FORWARD, 4.5, INCHES)
    wait(0.5, SECONDS)
    drivetrain.turn_for(LEFT, 90, DEGREES)

    # Take those 3 balls
    conveyorForward()
    drivetrain.drive_for(FORWARD, 20.5, INCHES)
    conveyorForward()

    drivetrain.turn_for(LEFT, 135, DEGREES)
    drivetrain.drive_for(REVERSE, 11.5, INCHES)

    # Drop into bottom goal
    conveyorBackward()
    wait(6, SECONDS)
    conveyorBackward()

autons = [
    left_auton, # AUTON_MODE = 0
    right_auton # AUTON_MODE = 1
]

AUTON_MODE = 0

if AUTON_MODE == 0:
    controller.screen.print("Left auton")
elif AUTON_MODE == 1:
    controller.screen.print("Right auton")

def autonomous():
    brain.screen.clear_screen()
    autons[AUTON_MODE]()

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")

    #controller.axis3.changed(leftStickChanged)
    #controller.axis2.changed(rightStickChanged)
    
    controller.buttonL1.pressed(conveyorForward)
    controller.buttonL2.pressed(conveyorBackward)

    controller.buttonR1.pressed(filterForward)
    controller.buttonR2.pressed(filterBackward)
    
    controller.buttonB.pressed(toggleScraper)
    controller.buttonX.pressed(toggleDoor)
    controller.buttonY.pressed(toggleHook)

    while True:
        checkOverheating()
        updateDT()
        wait(15, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()