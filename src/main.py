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

motorL1 = Motor(Ports.PORT14, True)
motorL2 = Motor(Ports.PORT13, True)
motorR1 = Motor(Ports.PORT12, False)
motorR2 = Motor(Ports.PORT11, False)

left_dt = MotorGroup(motorL1, motorL2)
right_dt = MotorGroup(motorR1, motorR2)

vision__RED_BALL = Signature(1, 10333, 15491, 12912,-1401, -947, -1174,3.9, 0)
vision__BLUE_BALL = Signature(2, -4779, -4069, -4424,6677, 7949, 7313,7, 0)
vision = Vision(Ports.PORT20, 50, vision__RED_BALL, vision__BLUE_BALL)

filterMotor = Motor(Ports.PORT3, True)
filterMotor.set_velocity(100, PERCENT)

conveyorMotor1 = Motor(Ports.PORT1, True)
conveyorMotor2 = Motor(Ports.PORT2, True)

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

def dampPercent(percent):
    if -5 <= percent and percent <= 5: return 0
    if percent >= 95: return 100
    if percent <= -95: return -100

    #return (percent * percent / 100) * (percent / abs(percent))
    return percent

def updateDT():
    speed = dampPercent(controller.axis3.position())
    turning = controller.axis2.position()

    left_dt.set_velocity(speed + turning)
    right_dt.set_velocity(speed - turning)

    left_dt.spin(FORWARD)
    right_dt.spin(FORWARD)

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
        conveyorGroup.set_velocity(70, PERCENT)
        conveyorGroup.spin(REVERSE)
        conveyorState = 2
    else:
        conveyorGroup.stop()
        conveyorState = 0

filterState = 0

def filterForward():
    global filterState

    if filterState != 1:
        filterMotor.set_velocity(50, PERCENT)
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

def checkOverheating():
    for motor, name in motorList:
        if motor not in overheatingMotors and motor.temperature() > 55:
            controller.screen.print(name + " is overheating")
            controller.rumble("..")
            overheatingMotors.add(motor)
        elif motor in overheatingMotors and motor.temperature() < 52:
            overheatingMotors.remove(motor)

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    
    drivetrain.drive_for(FORWARD, 12, INCHES)
    drivetrain.turn_for(RIGHT, 90, DEGREES)

    # Take those 3 balls
    conveyorForward()
    drivetrain.drive_for(FORWARD, 36, INCHES)
    conveyorForward()

    drivetrain.turn_for(LEFT, 135, DEGREES)
    drivetrain.drive_for(REVERSE, 22, INCHES)

    # Drop into middle goal
    conveyorForward()
    filterBackward()
    wait(5, SECONDS)
    conveyorForward()
    filterBackward()

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")

    # place driver control in this while loop

    controller.axis3.changed(updateDT)
    controller.axis2.changed(updateDT)
    
    controller.buttonL1.pressed(conveyorForward)
    controller.buttonL2.pressed(conveyorBackward)

    controller.buttonR1.pressed(filterForward)
    controller.buttonR2.pressed(filterBackward)
    
    controller.buttonB.pressed(toggleScraper)
    controller.buttonX.pressed(toggleDoor)
    controller.buttonY.pressed(toggleHook)

    while True:
        checkOverheating()
        wait(50)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()