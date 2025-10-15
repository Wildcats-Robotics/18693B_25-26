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

vision__RED_BALL = Signature(1, 10333, 15491, 12912,-1401, -947, -1174,3.9, 0)
vision__BLUE_BALL = Signature(2, -4779, -4069, -4424,6677, 7949, 7313,7, 0)
vision = Vision(Ports.PORT20, 50, vision__RED_BALL, vision__BLUE_BALL)

filterMotor = Motor(Ports.PORT3, True)
filterMotor.set_velocity(100, PERCENT)

conveyorGroup = MotorGroup(Motor(Ports.PORT1, True), Motor(Ports.PORT2, True))
conveyorGroup.set_velocity(80, PERCENT)

doorPiston = DigitalOut(brain.three_wire_port.a)
scraperPiston = DigitalOut(brain.three_wire_port.b)

inertial = Inertial(Ports.PORT19)
drivetrain = SmartDrive(
    MotorGroup(motorL1, motorL2),
    MotorGroup(motorR1, motorR2),
    inertial,
    220,
    307.975,
    257.175,
    DistanceUnits.MM,
    4/3
)


MODE_RED = 0
MODE_BLUE = 1

def dampPercent(percent):
    return 10 * math.sqrt(abs(percent)) * percent / abs(percent)

def leftStickChanged():
    vel = dampPercent(controller.axis3.position())
    motorL1.set_velocity(vel, PERCENT)
    motorL2.set_velocity(vel, PERCENT)
    
    motorL1.spin(FORWARD)
    motorL2.spin(FORWARD)

    print("Left stick percent: " + str(controller.axis3.position()) + "%")

    #print("Left torque: " + str(motorL1.torque()) + " NM")

def rightStickChanged():
    vel = dampPercent(controller.axis2.position())
    motorR1.set_velocity(vel, PERCENT)
    motorR2.set_velocity(vel, PERCENT)

    motorR1.spin(FORWARD)
    motorR2.spin(FORWARD)

    print("Right stick percent: " + str(controller.axis2.position()) + "%")

    #print("Right torque: " + str(motorR1.torque()) + " NM")

conveyorState = 0

def conveyorForward():
    global conveyorState

    if conveyorState != 1:
        conveyorGroup.spin(FORWARD)
        conveyorState = 1
    else:
        conveyorGroup.stop()
        conveyorState = 0
    
def conveyorBackward():
    global conveyorState

    if conveyorState != 2:
        conveyorGroup.spin(REVERSE)
        conveyorState = 2
    else:
        conveyorGroup.stop()
        conveyorState = 0

filterState = 0

def filterForward():
    global filterState

    if filterState != 1:
        filterMotor.spin(FORWARD)
        filterState = 1
    else:
        filterMotor.stop()
        filterState = 0

def filterBackward():
    global filterState

    if filterState != 2:
        filterMotor.spin(REVERSE)
        filterState = 2
    else:
        filterMotor.stop()
        filterState = 0
    
def toggleDoor():
    doorPiston.set(1 - doorPiston.value())

def toggleScraper():
    scraperPiston.set(1 - scraperPiston.value())

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place automonous code here

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")

    # place driver control in this while loop

    controller.axis3.changed(leftStickChanged)
    controller.axis2.changed(rightStickChanged)
    
    controller.buttonL1.pressed(conveyorForward)
    controller.buttonL2.pressed(conveyorBackward)

    controller.buttonR1.pressed(filterForward)
    controller.buttonR2.pressed(filterBackward)
    
    controller.buttonB.pressed(toggleScraper)
    controller.buttonX.pressed(toggleDoor)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()