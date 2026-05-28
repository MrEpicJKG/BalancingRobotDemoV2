#!/usr/bin/env pybricks-micropython
from ucollections import namedtuple
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, InfraredSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Init brick, motors, and sensors
ev3 = EV3Brick()
rightMotor = Motor(Port.A)
leftMotor = Motor(Port.D)
rearMotor = Motor(Port.B)
gyroSensor = GyroSensor(Port.S1)
irSensor = InfraredSensor(Port.S4)

# Init timers
# singleLoopTimer = StopWatch()
# controlLoopTimer = StopWatch()
# fallTimer = StopWatch()
# actionTimer = StopWatch()

# Init Constants                    #Format: <Data Type>, <Desc>
GYRO_CALIBRATION_LOOP_COUNT = 200   # int, Num of Iterations for Gyro calibration
GYRO_OFFSET_FACTOR = 0.0005         # float, Gyro Offset Factor from GyroBoy Project
TARGET_LOOP_PERIOD = 20             # int, in MS
REAR_MOTOR_SIT_UP_ROTATIONS = 1.0# float, num of rotations med motor makes to lift the bot until it is straight up

# Init other, non-constant variables
# ----- Vars relating to Sit-Up          
# sitUpTime = 800                     # int, duration in MS
sitUpSpeed = 1                      # int, rear motor rotation speed in Deg / Sec
hasSatUp = False                    # bool, is the robot already upright
# ------

prevError = 0                       # Var for Prev. Error for Beacon Derivative Controller
# Robot Action Def for when in Beacon mode
BeaconAction = namedtuple('Action', ['driveSpeed', 'steering'])

#Robot Action Def for when in Remote Control Mode (May not impelement. Just a placeholder for now)
#RCAction = namedtuple('Action', ['leftMotorSpeed', 'rightMotorSpeed'])

# def InitVars():
#     # Init other, non-constant variables
#     # ----- Vars relating to Sit-Up          
#     #global sitUpTime = 800                     # int, duration in MS
#     global sitUpSpeed                     # int, rear motor rotation speed in Deg / Sec
#     global hasSatUp                   # bool, is the robot already upright
#     sitUpSpeed = 1
#     hasSatUp = False
#     # ------

#     global prevError
#     prevError = 0                       # Var for Prev. Error for Beacon Derivative Controller
#     # Robot Action Def for when in Beacon mode
#     global BeaconAction
#     BeaconAction = namedtuple('Action', ['driveSpeed', 'steering'])

#     #Robot Action Def for when in Remote Control Mode (May not impelement. Just a placeholder for now)
#     #global RCAction
#     # RCAction = namedtuple('Action', ['leftMotorSpeed', 'rightMotorSpeed'])

#=====================================================================================
# Write your program here.
# def CalibrateGyro():



# def SitUp():
    # if hasSatUp == False:
    #     hasSatUp = True
    #     i = 0
    #     ev3.speaker.say("Press the center button to confirm the safety beam has been removed.")
    #     #cycle every 10 iterations turn the light on red if its off or off if it is on, until the center button is pressed.
    #     while ev3.Buttons.pressed() != [CENTER]:
    #     #{
    #         if i < 20 and i != 0 and i != 10 and i != 20:
    #             i += 1
    #         elif i == 0:
    #             ev3.light.off()
    #             i += 1
    #         elif i == 10:
    #             ev3.light.on(RED)
    #             i += 1
    #         elif i == 20:
    #             i = 0
    #     #}

    # rearMotor.run_target(sitUpSpeed, 0, Stop.HOLD, True) 
    # rearMotor.run_target(sitUpSpeed, 360 * REAR_MOTOR_SIT_UP_ROTATIONS, Stop.HOLD, True)

                
#InitVars()
# SitUp()
rearMotor.run_target(sitUpSpeed, 360 * REAR_MOTOR_SIT_UP_ROTATIONS, Stop.HOLD, True)