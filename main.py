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
gyroSensor = GyroSensor(Port.S1)
irSensor = InfraredSensor(Port.S4)

# Init timers
singleLoopTimer = StopWatch()
controlLoopTimer = StopWatch()
fallTimer = StopWatch()
actionTimer = StopWatch()

# Init Constants                    #Format: <Data Type>, <Desc>
GYRO_CALIBRATION_LOOP_COUNT = 200   # int, Num of Iterations for Gyro calibration
GYRO_OFFSET_FACTOR = 0.0005         # float, Gyro Offset Factor from GyroBoy Project
TARGET_LOOP_PERIOD = 20             # int, in MS

# Init other, non-constant variables
# ----- Vars relating to Sit-Up          
sitUpTime = 800                     # int, duration in MS
sitUpSpeed = -8000                    # int, wheel rotation speed in Deg / Sec
hasSatUp = False                    # bool, is the robot already upright
# ------

prevError = 0                       # Var for Prev. Error for Beacon Derivative Controller
# Robot Action Def for when in Beacon mode
BeaconAction = namedtuple('Action', ['driveSpeed', 'steering'])

#Robot Action Def for when in Remote Control Mode (May not impelement. Just a placeholder for now)
#RCAction = namedtuple('Action', ['leftMotorSpeed', 'rightMotorSpeed'])

#=====================================================================================
# Write your program here.

def SitUp():
    if(hasSatUp == False):
        leftMotor.run_time(sitUpSpeed, sitUpTime, Stop.COAST, False)
        rightMotor.run_time(sitUpSpeed, sitUpTime, Stop.COAST, True)

SitUp()
