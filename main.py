#!/usr/bin/env pybricks-micropython
from ucollections import namedtuple
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, InfraredSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
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
singleLoopTimer = StopWatch()
controlLoopTimer = StopWatch()
fallTimer = StopWatch()
actionTimer = StopWatch()
lightFlashTimer = StopWatch()

# Robot Action Def for when in Beacon mode
BeaconAction = namedtuple('Action', ['driveSpeed', 'steering'])

#Robot Action Def for when in Remote Control Mode (May not impelement. Just a placeholder for now)
#RCAction = namedtuple('Action', ['leftMotorSpeed', 'rightMotorSpeed'])

# Init Constants                    #Format: <Data Type>, <Desc>
GYRO_CALIBRATION_LOOP_COUNT = 200   # int, Num of Iterations for Gyro calibration
GYRO_OFFSET_FACTOR = 0.0005         # decimal, Gyro Offset Factor from GyroBoy Project
TARGET_LOOP_PERIOD = 20             # int, in MS
REAR_MOTOR_SIT_UP_ROTATIONS = 0.5   # decimal, num of rotations med motor makes to lift the bot until it is straight up

# Init other, non-constant variables
sitUpSpeed = 60                     # int, rear motor rotation speed in Deg / Sec
hasSatUp = False                    # bool, is the robot already upright
prevError = 0                       # decimal, Var for Prev. Error for Beacon Derivative Controller
lightFlashOnTime = 10               # int, Num millisecs for the bricks light to stay on while flashing
lightFlashOffTime = 90              # int, Num millisecs for the bricks light to stay off while flashing  

# Write your program here.
def CalibrateGyro():
#{
    ev3.speaker.set_volume(100)
    lightIsOn = False
    ev3.speaker.say("Preparing for gyro calibration. Please place me in my stand and press the center button when ready.")
    #Flash the light yellow every 0.5 seconds until the center button is pressed.
    lightFlashTimer.reset()
    ev3.light.off()
    while Button.CENTER not in ev3.buttons.pressed():
    #{
        timeVal = lightFlashTimer.time()   #get the current timer val
        #if the light is OFF and has been so for the set amount of time
        if lightIsOn == False and timeVal >= lightFlashOffTime:
        #{
            # reset the timer, turn the light ON, and toggle the variable
            lightFlashTimer.reset()
            ev3.light.on(Color.YELLOW)
            lightIsOn = True
        #}

        #if the light is ON and it has been so for the set amount of time
        elif lightIsOn == True and timeVal >= lightFlashOnTime:
        #{
            # reset the timer, turn the light OFF, and toggle the variable
            lightFlashTimer.reset()
            ev3.light.off()
            lightIsOn = False
        #}
    #}
    ev3.speaker.say("Do Not Touch Me Unitl I say I Am Done! Calibrating Gyro in 3...")
    wait(100)
    ev3.speaker.say("2...")
    wait(100)
    ev3.speaker.say("1...")

    ######  ACTUAL GYRO CALIBRATION ######
    while True:
    #{
        # Track the change in the Gyro value over time to get an offset value that can
        # be used to account for gyro drift later
        gyroMinRate = 440   # default to the max value, so we can go down from there
        gyroMaxRate = -440   # default to the min value, so we can go up from there
        gyroSum = 0
        for _ in range(GYRO_CALIBRATION_LOOP_COUNT):    # loop through the calibration to provide a period of time for the value to accumulate
        #{
            gyroSensorValue = gyroSensor.speed()        # get the rotational speed of the gyro (ideally 0, but looking for gyro drift)
            gyroSum += gyroSensorValue                  # add any drift we find above to a accumulator value
            if gyroSensorValue > gyroMaxRate:           # if the sensor value is greater than the highest previously recorded value...
            #{
                gyroMaxRate = gyroSensorValue           # ...update the previously recorded value to the current value
            #}
            if gyroSensorValue < gyroMinRate:           # if the sensor alue is less than the lowest previously recorded value...
            #{
                gyroMinRate = gyroSensorValue           # ...update the previously recorded value to the current value
            #}
            wait(5)                                     # wait 5 ms
        #}
        if gyroMaxRate - gyroMinRate < 2:               # if the accumulated gyro drift is less than 2 deg / sec
        #{
            break                                       # break out of the while loop
        #}
    #}
    ###### END OF ACTUAL GYRO CALIBRATION ######

    wait(20)
    ev3.speaker.say("Calibration Complete. Thank you for waiting. Please remove me from my stand now and set me on the ground.")
#}

CalibrateGyro()
