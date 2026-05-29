#!/usr/bin/env pybricks-micropython
from ucollections import namedtuple
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, InfraredSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

###############################################################
'''
NOTE: In my version of the code, I am using the angle() function of the gyro sensor,
instead of the speed() function. I am calculating the speed by using the distance / time.
(I suspect that is how it does it behind the scenes anyway. Gyros detect angles.
Thats what they do.)
The reason I am calculating the speed from the angle instead of the other way around
is because I am afraid that trying to calculate the angle from the speed will accumulate
math errors and become offset and unstable very quickly. Since I am making something 
that relys on precise calculation based on the angle, I figured it would be more important
for the angle to be accurate than the speed.
'''

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
calibrationTimer = StopWatch()

# Robot Action Def for when in Beacon mode
Action = namedtuple('BeaconAction', ['driveSpeed', 'steering'])

#Robot Action Def for when in Remote Control Mode (May not impelement. Just a placeholder for now)
#Action = namedtuple('RCAction', ['leftMotorSpeed', 'rightMotorSpeed'])

# Init Constants                    #Format: <Data Type>, <Desc>
GYRO_CALIBRATION_LOOP_COUNT = 200   # int, Num of Iterations for Gyro calibration
GYRO_OFFSET_FACTOR = 0.0005         # decimal, Gyro Offset Factor from GyroBoy Project
TARGET_LOOP_PERIOD = 20             # int, in MS
REAR_MOTOR_SIT_UP_ROTATIONS = 0.5   # decimal, num of rotations med motor makes to lift the bot until it is straight up

# Init other, non-constant variables
sitUpSpeed = -20                     # int, rear motor rotation speed in Deg / Sec
hasSatUp = False                    # bool, is the robot already upright
prevError = 0                       # decimal, Var for Prev. Error for Beacon Derivative Controller
lightFlashOnTime = 10               # int, Num millisecs for the bricks light to stay on while flashing
lightFlashOffTime = 90              # int, Num millisecs for the bricks light to stay off while flashing  
rearMotorLoweredRotVal = -55        # int, the rotational value for when the robot is fully sitting upright
rearMotorLiftedRotVal = 190         # int, the rotational value for the rear motor to be at when the rear wheel is fully lifted


def FlashTheLightUntilButtonPress(lightColor, button):
    lightIsOn = False
    #Flash the light in <lightColor> until <button> is pressed.
    lightFlashTimer.reset()
    lightFlashTimer.resume()
    ev3.light.off()
    while button not in ev3.buttons.pressed():
    #{
        timeVal = lightFlashTimer.time()   #get the current timer val
        #if the light is OFF and has been so for the set amount of time
        if lightIsOn == False and timeVal >= lightFlashOffTime:
        #{
            # reset the timer, turn the light ON, and toggle the variable
            lightFlashTimer.reset()
            ev3.light.on(lightColor)
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


def update_action():
#{
    # Beacon mode (on channel 1)
    '''
    Turn on beacon and set the channel to 1.

    The beacon method returns the relative distance (a value from 0 to 100 with 0 being very close and 100 far away) and 
    the approximate angle (-75 to 75 degrees) between the infrared sensor and beacon. 

    The Proportional controller outputs steering values to rotate the segway until the angle between the infrared 
    sensor and beacon is less than 10 degrees. While the PD controller outputs drive speed values to translate the
    segway towards the beacon and yields to a stop action when the relative distance is less than 10.
    '''
    relative_distance = irSensor.beacon(1)
    angle = irSensor.beacon(1)
    global prev_error
        
    # If the beacon is on and within range, the segway rotates until the angle between the infrared sensor and beacon is less 
    # than 10 degrees
    if relative_distance is not None:
    #{
        angle_error = 0 - angle
        K_angle = 4 # controller gain
        steering = K_angle * angle_error
        action = BeaconAction(drive_speed=0, steering=steering)
        yield action
    #}
        # if the angle between the infrared sensor and beacon is less than 10 degrees, the segway translates towards the beacon
        if abs(angle_error) < 10:
        #{
            error = 100 - relative_distance
            d_error = (error - prev_error)/action_timer.time()
            K_p = 6
            K_d = 2.5 # controller gains

            drive_speed = K_p * error + K_d * d_error
            action = BeaconAction(drive_speed=drive_speed, steering=0)
            prev_error = error
                
            if relative_distance > 10:
                yield action
            else:
                yield STOP
        #}
    else:
        yield
#}


# Stops both motors
def stop_motors():
#{
    left_motor.stop()
    right_motor.stop()
#}


def CalibrateGyro():
#{
    ev3.speaker.set_volume(100)
    ev3.speaker.say("Preparing for gyro calibration. Please place me in my stand and press the center button when ready.")
    FlashTheLightUntilButtonPress(Color.YELLOW, Button.CENTER)
    ev3.speaker.say("Do Not Touch Me Until I say I Am Done! Calibrating Gyro in 3...")
    wait(50)
    ev3.speaker.say("2...")
    wait(50)
    ev3.speaker.say("1...")

    ######  ACTUAL GYRO CALIBRATION ######
    calibrationTimer.reset()
    calibrationTimer.resume()
    while True:
    #{
        # Track the change in the Gyro value over time to get an offset value that can
        # be used to account for gyro drift later
        gyroMinRate = 440   # default to the max value, so we can go down from there
        gyroMaxRate = -440   # default to the min value, so we can go up from there
        gyroSum = 0
        lastGyroAngle = 100000  # used to calculate gyro speed via difference ovewr time, since we can only use speed or angle. default to impossible value
        lastTime = calibrationTimer.time()
        for _ in range(GYRO_CALIBRATION_LOOP_COUNT):    # loop through the calibration to provide a period of time for the value to accumulate
        #{
            if lastGyroAngle == 100000:
                lastGyroAngle = gyroSensor.angle()
                lastTime = calibrationTimer.time()
            else:
            #{  
                dist = gyroSensor.angle() - lastGyroAngle   # Calc difference in angles over the last iteration
                timeDiff = calibrationTimer.time() - lastTime   # calc the difference in time over the last iteration
                #prevent Divide By Zero Errors
                if dist == 0:
                    dist = 0.0000001
                if timeDiff == 0:
                    timeDiff = 0.0000001

                gyroSensorValue = dist / timeDiff           # get the rotational speed of the gyro (ideally 0, but looking for gyro drift)\
                lastGyroAngle = gyroSensor.angle()          # store the current angle so we can do it again on the next iteration
                lastTime = calibrationTimer.time()          # store the current time so we can do it again on the next iteration
                gyroSum += gyroSensorValue                  # add any drift we find above to a accumulator value
                if gyroSensorValue > gyroMaxRate:           # if the sensor value is greater than the highest previously recorded value...
                #{
                   gyroMaxRate = gyroSensorValue            # ...update the previously recorded value to the current value
                #}
                if gyroSensorValue < gyroMinRate:           # if the sensor alue is less than the lowest previously recorded value...
                #{
                    gyroMinRate = gyroSensorValue           # ...update the previously recorded value to the current value
                #}
                wait(5)                                     # wait 5 ms
                
            #}
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


def SitUp():
#{   
    rearMotor.reset_angle(0)
    wait(500)
    ev3.speaker.say("Once I am on the ground, please remove the safety bar from my rear lifting wheel, then press the center button.")
    FlashTheLightUntilButtonPress(Color.YELLOW, Button.CENTER)
    ev3.speaker.say("Press the center button again to confirm you removed the safety bar. You will damage the motor otherwise.") 
    FlashTheLightUntilButtonPress(Color.YELLOW, Button.CENTER)
    wait(200)
    rearMotor.run_target(sitUpSpeed, -90, Stop.HOLD, True)
    # rearMotor.run_until_stalled(sitUpSpeed, Stop.COAST)
    # while rearMotor.angle() > -60 and rearMotor.angle < 200 and gyroSensor.angle() < -10:
    #     {}
    # # Do nothing except wait for the motor to spin until the while conditions are met...
    # rearMotor.stop()  # ... then stop the motor

#}



################### MAIN LOOP ######################
# CalibrateGyro()
SitUp()
