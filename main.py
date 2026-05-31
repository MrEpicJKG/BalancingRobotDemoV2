#!/usr/bin/env pybricks-micropython
from ucollections import namedtuple
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, InfraredSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

###############################################################

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
# calibrationTimer = StopWatch()
# balanceTimer = StopWatch()

# Robot Action Def for when in Beacon mode
Action = namedtuple('BeaconAction', ['driveSpeed', 'steering'])

#Robot Action Def for when in Remote Control Mode (May not impelement. Just a placeholder for now)
#Action = namedtuple('RCAction', ['leftMotorSpeed', 'rightMotorSpeed'])

# Init Constants                    #Format: <Data Type>, <Desc>
GYRO_CALIBRATION_LOOP_COUNT = 200   # int, Num of Iterations for Gyro calibration
GYRO_OFFSET_FACTOR = 0.0005         # decimal, Gyro Offset Factor from GyroBoy Project
TARGET_LOOP_PERIOD = 20             # int, in MS

# Init other, non-constant variables
sitUpSpeed = -40                     # int, rear motor rotation speed in Deg / Sec
hasSatUp = False                    # bool, is the robot already upright
prevError = 0                       # decimal, Var for Prev. Error for Beacon Derivative Controller
lightFlashOnTime = 10               # int, Num millisecs for the bricks light to stay on while flashing
lightFlashOffTime = 90              # int, Num millisecs for the bricks light to stay off while flashing  
rearMotorLoweredRotVal = -90        # int, the rotational value for when the robot is fully sitting upright
rearMotorLiftedRotVal = 120         # int, the rotational value for the rear motor to be at when the rear wheel is fully lifted
rearMtrStartBalanceRetractSpd = 200  # int, the rotation in deg/sec for the rear motor to spin when retracting the rear leg as fast as possible
shouldRestart = False               # bool, should we restart after falling over


def CheckBatteryCharged():
        # Calculate current battery voltage
        battery_voltage = (ev3.battery.voltage())/1000

        # Battery warning for voltage less than 7.5V and exits program
        if battery_voltage < 6.5:
            ev3.light.on(Color.ORANGE)
            return False
        else:
            return True


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

def UpdateAction():
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
    relativeDistance = irSensor.beacon(1)[0]
    angle = irSensor.beacon(1)[1]
    global prevError
        
    # If the beacon is on and within range, the segway rotates until the angle between the infrared sensor and beacon is less 
    # than 10 degrees
    if relativeDistance is not None:
    #{
        angleError = 0 - angle
        K_angle = 4 # controller gain
        steering = K_angle * angleError
        action = BeaconAction(driveSpeed=0, steering=steering)
        yield action
    #}
        # if the angle between the infrared sensor and beacon is less than 10 degrees, the segway translates towards the beacon
        if abs(angleError) < 10:
        #{
            error = 100 - relativeDistance
            dError = (error - prevError)/actionTimer.time()
            K_p = 6
            K_d = 2.5 # controller gains

            driveSpeed = K_p * error + K_d * dError
            action = BeaconAction(driveSpeed=driveSpeed, steering=0)
            prevError = error
                
            if relativeDistance > 10:
                yield action
            else:
                yield STOP
        #}
    else:
        yield
#}

# Stops both motors
def StopMotors():
#{
    leftMotor.stop()
    rightMotor.stop()
#}

def CalibrateGyro():
#{
    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)
    rearMotor.reset_angle(0)
    fallTimer.reset()

    ev3.speaker.set_volume(100)
    ev3.speaker.say("Prepping for calibration. Place in stand and press button.") # TEMP -- Will be replaced with the commented line below. it is here to shorten troubleshooting time.
    # ev3.speaker.say("Welcome.  Preparing for gyro calibration. Please place me in my stand and press the center button when ready.")
    FlashTheLightUntilButtonPress(Color.YELLOW, Button.CENTER)
    ev3.speaker.say("Calibrating gyro") # TEMP -- Will be replaced with the commented line below. it is here to shorten troubleshooting time.
    # ev3.speaker.say("Do Not Touch Me Until I say I Am Done! Calibrating Gyro in 3...")
    # wait(50)
    # ev3.speaker.say("2...")
    # wait(50)
    # ev3.speaker.say("1...")

    ######  ACTUAL GYRO CALIBRATION ######
    # calibrationTimer.reset()
    # calibrationTimer.resume()
    while True:
    #{
        # Track the change in the Gyro value over time to get an offset value that can
        # be used to account for gyro drift later
        gyroMinRate = 440   # default to the max value, so we can go down from there
        gyroMaxRate = -440   # default to the min value, so we can go up from there
        gyroSum = 0
        for _ in range(GYRO_CALIBRATION_LOOP_COUNT): 
            gyroSensorValue = gyroSensor.speed()
            gyroSum += gyroSensorValue
            if gyroSensorValue > gyroMaxRate:
                gyroMaxRate = gyroSensorValue
            if gyroSensorValue < gyroMinRate:
                gyroMinRate = gyroSensorValue
            wait(5)
        if gyroMaxRate - gyroMinRate < 2: 
            break
    gyroOffset = gyroSum / GYRO_CALIBRATION_LOOP_COUNT
        # lastGyroAngle = 100000  # used to calculate gyro speed via difference ovewr time, since we can only use speed or angle. default to impossible value
        # lastTime = calibrationTimer.time()
        # for _ in range(GYRO_CALIBRATION_LOOP_COUNT):    # loop through the calibration to provide a period of time for the value to accumulate
        #{
            # if lastGyroAngle == 100000:
            #     lastGyroAngle = gyroSensor.angle()
            #     lastTime = calibrationTimer.time()
            # else:
            #{  
        #         dist = gyroSensor.angle() - lastGyroAngle   # Calc difference in angles over the last iteration
        #         timeDiff = calibrationTimer.time() - lastTime   # calc the difference in time over the last iteration
        #         #prevent Divide By Zero Errors
        #         if dist == 0:
        #             dist = 0.0000001
        #         if timeDiff == 0:
        #             timeDiff = 0.0000001

        #         gyroSensorValue = dist / timeDiff           # get the rotational speed of the gyro (ideally 0, but looking for gyro drift)\
        #         lastGyroAngle = gyroSensor.angle()          # store the current angle so we can do it again on the next iteration
        #         lastTime = calibrationTimer.time()          # store the current time so we can do it again on the next iteration
        #         gyroSum += gyroSensorValue                  # add any drift we find above to a accumulator value
        #         if gyroSensorValue > gyroMaxRate:           # if the sensor value is greater than the highest previously recorded value...
        #         #{
        #            gyroMaxRate = gyroSensorValue            # ...update the previously recorded value to the current value
        #         #}
        #         if gyroSensorValue < gyroMinRate:           # if the sensor alue is less than the lowest previously recorded value...
        #         #{
        #             gyroMinRate = gyroSensorValue           # ...update the previously recorded value to the current value
        #         #}
        #         wait(5)                                     # wait 5 ms
                
        #     #}
        # #}
        # if gyroMaxRate - gyroMinRate < 2:               # if the accumulated gyro drift is less than 2 deg / sec
        # #{
        #     break                                       # break out of the while loop
        # #}
    #}
    ###### END OF ACTUAL GYRO CALIBRATION ######

    wait(20)
    ev3.speaker.say("Calibration Done")  # TEMP -- Will be replaced with the commented line below. it is here to shorten troubleshooting time.
    # ev3.speaker.say("Calibration Complete. Thank you for waiting. Please remove me from my stand now and set me on the ground.")
    return gyroOffset
#}

def SitUp():
#{   
    rearMotor.reset_angle(0)
    wait(500)
    ev3.speaker.say("Remove Safety Bar") # TEMP -- Will be replaced with the commented line below. it is here to shorten troubleshooting time.
    # ev3.speaker.say("Once I am on the ground, please remove the safety bar from my rear lifting wheel, then press the center button.")
    FlashTheLightUntilButtonPress(Color.YELLOW, Button.CENTER)
    # ev3.speaker.say("Press Button")  # TEMP -- Will be replaced with the commented line below. it is here to shorten troubleshooting time.
    # ev3.speaker.say("Press the center button again to confirm you removed the safety bar. You will damage the motor otherwise.") 
    # FlashTheLightUntilButtonPress(Color.YELLOW, Button.CENTER)
    wait(200)
    rearMotor.run_target(sitUpSpeed, rearMotorLoweredRotVal, Stop.HOLD, True)
#}

def StartBalance():
#{
    rearMotor.run_target(rearMtrStartBalanceRetractSpd, rearMotorLiftedRotVal, Stop.HOLD, False)
    leftMotor.run_time(-500, 500, Stop.COAST, False)
    rightMotor.run_time(-500, 500, Stop.COAST, True)
#}

def MainBalanceLoop(gyroOffset):
#{
    motorPositionSum = 0
    wheelAngle = 0
    motorPositionChange = [0, 0, 0, 0]
    driveSpeed = 0
    steering = 0
    controlLoopCounter = 0
    robotBodyAngle = -0.2

    #prep the generator
    actionTask = UpdateAction()

    while True:
        # This timer measures how long a single loop takes. This will be used to help keep the loop time 
        # consistent, even when different actions are happening.
        singleLoopTimer.reset()

        # This calculates the average control loop period. This is used in the control feedback 
        # calculation instead of the single loop time to filter out random fluctuations.
        if controlLoopCounter == 0:
            # The first time through the loop, we need to assign a value to
            # avoid dividing by zero later.

            # Dividing by 1000 because default time is in milliseconds
            averageControlLoopPeriod = TARGET_LOOP_PERIOD / 1000
            controlLoopTimer.reset()
        else:
            averageControlLoopPeriod = (controlLoopTimer.time() / 1000 / controlLoopCounter)
        controlLoopCounter += 1

        # Calculate robot body angle and rate (or speed)
        gyroSensorValue = gyroSensor.speed()
        gyroOffset *= (1 - GYRO_OFFSET_FACTOR)
        gyroOffset += GYRO_OFFSET_FACTOR * gyroSensorValue
        robotBodyRate = gyroSensorValue - gyroOffset
        robotBodyAngle += robotBodyRate * averageControlLoopPeriod

        # Motor angle values
        leftMotorAngle = leftMotor.angle()
        rightMotorAngle = rightMotor.angle()

        # Calculate wheel angle and rate, the wheel rate is calculated using a moving average on 4 item motorPositionChange list
        previousMotorSum = motorPositionSum
        motorPositionSum = leftMotorAngle + rightMotorAngle
        change = motorPositionSum - previousMotorSum
        motorPositionChange.insert(0, change)
        del motorPositionChange[-1]
        wheelAngle += change - driveSpeed * averageControlLoopPeriod
        wheelRate = sum(motorPositionChange) / 4 / averageControlLoopPeriod

        # This is the main control feedback calculation
        outputPower = (-0.01 * driveSpeed) + (1.2 * robotBodyRate +
                                                     28 * robotBodyAngle +
                                                     0.075 * wheelRate +
                                                     0.12 * wheelAngle)

        # Motor limits
        if outputPower > 100:
            outputPower = 100
        if outputPower < -100:
            outputPower = -100

        # Drive motors
        leftMotor.dc(outputPower - 0.1 * steering)
        rightMotor.dc(outputPower + 0.1 * steering)

        # Check if robot fell down. If the output speed is +/-100% for more than one second, 
        # we know that we are no longer balancing properly.
        if abs(outputPower) < 100:
            fallTimer.reset()
        elif fallTimer.time() > 1000:
            break

        # This runs update_action() until the next "yield" statement
        action = next(actionTask)
        if action is not None:
            driveSpeed, steering = action

        # Make sure loop time is at least TARGET_LOOP_PERIOD. The output power calculation 
        # above depends on having a certain amount of time in each loop.
        wait(TARGET_LOOP_PERIOD - singleLoopTimer.time())

    # Handle falling over. If we get to this point in this program, it means
    # that the robot fell over.

    # Stop all motors
    StopMotors()
    ev3.speaker.say("I think I have fallen over. Please pick me up and place me back on my stand c" +
    "Then press the center button to try again.")
    FlashTheLightUntilButtonPress(Color.YELLOW, Button.CENTER)
    shouldRestart = True
#}

def MainSequence():
    ################### MAIN CODE SEQUENCE ######################
    if CheckBatteryCharged() == False:
       ev3.speaker.say("Low Battery Warning. Exiting Program")
       sys.exit()

    offsetVal = CalibrateGyro()
    SitUp()
    StartBalance()
    MainBalanceLoop(offsetVal)
    if shouldRestart == True:
        MainLoop()  # Recursion FTW!!!

#################################################################
MainSequence()
