#!/usr/bin/env pybricks-micropython
from ucollections import namedtuple
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, InfraredSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

###############################################################
# print("Reached ===== INIT - BEGIN =====") # <<<<<<<<<<<<<<<<<<<<<<<
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
BeaconAction = namedtuple('BeaconAction', ['driveSpeed', 'steering'])

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
# print("Reached ===== INIT - END =====") # <<<<<<<<<<<<<<<<<<<<<<<

def CheckBatteryCharged():
#{
    # print("Reached ===== BATTERY - BEGIN =====") # <<<<<<<<<<<<<<<<<<<<<<<
    # Calculate current battery voltage
    battery_voltage = (ev3.battery.voltage())/1000
    # print("Reached === BATTERY - MID 1 ===") # <<<<<<<<<<<<<<<<<<<<<<<

    # Battery warning for voltage less than 7.5V and exits program
    if battery_voltage < 6.5:
        ev3.light.on(Color.ORANGE)
        # print("Reached ===== BATTERY - END 1 =====") # <<<<<<<<<<<<<<<<<<<<<<<
        return False
    else:
        # print("Reached ===== BATTERY - END  2 =====") # <<<<<<<<<<<<<<<<<<<<<<<
        return True
#}


def FlashTheLightUntilButtonPress(lightColor, button):
#{
    # print("Reached ===== FLASH - BEGIN =====") # <<<<<<<<<<<<<<<<<<<<<<<
    lightIsOn = False
    #Flash the light in <lightColor> until <button> is pressed.
    lightFlashTimer.reset()
    lightFlashTimer.resume()
    ev3.light.off()
    # print("Reached === FLASH - MID 1 ===") # <<<<<<<<<<<<<<<<<<<<<<<
    while button not in ev3.buttons.pressed():
    #{
        timeVal = lightFlashTimer.time()   #get the current timer val
        #if the light is OFF and has been so for the set amount of time
        if lightIsOn == False and timeVal >= lightFlashOffTime:
        #{
            # print("Reached === FLASH - MID  2 ===") # <<<<<<<<<<<<<<<<<<<<<<<
            # reset the timer, turn the light ON, and toggle the variable
            lightFlashTimer.reset()
            ev3.light.on(lightColor)
            lightIsOn = True
        #}

        #if the light is ON and it has been so for the set amount of time
        elif lightIsOn == True and timeVal >= lightFlashOnTime:
        #{
            # print("Reached === FLASH - MID   3 ===") # <<<<<<<<<<<<<<<<<<<<<<<
            # reset the timer, turn the light OFF, and toggle the variable
            lightFlashTimer.reset()
            ev3.light.off()
            lightIsOn = False
        #}
    #}
    # print("Reached ===== FLASH - END =====") # <<<<<<<<<<<<<<<<<<<<<<<
#}

def UpdateAction():
#{
    # print("Reached ===== ACTION - BEGIN =====") # <<<<<<<<<<<<<<<<<<<<<<<
    # this while loop is necessary to make this a generator function that can yield multiple times. 
    # It will only exit when the program exits or if there is a KeyboardInterrupt
    while True: 
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
        # print("Reached === ACTION - MID 1 ===") # <<<<<<<<<<<<<<<<<<<<<<<
        # If the beacon is on and within range, the segway rotates until the angle between the infrared sensor and beacon is less 
        # than 10 degrees
        if relativeDistance is not None:
        #{
            # print("Reached === ACTION - MID  2 ===") # <<<<<<<<<<<<<<<<<<<<<<<
            angleError = 0 - angle
            K_angle = 4 # controller gain
            steering = K_angle * angleError
            action = BeaconAction(driveSpeed=0, steering=steering)
            # print("Reached ===== ACTION - END 1 =====") # <<<<<<<<<<<<<<<<<<<<<<<
            yield action

            # if the angle between the infrared sensor and beacon is less than 10 degrees, the segway translates towards the beacon
            if abs(angleError) < 10:
            #{
                # print("Reached === ACTION - MID   3 ===") # <<<<<<<<<<<<<<<<<<<<<<<
                error = 100 - relativeDistance
                dt = actionTimer.time()
                if dt == 0:
                    dt = 1
                dError = (error - prevError)/dt
                K_p = 6
                K_d = 2.5 # controller gains

                driveSpeed = K_p * error + K_d * dError
                action = BeaconAction(driveSpeed=driveSpeed, steering=0)
                prevError = error
                
                if relativeDistance > 10:
                    # print("Reached ===== ACTION - END  2 =====") # <<<<<<<<<<<<<<<<<<<<<<<
                    yield action
                else:
                    # print("Reached ===== ACTION - END   3 =====") # <<<<<<<<<<<<<<<<<<<<<<<
                    wait(5)
                    yield STOP
        #}
        else:
            # print("Reached ===== ACTION - END    4 =====") # <<<<<<<<<<<<<<<<<<<<<<<
            yield
    #}
#}

# Stops both motors
def StopMotors():
#{
    # print("Reached ===== STOP - BEGIN =====") # <<<<<<<<<<<<<<<<<<<<<<<
    leftMotor.stop()
    rightMotor.stop()
    # print("Reached ===== STOP - END =====") # <<<<<<<<<<<<<<<<<<<<<<<
#}

def CalibrateGyro():
#{
    # print("Reached ===== CALIBRATE - BEGIN =====") # <<<<<<<<<<<<<<<<<<<<<<<
    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)
    rearMotor.reset_angle(0)
    fallTimer.reset()

    # print("Reached === CALIBRATE - MID 1 ===") # <<<<<<<<<<<<<<<<<<<<<<<
    ev3.speaker.set_volume(100)
    print("###### SPEAK: Prepping for calibration")
    ev3.speaker.say("Prepping for calibration. Place in stand and press button.") # TEMP -- Will be replaced with the commented line below. it is here to shorten troubleshooting time.
    # ev3.speaker.say("Welcome.  Preparing for gyro calibration. Please place me in my stand and press the center button when ready.")
    FlashTheLightUntilButtonPress(Color.YELLOW, Button.CENTER)
    print("###### SPEAK: Calibrating")
    ev3.speaker.say("Calibrating gyro") # TEMP -- Will be replaced with the commented line below. it is here to shorten troubleshooting time.
    # ev3.speaker.say("Do Not Touch Me Until I say I Am Done! Calibrating Gyro in 3...")
    # wait(50)
    # ev3.speaker.say("2...")
    # wait(50)
    # ev3.speaker.say("1...")

    ######  ACTUAL GYRO CALIBRATION ######
    # calibrationTimer.reset()
    # calibrationTimer.resume()
    iters = 0
    while True:
    #{
        iters += 1
        # print("Reached === CALIBRATE - MID  2 ===") # <<<<<<<<<<<<<<<<<<<<<<<
        # Track the change in the Gyro value over time to get an offset value that can
        # be used to account for gyro drift later
        gyroMinRate = 440   # default to the max value, so we can go down from there
        gyroMaxRate = -440   # default to the min value, so we can go up from there
        gyroSum = 0
        for _ in range(GYRO_CALIBRATION_LOOP_COUNT):
            # print("Reached === CALIBRATE - MID   3 ===") # <<<<<<<<<<<<<<<<<<<<<<< 
            gyroSensorValue = gyroSensor.speed()
            gyroSum += gyroSensorValue
            if gyroSensorValue > gyroMaxRate:
                # print("Reached === CALIBRATE - MID    4 ===") # <<<<<<<<<<<<<<<<<<<<<<<
                gyroMaxRate = gyroSensorValue
            if gyroSensorValue < gyroMinRate:
                # print("Reached === CALIBRATE - MID     5 ===") # <<<<<<<<<<<<<<<<<<<<<<<
                gyroMinRate = gyroSensorValue
            wait(5)
        if gyroMaxRate - gyroMinRate < 2: 
            # print("Reached === CALIBRATE - MID      6 ===") # <<<<<<<<<<<<<<<<<<<<<<<
            break
        if iters >= 5:
            ev3.speaker.say("Unable to calibrate gyro. Area is to unstable. Please move to a more stable area and try again. Exiting Program.")
            sys.exit()
    gyroOffset = gyroSum / GYRO_CALIBRATION_LOOP_COUNT
    ###### END OF ACTUAL GYRO CALIBRATION ######

    wait(20)
    print("###### SPEAK: Calibration Done")
    ev3.speaker.say("Calibration Done")  # TEMP -- Will be replaced with the commented line below. it is here to shorten troubleshooting time.
    # ev3.speaker.say("Calibration Complete. Thank you for waiting. Please remove me from my stand now and set me on the ground.")
    print(">>>>>>>>>> GyroOffset: " + str(gyroOffset))
    # print("Reached ===== CALIBRATE - END =====") # <<<<<<<<<<<<<<<<<<<<<<<
    return gyroOffset
#}

def SitUp():
#{  
    # print("Reached ===== SITUP - BEGIN =====") # <<<<<<<<<<<<<<<<<<<<<<<
    rearMotor.reset_angle(0)
    wait(500)
    print("###### SPEAK: Remove Safety Bar")
    ev3.speaker.say("Remove Safety Bar") # TEMP -- Will be replaced with the commented line below. it is here to shorten troubleshooting time.
    # ev3.speaker.say("Once I am on the ground, please remove the safety bar from my rear lifting wheel, then press the center button.")
    FlashTheLightUntilButtonPress(Color.YELLOW, Button.CENTER)
    # ev3.speaker.say("Press Button")  # TEMP -- Will be replaced with the commented line below. it is here to shorten troubleshooting time.
    # ev3.speaker.say("Press the center button again to confirm you removed the safety bar. You will damage the motor otherwise.") 
    # FlashTheLightUntilButtonPress(Color.YELLOW, Button.CENTER)
    wait(200)
    rearMotor.run_target(sitUpSpeed, rearMotorLoweredRotVal, Stop.HOLD, True)
    # print("Reached ===== SITUP - END =====") # <<<<<<<<<<<<<<<<<<<<<<<
#}

def StartBalance():
#{
    # print("Reached ===== STARTBAL - BEGIN =====") # <<<<<<<<<<<<<<<<<<<<<<<
    rearMotor.run_target(rearMtrStartBalanceRetractSpd, rearMotorLiftedRotVal, Stop.HOLD, False)
    leftMotor.run_time(-500, 380, Stop.COAST, False)
    rightMotor.run_time(-500, 380, Stop.COAST, True)
    # print("Reached ===== STARTBAL - END =====") # <<<<<<<<<<<<<<<<<<<<<<<
#}

def MainBalanceLoop(gyroOffset):
#{
    # print("Reached ===== BALANCE - BEGIN =====") # <<<<<<<<<<<<<<<<<<<<<<<
    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)
    fallTimer.reset()
    controlLoopTimer.reset()
    controlLoopTimer.resume()
    actionTimer.reset()
    actionTimer.resume()
    motorPositionSum = 0
    wheelAngle = 0
    motorPositionChange = [0, 0, 0, 0]
    driveSpeed = 0
    steering = 0
    controlLoopCounter = 0
    robotBodyAngle = 0

    #prep the generator
    actionTask = UpdateAction()

    
    while True:
        # print("Reached === BALANCE - MID 1 ===") # <<<<<<<<<<<<<<<<<<<<<<<
        try:
            # This timer measures how long a single loop takes. This will be used to help keep the loop time 
            # consistent, even when different actions are happening.
            singleLoopTimer.reset()

            # This calculates the average control loop period. This is used in the control feedback 
            # calculation instead of the single loop time to filter out random fluctuations.
            if controlLoopCounter == 0:
                # print("Reached === BALANCE - MID  2 ===") # <<<<<<<<<<<<<<<<<<<<<<<
                # The first time through the loop, we need to assign a value to
                # avoid dividing by zero later.

                # Dividing by 1000 because default time is in milliseconds
                averageControlLoopPeriod = TARGET_LOOP_PERIOD / 1000
                controlLoopTimer.reset()
            else:
                # print("Reached === BALANCE - MID   3 ===") # <<<<<<<<<<<<<<<<<<<<<<<
                averageControlLoopPeriod = (controlLoopTimer.time() / 1000 / controlLoopCounter)
            controlLoopCounter += 1

            # print("Reached === BALANCE - MID    4 ===") # <<<<<<<<<<<<<<<<<<<<<<<
            # Calculate robot body angle and rate (or speed)
            gyroSensorValue = gyroSensor.speed()
            gyroOffset *= (1 - GYRO_OFFSET_FACTOR)
            gyroOffset += GYRO_OFFSET_FACTOR * gyroSensorValue
            robotBodyRate = gyroSensorValue - gyroOffset
            robotBodyAngle += robotBodyRate * averageControlLoopPeriod

            # Motor angle values
            leftMotorAngle = leftMotor.angle()
            rightMotorAngle = rightMotor.angle()
            # print("Reached === BALANCE - MID     5 ===") # <<<<<<<<<<<<<<<<<<<<<<<

            # Calculate wheel angle and rate, the wheel rate is calculated using a moving average on 4 item motorPositionChange list
            previousMotorSum = motorPositionSum
            motorPositionSum = leftMotorAngle + rightMotorAngle
            change = motorPositionSum - previousMotorSum
            motorPositionChange.insert(0, change)
            del motorPositionChange[-1]
            wheelAngle += change - driveSpeed * averageControlLoopPeriod
            wheelRate = sum(motorPositionChange) / 4 / averageControlLoopPeriod
            # print("Reached === BALANCE - MID      6 ===") # <<<<<<<<<<<<<<<<<<<<<<<

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
            # print("Reached === BALANCE - MID       7 ===") # <<<<<<<<<<<<<<<<<<<<<<<

            # Drive motors
            leftMotor.dc(outputPower - 0.1 * steering)
            rightMotor.dc(outputPower + 0.1 * steering)
            print("GyroVal: " + str(gyroSensorValue) + " | GyroOffset: " + str(gyroOffset) + " | BodyAngle: " + str(robotBodyAngle) + " | BodyRate: " + str(robotBodyRate) + " | WheelRate: " + str(wheelRate))
            print("WheelAngle: " + str(wheelAngle) + " | LeftAngle: " + str(leftMotorAngle) + " | RightAngle: " + str(rightMotorAngle))
            print("OutputPower: " + str(outputPower) + " | DriveSpeed: " + str(driveSpeed) + " | Steer: " + str(steering) + " | Lspd: " + str(leftMotor.speed()) + " | Rspd: " + str(rightMotor.speed()))
            print("----------------------------------------------")

            # Check if robot fell down. If the output speed is +/-100% for more than one second, 
            # we know that we are no longer balancing properly.
            if abs(outputPower) < 100:
                fallTimer.reset()
                # print("Reached === BALANCE - MID        8 ===") # <<<<<<<<<<<<<<<<<<<<<<<
            elif fallTimer.time() > 1000:
                # print("Reached === BALANCE - MID         9 ===") # <<<<<<<<<<<<<<<<<<<<<<<
                break

            # This runs update_action() until the next "yield" statement
            # print("Reached === BALANCE - MID          10 ===") # <<<<<<<<<<<<<<<<<<<<<<<
            try:
                action = next(actionTask)
            except StopIteration:
                actionTask = UpdateAction()
                action = next(actionTask)
            
            # Reset action timer for next beacon controller cycle
            actionTimer.reset()
                
            if action is not None:
                # print("Reached === BALANCE - MID           11 ===") # <<<<<<<<<<<<<<<<<<<<<<<
                driveSpeed, steering = action

            # Make sure loop time is at least TARGET_LOOP_PERIOD. The output power calculation 
            # above depends on having a certain amount of time in each loop.
            # print("Reached === BALANCE - MID            12 ===") # <<<<<<<<<<<<<<<<<<<<<<<
            wait(TARGET_LOOP_PERIOD - singleLoopTimer.time())
        except KeyboardInterrupt:
            StopMotors()
            # print("Reached ===== BALANCE - MID             13 - END OF WHILE - KeyboardInterrupt =====")
            break

    # print("Reached === BALANCE - MID              14 - NORMAL END OF WHILE ===") # <<<<<<<<<<<<<<<<<<<<<<<
    # Handle falling over. If we get to this point in this program, it means
    # that the robot fell over.

    # Stop all motors
    StopMotors()
    print("###### SPEAK: fallen over")
    ev3.speaker.say("I think I have fallen over. Please pick me up and place me back on my stand" +
    "Then press the center button to try again.")
    FlashTheLightUntilButtonPress(Color.YELLOW, Button.CENTER)
    shouldRestart = True
    # print("Reached ===== BALANCE - END =====") # <<<<<<<<<<<<<<<<<<<<<<<
#}

def MainSequence():
    ################### MAIN CODE SEQUENCE ######################
    # print("Reached === MAIN - BEGIN ===") # <<<<<<<<<<<<<<<<<<<<<<<
    if CheckBatteryCharged() == False:
        print("###### SPEAK: low battery")
        ev3.speaker.say("Low Battery Warning. Exiting Program")
        sys.exit()
    # print("Reached === MAIN - MID 1 ===") # <<<<<<<<<<<<<<<<<<<<<<<
    offsetVal = CalibrateGyro()
    # print("Reached === MAIN - MID  2 ===") # <<<<<<<<<<<<<<<<<<<<<<<
    SitUp()
    # print("Reached === MAIN - MID   3 ===") # <<<<<<<<<<<<<<<<<<<<<<<
    StartBalance()
    # print("Reached === MAIN - MID    4 ===") # <<<<<<<<<<<<<<<<<<<<<<<
    MainBalanceLoop(offsetVal)
    # print("Reached === MAIN - MID     5 ===") # <<<<<<<<<<<<<<<<<<<<<<<
    if shouldRestart == True:
        # print("Reached === MAIN - END ===") # <<<<<<<<<<<<<<<<<<<<<<<
        # The Recursion below is taking the place of the while loop that is 
        # in the original segway code near line 127 right after the stop_motors() function
        MainLoop()  # Recursion FTW!!!

#################################################################
MainSequence()