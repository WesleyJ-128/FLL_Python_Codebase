#!/usr/bin/env python3

# Import stuff
from configparser import ConfigParser
from time import sleep
from ev3dev2.display import *
from ev3dev2.button import *
from ev3dev2.sensor import lego, INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.motor import *
import re
from ev3dev2.sensor.lego import GyroSensor, UltrasonicSensor
from ev3dev2.sound import Sound
from sys import stderr


def configBuilder(configFile='robot.cfg'):
    """
    Creates a robot config file based on user input and sensor/motor values
    """
    # Make sure this is the global lego module, as it is used dynamicly later.  Unsure if needed.
    global lego
    # Instantiate brick interface objects
    btn = Button()
    disp = Display()
    spkr = Sound()
    # Instantiate a ConfigParser object to writ and read the INI format config file
    config = ConfigParser()

    # Create the config file sections
    config['Drivebase'] = {}
    config['Sensors'] = {}
    config['AuxMotors'] = {}
    config['Other'] = {}

    # Used when detecting motors and sensors, as iterables
    outPorts = ['OUTPUT_A', 'OUTPUT_B', 'OUTPUT_C', 'OUTPUT_D']
    senTypes = ['Color', 'Gyro', 'Touch', 'Ultrasonic', 'Infrared']

    # Ask the user if the robot is to be used for FLL
    disp.text_pixels("Is this robot for FLL?")
    disp.text_pixels("Y", False, 10, 110)
    disp.text_pixels("N", False, 150, 110)
    disp.update()
    spkr.beep()
    # Wait for a button press (the outer loop is to allow for selection retry if 
    # an invalid button is pressed)
    while True:
        while not btn.any():
            pass
        # Get the list of currently pressed buttons
        selection = btn.buttons_pressed
        # Check if the first buton in the list is the left button
        if selection[0] == 'left':
            # If it is, set the ForFLL value to TRUE and exit the outer loop
            btn.wait_for_released('left')
            config['Other']['ForFLL'] = 'TRUE'
            break
        elif selection[0] == 'right':
            # Otherwise, check if the selection is right. If it is, set ForFLL to FLASE 
            # and exit the outer loop
            btn.wait_for_released('right')
            config['Other']['ForFLL'] = 'FALSE'
            break
        else:
            # If both checks fail, play an error tone and go back to waiting for a button
            spkr.beep()
            spkr.beep('-f 380')
    # Clear the display for the next prompt
    disp.clear()

   # Auto-detect sensors 
    for i in senTypes:
        try:
            # When a sensor object is instantiated without a port, it will find the first port with 
            # that type of sensor connected.  If that sensor is not connected, it will throw an error.
            # The getattr function is used to get the correct sensor type dynamically
            sensorClass = getattr(lego, i + 'Sensor')
            # Instantiate the current sensor type
            mySensor = sensorClass()
            # Get the port that the sensor is connected to
            port = str(mySensor.address)
            # sensor.adress will return sometheing like ev3:in1, so this replaces anything containing in# with INPUT_#
            port = re.sub('.*in([0-9]).*', 'INPUT_\g<1>', port)
            # Add port value to the config file
            config['Sensors'][i + 'Port'] = port
        except:
            # If the sensor instantiation failed, none of that kind of sensor are connected, so write the port value as None
            config['Sensors'][i + 'Port'] = 'None'
        finally:
            # Clear the object and class variables as they are reused every loop cycle
            mySensor = None
            sensorClass = None
    
    # Detect motors
    # Repeat test for each port
    for i in outPorts:
        try:
            # Instanitiate a Large Motor object at the current port.  If there is a Lrge Motor at said port, this will suceed.  
            # Otherwise, it will throw an error and exit the try block.
            motor = LargeMotor(eval(i))
            # Print where the motor was found (port A, B, C, or D), and ask the user what the motor is being used for.
            disp.text_pixels("Found a Large Motor at " + "port " + i[-1])
            disp.text_pixels("What kind of motor is this?", False, 0, 10)
            disp.text_pixels("Press left for left motor,", False, 0, 20)
            disp.text_pixels("right for right motor,", False, 0, 30)
            disp.text_pixels("up for Aux1, or down for Aux2", False, 0, 40)
            disp.update()
            # Beep to signal that user input is required
            spkr.beep()

            # Loop is used to allow for invalid button presses to repeat selection
            while True:
                # Wait for any button to be pressed
                while not btn.any():
                    pass
                # Store what it is (first button pressed, if multiple)
                selection = btn.buttons_pressed[0]
                if selection == 'left':
                    # Wait for the button to be released, so the operation only occurs once
                    btn.wait_for_released('left')
                    # If left, store the current port as the left motor port
                    config['Drivebase']['LeftMotorPort'] = i
                    # Exit the loop, as this is a valid selection
                    break
                elif selection == 'right':
                    # Wait for the button to be released, so the operation only occurs once
                    btn.wait_for_released('right')
                    # If right, store the current port as the right motor port
                    config['Drivebase']['RightMotorPort'] = i
                    # Exit the loop, as this is a valid selection
                    break
                elif selection == 'up':
                    # Wait for the button to be released, so the operation only occurs once
                    btn.wait_for_released('up')
                    # If up, store the current port as the first auxillary motor port
                    config['AuxMotors']['AuxMotor1'] = i
                    # Exit the loop, as this is a valid selection
                    break
                elif selection == 'down':
                    # Wait for the button to be released, so the operation only occurs once
                    btn.wait_for_released('down')
                    # If down, store the current port as the second auxillary motor port
                    config['AuxMotors']['AuxMotor2'] = i
                    # Exit the loop, as this is a valid selection
                    break
                else:
                    # If the pressed button is not valid, play an error tone and repeat the selection menu
                    spkr.beep()
                    spkr.beep('-f 380')
        
        # If a motor is not found, no action is required.  Apparently 'try' doesn't work without an 'except'
        except:
            pass
    
    # This works exactly the same as the Large Motor detection, except that it is detecting Medium Motors instead, 
    # and there are no drivemotor selection options.
    for i in outPorts:
        try:
            motor = MediumMotor(eval(i))
            disp.text_pixels("Found a Med Motor at port " + i[-1])
            disp.text_pixels("Which AuxMotor is this?", False, 0, 10)
            disp.text_pixels("1", False, 10, 110)
            disp.text_pixels("2", False, 150, 110)
            disp.update()
            spkr.beep()
            while True:
                while not btn.any():
                    pass
                selection = btn.buttons_pressed[0]
                if selection == 'left':
                    btn.wait_for_released('left')
                    config['AuxMotors']['AuxMotor1'] = i
                    break
                elif selection == 'right':
                    btn.wait_for_released('right')
                    config['AuxMotors']['AuxMotor2'] = i
                    break
                else:
                    spkr.beep()
                    spkr.beep('-f 380')
        except:
            pass

    # Create a MoveTank object with the detected drive motors, as it is needed for the upcoming tests.
    mtrs = MoveTank(eval(config['Drivebase']['LeftMotorPort']), eval(config['Drivebase']['RightMotorPort']))

    # A Foward-Facing Ultrasonic sensor is the only way to allow for calculating the wheel circumfrence.
    if config['Sensors']['UltrasonicPort'] is not 'None':
        # Ask the user if the detected Ultrasonic sensor is facing fowards
        us = UltrasonicSensor(eval(config['Sensors']['UltrasonicPort']))
        disp.text_pixels("Does the Ultrasonic sensor")
        disp.text_pixels("face foward?", False, 0, 10)
        disp.text_pixels("Y", False, 10, 110)
        disp.text_pixels("N", False, 150, 110)
        disp.update()
        # Same selection system as before
        while not btn.any():
                pass
        selection = btn.buttons_pressed[0]
        if selection == 'left':
            usNav = True
        elif selection == 'right':
            usNav = False
        else:
            usNav = False
        spkr.beep()
    else:
        usNav = False
        us = None
    
    # Using the variable usNav set previously, check if the Ultrasonic sensor faces fowards.
    if usNav:
        # Ask the user to place the robot facing a wall, then wait for a button press.
        disp.text_pixels("Place the robot facing a wall, ")
        disp.text_pixels("and press any button.", False, 0, 10)
        disp.update()
        while not btn.any():
            pass
        # Set up some variables for averaging
        circsum = 0
        sign = 1
        tests = 3
        # Repeat the wheelcircumfrence test multiple times to get a more accurate average
        for j in range(tests):
            # Determine if this is an even or odd loop cycle.  This is used for calculating MotorsInverted,
            # as the motors are run backwards on odd cycles
            if j / 2 == round(j / 2):
                parity = 1
            else:
                parity = -1
            # More variables for averaging
            usSum = 0
            readings = 5
            # Take multiple sensor readings of how far away from the wall the robot is, and average them
            for i in range(readings):
                usSum += us.distance_centimeters
                sleep(0.1)
            stavg = round(usSum / readings, 2)
            # Reset averaging variable
            usSum = 0
            # Move the robot one wheel rotation, foward on even loop cycles, or backward on odd
            mtrs.on_for_rotations(parity * 25, parity * 25, 1)
            # Take multiple sensor readings of how far away from the wall the robot is, and average them
            for i in range(readings):
                usSum += us.distance_centimeters
                sleep(0.1)
            enavg = round(usSum / readings, 2)
            # The absolute difference between the starting average and the ending average is the wheel circumfrence; 
            # however, this is a cumulative sum of all wheel circumfrence measurements for averaging.
            circsum += abs(enavg - stavg)
            # Isolate the sign (-x or x to -1 or 1) of the wheel circumfrence (Used for MotorsInverted)
            sign = (enavg - stavg) / abs(enavg - stavg)
        # Calculate the average wheel circumfrence
        avg = round(circsum / tests, 2)
        # Write to config file variable
        config['Drivebase']['WheelCircumfrence'] = str(avg)

        # Set MotorsInverted in the config file, and set the 'polarity' variable (for inverting motor commands if necessary)
        polarity = 'normal'
        if sign < 0:
            config['Drivebase']['MotorsInverted'] = 'FALSE'
            polarity = 'normal'
        elif sign > 0:
            config['Drivebase']['MotorsInverted'] = 'TRUE'
            polarity = 'inversed'
        
        # Invert motor commands if necessary
        mtrs.set_polarity(polarity)
        # Ask the user to place the robot in a clear area, and wait for a button press
        disp.text_pixels("Place robot in clear area,")
        disp.text_pixels("and press any button.", False, 0, 10)
        disp.update()
        while not btn.any():
            pass
        # Instantiate a Gyro Sensor object as the degrees turned is necessary for calculating the Width Between Wheels
        gs = GyroSensor(eval(config['Sensors']['GyroPort']))
        # Create variables for averaging
        widthbetweenwheelssum = 0
        ang = 0
        # Repeat the trial multiple times and average the results
        for i in range(tests):
            # Reset the reported gyro angle to zero
            gs._ensure_mode(gs.MODE_GYRO_RATE)
            gs._ensure_mode(gs.MODE_GYRO_ANG)
            # Turn in place for one wheel rotation
            mtrs.on_for_degrees(25, -25, 360)
            # Wait for the robot to settle
            sleep(0.5)
            # Read the current angle of the robot
            ang = gs.angle
            # Calculate the width between the robot's wheels using the formula for arc length (ArcLength = (π * d * Θ) / 360) solved for d,
            # the diameter of the circle, which is the width between wheels.  Absolute value is used so the direction of turn does not matter.
            wbw = abs((360 * float(config['Drivebase']['WheelCircumfrence'])) / (ang * math.pi))
            # Add the calculated value to the running total (used for averaging)
            widthbetweenwheelssum += wbw
        # Calculate the average WBW value, round it to remove floating point errors, and store in the ConfigParser object
        config['Drivebase']['WidthBetweenWheels'] = str(round(widthbetweenwheelssum / tests, 2))

        # The motor move command previously made the robot turn right.  If the gyro reported this as a turn towards positive, it is not inverted.
        # Otherwise, the gyro is inverted.
        if ang > 0:
            config['Drivebase']['GyroInverted'] = 'FALSE'
        elif ang < 0:
            config['Drivebase']['GyroInverted'] = 'TRUE'
    
    else:
        # If the robot does not have an ultrasonic sensor that faces fowards, the robot cannot calculate the wheel circumfrence, which is required
        # to calculate widthbetweenwheels.  The robot also cannot calculate MotorsInverted or GyroInverted.  Therefore, empty valuse are written to
        # those parts of the config file, and the user is notified that they need to manually fill in that information.
        spkr.beep()
        spkr.beep()
        config['Drivebase']['WheelCircumfrence'] = ''
        config['Drivebase']['WidthBetweenWheels'] = ''
        config['Drivebase']['MotorsInverted'] = ''
        config['Drivebase']['GyroInverted'] = ''
        disp.text_pixels("The config file is missing some")
        disp.text_pixels("values that will need to be", False, 0, 10)
        disp.text_pixels("filled in before use.", False, 0, 20)
        disp.text_pixels("Press any button to exit.", False, 0, 40)
        disp.update()
        while not btn.any():
            pass
        
    # Write placeholder values for the PID gains, so those functions may be usable, if poorly.
    # Because the WheelCircumfrence is calculated, the wheelcircumfrence value will include any gearing,
    # so the effective gear ratio is one.
    config['Drivebase']['GearRatio'] = '1'
    config['Drivebase']['kp'] = '1'
    config['Drivebase']['ki'] = '0'
    config['Drivebase']['kd'] = '0.5'
    config['Sensors']['kpLine'] = '0.5'
    config['Sensors']['kiLine'] = '0'
    config['Sensors']['kdLine'] = '2'

    # Tell the user that the PID values may (will) need to be tuned.
    spkr.beep()
    spkr.beep()
    disp.text_pixels("The PID gains in the config")
    disp.text_pixels("file are placeholder values", False, 0, 10)
    disp.text_pixels("and should be manually tuned.", False, 0, 20)
    disp.text_pixels("Press any button to end.", False, 0, 40)
    disp.update()
    while not btn.any():
        pass

    #  Write all the values stored in the ConfigParser object named config to a file in INI format, with the name passed by configFile.
    with open(configFile, 'w') as configfile:
        config.write(configfile)    


if __name__ == "__main__":
    configBuilder('robot2.cfg')