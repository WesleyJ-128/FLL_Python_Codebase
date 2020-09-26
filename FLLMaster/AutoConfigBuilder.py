#!/usr/bin/env python3

from configparser import ConfigParser
from ev3dev2.display import *
from ev3dev2.button import *
from ev3dev2.sensor import lego
from ev3dev2.motor import *
import re
from time import sleep


def configBuilder(configFile='robot.cfg'):
    """
    Creates a robot config file based on user input and sensor/motor values
    """
    global lego
    btn = Button()
    disp = Display()
    config = ConfigParser()
    config['Drivebase'] = {}
    config['Sensors'] = {}
    config['AuxMotors'] = {}
    config['Other'] = {}
    outPorts = ['OUTPUT_A', 'OUTPUT_B', 'OUTPUT_C', 'OUTPUT_D']
    senTypes = ['Color', 'Gyro', 'Touch', 'Ultrasonic', 'Infrared']
    
    for i in senTypes:
        try:
            sensorClass = getattr(lego, i + 'Sensor')
            mySensor = sensorClass()
            port = str(mySensor.address)
            port = re.sub('.*in([0-9]).*', 'INPUT_\g<1>', port)
            globals()[i] = port
            config['Sensors'][i + 'Port'] = port
        except:
            config['Sensors'][i + 'Port'] = 'None'
        finally:
            mySensor = None
            sensorClass = None
    
    for i in outPorts:
        try:
            motor = LargeMotor(eval(i))
            disp.text_pixels("Found a Large Motor at " + "port " + i[-1])
            disp.text_pixels("What kind of motor is this?", False, 0, 10)
            disp.text_pixels("Press left for left motor,", False, 0, 20)
            disp.text_pixels("right for right motor,", False, 0, 30)
            disp.text_pixels("up for Aux1, or down for Aux2", False, 0, 40)
            disp.update()
            while not btn.any():
                pass
            selection = btn.buttons_pressed
            if selection[0] == 'left':
                btn.wait_for_released('left')
                lm = motor
                config['Drivebase']['LeftMotorPort'] = i
            elif selection[0] == 'right':
                btn.wait_for_released('right')
                rm = motor
                config['Drivebase']['RightMotorPort'] = i

        except:
            pass

    with open(configFile, 'w') as configfile:
        config.write(configfile)    


if __name__ == "__main__":
    configBuilder('robot2.cfg')

