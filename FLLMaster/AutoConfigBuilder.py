#!/usr/bin/env python3

from configparser import ConfigParser
from ev3dev2 import sensor
from ev3dev2.display import *
from ev3dev2.button import *
from ev3dev2.sensor import lego
import ev3dev2.sensor.lego
from ev3dev2.motor import *
import re


def configBuilder(configFile='robot.cfg'):
    """
    Creates a robot config file based on user input and sensor/motor values
    """
    global lego
    config = ConfigParser()
    config['Drivebase'] = {}
    config['Sensors'] = {}
    config['AuxMotors'] = {}
    config['Other'] = {}
    senTypes = ['Color', 'Gyro', 'Touch', 'Ultrasonic', 'Infrared']
    
    for i in senTypes:
        try:
            sensorClass = getattr(lego, i + 'Sensor')
            mySensor = sensorClass()
            port = str(mySensor.address)
            port = re.sub('.*in([0-9]).*', 'INPUT_\g<1>', port)
            config['Sensors'][i + 'Port'] = port
        except:
            config['Sensors'][i + 'Port'] = 'None'
        finally:
            mySensor = None
            sensorClass = None
    
    with open(configFile, 'w') as configfile:
        config.write(configfile)    

if __name__ == "__main__":
    configBuilder('robot2.cfg')

