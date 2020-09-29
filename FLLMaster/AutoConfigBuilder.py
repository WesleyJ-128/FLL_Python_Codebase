#!/usr/bin/env python3

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
    global lego
    btn = Button()
    disp = Display()
    spkr = Sound()
    config = ConfigParser()
    config['Drivebase'] = {}
    config['Sensors'] = {}
    config['AuxMotors'] = {}
    config['Other'] = {}
    outPorts = ['OUTPUT_A', 'OUTPUT_B', 'OUTPUT_C', 'OUTPUT_D']
    senTypes = ['Color', 'Gyro', 'Touch', 'Ultrasonic', 'Infrared']

    disp.text_pixels("Is this robot for FLL?")
    disp.text_pixels("Y", False, 10, 110)
    disp.text_pixels("N", False, 150, 110)
    disp.update()
    spkr.beep()
    while True:
        while not btn.any():
            pass
        selection = btn.buttons_pressed
        if selection[0] == 'left':
            btn.wait_for_released('left')
            config['Other']['ForFLL'] = 'TRUE'
            break
        elif selection[0] == 'right':
            btn.wait_for_released('right')
            config['Other']['ForFLL'] = 'FALSE'
            break
        else:
            spkr.beep()
            spkr.beep('-f 380')
    disp.clear()

    
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
    
    for i in outPorts:
        try:
            motor = LargeMotor(eval(i))
            disp.text_pixels("Found a Large Motor at " + "port " + i[-1])
            disp.text_pixels("What kind of motor is this?", False, 0, 10)
            disp.text_pixels("Press left for left motor,", False, 0, 20)
            disp.text_pixels("right for right motor,", False, 0, 30)
            disp.text_pixels("up for Aux1, or down for Aux2", False, 0, 40)
            disp.update()
            spkr.beep()
            while True:
                while not btn.any():
                    pass
                selection = btn.buttons_pressed
                if selection[0] == 'left':
                    btn.wait_for_released('left')
                    config['Drivebase']['LeftMotorPort'] = i
                    break
                elif selection[0] == 'right':
                    btn.wait_for_released('right')
                    config['Drivebase']['RightMotorPort'] = i
                    break
                elif selection[0] == 'up':
                    btn.wait_for_released('up')
                    config['AuxMotors']['AuxMotor1'] = i
                    break
                elif selection[0] == 'down':
                    btn.wait_for_released('down')
                    config['AuxMotors']['AuxMotor2'] = i
                    break
                else:
                    spkr.beep()
                    spkr.beep('-f 380')
        except:
            pass
    
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
                selection = btn.buttons_pressed
                if selection[0] == 'left':
                    btn.wait_for_released('left')
                    config['AuxMotors']['AuxMotor1'] = i
                    break
                elif selection[0] == 'right':
                    btn.wait_for_released('right')
                    config['AuxMotors']['AuxMotor2'] = i
                    break
                else:
                    spkr.beep()
                    spkr.beep('-f 380')
        except:
            pass

    mtrs = MoveTank(eval(config['Drivebase']['LeftMotorPort']), eval(config['Drivebase']['RightMotorPort']))

    if config['Sensors']['UltrasonicPort'] is not 'None':
        us = UltrasonicSensor(eval(config['Sensors']['UltrasonicPort']))
        disp.text_pixels("Does the Ultrasonic sensor")
        disp.text_pixels("face foward?", False, 0, 10)
        disp.text_pixels("Y", False, 10, 110)
        disp.text_pixels("N", False, 150, 110)
        disp.update()
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
    
    if usNav:
        disp.text_pixels("Place the robot facing a wall, ")
        disp.text_pixels("and press any button.", False, 0, 10)
        disp.update()
        while not btn.any():
            pass
        circsum = 0
        sign = 1
        tests = 3
        for j in range(tests):
            if j / 2 == round(j / 2):
                parity = 1
            else:
                parity = -1
            usSum = 0
            readings = 5
            for i in range(readings):
                usSum += us.distance_centimeters
                sleep(0.1)
            stavg = round(usSum / readings, 2)
            usSum = 0
            print(stavg, file=stderr)
            mtrs.on_for_rotations(parity * 25, parity * 25, 1)
            for i in range(readings):
                usSum += us.distance_centimeters
                sleep(0.1)
            enavg = round(usSum / readings, 2)
            print(enavg, file=stderr)
            circsum += abs(enavg - stavg)
            sign = (enavg - stavg) / abs(enavg - stavg)
        avg = round(circsum / tests, 2)
        config['Drivebase']['WheelCircumfrence'] = str(avg)

        polarity = 'normal'
        if sign < 0:
            config['Drivebase']['MotorsInverted'] = 'FALSE'
            polarity = 'normal'
        elif sign > 0:
            config['Drivebase']['MotorsInverted'] = 'TRUE'
            polarity = 'inversed'
        
        mtrs.set_polarity(polarity)
        disp.text_pixels("Place robot in clear area,")
        disp.text_pixels("and press any button.", False, 0, 10)
        disp.update()
        while not btn.any():
            pass
        gs = GyroSensor(eval(config['Sensors']['GyroPort']))
        widthbetweenwheelssum = 0
        ang = 0
        for i in range(tests):
            gs._ensure_mode(gs.MODE_GYRO_RATE)
            gs._ensure_mode(gs.MODE_GYRO_ANG)
            mtrs.on_for_degrees(25, -25, 360)
            sleep(0.5)
            ang = gs.angle
            wbw = abs((360 * float(config['Drivebase']['WheelCircumfrence'])) / (ang * math.pi))
            widthbetweenwheelssum += wbw
        config['Drivebase']['WidthBetweenWheels'] = str(round(widthbetweenwheelssum / tests, 2))

        if ang > 0:
            config['Drivebase']['GyroInverted'] = 'FALSE'
        elif ang < 0:
            config['Drivebase']['GyroInverted'] = 'TRUE'
    
    else:
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
        
    config['Drivebase']['GearRatio'] = '1'
    config['Drivebase']['kp'] = '1'
    config['Drivebase']['ki'] = '0'
    config['Drivebase']['kd'] = '0.5'
    config['Sensors']['kpLine'] = '0.5'
    config['Sensors']['kiLine'] = '0'
    config['Sensors']['kdLine'] = '2'

    spkr.beep()
    spkr.beep()
    disp.text_pixels("The PID gains in the config")
    disp.text_pixels("file are placeholder values", False, 0, 10)
    disp.text_pixels("and should be manually tuned.", False, 0, 20)
    disp.text_pixels("Press any button to end.", False, 0, 40)
    disp.update()
    while not btn.any():
        pass

    with open(configFile, 'w') as configfile:
        config.write(configfile)    


if __name__ == "__main__":
    configBuilder('robot2.cfg')