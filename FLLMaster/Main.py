#!/usr/bin/env python3
from DriveLibraries import *
from time import sleep

R1 = Robot('robot.cfg')
R1.gs.calibrate
R1.zeroGyro()
R1.spkr.speak("Starting")
R1.spkr.beep()
R1.reflectCal()
while True:
    R1.btn.wait_for_bump('enter')
    R1.spkr.beep()
    print(R1.correctedReflectedLightIntensity)

