#!/usr/bin/env python3
from DriveLibraries import *
from time import sleep

R1 = Robot('robot.cfg')
R1.gs.calibrate
R1.zeroGyro()
#R1.spkr.speak("Starting")
R1.spkr.beep()
R1.DriveBump(0, -30)