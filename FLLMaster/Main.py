#!/usr/bin/env python3
from DriveLibraries import *
from time import sleep

R1 = Robot('robot.cfg')
R1.gs.calibrate
R1.zeroGyro()
#R1.spkr.speak("Starting")
R1.spkr.beep()
R1.DriveAtHeading(0, 50, 30, False)
R1.ArcTurn(270, 10, 30)
R1.DriveAtHeading(270, 50, 30, False)
R1.ArcTurn(270, 10, 30)
R1.DriveAtHeading(270 * 2, 50, 30, False)
R1.ArcTurn(270, 10, 30)
R1.DriveAtHeading(270 * 3, 50, 30, False)
R1.ArcTurn(270, 10, 30)
R1.GyroTurn(0)