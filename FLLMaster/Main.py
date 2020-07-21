#!/usr/bin/env python3
from DriveLibraries import *
from time import sleep
from ev3dev2.sound import *

R1 = Robot('robot.cfg')
R1.gs.calibrate
R1.zeroGyro()
R1.spkr.speak("Starting")
#R1.DriveAtHeading(0, 110, 75, False)
#R1.DriveAtHeading(180, 130, 75, True)
R1.TurnToHeading(-180)
R1.DriveAtHeading(-180, 70, -75, True)
R1.DriveAtHeading(-180, 70, 75, True)
R1.TurnToHeading(0)