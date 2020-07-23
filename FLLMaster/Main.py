#!/usr/bin/env python3
from DriveLibraries import *
from time import sleep

R1 = Robot('robot.cfg')
R1.gs.calibrate()
sleep(1)
R1.spkr.beep()
R1.reflectCal()
R1.btn.wait_for_bump('enter')
# Write Code Here ----------------------------------------
R1.LineStop(0, 50, True)


