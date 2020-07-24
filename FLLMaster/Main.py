#!/usr/bin/env python3
from DriveLibraries import *
from time import sleep

R1 = Robot('robot.cfg')
R1.gs.calibrate()
sleep(1)
R1.spkr.beep()
R1.btn.wait_for_bump('enter')
R1.cs.calibrate_white()
R1.btn.wait_for_bump('enter')
# Write Code Here ----------------------------------------
while True:
    R1.zeroGyro()
    R1.LineStop(0, 75, True)
    R1.btn.wait_for_bump('enter')

