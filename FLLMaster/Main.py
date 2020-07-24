#!/usr/bin/env python3
from DriveLibraries import *
from time import sleep
from ev3dev2.display import *
disp = Display()

R1 = Robot('robot.cfg')
R1.gs.calibrate()
sleep(1)
R1.spkr.beep()
R1.reflectCal()
R1.btn.wait_for_bump('enter')
# Write Code Here ----------------------------------------

def left(state):
    if state:
        R1.kpLine = R1.kpLine - 0.01
        print(R1.kpLine)

def right(state):
    if state:
        R1.kpLine = R1.kpLine + 0.01
        print(R1.kpLine)
    
def up(state):
    pass
def down(state):
    pass
def enter(state):
    if state:
        R1.LineFollow(50, 20, True)
def backspace(state):
    pass

R1.btn.on_left = left
R1.btn.on_right = right
R1.btn.on_up = up
R1.btn.on_down = down
R1.btn.on_enter = enter
R1.btn.on_backspace = backspace


while True:    
    R1.btn.process()


