#!/usr/bin/env python3
from MenuLib import init, initthread, runCurrentMission
from time import sleep
loopIndex = 0

init('robot.cfg')
initthread()
from MenuLib import *
calibrate()
robot.reflectCal()

def left(state):
    if state:
        if not mission.is_alive():
            minusCount()
            display()
        else:
                abort()
                minusCount()
                display()
def right(state):
    if state:
        if not mission.is_alive():
            addCount()
            display()
        else:
            abort()
            minusCount()
            display()
def down(state):
    if state:
        if not mission.is_alive():
            calibrate()
        else:
            abort()
            minusCount()
            display()
def up(state):
    if state:
        if not mission.is_alive():
            robot.reflectCal()
        else:
            abort()
            minusCount()
            display()
def enter(state):
    if state:
        if not mission.is_alive():
            run()
            addCount()
            display()
        else:
            abort()
            minusCount()
            display()
def backspace(state):
    pass

robot.btn.on_left = left
robot.btn.on_right = right
robot.btn.on_up = up
robot.btn.on_down = down
robot.btn.on_enter = enter
robot.btn.on_backspace = backspace      

while True:    
    robot.btn.process()
    loopIndex = (loopIndex + 1) % 100
    displaySensor()
    checkDrift()