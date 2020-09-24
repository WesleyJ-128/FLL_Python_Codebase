#!/usr/bin/env python3

loopIndex = 0

from MenuLib import init, initthread
from sys import stderr

init('robot.cfg')
initthread()
from MenuLib import *
calibrate()
robot.reflectCal()
print("Finished Init", file=stderr)

def left():
    if not mission.is_alive():
        minusCount()
        display()
    else:
        abort()
def right():
    if not mission.is_alive():
        addCount()
        display()
    else:
        abort()
def down():
    if not mission.is_alive():
        calibrate()
    else:
        abort()
def up():
    if not mission.is_alive():
        robot.reflectCal()
    else:
        abort()
def enter():
    if not mission.is_alive():
        run()
        addCount()
        display()
    else:
        abort()

print("Functions Defined", file=stderr)

buttonMap = {
    'left': left,
    'right': right,
    'enter': enter,
    'up': up,
    'down': down
}

print("Map Defined", file=stderr)

while True:
    from MenuLib import mission
    buttonlist = robot.btn.buttons_pressed
    if buttonlist:
        buttonMap[buttonlist[0]]()
    loopIndex = (loopIndex + 1) % 100
    if not mission.is_alive():
        checkDrift()
        displaySensor()