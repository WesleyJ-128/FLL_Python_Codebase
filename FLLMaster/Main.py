#!/usr/bin/env python3
from MenuLib import init, initthread, runCurrentMission
from time import sleep
loopIndex = 0

init('robot.cfg')
initthread()
from MenuLib import *
calibrate()
robot.reflectCal()

def left():
    robot.btn.wait_for_released('left, right, up, down, enter')
    if not mission.is_alive():
        minusCount()
        display()
    else:
        abort()
        minusCount()
        display()
def right():
    robot.btn.wait_for_released('left, right, up, down, enter')
    if not mission.is_alive():
        addCount()
        display()
    else:
        abort()
        minusCount()
        display()
def down():
    robot.btn.wait_for_released('left, right, up, down, enter')
    if not mission.is_alive():
        calibrate()
    else:
        abort()
        minusCount()
        display()
def up():
    robot.btn.wait_for_released('left, right, up, down, enter')
    if not mission.is_alive():
        robot.reflectCal()
    else:
        abort()
        minusCount()
        display()
def enter():
    robot.btn.wait_for_released('left, right, up, down, enter')
    if not mission.is_alive():
        run()
        addCount()
        display()
    else:
        abort()
        minusCount()
        display()

buttonMap = {
    'left': left(),
    'right': right(),
    'enter': enter(),
    'up': up(),
    'down': down()
}

while True:
    buttonMap[robot.btn.buttons_pressed]()
    checkDrift()
    displaySensor()
    loopIndex = (loopIndex + 1) % 100