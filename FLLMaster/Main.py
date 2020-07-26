#!/usr/bin/env python3
from MenuLib import init
import MenuLib
from time import sleep
# Write Code Here ----------------------------------------

init()
from MenuLib import *
MenuLib.robot.reflectCal()

def left(state):
    if state:
        minusCount()
        display()
def right(state):
    if state:
        addCount()
        display()
def down(state):
    if state:
        calibrate()
def up(state):
    if state:
        abort()
        minusCount()
        display()
def enter(state):
    if state:
        run()
        addCount()
        display()
def backspace(state):
    if state:
        abort()

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