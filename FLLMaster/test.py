#!/usr/bin/env python3

# Program for testing stuff without all of the framework; less things to go wrong

from sys import stderr
from threading import Thread
import time
from DriveLibraries import Robot


robot = Robot('robot.cfg')
def drive():
    time.sleep(2)
    robot.DriveAtHeading(0.0, 100, 20, True)
    robot.GyroTurn(90)
    robot.DriveAtHeading(90, 20, 20, True)
    robot.GyroTurn(180)
    robot.DriveAtHeading(180, 100, 20, True)
    robot.GyroTurn(270)
    robot.DriveAtHeading(270, 20, 20, True)
    robot.GyroTurn(360)
program = Thread(target=drive)
program.start()
loopIndex = 0
while True:
    if not program.isAlive():
        robot.wheelPositions = robot.getWheelPositions()
        robot.currentYaw = robot.getYaw()
    robot.odometry.update(robot.currentYaw, robot.wheelPositions[0], robot.wheelPositions[1])
    loopIndex += 1
    if loopIndex % 20 == 0:
        print(robot.getPose(), file=stderr)

