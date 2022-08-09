#!/usr/bin/env python3

# Program for testing stuff without all of the framework; less things to go wrong

from sys import stderr
from threading import Thread
import time
from DriveLibraries import Robot
from Pathfinder import TrajectoryUtil

trajFile = open("Trajectory.txt")
list = eval(trajFile.read())
trajectory = TrajectoryUtil.createTrajectoryFromElements(list)

robot = Robot('robot.cfg')
def drive():
    robot.FollowTrajectory(trajectory, True)
program = Thread(target=drive)
program.start()
loopIndex = 0
while True:
    if not program.isAlive():
        robot.wheelPositions = robot.getWheelPositions()
        robot.currentYaw = robot.getYaw()
    robot.odometry.update(robot.currentYaw, robot.wheelPositions[0], robot.wheelPositions[1])
    loopIndex += 1
    #if loopIndex % 20 == 0:
    #    print(robot.getPose(), file=stderr)



# log = open("log.txt", "w")

# log.write("Forward Quasistatic Test Start\n")
# power = 10
# while power <= 100:
#     robot.tank.on(power, power)
#     time.sleep(0.3)
#     speed = (robot.lm.speed * robot.WheelCircumference) / (robot.lm.count_per_rot * 100)
#     log.write("{}, {}\n".format(power, speed))
#     power += 10
# robot.tank.off()
# log.write("Forward Quasistatic Test End\n")
# time.sleep(1)
# log.write("Reverse Quasistatic Test Start\n")
# power = -10
# while power >= -100:
#     robot.tank.on(power, power)
#     time.sleep(0.3)
#     speed = (robot.lm.speed * robot.WheelCircumference) / (robot.lm.count_per_rot * 100)
#     log.write("{}, {}\n".format(power, speed))
#     power -= 10
# robot.tank.off()
# log.write("Reverse Quasistatic Test End\n")
# time.sleep(1)
# log.write("Forward Dynamic Test Start\n")
# startTime = time.time()
# curTime = time.time() - startTime
# prevTime = 0
# prevSpeed = 0
# robot.tank.on(100, 100)
# while curTime <= 2:
#     speed = (robot.lm.speed * robot.WheelCircumference) / (robot.lm.count_per_rot * 100)
#     curTime = time.time() - startTime
#     #acceleration = (speed - prevSpeed) / (curTime - prevTime)
#     log.write("{}, {}\n".format(curTime, speed))
#     prevTime = curTime
#     prevSpeed = speed
# robot.tank.off()
# log.write("Forward Dynamic Test End\n")
# time.sleep(1)
# log.write("Reverse Dynamic Test Start\n")
# startTime = time.time()
# curTime = time.time() - startTime
# prevTime = 0
# prevSpeed = 0
# robot.tank.on(-100, -100)
# while curTime <= 2:
#     speed = (robot.lm.speed * robot.WheelCircumference) / (robot.lm.count_per_rot * 100)
#     curTime = time.time() - startTime
#     #acceleration = (speed - prevSpeed) / (curTime - prevTime)
#     log.write("{}, {}\n".format(curTime, speed))
#     prevTime = curTime
#     prevSpeed = speed
# robot.tank.off()
# log.write("Reverse Dynamic Test End")

# log.close()