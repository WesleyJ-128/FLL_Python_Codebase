from DriveLibraries import *
import multiprocessing
from time import sleep
Missions = None
mission = None
robot = None
programs = []
numPrograms = 0
count = 1
loopIndex = 0

def init(confFile):
    global programs
    global numPrograms
    global robot
    global Missions
    robot = Robot(confFile)
    import Missions
    programs = dir(Missions)
    i = 0
    index = -1

    length = len(programs)
    while i < length:
        if not ((programs[index][0] == "a") and (programs[index][1].isnumeric()) and (programs[index][2].isnumeric())):
            programs.pop(index)
        else:
            index -= 1
        i += 1
    numPrograms = len(programs)

    robot.disp.clear()
    robot.disp.text_grid(programs[0][3:], font=robot.fonts.load('timR24'))

def runCurrentMission():
    method_to_call = getattr(Missions, programs[count - 1])
    method_to_call()
    robot.rm.off(brake=False)
    robot.lm.off(brake=False)

def initthread():
    global mission
    mission = multiprocessing.Process(target=runCurrentMission)

def run():
    global mission
    mission = multiprocessing.Process(target=runCurrentMission)
    robot.zeroGyro()
    mission.start()
    
def display():
    name = programs[count - 1][3:]
    robot.disp.text_grid(name, font=robot.fonts.load('timR24'))

def abort():
    mission.terminate()
    mission.join()
    sleep(1)
    robot.lm.off(brake=False)
    robot.rm.off(brake=False)
    minusCount()
    display()

def addCount():
    global count
    count = (count % numPrograms) + 1

def minusCount():
    global count
    count = ((count + (numPrograms - 2)) % numPrograms) + 1

def checkDrift():
    ar = robot.gs.angle_and_rate
    rate = ar[1]
    if rate < -0.5 or rate > 0.5:
        robot.led.all_off()
        robot.led.set_color('LEFT', 'RED')
        robot.led.set_color('RIGHT', 'RED')
    else:
        robot.led.reset()

def displaySensor():
    if loopIndex == 0:
        robot.disp.rectangle(False, 0, 89, 177, 120, 'white', 'white')
        robot.disp.update
        robot.cs._ensure_mode(robot.cs.MODE_COL_REFLECT)
        robot.disp.text_pixels("P1, Color:" + str(robot.correctedRLI), False, 40, 90, font=robot.fonts.load('courR10'))
        robot.disp.text_pixels("P2, Gyro:" + str(robot.correctedAngle), False, 40, 100, font=robot.fonts.load('courR10'))
        robot.disp.update()

def calibrate():
    robot.led.all_off()
    robot.led.set_color('LEFT', 'YELLOW')
    robot.led.set_color('RIGHT', 'YELLOW')
    robot.spkr.beep('-f 369.99')
    robot.gs.calibrate()
    sleep(1)
    robot.spkr.beep()
    robot.led.reset()