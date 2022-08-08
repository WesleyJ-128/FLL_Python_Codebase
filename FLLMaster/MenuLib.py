# Import everything from DriveLibraries to allow interaction with the robot, and to create a Robot object
from DriveLibraries import *
# Used for running a mission in a killable way
import multiprocessing
# Some things need delays
from time import sleep
# Define variables with null values so they can be used as global variables later
Missions = None
mission = None
robot = None
programs = []
numPrograms = 0
loopIndex = 0

# count is used to determine which mission should be run, starts at 1 not 0
count = 1

def init(confFile):
    """
    Creates a ``Robot`` oject named ``robot`` using the filename given in ``confFile``, loads all mission functions 
    from ``Missions.py`` into ``programs`` alphabetically, using the three-character prefix on all mission function names, 
    and shows the first mission name on the display.
    """
    # Declare global variables
    global programs
    global numPrograms
    global robot
    global Missions

    # Instantiate a Robot object named "robot"
    robot = Robot(confFile)
    # Now that a robot object exists, the mission programs, which need a robot object, can be imported
    import Missions
    # This is how the menu auto-adds missions.  dir() returns a list of all objects contained in whatever it was given.
    # However, there are a lot more objects than just the mission functions in Missions.py, so the list needs to be pruned.
    programs = dir(Missions)
    # Setting up some varibles for pruning
    i = 0
    index = -1
    length = len(programs)

    # This will repeat the mission check on every element of the list of all objects in Missions.py
    while i < length:
        # This checks if the name of the object currently being checked starts with a##, like all missions should.
        if not ((programs[index][0] == "a") and (programs[index][1].isnumeric()) and (programs[index][2].isnumeric())):
            # If it does not, the object will be removed from the programs list, as it is not a mission
            programs.pop(index)
        else:
            # If it is, the index varible wil be decreased by one to skip over the mission program the next time around
            # (the scan starts at the back of the list).
            index -= 1
        # Increment i by 1 every loop cycle
        i += 1
    
    # Now that the only things left in programs are missions, the length of programs can be used as the number of missions
    # (used in AddCount and MinusCount, for rollover).
    numPrograms = len(programs)

    # Clear the display
    robot.disp.clear()
    # Dislpay the first mission name (the [3:] is a string split; it prevents the a## from being displayed)
    robot.disp.text_grid(programs[0][3:], font=robot.fonts.load('timR24'))

def runCurrentMission():
    """
    Gets the currently selected mission from ``Missions.py`` and runs it.  Once the mission has completed, 
    it stops the motors and allows them to glide to allow for manual robot positioning.
    """
    global robot
    # Finds the object in Missions.py that has the same name as the currently selected mission 
    # (count - 1 is used because count starts at 1 not 0), and creates the function pointer 
    # method_to_call, which is the selected mission
    method_to_call = getattr(Missions, programs[count - 1])
    # Runs the selected mission
    method_to_call()
    # Stops the  drive motors and allows them to glide
    robot.rm.off(brake=False)
    robot.lm.off(brake=False)
    # Attempts to stop auxillary motors.  If there are none, the try blocks will fail and the program will continue
    try:
        robot.m1.off(brake=False)
    except:
        pass
    try:
        robot.m2.off(brake=False)
    except:
        pass

def initthread():
    """
    Creates the ``mission`` process that is used to run missions while being able to abort said mission.  
    This is also done in ``run()``, but needs to be done before other things will work properly
    """
    # Declare mission as a global variable because it is being edited
    global mission
    # Create a process that will call runCurrentMission
    mission = multiprocessing.Process(target=runCurrentMission)

def run():
    """
    Called when the center button is pressed, this creates a process 
    to run the current mission, reset the gyro to zero, then start the mission.
    """
    # Declare mission as a global variable because it is being edited
    global mission
    # Create a process that will call runCurrentMission
    mission = multiprocessing.Process(target=runCurrentMission)
    
    # Reset the gyro angle to zero
    robot.zeroGyro()
    # Start the mission process
    mission.start()
    
def display():
    """
    Display the currently selected mission on the screen
    """
    # Read the currently selected mission from the programs list, using count
    # The [3:] is a string split so the a## prefix is not displayed, and 1 is subtracted
    # from count because count starts at one, while list indexes start at zero.
    name = programs[count - 1][3:]
    # Display the mission name on the screen, in 24pt Times New Roman
    robot.disp.text_grid(name, font=robot.fonts.load('timR24'))

def abort():
    """
    Kills the current mission process, stops the motors and allows them to glide, and undoes auto-advance
    """
    # Kill the mission process
    mission.terminate()
    # Wait for it to completely finish
    mission.join()
    # Stop the drive motors
    robot.lm.off(brake=False)
    robot.rm.off(brake=False)
    # Attempts to stop auxillary motors.  If there are none, the try blocks will fail and the program will continue
    try:
        robot.m1.off(brake=False)
    except:
        pass
    try:
        robot.m2.off(brake=False)
    except:
        pass
    # Undo the auto-advance so the mission could easily be redone
    minusCount()
    display()

def addCount():
    """
    Increases the ``count`` variable by one, resetting to one when numPrograms would have been ecxeeded
    """
    # Declare count as global because it is being edited
    global count
    # Take the modulo of count and numPrograms (to rollover), then add one
    count = (count % numPrograms) + 1

def minusCount():
    """
    Decreases the ``count`` variable by one, resetting to numPrograms when count would have gone below 1
    """
    # Declare count as global because it is being edited
    global count
    # (count + (numPrograms - 2)) % numPrograms is equal to two less than count, with the desired rollover behavior.
    # However, that is decreasing by two, and its range is zero to numPrograms - 1.  Thus, 1 is added to the result to
    # correct the range and decrease by only one.
    count = ((count + (numPrograms - 2)) % numPrograms) + 1

def checkDrift():
    """
    Turns the brick light green if the gyro reports a rate within one degree/second of zero, and red otherwise.
    This is used as a signal to the user that the gyro may be drifting.
    """
    # Read the gyro angle and rate.  Angle and rate mode is consistantly used because switching modes resets the angle.
    ar = robot.gs.angle_and_rate
    # angle_and_rate returns a list; this function only needs rate
    rate = ar[1]
    # Check if the reported rate is outside of 1 degree/second from zero
    if rate < -0.5 or rate > 0.5:
        # If it is, turn both lights red (lights are turned off to clear)
        robot.led.all_off()
        robot.led.set_color('LEFT', 'RED')
        robot.led.set_color('RIGHT', 'RED')
    else:
        # Otherwise, reset the lights to green
        robot.led.reset()

def displaySensor():
    """
    Periodically displays the color and gyro sensor values at the bottom of the screen.
    """
    # loopIndex is equal to zero every hundeth loop cycle, so this function only runs that frequently.
    # This is to prevent flickering of the printout.
    if loopIndex == 0:
        # Draws a white rectangle where the sensor values will be.  This effectively clears only 
        # that section of the screen.
        robot.disp.rectangle(False, 0, 89, 177, 120, 'white', 'white')
        # Display commands do not take effect until update() is called
        robot.disp.update()
        # Displays the current calibrated color sensor RLI reading in the correct place in 10pt Courier New
        robot.disp.text_pixels("P1, Color:" + str(robot.correctedRLI), False, 40, 90, font=robot.fonts.load('courR10'))
        # Displays the current gyro sensor angle reading
        robot.disp.text_pixels("P2, Gyro:" + str(robot.correctedAngle), False, 40, 100, font=robot.fonts.load('courR10'))
        # Apply pending changes
        robot.disp.update()

def calibrate():
    """
    Provides a UI wrapper for the built-in ``robot.gs.calibrate`` function to calibrate the gyro

    Use: When called, the brick lights will turn yellow, the robot will emit a low pitched beep that is not used anywhere else,
    and when calibration is complete, the robot will emit a normal beep and the lights will turn green again.  The robot should
    be completely still between the two beeps while the lights are yellow.
    """
    # Turn the lights yellow (clearing first)
    robot.led.all_off()
    robot.led.set_color('LEFT', 'YELLOW')
    robot.led.set_color('RIGHT', 'YELLOW')
    # Unique low pitched beep (Don't touch the robot!)
    robot.spkr.beep('-f 369.99')
    # Calibrate the gyro
    robot.gs.calibrate()
    # Ensure calibration has completely finished
    sleep(1)
    # Normal beep (Calibration finished; safe to touch robot)
    robot.spkr.beep()
    # Reset the lights to green
    robot.led.reset()