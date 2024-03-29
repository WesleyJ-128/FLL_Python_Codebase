#!/usr/bin/env python3

# This is the program that should be run during use; it creates the interactive menu.
# It is also the only executable file other than test.py, which is only for testing things.

# Initialize loopIndex variable, used in displaySensor()
loopIndex = 0

# Import only the functions required for setup; 
# MenuLib needs them run before importing other things
from MenuLib import init, initthread
# Used for printing to the VS Code output window
from sys import stderr

# Run the init function from MenuLib, config file name, 
# used to instatiate the robot object
init('robot.cfg')
# Create the mission process used for running the current mission
initthread()
# Now that setup has finished, import everything else from MenuLib
import MenuLib

# Calibrate the gyro
MenuLib.calibrate()
# Calibrate the color sensor
MenuLib.robot.reflectCal()
# Initialization has finished, print status message to VS Code output window
print("Finished Init", file=stderr)

# Define the functions for button presses
def left():
    """
    Called when left button pressed; if there is no mission running, 
    decrease count and update the display. Otherwise, abort the current mission.
    """
    if not MenuLib.mission.is_alive():
        MenuLib.minusCount()
        MenuLib.display()
    else:
        MenuLib.abort()
def right():
    """
    Called when right button pressed; if there is no mission running, 
    increase count and update the display. Otherwise, abort the current mission.
    """
    if not MenuLib.mission.is_alive():
        MenuLib.addCount()
        MenuLib.display()
    else:
        MenuLib.abort()
def down():
    """
    Called when bottom button pressed; if there is no mission running, 
    recalibrate the gyro. Otherwise, abort the current mission.
    """
    if not MenuLib.mission.is_alive():
        MenuLib.calibrate()
    else:
        MenuLib.abort()
def up():
    """
    Called when top button pressed; if there is no mission running, 
    recalibrate the color sensor. Otherwise, abort the current mission.
    """
    if not MenuLib.mission.is_alive():
        MenuLib.robot.reflectCal()
    else:
        MenuLib.abort()
def enter():
    """
    Called when center button pressed; if there is no mission running, launch the selected mission, 
    increase count, and update the display. Otherwise, abort the current mission.
    """
    if not MenuLib.mission.is_alive():
        MenuLib.run()
        MenuLib.addCount()
        MenuLib.display()
    else:
        MenuLib.abort()

# Print to the VS Code window that the button functions have been defined
print("Functions Defined", file=stderr)

# Define a dictionary for calling the functions defined above based on the names for the buttons
buttonMap = {
    'left': left,
    'right': right,
    'enter': enter,
    'up': up,
    'down': down
}

# Print to the VS Code window that the button dictionary has been defined
print("Map Defined", file=stderr)

# Start the infinite menu loop
while True:
    # Store the list of buttons currently pressed as buttonlist
    buttonlist = MenuLib.robot.btn.buttons_pressed
    # Check if there are any buttons pressed; if there is, run the corresponding function
    if buttonlist:
        buttonMap[buttonlist[0]]()
    # Increment loopIndex and reset to zero every hundreth loop
    loopIndex = (loopIndex + 1) % 100
    # Make sure there is no mission running (to prevent resource conflicts), 
    # and if there is none, display the sensor values and update the light color based on the gyro sensor
    if not MenuLib.mission.is_alive():
        MenuLib.checkDrift()
        MenuLib.displaySensor()