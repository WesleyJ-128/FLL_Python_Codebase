# FLL_Python_Codebase
Driving libraries and a master program designed for FLL-legal robots.  Uses a robot.cfg file for robot parameters, and uses the ev3dev2 python library and brick firmware.

## Use:
The file ``Main.py`` is the only executable file other than ``test.py``, and is the program that creates the menu.  This program takes a while to start, so it should be started well before any timers.  On startup, the robot will eventually emit the normal beep (A4) signalling that the config file has been read and some menu systems have been initialized. It will then emit a low-pitched beep (different from any other beep) and turn the lights yellow.  This is standard procedure for recalibrating the gyro (inside this program, anyway), and the robot should be absolutely still from the first low-pitched beep to the second normal beep.  After the second normal beep, the robot will start the color sensor calibration routine, emitting a high-pitched beep. This signals that the color sensor should be placed on the white surface.  After pressing the center button, the robot will emit a normal beep signalling that the white surface has been stored.  The robot will then emit a low-pitched beep.  Again, place the robot on the black surface and press the center button. The robot will emit a normal beep, and the complete setup is done.

The screen will then show the name of the current mission across the top; and the current gyro and color sensor values (angle and RLI) towards the bottom in the center.  The lights will also likely begin flickering between red and green.  The color of the light is determined by the rate reported by the gyro sensor; if it is within 1 degree/second of zero, the light will turn green.  If the gyro reports that it is moving any faster than that, the light will turn red.  Therefore, if the robot is not being manipulated and is not running a mission, the light should be green most of the time.  If it is not, the gyro is likely drifting and needs to be recalibrated.

### Button functions:
* The center button runs the mission whose name is currently being displayed on the screen.  It also advances to the next mission afterward.
* The left button changes the selection to the previous mission, looping around to the last when pressing left on the first mission.
* The right button works the same way as the left, but moving foward instead of back
* The bottom button recalibrates the gyro
* The top button recalibrates the color sensor

**When a mission is running, all buttons lose their normal function, and will abort the current mission.**

### Adding Missions
To add a mission, simply create a new function in ``Missions.py``.  Mission Names should be prefaced with a##, where ## is two numbers.  These numbers will not show up on the display, they are used to alphabetically order the missions; a00 is first, followed by a01, and so on.  It is reccomended that spaces are left between the numbers (a00, a05, a10...), to allow for future missions.
