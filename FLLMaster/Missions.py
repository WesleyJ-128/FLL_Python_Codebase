# Check if the program is running (rare) or if it is being used as a library
if __name__ == "__main__":
    # If it is running, import everything from Drivelibraries and create a robot object.
    from DriveLibraries import *
    robot = Robot('robot.cfg')
else:
    # If it is a library, import the robot object from MenuLib 
    # (which would have called Missions in the first place).
    from MenuLib import robot

# Mission Names should be prefaced with a##, where ## is two numbers.  These
# numbers will not show up on the display, and tell the program what order the
# missions should be in; a00 is first, followed by a01, and so on.  It is 
# reccomended that spaces are left between the numbers (a00, a05, a10...), to
# allow for future missions.

# Write Mission Programs Here ----------------------

def a05Star():
    # Drives a simple 4-pointed star, with arced sides
    robot.ArcTurn(-90, 30, 30)
    robot.ArcTurn(-90, 30, -30)
    robot.ArcTurn(-90, 30, 30)
    robot.ArcTurn(-90, 30, -30)

def a00LineFollow():
    # Follows a line for 50 cm, at 50% speed, stopping at end
    robot.LineFollow(50, 50, True)

def a06AvoidTest():
    robot.TriangleAvoid(0, 100, 50)

def a10Command_Key():
    # Drives in a pattern similar to the Apple command key
    #  o_o
    #  | |
    #  o‾o
    robot.DriveAtHeading(0, 50, 30, False)
    robot.ArcTurn(270, 10, 30)
    robot.DriveAtHeading(270, 50, 30, False)
    robot.ArcTurn(270, 10, 30)
    robot.DriveAtHeading(270 * 2, 50, 30, False)
    robot.ArcTurn(270, 10, 30)
    robot.DriveAtHeading(270 * 3, 50, 30, True)
    robot.GyroTurn(0)