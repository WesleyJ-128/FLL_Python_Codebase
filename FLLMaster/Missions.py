from DriveLibraries import *
robot = Robot('robot.cfg')
# Write Mission Programs Here ----------------------

def a00Star():
    robot.ArcTurn(-90, 30, 30)
    robot.ArcTurn(-90, 30, -30)
    robot.ArcTurn(-90, 30, 30)
    robot.ArcTurn(-90, 30, -30)
def a05LineFollow():
    robot.LineFollow(50, 50, True)
def a10Command_Key():
    robot.DriveAtHeading(0, 50, 30, False)
    robot.ArcTurn(270, 30, 30)
    robot.DriveAtHeading(270, 50, 30, False)
    robot.ArcTurn(270, 30, 30)
    robot.DriveAtHeading(270 * 2, 50, 30, False)
    robot.ArcTurn(270, 30, 30)
    robot.DriveAtHeading(270 * 3, 50, 30, True)
    robot.GyroTurn(0)
