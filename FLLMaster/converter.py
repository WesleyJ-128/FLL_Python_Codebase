import json
from Pathfinder import TrajectoryUtil

# file = open("../WPILib_Java_PathGen/src/main/deploy/paths/Lineup.wpilib.json")
# dict = json.loads(file.read())
# list = []
# for i in dict:
#     list.append(i["time"])
#     list.append(i["velocity"])
#     list.append(i["acceleration"])
#     list.append(i["pose"]["translation"]["x"])
#     list.append(i["pose"]["translation"]["y"])
#     list.append(i["pose"]["rotation"]["radians"])
#     list.append(i["curvature"])
# stringList = str(list)
# newList = [float(i) for i in stringList.strip("[]").split(", ")]
# print(newList)

from configparser import ConfigParser
conf = ConfigParser()
conf.read('robot.cfg')


list = conf.items('Trajectories')
dict = {list[i][0]: TrajectoryUtil.createTrajectoryFromElements([float(j) for j in list[i][1].strip("[]").split(", ")]) for i in range(len(list))}
print(dict)