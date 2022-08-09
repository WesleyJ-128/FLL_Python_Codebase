import json


file = open("../WPILib_Java_PathGen/src/main/deploy/paths/Loop.wpilib.json")
dict = json.loads(file.read())
list = []
for i in dict:
    list.append(i["time"])
    list.append(i["velocity"])
    list.append(i["acceleration"])
    list.append(i["pose"]["translation"]["x"])
    list.append(i["pose"]["translation"]["y"])
    list.append(i["pose"]["rotation"]["radians"])
    list.append(i["curvature"])
print(list)