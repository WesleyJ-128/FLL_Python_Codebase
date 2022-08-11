from fileinput import close
import json

def jsonToini(fileName):
    file = open("../WPILib_Java_PathGen/src/main/deploy/paths/{}.wpilib.json".format(fileName))
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
    file.close()
    return "{} = {}".format(fileName, list)

with open('robot.cfg', 'a') as cfg:
    cfg.write("\n" + jsonToini("SteelConstruction"))