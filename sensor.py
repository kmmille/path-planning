import numpy as np
from math import *
import csv
import copy

def createPedPath(filename, pedNum):
    all_paths = []
    for i in range(pedNum):
        waypoints = []
        with open('pedData/'+filename) as csvfile:
            dataReader = csv.reader(csvfile, delimiter=',')
            frame=1
            x = 0
            y = 0
            for row in dataReader:
                if row[-2] == str(frame + 1):
                    waypoints.append([x, y])
                    frame += 1
                    x = 0
                    y = 0

                if row[-2] == str(frame):
                    if row[0] == str(i):
                        x += float(row[1])/(25) + 115
                        y += float(row[2])/20 - 5

                    else:
                        x += 0
                        y += 0

        waypoints2 = copy.deepcopy(waypoints)
        waypoints2.reverse()
        waypoints.extend(waypoints2)
        all_paths.append(waypoints)
    return all_paths

def sense_peds(car_pos, pedNum, pedPaths, time_step, lookahead_step_num, d_sense):
    x, y, theta = car_pos
    ped_predict = []
    for i in range(pedNum):
        xp, yp = pedPaths[i][time_step]
        if sqrt((x - xp)**2 + (y - yp)**2) <= d_sense:
            ped_predict.append(pedPaths[i][time_step:time_step+lookahead_step_num])
    return ped_predict
