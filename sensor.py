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
    x = car_pos[0]
    y = car_pos[1]
    ped_predict = []
    for i in range(pedNum):
        xp, yp = pedPaths[i][time_step]
        if sqrt((x - xp)**2 + (y - yp)**2) <= d_sense:
            ped_predict.append(pedPaths[i][time_step:time_step+lookahead_step_num])
    return ped_predict

def ped_rects(car_pos, pedNum, pedPaths, time_step, time_stamp, lookahead_step_num, d_sense):
    all_rects = []
    A_time = np.array([[-1,0,0],[1,0,0],[0,-1,0],[0,1,0],[0,0,-1],[0,0,1]])
    t1 = time_step*time_stamp
    t2 = (time_stamp + lookahead_step_num)*time_step

    x = car_pos[0]
    y = car_pos[1]
    ped_predict = []
    for i in range(pedNum):
        xp, yp = pedPaths[i][time_stamp]
        if sqrt((x - xp)**2 + (y - yp)**2) <= d_sense:
            pred_path = pedPaths[i][time_stamp:time_stamp+lookahead_step_num]
            x_path = [pt[0] for pt in pred_path]
            y_path = [pt[1] for pt in pred_path]
            b = np.array([[-(min(x_path)-2)],[(max(x_path)+2)],[-(min(y_path)-2)],[-(max(y_path)-2)],[-t1],[t2]])
            all_rects.append([A_time, b])

    return all_rects


# if time_step*time_stamp - ts > 1:
#     ts = time_step*time_stamp
#     bloat_list = [3 for i in range(10)]
#     Theta = [np.array([[-1,0],[1,0],[0,-1],[0,1]]), np.array([[-(cur_pos[0]-1)],[(cur_pos[0]+1)],[-(cur_pos[1]-1)],[(cur_pos[1]+1)]])]
#     print(find_xref(Theta, goal, obs, 10, 5, bloat_list, old_wp = None))
