# Abstract sensor for the car

import numpy
import csv
import time
import numpy as np
from math import *
import copy

def sense(car_pos, all_paths, d_sense, t_all, t0, lookahead, ped_num):
    """
    car_pos: current car position
    all_paths: list of all pedestrian trajectories
    d_sense: sensing distance of car
    t_all: time points
    t0: current time
    lookahead: lookahead time
    ped_num: total number of pedestrians
    """
    A = np.array([[-1,0,0],[1,0,0],[0,-1,0],[0,1,0],[0,0,-1],[0,0,1]])
    all_b = []
    #Step 1: get time index
    t1 = t0 + lookahead
    idx0, idx1 = get_idx(t_all, t0, t1)
    # print(idx0, idx1)

    #Step 2: check distance between car and ped
    for i in range(ped_num):
        xp, yp = all_paths[i][idx0]
        if sqrt((car_pos[0] - xp)**2 + (car_pos[1] - yp)**2) <= d_sense:
            #Step 3: get trajectory over appropriate time period
            all_list = all_paths[i][idx0:idx1]
            xlist = [x[0] for x in all_list]
            ylist = [x[1] for x in all_list]

            #Step 4: get max and min values for the bounding box
            xmin = min(xlist)
            xmax = max(xlist)
            ymin = min(ylist)
            ymax = max(ylist)
            all_b.append((A, np.array([[-(xmin-2)], [xmax+ 2], [-(ymin-2)], [(ymax+2)], [-t0], [t1]])))

    return all_b

def ped_paths(id, filename, timestep):
    with open('pedData/'+filename) as csvfile:
        dataReader = csv.reader(csvfile, delimiter=',')
        frame=1
        waypoints = []
        t = []
        x = 0
        y = 0
        for row in dataReader:
            if row[-2] == str(frame + 1):
                waypoints.append([x, y])
                t.append((frame-1)*timestep)
                frame += 1
                x = 0
                y = 0
                # print(row[-2], frame)

            if row[-2] == str(frame):
                if row[0] == str(id):
                    # print('check')
                    x += float(row[1])/(25) + 110
                    y += float(row[2])/20 - 5
                    # print(x, y)

                else:
                    x += 0
                    y += 0
    return waypoints, t

def all_paths(ped_num, filename, timestep):
    all_wps = []
    for i in range(ped_num):
        wp, t = ped_paths(i, filename, timestep)
        all_wps.append(wp)
    return all_wps, t

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

def get_idx(t, t0, t1):
    idx0 = (np.abs(np.array(t) - t0)).argmin()
    idx1 = (np.abs(np.array(t) - t1)).argmin()
    return idx0, idx1

if __name__ == '__main__':
    filename = 'intersection_04_traj_ped.csv'
    ped_num = 112
    timestep = 32e-3
    ts = time.time()
    all_paths, t_all = all_paths(ped_num, filename, timestep)
    # print(get_idx(t, 0, 1))
    # print(time.time() - ts)

    car_pos = (110, 0)
    d_sense = 15
    t0 = 0
    lookahead = 4
    ts = time.time()
    all_b = sense(car_pos, all_paths, d_sense, t_all, t0, lookahead, ped_num)
    print(time.time()-ts)
    print(all_b)
