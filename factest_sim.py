import numpy as np
import csv
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection
from sensor import *
from threading import Thread
from control_utils import *
from scipy.integrate import odeint
from empty_map import map
from xref_planner import *
import time

filename = 'intersection_02_traj_ped.csv'
pedNum = 120
time_stamp = 0
time_step = 0.1 #64e-3
cur_pos = [124,4,0,0,0]
stop = 0
d_sense = 10
agent_paths = createPedPath(filename, pedNum)
lookahead_step_num = int(4/(time_step))
ts = 0
theta = []
goal = []
obs = []
pred_wps = None
sense_ped =[[0,0]]


def animate(i):
    global cur_pos, time_step, time_stamp, agent_paths, stop
    global pedNum, agent_paths, lookahead_step_num, d_sense
    global waypoints, ts, pred_wps, sense_ped

    tsyn = None
    obs = []
    if time_step*time_stamp - ts >= 1:
        theta, goal, obs = map(cur_pos)
        ped_preds, sense_ped = ped_rects(cur_pos, pedNum, agent_paths, time_step, time_stamp, lookahead_step_num, d_sense)
        # if len(ped_preds) > 0:
        #     print(len(ped_preds))
        obs.extend(ped_preds)
        # print(obs)

        bloat_list = [3 for i in range(10)]
        t1=time.time()
        pred_wps = find_xref(theta, goal, obs, 10, 5, bloat_list, old_wp = pred_wps)
        tsyn = time.time() - t1
        # if pred_wps == None:
        # print(pred_wps)
        ts = time_step*time_stamp

    # plot car
    current_pos.set_data(cur_pos[0], cur_pos[1])
    x_bound = [cur_pos[0] + 2*cos(cur_pos[2]) - 1.5*sin(cur_pos[2]),
               cur_pos[0] + 2*cos(cur_pos[2]) + 1.5*sin(cur_pos[2]),
               cur_pos[0] - 2*cos(cur_pos[2]) + 1.5*sin(cur_pos[2]),
               cur_pos[0] - 2*cos(cur_pos[2]) - 1.5*sin(cur_pos[2]),
               cur_pos[0] + 2*cos(cur_pos[2]) - 1.5*sin(cur_pos[2])]
    y_bound = [cur_pos[1] + 2*sin(cur_pos[2]) + 1.5*cos(cur_pos[2]),
               cur_pos[1] + 2*sin(cur_pos[2]) - 1.5*cos(cur_pos[2]),
               cur_pos[1] - 2*sin(cur_pos[2]) - 1.5*cos(cur_pos[2]),
               cur_pos[1] - 2*sin(cur_pos[2]) + 1.5*cos(cur_pos[2]),
               cur_pos[1] + 2*sin(cur_pos[2]) + 1.5*cos(cur_pos[2]),]
    # car_outline.set_data(x_bound, y_bound)

    # plot_pedestrians
    agent_pos_x = []
    agent_pos_y = []

    for j in range(pedNum):
        agent_pos_x.append(agent_paths[j][time_stamp][0])
        agent_pos_y.append(agent_paths[j][time_stamp][1])

    agent_pos.set_data(agent_pos_x, agent_pos_y)

    # print(pred_wps)
    if pred_wps != [] and pred_wps != None:
        wps_x = [wp[0] for wp in pred_wps]
        wps_y = [wp[1] for wp in pred_wps]
        path_proj.set_data(wps_x,wps_y)
        waypoints = pred_wps

    if not stop:
        if pred_wps != None:
            wps = set_waypoints(waypoints)
            t = [time_stamp*time_step, (time_stamp+1)*time_step]
            q = run_model(cur_pos, t, wps)
            v = q[-2]
            omega = q[-1]
            print(v)
        else:
            v = 0
            omega = 0

        # print(v)
        with open('names3.csv', 'a+') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',')
            spamwriter.writerow([cur_pos[0], cur_pos[1], tsyn, time_stamp*time_step, len(waypoints), len(obs)])

        cur_pos[0] = cur_pos[0] + v*cos(cur_pos[2])*time_step
        cur_pos[1] = cur_pos[1] + v*sin(cur_pos[2])*time_step
        cur_pos[2] = cur_pos[2] + omega*time_step

        sensed.set_data([ped[0] for ped in sense_ped], [ped[1] for ped in sense_ped])

        time_stamp += 1

        if pred_wps != None:
            if sqrt((cur_pos[0] - wps[2])**2 + (cur_pos[1] - wps[3])**2) < 2:
                waypoints.pop(0)
                if len(waypoints) < 2:
                    stop = 1
                    # print(time_step*time_stamp)

        # print(time_step*time_stamp - ts)




    return current_pos, agent_pos, path_proj, sensed

# def get_controller():
#     global cur_pos, time_step, time_stamp
#     global theta,goal,obs
#     ts = 0
#     while 1:
#         if time_step*time_stamp - ts > 1:
#             ts = time_step*time_stamp
#             print('replan')
#             print(len(obs))
#     return None
#
# def sense_env():
#     global cur_pos, pedNum, agent_paths, time_step, time_stamp, lookahead_step_num, d_sense
#     global theta, goal, obs
#
#     while 1:
#         theta, goal, obs = map(cur_pos)
#         ped_preds = ped_rects(cur_pos, pedNum, agent_paths, time_step, time_stamp, lookahead_step_num, d_sense)
#         obs.extend(ped_preds)
#     return None

# t1 = Thread(target = sense_env)
# t1.daemon = True
# t2 = Thread(target = get_controller)
# t2.daemon = True
#
# t1.start()
# t2.start()

if __name__ == '__main__':
    # filename = 'intersection_03_traj_ped.csv'
    # pedNum = 10
    # time_stamp = 0
    # time_step = 64e-3
    # cur_pos = [122,2,0,0,0]
    # stop = 0
    # d_sense = 10

    waypoints = [[122, 2, 0], [136, 5, 10], [138, 20, 20]]

    fig, ax = plt.subplots(figsize=(7, 7))

    patches = [Rectangle((113, 9), 19, 21), Rectangle((113, -22), 19, 21), Rectangle((141, -22), 19, 21), Rectangle((141, 9), 19, 21)]
    pc = PatchCollection(patches, facecolor='red')
    ax.add_collection(pc)


    current_pos, = plt.plot([], [] , ls='None', color='b', marker='o', label='car_pos')
    # car_outline, = plt.plot([], [] , color='b')
    agent_pos, = plt.plot([], [] , ls='None', color='k', marker='o', label='agent_pos')
    path_proj, = plt.plot([], [] , ls = '--', color='g')
    sensed, = plt.plot([], [] , ls='None', color='g', marker='o', label='agent_pos')

    ani = animation.FuncAnimation(fig, animate, frames=1000, interval = 100)

    plt.axis([118, 145, -3, 30])
    plt.grid()
    plt.show()
