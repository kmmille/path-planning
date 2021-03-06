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

filename = 'intersection_09_traj_ped.csv'
pedNum = 120
time_stamp = 0
time_step = 0.1 #64e-3
cur_pos = [122,6,0,0,0]
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
ts1 = 0


def animate(i):
    global cur_pos, time_step, time_stamp, agent_paths, stop
    global pedNum, agent_paths, lookahead_step_num, d_sense
    global waypoints, ts, pred_wps, sense_ped, ts1

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
        ts1 = ts

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
    else:
        waypoints = []
        # print(waypoints)

    if not stop:
        if pred_wps != None and pred_wps!=[]:
            wps = set_waypoints(waypoints)
            t = [time_stamp*time_step, (time_stamp+1)*time_step]
            q = run_model(cur_pos, t, wps)
            # print(wps)
            v = q[-2]
            omega = q[-1]
        else:
            v = 0
            omega = 0

        # print(v)
        with open('FACTEST_09_straight_4s.csv', 'a+') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',')
            spamwriter.writerow([cur_pos[0], cur_pos[1], tsyn, time_stamp*time_step, len(waypoints), len(obs)])
        #
        cur_pos[0] = cur_pos[0] + v*cos(cur_pos[2])*time_step
        cur_pos[1] = cur_pos[1] + v*sin(cur_pos[2])*time_step
        cur_pos[2] = cur_pos[2] + omega*time_step

        sensed.set_data([ped[0] for ped in sense_ped], [ped[1] for ped in sense_ped])

        time_stamp += 1

        if pred_wps != None:
            if sqrt((cur_pos[0] - wps[2])**2 + (cur_pos[1] - wps[3])**2) < 2:# and wps[-1] <= time_step*time_stamp - ts1:
                waypoints.pop(0)
                ts1 = time_step*time_stamp
                if len(waypoints) < 1:
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

def create_rects(ped_preds):
    # colors = ['orange', 'coral', 'crimson', 'red']
    rects = [[],[],[],[]]
    for A, rect in ped_preds:
        width = rect[1][0] + rect[0][0]
        height = rect[3][0] + rect[2][0]
        x = -rect[0][0]
        y = -rect[2][0]
        c = int(-rect[4][0])

        patch = Rectangle((x, y), width, height)
        rects[c].append(patch)
        # print(height, width, y)

    return rects

if __name__ == '__main__':
    # filename = 'intersection_03_traj_ped.csv'
    # pedNum = 10
    # time_stamp = 0
    # time_step = 64e-3
    # cur_pos = [122,2,0,0,0]
    # stop = 0
    # d_sense = 10

    # waypoints = [[122, 2, 0], [136, 5, 10], [138, 20, 20]]
    #
    fig, ax = plt.subplots(figsize=(7, 7))
    #
    patches = [Rectangle((113, 9), 19, 21), Rectangle((113, -22), 19, 21), Rectangle((141, -22), 19, 21), Rectangle((141, 9), 19, 21)]
    pc = PatchCollection(patches, facecolor='lightgreen', alpha = 0.6)
    ax.add_collection(pc)
    #
    #
    # plt.plot(cur_pos[0], cur_pos[1], ls='None', color='b', marker='o', label='Car')
    # xp = []
    # yp = []
    # for agent in agent_paths:
    #     xp.append(agent[0][0])
    #     yp.append(agent[0][1])
    # plt.plot(xp, yp , ls='None', color='k', marker='o', label='Pedestrians')
    # ped_preds, sense_ped = ped_rects(cur_pos, pedNum, agent_paths, 0.1, 0, int(4/0.1), d_sense)
    # all_rects = create_rects(ped_preds)
    # pc0 = PatchCollection(all_rects[0], facecolor = 'orange')
    # pc1 = PatchCollection(all_rects[1], facecolor = 'coral')
    # pc2 = PatchCollection(all_rects[2], facecolor = 'crimson')
    # pc3 = PatchCollection(all_rects[3], facecolor = 'maroon')
    # ax.add_collection(pc0)
    # ax.add_collection(pc1)
    # ax.add_collection(pc2)
    # ax.add_collection(pc3)
    #
    #
    # xs = []
    # ys = []
    # for ped in sense_ped:
    #     xs.append(ped[0])
    #     ys.append(ped[1])
    # sensed = plt.plot(xs, ys , ls='None', color='g', marker='o', label='Sensed pedestrians')
    #
    # theta, goal, obs = map(cur_pos)
    # obs.extend(ped_preds)
    # bloat_list = [3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3]
    # pred_wps = find_xref(theta, goal, obs, 10, 5, bloat_list, old_wp = pred_wps)
    #
    # xlist = []
    # ylist = []
    # for x, y, t in pred_wps:
    #     xlist.append(x)
    #     ylist.append(y)
    #
    # plt.plot([135.5], [22.5], ls = 'None', marker = 'o', color = 'g', markersize = 20, alpha = 0.6)
    # plt.plot(xlist, ylist,ls = '--', color='g', label = 'Synthesized path')
    # print(pred_wps)

    current_pos, = plt.plot([], [] , ls='None', color='b', marker='o', label='car_pos')
    # car_outline, = plt.plot([], [] , color='b')
    agent_pos, = plt.plot([], [] , ls='None', color='k', marker='o', label='agent_pos')
    path_proj, = plt.plot([], [] , ls = '--', color='g')
    sensed, = plt.plot([], [] , ls='None', color='g', marker='o', label='agent_pos')

    ani = animation.FuncAnimation(fig, animate, frames=1000, interval = 100)

    plt.axis([118, 145, -3, 30])
    plt.legend()
    plt.xticks([])
    plt.yticks([])
    # plt.grid()
    plt.show()
